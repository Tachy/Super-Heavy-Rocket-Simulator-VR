#include "starsim/sim/session_runner.hpp"

#include <exception>

namespace starsim::sim {

namespace {

constexpr size_t max_kept_results = 1000;  // [E] results nobody asked for: dropped oldest first

// Runs `call`; returns the exception text, nullopt when it returned normally.
template <typename F>
std::optional<std::string> guarded(F&& call) {
    try {
        call();
    } catch (const std::exception& e) {
        return std::string(e.what());
    } catch (...) {
        return std::string("unknown exception");
    }
    return std::nullopt;
}

// FlightSession::advance_to stops once less than one tick is left (it never steps past the target), so a target
// counts as reached within one tick.
bool reached(time::Epoch now, time::Epoch target) { return target - now < FlightSession::tick_s; }

}  // namespace

bool SessionRunner::target_reached_locked() const { return !target_ || reached(reached_, *target_); }

SessionRunner::SessionRunner(std::unique_ptr<FlightSession> session, Observer observer)
    : session_(std::move(session)), observer_(std::move(observer)), worker_([this] { run(); }) {}

SessionRunner::~SessionRunner() {
    {
        std::lock_guard lock(mutex_);
        stop_ = true;
        queue_.clear();
        latest_.reset();
    }
    wake_.notify_all();
    worker_.join();
}

std::uint64_t SessionRunner::post(Command command) {
    return post_deferred([command = std::move(command)](FlightSession& session) {
        command(session);
        return Poll{};
    });
}

std::uint64_t SessionRunner::post_deferred(DeferredCommand command) {
    std::uint64_t id = 0;
    {
        std::lock_guard lock(mutex_);
        id = next_id_++;
        queue_.emplace_back(id, std::move(command));
        results_[id] = Result{};
        while (results_.size() > max_kept_results) results_.erase(results_.begin());
    }
    wake_.notify_all();
    return id;
}

bool SessionRunner::chunk_due_locked() const {
    return latest_ || (active_ && !stalled_ && !target_reached_locked());
}

void SessionRunner::set_latest(Command command) {
    {
        std::lock_guard lock(mutex_);
        latest_ = std::move(command);
    }
    wake_.notify_all();
}

void SessionRunner::set_target(time::Epoch target) {
    {
        std::lock_guard lock(mutex_);
        target_ = target;
    }
    wake_.notify_all();
}

void SessionRunner::run() {
    // [E] a deferred command's poll interval while there is nothing else to do
    constexpr auto deferred_poll = std::chrono::milliseconds(2);
    std::unique_lock lock(mutex_);
    while (true) {
        wake_.wait(lock, [this] { return stop_ || deferred_ || !queue_.empty() || chunk_due_locked(); });
        if (stop_) return;
        if (deferred_) poll_deferred(lock);
        if (stop_) return;
        if (!deferred_ && !queue_.empty()) {
            run_command(lock);
        } else if (chunk_due_locked()) {
            run_chunk(lock);  // also while a deferred command plans: the session flies on
        } else if (deferred_) {
            wake_.wait_for(lock, deferred_poll, [this] { return stop_ || chunk_due_locked(); });
        }
        idle_cv_.notify_all();
    }
}

void SessionRunner::run_command(std::unique_lock<std::mutex>& lock) {
    auto [id, command] = std::move(queue_.front());
    queue_.pop_front();
    running_id_ = id;
    stalled_ = false;  // a command may have changed what made the advance throw
    running_since_ = std::chrono::steady_clock::now();
    lock.unlock();
    Result r{true, false, {}};
    Poll poll;
    if (auto error = guarded([&] { poll = command(*session_); })) {
        r.threw = true;
        r.error = *error;
        poll = nullptr;
    }
    // reached_ has a single writer (this thread): reading it unlocked is safe.
    const bool active = session_->active();
    const auto now = active ? session_->epoch() : reached_;
    const auto observer_error = guarded([&] { observer_(*session_, now); });
    lock.lock();
    active_ = active;
    reached_ = now;
    if (observer_error) set_error_locked(*observer_error);
    if (poll) {
        deferred_.emplace(id, std::move(poll));  // running_id_ stays: the command goes on
        return;
    }
    finish_locked(id, r);
}

void SessionRunner::poll_deferred(std::unique_lock<std::mutex>& lock) {
    const auto id = deferred_->first;
    const Poll poll = deferred_->second;
    lock.unlock();
    Result r{true, false, {}};
    bool done = false;
    if (auto error = guarded([&] { done = poll(*session_); })) {
        r.threw = true;
        r.error = *error;
        done = true;
    }
    std::optional<std::string> observer_error;
    bool active = false;
    time::Epoch now = reached_;
    if (done) {  // the outcome may have changed the session: the snapshot follows
        active = session_->active();
        now = active ? session_->epoch() : reached_;
        observer_error = guarded([&] { observer_(*session_, now); });
    }
    lock.lock();
    if (!done) return;
    deferred_.reset();
    active_ = active;
    reached_ = now;
    if (observer_error) set_error_locked(*observer_error);
    finish_locked(id, r);
}

void SessionRunner::finish_locked(std::uint64_t id, const Result& r) {
    running_id_ = 0;
    if (const auto it = results_.find(id); it != results_.end()) it->second = r;
    finished_.emplace_back(id, r);
    while (finished_.size() > max_kept_results) finished_.pop_front();
}

void SessionRunner::run_chunk(std::unique_lock<std::mutex>& lock) {
    std::optional<Command> latest = std::move(latest_);
    latest_.reset();
    const auto target = target_;
    chunk_running_ = true;
    lock.unlock();
    time::Epoch now = reached_;
    const auto input_error = guarded([&] {
        if (latest) (*latest)(*session_);
    });
    const auto advance_error = guarded([&] {
        if (session_->active()) {
            now = session_->epoch();
            if (target && !reached(now, *target)) now = session_->advance_to(*target, max_ticks_per_chunk);
        }
    });
    // The snapshot follows also after a failed input or advance (the session may have moved).
    const auto observer_error = guarded([&] { observer_(*session_, now); });
    const bool active = session_->active();
    lock.lock();
    chunk_running_ = false;
    active_ = active;
    reached_ = now;
    // An advance that throws would throw again on the next chunk: stop driving the target until a command arrives.
    if (advance_error) stalled_ = true;
    for (const auto* error : {&input_error, &advance_error, &observer_error}) {
        if (*error) set_error_locked(**error);
    }
}

void SessionRunner::set_error_locked(const std::string& error) {
    last_error_ = error;
    error_taken_ = false;
}

bool SessionRunner::idle_locked() const {
    return queue_.empty() && !latest_ && running_id_ == 0 && !chunk_running_ &&
           (!active_ || stalled_ || target_reached_locked());
}

bool SessionRunner::idle() const {
    std::lock_guard lock(mutex_);
    return idle_locked();
}

bool SessionRunner::wait_idle(double timeout_s) const {
    std::unique_lock lock(mutex_);
    return idle_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), [this] { return idle_locked(); });
}

double SessionRunner::busy_s() const {
    std::lock_guard lock(mutex_);
    if (running_id_ == 0) return 0.0;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - running_since_).count();
}

std::string SessionRunner::last_error() const {
    std::lock_guard lock(mutex_);
    return last_error_;
}

std::string SessionRunner::take_error() {
    std::lock_guard lock(mutex_);
    if (error_taken_) return {};
    error_taken_ = true;
    return last_error_;
}

SessionRunner::Result SessionRunner::result(std::uint64_t id) const {
    std::lock_guard lock(mutex_);
    const auto it = results_.find(id);
    if (it == results_.end()) return Result{true, true, "unknown command id"};
    return it->second;
}

std::deque<std::pair<std::uint64_t, SessionRunner::Result>> SessionRunner::take_finished() {
    std::lock_guard lock(mutex_);
    return std::exchange(finished_, {});
}

}  // namespace starsim::sim
