#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <optional>
#include <stdexcept>
#include <utility>

namespace starsim::gnc {

// One piece of planner work on a worker thread (Task 11b, planner work off the sim thread). The work owns copies of
// everything it reads (its own vehicle copies and WorkerEnvironment); the caller touches the result only after
// collect() - the future's get() orders the worker's writes before the caller's reads. One task at a time; the
// destructor waits for a running one (its result is dropped). Not thread-safe itself: call from one thread.
template <typename T>
class BackgroundTask {
public:
    BackgroundTask() = default;
    ~BackgroundTask() {
        if (pending_.valid()) pending_.wait();
    }
    BackgroundTask(const BackgroundTask&) = delete;
    BackgroundTask& operator=(const BackgroundTask&) = delete;

    bool busy() const { return pending_.valid(); }  // submitted and not yet collected
    bool ready() const {  // its result is there (not yet collected)
        return pending_.valid() && pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }
    void submit(std::function<T()> work) {
        if (busy()) throw std::logic_error("BackgroundTask::submit while a task is pending");
        if (gate_.valid()) {  // tests only (fix round 3): the job is held until the test opens the gate
            work = [gate = gate_, inner = std::move(work)]() -> T {
                if (!gate.get()) throw std::runtime_error("BackgroundTask: job called off by its test gate");
                return inner();
            };
        }
        pending_ = std::async(std::launch::async, std::move(work));
    }
    // Tests only: jobs submitted from now on wait for `gate` before they run; true runs them, false (or a broken
    // promise) lets them fail at once without running. An invalid future removes the gate (production: never set).
    void set_gate_for_test(std::shared_future<bool> gate) { gate_ = std::move(gate); }
    // Tests only: blocks until a submitted job has ended (its result stays to be collected).
    void wait_done() const {
        if (pending_.valid()) pending_.wait();
    }
    // Drops a finished job's result without blocking (its failure included: a dropped result's failure is no one's
    // concern). True when nothing is pending afterwards.
    bool reap() {
        if (!ready()) return !busy();
        discard();
        return true;
    }
    // The result once ready (wait: block until it is); none while it works or without a task. Rethrows its exception.
    std::optional<T> collect(bool wait) {
        if (!pending_.valid()) return std::nullopt;
        if (!wait && pending_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return std::nullopt;
        return pending_.get();
    }
    // Drops a pending task's result (waits for it to end: its inputs may not outlive it otherwise).
    void discard() {
        if (!pending_.valid()) return;
        try {
            pending_.get();
        } catch (...) {  // a dropped result's failure is no one's concern
        }
    }

private:
    std::future<T> pending_;
    std::shared_future<bool> gate_;  // tests only
};

}  // namespace starsim::gnc
