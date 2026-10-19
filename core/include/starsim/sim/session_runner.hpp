#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// Runs a FlightSession on its own thread (l6, user 2026-10-18; CLAUDE.md: "the simulation runs on its own thread").
// From construction on only the worker touches the session: commands run in posting order, between them the session
// advances towards the target epoch in chunks; the observer runs on the worker after every command and every chunk
// (the GDExtension builds its snapshot there). A long call (a return plan: seconds) holds the simulation clock -
// the caller sees busy_s() grow and the epoch stand.
class SessionRunner {
public:
    using Command = std::function<void(FlightSession&)>;
    using Observer = std::function<void(const FlightSession&, time::Epoch epoch)>;

    // [E] One chunk advances at most this many ticks (FlightSession::advance_to's limit) before commands and the
    // observer get their turn again: 4000 ticks = 20 s of flight at 200 Hz.
    static constexpr int max_ticks_per_chunk = 4000;

    SessionRunner(std::unique_ptr<FlightSession> session, Observer observer);
    // Drops the queued commands, waits for the running call (a plan may take seconds) and joins.
    ~SessionRunner();
    SessionRunner(const SessionRunner&) = delete;
    SessionRunner& operator=(const SessionRunner&) = delete;

    // Queues a command; returns its id (> 0, increasing). An exception it throws is caught: result(id) carries it.
    std::uint64_t post(Command command);
    // Overwrites the single "latest" slot (pilot input): run once before the next chunk; older values are dropped.
    void set_latest(Command command);
    // The session advances towards `target` whenever no command waits. A new target replaces the old one, also an
    // earlier one (a new flight may start before the last target); the session itself never steps backwards.
    void set_target(time::Epoch target);

    struct Result {
        bool done = false;
        bool threw = false;
        std::string error;  // what() of the exception
    };
    Result result(std::uint64_t id) const;  // done == false while queued or running (unknown ids: done, error set)
    // Finished results not yet taken, oldest first (the GDExtension emits command_finished for them).
    std::deque<std::pair<std::uint64_t, Result>> take_finished();

    bool idle() const;                       // no command queued or running, and the target reached (or inactive)
    bool wait_idle(double timeout_s) const;  // true when idle within the timeout (tests, scripts)
    double busy_s() const;                   // wall seconds the current command has been running (0 without one)
    std::string last_error() const;          // of the latest input, advance or observer exception ("" none)

private:
    void run();
    void run_command(std::unique_lock<std::mutex>& lock);  // front of the queue; lock held on entry and exit
    void run_chunk(std::unique_lock<std::mutex>& lock);    // latest input + one advance chunk; lock as above
    bool idle_locked() const;                              // mutex_ held
    bool target_reached_locked() const;                    // no target, or less than one tick left; mutex_ held

    std::unique_ptr<FlightSession> session_;
    Observer observer_;
    mutable std::mutex mutex_;
    mutable std::condition_variable wake_, idle_cv_;
    std::deque<std::pair<std::uint64_t, Command>> queue_;
    std::optional<Command> latest_;
    std::optional<time::Epoch> target_;
    time::Epoch reached_;
    std::uint64_t next_id_ = 1;
    std::uint64_t running_id_ = 0;  // 0: none
    std::chrono::steady_clock::time_point running_since_;
    std::map<std::uint64_t, Result> results_;  // kept until 1,000 newer ids exist [E]
    std::deque<std::pair<std::uint64_t, Result>> finished_;
    std::string last_error_;
    bool active_ = false;  // the session's active(), as of the last command / chunk (worker writes under the lock)
    bool stop_ = false;
    std::thread worker_;  // last: started after every member is initialised
};

}  // namespace starsim::sim
