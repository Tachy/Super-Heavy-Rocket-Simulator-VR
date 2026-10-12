#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <thread>

#include "starsim/gnc/descent_replanner.hpp"

using namespace starsim;
using namespace starsim::gnc;

namespace {

time::Epoch at(double s) { return time::Epoch::from_tdb_seconds_past_j2000(s); }

// Fake planner: marks the plan with the request's start phase, optionally slow.
PlanFunction fake(std::chrono::milliseconds delay, std::atomic<int>* calls = nullptr) {
    return [delay, calls](DescentPlanner&, const PlanRequest& r) {
        std::this_thread::sleep_for(delay);
        if (calls) ++*calls;
        DescentPlan p;
        p.start_phase = r.start_phase;
        p.converged = true;
        return p;
    };
}

PlanRequest request(double snapshot_s, int phase = 0) {
    PlanRequest r;
    r.snapshot_epoch = at(snapshot_s);
    r.start_phase = phase;
    return r;
}

}  // namespace

TEST_CASE("A re-plan takes effect at its snapshot plus the latency", "[gnc][replanner]") {
    DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(0)));
    replanner.submit(request(10.0, 1));
    CHECK(replanner.busy());
    CHECK_FALSE(replanner.collect(at(10.4), true).has_value());  // too early, even when waiting
    const auto outcome = replanner.collect(at(10.5), true);
    REQUIRE(outcome.has_value());
    CHECK(outcome->plan->start_phase == 1);
    CHECK(outcome->snapshot_epoch - at(10.0) == 0.0);  // Epoch has no operator==
    CHECK_FALSE(replanner.busy());
}

TEST_CASE("Waiting mode blocks until the plan is ready", "[gnc][replanner]") {
    DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(200)));
    replanner.submit(request(0.0));
    CHECK(replanner.collect(at(0.5), true).has_value());
}

TEST_CASE("Real-time mode never waits for a slow plan", "[gnc][replanner]") {
    DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(300)));
    replanner.submit(request(0.0));
    const auto start = std::chrono::steady_clock::now();
    CHECK_FALSE(replanner.collect(at(0.5), false).has_value());
    CHECK(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(50));
    CHECK(replanner.busy());
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    CHECK(replanner.collect(at(0.6), false).has_value());  // used once it arrives
}

TEST_CASE("The first plan is made synchronously", "[gnc][replanner]") {
    std::atomic<int> calls{0};
    DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(0), &calls));
    const auto outcome = replanner.plan_now(request(3.0, 0));
    CHECK(calls == 1);
    CHECK(outcome.plan->converged);
    CHECK(outcome.solve_ms >= 0.0);
    CHECK_FALSE(replanner.busy());
}

TEST_CASE("Destroying the replanner while it works waits for the worker", "[gnc][replanner]") {
    std::atomic<int> calls{0};
    {
        DescentReplanner replanner(0.5, fake(std::chrono::milliseconds(150), &calls));
        replanner.submit(request(0.0));
    }
    CHECK(calls == 1);
}
