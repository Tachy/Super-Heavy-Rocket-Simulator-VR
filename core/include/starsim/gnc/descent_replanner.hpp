#pragma once

#include <functional>
#include <future>
#include <memory>
#include <optional>

#include "starsim/gnc/descent_planner.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::gnc {

// Everything one re-plan needs; owned by the request (the worker touches nothing else).
struct PlanRequest {
    DescentModel model;
    DescentState initial;
    int start_phase = 0;
    DescentTarget target;
    ScvxSettings settings;
    std::shared_ptr<const DescentPlan> warm_start;  // null: fresh plan
    double warm_start_elapsed_s = 0.0;
    time::Epoch snapshot_epoch;                     // the plan starts here (node 0 = initial)
};

struct PlanOutcome {
    std::shared_ptr<const DescentPlan> plan;
    time::Epoch snapshot_epoch;
    double solve_ms = 0.0;  // wall-clock time of the planner
};

using PlanFunction = std::function<DescentPlan(DescentPlanner&, const PlanRequest&)>;
PlanFunction default_plan_function();  // DescentPlanner::plan on the request

// Plans on one worker thread with a fixed latency in simulation time: a plan from a snapshot at t takes effect at
// t + latency. Waiting mode (tests, time warp) blocks at that moment until the plan is ready - deterministic.
// Real-time mode never blocks: a late plan is used at the first collect after it is ready. One request at a time;
// the planner (and its solvers) is used by one thread at a time. Not thread-safe itself: call from one thread.
class DescentReplanner {
public:
    explicit DescentReplanner(double latency_s, PlanFunction plan = default_plan_function());
    ~DescentReplanner();  // waits for a running request
    DescentReplanner(const DescentReplanner&) = delete;
    DescentReplanner& operator=(const DescentReplanner&) = delete;

    PlanOutcome plan_now(const PlanRequest& request);  // synchronous; precondition: !busy()
    void submit(PlanRequest request);                  // precondition: !busy()
    bool busy() const { return pending_.valid(); }     // submitted and not yet collected
    std::optional<PlanOutcome> collect(time::Epoch now, bool wait);

private:
    double latency_s_;
    PlanFunction plan_;
    DescentPlanner planner_;
    std::future<PlanOutcome> pending_;
    time::Epoch pending_snapshot_;
};

}  // namespace starsim::gnc
