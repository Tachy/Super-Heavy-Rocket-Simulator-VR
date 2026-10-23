#include "starsim/gnc/descent_replanner.hpp"

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "starsim/gnc/descent_request_io.hpp"

namespace starsim::gnc {

namespace {

constexpr double epoch_tolerance_s = 1e-9;  // tick sums: "at snapshot + latency" must not miss by rounding

PlanOutcome run(DescentPlanner& planner, const PlanFunction& plan, const PlanRequest& request) {
    if (const auto dumped = dump_plan_request_if_enabled(request); !dumped.empty()) {
        std::printf("[scvx] request dumped to %s\n", dumped.c_str());  // diagnostics: replay with read_plan_request
    }
    const auto start = std::chrono::steady_clock::now();
    PlanOutcome outcome;
    outcome.plan = std::make_shared<const DescentPlan>(plan(planner, request));
    outcome.snapshot_epoch = request.snapshot_epoch;
    outcome.start_offset_s = request.start_offset_s;
    outcome.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return outcome;
}

}  // namespace

PlanFunction default_plan_function() {
    return [](DescentPlanner& planner, const PlanRequest& r) {
        return planner.plan(r.model, r.initial, r.start_phase, r.target, r.settings, r.warm_start.get(),
                            r.warm_start_elapsed_s);
    };
}

DescentReplanner::DescentReplanner(double latency_s, PlanFunction plan) : latency_s_(latency_s), plan_(std::move(plan)) {}

DescentReplanner::~DescentReplanner() {
    if (pending_.valid()) pending_.wait();
}

PlanOutcome DescentReplanner::plan_now(const PlanRequest& request) {
    if (busy()) throw std::logic_error("DescentReplanner::plan_now while a request is pending");
    return run(planner_, plan_, request);
}

void DescentReplanner::submit(PlanRequest request) {
    if (busy()) throw std::logic_error("DescentReplanner::submit while a request is pending");
    pending_snapshot_ = request.snapshot_epoch;
    pending_ = std::async(std::launch::async, [this, r = std::move(request)] { return run(planner_, plan_, r); });
}

std::optional<PlanOutcome> DescentReplanner::collect(time::Epoch now, bool wait) {
    if (!pending_.valid() || now - pending_snapshot_ < latency_s_ - epoch_tolerance_s) return std::nullopt;
    if (!wait && pending_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return std::nullopt;
    return pending_.get();
}

}  // namespace starsim::gnc
