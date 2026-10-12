#pragma once

#include <memory>
#include <string>
#include <vector>

#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/descent_replanner.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/gnc/descent_tracking.hpp"

namespace starsim::gnc {

struct ScvxLandingParameters {
    double replan_interval_s = 1.0;        // spec
    double plan_latency_s = 0.5;           // spec: a plan takes effect at snapshot + this
    double min_replan_time_to_go_s = 3.0;  // [E] no catch re-plans closer to the end (phase durations degenerate)
    int failures_before_abort = 2;         // [E] consecutive failed re-plans before the catch is given up
    bool realtime = false;                 // never wait for a re-plan (interactive play)
    SnapshotSettings snapshot;
    ScvxSettings settings;
    TrackingGains tracking;
};

struct ScvxLandingCommand {
    std::vector<std::string> run_groups;
    double throttle = 1.0;
    math::Vec3 axis_gcrf;
    math::Vec3 roll_reference_gcrf;               // zero: free roll
    AttitudeController::Feedforward feedforward;  // body axes, relative to inertial space
    bool abort = false;
    double time_to_go_s = 0.0;
    double offset_m = 0.0;  // horizontal distance of the CG from the plan's end point
};

// Throttle of `run_groups` that gives `thrust_n` together with the engines still starting or running down (engines
// just starting count as slightly spooled so the throttle stays finite).
double landing_throttle(const vehicle::Vehicle& vehicle, const std::vector<std::string>& run_groups, double thrust_n,
                        double pressure_pa);

// Landing burn on SCvx plans: the first plan synchronously at ignition, re-plans every replan_interval_s from a
// snapshot (worker thread, effective at snapshot + latency), tracking of the active plan (thrust acceleration with a
// position/velocity correction, attitude with feed-forward, pins rolled across the arms), engine switch at the
// planned time. failures_before_abort consecutive failed re-plans give up the catch: from then on the plans go to
// the sea (keep-out around the tower) and the command says abort (the tower keeps its arms open).
class ScvxLandingGuidance {
public:
    ScvxLandingGuidance(LandingSite site, ScvxLandingParameters parameters, PlanFunction plan = default_plan_function());

    // First plan; false if it failed (the caller falls back to other guidance).
    bool start(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    ScvxLandingCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    bool aborting() const { return aborting_; }
    void set_realtime(bool realtime) { parameters_.realtime = realtime; }

    struct Statistics {
        int plans = 0;       // collected plans incl. the first
        int failures = 0;    // of these not converged
        double last_solve_ms = 0.0;
        double max_solve_ms = 0.0;
    };
    const Statistics& statistics() const { return stats_; }

private:
    PlanRequest request(const DescentSnapshot& snapshot, int start_phase) const;
    void accept(const PlanOutcome& outcome);

    LandingSite site_;
    ScvxLandingParameters parameters_;
    PlanFrame frame_;
    DescentReplanner replanner_;
    std::shared_ptr<const DescentPlan> plan_;
    time::Epoch plan_epoch_;
    DescentModel model_;  // of the latest snapshot (gravity after the plan's end)
    time::Epoch next_request_;
    bool final_engines_ = false;
    bool aborting_ = false;
    int consecutive_failures_ = 0;
    Statistics stats_;
};

}  // namespace starsim::gnc
