#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/descent_replanner.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/gnc/descent_tracking.hpp"
#include "starsim/gnc/tvc_lateral.hpp"

namespace starsim::gnc {

struct ScvxLandingParameters {
    double replan_interval_s = 1.0;        // spec
    double plan_latency_s = 0.5;           // spec: a plan takes effect at snapshot + this
    double min_replan_time_to_go_s = 3.0;  // [E] no catch re-plans closer to the end (phase durations degenerate)
    int failures_before_abort = 2;         // [E] consecutive re-plans that find the catch unreachable (virtual
                                           // control >= settings.stall_virtual_control) before it is given up
    int unconverged_before_abort = 4;      // [E] consecutive re-plans that do not converge at all (a marginal first
                                           // plan the vehicle could not follow) before the target is given up
    bool realtime = false;                 // never wait for a re-plan (interactive play)
    // Burn profile that times the ignition (BoosterReturnGuidance::predict_landing_burn), V3 baseline: 13 engines,
    // 5 from ~200 m/s, 3 for the last slow metres. Flights 13/14 light the burn at ~2.8-2.9 km falling ~0.35-0.4 km/s
    // [D]: 13 engines near minimum throttle reproduce that (scenario: 2.88 km at 355 m/s).
    double ignition_throttle = 0.35;       // [E] 13 engines
    double divert_speed_mps = 200.0;       // [E] switch to the 5 divert engines
    double divert_throttle = 0.4;          // [E]
    double switch_speed_mps = 22.0;        // [E] switch to the 3 centre engines
    double final_throttle = 0.4;           // [E] near hover
    SnapshotSettings snapshot;
    ScvxSettings settings;
    TrackingGains tracking;
    // Lateral control by LQR on the gimbaled-rocket dynamics (body torque command); false: attitude-controller
    // tracking of the commanded axis.
    bool lqr_lateral = true;
    TvcLateralWeights lateral;
    // LQR control effort scale as a fraction of the gimbals' tilt acceleration at the current thrust (0: the fixed
    // scale in `lateral`).
    // [E] Campaign with the calibrated descent drag: 0.3 left a limit cycle on 3 engines near minimum throttle (each
    // re-plan's small reference step saturated the gimbals) in 1 of 21 cases; 0.2 and 0.15 catch all 21.
    double lateral_effort_fraction = 0.2;
    double final_lateral_effort_fraction = 0.0;  // the same for the final engine set (0: lateral_effort_fraction)
    double roll_frequency_rad_s = 1.0;  // [E] pins towards the arms
    // An engine switch that moves the thrust centroid (the ship's 2 -> 1) turns the thrust axis against the hull; over
    // this time before the planned switch the lateral control blends the steered axis towards the next set's, so the
    // hull leans over in advance instead of the thrust pushing sideways after the switch (0: no blending). [E]
    double switch_prerotation_s = 0.0;
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
    std::optional<math::Vec3> torque_body_nm;  // LQR lateral control: body torque for the gimbals / RCS
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
    // A converged abort plan is being flown.
    bool has_abort_plan() const { return plan_ && plan_->goal == DescentGoal::Abort && plan_->converged; }
    int engine_set() const { return engine_set_; }  // phase whose engine set runs
    // End of the active plan (the catch or touchdown it flies to); none before the first plan.
    std::optional<time::Epoch> plan_end_epoch() const;
    // Gives the target up now (pilot or tower command): the next plan goes to the sea. False (no effect) without a sea.
    bool request_abort();
    void set_realtime(bool realtime) { parameters_.realtime = realtime; }

    struct Statistics {
        int plans = 0;       // collected plans incl. the first
        int failures = 0;    // of these not converged
        double last_solve_ms = 0.0;
        double max_solve_ms = 0.0;
    };
    const Statistics& statistics() const { return stats_; }

private:
    // A re-plan (a plan exists) starts where it takes effect: the snapshot state predicted over the plan latency with
    // the active plan's thrust (the engines fly that plan meanwhile).
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
    int engine_set_ = 0;  // phase whose engine set runs (only ever increases)
    bool aborting_ = false;
    int consecutive_failures_ = 0;
    int consecutive_unconverged_ = 0;
    bool sea_required_ = true;  // abort plans must end beyond the shoreline; false once that proved unreachable
    Statistics stats_;
    TvcLateralGains lqr_gains_;  // cached for (lqr_a_, lqr_c_)
    double lqr_a_ = -1.0;
    double lqr_c_ = -1.0;
    int lqr_set_ = -1;
};

}  // namespace starsim::gnc
