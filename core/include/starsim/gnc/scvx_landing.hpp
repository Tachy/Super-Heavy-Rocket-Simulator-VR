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
    bool realtime = false;                 // never wait for a re-plan (interactive play)
    // Burn profile that times the ignition (BoosterReturnGuidance::predict_landing_burn), after recorded catches:
    // 13 engines from ~2 km for ~5 s, 3 engines from ~500 m at ~80 m/s for ~20 s - a long final segment that
    // leaves the slow-turning 3-engine booster time to settle laterally.
    double ignition_throttle = 0.5;        // [E] 13 engines near minimum throttle
    double switch_speed_mps = 83.0;        // [E] ~300 km/h on recorded catches
    double final_throttle = 0.6;           // [E]
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
    double roll_frequency_rad_s = 1.0;  // [E] pins towards the arms
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
    TvcLateralGains lqr_gains_;  // cached for (lqr_a_, lqr_c_)
    double lqr_a_ = -1.0;
    double lqr_c_ = -1.0;
    int lqr_set_ = -1;
};

}  // namespace starsim::gnc
