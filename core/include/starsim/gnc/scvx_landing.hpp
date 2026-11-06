#pragma once

#include <chrono>
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
    // Share of snapshot.reserve_kg a plan may end below the floor at a penalty (DescentLimits::soft_reserve_kg). Where
    // the reserve is planning margin (our tanks burn to empty; Super Heavy, Falcon 9, HLS):
    // [E] re-plans of the target: a burn short of propellant plans to the floor, and the re-plans from measured states
    // need a little more (Falcon 9 ASDS seed 117: ~0.1 t). The first plan keeps the whole reserve.
    double replan_reserve_fraction = 0.5;
    // [D] abort plans (to the sea): the stage is given up, the water beyond the shoreline is what matters - the whole
    // margin is theirs (Falcon 9 lit 900 m beyond its zone: the abort reaches the water only with ~3.3 t of the 5 t).
    double abort_reserve_fraction = 1.0;
    // The water counts as in reach while this x the estimated cost (estimate_sea_reach, a lower bound) fits the usable
    // propellant; beyond, the abort gives the water up from the live state instead of finding out late.
    // [D] flown propellant over the estimate from the abort on (Block 3 booster, shore 765 m): 1.45 from 1500 m, 1.39 from
    // 800 m; from 400 m (needs 1.34+) it ran out 55 m short of the water. Re-measured with the derived 1.4 s catch
    // approach and the engines cut at the abort plan's end (planner plan Task 5): 1.41 from 1500 m and from 800 m. 1.5
    // stays: it covers the measured ratios (1.39-1.45) with a little margin; 400 and 250 m lie near the edge (in reach,
    // flown to the water with 4.4 t with the derived landing rates, fix round 1), 150 m gives the water up.
    double sea_reach_margin = 1.5;
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

// A first plan in real-time play (Task 11b fix round 1) is planned from the state predicted at a hand-over epoch ahead
// of its snapshot: the vehicle flies a known hold until then (the ship its flip thrust along the axis, the booster its
// unpowered fall, the HLS its braking thrust), the plan takes effect at the hand-over, and a plan that is not there by
// then is waited for (the synchronous solve). So the sim time between snapshot and plan is the hold, never more.
struct FirstPlanHold {
    double duration_s = 0.0;                  // snapshot -> hand-over (0: the plan takes effect at its snapshot)
    std::optional<math::Vec3> thrust_body_n;  // the held thrust, body axes; none: the snapshot's thrust along the body axis
    bool engines_running = true;              // false: no engine burns in the hold (no mass flow)
    // The held thrust comes down from its start at the start set's throttle rate to its minimum (the ship commands its
    // flip engines to minimum throttle: the flip thrust held for seconds would turn the sink into a climb).
    bool throttle_down = false;
};

// The hold's length from the measured first-plan wall times (process-wide, per vehicle kind: the solves differ by an
// order of magnitude): margin x the latest measured wall time, at least the plan latency, at most first_plan_hold_max_s.
// Before a first measurement the kind's measured reference (Task 3 / 11b).
enum class FirstPlanKind { Booster, Ship, Lunar };
double first_plan_hold_s(FirstPlanKind kind);
void record_first_plan_wall_s(FirstPlanKind kind, double wall_s);
// [E] Bound of the hold. Past it the sim waits for the plan (as the synchronous first plan always did): the gap between
// the snapshot and the plan's start stays the predicted hold. 3 s of the ship's flip thrust (3 engines at 0.8) is about
// the flip's own settling time; the booster's unpowered fall over 3 s is the timing error the ignition criterion already
// tolerates (~1 s over 120 s, booster_return.cpp forecast_landing) times three.
constexpr double first_plan_hold_max_s = 3.0;
constexpr double first_plan_hold_margin = 1.3;  // [E] over the latest measured wall time

// The state after `hold` from x under the held thrust (planning model; attitude and body rate held - the attitude
// controller keeps them).
DescentState predict_hold(const DescentModel& model, const DescentState& x, const math::Vec3& thrust_body_n,
                          bool engines_running, double duration_s, bool throttle_down = false);
// The held thrust's magnitude after `t_s` (throttle_down: at the start set's throttle rate down to its minimum).
double hold_thrust_n(const DescentModel& model, double thrust_n, bool throttle_down, double t_s);

// Landing burn on SCvx plans: the first plan synchronously at ignition, re-plans every replan_interval_s from a
// snapshot (worker thread, effective at snapshot + latency), tracking of the active plan (thrust acceleration with a
// position/velocity correction, attitude with feed-forward, pins rolled across the arms), engine switch at the
// planned time. failures_before_abort consecutive failed re-plans give up the catch: from then on the plans go to
// the sea (keep-out around the tower) and the command says abort (the tower keeps its arms open).
class ScvxLandingGuidance {
public:
    ScvxLandingGuidance(LandingSite site, ScvxLandingParameters parameters, PlanFunction plan = default_plan_function());

    // First plan; false if it failed (the caller falls back to other guidance). Synchronous: begin + poll.
    bool start(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    // The first plan in parts (Task 11b): begin takes the snapshot at `epoch` and plans - on the replanner's worker with
    // `background` (real-time play: the vehicle flies the hold meanwhile), else at once; poll gives the outcome at the
    // hand-over (epoch + hold.duration_s; the refused catch's abort plan and the abort without the water are planned
    // in turn, from the same predicted state): true - the plan takes effect at the hand-over, false - it failed. At
    // the hand-over poll waits for a plan still under way. Same requests either way, so the same plans; update() only
    // after poll returned true.
    void begin(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch, bool background,
               const FirstPlanHold& hold = {});
    std::optional<bool> poll(time::Epoch now);
    bool first_plan_pending() const { return first_snapshot_.has_value() || first_decided_.has_value(); }
    time::Epoch handover_epoch() const { return first_handover_; }
    double first_plan_wall_s() const { return first_wall_s_; }  // begin -> decided (diagnostics, the hold's budget)
    ScvxLandingCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    bool aborting() const { return aborting_; }
    // A converged abort plan is being flown.
    bool has_abort_plan() const { return plan_ && plan_->goal == DescentGoal::Abort && plan_->converged && !abort_cannot_clear_; }
    // Abort plans must still end in the water (false: the reach estimate or a failed plan gave it up).
    bool sea_required() const { return sea_required_; }
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
        // LQR lateral control: commanded tilt acceleration (plan feed-forward + feedback) over what the gimbals give at
        // the thrust (their whole deflection) - the authority the planned allowance (DescentLimits::gimbal_fraction)
        // left the controller. Over the whole burn and over its last terminal_window_s (the touchdown approach).
        static constexpr double terminal_window_s = 10.0;
        int lateral_ticks = 0;
        int clipped_ticks = 0;               // the command exceeded the gimbals' reach (clipped)
        double peak_gimbal_use = 0.0;        // largest ratio before the clip
        int terminal_ticks = 0;
        int terminal_clipped_ticks = 0;
        double terminal_peak_gimbal_use = 0.0;
        double terminal_peak_feedback_use = 0.0;  // the feedback alone (the plan's tilt acceleration taken out)
    };
    const Statistics& statistics() const { return stats_; }

private:
    // A re-plan (a plan exists) starts where it takes effect: the snapshot state predicted over the plan latency with
    // the active plan's thrust (the engines fly that plan meanwhile).
    PlanRequest request(const DescentSnapshot& snapshot, int start_phase) const;
    void accept(const PlanOutcome& outcome);
    // While an abort still aims at the water: gives it up (sticky) if the live state cannot reach it on the propellant left.
    void check_sea_reach(const DescentSnapshot& snapshot, int engine_set);
    // A plan for the target that reaches it only by tilting beyond its limits (DescentPlan::planned_tilt_excess): the
    // target is out of reach, as for a plan that does not converge.
    bool beyond_limits(const DescentPlan& plan) const;
    // A converged abort plan whose hull still overlaps a keep-out box.
    bool abort_strikes(const DescentPlan& plan) const;
    void submit_first();  // the first plan's next request from its snapshot
    // One first-plan outcome: the next request submitted (none) or the first plan decided.
    std::optional<bool> first_step(const PlanOutcome& outcome);

    LandingSite site_;
    ScvxLandingParameters parameters_;
    PlanFrame frame_;
    DescentReplanner replanner_;
    std::shared_ptr<const DescentPlan> plan_;
    PlanPath path_;  // plan_'s own trajectory between its nodes: the tracking reference (sample_plan)
    time::Epoch plan_epoch_;
    time::Epoch first_plan_epoch_;  // diagnostics
    // The first plan under way (begin .. poll): its snapshot, epoch, which refusal step is next, the outcome of a plan
    // made at once, and its wall-clock start (diagnostics).
    std::optional<DescentSnapshot> first_snapshot_;
    time::Epoch first_epoch_;
    time::Epoch first_handover_;  // the first plan takes effect here (first_epoch_ + the hold)
    FirstPlanHold first_hold_;
    std::optional<DescentState> first_initial_;  // the first plan's start: the snapshot state flown through the hold
    int first_stage_ = 0;
    bool first_background_ = false;
    std::optional<PlanOutcome> first_outcome_;
    std::optional<bool> first_decided_;  // decided before the hand-over: given at it
    std::chrono::steady_clock::time_point first_wall_start_;
    double first_wall_s_ = 0.0;
    double first_begin_ms_ = 0.0;  // the snapshot, the hold's prediction and the submission (sim thread)
    DescentModel model_;  // of the latest snapshot (gravity after the plan's end)
    time::Epoch next_request_;
    int engine_set_ = 0;  // phase whose engine set runs (only ever increases)
    bool aborting_ = false;
    int consecutive_failures_ = 0;
    int consecutive_unconverged_ = 0;
    bool sea_required_ = true;  // abort plans must end beyond the shoreline; false once that proved unreachable
    bool sea_reach_checked_ = false;
    bool abort_cannot_clear_ = false;  // an abort plan without the water still strikes (sticky)
    Statistics stats_;
    TvcLateralGains lqr_gains_;  // cached for (lqr_a_, lqr_c_)
    double lqr_a_ = -1.0;
    double lqr_c_ = -1.0;
    int lqr_set_ = -1;
};

}  // namespace starsim::gnc
