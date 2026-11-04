#include "starsim/gnc/scvx_landing.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/sea_reach.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double min_spool_for_throttle = 0.05;
constexpr double min_lqr_thrust_acceleration_mps2 = 1.0;  // LQR design: thrust acceleration floor
constexpr double lqr_retune_fraction = 0.02;              // recompute the gains when a or c change this much
constexpr double roll_damping = 0.9;  // engines just starting: throttle as if this far spooled
constexpr double epoch_tolerance_s = 1e-9;       // tick sums: a request due "now" must not slip a tick

bool contains(const std::vector<std::string>& groups, const std::string& group) {
    return std::find(groups.begin(), groups.end(), group) != groups.end();
}

// The engines dropped at the switch to phase `next` run down over their shutdown time: a linear run-down has the
// impulse of a step at half that time, so the switch is commanded this much before the planned one.
double switch_lead_s(const vehicle::Vehicle& v, const SnapshotSettings& s, int next) {
    double lead = 0.0;
    const auto& before = s.phase_groups[static_cast<size_t>(next - 1)];
    const auto& after = s.phase_groups[static_cast<size_t>(next)];
    for (const auto& e : v.stages().front().engines) {
        if (contains(before, e.group_id) && !contains(after, e.group_id)) {
            lead = std::max(lead, 0.5 * e.performance->config().shutdown_s);
        }
    }
    return lead;
}

constexpr double prediction_step_s = 0.05;  // RK4 step of the latency prediction

// x after `duration_s` of flying `plan` from its time t0_s (planning model, the plan's body-frame thrust).
DescentState predict_on_plan(const DescentModel& model, const DescentPlan& plan, double t0_s, double duration_s,
                             DescentState x) {
    const int steps = std::max(1, static_cast<int>(std::ceil(duration_s / prediction_step_s)));
    const double h = duration_s / steps;
    for (int i = 0; i < steps; ++i) {
        const double t = t0_s + i * h;
        const int phase = plan.phase_at(t + 0.5 * h);
        const auto f = [&](const DescentState& y, double tt) { return model.derivative(y, plan.thrust_at(tt), phase); };
        const auto k1 = f(x, t);
        const auto k2 = f(x + k1 * (0.5 * h), t + 0.5 * h);
        const auto k3 = f(x + k2 * (0.5 * h), t + 0.5 * h);
        const auto k4 = f(x + k3 * h, t + h);
        x = x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
    }
    const auto q = state_attitude(x).normalized();
    x[descent_index::attitude] = q.w;
    x[descent_index::attitude + 1] = q.x;
    x[descent_index::attitude + 2] = q.y;
    x[descent_index::attitude + 3] = q.z;
    return x;
}

// The latest measured first-plan wall time per kind (seconds; 0: none yet).
std::array<std::atomic<double>, 3>& measured_first_plan_s() {
    static std::array<std::atomic<double>, 3> measured{};
    return measured;
}

// Diagnostics (STARSIM_DEBUG_PLAN_PATH): one line per plan, then its nodes in the plan frame.
void print_plan_path(const DescentPlan& plan, double since_first_s, double solve_ms) {
    if (!std::getenv("STARSIM_DEBUG_PLAN_PATH")) return;
    std::printf("[plan] t %.2f goal %d conv %d it %d vc %.3g solve_ms %.1f overlap %.2f dur %.2f/%.2f/%.2f\n", since_first_s,
                static_cast<int>(plan.goal), plan.converged, plan.iterations, plan.virtual_control, solve_ms, plan.keep_out_overlap_m,
                plan.reference.duration_s[0], plan.reference.duration_s[1], plan.reference.duration_s[2]);
    for (const auto& x : plan.reference.x) {
        const auto q = state_attitude(x);
        const Vec3 axis = q.rotate({1.0, 0.0, 0.0});
        std::printf("[node] %.3f %.3f %.3f %.3f %.4f %.4f %.1f\n", x[descent_index::position], x[descent_index::position + 1],
                    x[descent_index::position + 2], axis.x, axis.y, axis.z, x[descent_index::mass]);
    }
}

}  // namespace

double first_plan_hold_s(FirstPlanKind kind) {
    // [M] references before a first measurement: the booster's synchronous first plans 0.25-0.95 s (Task 3), the
    // ship's first plan on the worker 1.87 s (Task 11b, Mach 2.6 paced), the HLS approach plan 1.0 s (fix round 1).
    constexpr std::array<double, 3> reference_s{0.95, 1.87, 1.0};
    const auto k = static_cast<size_t>(kind);
    const double measured = measured_first_plan_s()[k].load();
    const double wall = measured > 0.0 ? measured : reference_s[k];
    return std::clamp(first_plan_hold_margin * wall, ScvxLandingParameters{}.plan_latency_s, first_plan_hold_max_s);
}

void record_first_plan_wall_s(FirstPlanKind kind, double wall_s) {
    if (wall_s > 0.0) measured_first_plan_s()[static_cast<size_t>(kind)].store(wall_s);
}

double hold_thrust_n(const DescentModel& model, double thrust_n, bool throttle_down, double t_s) {
    const auto& set = model.engine_sets[0];
    if (!throttle_down || !(set.throttle_rate_per_s > 0.0)) return thrust_n;
    return std::max(thrust_n - set.throttle_rate_per_s * set.thrust_max_n * t_s, std::min(set.thrust_min_n, thrust_n));
}

DescentState predict_hold(const DescentModel& model, const DescentState& x0, const Vec3& thrust_body_n,
                          bool engines_running, double duration_s, bool throttle_down) {
    if (!(duration_s > 0.0)) return x0;
    const double u0 = norm(thrust_body_n);
    const auto f = [&](const DescentState& y, double t) {
        Vec3 u;
        if (engines_running && u0 > 0.0) u = thrust_body_n * (hold_thrust_n(model, u0, throttle_down, t) / u0);
        auto d = model.derivative(y, u, 0);
        if (!engines_running) d[descent_index::mass] = 0.0;  // the model's flow has a pressure term that burns at zero thrust
        for (int i = descent_index::attitude; i < descent_index::rate + 3; ++i) d[i] = 0.0;  // held by the controller
        return d;
    };
    const int steps = std::max(1, static_cast<int>(std::ceil(duration_s / prediction_step_s)));
    const double h = duration_s / steps;
    DescentState x = x0;
    for (int i = 0; i < steps; ++i) {
        const double t = i * h;
        const auto k1 = f(x, t);
        const auto k2 = f(x + k1 * (0.5 * h), t + 0.5 * h);
        const auto k3 = f(x + k2 * (0.5 * h), t + 0.5 * h);
        const auto k4 = f(x + k3 * h, t + h);
        x = x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
    }
    return x;
}

double landing_throttle(const vehicle::Vehicle& v, const std::vector<std::string>& run_groups, double thrust_n,
                        double pressure_pa) {
    // Thrust of an engine: throttle x vacuum thrust - ambient pressure x exit area; the pressure loss does not scale
    // with the throttle, so the throttle solves sum spool (throttle F_vac - loss) = thrust - residual.
    double vacuum = 0.0, loss = 0.0, residual = 0.0;
    for (const auto& e : v.stages().front().engines) {
        if (e.starved) continue;
        if (contains(run_groups, e.group_id)) {
            const double spool = std::max(e.spool, min_spool_for_throttle);
            const double f_vac = e.performance->thrust_n(1.0, 0.0);
            vacuum += spool * f_vac;
            loss += spool * (f_vac - e.performance->thrust_n(1.0, pressure_pa));
        } else {
            residual += e.spool * e.performance->thrust_n(e.throttle, pressure_pa);  // still starting or running down
        }
    }
    return vacuum > 0.0 ? std::clamp((thrust_n - residual + loss) / vacuum, 0.0, 1.0) : 1.0;
}

ScvxLandingGuidance::ScvxLandingGuidance(LandingSite site, ScvxLandingParameters parameters, PlanFunction plan)
    : site_(std::move(site)),
      parameters_(std::move(parameters)),
      frame_(plan_frame(site_)),
      replanner_(parameters_.plan_latency_s, std::move(plan)) {}

PlanRequest ScvxLandingGuidance::request(const DescentSnapshot& snapshot, int start_phase) const {
    PlanRequest r;
    r.model = snapshot.model;
    r.initial = snapshot.initial;
    r.start_phase = start_phase;
    r.target = aborting_ ? snapshot.abort_target : snapshot.catch_target;
    if (!sea_required_) r.target.sea_min_distance_m = 0.0;
    r.settings = parameters_.settings;
    r.settings.limits.min_mass_kg = snapshot.min_mass_kg;
    r.settings.limits.phase0_initial_spool = snapshot.phase0_spool;
    r.settings.limits.phase0_startup_s = snapshot.startup_s;
    // ignition engines the next set drops run until they are at thrust (the rest of their start-up)
    const auto& groups = parameters_.snapshot.phase_groups;
    const bool drops = std::any_of(groups[0].begin(), groups[0].end(),
                                   [&](const std::string& g) { return !contains(groups[1], g); });
    // A first plan after a hold (real-time play): it starts at the hand-over, from the state the hold leads to.
    const double hold_s = !plan_ && first_initial_ ? first_hold_.duration_s : 0.0;
    if (hold_s > 0.0) {
        r.initial = *first_initial_;
        r.start_offset_s = hold_s;
    }
    if (start_phase == 0 && drops && !aborting_) {
        // counted from the plan's start (a re-plan starts the latency after the snapshot)
        // (a hold with the engines off: their start-up begins at the hand-over, the plan's start)
        const double offset_s = plan_ ? parameters_.plan_latency_s : first_hold_.engines_running ? hold_s : 0.0;
        r.settings.limits.phase0_min_s =
            std::max((1.0 - std::min(snapshot.phase0_spool, 1.0)) * snapshot.startup_s - offset_s, 0.0);
    }
    if (plan_) {
        r.start_offset_s = parameters_.plan_latency_s;
        r.initial = predict_on_plan(snapshot.model, *plan_, snapshot.epoch - plan_epoch_, r.start_offset_s, snapshot.initial);
    }
    if (plan_ && plan_->goal == r.target.goal) {
        r.warm_start = plan_;
        r.warm_start_elapsed_s = snapshot.epoch + r.start_offset_s - plan_epoch_;
    }
    // A re-plan, or a first plan whose start set already burns at thrust (the ship's flip engines at the flip
    // throttle): the engines are running, their thrust changes at the throttle rate from what they give now. Without
    // it the ship's first plan after the flip asked 8.7 m/s^2 at once while the three engines came down from 0.8 at
    // 0.5 / s (~19 m/s^2 flown on 212 t): ~9 m/s of sink braked off unplanned, the catch ~3 s later than planned.
    const bool running = !plan_ && snapshot.thrust_n > 0.0 && snapshot.phase0_spool >= 1.0;
    if (plan_ || running) {
        // after a hold: the thrust it ends on (the hold's own throttle-down)
        r.settings.limits.initial_thrust_n =
            hold_s > 0.0 ? hold_thrust_n(snapshot.model, snapshot.thrust_n, first_hold_.throttle_down, hold_s) : snapshot.thrust_n;
        r.settings.limits.initial_thrust_window_s = plan_ ? parameters_.plan_latency_s : 0.0;
    }
    if (aborting_) r.settings.limits.min_phase_s = {};  // mid-flight: no forced phase durations
    const double reserve_fraction = aborting_ ? parameters_.abort_reserve_fraction
                                    : plan_   ? parameters_.replan_reserve_fraction
                                              : 0.0;
    r.settings.limits.soft_reserve_kg = reserve_fraction * parameters_.snapshot.reserve_kg;
    r.snapshot_epoch = snapshot.epoch;
    return r;
}

void ScvxLandingGuidance::check_sea_reach(const DescentSnapshot& snapshot, int engine_set) {
    if (!aborting_ || sea_reach_checked_ || !sea_required_ || !site_.sea) return;
    sea_reach_checked_ = true;  // once, at the abort: the plans then fly it (their cost is not this bound)
    const auto& x = snapshot.initial;
    const auto& target = snapshot.abort_target;
    const Vec3 seaward = math::normalized(target.sea_target_m);
    const auto& set = snapshot.model.engine_sets[static_cast<size_t>(engine_set)];
    const auto& limits = parameters_.settings.limits;
    SeaReachInput in;
    in.distance_m = target.sea_min_distance_m - dot(state_position(x), seaward);
    in.seaward_speed_mps = dot(state_velocity(x), seaward);
    in.height_m = x[descent_index::position] - target.ground_cg_height_m;
    in.sink_mps = -x[descent_index::velocity];
    in.mass_kg = x[descent_index::mass];
    in.max_accel_mps2 = set.thrust_max_n / in.mass_kg;
    in.exhaust_velocity_mps = 1.0 / set.mass_flow_per_thrust_s_m;
    in.gravity_mps2 = norm(snapshot.model.gravity_mps2);
    in.tilt_max_rad = limits.tilt_max_rad[static_cast<size_t>(engine_set)];
    in.tilt_rate_radps = limits.rate_max_radps[static_cast<size_t>(engine_set)];
    const auto estimate = estimate_sea_reach(in);
    // The propellant above the floor the abort plans keep (the part of the reserve they may not use).
    const double floor_kg = snapshot.min_mass_kg - parameters_.abort_reserve_fraction * parameters_.snapshot.reserve_kg;
    const double usable_kg = in.mass_kg - floor_kg;
    if (std::getenv("STARSIM_DEBUG_SCVX")) {  // diagnostics: the reach decision
        std::printf("[reach] distance %.0f speed %.1f h %.0f sink %.1f m %.0f: T %.1f s dv %.0f need %.0f usable %.0f kg\n",
                    in.distance_m, in.seaward_speed_mps, in.height_m, in.sink_mps, in.mass_kg, estimate.time_s,
                    estimate.delta_v_mps, estimate.propellant_kg, usable_kg);
    }
    if (estimate.propellant_kg * parameters_.sea_reach_margin <= usable_kg) return;
    sea_required_ = false;
}

void ScvxLandingGuidance::accept(const PlanOutcome& outcome) {
    if (std::getenv("STARSIM_DEBUG_SCVX")) {  // diagnostics: plans and tracking
        const auto& p = *outcome.plan;
        const auto& x0 = p.reference.x.front();
        std::printf("[scvx] snap %.2f phase %d goal %d conv %d it %d vc %.3g step %.3g defect %.3g status %d dur %.2f/%.2f/%.2f "
                    "h %.1f v %.1f,%.1f,%.1f m %.0f tilt %.1f deg rate %.3f lat %.1f,%.1f\n",
                    outcome.snapshot_epoch - plan_epoch_, p.start_phase, static_cast<int>(p.goal), p.converged, p.iterations,
                    p.virtual_control, p.last_step, p.nonlinear_defect, static_cast<int>(p.status), p.reference.duration_s[0],
                    p.reference.duration_s[1], p.reference.duration_s[2], x0[1], x0[4], x0[5], x0[6], x0[0],
                    2.0 * std::asin(std::min(1.0, std::hypot(x0[9], x0[10]))) * 57.29577951308232, std::hypot(x0[12], x0[13]),
                    x0[2], x0[3]);
    }
    print_plan_path(*outcome.plan, outcome.snapshot_epoch + outcome.start_offset_s - first_plan_epoch_, outcome.solve_ms);
    ++stats_.plans;
    stats_.last_solve_ms = outcome.solve_ms;
    stats_.max_solve_ms = std::max(stats_.max_solve_ms, outcome.solve_ms);
    const bool stale = aborting_ && outcome.plan->goal != DescentGoal::Abort;  // requested before the abort
    const bool beyond = beyond_limits(*outcome.plan);
    const bool strikes = abort_strikes(*outcome.plan);
    if (outcome.plan->converged && !stale && !beyond && !strikes) {
        plan_ = outcome.plan;
        plan_epoch_ = outcome.snapshot_epoch + outcome.start_offset_s;
        consecutive_failures_ = 0;
        consecutive_unconverged_ = 0;
        return;
    }
    if (outcome.plan->converged && !beyond && !strikes) return;
    ++stats_.failures;
    // Not even without the water: no abort plan clears the structures from here (the upright descent takes over).
    if (strikes && !sea_required_) {
        abort_cannot_clear_ = true;
        return;
    }
    if (outcome.plan->goal == DescentGoal::Abort && sea_required_) {
        // One failed re-plan does not make the water unreachable while a converged abort plan to it is being flown
        // (a far shore leaves little margin: the re-plan at the end of the tilt-up stalls for a cycle or two).
        if (plan_ && plan_->goal == DescentGoal::Abort && ++consecutive_unconverged_ < parameters_.unconverged_before_abort) return;
        // The water beyond the shoreline is out of reach from here: the next abort plans only pull towards it.
        sea_required_ = false;
        next_request_ = plan_epoch_;
        return;
    }
    // Re-plans that keep failing leave the vehicle on an ever older plan: give the target up (the abort plans for the
    // sea), even when each failure alone looked recoverable.
    if (!aborting_ && ++consecutive_unconverged_ >= parameters_.unconverged_before_abort && site_.sea) {
        aborting_ = true;
        return;
    }
    // Only an unreachable catch counts towards the abort: a re-plan that ran out of iterations while the dynamics
    // nearly hold leaves the previous plan in charge.
    if (!beyond && outcome.plan->virtual_control < parameters_.settings.stall_virtual_control) return;
    if (++consecutive_failures_ >= parameters_.failures_before_abort) aborting_ = true;
}

bool ScvxLandingGuidance::beyond_limits(const DescentPlan& plan) const {
    // Only converged plans (an unconverged plan's slacks mean nothing) and only where the sea offers an abort: without
    // one (the Moon) the plan is flown as it is.
    return site_.sea && plan.converged && plan.goal != DescentGoal::Abort && !plan.feasible(parameters_.settings);
}

bool ScvxLandingGuidance::abort_strikes(const DescentPlan& plan) const {
    return plan.converged && plan.goal == DescentGoal::Abort && plan.keep_out_overlap_m > 0.0;
}

bool ScvxLandingGuidance::request_abort() {
    if (!site_.sea) return false;
    if (aborting_) return true;
    aborting_ = true;
    next_request_ = plan_epoch_;  // re-plan on the next update
    return true;
}

bool ScvxLandingGuidance::start(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch) {
    begin(body, env, epoch, false);
    const bool started = *poll(epoch);  // planned at once: decided
    if (std::getenv("STARSIM_DEBUG_REALTIME")) {  // diagnostics: the sim thread's hold
        std::printf("[realtime] first plan at once: %.0f ms on the sim thread\n", 1000.0 * first_wall_s_);
    }
    return started;
}

void ScvxLandingGuidance::begin(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch,
                                bool background, const FirstPlanHold& hold) {
    if (first_plan_pending() || replanner_.busy()) {
        throw std::logic_error("ScvxLandingGuidance::begin while a first plan is under way");
    }
    first_wall_start_ = std::chrono::steady_clock::now();  // diagnostics: the first plan's wall time
    first_snapshot_ = take_descent_snapshot(body, env, epoch, site_, frame_, parameters_.snapshot);
    model_ = first_snapshot_->model;
    first_epoch_ = epoch;
    first_hold_ = hold;
    first_handover_ = epoch + std::max(hold.duration_s, 0.0);
    first_initial_.reset();
    if (hold.duration_s > 0.0) {
        const auto& s = *first_snapshot_;
        const Vec3 thrust = hold.thrust_body_n.value_or(Vec3{s.thrust_n, 0.0, 0.0});
        first_initial_ =
            predict_hold(s.model, s.initial, thrust, hold.engines_running, hold.duration_s, hold.throttle_down);
    }
    first_stage_ = 0;
    first_background_ = background;
    check_sea_reach(*first_snapshot_, 0);  // an abort requested before the first plan
    submit_first();
    first_begin_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - first_wall_start_).count();
}

void ScvxLandingGuidance::submit_first() {
    auto r = request(*first_snapshot_, 0);
    if (first_background_) {
        replanner_.submit(std::move(r), 0.0);  // collected as soon as it is there; it takes effect at the hand-over
    } else {
        first_outcome_ = replanner_.plan_now(r);
    }
}

std::optional<bool> ScvxLandingGuidance::poll(time::Epoch now) {
    // At the hand-over a plan still under way is waited for (the synchronous solve): the hold never runs longer.
    const bool due = now - first_handover_ >= -epoch_tolerance_s;
    const auto poll_start = std::chrono::steady_clock::now();
    while (first_snapshot_) {
        std::optional<PlanOutcome> outcome = std::exchange(first_outcome_, std::nullopt);
        if (!outcome) outcome = replanner_.collect(now, due || !parameters_.realtime);
        if (!outcome) return std::nullopt;  // still planning
        if (const auto decided = first_step(*outcome)) {
            first_wall_s_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - first_wall_start_).count();
            first_decided_ = decided;
        }
    }
    if (!due || !first_decided_) return std::nullopt;
    if (first_background_ && std::getenv("STARSIM_DEBUG_REALTIME")) {  // diagnostics: the sim thread's share
        std::printf("[realtime] first plan: begin %.1f ms on the sim thread, worker %.0f ms, hold %.2f s, waited %.0f ms at "
                    "the hand-over\n",
                    first_begin_ms_, 1000.0 * first_wall_s_, first_hold_.duration_s,
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - poll_start).count());
    }
    return std::exchange(first_decided_, std::nullopt);
}

std::optional<bool> ScvxLandingGuidance::first_step(const PlanOutcome& outcome) {
    const auto& snapshot = *first_snapshot_;
    // An abort requested while the catch plan was computed (real-time play): plan the abort from the same state instead.
    if (aborting_ && outcome.plan->goal != DescentGoal::Abort) {
        first_stage_ = 1;
        check_sea_reach(snapshot, 0);
        submit_first();
        return std::nullopt;
    }
    if (first_stage_ < 1) {
        first_stage_ = 1;
        if ((!outcome.plan->converged || beyond_limits(*outcome.plan)) && site_.sea && !aborting_) {
            if (std::getenv("STARSIM_DEBUG_SCVX")) {  // diagnostics: the refused catch plan
                const auto& p = *outcome.plan;
                const auto& x0 = snapshot.initial;
                std::printf("[scvx] first catch plan refused: it %d vc %.3g step %.3g defect %.3g status %d dur %.2f/%.2f/%.2f | "
                            "h %.1f lat %.1f,%.1f v %.1f,%.1f,%.1f m %.0f min %.0f\n",
                            p.iterations, p.virtual_control, p.last_step, p.nonlinear_defect, static_cast<int>(p.status),
                            p.reference.duration_s[0], p.reference.duration_s[1], p.reference.duration_s[2], x0[1], x0[2],
                            x0[3], x0[4], x0[5], x0[6], x0[0], snapshot.min_mass_kg);
            }
            // The catch is out of reach from the start: plan the abort to the sea right away.
            ++stats_.plans;
            ++stats_.failures;
            aborting_ = true;
            check_sea_reach(snapshot, 0);
            submit_first();
            return std::nullopt;
        }
    }
    if (first_stage_ < 2) {
        first_stage_ = 2;
        if ((!outcome.plan->converged || abort_strikes(*outcome.plan)) && aborting_ && sea_required_) {
            // Not even the water beyond the shoreline is in reach: an abort that only pulls towards the sea.
            ++stats_.plans;
            ++stats_.failures;
            sea_required_ = false;
            submit_first();
            return std::nullopt;
        }
    }
    if (std::getenv("STARSIM_DEBUG_SCVX")) {  // diagnostics: plans and tracking
        const auto& p = *outcome.plan;
        std::printf("[scvx] first: conv %d it %d vc %.3g step %.3g defect %.3g status %d dur %.2f/%.2f/%.2f spool %.2f "
                    "solve %.0f ms, start %.0f ms%s\n",
                    p.converged, p.iterations, p.virtual_control, p.last_step, p.nonlinear_defect, static_cast<int>(p.status),
                    p.reference.duration_s[0], p.reference.duration_s[1], p.reference.duration_s[2], snapshot.phase0_spool,
                    outcome.solve_ms,
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - first_wall_start_).count(),
                    first_background_ ? " (background)" : "");
    }
    const time::Epoch epoch = first_handover_;  // the snapshot itself without a hold
    first_snapshot_.reset();
    first_plan_epoch_ = epoch;
    print_plan_path(*outcome.plan, 0.0, outcome.solve_ms);
    ++stats_.plans;
    stats_.last_solve_ms = stats_.max_solve_ms = outcome.solve_ms;
    if (!outcome.plan->converged || abort_strikes(*outcome.plan)) {
        ++stats_.failures;
        return false;
    }
    plan_ = outcome.plan;
    plan_epoch_ = epoch;
    next_request_ = epoch + parameters_.replan_interval_s;
    return true;
}

std::optional<time::Epoch> ScvxLandingGuidance::plan_end_epoch() const {
    if (!plan_) return std::nullopt;
    return plan_epoch_ + plan_->duration_s();
}

ScvxLandingCommand ScvxLandingGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                               time::Epoch epoch) {
    if (auto outcome = replanner_.collect(epoch, !parameters_.realtime)) accept(*outcome);

    const double t = epoch - plan_epoch_;
    const double time_to_go = plan_->duration_s() - t;
    const bool abort_pending = aborting_ && plan_->goal != DescentGoal::Abort;  // no abort plan yet
    if (epoch - next_request_ >= -epoch_tolerance_s && !replanner_.busy() &&
        (time_to_go > parameters_.min_replan_time_to_go_s || abort_pending)) {
        const auto snapshot = take_descent_snapshot(body, env, epoch, site_, frame_, parameters_.snapshot);
        model_ = snapshot.model;
        const int phase = std::max(engine_set_, plan_->phase_at(t + parameters_.plan_latency_s));
        check_sea_reach(snapshot, phase);
        replanner_.submit(request(snapshot, phase));
        next_request_ = epoch + parameters_.replan_interval_s;
    }

    const auto body_frame = site_body_frame(site_, env, epoch);
    const auto& earth = body_frame.transform;  // the site body-fixed frame (Earth: ITRF93, Moon: MOON_ME)
    const auto x = plan_state(body, body_frame, frame_);
    const auto ref = sample_plan(*plan_, model_, epoch - plan_epoch_, parameters_.snapshot.catch_sink_mps);
    while (engine_set_ + 1 < descent_phases) {
        // Slower than the switch window on this set: switch now (its engines at minimum throttle would climb); else
        // at the planned switch, the run-down of the dropped engines ahead.
        const int next = engine_set_ + 1;
        // Only a set that exceeds the weight at minimum throttle climbs (every Earth set; not the HLS's single Raptor
        // on the Moon, which hovers on it until the planned switch).
        const bool climbs = model_.engine_sets[static_cast<size_t>(engine_set_)].thrust_min_n >
                            x[descent_index::mass] * norm(model_.gravity_mps2);
        const bool too_slow =
            climbs && -x[descent_index::velocity] < parameters_.settings.limits.switches[static_cast<size_t>(engine_set_)].min_sink_mps;
        const bool planned = ref.phase >= next ||
                             plan_->phase_at(epoch - plan_epoch_ + switch_lead_s(body.vehicle(), parameters_.snapshot, next)) >= next;
        if (!too_slow && !planned) break;
        engine_set_ = next;
    }
    const auto track = tracking_command(ref, state_position(x), state_velocity(x), parameters_.tracking);
    const Vec3& axis_plan = track.axis;
    if (std::getenv("STARSIM_DEBUG_SCVX")) {  // diagnostics: plans and tracking
        static int tick = 0;  // print every 40 ticks (0.2 s)
        if (++tick % 40 == 0 || ref.finished) {
            const Vec3 r = state_position(x), v = state_velocity(x);
            const auto q = state_attitude(x);
            const Vec3 ax = q.rotate({1.0, 0.0, 0.0});
            std::printf("  t %.2f ph %d fin %d ref r %.1f,%.2f,%.2f v %.1f,%.2f,%.2f | r %.1f,%.2f,%.2f v %.1f,%.2f,%.2f "
                        "| a %.1f,%.2f,%.2f axis %.3f,%.3f ref %.3f,%.3f m %.0f\n",
                        epoch - plan_epoch_, ref.phase, ref.finished, ref.position_m.x, ref.position_m.y, ref.position_m.z,
                        ref.velocity_mps.x, ref.velocity_mps.y, ref.velocity_mps.z, r.x, r.y, r.z, v.x, v.y, v.z, track.thrust_acceleration_mps2, axis_plan.y,
                        axis_plan.z, ax.y, ax.z, ref.axis.y, ref.axis.z, x[0]);
        }
    }

    ScvxLandingCommand cmd;
    cmd.run_groups = parameters_.snapshot.phase_groups[static_cast<size_t>(engine_set_)];
    const Vec3 r_fixed = body_frame.to_fixed(body.state().position_gcrf_m);
    const double pressure = site_.body == ephemeris::Body::Moon
                                ? 0.0
                                : env.atmosphere.at(frames::fixed_to_geodetic(r_fixed).height_m).pressure_pa;
    cmd.throttle = landing_throttle(body.vehicle(), cmd.run_groups, x[descent_index::mass] * track.thrust_acceleration_mps2, pressure);
    cmd.axis_gcrf = earth.rotation * frame_.direction_to_itrf(axis_plan);
    // Planned rotation in the vehicle's own body axes (its roll differs from the plan's).
    const auto& attitude = body.state().attitude;
    const auto to_body = [&](const Vec3& plan_vector) {
        return attitude.inverse_rotate(earth.rotation * frame_.direction_to_itrf(plan_vector));
    };
    const Vec3 omega_body = attitude.inverse_rotate(body_frame.rotation_gcrf_radps);
    cmd.feedforward.rate_body_radps = to_body(ref.rate_plan_radps) + omega_body;
    cmd.feedforward.acceleration_body_radps2 = to_body(ref.angular_acceleration_plan_radps2);
    if (site_.catch_target && !aborting_) {
        const Vec3 up_itrf = frame_.direction_to_itrf({1.0, 0.0, 0.0});
        const Vec3 body_y = transpose(earth.rotation) * body.state().attitude.rotate({0.0, 1.0, 0.0});
        cmd.roll_reference_gcrf = earth.rotation * catch_roll_reference(*site_.catch_target, up_itrf, body_y);
    }
    if (parameters_.lqr_lateral) {
        // Lateral LQR per plan axis (position, velocity, tilt, tilt rate against the plan; input: tilt acceleration),
        // plus the plan's tilt acceleration as feed-forward. Tilt of the body axis towards y/z: axis'' = alpha x axis.
        const auto q = state_attitude(x);
        const auto mass = body.vehicle().mass_properties();
        Vec3 axis = q.rotate({1.0, 0.0, 0.0});  // the thrust axis (plan_state)
        if (parameters_.switch_prerotation_s > 0.0 && engine_set_ + 1 < descent_phases) {
            double t_switch = 0.0;  // planned switch from this set, plan time
            for (int p = 0; p <= engine_set_; ++p) t_switch += plan_->reference.duration_s[static_cast<size_t>(p)];
            const double blend = std::clamp(1.0 - (t_switch - t) / parameters_.switch_prerotation_s, 0.0, 1.0);
            if (blend > 0.0) {
                const auto& next = parameters_.snapshot.phase_groups[static_cast<size_t>(engine_set_ + 1)];
                const Vec3 shift = thrust_axis_body(body.vehicle(), mass.cg_station_m, next) -
                                   thrust_axis_body(body.vehicle(), mass.cg_station_m);
                axis = axis + q.rotate(shift) * blend;  // small angles: hull and thrust axes differ by degrees
            }
        }
        const Vec3 axis_rate = cross(q.rotate(state_rate(x)), axis);
        const Vec3 ref_axis_rate = cross(ref.rate_plan_radps, ref.axis);
        const Vec3 ref_axis_accel = cross(ref.angular_acceleration_plan_radps2, ref.axis);
        const double a = std::max(track.thrust_acceleration_mps2, min_lqr_thrust_acceleration_mps2);
        const double c = mass.inertia_kg_m2.y / (mass.mass_kg * model_.gimbal_lever_m);
        // Tilt acceleration the gimbals can give at this thrust: a sin(gimbal_max) / c. The LQR's control effort
        // scale follows it (a design for more than the engines can give saturates and swings up).
        const int set = engine_set_;
        const double alpha_max = a * std::sin(model_.engine_sets[static_cast<size_t>(set)].gimbal_max_rad) / c;
        if (set != lqr_set_ || std::abs(a - lqr_a_) > lqr_retune_fraction * a ||
            std::abs(c - lqr_c_) > lqr_retune_fraction * c) {
            TvcLateralWeights weights = parameters_.lateral;
            if (parameters_.lateral_effort_fraction > 0.0) {
                const double f = set == descent_phases - 1 && parameters_.final_lateral_effort_fraction > 0.0
                                     ? parameters_.final_lateral_effort_fraction
                                     : parameters_.lateral_effort_fraction;
                weights.angular_accel_radps2 = f * alpha_max;
            }
            lqr_gains_ = tvc_lateral_gains(a, c, weights);
            lqr_a_ = a;
            lqr_c_ = c;
            lqr_set_ = set;
        }
        const auto& k = lqr_gains_.k;
        const Vec3 p = state_position(x), v = state_velocity(x);
        auto tilt_accel = [&](double e_p, double e_v, double e_t, double e_w, double ff) {
            return ff - (k[0] * e_p + k[1] * e_v + k[2] * e_t + k[3] * e_w);
        };
        const double uy = tilt_accel(p.y - ref.position_m.y, v.y - ref.velocity_mps.y, axis.y - ref.axis.y,
                                     axis_rate.y - ref_axis_rate.y, ref_axis_accel.y);
        const double uz = tilt_accel(p.z - ref.position_m.z, v.z - ref.velocity_mps.z, axis.z - ref.axis.z,
                                     axis_rate.z - ref_axis_rate.z, ref_axis_accel.z);
        // uy, uz are the net tilt acceleration, as the plan's: it contains the hull normal force's moment. The model's
        // moment at the measured state is cancelled out, the gimbals give the rest (the LQR's w' = u holds with it).
        const auto hull = model_.hull_normal_force(x);
        const Vec3 aero_alpha_plan = q.rotate(Vec3{hull.moment_body_nm.x / mass.inertia_kg_m2.x,
                                                   hull.moment_body_nm.y / mass.inertia_kg_m2.y,
                                                   hull.moment_body_nm.z / mass.inertia_kg_m2.z});
        const Vec3 aero_tilt = cross(aero_alpha_plan, q.rotate({1.0, 0.0, 0.0}));
        const double gy = uy - aero_tilt.y, gz = uz - aero_tilt.z;  // the gimbals' part
        const double u_norm = std::hypot(gy, gz);
        const double u_scale = u_norm > alpha_max ? alpha_max / u_norm : 1.0;  // within the gimbals' reach
        const double use = alpha_max > 0.0 ? u_norm / alpha_max : 0.0;
        ++stats_.lateral_ticks;
        if (u_scale < 1.0) ++stats_.clipped_ticks;
        stats_.peak_gimbal_use = std::max(stats_.peak_gimbal_use, use);
        if (time_to_go < Statistics::terminal_window_s) {
            ++stats_.terminal_ticks;
            if (u_scale < 1.0) ++stats_.terminal_clipped_ticks;
            stats_.terminal_peak_gimbal_use = std::max(stats_.terminal_peak_gimbal_use, use);
            const double feedback = alpha_max > 0.0 ? std::hypot(uy - ref_axis_accel.y, uz - ref_axis_accel.z) / alpha_max : 0.0;
            stats_.terminal_peak_feedback_use = std::max(stats_.terminal_peak_feedback_use, feedback);
        }
        // axis'' = alpha x axis, axis ~ +x. The allocation of a body with flaps (the ship) cancels its modelled
        // aerodynamic moment itself and so takes the net torque; a booster's goes to the gimbals as it is.
        const bool net_torque = body.vehicle().flap_count() > 0;
        const double ty = gy * u_scale + (net_torque ? aero_tilt.y : 0.0);
        const double tz = gz * u_scale + (net_torque ? aero_tilt.z : 0.0);
        Vec3 alpha_body = to_body(Vec3{0.0, -tz, ty});
        // Roll: towards the pin reference (free roll without one), PD in body x.
        const double wr = parameters_.roll_frequency_rad_s;
        const double roll_rate = body.state().angular_velocity_body_radps.x - omega_body.x;
        double roll_error = 0.0;
        if (norm(cmd.roll_reference_gcrf) > 0.0) {
            const Vec3 ref_body = attitude.inverse_rotate(cmd.roll_reference_gcrf);
            roll_error = std::atan2(ref_body.z, ref_body.y);
        }
        alpha_body.x = wr * wr * roll_error - 2.0 * roll_damping * wr * roll_rate;
        cmd.torque_body_nm = Vec3{mass.inertia_kg_m2.x * alpha_body.x, mass.inertia_kg_m2.y * alpha_body.y,
                                  mass.inertia_kg_m2.z * alpha_body.z};
    }
    cmd.abort = aborting_;
    cmd.time_to_go_s = std::max(time_to_go, 0.0);
    const Vec3 end = state_position(plan_->reference.x.back());
    cmd.offset_m = std::hypot(state_position(x).y - end.y, state_position(x).z - end.z);
    return cmd;
}

}  // namespace starsim::gnc
