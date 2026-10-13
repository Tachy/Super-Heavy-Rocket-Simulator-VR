#include "starsim/gnc/scvx_landing.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <utility>

#include "starsim/frames/geodetic.hpp"

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

// The engines dropped at the switch run down over their shutdown time: a linear run-down has the impulse of a step at
// half that time, so the switch is commanded this much before the planned one.
double switch_lead_s(const vehicle::Vehicle& v, const SnapshotSettings& s) {
    double lead = 0.0;
    for (const auto& e : v.stages().front().engines) {
        if (contains(s.ignition_groups, e.group_id) && !contains(s.final_groups, e.group_id)) {
            lead = std::max(lead, 0.5 * e.performance->config().shutdown_s);
        }
    }
    return lead;
}

}  // namespace

double landing_throttle(const vehicle::Vehicle& v, const std::vector<std::string>& run_groups, double thrust_n,
                        double pressure_pa) {
    double available = 0.0, residual = 0.0;
    for (const auto& e : v.stages().front().engines) {
        if (e.starved) continue;
        if (contains(run_groups, e.group_id)) {
            available += std::max(e.spool, min_spool_for_throttle) * e.performance->thrust_n(1.0, pressure_pa);
        } else {
            residual += e.spool * e.performance->thrust_n(e.throttle, pressure_pa);  // still starting or running down
        }
    }
    return available > 0.0 ? std::clamp((thrust_n - residual) / available, 0.0, 1.0) : 1.0;
}

ScvxLandingGuidance::ScvxLandingGuidance(LandingSite site, ScvxLandingParameters parameters, PlanFunction plan)
    : site_(std::move(site)),
      parameters_(std::move(parameters)),
      frame_(plan_frame(site_.latitude_rad, site_.longitude_rad, site_.height_m)),
      replanner_(parameters_.plan_latency_s, std::move(plan)) {}

PlanRequest ScvxLandingGuidance::request(const DescentSnapshot& snapshot, int start_phase) const {
    PlanRequest r;
    r.model = snapshot.model;
    r.initial = snapshot.initial;
    r.start_phase = start_phase;
    r.target = aborting_ ? snapshot.abort_target : snapshot.catch_target;
    r.settings = parameters_.settings;
    r.settings.limits.min_mass_kg = snapshot.min_mass_kg;
    r.settings.limits.phase0_initial_spool = snapshot.phase0_spool;
    r.settings.limits.phase0_startup_s = snapshot.startup_s;
    if (plan_ && plan_->goal == r.target.goal) {
        r.warm_start = plan_;
        r.warm_start_elapsed_s = snapshot.epoch - plan_epoch_;
    }
    if (aborting_) r.settings.limits.min_phase_s = {0.0, 0.0};  // mid-flight: no forced phase durations
    r.snapshot_epoch = snapshot.epoch;
    return r;
}

void ScvxLandingGuidance::accept(const PlanOutcome& outcome) {
    if (std::getenv("STARSIM_DEBUG_SCVX")) {  // diagnostics: plans and tracking
        const auto& p = *outcome.plan;
        const auto& x0 = p.reference.x.front();
        std::printf("[scvx] snap %.2f phase %d goal %d conv %d it %d vc %.3g step %.3g defect %.3g status %d dur %.2f/%.2f "
                    "h %.1f v %.1f,%.1f,%.1f m %.0f tilt %.1f deg rate %.3f lat %.1f,%.1f\n",
                    outcome.snapshot_epoch - plan_epoch_, p.start_phase, static_cast<int>(p.goal), p.converged, p.iterations,
                    p.virtual_control, p.last_step, p.nonlinear_defect, static_cast<int>(p.status), p.reference.duration_s[0],
                    p.reference.duration_s[1], x0[1], x0[4], x0[5], x0[6], x0[0],
                    2.0 * std::asin(std::min(1.0, std::hypot(x0[9], x0[10]))) * 57.29577951308232, std::hypot(x0[12], x0[13]),
                    x0[2], x0[3]);
    }
    ++stats_.plans;
    stats_.last_solve_ms = outcome.solve_ms;
    stats_.max_solve_ms = std::max(stats_.max_solve_ms, outcome.solve_ms);
    const bool stale = aborting_ && outcome.plan->goal == DescentGoal::Catch;  // requested before the abort
    if (outcome.plan->converged && !stale) {
        plan_ = outcome.plan;
        plan_epoch_ = outcome.snapshot_epoch;
        consecutive_failures_ = 0;
        return;
    }
    if (outcome.plan->converged) return;
    ++stats_.failures;
    // Only an unreachable catch counts towards the abort: a re-plan that ran out of iterations while the dynamics
    // nearly hold leaves the previous plan in charge.
    if (outcome.plan->virtual_control < parameters_.settings.stall_virtual_control) return;
    if (++consecutive_failures_ >= parameters_.failures_before_abort) aborting_ = true;
}

bool ScvxLandingGuidance::start(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch) {
    const auto snapshot = take_descent_snapshot(body, env, epoch, site_, frame_, parameters_.snapshot);
    model_ = snapshot.model;
    const auto outcome = replanner_.plan_now(request(snapshot, 0));
    if (std::getenv("STARSIM_DEBUG_SCVX")) {  // diagnostics: plans and tracking
        const auto& p = *outcome.plan;
        std::printf("[scvx] first: conv %d it %d vc %.3g step %.3g defect %.3g status %d dur %.2f/%.2f spool %.2f\n",
                    p.converged, p.iterations, p.virtual_control, p.last_step, p.nonlinear_defect, static_cast<int>(p.status),
                    p.reference.duration_s[0], p.reference.duration_s[1], snapshot.phase0_spool);
    }
    ++stats_.plans;
    stats_.last_solve_ms = stats_.max_solve_ms = outcome.solve_ms;
    if (!outcome.plan->converged) {
        ++stats_.failures;
        return false;
    }
    plan_ = outcome.plan;
    plan_epoch_ = epoch;
    next_request_ = epoch + parameters_.replan_interval_s;
    return true;
}

ScvxLandingCommand ScvxLandingGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                               time::Epoch epoch) {
    if (auto outcome = replanner_.collect(epoch, !parameters_.realtime)) accept(*outcome);

    const double t = epoch - plan_epoch_;
    const double time_to_go = plan_->duration_s() - t;
    const bool abort_pending = aborting_ && plan_->goal == DescentGoal::Catch;  // no abort plan yet
    if (epoch - next_request_ >= -epoch_tolerance_s && !replanner_.busy() &&
        (time_to_go > parameters_.min_replan_time_to_go_s || abort_pending)) {
        const auto snapshot = take_descent_snapshot(body, env, epoch, site_, frame_, parameters_.snapshot);
        model_ = snapshot.model;
        const int phase = final_engines_ ? 1 : plan_->phase_at(t);
        replanner_.submit(request(snapshot, phase));
        next_request_ = epoch + parameters_.replan_interval_s;
    }

    const auto earth = env.earth_orientation.fixed_to_gcrf(epoch);
    const auto x = plan_state(body, earth, frame_);
    // Slower than the hand-over sink rate on the ignition set: switch now (thirteen engines would climb).
    if (!final_engines_ && -x[descent_index::velocity] < parameters_.settings.limits.switch_min_sink_mps) final_engines_ = true;
    const auto ref = sample_plan(*plan_, model_, epoch - plan_epoch_, parameters_.snapshot.catch_sink_mps);
    if (ref.phase == 1 || plan_->phase_at(epoch - plan_epoch_ + switch_lead_s(body.vehicle(), parameters_.snapshot)) == 1) {
        final_engines_ = true;
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
    cmd.run_groups = final_engines_ ? parameters_.snapshot.final_groups : parameters_.snapshot.ignition_groups;
    const Vec3 r_fixed = transpose(earth.rotation) * body.state().position_gcrf_m;
    const double pressure = env.atmosphere.at(frames::fixed_to_geodetic(r_fixed).height_m).pressure_pa;
    cmd.throttle = landing_throttle(body.vehicle(), cmd.run_groups, x[descent_index::mass] * track.thrust_acceleration_mps2, pressure);
    cmd.axis_gcrf = earth.rotation * frame_.direction_to_itrf(axis_plan);
    // Planned rotation in the vehicle's own body axes (its roll differs from the plan's).
    const auto& attitude = body.state().attitude;
    const auto to_body = [&](const Vec3& plan_vector) {
        return attitude.inverse_rotate(earth.rotation * frame_.direction_to_itrf(plan_vector));
    };
    const Vec3 omega_body = attitude.inverse_rotate(vehicle::earth_rotation_vector(earth));
    cmd.feedforward.rate_body_radps = to_body(ref.rate_plan_radps) + omega_body;
    cmd.feedforward.acceleration_body_radps2 = to_body(ref.angular_acceleration_plan_radps2);
    if (site_.catch_target && !aborting_) {
        const Vec3 up_itrf = frame_.direction_to_itrf({1.0, 0.0, 0.0});
        const Vec3 body_y = transpose(earth.rotation) * body.state().attitude.rotate({0.0, 1.0, 0.0});
        cmd.roll_reference_gcrf = earth.rotation * catch_roll_reference(site_.catch_target->across_itrf, up_itrf,
                                                                        site_.catch_target->pin_azimuth_rad, body_y);
    }
    if (parameters_.lqr_lateral) {
        // Lateral LQR per plan axis (position, velocity, tilt, tilt rate against the plan; input: tilt acceleration),
        // plus the plan's tilt acceleration as feed-forward. Tilt of the body axis towards y/z: axis'' = alpha x axis.
        const auto q = state_attitude(x);
        const Vec3 axis = q.rotate({1.0, 0.0, 0.0});
        const Vec3 axis_rate = cross(q.rotate(state_rate(x)), axis);
        const Vec3 ref_axis_rate = cross(ref.rate_plan_radps, ref.axis);
        const Vec3 ref_axis_accel = cross(ref.angular_acceleration_plan_radps2, ref.axis);
        const auto mass = body.vehicle().mass_properties();
        const double a = std::max(track.thrust_acceleration_mps2, min_lqr_thrust_acceleration_mps2);
        const double c = mass.inertia_kg_m2.y / (mass.mass_kg * model_.gimbal_lever_m);
        // Tilt acceleration the gimbals can give at this thrust: a sin(gimbal_max) / c. The LQR's control effort
        // scale follows it (a design for more than the engines can give saturates and swings up).
        const int set = final_engines_ ? 1 : 0;
        const double alpha_max = a * std::sin(model_.engine_sets[static_cast<size_t>(set)].gimbal_max_rad) / c;
        if (set != lqr_set_ || std::abs(a - lqr_a_) > lqr_retune_fraction * a ||
            std::abs(c - lqr_c_) > lqr_retune_fraction * c) {
            TvcLateralWeights weights = parameters_.lateral;
            if (parameters_.lateral_effort_fraction > 0.0) {
                weights.angular_accel_radps2 = parameters_.lateral_effort_fraction * alpha_max;
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
        const double u_norm = std::hypot(uy, uz);
        const double u_scale = u_norm > alpha_max ? alpha_max / u_norm : 1.0;  // within the gimbals' reach
        Vec3 alpha_body = to_body(Vec3{0.0, -uz * u_scale, uy * u_scale});  // axis'' = alpha x axis, axis ~ +x
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
