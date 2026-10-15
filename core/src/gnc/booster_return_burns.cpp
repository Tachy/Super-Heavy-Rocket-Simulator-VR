#include "starsim/gnc/booster_return.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"

#include "booster_return_detail.hpp"

namespace starsim::gnc {

using math::Mat3;
using math::Vec3;
using namespace booster_return_detail;

// Entry burn: retrograde (relative to the ground) on the entry engines, the axis leaning towards the drag-aware impact
// miss of the landing site (small correction).
void BoosterReturnGuidance::entry_burn(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                       time::Epoch epoch, ReturnCommand& cmd) {
    const auto& s = body.state();
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const Mat3 to_fixed = transpose(transform.rotation);
    const Vec3 v_fixed = to_fixed * (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(transform), s.position_gcrf_m));
    const Vec3 site_up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 site_fixed = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
    if (tick_counter_ % 40 == 0 || !entry_predicted_) {
        entry_predicted_ = true;
        // Shooting on the impact prediction of the rest of the burn (it carries the drag that bleeds off the sideways
        // speed in the descent). Along the ground track: the burn's delta-v (a secant step per update on a probe burn
        // that is longer by a fixed amount). Across: the lean - flown straight retrograde and leaning fully across, the
        // lean in between that cancels the cross-track miss.
        auto burn = pending_entry_burn(body.vehicle());
        const auto straight = burn ? predict_impact(body, env, epoch, site_.height_m, nullptr, &*burn, drag_scale_) : Impact{};
        if (straight.valid) {
            const Vec3 miss = site_fixed + landing_burn_lead_itrf(body, env, straight) - straight.position_itrf_m;
            const Vec3 horizontal = miss - site_up * dot(miss, site_up);
            impact_miss_m_ = norm(horizontal);
            const Vec3 track = math::normalized_or_zero(v_fixed - site_up * dot(v_fixed, site_up));
            const double along_m = dot(horizontal, track);
            const Vec3 across = horizontal - track * along_m;
            const double change_limit = parameters_.entry_max_delta_v_change * parameters_.entry_delta_v_mps;
            if (std::abs(along_m) > trim_deadband_m && change_limit > 0.0 && burn->delta_v_mps > 0.0) {
                auto longer = *burn;
                longer.delta_v_mps += entry_delta_v_probe_mps;
                const auto probe = predict_impact(body, env, epoch, site_.height_m, nullptr, &longer, drag_scale_);
                if (probe.valid) {
                    const double moved_m = dot(probe.position_itrf_m - straight.position_itrf_m, track);  // < 0: shorter
                    if (moved_m < -1.0) {
                        entry_delta_v_change_mps_ = std::clamp(
                            entry_delta_v_change_mps_ + along_m * entry_delta_v_probe_mps / moved_m, -change_limit,
                            change_limit);
                    }
                }
            }
            entry_lean_rad_ = 0.0;
            if (norm(across) > trim_deadband_m && parameters_.entry_max_lean_rad > 0.0) {
                steer_itrf_ = across / norm(across);
                burn->lean_itrf = steer_itrf_;
                burn->lean_rad = parameters_.entry_max_lean_rad;
                const auto leaning = predict_impact(body, env, epoch, site_.height_m, nullptr, &*burn, drag_scale_);
                if (leaning.valid) {
                    const double moved_m = dot(leaning.position_itrf_m - straight.position_itrf_m, steer_itrf_);
                    if (moved_m > 0.0) {
                        entry_lean_rad_ = parameters_.entry_max_lean_rad * std::min(norm(across) / moved_m, 1.0);
                    }
                }
            }
        }
    }
    Vec3 axis = -math::normalized(v_fixed);
    if (entry_lean_rad_ > 0.0) axis = math::normalized(axis + steer_itrf_ * std::tan(entry_lean_rad_));
    cmd.axis_gcrf = transform.rotation * axis;
    cmd.run_groups = parameters_.entry_groups;
    cmd.throttle = parameters_.entry_throttle;
    cmd.impact_miss_m = impact_miss_m_;
}

BoosterReturnGuidance::PendingBurn BoosterReturnGuidance::entry_burn_ahead(const vehicle::Vehicle& vehicle,
                                                                           const BoosterReturnParameters& parameters) {
    const auto engines = engine_set(vehicle, parameters.entry_groups, 0.0);
    return {parameters.entry_altitude_m, parameters.entry_delta_v_mps, parameters.entry_throttle * engines.vacuum_thrust_n,
            parameters.entry_throttle * engines.mass_flow_kgps};
}

std::optional<BoosterReturnGuidance::PendingBurn> BoosterReturnGuidance::pending_entry_burn(
    const vehicle::Vehicle& vehicle) const {
    if (parameters_.entry_altitude_m <= 0.0 || entry_done_) return std::nullopt;
    auto burn = entry_burn_ahead(vehicle, parameters_);
    burn.altitude_m += entry_altitude_change_m_;
    // While burning: the rest from here (the altitude is already crossed: it continues at once).
    if (phase_ == ReturnPhase::EntryBurn) burn.altitude_m = 1e9;
    burn.delta_v_mps = std::max(parameters_.entry_delta_v_mps + entry_delta_v_change_mps_ - entry_delta_v_mps_, 0.0);
    return burn;
}

// ZEM/ZEV landing burn: the last fallback (SCvx catch and sea-abort plans both failed: over the arms, ~1-3 m), or with
// scvx_landing off a ditching beyond the shoreline (Super Heavy has no legs).
void BoosterReturnGuidance::explicit_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                             time::Epoch epoch, ReturnCommand& cmd) {
    const auto& s = body.state();
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const Mat3 to_fixed = transpose(transform.rotation);
    const Vec3 omega = vehicle::earth_rotation_vector(transform);
    const Vec3 r_fixed = to_fixed * s.position_gcrf_m;
    const Vec3 v_fixed = to_fixed * (s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
    const auto mass = body.vehicle().mass_properties();
    const Vec3 site_up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 site_fixed = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
    const double pressure = env.atmosphere.at(frames::fixed_to_geodetic(r_fixed).height_m).pressure_pa;
    // Gravity + centrifugal acceleration in the Earth-fixed frame (Coriolis left to the feedback).
    const Vec3 omega_fixed = to_fixed * omega;
    const Vec3 g_fixed = to_fixed * env.gravity.acceleration_mps2(s.position_gcrf_m, epoch) -
                         cross(omega_fixed, cross(omega_fixed, r_fixed));
    const bool catching = site_.catch_target && !ditch_;
    // CG position at touchdown (ditching: at sea; on legs: the feet on the ground) / catch
    Vec3 r_pad = site_fixed + site_up * (mass.cg_station_m + target_bottom_height_m_);
    if (catching) {
        r_pad = site_.catch_target->pin_point_itrf_m - site_up * (site_.catch_target->pin_station_m - mass.cg_station_m);
    }
    const double height = dot(r_fixed - r_pad, site_up);
    const double sink = -dot(v_fixed, site_up);
    const auto final_set = engine_set(body.vehicle(), parameters_.landing_groups, pressure);
    const double g = norm(g_fixed);
    // Gate where the centre engines take over: sink rate gate_speed, height such that they finish at
    // the design throttle with a constant deceleration.
    const double final_deceleration =
        std::max(parameters_.final_throttle * final_set.thrust_n / mass.mass_kg - g, 1.0);
    const double gate_height = parameters_.gate_speed_mps * parameters_.gate_speed_mps / (2.0 * final_deceleration);
    if (!final_engines_ && (height <= gate_height || sink <= parameters_.gate_speed_mps)) final_engines_ = true;

    // Vertical channel: the current segment (gate, then touchdown), constant-deceleration time-to-go.
    const double target_height = final_engines_ ? 0.0 : gate_height;
    const double target_sink = final_engines_ ? touchdown_speed_mps : parameters_.gate_speed_mps;
    const double t = std::max(2.0 * std::max(height - target_height, 0.0) / (std::max(sink, 0.0) + target_sink),
                              minimum_time_to_go_s);
    const double g_up = dot(g_fixed, site_up);
    const double zem_up = target_height - (height - sink * t + 0.5 * g_up * t * t);
    const double zev_up = -target_sink - (-sink + g_up * t);
    double a_vertical = 6.0 * zem_up / (t * t) - 2.0 * zev_up / t;
    if (final_engines_) {
        // Along v_ref(h) the reference deceleration is (dv_ref/dh) sink = sink / tau.
        const double v_ref = touchdown_speed_mps + std::max(height, 0.0) / approach_time_constant_s;
        a_vertical = -g_up + std::max(sink, 0.0) / approach_time_constant_s + approach_gain_per_s * (sink - v_ref);
    }
    // Horizontal channel.
    const Vec3 offset_side = (r_fixed - r_pad) - site_up * height;
    const Vec3 v_side = v_fixed + site_up * sink;
    const Vec3 g_side = g_fixed - site_up * g_up;
    // Inner engines: divert while closing in on the pad, with the constant deceleration that stops
    // exactly over it (front-loaded; energy-optimal ZEM/ZEV would leave the braking to the end, where
    // the tilt is limited); cross-track motion is critically damped. Centre engines: slow, gentle hold
    // only - they turn the booster with seconds of lag, an upright touchdown comes first.
    const double w = 1.0 / (final_engines_ ? hold_time_constant_s : divert_time_constant_s);
    Vec3 a_side = -(offset_side * (w * w) + v_side * (2.0 * w));
    const double distance = norm(offset_side);
    if (!final_engines_ && distance > divert_hold_radius_m) {
        const Vec3 towards = -offset_side / distance;
        const double closing = dot(v_side, towards);
        if (closing > 0.0) {
            const Vec3 v_cross = v_side - towards * closing;
            a_side = -towards * (closing * closing / (2.0 * distance)) - v_cross * (2.0 * w);
        }
    }
    a_side -= g_side;

    const auto& groups = final_engines_ ? parameters_.landing_groups : parameters_.landing_ignition_groups;
    // Tilt limit: the thrust stays within max_tilt of the vertical, tapering to a few degrees for the
    // touchdown (structural limit of the landing).
    const double a_up = std::max(a_vertical, 0.1 * g);
    const double tilt_max = final_engines_ ? parameters_.max_tilt_rad : parameters_.max_tilt_ignition_rad;
    const double tilt_limit =
        std::clamp(height / tilt_taper_height_m, 0.0, 1.0) * std::max(tilt_max - touchdown_tilt_rad, 0.0) +
        touchdown_tilt_rad;
    const double side_limit = a_up * std::tan(tilt_limit);
    if (norm(a_side) > side_limit) a_side = a_side * (side_limit / norm(a_side));
    const Vec3 a_cmd = site_up * a_up + a_side;
    const Vec3 axis_fixed = math::normalized(a_cmd);
    cmd.axis_gcrf = transform.rotation * axis_fixed;
    if (catching) {
        const Vec3 body_y = to_fixed * s.attitude.rotate(Vec3{0.0, 1.0, 0.0});
        cmd.roll_reference_gcrf = transform.rotation * catch_roll_reference(site_.catch_target->across_itrf, site_up,
                                                                            site_.catch_target->pin_azimuth_rad, body_y);
    }
    cmd.abort = ditch_;  // the tower keeps its arms open
    // Throttle for that thrust (nozzle pressure loss, engines still starting or running down).
    cmd.throttle = landing_throttle(body.vehicle(), groups, mass.mass_kg * norm(a_cmd), pressure);
    cmd.run_groups = groups;
    cmd.time_to_go_s = t;
    cmd.impact_miss_m = norm(offset_side);
    if (site_.pad && t <= body.vehicle().stages().front().config.landing_legs.deploy_time_s + leg_deploy_margin_s) {
        legs_deployed_ = true;
    }
    // Touchdown sensed (or ground contact): engines off.
    const bool catch_over = catching && (catch_contact_ || height < -catch_missed_height_m);
    if (body.resting() || body.impacted() || catch_over || leg_contact_ ||
        (!catching && final_engines_ && height < touchdown_cutoff_height_m)) {
        phase_ = ReturnPhase::Landed;
    }
}

}  // namespace starsim::gnc
