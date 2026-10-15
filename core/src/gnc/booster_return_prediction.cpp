#include "starsim/gnc/booster_return.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/lambert.hpp"

#include "booster_return_detail.hpp"

namespace starsim::gnc {

using math::Vec3;
using namespace booster_return_detail;

BoosterReturnGuidance::Impact BoosterReturnGuidance::predict_impact(const vehicle::FlightBody& body,
                                                                    const vehicle::FlightEnvironment& env,
                                                                    time::Epoch epoch, double ground_height_m,
                                                                    const Vec3* velocity_gcrf_mps, const PendingBurn* burn,
                                                                    double drag_scale) {
    const auto t0 = env.earth_orientation.fixed_to_gcrf(epoch);
    const Vec3 omega = vehicle::earth_rotation_vector(t0);
    const double w = norm(omega);
    const auto mass = body.vehicle().mass_properties();
    const double area = constants::pi * 0.25 * body.vehicle().diameter_m() * body.vehicle().diameter_m();
    const auto& drag = body.vehicle().aero().drag_curve(true);  // the returning booster falls engines first

    double m = mass.mass_kg;
    double burn_thrust_n = 0.0;  // during the pending burn (finite model)
    auto fixed_at = [&](double t, const Vec3& r_gcrf) { return transpose(rotation_z(w * t) * t0.rotation) * r_gcrf; };
    auto acceleration = [&](double t, const Vec3& r, const Vec3& v) {
        const auto geo = frames::fixed_to_geodetic(fixed_at(t, r));
        const auto air = env.atmosphere.at(geo.height_m);
        const Vec3 v_air = v - cross(omega, r);
        const double speed = norm(v_air);
        Vec3 a = env.gravity.acceleration_mps2(r, epoch + t);
        if (speed > 1e-3 && air.density_kg_m3 > 0.0) {
            const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
            const double force = drag_scale * 0.5 * air.density_kg_m3 * speed * speed * area * drag.cd(mach, false);
            a -= v_air * (force / (speed * m));
        }
        if (burn_thrust_n > 0.0 && speed > 1e-3) {
            Vec3 axis = -v_air / speed;
            if (burn->lean_rad != 0.0) {
                axis = math::normalized(axis + rotation_z(w * t) * t0.rotation * burn->lean_itrf * std::tan(burn->lean_rad));
            }
            a += axis * (burn_thrust_n / m);
        }
        return a;
    };

    Vec3 r = body.state().position_gcrf_m;
    Vec3 v = velocity_gcrf_mps ? *velocity_gcrf_mps : body.state().velocity_gcrf_mps;
    const double dt = 1.0;
    double previous_height = frames::fixed_to_geodetic(fixed_at(0.0, r)).height_m - mass.cg_station_m;
    Vec3 previous_r = r;
    Vec3 v_prev = v;
    bool burn_pending = burn && burn->delta_v_mps > 0.0;
    bool burning = false;
    double burned_mps = 0.0;
    for (double t = 0.0; t < 900.0; t += dt) {
        if (burning) {
            // Constant thrust over the step; the last step only for the rest of the delta-v.
            const double step_mps = burn->thrust_n / m * dt;
            const double fraction = std::min(1.0, (burn->delta_v_mps - burned_mps) / step_mps);
            burn_thrust_n = burn->thrust_n * fraction;
            burned_mps += step_mps * fraction;
            m -= burn->mass_flow_kgps * fraction * dt;
            if (burned_mps >= burn->delta_v_mps - 1e-9) burning = false;
        } else {
            burn_thrust_n = 0.0;
        }
        // RK4 on (r, v)
        const Vec3 a1 = acceleration(t, r, v);
        const Vec3 a2 = acceleration(t + dt / 2, r + v * (dt / 2), v + a1 * (dt / 2));
        const Vec3 a3 = acceleration(t + dt / 2, r + (v + a1 * (dt / 2)) * (dt / 2), v + a2 * (dt / 2));
        const Vec3 a4 = acceleration(t + dt, r + (v + a2 * (dt / 2)) * dt, v + a3 * dt);
        const Vec3 r_next = r + v * dt + (a1 + a2 + a3) * (dt * dt / 6.0);
        const Vec3 v_next = v + (a1 + a2 * 2.0 + a3 * 2.0 + a4) * (dt / 6.0);
        previous_r = r;
        v_prev = v;
        r = r_next;
        v = v_next;
        const double height = frames::fixed_to_geodetic(fixed_at(t + dt, r)).height_m - mass.cg_station_m;
        if (burn_pending && height < previous_height && height + mass.cg_station_m - ground_height_m < burn->altitude_m) {
            burn_pending = false;
            if (burn->thrust_n > 0.0) {
                burning = true;  // from the next step
            } else {
                const Vec3 v_air = v - cross(omega, r);
                v -= math::normalized(v_air) * std::min(burn->delta_v_mps, norm(v_air));
            }
        }
        if (height <= ground_height_m) {
            const double f = (previous_height - ground_height_m) / (previous_height - height);
            const Vec3 r_hit = previous_r + (r - previous_r) * f;
            const double t_hit = t + f * dt;
            const Vec3 v_hit = v_prev + (v - v_prev) * f;
            const Vec3 v_rel_hit = transpose(rotation_z(w * t_hit) * t0.rotation) * (v_hit - cross(omega, r_hit));
            return {fixed_at(t_hit, r_hit), v_rel_hit, t_hit, true};
        }
        previous_height = height;
    }
    return {};
}

BoosterReturnGuidance::BurnPrediction BoosterReturnGuidance::predict_landing_burn(
    const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch) const {
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const Vec3 omega = vehicle::earth_rotation_vector(transform);
    const auto& st = body.state();
    const Vec3 v_rel = st.velocity_gcrf_mps - cross(omega, st.position_gcrf_m);
    const auto geo = frames::fixed_to_geodetic(transpose(transform.rotation) * st.position_gcrf_m);
    const Vec3 up = transform.rotation * up_at(geo.latitude_rad, geo.longitude_rad);
    const auto mass = body.vehicle().mass_properties();
    const double area = constants::pi * 0.25 * body.vehicle().diameter_m() * body.vehicle().diameter_m();
    const auto& drag = body.vehicle().aero().drag_curve(true);
    const bool scvx = parameters_.scvx_landing && (site_.catch_target || site_.pad);
    // Segments: ignition set, (SCvx only) divert set, final set; each until its switch speed.
    struct Segment {
        const std::vector<std::string>* groups;
        double throttle;
        double end_speed_mps;
    };
    std::vector<Segment> segments;
    if (scvx) {
        segments = {{&parameters_.landing_ignition_groups, parameters_.scvx.ignition_throttle, parameters_.scvx.divert_speed_mps},
                    {&parameters_.landing_divert_groups, parameters_.scvx.divert_throttle, parameters_.scvx.switch_speed_mps},
                    {&parameters_.landing_groups, parameters_.scvx.final_throttle, 0.0}};
    } else {
        segments = {{&parameters_.landing_ignition_groups, parameters_.landing_throttle, parameters_.gate_speed_mps},
                    {&parameters_.landing_groups, parameters_.final_throttle, 0.0}};
    }
    const double g = norm(env.gravity.acceleration_mps2(st.position_gcrf_m, epoch));

    double speed = norm(v_rel);
    const double sin_gamma = speed > 0.0 ? std::clamp(-dot(v_rel, up) / speed, 0.0, 1.0) : 1.0;  // descent angle
    double h = geo.height_m - mass.cg_station_m - site_.height_m - target_bottom_height_m_;
    double m = mass.mass_kg;
    const double dt = 0.05;
    double t = 0.0;
    size_t segment = 0;
    for (; t < 200.0 && speed > 0.0; t += dt) {
        const auto air = env.atmosphere.at(site_.height_m + mass.cg_station_m + h);
        while (segment + 1 < segments.size() && speed <= segments[segment].end_speed_mps) ++segment;  // as the guidance
        const auto engines = engine_set(body.vehicle(), *segments[segment].groups, air.pressure_pa);
        const double throttle = segments[segment].throttle;
        // Only the ignition engines start up (the later sets are already running).
        const double spool = segment == 0 && engines.startup_s > 0.0 ? std::clamp(t / engines.startup_s, 0.0, 1.0) : 1.0;
        const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
        const double drag_n = drag_scale_ * 0.5 * air.density_kg_m3 * speed * speed * area * drag.cd(mach, spool > 0.0);
        const double thrust_n = std::max(throttle * engines.vacuum_thrust_n - (engines.vacuum_thrust_n - engines.thrust_n), 0.0);
        speed -= ((spool * thrust_n + drag_n) / m - g * sin_gamma) * dt;
        h -= speed * sin_gamma * dt;
        m -= spool * throttle * engines.mass_flow_kgps * dt;
    }
    return {h, t};
}

Vec3 BoosterReturnGuidance::landing_burn_lead_itrf(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                                   const Impact& impact) const {
    if (!site_.pad || !impact.valid) return {};
    const Vec3 up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 v = impact.velocity_itrf_mps;
    const Vec3 horizontal = v - up * dot(v, up);
    const double v_h = norm(horizontal);
    const double v_z = -dot(v, up);
    if (v_h < 1.0 || v_z <= 0.0) return {};

    const auto& vehicle = body.vehicle();
    const auto mass = vehicle.mass_properties();
    // Mass at the landing burn: the entry burn still ahead burns its propellant first.
    double m0 = mass.mass_kg;
    if (const auto burn = pending_entry_burn(vehicle); burn && burn->mass_flow_kgps > 0.0) {
        m0 *= std::exp(-burn->delta_v_mps / (burn->thrust_n / burn->mass_flow_kgps));
    }
    const double area = constants::pi * 0.25 * vehicle.diameter_m() * vehicle.diameter_m();
    const auto& drag = vehicle.aero().drag_curve(true);
    const double g = constants::standard_gravity_mps2;
    const double throttle = parameters_.scvx_landing ? parameters_.scvx.ignition_throttle : parameters_.landing_throttle;
    // Fall from height h0 (CG above the touchdown CG height) at the impact's velocity: horizontal travel and the
    // height where the motion stops (burn) or the ground is reached (ballistic).
    struct End {
        double x_m = 0.0;
        double z_m = 0.0;
    };
    auto fall = [&](double h0, bool burn) {
        double x = 0.0, z = h0, vx = v_h, vz = -v_z, m = m0;
        const double dt = 0.05;
        for (double t = 0.0; t < 200.0; t += dt) {
            const double speed = std::hypot(vx, vz);
            if (burn && (speed < 1.0 || vz >= 0.0)) break;
            if (!burn && z <= 0.0) break;
            const double height = site_.height_m + mass.cg_station_m + target_bottom_height_m_ + z;
            const auto air = env.atmosphere.at(height);
            const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
            double decel = drag_scale_ * 0.5 * air.density_kg_m3 * speed * speed * area * drag.cd(mach, burn) / m;
            if (burn) {
                const auto engines = engine_set(vehicle, parameters_.landing_ignition_groups, air.pressure_pa);
                const double spool = engines.startup_s > 0.0 ? std::clamp(t / engines.startup_s, 0.0, 1.0) : 1.0;
                const double thrust_n =
                    std::max(throttle * engines.vacuum_thrust_n - (engines.vacuum_thrust_n - engines.thrust_n), 0.0);
                decel += spool * thrust_n / m;
                m -= spool * throttle * engines.mass_flow_kgps * dt;
            }
            vx -= decel * vx / speed * dt;
            vz -= (decel * vz / speed + g) * dt;
            x += vx * dt;
            z += vz * dt;
        }
        return End{x, z};
    };
    // Ignition height: the burn stops on the ground (bisection on the start height).
    double low = 0.0, high = 20000.0;
    for (int i = 0; i < 30; ++i) {
        const double mid = 0.5 * (low + high);
        (fall(mid, true).z_m > 0.0 ? high : low) = mid;
    }
    const double ignition_m = 0.5 * (low + high);
    const double lead_m = fall(ignition_m, false).x_m - fall(ignition_m, true).x_m;
    return horizontal * (std::max(lead_m, 0.0) / v_h);
}

double BoosterReturnGuidance::meco_propellant_kg(const vehicle::Vehicle& stack, const math::StateVector& state_gcrf,
                                                const frames::BodyOrientation& earth, time::Epoch epoch,
                                                const LandingSite& site, const BoosterReturnParameters& parameters,
                                                double landing_propellant_kg) {
    const auto& booster = stack.stages().front();
    double run_down_kg = 0.0;
    for (const auto& e : booster.engines) {
        if (e.producing_thrust()) {
            run_down_kg += 0.5 * e.performance->config().shutdown_s * e.performance->mass_flow_kgps(e.throttle);
        }
    }
    const double dry_kg = booster.config.dry_mass_kg + booster.extra_mass_kg;
    // The RCS gas from the main tanks is spent on the way (counted at the end: the rocket equations see it as extra mass).
    landing_propellant_kg += parameters.rcs_propellant_kg;
    // Entry burn (rocket equation on its delta-v): the propellant to have left after the boostback.
    double after_boostback_kg = landing_propellant_kg;
    if (parameters.entry_altitude_m > 0.0 && parameters.entry_delta_v_mps > 0.0) {
        const auto entry = engine_set(stack, parameters.entry_groups, 0.0);
        const double entry_exhaust_mps = entry.mass_flow_kgps > 0.0 ? entry.vacuum_thrust_n / entry.mass_flow_kgps : 1.0;
        after_boostback_kg = (landing_propellant_kg + dry_kg) * std::exp(parameters.entry_delta_v_mps / entry_exhaust_mps) - dry_kg;
    }
    if (!parameters.boostback) return run_down_kg + after_boostback_kg;

    double flip_kg = 0.0;
    for (const auto& e : booster.engines) {
        const auto& groups = parameters.landing_groups;
        if (e.starved || std::find(groups.begin(), groups.end(), e.group_id) == groups.end()) continue;
        flip_kg += parameters.flip_duration_s * e.performance->mass_flow_kgps(e.performance->config().throttle_min);
    }

    const auto after_flip = orbit::propagate_two_body(state_gcrf, constants::earth_gm_m3ps2, parameters.flip_duration_s);
    const double time_left = std::max(parameters.flight_time_s - parameters.flip_duration_s, 30.0);
    const Vec3 site_fixed = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const Vec3 aim_gcrf =
        earth.fixed_to_gcrf(epoch + parameters.flip_duration_s + time_left).position(site_fixed);
    const Vec3 normal = math::normalized(cross(after_flip.position_m, aim_gcrf));
    const auto arc = orbit::solve_lambert(after_flip.position_m, aim_gcrf, time_left, constants::earth_gm_m3ps2, normal);
    const double delta_v =
        arc ? (1.0 + parameters.boostback_loss_fraction) * norm(arc->departure_velocity_mps - after_flip.velocity_mps)
            : 0.0;

    // Rocket equation on the booster alone after the flip: with dry mass m_d and propellant Q, the boostback leaves
    // Q - (m_d + Q)(1 - e) >= landing  <=>  Q >= (landing + m_d (1 - e)) / e,   e = exp(-delta_v / c).
    const auto engines = engine_set(stack, parameters.boostback_groups, 0.0);
    const double exhaust_mps = engines.mass_flow_kgps > 0.0 ? engines.vacuum_thrust_n / engines.mass_flow_kgps : 1.0;
    const double e = std::exp(-delta_v / exhaust_mps);
    return run_down_kg + flip_kg + (after_boostback_kg + dry_kg * (1.0 - e)) / e;
}

BoosterReturnGuidance::Impact BoosterReturnGuidance::predict(const vehicle::FlightBody& body,
                                                             const vehicle::FlightEnvironment& env, time::Epoch epoch,
                                                             const Vec3* velocity_gcrf_mps) const {
    const auto burn = pending_entry_burn(body.vehicle());
    return predict_impact(body, env, epoch, site_.height_m, velocity_gcrf_mps, burn ? &*burn : nullptr, drag_scale_);
}

void BoosterReturnGuidance::estimate_drag_scale(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                                time::Epoch epoch) {
    if (body.vehicle().any_engine_producing_thrust()) return;
    const auto& s = body.state();
    const auto t = env.earth_orientation.fixed_to_gcrf(epoch);
    const Vec3 v_air = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    const double speed = norm(v_air);
    const Vec3 a_aero = s.attitude.rotate(body.aero_acceleration_body_mps2());
    const double drag_mps2 = -dot(a_aero, v_air) / std::max(speed, 1e-3);
    if (speed < 1.0 || drag_mps2 < drag_scale_min_acceleration_mps2) return;
    const auto geo = frames::fixed_to_geodetic(transpose(t.rotation) * s.position_gcrf_m);
    const auto air = env.atmosphere.at(geo.height_m);
    const auto& vehicle = body.vehicle();
    const double area = constants::pi * 0.25 * vehicle.diameter_m() * vehicle.diameter_m();
    const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
    const double model_mps2 = 0.5 * air.density_kg_m3 * speed * speed * area * vehicle.aero().drag_curve(true).cd(mach, false) /
                              vehicle.mass_properties().mass_kg;
    if (model_mps2 <= 0.0) return;
    drag_scale_ += drag_scale_filter_gain * (std::clamp(drag_mps2 / model_mps2, 0.5, 2.0) - drag_scale_);
}

}  // namespace starsim::gnc
