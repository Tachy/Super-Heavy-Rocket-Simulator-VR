#include "starsim/gnc/booster_return.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/lambert.hpp"

namespace starsim::gnc {

using math::Mat3;
using math::Vec3;

namespace {

constexpr double gm_earth = 3.986004418e14;
constexpr double touchdown_speed_mps = 1.0;
constexpr double trim_deadband_m = 2.0;
constexpr double aero_steering_min_q_pa = 2000.0;  // below: RCS translation trim
constexpr double aero_steering_time_fraction = 0.5;
constexpr double aero_steering_max_alpha_rad = 8.0 * constants::deg_to_rad;  // [E] fin trim authority, heating
constexpr double minimum_time_to_go_s = 0.3;
// Centre-engine approach: sink rate follows touchdown speed + height / tau (an exponential approach, the V3 profile
// ending in a near hover; the same shape as the SCvx plans' approach limit), with a proportional sink-rate loop.
constexpr double approach_time_constant_s = 3.0;  // [E]
constexpr double approach_gain_per_s = 1.0;       // [E]
constexpr double touchdown_cutoff_height_m = 0.05; // engine cutoff when the legs / catch arms take the load
constexpr double divert_time_constant_s = 2.0;      // cross-track damping during the divert
constexpr double hold_time_constant_s = 4.0;        // position hold on the centre engines
constexpr double divert_hold_radius_m = 3.0;        // inside: position hold instead of braking profile
constexpr double catch_missed_height_m = 5.0;  // CG this far below the catch height: the arms were missed
constexpr double tilt_taper_height_m = 60.0;                           // below: tilt limit tapers off
constexpr double leg_deploy_margin_s = 5.0;      // [E] legs locked this long before the planned touchdown
constexpr double pad_cutoff_after_plan_s = 0.5;  // pad landing: engines off this long after the plan's end (spec)
// Entry burn steering: the axis leans from retrograde towards the drag-aware impact miss, fully at
// entry_steering_miss_m, by at most entry_steering_max_rad [E] (a small correction; the grid fins do the rest).
constexpr double entry_steering_miss_m = 2000.0;
constexpr double entry_steering_max_rad = 5.0 * constants::deg_to_rad;
constexpr double touchdown_tilt_rad = 4.0 * constants::deg_to_rad;     // tilt limit at touchdown

Vec3 up_at(double lat, double lon) { return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)}; }

// Rotation about z by angle a (Earth rotation over short horizons; precession/nutation negligible).
Mat3 rotation_z(double a) {
    Mat3 m = Mat3::identity();
    m(0, 0) = std::cos(a);
    m(0, 1) = -std::sin(a);
    m(1, 0) = std::sin(a);
    m(1, 1) = std::cos(a);
    return m;
}

struct EngineSet {
    double thrust_n = 0.0;        // full throttle at the given ambient pressure
    double vacuum_thrust_n = 0.0; // full throttle in vacuum (the pressure loss does not scale with the throttle)
    double mass_flow_kgps = 0.0;  // full throttle
    double startup_s = 0.0;
    double shutdown_s = 0.0;
};

EngineSet engine_set(const vehicle::Vehicle& v, const std::vector<std::string>& groups, double pressure_pa) {
    EngineSet set;
    for (const auto& stage : v.stages()) {
        for (const auto& e : stage.engines) {
            if (e.starved || std::find(groups.begin(), groups.end(), e.group_id) == groups.end()) continue;
            set.thrust_n += e.performance->thrust_n(1.0, pressure_pa);
            set.vacuum_thrust_n += e.performance->thrust_n(1.0, 0.0);
            set.mass_flow_kgps += e.performance->mass_flow_kgps(1.0);
            set.startup_s = std::max(set.startup_s, e.performance->config().startup_s);
            set.shutdown_s = std::max(set.shutdown_s, e.performance->config().shutdown_s);
        }
    }
    return set;
}

// Hinge station of the grid fins (vehicle frame), or a negative value without fins.
double grid_fin_station(const vehicle::Vehicle& v) {
    for (const auto& stage : v.stages()) {
        if (!stage.grid_fins.empty()) return stage.grid_fins.front().geometry.station_m;
    }
    return -1.0;
}

// Lateral acceleration (perpendicular to the flight path) of a booster falling engines first at the angle
// of attack alpha, with the grid fins holding that attitude (they cancel the aerodynamic moment about the
// CG). Positive: the force points away from the side the top leans to.
double lateral_acceleration(const vehicle::FlightBody& body, double speed, const environment::AtmosphereState& air,
                            double alpha, double fin_station) {
    const auto mass = body.vehicle().mass_properties();
    const Vec3 v_body{-speed * std::cos(alpha), speed * std::sin(alpha), 0.0};  // top leans towards +y
    const auto aero = body.vehicle().aero().compute(v_body, air.density_kg_m3, air.speed_of_sound_mps, false,
                                                     air.mean_free_path_m);
    const Vec3 moment = cross(Vec3{aero.normal_force_station_m - mass.cg_station_m, 0.0, 0.0}, aero.force_body_n);
    const double fin_force_y = -moment.z / (fin_station - mass.cg_station_m);
    const Vec3 total = aero.force_body_n + Vec3{0.0, fin_force_y, 0.0};
    const Vec3 u = v_body / speed;
    const Vec3 perpendicular = total - u * dot(total, u);
    return -perpendicular.y / mass.mass_kg;
}

}  // namespace

const char* to_string(ReturnPhase phase) {
    switch (phase) {
        case ReturnPhase::Flip: return "Wende";
        case ReturnPhase::Boostback: return "Boostback";
        case ReturnPhase::Coast: return "Abstieg";
        case ReturnPhase::EntryBurn: return "Wiedereintritt";
        case ReturnPhase::LandingBurn: return "Landebrennen";
        case ReturnPhase::Landed: return "Gelandet";
    }
    return "";
}

BoosterReturnParameters booster_return_parameters(const std::string& stage_id, const config::ReturnProfile& profile,
                                                  BoosterReturnParameters p) {
    p.boostback = profile.boostback;
    const auto ids = [&](const std::vector<std::string>& local) {
        std::vector<std::string> out;
        for (const auto& id : local) out.push_back(stage_id + "/" + id);
        return out;
    };
    if (!profile.boostback_engine_groups.empty()) p.boostback_groups = ids(profile.boostback_engine_groups);
    if (!profile.entry_engine_groups.empty()) p.entry_groups = ids(profile.entry_engine_groups);
    if (profile.landing_engine_phases.size() == 3) {
        p.landing_ignition_groups = ids(profile.landing_engine_phases[0]);
        p.landing_divert_groups = ids(profile.landing_engine_phases[1]);
        p.landing_groups = ids(profile.landing_engine_phases[2]);
    }
    if (profile.landing_phase_throttles.size() == 3) {
        p.scvx.ignition_throttle = profile.landing_phase_throttles[0];
        p.scvx.divert_throttle = profile.landing_phase_throttles[1];
        p.scvx.final_throttle = profile.landing_phase_throttles[2];
    }
    if (profile.landing_switch_sink_mps.size() == 2) {
        auto& switches = p.scvx.settings.limits.switches;
        for (size_t i = 0; i < 2; ++i) {
            const auto [low, high] = profile.landing_switch_sink_mps[i];
            switches[i] = {0.0, 0.0, low, high};
        }
        // The burn prediction switches in the middle of each window.
        p.scvx.divert_speed_mps = 0.5 * (switches[0].min_sink_mps + switches[0].max_sink_mps);
        p.scvx.switch_speed_mps = 0.5 * (switches[1].min_sink_mps + switches[1].max_sink_mps);
    }
    if (profile.return_flight_time_s > 0.0) p.flight_time_s = profile.return_flight_time_s;
    if (profile.boostback_loss_fraction >= 0.0) p.boostback_loss_fraction = profile.boostback_loss_fraction;
    if (profile.entry_burn_altitude_m > 0.0) {
        p.entry_altitude_m = profile.entry_burn_altitude_m;
        p.entry_delta_v_mps = profile.entry_burn_delta_v_mps;
        if (profile.entry_burn_throttle > 0.0) p.entry_throttle = profile.entry_burn_throttle;
    }
    return p;
}

BoosterReturnParameters booster_return_parameters(const config::StageConfig& stage, BoosterReturnParameters base) {
    return booster_return_parameters(stage.id, stage.return_profile, std::move(base));
}

BoosterReturnGuidance::BoosterReturnGuidance(LandingSite site, BoosterReturnParameters parameters)
    : site_(std::move(site)), parameters_(std::move(parameters)) {
    if (site_.catch_target && site_.sea && !parameters_.scvx_landing) {
        // The explicit laws cannot hit the arms (Super Heavy has no legs): the return aims at the sea.
        ditch_ = true;
        const Vec3 up = up_at(site_.latitude_rad, site_.longitude_rad);
        const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
        const Vec3 north = cross(up, east);
        const Vec3 seaward = north * std::cos(site_.sea->direction_rad) + east * std::sin(site_.sea->direction_rad);
        const auto sea = frames::fixed_to_geodetic(
            frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m}) +
            seaward * (site_.sea->shoreline_distance_m + parameters_.scvx.snapshot.sea_margin_m));
        site_.latitude_rad = sea.latitude_rad;
        site_.longitude_rad = sea.longitude_rad;
    } else if (site_.catch_target) {
        const auto pins = frames::fixed_to_geodetic(site_.catch_target->pin_point_itrf_m);
        target_bottom_height_m_ = pins.height_m - site_.height_m - site_.catch_target->pin_station_m;
    } else if (site_.pad) {
        target_bottom_height_m_ = site_.pad->contact_bottom_height_m;  // on the legs
    }
    if (!parameters_.boostback) phase_ = ReturnPhase::Coast;  // drone ship: coast from separation
}

BoosterReturnGuidance::Impact BoosterReturnGuidance::predict_impact(const vehicle::FlightBody& body,
                                                                    const vehicle::FlightEnvironment& env,
                                                                    time::Epoch epoch, double ground_height_m,
                                                                    const Vec3* velocity_gcrf_mps, const PendingBurn* burn) {
    const auto t0 = env.earth_orientation.fixed_to_gcrf(epoch);
    const Vec3 omega = vehicle::earth_rotation_vector(t0);
    const double w = norm(omega);
    const auto mass = body.vehicle().mass_properties();
    const double area = constants::pi * 0.25 * body.vehicle().diameter_m() * body.vehicle().diameter_m();
    const auto& drag = body.vehicle().aero().drag_curve(true);  // the returning booster falls engines first

    auto fixed_at = [&](double t, const Vec3& r_gcrf) { return transpose(rotation_z(w * t) * t0.rotation) * r_gcrf; };
    auto acceleration = [&](double t, const Vec3& r, const Vec3& v) {
        const auto geo = frames::fixed_to_geodetic(fixed_at(t, r));
        const auto air = env.atmosphere.at(geo.height_m);
        const Vec3 v_air = v - cross(omega, r);
        const double speed = norm(v_air);
        Vec3 a = env.gravity.acceleration_mps2(r, epoch + t);
        if (speed > 1e-3 && air.density_kg_m3 > 0.0) {
            const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
            const double force = 0.5 * air.density_kg_m3 * speed * speed * area * drag.cd(mach, false);
            a -= v_air * (force / (speed * mass.mass_kg));
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
    for (double t = 0.0; t < 900.0; t += dt) {
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
            const Vec3 v_air = v - cross(omega, r);
            v -= math::normalized(v_air) * std::min(burn->delta_v_mps, norm(v_air));
            burn_pending = false;
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
        const double drag_n = 0.5 * air.density_kg_m3 * speed * speed * area * drag.cd(mach, spool > 0.0);
        const double thrust_n = std::max(throttle * engines.vacuum_thrust_n - (engines.vacuum_thrust_n - engines.thrust_n), 0.0);
        speed -= ((spool * thrust_n + drag_n) / m - g * sin_gamma) * dt;
        h -= speed * sin_gamma * dt;
        m -= spool * throttle * engines.mass_flow_kgps * dt;
    }
    return {h, t};
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

    const auto after_flip = orbit::propagate_two_body(state_gcrf, gm_earth, parameters.flip_duration_s);
    const double time_left = std::max(parameters.flight_time_s - parameters.flip_duration_s, 30.0);
    const Vec3 site_fixed = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const Vec3 aim_gcrf =
        earth.fixed_to_gcrf(epoch + parameters.flip_duration_s + time_left).position(site_fixed);
    const Vec3 normal = math::normalized(cross(after_flip.position_m, aim_gcrf));
    const auto arc = orbit::solve_lambert(after_flip.position_m, aim_gcrf, time_left, gm_earth, normal);
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

ReturnCommand BoosterReturnGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                            time::Epoch epoch) {
    const auto& s = body.state();
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const Mat3 to_fixed = transpose(transform.rotation);
    const Vec3 omega = vehicle::earth_rotation_vector(transform);
    const Vec3 r_fixed = to_fixed * s.position_gcrf_m;
    const Vec3 v_rel_gcrf = s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m);
    const Vec3 v_fixed = to_fixed * v_rel_gcrf;
    const auto mass = body.vehicle().mass_properties();
    const Vec3 site_up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 site_fixed = frames::geodetic_to_fixed({site_.latitude_rad, site_.longitude_rad, site_.height_m});
    const auto geo = frames::fixed_to_geodetic(r_fixed);
    const double pressure = env.atmosphere.at(geo.height_m).pressure_pa;
    const Vec3 body_axis = s.attitude.rotate(Vec3{1.0, 0.0, 0.0});
    auto horizontal_miss = [&](const Impact& impact) {
        const Vec3 miss = site_fixed - impact.position_itrf_m;
        return miss - site_up * dot(miss, site_up);
    };

    if (!started_) {
        started_ = true;
        boostback_start_ = epoch;
        last_update_ = epoch;
    }
    const double dt = epoch - last_update_;
    last_update_ = epoch;
    ++tick_counter_;
    ReturnCommand cmd;

    if (phase_ == ReturnPhase::Flip || phase_ == ReturnPhase::Boostback) {
        const double time_left = std::max(boostback_start_ + parameters_.flight_time_s - epoch, 30.0);
        const auto at_landing = env.earth_orientation.fixed_to_gcrf(epoch + time_left);
        const Vec3 aim_fixed = site_fixed + aim_offset_itrf_ + site_up * mass.cg_station_m;
        const Vec3 aim_gcrf = at_landing.position(aim_fixed);
        const Vec3 normal = math::normalized(cross(s.position_gcrf_m, aim_gcrf));
        const auto arc = orbit::solve_lambert(s.position_gcrf_m, aim_gcrf, time_left, gm_earth, normal);
        if (arc && tick_counter_ % 20 == 1) {
            // Shooting on the aim point: where would the Lambert arc really come down (drag, rotation)?
            const auto impact = predict(body, env, epoch, &arc->departure_velocity_mps);
            if (impact.valid) {
                const Vec3 miss = horizontal_miss(impact);
                aim_offset_itrf_ += miss;
                impact_miss_m_ = norm(miss);
            }
        }
        const Vec3 v_go = arc ? arc->departure_velocity_mps - s.velocity_gcrf_mps : -v_rel_gcrf;
        cmd.axis_gcrf = math::normalized(v_go);
        cmd.impact_miss_m = impact_miss_m_;

        const double pointing = std::acos(std::clamp(dot(body_axis, cmd.axis_gcrf), -1.0, 1.0));
        if (phase_ == ReturnPhase::Flip) {
            // Turn with the centre engines idling (gimbal authority), then go to full boostback thrust.
            cmd.run_groups = parameters_.landing_groups;
            cmd.throttle = 0.0;  // limited to the minimum throttle by the engines
            if (pointing < parameters_.flip_tolerance_rad) phase_ = ReturnPhase::Boostback;
        } else {
            cmd.run_groups = parameters_.boostback_groups;
            cmd.throttle = 1.0;
            // Cutoff when the remaining delta-v equals the shutdown tail-off impulse.
            const auto engines = engine_set(body.vehicle(), parameters_.boostback_groups, pressure);
            const double tail_off_mps = 0.5 * engines.thrust_n / mass.mass_kg * engines.shutdown_s;
            if (norm(v_go) < std::max(tail_off_mps, 0.5)) phase_ = ReturnPhase::Coast;
        }
        cmd.phase = phase_;
        if (phase_ == ReturnPhase::Coast) cmd.run_groups.clear();
        return cmd;
    }

    if (phase_ == ReturnPhase::Coast) {
        // Steering: in the atmosphere with the grid fins (they hold an angle of attack, the body's cross-flow
        // force moves the impact point), above it by RCS translation.
        const auto air_now = env.atmosphere.at(geo.height_m);
        const double speed = norm(v_fixed);
        const double dynamic_pressure = 0.5 * air_now.density_kg_m3 * speed * speed;
        const double fin_station = grid_fin_station(body.vehicle());
        const bool aero_steering = fin_station > 0.0 && dynamic_pressure > aero_steering_min_q_pa;
        if (tick_counter_ % (aero_steering ? 40 : 200) == 0) {
            const auto impact = predict(body, env, epoch);
            if (impact.valid) {
                const Vec3 miss = horizontal_miss(impact);
                impact_miss_m_ = norm(miss);
                trim_itrf_ = {};
                steer_itrf_ = {};
                if (aero_steering) {
                    // Constant lateral acceleration that removes the miss within a fraction of the time to
                    // impact (margin for the prediction model, which ignores the steering force itself); the
                    // angle of attack comes from the aerodynamic model by bisection. Its sign too: for the blunt tail-first
                    // booster the tilted axial drag outweighs the cross-flow force, so the force points to
                    // the side the top leans to.
                    const double t_correct = aero_steering_time_fraction * impact.time_s;
                    const Vec3 required = miss * (2.0 / std::max(t_correct * t_correct, 1.0));
                    auto lateral = [&](double alpha) {
                        return lateral_acceleration(body, speed, air_now, alpha, fin_station);
                    };
                    const double at_max = lateral(aero_steering_max_alpha_rad);
                    if (at_max != 0.0 && impact_miss_m_ > trim_deadband_m) {
                        double low = 0.0, high = aero_steering_max_alpha_rad;
                        if (std::abs(at_max) > norm(required)) {
                            for (int i = 0; i < 20; ++i) {
                                const double mid = 0.5 * (low + high);
                                (std::abs(lateral(mid)) < norm(required) ? low : high) = mid;
                            }
                        }
                        // Lean towards -d if the force points away from the lean, towards d otherwise.
                        const double lean = at_max > 0.0 ? -1.0 : 1.0;
                        steer_itrf_ = math::normalized(required) * (-lean * std::tan(high));
                    }
                } else if (impact_miss_m_ > trim_deadband_m) {
                    // Proportional trim: full RCS force from 100 m miss.
                    trim_itrf_ = miss * (std::min(impact_miss_m_ / 100.0, 1.0) / impact_miss_m_);
                }
            }
        }
        // Engines first; for a lateral push towards d the top leans towards -d.
        cmd.axis_gcrf = transform.rotation * math::normalized(-math::normalized(v_fixed) - steer_itrf_);
        cmd.impact_miss_m = impact_miss_m_;
        cmd.rcs_force_gcrf = transform.rotation * trim_itrf_;
        if (site_.catch_target) {
            // Roll the pins towards the arms already on the way down (grid fins, above the air RCS): recorded
            // boosters light the landing burn already rolled for the catch.
            const Vec3 body_y = to_fixed * s.attitude.rotate(Vec3{0.0, 1.0, 0.0});
            cmd.roll_reference_gcrf = transform.rotation * catch_roll_reference(site_.catch_target->across_itrf, site_up,
                                                                                site_.catch_target->pin_azimuth_rad, body_y);
        }
        const double height = geo.height_m - mass.cg_station_m - site_.height_m;
        const bool descending = dot(v_fixed, site_up) < 0.0;
        if (descending && !entry_done_ && parameters_.entry_altitude_m > 0.0 &&
            geo.height_m - site_.height_m < parameters_.entry_altitude_m) {
            phase_ = ReturnPhase::EntryBurn;
            entry_delta_v_mps_ = 0.0;
        }
        if (phase_ == ReturnPhase::Coast && descending && (height < 30e3 || tick_counter_ % 20 == 0)) {
            const auto burn = predict_landing_burn(body, env, epoch);
            if (burn.stop_height_m <= 0.0) phase_ = ReturnPhase::LandingBurn;
        }
        cmd.phase = phase_;
        if (phase_ == ReturnPhase::Coast) return cmd;
    }

    if (phase_ == ReturnPhase::EntryBurn) {
        if (body.vehicle().any_engine_producing_thrust()) {
            double thrust_n = 0.0;
            for (const auto& e : body.vehicle().stages().front().engines) {
                if (e.producing_thrust()) thrust_n += e.spool * e.performance->thrust_n(e.throttle, pressure);
            }
            entry_delta_v_mps_ += thrust_n / mass.mass_kg * dt;
        }
        if (entry_delta_v_mps_ >= parameters_.entry_delta_v_mps) {
            entry_done_ = true;
            phase_ = ReturnPhase::Coast;
            cmd.phase = phase_;
            cmd.axis_gcrf = transform.rotation * (-math::normalized(v_fixed));
            return cmd;
        }
        entry_burn(body, env, epoch, cmd);
        cmd.phase = phase_;
        return cmd;
    }

    if (phase_ == ReturnPhase::LandingBurn) {
        if (!landing_started_) start_landing(body, env, epoch);
        if (scvx_) {
            const auto c = scvx_->update(body, env, epoch);
            cmd.run_groups = c.run_groups;
            cmd.throttle = c.throttle;
            cmd.axis_gcrf = c.axis_gcrf;
            cmd.roll_reference_gcrf = c.roll_reference_gcrf;
            cmd.feedforward = c.feedforward;
            cmd.torque_body_nm = c.torque_body_nm;
            cmd.scvx = true;
            cmd.abort = c.abort;
            cmd.time_to_go_s = c.time_to_go_s;
            cmd.impact_miss_m = c.offset_m;
            bool over = false;
            if (site_.catch_target) {
                const Vec3 r_catch = site_.catch_target->pin_point_itrf_m -
                                     site_up * (site_.catch_target->pin_station_m - mass.cg_station_m);
                const double height = dot(r_fixed - r_catch, site_up);
                over = !c.abort && (catch_contact_ || height < -catch_missed_height_m);
            } else {
                // Pad: the legs go out before the touchdown; the engines stop on leg contact, or shortly after the plan's
                // end (a plan ending a little high: the stage drops onto its legs).
                const double deploy_s = body.vehicle().stages().front().config.landing_legs.deploy_time_s;
                if (c.time_to_go_s <= deploy_s + leg_deploy_margin_s) legs_deployed_ = true;
                if (c.time_to_go_s > 0.0 || !plan_end_) plan_end_ = epoch + c.time_to_go_s;
                over = leg_contact_ || epoch - *plan_end_ > pad_cutoff_after_plan_s;
            }
            if (body.resting() || body.impacted() || over) phase_ = ReturnPhase::Landed;
        } else {
            explicit_landing(body, env, epoch, cmd);
        }
    }
    if (phase_ == ReturnPhase::Landed) {
        cmd.run_groups.clear();
        cmd.axis_gcrf = transform.rotation * site_up;
    }
    cmd.deploy_legs = legs_deployed_;
    cmd.phase = phase_;
    return cmd;
}

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
    if (tick_counter_ % 40 == 0) {
        const auto impact = predict(body, env, epoch);
        if (impact.valid) {
            const Vec3 miss = site_fixed - impact.position_itrf_m;
            steer_itrf_ = miss - site_up * dot(miss, site_up);
            impact_miss_m_ = norm(steer_itrf_);
        }
    }
    Vec3 axis = -math::normalized(v_fixed);
    if (impact_miss_m_ > trim_deadband_m) {
        const double lean = entry_steering_max_rad * std::min(impact_miss_m_ / entry_steering_miss_m, 1.0);
        axis = math::normalized(axis + math::normalized(steer_itrf_) * std::tan(lean));
    }
    cmd.axis_gcrf = transform.rotation * axis;
    cmd.run_groups = parameters_.entry_groups;
    cmd.throttle = parameters_.entry_throttle;
    cmd.impact_miss_m = impact_miss_m_;
}

std::optional<BoosterReturnGuidance::PendingBurn> BoosterReturnGuidance::pending_entry_burn() const {
    if (parameters_.entry_altitude_m <= 0.0 || entry_done_) return std::nullopt;
    // While burning: the rest from here (the altitude is already crossed: the impulse applies at once).
    const double altitude = phase_ == ReturnPhase::EntryBurn ? 1e9 : parameters_.entry_altitude_m;
    return PendingBurn{altitude, std::max(parameters_.entry_delta_v_mps - entry_delta_v_mps_, 0.0)};
}

BoosterReturnGuidance::Impact BoosterReturnGuidance::predict(const vehicle::FlightBody& body,
                                                             const vehicle::FlightEnvironment& env, time::Epoch epoch,
                                                             const Vec3* velocity_gcrf_mps) const {
    const auto burn = pending_entry_burn();
    return predict_impact(body, env, epoch, site_.height_m, velocity_gcrf_mps, burn ? &*burn : nullptr);
}

void BoosterReturnGuidance::set_realtime(bool realtime) {
    parameters_.scvx.realtime = realtime;
    if (scvx_) scvx_->set_realtime(realtime);
}

void BoosterReturnGuidance::start_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                          time::Epoch epoch) {
    landing_started_ = true;
    if (!parameters_.scvx_landing || !(site_.catch_target || site_.pad)) return;
    auto p = parameters_.scvx;
    p.snapshot.phase_groups = {parameters_.landing_ignition_groups, parameters_.landing_divert_groups,
                               parameters_.landing_groups};
    // A pad landing ends on one engine whose minimum thrust exceeds the weight: no exponential approach (infeasible at
    // the end, see DescentLimits::approach_time_constant_s).
    if (site_.pad) p.settings.limits.approach_time_constant_s = 0.0;
    scvx_ = std::make_unique<ScvxLandingGuidance>(site_, p);
    if (!scvx_->start(body, env, epoch)) scvx_.reset();  // fallback: the explicit laws
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
