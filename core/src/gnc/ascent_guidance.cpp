#include "starsim/gnc/ascent_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "starsim/frames/geodetic.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/j2_insertion.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

// [E] re-solve the J2-aware insertion when the predicted cutoff point moves by more than this.
constexpr double j2_resolve_angle_rad = 2.0 * constants::deg_to_rad;
// [E] max-Q limiter: below this air speed q is negligible (~10 Pa at sea level) and the law divides by the speed
constexpr double min_limited_air_speed_mps = 5.0;
// [E] max-Q limiter: half step of the central difference of the air density with height (US76 layers are smooth on it)
constexpr double density_gradient_step_m = 50.0;

constexpr double earth_rotation_rate_radps = 7.2921150e-5;
constexpr double equatorial_radius_m = 6378137.0;

struct LocalFrame {
    Vec3 up, east, north;
    double latitude_rad = 0.0;
};

LocalFrame local_frame(const Vec3& position_gcrf, const frames::BodyOrientation& earth, time::Epoch epoch) {
    const auto t = earth.fixed_to_gcrf(epoch);
    const auto g = frames::fixed_to_geodetic(transpose(t.rotation) * position_gcrf);
    const Vec3 up{std::cos(g.latitude_rad) * std::cos(g.longitude_rad),
                  std::cos(g.latitude_rad) * std::sin(g.longitude_rad), std::sin(g.latitude_rad)};
    const Vec3 east{-std::sin(g.longitude_rad), std::cos(g.longitude_rad), 0.0};
    return {t.rotation * up, t.rotation * east, t.rotation * cross(up, east), g.latitude_rad};
}

// Thrust and mass flow of the engines that are starting or running (at full spool).
struct Performance {
    double thrust_n = 0.0;
    double mass_flow_kgps = 0.0;
};

Performance current_performance(const vehicle::Vehicle& v, double ambient_pressure_pa) {
    Performance p;
    for (const auto& stage : v.stages()) {
        for (const auto& e : stage.engines) {
            if (e.mode != vehicle::EngineMode::Starting && e.mode != vehicle::EngineMode::Running) continue;
            p.thrust_n += e.performance->thrust_n(e.throttle, ambient_pressure_pa);
            p.mass_flow_kgps += e.performance->mass_flow_kgps(e.throttle);
        }
    }
    return p;
}

double shutdown_time_s(const vehicle::Vehicle& v) {
    double t = 0.0;
    for (const auto& stage : v.stages()) {
        for (const auto& e : stage.engines) t = std::max(t, e.performance->config().shutdown_s);
    }
    return t;
}

}  // namespace

const char* to_string(AscentPhase phase) {
    switch (phase) {
        case AscentPhase::Vertical: return "Senkrechter Aufstieg";
        case AscentPhase::PitchKick: return "Nickmanöver";
        case AscentPhase::GravityTurn: return "Gravity Turn";
        case AscentPhase::ClosedLoop: return "Geschlossene Führung (PEG)";
        case AscentPhase::Cutoff: return "Brennschluss";
    }
    return "";
}

double launch_azimuth_rad(double latitude_rad, double inclination_rad, double orbital_speed_mps, bool northerly) {
    const double s = std::clamp(std::cos(inclination_rad) / std::cos(latitude_rad), -1.0, 1.0);
    double inertial = std::asin(s);
    if (!northerly) inertial = constants::pi - inertial;
    const double surface_speed = earth_rotation_rate_radps * equatorial_radius_m * std::cos(latitude_rad);
    return std::atan2(orbital_speed_mps * std::sin(inertial) - surface_speed, orbital_speed_mps * std::cos(inertial));
}

math::Vec3 ascent_plane_normal(const Vec3& position_gcrf_m, const frames::BodyOrientation& earth, time::Epoch epoch,
                               double inclination_rad, bool northerly) {
    const auto frame = local_frame(position_gcrf_m, earth, epoch);
    const double s = std::clamp(std::cos(inclination_rad) / std::cos(frame.latitude_rad), -1.0, 1.0);
    const double inertial = northerly ? std::asin(s) : constants::pi - std::asin(s);
    const Vec3 heading = std::sin(inertial) * frame.east + std::cos(inertial) * frame.north;
    return math::normalized(cross(frame.up, heading));
}

AscentGuidance::AscentGuidance(AscentTarget target, PitchProgram program, double gm_earth_m3ps2)
    : target_(target), program_(program), gm_(gm_earth_m3ps2), peg_(gm_earth_m3ps2) {
    const double rp = equatorial_radius_m + target_.periapsis_altitude_m;
    const double ra = equatorial_radius_m + target_.apoapsis_altitude_m;
    target_radius_m_ = rp;
    target_speed_mps_ = std::sqrt(gm_ * (2.0 / rp - 2.0 / (rp + ra)));  // vis-viva at periapsis
}

void AscentGuidance::start(const vehicle::FlightBody& body, const frames::BodyOrientation& earth, time::Epoch epoch) {
    cutoff_epoch_.reset();
    const auto frame = local_frame(body.state().position_gcrf_m, earth, epoch);
    const double latitude = frame.latitude_rad;
    azimuth_rad_ = gnc::launch_azimuth_rad(latitude, target_.inclination_rad, target_speed_mps_, target_.northerly);
    plane_normal_ = ascent_plane_normal(body.state().position_gcrf_m, earth, epoch, target_.inclination_rad,
                                        target_.northerly);
    liftoff_ = epoch;
    kick_start_s_ = -1.0;
    peg_.reset();
    insertion_active_ = false;
}

GuidanceCommand AscentGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                       time::Epoch epoch, bool closed_loop) {
    if (closed_loop) return peg(body, env, epoch);
    GuidanceCommand cmd = open_loop(body, env, epoch);
    cmd.throttle = booster_throttle(body, env, epoch);
    return cmd;
}

double max_q_thrust_acceleration_mps2(double limit_pa, double density_kg_m3, double density_rate_kgpm3ps,
                                      double air_speed_mps, double other_acceleration_mps2) {
    constexpr double infinity = std::numeric_limits<double>::infinity();
    if (!(limit_pa > 0.0) || !(density_kg_m3 > 0.0) || !(air_speed_mps > min_limited_air_speed_mps)) return infinity;
    const double v = air_speed_mps;
    const double q = 0.5 * density_kg_m3 * v * v;
    const double q_rate_wanted = (limit_pa - q) / max_q_time_constant_s;
    const double speed_rate_wanted = (q_rate_wanted - 0.5 * v * v * density_rate_kgpm3ps) / (density_kg_m3 * v);
    return speed_rate_wanted - other_acceleration_mps2;
}

double booster_throttle_for_thrust(const vehicle::Vehicle& vehicle, double thrust_n, double ambient_pressure_pa) {
    double low = 0.0, high = 1.0;
    bool any = false;
    for (const auto& e : vehicle.stages().front().engines) {
        if (!e.producing_thrust()) continue;
        const auto& cfg = e.performance->config();
        low = any ? std::max(low, cfg.throttle_min) : cfg.throttle_min;
        high = any ? std::min(high, cfg.throttle_max) : cfg.throttle_max;
        any = true;
    }
    if (!any || !(high > low)) return high;
    // thrust is linear in the throttle (EnginePerformance::thrust_n): interpolate between the range's ends
    double thrust_low_n = 0.0, thrust_high_n = 0.0;
    for (const auto& e : vehicle.stages().front().engines) {
        if (!e.producing_thrust()) continue;
        thrust_low_n += e.spool * e.performance->thrust_n(low, ambient_pressure_pa);
        thrust_high_n += e.spool * e.performance->thrust_n(high, ambient_pressure_pa);
    }
    if (!(thrust_high_n > thrust_low_n)) return high;
    return std::clamp(low + (high - low) * (thrust_n - thrust_low_n) / (thrust_high_n - thrust_low_n), low, high);
}

double ascent_throttle_cap(const PitchProgram& program) {
    return program.max_ascent_throttle > 0.0 ? std::min(program.max_ascent_throttle, 1.0) : 1.0;
}

double dynamic_pressure_limit_pa(const PitchProgram& program, double mach) {
    const auto& points = program.max_dynamic_pressure_by_mach;
    if (points.empty()) return 0.0;
    if (!(mach > points.front().first)) return points.front().second;
    for (size_t i = 1; i < points.size(); ++i) {
        if (mach > points[i].first) continue;
        const auto& [m0, q0] = points[i - 1];
        const auto& [m1, q1] = points[i];
        return q0 + (q1 - q0) * (mach - m0) / (m1 - m0);
    }
    return points.back().second;
}

double AscentGuidance::booster_throttle(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                        time::Epoch epoch) const {
    return std::min(ascent_throttle_cap(program_), max_q_throttle(body, env, epoch));
}

double AscentGuidance::max_q_throttle(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                      time::Epoch epoch) const {
    if (program_.max_dynamic_pressure_by_mach.empty()) return 1.0;
    const auto& v = body.vehicle();
    const auto& s = body.state();
    const auto& air = body.air_data();
    const Vec3& air_velocity_body = air.air_velocity_body_mps;
    const double speed = norm(air_velocity_body);
    const double density = air.air.density_kg_m3;
    if (!(speed > min_limited_air_speed_mps) || !(density > 0.0) || !(air.air.speed_of_sound_mps > 0.0)) return 1.0;
    const double limit_pa = dynamic_pressure_limit_pa(program_, speed / air.air.speed_of_sound_mps);
    const Vec3 along_gcrf = s.attitude.rotate(air_velocity_body / speed);
    // the air's density change along the path: d rho / dh (US76, scaled as the truth air) times the climb rate
    const auto frame = local_frame(s.position_gcrf_m, env.earth_orientation, epoch);
    const double height_m =
        frames::fixed_to_geodetic(transpose(env.earth_orientation.fixed_to_gcrf(epoch).rotation) * s.position_gcrf_m).height_m;
    const double density_gradient = env.air_density_factor *
                                    (env.atmosphere.at(height_m + density_gradient_step_m).density_kg_m3 -
                                     env.atmosphere.at(height_m - density_gradient_step_m).density_kg_m3) /
                                    (2.0 * density_gradient_step_m);
    const double density_rate = density_gradient * speed * dot(along_gcrf, frame.up);
    // along the path besides the thrust: the accelerometer (last tick) less the engines' thrust now, gravity, and the
    // centrifugal term of the Earth-fixed air (the Coriolis term is normal to the air velocity)
    const double ambient_pa = env.atmosphere.at(height_m).pressure_pa;
    double thrust_n = 0.0;
    for (const auto& e : v.stages().front().engines) {
        if (e.producing_thrust()) thrust_n += e.spool * e.performance->thrust_n(e.throttle, ambient_pa);
    }
    const double mass_kg = v.mass_properties().mass_kg;
    const double thrust_along = air_velocity_body.x / speed;  // the thrust axis (body x) against the air velocity
    const Vec3 omega = vehicle::earth_rotation_vector(env.earth_orientation.fixed_to_gcrf(epoch));
    const double other_mps2 = dot(body.specific_force_body_mps2(), air_velocity_body / speed) -
                              thrust_n * thrust_along / mass_kg +
                              dot(env.gravity.acceleration_mps2(s.position_gcrf_m, epoch), along_gcrf) -
                              dot(cross(omega, cross(omega, s.position_gcrf_m)), along_gcrf);
    const double acceleration =
        max_q_thrust_acceleration_mps2(limit_pa, density, density_rate, speed, other_mps2);
    if (!std::isfinite(acceleration) || !(thrust_along > 0.0)) return 1.0;
    return booster_throttle_for_thrust(v, mass_kg * acceleration / thrust_along, ambient_pa);
}

GuidanceCommand AscentGuidance::open_loop(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                          time::Epoch epoch) {
    const auto& s = body.state();
    const auto frame = local_frame(s.position_gcrf_m, env.earth_orientation, epoch);
    const Vec3 heading = std::sin(azimuth_rad_) * frame.east + std::cos(azimuth_rad_) * frame.north;
    const auto earth = env.earth_orientation.fixed_to_gcrf(epoch);
    const Vec3 relative = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(earth), s.position_gcrf_m);
    const double t = epoch - liftoff_;

    GuidanceCommand cmd;
    if (kick_start_s_ < 0.0) {
        if (dot(relative, frame.up) < program_.vertical_rise_speed_mps) {
            cmd.phase = AscentPhase::Vertical;
            cmd.thrust_direction_gcrf = frame.up;
            return cmd;
        }
        kick_start_s_ = t;
    }
    const double progress = (t - kick_start_s_) / program_.kick_duration_s;
    if (progress < 1.0) {
        const double tilt = program_.kick_angle_rad * progress;
        cmd.phase = AscentPhase::PitchKick;
        cmd.thrust_direction_gcrf = std::cos(tilt) * frame.up + std::sin(tilt) * heading;
        return cmd;
    }
    // Gravity turn: thrust along the air-relative velocity (zero angle of attack).
    cmd.phase = AscentPhase::GravityTurn;
    cmd.thrust_direction_gcrf = math::normalized(relative);
    return cmd;
}

void AscentGuidance::update_insertion(const Vec3& r_hat, double r, double vt, double t_go,
                                      const vehicle::FlightEnvironment& env, time::Epoch epoch) {
    const double j2 = env.gravity.central_j2();
    const bool circular = circular_target(target_);
    if (j2 == 0.0 || !circular) return;
    const Vec3 pole = env.earth_orientation.fixed_to_gcrf(epoch).rotation * Vec3{0.0, 0.0, 1.0};
    const Vec3 node = math::normalized(cross(pole, plane_normal_));
    const Vec3 in_plane = cross(plane_normal_, node);
    // Cutoff point: the current position advanced along the target plane by the angle flown in the time to go.
    const double angle = 0.5 * (vt + target_speed_mps_) * t_go / r;
    const Vec3 forward = math::normalized(cross(plane_normal_, r_hat));
    const Vec3 at_cutoff = r_hat * std::cos(angle) + forward * std::sin(angle);
    const double u = std::atan2(dot(at_cutoff, in_plane), dot(at_cutoff, node));
    if (insertion_active_ && std::abs(std::remainder(u - insertion_u_rad_, 2.0 * constants::pi)) < j2_resolve_angle_rad) return;
    const double inclination = std::acos(std::clamp(dot(plane_normal_, pole), -1.0, 1.0));
    const auto s = j2_circular_insertion(gm_, target_radius_m_, inclination, u, j2, environment::earth_j2_reference_radius_m);
    insertion_active_ = true;
    insertion_u_rad_ = u;
    insertion_radius_m_ = s.radius_m;
    insertion_radial_mps_ = s.radial_speed_mps;
    insertion_horizontal_mps_ = s.horizontal_speed_mps;
}

GuidanceCommand AscentGuidance::peg(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                    time::Epoch epoch) {
    const auto& s = body.state();
    const auto perf = current_performance(body.vehicle(), 0.0);
    GuidanceCommand cmd;
    cmd.phase = AscentPhase::ClosedLoop;
    if (perf.thrust_n > 0.0 && peg_.converged()) {
        const double r = norm(s.position_gcrf_m);
        const Vec3 r_hat = s.position_gcrf_m / r;
        const Vec3 theta_hat = math::normalized(cross(plane_normal_, r_hat));
        update_insertion(r_hat, r, dot(s.velocity_gcrf_mps, theta_hat), peg_.time_to_go_s(), env, epoch);
    }
    PegTarget target;
    target.radius_m = insertion_active_ ? insertion_radius_m_ : target_radius_m_;
    target.horizontal_speed_mps = insertion_active_ ? insertion_horizontal_mps_ : target_speed_mps_;
    target.radial_speed_mps = insertion_active_ ? insertion_radial_mps_ : 0.0;
    target.plane_normal = plane_normal_;
    target.terminal_semi_major_axis_m =
        insertion_active_ ? 0.0 : 0.5 * (target_radius_m_ + equatorial_radius_m + target_.apoapsis_altitude_m);
    const auto peg = peg_.update({s.position_gcrf_m, s.velocity_gcrf_mps},
                                 {perf.thrust_n, perf.mass_flow_kgps, body.vehicle().mass_properties().mass_kg,
                                  shutdown_time_s(body.vehicle())},
                                 target, epoch);
    cmd.thrust_direction_gcrf = peg.thrust_direction;
    cmd.time_to_go_s = peg.time_to_go_s;
    if (perf.thrust_n > 0.0 && peg_.converged()) cutoff_epoch_ = epoch + peg.time_to_go_s;
    if (peg.cutoff) {
        cmd.cutoff = true;
        cmd.phase = AscentPhase::Cutoff;
    }
    return cmd;
}

}  // namespace starsim::gnc
