#include "starsim/gnc/ascent_guidance.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/j2_insertion.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

// [E] re-solve the J2-aware insertion when the predicted cutoff point moves by more than this.
constexpr double j2_resolve_angle_rad = 2.0 * constants::deg_to_rad;

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
    const auto& schedule = program_.booster_throttle;
    const double t = epoch - liftoff_;
    if (!schedule.empty()) {
        if (t <= schedule.front().first) {
            cmd.throttle = schedule.front().second;
        } else if (t >= schedule.back().first) {
            cmd.throttle = schedule.back().second;
        } else {
            for (size_t i = 1; i < schedule.size(); ++i) {
                if (t > schedule[i].first) continue;
                const auto& [t0, f0] = schedule[i - 1];
                const auto& [t1, f1] = schedule[i];
                cmd.throttle = f0 + (f1 - f0) * (t - t0) / (t1 - t0);
                break;
            }
        }
    }
    return cmd;
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
    const bool circular = std::abs(target_.apoapsis_altitude_m - target_.periapsis_altitude_m) < 1.0;
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
    if (peg.cutoff) {
        cmd.cutoff = true;
        cmd.phase = AscentPhase::Cutoff;
    }
    return cmd;
}

}  // namespace starsim::gnc
