#include "starsim/gnc/ascent_guidance.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double earth_rotation_rate_radps = 7.2921150e-5;
constexpr double equatorial_radius_m = 6378137.0;
constexpr double peg_freeze_time_s = 10.0;  // stop re-solving A, B close to cutoff (ill-conditioned)
constexpr double terminal_time_s = 5.0;     // switch to velocity-to-be-gained steering

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

AscentGuidance::AscentGuidance(AscentTarget target, PitchProgram program, double gm_earth_m3ps2)
    : target_(target), program_(program), gm_(gm_earth_m3ps2) {
    const double rp = equatorial_radius_m + target_.periapsis_altitude_m;
    const double ra = equatorial_radius_m + target_.apoapsis_altitude_m;
    target_radius_m_ = rp;
    target_speed_mps_ = std::sqrt(gm_ * (2.0 / rp - 2.0 / (rp + ra)));  // vis-viva at periapsis
}

void AscentGuidance::start(const vehicle::FlightBody& body, const frames::BodyOrientation& earth, time::Epoch epoch) {
    const auto frame = local_frame(body.state().position_gcrf_m, earth, epoch);
    const double latitude = frame.latitude_rad;
    azimuth_rad_ = gnc::launch_azimuth_rad(latitude, target_.inclination_rad, target_speed_mps_, target_.northerly);
    // Target plane: through the Earth's centre, containing the site and the inertial launch direction.
    const double s = std::clamp(std::cos(target_.inclination_rad) / std::cos(latitude), -1.0, 1.0);
    const double inertial = target_.northerly ? std::asin(s) : constants::pi - std::asin(s);
    const Vec3 heading = std::sin(inertial) * frame.east + std::cos(inertial) * frame.north;
    plane_normal_ = math::normalized(cross(frame.up, heading));
    liftoff_ = epoch;
    kick_start_s_ = -1.0;
    peg_converged_ = false;
}

GuidanceCommand AscentGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                       time::Epoch epoch, bool closed_loop) {
    return closed_loop ? peg(body, env, epoch) : open_loop(body, env, epoch);
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

GuidanceCommand AscentGuidance::peg(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                    time::Epoch epoch) {
    const auto& s = body.state();
    const double r = norm(s.position_gcrf_m);
    const Vec3 r_hat = s.position_gcrf_m / r;
    const Vec3 n_hat = plane_normal_;
    const Vec3 theta_hat = math::normalized(cross(n_hat, r_hat));
    const double vr = dot(s.velocity_gcrf_mps, r_hat);
    const double vt = dot(s.velocity_gcrf_mps, theta_hat);
    const double vh = dot(s.velocity_gcrf_mps, n_hat);

    const auto perf = current_performance(body.vehicle(), 0.0);
    GuidanceCommand cmd;
    cmd.phase = AscentPhase::ClosedLoop;
    if (perf.thrust_n <= 0.0) {
        cmd.thrust_direction_gcrf = math::normalized(s.velocity_gcrf_mps);
        return cmd;
    }
    const double mass = body.vehicle().mass_properties().mass_kg;
    const double a = perf.thrust_n / mass;
    const double ve = perf.thrust_n / perf.mass_flow_kgps;
    const double tau = ve / a;
    const double r_t = target_radius_m_;
    const double vt_t = target_speed_mps_;

    auto solve_ab = [&](double t_go) {
        const double b0 = -ve * std::log(1.0 - t_go / tau);
        const double b1 = b0 * tau - ve * t_go;
        const double c0 = b0 * t_go - b1;
        const double c1 = c0 * tau - ve * t_go * t_go / 2.0;
        const double det = b0 * c1 - b1 * c0;
        const double rhs_v = 0.0 - vr;                // target radial speed: 0 (insertion at periapsis)
        const double rhs_r = r_t - r - vr * t_go;
        peg_a_ = (rhs_v * c1 - b1 * rhs_r) / det;
        peg_b_ = (b0 * rhs_r - c0 * rhs_v) / det;
    };
    auto estimate_time_to_go = [&](double t_go) {
        const double c = (gm_ / (r * r) - vt * vt / r) / a;
        const double fr = peg_a_ + c;
        const double a_t = a / (1.0 - t_go / tau);
        const double c_t = (gm_ / (r_t * r_t) - vt_t * vt_t / r_t) / a_t;
        const double fr_t = peg_a_ + peg_b_ * t_go + c_t;
        const double fr_dot = (fr_t - fr) / t_go;
        const double f_theta = 1.0 - fr * fr / 2.0;
        const double f_theta_dot = -fr * fr_dot;
        const double f_theta_ddot = -fr_dot * fr_dot / 2.0;
        const double dh = r_t * vt_t - r * vt;
        const double r_mean = 0.5 * (r + r_t);
        double dv = dh / r_mean + ve * t_go * (f_theta_dot + f_theta_ddot * tau) + f_theta_ddot * ve * t_go * t_go / 2.0;
        dv /= f_theta + f_theta_dot * tau + f_theta_ddot * tau * tau;
        return tau * (1.0 - std::exp(-dv / ve));
    };

    if (!peg_converged_) {
        peg_t_ = std::clamp(tau * (1.0 - std::exp(-std::max(vt_t - vt, 100.0) / ve)), 10.0, 0.95 * tau);
        for (int i = 0; i < 30; ++i) {
            solve_ab(peg_t_);
            peg_t_ = std::clamp(estimate_time_to_go(peg_t_), 0.5, 0.95 * tau);
        }
        peg_converged_ = true;
        peg_last_ = epoch;
    } else {
        const double elapsed = epoch - peg_last_;
        peg_last_ = epoch;
        peg_t_ -= elapsed;
        if (peg_t_ > peg_freeze_time_s) {
            solve_ab(peg_t_);
            peg_t_ = std::clamp(estimate_time_to_go(peg_t_), 0.5, 0.95 * tau);
        } else {
            peg_a_ += peg_b_ * elapsed;  // advance the frozen linear steering law
        }
    }

    const double c = (gm_ / (r * r) - vt * vt / r) / a;
    const double fr = std::clamp(peg_a_ + c, -0.9, 0.9);
    // Out-of-plane: null the velocity and position offset from the target plane over the remaining burn.
    const double t_go = std::max(peg_t_, 5.0);
    const double plane_offset = dot(s.position_gcrf_m, n_hat);
    const double fh = std::clamp((-vh / t_go - 2.0 * plane_offset / (t_go * t_go)) / a, -0.3, 0.3);
    const double ft = std::sqrt(std::max(0.0, 1.0 - fr * fr - fh * fh));

    cmd.thrust_direction_gcrf = math::normalized(fr * r_hat + ft * theta_hat + fh * n_hat);
    cmd.time_to_go_s = peg_t_;

    // Terminal phase (last seconds): steer along the velocity still to be gained, v_go = v_target - v, with
    // v_target horizontal, in the target plane, at the vis-viva speed for the current radius. Cutoff when
    // the shutdown transient (half thrust over the shutdown time) supplies the rest - along v_go, so it
    // adds no radial error. The PEG time-to-go alone runs ~1 s short here (large final radial speed).
    const double target_semi_major_axis = 0.5 * (target_radius_m_ + equatorial_radius_m + target_.apoapsis_altitude_m);
    const double required_speed = std::sqrt(gm_ * (2.0 / r - 1.0 / target_semi_major_axis));
    // Horizontal direction in the CURRENT orbital plane: the residual plane error (~0.05 deg) must not keep
    // |v_go| from converging (it would overshoot the in-plane speed).
    const Vec3 h_now = cross(s.position_gcrf_m, s.velocity_gcrf_mps);
    const Vec3 horizontal_now = math::normalized(cross(h_now, r_hat));
    const Vec3 v_go = required_speed * horizontal_now - s.velocity_gcrf_mps;
    const double tail_off_dv = 0.5 * a * shutdown_time_s(body.vehicle());
    if (peg_t_ < terminal_time_s) {
        cmd.thrust_direction_gcrf = math::normalized(v_go);
        if (norm(v_go) <= tail_off_dv || dot(v_go, s.velocity_gcrf_mps) < 0.0 || peg_t_ < -5.0) {
            cmd.cutoff = true;
            cmd.phase = AscentPhase::Cutoff;
        }
    }
    return cmd;
}

}  // namespace starsim::gnc
