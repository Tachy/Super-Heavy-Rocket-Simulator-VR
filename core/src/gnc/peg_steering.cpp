#include "starsim/gnc/peg_steering.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double peg_freeze_time_s = 10.0;  // stop re-solving A, B close to cutoff (ill-conditioned)
constexpr double terminal_time_s = 5.0;     // switch to velocity-to-be-gained steering
// [E] time-to-go estimate: below this fraction of f_theta the direct dv solve is ill-conditioned (Earth upper stages
// stay near 1; the lunar ascent from the surface starts near -2 / 0.78).
constexpr double min_denominator_fraction = 0.5;
// The fallback needs a meaningful steering law, |fr| < 1 (f_theta = 1 - fr^2/2 > 0.5); degenerate states (e.g. an Earth
// upper stage after its insertion, fr ~ 40) keep the direct solve and its clamped time to go, as before l4.
constexpr double min_cosine_factor = 0.5;

}  // namespace

PegCommand PegSteering::update(const PegState& state, const PegVehicle& vehicle, const PegTarget& target,
                               time::Epoch epoch) {
    const double r = norm(state.position_m);
    const Vec3 r_hat = state.position_m / r;
    const Vec3 n_hat = target.plane_normal;
    const Vec3 theta_hat = math::normalized(cross(n_hat, r_hat));
    const double vr = dot(state.velocity_mps, r_hat);
    const double vt = dot(state.velocity_mps, theta_hat);
    const double vh = dot(state.velocity_mps, n_hat);

    PegCommand cmd;
    if (vehicle.thrust_n <= 0.0) {
        cmd.thrust_direction = math::normalized(state.velocity_mps);
        cmd.time_to_go_s = t_;
        return cmd;
    }
    const double mass = vehicle.mass_kg;
    const double a = vehicle.thrust_n / mass;
    const double ve = vehicle.thrust_n / vehicle.mass_flow_kgps;
    const double tau = ve / a;
    const double r_t = target.radius_m;
    const double vt_t = target.horizontal_speed_mps;
    const double vr_t = target.radial_speed_mps;

    auto solve_ab = [&](double t_go) {
        const double b0 = -ve * std::log(1.0 - t_go / tau);
        const double b1 = b0 * tau - ve * t_go;
        const double c0 = b0 * t_go - b1;
        const double c1 = c0 * tau - ve * t_go * t_go / 2.0;
        const double det = b0 * c1 - b1 * c0;
        const double rhs_v = vr_t - vr;               // target radial speed: 0 (insertion at periapsis; J2: solved)
        const double rhs_r = r_t - r - vr * t_go;
        a_ = (rhs_v * c1 - b1 * rhs_r) / det;
        b_ = (b0 * rhs_r - c0 * rhs_v) / det;
    };
    auto estimate_time_to_go = [&](double t_go) {
        const double c = (gm_ / (r * r) - vt * vt / r) / a;
        const double fr = a_ + c;
        const double a_t = a / (1.0 - t_go / tau);
        const double c_t = (gm_ / (r_t * r_t) - vt_t * vt_t / r_t) / a_t;
        const double fr_t = a_ + b_ * t_go + c_t;
        const double fr_dot = (fr_t - fr) / t_go;
        const double f_theta = 1.0 - fr * fr / 2.0;
        const double f_theta_dot = -fr * fr_dot;
        const double f_theta_ddot = -fr_dot * fr_dot / 2.0;
        const double dh = r_t * vt_t - r * vt;
        const double r_mean = 0.5 * (r + r_t);
        double dv = dh / r_mean + ve * t_go * (f_theta_dot + f_theta_ddot * tau) + f_theta_ddot * ve * t_go * t_go / 2.0;
        const double denominator = f_theta + f_theta_dot * tau + f_theta_ddot * tau * tau;
        if (f_theta > min_cosine_factor && denominator < min_denominator_fraction * f_theta) {
            // Strongly varying radial steering at a high thrust-to-weight (lunar ascent from the surface): solving the
            // horizontal-impulse equation for dv as above is ill-conditioned (denominator -> 0 or negative, the
            // iteration runs to a short, wrong time to go). Same equation, the thrust integrals taken at this t_go:
            // dh / r_mean = f_theta L + f_theta_dot S + f_theta_ddot Q (L = dv, S = int a t, Q = int a t^2).
            const double l = -ve * std::log(1.0 - t_go / tau);
            const double s_int = l * tau - ve * t_go;
            const double q_int = s_int * tau - ve * t_go * t_go / 2.0;
            dv = (dh / r_mean - f_theta_dot * s_int - f_theta_ddot * q_int) / f_theta;
        } else {
            dv /= denominator;
        }
        return tau * (1.0 - std::exp(-dv / ve));
    };

    if (!converged_) {
        t_ = std::clamp(tau * (1.0 - std::exp(-std::max(vt_t - vt, 100.0) / ve)), 10.0, 0.95 * tau);
        for (int i = 0; i < 30; ++i) {
            solve_ab(t_);
            t_ = std::clamp(estimate_time_to_go(t_), 0.5, 0.95 * tau);
        }
        converged_ = true;
        last_ = epoch;
    } else {
        const double elapsed = epoch - last_;
        last_ = epoch;
        t_ -= elapsed;
        if (t_ > peg_freeze_time_s) {
            solve_ab(t_);
            t_ = std::clamp(estimate_time_to_go(t_), 0.5, 0.95 * tau);
        } else {
            a_ += b_ * elapsed;  // advance the frozen linear steering law
        }
    }

    const double c = (gm_ / (r * r) - vt * vt / r) / a;
    const double fr = std::clamp(a_ + c, -0.9, 0.9);
    // Out-of-plane: null the velocity and position offset from the target plane together at the end of the burn
    // (zero-effort miss / velocity law: acceleration linear in time). A steep target plane from a low-latitude site
    // (Falcon 9 to 51.6 deg) starts the upper stage ~200 m/s and ~50 km off the plane; a law that only nulls the offset
    // under constant acceleration overshot it (+110 m/s, ~0.9 deg of inclination).
    const double t_go = std::max(t_, 5.0);
    const double plane_offset = dot(state.position_m, n_hat);
    const double fh = std::clamp((-4.0 * vh / t_go - 6.0 * plane_offset / (t_go * t_go)) / a, -0.5, 0.5);
    const double ft = std::sqrt(std::max(0.0, 1.0 - fr * fr - fh * fh));

    cmd.thrust_direction = math::normalized(fr * r_hat + ft * theta_hat + fh * n_hat);
    cmd.time_to_go_s = t_;

    // Terminal phase (last seconds): steer along the velocity still to be gained, v_go = v_target - v, with
    // v_target horizontal, in the target plane, at the vis-viva speed for the current radius. Cutoff when
    // the shutdown transient (half thrust over the shutdown time) supplies the rest - along v_go, so it
    // adds no radial error. The PEG time-to-go alone runs ~1 s short here (large final radial speed).
    const bool kepler = target.terminal_semi_major_axis_m > 0.0;
    const double required_speed = kepler ? std::sqrt(gm_ * (2.0 / r - 1.0 / target.terminal_semi_major_axis_m)) : 0.0;
    // Horizontal direction in the CURRENT orbital plane: the residual plane error (~0.05 deg) must not keep
    // |v_go| from converging (it would overshoot the in-plane speed).
    const Vec3 h_now = cross(state.position_m, state.velocity_mps);
    const Vec3 horizontal_now = math::normalized(cross(h_now, r_hat));
    // ... turned into the target plane by the out-of-plane speed left (the plane steering above ends ~10 m/s short
    // of it on a steep plane: 0.1 deg of inclination), within what the tail-off cannot fix anyway.
    // Solved insertion (e.g. J2-aware circular): the given horizontal and radial speed instead of the Kepler speed.
    const Vec3 v_target = kepler ? required_speed * horizontal_now
                                 : target.horizontal_speed_mps * horizontal_now + target.radial_speed_mps * r_hat;
    const Vec3 v_go = v_target - state.velocity_mps - vh * n_hat;
    const double tail_off_dv = 0.5 * a * vehicle.shutdown_s;
    if (t_ < terminal_time_s) {
        cmd.thrust_direction = math::normalized(v_go);
        if (norm(v_go) <= tail_off_dv || dot(v_go, state.velocity_mps) < 0.0 || t_ < -5.0) cmd.cutoff = true;
        if (cmd.cutoff && std::getenv("STARSIM_DEBUG_PEG")) {  // diagnostics: why and where the burn ends
            std::printf("[peg] cutoff: r %.1f km (target %.1f), vr %.2f m/s (target %.2f), vt %.2f m/s (required %.2f), "
                        "|v_go| %.2f (tail-off %.2f), v_go.v %.1f, t_go %.2f s, a %.2f m/s^2\n",
                        r / 1e3, r_t / 1e3, vr, vr_t, vt, kepler ? required_speed : vt_t, norm(v_go), tail_off_dv,
                        dot(v_go, state.velocity_mps), t_, a);
        }
    }
    return cmd;
}

}  // namespace starsim::gnc
