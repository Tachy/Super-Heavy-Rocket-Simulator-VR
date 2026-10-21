#include "starsim/gnc/flip_brake.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

Vec3 flip_brake_axis(const Vec3& to_axis_m, const Vec3& velocity_mps, const Vec3& up, double thrust_accel_mps2,
                     const FlipBrakeParameters& p) {
    if (thrust_accel_mps2 <= 0.0) return up;
    const Vec3 d = to_axis_m - up * dot(to_axis_m, up);
    const Vec3 v = velocity_mps - up * dot(velocity_mps, up);
    const double a_max = thrust_accel_mps2 * std::sin(p.tilt_max_rad);
    const double distance = norm(d);
    Vec3 v_ref{0.0, 0.0, 0.0};
    Vec3 a_feedforward{0.0, 0.0, 0.0};
    if (distance > 1e-6) {
        const Vec3 direction = d / distance;
        const double profile = std::sqrt(2.0 * p.brake_fraction * a_max * distance);
        v_ref = direction * std::min(p.max_speed_mps, profile);
        // on the stopping profile the reference itself decelerates at brake_fraction * a_max: fed forward
        if (profile < p.max_speed_mps) a_feedforward = direction * (-p.brake_fraction * a_max);
    }
    Vec3 a = (v_ref - v) * p.gain_per_s + a_feedforward;
    const double a_norm = norm(a);
    if (a_norm > a_max) a = a * (a_max / a_norm);
    const double vertical = std::sqrt(std::max(thrust_accel_mps2 * thrust_accel_mps2 - dot(a, a), 0.0));
    return math::normalized(a + up * vertical);
}

}  // namespace starsim::gnc
