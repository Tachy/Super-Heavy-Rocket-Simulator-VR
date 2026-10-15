#include "starsim/gnc/apollo_braking.hpp"

#include <cmath>

// Derivation: with a(tau) = a0 + a1 tau + a2 tau^2 on [0, T] and the end conditions r(T) = r_T, v(T) = v_T,
// a(T) = a_T: a0 = a_T + 12 (r_T - r) / T^2 - 6 (v_T + v) / T; the end jerk is
// j(T) = (24 (r_T - r - v T) / T^2 - 18 (v_T - v) / T + 6 a_T) / T, and j_z(T) = J_T gives Klumpp's cubic
// J_T T^3 - 6 a_T T^2 + (18 v_T + 6 v) T - 24 (r_T - r) = 0 (downrange components). Check: constant acceleration
// 1 m/s^2 from rest over 10 s (r_T = 50, v_T = 10, a_T = 1, J = 0) satisfies the cubic at T = 10 and gives a0 = 1.
// Klumpp counts the target-referenced time negative; here t_go is positive and the formulas are converted accordingly.

namespace starsim::gnc {

namespace {
constexpr int max_newton_iterations = 30;
constexpr double newton_tolerance_s = 1e-9;
}  // namespace

std::optional<double> braking_time_to_go_s(const BrakingTarget& target, double r, double v, double guess_s) {
    const double j = target.jerk_downrange_mps3, a = target.acceleration_mps2.z;
    const double vt = target.velocity_mps.z, rt = target.position_m.z;
    double t = guess_s;
    for (int i = 0; i < max_newton_iterations; ++i) {
        const double f = j * t * t * t - 6.0 * a * t * t + (18.0 * vt + 6.0 * v) * t - 24.0 * (rt - r);
        const double df = 3.0 * j * t * t - 12.0 * a * t + 18.0 * vt + 6.0 * v;
        if (df == 0.0) return std::nullopt;
        const double step = f / df;
        t -= step;
        if (std::abs(step) < newton_tolerance_s) return t > 0.0 ? std::optional<double>(t) : std::nullopt;
    }
    return std::nullopt;
}

math::Vec3 braking_acceleration(const BrakingTarget& target, const math::Vec3& r, const math::Vec3& v, double t_go) {
    return target.acceleration_mps2 + (target.position_m - r) * (12.0 / (t_go * t_go)) -
           (target.velocity_mps + v) * (6.0 / t_go);
}

ApolloBrakingGuidance::Command ApolloBrakingGuidance::update(const math::Vec3& r, const math::Vec3& v,
                                                             const math::Vec3& gravity_mps2) {
    Command cmd;
    const auto t_go = braking_time_to_go_s(target_, r.z, v.z, t_go_);
    if (!t_go) return cmd;
    t_go_ = *t_go;
    cmd.time_to_go_s = t_go_;
    cmd.thrust_acceleration_mps2 = braking_acceleration(target_, r, v, t_go_) - gravity_mps2;
    cmd.valid = true;
    return cmd;
}

}  // namespace starsim::gnc
