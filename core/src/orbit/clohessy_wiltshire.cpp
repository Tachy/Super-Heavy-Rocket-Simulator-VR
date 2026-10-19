#include "starsim/orbit/clohessy_wiltshire.hpp"

#include <cmath>

#include "starsim/frames/lvlh.hpp"

namespace starsim::orbit {

using math::Vec3;

namespace {
constexpr double singular_tolerance = 1e-9;  // of the dimensionless determinants below
}

// CW in this LVLH convention: x'' = 2 n z', z'' = -2 n x' + 3 n^2 z, y'' = -n^2 y (Fehse, Automated Rendezvous and
// Docking of Spacecraft, eq. 3.25 with x = V-bar, z = R-bar).
RelativeState cw_propagate(const RelativeState& state, double n, double t) {
    const double tau = n * t, s = std::sin(tau), c = std::cos(tau);
    const Vec3& r = state.position_m;
    const Vec3& v = state.velocity_mps;
    RelativeState out;
    out.position_m.x = r.x + 6.0 * (tau - s) * r.z + (4.0 * s - 3.0 * tau) / n * v.x + 2.0 * (1.0 - c) / n * v.z;
    out.position_m.y = r.y * c + v.y / n * s;
    out.position_m.z = (4.0 - 3.0 * c) * r.z + 2.0 * (c - 1.0) / n * v.x + s / n * v.z;
    out.velocity_mps.x = 6.0 * n * (1.0 - c) * r.z + (4.0 * c - 3.0) * v.x + 2.0 * s * v.z;
    out.velocity_mps.y = -r.y * n * s + v.y * c;
    out.velocity_mps.z = 3.0 * n * s * r.z - 2.0 * s * v.x + c * v.z;
    return out;
}

std::optional<Vec3> cw_transfer_velocity(const Vec3& from, const Vec3& to, double n, double t) {
    const double tau = n * t, s = std::sin(tau), c = std::cos(tau);
    // In plane: [x; z](t) - (terms of the initial position) = (1/n) [[4s - 3 tau, 2(1 - c)], [2(c - 1), s]] [vx; vz].
    const double a11 = (4.0 * s - 3.0 * tau), a12 = 2.0 * (1.0 - c), a21 = 2.0 * (c - 1.0), a22 = s;
    const double det = a11 * a22 - a12 * a21;  // 8 - 8 cos tau - 3 tau sin tau
    if (std::abs(det) < singular_tolerance) return std::nullopt;
    const double bx = n * (to.x - (from.x + 6.0 * (tau - s) * from.z));
    const double bz = n * (to.z - (4.0 - 3.0 * c) * from.z);
    Vec3 v;
    v.x = (a22 * bx - a12 * bz) / det;
    v.z = (a11 * bz - a21 * bx) / det;
    const double dy = to.y - from.y * c;
    if (std::abs(s) < singular_tolerance) {
        if (std::abs(dy) > 1e-6) return std::nullopt;
        v.y = 0.0;
    } else {
        v.y = n * dy / s;
    }
    return v;
}

RelativeState lvlh_relative_state(const math::StateVector& target, const math::StateVector& chaser) {
    const auto to_inertial = frames::lvlh_to_inertial(target);
    const Vec3 omega = frames::lvlh_rate_inertial_radps(target);
    const Vec3 dr = chaser.position_m - target.position_m;
    const Vec3 dv = chaser.velocity_mps - target.velocity_mps - cross(omega, dr);
    return {to_inertial.inverse_rotate(dr), to_inertial.inverse_rotate(dv)};
}

math::StateVector chaser_from_lvlh(const math::StateVector& target, const RelativeState& relative) {
    const auto to_inertial = frames::lvlh_to_inertial(target);
    const Vec3 omega = frames::lvlh_rate_inertial_radps(target);
    const Vec3 dr = to_inertial.rotate(relative.position_m);
    return {target.position_m + dr, target.velocity_mps + to_inertial.rotate(relative.velocity_mps) + cross(omega, dr)};
}

}  // namespace starsim::orbit
