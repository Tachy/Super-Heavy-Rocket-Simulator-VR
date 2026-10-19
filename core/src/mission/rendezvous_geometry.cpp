#include "starsim/mission/rendezvous_geometry.hpp"

#include <cmath>

#include "starsim/orbit/clohessy_wiltshire.hpp"

namespace starsim::mission {

using math::Vec3;

Vec3 point_behind_below(const math::StateVector& target, double behind_m, double below_m) {
    const double r = norm(target.position_m);
    const Vec3 radial = target.position_m / r;
    const Vec3 normal = normalized(cross(target.position_m, target.velocity_mps));
    const Vec3 along = cross(normal, radial);
    const double angle = -behind_m / r;
    return (r - below_m) * (std::cos(angle) * radial + std::sin(angle) * along);
}

double approach_ellipsoid_metric(const math::StateVector& target, const Vec3& chaser_position_m) {
    const Vec3 p = orbit::lvlh_relative_state(target, {chaser_position_m, target.velocity_mps}).position_m;
    const auto& a = approach_ellipsoid_semi_axes_m;
    return (p.x / a[0]) * (p.x / a[0]) + (p.y / a[1]) * (p.y / a[1]) + (p.z / a[2]) * (p.z / a[2]);
}

double in_plane_angle_rad(const math::StateVector& target, const Vec3& position_m) {
    const Vec3 radial = normalized(target.position_m);
    const Vec3 normal = normalized(cross(target.position_m, target.velocity_mps));
    const Vec3 along = cross(normal, radial);
    return std::atan2(dot(position_m, along), dot(position_m, radial));
}

}  // namespace starsim::mission
