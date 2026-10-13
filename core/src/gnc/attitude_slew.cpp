#include "starsim/gnc/attitude_slew.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Vec3;

math::Quat attitude_from_axes(const Vec3& x_axis, const Vec3& y_axis) {
    const Vec3 x = math::normalized(x_axis);
    const Vec3 y = math::normalized(y_axis - x * dot(x, y_axis));
    const Vec3 z = cross(x, y);
    math::Mat3 m;
    const Vec3 cols[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = cols[c].x;
        m(1, c) = cols[c].y;
        m(2, c) = cols[c].z;
    }
    return math::Quat::from_matrix(m).normalized();
}

math::Quat slew_toward(const math::Quat& from, const math::Quat& to, double max_angle_rad) {
    math::Quat d = (to * from.conjugate()).normalized();
    if (d.w < 0.0) d = {-d.w, -d.x, -d.y, -d.z};
    const double angle = 2.0 * std::acos(std::clamp(d.w, -1.0, 1.0));
    if (angle <= max_angle_rad || angle < 1e-12) return to;
    const Vec3 axis{d.x, d.y, d.z};
    return (math::Quat::from_axis_angle(axis, max_angle_rad) * from).normalized();
}

}  // namespace starsim::gnc
