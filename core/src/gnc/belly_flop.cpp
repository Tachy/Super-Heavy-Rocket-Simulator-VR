#include "starsim/gnc/belly_flop.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double min_sink_rate_mps = 20.0;

}  // namespace

EntryAttitude belly_flop_attitude(const Vec3& up, const Vec3& nose_horizontal, const Vec3& tilt_direction,
                                  double tilt_rad, double belly_azimuth_rad) {
    const Vec3 b = -(up * std::cos(tilt_rad) + tilt_direction * std::sin(tilt_rad));
    Vec3 x = nose_horizontal - b * dot(b, nose_horizontal);
    if (norm(x) < 1e-9) x = cross(b, std::abs(b.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0});
    x = math::normalized(x);
    const Vec3 c = cross(x, b);  // body direction at belly azimuth + 90 deg (as entry_attitude)
    const Vec3 y = b * std::cos(belly_azimuth_rad) - c * std::sin(belly_azimuth_rad);
    return {x, y};
}

Vec3 belly_flop_acceleration(const Vec3& offset_m, const Vec3& velocity_mps, double height_above_gate_m,
                             double sink_rate_mps, double gain) {
    const double t_go = std::max(height_above_gate_m, 1.0) / std::max(sink_rate_mps, min_sink_rate_mps);
    return -(offset_m + velocity_mps * t_go) * (gain / (t_go * t_go));
}

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
