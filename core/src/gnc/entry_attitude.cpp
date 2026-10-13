#include "starsim/gnc/entry_attitude.hpp"

#include <cmath>

#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

struct Axes {
    Vec3 x, y, z;  // body axes in GCRF
};

Axes entry_axes(const Vec3& air_velocity_gcrf, const Vec3& up_gcrf, double alpha_rad, double bank_rad,
                double belly_azimuth_rad) {
    const Vec3 u = math::normalized(air_velocity_gcrf);
    Vec3 l0 = up_gcrf - u * dot(up_gcrf, u);
    if (norm(l0) < 1e-9) {  // vertical flight: any lift direction perpendicular to the velocity
        l0 = cross(u, std::abs(u.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0});
    }
    l0 = math::normalized(l0);
    const Vec3 side = cross(u, l0);
    const Vec3 l = l0 * std::cos(bank_rad) - side * std::sin(bank_rad);  // right-handed about u
    Axes a;
    a.x = u * std::cos(alpha_rad) + l * std::sin(alpha_rad);
    const Vec3 b = u * std::sin(alpha_rad) - l * std::cos(alpha_rad);
    const Vec3 c = cross(a.x, b);  // body direction at belly azimuth + 90 deg
    a.y = b * std::cos(belly_azimuth_rad) - c * std::sin(belly_azimuth_rad);
    a.z = cross(a.x, a.y);
    return a;
}

}  // namespace

EntryAttitude entry_attitude(const Vec3& air_velocity_gcrf, const Vec3& up_gcrf, double alpha_rad, double bank_rad,
                             double belly_azimuth_rad) {
    const Axes a = entry_axes(air_velocity_gcrf, up_gcrf, alpha_rad, bank_rad, belly_azimuth_rad);
    return {a.x, a.y};
}

math::Quat entry_attitude_quaternion(const Vec3& air_velocity_gcrf, const Vec3& up_gcrf, double alpha_rad,
                                     double bank_rad, double belly_azimuth_rad) {
    const Axes a = entry_axes(air_velocity_gcrf, up_gcrf, alpha_rad, bank_rad, belly_azimuth_rad);
    math::Mat3 m;
    const Vec3 cols[3] = {a.x, a.y, a.z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = cols[c].x;
        m(1, c) = cols[c].y;
        m(2, c) = cols[c].z;
    }
    return math::Quat::from_matrix(m).normalized();
}

}  // namespace starsim::gnc
