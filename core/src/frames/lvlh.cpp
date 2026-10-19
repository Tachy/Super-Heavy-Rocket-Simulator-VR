#include "starsim/frames/lvlh.hpp"

#include "starsim/math/mat3.hpp"

namespace starsim::frames {

using math::Quat;
using math::Vec3;

namespace {

Quat from_columns(const Vec3& x, const Vec3& y, const Vec3& z) {
    math::Mat3 m;
    const Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = columns[c].x;
        m(1, c) = columns[c].y;
        m(2, c) = columns[c].z;
    }
    return Quat::from_matrix(m);
}

}  // namespace

Quat lvlh_to_inertial(const math::StateVector& state) {
    const Vec3 z = -math::normalized(state.position_m);
    const Vec3 y = -math::normalized(cross(state.position_m, state.velocity_mps));
    return from_columns(cross(y, z), y, z);
}

Vec3 lvlh_rate_inertial_radps(const math::StateVector& state) {
    const double r2 = dot(state.position_m, state.position_m);
    return cross(state.position_m, state.velocity_mps) * (1.0 / r2);
}

}  // namespace starsim::frames
