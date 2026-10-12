#pragma once

#include "starsim/math/vec3.hpp"
#include "starsim/math/vec_n.hpp"

namespace starsim::math {

// Translational state (position, velocity). The frame/centre is given by the variable name.
struct StateVector {
    Vec3 position_m;
    Vec3 velocity_mps;

    friend constexpr bool operator==(const StateVector&, const StateVector&) = default;
};

using OdeState6 = VecN<6>;

constexpr OdeState6 pack(const StateVector& s) {
    return {{s.position_m.x, s.position_m.y, s.position_m.z, s.velocity_mps.x, s.velocity_mps.y, s.velocity_mps.z}};
}

constexpr StateVector unpack(const OdeState6& y) {
    return {{y[0], y[1], y[2]}, {y[3], y[4], y[5]}};
}

}  // namespace starsim::math
