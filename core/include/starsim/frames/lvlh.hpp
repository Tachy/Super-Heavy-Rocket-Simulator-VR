#pragma once

#include "starsim/math/quat.hpp"
#include "starsim/math/state_vector.hpp"

namespace starsim::frames {

// Local vertical / local horizontal axes of an orbit state relative to its central body (inertial axes): x completes
// the triad (along the velocity on a circular orbit), y = -orbit normal, z = nadir. Returns the rotation LVLH -> inertial.
math::Quat lvlh_to_inertial(const math::StateVector& state);

// Angular velocity of the LVLH frame in inertial axes: (r x v) / r^2 (the orbit rate on a circular orbit).
math::Vec3 lvlh_rate_inertial_radps(const math::StateVector& state);

}  // namespace starsim::frames
