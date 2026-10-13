#pragma once

#include "starsim/math/quat.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Commanded attitudes that change smoothly (the entry's terminal phase).

// Body -> frame rotation from the body x-axis and body +y (made orthogonal).
math::Quat attitude_from_axes(const math::Vec3& x, const math::Vec3& y);
// `from` turned towards `to` by at most `max_angle_rad` (shortest rotation).
math::Quat slew_toward(const math::Quat& from, const math::Quat& to, double max_angle_rad);

}  // namespace starsim::gnc
