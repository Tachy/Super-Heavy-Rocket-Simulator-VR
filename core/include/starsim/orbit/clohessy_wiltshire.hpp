#pragma once

#include <optional>

#include "starsim/math/state_vector.hpp"

namespace starsim::orbit {

// Relative motion of a chaser near a target on a circular orbit (Clohessy-Wiltshire / Hill) in the target's LVLH frame
// of frames/lvlh.hpp: x along the velocity (V-bar), y = -orbit normal (H-bar), z = nadir (R-bar, towards the Earth).
// Linear and two-body (no J2, no drag): first guesses and timing only; the burns are targeted in the full model.
struct RelativeState {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;  // relative to the rotating LVLH frame
};

// Closed-form CW state after `t_s`; `n_radps`: mean motion of the target orbit.
RelativeState cw_propagate(const RelativeState& state, double n_radps, double t_s);

// Initial relative velocity that carries the chaser from `from_m` to `to_m` in `t_s` (two-impulse transfer). Empty when
// the in-plane problem is singular (t at a multiple of the period) or the out-of-plane one (t at a multiple of half
// the period with an out-of-plane change).
std::optional<math::Vec3> cw_transfer_velocity(const math::Vec3& from_m, const math::Vec3& to_m, double n_radps,
                                               double t_s);

// The chaser relative to the target in the target's LVLH frame (velocity relative to the rotating frame), and back.
RelativeState lvlh_relative_state(const math::StateVector& target, const math::StateVector& chaser);
math::StateVector chaser_from_lvlh(const math::StateVector& target, const RelativeState& relative);

}  // namespace starsim::orbit
