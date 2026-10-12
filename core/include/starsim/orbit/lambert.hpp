#pragma once

#include <optional>

#include "starsim/math/vec3.hpp"

namespace starsim::orbit {

struct LambertSolution {
    math::Vec3 departure_velocity_mps;
    math::Vec3 arrival_velocity_mps;
};

// Lambert's problem (single revolution): the two-body arc from r1 to r2 in `time_of_flight_s`.
// Universal-variable formulation (Bate, Mueller & White; Curtis, Algorithm 5.2) solved by bisection on z,
// which is robust over the whole single-revolution range. The transfer direction follows
// `reference_normal` (angular momentum on the same side). Returns nothing for a 180 deg transfer
// (plane undefined) or when no solution exists.
std::optional<LambertSolution> solve_lambert(const math::Vec3& r1_m, const math::Vec3& r2_m, double time_of_flight_s,
                                             double gm_m3ps2, const math::Vec3& reference_normal = {0.0, 0.0, 1.0});

}  // namespace starsim::orbit
