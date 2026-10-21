#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// The ship's last-resort abort (ship catch robustness, 2026-10-21): when no abort plan converges the ship descends
// upright on a deterministic law - no optimiser, nothing that can fail to converge. Horizontal: away from the tower
// inside clear_radius, else stop; vertical: sink profile sqrt(final_sink^2 + 2 f (a_max - g) h).
struct UprightDescentParameters {
    double tilt_max_rad = 15.0 * constants::deg_to_rad;  // [E] spec
    double gain_per_s = 0.5;             // [E] horizontal velocity loop
    double away_speed_mps = 5.0;         // [E] drift away from the tower inside clear_radius
    double clear_radius_m = 60.0;        // [E] tower keep-out (~25 m) + a margin for the hull's lean
    double final_sink_mps = 2.0;         // [E] spec: contact at <= 3 m/s
    double deceleration_fraction = 0.5;  // [E] share of the excess acceleration the sink profile plans with
    double sink_gain_per_s = 1.0;        // [E] vertical velocity loop
};

struct UprightDescentCommand {
    math::Vec3 axis;                 // unit thrust axis
    double acceleration_mps2 = 0.0;  // thrust acceleration wanted (<= max_accel)
};

// from_tower_m: from the tower centre to the CG; velocity_mps Earth-fixed; height_m: of the engines above the ground
// or water; min/max_accel_mps2: of the running engines at minimum / full throttle. One frame for all vectors.
UprightDescentCommand upright_descent(const math::Vec3& from_tower_m, const math::Vec3& velocity_mps, const math::Vec3& up,
                                      double height_m, double min_accel_mps2, double max_accel_mps2, double gravity_mps2,
                                      const UprightDescentParameters& p = {});

}  // namespace starsim::gnc
