#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// The ship's flip brake (ship catch robustness, 2026-10-21): once the flip has the hull below the flaps' tilt, the thrust
// axis leans against the horizontal velocity relative to a reference that points to the catch axis with a stopping
// profile v_ref = min(max_speed, sqrt(2 * brake_fraction * a_max * d)), a_max = thrust acceleration * sin(tilt_max).
struct FlipBrakeParameters {
    double tilt_max_rad = 35.0 * constants::deg_to_rad;  // [E] spec: three Raptors ~2-3 g give ~15-20 m/s^2 across
    double brake_fraction = 0.7;  // [E] the profile plans with this share of a_max (margin for the slew and spool)
    double gain_per_s = 0.8;      // [E] horizontal velocity loop
    double max_speed_mps = 40.0;  // [E] cap of the reference speed towards the axis
};

// Unit thrust axis. to_axis_m: from the CG to the catch axis; velocity_mps: Earth-fixed; up: local vertical (unit); all
// in one frame, vertical parts are ignored. Zero thrust (engines still spooling): the vertical.
math::Vec3 flip_brake_axis(const math::Vec3& to_axis_m, const math::Vec3& velocity_mps, const math::Vec3& up,
                           double thrust_accel_mps2, const FlipBrakeParameters& p = {});

}  // namespace starsim::gnc
