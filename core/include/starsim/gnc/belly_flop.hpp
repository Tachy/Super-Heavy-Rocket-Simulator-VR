#pragma once

#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Subsonic belly flop: the ship falls belly down at terminal speed. The aerodynamic force (~ the weight) acts along
// the belly normal, so tilting the belly by `tilt_rad` away from a horizontal direction h accelerates the ship
// towards h with ~ g tan(tilt). Belly = -(up cos(tilt) + h sin(tilt)); the nose along `nose_horizontal` made
// perpendicular to the belly (a tilt towards the nose is a nose-down pitch: angle of attack 90 deg - tilt).
// Needs no velocity (the fall is near vertical). All vectors in one frame; `tilt_direction` horizontal unit.
EntryAttitude belly_flop_attitude(const math::Vec3& up, const math::Vec3& nose_horizontal,
                                  const math::Vec3& tilt_direction, double tilt_rad, double belly_azimuth_rad);

// Zero-effort-miss steering to the gate point: horizontal acceleration that brings the ship over the gate when it
// sinks to the gate height. `offset_m`: horizontal position relative to the gate point; `velocity_mps`: horizontal
// velocity relative to the ground; time to go = height above the gate / sink rate (>= 20 m/s).
// a = -gain (offset + velocity t_go) / t_go^2.
math::Vec3 belly_flop_acceleration(const math::Vec3& offset_m, const math::Vec3& velocity_mps, double height_above_gate_m,
                                   double sink_rate_mps, double gain);

// Body -> frame rotation from the body x-axis and body +y (made orthogonal).
math::Quat attitude_from_axes(const math::Vec3& x, const math::Vec3& y);
// `from` turned towards `to` by at most `max_angle_rad` (shortest rotation).
math::Quat slew_toward(const math::Quat& from, const math::Quat& to, double max_angle_rad);

}  // namespace starsim::gnc
