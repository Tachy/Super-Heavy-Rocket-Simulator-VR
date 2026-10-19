#pragma once

#include "starsim/math/state_vector.hpp"

namespace starsim::mission {

// ISS approach ellipsoid, semi-axes in the station's LVLH frame (x V-bar, y H-bar, z R-bar) [S: NASA, "approach
// ellipsoid 4 x 2 x 2 km" centred on the station's centre of mass], and the keep-out sphere [S: NASA, 200 m].
inline constexpr double approach_ellipsoid_semi_axes_m[3] = {2000.0, 1000.0, 1000.0};
inline constexpr double keep_out_sphere_radius_m = 200.0;

// A point `behind_m` along track behind the target (arc length on the target's radius: the target position turned
// about its orbit normal by -behind_m / r) and `below_m` below the target's radius.
math::Vec3 point_behind_below(const math::StateVector& target, double behind_m, double below_m);

// (x/a)^2 + (y/b)^2 + (z/c)^2 of the chaser in the target's LVLH frame; < 1: inside the approach ellipsoid.
double approach_ellipsoid_metric(const math::StateVector& target, const math::Vec3& chaser_position_m);

// Angle of the position's projection onto the target orbit plane, from the target position about the orbit normal,
// in (-pi, pi]; > 0 ahead of the target.
double in_plane_angle_rad(const math::StateVector& target, const math::Vec3& position_m);

}  // namespace starsim::mission
