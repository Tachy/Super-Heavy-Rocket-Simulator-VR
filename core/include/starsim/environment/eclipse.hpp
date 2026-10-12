#pragma once

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Fraction (0..1) of a light source's disk hidden by an occluding sphere, as seen by an observer.
// Inputs are vectors FROM the observer (use apparent positions, see Ephemeris::apparent_position_gcrf).
// Disk-overlap model of Montenbruck & Gill, "Satellite Orbits", eq. 3.85-3.90.
// Used for solar eclipses on the ground and for spacecraft shadow (solar radiation pressure, power).
double occulted_fraction(const math::Vec3& to_source_m, double source_radius_m, const math::Vec3& to_occluder_m,
                         double occluder_radius_m);

}  // namespace starsim::environment
