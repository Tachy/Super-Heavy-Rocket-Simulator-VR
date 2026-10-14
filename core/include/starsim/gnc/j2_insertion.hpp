#pragma once

#include <vector>

#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// State at orbit insertion in the orbit's own frame: radius, radial and horizontal (in-plane) speed.
struct InsertionState {
    double radius_m = 0.0;
    double radial_speed_mps = 0.0;
    double horizontal_speed_mps = 0.0;
};

// Insertion into a circular orbit of `radius_m` in a point-mass + J2 field: the state at argument of latitude
// `arg_latitude_rad` (from the ascending node, orbit inclined by `inclination_rad` to the equator) whose next revolution
// stays closest to the radius (least squares over the revolution). A Kepler-circular insertion leaves an eccentricity
// that J2 excites: ~10 km of radius range at 200 km. With j2 = 0: the Kepler circular state.
InsertionState j2_circular_insertion(double gm_m3ps2, double radius_m, double inclination_rad, double arg_latitude_rad,
                                     double j2 = constants::earth_j2,
                                     double equatorial_radius_m = constants::wgs84_semi_major_axis_m);

// Radius over one revolution from `state` in that field (`samples` evenly in time, one Kepler period).
std::vector<double> j2_radius_profile(double gm_m3ps2, const InsertionState& state, double inclination_rad,
                                      double arg_latitude_rad, int samples, double j2 = constants::earth_j2,
                                      double equatorial_radius_m = constants::wgs84_semi_major_axis_m);

}  // namespace starsim::gnc
