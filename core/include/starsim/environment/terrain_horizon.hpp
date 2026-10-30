#pragma once

#include <vector>

#include "starsim/environment/terrain.hpp"

namespace starsim::environment {

// The sampling of a horizon march along a great circle: the first terrain sample lies min_distance_m (> 0) from the
// observer, the following steps start at first_step_m and grow geometrically (constant angular resolution as seen from
// the observer, a few % of the distance).
struct HorizonMarch {
    double min_distance_m = 0.0;
    double first_step_m = 0.0;
};

// Terrain horizon elevation (rad, + above the local horizontal of the point's sphere direction) seen from a point at
// (latitude_rad, longitude_rad) at height_m above the terrain's reference sphere, along the great circle in
// azimuth_rad (clockwise from north). Samples terrain from min_distance_m outwards until the relief bound
// (Terrain::max_height_m) cannot beat the running maximum any more, or the antipode. Thread-safe (reads only).
// The latitude / longitude are the terrain's own (Moon: planetocentric on the sphere); at a pole the longitude fixes
// what north and east mean.
double horizon_elevation_rad(const Terrain& terrain, double sphere_radius_m, double latitude_rad, double longitude_rad,
                             double height_m, double azimuth_rad, const HorizonMarch& march);

// Many points, azimuth_count azimuths k * 2 pi / azimuth_count each; result point-major, azimuth-minor. Serial and free
// of shared mutable state: several calls may run on different threads at once.
std::vector<float> horizon_elevations(const Terrain& terrain, double sphere_radius_m,
                                      const std::vector<double>& latitude_rad, const std::vector<double>& longitude_rad,
                                      const std::vector<double>& height_m, int azimuth_count, const HorizonMarch& march);

}  // namespace starsim::environment
