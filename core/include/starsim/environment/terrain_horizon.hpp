#pragma once

#include <algorithm>
#include <vector>

#include "starsim/environment/terrain.hpp"

namespace starsim::environment {

// The sampling of a horizon march along a great circle: the first terrain sample lies min_distance_m (> 0) from the
// observer, the following steps start at first_step_m and grow geometrically (constant angular resolution as seen from
// the observer, a few % of the distance).
// The march ends at max_distance_m when that is > 0 (a distance band), else when the relief bound cannot beat the
// running maximum any more. With sample_from_m / max_distance_m one march sequence splits into distance bands.
struct HorizonMarch {
    double min_distance_m = 0.0;
    double first_step_m = 0.0;
    double max_distance_m = 0.0;
    double sample_from_m = 0.0;  // samples before this distance are walked but not evaluated (a band of the sequence)
};

// Terrain horizon elevation (rad, + above the local horizontal of the point's sphere direction) seen from a point at
// (latitude_rad, longitude_rad) at height_m above the terrain's reference sphere, along the great circle in
// azimuth_rad (clockwise from north); throws std::invalid_argument for march.first_step_m <= 0. Samples terrain from
// min_distance_m outwards until the relief bound (Terrain::max_height_m) cannot beat the running maximum any more, or
// the antipode. Thread-safe (reads only).
// The latitude / longitude are the terrain's own (Moon: planetocentric on the sphere); at a pole the longitude fixes
// what north and east mean.
double horizon_elevation_rad(const Terrain& terrain, double sphere_radius_m, double latitude_rad, double longitude_rad,
                             double height_m, double azimuth_rad, const HorizonMarch& march);

// Many points, azimuth_count azimuths k * 2 pi / azimuth_count each; result point-major, azimuth-minor. Serial and free
// of shared mutable state: several calls may run on different threads at once.
std::vector<float> horizon_elevations(const Terrain& terrain, double sphere_radius_m,
                                      const std::vector<double>& latitude_rad, const std::vector<double>& longitude_rad,
                                      const std::vector<double>& height_m, int azimuth_count,
                                      const HorizonMarch& march);

// [E] The range around a march's start within which the finest data sets its first step: the march's step grows by
// 2 % of the distance, so it reaches half the coarse 1.9 km grid's resolution (950 m) at ~ 950 / 0.02 = 48 km; finer
// data (a 20 m site tile) further out than that is reached with steps that large anyway.
constexpr double first_step_data_range_m = 50e3;

// First step of a vertex's march: half the finest data resolution within first_step_data_range_m or the lattice
// spacing, whichever is larger (no use sampling a 1.9 km grid every 10 m, but a start just outside a 20 m tile must
// not step over the tile's relief: no jump of the horizon along the tile's outline). It depends on the vertex only
// (not on the node), so shared vertices agree.
inline double lattice_first_step_m(const Terrain& terrain, double latitude_rad, double longitude_rad,
                                   double spacing_m) {
    return std::max(0.5 * terrain.finest_resolution_within_m(latitude_rad, longitude_rad, first_step_data_range_m),
                    spacing_m);
}

// Horizons for an n x n vertex lattice (row-major, n - 1 divisible by 16, typically 65) of one terrain node whose
// vertices are spaced ~spacing_m apart (latitude_rad / longitude_rad / height_m per vertex, height above the sphere).
// Same output layout and meaning as horizon_elevations (vertex-major, azimuth-minor; azimuth from each vertex's own
// north); the march of a vertex is horizon_elevations' with HorizonMarch{spacing_m, lattice_first_step_m(...)}: the
// near terrain closer than one spacing is the relief normal's job. The samples are split into distance bands (the
// maximum of them per vertex / azimuth): every vertex from one spacing to 16 spacings, every 2nd vertex to 32, every
// 4th to 64, every 8th to 128, every 16th beyond; the sparse bands are bilinearly interpolated (the azimuth profile
// re-sampled where the local norths differ, as near the poles). Vertices shared by neighbouring nodes get identical
// values along the shared edge. Serial and thread-safe.
std::vector<float> horizon_elevations_lattice(const Terrain& terrain, double sphere_radius_m, int n,
                                              const std::vector<double>& latitude_rad,
                                              const std::vector<double>& longitude_rad,
                                              const std::vector<double>& height_m, double spacing_m,
                                              int azimuth_count);

}  // namespace starsim::environment
