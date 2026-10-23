#pragma once

#include "starsim/environment/height_grid.hpp"

namespace starsim::environment {

// Node-registered latitude / longitude grid (EGM2008, Earth DEM tiles): line 0 at north_latitude_deg, sample 0 at
// west_longitude_deg, lines go south, samples go east. global_longitude: the samples span 360 deg and wrap (sample
// `samples` is sample 0 again; no duplicated +180 column).
struct GeographicGrid {
    HeightGrid grid;
    double north_latitude_deg = 0.0;
    double west_longitude_deg = 0.0;
    double step_latitude_deg = 0.0;
    double step_longitude_deg = 0.0;
    bool global_longitude = false;

    // Inside the nodes' extent (a global grid: every longitude; latitude between the first and last line).
    bool covers(double latitude_rad, double longitude_rad) const;
    // Bilinear; latitude clamped to the first / last line, longitude wrapped when global (clamped to the extent else).
    double value(double latitude_rad, double longitude_rad) const;
    // Metres to the nearest edge of the extent on a sphere of radius_m (a global grid: the latitude edges only);
    // >= 0 inside, negative outside.
    double edge_distance_m(double latitude_rad, double longitude_rad, double radius_m) const;
};

}  // namespace starsim::environment
