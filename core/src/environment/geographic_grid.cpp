#include "starsim/environment/geographic_grid.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr double rad_to_deg = 1.0 / constants::deg_to_rad;

// Degrees east of the west edge in [0, 360).
double east_of_west_deg(double longitude_rad, double west_deg) {
    const double d = std::fmod(longitude_rad * rad_to_deg - west_deg, 360.0);
    return d < 0.0 ? d + 360.0 : d;
}

}  // namespace

bool GeographicGrid::covers(double latitude_rad, double longitude_rad) const {
    const double lat = latitude_rad * rad_to_deg;
    const double south = north_latitude_deg - step_latitude_deg * (grid.lines - 1);
    if (lat > north_latitude_deg || lat < south) return false;
    if (global_longitude) return true;
    return east_of_west_deg(longitude_rad, west_longitude_deg) <= step_longitude_deg * (grid.samples - 1);
}

double GeographicGrid::value(double latitude_rad, double longitude_rad) const {
    const double line =
        std::clamp((north_latitude_deg - latitude_rad * rad_to_deg) / step_latitude_deg, 0.0, grid.lines - 1.0);
    const double east = east_of_west_deg(longitude_rad, west_longitude_deg) / step_longitude_deg;
    if (!global_longitude) return grid.bilinear(line, std::min(east, grid.samples - 1.0));
    const int l0 = std::min(static_cast<int>(std::floor(line)), grid.lines - 2);
    const double fl = line - l0;
    const int n = grid.samples;
    const int s0 = std::min(static_cast<int>(std::floor(east)), n - 1);  // east < n up to rounding
    const double fs = east - s0;
    const int s1 = (s0 + 1) % n;
    return (1.0 - fl) * ((1.0 - fs) * grid.at(l0, s0) + fs * grid.at(l0, s1)) +
           fl * ((1.0 - fs) * grid.at(l0 + 1, s0) + fs * grid.at(l0 + 1, s1));
}

double GeographicGrid::edge_distance_m(double latitude_rad, double longitude_rad, double radius_m) const {
    const double lat = latitude_rad * rad_to_deg;
    const double south = north_latitude_deg - step_latitude_deg * (grid.lines - 1);
    double d = radius_m * constants::deg_to_rad * std::min(north_latitude_deg - lat, lat - south);
    if (!global_longitude) {
        const double east = east_of_west_deg(longitude_rad, west_longitude_deg);
        const double span = step_longitude_deg * (grid.samples - 1);
        // Outside the extent the nearer edge gives the (negative) margin.
        const double margin_deg = east <= span ? std::min(east, span - east) : -std::min(east - span, 360.0 - east);
        d = std::min(d, radius_m * constants::deg_to_rad * margin_deg * std::cos(latitude_rad));
    }
    return d;
}

}  // namespace starsim::environment
