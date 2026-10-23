#pragma once

#include <utility>

#include "starsim/environment/geographic_grid.hpp"

namespace starsim::environment {

// EGM2008 geoid: the undulation N of mean sea level above the WGS84 ellipsoid (h_ellipsoid = H_sea_level + N).
class Geoid {
public:
    explicit Geoid(GeographicGrid undulation) : grid_(std::move(undulation)) {}

    double undulation_m(double latitude_rad, double longitude_rad) const;
    const GeographicGrid& grid() const { return grid_; }

private:
    GeographicGrid grid_;
};

}  // namespace starsim::environment
