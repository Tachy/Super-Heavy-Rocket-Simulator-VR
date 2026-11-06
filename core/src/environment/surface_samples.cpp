#include "starsim/environment/surface_samples.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace starsim::environment {

SurfaceSamples sample_surface(const Terrain& terrain, std::span<const double> latitude_rad,
                              std::span<const double> longitude_rad) {
    if (latitude_rad.size() != longitude_rad.size()) {
        throw std::invalid_argument("surface samples: latitudes and longitudes must have the same count");
    }
    SurfaceSamples samples;
    samples.height_m.reserve(latitude_rad.size());
    samples.water.reserve(latitude_rad.size());
    for (size_t i = 0; i < latitude_rad.size(); ++i) {
        samples.height_m.push_back(terrain.height_m(latitude_rad[i], longitude_rad[i]));
        samples.water.push_back(terrain.water(latitude_rad[i], longitude_rad[i]) ? 1 : 0);
    }
    return samples;
}

double finest_resolution_m(const Terrain& terrain, std::span<const double> latitude_rad,
                           std::span<const double> longitude_rad) {
    if (latitude_rad.size() != longitude_rad.size()) {
        throw std::invalid_argument("surface resolution: latitudes and longitudes must have the same count");
    }
    double finest = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < latitude_rad.size(); ++i) {
        finest = std::min(finest, terrain.resolution_m(latitude_rad[i], longitude_rad[i]));
    }
    return finest;
}

}  // namespace starsim::environment
