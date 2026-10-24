#include "starsim/environment/surface_samples.hpp"

#include <stdexcept>

namespace starsim::environment {

SurfaceSamples sample_surface(const Terrain& terrain, std::span<const double> latitude_rad,
                              std::span<const double> longitude_rad) {
    if (latitude_rad.size() != longitude_rad.size()) {
        throw std::invalid_argument("Geländeproben: Breiten und Längen müssen gleich viele sein");
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

}  // namespace starsim::environment
