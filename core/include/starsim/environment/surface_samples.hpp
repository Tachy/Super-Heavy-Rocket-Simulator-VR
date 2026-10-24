#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "starsim/environment/terrain.hpp"

namespace starsim::environment {

// Heights and water flags of a terrain at many latitude / longitude points (the terrain renderer's tile meshes).
struct SurfaceSamples {
    std::vector<double> height_m;   // above the shape (Earth: WGS84 ellipsoid)
    std::vector<std::uint8_t> water;  // 1 = water
};

// Pointwise Terrain::height_m / Terrain::water. The spans must have equal sizes (std::invalid_argument otherwise).
// Reads the immutable terrain only: callable from several threads at once.
SurfaceSamples sample_surface(const Terrain& terrain, std::span<const double> latitude_rad,
                              std::span<const double> longitude_rad);

// The finest Terrain::resolution_m at the given points (m): a terrain renderer node sampling its whole lattice finds a
// site tile that covers only part of it. Infinity without points. The spans must have equal sizes
// (std::invalid_argument otherwise). Thread-safe like sample_surface.
double finest_resolution_m(const Terrain& terrain, std::span<const double> latitude_rad,
                           std::span<const double> longitude_rad);

}  // namespace starsim::environment
