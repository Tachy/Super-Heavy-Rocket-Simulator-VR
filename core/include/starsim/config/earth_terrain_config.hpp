#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace starsim::config {

// An area of config/earth_terrain.toml: the data pipeline cuts a local and a regional height tile around its centre.
struct EarthTerrainArea {
    std::string id;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double local_half_size_m = 0.0;
    double regional_half_size_m = 0.0;
};

// The areas of config/earth_terrain.toml, in file order. Throws ConfigError.
std::vector<EarthTerrainArea> load_earth_terrain_areas(const std::filesystem::path& config_dir);

}  // namespace starsim::config
