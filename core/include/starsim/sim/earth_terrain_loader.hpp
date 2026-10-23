#pragma once

#include <filesystem>
#include <memory>

#include "starsim/environment/earth_terrain.hpp"

namespace starsim::sim {

// The Earth's terrain from the data pipeline's files: data/geoid/egm2008_25 (the fine geoid),
// data/terrain/earth/global_relief and the <area>_regional / <area>_local tiles of every area in
// config/earth_terrain.toml, with a plateau for every site of config/launch_sites.toml (its height_m).
// Throws config::ConfigError (German, names tools/data_pipeline/fetch.py) when a file is missing or stale: a tile's
// area, level, centre (by more than one cell) or half size (by more than one cell) differs from earth_terrain.toml.
std::shared_ptr<const environment::EarthTerrain> load_earth_terrain(const std::filesystem::path& config_dir,
                                                                    const std::filesystem::path& data_dir);

// Whether the downloaded Earth terrain is there (the fine geoid and the global relief; the area tiles are checked by
// load_earth_terrain).
bool earth_terrain_available(const std::filesystem::path& data_dir);

}  // namespace starsim::sim
