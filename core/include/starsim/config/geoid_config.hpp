#pragma once

#include <filesystem>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/environment/geoid.hpp"

namespace starsim::config {

// The committed 1-degree EGM2008 grid (config/geoid/egm2008_1deg.csv; 181 x 361 values in cm, written by
// tools/data_pipeline/geoid.py). The duplicated +180 column is dropped: the grid wraps. Throws ConfigError.
environment::Geoid load_coarse_geoid(const std::filesystem::path& config_dir);

// A geographic grid file (TerrainGridFile with projection "geographic") as a grid; throws ConfigError otherwise.
environment::GeographicGrid geographic_grid(const TerrainGridFile& file);
environment::GeographicGrid geographic_grid(TerrainGridFile&& file);  // moves the heights (large Earth tiles)

}  // namespace starsim::config
