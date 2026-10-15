#pragma once

#include <filesystem>
#include <memory>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/environment/lunar_terrain.hpp"

namespace starsim::sim {

// The Moon's terrain from the data pipeline's files (data/terrain): the global grid LDEM_16 and the tile of every site
// in config/lunar_sites.toml. Throws (German, names tools/data_pipeline/fetch.py) when a file is missing.
std::shared_ptr<const environment::LunarTerrain> load_lunar_terrain(const std::filesystem::path& config_dir,
                                                                     const std::filesystem::path& data_dir);

// A landing site with the terrain height there (above the 1737.4 km sphere).
struct LunarSite {
    config::LunarSiteConfig config;
    double height_m = 0.0;
};
LunarSite lunar_site(const config::LunarSiteConfig& site, const environment::LunarTerrain& terrain);

}  // namespace starsim::sim
