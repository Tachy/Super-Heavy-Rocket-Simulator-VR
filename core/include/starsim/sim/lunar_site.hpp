#pragma once

#include <filesystem>
#include <memory>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/environment/lunar_terrain.hpp"

namespace starsim::sim {

// The Moon's terrain from the data pipeline's files (data/terrain): the global grid alone (sessions without a landing
// site, orbit only) or with the height tile of the active site (data/terrain/sites/<cache key>) on top. Throws
// DataMissing: data.lunar_terrain_missing for the global grid, data.lunar_site_tile_missing(latitude, longitude) for
// the site's tile.
std::shared_ptr<const environment::LunarTerrain> load_lunar_terrain(const std::filesystem::path& config_dir,
                                                                     const std::filesystem::path& data_dir);
std::shared_ptr<const environment::LunarTerrain> load_lunar_terrain(const std::filesystem::path& config_dir,
                                                                     const std::filesystem::path& data_dir,
                                                                     const config::LunarSiteConfig& site);
// The global grid and the site's height tile are both present (nothing is loaded).
bool lunar_site_tiles_present(const std::filesystem::path& config_dir, const std::filesystem::path& data_dir,
                              const config::LunarSiteConfig& site);

// A landing site with the terrain height there (above the 1737.4 km sphere).
struct LunarSite {
    config::LunarSiteConfig config;
    double height_m = 0.0;
};
LunarSite lunar_site(const config::LunarSiteConfig& site, const environment::LunarTerrain& terrain);

}  // namespace starsim::sim
