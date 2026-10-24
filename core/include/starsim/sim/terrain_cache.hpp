#pragma once

#include <filesystem>
#include <memory>

#include "starsim/environment/earth_terrain.hpp"
#include "starsim/environment/lunar_terrain.hpp"

namespace starsim::sim {

// Process-wide terrain instances: one per (canonical config_dir, data_dir) while anyone holds it (weak cache under a
// mutex), so every FlightSim.setup, mission and the terrain renderer share one copy of the grids instead of loading a
// private ~770 MB one each. Loader errors (ConfigError ...) propagate and are not cached.
std::shared_ptr<const environment::EarthTerrain> shared_earth_terrain(const std::filesystem::path& config_dir,
                                                                      const std::filesystem::path& data_dir);
std::shared_ptr<const environment::LunarTerrain> shared_lunar_terrain(const std::filesystem::path& config_dir,
                                                                      const std::filesystem::path& data_dir);

}  // namespace starsim::sim
