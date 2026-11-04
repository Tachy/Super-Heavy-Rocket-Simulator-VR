#pragma once

#include <filesystem>
#include <memory>

#include "starsim/environment/detailed_lunar_terrain.hpp"
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
// The Moon's composed physics surface (LOLA + procedural detail + rocks, per the fidelity flags), cached the same way
// per (directories, options): the session, the planners and the renderer share one instance and its contact cache.
// The parts are shared too: the LOLA terrain (shared_lunar_terrain), the detail generator (config_dir /
// moon_surface.toml) and the rocks with the abundance map (data_dir / terrain; loaded only with rocks on). Returned as
// the DetailedLunarTerrain (an environment::Terrain) so a session can rebuild it with other flags from the same parts.
std::shared_ptr<const environment::DetailedLunarTerrain> shared_moon_surface(const std::filesystem::path& config_dir,
                                                                            const std::filesystem::path& data_dir,
                                                                            environment::SurfaceDetailOptions options);

}  // namespace starsim::sim
