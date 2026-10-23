#include "starsim/sim/earth_terrain_loader.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "starsim/config/earth_terrain_config.hpp"
#include "starsim/config/geoid_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::sim {

namespace {

std::filesystem::path geoid_file(const std::filesystem::path& data_dir) { return data_dir / "geoid" / "egm2008_25.toml"; }
std::filesystem::path earth_dir(const std::filesystem::path& data_dir) { return data_dir / "terrain" / "earth"; }

config::TerrainGridFile load_geographic(const std::filesystem::path& toml, const std::string& height_reference) {
    auto f = config::load_terrain_grid_file(toml);
    if (f.projection != "geographic" || f.height_reference != height_reference) {
        throw config::ConfigError("Erdgelände: " + toml.string() + " ist kein geografisches Gitter mit " + height_reference +
                                  " - bitte python tools/data_pipeline/fetch.py ausführen");
    }
    return f;
}

[[noreturn]] void stale(const std::string& tile, const std::string& what) {
    throw config::ConfigError("Erdgelände: Kachel " + tile + " passt nicht mehr zu earth_terrain.toml (" + what +
                              ") - bitte python tools/data_pipeline/fetch.py erneut ausführen");
}

// A tile cut for another area, level, centre or size than the area now has in config/earth_terrain.toml would be
// used silently; one cell of slack (the pipeline snaps the box to the source grid).
environment::GeographicGrid area_tile(const std::filesystem::path& data_dir, const config::EarthTerrainArea& area,
                                      const std::string& level, double half_size_m) {
    const std::string name = area.id + "_" + level;
    auto f = load_geographic(earth_dir(data_dir) / (name + ".toml"), "wgs84_ellipsoid");
    const double rad_to_deg = 1.0 / constants::deg_to_rad;
    const double cell_m = f.step_latitude_deg * constants::deg_to_rad * f.sphere_radius_m;
    if (f.area != area.id) stale(name, "Gebiet '" + f.area + "'");
    if (f.level != level) stale(name, "Ebene '" + f.level + "'");
    if (std::abs(f.area_latitude_deg - area.latitude_rad * rad_to_deg) > f.step_latitude_deg ||
        std::abs(f.area_longitude_deg - area.longitude_rad * rad_to_deg) > f.step_longitude_deg) {
        stale(name, "Mittelpunkt verschoben");
    }
    if (std::abs(f.half_size_m - half_size_m) > cell_m) stale(name, "Größe geändert");
    return config::geographic_grid(std::move(f));
}

}  // namespace

bool earth_terrain_available(const std::filesystem::path& data_dir) {
    return std::filesystem::exists(geoid_file(data_dir)) &&
           std::filesystem::exists(earth_dir(data_dir) / "global_relief.toml");
}

std::shared_ptr<const environment::EarthTerrain> load_earth_terrain(const std::filesystem::path& config_dir,
                                                                    const std::filesystem::path& data_dir) {
    environment::Geoid geoid(config::geographic_grid(load_geographic(geoid_file(data_dir), "egm2008_undulation")));
    auto relief = config::geographic_grid(load_geographic(earth_dir(data_dir) / "global_relief.toml", "wgs84_ellipsoid"));
    std::vector<environment::GeographicGrid> regional, local;
    for (const auto& area : config::load_earth_terrain_areas(config_dir)) {
        regional.push_back(area_tile(data_dir, area, "regional", area.regional_half_size_m));
        local.push_back(area_tile(data_dir, area, "local", area.local_half_size_m));
    }
    std::vector<environment::Plateau> plateaus;
    for (const auto& site : config::load_launch_sites(config_dir)) {
        plateaus.push_back({site.id, site.latitude_rad, site.longitude_rad, site.height_m, site.plateau_radius_m,
                            site.plateau_apron_m});
    }
    return std::make_shared<const environment::EarthTerrain>(std::move(geoid), std::move(relief), std::move(regional),
                                                             std::move(local), std::move(plateaus));
}

}  // namespace starsim::sim
