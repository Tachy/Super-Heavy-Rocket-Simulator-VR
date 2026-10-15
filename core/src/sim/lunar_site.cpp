#include "starsim/sim/lunar_site.hpp"

#include <stdexcept>
#include <utility>

namespace starsim::sim {

namespace {

environment::HeightGrid grid_of(config::TerrainGridFile& f) { return {f.lines, f.samples, f.dn_scale_m, std::move(f.dn)}; }

}  // namespace

std::shared_ptr<const environment::LunarTerrain> load_lunar_terrain(const std::filesystem::path& config_dir,
                                                                     const std::filesystem::path& data_dir) {
    auto global_file = config::load_terrain_grid_file(data_dir / "terrain" / "ldem_16.toml");
    if (global_file.projection != "simple_cylindrical") {
        throw std::runtime_error("Mondgelände: ldem_16 ist kein zylindrisches Gitter");
    }
    environment::CylindricalGrid global;
    global.pixels_per_degree = global_file.pixels_per_degree;
    global.zero_line = global_file.zero_line;
    global.zero_sample = global_file.zero_sample;
    const double radius = global_file.sphere_radius_m;
    global.grid = grid_of(global_file);
    std::vector<environment::PolarStereographicTile> tiles;
    for (const auto& site : config::load_lunar_sites(config_dir)) {
        auto f = config::load_terrain_grid_file(data_dir / "terrain" / (site.id + ".toml"));
        if (f.projection != "polar_stereographic_south") {
            throw std::runtime_error("Mondgelände: Kachel " + site.id + " ist nicht polarstereografisch (Süd)");
        }
        environment::PolarStereographicTile t;
        t.sphere_radius_m = f.sphere_radius_m;
        t.scale_m = f.scale_m;
        t.pole_line = f.pole_line;
        t.pole_sample = f.pole_sample;
        t.first_line = f.first_line;
        t.first_sample = f.first_sample;
        t.grid = grid_of(f);
        tiles.push_back(std::move(t));
    }
    return std::make_shared<const environment::LunarTerrain>(std::move(global), std::move(tiles), radius);
}

LunarSite lunar_site(const config::LunarSiteConfig& site, const environment::LunarTerrain& terrain) {
    return {site, terrain.height_m(site.latitude_rad, site.longitude_rad)};
}

}  // namespace starsim::sim
