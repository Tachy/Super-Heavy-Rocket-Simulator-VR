#include "starsim/sim/lunar_site.hpp"

#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "starsim/config/data_missing.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/report/report.hpp"

namespace starsim::sim {

namespace {

environment::HeightGrid grid_of(config::TerrainGridFile& f) { return {f.lines, f.samples, f.dn_scale_m, std::move(f.dn)}; }

}  // namespace

std::shared_ptr<const environment::LunarTerrain> load_lunar_terrain(const std::filesystem::path& config_dir,
                                                                     const std::filesystem::path& data_dir) {
    const auto global_name = config::load_lunar_global_grid_name(config_dir);
    auto global_file = config::load_terrain_grid_file(data_dir / "terrain" / (global_name + ".toml"));
    if (global_file.projection != "simple_cylindrical") {
        throw std::runtime_error("lunar terrain: " + global_name + " is not a cylindrical grid");
    }
    environment::CylindricalGrid global;
    global.pixels_per_degree = global_file.pixels_per_degree;
    global.zero_line = global_file.zero_line;
    global.zero_sample = global_file.zero_sample;
    const double radius = global_file.sphere_radius_m;
    global.grid = grid_of(global_file);
    std::vector<std::shared_ptr<const environment::LunarTile>> tiles;
    for (const auto& site : config::load_lunar_sites(config_dir)) {
        const auto path = data_dir / "terrain" / "sites" / (site.height_tile_key + ".toml");
        if (!std::filesystem::exists(path)) {
            throw config::DataMissing(report::make(report::Code::data_lunar_terrain_missing),
                                      "lunar terrain: the tile of site " + site.id + " is missing (" + path.string() +
                                          ") - run python tools/data_pipeline/fetch.py");
        }
        auto f = config::load_terrain_grid_file(path);
        // A site moved or resized after the download has another cache key and finds no tile; a tile that does not
        // centre on its site anyway (1 px) is refused.
        const double half_lines = 0.5 * (f.lines - 1), half_samples = 0.5 * (f.samples - 1);
        double line = 0.0, sample = 0.0, pixel_m = 0.0;
        std::shared_ptr<const environment::LunarTile> tile;
        if (f.projection == "polar_stereographic_south" || f.projection == "polar_stereographic_north") {
            environment::PolarStereographicTile t;
            t.north = f.projection == "polar_stereographic_north";
            t.sphere_radius_m = f.sphere_radius_m;
            t.scale_m = f.scale_m;
            t.pole_line = f.pole_line;
            t.pole_sample = f.pole_sample;
            t.first_line = f.first_line;
            t.first_sample = f.first_sample;
            t.grid = grid_of(f);
            std::tie(line, sample) = t.position(site.latitude_rad, site.longitude_rad);
            pixel_m = f.scale_m;
            const bool north = t.north;
            tile = environment::make_polar_tile(std::move(t), north);
        } else if (f.projection == "simple_cylindrical") {
            // Task 3 writes zero_line = ppd lat(0, 0), zero_sample = -ppd lon(0, 0) (pixel centres).
            const double first_lat_rad = f.zero_line / f.pixels_per_degree * constants::deg_to_rad;
            const double first_lon_rad = -f.zero_sample / f.pixels_per_degree * constants::deg_to_rad;
            const double dlon_deg = std::remainder((site.longitude_rad - first_lon_rad) * constants::rad_to_deg, 360.0);
            line = (first_lat_rad - site.latitude_rad) * constants::rad_to_deg * f.pixels_per_degree;
            sample = dlon_deg * f.pixels_per_degree;
            pixel_m = f.sphere_radius_m * constants::deg_to_rad / f.pixels_per_degree;
            environment::CylindricalGrid t;
            t.pixels_per_degree = f.pixels_per_degree;
            t.zero_line = f.zero_line;
            t.zero_sample = f.zero_sample;
            const double radius_m = f.sphere_radius_m;
            t.grid = grid_of(f);
            tile = environment::make_cylindrical_tile(std::move(t), first_lat_rad, first_lon_rad, radius_m);
        } else {
            throw std::runtime_error("lunar terrain: tile " + site.id + " has the unknown projection " + f.projection);
        }
        if (std::abs(line - half_lines) > 1.0 || std::abs(sample - half_samples) > 1.0 ||
            std::abs(half_lines * pixel_m - site.tile_half_size_m) > pixel_m) {
            throw config::ConfigError("lunar terrain: tile " + site.id +
                                      " does not match lunar_sites.toml - run python tools/data_pipeline/fetch.py again");
        }
        tiles.push_back(std::move(tile));
    }
    return std::make_shared<const environment::LunarTerrain>(std::move(global), std::move(tiles), radius);
}

LunarSite lunar_site(const config::LunarSiteConfig& site, const environment::LunarTerrain& terrain) {
    return {site, terrain.height_m(site.latitude_rad, site.longitude_rad)};
}

}  // namespace starsim::sim
