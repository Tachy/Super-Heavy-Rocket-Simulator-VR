#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace starsim::config {

// A lunar landing site (config/lunar_sites.toml: id and MOON_ME latitude / longitude). The data pipeline cuts its
// terrain tile from the products of lunar_terrain_products.toml into data/terrain/sites/<height_tile_key>.toml.
struct LunarSiteConfig {
    std::string id;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double tile_half_size_m = 0.0;  // of the height tile: the product's (lunar_terrain_products.toml)
    std::string height_tile_key;    // cache key of the height tile (config/lunar_site_tile.hpp)
};

// Base name (in data/terrain) of the global lunar height grid: `[global] grid` of lunar_sites.toml ("ldem_64").
std::string load_lunar_global_grid_name(const std::filesystem::path& config_dir);

// A site is a place, not a list entry: any latitude / longitude (MOON_ME), id "site_<lat>_<lon>" at 0.01 deg
// ("site_-84.79_29.20"). Tile size and key resolve from the products exactly as for the presets (the product that holds
// the whole tile: polar 20 m products 10 km, ...). Throws report::Refusal lunar.site_no_product when none does.
// The products overload skips re-reading the config (config/lunar_site_tile.hpp).
struct LunarTerrainProduct;
LunarSiteConfig lunar_site_at(const std::filesystem::path& config_dir, double latitude_rad, double longitude_rad);
LunarSiteConfig lunar_site_at(const std::vector<LunarTerrainProduct>& products, double latitude_rad,
                              double longitude_rad);

LunarSiteConfig load_lunar_site(const std::filesystem::path& config_dir, std::string_view id);
std::vector<LunarSiteConfig> load_lunar_sites(const std::filesystem::path& config_dir);

// A height grid written by tools/data_pipeline/lunar_tile.py: TOML header + int16 heights (LE, row-major).
struct TerrainCheck {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;  // the pipeline's bilinear height from the raw DN
};
struct TerrainGridFile {
    std::string product;
    std::string kind = "height";  // "height" (DN x dn_scale_m = metres) or "albedo" (DN x dn_scale_m = relative albedo)
    std::string projection;  // "polar_stereographic_south" / "_north", "simple_cylindrical" or "geographic"
    double sphere_radius_m = 0.0;
    double dn_scale_m = 0.0;
    int lines = 0;
    int samples = 0;
    std::vector<std::int16_t> dn;
    // polar stereographic
    double pole_line = 0.0;
    double pole_sample = 0.0;
    double scale_m = 0.0;
    int first_line = 0;
    int first_sample = 0;
    // simple cylindrical
    double pixels_per_degree = 0.0;
    double zero_line = 0.0;
    double zero_sample = 0.0;
    // geographic (Earth: EGM2008, DEM tiles; sphere_radius_m is the WGS84 semi-major axis there)
    double north_latitude_deg = 0.0;
    double west_longitude_deg = 0.0;
    double step_latitude_deg = 0.0;
    double step_longitude_deg = 0.0;
    bool global_longitude = false;
    std::string height_reference;  // "egm2008_undulation" or "wgs84_ellipsoid"
    // Earth DEM tiles (tools/data_pipeline/earth_terrain.py; empty / 0 where the header has none): the area of
    // config/earth_terrain.toml the tile was cut for, its level ("global", "regional", "local"), centre and half size.
    std::string area;
    std::string level;
    double area_latitude_deg = 0.0;
    double area_longitude_deg = 0.0;
    double half_size_m = 0.0;
    std::vector<TerrainCheck> checks;
};

// Throws ConfigError (German, names tools/data_pipeline/fetch.py) when the file or its heights are missing.
TerrainGridFile load_terrain_grid_file(const std::filesystem::path& toml_path);

// Writes the TOML header and, next to it, the int16 grid "<toml stem>.i16" (the format load_terrain_grid_file reads).
// Polar stereographic and simple cylindrical grids only; the checks become [[check]] tables.
void write_terrain_grid_file(const TerrainGridFile& file, const std::filesystem::path& toml_path);

}  // namespace starsim::config
