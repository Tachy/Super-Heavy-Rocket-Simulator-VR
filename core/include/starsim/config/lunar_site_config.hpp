#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace starsim::config {

// A lunar landing site (config/lunar_sites.toml; MOON_ME latitude / longitude). The data pipeline crops a LOLA tile
// around it into data/terrain/<id>.toml.
struct LunarSiteConfig {
    std::string id;
    std::string name;     // German (UI)
    std::string product;  // LOLA product the tile is cropped from
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double tile_half_size_m = 0.0;
};

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
    std::string projection;  // "polar_stereographic_south", "simple_cylindrical" or "geographic"
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
    std::vector<TerrainCheck> checks;
};

// Throws ConfigError (German, names tools/data_pipeline/fetch.py) when the file or its heights are missing.
TerrainGridFile load_terrain_grid_file(const std::filesystem::path& toml_path);

}  // namespace starsim::config
