#include "starsim/environment/weather_snapshot_build.hpp"

#include <cstddef>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "starsim/config/geoid_config.hpp"
#include "starsim/weather_decode/weather_decode.hpp"

namespace starsim::environment {

namespace {

using weather_decode::FloatGrid;

constexpr int grid_rows = 721, grid_columns = 1440;  // 0.25 deg, 90 N .. 90 S, 0 .. 359.75 E

std::vector<std::byte> read_file(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + p.string());
    std::vector<char> chars{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    std::vector<std::byte> bytes(chars.size());
    for (std::size_t i = 0; i < chars.size(); ++i) bytes[i] = static_cast<std::byte>(chars[i]);
    return bytes;
}

FloatGrid era5_file(const std::filesystem::path& file, int levels) {
    return weather_decode::decode_zarr_blosc_chunk(read_file(file), levels, grid_rows, grid_columns);
}

FloatGrid era5_grid(const std::filesystem::path& dir, const std::string& key, const char* variable, int levels) {
    return era5_file(dir / era5_file_name(key, variable), levels);
}

FloatGrid gfs_record(const std::filesystem::path& file) {
    const auto field = weather_decode::decode_grib2_message(read_file(file));
    if (field.grid.rows != grid_rows || field.grid.columns != grid_columns || field.first_latitude_deg != 90.0 ||
        field.first_longitude_deg != 0.0) {
        throw std::runtime_error("unexpected GFS grid in " + file.string());
    }
    return field.grid;
}

FloatGrid gfs_levels(const std::filesystem::path& dir, const std::string& key, const char* variable) {
    FloatGrid out;
    out.levels = static_cast<int>(gfs_levels_mb.size());
    out.rows = grid_rows;
    out.columns = grid_columns;
    out.values.reserve(static_cast<std::size_t>(out.levels) * grid_rows * grid_columns);
    for (int mb : gfs_levels_mb) {
        const FloatGrid g = gfs_record(dir / gfs_level_file_name(key, variable, mb));
        out.values.insert(out.values.end(), g.values.begin(), g.values.end());
    }
    return out;
}

}  // namespace

WeatherSnapshot build_weather_snapshot_from_raw(const WeatherSnapshotId& id, const std::filesystem::path& raw_dir,
                                                const std::filesystem::path& config_dir, int downsample) {
    if (id.kind == WeatherSourceKind::off) throw std::runtime_error("no weather snapshot to build for kind off");
    const std::string key = id.cache_key();
    const auto geoid = config::load_coarse_geoid(config_dir);
    std::optional<WeatherSnapshot> snapshot;
    if (id.kind == WeatherSourceKind::gfs) {
        GfsFields f;
        for (int mb : gfs_levels_mb) f.levels_pa.push_back(mb * 100.0);
        f.cloud_percent = gfs_levels(raw_dir, key, "TCDC");
        f.liquid_kg_kg = gfs_levels(raw_dir, key, "CLMR");
        f.ice_kg_kg = gfs_levels(raw_dir, key, "ICMR");
        f.temperature_k = gfs_levels(raw_dir, key, "TMP");
        f.geopotential_height_m = gfs_levels(raw_dir, key, "HGT");
        f.surface_geopotential_height_m = gfs_record(raw_dir / gfs_surface_height_file_name(key));
        snapshot = WeatherSnapshot::from_gfs(id, f, geoid);
    } else {
        Era5Fields f;
        const int levels = static_cast<int>(era5_levels_hpa.size());
        for (int hpa : era5_levels_hpa) f.levels_pa.push_back(hpa * 100.0);
        f.cloud_fraction = era5_grid(raw_dir, key, "fraction_of_cloud_cover", levels);
        f.liquid_kg_kg = era5_grid(raw_dir, key, "specific_cloud_liquid_water_content", levels);
        f.ice_kg_kg = era5_grid(raw_dir, key, "specific_cloud_ice_water_content", levels);
        f.temperature_k = era5_grid(raw_dir, key, "temperature", levels);
        f.surface_pressure_pa = era5_grid(raw_dir, key, "surface_pressure", 1);
        f.total_column_water_vapour_kg_m2 = era5_grid(raw_dir, key, "total_column_water_vapour", 1);
        f.surface_geopotential_m2_s2 = era5_file(raw_dir / era5_static_orography_file_name, 1);
        snapshot = WeatherSnapshot::from_era5(id, f, geoid);
    }
    if (downsample > 1) return snapshot->downsampled(downsample);
    return std::move(*snapshot);
}

}  // namespace starsim::environment
