// weather_snapshot (M8 weather and clouds): builds one weather snapshot cache file (.wthr) from the raw downloads.
// Usage:
//   weather_snapshot --era5|--gfs <raw dir> --id <cache key> [--config <config dir>] --out <file> [--downsample N]
// Raw files, named by weather_source.hpp (era5_file_name, gfs_level_file_name, ...) as the request planning saves them:
//   ERA5: <raw dir>/<key>_<variable>.blosc for fraction_of_cloud_cover, specific_cloud_liquid_water_content,
//         specific_cloud_ice_water_content, temperature (37 x 721 x 1440), surface_pressure,
//         total_column_water_vapour (721 x 1440) and the static era5_geopotential_at_surface.blosc; key "era5_<hour>".
//   GFS:  <raw dir>/<key>_<VAR>_<llll>mb.grb2 for TCDC, CLMR, ICMR, HGT, TMP on 1000..50 mb and <key>_HGT_surface.grb2;
//         key "gfs_<cycle hour>_f<lead>".
// Exit code 0 on success, 1 on an error.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "starsim/config/geoid_config.hpp"
#include "starsim/environment/weather_snapshot.hpp"
#include "starsim/environment/weather_snapshot_file.hpp"
#include "starsim/weather_decode/weather_decode.hpp"

namespace {

using namespace starsim;
using environment::WeatherSnapshotId;
using environment::WeatherSourceKind;
using weather_decode::FloatGrid;

constexpr int grid_rows = 721, grid_columns = 1440;  // 0.25 deg, 90 N .. 90 S, 0 .. 359.75 E
// The ARCO-ERA5 store's 37 pressure levels in its order (`level` array, gen_weather_fixtures.py era5_columns.toml).
const std::vector<int> era5_levels_hpa{1,   2,   3,   5,   7,   10,  20,  30,  50,  70,  100, 125, 150,
                                       175, 200, 225, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700,
                                       750, 775, 800, 825, 850, 875, 900, 925, 950, 975, 1000};
// GFS 0.25 deg pressure levels shared by the five variables (weather_source.cpp gfs_levels_mb).
const std::vector<int> gfs_levels_mb{1000, 975, 950, 925, 900, 850, 800, 750, 700, 650, 600,
                                     550,  500, 450, 400, 350, 300, 250, 200, 150, 100, 50};

std::vector<std::byte> read_file(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + p.string());
    std::vector<char> chars{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    std::vector<std::byte> bytes(chars.size());
    for (std::size_t i = 0; i < chars.size(); ++i) bytes[i] = static_cast<std::byte>(chars[i]);
    return bytes;
}

WeatherSnapshotId parse_key(const std::string& key) {
    WeatherSnapshotId id;
    if (key.rfind("era5_", 0) == 0) {
        id.kind = WeatherSourceKind::era5;
        id.valid_hour = std::stoll(key.substr(5));
        return id;
    }
    const auto f = key.find("_f");
    if (key.rfind("gfs_", 0) == 0 && f != std::string::npos) {
        id.kind = WeatherSourceKind::gfs;
        id.cycle_hour = std::stoll(key.substr(4, f - 4));
        id.lead_h = std::stoi(key.substr(f + 2));
        id.valid_hour = id.cycle_hour + id.lead_h;
        return id;
    }
    throw std::runtime_error("unknown cache key " + key);
}

FloatGrid era5_file(const std::filesystem::path& file, int levels) {
    return weather_decode::decode_zarr_blosc_chunk(read_file(file), levels, grid_rows, grid_columns);
}

FloatGrid era5_grid(const std::filesystem::path& dir, const std::string& key, const char* variable, int levels) {
    return era5_file(dir / environment::era5_file_name(key, variable), levels);
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
        const FloatGrid g = gfs_record(dir / environment::gfs_level_file_name(key, variable, mb));
        out.values.insert(out.values.end(), g.values.begin(), g.values.end());
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string source, key;
    std::filesystem::path raw_dir, out_file, config_dir = STARSIM_CONFIG_DIR;
    int downsample = 1;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--era5" || arg == "--gfs") && i + 1 < argc) {
            source = arg.substr(2);
            raw_dir = argv[++i];
        } else if (arg == "--id" && i + 1 < argc) {
            key = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            config_dir = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            out_file = argv[++i];
        } else if (arg == "--downsample" && i + 1 < argc) {
            downsample = std::stoi(argv[++i]);
        } else {
            source.clear();
            break;
        }
    }
    if (source.empty() || key.empty() || out_file.empty()) {
        std::cerr << "usage: weather_snapshot --era5|--gfs <raw dir> --id <cache key> [--config <dir>] --out <file> "
                     "[--downsample N]\n";
        return 1;
    }
    try {
        const auto t0 = std::chrono::steady_clock::now();
        const WeatherSnapshotId id = parse_key(key);
        const auto geoid = config::load_coarse_geoid(config_dir);
        std::optional<environment::WeatherSnapshot> snapshot;
        if (source == "era5") {
            if (id.kind != WeatherSourceKind::era5) throw std::runtime_error("--era5 needs an era5_<hour> key");
            environment::Era5Fields f;
            const int levels = static_cast<int>(era5_levels_hpa.size());
            for (int hpa : era5_levels_hpa) f.levels_pa.push_back(hpa * 100.0);
            f.cloud_fraction = era5_grid(raw_dir, key, "fraction_of_cloud_cover", levels);
            f.liquid_kg_kg = era5_grid(raw_dir, key, "specific_cloud_liquid_water_content", levels);
            f.ice_kg_kg = era5_grid(raw_dir, key, "specific_cloud_ice_water_content", levels);
            f.temperature_k = era5_grid(raw_dir, key, "temperature", levels);
            f.surface_pressure_pa = era5_grid(raw_dir, key, "surface_pressure", 1);
            f.total_column_water_vapour_kg_m2 = era5_grid(raw_dir, key, "total_column_water_vapour", 1);
            f.surface_geopotential_m2_s2 = era5_file(raw_dir / environment::era5_static_orography_file_name, 1);
            snapshot = environment::WeatherSnapshot::from_era5(id, f, geoid);
        } else {
            if (id.kind != WeatherSourceKind::gfs) throw std::runtime_error("--gfs needs a gfs_<cycle>_f<lead> key");
            environment::GfsFields f;
            for (int mb : gfs_levels_mb) f.levels_pa.push_back(mb * 100.0);
            f.cloud_percent = gfs_levels(raw_dir, key, "TCDC");
            f.liquid_kg_kg = gfs_levels(raw_dir, key, "CLMR");
            f.ice_kg_kg = gfs_levels(raw_dir, key, "ICMR");
            f.temperature_k = gfs_levels(raw_dir, key, "TMP");
            f.geopotential_height_m = gfs_levels(raw_dir, key, "HGT");
            f.surface_geopotential_height_m = gfs_record(raw_dir / environment::gfs_surface_height_file_name(key));
            snapshot = environment::WeatherSnapshot::from_gfs(id, f, geoid);
        }
        if (downsample > 1) snapshot = snapshot->downsampled(downsample);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (out_file.has_parent_path()) std::filesystem::create_directories(out_file.parent_path());
        environment::write_weather_snapshot(out_file, *snapshot);
        std::printf("weather snapshot %s: %d x %d, built in %.1f s\nwrote %s (%.1f MB)\n", key.c_str(),
                    snapshot->rows(), snapshot->columns(), seconds, out_file.string().c_str(),
                    std::filesystem::file_size(out_file) / 1e6);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "weather_snapshot: " << e.what() << "\n";
        return 1;
    }
}
