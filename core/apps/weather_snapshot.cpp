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

#include "starsim/environment/weather_snapshot_build.hpp"
#include "starsim/environment/weather_snapshot_file.hpp"

namespace {

using namespace starsim;
using environment::WeatherSnapshotId;
using environment::WeatherSourceKind;

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
        if (source == "era5" && id.kind != WeatherSourceKind::era5) throw std::runtime_error("--era5 needs an era5_<hour> key");
        if (source == "gfs" && id.kind != WeatherSourceKind::gfs) throw std::runtime_error("--gfs needs a gfs_<cycle>_f<lead> key");
        const auto snapshot = environment::build_weather_snapshot_from_raw(id, raw_dir, config_dir, downsample);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (out_file.has_parent_path()) std::filesystem::create_directories(out_file.parent_path());
        environment::write_weather_snapshot(out_file, snapshot);
        std::printf("weather snapshot %s: %d x %d, built in %.1f s\nwrote %s (%.1f MB)\n", key.c_str(),
                    snapshot.rows(), snapshot.columns(), seconds, out_file.string().c_str(),
                    std::filesystem::file_size(out_file) / 1e6);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "weather_snapshot: " << e.what() << "\n";
        return 1;
    }
}
