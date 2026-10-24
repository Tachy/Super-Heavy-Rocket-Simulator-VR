// atmosphere_tables (M8 atmosphere): precomputes the Earth atmosphere tables and writes the cache file. Usage:
//   atmosphere_tables [--config <config dir>] [--out <cache dir>]
// Defaults: the repository's config/ and data/atmosphere/. Always recomputes (tools/package.ps1 ships the file);
// prints the precompute time and the file. Exit code 0 on success, 1 on an error.
#include <chrono>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "starsim/config/atmosphere_optics_config.hpp"
#include "starsim/environment/atmosphere_table_file.hpp"
#include "starsim/environment/atmosphere_tables.hpp"

int main(int argc, char** argv) {
    using namespace starsim;
    std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
    std::filesystem::path out_dir = std::filesystem::path(STARSIM_DATA_DIR) / "atmosphere";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--config" || arg == "--out") && i + 1 < argc) {
            (arg == "--config" ? config_dir : out_dir) = argv[++i];
        } else {
            std::cerr << "usage: atmosphere_tables [--config <config dir>] [--out <cache dir>]\n";
            return 1;
        }
    }
    try {
        const auto optics = config::load_atmosphere_optics(config_dir / "atmosphere" / "earth_optics.toml");
        const auto t0 = std::chrono::steady_clock::now();
        const auto tables = environment::precompute_atmosphere(optics);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::filesystem::create_directories(out_dir);
        const std::filesystem::path file = out_dir / environment::atmosphere_cache_file_name;
        environment::write_atmosphere_tables(file, tables);
        std::printf("atmosphere tables: precompute %.1f s (%d scattering orders), hash %016llx\nwrote %s (%.1f MB)\n",
                    seconds, optics.scattering_orders, static_cast<unsigned long long>(tables.input_hash),
                    file.string().c_str(), std::filesystem::file_size(file) / 1e6);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "atmosphere_tables: " << e.what() << "\n";
        return 1;
    }
}
