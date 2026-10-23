#include "starsim/config/geoid_config.hpp"

#include <charconv>
#include <fstream>
#include <limits>
#include <string>
#include <utility>

#include "starsim/config/vehicle_config.hpp"

namespace starsim::config {

namespace {

constexpr int coarse_lines = 181;    // 90 N .. 90 S
constexpr int coarse_columns = 361;  // 180 W .. 180 E inclusive
constexpr double centimetre = 0.01;

}  // namespace

environment::Geoid load_coarse_geoid(const std::filesystem::path& config_dir) {
    const auto path = config_dir / "geoid" / "egm2008_1deg.csv";
    std::ifstream in(path);
    if (!in) throw ConfigError("geoid grid not found: " + path.string());
    environment::GeographicGrid g;
    const int samples = coarse_columns - 1;
    g.grid = {coarse_lines, samples, centimetre, {}};
    g.grid.dn.reserve(static_cast<size_t>(coarse_lines) * samples);
    std::string line;
    int rows = 0;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (rows >= coarse_lines) throw ConfigError(path.string() + ": more than 181 rows");
        const char* p = line.data();
        const char* end = p + line.size();
        for (int c = 0; c < coarse_columns; ++c) {
            int v = 0;
            const auto [next, ec] = std::from_chars(p, end, v);
            if (ec != std::errc() || v < std::numeric_limits<std::int16_t>::min() ||
                v > std::numeric_limits<std::int16_t>::max()) {
                throw ConfigError(path.string() + ": bad value in row " + std::to_string(rows + 1) + ", column " +
                                  std::to_string(c + 1));
            }
            if (c < samples) g.grid.dn.push_back(static_cast<std::int16_t>(v));
            p = next;
            if (c + 1 < coarse_columns) {
                if (p == end || *p != ',') throw ConfigError(path.string() + ": row " + std::to_string(rows + 1) + " is short");
                ++p;
            }
        }
        ++rows;
    }
    if (rows != coarse_lines) throw ConfigError(path.string() + ": expected 181 rows, found " + std::to_string(rows));
    g.north_latitude_deg = 90.0;
    g.west_longitude_deg = -180.0;
    g.step_latitude_deg = 1.0;
    g.step_longitude_deg = 1.0;
    g.global_longitude = true;
    return environment::Geoid(std::move(g));
}

environment::GeographicGrid geographic_grid(const TerrainGridFile& file) {
    TerrainGridFile copy = file;
    return geographic_grid(std::move(copy));
}

environment::GeographicGrid geographic_grid(TerrainGridFile&& file) {
    if (file.projection != "geographic") throw ConfigError("terrain grid '" + file.product + "' is not geographic");
    environment::GeographicGrid g;
    g.grid = {file.lines, file.samples, file.dn_scale_m, std::move(file.dn)};
    g.north_latitude_deg = file.north_latitude_deg;
    g.west_longitude_deg = file.west_longitude_deg;
    g.step_latitude_deg = file.step_latitude_deg;
    g.step_longitude_deg = file.step_longitude_deg;
    g.global_longitude = file.global_longitude;
    return g;
}

}  // namespace starsim::config
