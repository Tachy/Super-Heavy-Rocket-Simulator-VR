// Weather snapshots on ellipsoidal height layers and their cache file (M8 weather and clouds, Task 3). Fixtures:
// tests/validation/weather/era5_columns.toml (six real ERA5 columns, 2025-06-27 08 UTC) and gfs_column.toml (one real
// GFS column, 2026-09-20 00Z f006), written by tools/validation/gen_weather_fixtures.py.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <toml++/toml.hpp>

#include "starsim/config/geoid_config.hpp"
#include "starsim/environment/cloud_optics.hpp"
#include "starsim/environment/cloud_render_texels.hpp"
#include "starsim/environment/weather_snapshot.hpp"
#include "starsim/environment/weather_snapshot_file.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::environment;
using weather_decode::FloatGrid;

namespace {

const std::string fixture_dir = STARSIM_WEATHER_FIXTURE_DIR;
constexpr double deg = constants::deg_to_rad;

std::vector<double> doubles(const toml::node_view<const toml::node>& node) {
    std::vector<double> out;
    const auto* array = node.as_array();
    REQUIRE(array != nullptr);
    for (const auto& v : *array) out.push_back(v.value<double>().value());
    return out;
}

struct Era5Column {
    std::string name;
    double latitude_deg = 0, longitude_deg = 0, surface_pressure_pa = 0, surface_geopotential_m2_s2 = 0,
           total_column_water_vapour_kg_m2 = 0;
    std::vector<double> temperature_k, cloud_fraction, liquid, ice, geopotential_m2_s2;
};

struct Era5ColumnSet {
    std::vector<double> levels_pa;
    std::vector<Era5Column> columns;
    const Era5Column& by_name(const std::string& name) const {
        for (const auto& c : columns) {
            if (c.name == name) return c;
        }
        FAIL("no column " << name);
        return columns.front();
    }
};

const Era5ColumnSet& era5_columns() {
    static const Era5ColumnSet set = [] {
        const toml::table t = toml::parse_file(fixture_dir + "/era5_columns.toml");
        Era5ColumnSet s;
        for (double hpa : doubles(t["levels_hpa"])) s.levels_pa.push_back(hpa * 100.0);
        for (const auto& node : *t["column"].as_array()) {
            const auto& c = *node.as_table();
            Era5Column col;
            col.name = c["name"].value<std::string>().value();
            col.latitude_deg = c["latitude_deg"].value<double>().value();
            col.longitude_deg = c["longitude_deg"].value<double>().value();
            col.surface_pressure_pa = c["surface_pressure_pa"].value<double>().value();
            col.surface_geopotential_m2_s2 = c["surface_geopotential_m2_s2"].value<double>().value();
            col.temperature_k = doubles(c["temperature_k"]);
            col.total_column_water_vapour_kg_m2 = c["total_column_water_vapour_kg_m2"].value<double>().value();
            col.cloud_fraction = doubles(c["cloud_fraction"]);
            col.liquid = doubles(c["liquid_kg_kg"]);
            col.ice = doubles(c["ice_kg_kg"]);
            col.geopotential_m2_s2 = doubles(c["geopotential_m2_s2"]);
            s.columns.push_back(std::move(col));
        }
        return s;
    }();
    return set;
}

const Geoid& coarse_geoid() {
    static const Geoid geoid = config::load_coarse_geoid(STARSIM_CONFIG_DIR);
    return geoid;
}

FloatGrid grid(int levels, int rows, int columns) {
    FloatGrid g;
    g.levels = levels;
    g.rows = rows;
    g.columns = columns;
    g.values.assign(static_cast<std::size_t>(levels) * rows * columns, 0.0f);
    return g;
}

void set(FloatGrid& g, int level, int row, int column, double value) {
    g.values[(static_cast<std::size_t>(level) * g.rows + row) * g.columns + column] = static_cast<float>(value);
}

// ERA5 fields on a rows x columns grid whose cells take the fixture columns chosen by pick(row, column).
template <class Pick>
Era5Fields era5_fields(int rows, int columns, Pick pick) {
    const auto& set_ = era5_columns();
    const int levels = static_cast<int>(set_.levels_pa.size());
    Era5Fields f;
    f.levels_pa = set_.levels_pa;
    f.cloud_fraction = f.liquid_kg_kg = f.ice_kg_kg = f.temperature_k = grid(levels, rows, columns);
    f.surface_pressure_pa = f.surface_geopotential_m2_s2 = f.total_column_water_vapour_kg_m2 = grid(1, rows, columns);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < columns; ++c) {
            const Era5Column& col = pick(r, c);
            for (int k = 0; k < levels; ++k) {
                set(f.cloud_fraction, k, r, c, col.cloud_fraction[k]);
                set(f.liquid_kg_kg, k, r, c, col.liquid[k]);
                set(f.ice_kg_kg, k, r, c, col.ice[k]);
                set(f.temperature_k, k, r, c, col.temperature_k[k]);
            }
            set(f.surface_pressure_pa, 0, r, c, col.surface_pressure_pa);
            set(f.surface_geopotential_m2_s2, 0, r, c, col.surface_geopotential_m2_s2);
            set(f.total_column_water_vapour_kg_m2, 0, r, c, col.total_column_water_vapour_kg_m2);
        }
    }
    return f;
}

WeatherSnapshotId era5_id() {
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::era5;
    id.valid_hour = 1100000;
    return id;
}

// Grid latitude / longitude of a node (rows from +90 deg to -90 deg, columns east from 0).
double node_latitude(int row, int rows) { return constants::pi / 2.0 - row * constants::pi / (rows - 1); }
double node_longitude(int column, int columns) { return column * constants::two_pi / columns; }

std::filesystem::path unique_temp(const std::string& stem) {
    static std::atomic<int> counter{0};
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           (stem + "_" + std::to_string(stamp) + "_" + std::to_string(counter++) + weather_snapshot_file_extension);
}

std::vector<char> read_bytes(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void write_bytes(const std::filesystem::path& p, const std::vector<char>& bytes) {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

}  // namespace

TEST_CASE("ERA5 level order is the store's", "[weather]") {
    // era5_levels_hpa indexes the 4D chunks' level axis: it must be the store's `level` array, in its order.
    const auto& set_ = era5_columns();
    REQUIRE(set_.levels_pa.size() == era5_levels_hpa.size());
    for (std::size_t k = 0; k < era5_levels_hpa.size(); ++k) CHECK(set_.levels_pa[k] == era5_levels_hpa[k] * 100.0);
}

TEST_CASE("Hypsometric heights match ERA5 geopotential", "[weather]") {
    const auto& set_ = era5_columns();
    REQUIRE(set_.columns.size() == 6);
    for (const auto& c : set_.columns) {
        const auto moist = hypsometric_level_heights_gpm(set_.levels_pa, c.temperature_k, c.total_column_water_vapour_kg_m2,
                                                         c.surface_pressure_pa, c.surface_geopotential_m2_s2);
        const auto dry = hypsometric_level_heights_gpm(set_.levels_pa, c.temperature_k, 0.0, c.surface_pressure_pa,
                                                       c.surface_geopotential_m2_s2);
        double max_moist = 0.0, max_dry = 0.0;
        int checked = 0;
        for (std::size_t k = 0; k < set_.levels_pa.size(); ++k) {
            const double p = set_.levels_pa[k];
            if (p >= c.surface_pressure_pa) {
                CHECK(std::isnan(moist[k]));  // at or below the surface
                continue;
            }
            if (p < 10000.0) continue;  // up to 100 hPa
            const double store_gpm = c.geopotential_m2_s2[k] / constants::standard_gravity_mps2;
            max_moist = std::max(max_moist, std::abs(moist[k] - store_gpm));
            max_dry = std::max(max_dry, std::abs(dry[k] - store_gpm));
            ++checked;
        }
        std::printf("hypsometric %-18s levels %2d  max |dZ| moist %5.1f m, dry %5.1f m\n", c.name.c_str(), checked,
                    max_moist, max_dry);
        INFO(c.name);
        CHECK(checked >= 8);
        CHECK(max_moist < 30.0);
    }
}

TEST_CASE("Geopotential height converts to ellipsoidal height", "[weather]") {
    // Independent: integrate the WGS84 normal gravity with its second-order height series (NIMA TR8350.2 eq. 4-3),
    // gamma_h = gamma (1 - 2/a (1 + f + m - 2 f sin^2 phi) h + 3 h^2 / a^2), and solve g0 Z = int_0^h gamma_h dh.
    const double a = 6378137.0, f = 1.0 / 298.257223563, m = 0.00344978600308;
    const double ge = 9.7803253359, k = 0.00193185265241, e2 = 0.00669437999013;
    for (double lat_deg : {0.0, 45.0, 90.0}) {
        const double s2 = std::pow(std::sin(lat_deg * deg), 2);
        const double gamma = ge * (1.0 + k * s2) / std::sqrt(1.0 - e2 * s2);
        const double c1 = 2.0 / a * (1.0 + f + m - 2.0 * f * s2), c2 = 3.0 / (a * a);
        const auto potential = [&](double h) { return gamma * (h - c1 * h * h / 2.0 + c2 * h * h * h / 3.0); };
        double lo = 9000.0, hi = 11000.0;
        for (int i = 0; i < 80; ++i) {
            const double mid = 0.5 * (lo + hi);
            (potential(mid) < constants::standard_gravity_mps2 * 10000.0 ? lo : hi) = mid;
        }
        INFO("latitude " << lat_deg);
        CHECK(std::abs(geometric_height_m(10000.0, lat_deg * deg) - lo) < 1.0);
    }
    CHECK(geometric_height_m(0.0, 0.3) == Catch::Approx(0.0).margin(1e-9));

    // The geoid is added: sea-level ground (zero surface geopotential) at a grid node is the undulation there.
    const int rows = 5, columns = 8;
    auto fields = era5_fields(rows, columns, [](int, int) -> const Era5Column& { return era5_columns().by_name("tropics"); });
    for (auto& v : fields.surface_geopotential_m2_s2.values) v = 0.0f;
    const auto snapshot = WeatherSnapshot::from_era5(era5_id(), fields, coarse_geoid());
    const double lat = node_latitude(2, rows), lon = node_longitude(2, columns);  // 0 N, 90 E
    const double n = coarse_geoid().undulation_m(lat, lon);
    CHECK(std::abs(n) > 10.0);
    CHECK(snapshot.ground_height_m(lat, lon) == Catch::Approx(n).margin(1e-3));
}

TEST_CASE("Water content in kg per cubic metre", "[weather]") {
    CHECK(cloud_water_density_kg_m3(1e-4, 85000.0, 280.0) == Catch::Approx(1e-4 * 85000.0 / (287.05 * 280.0)));

    // ERA5's contents are per kg of moist air: the build divides by R_d T_v. The vapour profile puts q_s = W g0 a / p_s
    // at the surface (a = R_d T_1 / (g0 H_v)): T_v / T - 1 = (R_v / R_d - 1) q at every level, largest at the bottom.
    const auto& set_ = era5_columns();
    const auto& c = set_.by_name("tropics");
    const auto tv = level_virtual_temperatures_k(set_.levels_pa, c.temperature_k, c.total_column_water_vapour_kg_m2,
                                                 c.surface_pressure_pa);
    const auto dry = level_virtual_temperatures_k(set_.levels_pa, c.temperature_k, 0.0, c.surface_pressure_pa);
    const std::size_t bottom = set_.levels_pa.size() - 1;  // 1000 hPa, above the 1008.8 hPa surface
    REQUIRE(set_.levels_pa[bottom] == 100000.0);
    const double a = 287.05 * c.temperature_k[bottom] / (9.80665 * 2000.0);
    const double q_s = c.total_column_water_vapour_kg_m2 * 9.80665 * a / c.surface_pressure_pa;
    const double q_bottom = q_s * std::pow(100000.0 / c.surface_pressure_pa, a - 1.0);
    CHECK(tv[bottom] == Catch::Approx(c.temperature_k[bottom] * (1.0 + (461.51 / 287.05 - 1.0) * q_bottom)));
    CHECK(tv[bottom] / c.temperature_k[bottom] - 1.0 > 0.01);
    CHECK(dry[bottom] == Catch::Approx(c.temperature_k[bottom]));
    for (std::size_t k = 0; k + 1 < set_.levels_pa.size(); ++k) {
        CHECK(tv[k] / c.temperature_k[k] <= tv[k + 1] / c.temperature_k[k + 1] + 1e-15);  // drier upward
    }
}

TEST_CASE("No cloud under the ground", "[weather]") {
    // Every cell is the Himalaya column (surface at 514 hPa, 5632 gpm): the ERA5 levels 550 .. 1000 hPa lie under the
    // ground and carry extrapolated values that must not appear.
    const int rows = 5, columns = 8;
    const auto& himalaya = era5_columns().by_name("himalaya");
    const auto snapshot = WeatherSnapshot::from_era5(
        era5_id(), era5_fields(rows, columns, [&](int, int) -> const Era5Column& { return himalaya; }), coarse_geoid());
    const double lat = 30.0 * deg, lon = 85.0 * deg;
    const double ground = snapshot.ground_height_m(lat, lon);
    CHECK(ground > 5500.0);
    CHECK(ground < 5800.0);
    for (double h = -500.0; h < ground; h += 25.0) {
        const auto cell = snapshot.at(lat, lon, h);
        INFO("height " << h);
        CHECK(cell.cloud_fraction == 0.0);
        CHECK(cell.liquid_water_kg_m3 == 0.0);
        CHECK(cell.ice_water_kg_m3 == 0.0);
    }
    for (double h = ground; h < 21000.0; h += 25.0) {
        const auto cell = snapshot.at(lat, lon, h);
        CHECK(std::isfinite(cell.cloud_fraction));
        CHECK(std::isfinite(cell.liquid_water_kg_m3));
        CHECK(std::isfinite(cell.ice_water_kg_m3));
    }
    // Layers wholly under the ground are empty in the render form.
    const auto texels = snapshot.render_texels();
    REQUIRE(texels.size() == static_cast<std::size_t>(weather_layer_count) * rows * columns * 4);
    const int layers_under = static_cast<int>(std::floor(weather_layer_coordinate(ground)));
    for (int layer = 0; layer < layers_under; ++layer) {
        for (std::size_t i = 0; i < static_cast<std::size_t>(rows) * columns * 4; ++i) {
            CHECK(texels[static_cast<std::size_t>(layer) * rows * columns * 4 + i] == 0);
        }
    }
}

TEST_CASE("Cloud water path is kept on the layers", "[weather]") {
    // The tropical column's liquid + ice path from the ground to 20 km: profile (piecewise linear in height between the
    // levels above the ground, held below the lowest one) vs the layer means; the 8-bit codes cost <= 1.97 %.
    const auto& set_ = era5_columns();
    const auto& c = set_.by_name("tropics");
    const int rows = 3, columns = 4;
    const auto snapshot = WeatherSnapshot::from_era5(
        era5_id(), era5_fields(rows, columns, [&](int, int) -> const Era5Column& { return c; }), coarse_geoid());
    const double lat = node_latitude(1, rows), lon = node_longitude(1, columns);
    const auto z = hypsometric_level_heights_gpm(set_.levels_pa, c.temperature_k, c.total_column_water_vapour_kg_m2,
                                                 c.surface_pressure_pa, c.surface_geopotential_m2_s2);
    const auto tv = level_virtual_temperatures_k(set_.levels_pa, c.temperature_k, c.total_column_water_vapour_kg_m2,
                                                 c.surface_pressure_pa);
    const double n = coarse_geoid().undulation_m(lat, lon);
    struct Node { double h, water; };
    std::vector<Node> nodes;
    for (std::size_t k = 0; k < z.size(); ++k) {
        if (std::isnan(z[k])) continue;
        nodes.push_back({geometric_height_m(z[k], lat) + n,
                         cloud_water_density_kg_m3(c.liquid[k] + c.ice[k], set_.levels_pa[k], tv[k])});
    }
    std::sort(nodes.begin(), nodes.end(), [](const Node& x, const Node& y) { return x.h < y.h; });
    const double ground = snapshot.ground_height_m(lat, lon);
    const auto profile = [&](double h) {
        if (h <= nodes.front().h) return nodes.front().water;
        for (std::size_t i = 1; i < nodes.size(); ++i) {
            if (h <= nodes[i].h) {
                const double t = (h - nodes[i - 1].h) / (nodes[i].h - nodes[i - 1].h);
                return nodes[i - 1].water + t * (nodes[i].water - nodes[i - 1].water);
            }
        }
        return nodes.back().water;
    };
    double path_profile = 0.0, path_layers = 0.0;
    for (double h = ground + 0.5; h < weather_top_m; h += 1.0) path_profile += profile(h);
    const auto texels = snapshot.render_texels();
    for (int layer = 0; layer < weather_layer_count; ++layer) {
        const double bottom = std::max(weather_layer_bottom_m(layer), ground);
        const double top = weather_layer_bottom_m(layer + 1);
        if (top <= bottom) continue;
        const std::size_t i = ((static_cast<std::size_t>(layer) * rows + 1) * columns + 1) * 4;
        path_layers += (decode_weather_water(texels[i + 1]) + decode_weather_water(texels[i + 2])) * (top - bottom);
    }
    std::printf("tropical water path: profile %.4f kg/m^2, layers %.4f kg/m^2\n", path_profile, path_layers);
    CHECK(path_profile > 0.01);
    CHECK(path_layers == Catch::Approx(path_profile).epsilon(0.02));
}

TEST_CASE("GFS heights come from HGT", "[weather]") {
    const toml::table t = toml::parse_file(fixture_dir + "/gfs_column.toml");
    const auto levels_mb = doubles(t["levels_mb"]);
    const auto hgt = doubles(t["geopotential_height_m"]);
    const auto tmp = doubles(t["temperature_k"]);
    const double surface_gpm = t["surface_height_m"].value<double>().value();
    const int levels = static_cast<int>(levels_mb.size()), rows = 5, columns = 8;
    GfsFields f;
    for (double mb : levels_mb) f.levels_pa.push_back(mb * 100.0);
    f.cloud_percent = f.liquid_kg_kg = f.ice_kg_kg = f.temperature_k = f.geopotential_height_m =
        grid(levels, rows, columns);
    f.surface_geopotential_height_m = grid(1, rows, columns);
    const auto index_of = [&](double mb) {
        return static_cast<int>(std::find(levels_mb.begin(), levels_mb.end(), mb) - levels_mb.begin());
    };
    const int below_ground = index_of(850.0), cloud_level = index_of(500.0);
    REQUIRE(hgt[below_ground] < surface_gpm);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < columns; ++c) {
            for (int k = 0; k < levels; ++k) {
                set(f.temperature_k, k, r, c, tmp[k]);
                set(f.geopotential_height_m, k, r, c, hgt[k]);
            }
            // A cloud on the under-ground 850 mb level (must be ignored) and one at 500 mb.
            set(f.cloud_percent, below_ground, r, c, 100.0);
            set(f.liquid_kg_kg, below_ground, r, c, 1e-3);
            set(f.cloud_percent, cloud_level, r, c, 80.0);
            set(f.liquid_kg_kg, cloud_level, r, c, 2e-4);
            set(f.surface_geopotential_height_m, 0, r, c, surface_gpm);
        }
    }
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::gfs;
    const auto snapshot = WeatherSnapshot::from_gfs(id, f, coarse_geoid());
    const double lat = node_latitude(2, rows), lon = node_longitude(2, columns);
    const double n = coarse_geoid().undulation_m(lat, lon);
    CHECK(snapshot.ground_height_m(lat, lon) == Catch::Approx(geometric_height_m(surface_gpm, lat) + n).margin(1e-3));
    // Just above the ground: the lowest level above it (800 mb) is clear.
    CHECK(snapshot.at(lat, lon, snapshot.ground_height_m(lat, lon) + 30.0).liquid_water_kg_m3 == 0.0);
    // The 500 mb cloud at its height (layer mean of a 1 km layer around a peak: below the peak, above half of it).
    const double h500 = geometric_height_m(hgt[cloud_level], lat) + n;
    const auto cell = snapshot.at(lat, lon, h500);
    const double peak = cloud_water_density_kg_m3(2e-4, 50000.0, tmp[cloud_level]);
    CHECK(cell.liquid_water_kg_m3 > 0.3 * peak);
    CHECK(cell.liquid_water_kg_m3 < peak * 1.02);
    CHECK(cell.cloud_fraction > 0.2);
    CHECK(cell.cloud_fraction <= 0.8 + 1e-9);
}

TEST_CASE("Poles and the date line", "[weather]") {
    const int rows = 9, columns = 16;
    const auto& cols = era5_columns().columns;
    const auto snapshot = WeatherSnapshot::from_era5(
        era5_id(), era5_fields(rows, columns, [&](int r, int c) -> const Era5Column& { return cols[(r * 5 + c) % 6]; }),
        coarse_geoid());
    for (double pole : {constants::pi / 2.0, -constants::pi / 2.0}) {
        const double ground = snapshot.ground_height_m(pole, 0.0);
        CHECK(std::isfinite(ground));
        for (double h : {ground + 100.0, 3000.0, 8000.0, 12000.0}) {
            const auto ref = snapshot.at(pole, 0.0, h);
            CHECK(std::isfinite(ref.cloud_fraction));
            CHECK(std::isfinite(ref.liquid_water_kg_m3));
            CHECK(std::isfinite(ref.ice_water_kg_m3));
            for (double lon_deg = -180.0; lon_deg <= 360.0; lon_deg += 17.0) {
                const auto cell = snapshot.at(pole, lon_deg * deg, h);
                CHECK(cell.cloud_fraction == Catch::Approx(ref.cloud_fraction).margin(1e-12));
                CHECK(cell.liquid_water_kg_m3 == Catch::Approx(ref.liquid_water_kg_m3).margin(1e-15));
                CHECK(cell.ice_water_kg_m3 == Catch::Approx(ref.ice_water_kg_m3).margin(1e-15));
                CHECK(snapshot.ground_height_m(pole, lon_deg * deg) == Catch::Approx(ground).margin(1e-9));
            }
        }
    }
    for (double lat_deg : {-60.0, 12.5, 37.0}) {
        for (double h : {500.0, 2600.0, 9000.0}) {
            const auto east = snapshot.at(lat_deg * deg, 359.9 * deg, h);
            const auto west = snapshot.at(lat_deg * deg, -0.1 * deg, h);
            CHECK(east.cloud_fraction == Catch::Approx(west.cloud_fraction).margin(1e-12));
            CHECK(east.liquid_water_kg_m3 == Catch::Approx(west.liquid_water_kg_m3).margin(1e-15));
            CHECK(east.ice_water_kg_m3 == Catch::Approx(west.ice_water_kg_m3).margin(1e-15));
        }
    }
}

TEST_CASE("Layer mapping", "[weather]") {
    for (int i = 0; i <= 16; ++i) CHECK(weather_layer_bottom_m(i) == Catch::Approx(250.0 * i));
    for (int i = 17; i <= 32; ++i) CHECK(weather_layer_bottom_m(i) == Catch::Approx(4000.0 + 1000.0 * (i - 16)));
    for (int i = 0; i <= 32; ++i) CHECK(weather_layer_coordinate(weather_layer_bottom_m(i)) == Catch::Approx(i));
    CHECK(weather_layer_coordinate(-100.0) == 0.0);
    CHECK(weather_layer_coordinate(25000.0) == 32.0);
    CHECK(weather_layer_coordinate(4000.0 - 1e-6) == Catch::Approx(16.0).margin(1e-6));
    CHECK(weather_layer_coordinate(4000.0 + 1e-6) == Catch::Approx(16.0).margin(1e-6));
    double previous = -1.0;
    for (double h = -10.0; h <= 20010.0; h += 0.5) {
        const double s = weather_layer_coordinate(h);
        CHECK(s >= previous);
        previous = s;
    }
}

TEST_CASE("Render encoding error", "[weather]") {
    for (double v = 1e-5; v <= 3e-3; v *= 1.013) {
        const double back = decode_weather_water(encode_weather_water(v));
        INFO("value " << v);
        CHECK(std::abs(back / v - 1.0) <= 0.02);
    }
    CHECK(encode_weather_water(0.0) == 0);
    CHECK(decode_weather_water(0) == 0.0);
    CHECK(encode_weather_water(1e-9) == 0);
    CHECK(encode_weather_water(1.0) == 255);
    for (double f = 0.0; f <= 1.0; f += 0.01) {
        CHECK(std::abs(decode_weather_cloud_fraction(encode_weather_cloud_fraction(f)) - f) <= 0.5 / 255.0 + 1e-12);
    }
}

TEST_CASE("Uniform slab and downsampling", "[weather]") {
    const auto slab = WeatherSnapshot::uniform_slab(9, 16, 1000.0, 2000.0, 5e-4, 0.75);
    CHECK(slab.rows() == 9);
    CHECK(slab.columns() == 16);
    const auto inside = slab.at(0.3, 1.0, 1500.0);
    CHECK(inside.liquid_water_kg_m3 == Catch::Approx(5e-4).epsilon(0.02));
    CHECK(inside.cloud_fraction == Catch::Approx(0.75).margin(0.5 / 255.0));
    CHECK(inside.ice_water_kg_m3 == 0.0);
    CHECK(slab.at(0.3, 1.0, 5000.0).liquid_water_kg_m3 == 0.0);
    CHECK(slab.at(0.3, 1.0, 21000.0).liquid_water_kg_m3 == 0.0);
    const auto coarse = slab.downsampled(4);
    CHECK(coarse.rows() == 3);
    CHECK(coarse.columns() == 4);
    CHECK(coarse.render_texels().size() == static_cast<std::size_t>(weather_layer_count) * 3 * 4 * 4);
    CHECK(coarse.at(-0.4, 2.0, 1500.0).liquid_water_kg_m3 == Catch::Approx(inside.liquid_water_kg_m3).epsilon(1e-9));
}

TEST_CASE("Snapshot files round-trip; damaged ones are refused", "[weather]") {
    const int rows = 9, columns = 16;
    const auto& cols = era5_columns().columns;
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::gfs;
    id.valid_hour = 1110054;
    id.cycle_hour = 1110000;
    id.lead_h = 54;
    id.analog_hour = 7;
    auto fields = era5_fields(rows, columns, [&](int r, int c) -> const Era5Column& { return cols[(r + c) % 6]; });
    fields.temperature_2m_k = fields.dewpoint_2m_k = grid(1, rows, columns);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < columns; ++c) {
            set(fields.temperature_2m_k, 0, r, c, 285.0 + r);
            set(fields.dewpoint_2m_k, 0, r, c, 280.0 + 0.5 * c);
        }
    }
    const auto snapshot = WeatherSnapshot::from_era5(id, fields, coarse_geoid());
    const auto file = unique_temp("weather_round_trip");
    write_weather_snapshot(file, snapshot);
    const auto back = read_weather_snapshot(file);
    REQUIRE(back.has_value());
    CHECK(back->id().kind == id.kind);
    CHECK(back->id().valid_hour == id.valid_hour);
    CHECK(back->id().cycle_hour == id.cycle_hour);
    CHECK(back->id().lead_h == id.lead_h);
    CHECK(back->id().analog_hour == id.analog_hour);
    CHECK(back->rows() == rows);
    CHECK(back->columns() == columns);
    CHECK(back->render_texels() == snapshot.render_texels());
    CHECK(back->ground_heights_m() == snapshot.ground_heights_m());
    CHECK(back->lcl_heights_m() == snapshot.lcl_heights_m());

    const auto bytes = read_bytes(file);
    const auto damaged = unique_temp("weather_damaged");
    write_bytes(damaged, std::vector<char>(bytes.begin(), bytes.end() - 1));
    CHECK_FALSE(read_weather_snapshot(damaged).has_value());
    write_bytes(damaged, std::vector<char>(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(bytes.size() / 2)));
    CHECK_FALSE(read_weather_snapshot(damaged).has_value());
    auto flipped = bytes;
    flipped[flipped.size() / 2] ^= 0x10;
    write_bytes(damaged, flipped);
    CHECK_FALSE(read_weather_snapshot(damaged).has_value());
    write_bytes(damaged, {});
    CHECK_FALSE(read_weather_snapshot(damaged).has_value());
    CHECK_FALSE(read_weather_snapshot(unique_temp("weather_missing")).has_value());
    std::filesystem::remove(file);
    std::filesystem::remove(damaged);
}

TEST_CASE("Lifting condensation level follows Romps' exact expression", "[weather]") {
    // Reference: an independent Python port of the published code of Romps 2017 (J. Atmos. Sci. 74, 3891, doi:10.1175/
    // JAS-D-17-0102.1; scipy.special.lambertw, branch -1, his constants), dew point -> vapour pressure with his
    // saturation pressure: T = 303.15 K, Td = 293.15 K, p = 100 kPa -> 1281.38 m (Espy: 1250 m).
    CHECK(lifting_condensation_level_m(303.15, 293.15, 100000.0) == Catch::Approx(1281.38).epsilon(0.01));
    CHECK(lifting_condensation_level_m(303.15, 293.15, 101325.0) == Catch::Approx(1281.15).epsilon(0.01));
    CHECK(lifting_condensation_level_m(303.15, 293.15) == lifting_condensation_level_m(303.15, 293.15, 101325.0));
    CHECK(lifting_condensation_level_m(288.15, 283.15, 101325.0) == Catch::Approx(631.71).epsilon(0.01));
    CHECK(lifting_condensation_level_m(260.0, 255.0, 101325.0) == Catch::Approx(611.11).epsilon(0.01));
    CHECK(lifting_condensation_level_m(310.0, 280.0, 101325.0) == Catch::Approx(3730.05).epsilon(0.01));
    // Saturated or supersaturated 2 m air: the base is at the ground.
    CHECK(lifting_condensation_level_m(290.0, 290.0) == 0.0);
    CHECK(lifting_condensation_level_m(290.0, 291.0) == 0.0);
    // Invalid input.
    CHECK(std::isnan(lifting_condensation_level_m(std::nan(""), 280.0)));
    CHECK(std::isnan(lifting_condensation_level_m(290.0, 0.0)));
    // Monotone in the spread, near the 125 m/K rule at a small one.
    double previous = 0.0;
    for (double spread = 0.5; spread < 40.0; spread += 0.5) {
        const double z = lifting_condensation_level_m(295.0, 295.0 - spread);
        CHECK(z > previous);
        previous = z;
    }
    CHECK(lifting_condensation_level_m(295.0, 294.0) == Catch::Approx(125.0).epsilon(0.1));
}

TEST_CASE("A built snapshot stores the lifting condensation level per column", "[weather]") {
    const int rows = 5, columns = 8;
    const auto& cols = era5_columns().columns;
    auto fields = era5_fields(rows, columns, [&](int, int) -> const Era5Column& { return cols[0]; });
    fields.temperature_2m_k = fields.dewpoint_2m_k = grid(1, rows, columns);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < columns; ++c) {
            set(fields.temperature_2m_k, 0, r, c, 300.0);
            set(fields.dewpoint_2m_k, 0, r, c, 300.0 - 2.0 * (c + 1));
        }
    }
    const auto snapshot = WeatherSnapshot::from_era5(era5_id(), fields, coarse_geoid());
    const double ps = cols[0].surface_pressure_pa;
    // Interior rows keep their own column's value (pole rows are averaged), with the column's surface pressure.
    for (int c = 0; c < columns; ++c) {
        CHECK(snapshot.lcl_m(2, c) ==
              Catch::Approx(lifting_condensation_level_m(300.0, 300.0 - 2.0 * (c + 1), ps)).epsilon(1e-5));
    }
    CHECK(snapshot.lcl_m(2, 7) > snapshot.lcl_m(2, 0));
    // Pole rows: one value (the mean of the row).
    for (int c = 1; c < columns; ++c) CHECK(snapshot.lcl_m(0, c) == snapshot.lcl_m(0, 0));
    // Downsampling averages it like the ground.
    const auto coarse = snapshot.downsampled(4);
    CHECK(coarse.lcl_m(1, 0) > 0.0f);
    // Without the 2 m fields the level is unknown, which is not the saturated 0.
    const auto bare = WeatherSnapshot::from_era5(
        era5_id(), era5_fields(rows, columns, [&](int, int) -> const Era5Column& { return cols[0]; }), coarse_geoid());
    CHECK(bare.lcl_m(2, 3) == weather_lcl_unknown_m);
    // Only one of the pair, or a wrong shape, is refused.
    auto half = fields;
    half.dewpoint_2m_k = FloatGrid{};
    CHECK_THROWS_AS(WeatherSnapshot::from_era5(era5_id(), half, coarse_geoid()), std::invalid_argument);
}

TEST_CASE("A format version 1 snapshot file is refused", "[weather]") {
    const int rows = 5, columns = 8;
    const auto snapshot = WeatherSnapshot::uniform_slab(rows, columns, 1000.0, 2000.0, 5e-4, 0.5);
    const auto file = unique_temp("weather_v1");
    write_weather_snapshot(file, snapshot);
    REQUIRE(read_weather_snapshot(file).has_value());
    auto bytes = read_bytes(file);
    for (const std::uint32_t old_version : {2u, 3u, 4u, 5u, 6u}) {  // the current layout stamped 2 .. 6: refused
        auto v2 = bytes;
        std::memcpy(v2.data() + 8, &old_version, sizeof(old_version));
        v2.resize(v2.size() - sizeof(std::uint64_t));
        std::uint64_t h = 14695981039346656037ull;
        for (char b : v2) h = (h ^ static_cast<unsigned char>(b)) * 1099511628211ull;
        v2.insert(v2.end(), reinterpret_cast<const char*>(&h), reinterpret_cast<const char*>(&h) + sizeof(h));
        const auto v2_file = unique_temp("weather_v2_old");
        write_bytes(v2_file, v2);
        CHECK_FALSE(read_weather_snapshot(v2_file).has_value());
        std::filesystem::remove(v2_file);
    }
    // The version 1 layout: no LCL block after the ground heights; header of 64 bytes; checksum recomputed (FNV-1a 64).
    const std::size_t cells = static_cast<std::size_t>(rows) * columns, header = 64;
    bytes.erase(bytes.begin() + static_cast<std::ptrdiff_t>(header + cells * sizeof(float)),
                bytes.begin() + static_cast<std::ptrdiff_t>(header + 2 * cells * sizeof(float)));
    const std::uint32_t one = 1;
    std::memcpy(bytes.data() + 8, &one, sizeof(one));
    bytes.resize(bytes.size() - sizeof(std::uint64_t));
    std::uint64_t hash = 14695981039346656037ull;
    for (char b : bytes) hash = (hash ^ static_cast<unsigned char>(b)) * 1099511628211ull;
    bytes.insert(bytes.end(), reinterpret_cast<const char*>(&hash), reinterpret_cast<const char*>(&hash) + sizeof(hash));
    const auto old = unique_temp("weather_v1_old");
    write_bytes(old, bytes);
    CHECK_FALSE(read_weather_snapshot(old).has_value());
    // The same bytes with the right version number (and the missing block) are not a valid version 2 file either.
    const std::uint32_t two = 2;
    // Version 2 stored unknown levels as 0 and had no base redistribution; version 3 moved only the lowest liquid run; version 4 merged a deck far above the lifted cloud.
    CHECK(weather_snapshot_file_version == 7);  // 6: no dropped-condensate block; 5: no total cloud cover / cumulus water
    std::memcpy(bytes.data() + 8, &two, sizeof(two));
    bytes.resize(bytes.size() - sizeof(std::uint64_t));
    hash = 14695981039346656037ull;
    for (char b : bytes) hash = (hash ^ static_cast<unsigned char>(b)) * 1099511628211ull;
    bytes.insert(bytes.end(), reinterpret_cast<const char*>(&hash), reinterpret_cast<const char*>(&hash) + sizeof(hash));
    write_bytes(old, bytes);
    CHECK_FALSE(read_weather_snapshot(old).has_value());
    std::filesystem::remove(file);
    std::filesystem::remove(old);
}

TEST_CASE("Writing a snapshot removes stale temp files of killed writers", "[weather]") {
    const auto dir = unique_temp("weather_stale_dir").replace_extension();
    std::filesystem::create_directories(dir);
    const auto stale = dir / "era5_1.wthr.tmp.123.456";
    const auto fresh = dir / "era5_2.wthr.tmp.789.012";
    const auto other = dir / "notes.txt";
    for (const auto& p : {stale, fresh, other}) write_bytes(p, {'x'});
    const auto old = std::filesystem::file_time_type::clock::now() - std::chrono::hours(1);
    std::filesystem::last_write_time(stale, old);
    std::filesystem::last_write_time(other, old);
    write_weather_snapshot(dir / "era5_3.wthr", WeatherSnapshot::uniform_slab(3, 4, 0.0, 1000.0, 1e-4, 0.5));
    CHECK_FALSE(std::filesystem::exists(stale));  // older than 10 minutes: a killed writer's
    CHECK(std::filesystem::exists(fresh));        // may belong to a live writer
    CHECK(std::filesystem::exists(other));        // not a snapshot temp file
    CHECK(read_weather_snapshot(dir / "era5_3.wthr").has_value());
    std::filesystem::remove_all(dir);
}

// Real-date plausibility (spec Validation): the snapshot built from the 37-level ERA5 columns of two real hours against the
// store's own single-level total_cloud_cover (tcc) and cloud_base_height (cbh). tests/validation/weather/era5_plausibility.toml.
namespace {

struct PlausibilityColumn {
    std::string name, time_utc;
    double surface_pressure_pa = 0, surface_geopotential_m2_s2 = 0, total_column_water_vapour_kg_m2 = 0, total_cloud_cover = 0,
           cloud_base_height_m = 0, temperature_2m_k = 0, dewpoint_2m_k = 0;
    std::vector<double> temperature_k, cloud_fraction, liquid, ice;
};

}  // namespace

TEST_CASE("Real-date columns agree with ERA5's total cloud cover and cloud base", "[weather]") {
    const toml::table t = toml::parse_file(fixture_dir + "/era5_plausibility.toml");
    std::vector<double> levels_pa;
    for (double hpa : doubles(t["levels_hpa"])) levels_pa.push_back(hpa * 100.0);
    std::vector<PlausibilityColumn> cells;
    for (const auto& node : *t["column"].as_array()) {
        const auto& c = *node.as_table();
        PlausibilityColumn col;
        col.name = c["name"].value<std::string>().value();
        col.time_utc = c["time_utc"].value<std::string>().value();
        col.surface_pressure_pa = c["surface_pressure_pa"].value<double>().value();
        col.surface_geopotential_m2_s2 = c["surface_geopotential_m2_s2"].value<double>().value();
        col.total_column_water_vapour_kg_m2 = c["total_column_water_vapour_kg_m2"].value<double>().value();
        col.total_cloud_cover = c["total_cloud_cover"].value<double>().value();
        col.cloud_base_height_m = c["cloud_base_height_m"].value<double>().value();
        col.temperature_2m_k = c["temperature_2m_k"].value<double>().value();
        col.dewpoint_2m_k = c["dewpoint_2m_k"].value<double>().value();
        col.temperature_k = doubles(c["temperature_k"]);
        col.cloud_fraction = doubles(c["cloud_fraction"]);
        col.liquid = doubles(c["liquid_kg_kg"]);
        col.ice = doubles(c["ice_kg_kg"]);
        cells.push_back(std::move(col));
    }
    REQUIRE(cells.size() == 10);

    // One snapshot per hour (5 cells each) on a 3 x 5 grid: row 1 is the equator, the cells sit at the five nodes' longitudes.
    const int levels = static_cast<int>(levels_pa.size()), rows = 3, columns = 5;
    double deficit_sum = 0.0;
    for (int hour = 0; hour < 2; ++hour) {
        Era5Fields f;
        f.levels_pa = levels_pa;
        f.cloud_fraction = f.liquid_kg_kg = f.ice_kg_kg = f.temperature_k = grid(levels, rows, columns);
        f.surface_pressure_pa = f.surface_geopotential_m2_s2 = f.total_column_water_vapour_kg_m2 = grid(1, rows, columns);
        f.temperature_2m_k = f.dewpoint_2m_k = f.total_cloud_cover = grid(1, rows, columns);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < columns; ++c) {
                const PlausibilityColumn& col = cells[static_cast<std::size_t>(hour * columns + c)];
                set(f.total_cloud_cover, 0, r, c, col.total_cloud_cover);
                set(f.temperature_2m_k, 0, r, c, col.temperature_2m_k);
                set(f.dewpoint_2m_k, 0, r, c, col.dewpoint_2m_k);
                for (int k = 0; k < levels; ++k) {
                    set(f.cloud_fraction, k, r, c, col.cloud_fraction[k]);
                    set(f.liquid_kg_kg, k, r, c, col.liquid[k]);
                    set(f.ice_kg_kg, k, r, c, col.ice[k]);
                    set(f.temperature_k, k, r, c, col.temperature_k[k]);
                }
                set(f.surface_pressure_pa, 0, r, c, col.surface_pressure_pa);
                set(f.surface_geopotential_m2_s2, 0, r, c, col.surface_geopotential_m2_s2);
                set(f.total_column_water_vapour_kg_m2, 0, r, c, col.total_column_water_vapour_kg_m2);
            }
        }
        const auto snapshot = WeatherSnapshot::from_era5(era5_id(), f, coarse_geoid());
        for (int c = 0; c < columns; ++c) {
            const PlausibilityColumn& col = cells[static_cast<std::size_t>(hour * columns + c)];
            const double lat = node_latitude(1, rows), lon = node_longitude(c, columns);
            const double ground = snapshot.ground_height_m(lat, lon);
            // The renderer's cover: the largest cloud fraction in the column (maximum overlap, the lower bound of any cover).
            const double cover = cloud_column_depths_above(snapshot, lat, lon, ground).max_fraction;
            // ERA5's own scheme on the snapshot's layers, top to bottom (Geleyn & Hollingsworth 1979): layers that touch
            // overlap maximally, separated clouds randomly, 1 - C = prod_k (1 - max(C_k, C_k-1)) / (1 - C_k-1).
            double clear = 1.0, above = 0.0;
            // Cloud base as ERA5 defines cbh (ECMWF IFS documentation): the lowest level where the cloud fraction exceeds
            // 1 % and there is condensate; here the lowest layer whose fraction exceeds 1 % with condensate in it.
            double base_agl_m = std::nan(""), layer_m = 0.0;
            for (int layer = weather_layer_count - 1; layer >= 0; --layer) {
                const double bottom = weather_layer_bottom_m(layer), top = weather_layer_bottom_m(layer + 1);
                if (top <= ground) break;
                const WeatherCell cell = snapshot.at(lat, lon, 0.5 * (bottom + top));
                const double fraction = cell.liquid_water_kg_m3 + cell.ice_water_kg_m3 > 0.0 ? cell.cloud_fraction : 0.0;
                clear *= above < 1.0 ? (1.0 - std::max(fraction, above)) / (1.0 - above) : 1.0;
                above = fraction;
                const WeatherCell data = cell;  // the shipped (scaled) snapshot: a trace layer is not scaled across 1 %
                if (data.liquid_water_kg_m3 + data.ice_water_kg_m3 > 0.0 && data.cloud_fraction > 0.01) {
                    base_agl_m = std::max(bottom, ground) - ground;
                    layer_m = top - bottom;
                }
            }
            const double cover_max_random = 1.0 - clear;
            INFO(col.time_utc << " " << col.name);
            std::printf("plausibility %s %-26s tcc %.2f -> max %.2f, max-random %.3f   cbh %7.0f m, lcl %6.0f m -> base "
                        "%7.0f m (layer %4.0f m), cumulus water added %.1f g/m^2\n",
                        col.time_utc.c_str(), col.name.c_str(), col.total_cloud_cover, cover, cover_max_random,
                        col.cloud_base_height_m, snapshot.lcl_m(1, c), base_agl_m, layer_m,
                        1e3 * snapshot.added_liquid_path_kg_m2(1, c));
            // ERA5's tcc is maximum-random overlap of the 137 model levels. The 37 pressure levels' fractions averaged onto
            // 32 layers fall below it (where one cloud spans several levels at different horizontal places the levels'
            // fractions no longer add up: 0.0 .. 0.36 below, mean 0.14, before static clouds Task 9c). Re-baselined for
            // Task 9c: the build scales the layers' fractions to tcc (scale_cloud_cover_to_total), so the maximum-random
            // cover is tcc up to the 8-bit fraction codes (measured within 0.004); the renderer's maximum overlap stays
            // below it.
            deficit_sum += col.total_cloud_cover - cover_max_random;
            CHECK(std::abs(cover_max_random - col.total_cloud_cover) <= 0.02);
            CHECK(cover <= cover_max_random + 1e-9);
            // Re-baselined for the cloud bases at the LCL (static clouds, Task 7): a boundary-layer cloud (base below 3 km)
            // starts no more than one layer below the LCL of the column's 2 m air (the layer that holds the LCL is the
            // lowest one with cloud). The tolerance against ERA5's cbh below is unchanged and still holds with the LCL
            // applied (all ten columns pass it); the data's base stays wherever it lies above the LCL.
            const double lcl_agl_m = snapshot.lcl_m(1, c);
            if (lcl_agl_m > 0.0 && lcl_agl_m < constants::cloud_boundary_layer_top_m &&
                base_agl_m < constants::cloud_boundary_layer_top_m) {
                CHECK(base_agl_m >= lcl_agl_m - layer_m);
            }
            if (std::isnan(col.cloud_base_height_m)) {
                CHECK(std::isnan(base_agl_m));
            } else {
                REQUIRE_FALSE(std::isnan(base_agl_m));
                // Within one layer plus the spacing of the pressure levels around ERA5's cbh (the snapshot cannot place a cloud
                // base finer than the levels it interpolates between: 25 hPa ~ 250 m near the ground, 50 hPa ~ 600 m at 3 km).
                const auto z = hypsometric_level_heights_gpm(levels_pa, col.temperature_k, col.total_column_water_vapour_kg_m2,
                                                             col.surface_pressure_pa, col.surface_geopotential_m2_s2);
                const double ground_gpm = col.surface_geopotential_m2_s2 / constants::standard_gravity_mps2;
                const double target_gpm = ground_gpm + col.cloud_base_height_m;
                double spacing_m = 0.0, lower_gpm = ground_gpm;  // below the lowest level: from the ground
                for (int k = levels - 1; k >= 0; --k) {           // bottom-up over the levels above the ground
                    if (std::isnan(z[static_cast<std::size_t>(k)])) continue;
                    const double upper_gpm = z[static_cast<std::size_t>(k)];
                    if (upper_gpm >= target_gpm || k == 0) {
                        spacing_m = upper_gpm - lower_gpm;
                        break;
                    }
                    lower_gpm = upper_gpm;
                }
                INFO("level spacing " << spacing_m << " m");
                CHECK(std::abs(base_agl_m - col.cloud_base_height_m) <= layer_m + spacing_m);
            }
        }
    }
    // Before Task 9c: 0.14 (the systematic part of the pressure-level limit); scaled to tcc: within the codes.
    CHECK(std::abs(deficit_sum / static_cast<double>(cells.size())) <= 0.03);
}

// ---- Cloud bases at the lifting condensation level (static clouds, Task 7).

namespace {

// A column of layer means: liquid cloud (fraction f, density d) on the layers [first, last].
struct LayerColumn {
    std::array<double, weather_layer_count> fraction{}, liquid{}, ice{};
    void cloud(int first, int last, double f, double d) {
        for (int k = first; k <= last; ++k) {
            fraction[static_cast<std::size_t>(k)] = f;
            liquid[static_cast<std::size_t>(k)] = d;
        }
    }
    double liquid_path(double ground_m) const {
        double path = 0.0;
        for (int k = 0; k < weather_layer_count; ++k) {
            path += liquid[static_cast<std::size_t>(k)] *
                    std::max(0.0, weather_layer_bottom_m(k + 1) - std::max(weather_layer_bottom_m(k), ground_m));
        }
        return path;
    }
    void raise(double ground_m, double lcl_m) { raise_liquid_cloud_to_lcl(fraction, liquid, ice, ground_m, lcl_m); }
};

}  // namespace

TEST_CASE("Liquid cloud from the ground up is lifted to the lifting condensation level", "[weather]") {
    LayerColumn column;
    column.cloud(0, 7, 0.6, 3e-4);  // 0 .. 2 km
    const double path = column.liquid_path(0.0);
    column.raise(0.0, 800.0);
    // No condensate under 800 m: layers 0..2 (to 750 m) are empty, layer 3 (750 .. 1000 m) keeps the part above 800 m.
    for (int k = 0; k < 3; ++k) {
        CHECK(column.liquid[static_cast<std::size_t>(k)] == 0.0);
        CHECK(column.fraction[static_cast<std::size_t>(k)] == 0.0);
    }
    CHECK(column.liquid[3] > 0.0);
    CHECK(column.liquid[3] < column.liquid[4]);  // 200 of its 250 m remain, scaled alike
    CHECK(column.liquid[3] / column.liquid[4] == Catch::Approx(0.8).epsilon(1e-12));
    // The column's water path is kept, the profile above is scaled alike, the covered area does not shrink.
    CHECK(column.liquid_path(0.0) == Catch::Approx(path).epsilon(1e-6));
    CHECK(column.liquid[5] / column.liquid[4] == Catch::Approx(1.0).epsilon(1e-12));
    for (int k = 3; k < 8; ++k) CHECK(column.fraction[static_cast<std::size_t>(k)] == Catch::Approx(0.6));
    for (int k = 8; k < weather_layer_count; ++k) CHECK(column.liquid[static_cast<std::size_t>(k)] == 0.0);
}

TEST_CASE("The lifting condensation level is measured from the ground, not the ellipsoid", "[weather]") {
    LayerColumn column;
    column.cloud(8, 15, 0.5, 2e-4);  // 2 .. 4 km, over a ground at 1500 m: the data base is 500 m above the ground
    const double ground = 1500.0, path = column.liquid_path(ground);
    column.raise(ground, 1000.0);  // LCL at 2500 m above the ellipsoid
    CHECK(column.liquid[8] == 0.0);
    CHECK(column.liquid[9] == 0.0);  // 2250 .. 2500 m
    CHECK(column.liquid[10] > 0.0);  // 2500 .. 2750 m
    CHECK(column.liquid_path(ground) == Catch::Approx(path).epsilon(1e-6));
    // The same LCL taken from the ellipsoid (ground 0): the data base 2000 m is above it, nothing moves.
    LayerColumn wrong;
    wrong.cloud(8, 15, 0.5, 2e-4);
    wrong.raise(0.0, 1000.0);
    CHECK(wrong.liquid[8] == 2e-4);
}

TEST_CASE("A cloud wholly under the lifting condensation level is lifted keeping depth and density", "[weather]") {
    LayerColumn column;
    column.cloud(0, 1, 0.8, 4e-4);  // 0 .. 500 m
    const double path = column.liquid_path(0.0);
    column.raise(0.0, 1000.0);
    CHECK(column.liquid_path(0.0) == Catch::Approx(path).epsilon(1e-6));
    for (int k = 0; k < 4; ++k) CHECK(column.liquid[static_cast<std::size_t>(k)] == 0.0);  // 0 .. 1 km
    CHECK(column.liquid[4] == Catch::Approx(4e-4).epsilon(1e-12));  // 1000 .. 1250 m
    CHECK(column.liquid[5] == Catch::Approx(4e-4).epsilon(1e-12));  // 1250 .. 1500 m
    CHECK(column.liquid[6] == 0.0);
    CHECK(column.fraction[4] == Catch::Approx(0.8));
}

TEST_CASE("Boundary-layer cloud under the level: a deck far above keeps its water, a continuation takes it",
          "[weather]") {
    // Low cloud 0 .. 500 m, LCL 800 m (depth 500 m: the cloud lifts to 800 .. 1300 m).
    LayerColumn apart;
    apart.cloud(0, 1, 0.8, 4e-4);
    apart.cloud(8, 9, 0.5, 2e-4);  // a deck 2.0 .. 2.5 km: another cloud
    const double path = apart.liquid_path(0.0);
    apart.raise(0.0, 800.0);
    CHECK(apart.liquid_path(0.0) == Catch::Approx(path).epsilon(1e-6));
    CHECK(apart.liquid[8] == 2e-4);  // unchanged
    CHECK(apart.liquid[9] == 2e-4);
    CHECK(apart.fraction[8] == 0.5);
    for (int k : {0, 1, 2}) CHECK(apart.liquid[static_cast<std::size_t>(k)] == 0.0);
    CHECK(apart.liquid[3] > 0.0);  // 800 .. 1000 m
    CHECK(apart.liquid[4] > 0.0);  // 1000 .. 1250 m
    CHECK(apart.liquid[5] > 0.0);  // 1250 .. 1300 m
    for (int k = 6; k < 8; ++k) CHECK(apart.liquid[static_cast<std::size_t>(k)] == 0.0);
    // A run starting at 1000 m (within LCL + depth = 1300 m) is the same cloud: it takes the water.
    LayerColumn joined;
    joined.cloud(0, 1, 0.8, 4e-4);
    joined.cloud(4, 5, 0.5, 2e-4);
    const double joined_path = joined.liquid_path(0.0);
    joined.raise(0.0, 800.0);
    CHECK(joined.liquid_path(0.0) == Catch::Approx(joined_path).epsilon(1e-6));
    CHECK(joined.liquid[4] > 2e-4);
    CHECK(joined.liquid[3] == 0.0);
}

TEST_CASE("Clouds the lifting condensation level does not govern stay as they are", "[weather]") {
    const auto unchanged = [](auto setup, double ground, double lcl) {
        LayerColumn column;
        setup(column);
        const LayerColumn before = column;
        column.raise(ground, lcl);
        CHECK(column.fraction == before.fraction);
        CHECK(column.liquid == before.liquid);
        CHECK(column.ice == before.ice);
    };
    // Saturated surface air (LCL 0: fog), unknown level, a level above the boundary layer.
    unchanged([](LayerColumn& c) { c.cloud(0, 7, 0.6, 3e-4); }, 0.0, 0.0);
    unchanged([](LayerColumn& c) { c.cloud(0, 7, 0.6, 3e-4); }, 0.0, weather_lcl_unknown_m);
    unchanged([](LayerColumn& c) { c.cloud(0, 7, 0.6, 3e-4); }, 0.0, std::nan(""));
    unchanged([](LayerColumn& c) { c.cloud(0, 7, 0.6, 3e-4); }, 0.0, 3500.0);
    // A base at or above the LCL stays where the data has it.
    unchanged([](LayerColumn& c) { c.cloud(4, 9, 0.6, 3e-4); }, 0.0, 1000.0);
    unchanged([](LayerColumn& c) { c.cloud(6, 9, 0.6, 3e-4); }, 0.0, 800.0);
    // A cloud whose base is above the boundary layer (3 km above the ground).
    unchanged([](LayerColumn& c) { c.cloud(12, 20, 0.6, 3e-4); }, 0.0, 2900.0);
    // An ice layer at 8 km (a cirrus deck) and no liquid at all.
    unchanged([](LayerColumn& c) {
        for (int k = 20; k < 22; ++k) {
            c.fraction[static_cast<std::size_t>(k)] = 0.9;
            c.ice[static_cast<std::size_t>(k)] = 2e-5;
        }
    }, 0.0, 800.0);
    // Ice beside the boundary-layer liquid is kept; its cloud fraction survives where liquid is cut.
    LayerColumn mixed;
    mixed.cloud(0, 7, 0.6, 3e-4);
    mixed.ice[1] = 1e-5;
    const auto ice_before = mixed.ice;
    mixed.raise(0.0, 800.0);
    CHECK(mixed.ice == ice_before);
    CHECK(mixed.fraction[1] == 0.6);
    CHECK(mixed.liquid[1] == 0.0);
}

TEST_CASE("The built snapshot raises boundary-layer cloud to the level and keeps fog", "[weather]") {
    const int rows = 5, columns = 8;
    const auto& base_column = era5_columns().by_name("starbase");
    auto fields = era5_fields(rows, columns, [&](int, int) -> const Era5Column& { return base_column; });
    // A liquid cloud on the lowest levels of the column: the layers from the ground to ~1.5 km.
    const int levels = static_cast<int>(fields.levels_pa.size());
    for (int k = 0; k < levels; ++k) {
        const double p = fields.levels_pa[static_cast<std::size_t>(k)];
        const bool low = p >= 85000.0 && p <= base_column.surface_pressure_pa;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < columns; ++c) {
                set(fields.cloud_fraction, k, r, c, low ? 0.7 : 0.0);
                set(fields.liquid_kg_kg, k, r, c, low ? 3e-4 : 0.0);
                set(fields.ice_kg_kg, k, r, c, 0.0);
            }
        }
    }
    const auto plain = WeatherSnapshot::from_era5(era5_id(), fields, coarse_geoid());
    // Columns 0-3: 2 m spread 7 K (LCL ~ 870 m), columns 4-7: 0.5 K (fog).
    fields.temperature_2m_k = fields.dewpoint_2m_k = grid(1, rows, columns);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < columns; ++c) {
            set(fields.temperature_2m_k, 0, r, c, 295.0);
            set(fields.dewpoint_2m_k, 0, r, c, c < 4 ? 288.0 : 294.5);
        }
    }
    const auto snapshot = WeatherSnapshot::from_era5(era5_id(), fields, coarse_geoid());
    const int r = 2;
    const std::size_t cells = static_cast<std::size_t>(rows) * columns;
    const auto liquid_at = [&](const WeatherSnapshot& s, int layer, int c) {
        return decode_weather_water(s.texels()[(static_cast<std::size_t>(layer) * cells + r * columns + c) * 4 + 1]);
    };
    const auto path = [&](const WeatherSnapshot& s, int c) {
        double sum = 0.0;
        const double ground = s.ground_heights_m()[static_cast<std::size_t>(r * columns + c)];
        for (int k = 0; k < weather_layer_count; ++k) {
            sum += liquid_at(s, k, c) *
                   std::max(0.0, weather_layer_bottom_m(k + 1) - std::max(weather_layer_bottom_m(k), ground));
        }
        return sum;
    };
    const double lcl = snapshot.lcl_m(r, 1);
    const double ground = snapshot.ground_heights_m()[static_cast<std::size_t>(r * columns + 1)];
    REQUIRE(lcl > 700.0);
    REQUIRE(lcl < 1100.0);
    CHECK(path(plain, 1) > 0.05);
    // The data had cloud at the ground; the built snapshot has none in the layers wholly under the LCL.
    CHECK(liquid_at(plain, 0, 1) > 0.0);
    for (int k = 0; k < weather_layer_count; ++k) {
        if (weather_layer_bottom_m(k + 1) <= ground + lcl) CHECK(liquid_at(snapshot, k, 1) == 0.0);
    }
    // The liquid path is kept (the 8-bit codes cost <= 1.97 % per layer).
    CHECK(path(snapshot, 1) == Catch::Approx(path(plain, 1)).epsilon(0.03));
    // Fog: 2 m spread under cloud_fog_spread_k: the level is stored as 0 and the column is as without the 2 m fields.
    CHECK(snapshot.lcl_m(r, 6) == 0.0f);
    for (int k = 0; k < weather_layer_count; ++k) CHECK(liquid_at(snapshot, k, 6) == liquid_at(plain, k, 6));
}

TEST_CASE("An unknown level is skipped by the averages and leaves the cloud alone", "[weather]") {
    const int rows = 9, columns = 8;
    const auto& cols = era5_columns().columns;
    auto fields = era5_fields(rows, columns, [&](int, int) -> const Era5Column& { return cols[0]; });
    fields.temperature_2m_k = fields.dewpoint_2m_k = grid(1, rows, columns);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < columns; ++c) {
            // All but the first two columns have no valid dew point.
            set(fields.temperature_2m_k, 0, r, c, 300.0);
            set(fields.dewpoint_2m_k, 0, r, c, c < 2 ? 290.0 : std::nan(""));
        }
    }
    const auto snapshot = WeatherSnapshot::from_era5(era5_id(), fields, coarse_geoid());
    CHECK(snapshot.lcl_m(4, 5) == weather_lcl_unknown_m);
    CHECK(snapshot.lcl_m(4, 1) > 0.0f);
    // Poles: the mean of the known columns only.
    CHECK(snapshot.lcl_m(0, 0) == snapshot.lcl_m(0, 7));
    CHECK(snapshot.lcl_m(0, 0) > 0.0f);
    // A box over known and unknown columns: the known ones' lowest base, not diluted by the unknown.
    const auto coarse = snapshot.downsampled(4);
    CHECK(coarse.lcl_m(1, 0) == Catch::Approx(snapshot.lcl_m(4, 1)).epsilon(0.05));
    // A box over unknown columns only stays unknown.
    CHECK(coarse.lcl_m(1, 1) == weather_lcl_unknown_m);
}

TEST_CASE("GFS takes the standard pressure at the column's ground height for the level", "[weather]") {
    // US Standard Atmosphere 1976: 84 559.9 Pa at 1500 m.
    CHECK(standard_pressure_at_height_pa(0.0) == Catch::Approx(101325.0));
    CHECK(standard_pressure_at_height_pa(1500.0) == Catch::Approx(84559.9).epsilon(2e-4));
    const int rows = 3, columns = 4, levels = 2;
    GfsFields f;
    f.levels_pa = {70000.0, 50000.0};
    f.cloud_percent = f.liquid_kg_kg = f.ice_kg_kg = grid(levels, rows, columns);
    f.temperature_k = f.geopotential_height_m = grid(levels, rows, columns);
    f.surface_geopotential_height_m = f.temperature_2m_k = f.dewpoint_2m_k = grid(1, rows, columns);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < columns; ++c) {
            set(f.temperature_k, 0, r, c, 275.0);
            set(f.temperature_k, 1, r, c, 255.0);
            set(f.geopotential_height_m, 0, r, c, 3100.0);
            set(f.geopotential_height_m, 1, r, c, 5600.0);
            set(f.surface_geopotential_height_m, 0, r, c, 1500.0);
            set(f.temperature_2m_k, 0, r, c, 290.0);
            set(f.dewpoint_2m_k, 0, r, c, 280.0);
        }
    }
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::gfs;
    const auto snapshot = WeatherSnapshot::from_gfs(id, f, coarse_geoid());
    const double expected = lifting_condensation_level_m(290.0, 280.0, standard_pressure_at_height_pa(1500.0));
    CHECK(snapshot.lcl_m(1, 1) == Catch::Approx(expected).epsilon(1e-5));
    CHECK(expected != Catch::Approx(lifting_condensation_level_m(290.0, 280.0, 101325.0)).margin(0.5));
}

TEST_CASE("All liquid under the level is moved, every run, into the cloud above it", "[weather]") {
    LayerColumn column;
    column.cloud(0, 0, 0.3, 1e-4);  // 0 .. 250 m: a first run
    column.cloud(2, 7, 0.6, 3e-4);  // 500 .. 2000 m
    const double path = column.liquid_path(0.0);
    column.raise(0.0, 800.0);
    for (int k = 0; k < 3; ++k) {
        CHECK(column.liquid[static_cast<std::size_t>(k)] == 0.0);
        CHECK(column.fraction[static_cast<std::size_t>(k)] == 0.0);
    }
    CHECK(column.liquid_path(0.0) == Catch::Approx(path).epsilon(1e-6));
    CHECK(column.liquid[3] / column.liquid[4] == Catch::Approx(0.8).epsilon(1e-12));  // 200 of 250 m kept
    CHECK(column.liquid[5] / column.liquid[4] == Catch::Approx(1.0).epsilon(1e-12));
    for (int k = 3; k < 8; ++k) CHECK(column.fraction[static_cast<std::size_t>(k)] >= 0.6);
}

TEST_CASE("A trace under the cloud is no cloud base but is moved up with the rest", "[weather]") {
    // A trace below the smallest water code on 0 .. 500 m, the cloud from 1000 m (base above the level 800 m).
    LayerColumn column;
    column.cloud(0, 1, 0.5, 1e-10);
    column.cloud(4, 7, 0.6, 3e-4);
    const double path = column.liquid_path(0.0);
    column.raise(0.0, 800.0);
    CHECK(column.liquid[0] == 0.0);
    CHECK(column.liquid[1] == 0.0);
    CHECK(column.fraction[0] == 0.0);
    CHECK(column.liquid_path(0.0) == Catch::Approx(path).epsilon(1e-6));
    CHECK(column.liquid[4] == Catch::Approx(3e-4).epsilon(1e-6));  // the cloud itself barely changes
    CHECK(column.liquid[5] / column.liquid[4] == Catch::Approx(1.0).epsilon(1e-12));
    CHECK(column.fraction[4] == 0.6);
    // Water but a cloud fraction under cloud_min_cover on 0 .. 250 m: the cloud from 500 m is the boundary-layer one.
    LayerColumn thin;
    thin.cloud(0, 0, 0.001, 1e-4);
    thin.cloud(2, 7, 0.6, 3e-4);
    const double thin_path = thin.liquid_path(0.0);
    thin.raise(0.0, 800.0);
    for (int k = 0; k < 3; ++k) CHECK(thin.liquid[static_cast<std::size_t>(k)] == 0.0);
    CHECK(thin.liquid_path(0.0) == Catch::Approx(thin_path).epsilon(1e-6));
    // Only traces in the column: no significant cloud, nothing moves.
    LayerColumn trace;
    trace.cloud(0, 3, 0.5, 1e-10);
    const LayerColumn trace_before = trace;
    trace.raise(0.0, 800.0);
    CHECK(trace.liquid == trace_before.liquid);
}

TEST_CASE("A level at or above the weather top leaves the column alone and draws no base", "[weather]") {
    LayerColumn column;
    column.cloud(0, 7, 0.6, 3e-4);
    const LayerColumn before = column;
    for (double lcl : {weather_top_m, 25000.0}) {
        column.raise(0.0, lcl);
        CHECK(column.liquid == before.liquid);
        CHECK(column.fraction == before.fraction);
    }
    // A built column with very dry 2 m air: its level is capped at the weather top, its cloud not cut.
    REQUIRE(lifting_condensation_level_m(340.0, 120.0) > weather_top_m);
    const auto dry = WeatherSnapshot::gfs_test_column(5, 8, 2000.0, 3e-4, 70.0, 340.0, 120.0);
    CHECK(dry.lcl_m(2, 3) == static_cast<float>(weather_top_m));
    CHECK(cloud_render_base_texels(dry)[2 * (2 * 8 + 3) + 1] == 0.0f);
}

TEST_CASE("A coarse cell takes the lowest known base of its box", "[weather]") {
    const int rows = 9, columns = 8;
    std::vector<std::uint8_t> texels(static_cast<std::size_t>(weather_layer_count) * rows * columns * 4, 0);
    std::vector<float> ground(static_cast<std::size_t>(rows) * columns, 100.0f);
    std::vector<float> lcl(static_cast<std::size_t>(rows) * columns, 1500.0f);
    const auto at = [&](int r, int c) -> float& { return lcl[static_cast<std::size_t>(r) * columns + c]; };
    // Box of coarse cell (1, 0): rows 2..6, columns 6, 7, 0, 1, 2. One column at 500 m, one NaN (skipped).
    at(4, 1) = 500.0f;
    at(5, 0) = std::nanf("");
    // Box of coarse cell (1, 1): columns 2..6. A saturated column (fog): the box is saturated (no cut).
    at(3, 4) = 0.0f;
    // Rows 0 and 8 belong to the pole boxes only; a higher ground beside the low base: the lowest base height counts.
    const auto snapshot = WeatherSnapshot::from_parts({}, rows, columns, ground, texels, lcl);
    const auto coarse = snapshot.downsampled(4);
    CHECK(coarse.lcl_m(1, 0) == Catch::Approx(500.0).margin(1e-3));
    CHECK(coarse.lcl_m(1, 1) == 0.0f);
    // The lowest base height above the box's mean ground: a 400 m higher ground under a 1500 m level elsewhere.
    std::vector<float> hilly = ground;
    for (int r = 2; r <= 6; ++r) hilly[static_cast<std::size_t>(r) * columns + 6] = 500.0f;  // column 6 of box (1, 0)
    std::vector<float> plain_lcl(lcl.size(), 1500.0f);
    const auto hill = WeatherSnapshot::from_parts({}, rows, columns, hilly, texels, plain_lcl).downsampled(4);
    const double box_ground = hill.ground_heights_m()[static_cast<std::size_t>(1) * hill.columns()];
    CHECK(box_ground > 100.0);
    CHECK(hill.lcl_m(1, 0) == Catch::Approx(100.0 + 1500.0 - box_ground).margin(1e-3));
}

TEST_CASE("A snapshot built from fields gets the flat base and keeps the column's optical depth", "[weather]") {
    // 2 m spread 7 K (LCL ~ 870 m) against a saturated surface (0.5 K: fog, the column as the data has it).
    const auto raised = WeatherSnapshot::gfs_test_column(5, 8, 2000.0, 3e-4, 70.0, 295.0, 288.0);
    const auto fog = WeatherSnapshot::gfs_test_column(5, 8, 2000.0, 3e-4, 70.0, 295.0, 294.5);
    const int r = 2, c = 3;
    const std::size_t cell = static_cast<std::size_t>(r) * 8 + c;
    const double lcl = raised.lcl_m(r, c), ground = raised.ground_heights_m()[cell];
    REQUIRE(lcl > 700.0);
    REQUIRE(lcl < 1100.0);
    CHECK(fog.lcl_m(r, c) == 0.0f);
    const std::size_t cells = static_cast<std::size_t>(5) * 8;
    for (int k = 0; k < weather_layer_count; ++k) {
        const std::size_t i = (static_cast<std::size_t>(k) * cells + cell) * 4;
        if (weather_layer_bottom_m(k + 1) <= ground + lcl) {
            CHECK(raised.texels()[i + 1] == 0);
            CHECK(raised.texels()[i] == 0);
        }
    }
    CHECK(fog.texels()[cell * 4 + 1] > 0);  // the fog column has liquid at the ground
    // The column's optical depth is kept (the 8-bit water codes: <= 1.97 % per layer).
    // Static clouds Task 9c: plus the cumulus water from the adiabatic parcel (the fog column has no convective base),
    // the liquid path's extinction 3 W / (2 rho_w r_eff).
    const double tau = cloud_render_column_optical_depth(raised, r, c);
    CHECK(tau > 1.0);
    CHECK(fog.added_liquid_path_kg_m2(r, c) == 0.0f);
    const double added_tau = 1.5 * raised.added_liquid_path_kg_m2(r, c) /
                             (constants::liquid_water_density_kg_m3 * constants::cloud_droplet_effective_radius_m);
    CHECK(tau == Catch::Approx(cloud_render_column_optical_depth(fog, r, c) + added_tau).epsilon(0.03));
    // The base texel: the level above the ground; the fog column has none.
    const auto base = cloud_render_base_texels(raised);
    CHECK(base[2 * cell] == Catch::Approx(ground + lcl));
    CHECK(base[2 * cell + 1] > 1.0f);
    CHECK(cloud_render_base_texels(fog)[2 * cell + 1] == 0.0f);
}
