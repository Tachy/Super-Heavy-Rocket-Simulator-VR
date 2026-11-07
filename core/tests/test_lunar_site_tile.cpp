// Lunar site tiles: request planning (product choice, byte ranges), assembly from the range bodies (file seams, the
// 0/360 wrap, albedo relative to its own low-pass) and the tile file round trip. The bodies are synthetic: a smooth
// function of the product's own pixel grid, written where the request says the bytes are.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <set>
#include <thread>

#include "starsim/config/lunar_site_tile.hpp"
#include "starsim/environment/lunar_tile.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/report/report.hpp"

using namespace starsim;
using config::LunarTerrainProduct;
using config::SiteTileRequest;
using Bodies = std::vector<std::vector<std::uint8_t>>;
using PixelFunction = std::function<double(long long line, long long sample)>;  // product grid, metres or albedo

namespace {

constexpr double deg = constants::deg_to_rad;

const LunarTerrainProduct& product_of(const std::vector<LunarTerrainProduct>& all, const std::string& id) {
    return *std::find_if(all.begin(), all.end(), [&](const auto& p) { return p.id == id; });
}

// Product grid position (line, sample) of a file pixel: polar files are the grid, cylindrical files sit by their
// first latitude / longitude.
std::pair<long long, long long> grid_position(const LunarTerrainProduct& p, std::size_t file, long long row, long long col) {
    const auto& f = p.files[file];
    if (p.projection != "simple_cylindrical") return {row, col};
    return {std::llround((p.latitude_max_deg - f.first_latitude_deg) * f.pixels_per_degree) + row,
            std::llround(f.first_longitude_deg * f.pixels_per_degree) + col};
}

Bodies synthetic_bodies(const SiteTileRequest& r, const LunarTerrainProduct& p, const PixelFunction& value) {
    const std::uint64_t bpp = p.sample_type == "int16" ? 2 : 4;
    Bodies bodies;
    for (const auto& range : r.ranges) {
        const auto& f = p.files[range.file_index];
        std::vector<std::uint8_t> body(range.last - range.first + 1, 0);
        for (std::uint64_t at = 0; at + bpp <= body.size(); at += bpp) {
            const std::uint64_t absolute = range.first + at;
            if (absolute < f.header_bytes) continue;
            const std::uint64_t position = absolute - f.header_bytes;
            const long long row = static_cast<long long>(position / f.record_bytes);
            const long long col = static_cast<long long>(position % f.record_bytes / bpp);
            if (row >= f.lines || col >= f.samples) continue;
            const auto [line, sample] = grid_position(p, range.file_index, row, col);
            const double v = (value(line, sample) - p.offset) / p.scale;
            if (bpp == 2) {
                const auto raw = static_cast<std::int16_t>(std::llround(v));
                std::memcpy(&body[at], &raw, 2);
            } else {
                const auto raw = static_cast<float>(v);
                std::memcpy(&body[at], &raw, 4);
            }
        }
        bodies.push_back(std::move(body));
    }
    return bodies;
}

std::filesystem::path unique_dir(const char* name) {
    const auto dir = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

PixelFunction height_function(const LunarTerrainProduct& p) {
    if (p.projection == "simple_cylindrical") {
        const double width = 360.0 * p.files.front().pixels_per_degree;
        return [width](long long l, long long s) {
            return 900.0 * std::sin(constants::two_pi * static_cast<double>(s) / width) + 0.05 * static_cast<double>(l);
        };
    }
    return [](long long l, long long s) {
        return 300.0 * std::sin(0.01 * static_cast<double>(l)) + 200.0 * std::cos(0.013 * static_cast<double>(s));
    };
}

}  // namespace

TEST_CASE("Lunar site tile: plan picks SLDEM at Apollo 15 and the north polar product at 85 N",
          "[config][lunar_site_tile]") {
    const auto products = config::load_lunar_terrain_products(STARSIM_CONFIG_DIR);
    REQUIRE(!products.empty());
    CHECK(products.front().id == "ldem_80s_20m");
    CHECK(config::plan_site_tile(products, "height", 26.1322 * deg, 3.6339 * deg).product_id == "sldem2015_512");
    const auto north = config::plan_site_tile(products, "height", 85.0 * deg, 30.0 * deg);
    CHECK(north.product_id == "ldem_80n_20m");
    CHECK(north.projection == "polar_stereographic_north");
    CHECK(north.half_size_m == 10000.0);  // the polar 20 m products carry their own half size (ruling R1)
    CHECK(config::plan_site_tile(products, "height", -84.7906 * deg, 0.0).product_id == "ldem_80s_20m");
    // The seam: SLDEM2015 ends and the 60 m polar product starts at 60 deg; the 100 m product holds the tile.
    CHECK(config::plan_site_tile(products, "height", 60.0 * deg, 20.0 * deg).product_id == "ldem_45n_100m");
    CHECK(config::plan_site_tile(products, "height", -70.0 * deg, 20.0 * deg).product_id == "ldem_60s_60m");
    CHECK(config::plan_site_tile(products, "albedo", 26.1322 * deg, 3.6339 * deg).product_id == "wac_emp_643nm_equatorial");
    CHECK(config::plan_site_tile(products, "albedo", -75.0 * deg, 20.0 * deg).product_id == "wac_emp_643nm_south");
    CHECK(config::plan_site_tile(products, "albedo", 75.0 * deg, 20.0 * deg).product_id == "wac_emp_643nm_north");
}

TEST_CASE("Lunar site tile: cache key", "[config][lunar_site_tile]") {
    const auto products = config::load_lunar_terrain_products(STARSIM_CONFIG_DIR);
    CHECK(config::plan_site_tile(products, "height", 26.1322 * deg, 3.6339 * deg).cache_key ==
          "height_sldem2015_512_2613_363_75");
    CHECK(config::plan_site_tile(products, "height", -84.7906 * deg, -20.0 * deg).cache_key ==
          "height_ldem_80s_20m_-8479_34000_10");
}

TEST_CASE("Lunar site tile: ranges cover exactly the crop rows, merged when adjacent", "[config][lunar_site_tile]") {
    const auto products = config::load_lunar_terrain_products(STARSIM_CONFIG_DIR);
    const auto r = config::plan_site_tile(products, "height", 26.1322 * deg, 3.6339 * deg);
    // 75 km at 512 px/deg (59.2 m): 2 x 1266 + 1 lines; one range per row (the crop is narrower than a record)
    CHECK(r.lines == 2533);
    CHECK(r.ranges.size() == static_cast<size_t>(r.lines));
    CHECK(r.segments.size() == static_cast<size_t>(r.lines));
    std::set<std::uint64_t> rows;
    for (const auto& b : r.ranges) {
        CHECK(b.file_index == 0);
        CHECK(b.last - b.first + 1 == static_cast<std::uint64_t>(r.samples) * 4);  // float32 pixels of one row
        CHECK(b.first % 92160 + (b.last - b.first + 1) <= 92160);                   // inside one record
        rows.insert(b.first / 92160);
        CHECK(b.url.find("sldem2015_512_00n_30n_000_045_float.img") != std::string::npos);
    }
    CHECK(rows.size() == static_cast<size_t>(r.lines));

    // With a gap allowance the rows join into blocks (at most max_block_bytes; the segments keep their row).
    config::RangePolicy merge;
    merge.max_gap_bytes = 1 << 20;
    merge.max_block_bytes = 4 << 20;
    const auto m = config::plan_site_tile(products, "height", 26.1322 * deg, 3.6339 * deg, merge);
    CHECK(m.ranges.size() < 100);
    CHECK(m.ranges.size() >= 2533ull * 92160 / (4 << 20));
    for (const auto& b : m.ranges) CHECK(b.last - b.first + 1 <= (4u << 20));
    for (std::size_t i = 0; i < m.segments.size(); ++i) {
        const auto& seg = m.segments[i];
        const auto& b = m.ranges[seg.range_index];
        CHECK(b.first + seg.body_offset == r.ranges[i].first);  // the same bytes as the unmerged plan
    }
}

TEST_CASE("Lunar site tile: albedo rows merge into large blocks", "[config][lunar_site_tile]") {
    const auto products = config::load_lunar_terrain_products(STARSIM_CONFIG_DIR);
    const auto r = config::plan_site_tile(products, "albedo", 26.1322 * deg, 3.6339 * deg);
    CHECK(r.lines == 1505);
    const auto policy = config::default_range_policy("albedo");
    CHECK(r.ranges.size() >= 6);
    CHECK(r.ranges.size() <= 10);  // not ~1500 requests: the LROC host blocks after that many
    std::uint64_t total = 0;
    for (const auto& b : r.ranges) {
        CHECK(b.last - b.first + 1 <= policy.max_block_bytes);
        total += b.last - b.first + 1;
    }
    CHECK(total >= static_cast<std::uint64_t>(r.lines - 1) * 109440 - r.ranges.size() * 109440);  // whole rows, ~165 MB
    CHECK(total <= 170'000'000);
}

TEST_CASE("Lunar site tile: site tile across a file seam and the 0/360 wrap", "[config][lunar_site_tile]") {
    const auto products = config::load_lunar_terrain_products(STARSIM_CONFIG_DIR);
    for (const double lon_deg : {45.0, 0.0}) {
        CAPTURE(lon_deg);
        const auto r = config::plan_site_tile(products, "height", 10.0 * deg, lon_deg * deg);
        REQUIRE(r.product_id == "sldem2015_512");
        std::set<std::size_t> files;
        for (const auto& b : r.ranges) files.insert(b.file_index);
        CHECK(files.size() == 2);  // two files: 0/45 deg, or 315/360 and 0/45 (the wrap)
        CHECK(r.ranges.size() == 2 * static_cast<size_t>(r.lines));
        CHECK(r.segments.size() == 2 * static_cast<size_t>(r.lines));
        const auto& p = product_of(products, r.product_id);
        const auto value = height_function(p);
        const auto tile = config::assemble_site_tile(r, synthetic_bodies(r, p, value));
        REQUIRE(tile.projection == "simple_cylindrical");
        const long long first_line = std::llround((60.0 - r.first_latitude_rad / deg) * 512.0 - 0.5);
        const long long first_sample = std::llround(r.first_longitude_rad / deg * 512.0 - 0.5);
        const long long width = 360 * 512;
        double worst = 0.0;
        for (int l = 0; l < tile.lines; l += 7) {
            for (int s = 0; s < tile.samples; ++s) {
                const double expected = value(first_line + l, (first_sample + s) % width);
                const double got = tile.dn[static_cast<size_t>(l) * static_cast<size_t>(tile.samples) + static_cast<size_t>(s)] *
                                   tile.dn_scale_m;
                worst = std::max(worst, std::abs(got - expected));
            }
        }
        CHECK(worst <= 0.25 + 1e-3);  // DN rounding + float32 km
        // zero_line / zero_sample place pixel (0, 0) at the first latitude / longitude
        CHECK((tile.zero_line - 0.0) / 512.0 * deg == Catch::Approx(r.first_latitude_rad).margin(1e-12));
        CHECK(-tile.zero_sample / 512.0 * deg == Catch::Approx(r.first_longitude_rad).margin(1e-12));
    }
}

TEST_CASE("Lunar site tile: assemble then load_terrain_grid_file round-trips heights within 0.25 m",
          "[config][lunar_site_tile]") {
    const auto products = config::load_lunar_terrain_products(STARSIM_CONFIG_DIR);
    const auto dir = unique_dir("starsim_site_tile_roundtrip");
    struct Site {
        double lat_deg, lon_deg;
        const char* id;
    };
    for (const Site s : {Site{-84.7906, 0.0, "ldem_80s_20m"}, Site{85.0, 30.0, "ldem_80n_20m"},
                         Site{26.1322, 3.6339, "sldem2015_512"}}) {
        CAPTURE(s.id);
        const auto r = config::plan_site_tile(products, "height", s.lat_deg * deg, s.lon_deg * deg);
        REQUIRE(r.product_id == s.id);
        const auto& p = product_of(products, r.product_id);
        const auto value = height_function(p);
        const auto file = config::assemble_site_tile(r, synthetic_bodies(r, p, value));
        const auto path = dir / (r.cache_key + ".toml");
        config::write_terrain_grid_file(file, path);
        const auto back = config::load_terrain_grid_file(path);
        CHECK(back.kind == "height");
        CHECK(back.product == s.id);
        CHECK(back.projection == file.projection);
        CHECK(back.lines == file.lines);
        CHECK(back.samples == file.samples);
        CHECK(back.dn_scale_m == 0.5);
        CHECK(back.dn == file.dn);
        REQUIRE(back.checks.size() == 9);
        if (back.projection != "simple_cylindrical") {
            // polar: the grid is the product's, so the stored DN are the function at the crop's own positions
            CHECK(back.first_line == r.first_line);
            double worst = 0.0;
            for (int l = 0; l < back.lines; l += 11) {
                for (int c = 0; c < back.samples; c += 3) {
                    const double expected = value(back.first_line + l, back.first_sample + c);
                    worst = std::max(worst, std::abs(back.dn[static_cast<size_t>(l) * static_cast<size_t>(back.samples) +
                                                              static_cast<size_t>(c)] * back.dn_scale_m - expected));
                }
            }
            CHECK(worst <= 0.25 + 1e-3);  // DN rounding + float32 km
            // the check points are where the projection says they are: the grid position of their latitude / longitude
            // reads the same bilinear height (the north line sign is the one LunarTerrain uses)
            environment::PolarStereographicTile tile;
            tile.north = back.projection == "polar_stereographic_north";
            tile.sphere_radius_m = back.sphere_radius_m;
            tile.scale_m = back.scale_m;
            tile.pole_line = back.pole_line;
            tile.pole_sample = back.pole_sample;
            tile.first_line = back.first_line;
            tile.first_sample = back.first_sample;
            tile.grid = {back.lines, back.samples, back.dn_scale_m, back.dn};
            for (const auto& c : back.checks) {
                const auto [line, sample] = tile.position(c.latitude_rad, c.longitude_rad);
                REQUIRE(tile.inside(line, sample));
                CHECK(tile.grid.bilinear(line, sample) == Catch::Approx(c.height_m).margin(1e-6));
            }
        }
    }
    std::filesystem::remove_all(dir);
}

TEST_CASE("Lunar site tile: the tile write is atomic (temp files are not the tile, concurrent writers both complete)",
          "[config][lunar_site_tile]") {
    const auto dir = unique_dir("starsim_site_tile_atomic");
    std::filesystem::create_directories(dir);
    config::TerrainGridFile file;
    file.product = "synthetic";
    file.kind = "height";
    file.projection = "simple_cylindrical";
    file.sphere_radius_m = 1737400.0;
    file.dn_scale_m = 0.5;
    file.lines = 40;
    file.samples = 30;
    file.pixels_per_degree = 128.0;
    file.zero_line = 20.0;
    file.zero_sample = 15.0;
    file.dn.resize(1200);
    for (std::size_t i = 0; i < file.dn.size(); ++i) file.dn[i] = static_cast<std::int16_t>(i % 977);
    const auto path = dir / "tile.toml";

    // A crashed writer leaves temp files (a partial grid, a partial toml): the tile is not there, only the .toml counts.
    {
        std::ofstream(dir / "tile.i16.123.1.1.tmp", std::ios::binary) << "partial";
        std::ofstream(dir / "tile.toml.123.1.1.tmp", std::ios::binary) << "[grid]\nprod";
    }
    CHECK_FALSE(std::filesystem::exists(path));

    // Two workers on one tile: both finish without an error, the result is complete and readable.
    std::atomic<int> failures{0};
    std::vector<std::thread> workers;
    for (int w = 0; w < 4; ++w) {
        workers.emplace_back([&] {
            try {
                for (int k = 0; k < 10; ++k) config::write_terrain_grid_file(file, path);
            } catch (const std::exception&) {
                ++failures;
            }
        });
    }
    for (auto& t : workers) t.join();
    CHECK(failures == 0);
    const auto back = config::load_terrain_grid_file(path);
    CHECK(back.dn == file.dn);
    // only the final pair and the crashed writer's leftovers remain: this call's temp files are gone
    int leftovers = 0;
    for (const auto& e : std::filesystem::directory_iterator(dir)) {
        if (e.path().extension() == ".tmp" && e.path().filename().string().find(".123.1.1.") == std::string::npos) ++leftovers;
    }
    CHECK(leftovers == 0);
    std::filesystem::remove_all(dir);
}

TEST_CASE("Lunar site tile: albedo tile is relative: mean 1.0 within 1e-3 over the tile", "[config][lunar_site_tile]") {
    auto products = config::load_lunar_terrain_products(STARSIM_CONFIG_DIR);
    for (auto& p : products) {
        if (p.kind == "albedo") p.tile_half_size_m = 8000.0;  // small tile: the bodies stay small
    }
    const auto dir = unique_dir("starsim_site_tile_albedo");
    const auto r = config::plan_site_tile(products, "albedo", 10.0 * deg, 100.0 * deg);
    REQUIRE(r.product_id == "wac_emp_643nm_equatorial");
    CHECK(r.ranges.size() == 1);  // 161 rows join into one block
    const auto& p = product_of(products, r.product_id);
    const PixelFunction albedo = [](long long l, long long s) {
        return 0.12 + 0.03 * std::sin(0.02 * static_cast<double>(s)) + 0.00002 * static_cast<double>(l - 18000) +
               0.01 * std::sin(0.5 * static_cast<double>(l));
    };
    auto bodies = synthetic_bodies(r, p, albedo);
    // put the no-data constant in the tile's first pixel as well
    const float bad = -1.0e32f;
    std::memcpy(&bodies[r.segments[0].range_index][r.segments[0].body_offset], &bad, 4);
    const auto file = config::assemble_site_tile(r, bodies);
    const auto path = dir / "albedo.toml";
    config::write_terrain_grid_file(file, path);
    const auto back = config::load_terrain_grid_file(path);
    CHECK(back.kind == "albedo");
    CHECK(back.dn_scale_m == Catch::Approx(1.0 / 16384.0));
    CHECK(back.checks.empty());
    double sum = 0.0, low = 10.0, high = 0.0;
    for (const auto dn : back.dn) {
        const double v = dn * back.dn_scale_m;
        sum += v;
        low = std::min(low, v);
        high = std::max(high, v);
    }
    CHECK(sum / static_cast<double>(back.dn.size()) == Catch::Approx(1.0).margin(1e-3));
    CHECK(low > 0.5);   // relative values around 1, the no-data pixel is exactly 1.0
    CHECK(high < 1.5);
    CHECK(back.dn[0] == 16384);
    std::filesystem::remove_all(dir);
}

TEST_CASE("Lunar site tile: no product for the site refuses with lunar.site_no_product", "[config][lunar_site_tile]") {
    const auto products = config::load_lunar_terrain_products(STARSIM_CONFIG_DIR);
    // An albedo tile across 60 deg would need the equatorial and the polar mosaic (two projections): refused.
    try {
        config::plan_site_tile(products, "albedo", 60.0 * deg, 20.0 * deg);
        FAIL("expected a refusal");
    } catch (const report::Refusal& e) {
        CHECK(e.report().code == report::Code::lunar_site_no_product);
        CHECK(report::id_of(e.report().code) == "lunar.site_no_product");
    }
    // Without the products that hold the tile.
    auto only_polar = products;
    only_polar.erase(std::remove_if(only_polar.begin(), only_polar.end(),
                                    [](const auto& p) { return p.projection == "simple_cylindrical"; }),
                     only_polar.end());
    CHECK_THROWS_AS(config::plan_site_tile(only_polar, "height", 0.0, 0.0), report::Refusal);
    CHECK_THROWS_AS(config::plan_site_tile(products, "nothing", 0.0, 0.0), report::Refusal);
    // A polar tile that runs over the edge of its product (the 80 deg product ends at 80 deg)
    CHECK(config::plan_site_tile(products, "height", 80.05 * deg, 0.0).product_id != "ldem_80n_20m");
}

TEST_CASE("Lunar site tile: north polar tiles agree with LDEM_64", "[config][lunar_site_tile][data]") {
    const std::filesystem::path sites = std::filesystem::path(STARSIM_DATA_DIR) / "terrain" / "sites";
    const std::filesystem::path global_path = std::filesystem::path(STARSIM_DATA_DIR) / "terrain" / "ldem_64.toml";
    if (!std::filesystem::exists(sites) || !std::filesystem::exists(global_path)) SKIP("site tiles / LDEM_64 not installed");
    auto global_file = config::load_terrain_grid_file(global_path);
    environment::CylindricalGrid global;
    global.pixels_per_degree = global_file.pixels_per_degree;
    global.zero_line = global_file.zero_line;
    global.zero_sample = global_file.zero_sample;
    global.grid = {global_file.lines, global_file.samples, global_file.dn_scale_m, std::move(global_file.dn)};
    // Orientation limit: measured rms against LDEM_16 was 72-104 m (ldem_45n_100m, ldem_80n_20m, ldem_60n_60m, 2026-11-06,
    // fix round 1 of the lunar sites plan), against LDEM_64 11 m for the real 75 N tile (ldem_60n_60m, task 4); a wrong sign or axis choice gives 800-1950 m.
    constexpr double kOrientationRmsLimitM = 150.0;
    int checked = 0;
    for (const auto& entry : std::filesystem::directory_iterator(sites)) {
        if (entry.path().extension() != ".toml") continue;
        const auto file = config::load_terrain_grid_file(entry.path());
        if (file.projection != "polar_stereographic_north" || file.kind != "height") continue;
        double square = 0.0;
        for (const auto& c : file.checks) {
            const double d = c.height_m - global.height_m(c.latitude_rad, c.longitude_rad);
            square += d * d;
        }
        const double rms = std::sqrt(square / static_cast<double>(file.checks.size()));
        std::printf("[lunar_site_tile] %s: rms against LDEM_64 %.0f m\n", entry.path().filename().string().c_str(), rms);
        CHECK(rms < kOrientationRmsLimitM);
        ++checked;
    }
    if (checked == 0) SKIP("no north polar site tile downloaded");
}
