// Lunar terrain (lunar l2): polar stereographic tile, global grid, blend, normals; the real LOLA data against the
// pipeline's check points (raw DN, bilinear); the Moon reference body's surface.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/data_missing.hpp"
#include "starsim/config/lunar_site_config.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/lunar_tile.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/report/report.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using environment::LunarTerrain;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double R = 1737400.0;
constexpr double site_lat = -84.79 * deg, site_lon = 29.2 * deg;

// A flat global grid at `h` m (DN = 2h) with LDEM_16's layout.
environment::CylindricalGrid flat_global(double h) {
    environment::CylindricalGrid g;
    g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, static_cast<std::int16_t>(2.0 * h))};
    g.pixels_per_degree = 16.0;
    g.zero_line = 1439.5;
    g.zero_sample = -0.5;
    return g;
}

environment::PolarStereographicTile empty_tile() {
    environment::PolarStereographicTile t;
    t.sphere_radius_m = R;
    t.scale_m = 20.0;
    t.pole_line = t.pole_sample = 15199.5;
    return t;
}

// DN steps per pixel for a slope (heights are stored in 0.5 m steps: a rounded slope would add up to +-1.4 deg of
// quantisation noise per cell), and the slope those steps represent.
int slope_steps(double slope_rad) { return static_cast<int>(std::lround(std::tan(slope_rad) * 20.0 / 0.5)); }
double exact_slope_rad(double slope_rad) { return std::atan(slope_steps(slope_rad) * 0.5 / 20.0); }

// A 201 x 201 tile (20 m) centred on (lat, lon) whose heights rise by exact_slope_rad(slope) along +sample.
environment::PolarStereographicTile sloped_tile(double lat, double lon, double base_m, double slope_rad) {
    auto t = empty_tile();
    const auto [line, sample] = t.position(lat, lon);  // in the full product (first_* = 0)
    t.first_line = static_cast<int>(std::lround(line)) - 100;
    t.first_sample = static_cast<int>(std::lround(sample)) - 100;
    t.grid = {201, 201, 0.5, std::vector<std::int16_t>(201 * 201)};
    for (int l = 0; l < 201; ++l) {
        for (int s = 0; s < 201; ++s) {
            t.grid.dn[static_cast<size_t>(l) * 201 + s] =
                static_cast<std::int16_t>(std::lround(2.0 * base_m) + (s - 100) * slope_steps(slope_rad));
        }
    }
    return t;
}

}  // namespace

TEST_CASE("Polar stereographic positions put the pole at the pole pixel and longitude 0 up", "[terrain]") {
    const auto t = empty_tile();
    const auto [lp, sp] = t.position(-90.0 * deg, 0.0);
    CHECK(lp == Catch::Approx(15199.5));
    CHECK(sp == Catch::Approx(15199.5));
    const auto [l0, s0] = t.position(-85.0 * deg, 0.0);
    CHECK(s0 == Catch::Approx(15199.5));
    CHECK(15199.5 - l0 == Catch::Approx(2.0 * R * std::tan(2.5 * deg) / 20.0));
    const auto [l90, s90] = t.position(-85.0 * deg, 90.0 * deg);
    CHECK(s90 > 15199.5);
    CHECK(l90 == Catch::Approx(15199.5));
}

TEST_CASE("Tile heights are the raw grid at nodes and bilinear between", "[terrain]") {
    auto g = sloped_tile(site_lat, site_lon, 5000.0, 0.0).grid;
    CHECK(g.bilinear(10.0, 20.0) == g.at(10, 20));
    g.dn[static_cast<size_t>(10) * 201 + 20] = 100;
    g.dn[static_cast<size_t>(10) * 201 + 21] = 200;
    g.dn[static_cast<size_t>(11) * 201 + 20] = 300;
    g.dn[static_cast<size_t>(11) * 201 + 21] = 400;
    CHECK(g.bilinear(10.5, 20.5) == Catch::Approx(0.5 * 250.0));
    CHECK(g.bilinear(200.0, 200.0) == g.at(200, 200));  // the last node is inside
}

TEST_CASE("The global grid wraps in longitude", "[terrain]") {
    auto g = flat_global(0.0);
    const int line = 1439;  // just north of the equator
    g.grid.dn[static_cast<size_t>(line) * 5760 + 5759] = 200;  // lon 359.97 deg
    g.grid.dn[static_cast<size_t>(line) * 5760 + 0] = 400;     // lon 0.03 deg
    const double lat = (1439.5 - line) / 16.0 * deg;
    CHECK(g.height_m(lat, 0.0) == Catch::Approx(150.0));  // halfway: (100 + 200) / 2 m
    CHECK(g.height_m(lat, constants::two_pi) == Catch::Approx(150.0));
}

TEST_CASE("Terrain normals follow the slope of the grid", "[terrain]") {
    for (double slope : {0.0, 5.0, 10.0, 15.0}) {
        const LunarTerrain terrain(flat_global(5000.0), {sloped_tile(site_lat, site_lon, 5000.0, slope * deg)}, R);
        const Vec3 p = frames::geodetic_to_fixed({site_lat, site_lon, 5000.0}, {R, 0.0});
        const Vec3 n = terrain.normal_fixed(p);
        const double tilt = std::acos(dot(n, math::normalized(p))) / deg;
        // the grid's slope (pixels are 20 m at the pole; the projection scale here is ~1.002: < 0.03 deg)
        CHECK(tilt == Catch::Approx(exact_slope_rad(slope * deg) / deg).margin(0.05));
    }
}

TEST_CASE("The terrain blends from a tile into the global grid without a step", "[terrain]") {
    // Tile 300 m above the global grid: inside its core the tile, beyond its edge the grid, between no jump.
    const LunarTerrain terrain(flat_global(5000.0), {sloped_tile(site_lat, site_lon, 5300.0, 0.0)}, R);
    CHECK(terrain.height_m(site_lat, site_lon) == Catch::Approx(5300.0));
    CHECK(terrain.in_tile(site_lat, site_lon));
    double previous = terrain.height_m(site_lat, site_lon);
    double largest_step = 0.0;
    for (int k = 1; k <= 200; ++k) {  // 4 km along the meridian (the tile reaches 2 km)
        const double h = terrain.height_m(site_lat + k * 20.0 / R, site_lon);
        largest_step = std::max(largest_step, std::abs(h - previous));
        previous = h;
    }
    CHECK(largest_step < 10.0);  // 300 m over a 1 km blend: ~6 m per 20 m at the steepest
    CHECK(previous == Catch::Approx(5000.0));
    CHECK(!terrain.in_tile(site_lat + 4000.0 / R, site_lon));
}

TEST_CASE("The Moon's surface height and normal come from its terrain", "[environment][lunar][terrain]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    auto moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    const Vec3 p = frames::geodetic_to_fixed({site_lat, site_lon, 0.0}, moon.shape());
    CHECK(moon.surface_height_m(p) == 0.0);
    CHECK(norm(moon.surface_normal_fixed(p) - math::normalized(p)) < 1e-15);
    const auto tile = sloped_tile(site_lat, site_lon, 5000.0, 10.0 * deg);
    const double sample = tile.position(site_lat, site_lon).second;  // the site lies within half a pixel of the centre
    const LunarTerrain terrain(flat_global(5000.0), {tile}, R);
    moon.set_terrain(&terrain);
    CHECK(moon.surface_height_m(p) == Catch::Approx(5000.0 + (sample - 100.0) * slope_steps(10.0 * deg) * 0.5));
    CHECK(std::acos(dot(moon.surface_normal_fixed(p), math::normalized(p))) ==
          Catch::Approx(exact_slope_rad(10.0 * deg)).margin(0.05 * deg));
}

TEST_CASE("LOLA data: the site tile matches the pipeline's raw-grid check points", "[terrain][data]") {
    const std::filesystem::path data = STARSIM_DATA_DIR;
    const auto site_config = config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2");
    const auto file = config::load_terrain_grid_file(data / "terrain" / "sites" / (site_config.height_tile_key + ".toml"));
    const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, data, site_config);
    REQUIRE(file.checks.size() == 9);
    for (const auto& c : file.checks) {
        CHECK(terrain->height_m(c.latitude_rad, c.longitude_rad) == Catch::Approx(c.height_m).margin(1e-6));
    }
    const auto site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
    std::printf("[terrain] %s: %.1f m above the 1737.4 km sphere\n", site.config.id.c_str(), site.height_m);
    CHECK(site.height_m > 4000.0);  // Mons Mouton region: 5,000-6,000 m [S: NASA 2024]
    CHECK(site.height_m < 7000.0);
}

TEST_CASE("Missing terrain data is a report code", "[terrain]") {
    try {
        sim::load_lunar_terrain(STARSIM_CONFIG_DIR, std::filesystem::temp_directory_path() / "starsim_no_data");
        FAIL("expected an error");
    } catch (const config::DataMissing& e) {
        CHECK(e.report().code == report::Code::data_lunar_terrain_missing);
    }
}

TEST_CASE("A site moved after its tile was downloaded finds no tile", "[terrain][data]") {
    // The cache key names the position: a site moved 3 km has another key and the old tile is never reused.
    const auto moved = config::lunar_site_at(STARSIM_CONFIG_DIR, -84.6906 * deg, 29.1957 * deg);
    CHECK(moved.height_tile_key != config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2").height_tile_key);
    try {
        sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, moved);
        FAIL("expected an error");
    } catch (const config::DataMissing& e) {
        CHECK(e.report().code == report::Code::data_lunar_site_tile_missing);
    }
}

TEST_CASE("terrain for Apollo 11 uses its 60 m tile", "[sim][lunar][terrain][data]") {
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!sim::lunar_site_tiles_present(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, site)) SKIP("Apollo 11 tile not installed");
    const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, site);
    const double fine = terrain->finest_resolution_within_m(site.latitude_rad, site.longitude_rad, 100.0);
    CHECK(fine <= 61.0);
    CHECK(fine > 55.0);
    // Without the site the global grid alone: ~473 m.
    const auto global_only = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
    CHECK(global_only->finest_resolution_within_m(site.latitude_rad, site.longitude_rad, 100.0) > 400.0);
    // Another site's tile is not in this terrain: Apollo 15's neighbourhood is the global grid.
    const auto a15 = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_15");
    CHECK(terrain->finest_resolution_within_m(a15.latitude_rad, a15.longitude_rad, 100.0) > 400.0);
}

TEST_CASE("a tile is accepted for any site of its cache key (R19: exact user coordinates)", "[sim][lunar][terrain][data]") {
    // The Apollo 17 tile was built for (20.19089, 30.77168); its key 2019_3077 holds every coordinate that rounds to it.
    const auto exact = config::lunar_site_at(STARSIM_CONFIG_DIR, 20.19089 * deg, 30.77168 * deg);
    if (!sim::lunar_site_tiles_present(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, exact)) SKIP("Apollo 17 tile not installed");
    for (const auto& [lat, lon] : {std::pair{20.19, 30.77}, std::pair{20.1949, 30.7749}, std::pair{20.1851, 30.7651}}) {
        const auto site = config::lunar_site_at(STARSIM_CONFIG_DIR, lat * deg, lon * deg);
        REQUIRE(site.height_tile_key == exact.height_tile_key);
        const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, site);
        CHECK(terrain->finest_resolution_within_m(site.latitude_rad, site.longitude_rad, 100.0) < 400.0);
    }
    // Another key (0.01 deg further) has no tile: refused as missing, never matched to this tile.
    const auto other = config::lunar_site_at(STARSIM_CONFIG_DIR, 20.2 * deg, 30.77 * deg);
    CHECK(other.height_tile_key != exact.height_tile_key);
    CHECK_THROWS_AS(sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, other), config::DataMissing);
}

TEST_CASE("free coordinates resolve like presets and get a coordinate id", "[sim][lunar][terrain][config]") {
    const auto preset = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    const auto free = config::lunar_site_at(STARSIM_CONFIG_DIR, preset.latitude_rad, preset.longitude_rad);
    CHECK(free.id == "site_0.67_23.47");
    CHECK(free.height_tile_key == preset.height_tile_key);
    CHECK(free.tile_half_size_m == preset.tile_half_size_m);
    const auto polar = config::lunar_site_at(STARSIM_CONFIG_DIR, -84.7906 * deg, 29.1957 * deg);
    CHECK(polar.id == "site_-84.79_29.20");
    CHECK(polar.tile_half_size_m == 10000.0);  // R1: the polar 20 m products use 10 km
    CHECK(config::lunar_site_at(STARSIM_CONFIG_DIR, -0.001 * deg, 0.0).id == "site_0.00_0.00");
}

TEST_CASE("free coordinate ids wrap the longitude into [-180, 180) hundredths", "[sim][lunar][terrain][config]") {
    const auto id = [](double lat_deg, double lon_deg) {
        return config::lunar_site_at(STARSIM_CONFIG_DIR, lat_deg * deg, lon_deg * deg).id;
    };
    CHECK(id(-30.0, -170.0) == "site_-30.00_-170.00");
    CHECK(id(-30.0, 190.0) == id(-30.0, -170.0));
    CHECK(id(10.0, 359.999) == "site_10.00_0.00");
    CHECK(id(10.0, -0.001) == "site_10.00_0.00");
    CHECK(id(-0.001, 180.0) == "site_0.00_-180.00");
}

TEST_CASE("free coordinates without a cached tile raise data.lunar_site_tile_missing", "[sim][lunar][terrain][data]") {
    if (!std::filesystem::exists(std::filesystem::path(STARSIM_DATA_DIR) / "terrain" / "ldem_64.toml")) {
        SKIP("LDEM_64 not installed");
    }
    const auto site = config::lunar_site_at(STARSIM_CONFIG_DIR, 10.37 * deg, -45.13 * deg);  // no tile downloaded here
    REQUIRE(!sim::lunar_site_tiles_present(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, site));
    try {
        sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, site);
        FAIL("expected an error");
    } catch (const config::DataMissing& e) {
        CHECK(e.report().code == report::Code::data_lunar_site_tile_missing);
        CHECK(e.report().params.size() == 2);
    }
}

namespace {

// A synthetic grid whose heights depend on line and sample (distinct at every pixel, 0.5 m steps).
environment::HeightGrid ramp_grid(int lines, int samples) {
    environment::HeightGrid g{lines, samples, 0.5, std::vector<std::int16_t>(static_cast<size_t>(lines) * samples)};
    for (int l = 0; l < lines; ++l) {
        for (int s = 0; s < samples; ++s) {
            g.dn[static_cast<size_t>(l) * samples + s] = static_cast<std::int16_t>(7 * l + 3 * s + (l * s) % 11);
        }
    }
    return g;
}

Vec3 unit_of(double lat, double lon) {
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

}  // namespace

TEST_CASE("north polar tile mirrors the south one", "[environment][lunar][terrain]") {
    environment::PolarStereographicTile south;
    south.grid = ramp_grid(401, 401);
    south.sphere_radius_m = R;
    south.scale_m = 60.0;
    south.pole_line = south.pole_sample = 200.0;  // the pole at the centre pixel
    auto north = south;
    north.grid.dn.assign(south.grid.dn.rbegin(), south.grid.dn.rend());  // rotated by 180 deg about the pole pixel
    const auto south_tile = environment::make_polar_tile(south, false);
    const auto north_tile = environment::make_polar_tile(north, true);
    // (+lat, lon) on the north tile is the point reflection through the pole of (-lat, -lon) on the south tile.
    for (const auto [lat_deg, lon_deg] : {std::pair{89.9, 30.0}, {89.8, 200.0}, {89.85, -75.0}}) {
        const Vec3 n = unit_of(lat_deg * deg, lon_deg * deg), s = unit_of(-lat_deg * deg, -lon_deg * deg);
        REQUIRE(north_tile->contains(n));
        REQUIRE(south_tile->contains(s));
        CHECK(std::abs(north_tile->height_m(n) - south_tile->height_m(s)) < 1e-9);
        CHECK(std::abs(north_tile->edge_distance_m(n) - south_tile->edge_distance_m(s)) < 1e-6);
    }
    // the PDS north convention: longitude 0 is down the lines, longitude 90 along the samples
    environment::PolarStereographicTile probe = south;
    probe.north = true;
    const auto [l0, s0] = probe.position(85.0 * deg, 0.0);
    CHECK(l0 - 200.0 == Catch::Approx(2.0 * R * std::tan(2.5 * deg) / 60.0));
    CHECK(s0 == Catch::Approx(200.0));
    const auto [l90, s90] = probe.position(85.0 * deg, 90.0 * deg);
    CHECK(s90 > 200.0);
    const auto [ld, sd] = probe.position_of_direction(unit_of(85.0 * deg, 33.0 * deg));
    const auto [lp, sp] = probe.position(85.0 * deg, 33.0 * deg);
    CHECK(ld == Catch::Approx(lp).margin(1e-9));
    CHECK(sd == Catch::Approx(sp).margin(1e-9));
    CHECK(!north_tile->contains(unit_of(-85.0 * deg, 30.0 * deg)));
    CHECK(std::isinf(north_tile->outside_distance_m(unit_of(-85.0 * deg, 30.0 * deg))));
}

TEST_CASE("cylindrical tile samples bilinearly and blends to the global grid over 1 km",
          "[environment][lunar][terrain]") {
    // 20 ppd (~1.5 km per pixel is too coarse for a 1 km blend): 2000 ppd = 15 m, a 600 x 600 tile (9 km) at 0 N 23 E.
    environment::CylindricalGrid cg;
    cg.pixels_per_degree = 2000.0;
    cg.grid = {600, 600, 0.5, std::vector<std::int16_t>(600 * 600, 2 * 300)};  // 300 m everywhere
    const double half = 299.5 / 2000.0 * deg;
    const auto tile = environment::make_cylindrical_tile(cg, half, 23.0 * deg - half, R);
    const LunarTerrain terrain(flat_global(0.0), {tile}, R);
    const double px_m = R * deg / 2000.0;
    CHECK(tile->scale_m() == Catch::Approx(px_m));
    CHECK(terrain.in_tile(0.0, 23.0 * deg));
    CHECK(terrain.height_m(0.0, 23.0 * deg) == Catch::Approx(300.0));
    CHECK(terrain.height_at_direction_m(unit_of(0.0, 23.0 * deg)) == Catch::Approx(300.0));
    // bilinear between nodes: a ramp along the samples, 1 m per pixel
    environment::CylindricalGrid ramp = cg;
    for (int l = 0; l < 600; ++l) {
        for (int s = 0; s < 600; ++s) ramp.grid.dn[static_cast<size_t>(l) * 600 + s] = static_cast<std::int16_t>(2 * s);
    }
    const auto rt = environment::make_cylindrical_tile(ramp, half, 23.0 * deg - half, R);
    const double lon_node = 23.0 * deg - half;  // sample 0
    CHECK(rt->height_m(unit_of(0.0, lon_node + 10.25 / 2000.0 * deg)) == Catch::Approx(10.25).margin(0.01));
    // blend along the equator (parallel = pixel size at 0 N): the edge is 299.5 px from the centre
    const double edge_m = 299.5 * px_m;
    const auto weight_at = [&](double inside_m) {
        return terrain.tile_weight(unit_of(0.0, 23.0 * deg + (edge_m - inside_m) / R));
    };
    CHECK(weight_at(2000.0) == 1.0);
    CHECK(weight_at(-50.0) == 0.0);
    CHECK(weight_at(500.0) == Catch::Approx(0.5).margin(0.002));
    CHECK(terrain.height_m(0.0, 23.0 * deg + (edge_m + 50.0) / R) == Catch::Approx(0.0));
    CHECK(terrain.height_m(0.0, 23.0 * deg + (edge_m - 500.0) / R) == Catch::Approx(150.0).margin(1.0));
    CHECK(terrain.finest_resolution_within_m(0.0, 23.0 * deg + (edge_m + 500.0) / R, 600.0) == Catch::Approx(px_m));
    CHECK(terrain.finest_resolution_within_m(0.0, 23.0 * deg + (edge_m + 2000.0) / R, 600.0) > 1000.0);
}

TEST_CASE("cylindrical tile across the 0/360 seam", "[environment][lunar][terrain]") {
    environment::CylindricalGrid cg;
    cg.pixels_per_degree = 1000.0;
    cg.grid = ramp_grid(400, 400);
    const double half = 199.5 / 1000.0 * deg;
    const auto tile = environment::make_cylindrical_tile(cg, 10.0 * deg + half, -half, R);  // centred on 10 N, lon 0
    const Vec3 west = unit_of(10.0 * deg, -0.01 * deg), wrapped = unit_of(10.0 * deg, 359.99 * deg);
    REQUIRE(tile->contains(west));
    REQUIRE(tile->contains(wrapped));
    CHECK(std::abs(tile->height_m(west) - tile->height_m(wrapped)) < 1e-6);
    CHECK(tile->height_m(unit_of(10.0 * deg, 0.01 * deg)) != tile->height_m(west));  // the two sides differ
    CHECK(!tile->contains(unit_of(10.0 * deg, 180.0 * deg)));
    const LunarTerrain terrain(flat_global(0.0), {tile}, R);
    CHECK(terrain.in_tile(10.0 * deg, 359.99 * deg));
    CHECK(terrain.in_tile(10.0 * deg, -0.01 * deg));
}
