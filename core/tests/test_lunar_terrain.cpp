// Lunar terrain (lunar l2): polar stereographic tile, global grid, blend, normals; the real LOLA data against the
// pipeline's check points (raw DN, bilinear); the Moon reference body's surface.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
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
    const auto file = config::load_terrain_grid_file(data / "terrain" / "mons_mouton_im2.toml");
    const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, data);
    REQUIRE(file.checks.size() == 9);
    for (const auto& c : file.checks) {
        CHECK(terrain->height_m(c.latitude_rad, c.longitude_rad) == Catch::Approx(c.height_m).margin(1e-6));
    }
    const auto site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
    std::printf("[terrain] %s: %.1f m above the 1737.4 km sphere\n", site.config.name.c_str(), site.height_m);
    CHECK(site.height_m > 4000.0);  // Mons Mouton region: 5,000-6,000 m [S: NASA 2024]
    CHECK(site.height_m < 7000.0);
}

TEST_CASE("Missing terrain data names the fetch script", "[terrain]") {
    try {
        sim::load_lunar_terrain(STARSIM_CONFIG_DIR, std::filesystem::temp_directory_path() / "starsim_no_data");
        FAIL("expected an error");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("tools/data_pipeline/fetch.py") != std::string::npos);
    }
}
