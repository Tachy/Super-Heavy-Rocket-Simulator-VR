// Geodetic surface sampling (M8 Plan B, Task 1): sample_surface against the pointwise Terrain calls and the
// terrains' resolution_m (finest covering grid) on synthetic grids.

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/earth_terrain.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/surface_samples.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using environment::EarthTerrain;
using environment::GeographicGrid;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double centre_lat_deg = 26.0, centre_lon_deg = -97.0;

// A global 3 x 4 grid (90 deg nodes) of one value.
GeographicGrid constant_global(double value_m, double dn_scale_m) {
    GeographicGrid g;
    g.grid = {3, 4, dn_scale_m, std::vector<std::int16_t>(12, static_cast<std::int16_t>(std::lround(value_m / dn_scale_m)))};
    g.north_latitude_deg = 90.0;
    g.west_longitude_deg = -180.0;
    g.step_latitude_deg = 90.0;
    g.step_longitude_deg = 90.0;
    g.global_longitude = true;
    return g;
}

// A square box of n x n nodes, step_deg apart, centred on (26 N, 97 W), all `value_m` (DN in centimetres).
GeographicGrid box(int n, double step_deg, double value_m) {
    GeographicGrid g;
    g.grid = {n, n, 0.01, std::vector<std::int16_t>(static_cast<size_t>(n) * n,
                                                    static_cast<std::int16_t>(std::lround(value_m / 0.01)))};
    g.north_latitude_deg = centre_lat_deg + 0.5 * (n - 1) * step_deg;
    g.west_longitude_deg = centre_lon_deg - 0.5 * (n - 1) * step_deg;
    g.step_latitude_deg = step_deg;
    g.step_longitude_deg = step_deg;
    return g;
}

// Global 10 m (geoid -25 m, so the sea lies below -25 m: the global relief is land), a regional box (0.4 deg) at
// 0.01 deg steps, a local box (0.1 deg) at 0.001 deg steps.
EarthTerrain layered() {
    return EarthTerrain(environment::Geoid(constant_global(-25.0, 0.01)), constant_global(10.0, 1.0),
                        {box(41, 0.01, 20.0)}, {box(101, 0.001, 30.0)}, {});
}

double metres(double step_deg) { return step_deg * deg * constants::wgs84_semi_major_axis_m; }

}  // namespace

TEST_CASE("sample_surface matches pointwise height and water", "[environment][surface_samples]") {
    const auto terrain = layered();
    std::vector<double> lat, lon;
    for (int i = 0; i < 40; ++i) {  // local box, regional ring, global
        lat.push_back((centre_lat_deg + 0.003 * i - 0.06) * deg);
        lon.push_back((centre_lon_deg - 0.2 + 0.011 * i) * deg);
    }
    lat.push_back(60.0 * deg);
    lon.push_back(10.0 * deg);
    const auto s = environment::sample_surface(terrain, lat, lon);
    REQUIRE(s.height_m.size() == lat.size());
    REQUIRE(s.water.size() == lat.size());
    for (size_t i = 0; i < lat.size(); ++i) {
        CHECK(s.height_m[i] == terrain.height_ellipsoid_m(lat[i], lon[i]));
        CHECK(s.water[i] == (terrain.water(lat[i], lon[i]) ? 1 : 0));
    }
}

TEST_CASE("sample_surface flags water", "[environment][surface_samples]") {
    // Global relief -40 m below the geoid (-25 m): water everywhere outside the boxes.
    const EarthTerrain sea(environment::Geoid(constant_global(-25.0, 0.01)), constant_global(-40.0, 1.0), {}, {}, {});
    const std::vector<double> lat{0.1, -0.4}, lon{1.0, 2.0};
    const auto s = environment::sample_surface(sea, lat, lon);
    CHECK(s.water == std::vector<std::uint8_t>{1, 1});
    CHECK(s.height_m[0] == Catch::Approx(-25.0));
}

TEST_CASE("sample_surface rejects mismatched spans", "[environment][surface_samples]") {
    const auto terrain = layered();
    const std::vector<double> lat{0.1, 0.2}, lon{0.1};
    CHECK_THROWS_AS(environment::sample_surface(terrain, lat, lon), std::invalid_argument);
}

TEST_CASE("resolution follows the finest grid", "[environment][surface_samples]") {
    const auto terrain = layered();
    const auto at = [&](double lat_deg, double lon_deg) { return terrain.resolution_m(lat_deg * deg, lon_deg * deg); };
    CHECK(at(centre_lat_deg, centre_lon_deg) == Catch::Approx(metres(0.001)).epsilon(1e-9));  // local
    CHECK(at(centre_lat_deg + 0.15, centre_lon_deg) == Catch::Approx(metres(0.01)).epsilon(1e-9));  // regional only
    CHECK(at(10.0, 20.0) == Catch::Approx(metres(90.0)).epsilon(1e-9));  // global relief
}

namespace {

constexpr double moon_radius_m = 1737400.0;
constexpr double tile_lat_deg = -84.79, tile_lon_deg = 29.2;

// LDEM_16-like global grid (flat) and a 201 x 201 tile at 20 m (4 km square) centred on (tile_lat, tile_lon).
environment::LunarTerrain flat_moon_with_tile() {
    environment::CylindricalGrid global;
    global.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, 0)};
    global.pixels_per_degree = 16.0;
    global.zero_line = 1439.5;
    global.zero_sample = -0.5;
    environment::PolarStereographicTile tile;
    tile.sphere_radius_m = moon_radius_m;
    tile.scale_m = 20.0;
    tile.pole_line = tile.pole_sample = 15199.5;
    const auto [line, sample] = tile.position(tile_lat_deg * deg, tile_lon_deg * deg);
    tile.first_line = static_cast<int>(std::lround(line)) - 100;
    tile.first_sample = static_cast<int>(std::lround(sample)) - 100;
    tile.grid = {201, 201, 0.5, std::vector<std::int16_t>(201 * 201, 0)};
    return environment::LunarTerrain(std::move(global), {tile}, moon_radius_m);
}

}  // namespace

TEST_CASE("finest resolution over points finds a tile covering one of them", "[environment][surface_samples]") {
    const auto moon = flat_moon_with_tile();
    const double ldem_m = moon_radius_m * deg / 16.0;
    // A row of points 1 km apart along the meridian (~0.033 deg): only the middle one lies in the 4 km tile.
    const double step_deg = 1000.0 / (moon_radius_m * deg);
    std::vector<double> lat, lon;
    for (int k = -5; k <= 5; ++k) {
        if (k == 0) continue;
        lat.push_back((tile_lat_deg + 3.0 * step_deg * k) * deg);
        lon.push_back(tile_lon_deg * deg);
    }
    CHECK(environment::finest_resolution_m(moon, lat, lon) == Catch::Approx(ldem_m).epsilon(1e-9));
    lat.push_back(tile_lat_deg * deg);
    lon.push_back(tile_lon_deg * deg);
    CHECK(environment::finest_resolution_m(moon, lat, lon) == Catch::Approx(20.0).epsilon(1e-9));
    CHECK(std::isinf(environment::finest_resolution_m(moon, {}, {})));
    const std::vector<double> one{0.1};
    CHECK_THROWS_AS(environment::finest_resolution_m(moon, one, {}), std::invalid_argument);
}

TEST_CASE("lunar resolution in and outside the tile", "[environment][surface_samples]") {
    constexpr double R = 1737400.0;
    environment::CylindricalGrid global;
    global.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, 0)};
    global.pixels_per_degree = 16.0;
    global.zero_line = 1439.5;
    global.zero_sample = -0.5;
    environment::PolarStereographicTile tile;
    tile.sphere_radius_m = R;
    tile.scale_m = 20.0;
    tile.pole_line = tile.pole_sample = 15199.5;
    const double lat = -84.79 * deg, lon = 29.2 * deg;
    const auto [line, sample] = tile.position(lat, lon);
    tile.first_line = static_cast<int>(std::lround(line)) - 100;
    tile.first_sample = static_cast<int>(std::lround(sample)) - 100;
    tile.grid = {201, 201, 0.5, std::vector<std::int16_t>(201 * 201, 0)};
    const environment::LunarTerrain moon(std::move(global), {tile}, R);
    CHECK(moon.resolution_m(lat, lon) == Catch::Approx(20.0).epsilon(1e-9));
    CHECK(moon.resolution_m(10.0 * deg, 50.0 * deg) == Catch::Approx(R * deg / 16.0).epsilon(1e-9));  // ~1.9 km
}
