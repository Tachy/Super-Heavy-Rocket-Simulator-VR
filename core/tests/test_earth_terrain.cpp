// Earth terrain (M8 Plan A, Task 3): the Terrain interface, EarthTerrain's precedence (plateau > local > regional >
// global relief) with its edge blends, water on the geoid, normals, the loader's staleness check and the downloaded
// tiles against the pipeline's check points.

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/geoid_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/earth_terrain.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/environment/terrain.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/earth_terrain_loader.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using environment::EarthTerrain;
using environment::GeographicGrid;
using environment::Plateau;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double a = constants::wgs84_semi_major_axis_m;
constexpr double geoid_n = -25.0;  // synthetic undulation everywhere
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

environment::Geoid constant_geoid(double n_m) { return environment::Geoid(constant_global(n_m, 0.01)); }

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

// Global 10 m, a regional box (0.4 deg) at 20 m, a local box (0.1 deg) at `local_m`; geoid -25 m.
EarthTerrain layered(double local_m, std::vector<Plateau> plateaus = {}) {
    return EarthTerrain(constant_geoid(geoid_n), constant_global(10.0, 1.0), {box(41, 0.01, 20.0)},
                        {box(101, 0.001, local_m)}, std::move(plateaus));
}

double lon_deg_of_metres_east(double metres, double lat_deg) { return metres / (a * deg * std::cos(lat_deg * deg)); }

// --- synthetic data / config directories for the loader ---

void write_grid_file(const std::filesystem::path& toml, const GeographicGrid& g, const std::string& extra,
                     const std::string& height_reference = "wgs84_ellipsoid") {
    std::filesystem::create_directories(toml.parent_path());
    const auto heights = toml.stem().string() + ".i16";
    std::ofstream t(toml);
    t.precision(17);
    t << "[grid]\nproduct = \"synthetic\"\nprojection = \"geographic\"\nheight_reference = \"" << height_reference << "\"\n"
      << "sphere_radius_m = 6378137.0\ndn_scale_m = " << g.grid.dn_scale_m << "\nheights = \"" << heights << "\"\n"
      << "lines = " << g.grid.lines << "\nsamples = " << g.grid.samples << "\nnorth_latitude_deg = " << g.north_latitude_deg
      << "\nwest_longitude_deg = " << g.west_longitude_deg << "\nstep_latitude_deg = " << g.step_latitude_deg
      << "\nstep_longitude_deg = " << g.step_longitude_deg
      << "\nglobal_longitude = " << (g.global_longitude ? "true" : "false") << "\n"
      << extra << "unknown_extra_key = \"ignored\"\n";
    std::ofstream h(toml.parent_path() / heights, std::ios::binary);
    h.write(reinterpret_cast<const char*>(g.grid.dn.data()), static_cast<std::streamsize>(2 * g.grid.dn.size()));
}

std::string area_header(const std::string& level, double half_size_m) {
    std::ostringstream s;
    s.precision(17);
    s << "area = \"testarea\"\narea_latitude_deg = " << centre_lat_deg << "\narea_longitude_deg = " << centre_lon_deg
      << "\nlevel = \"" << level << "\"\nhalf_size_m = " << half_size_m << "\n";
    return s.str();
}

void write_area_config(const std::filesystem::path& config_dir, double lat_deg, double lon_deg) {
    std::filesystem::create_directories(config_dir);
    std::ofstream t(config_dir / "earth_terrain.toml");
    t.precision(17);
    t << "[[area]]\nid = \"testarea\"\nlatitude_deg = " << lat_deg << "\nlongitude_deg = " << lon_deg
      << "\nlocal_half_size_m = 5000.0\nregional_half_size_m = 20000.0\n\n[global]\nrelief_step_arcmin = 2.0\n";
    std::filesystem::copy_file(std::filesystem::path(STARSIM_CONFIG_DIR) / "launch_sites.toml",
                               config_dir / "launch_sites.toml", std::filesystem::copy_options::overwrite_existing);
}

// data/ layout with the synthetic grids: geoid/egm2008_25, terrain/earth/global_relief, testarea_{regional,local}.
void write_data_dir(const std::filesystem::path& data_dir) {
    write_grid_file(data_dir / "geoid" / "egm2008_25.toml", constant_global(geoid_n, 0.01), "", "egm2008_undulation");
    const auto earth = data_dir / "terrain" / "earth";
    write_grid_file(earth / "global_relief.toml", constant_global(10.0, 1.0), "area = \"global\"\nlevel = \"global\"\n");
    write_grid_file(earth / "testarea_regional.toml", box(41, 0.01, 20.0), area_header("regional", 20000.0));
    write_grid_file(earth / "testarea_local.toml", box(101, 0.001, 30.0), area_header("local", 5000.0));
}

}  // namespace

TEST_CASE("local grid wins inside and blends at the edge", "[environment][terrain][earth_terrain]") {
    const auto t = layered(30.0);
    const double lat = centre_lat_deg * deg;
    const double local_west_deg = centre_lon_deg - 0.05, regional_west_deg = centre_lon_deg - 0.2;
    CHECK(t.height_ellipsoid_m(lat, centre_lon_deg * deg) == Catch::Approx(30.0));
    // Outside the local box (100 m west of it) the regional grid holds; outside the regional one the global relief.
    CHECK(t.height_ellipsoid_m(lat, (local_west_deg - lon_deg_of_metres_east(100.0, 26.0)) * deg) == Catch::Approx(20.0));
    CHECK(t.height_ellipsoid_m(lat, (regional_west_deg - lon_deg_of_metres_east(100.0, 26.0)) * deg) ==
          Catch::Approx(10.0));
    // blend_m inside the edge: the finer grid exactly.
    CHECK(t.height_ellipsoid_m(lat, (local_west_deg + lon_deg_of_metres_east(EarthTerrain::blend_m, 26.0)) * deg) ==
          Catch::Approx(30.0).margin(1e-6));
    CHECK(t.height_ellipsoid_m(lat, (regional_west_deg + lon_deg_of_metres_east(EarthTerrain::blend_m, 26.0)) * deg) ==
          Catch::Approx(20.0).margin(1e-6));
    // Continuous across both bands: 1 m steps from 100 m outside to 700 m inside each edge.
    for (const double edge_deg : {local_west_deg, regional_west_deg}) {
        double previous = t.height_ellipsoid_m(lat, (edge_deg - lon_deg_of_metres_east(100.0, 26.0)) * deg);
        double worst = 0.0;
        for (double m = -99.0; m <= 700.0; m += 1.0) {
            const double h = t.height_ellipsoid_m(lat, (edge_deg + lon_deg_of_metres_east(m, 26.0)) * deg);
            worst = std::max(worst, std::abs(h - previous));
            previous = h;
        }
        CAPTURE(edge_deg);
        CHECK(worst < 0.05);  // smoothstep over 500 m: <= 1.5 x 10 m / 500 m per metre
    }
}

TEST_CASE("plateau is flat and blends over the apron", "[environment][terrain][earth_terrain]") {
    const Plateau pad{"pad", centre_lat_deg * deg, centre_lon_deg * deg, 40.0, 150.0, 50.0};
    const auto t = layered(30.0, {pad});
    const Vec3 c = frames::geodetic_to_fixed({pad.latitude_rad, pad.longitude_rad, 0.0});
    for (const double north_m : {0.0, 100.0, 149.0, 160.0, 175.0, 190.0, 199.0, 210.0, 400.0}) {
        const double lat = pad.latitude_rad + north_m / a;  // roughly north_m north of the centre
        const double d = norm(frames::geodetic_to_fixed({lat, pad.longitude_rad, 0.0}) - c);
        const double expected = d <= 150.0 ? 40.0 : d >= 200.0 ? 30.0 : 40.0 + (d - 150.0) / 50.0 * (30.0 - 40.0);
        CAPTURE(north_m, d);
        CHECK(t.height_ellipsoid_m(lat, pad.longitude_rad) == Catch::Approx(expected).margin(1e-6));
        CHECK_FALSE(t.water(lat, pad.longitude_rad));
    }
}

TEST_CASE("plateau is never water", "[environment][terrain][earth_terrain]") {
    // Sea all around (the terrain sits on the geoid); the pad itself lies 5 m below sea level.
    const Plateau pad{"pad", centre_lat_deg * deg, centre_lon_deg * deg, geoid_n - 5.0, 150.0, 50.0};
    const auto t = layered(geoid_n, {pad});
    const double lon = pad.longitude_rad;
    CHECK_FALSE(t.water(pad.latitude_rad, lon));
    CHECK(t.height_ellipsoid_m(pad.latitude_rad, lon) == Catch::Approx(geoid_n - 5.0));
    const double apron_lat = pad.latitude_rad + 175.0 / a;
    CHECK_FALSE(t.water(apron_lat, lon));
    const double sea_lat = pad.latitude_rad + 400.0 / a;
    CHECK(t.water(sea_lat, lon));
    CHECK(t.height_ellipsoid_m(sea_lat, lon) == Catch::Approx(geoid_n));
}

TEST_CASE("zero orthometric is water and sits on the geoid", "[environment][terrain][earth_terrain]") {
    const double lat = centre_lat_deg * deg, lon = centre_lon_deg * deg;
    const auto sea = layered(geoid_n);
    CHECK(sea.water(lat, lon));
    CHECK(sea.height_ellipsoid_m(lat, lon) == Catch::Approx(geoid_n));
    const auto shallow = layered(geoid_n + 0.04);  // within the threshold: still water
    CHECK(shallow.water(lat, lon));
    CHECK(shallow.height_ellipsoid_m(lat, lon) == Catch::Approx(geoid_n));
    const auto land = layered(geoid_n + 0.10);  // above it: land at its own height
    CHECK_FALSE(land.water(lat, lon));
    CHECK(land.height_ellipsoid_m(lat, lon) == Catch::Approx(geoid_n + 0.10));
    const auto& as_terrain = static_cast<const environment::Terrain&>(sea);
    CHECK(as_terrain.water_at_fixed(frames::geodetic_to_fixed({lat, lon, 100.0})));
    CHECK(as_terrain.height_at_fixed_m(frames::geodetic_to_fixed({lat, lon, 100.0})) == Catch::Approx(geoid_n));
    CHECK(sea.geoid().undulation_m(lat, lon) == Catch::Approx(geoid_n));
}

TEST_CASE("sea floor below the geoid is water at N", "[environment][terrain][earth_terrain]") {
    const auto t = layered(-60.0);
    const double lat = centre_lat_deg * deg, lon = centre_lon_deg * deg;
    CHECK(t.water(lat, lon));
    CHECK(t.height_ellipsoid_m(lat, lon) == Catch::Approx(geoid_n));
}

TEST_CASE("normal of a tilted plane grid", "[environment][terrain][earth_terrain]") {
    // 41 x 41 nodes of 0.001 deg; h = 100 + 2 m per sample east + 1 m per line north (~2 % and ~1 %), exact in DN.
    GeographicGrid tilted;
    tilted.grid = {41, 41, 1.0, std::vector<std::int16_t>(41 * 41)};
    for (int l = 0; l < 41; ++l) {
        for (int s = 0; s < 41; ++s) tilted.grid.dn[static_cast<size_t>(l) * 41 + s] = static_cast<std::int16_t>(100 + 2 * s + (40 - l));
    }
    tilted.north_latitude_deg = centre_lat_deg + 0.02;
    tilted.west_longitude_deg = centre_lon_deg - 0.02;
    tilted.step_latitude_deg = tilted.step_longitude_deg = 0.001;
    const EarthTerrain t(constant_geoid(geoid_n), constant_global(100.0, 1.0), {}, {tilted}, {});
    for (const auto& [lat_deg, lon_deg] : {std::pair{26.0, -97.0}, std::pair{26.0031, -96.9987}}) {
        const double lat = lat_deg * deg, lon = lon_deg * deg;
        const double h = t.height_ellipsoid_m(lat, lon);
        // Geodetic tangents: dP/dlon = (N + h) cos(lat) east + dh/dlon up, dP/dlat = (M + h) north + dh/dlat up.
        const double e2 = frames::wgs84.first_eccentricity_squared();
        const double w = 1.0 - e2 * std::sin(lat) * std::sin(lat);
        const double n_radius = a / std::sqrt(w), m_radius = a * (1.0 - e2) / (w * std::sqrt(w));
        const double slope_east = (2.0 / (0.001 * deg)) / ((n_radius + h) * std::cos(lat));
        const double slope_north = (1.0 / (0.001 * deg)) / (m_radius + h);
        const Vec3 up{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
        const Vec3 east{-std::sin(lon), std::cos(lon), 0.0};
        const Vec3 north{-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)};
        const Vec3 expected = math::normalized(up - east * slope_east - north * slope_north);
        CAPTURE(lat_deg, lon_deg, slope_east, slope_north);
        const Vec3 n = t.normal_fixed(frames::geodetic_to_fixed({lat, lon, h + 3.0}));
        CHECK(norm(n - expected) < 1e-6);
        CHECK(slope_east == Catch::Approx(0.02).epsilon(0.01));
    }
}

TEST_CASE("lunar terrain through the Terrain interface is unchanged", "[environment][terrain][earth_terrain]") {
    environment::CylindricalGrid g;  // 1 px / deg, heights 0.5 m x (line + sample)
    g.grid = {180, 360, 0.5, std::vector<std::int16_t>(180 * 360)};
    for (int l = 0; l < 180; ++l) {
        for (int s = 0; s < 360; ++s) g.grid.dn[static_cast<size_t>(l) * 360 + s] = static_cast<std::int16_t>(l + s);
    }
    g.pixels_per_degree = 1.0;
    g.zero_line = 89.5;
    g.zero_sample = -0.5;
    const environment::LunarTerrain lunar(g, {}, constants::moon_mean_radius_m);
    const environment::Terrain& terrain = lunar;
    const Vec3 p = frames::geodetic_to_fixed({-40.3 * deg, 12.7 * deg, 50.0}, {constants::moon_mean_radius_m, 0.0});
    CHECK(terrain.height_at_fixed_m(p) == lunar.height_at_fixed_m(p));
    CHECK(norm(terrain.normal_fixed(p) - lunar.normal_fixed(p)) == 0.0);
    CHECK_FALSE(terrain.water_at_fixed(p));
}

TEST_CASE("a terrain field on the Earth follows the ellipsoid and reports water", "[contact][terrain][earth_terrain]") {
    test::spice_session();
    const auto itrf = spice::SpiceOrientation::earth();
    const environment::Us76Atmosphere air;
    const auto earth = environment::ReferenceBody::earth(itrf, air, constants::earth_gm_m3ps2);
    // Local box: land at 30 m west of 97 W (samples < 50), sea floor at -60 m east of it (samples > 50).
    auto local = box(101, 0.001, 30.0);
    for (int l = 0; l < 101; ++l) {
        for (int s = 51; s < 101; ++s) local.grid.dn[static_cast<size_t>(l) * 101 + s] = -6000;
    }
    const auto terrain = std::make_shared<const EarthTerrain>(constant_geoid(geoid_n), constant_global(10.0, 1.0),
                                                              std::vector<GeographicGrid>{}, std::vector{local},
                                                              std::vector<Plateau>{});
    const double lat = centre_lat_deg * deg, lon = (centre_lon_deg - 0.02) * deg;
    const Vec3 origin = frames::geodetic_to_fixed({lat, lon, 30.0});
    const Vec3 up{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    const Vec3 east{-std::sin(lon), std::cos(lon), 0.0};
    const contact::TerrainField field(earth, origin, contact::from_columns(east, cross(up, east), up), terrain);
    const auto here = field.below(0.0, 0.0);
    CHECK(std::abs(here.height_m) < 1e-6);
    CHECK_FALSE(here.water);
    CHECK(here.normal.z == Catch::Approx(1.0));
    // 3 km east: the sea at the geoid, 55 m below the site, plus the drop of the curved surface d^2 / 2 N.
    const double d = 3000.0;
    const double n_radius = a / std::sqrt(1.0 - frames::wgs84.first_eccentricity_squared() * std::pow(std::sin(lat), 2));
    const auto sea = field.below(d, 0.0);
    CHECK(sea.water);
    CHECK(sea.height_m == Catch::Approx(geoid_n - 30.0 - d * d / (2.0 * n_radius)).margin(0.02));
}

TEST_CASE("stale local tile is refused", "[environment][terrain][earth_terrain]") {
    const auto root = std::filesystem::temp_directory_path() / "starsim_earth_terrain_stale_tile";
    std::filesystem::remove_all(root);
    write_data_dir(root / "data");
    write_area_config(root / "config", centre_lat_deg, centre_lon_deg);
    CHECK(sim::earth_terrain_available(root / "data"));
    CHECK_FALSE(sim::earth_terrain_available(root / "no_data"));
    {  // matching files load (unknown header keys ignored)
        const auto t = sim::load_earth_terrain(root / "config", root / "data");
        REQUIRE(t);
        CHECK(t->height_ellipsoid_m(25.5 * deg, -95.0 * deg) == Catch::Approx(10.0));  // global relief
        const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");  // a plateau of every site
        CHECK(t->height_ellipsoid_m(site.latitude_rad, site.longitude_rad) == Catch::Approx(site.height_m));
    }
    // The area centre moved by two cells of the local grid (0.002 deg): the local tile no longer fits.
    write_area_config(root / "config", centre_lat_deg + 0.002, centre_lon_deg);
    try {
        sim::load_earth_terrain(root / "config", root / "data");
        FAIL("expected a ConfigError");
    } catch (const config::ConfigError& e) {
        CHECK(std::string(e.what()).find("testarea_local") != std::string::npos);
        CHECK(std::string(e.what()).find("tools/data_pipeline/fetch.py") != std::string::npos);
    }
    // A missing file names the fetch script too.
    write_area_config(root / "config", centre_lat_deg, centre_lon_deg);
    std::filesystem::remove(root / "data" / "terrain" / "earth" / "testarea_regional.toml");
    CHECK_THROWS_AS(sim::load_earth_terrain(root / "config", root / "data"), config::ConfigError);
    std::filesystem::remove_all(root);
}

TEST_CASE("loaded Earth terrain at the pads", "[environment][terrain][earth_terrain][data]") {
    const std::filesystem::path data_dir = STARSIM_DATA_DIR;
    REQUIRE(sim::earth_terrain_available(data_dir));
    // Every grid file reproduces the pipeline's check points (bilinear from the stored DN).
    const auto earth = data_dir / "terrain" / "earth";
    for (const auto& name : {"global_relief", "starbase_regional", "starbase_local", "cape_regional", "cape_local"}) {
        const auto file = config::load_terrain_grid_file(earth / (std::string(name) + ".toml"));
        CHECK(file.height_reference == "wgs84_ellipsoid");
        REQUIRE(!file.checks.empty());
        const auto grid = config::geographic_grid(file);
        for (const auto& c : file.checks) {
            CAPTURE(name, c.latitude_rad / deg, c.longitude_rad / deg);
            CHECK(std::abs(grid.value(c.latitude_rad, c.longitude_rad) - c.height_m) <= 0.01);
        }
    }
    const auto terrain = sim::load_earth_terrain(STARSIM_CONFIG_DIR, data_dir);
    REQUIRE(terrain);
    for (const auto& site : config::load_launch_sites(STARSIM_CONFIG_DIR)) {
        CAPTURE(site.id);
        CHECK(std::abs(terrain->height_ellipsoid_m(site.latitude_rad, site.longitude_rad) - site.height_m) <= 0.01);
        CHECK_FALSE(terrain->water(site.latitude_rad, site.longitude_rad));
        CHECK(site.plateau_radius_m > 0.0);
        CHECK(site.plateau_apron_m > 0.0);
    }
    // Open sea east of Starbase (Gulf of Mexico, ~5 km offshore): water at the fine geoid.
    const double lat = 25.99686 * deg, lon = -97.10 * deg;
    CHECK(terrain->water(lat, lon));
    CHECK(terrain->height_ellipsoid_m(lat, lon) == Catch::Approx(terrain->geoid().undulation_m(lat, lon)));
}
