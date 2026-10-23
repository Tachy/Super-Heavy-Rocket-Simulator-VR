// EGM2008 geoid (M8 part 1): the node-registered geographic grid (antimeridian, poles, edges), the committed 1-degree
// grid and the 2.5' data grid against reference points sampled from NGA's 2.5' grid
// (tests/validation/geoid/egm2008_points.csv).

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/geoid_config.hpp"
#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/geographic_grid.hpp"
#include "starsim/environment/geoid.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using environment::GeographicGrid;

namespace {

constexpr double deg = constants::deg_to_rad;

struct ReferencePoint {
    std::string name;
    double latitude_rad, longitude_rad, undulation_m;
};

std::vector<ReferencePoint> reference_points() {
    std::ifstream in(std::filesystem::path(STARSIM_GEOID_FIXTURE_DIR) / "egm2008_points.csv");
    REQUIRE(in);
    std::vector<ReferencePoint> points;
    std::string line;
    bool header_seen = false;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (!header_seen) {  // name,latitude_deg,longitude_deg,undulation_m
            header_seen = true;
            continue;
        }
        std::stringstream row(line);
        std::string name, lat, lon, n;
        std::getline(row, name, ',');
        std::getline(row, lat, ',');
        std::getline(row, lon, ',');
        std::getline(row, n, ',');
        points.push_back({name, std::stod(lat) * deg, std::stod(lon) * deg, std::stod(n)});
    }
    return points;
}

// 3 lines (90 N, 0, 90 S) x 4 samples (180 W, 90 W, 0, 90 E), values 0.1 m x (10 sample + line).
GeographicGrid synthetic_global() {
    GeographicGrid g;
    g.grid = {3, 4, 0.1, std::vector<std::int16_t>(12)};
    for (int l = 0; l < 3; ++l) {
        for (int s = 0; s < 4; ++s) g.grid.dn[static_cast<size_t>(l * 4 + s)] = static_cast<std::int16_t>(10 * s + l);
    }
    g.north_latitude_deg = 90.0;
    g.west_longitude_deg = -180.0;
    g.step_latitude_deg = 90.0;
    g.step_longitude_deg = 90.0;
    g.global_longitude = true;
    return g;
}

}  // namespace

TEST_CASE("geographic grid wraps the antimeridian", "[environment][geoid]") {
    const auto g = synthetic_global();
    CHECK(g.value(0.0, 180.0 * deg) == g.value(0.0, -180.0 * deg));
    CHECK(g.value(0.0, -180.0 * deg) == Catch::Approx(0.1));  // line 1, sample 0
    // Between the last node (90 E, 3.1 m) and the first again (180 E = 180 W, 0.1 m): a ramp down.
    CHECK(g.value(0.0, 135.0 * deg) == Catch::Approx(0.5 * (3.1 + 0.1)));
    const double slope_per_deg = std::abs(3.1 - 0.1) / 90.0;
    const double east = g.value(0.0, 179.99 * deg), west = g.value(0.0, -179.99 * deg);
    CHECK(std::abs(east - west) <= 0.02 * slope_per_deg + 1e-12);
    CHECK(g.value(0.0, 540.0 * deg) == Catch::Approx(g.value(0.0, 180.0 * deg)));

    const auto coarse = config::load_coarse_geoid(STARSIM_CONFIG_DIR);
    CHECK(coarse.undulation_m(25.3 * deg, 180.0 * deg) == coarse.undulation_m(25.3 * deg, -180.0 * deg));
}

TEST_CASE("geographic grid clamps the poles", "[environment][geoid]") {
    const auto g = synthetic_global();
    CHECK(std::isfinite(g.value(90.0 * deg, 12.0 * deg)));
    CHECK(std::isfinite(g.value(-90.0 * deg, 12.0 * deg)));
    CHECK(g.value(95.0 * deg, 0.0) == Catch::Approx(g.value(90.0 * deg, 0.0)));
    CHECK(g.value(-95.0 * deg, 0.0) == Catch::Approx(g.value(-90.0 * deg, 0.0)));
    const auto coarse = config::load_coarse_geoid(STARSIM_CONFIG_DIR);
    CHECK(coarse.undulation_m(90.0 * deg, 0.0) == Catch::Approx(14.9).margin(0.05));   // EGM2008 north pole N ~ 14.9 m
    CHECK(coarse.undulation_m(-90.0 * deg, 0.0) == Catch::Approx(-30.1).margin(0.2));  // south pole N ~ -30 m
}

TEST_CASE("geographic grid covers a region and measures the edge distance", "[environment][geoid]") {
    GeographicGrid g;  // 5 x 5 nodes of 1 deg, 10 N .. 6 N, 20 E .. 24 E
    g.grid = {5, 5, 1.0, std::vector<std::int16_t>(25, 3)};
    g.north_latitude_deg = 10.0;
    g.west_longitude_deg = 20.0;
    g.step_latitude_deg = 1.0;
    g.step_longitude_deg = 1.0;
    constexpr double R = 6378137.0;
    CHECK(g.covers(8.0 * deg, 22.0 * deg));
    CHECK_FALSE(g.covers(11.0 * deg, 22.0 * deg));
    CHECK_FALSE(g.covers(8.0 * deg, 25.0 * deg));
    CHECK(g.value(8.0 * deg, 22.0 * deg) == Catch::Approx(3.0));
    CHECK(g.edge_distance_m(9.5 * deg, 22.0 * deg, R) == Catch::Approx(0.5 * deg * R).epsilon(1e-9));  // north edge
    CHECK(g.edge_distance_m(8.0 * deg, 22.0 * deg, R) == Catch::Approx(2.0 * deg * R * std::cos(8.0 * deg)).epsilon(1e-9));
    CHECK(g.edge_distance_m(8.0 * deg, 20.5 * deg, R) == Catch::Approx(0.5 * deg * R * std::cos(8.0 * deg)).epsilon(1e-9));
    CHECK(g.edge_distance_m(10.0 * deg, 22.0 * deg, R) == Catch::Approx(0.0).margin(1e-6));
    CHECK(g.edge_distance_m(11.0 * deg, 22.0 * deg, R) < 0.0);
    // A tile across the antimeridian: west edge 178 E, covers up to 182 E = 178 W.
    g.west_longitude_deg = 178.0;
    CHECK(g.covers(8.0 * deg, -179.0 * deg));
    CHECK(g.edge_distance_m(8.0 * deg, 179.5 * deg, R) > 0.0);

    auto global = synthetic_global();  // a global grid has no longitude edge
    CHECK(global.covers(0.0, 123.0 * deg));
    CHECK(global.edge_distance_m(0.0, 123.0 * deg, R) == Catch::Approx(90.0 * deg * R).epsilon(1e-9));
}

TEST_CASE("a regional grid clamps a point outside to the nearer edge", "[environment][geoid]") {
    GeographicGrid g;  // 3 x 5 nodes of 1 deg, 10 N .. 8 N, 20 E .. 24 E; value = 1 + sample
    g.grid = {3, 5, 1.0, std::vector<std::int16_t>(15)};
    for (int l = 0; l < 3; ++l) {
        for (int s = 0; s < 5; ++s) g.grid.dn[static_cast<size_t>(l * 5 + s)] = static_cast<std::int16_t>(1 + s);
    }
    g.north_latitude_deg = 10.0;
    g.west_longitude_deg = 20.0;
    g.step_latitude_deg = 1.0;
    g.step_longitude_deg = 1.0;
    CHECK(g.value(9.0 * deg, 19.99 * deg) == Catch::Approx(1.0));  // just west: the west edge, not the east one
    CHECK(g.value(9.0 * deg, 19.0 * deg) == Catch::Approx(1.0));
    CHECK(g.value(9.0 * deg, 24.01 * deg) == Catch::Approx(5.0));  // just east: the east edge
    CHECK(g.value(9.0 * deg, 120.0 * deg) == Catch::Approx(5.0));  // 96 deg east of it, 260 deg west: east edge
}

TEST_CASE("a global grid covers the poles exactly", "[environment][geoid]") {
    const auto g = synthetic_global();
    CHECK(g.covers(0.5 * constants::pi, 0.0));
    CHECK(g.covers(-0.5 * constants::pi, 0.0));
    CHECK(g.covers(90.0 * deg, 33.0 * deg));
    CHECK(g.covers(-90.0 * deg, -33.0 * deg));
    CHECK_FALSE(g.covers(0.5 * constants::pi + 1e-6, 0.0));
}

TEST_CASE("the coarse geoid file is refused when missing", "[environment][geoid][config]") {
    CHECK_THROWS_AS(config::load_coarse_geoid(std::filesystem::temp_directory_path() / "starsim_no_geoid_dir"),
                    config::ConfigError);
}

TEST_CASE("coarse geoid matches EGM2008 reference points", "[environment][geoid]") {
    const auto geoid = config::load_coarse_geoid(STARSIM_CONFIG_DIR);
    const auto points = reference_points();
    REQUIRE(points.size() >= 8);
    double worst = 0.0;
    for (const auto& p : points) {
        const double n = geoid.undulation_m(p.latitude_rad, p.longitude_rad);
        worst = std::max(worst, std::abs(n - p.undulation_m));
        INFO(p.name << ": N = " << n << " m, reference " << p.undulation_m << " m");
        // [E] 1-degree bilinear interpolation error of EGM2008 over ocean and coast; measured in the points file
        CHECK(std::abs(n - p.undulation_m) <= 2.0);
    }
    INFO("worst error " << worst << " m");
}

TEST_CASE("fine geoid matches EGM2008 reference points", "[environment][geoid][data]") {
    const std::filesystem::path file = std::filesystem::path(STARSIM_DATA_DIR) / "geoid" / "egm2008_25.toml";
    const auto tf = config::load_terrain_grid_file(file);
    CHECK(tf.projection == "geographic");
    CHECK(tf.global_longitude);
    CHECK(tf.height_reference == "egm2008_undulation");
    const environment::Geoid geoid(config::geographic_grid(tf));
    for (const auto& p : reference_points()) {
        INFO(p.name);
        // [E] 0.05 m: the int16 centimetre storage (0.005 m) and the points file's 4 decimals
        CHECK(std::abs(geoid.undulation_m(p.latitude_rad, p.longitude_rad) - p.undulation_m) <= 0.05);
    }
    for (const auto& c : tf.checks) {  // the pipeline's own check points
        CHECK(std::abs(geoid.undulation_m(c.latitude_rad, c.longitude_rad) - c.height_m) <= 0.01);
    }
}
