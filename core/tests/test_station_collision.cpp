// ISS i5: hull contact of a visiting vehicle with the station's collision primitives.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/contact/station_collision.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
}

TEST_CASE("point_in_capsule: depth and outward normal", "[iss][docking]") {
    const auto in = contact::point_in_capsule({0, 0, 0}, {10, 0, 0}, 2.0, {5, 1.5, 0});
    CHECK(in.inside);
    CHECK(in.depth_m == Catch::Approx(0.5));
    CHECK(in.normal.y == Catch::Approx(1.0));
    CHECK_FALSE(contact::point_in_capsule({0, 0, 0}, {10, 0, 0}, 2.0, {12.5, 0, 0}).inside);  // beyond the cap
    CHECK(contact::point_in_capsule({0, 0, 0}, {10, 0, 0}, 2.0, {11.5, 0, 0}).inside);       // in the end cap
}

TEST_CASE("Hull sample points of the Crew Dragon: trunk, tapered capsule, docking system left out", "[iss][docking]") {
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    const contact::StationCollisionParameters p;
    const auto pts = contact::hull_sample_points(dragon, p);
    REQUIRE(pts.size() > 50);
    double max_x = 0.0, max_r_top = 0.0, max_r = 0.0;
    for (const auto& q : pts) {
        max_x = std::max(max_x, q.x);
        max_r = std::max(max_r, std::hypot(q.y, q.z));
        if (q.x > 6.5) max_r_top = std::max(max_r_top, std::hypot(q.y, q.z));
    }
    CHECK(max_x <= 3.7 + 4.0 - p.port_clearance_m + 1e-9);  // below the docking port region (port at capsule 4.0 m)
    CHECK(max_r == Catch::Approx(2.0));                      // the 4 m heat shield
    CHECK(max_r_top < 1.5);                                  // the capsule narrows towards the top
}

TEST_CASE("Hull contact: a Dragon point inside the Harmony capsule is pushed out; outside nothing", "[iss][docking]") {
    const auto iss = config::load_vehicle(config_dir, "iss");
    const auto shape = contact::station_shape(iss);
    const contact::StationCollisionParameters p;
    // a point 0.1 m inside us_modules (axis x 26..69, radius 2.2) from +y, moving inward at 0.05 m/s
    const auto c = contact::hull_contact(shape, {{50.0, 2.1, 0.0}}, {{0.0, -0.05, 0.0}}, {50.0, 4.0, 0.0}, p);
    REQUIRE(c.any);
    CHECK(c.force_station_n.y > 0.0);
    CHECK(c.force_station_n.y == Catch::Approx(p.stiffness_n_per_m * 0.1 + p.damping_ns_per_m * 0.05));
    CHECK(c.max_depth_m == Catch::Approx(0.1));
    CHECK(c.torque_station_nm.z == Catch::Approx(0.0).margin(1e-6));  // the force runs through the CG line here
    CHECK_FALSE(contact::hull_contact(shape, {{50.0, 2.5, 0.0}}, {{}}, {50.0, 4.0, 0.0}, p).any);
}

TEST_CASE("Hull contact: the truss box pushes along its nearest face normal", "[iss][docking]") {
    const auto iss = config::load_vehicle(config_dir, "iss");
    const auto shape = contact::station_shape(iss);
    const contact::StationCollisionParameters p;
    // truss: centre (38, 0, -4.5), half extents (2.5, 54.5, 2.5); a point 0.05 m below its zenith face
    const auto c = contact::hull_contact(shape, {{38.0, 30.0, -6.95}}, {{}}, {38.0, 30.0, -10.0}, p);
    REQUIRE(c.any);
    CHECK(c.force_station_n.z < 0.0);  // out through the -z (zenith) face
    CHECK(c.max_depth_m == Catch::Approx(0.05));
}

TEST_CASE("Hull contact: a Dragon docked at either IDA touches nothing", "[iss][docking]") {
    const auto iss = config::load_vehicle(config_dir, "iss");
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    const auto shape = contact::station_shape(iss);
    const contact::StationCollisionParameters p;
    const auto pts = contact::hull_sample_points(dragon, p);
    const double port_station = 3.7 + 4.0;  // Dragon port in its vehicle frame
    for (const auto& port : iss.stages.front().docking_ports) {
        std::vector<Vec3> in_station;
        for (const auto& q : pts) {
            // chaser x runs against the port axis: the port point maps onto the station port, lateral q.y / q.z onto two
            // perpendicular unit vectors of the port (the hull is axisymmetric, the clocking does not matter here)
            const double along = port_station - q.x;  // outward from the station port
            in_station.push_back(port.position_m + along * port.axis + q.y * port.up + q.z * cross(port.axis, port.up));
        }
        const auto c = contact::hull_contact(shape, in_station, std::vector<Vec3>(in_station.size()), port.position_m, p);
        INFO(port.id);
        CHECK_FALSE(c.any);
    }
}
