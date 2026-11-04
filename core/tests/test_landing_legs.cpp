// Landing surface and crush-core leg contact (milestone c2, pure contact units).

#include <algorithm>
#include <cmath>
#include <memory>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/landing_legs.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

config::LandingLegConfig falcon_legs() {
    return config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5").stages.front().landing_legs;
}

// Upright body (x up) whose feet sit `sink_m` below the surface (negative: above it).
contact::SitePose upright(const contact::LegGeometry& g, double cg_station_m, double sink_m, Vec3 velocity = {}) {
    contact::SitePose pose;
    pose.cg_station_m = cg_station_m;
    // body x -> site z (up), body y -> site x (east), body z -> site y (north)
    pose.body_to_site = contact::from_columns({0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    const double foot_station = g.feet.front().x;
    pose.cg_m = {0.0, 0.0, cg_station_m - foot_station - sink_m};
    pose.velocity_mps = velocity;
    return pose;
}

}  // namespace

TEST_CASE("Upright stage on four legs shares the load and has no net torque", "[contact][legs]") {
    const auto legs = falcon_legs();
    const auto g = contact::make_leg_geometry(legs, 0.0, 1.83);
    REQUIRE(g.feet.size() == 4);
    CHECK(g.feet.front().x == Catch::Approx(legs.foot_station_m));
    CHECK(std::hypot(g.feet.front().y, g.feet.front().z) == Catch::Approx(legs.footprint_radius_m));
    std::vector<double> crushed(4, 0.0);
    const contact::LandingSurface pad;
    const double weight = 27000.0 * 9.81;
    const double sink = weight / (4.0 * legs.stiffness_n_m);  // static compression of each foot
    const auto c = contact::leg_contact(g, crushed, upright(g, 12.0, sink), pad);
    CHECK(c.feet_in_contact == 4);
    CHECK(c.force_n.z == Catch::Approx(weight).epsilon(1e-6));
    CHECK(std::abs(c.force_n.x) < 1e-6);
    CHECK(norm(c.torque_nm) < 1e-3);
    CHECK_FALSE(c.base_contact);
    CHECK_FALSE(c.stroke_exhausted);
    for (double s : crushed) CHECK(s == 0.0);
}

TEST_CASE("A foot pressed beyond the crush force crushes its core plastically", "[contact][legs]") {
    const auto legs = falcon_legs();
    const auto g = contact::make_leg_geometry(legs, 0.0, 1.83);
    std::vector<double> crushed(4, 0.0);
    const contact::LandingSurface pad;
    const double elastic = legs.crush_force_n / legs.stiffness_n_m;
    auto c = contact::leg_contact(g, crushed, upright(g, 12.0, 0.2), pad);
    CHECK(c.max_foot_force_n == Catch::Approx(legs.crush_force_n));
    for (double s : crushed) CHECK(s == Catch::Approx(0.2 - elastic));
    // Unloaded and pressed again less deep: elastic only, the stroke stays.
    c = contact::leg_contact(g, crushed, upright(g, 12.0, 0.1), pad);
    CHECK(c.feet_in_contact == 0);
    CHECK(crushed.front() == Catch::Approx(0.2 - elastic));
    // Beyond the stroke.
    c = contact::leg_contact(g, crushed, upright(g, 12.0, legs.stroke_m + 0.05), pad);
    CHECK(c.stroke_exhausted);
}

TEST_CASE("Damping acts on the sink rate, never pulls", "[contact][legs]") {
    const auto legs = falcon_legs();
    const auto g = contact::make_leg_geometry(legs, 0.0, 1.83);
    std::vector<double> crushed(4, 0.0);
    const contact::LandingSurface pad;
    // Barely touching, moving up fast: no pulling force.
    const auto c = contact::leg_contact(g, crushed, upright(g, 12.0, 1e-4, {0.0, 0.0, 3.0}), pad);
    CHECK(c.force_n.z >= 0.0);
    CHECK(c.feet_in_contact == 0);
}

TEST_CASE("Base ring below the surface is a base contact", "[contact][legs]") {
    const auto legs = falcon_legs();
    const auto g = contact::make_leg_geometry(legs, 0.0, 1.83);
    std::vector<double> crushed(4, 0.0);
    const contact::LandingSurface pad;
    // Feet 0.4 m deep do not reach the base (feet foot_station_m below it), 0.1 m beyond that does.
    CHECK_FALSE(contact::leg_contact(g, crushed, upright(g, 12.0, 0.4), pad).base_contact);
    std::vector<double> crushed2(4, 0.0);
    CHECK(contact::leg_contact(g, crushed2, upright(g, 12.0, -legs.foot_station_m + 0.1), pad).base_contact);
}

TEST_CASE("Sliding feet feel Coulomb friction against the slip", "[contact][legs]") {
    const auto legs = falcon_legs();
    const auto g = contact::make_leg_geometry(legs, 0.0, 1.83);
    std::vector<double> crushed(4, 0.0);
    const contact::LandingSurface pad;
    const auto c = contact::leg_contact(g, crushed, upright(g, 12.0, 0.003, {2.0, 0.0, 0.0}), pad);
    REQUIRE(c.feet_in_contact == 4);
    CHECK(c.force_n.x == Catch::Approx(-legs.friction * c.force_n.z).epsilon(1e-9));
}

TEST_CASE("Landing surface: shoreline and drone ship deck", "[contact][legs]") {
    contact::LandingSurface pad;
    pad.sea = config::SeaConfig{90.0 * deg, 500.0};
    CHECK_FALSE(pad.below(100.0, 0.0).water);
    CHECK(pad.below(600.0, 0.0).water);
    CHECK(pad.below(600.0, 0.0).height_m == 0.0);

    contact::LandingSurface ship;
    ship.deck = contact::LandingSurface::Deck{45.0, 26.0, 90.0 * deg, 3.0};  // long axis east-west
    CHECK(ship.below(40.0, 20.0).height_m == 0.0);
    CHECK_FALSE(ship.below(40.0, 20.0).water);
    CHECK(ship.below(0.0, 30.0).water);
    CHECK(ship.below(0.0, 30.0).height_m == -3.0);
    CHECK(ship.below(50.0, 0.0).water);
}

TEST_CASE("A foot beyond the deck edge drops to the sea", "[contact][legs]") {
    const auto legs = falcon_legs();
    const auto g = contact::make_leg_geometry(legs, 0.0, 1.83);
    contact::LandingSurface ship;
    ship.deck = contact::LandingSurface::Deck{45.0, 26.0, 0.0, 3.0};
    // Stage 20 m east of the centre line: the two eastern feet (azimuths 45 / 315 deg: cos a > 0, 6.36 m east of the axis)
    // are over the water at x = 26.36 > 26, the western two (135 / 225 deg) stay on the deck at x = 13.64.
    auto pose = upright(g, 12.0, 0.003);
    pose.cg_m.x = 20.0;
    std::vector<double> crushed(4, 0.0);
    const auto c = contact::leg_contact(g, crushed, pose, ship);
    CHECK(c.feet_in_contact == 2);
    CHECK_FALSE(c.water);
    pose.cg_m.z -= 3.5;  // sunk below the sea level: the feet over the water are in the water
    std::vector<double> crushed2(4, 0.0);
    CHECK(contact::leg_contact(g, crushed2, pose, ship).water);
}

// Foot compression for the models (plan 2026-11-04, Task 6): a drawn foot raised by it along the station axis stands on the surface.
// Tilted body (the station axis 0.6 deg off the 7 deg slope's normal, so it is neither vertical nor normal), all six feet in the ground by different amounts.
// The contact point of a foot is computed independently of the core's formula: the line foot + s * axis against the plane z = tan(7 deg) * y.
TEST_CASE("Foot compression along the stroke axis puts each HLS foot on a 7 degree slope", "[contact][legs]") {
    const auto legs = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls").stages.front().landing_legs;
    REQUIRE(legs.fixed);
    const auto g = contact::make_leg_geometry(legs, 0.0, 4.5);
    REQUIRE(g.feet.size() == 6);
    const double slope_rad = 7.0 * deg;
    contact::LandingSurface slope;
    slope.field = std::make_shared<contact::PlaneField>(slope_rad, 0.0);  // rises towards the north: z = tan(slope) * y
    const double tilt_rad = slope_rad + 0.6 * deg;  // the axis leans away from the vertical towards the downhill side (south)
    const Vec3 axis{0.0, -std::sin(tilt_rad), std::cos(tilt_rad)};
    const Vec3 east{1.0, 0.0, 0.0};
    contact::SitePose pose;
    pose.cg_station_m = 12.0;
    pose.body_to_site = contact::from_columns(axis, east, cross(axis, east));
    const Vec3 normal{0.0, -std::sin(slope_rad), std::cos(slope_rad)};
    auto foot_site = [&](size_t i) { return pose.cg_m + pose.body_to_site * (g.feet[i] - Vec3{12.0, 0.0, 0.0}); };
    pose.cg_m = {0.0, 0.0, 20.0};
    double highest = -1e9;  // the foot highest above the plane (along the normal): it must be in the ground too
    for (size_t i = 0; i < 6; ++i) highest = std::max(highest, dot(normal, foot_site(i)));
    pose.cg_m -= normal * (highest + 0.005);  // the highest foot 5 mm in, the others deeper
    std::vector<double> crushed(6, 0.0);
    const auto c = contact::leg_contact(g, crushed, pose, slope);
    REQUIRE(c.foot_compression_m.size() == 6);
    CHECK(c.feet_in_contact == 6);
    double lowest_c = 1e9, highest_c = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        const Vec3 p = foot_site(i);
        // independent contact point: the foot's line along the station axis meets the plane z = tan(slope) y
        const double t = std::tan(slope_rad);
        const double s = (t * p.y - p.z) / (axis.z - t * axis.y);
        const Vec3 contact_point = p + axis * s;
        CHECK(std::abs(contact_point.z - t * contact_point.y) < 1e-9);
        CHECK(std::abs(c.foot_compression_m[i] - s) < 0.02);
        const Vec3 raised = p + axis * c.foot_compression_m[i];
        CHECK(norm(raised - contact_point) < 0.02);
        CHECK(std::abs(raised.z - t * raised.y) < 0.02);
        lowest_c = std::min(lowest_c, c.foot_compression_m[i]);
        highest_c = std::max(highest_c, c.foot_compression_m[i]);
    }
    CHECK(highest_c - lowest_c > 0.05);  // really different compressions (tilt over 12 m footprint: ~0.2 m)
    CHECK(contact::foot_compression_along_axis_m(0.01, Vec3{0.0, 0.0, 1.0}, Vec3{1.0, 0.0, 0.0}) == 0.0);  // surface parallel to the axis
    CHECK(contact::foot_compression_along_axis_m(-0.01, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 1.0}) == 0.0);
}
