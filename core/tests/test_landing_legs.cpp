// Landing surface and crush-core leg contact (milestone c2, pure contact units).

#include <cmath>

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
    // Feet 0.5 m deep would not reach the base (feet 1.5 m below it), 1.6 m does.
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
    // Stage 20 m east of the centre line: the east foot (at +9 m) is over the water at x = 29 > 26.
    auto pose = upright(g, 12.0, 0.003);
    pose.cg_m.x = 20.0;
    std::vector<double> crushed(4, 0.0);
    const auto c = contact::leg_contact(g, crushed, pose, ship);
    CHECK(c.feet_in_contact == 3);
    CHECK_FALSE(c.water);
    pose.cg_m.z -= 3.5;  // sunk below the sea level: that foot is in the water
    std::vector<double> crushed2(4, 0.0);
    CHECK(contact::leg_contact(g, crushed2, pose, ship).water);
}
