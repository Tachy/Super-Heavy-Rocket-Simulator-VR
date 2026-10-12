#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/contact/pin_contact.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::contact;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using math::Vec3;

namespace {

// Arm: top surface at z = 0, 36 m long along x, 2 m wide; the second arm far away.
std::array<ArmContactState, 2> arms() {
    ArmContactState near{{{0.0, 0.0, -1.25}, math::Mat3::identity(), {18.0, 1.0, 1.25}}, {-18.0, 0.0, 0.0}, 0.0, 0.0,
                         {0.0, 1.0, 0.0}};
    ArmContactState far = near;
    far.box.center = {0.0, 100.0, -1.25};
    return {near, far};
}

// One pin of three points 1 cm inside the arm top; body frame = site frame, CG at the origin.
CatchBodyGeometry one_pin() { return {{{{0.0, 0.0, -0.01}, {0.0, 0.3, -0.01}, {0.0, 0.6, -0.01}}}, {}, 0.0, 5.0e6}; }

SitePose at_rest() { return {{}, {}, math::Mat3::identity(), {}, 0.0}; }

constexpr double pin_stiffness = 4.0e7;  // must match pin_contact.cpp

}  // namespace

TEST_CASE("Pin contact pushes the body out of the arm", "[contact]") {
    const auto c = catch_contact(one_pin(), at_rest(), arms());
    REQUIRE(c.pin_points_in_contact.size() == 1);
    CHECK(c.pin_points_in_contact[0] == 3);
    CHECK(c.all_pins_in_contact());
    CHECK_THAT(c.force_n.z, WithinRel(pin_stiffness * 0.01, 1e-12));
    CHECK_THAT(c.force_n.x, WithinAbs(0.0, 1e-9));
    CHECK_THAT(c.vertical_load_n, WithinRel(pin_stiffness * 0.01, 1e-12));
    CHECK_THAT(c.max_pin_load_n, WithinRel(pin_stiffness * 0.01, 1e-12));
    // Torque about the CG: r x F summed over the points (y = 0, 0.3, 0.6).
    CHECK_THAT(c.torque_nm.x, WithinRel(0.9 * pin_stiffness * 0.01 / 3.0, 1e-9));
}

TEST_CASE("Contact never pulls and friction opposes sliding", "[contact]") {
    SitePose leaving = at_rest();
    leaving.velocity_mps = {0.0, 0.0, 1.0};  // separating fast: damping exceeds the spring force
    CHECK(norm(catch_contact(one_pin(), leaving, arms()).force_n) == 0.0);

    SitePose sliding = at_rest();
    sliding.velocity_mps = {1.0, 0.0, 0.0};
    const auto c = catch_contact(one_pin(), sliding, arms());
    CHECK_THAT(c.force_n.x, WithinRel(-0.3 * pin_stiffness * 0.01, 1e-9));  // mu = 0.3
}

TEST_CASE("No contact above the arm", "[contact]") {
    SitePose above = at_rest();
    above.cg_m = {0.0, 0.0, 0.5};
    const auto c = catch_contact(one_pin(), above, arms());
    CHECK_FALSE(c.any_contact());
    CHECK(c.pin_points_in_contact[0] == 0);
}

TEST_CASE("Hull against the inner face pushes the arm outward", "[contact]") {
    const CatchBodyGeometry hull_only{{}, {{0.0, -0.95, -1.0}}, 0.0, 5.0e6};
    const auto c = catch_contact(hull_only, at_rest(), arms());
    CHECK(c.force_n.y < 0.0);         // body pushed towards -y
    CHECK(c.hull_push_n[0] > 0.0);    // arm pushed along its outward direction (+y)
}

TEST_CASE("Catch geometry from the pin config", "[contact]") {
    config::CatchPinConfig pins;
    pins.azimuths_rad = {90.0 * constants::deg_to_rad, 270.0 * constants::deg_to_rad};
    pins.station_m = 65.5;
    pins.protrusion_m = 1.0;
    pins.max_load_n = 5.0e6;
    const auto g = make_catch_geometry(pins, 0.0, 4.5, 2.5);
    REQUIRE(g.pins.size() == 2);
    REQUIRE(g.pins[0].size() == 3);
    CHECK_THAT(g.pin_station_m, WithinAbs(65.5, 1e-12));
    CHECK_THAT(g.pins[0][0].x, WithinAbs(65.5, 1e-12));
    CHECK_THAT(g.pins[0][1].z, WithinAbs(5.0, 1e-12));   // azimuth 90 deg: body +z, mid-pin radius 4.5 + 0.5
    CHECK_THAT(g.pins[1][1].z, WithinAbs(-5.0, 1e-12));
    CHECK(g.hull.size() == 16);
    CHECK_THAT(g.hull[0].x, WithinAbs(65.5 - 1.25, 1e-12));
    CHECK_THAT(std::hypot(g.hull[3].y, g.hull[3].z), WithinAbs(4.5, 1e-12));
    CHECK(make_catch_geometry({}, 0.0, 4.5, 2.5).pins.empty());
}
