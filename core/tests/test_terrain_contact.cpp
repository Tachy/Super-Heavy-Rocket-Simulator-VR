// Landing surfaces with a normal (lunar l2): sloped planes and terrain under the feet.

#include <cmath>
#include <memory>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/contact/landing_legs.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

config::LandingLegConfig legs() {
    config::LandingLegConfig c;
    c.azimuths_rad = {0.0, 60 * deg, 120 * deg, 180 * deg, 240 * deg, 300 * deg};
    c.footprint_radius_m = 8.0;
    c.foot_station_m = -1.5;
    c.stroke_m = 0.6;
    c.crush_force_n = 5e5;
    c.stiffness_n_m = 4e7;
    c.damping_n_s_m = 8e5;
    c.friction = 0.6;
    c.fixed = true;
    return c;
}

// Upright stage (body x up, y north, z west) with its CG `cg_height_m` above the site origin, at rest.
contact::SitePose upright(double cg_height_m) {
    contact::SitePose pose;
    pose.cg_station_m = 20.0;
    pose.body_to_site = contact::from_columns({0, 0, 1}, {0, 1, 0}, {-1, 0, 0});
    pose.cg_m = {0.0, 0.0, cg_height_m};
    return pose;
}

}  // namespace

TEST_CASE("A plane field rises towards its uphill azimuth with a tilted normal", "[contact][terrain]") {
    const contact::PlaneField plane(10.0 * deg, 90.0 * deg);  // rising towards the east
    const auto s = plane.below(100.0, 50.0);
    CHECK(s.height_m == Catch::Approx(100.0 * std::tan(10.0 * deg)));
    CHECK(s.normal.x == Catch::Approx(-std::sin(10.0 * deg)));
    CHECK(s.normal.z == Catch::Approx(std::cos(10.0 * deg)));
    CHECK(s.normal.y == Catch::Approx(0.0).margin(1e-15));
    CHECK(!s.water);
}

TEST_CASE("Flat ground keeps the vertical contact force", "[contact][terrain]") {
    const auto geometry = contact::make_leg_geometry(legs(), 0.0, 4.5);
    std::vector<double> crushed;
    const auto c = contact::leg_contact(geometry, crushed, upright(21.5 - 0.01), contact::LandingSurface{});
    CHECK(c.feet_in_contact == 6);
    CHECK(c.force_n.z == Catch::Approx(6 * 4e7 * 0.01));
    CHECK(c.force_n.x == 0.0);
    CHECK(c.force_n.y == 0.0);
}

TEST_CASE("On a slope the feet push along the normal", "[contact][terrain]") {
    const auto geometry = contact::make_leg_geometry(legs(), 0.0, 4.5);
    contact::LandingSurface slope;
    slope.field = std::make_shared<contact::PlaneField>(10.0 * deg, 90.0 * deg);
    std::vector<double> crushed;
    // Uphill (east) feet are in the ground, downhill ones above it; at rest no friction: a sum of normals.
    const auto c = contact::leg_contact(geometry, crushed, upright(21.5 - 0.02), slope);
    REQUIRE(c.feet_in_contact > 0);
    REQUIRE(c.feet_in_contact < 6);
    CHECK(c.force_n.x < 0.0);  // leans downhill (west)
    CHECK(c.force_n.x / c.force_n.z == Catch::Approx(-std::tan(10.0 * deg)));
    CHECK(c.force_n.y == Catch::Approx(0.0).margin(1e-6 * c.force_n.z));
}

TEST_CASE("Friction holds a foot on a slope against its slip", "[contact][terrain]") {
    const auto geometry = contact::make_leg_geometry(legs(), 0.0, 4.5);
    contact::LandingSurface slope;
    slope.field = std::make_shared<contact::PlaneField>(10.0 * deg, 90.0 * deg);
    // the uphill (east) feet stand 8 cos 30 deg = 6.93 m east: 2 cm into the plane
    auto pose = upright(21.5 + 8.0 * std::cos(30.0 * deg) * std::tan(10.0 * deg) - 0.02);
    pose.velocity_mps = {-0.05, 0.0, 0.0};  // sliding downhill
    std::vector<double> crushed;
    const auto c = contact::leg_contact(geometry, crushed, pose, slope);
    REQUIRE(c.feet_in_contact > 0);
    // Friction opposes the slip along the plane: the force has more uphill (east) push than the normals alone.
    const Vec3 n{-std::sin(10.0 * deg), 0.0, std::cos(10.0 * deg)};
    const Vec3 along = c.force_n - n * dot(c.force_n, n);
    CHECK(dot(along, Vec3{std::cos(10.0 * deg), 0.0, std::sin(10.0 * deg)}) > 0.0);
}

TEST_CASE("A terrain field follows the body's surface in the site frame", "[contact][terrain]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    auto moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    // Bare sphere: the ground drops below the site plane by d^2 / 2R; the normal is the sphere's radial direction.
    const double lat = -84.79 * deg, lon = 29.2 * deg;
    const Vec3 origin = frames::geodetic_to_fixed({lat, lon, 0.0}, moon.shape());
    const Vec3 up = math::normalized(origin);
    const Vec3 east = math::normalized(Vec3{-std::sin(lon), std::cos(lon), 0.0});
    const Vec3 north = cross(up, east);
    const contact::TerrainField field(moon, origin, contact::from_columns(east, north, up));
    const double d = 2000.0;
    const auto s = field.below(d, 0.0);
    const double R = constants::moon_mean_radius_m;
    CHECK(s.height_m == Catch::Approx(R * R / std::sqrt(R * R + d * d) - R).epsilon(1e-6));
    CHECK(s.normal.x == Catch::Approx(d / R).epsilon(1e-4));  // the radial there leans east, away from the site
    const auto here = field.below(0.0, 0.0);
    CHECK(std::abs(here.height_m) < 1e-6);
    CHECK(here.normal.z == Catch::Approx(1.0));
}
