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

namespace {

// Level ground with a rock (spherical cap: height 0.2 m, plan radius 0.3 m, 0.6 m north of foot 0 at (0, 8)) and a
// shallow bowl (radius 3 m, depth 0.05 m, centred under foot 3 at (0, -8)). The disc support is the analytic maximum
// of the surface over the disc (cap: its top when the cap centre lies under the disc, else its height at the disc
// point nearest to it; bowl: at the disc point farthest from its centre).
class BumpField final : public contact::SurfaceField {
public:
    static constexpr double rock_x = 0.0, rock_y = 8.6, rock_h = 0.2, rock_r = 0.3;
    static constexpr double bowl_x = 0.0, bowl_y = -8.0, bowl_r = 3.0, bowl_d = 0.05;
    static double cap(double d) { return d < rock_r ? rock_h * (1.0 - d * d / (rock_r * rock_r)) : 0.0; }
    static double bowl(double d) { return d < bowl_r ? -bowl_d + bowl_d * d * d / (bowl_r * bowl_r) : 0.0; }
    contact::SurfaceSupport below(double e, double n) const override { return below_disc(e, n, 0.0); }
    contact::SurfaceSupport below_disc(double e, double n, double r) const override {
        const double d_rock = std::hypot(e - rock_x, n - rock_y), d_bowl = std::hypot(e - bowl_x, n - bowl_y);
        double h = d_bowl < bowl_r ? bowl(std::min(d_bowl + r, bowl_r)) : 0.0;  // level ground outside the bowl
        if (d_rock - r < rock_r) h = std::max(h, cap(std::max(d_rock - r, 0.0)));
        return {h, false, {0.0, 0.0, 1.0}};
    }
};

}  // namespace

TEST_CASE("A foot on a rock compresses that leg; a foot in a small crater extends", "[moon_surface][contact][legs]") {
    contact::LandingSurface surface;
    surface.field = std::make_shared<BumpField>();
    constexpr double preload = 0.05;  // feet 5 cm below the level ground
    const auto pose = upright(21.5 - preload);
    auto with_pad = legs();
    with_pad.foot_pad_radius_m = 1.0;
    std::vector<double> crushed;
    const auto disc = contact::leg_contact(contact::make_leg_geometry(with_pad, 0.0, 4.5), crushed, pose, surface);
    crushed.clear();
    const auto point = contact::leg_contact(contact::make_leg_geometry(legs(), 0.0, 4.5), crushed, pose, surface);
    // Foot 0 at (0, 8): the rock (0.6 m off) lies under the 1 m pad: compressed by the rock top + preload; a point foot
    // misses it. Foot 3 at (0, -8): the pad rests on the bowl at 1 m from its centre (-0.0444 m): compressed less than
    // on level ground; a point foot at the bowl floor (-0.05 m) only just touches.
    const double pad_in_bowl = BumpField::bowl(1.0);
    CHECK(std::abs(disc.foot_compression_m[0] - (BumpField::rock_h + preload)) <= 1e-3);
    CHECK(std::abs(point.foot_compression_m[0] - preload) <= 1e-3);
    CHECK(std::abs(disc.foot_compression_m[3] - (preload + pad_in_bowl)) <= 1e-3);
    CHECK(disc.foot_compression_m[3] < disc.foot_compression_m[1]);
    CHECK(std::abs(point.foot_compression_m[3] - (preload - BumpField::bowl_d)) <= 1e-3);
    for (const size_t i : {size_t{1}, size_t{2}, size_t{4}, size_t{5}}) {
        CHECK(std::abs(disc.foot_compression_m[i] - preload) <= 1e-3);
    }
}

TEST_CASE("A point foot (foot_pad_radius_m = 0) keeps the point query", "[moon_surface][contact][legs]") {
    contact::LandingSurface surface;
    surface.field = std::make_shared<BumpField>();
    const auto pose = upright(21.5 - 0.05);
    std::vector<double> a, b;
    const auto geometry = contact::make_leg_geometry(legs(), 0.0, 4.5);
    const auto with_field = contact::leg_contact(geometry, a, pose, surface);
    CHECK(legs().foot_pad_radius_m == 0.0);
    for (size_t i = 0; i < geometry.feet.size(); ++i) {
        const auto p = pose.cg_m + pose.body_to_site * (geometry.feet[i] - Vec3{pose.cg_station_m, 0.0, 0.0});
        const auto s = surface.below(p.x, p.y);
        CHECK(with_field.foot_compression_m[i] == std::max(0.0, s.height_m - p.z));
    }
    (void)b;
}
