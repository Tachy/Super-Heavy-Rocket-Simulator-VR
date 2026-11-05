// Landing surface and crush-core leg contact (milestone c2, pure contact units).

#include <algorithm>
#include <cmath>
#include <memory>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/landing_legs.hpp"
#include "starsim/contact/leg_kinematics.hpp"
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

// Task 6b fix 1: the leg kinematics (contact::leg_pose) - deployed the foot is the config's foot point, stowed the leg and
// its folded foot lie along the hull on the hinge radius, tip up; in between the foot moves smoothly.
TEST_CASE("HLS leg kinematics: deployed on the config foot, stowed along the hull", "[contact][legs]") {
    const auto legs = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls").stages.front().landing_legs;
    REQUIRE(contact::has_leg_kinematics(legs));
    for (const double az : legs.azimuths_rad) {
        const Vec3 radial{0.0, std::cos(az), std::sin(az)};
        const auto out = contact::leg_pose(legs, az, 1.0);
        CHECK(norm(out.foot_m - (Vec3{legs.foot_station_m, 0.0, 0.0} + legs.footprint_radius_m * radial)) < 1e-9);
        CHECK(norm(out.knuckle_m - out.foot_m - Vec3{legs.knuckle_height_m, 0.0, 0.0}) < 1e-9);
        const auto in = contact::leg_pose(legs, az, 0.0);
        const double length = norm(out.knuckle_m - out.hinge_m);
        CHECK(norm(in.knuckle_m - (in.hinge_m + Vec3{length, 0.0, 0.0})) < 1e-9);  // along the hull, tip up
        CHECK(norm(in.foot_m - (in.knuckle_m + Vec3{legs.knuckle_height_m, 0.0, 0.0})) < 1e-9);  // folded into the line
        CHECK(dot(in.foot_m, radial) == Catch::Approx(legs.hinge_radius_m));
        double path = 0.0, longest = 0.0;
        for (int k = 0; k < 20; ++k) {
            const double step = norm(contact::leg_pose(legs, az, 0.05 * (k + 1)).foot_m - contact::leg_pose(legs, az, 0.05 * k).foot_m);
            path += step;
            longest = std::max(longest, step);
        }
        CHECK(longest < 0.25 * path);
    }
    CHECK(contact::leg_deployed_angle_rad(legs) == Catch::Approx(97.5164 * deg).margin(1e-4));
}

// Fix round 1 of Task 5 (Moon procedural surface plan): Coulomb stick-slip friction. An upright HLS (translation only,
// rotation locked: the friction limit's inertia is huge, so the stick spring is tuned to the full mass) stands on a
// plane in lunar gravity, integrated semi-implicitly at the session's 200 Hz tick.
namespace {
struct SlopeRun {
    double drift_m;      // horizontal travel between t = 10 s and t = 30 s
    double speed_mps;    // horizontal speed at the end
};
SlopeRun stand_on_slope(double slope_deg, double duration_s) {
    const auto legs = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls").stages.front().landing_legs;
    const auto g = contact::make_leg_geometry(legs, 0.0, 4.5);
    contact::LandingSurface slope;
    slope.field = std::make_shared<contact::PlaneField>(slope_deg * deg, 0.0);  // rises towards the north
    constexpr double mass_kg = 315.0e3, gravity_mps2 = 1.62, dt_s = 0.005;  // HLS ~0.51 MN on the Moon
    const contact::FrictionLimit limit{mass_kg, {1e15, 1e15, 1e15}, dt_s};
    auto pose = upright(g, 12.0, 0.0);
    // start with the foot nearest the plane just touching
    double nearest = 1e9;
    for (const auto& f : g.feet) {
        const Vec3 p = pose.cg_m + pose.body_to_site * (f - Vec3{12.0, 0.0, 0.0});
        nearest = std::min(nearest, p.z - std::tan(slope_deg * deg) * p.y);
    }
    pose.cg_m.z -= nearest;
    std::vector<double> crushed;
    std::vector<contact::FootStick> stick;
    Vec3 at_10s;
    const int steps = static_cast<int>(duration_s / dt_s);
    for (int i = 0; i < steps; ++i) {
        if (i == static_cast<int>(10.0 / dt_s)) at_10s = pose.cg_m;
        const auto c = contact::leg_contact(g, crushed, pose, slope, &limit, &stick);
        pose.velocity_mps += (c.force_n / mass_kg + Vec3{0.0, 0.0, -gravity_mps2}) * dt_s;
        pose.cg_m += pose.velocity_mps * dt_s;
    }
    return {std::hypot(pose.cg_m.x - at_10s.x, pose.cg_m.y - at_10s.y),
            std::hypot(pose.velocity_mps.x, pose.velocity_mps.y)};
}
}  // namespace

TEST_CASE("Stick friction: a lander resting on a 9 degree plane does not creep", "[contact][legs]") {
    // tan 9 deg = 0.16 of the load tangential, below mu = 0.6: the pads stick. The regularised law crept at
    // 0.1 m/s x 0.16 / 0.6 ~ 2.6 cm/s (0.5 m in 20 s).
    const auto run = stand_on_slope(9.0, 30.0);
    CAPTURE(run.drift_m, run.speed_mps);
    CHECK(run.drift_m < 1.0e-3);
    CHECK(run.speed_mps < 1.0e-4);
}

TEST_CASE("Stick friction: above the friction angle the lander slides at g (sin - mu cos)", "[contact][legs]") {
    // 35 deg > atan 0.6 = 31 deg: the pads slip; along the slope a = 1.62 (sin 35 - 0.6 cos 35) = 0.133 m/s^2.
    const auto run = stand_on_slope(35.0, 12.0);
    const double expected = 1.62 * (std::sin(35.0 * deg) - 0.6 * std::cos(35.0 * deg)) * std::cos(35.0 * deg);
    CAPTURE(run.speed_mps, expected);
    // horizontal speed after 12 s of sliding from rest (the first contact settles within a fraction of a second)
    CHECK(run.speed_mps == Catch::Approx(expected * 12.0).epsilon(0.1));
}
