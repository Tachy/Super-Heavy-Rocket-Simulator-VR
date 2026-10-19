// ISS i2: relative state of a chaser port against a target port (gnc/relative_navigation).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <stdexcept>

#include "starsim/gnc/relative_navigation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;

math::Mat3 axes_of(const math::Quat& q) {  // port axes rotated from the identity
    return math::Mat3::from_columns(q.rotate({1.0, 0.0, 0.0}), q.rotate({0.0, 1.0, 0.0}), q.rotate({0.0, 0.0, 1.0}));
}

// Target at the origin, axes identity, turning at the ISS orbital rate about z.
vehicle::PortKinematics target() {
    vehicle::PortKinematics t;
    t.axes_gcrf = axes_of({});
    t.angular_velocity_gcrf_radps = {0.0, 0.0, 1.1e-3};
    return t;
}

// A docked-attitude chaser (axis opposed, up equal) at `position` co-moving with the target's rotation.
vehicle::PortKinematics chaser(const vehicle::PortKinematics& t, const Vec3& position) {
    vehicle::PortKinematics c;
    c.position_gcrf_m = position;
    c.velocity_gcrf_mps = cross(t.angular_velocity_gcrf_radps, position);
    c.axes_gcrf = axes_of(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi));  // x -> -x, z stays
    c.angular_velocity_gcrf_radps = t.angular_velocity_gcrf_radps;
    return c;
}
}  // namespace

TEST_CASE("A co-rotating chaser reads zero relative velocity", "[iss][rendezvous][relnav]") {
    const auto t = target();
    const auto r = gnc::relative_port_state(chaser(t, {10.0, 0.0, 0.0}), t);
    CHECK(norm(r.position_m - Vec3{10.0, 0.0, 0.0}) < 1e-12);
    CHECK(norm(r.velocity_mps) < 1e-12);  // omega x r of the rotating frame is no relative motion
    CHECK(norm(r.angular_rate_radps) < 1e-15);
    CHECK(r.range_m == Catch::Approx(10.0));
    CHECK(r.closing_speed_mps == Catch::Approx(0.0).margin(1e-12));
    CHECK(r.misalignment_rad == Catch::Approx(0.0).margin(1e-9));
    CHECK(r.clocking_rad == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("An approaching chaser closes along the port axis", "[iss][rendezvous][relnav]") {
    const auto t = target();
    auto c = chaser(t, {10.0, 0.0, 0.0});
    c.velocity_gcrf_mps += Vec3{-0.1, 0.0, 0.0};
    const auto r = gnc::relative_port_state(c, t);
    CHECK(norm(r.velocity_mps - Vec3{-0.1, 0.0, 0.0}) < 1e-12);
    CHECK(r.closing_speed_mps == Catch::Approx(0.1));
}

TEST_CASE("Relative position is expressed in the target port frame", "[iss][rendezvous][relnav]") {
    auto t = target();
    t.position_gcrf_m = {100.0, -50.0, 20.0};
    t.axes_gcrf = axes_of(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * constants::pi));  // x_T = GCRF +y
    t.angular_velocity_gcrf_radps = {};
    auto c = chaser(t, t.position_gcrf_m + Vec3{0.0, 10.0, 0.0});
    c.axes_gcrf = axes_of(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, -0.5 * constants::pi));  // x_C = -y
    const auto r = gnc::relative_port_state(c, t);
    CHECK(norm(r.position_m - Vec3{10.0, 0.0, 0.0}) < 1e-12);
    CHECK(r.misalignment_rad == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("Misalignment and clocking of the chaser port", "[iss][rendezvous][relnav]") {
    const auto t = target();
    const math::Quat docked = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi);
    auto tilted = chaser(t, {5.0, 0.0, 0.0});
    tilted.axes_gcrf = axes_of(math::Quat::from_axis_angle({0.0, 1.0, 0.0}, 3.0 * deg) * docked);
    CHECK(gnc::relative_port_state(tilted, t).misalignment_rad == Catch::Approx(3.0 * deg).epsilon(1e-9));
    auto clocked = chaser(t, {5.0, 0.0, 0.0});
    clocked.axes_gcrf = axes_of(math::Quat::from_axis_angle({1.0, 0.0, 0.0}, 10.0 * deg) * docked);
    const auto r = gnc::relative_port_state(clocked, t);
    CHECK(r.misalignment_rad == Catch::Approx(0.0).margin(1e-9));
    CHECK(r.clocking_rad == Catch::Approx(10.0 * deg).epsilon(1e-9));  // + about the target axis
}

TEST_CASE("A covered chaser port and the relative_nav flag", "[iss][rendezvous][relnav]") {
    const auto t = target();
    auto c = chaser(t, {10.0, 0.0, 0.0});
    c.covered = true;
    CHECK(gnc::relative_port_state(c, t).chaser_port_covered);
    CHECK(gnc::parse_relative_nav("ideal") == gnc::RelativeNavMode::Ideal);
    CHECK_THROWS_AS(gnc::parse_relative_nav("lidar"), std::invalid_argument);
}
