// Ship catch robustness: the flip brake and the upright descent (pure guidance laws).

#include <algorithm>
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/flip_brake.hpp"
#include "starsim/gnc/upright_descent.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const Vec3 up{0.0, 0.0, 1.0};
double tilt(const Vec3& axis) { return std::acos(std::clamp(axis.z, -1.0, 1.0)); }
}  // namespace

TEST_CASE("Flip brake: on the axis at rest the thrust stays vertical", "[ship][landing]") {
    const auto a = gnc::flip_brake_axis({0.0, 0.0, 0.0}, {0.0, 0.0, -60.0}, up, 40.0);
    CHECK(a.z == Catch::Approx(1.0).margin(1e-9));
}

TEST_CASE("Flip brake: the seed-18 flip end brakes to stop at the axis", "[ship][landing]") {
    // 25 m short of the axis (axis along +x), 29 m/s towards it: stopping at the axis needs 29^2 / 50 = 16.8 m/s^2
    const gnc::FlipBrakeParameters p;
    const double thrust = 40.0;
    const auto a = gnc::flip_brake_axis({25.0, 0.0, 0.0}, {29.0, 0.0, -50.0}, up, thrust, p);
    CHECK(-a.x * thrust > 0.95 * 29.0 * 29.0 / 50.0);  // against the motion, about the stopping deceleration
    CHECK(tilt(a) <= p.tilt_max_rad + 1e-9);
    CHECK(std::abs(a.y) < 1e-9);
}

TEST_CASE("Flip brake: far from the axis and slow it leans towards the axis", "[ship][landing]") {
    const auto a = gnc::flip_brake_axis({0.0, 300.0, 0.0}, {0.0, 5.0, -60.0}, up, 40.0);
    CHECK(a.y > 0.0);
}

TEST_CASE("Flip brake: overshot and moving away it pushes back", "[ship][landing]") {
    // the axis lies 30 m behind (-x), the ship moves away at 10 m/s (+x)
    const auto a = gnc::flip_brake_axis({-30.0, 0.0, 0.0}, {10.0, 0.0, -40.0}, up, 40.0);
    CHECK(a.x < 0.0);
}

TEST_CASE("Flip brake: no thrust yet, the axis is the vertical; tilt never beyond the limit", "[ship][landing]") {
    const auto a = gnc::flip_brake_axis({25.0, 0.0, 0.0}, {29.0, 0.0, -50.0}, up, 0.0);
    CHECK(a.z == Catch::Approx(1.0));
    const gnc::FlipBrakeParameters p;
    for (double vx = -60.0; vx <= 60.0; vx += 7.5) {
        for (double d = -200.0; d <= 200.0; d += 37.0) {
            const auto b = gnc::flip_brake_axis({d, 0.5 * d, 0.0}, {vx, -0.3 * vx, -40.0}, up, 35.0, p);
            CHECK(tilt(b) <= p.tilt_max_rad + 1e-9);
            CHECK(std::abs(std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z) - 1.0) < 1e-12);
        }
    }
}

TEST_CASE("Upright descent: brakes a fast sink and stays upright", "[ship][landing]") {
    const gnc::UprightDescentParameters p;
    // 30 m up at 40 m/s: the profile wants sqrt(4 + 2 * 0.5 * 30.2 * 30) = 30 m/s
    const auto c = gnc::upright_descent({200.0, 0.0, 0.0}, {0.0, 0.0, -40.0}, up, 30.0, 8.0, 40.0, 9.81, p);
    CHECK(c.acceleration_mps2 > 9.81);
    CHECK(tilt(c.axis) <= p.tilt_max_rad + 1e-9);
}

TEST_CASE("Upright descent: next to the tower it moves away; above its centre it still picks a way", "[ship][landing]") {
    const auto c = gnc::upright_descent({0.0, 20.0, 0.0}, {0.0, 0.0, -5.0}, up, 100.0, 8.0, 40.0, 9.81);
    CHECK(c.axis.y > 0.0);  // away from the tower
    const auto o = gnc::upright_descent({0.0, 0.0, 0.0}, {0.0, 0.0, -5.0}, up, 100.0, 8.0, 40.0, 9.81);
    CHECK(std::isfinite(o.axis.x));
    CHECK(std::hypot(o.axis.x, o.axis.y) > 0.0);
}

TEST_CASE("Upright descent: at the ground at the final sink and clear of the tower it holds the weight", "[ship][landing]") {
    const gnc::UprightDescentParameters p;
    const auto c = gnc::upright_descent({300.0, 0.0, 0.0}, {0.0, 0.0, -p.final_sink_mps}, up, 0.0, 8.0, 40.0, 9.81, p);
    CHECK(c.acceleration_mps2 == Catch::Approx(9.81).margin(0.5));
    CHECK(c.axis.z == Catch::Approx(1.0).margin(1e-6));
    for (double vh = -30.0; vh <= 30.0; vh += 6.0) {
        const auto d = gnc::upright_descent({50.0, 10.0, 0.0}, {vh, -vh, -20.0}, up, 80.0, 8.0, 40.0, 9.81, p);
        CHECK(tilt(d.axis) <= p.tilt_max_rad + 1e-9);
    }
}
