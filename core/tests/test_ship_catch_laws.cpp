// Ship catch robustness: the upright descent (pure guidance law of the abort fallback).

#include <algorithm>
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/upright_descent.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const Vec3 up{0.0, 0.0, 1.0};
double tilt(const Vec3& axis) { return std::acos(std::clamp(axis.z, -1.0, 1.0)); }
}  // namespace

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
