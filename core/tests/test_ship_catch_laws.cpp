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

TEST_CASE("Upright descent: propellant to stop a sink", "[ship][landing]") {
    const gnc::UprightDescentParameters p;
    // 280 t sinking 30 m/s, 20 m/s^2 of thrust, ve 3240 m/s: dv = 30 + 9.81 * 30 / (0.5 * 10.19) = 87.8 m/s
    const double dv = 30.0 + 9.81 * 30.0 / (p.deceleration_fraction * (20.0 - 9.81));
    CHECK(gnc::upright_stop_propellant_kg(280e3, 30.0, 20.0, 9.81, 3240.0, p) ==
          Catch::Approx(280e3 * (1.0 - std::exp(-dv / 3240.0))));
    CHECK(gnc::upright_stop_propellant_kg(280e3, 0.0, 20.0, 9.81, 3240.0, p) == 0.0);  // not sinking
    CHECK(gnc::upright_stop_propellant_kg(280e3, 30.0, 9.0, 9.81, 3240.0, p) > 280e3 - 1.0);  // cannot stop: all of it
}

TEST_CASE("Upright descent: brakes a fast sink and stays upright", "[ship][landing]") {
    const gnc::UprightDescentParameters p;
    // 30 m up at 40 m/s: the profile wants sqrt(4 + 2 * 0.5 * 30.2 * 30) = 30 m/s
    const auto c = gnc::upright_descent({200.0, 0.0, 0.0}, {0.0, 0.0, -40.0}, up, 30.0, 8.0, 40.0, 9.81, p);
    CHECK(c.acceleration_mps2 > 9.81);
    CHECK(tilt(c.axis) <= p.tilt_max_rad + 1e-9);
}

// Vehicle physics Task 6b, ship catch case 20: below the profile the full braking feedforward lifted the ship off 0.2 m
// above the ground until its last tonne was gone, and it fell from 1.3 m at 5 m/s.
TEST_CASE("Upright descent: slower than the profile it sinks to the ground, it does not climb", "[ship][landing]") {
    const gnc::UprightDescentParameters p;
    constexpr double g = 9.81, a_min = 5.53, a_max = 21.45, dt = 0.01;
    double h = 0.2, sink = 0.0, highest = h, burn_s = 0.0;
    while (h > 0.0 && burn_s < 30.0) {
        const auto c = gnc::upright_descent({300.0, 0.0, 0.0}, {0.0, 0.0, -sink}, up, h, a_min, a_max, g, p);
        sink += (g - c.acceleration_mps2) * dt;
        h -= sink * dt;
        highest = std::max(highest, h);
        burn_s += dt;
    }
    CHECK(highest < 0.2 + 1e-9);
    CHECK(burn_s < 3.0);
    CHECK(sink <= p.final_sink_mps + 0.5);
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
