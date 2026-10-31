// The reach estimate of an abort towards the water (physics of the live state, no plan).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/gnc/sea_reach.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Matchers::WithinAbs;

namespace {

gnc::SeaReachInput ship() {
    gnc::SeaReachInput in;
    in.distance_m = 800.0;
    in.height_m = 600.0;
    in.sink_mps = 60.0;
    in.mass_kg = 190e3;
    in.max_accel_mps2 = 26.0;
    in.exhaust_velocity_mps = 3400.0;
    in.gravity_mps2 = 9.8;
    in.tilt_max_rad = 20.0 * constants::deg_to_rad;
    return in;
}

}  // namespace

TEST_CASE("Sea reach without a distance is the upright stop", "[gnc][abort]") {
    auto in = ship();
    in.distance_m = -10.0;  // already over the water
    const auto e = gnc::estimate_sea_reach(in);
    // Fall then brake at a = 16.2 m/s^2: peak^2 (1/2g + 1/2a) = h + v0^2 / 2g.
    const double peak = std::sqrt((600.0 + 60.0 * 60.0 / 19.6) / (0.5 / 9.8 + 0.5 / 16.2));
    CHECK_THAT(e.time_s, WithinAbs((peak - 60.0) / 9.8 + peak / 16.2, 1e-9));
    CHECK_THAT(e.delta_v_mps, WithinAbs(9.8 * e.time_s + 60.0, 1e-9));
    CHECK_THAT(e.propellant_kg, WithinAbs(190e3 * (1.0 - std::exp(-e.delta_v_mps / 3400.0)), 1e-6));
}

TEST_CASE("Sea reach: bang-bang flight time at the tilt limit", "[gnc][abort]") {
    auto in = ship();
    in.sink_mps = 0.0;  // hover: a_lateral = g tan(tilt), d = a T^2 / 4
    in.height_m = 0.0;
    const auto e = gnc::estimate_sea_reach(in);
    const double a = 9.8 * std::tan(in.tilt_max_rad);
    CHECK_THAT(e.time_s, WithinAbs(2.0 * std::sqrt(800.0 / a), 1e-6));
    CHECK_THAT(e.delta_v_mps, WithinAbs(9.8 * e.time_s / std::cos(in.tilt_max_rad), 1e-6));
}

TEST_CASE("Sea reach grows with the distance, shrinks with speed towards the sea", "[gnc][abort]") {
    auto in = ship();
    const auto base = gnc::estimate_sea_reach(in);
    auto farther = in;
    farther.distance_m = 1200.0;
    CHECK(gnc::estimate_sea_reach(farther).propellant_kg > base.propellant_kg);
    auto moving = in;
    moving.seaward_speed_mps = 30.0;
    CHECK(gnc::estimate_sea_reach(moving).propellant_kg < base.propellant_kg);
    auto away = in;
    away.seaward_speed_mps = -30.0;
    CHECK(gnc::estimate_sea_reach(away).propellant_kg > base.propellant_kg);
    auto turning = in;
    turning.tilt_rate_radps = 0.2;
    CHECK(gnc::estimate_sea_reach(turning).propellant_kg > base.propellant_kg);
}
