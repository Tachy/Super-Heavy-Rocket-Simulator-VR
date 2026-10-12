#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/lambert.hpp"

using namespace starsim;
using orbit::KeplerianElements;

namespace {
constexpr double gm_earth = 398600.435507e9;
constexpr double deg = constants::deg_to_rad;
}  // namespace

TEST_CASE("Keplerian elements round trip", "[orbit]") {
    const KeplerianElements original = GENERATE(
        KeplerianElements{6.798e6, 0.0005, 51.64 * deg, 123.0 * deg, 45.0 * deg, 300.0 * deg},  // ISS-like
        KeplerianElements{2.4e7, 0.72, 28.5 * deg, 10.0 * deg, 180.0 * deg, 2.0 * deg},         // GTO
        KeplerianElements{1.9e8, 0.966, 5.0 * deg, 350.0 * deg, 270.0 * deg, 170.0 * deg},      // translunar
        KeplerianElements{-3.0e7, 1.2, 120.0 * deg, 75.0 * deg, 15.0 * deg, 40.0 * deg});       // hyperbolic

    const auto state = orbit::state_from_elements(original, gm_earth);
    const auto back = orbit::elements_from_state(state, gm_earth);
    CHECK(back.semi_major_axis_m == Catch::Approx(original.semi_major_axis_m).epsilon(1e-10));
    CHECK(back.eccentricity == Catch::Approx(original.eccentricity).margin(1e-10));
    CHECK(back.inclination_rad == Catch::Approx(original.inclination_rad).margin(1e-10));
    CHECK(back.raan_rad == Catch::Approx(original.raan_rad).margin(1e-10));
    CHECK(back.argument_of_periapsis_rad == Catch::Approx(original.argument_of_periapsis_rad).margin(1e-8));
    CHECK(back.true_anomaly_rad == Catch::Approx(original.true_anomaly_rad).margin(1e-8));
}

TEST_CASE("Two-body propagation returns after one period", "[orbit]") {
    const KeplerianElements gto{2.4e7, 0.72, 28.5 * deg, 10.0 * deg, 180.0 * deg, 2.0 * deg};
    const auto start = orbit::state_from_elements(gto, gm_earth);
    const auto end = orbit::propagate_two_body(start, gm_earth, orbit::orbital_period_s(gto.semi_major_axis_m, gm_earth));
    CHECK(norm(end.position_m - start.position_m) < 1e-4);
    CHECK(norm(end.velocity_mps - start.velocity_mps) < 1e-7);
}

TEST_CASE("Two-body propagation advances the mean anomaly correctly", "[orbit]") {
    // Circular orbit: after a quarter period the true anomaly has advanced by 90 deg.
    const KeplerianElements circular{7.0e6, 0.0, 0.0, 0.0, 0.0, 0.0};
    const auto start = orbit::state_from_elements(circular, gm_earth);
    const auto quarter = orbit::propagate_two_body(start, gm_earth, 0.25 * orbit::orbital_period_s(7.0e6, gm_earth));
    CHECK(quarter.position_m.x == Catch::Approx(0.0).margin(1e-5));
    CHECK(quarter.position_m.y == Catch::Approx(7.0e6).epsilon(1e-12));
}

TEST_CASE("Two-body propagation handles hyperbolas backwards", "[orbit]") {
    const KeplerianElements hyperbola{-3.0e7, 1.2, 120.0 * deg, 75.0 * deg, 15.0 * deg, 40.0 * deg};
    const auto start = orbit::state_from_elements(hyperbola, gm_earth);
    const auto later = orbit::propagate_two_body(start, gm_earth, 20000.0);
    const auto back = orbit::propagate_two_body(later, gm_earth, -20000.0);
    CHECK(norm(back.position_m - start.position_m) < 1e-4);
    CHECK(orbit::specific_energy(later, gm_earth) ==
          Catch::Approx(orbit::specific_energy(start, gm_earth)).epsilon(1e-12));
}

TEST_CASE("Lambert: Curtis example 5.2", "[orbit]") {
    // r1 = (5000, 10000, 2100) km, r2 = (-14600, 2500, 7000) km, 1 h, mu = 398600 km^3/s^2.
    const double mu = 398600e9;
    const math::Vec3 r1{5000e3, 10000e3, 2100e3}, r2{-14600e3, 2500e3, 7000e3};
    const auto sol = orbit::solve_lambert(r1, r2, 3600.0, mu);
    REQUIRE(sol);
    CHECK(sol->departure_velocity_mps.x == Catch::Approx(-5992.5).margin(1.0));
    CHECK(sol->departure_velocity_mps.y == Catch::Approx(1925.4).margin(1.0));
    CHECK(sol->departure_velocity_mps.z == Catch::Approx(3245.6).margin(1.0));
    CHECK(sol->arrival_velocity_mps.x == Catch::Approx(-3312.5).margin(1.0));
    CHECK(sol->arrival_velocity_mps.y == Catch::Approx(-4196.6).margin(1.0));
    CHECK(sol->arrival_velocity_mps.z == Catch::Approx(-385.29).margin(1.0));
}

TEST_CASE("Lambert solutions reproduce the arc (elliptic, long way, hyperbolic)", "[orbit]") {
    const double mu = gm_earth;
    struct Case { math::Vec3 r1, r2; double tof; math::Vec3 normal; };
    const auto c = GENERATE(Case{{7.0e6, 0, 0}, {0, 8.0e6, 1.0e6}, 2000.0, {0, 0, 1}},
                            Case{{7.0e6, 0, 0}, {0, 8.0e6, 1.0e6}, 6000.0, {0, 0, -1}},   // long way
                            Case{{6.8e6, 0, 0}, {-3.0e8, 2.0e8, 5.0e7}, 3.0 * 86400.0, {0, 0, 1}},  // translunar
                            Case{{7.0e6, 0, 0}, {1.0e7, 2.0e7, 0}, 1500.0, {0, 0, 1}});     // hyperbolic
    const auto sol = orbit::solve_lambert(c.r1, c.r2, c.tof, mu, c.normal);
    REQUIRE(sol);
    const auto end = orbit::propagate_two_body({c.r1, sol->departure_velocity_mps}, mu, c.tof);
    CHECK(norm(end.position_m - c.r2) < 1.0);
    CHECK(norm(end.velocity_mps - sol->arrival_velocity_mps) < 1e-3);
}
