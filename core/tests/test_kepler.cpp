#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

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
