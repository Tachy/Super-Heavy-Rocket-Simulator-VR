// Refuelling r2: propellant boil-off by a heat balance on the hull.

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/thermal_radiation.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("plate view factor to a sphere: nadir, horizon and the orientation average", "[boil_off][environment]") {
    const double h = (constants::earth_mean_radius_m + 200.0e3) / constants::earth_mean_radius_m;
    CHECK(environment::plate_view_factor_to_sphere(0.0, h) == Catch::Approx(1.0 / (h * h)).epsilon(1e-12));

    // Continuous where the sphere starts to sink below the plate's horizon, and zero once it is gone.
    const double full = std::acos(1.0 / h);
    CHECK(environment::plate_view_factor_to_sphere(full - 1e-9, h) ==
          Catch::Approx(environment::plate_view_factor_to_sphere(full + 1e-9, h)).margin(1e-7));
    const double gone = constants::pi - full;
    CHECK(environment::plate_view_factor_to_sphere(gone - 1e-9, h) == Catch::Approx(0.0).margin(1e-7));
    CHECK(environment::plate_view_factor_to_sphere(gone + 0.1, h) == 0.0);

    // Over all orientations a plate sees the sphere as a small sphere does: (1 - cos rho) / 2, sin rho = 1 / h.
    constexpr int n = 20000;
    double sum = 0.0;
    for (int k = 0; k < n; ++k) {
        sum += environment::plate_view_factor_to_sphere(std::acos(1.0 - 2.0 * (k + 0.5) / n), h);
    }
    CHECK(sum / n == Catch::Approx(0.5 * (1.0 - std::sqrt(1.0 - 1.0 / (h * h)))).epsilon(1e-6));
}

TEST_CASE("earth orbit radiation: sunlit noon side and umbra", "[boil_off][environment]") {
    const Vec3 position{constants::earth_mean_radius_m + 400.0e3, 0.0, 0.0};
    const Vec3 sun_gcrf{constants::astronomical_unit_m, 0.0, 0.0};

    const auto noon = environment::earth_orbit_radiation(position, sun_gcrf - position, nullptr);
    CHECK(noon.sunlit_fraction == 1.0);
    CHECK(noon.albedo_phase == Catch::Approx(1.0));
    CHECK(noon.solar_flux_w_m2 == Catch::Approx(constants::solar_constant_w_m2).epsilon(1e-4));
    CHECK(noon.nadir_direction.x == Catch::Approx(-1.0));
    CHECK(noon.distance_over_radius == Catch::Approx(position.x / constants::earth_mean_radius_m));

    const auto midnight = environment::earth_orbit_radiation(-position, sun_gcrf + position, nullptr);
    CHECK(midnight.sunlit_fraction == 0.0);
    CHECK(midnight.albedo_phase == 0.0);
}

TEST_CASE("absorbed flux of a flat element: sun, albedo and Earth infrared", "[boil_off][environment]") {
    environment::RadiationEnvironment env;
    env.sun_direction = {0.0, 1.0, 0.0};
    env.solar_flux_w_m2 = constants::solar_constant_w_m2;
    env.sunlit_fraction = 1.0;
    env.nadir_direction = {-1.0, 0.0, 0.0};
    env.distance_over_radius = 1.1;
    env.albedo_phase = 0.5;

    // Facing the Sun, edge-on to the Earth (the sphere half below its horizon).
    const double side = environment::plate_view_factor_to_sphere(constants::pi / 2.0, 1.1);
    CHECK(environment::absorbed_flux_w_m2({0.0, 1.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * (constants::solar_constant_w_m2 +
                               constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * side) +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * side));
    // Facing the Earth: no direct sun, the full view factor 1 / h^2.
    const double down = 1.0 / (1.1 * 1.1);
    CHECK(environment::absorbed_flux_w_m2({-1.0, 0.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * down +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * down));
    // In the umbra the direct sun is gone, albedo and infrared stay (the Earth below is still lit at phase 0.5).
    env.sunlit_fraction = 0.0;
    CHECK(environment::absorbed_flux_w_m2({0.0, 1.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * side +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * side));
}
