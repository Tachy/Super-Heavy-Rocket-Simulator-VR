#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/atmospheric_refraction.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::environment;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double arcmin = deg / 60.0;
constexpr double earth_radius_m = 6371.0e3;

}  // namespace

TEST_CASE("Refraction from sea level: Laplace at 45 deg and the horizon", "[refraction][validation]") {
    const RefractionProfile profile{Us76Atmosphere{}};
    // Laplace's theorem: away from the horizon R = a (1 - b) tan z - a (b - a / 2) tan^3 z for any profile, a = n0 - 1,
    // b = H / r0 with the homogeneous-atmosphere height H = p0 / (rho0 g0) = 8434 m. 45 deg: 57.16 arcsec; tolerance
    // 0.6 %: the quadrature error at US76's lapse-rate breaks (atmospheric_refraction.cpp).
    const double a = constants::standard_air_refractivity_550nm;
    const double b = 8434.0 / earth_radius_m;
    CHECK(refraction_bending_rad(profile, earth_radius_m, 0.0, 45.0 * deg).value() ==
          Catch::Approx(a * (1.0 - b) - a * (b - a / 2.0)).epsilon(6.0e-3));
    // The horizon: Bennett 1982 (J. Navigation 35, 255) 34.5 arcmin at 1010 hPa / 10 deg C, scaled with P / T to the
    // US76 sea level (1013.25 hPa / 15 deg C): 34.0 arcmin; it depends on the lapse rate, tolerance 5 %.
    CHECK(refraction_bending_rad(profile, earth_radius_m, 0.0, 0.0).value() ==
          Catch::Approx(34.0 * arcmin).epsilon(0.05));
    CHECK_FALSE(refraction_bending_rad(profile, earth_radius_m, 0.0, -0.1 * deg).has_value());  // into the sea
}

TEST_CASE("Refraction lift table inverts the bending: ground and orbit", "[refraction]") {
    const RefractionProfile profile{Us76Atmosphere{}};
    for (const double altitude_m : {2.0, 400.0e3}) {
        const auto table = refraction_lift_table(profile, earth_radius_m, altitude_m);
        // Sources near the bottom of the table (the setting Sun's band at the horizon / limb) and further up.
        for (const double x : {0.0025, 0.01, 0.25}) {
            const double true_elevation = table.true_elevation_min_rad + x * table.true_elevation_span_rad;
            const double lift = table.lift_at(true_elevation);
            const auto bending = refraction_bending_rad(profile, earth_radius_m, altitude_m, true_elevation + lift);
            REQUIRE(bending.has_value());
            CHECK(lift == Catch::Approx(*bending).epsilon(0.01).margin(0.5 / 3600.0 * deg));
        }
    }
}
