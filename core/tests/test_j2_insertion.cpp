// Insertion into a circular orbit in the J2 field: the state at cutoff whose revolution stays closest to the radius.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "starsim/gnc/j2_insertion.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double gm = 3.986004418e14;
constexpr double radius_m = 6378137.0 + 200e3;

double spread_km(const std::vector<double>& radii) {
    const auto [lo, hi] = std::minmax_element(radii.begin(), radii.end());
    return (*hi - *lo) / 1000.0;
}

}  // namespace

TEST_CASE("Without J2 the insertion is the Kepler circular orbit", "[gnc][orbit]") {
    const auto s = gnc::j2_circular_insertion(gm, radius_m, 26.5 * deg, 40.0 * deg, 0.0);
    CHECK(s.radius_m == Catch::Approx(radius_m).margin(0.01));
    CHECK(std::abs(s.radial_speed_mps) < 1e-4);
    CHECK(s.horizontal_speed_mps == Catch::Approx(std::sqrt(gm / radius_m)).margin(1e-4));
}

TEST_CASE("A J2-aware insertion keeps the revolution near the target radius", "[gnc][orbit]") {
    for (const double inclination_deg : {26.5, 51.6, 97.0}) {
        for (const double u_deg : {0.0, 45.0, 120.0}) {
            CAPTURE(inclination_deg, u_deg);
            const double i = inclination_deg * deg, u = u_deg * deg;
            const gnc::InsertionState kepler{radius_m, 0.0, std::sqrt(gm / radius_m)};
            const auto aware = gnc::j2_circular_insertion(gm, radius_m, i, u);
            const auto kepler_r = gnc::j2_radius_profile(gm, kepler, i, u, 180);
            const auto aware_r = gnc::j2_radius_profile(gm, aware, i, u, 180);
            double mean = 0.0;
            for (const double r : aware_r) mean += r / static_cast<double>(aware_r.size());
            std::printf("[j2] i %.1f deg, u %3.0f deg: radius spread Kepler-circular %.1f km, J2-aware %.1f km (mean %+.2f km)\n",
                        inclination_deg, u_deg, spread_km(kepler_r), spread_km(aware_r), (mean - radius_m) / 1000.0);
            CHECK(spread_km(aware_r) < 0.5 * spread_km(kepler_r));
            CHECK(std::abs(mean - radius_m) < 1000.0);
        }
    }
}
