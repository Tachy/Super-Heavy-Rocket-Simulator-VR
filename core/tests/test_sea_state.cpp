// Sea state: JONSWAP spectrum and its discretisation into regular wave components.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <stdexcept>

#include "starsim/environment/sea_state.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;

TEST_CASE("The JONSWAP spectrum integrates to Hs^2/16 and peaks at Tp", "[sea_state]") {
    const double hs = 2.5, tp = 8.0;
    const double wp = constants::two_pi / tp;
    const int n = 200000;
    const double top = 10.0 * wp, dw = top / n;
    double integral = 0.0, best_s = 0.0, best_w = 0.0;
    for (int i = 0; i < n; ++i) {
        const double w = (i + 0.5) * dw;
        const double s = environment::jonswap_spectral_density(w, hs, tp);
        integral += s * dw;
        if (s > best_s) {
            best_s = s;
            best_w = w;
        }
    }
    CHECK(integral == Approx(hs * hs / 16.0).epsilon(0.01));
    CHECK(best_w == Approx(wp).epsilon(0.01));
}

TEST_CASE("A discretised sea keeps the spectral variance", "[sea_state]") {
    const environment::SeaState sea({2.5, 8.0, 0.3, 7});
    CHECK(sea.variance_m2() == Approx(0.390625).epsilon(0.01));
    REQUIRE(sea.components().size() == 48);
    for (const auto& c : sea.components()) {
        const double g = constants::standard_gravity_mps2;
        CHECK(c.angular_frequency_radps * c.angular_frequency_radps ==
              Approx(g * c.wavenumber_radpm).epsilon(1e-12));
    }
}

TEST_CASE("The sampled elevation has the spectral variance", "[sea_state]") {
    const environment::SeaState sea({2.5, 8.0, 0.3, 7});
    double sum = 0.0;
    int count = 0;
    for (double t = 0.0; t <= 3600.0; t += 0.25) {
        const double e = sea.elevation_m(0.0, 0.0, t);
        sum += e * e;
        ++count;
    }
    CHECK(sum / count == Approx(sea.variance_m2()).epsilon(0.10));
}

TEST_CASE("The same seed gives the same sea, another seed another", "[sea_state]") {
    const environment::SeaState a({2.5, 8.0, 0.3, 7}), b({2.5, 8.0, 0.3, 7}), c({2.5, 8.0, 0.3, 8});
    CHECK(a.elevation_m(10.0, -20.0, 33.0) == b.elevation_m(10.0, -20.0, 33.0));
    CHECK(a.elevation_m(10.0, -20.0, 33.0) != c.elevation_m(10.0, -20.0, 33.0));
}

TEST_CASE("A calm sea is flat and bad parameters are refused", "[sea_state]") {
    const environment::SeaState calm({0.0, 8.0, 0.0, 1});
    CHECK(calm.calm());
    CHECK(calm.components().empty());
    CHECK(calm.elevation_m(3.0, 4.0, 5.0) == 0.0);
    CHECK_THROWS_AS(environment::SeaState({-1.0, 8.0, 0.0, 1}), std::invalid_argument);
    CHECK_THROWS_AS(environment::SeaState({2.0, 0.0, 0.0, 1}), std::invalid_argument);
}

TEST_CASE("Waves travel towards the heading", "[sea_state]") {
    const environment::SeaState sea({2.5, 8.0, constants::pi / 2.0, 3}, 1);
    REQUIRE(sea.components().size() == 1);
    const auto& w = sea.components()[0];
    const double c = w.angular_frequency_radps / w.wavenumber_radpm;
    const double e0 = sea.elevation_m(0.0, 0.0, 0.0);
    for (double t : {1.0, 7.5, 40.0}) CHECK(sea.elevation_m(c * t, 0.0, t) == Approx(e0).margin(1e-9));
    // crests do not vary along the crest line (north for an east-going sea)
    CHECK(sea.elevation_m(0.0, 500.0, 0.0) == Approx(e0).margin(1e-9));
}
