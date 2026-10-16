// Radiative heating (lunar l5): NASA TM-102652 equilibrium tables, interpolated by density, velocity and nose radius.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/aero/entry_heating.hpp"
#include "starsim/aero/radiative_heating.hpp"

using namespace starsim;

namespace {
const aero::RadiativeHeatingTable& table() {
    static const auto t = aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR);
    return t;
}
}  // namespace

TEST_CASE("Radiative table reproduces the tabulated values at its nodes", "[aero][heating]") {
    // transcription check: values read from the scan of NASA TM-102652 (MW/m^2)
    CHECK(table().heat_rate_w_m2(1.8410e-2, 8000.0, 5.0) == Catch::Approx(51.55e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(1.8410e-2, 11000.0, 10.0) == Catch::Approx(770.9e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 9000.0, 5.0) == Catch::Approx(0.1822e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 9500.0, 5.0) == Catch::Approx(0.6324e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 10000.0, 5.0) == Catch::Approx(1.940e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 11000.0, 5.0) == Catch::Approx(6.784e6).epsilon(1e-6));
    CHECK(table().heat_rate_w_m2(3.0592e-4, 12000.0, 1.0) == Catch::Approx(8.765e6).epsilon(1e-6));
}

TEST_CASE("Radiative heating interpolates monotonically and vanishes below 8 km/s", "[aero][heating]") {
    const double rho = 3.0592e-4;
    double last = 0.0;
    for (double v = 8000.0; v <= 12000.0; v += 250.0) {  // grows with speed
        const double q = table().heat_rate_w_m2(rho, v, 6.0);
        CHECK(q >= last);
        last = q;
    }
    // between the 5 m and 10 m radii (log-log): 6 m lies between them
    const double q5 = table().heat_rate_w_m2(rho, 11000.0, 5.0), q10 = table().heat_rate_w_m2(rho, 11000.0, 10.0);
    const double q6 = table().heat_rate_w_m2(rho, 11000.0, 6.0);
    CHECK(q6 > q5);
    CHECK(q6 < q10);
    // LEO entries stay untouched (b2/b4/d3 bit-identical): zero below 8 km/s
    CHECK(table().heat_rate_w_m2(rho, 7900.0, 6.0) == 0.0);
    CHECK(table().heat_rate_w_m2(0.0, 11000.0, 6.0) == 0.0);
}

TEST_CASE("Radiative table agrees with Tauber-Sutton inside the correlation's range", "[aero][heating][validation]") {
    // Tauber & Sutton, J. Spacecraft Rockets 28(1) 1991: q = C Rn^a rho^1.22 f(V) W/cm^2, C = 4.736e4,
    // a = 1.072e6 V^-1.88 rho^-0.325 (V m/s, rho kg/m^3, Rn m); valid 0.3-3 m, 10-16 km/s, 54-72 km; their stated fit
    // error against these tables is 20-30 %. f(V) at 11 km/s: 151 (Tauber & Sutton Table 2).
    const double rho = 3.0592e-4, v = 11000.0, rn = 2.3;
    const double a = 1.072e6 * std::pow(v, -1.88) * std::pow(rho, -0.325);
    const double tauber_w_m2 = 4.736e4 * std::pow(rn, a) * std::pow(rho, 1.22) * 151.0 * 1e4;
    CHECK(std::abs(table().heat_rate_w_m2(rho, v, rn) / tauber_w_m2 - 1.0) < 0.3);
}

TEST_CASE("Lunar-return glide exceeds the ship's heat limit, a 9.5 km/s glide does not", "[aero][heating][validation]") {
    // The l5 finding (spec amendment 2026-10-16): equilibrium glide with lift down holds the altitude where
    // q (C_L A) = m (v^2/r - g); ship 290 t, C_L A ~354 m^2 (L/D 0.71, C_D A 498 m^2), nose radius 6 m.
    const auto glide_rate = [&](double v) {
        const double r = 6.378137e6 + 63e3;
        const double q = 290e3 * (v * v / r - 9.6) / 354.0;
        const double rho = 2.0 * q / (v * v);
        return aero::entry_heat_rate_w_m2(rho, v, 6.0, &table());
    };
    CHECK(glide_rate(10900.0) > 1.2e6);
    CHECK(glide_rate(9500.0) < 0.9 * 1.2e6);
}
