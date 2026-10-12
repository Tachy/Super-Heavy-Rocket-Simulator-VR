// Validation: DE440 via SPICE against JPL Horizons (DE441) geocentric vectors.

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

struct MaxError {
    double position_m = 0.0;
    double velocity_mps = 0.0;
};

MaxError compare_with_fixture(ephemeris::Body body, const char* fixture) {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    MaxError worst;
    for (const auto& row : test::read_fixture(fixture)) {
        const auto state = ephemeris.state_gcrf(body, test::epoch_from_jd_tdb(row[0]));
        const math::Vec3 ref_position_m{row[1] * 1e3, row[2] * 1e3, row[3] * 1e3};
        const math::Vec3 ref_velocity_mps{row[4] * 1e3, row[5] * 1e3, row[6] * 1e3};
        const double dr = norm(state.position_m - ref_position_m);
        if (std::getenv("STARSIM_VERBOSE")) {
            std::printf("  jd %.3f |dr| %.4f m\n", row[0], dr);
        }
        worst.position_m = std::max(worst.position_m, dr);
        worst.velocity_mps = std::max(worst.velocity_mps, norm(state.velocity_mps - ref_velocity_mps));
    }
    std::printf("[validation] %s: max |dr| = %.4f m, max |dv| = %.3e m/s\n", fixture, worst.position_m,
                worst.velocity_mps);
    return worst;
}

}  // namespace

TEST_CASE("Moon geocentric state matches JPL Horizons within 3.5 m", "[ephemeris][validation]") {
    // Horizons serves DE441. Measured DE440-DE441 lunar difference grows linearly from 0.34 m (1990)
    // to 3.27 m (2040); the Sun agrees to < 4 cm, so the SPICE/frame chain itself is exact.
    const auto worst = compare_with_fixture(ephemeris::Body::Moon, "horizons_moon_geocentric.csv");
    CHECK(worst.position_m < 3.5);
    CHECK(worst.velocity_mps < 1e-5);
}

TEST_CASE("Sun geocentric state matches JPL Horizons within 50 m", "[ephemeris][validation]") {
    // 50 m at 1.5e11 m = 3e-10 relative: DE440 vs DE441 differences for the Earth-Moon barycentre.
    const auto worst = compare_with_fixture(ephemeris::Body::Sun, "horizons_sun_geocentric.csv");
    CHECK(worst.position_m < 50.0);
    CHECK(worst.velocity_mps < 1e-5);
}

TEST_CASE("GM values are consistent with DE440", "[ephemeris]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    CHECK(ephemeris.gm_m3ps2(ephemeris::Body::Earth) == Catch::Approx(398600.435507e9).epsilon(1e-12));
    CHECK(ephemeris.gm_m3ps2(ephemeris::Body::Moon) > 4.9028e12);
    CHECK(ephemeris.gm_m3ps2(ephemeris::Body::Moon) < 4.9029e12);
    CHECK(ephemeris.gm_m3ps2(ephemeris::Body::Sun) > 1.32712440e20);
}
