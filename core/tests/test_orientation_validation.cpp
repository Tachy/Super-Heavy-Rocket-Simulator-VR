// Validation of the Earth orientation chain (geodetic -> ITRF -> GCRF) against JPL Horizons:
// Horizons gives the Moon both geocentric and topocentric; their difference is the site position
// in ICRF. This checks WGS84 geodesy, EOP-based Earth rotation, precession and nutation together.

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/frames/geodetic.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

double max_site_error_m(const char* fixture) {
    test::spice_session();
    const auto earth = spice::SpiceOrientation::earth();
    // Both Moon vectors come from Horizons, so ephemeris differences (DE441 vs DE440) cancel out.
    const auto geocentric_rows = test::read_fixture("horizons_moon_geocentric.csv");
    double worst = 0.0;
    for (const auto& row : test::read_fixture(fixture)) {
        const auto epoch = test::epoch_from_jd_tdb(row[0]);
        const auto geocentric = std::find_if(geocentric_rows.begin(), geocentric_rows.end(),
                                             [&](const auto& g) { return g[0] == row[0]; });
        REQUIRE(geocentric != geocentric_rows.end());
        const math::Vec3 moon_topocentric_m{row[1] * 1e3, row[2] * 1e3, row[3] * 1e3};
        const math::Vec3 moon_geocentric_m{(*geocentric)[1] * 1e3, (*geocentric)[2] * 1e3,
                                           (*geocentric)[3] * 1e3};
        const math::Vec3 site_reference_gcrf_m = moon_geocentric_m - moon_topocentric_m;

        const frames::GeodeticPosition site{row[8] * constants::deg_to_rad, row[7] * constants::deg_to_rad,
                                            row[9] * 1e3};
        const auto site_gcrf_m = earth.fixed_to_gcrf(epoch).position(frames::geodetic_to_fixed(site));
        const math::Vec3 error_m = site_gcrf_m - site_reference_gcrf_m;
        if (std::getenv("STARSIM_VERBOSE")) {
            // Error in the Earth-fixed frame: separates polar motion / UT1 / height effects.
            const auto error_fixed_m = transpose(earth.fixed_to_gcrf(epoch).rotation) * error_m;
            std::printf("  jd %.3f |dr| %.4f m  fixed (%.3f, %.3f, %.3f)\n", row[0], norm(error_m),
                        error_fixed_m.x, error_fixed_m.y, error_fixed_m.z);
        }
        worst = std::max(worst, norm(error_m));
    }
    std::printf("[validation] %s: max site position error = %.4f m\n", fixture, worst);
    return worst;
}

}  // namespace

TEST_CASE("Starbase position in GCRF matches Horizons within 15 cm", "[frames][validation]") {
    CHECK(max_site_error_m("horizons_moon_topocentric_starbase_olp1.csv") < 0.15);
}

TEST_CASE("KSC LC-39A position in GCRF matches Horizons within 15 cm", "[frames][validation]") {
    CHECK(max_site_error_m("horizons_moon_topocentric_ksc_lc39a.csv") < 0.15);
}

TEST_CASE("Earth-fixed velocity transform yields the rotation speed of the surface", "[frames]") {
    test::spice_session();
    const auto earth = spice::SpiceOrientation::earth();
    const auto transform = earth.fixed_to_gcrf(time::Epoch::from_tdb_seconds_past_j2000(8.0e8));
    const math::Vec3 equator_m{constants::wgs84_semi_major_axis_m, 0.0, 0.0};
    const auto velocity_mps = transform.velocity(equator_m, {});
    // omega_E * a = 7.2921150e-5 * 6378137 = 465.10 m/s
    CHECK(norm(velocity_mps) == Catch::Approx(465.10).margin(0.05));
}

TEST_CASE("Moon principal-axes frame is available", "[frames]") {
    test::spice_session();
    const auto moon = spice::SpiceOrientation::moon();
    const auto transform = moon.fixed_to_gcrf(time::Epoch::from_tdb_seconds_past_j2000(8.0e8));
    // Rotation matrix must be orthonormal.
    const auto identity = transpose(transform.rotation) * transform.rotation;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(identity(i, j) == Catch::Approx(i == j ? 1.0 : 0.0).margin(1e-12));
        }
    }
}
