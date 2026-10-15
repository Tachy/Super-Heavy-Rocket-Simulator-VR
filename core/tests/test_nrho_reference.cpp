// NRHO reference in the full force model (DE440 Earth, Moon, Sun + Earth J2) by multiple shooting from the CR3BP 9:2
// southern L2 NRHO: continuity, period, perilune / apolune radii and their positions over the poles vs the literature.

#include <cmath>
#include <cstdio>
#include <stdexcept>

#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
mission::NrhoSeed seed() {
    // [S: Zimovan, Howell & Davis 2017, Table 1] 9:2 southern L2 NRHO at apolune
    return {{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2};
}
}  // namespace

TEST_CASE("NRHO reference by multiple shooting in the full force model", "[mission][nrho][validation]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    const auto first_perilune = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    const mission::NrhoReference nrho{gravity, ephemeris, seed(), first_perilune};

    std::printf("[validation] NRHO shooting: %d iterations, continuity %.3f m\n", nrho.iterations(),
                nrho.max_continuity_error_m());
    CHECK(nrho.max_continuity_error_m() < 1.0);
    REQUIRE(nrho.perilune_epochs().size() >= 6);
    CHECK(nrho.perilune_epochs().front() == first_perilune);

    auto moon_relative = [&](time::Epoch t) {
        const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t);
        return nrho.state_gcrf(t).position_m - moon.position_m;
    };
    for (size_t i = 0; i + 1 < nrho.perilune_epochs().size(); ++i) {
        const auto tp = nrho.perilune_epochs()[i];
        const auto ta = nrho.apolune_epochs()[i];
        const double period_d = (nrho.perilune_epochs()[i + 1] - tp) / 86400.0;
        const double rp_km = norm(moon_relative(tp)) / 1000.0;
        const double ra_km = norm(moon_relative(ta)) / 1000.0;
        // pole: MOON_ME z in GCRF
        const auto pole_m = moon_me.fixed_to_gcrf(tp).rotation;
        const math::Vec3 pole{pole_m(0, 2), pole_m(1, 2), pole_m(2, 2)};
        const double peri_lat_deg = std::asin(dot(normalized(moon_relative(tp)), pole)) / constants::deg_to_rad;
        const double apo_lat_deg = std::asin(dot(normalized(moon_relative(ta)), pole)) / constants::deg_to_rad;
        std::printf("[validation] NRHO rev %zu: period %.3f d, perilune %.0f km (lat %+.1f), apolune %.0f km (lat %+.1f)\n",
                    i, period_d, rp_km, peri_lat_deg, ra_km, apo_lat_deg);
        // [S: Davis et al., AAS 2020, 15-year Gateway NRHO in the ephemeris model: perilune-to-perilune 6.26-6.76 d
        // (mean 6.56), rp 3,195-3,557 km, ra 70,005-71,849 km; perilune north, apolune south]. This reference is a
        // six-revolution fit to CR3BP perilunes, not Gateway's long-span design: its perilune radius swings by
        // ~+-350 km between revolutions (2,860 km in one), hence the wider radius band.
        CHECK(period_d > 6.2);
        CHECK(period_d < 6.8);
        CHECK(rp_km > 2800.0);
        CHECK(rp_km < 3800.0);
        CHECK(ra_km > 66000.0);
        CHECK(ra_km < 73000.0);
        CHECK(peri_lat_deg > 60.0);
        CHECK(apo_lat_deg < -60.0);
    }
}

TEST_CASE("NRHO reference refuses epochs outside its span", "[mission][nrho]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    const auto first_perilune = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    mission::NrhoReferenceOptions options;
    options.revolutions = 2;
    const mission::NrhoReference nrho{gravity, ephemeris, seed(), first_perilune, options};
    CHECK(nrho.covers(first_perilune));
    CHECK_FALSE(nrho.covers(first_perilune + 30.0 * 86400.0));
    CHECK_THROWS_AS(nrho.state_gcrf(first_perilune + 30.0 * 86400.0), std::out_of_range);
}
