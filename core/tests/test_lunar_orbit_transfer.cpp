// NRHO <-> 100 km polar LLO two-burn transfers vs the literature (~0.75-0.9 km/s each way) and NRHO station keeping.

#include <algorithm>
#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/mission/lunar_orbit_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    time::Epoch first_perilune = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    // [S: Zimovan, Howell & Davis 2017, Table 1] 9:2 southern L2 NRHO at apolune
    mission::NrhoReference nrho{gravity, ephemeris,
                                mission::NrhoSeed{{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2},
                                first_perilune, {.revolutions = 6}};
    mission::LunarOrbitTransfer transfer{gravity, ephemeris, moon};
    mission::PositionTargeter coast{gravity};
};
}  // namespace

TEST_CASE("NRHO to LLO and back: delta-v vs the literature", "[mission][lunar][transfer][validation]") {
    test::spice_session();
    Fixture fx;
    const auto start = fx.first_perilune + 2.0 * 86400.0;
    const auto down = fx.transfer.nrho_to_llo(fx.nrho.state_gcrf(start), start, fx.nrho, {});
    REQUIRE(down.feasible);
    std::printf("[validation] NRHO -> LLO: %.0f + %.0f = %.0f m/s\n", norm(down.first.delta_v_gcrf_mps),
                norm(down.second.delta_v_gcrf_mps), down.total_delta_v_mps);
    CHECK(down.total_delta_v_mps > 700.0);  // [S] spec: ~0.75-0.9 km/s each way
    CHECK(down.total_delta_v_mps < 950.0);

    // Fly it impulsively and go back up from the LLO.
    auto s = fx.coast.coast(fx.nrho.state_gcrf(start), start, down.first.epoch);
    s.velocity_mps += down.first.delta_v_gcrf_mps;
    s = fx.coast.coast(s, down.first.epoch, down.second.epoch);
    s.velocity_mps += down.second.delta_v_gcrf_mps;
    const auto up = fx.transfer.llo_to_nrho(s, down.second.epoch, fx.nrho);
    REQUIRE(up.feasible);
    std::printf("[validation] LLO -> NRHO: %.0f + %.0f = %.0f m/s, departure %.1f d after arrival\n",
                norm(up.first.delta_v_gcrf_mps), norm(up.second.delta_v_gcrf_mps), up.total_delta_v_mps,
                (up.first.epoch - down.second.epoch) / 86400.0);
    CHECK(up.total_delta_v_mps > 700.0);
    CHECK(up.total_delta_v_mps < 1000.0);
}

TEST_CASE("NRHO station keeping holds the reference for three revolutions", "[mission][lunar][nrho][validation]") {
    test::spice_session();
    Fixture fx;
    // Insertion error: 0.5 m/s at the first perilune.
    auto s = fx.nrho.state_gcrf(fx.first_perilune);
    s.velocity_mps += math::Vec3{0.3, -0.3, 0.2};
    time::Epoch t = fx.first_perilune;
    double spent = 0.0, worst_km = 0.0;
    for (int rev = 0; rev < 3; ++rev) {
        const auto apolune = fx.nrho.apolune_epochs()[rev];
        s = fx.coast.coast(s, t, apolune);
        t = apolune;
        if (const auto burn = fx.transfer.station_keeping(s, t, fx.nrho)) {
            s = fx.coast.coast(s, t, burn->epoch);
            t = burn->epoch;
            s.velocity_mps += burn->delta_v_gcrf_mps;
            spent += norm(burn->delta_v_gcrf_mps);
        }
        const auto perilune = fx.nrho.perilune_epochs()[rev + 1];
        s = fx.coast.coast(s, t, perilune);
        t = perilune;
        worst_km = std::max(worst_km, norm(s.position_m - fx.nrho.state_gcrf(t).position_m) / 1000.0);
    }
    std::printf("[validation] NRHO station keeping: %.2f m/s over 3 revolutions, worst perilune offset %.2f km\n",
                spent, worst_km);
    CHECK(worst_km < 5.0);  // [E]
    CHECK(spent < 10.0);    // [E]
}
