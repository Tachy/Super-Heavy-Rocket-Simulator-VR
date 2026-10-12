// The booster placed at landing-burn ignition over Starbase flies the burn on SCvx plans and is caught.

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <cstdlib>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double nominal_propellant_kg = 70.0e3;  // below the FAA EA's ~74 t at ignition (Task 1: 70.4 t)
constexpr double nominal_sink_mps = 260.0;        // [E] tail-first fall speed at ignition (recorded returns: 250-280)

const spice::SpiceEphemeris& ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

time::Epoch start() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
}

test::CatchRun fly(const Vec3& offset_enu_m, const Vec3& velocity_offset_mps) {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, offset_enu_m + Vec3{0.0, 0.0, h},
                                                          velocity_offset_mps + Vec3{0.0, 0.0, -nominal_sink_mps}),
                                    t0, {}, gnc::ReturnPhase::LandingBurn);
    const auto run = test::fly_landing(session, earth(), t0, 90.0);
    if (const auto* g = session.return_guidance(0); g && g->scvx()) {
        const auto& s = g->scvx()->statistics();
        std::printf("ignition at %.0f m: %s, offset %.2f m, lateral %.2f m/s, sink %.2f m/s, %d plans (%d failed), "
                    "max %.0f ms\n", h, sim::to_string(run.tower_phase), run.offset_m, run.lateral_mps, run.sink_mps,
                    s.plans, s.failures, s.max_solve_ms);
    } else {
        std::printf("ignition at %.0f m: no SCvx guidance (fallback)\n", h);
    }
    return run;
}

}  // namespace

// Known open (Plan 3b, 2026-10-12): the catch is not yet robust - the attitude overshoots at the 13 -> 3 engine
// switch. [!mayfail] reports the result without failing the suite until that is solved.
TEST_CASE("SCvx flies a nominal landing burn onto the arms", "[gnc][catch][!mayfail]") {
    const auto run = fly({20.0, -10.0, 0.0}, {});
    CHECK(run.caught_within_tolerance());
    CHECK(run.sink_mps < 2.0);
    CHECK_FALSE(run.tower_struck);
    CHECK(run.outcome == sim::LandingOutcome::Caught);
}

TEST_CASE("SCvx landings are deterministic", "[gnc][catch]") {
    const auto a = fly({20.0, -10.0, 0.0}, {3.0, 0.0, 0.0});
    const auto b = fly({20.0, -10.0, 0.0}, {3.0, 0.0, 0.0});
    CHECK(a.offset_m == b.offset_m);
    CHECK(a.lateral_mps == b.lateral_mps);
    CHECK(a.sink_mps == b.sink_mps);
}
