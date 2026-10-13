// Falcon 9 return to the landing zone (milestone c3): entry burn, landing on SCvx plans onto the legs at LZ-40.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const spice::SpiceEphemeris& de440() {
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
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 10, 40, 0.0});
}

sim::LaunchSite lz40() {
    return sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40"));
}

gnc::BoosterReturnParameters falcon9_parameters() {
    return gnc::booster_return_parameters(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5").stages.front());
}

}  // namespace

TEST_CASE("A Falcon 9 MECO plan keeps the entry burn's propellant", "[gnc][falcon9][rtls]") {
    // The stack after ~2 min of flight east of the Cape, coasting north-east at ~1.9 km/s.
    const auto t0 = start();
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const vehicle::Vehicle stack(cfg, STARSIM_CONFIG_DIR);
    const auto site = lz40();
    const gnc::LandingSite landing{site.latitude_rad, site.longitude_rad, site.height_m, {}, {}, {}};
    const auto tr = earth().fixed_to_gcrf(t0);
    const Vec3 r_fixed = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad + 0.6 * constants::deg_to_rad, 60e3});
    const Vec3 up = math::normalized(r_fixed);
    const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
    const math::StateVector state{tr.position(r_fixed), tr.velocity(r_fixed, east * 1500.0 + up * 1000.0)};
    auto with_entry = falcon9_parameters();
    auto without = with_entry;
    without.entry_altitude_m = 0.0;
    const double landing_kg = 11300.0;
    const double a = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, state, earth(), t0, landing, with_entry, landing_kg);
    const double b = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, state, earth(), t0, landing, without, landing_kg);
    // Entry burn by the rocket equation on the stage (dry 25.6 t + 11.3 t) at the centre engine's vacuum exhaust
    // velocity (311 s): ~1.6 t (the CRS-35 gauge: 3.5 -> 3.1 % of 395.7 t), more after the boostback's leverage.
    std::printf("[falcon9] MECO propellant with entry burn %.1f t, without %.1f t\n", a / 1e3, b / 1e3);
    CHECK(a - b > 1500.0);
    CHECK(a - b < 3000.0);
}

TEST_CASE("A falling Falcon 9 stage flies its entry burn on the centre engine", "[gnc][falcon9][rtls]") {
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(lz40());
    const auto cfg = test::booster_config("falcon9_block5", 14000.0);
    // 70 km above the zone, coming back from the east at 300 m/s, falling at 1100 m/s, engines first.
    const Vec3 velocity{-300.0, 0.0, -1100.0};
    session.place_returning_booster(cfg, test::state_over_site(session.launch_site(), earth(), t0, {20e3, 0.0, 70e3}, velocity),
                                    t0, falcon9_parameters(), gnc::ReturnPhase::Coast);
    bool burned = false, ended = false;
    double ignition_altitude_m = 0.0;
    std::vector<std::string> groups;
    for (double t = 0.0; t < 60.0 && !ended;) {
        t += sim::FlightSession::tick_s;
        session.advance_to(t0 + t, 10);
        const auto* status = session.return_status(0);
        REQUIRE(status != nullptr);
        if (status->phase == gnc::ReturnPhase::EntryBurn && !burned) {
            burned = true;
            groups = status->run_groups;
            const auto tr = earth().fixed_to_gcrf(session.epoch());
            ignition_altitude_m = frames::fixed_to_geodetic(transpose(tr.rotation) * session.body(0).state().position_gcrf_m).height_m -
                                  session.launch_site().height_m;
        }
        if (burned && status->phase != gnc::ReturnPhase::EntryBurn) ended = true;
    }
    const auto* guidance = session.return_guidance(0);
    std::printf("[falcon9] entry burn from %.1f km, %.0f m/s burned\n", ignition_altitude_m / 1e3, guidance->entry_delta_v_mps());
    REQUIRE(burned);
    REQUIRE(ended);
    CHECK(groups == std::vector<std::string>{"first_stage/center"});
    CHECK(ignition_altitude_m < 56.6e3);
    CHECK(ignition_altitude_m > 55.0e3);
    CHECK(guidance->entry_delta_v_mps() >= 125.0);
    CHECK(guidance->entry_delta_v_mps() < 130.0);
    CHECK(guidance->phase() == gnc::ReturnPhase::Coast);
}
