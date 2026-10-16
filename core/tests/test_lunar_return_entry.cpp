// The lunar return's entry to the catch from the hand-over state of the full flight (l5 Task 8): a fast fixture for the
// profiled entry guidance - the full flight ("Lunar return direct: the ship is caught at Starbase") takes 2-20 min.

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_test_macros.hpp>

#include "lunar_return_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/lunar_return_mission.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

// Printed by the full flight with STARSIM_VERBOSE ("[lunar return] hand-over: ..."), 2026-10-16, mission
// ship_lunar_return_llo, after Task 8b (one bank sign's track aimed, lateral offset ~957 km): the hand-over to the
// entry guidance after the braking burn (entry 9,695 m/s, -4.84 deg).
constexpr double handover_epoch_tdb_s = 848008899.5954994;
constexpr double handover_propellant_kg = 38299.974712633535;
const math::Vec3 handover_position_gcrf_m{4275858.3141308269, 4295318.109720937, 3471568.3346307138};
const math::Vec3 handover_velocity_gcrf_mps{-8233.8020619169147, 4988.3298088529864, -869.64927265182439};
const math::Quat handover_attitude{-0.48611775575449828, -0.77773069077875545, 0.31784267710805708,
                                   0.24041741361597196};
constexpr double handover_initial_bank_rad = 113.25 * deg;  // the budget's window middle (108.2 .. 118.3 deg)

}  // namespace

TEST_CASE("Lunar return entry from the hand-over state is caught at Starbase", "[lunar][return][entry][catch][slow]") {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    double full_kg = 0.0;
    for (const auto& tank : ship.stages.front().tanks) full_kg += tank.mass_kg;
    for (auto& tank : ship.stages.front().tanks) tank.mass_kg *= handover_propellant_kg / full_kg;
    auto p = gnc::EntryGuidanceParameters::lunar_return();
    p.initial_bank_rad = handover_initial_bank_rad;
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(handover_epoch_tdb_s);
    t.session.place_entering_ship(ship, {handover_position_gcrf_m, handover_velocity_gcrf_mps, handover_attitude, {}},
                                  epoch, p);
    const auto& body = t.session.body(0);
    double peak_load_g = 0.0;
    while (!body.caught() && !body.impacted() && body.resting() == false && t.session.epoch() - epoch < 7200.0) {
        t.session.advance_to(t.session.epoch() + 1.0, 400);
        peak_load_g = std::max(peak_load_g, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
    }
    const auto* es = t.session.entry_status(0);
    std::printf("[lunar return entry] %s after %.0f s, peak heat %.0f kW/m2 (radiative now %.0f), load %.2f g, "
                "phase %s, downrange %.1f km, crossrange %.1f km, %.1f t left, %s\n",
                body.caught() ? "caught" : body.impacted() ? "impacted" : "open", t.session.epoch() - epoch,
                body.peak_heat_rate_w_m2() / 1000.0, body.radiative_heat_rate_w_m2() / 1000.0, peak_load_g,
                es ? gnc::to_string(es->phase) : "-", es ? es->downrange_error_m / 1000.0 : 0.0,
                es ? es->crossrange_m / 1000.0 : 0.0, body.vehicle().propellant_kg() / 1000.0,
                body.breakup_reason().c_str());
    CHECK(body.peak_heat_rate_w_m2() < 1.2e6);
    CHECK(body.caught());
}
