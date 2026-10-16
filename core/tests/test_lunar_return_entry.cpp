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
// ship_lunar_return_llo: 300 s before the entry interface after the braking burn (entry 9,695 m/s, -4.79 deg).
constexpr double handover_epoch_tdb_s = 848010144.51427424;
constexpr double handover_propellant_kg = 40961.594461668617;
const math::Vec3 handover_position_gcrf_m{7652595.271483006, 2902430.5353620369, 1379164.087664051};
const math::Vec3 handover_velocity_gcrf_mps{-6422.7515679064272, 5153.728496732826, 2796.6612352269972};
const math::Quat handover_attitude{0.75604314923222615, 0.54098331220318341, -0.035684523440015471,
                                   -0.36669118779096643};
constexpr double handover_initial_bank_rad = 114.2 * deg;  // the budget's window middle in that flight

}  // namespace

// Task 8b (open): the plan aims at the average of both bank signs, the site lies ~1,000 km beside either track and
// the large reversal at lunar-return speed loses the ship - [!mayfail] until the planner aims at one track.
TEST_CASE("Lunar return entry from the hand-over state is caught at Starbase",
          "[lunar][return][entry][catch][slow][!mayfail]") {
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
