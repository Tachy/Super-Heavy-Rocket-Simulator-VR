// The lunar return's entry to the catch from the hand-over state of the full flight (l5 Task 8): a fast fixture for the
// profiled entry guidance - the full flight ("Lunar return direct: the ship is caught at Starbase") takes 2-20 min.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_test_macros.hpp>

#include "lunar_return_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
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

namespace {

// Printed by the full aerocapture flight with STARSIM_VERBOSE ("[aircatch] hand-over: ..."), 2026-10-16, mission
// ship_lunar_aircatch_llo, after the Task 9 bank window (entry 9,791 m/s, -4.80 deg): the state when the lunar step
// Aerocapture began (before the entry lead).
constexpr double aircatch_epoch_tdb_s = 848015409.80619097;
constexpr double aircatch_propellant_kg = 55232.594608910949;
const math::Vec3 aircatch_position_gcrf_m{7637329.4370075827, 3180820.6776978425, 694670.78027698526};
const math::Vec3 aircatch_velocity_gcrf_mps{-6418.4924096321256, 4848.528567779993, 3552.9476769158355};
const math::Quat aircatch_attitude{0.70499922608591592, 0.60293122124938459, -0.029287009969236166,
                                   -0.37227987416464947};
constexpr double aircatch_initial_bank_rad = 104.0 * deg;  // the budget's window low edge (104.0 .. 114.1 deg)

}  // namespace

namespace {

// Printed by the full aerocapture flight with STARSIM_VERBOSE ("[aircatch] b4 hand-over: ..." and "b4 tanks"),
// 2026-10-16, after the circularisation (Task 9b: free drift in b4's wait): the ship hands over to b4 in a ~200 km orbit.
constexpr double b4_epoch_tdb_s = 848086063.74418986;
constexpr double b4_fuel_kg = 9400.0;
constexpr double b4_oxidiser_kg = 33700.0;
const math::Vec3 b4_position_gcrf_m{-4526758.3164702393, 3509318.4123188877, 3231731.3137404933};
const math::Vec3 b4_velocity_gcrf_mps{-5370.6603081447665, -5391.3997768855434, -1661.8759154589909};

}  // namespace

// Task 9b: free drift in the wait (0 t for ~93 h; the held burn attitude spent 6-8 t), the deorbit 63.3 m/s, caught with
// 3.5 t left. (The 9a hand-over state, 108 h wait, reached the terminal phase with 35.1 t and ran dry in the landing:
// the catch margin itself is thin - Task 10 campaign.)
TEST_CASE("Ship return (b4) from the aerocapture's circular orbit is caught",
          "[lunar][return][aerocapture][catch][slow]") {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_aircatch_llo");
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    double fuel_kg = 0.0, oxidiser_kg = 0.0;
    for (const auto& tank : ship.stages.front().tanks) {
        (tank.contents == config::TankContents::Fuel ? fuel_kg : oxidiser_kg) += tank.mass_kg;
    }
    for (auto& tank : ship.stages.front().tanks) {
        tank.mass_kg *= tank.contents == config::TankContents::Fuel ? b4_fuel_kg / fuel_kg : b4_oxidiser_kg / oxidiser_kg;
    }
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(b4_epoch_tdb_s);
    t.session.place_in_orbit(ship, {b4_position_gcrf_m, b4_velocity_gcrf_mps}, epoch);
    gnc::DeorbitParameters p;
    p.entry_interface_height_m = m.deorbit.entry_interface_height_m;
    p.entry_flight_path_rad = m.deorbit.entry_flight_path_rad;
    p.nominal_bank_rad = m.deorbit.nominal_bank_rad;
    p.max_crossrange_m = m.deorbit.max_crossrange_m;
    p.search_window_s = m.deorbit.search_window_s;
    const bool planned = t.session.plan_ship_return(p);
    const auto* plan = t.session.ship_return_plan();
    REQUIRE(plan != nullptr);
    INFO(plan->reason);
    REQUIRE(planned);
    const auto& body = t.session.body(0);
    while (!body.caught() && !body.impacted() && t.session.epoch() - epoch < 8.0 * 86400.0) {
        const bool returning = t.session.autopilot().state == sim::AutopilotState::Return;
        t.session.advance_to(t.session.epoch() + (returning ? 5.0 : 3600.0), 2000);
        const auto& burn = t.session.autopilot().burn;
        if (std::getenv("STARSIM_VERBOSE")) {  // state changes (Task 9b: propellant through deorbit and landing)
            static int last_state = -1;
            static int calls = 0;
            const int state = static_cast<int>(t.session.autopilot().state);
            const auto tel = t.session.telemetry(0);
            const auto* es = t.session.entry_status(0);
            if (returning && tel.altitude_m < 60e3 && calls++ % 2 == 0) {
                std::printf("[b4 fixture] %.1f km %.0f m/s fpa %.1f, %.2f t, entry %s down %.2f km cross %.2f km\n",
                            tel.altitude_m / 1000.0, tel.speed_relative_mps, tel.flight_path_angle_rad / deg,
                            body.vehicle().propellant_kg() / 1000.0, es ? gnc::to_string(es->phase) : "-",
                            es ? es->downrange_error_m / 1000.0 : 0.0, es ? es->crossrange_m / 1000.0 : 0.0);
                std::fflush(stdout);
            }
            if (state != last_state || ++calls % 25 == 0) {
                last_state = state;
                std::printf("[b4 fixture] %+.1f h %s: %.1f km %.0f m/s, %.1f t, remaining %.2f, ign %.0f s, entry %s, "
                            "rate %.5f\n",
                            (t.session.epoch() - epoch) / 3600.0, sim::to_string(t.session.autopilot().state),
                            tel.altitude_m / 1000.0, tel.speed_relative_mps, body.vehicle().propellant_kg() / 1000.0,
                            burn.remaining_dv_mps, burn.time_to_ignition_s, es ? gnc::to_string(es->phase) : "-",
                            norm(body.state().angular_velocity_body_radps));
                std::fflush(stdout);
            }
        }
    }
    std::printf("[b4 fixture] plan %.1f m/s in %.1f h; %s, peak heat %.0f kW/m2, %.1f t left, %s\n", plan->delta_v_mps,
                (plan->node.epoch - epoch) / 3600.0, body.caught() ? "caught" : body.impacted() ? "impacted" : "open",
                body.peak_heat_rate_w_m2() / 1000.0, body.vehicle().propellant_kg() / 1000.0,
                body.breakup_reason().c_str());
    CHECK(body.caught());
}

TEST_CASE("Lunar return aerocapture from the hand-over state exits near 400 km", "[lunar][return][entry][aerocapture][slow]") {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_aircatch_llo");
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    double full_kg = 0.0;
    for (const auto& tank : ship.stages.front().tanks) full_kg += tank.mass_kg;
    for (auto& tank : ship.stages.front().tanks) tank.mass_kg *= aircatch_propellant_kg / full_kg;
    // as the session: one bank sign, no plane steering
    auto p = gnc::EntryGuidanceParameters::aerocapture(m.aerocapture_apoapsis_m, -1.0);
    p.initial_bank_rad = aircatch_initial_bank_rad;
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(aircatch_epoch_tdb_s);
    t.session.place_entering_ship(ship, {aircatch_position_gcrf_m, aircatch_velocity_gcrf_mps, aircatch_attitude, {}},
                                  epoch, p);
    const auto& body = t.session.body(0);
    double peak_load_g = 0.0, min_height_m = 1e9;
    const auto exited = [&] {
        const auto* es = t.session.entry_status(0);
        return es && es->phase == gnc::EntryPhase::Exit;
    };
    while (!exited() && !body.impacted() && t.session.epoch() - epoch < 3600.0) {
        t.session.advance_to(t.session.epoch() + 1.0, 400);
        peak_load_g = std::max(peak_load_g, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
        min_height_m = std::min(min_height_m, t.session.telemetry(0).altitude_m);
    }
    const auto& s = body.state();
    const auto e = orbit::elements_from_state({s.position_gcrf_m, s.velocity_gcrf_mps}, constants::earth_gm_m3ps2);
    const double apoapsis_m = e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m;
    std::printf("[aircatch entry] %s after %.0f s, apoapsis %.1f km, lowest %.1f km, peak heat %.0f kW/m2, load %.2f g, "
                "%s\n",
                exited() ? "exited" : body.impacted() ? "impacted" : "open", t.session.epoch() - epoch,
                e.eccentricity < 1.0 ? apoapsis_m / 1000.0 : -1.0, min_height_m / 1000.0,
                body.peak_heat_rate_w_m2() / 1000.0, peak_load_g, body.breakup_reason().c_str());
    REQUIRE(exited());
    CHECK(e.eccentricity < 1.0);
    CHECK(std::abs(apoapsis_m - 400e3) < 100e3);
    CHECK(body.peak_heat_rate_w_m2() < 1.2e6);
}

// Hidden probe (Task 9b): the aerocapture pass from the hand-over state with truth dispersions from the environment
// (STARSIM_PROBE_DENSITY, STARSIM_PROBE_AERO) - where the guidance's reach ends.
TEST_CASE("Aerocapture dispersion probe", "[.][probe][aerocapture]") {
    const auto factor = [](const char* name) {
        const char* v = std::getenv(name);
        return v ? std::atof(v) : 1.0;
    };
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_aircatch_llo");
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    double full_kg = 0.0;
    for (const auto& tank : ship.stages.front().tanks) full_kg += tank.mass_kg;
    for (auto& tank : ship.stages.front().tanks) tank.mass_kg *= aircatch_propellant_kg / full_kg;
    auto p = gnc::EntryGuidanceParameters::aerocapture(m.aerocapture_apoapsis_m, -1.0);
    p.initial_bank_rad = aircatch_initial_bank_rad;
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(aircatch_epoch_tdb_s);
    t.session.place_entering_ship(ship, {aircatch_position_gcrf_m, aircatch_velocity_gcrf_mps, aircatch_attitude, {}},
                                  epoch, p);
    t.session.set_air_density_factor(factor("STARSIM_PROBE_DENSITY"));
    t.session.set_aero_dispersion(0, factor("STARSIM_PROBE_AERO"));
    const auto& body = t.session.body(0);
    double min_height_m = 1e9;
    const auto exited = [&] {
        const auto* es = t.session.entry_status(0);
        return es && es->phase == gnc::EntryPhase::Exit;
    };
    while (!exited() && !body.impacted() && t.session.epoch() - epoch < 3600.0 &&
           !(min_height_m < 40e3 && t.session.telemetry(0).altitude_m < 40e3)) {
        t.session.advance_to(t.session.epoch() + 1.0, 400);
        min_height_m = std::min(min_height_m, t.session.telemetry(0).altitude_m);
    }
    const auto& s = body.state();
    const auto e = orbit::elements_from_state({s.position_gcrf_m, s.velocity_gcrf_mps}, constants::earth_gm_m3ps2);
    std::printf("[aircatch probe] density x%.2f aero x%.2f: %s, apoapsis %.1f km, lowest %.1f km, peak %.0f kW/m2, %s\n",
                factor("STARSIM_PROBE_DENSITY"), factor("STARSIM_PROBE_AERO"),
                exited() ? "exited" : body.impacted() ? "impacted" : "stays in",
                e.eccentricity < 1.0 ? (e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m) / 1000.0 : -1.0,
                min_height_m / 1000.0, body.peak_heat_rate_w_m2() / 1000.0, body.breakup_reason().c_str());
}
