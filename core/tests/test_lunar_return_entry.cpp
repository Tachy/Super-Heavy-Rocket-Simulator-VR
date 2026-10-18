// The lunar return's entry to the catch from the hand-over state of the full flight (l5 Task 8): a fast fixture for the
// profiled entry guidance - the full flight ("Lunar return direct: the ship is caught at Starbase") takes 2-20 min.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "lunar_return_test_support.hpp"
#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/lunar_return_mission.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

// Printed by the full flight with STARSIM_VERBOSE ("[lunar return] hand-over: ..."), 2026-10-17, mission
// ship_lunar_return_llo, after l5 Task 10 (held track aimed 300 km beyond the site, entry trim, closed-loop braking):
// the hand-over to the entry guidance after the braking burn (entry 9,695 m/s, -4.87 deg; the full flight caught 13.6 t).
// (The Task 8b state aimed the held track at the site itself: any drift to the far side could not be reversed.)
constexpr double handover_epoch_tdb_s = 848008959.11018324;
constexpr double handover_propellant_kg = 42958.659955708943;
const math::Vec3 handover_position_gcrf_m{7086860.4675348997, 2531187.6710331743, 2929169.4508524211};
const math::Vec3 handover_velocity_gcrf_mps{-6563.1519895914944, 5826.099272684959, 1092.0146363209715};
const math::Quat handover_attitude{-0.55696594189169446, -0.50181461927849202, 0.23702032888641714,
                                   0.6178935111697279};
constexpr double handover_initial_bank_rad = 113.25 * deg;  // the budget's window middle (108.2 .. 118.3 deg)

}  // namespace

namespace {

struct HandoverFlight {
    bool caught = false;
    bool impacted = false;
    double seconds = 0.0;
    double peak_heat_w_m2 = 0.0;
    double peak_load_g = 0.0;
    double propellant_t = 0.0;
    double downrange_km = 0.0;
    double crossrange_km = 0.0;
    double gate_height_m = -1.0;  // height where the entry handed over to the flip (-1: never)
    std::string phase;
    std::string breakup;
};

// Flies the direct entry from the hand-over state to the catch (or impact, or `limit_s`), with truth air density and
// hull aero factors.
struct HandoverState {
    double epoch_tdb_s = handover_epoch_tdb_s;
    double propellant_kg = handover_propellant_kg;
    math::Vec3 position_gcrf_m = handover_position_gcrf_m;
    math::Vec3 velocity_gcrf_mps = handover_velocity_gcrf_mps;
    math::Quat attitude = handover_attitude;
    double initial_bank_rad = handover_initial_bank_rad;
    std::string mission = "ship_lunar_return_llo";
};

HandoverFlight fly_handover(double density_factor, double aero_factor, double limit_s = 7200.0,
                            const HandoverState& h = {}) {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, h.mission);
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    double full_kg = 0.0;
    for (const auto& tank : ship.stages.front().tanks) full_kg += tank.mass_kg;
    for (auto& tank : ship.stages.front().tanks) tank.mass_kg *= h.propellant_kg / full_kg;
    auto p = gnc::EntryGuidanceParameters::lunar_return();
    p.initial_bank_rad = h.initial_bank_rad;
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(h.epoch_tdb_s);
    t.session.place_entering_ship(ship, {h.position_gcrf_m, h.velocity_gcrf_mps, h.attitude, {}}, epoch, p);
    t.session.set_air_density_factor(density_factor);
    t.session.set_aero_dispersion(0, aero_factor);
    const auto& body = t.session.body(0);
    if (const char* flaps = std::getenv("STARSIM_PROBE_FLAPS")) {  // diagnostics: initial flap deflections, "90,0,0,90"
        auto& vehicle = const_cast<vehicle::FlightBody&>(body).vehicle();
        const char* c = flaps;
        for (auto& stage : vehicle.stages()) {
            for (auto& flap : stage.flaps) {
                flap.deflection_rad = std::atof(c) * deg;
                while (*c && *c != ',') ++c;
                if (*c == ',') ++c;
            }
        }
    }
    HandoverFlight f;
    while (!body.caught() && !body.impacted() && body.resting() == false && t.session.epoch() - epoch < limit_s) {
        t.session.advance_to(t.session.epoch() + 1.0, 400);
        f.peak_load_g = std::max(f.peak_load_g, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
        if (const auto* gate = t.session.entry_status(0); gate && gate->phase == gnc::EntryPhase::Gate && f.gate_height_m < 0.0) {
            f.gate_height_m = t.session.telemetry(0).altitude_m;
        }
    }
    const auto* es = t.session.entry_status(0);
    f.caught = body.caught();
    f.impacted = body.impacted();
    f.seconds = t.session.epoch() - epoch;
    f.peak_heat_w_m2 = body.peak_heat_rate_w_m2();
    f.propellant_t = body.vehicle().propellant_kg() / 1000.0;
    f.phase = es ? gnc::to_string(es->phase) : "-";
    f.downrange_km = es ? es->downrange_error_m / 1000.0 : 0.0;
    f.crossrange_km = es ? es->crossrange_m / 1000.0 : 0.0;
    f.breakup = body.breakup_reason();
    std::printf("[lunar return entry] density x%.3f aero x%.3f: %s after %.0f s, peak heat %.0f kW/m2, load %.2f g, "
                "phase %s, downrange %.1f km, crossrange %.1f km, %.1f t left, %s\n",
                density_factor, aero_factor, f.caught ? "caught" : f.impacted ? "impacted" : "open", f.seconds,
                f.peak_heat_w_m2 / 1000.0, f.peak_load_g, f.phase.c_str(), f.downrange_km, f.crossrange_km,
                f.propellant_t, f.breakup.c_str());
    return f;
}

}  // namespace

TEST_CASE("Lunar return entry from the hand-over state is caught at Starbase", "[lunar][return][entry][catch][slow]") {
    const auto f = fly_handover(1.0, 1.0);
    CHECK(f.peak_heat_w_m2 < 1.2e6);
    CHECK(f.caught);
}

// l5 Task 10: the entry controller's flaps trim the MODELLED hull moment; with the true hull aero x0.92 a PD law alone
// held an alpha error growing with the dynamic pressure (11 deg at 32 kPa), the drag ran 40 % low and the ship came
// 98 km short of the gate. The estimated hull moment scale (FlightSession::update_hull_moment_scale, k = 0.918) trims
// it: caught.
TEST_CASE("Lunar return entry with the hull aero 8 % low is caught (hull moment scale)",
          "[lunar][return][entry][catch][slow]") {
    const auto f = fly_handover(1.0, 0.92);
    CHECK(std::abs(f.downrange_km) < 1.0);
    CHECK(f.caught);
}

// l5 final review (Review Focus 3): a braking burn that was not flown or cut short - the ship enters faster than
// planned. The guidance cuts the bank back on the predicted limits; beyond the corridor the outcome is honest. The
// gate's sink-scaled hand-over height (gate x (sink / 100 m/s)^2) once fired hundreds of km up for such a dive: no
// flip outside the dense air.
TEST_CASE("Lunar return entry after a short braking burn hands over to the flip only near the ground",
          "[lunar][return][entry][slow]") {
    for (const double missing_mps : {500.0, 1030.0}) {  // half of the burn, none of it (~1,030 m/s planned)
        // the same two-body perigee at the higher speed (the braking burn changed the speed, not the aim point)
        HandoverState h;
        const double mu = constants::earth_gm_m3ps2;
        const math::Vec3 r = h.position_gcrf_m, v = h.velocity_gcrf_mps;
        const double r_m = norm(r), hm = norm(cross(r, v)), energy = 0.5 * dot(v, v) - mu / r_m;
        const double perigee_m = hm * hm / mu / (1.0 + std::sqrt(1.0 + 2.0 * energy * hm * hm / (mu * mu)));
        const double speed = norm(v) + missing_mps;
        const double v_perigee = std::sqrt(speed * speed - 2.0 * mu / r_m + 2.0 * mu / perigee_m);
        const double v_horizontal = perigee_m * v_perigee / r_m;
        const math::Vec3 up = math::normalized(r);
        const math::Vec3 along = math::normalized(v - up * dot(v, up));
        h.velocity_gcrf_mps = along * v_horizontal - up * std::sqrt(speed * speed - v_horizontal * v_horizontal);
        const auto f = fly_handover(1.0, 1.0, 7200.0, h);
        std::printf("[lunar return entry] braking short by %.0f m/s: gate at %.0f m, '%s'\n", missing_mps,
                    f.gate_height_m, f.breakup.c_str());
        INFO("short by " << missing_mps << " m/s");
        CHECK(f.gate_height_m < 5000.0);  // never handed over, or near the gate (800 m, more in a fast fall)
        CHECK((f.caught || f.impacted || !f.breakup.empty()));
    }
}

// Hidden probe (Task 10): the direct entry from the hand-over state with truth dispersions from the environment
// (STARSIM_PROBE_DENSITY, STARSIM_PROBE_AERO, STARSIM_PROBE_SECONDS).
TEST_CASE("Lunar return entry from a printed hand-over state", "[.lunar_handover_replay]") {
    // STARSIM_HANDOVER_STATE: the 12 numbers of a '[lunar return] hand-over:' line (epoch, propellant, r, v, q);
    // STARSIM_HANDOVER_BANK_DEG: the plan's initial bank; STARSIM_HANDOVER_MISSION: the mission id.
    const char* text = std::getenv("STARSIM_HANDOVER_STATE");
    REQUIRE(text != nullptr);
    double g[12] = {};
    const char* c = text;
    for (double& value : g) {
        char* end = nullptr;
        value = std::strtod(c, &end);
        REQUIRE(end != c);
        c = end;
    }
    HandoverState h;
    h.epoch_tdb_s = g[0];
    h.propellant_kg = g[1];
    h.position_gcrf_m = {g[2], g[3], g[4]};
    h.velocity_gcrf_mps = {g[5], g[6], g[7]};
    h.attitude = math::Quat{g[8], g[9], g[10], g[11]};
    if (const char* b = std::getenv("STARSIM_HANDOVER_BANK_DEG")) h.initial_bank_rad = std::atof(b) * deg;
    if (const char* id = std::getenv("STARSIM_HANDOVER_MISSION")) h.mission = id;
    const auto f = fly_handover(1.0, 1.0, 7200.0, h);
    CHECK(f.caught);
}

TEST_CASE("Lunar return entry dispersion probe", "[.][probe][entry]") {
    const auto factor = [](const char* name, double fallback) {
        const char* v = std::getenv(name);
        return v ? std::atof(v) : fallback;
    };
    fly_handover(factor("STARSIM_PROBE_DENSITY", 1.0), factor("STARSIM_PROBE_AERO", 1.0),
                 factor("STARSIM_PROBE_SECONDS", 7200.0));
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

// l5 Task 10 (aircatch campaign seed 7): a shallow pass whose lowest point stays above 80 km climbed out to a
// 26,000 km apoapsis without phase Exit - the session ticked the guidance at 200 Hz for days. The hand-over velocity
// turned 0.3 deg shallower (lowest point 80-120 km; 0.6 deg misses the air at 146 km): the pass must end in Exit.
TEST_CASE("A shallow aerocapture pass that stays above 80 km ends in Exit", "[lunar][return][entry][aerocapture][slow]") {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_aircatch_llo");
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    double full_kg = 0.0;
    for (const auto& tank : ship.stages.front().tanks) full_kg += tank.mass_kg;
    for (auto& tank : ship.stages.front().tanks) tank.mass_kg *= aircatch_propellant_kg / full_kg;
    auto p = gnc::EntryGuidanceParameters::aerocapture(m.aerocapture_apoapsis_m, -1.0);
    p.initial_bank_rad = aircatch_initial_bank_rad;
    const auto n = math::normalized(cross(aircatch_position_gcrf_m, aircatch_velocity_gcrf_mps));
    const auto shallow = math::Quat::from_axis_angle(n, -0.3 * constants::deg_to_rad).rotate(aircatch_velocity_gcrf_mps);
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(aircatch_epoch_tdb_s);
    t.session.place_entering_ship(ship, {aircatch_position_gcrf_m, shallow, aircatch_attitude, {}}, epoch, p);
    const auto& body = t.session.body(0);
    double min_height_m = 1e9;
    const auto exited = [&] {
        const auto* es = t.session.entry_status(0);
        return es && es->phase == gnc::EntryPhase::Exit;
    };
    while (!exited() && !body.impacted() && t.session.epoch() - epoch < 3600.0) {
        t.session.advance_to(t.session.epoch() + 1.0, 400);
        min_height_m = std::min(min_height_m, t.session.telemetry(0).altitude_m);
    }
    std::printf("[aircatch shallow] %s after %.0f s, lowest %.1f km, height %.1f km\n",
                exited() ? "exited" : body.impacted() ? "impacted" : "open", t.session.epoch() - epoch,
                min_height_m / 1000.0, t.session.telemetry(0).altitude_m / 1000.0);
    CHECK(min_height_m > 80e3);  // the case this test is about: never below the dip height
    CHECK(min_height_m < 120e3);
    CHECK(exited());
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

// Diagnostics (l5 Task 10): the flip and catch of a full flight's gate, replayed in seconds. STARSIM_GATE_STATE = the
// 16 numbers of a "[gate] state" line (STARSIM_DEBUG_FLIP: TDB, GCRF position and velocity, attitude w x y z, body
// rates, fuel and oxidiser kg); STARSIM_RETURN_AERO / _DENSITY / _THRUST the flight's truth factors.
TEST_CASE("Lunar return catch from a printed gate state", "[.lunar_gate_replay]") {
    const char* text = std::getenv("STARSIM_GATE_STATE");
    REQUIRE(text != nullptr);
    double g[16] = {};
    {
        const char* p = text;
        for (double& value : g) {
            char* end = nullptr;
            value = std::strtod(p, &end);
            REQUIRE(end != p);
            p = end;
        }
    }
    const auto factor = [](const char* name) {
        const char* v = std::getenv(name);
        return v ? std::atof(v) : 1.0;
    };
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    double fuel_kg = 0.0, oxidiser_kg = 0.0;
    for (const auto& tank : ship.stages.front().tanks) {
        if (tank.contents == config::TankContents::Fuel) fuel_kg += tank.mass_kg;
        if (tank.contents == config::TankContents::Oxidizer) oxidiser_kg += tank.mass_kg;
    }
    for (auto& tank : ship.stages.front().tanks) {
        if (tank.contents == config::TankContents::Fuel) tank.mass_kg *= g[14] / fuel_kg;
        if (tank.contents == config::TankContents::Oxidizer) tank.mass_kg *= g[15] / oxidiser_kg;
    }
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(g[0]);
    vehicle::RigidBodyState state{{g[1], g[2], g[3]}, {g[4], g[5], g[6]}, math::Quat{g[7], g[8], g[9], g[10]}, {g[11], g[12], g[13]}};
    if (const char* raise = std::getenv("STARSIM_GATE_RAISE_M")) {  // diagnostics: the same gate this much higher
        state.position_gcrf_m = state.position_gcrf_m * (1.0 + std::atof(raise) / norm(state.position_gcrf_m));
    }
    {  // the gate in the pad's east-north-up frame
        const auto& site = t.session.launch_site();
        const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
        const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
        const math::Vec3 east{-so, co, 0.0}, north{-sl * co, -sl * so, cl}, up{cl * co, cl * so, sl};
        const auto tr = t.earth.fixed_to_gcrf(epoch);
        const math::Vec3 r = transpose(tr.rotation) * state.position_gcrf_m -
                             frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
        const math::Vec3 v = transpose(tr.rotation) *
                             (state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), state.position_gcrf_m));
        const math::Vec3 nose = transpose(tr.rotation) * state.attitude.rotate({1.0, 0.0, 0.0});
        const auto enu = [&](const math::Vec3& x) { return math::Vec3{dot(x, east), dot(x, north), dot(x, up)}; };
        const auto re = enu(r), ve = enu(v), ne = enu(nose);
        std::printf("[gate replay] gate ENU %.1f %.1f %.1f m, velocity %.1f %.1f %.1f m/s, nose %.3f %.3f %.3f\n", re.x,
                    re.y, re.z, ve.x, ve.y, ve.z, ne.x, ne.y, ne.z);
    }
    t.session.place_landing_ship(ship, state, epoch, gnc::ship_landing_parameters());
    t.session.set_aero_dispersion(0, factor("STARSIM_RETURN_AERO"));
    t.session.set_thrust_dispersion(0, factor("STARSIM_RETURN_THRUST"));
    t.session.set_air_density_factor(factor("STARSIM_RETURN_DENSITY"));
    const auto run = test::fly_landing(t.session, t.earth, epoch, 120.0);
    const auto* guidance = t.session.ship_landing(0);
    std::printf("[gate replay] %s, offset %.2f m, lateral %.2f m/s, sink %.2f m/s, %d plans (%d failed), %.1f t left\n",
                sim::to_string(run.tower_phase), run.offset_m, run.lateral_mps, run.sink_mps,
                guidance && guidance->scvx() ? guidance->scvx()->statistics().plans : 0,
                guidance && guidance->scvx() ? guidance->scvx()->statistics().failures : 0,
                t.session.body(0).vehicle().propellant_kg() / 1000.0);
}
