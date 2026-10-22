// Crew escape (docs/superpowers/specs/2026-10-22-crew-escape-design.md), Task 6: the crewed ship return missions - from
// orbit and the lunar direct return - and a crew lost with a ship that breaks up above the seat envelope.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdio>
#include <string>

#include "lunar_return_test_support.hpp"
#include "return_scenario.hpp"
#include "starsim/config/crew_escape_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/crew_escape.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/sim/lunar_return_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/refuelling_mission.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

const spice::SpiceEphemeris& spice_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

void fail_group(sim::FlightSession& session, size_t body, const std::string& group) {
    const auto& engines = session.body(body).vehicle().stages().front().engines;
    for (size_t i = 0; i < engines.size(); ++i) {
        if (engines[i].group_id == group) session.fail_engine(body, 0, i);
    }
}

// Advances in 1-day calls of at most `ticks` ticks until the controlled ship has its entry guidance (or `max_s`).
bool fly_to_entry(sim::FlightSession& session, double max_s, int ticks = 200) {
    const auto start = session.epoch();
    while (session.epoch() - start < max_s) {
        if (session.entry_guidance(session.controlled_index())) return true;
        session.advance_to(session.epoch() + 86400.0, ticks);
    }
    return session.entry_guidance(session.controlled_index()) != nullptr;
}

// Flies on until the ship and every fired crew member are down (or `max_s`).
void fly_out(sim::FlightSession& session, double max_s) {
    const auto start = session.epoch();
    const auto down = [&] {
        const auto& ship = session.body(session.controlled_index());
        if (!(ship.impacted() || ship.resting() || ship.caught())) return false;
        for (const auto& m : session.crew_escape_state()->members) {
            if (m.body_index && !m.touchdown_enu_m) return false;
        }
        return true;
    };
    while (session.epoch() - start < max_s && !down()) session.advance_to(session.epoch() + 1.0, 100000);
}

void print(const sim::FlightSession& session, const char* name) {
    const auto& s = *session.crew_escape_state();
    std::printf("%s: armed %d fired %d (%s)\n", name, s.armed, s.fired, s.give_up_reason.c_str());
    for (const auto& m : s.members) {
        std::printf("  %s %s, peak %.1f g, clearance %.1f m, touchdown %s %.1f m/s: %s %s\n", gnc::to_string(m.mode),
                    gnc::to_string(m.phase), m.peak_load_g, m.min_clearance_m, m.touchdown_over_water ? "water" : "land",
                    m.touchdown_sink_mps, sim::to_string(m.outcome), m.reason.c_str());
    }
}

// Places the ship return mission's ship on its start orbit (as test_ship_return_mission.cpp).
time::Epoch place_orbit_return(sim::FlightSession& session, const config::ShipReturnMission& m) {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc(m.epoch_utc);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id);
    while (!cfg.stages.empty() && cfg.stages.front().id != m.stage_id) cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], m.propellant_kg);
    const double re = constants::wgs84_semi_major_axis_m;
    const double rp = re + m.periapsis_height_m, ra = re + m.apoapsis_height_m;
    const orbit::KeplerianElements elements{0.5 * (rp + ra), (ra - rp) / (ra + rp), m.inclination_rad, m.raan_rad, 0.0,
                                            m.argument_of_latitude_rad};
    const auto s = orbit::state_from_elements(elements, session.gm_earth_m3ps2());
    const Vec3 x = math::normalized(s.velocity_mps);
    const Vec3 z = math::normalized(s.position_m);  // belly (body -z) to the Earth
    const Vec3 y = cross(z, x);
    math::Mat3 r;
    const Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        r(0, c) = columns[c].x;
        r(1, c) = columns[c].y;
        r(2, c) = columns[c].z;
    }
    session.place_body(cfg, {s.position_m, s.velocity_mps, math::Quat::from_matrix(r), {}}, t0);
    return t0;
}

gnc::DeorbitParameters deorbit_parameters(const config::ShipReturnMission& m) {
    auto p = sim::ship_deorbit_parameters(m.deorbit);
    p.search_window_s = m.deorbit.search_window_s;
    return p;
}

}  // namespace

TEST_CASE("Crewed missions load their crew escape", "[crew_escape][mission]") {
    CHECK(config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase_crewed").crew_escape_id ==
          "starship_crew");
    CHECK(config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo_crewed").crew_escape_id ==
          "starship_crew");
    CHECK(config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase").crew_escape_id.empty());
    CHECK(config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo").crew_escape_id.empty());
}

TEST_CASE("Crewed orbit return with an engine failed in the entry", "[crew_escape][mission][slow]") {
    const auto m = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase_crewed");
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    place_orbit_return(session, m);
    session.set_crew_escape(config::load_crew_escape(STARSIM_CONFIG_DIR, m.crew_escape_id));
    REQUIRE(session.plan_ship_return(deorbit_parameters(m)));
    REQUIRE(fly_to_entry(session, 3.0 * 86400.0));
    fail_group(session, session.controlled_index(), "ship/sea_level_1");
    fly_out(session, 3600.0);
    print(session, "crewed orbit return, engine failed");
    const auto& s = *session.crew_escape_state();
    CHECK(s.armed);
    CHECK(s.early);
    CHECK(s.fired);
    for (const auto& m2 : s.members) {
        CHECK(m2.outcome == sim::CrewOutcome::Rescued);
        CHECK(m2.touchdown_over_water);
    }
    CHECK(s.ship_impact_over_water);
    CHECK(session.ship_return_result(session.controlled_index()) == sim::ReturnResult::Lost);
}

TEST_CASE("Crewed lunar direct return with an engine failed in the entry", "[crew_escape][mission][slow]") {
    test::LunarReturnSession t;
    const auto start = sim::start_mission(t.session, STARSIM_CONFIG_DIR, "ship_lunar_return_llo_crewed", {}, t.time);
    INFO(start.refusal);
    REQUIRE(start.return_planned);
    REQUIRE(t.session.crew_escape_state() != nullptr);
    REQUIRE(fly_to_entry(t.session, 8.0 * 86400.0));
    fail_group(t.session, t.session.controlled_index(), "ship/sea_level_1");
    fly_out(t.session, 3600.0);
    print(t.session, "crewed lunar return, engine failed");
    const auto& s = *t.session.crew_escape_state();
    CHECK(s.armed);
    CHECK(s.fired);
    for (const auto& m2 : s.members) CHECK(m2.outcome == sim::CrewOutcome::Rescued);
    CHECK(t.session.ship_return_result(t.session.controlled_index()) == sim::ReturnResult::Lost);
}

TEST_CASE("Crew outside the envelope is lost", "[crew_escape][mission]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], 35000.0);
    cfg.stages[0].heat_shield.max_heat_rate_w_m2 = 1.0e4;  // breaks up in the hypersonic entry, far above the envelope
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    const Vec3 west{-std::cos(26.0 * deg), 0.0, 0.0};
    auto state = test::state_over_site(session.launch_site(), earth(), t0, {-1500e3, 0.0, 80e3},
                                       {6500.0, 0.0, -100.0});
    (void)west;
    const auto t = earth().fixed_to_gcrf(t0);
    const Vec3 v_rel = state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), 55.0 * deg, 0.0,
                                                    cfg.stages[0].aero.belly_azimuth_rad);
    session.place_entering_ship(cfg, state, t0);
    session.set_crew_escape(config::load_crew_escape(STARSIM_CONFIG_DIR, "starship_crew"));
    for (int i = 0; i < 600 && !session.body(0).impacted(); ++i) session.advance_to(session.epoch() + 1.0, 100000);
    REQUIRE(session.body(0).impacted());
    INFO(session.body(0).breakup_reason());
    const auto& s = *session.crew_escape_state();
    CHECK_FALSE(s.fired);
    for (const auto& m : s.members) {
        CHECK(m.outcome == sim::CrewOutcome::Lost);
        CHECK_THAT(m.reason, Catch::Matchers::ContainsSubstring("außerhalb der Schleudersitz-Grenzen"));
    }
}
