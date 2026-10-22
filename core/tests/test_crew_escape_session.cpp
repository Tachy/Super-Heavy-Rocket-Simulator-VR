// Crew escape (docs/superpowers/specs/2026-10-22-crew-escape-design.md), Task 5: the session gives a crewed ship up,
// fires the seats, flies every crew member to the touchdown and the empty ship to its impact.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdio>
#include <optional>
#include <set>
#include <string>

#include "return_scenario.hpp"
#include "starsim/config/crew_escape_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/crew_escape.hpp"
#include "starsim/sim/landing_outcome.hpp"
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

const config::CrewEscapeConfig& crew() {
    static const auto c = config::load_crew_escape(STARSIM_CONFIG_DIR, "starship_crew");
    return c;
}

config::VehicleConfig ship_with(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    auto& ship = cfg.stages[0];
    config::fill_tanks(ship, propellant_kg);
    for (auto& f : ship.flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    return cfg;
}

time::Epoch t0() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
}

const Vec3 to_tower{std::sin(315.0 * deg), std::cos(315.0 * deg), 0.0};

// A belly-first ship `offset_enu_m` from the pad (height in z) falling with `velocity_enu_mps`.
vehicle::RigidBodyState belly_state(const sim::FlightSession& session, const config::VehicleConfig& cfg,
                                    const Vec3& offset_enu_m, const Vec3& velocity_enu_mps, double alpha_deg = 80.0) {
    auto state = test::state_over_site(session.launch_site(), earth(), t0(), offset_enu_m, velocity_enu_mps);
    const auto t = earth().fixed_to_gcrf(t0());
    const Vec3 v_rel = state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), alpha_deg * deg, 0.0,
                                                    cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    return state;
}

void set_starbase(sim::FlightSession& session) {
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
}

void fail_group(sim::FlightSession& session, const std::string& group) {
    const auto& engines = session.body(0).vehicle().stages().front().engines;
    for (size_t i = 0; i < engines.size(); ++i) {
        if (engines[i].group_id == group) session.fail_engine(0, 0, i);
    }
}

bool all_down(const sim::FlightSession& session) {
    const auto* s = session.crew_escape_state();
    if (!s || s->members.empty()) return false;
    for (const auto& m : s->members) {
        if (m.outcome == sim::CrewOutcome::Flying) return false;
        if (m.body_index && !m.touchdown_enu_m) return false;  // lost in the air: still falling
    }
    return session.body(0).impacted() || session.body(0).resting();
}

// Every fired member's body is its own, of the right kind (a seat before the separation, a parachutist after).
bool members_mapped(const sim::FlightSession& session) {
    const auto* s = session.crew_escape_state();
    std::set<size_t> used{session.controlled_index()};
    for (const auto& m : s->members) {
        if (!m.body_index) continue;
        if (*m.body_index >= session.body_count() || !used.insert(*m.body_index).second) return false;
        const auto& id = session.body(*m.body_index).vehicle().id();
        const bool separated = m.phase == gnc::SeatPhase::Main || m.phase == gnc::SeatPhase::Down ||
                               m.phase == gnc::SeatPhase::Separated;
        if (id != (separated ? crew().parachutist_vehicle_id : crew().seat_vehicle_id)) return false;
    }
    return true;
}

// Advances in `step_s` until every member and the ship are down (or `max_s`); checks the member mapping each step.
void fly_out(sim::FlightSession& session, double max_s, double step_s = 0.5, bool* mapped = nullptr) {
    const auto start = session.epoch();
    while (session.epoch() - start < max_s && !all_down(session)) {
        session.advance_to(session.epoch() + step_s, 1000000);
        if (mapped && session.crew_escape_state() && !members_mapped(session)) *mapped = false;
    }
}

void print(const sim::FlightSession& session, const char* name) {
    const auto& s = *session.crew_escape_state();
    std::printf("%s: armed %d fired %d (%s) early %d, give-up %.1f s\n", name, s.armed, s.fired, s.give_up_reason.c_str(),
                s.early, s.give_up_time_s);
    for (size_t i = 0; i < s.members.size(); ++i) {
        const auto& m = s.members[i];
        std::printf("  member %zu: %s, %s, fired %.1f s, peak %.1f g, clearance %.1f m, touchdown E %.0f N %.0f m %s at "
                    "%.1f m/s: %s %s\n",
                    i, gnc::to_string(m.mode), gnc::to_string(m.phase), m.fire_time_s, m.peak_load_g, m.min_clearance_m,
                    m.touchdown_enu_m ? m.touchdown_enu_m->x : 0.0, m.touchdown_enu_m ? m.touchdown_enu_m->y : 0.0,
                    m.touchdown_over_water ? "water" : "land", m.touchdown_sink_mps, sim::to_string(m.outcome),
                    m.reason.c_str());
    }
    if (s.ship_impact_enu_m) {
        std::printf("  ship impact E %.0f N %.0f m at %.1f m/s, tower %.0f m, shore %.0f m, %s\n", s.ship_impact_enu_m->x,
                    s.ship_impact_enu_m->y, s.ship_impact_speed_mps, s.ship_impact_tower_distance_m,
                    s.ship_impact_shore_distance_m, s.ship_impact_over_water ? "water" : "land");
    }
}

}  // namespace

TEST_CASE("Crew escape geometry: hull and plume clearance", "[crew_escape][session]") {
    CHECK(sim::hull_clearance_m({0, 0, 0}, {0, 0, 50}, 4.5, {10, 0, 20}) == Catch::Approx(5.5));
    CHECK(sim::hull_clearance_m({0, 0, 0}, {0, 0, 50}, 4.5, {3.5, 0, 20}) == Catch::Approx(-1.0));
    CHECK(sim::hull_clearance_m({0, 0, 0}, {0, 0, 50}, 4.5, {0, 0, 60}) == Catch::Approx(5.5));
    CHECK(sim::plume_clearance_m({0, 0, 0}, {0, 0, -1}, 15.0 * deg, 100.0, {0, 0, -50}) == 0.0);
    CHECK(sim::plume_clearance_m({0, 0, 0}, {0, 0, -1}, 15.0 * deg, 100.0, {20, 0, -50}) > 0.0);
    CHECK(sim::plume_clearance_m({0, 0, 0}, {0, 0, -1}, 15.0 * deg, 100.0, {0, 0, 10}) == Catch::Approx(10.0));
    sim::LoadJudge j;
    CHECK_FALSE(j.exceeded(25.0, 0.05, 20.0, 0.1));
    CHECK_FALSE(j.exceeded(25.0, 0.05, 20.0, 0.1));
    CHECK(j.exceeded(25.0, 0.05, 20.0, 0.1));
}

TEST_CASE("Uncrewed ship ignores the crew escape", "[crew_escape][session]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_with(35000.0);
    session.place_landing_ship(cfg, belly_state(session, cfg, to_tower * -220.0 + Vec3{0, 0, 950}, to_tower * 3.0 + Vec3{0, 0, -100}),
                               t0());
    session.advance_to(t0() + 5.0, 100000);
    CHECK(session.crew_escape_state() == nullptr);
}

TEST_CASE("Crewed ship with a failed engine before the gate gives up and the crew is rescued",
          "[crew_escape][session][slow]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_with(35000.0);
    session.place_entering_ship(cfg, belly_state(session, cfg, {-2000.0, 0.0, 12000.0}, {0.0, 0.0, -120.0}), t0());
    session.set_crew_escape(crew());
    fail_group(session, "ship/sea_level_2");
    bool mapped = true;
    fly_out(session, 900.0, 0.5, &mapped);
    print(session, "engine failed in the entry");
    const auto& s = *session.crew_escape_state();
    CHECK(s.armed);
    CHECK(s.early);
    CHECK_THAT(s.give_up_reason, Catch::Matchers::ContainsSubstring("Triebwerk"));
    CHECK(s.fired);
    CHECK(mapped);
    REQUIRE(s.members.size() == 4);
    for (const auto& m : s.members) {
        CHECK(m.outcome == sim::CrewOutcome::Rescued);
        CHECK(m.touchdown_over_water);
        CHECK(m.touchdown_sink_mps <= 8.0);
        CHECK(m.peak_load_g <= 20.0);
        CHECK(m.min_clearance_m >= 1.0);
    }
    CHECK(session.ship_return_result(0) == sim::ReturnResult::Lost);
    REQUIRE(s.ship_impact_enu_m);
    CHECK(s.ship_impact_over_water);
}

TEST_CASE("A give-up below the minimum firing height fires at once", "[crew_escape][session]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_with(20000.0);  // below min_catch_propellant_kg
    session.place_entering_ship(cfg, belly_state(session, cfg, to_tower * -300.0 + Vec3{0, 0, 1200}, {0.0, 0.0, -95.0}),
                                t0());
    session.set_crew_escape(crew());
    session.advance_to(t0() + 0.01, 100);  // two ticks
    const auto& s = *session.crew_escape_state();
    CHECK(s.armed);
    CHECK_FALSE(s.early);
    CHECK(s.fired);
    CHECK(s.members.front().mode == gnc::SeatMode::Mode1);
    fly_out(session, 600.0);
    print(session, "give-up at 1.2 km");
    for (const auto& m : s.members) CHECK(m.outcome != sim::CrewOutcome::Flying);
}

TEST_CASE("A catch abort after the flip ejects the crew", "[crew_escape][session][slow]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_with(35000.0);
    session.place_landing_ship(cfg, belly_state(session, cfg, to_tower * -220.0 + Vec3{0, 0, 950}, to_tower * 3.0 + Vec3{0, 0, -100}),
                               t0());
    session.set_crew_escape(crew());
    session.advance_to(t0() + 1.0, 100000);
    REQUIRE(session.abort_landing(0));
    bool mapped = true;
    fly_out(session, 600.0, 0.5, &mapped);
    print(session, "catch abort in the flip");
    const auto& s = *session.crew_escape_state();
    CHECK(s.armed);
    CHECK(s.fired);
    CHECK(mapped);
    for (const auto& m : s.members) {
        CHECK(m.outcome != sim::CrewOutcome::Flying);
        CHECK(std::isfinite(m.peak_load_g));
        CHECK(std::isfinite(m.min_clearance_m));
        CHECK(m.touchdown_enu_m.has_value());
    }
    CHECK(session.ship_return_result(0) == sim::ReturnResult::Lost);
}

TEST_CASE("A crewed ship that is caught keeps its crew aboard", "[crew_escape][session][slow]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_with(35000.0);
    session.place_landing_ship(cfg, belly_state(session, cfg, to_tower * -220.0 + Vec3{0, 0, 950}, to_tower * 3.0 + Vec3{0, 0, -100}),
                               t0());
    session.set_crew_escape(crew());
    for (int i = 0; i < 240 && !session.body(0).caught(); ++i) session.advance_to(session.epoch() + 0.5, 100000);
    session.advance_to(session.epoch() + 10.0, 100000);  // the arms settle
    REQUIRE(session.body(0).caught());
    const auto& s = *session.crew_escape_state();
    CHECK_FALSE(s.armed);
    CHECK_FALSE(s.fired);
    for (const auto& m : s.members) CHECK(m.outcome == sim::CrewOutcome::Rescued);
}

// Final review I1: a catch that misses (no abort flag: the CG passes the catch point) must arm the seats too.
TEST_CASE("A missed catch ejects the crew", "[crew_escape][session][slow]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_with(27000.0);  // the calibration's first missed gate load
    session.place_landing_ship(cfg, belly_state(session, cfg, to_tower * -220.0 + Vec3{0, 0, 950}, to_tower * 3.0 + Vec3{0, 0, -100}),
                               t0());
    session.set_crew_escape(crew());
    fly_out(session, 600.0);  // runs dry hovering at the arms (~93 m) and drops onto them
    print(session, "missed catch");
    const auto& s = *session.crew_escape_state();
    CHECK(s.armed);
    CHECK(s.fired);
    for (const auto& m : s.members) {
        CHECK(m.outcome != sim::CrewOutcome::Flying);
        CHECK(m.reason.find("außerhalb") == std::string::npos);
    }
}

// Final review I2: an aerocapture pass is no catch attempt - a failed flip engine is judged on the final entry.
TEST_CASE("An aerocapture pass does not give the ship up", "[crew_escape][session]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_with(35000.0);
    gnc::EntryGuidanceParameters p = gnc::EntryGuidanceParameters::lunar_return();
    p.aerocapture_apoapsis_m = 400e3;
    session.place_entering_ship(cfg, belly_state(session, cfg, {-2000e3, 0.0, 100e3}, {10500.0, 0.0, -300.0}, 55.0),
                                t0(), p);
    session.set_crew_escape(crew());
    fail_group(session, "ship/sea_level_2");
    session.advance_to(t0() + 30.0, 100000);
    CHECK_FALSE(session.crew_escape_state()->armed);
    CHECK_FALSE(session.entry_guidance(0)->given_up());
}

TEST_CASE("Tumbling ship: seats fire along the hull", "[crew_escape][session]") {
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_with(20000.0);
    auto state = belly_state(session, cfg, to_tower * -300.0 + Vec3{0, 0, 1200}, {0.0, 0.0, -95.0}, 30.0);
    state.angular_velocity_body_radps += Vec3{0.0, 0.5, 0.0};
    session.place_entering_ship(cfg, state, t0());
    session.set_crew_escape(crew());
    fly_out(session, 600.0);
    print(session, "tumbling");
    const auto& s = *session.crew_escape_state();
    CHECK(s.fired);
    for (const auto& m : s.members) {
        CHECK(m.outcome != sim::CrewOutcome::Flying);
        CHECK(std::isfinite(m.peak_load_g));
        CHECK(std::isfinite(m.min_clearance_m));
    }
}

TEST_CASE("One long advance gives the same touchdown", "[crew_escape][session]") {
    std::optional<Vec3> touchdown[2];
    for (int run = 0; run < 2; ++run) {
        sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
        set_starbase(session);
        const auto cfg = ship_with(20000.0);
        session.place_entering_ship(cfg, belly_state(session, cfg, to_tower * -300.0 + Vec3{0, 0, 1200}, {0.0, 0.0, -95.0}),
                                    t0());
        session.set_crew_escape(crew());
        if (run == 0) {
            fly_out(session, 600.0, 0.1);
        } else {
            session.advance_to(t0() + 600.0, 100000000);
        }
        touchdown[run] = session.crew_escape_state()->members.back().touchdown_enu_m;
    }
    REQUIRE(touchdown[0]);
    REQUIRE(touchdown[1]);
    CHECK(norm(*touchdown[0] - *touchdown[1]) < 1.0);
}
