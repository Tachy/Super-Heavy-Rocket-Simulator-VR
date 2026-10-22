#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>

#include <catch2/catch_test_macros.hpp>

#include "lunar_return_test_support.hpp"
#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/sim/lunar_return_mission.hpp"
#include "starsim/sim/mission_event_observer.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/mission_timeline.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("Body serials are unique and survive reindexing", "[timeline]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);

    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const double deg = constants::deg_to_rad;
    session.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, t0);
    REQUIRE(session.body_count() == 1);
    const uint32_t before = session.body_serial(0);
    CHECK(before >= 1);

    session.ignite();
    session.release_clamps();
    session.advance_to(t0 + 10.0);
    session.stage();
    REQUIRE(session.body_count() == 2);

    const uint32_t a = session.body_serial(0);
    const uint32_t b = session.body_serial(1);
    CHECK(a >= 1);
    CHECK(b >= 1);
    CHECK(a != b);
    CHECK((a == before || b == before));
}

namespace {
using sim::MissionEvent;
using sim::MissionEventKind;
const time::Epoch kBase = time::Epoch::from_tdb_seconds_past_j2000(1.0e8);
MissionEvent event(MissionEventKind kind, double offset_s, bool predicted = false) {
    return MissionEvent{kind, kBase + offset_s, 0, "", predicted};
}
}  // namespace

TEST_CASE("T-0 of a pad mission is the scheduled liftoff, then the real one", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(true);
    timeline.set_scheduled_liftoff(kBase + 100.0);
    REQUIRE(timeline.t0().has_value());
    CHECK(*timeline.t0() == kBase + 100.0);
    CHECK(*timeline.mission_clock_s(kBase) == -100.0);

    timeline.record(event(MissionEventKind::Liftoff, 101.5));
    CHECK(*timeline.t0() == kBase + 101.5);
}

TEST_CASE("An engine start before liftoff is no T-0", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(true);
    timeline.record(event(MissionEventKind::EngineStart, 0.0));
    CHECK_FALSE(timeline.t0().has_value());
}

TEST_CASE("T-0 of an orbit mission is its first action", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(false);
    timeline.set_predicted({event(MissionEventKind::Ignition, 3600.0, true)});
    REQUIRE(timeline.t0().has_value());
    CHECK(*timeline.t0() == kBase + 3600.0);

    timeline.record(event(MissionEventKind::Undocking, 10.0));
    CHECK(*timeline.t0() == kBase + 10.0);
}

TEST_CASE("No action, no T-0", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(false);
    CHECK_FALSE(timeline.t0().has_value());
    CHECK_FALSE(timeline.mission_clock_s(kBase).has_value());
}

TEST_CASE("Recorded events stay sorted; predicted are replaced", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(true);
    timeline.record(event(MissionEventKind::Meco, 160.0));
    timeline.record(event(MissionEventKind::Liftoff, 0.0, true));  // predicted flag is forced off
    timeline.record(event(MissionEventKind::MaxQ, 70.0));
    REQUIRE(timeline.recorded().size() == 3);
    CHECK(timeline.recorded()[0].kind == MissionEventKind::Liftoff);
    CHECK(timeline.recorded()[1].kind == MissionEventKind::MaxQ);
    CHECK(timeline.recorded()[2].kind == MissionEventKind::Meco);
    CHECK_FALSE(timeline.recorded()[0].predicted);

    timeline.set_predicted({event(MissionEventKind::Seco, 500.0), event(MissionEventKind::Landed, 900.0)});
    timeline.set_predicted({event(MissionEventKind::Splashdown, 1200.0)});
    REQUIRE(timeline.predicted().size() == 1);
    CHECK(timeline.predicted()[0].kind == MissionEventKind::Splashdown);
    CHECK(timeline.predicted()[0].predicted);
    CHECK(std::string(sim::to_string(MissionEventKind::Meco)) == "MECO");
}

namespace {
const spice::SpiceEphemeris& timeline_ephemeris() {
    static const spice::SpiceEphemeris e;
    return e;
}
const spice::SpiceOrientation& timeline_earth() {
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

// The kinds of `events` (optionally of one body) contain `expected` in this order (other events between them allowed).
bool contains_in_order(const std::vector<MissionEvent>& events, const std::vector<MissionEventKind>& expected,
                       uint32_t serial = 0) {
    size_t k = 0;
    for (const auto& e : events) {
        if (k < expected.size() && e.kind == expected[k] && (serial == 0 || e.body_serial == serial)) ++k;
    }
    return k == expected.size();
}

const MissionEvent* find_event(const sim::MissionTimeline& t, MissionEventKind kind) {
    for (const auto& e : t.recorded()) {
        if (e.kind == kind) return &e;
    }
    return nullptr;
}

void print_events(const sim::MissionTimeline& t) {
    if (!std::getenv("STARSIM_VERBOSE")) return;
    for (const auto& e : t.recorded()) {
        std::printf("  [timeline] %.2f %-16s body %u %s\n", e.epoch.tdb_seconds_past_j2000(), sim::to_string(e.kind),
                    e.body_serial, e.body_name.c_str());
    }
}
}  // namespace

TEST_CASE("A Starship ascent records liftoff, Max Q, MECO, separation and SECO", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);

    double q_peak_pa = 0.0;  // the stack's q while climbing, sampled by the test
    time::Epoch q_peak_epoch = t0;
    const auto& t = session.timeline();
    while (session.epoch() - t0 < 900.0 && !find_event(t, MissionEventKind::Seco)) {
        session.advance_to(session.epoch() + 0.1, 100);
        if (session.body_count() == 0) break;
        const auto tm = session.telemetry(0);
        if (tm.vertical_speed_mps > 0.0 && tm.dynamic_pressure_pa > q_peak_pa) {
            q_peak_pa = tm.dynamic_pressure_pa;
            q_peak_epoch = session.epoch();
        }
    }
    print_events(t);
    using K = MissionEventKind;
    CHECK(contains_in_order(t.recorded(), {K::EngineStart, K::Liftoff, K::MaxQ, K::Meco, K::StageSeparation, K::Seco}));
    const auto* liftoff = find_event(t, K::Liftoff);
    const auto* max_q = find_event(t, K::MaxQ);
    const auto* meco = find_event(t, K::Meco);
    const auto* seco = find_event(t, K::Seco);
    REQUIRE(liftoff);
    REQUIRE(max_q);
    REQUIRE(meco);
    REQUIRE(seco);
    CHECK(max_q->epoch - liftoff->epoch > 50.0);
    CHECK(max_q->epoch - liftoff->epoch < 80.0);
    CHECK(q_peak_pa >= 10e3);
    CHECK(std::abs(max_q->epoch - q_peak_epoch) <= 0.2);  // the observer's 0.1 s sampling
    CHECK(meco->body_serial != seco->body_serial);
    // the part that acts: the stack at the pad, the booster at MECO and separation, the ship at SECO
    CHECK(liftoff->body_name == "Heavy Launcher Mk3");
    CHECK(meco->body_name == "Super Heavy Block 3");
    CHECK(find_event(t, K::StageSeparation)->body_name == "Super Heavy Block 3");
    CHECK(seco->body_name == "Starship Block 3");
    CHECK(find_event(t, K::EntryInterface) == nullptr);  // the 200 km orbit: no entry
    CHECK(*t.t0() == liftoff->epoch);
}

TEST_CASE("Booster return records boostback, landing burn and the catch", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);  // booster return on, tower catch

    const auto& t = session.timeline();
    std::optional<uint32_t> booster;
    while (session.epoch() - t0 < 900.0 && !find_event(t, MissionEventKind::Caught)) {
        session.advance_to(session.epoch() + 1.0, 1000);
        if (!booster) {
            if (const auto* meco = find_event(t, MissionEventKind::Meco)) booster = meco->body_serial;
        }
        if (booster) {
            bool down = false;
            for (size_t i = 0; i < session.body_count(); ++i) {
                if (session.body_serial(i) == *booster) down = session.landing_outcome(i) != sim::LandingOutcome::Flying;
            }
            if (down) {
                session.advance_to(session.epoch() + 1.0, 1000);  // one more observation
                break;
            }
        }
    }
    print_events(t);
    REQUIRE(booster);
    using K = MissionEventKind;
    CHECK(contains_in_order(t.recorded(), {K::Boostback, K::LandingBurn, K::Caught}, *booster));
}

TEST_CASE("An undocking mission starts its clock at the undocking", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto st = sim::start_mission(session, STARSIM_CONFIG_DIR, "crew_dragon_iss_departure", {}, time);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    const auto& t = session.timeline();
    CHECK(t.recorded().empty());  // a scenario that starts docked: no Docking event
    const auto start = session.epoch();
    while (session.epoch() - start < 3600.0 && !find_event(t, MissionEventKind::Undocking)) {
        session.advance_to(session.epoch() + 10.0, 10000000);
    }
    print_events(t);
    const auto* undocking = find_event(t, MissionEventKind::Undocking);
    REQUIRE(undocking);
    REQUIRE(t.t0().has_value());
    CHECK(*t.t0() == undocking->epoch);
}

TEST_CASE("start_mission resets the timeline; campaign launches do not", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);
    session.advance_to(t0 + 15.0, 100000);
    const auto& t = session.timeline();
    REQUIRE(find_event(t, MissionEventKind::Liftoff));
    const auto first_liftoff = find_event(t, MissionEventKind::Liftoff)->epoch;

    const auto t1 = t0 + 3600.0;
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t1, time);
    CHECK(t.recorded().empty());
    session.advance_to(t1 + 15.0, 100000);
    REQUIRE(find_event(t, MissionEventKind::Liftoff));
    CHECK(find_event(t, MissionEventKind::Liftoff)->epoch != first_liftoff);
    const size_t events = t.recorded().size();

    // a campaign driver places its next vehicle without a mission start: the log goes on
    const double deg = constants::deg_to_rad;
    session.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, session.epoch());
    CHECK(t.recorded().size() == events);
    session.ignite();
    session.advance_to(session.epoch() + 5.0, 100000);
    session.release_clamps();
    session.advance_to(session.epoch() + 1.0, 100000);
    CHECK(std::count_if(t.recorded().begin(), t.recorded().end(),
                        [](const MissionEvent& e) { return e.kind == MissionEventKind::Liftoff; }) == 2);
}

namespace {
constexpr double deg = constants::deg_to_rad;

// The ship alone; `propellant_kg` < 0: its landing propellant (as the entry tests).
// The ship alone; `propellant_kg` < 0: its landing propellant (as the entry tests).
// The ship alone; `propellant_kg` < 0: its landing propellant (as the entry tests).
config::VehicleConfig ship_alone(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    auto& ship = cfg.stages[0];
    config::fill_tanks(ship, propellant_kg < 0.0 ? ship.landing_propellant_kg : propellant_kg);
    for (auto& f : ship.flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    return cfg;
}

// A belly-first ship `offset_enu_m` from the Starbase pad (height in z) with `velocity_enu_mps`.
vehicle::RigidBodyState belly_state(const sim::FlightSession& session, const config::VehicleConfig& cfg, time::Epoch t0,
                                    const math::Vec3& offset_enu_m, const math::Vec3& velocity_enu_mps, double alpha_deg) {
    auto state = test::state_over_site(session.launch_site(), timeline_earth(), t0, offset_enu_m, velocity_enu_mps);
    const auto t = timeline_earth().fixed_to_gcrf(t0);
    const math::Vec3 v_rel = state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), alpha_deg * deg, 0.0,
                                                    cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    return state;
}

void set_starbase(sim::FlightSession& session) {
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
}

size_t count_of(const sim::MissionTimeline& t, MissionEventKind kind) {
    return static_cast<size_t>(std::count_if(t.recorded().begin(), t.recorded().end(),
                                             [&](const MissionEvent& e) { return e.kind == kind; }));
}
}  // namespace

// The hand-over to the entry guidance of test_lunar_return_entry.cpp (mission ship_lunar_return_llo, printed
// 2026-10-17): a direct lunar-return entry at 9,695 m/s that lofts at entry q.
constexpr double handover_epoch_tdb_s = 848008959.11018324;
constexpr double handover_propellant_kg = 42958.659955708943;
const math::Vec3 handover_position_gcrf_m{7086860.4675348997, 2531187.6710331743, 2929169.4508524211};
const math::Vec3 handover_velocity_gcrf_mps{-6563.1519895914944, 5826.099272684959, 1092.0146363209715};
const math::Quat handover_attitude{-0.55696594189169446, -0.50181461927849202, 0.23702032888641714,
                                   0.6178935111697279};
constexpr double handover_initial_bank_rad = 113.25 * deg;

TEST_CASE("A lunar-return entry's loft records no Max Q", "[timeline]") {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    t.session.set_boil_off(m.boil_off);
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    config::fill_tanks(ship.stages.front(), handover_propellant_kg);
    auto p = gnc::EntryGuidanceParameters::lunar_return();
    p.initial_bank_rad = handover_initial_bank_rad;
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(handover_epoch_tdb_s);
    t.session.place_entering_ship(ship, {handover_position_gcrf_m, handover_velocity_gcrf_mps, handover_attitude, {}},
                                  epoch, p);
    // Fly until the ascent's Max Q rule would have fired on the pull-up: climbing at >= 10 kPa, then q 10 % below.
    double q_climbing_pa = 0.0;
    bool would_fire = false;
    double altitude_m = 1e9;
    while (!would_fire && !t.session.body(0).impacted() && t.session.epoch() - epoch < 3600.0 && altitude_m > 30e3) {
        t.session.advance_to(t.session.epoch() + 1.0, 400);
        const auto tm = t.session.telemetry(0);
        altitude_m = tm.altitude_m;
        if (tm.vertical_speed_mps > 0.0) q_climbing_pa = std::max(q_climbing_pa, tm.dynamic_pressure_pa);
        would_fire = q_climbing_pa >= sim::MissionEventObserver::max_q_min_pa &&
                     tm.dynamic_pressure_pa < sim::MissionEventObserver::max_q_drop_fraction * q_climbing_pa;
    }
    print_events(t.session.timeline());
    if (std::getenv("STARSIM_VERBOSE")) std::printf("  [loft] climbing q up to %.0f Pa\n", q_climbing_pa);
    CHECK(would_fire);
    CHECK(count_of(t.session.timeline(), MissionEventKind::MaxQ) == 0);
    CHECK(count_of(t.session.timeline(), MissionEventKind::EntryInterface) == 1);
}

TEST_CASE("A ship's flip and catch records one landing burn", "[timeline]") {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_alone(-1.0);  // its landing propellant
    // 28 km west of the pad, 26 km up, Mach ~2.6, 8 deg down (test_flight_session_entry: caught in the arms)
    const double speed = 780.0, gamma = -8.0 * deg;
    session.place_entering_ship(
        cfg, belly_state(session, cfg, t0, {-28000.0, 0.0, 26000.0}, {speed * std::cos(gamma), 0.0, speed * std::sin(gamma)},
                         65.0),
        t0);
    while (session.epoch() - t0 < 600.0 && session.landing_outcome(0) == sim::LandingOutcome::Flying) {
        session.advance_to(session.epoch() + 1.0, 1000);
    }
    session.advance_to(session.epoch() + 1.0, 1000);  // one more observation
    print_events(session.timeline());
    CHECK(count_of(session.timeline(), MissionEventKind::LandingBurn) == 1);
    CHECK(count_of(session.timeline(), MissionEventKind::Caught) == 1);
}

TEST_CASE("Falcon 9 cold staging records MECO without a second cutoff", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_rtls", t0, time);
    const auto& t = session.timeline();
    std::optional<time::Epoch> separation;
    while (session.epoch() - t0 < 600.0 && (!separation || session.epoch() - *separation < 20.0)) {
        session.advance_to(session.epoch() + 0.5, 1000);
        if (!separation) {
            if (const auto* e = find_event(t, MissionEventKind::StageSeparation)) separation = e->epoch;
        }
    }
    print_events(t);
    const auto* meco = find_event(t, MissionEventKind::Meco);
    REQUIRE(meco);
    REQUIRE(separation);
    CHECK(meco->body_name == "Falcon 9 Block 5 first stage");
    for (const auto& e : t.recorded()) {  // the tail-off after the MECO is no event of its own
        if (e.body_serial == meco->body_serial && e.kind == MissionEventKind::Cutoff) CHECK(e.epoch - meco->epoch > 5.0);
    }
}

TEST_CASE("Mission event probe", "[.][probe][timeline_probe]") {  // STARSIM_PROBE_MISSION, STARSIM_PROBE_S: prints the log
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const char* mission = std::getenv("STARSIM_PROBE_MISSION");
    const char* duration = std::getenv("STARSIM_PROBE_S");
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const auto st = sim::start_mission(session, STARSIM_CONFIG_DIR, mission ? mission : "starship_leo", t0, time);
    const auto start = session.epoch();
    const double limit_s = duration ? std::atof(duration) : 600.0;
    while (session.epoch() - start < limit_s && session.body_count() > 0) {
        session.advance_to(session.epoch() + 60.0, 10000000);
    }
    for (const auto& e : session.timeline().recorded()) {
        std::printf("  [probe] T%+10.1f %-16s body %u %s\n", e.epoch - start, sim::to_string(e.kind), e.body_serial,
                    e.body_name.c_str());
    }
    std::printf("  [probe] flown %.0f s, leg %s: %s; %zu bodies\n", session.epoch() - start,
                sim::to_string(session.iss_mission_leg()), session.iss_mission_status().c_str(), session.body_count());
    (void)st;
}
