#include <algorithm>
#include <cmath>
#include <optional>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_test_macros.hpp>

#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
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
}
