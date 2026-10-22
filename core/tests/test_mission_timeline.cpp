#include <algorithm>

#include <catch2/catch_test_macros.hpp>

#include "starsim/sim/flight_session.hpp"
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
