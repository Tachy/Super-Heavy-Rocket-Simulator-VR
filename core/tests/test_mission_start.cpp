// Mission kinds and the core mission start (d5 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <stdexcept>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

const spice::SpiceEphemeris& spice_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& spice_earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

}  // namespace

TEST_CASE("Missions name their kind; ascent is the default", "[mission][config]") {
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "falcon9_crew_dragon") == config::MissionKind::Ascent);
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "starship_leo") == config::MissionKind::Ascent);
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "crew_dragon_return") == config::MissionKind::CapsuleReturn);
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "ship_return_starbase") == config::MissionKind::ShipReturn);
    CHECK_THROWS_AS(config::mission_kind(STARSIM_CONFIG_DIR, "no_such_mission"), config::ConfigError);
}

TEST_CASE("A loader refuses a mission of another kind", "[mission][config]") {
    CHECK_THROWS_AS(config::load_ascent_mission(STARSIM_CONFIG_DIR, "crew_dragon_return"), config::ConfigError);
    CHECK_THROWS_AS(config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "falcon9_crew_dragon"), config::ConfigError);
    CHECK_THROWS_AS(config::load_ship_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return"), config::ConfigError);
}

TEST_CASE("An ascent mission starts at the requested epoch with its site", "[mission][sim]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    const auto t = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_crew_dragon", t, time);
    CHECK(s.kind == config::MissionKind::Ascent);
    CHECK(s.site_id == "ccsfs_slc40");
    CHECK(s.epoch == t);
    CHECK(session.epoch() == t);
    CHECK(session.autopilot().state == sim::AutopilotState::PreLaunch);
}

TEST_CASE("liftoff_utc is loaded", "[mission][config]") {
    const auto crs35 = config::load_ascent_mission(STARSIM_CONFIG_DIR, "crs35_ascent");
    REQUIRE(crs35.liftoff_utc);
    CHECK(*crs35.liftoff_utc == time::UtcDateTime{2026, 10, 13, 10, 33, 0.0});  // flights.toml CRS-35
    const auto flight14 = config::load_ascent_mission(STARSIM_CONFIG_DIR, "flight14_ascent");
    REQUIRE(flight14.liftoff_utc);
    CHECK(*flight14.liftoff_utc == time::UtcDateTime{2026, 9, 28, 12, 48, 59.0});  // flights.toml flight 14
    CHECK_FALSE(config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo").liftoff_utc.has_value());
}

TEST_CASE("An ascent mission with liftoff_utc lifts off at it, whatever the requested epoch", "[mission][sim]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "flight14_ascent");
    REQUIRE(m.liftoff_utc);
    const auto liftoff = time.from_utc(*m.liftoff_utc);
    const auto requested = liftoff + 3.0 * 86400.0;
    sim::StartOptions waiting;
    waiting.wait_on_pad = true;
    waiting.countdown_s = 600.0;  // longer than the ignition lead
    const double countdown_s = m.countdown_s.value_or(waiting.countdown_s);
    {
        sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
        const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, "flight14_ascent", requested, time, waiting);
        REQUIRE(s.refusal.empty());
        REQUIRE(session.timeline().t0());
        CHECK(std::abs(*session.timeline().t0() - liftoff) < 1e-6);
        CHECK(std::abs((liftoff - s.epoch) - countdown_s) < 1e-6);
        CHECK(session.epoch() == s.epoch);
    }
    {
        sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
        const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, "flight14_ascent", requested, time);
        REQUIRE(s.refusal.empty());
        CHECK(std::abs(s.epoch - liftoff) < 1e-6);
        CHECK(std::abs(session.epoch() - liftoff) < 1e-6);
    }
}

// r2 review: a lunar return mission turns boil-off off; the next mission in the same session must have it again.
TEST_CASE("Every mission start sets the boil-off flag of its mission", "[mission][sim][boil_off]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    session.set_boil_off(false);  // as a lunar return mission leaves it
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", time::Epoch::from_tdb_seconds_past_j2000(8.4e8),
                       time);
    CHECK(session.environment().boil_off);
}

TEST_CASE("A capsule return mission starts at its own epoch and plans the deorbit", "[mission][sim][dragon]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    const auto requested = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);  // ignored: the RAAN belongs to the epoch
    const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, "crew_dragon_return", requested, time);
    const auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    CHECK(s.kind == config::MissionKind::CapsuleReturn);
    CHECK(s.site_id.empty());
    CHECK(s.epoch == time.from_utc(m.epoch_utc));
    CHECK(session.epoch() == s.epoch);
    CHECK(s.return_planned);
    CHECK(session.capsule_return_step() == sim::CapsuleReturnStep::DeorbitBurn);
    CHECK(session.splashdown_zone().has_value());
}

TEST_CASE("A ship return mission is not started, it is flown from orbit", "[mission][sim]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    CHECK_THROWS_AS(sim::start_mission(session, STARSIM_CONFIG_DIR, "ship_return_starbase",
                                       time::Epoch::from_tdb_seconds_past_j2000(8.4e8), time),
                    std::invalid_argument);
}
