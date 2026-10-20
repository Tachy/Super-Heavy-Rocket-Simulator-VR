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
