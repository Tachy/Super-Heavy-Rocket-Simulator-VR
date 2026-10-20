// ISS i6 campaigns: the three missions through their start functions - orbit start to hard capture at both ports,
// the refusals, the pad mission end to end, the departure to splashdown.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include "rendezvous_test_support.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/sim/iss_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "test_support.hpp"

using namespace starsim;
using sim::IssMissionLeg;

namespace {
bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

struct Session {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session{ephemeris, earth, rendezvous_test::config_dir};
    // the committed fixture stands in for data/iss_oem (git-ignored)
    const orbit::OemArchive archive = orbit::OemArchive::scan(rendezvous_test::fixture_dir, clock);
};

// Flies until `done` or `limit_s` (the sequence stops the coast at its leg changes).
template <class Done>
void fly(sim::FlightSession& s, Done done, double limit_s) {
    const auto start = s.epoch();
    while (!done() && s.epoch() - start < limit_s && s.iss_mission_leg() != IssMissionLeg::Ended) {
        s.advance_to(s.epoch() + 600.0, 10000000);
    }
}
}  // namespace

TEST_CASE("ISS mission: crew_dragon_iss from the window's separation to hard capture at IDA-2",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    const auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    const auto wall = std::chrono::steady_clock::now();
    const auto st = sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    const double planning_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count();
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    REQUIRE(st.launch_window);
    CHECK(planning_s < 10.0);  // spec: a few seconds
    CHECK(st.epoch == st.launch_window->separation);
    const double kg0 = f.session.body(f.session.controlled_index()).vehicle().rcs_propellant_kg();
    fly(f.session, [&] { return f.session.iss_mission_leg() == IssMissionLeg::Docked; }, 30.0 * 3600.0);
    INFO(f.session.iss_mission_status());
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Docked);
    // the Dragon is held on the station now: its own tank still shows what the flight used
    const double used_kg = kg0 - f.session.body(f.session.controlled_index()).vehicle().rcs_propellant_kg();
    CHECK(f.session.epoch() - st.launch_window->liftoff < m.mission_limit_s + 3.0 * 3600.0);
    CHECK(used_kg < 2562.0 - 900.0);  // the deorbit + abort reserve stays (PhasingParameters::reserve_kg)
    if (verbose()) {
        std::printf("[iss_mission] crew_dragon_iss: planning %.1f s, docked %.1f h after liftoff, %.0f kg\n",
                    planning_s, (f.session.epoch() - st.launch_window->liftoff) / 3600.0, used_kg);
    }
}

TEST_CASE("ISS mission: crew_dragon_iss at IDA-3 with a set phase on the synthetic orbit",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    m.port = "ida3";
    m.iss_orbit = "synthetic";
    m.phase_rad = 40.0 * constants::deg_to_rad;
    const auto st = sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    CHECK_FALSE(st.launch_window);
    CHECK(st.epoch == f.clock.from_utc(m.epoch_utc));
    fly(f.session, [&] { return f.session.iss_mission_leg() == IssMissionLeg::Docked; }, 30.0 * 3600.0);
    INFO(f.session.iss_mission_status());
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Docked);
    if (verbose()) {
        std::printf("[iss_mission] crew_dragon_iss IDA-3, 40 deg, synthetic: docked %.1f h after the start\n",
                    (f.session.epoch() - st.epoch) / 3600.0);
    }
}

TEST_CASE("ISS mission: refusals - OEM date outside the archive, unknown port", "[iss][iss_mission]") {
    test::spice_session();
    Session f;
    auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    m.iss_orbit = "oem";
    m.epoch_utc = {2027, 3, 1, 0, 0, 0.0};  // beyond the fixture's window
    m.phase_rad = 30.0 * constants::deg_to_rad;
    CHECK_THROWS_AS(sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock),
                    std::runtime_error);
    CHECK(f.session.body_count() == 0);  // nothing placed (Review Focus 1)
    auto p = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    p.port = "ida9";
    CHECK_THROWS_AS(sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, p, f.archive, f.clock),
                    std::runtime_error);
    CHECK(f.session.body_count() == 0);
}

TEST_CASE("ISS mission: start_mission dispatches crew_dragon_iss", "[iss][iss_mission]") {
    test::spice_session();
    Session f;
    const auto st = sim::start_mission(f.session, rendezvous_test::config_dir, "crew_dragon_iss", {}, f.clock);
    CHECK(st.kind == config::MissionKind::IssRendezvous);
    // data/iss_oem may be absent on a fresh checkout: "auto" then flies the synthetic orbit - either way planned
    INFO(st.refusal);
    CHECK(st.return_planned);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    CHECK_FALSE(st.warnings.empty());  // at least the ISS source note
}
