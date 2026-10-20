// ISS i6: the mission sequence chains phasing -> proximity -> docking (and its endings) in the flight session.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "docking_test_support.hpp"
#include "rendezvous_test_support.hpp"
#include "starsim/sim/flight_session.hpp"

using namespace starsim;
using sim::IssMissionLeg;

namespace {
sim::IssMissionSequence docking_to(const std::string& port) {
    sim::IssMissionSequence q;
    q.station_port = port;
    return q;
}

// The i4 start: i3's measured Waypoint 0 arrival.
proximity_test::DragonNearIss at_waypoint0() {
    return proximity_test::DragonNearIss(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.012, 0.0, -0.012}));
}
}  // namespace

TEST_CASE("ISS mission sequence: Waypoint 0 hands over to the proximity flight, also through one long call",
          "[iss][iss_mission][proximity]") {
    test::spice_session();
    auto d = at_waypoint0();
    auto& s = d.session;
    s.mark_rendezvous_at_waypoint0_for_test();
    s.set_iss_mission(docking_to("ida2"));
    CHECK(s.iss_mission_leg() == IssMissionLeg::Rendezvous);
    CHECK_FALSE(s.proximity_hold());  // no proximity flight yet: refused, the sequence goes on (Review Focus 3)
    CHECK(s.iss_mission_leg() == IssMissionLeg::Rendezvous);
    s.advance_to(d.t0 + 3600.0, 2);  // two ticks: the first changes the leg, nothing coasts past it
    CHECK(s.iss_mission_leg() == IssMissionLeg::Proximity);
    CHECK(s.proximity_active());
    CHECK(s.epoch() - d.t0 < 1.0);
}

TEST_CASE("ISS mission sequence: proximity -> docking -> docked completes the mission",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    auto d = at_waypoint0();
    auto& s = d.session;
    s.mark_rendezvous_at_waypoint0_for_test();
    s.set_iss_mission(docking_to("ida2"));
    bool saw_docking = false;
    for (double t = 0.0; t < 4.0 * 3600.0 && s.iss_mission_leg() != IssMissionLeg::Docked &&
                         s.iss_mission_leg() != IssMissionLeg::Ended;
         t += 10.0) {
        s.advance_to(d.t0 + t + 10.0, 1000000);
        saw_docking = saw_docking || s.iss_mission_leg() == IssMissionLeg::Docking;
    }
    INFO(s.iss_mission_status());
    CHECK(s.iss_mission_leg() == IssMissionLeg::Docked);
    CHECK(saw_docking);
    CHECK(s.docked());
    CHECK(s.iss_mission_status().find("Mission abgeschlossen") != std::string::npos);
}

TEST_CASE("ISS mission sequence: an abort waits for resume", "[iss][iss_mission][proximity]") {
    test::spice_session();
    auto d = at_waypoint0();
    auto& s = d.session;
    s.mark_rendezvous_at_waypoint0_for_test();
    s.set_iss_mission(docking_to("ida2"));
    s.advance_to(d.t0 + 10.0, 100000);
    REQUIRE(s.iss_mission_leg() == IssMissionLeg::Proximity);
    REQUIRE(s.proximity_abort());
    s.advance_to(d.t0 + 1800.0, 1000000);
    CHECK(s.iss_mission_leg() == IssMissionLeg::Proximity);  // neither restarted nor ended
    CHECK(s.proximity_phase() == gnc::ProximityPhase::Abort);
}

TEST_CASE("ISS mission sequence: a collision ends the mission", "[iss][iss_mission][docking]") {
    test::spice_session();
    auto c = docking_test::DragonAtPort::beside_modules(0.2, 0.05);
    auto& s = c.session;
    s.set_iss_mission(docking_to("ida2"));
    s.mark_iss_leg_for_test(IssMissionLeg::Proximity);
    REQUIRE(docking_test::DragonAtPort::wait_for(
        s, c.t0, [&] { return s.iss_mission_leg() == IssMissionLeg::Ended; }, 120.0));
    CHECK(s.iss_mission_status() == "Mission beendet: Kollision");
}

TEST_CASE("ISS mission sequence: a refused phasing flight ends the mission with the planner's reason",
          "[iss][iss_mission][rendezvous]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession s(ephemeris, earth, rendezvous_test::config_dir);
    const auto start = rendezvous_test::start(clock, earth, 30.0, 6.0 * 3600.0);
    s.place_in_orbit(config::load_vehicle(rendezvous_test::config_dir, "crew_dragon"), start.dragon, start.epoch);
    s.add_station(start.iss.vehicle, start.iss.state_gcrf, start.iss.epoch, start.iss.impulses);
    auto q = docking_to("ida2");
    q.from_launch = true;  // Ascent: the Dragon is already free in orbit
    q.phasing.mission_limit_s = 1800.0;  // no phasing closes in half an hour
    s.set_iss_mission(q);
    CHECK(s.iss_mission_leg() == IssMissionLeg::Ascent);
    s.advance_to(start.epoch + 10.0, 10000);
    CHECK(s.iss_mission_leg() == IssMissionLeg::Ended);
    CHECK(s.iss_mission_status().rfind("Mission beendet: ", 0) == 0);
}

TEST_CASE("ISS mission sequence: cleared with the bodies", "[iss][iss_mission]") {
    test::spice_session();
    auto d = at_waypoint0();
    d.session.set_iss_mission(docking_to("ida3"));
    const auto dragon = d.session.body(0).state();
    d.session.place_in_orbit(config::load_vehicle(proximity_test::config_dir, "crew_dragon"),
                             {dragon.position_gcrf_m, dragon.velocity_gcrf_mps}, d.session.epoch());
    CHECK(d.session.iss_mission_leg() == IssMissionLeg::None);
    CHECK_FALSE(d.session.iss_mission());
}

TEST_CASE("ISS mission sequence: dock_in_place mates the pair rigidly with momentum conserved",
          "[iss][iss_mission][docking]") {
    test::spice_session();
    auto d = docking_test::DragonAtPort::place("ida2", 30.0, 0.0, 0.0, 0.0);  // 30 m out, at rest
    auto& s = d.session;
    s.dock_in_place();
    CHECK(s.docked());
    CHECK(norm(s.relative_port_state()->position_m) < 1e-6);
    const auto& r = s.docking_merge_record();
    CHECK(norm(r.momentum_after - r.momentum_before) / norm(r.momentum_before) < 1e-12);
    s.advance_to(d.t0 + 3600.0, 1000000);
    CHECK(norm(s.relative_port_state()->position_m) < 0.001);
    CHECK_THROWS_AS(s.dock_in_place(), std::logic_error);  // docked already
    REQUIRE(s.undock());
}

TEST_CASE("ISS mission sequence: WaitUndock undocks at its epoch, also through one long call",
          "[iss][iss_mission][docking]") {
    test::spice_session();
    auto d = docking_test::DragonAtPort::place("ida2", 30.0, 0.0, 0.0, 0.0);
    auto& s = d.session;
    s.dock_in_place();
    sim::IssMissionSequence q;
    q.station_port = "ida2";
    q.undock_epoch = d.t0 + 1234.0;
    s.set_iss_mission(q);
    CHECK(s.iss_mission_leg() == IssMissionLeg::WaitUndock);
    s.advance_to(d.t0 + 1300.0, 1000000);  // one call across the undock epoch
    CHECK(s.iss_mission_leg() == IssMissionLeg::Departure);
    CHECK(s.docking_phase() == contact::DockingPhase::HooksOpening);
    const auto& events = *s.docking_events();
    const auto opened = std::find_if(events.begin(), events.end(), [](const auto& e) {
        return e.phase == contact::DockingPhase::HooksOpening;
    });
    REQUIRE(opened != events.end());
    CHECK(std::abs(opened->t_s - 1234.0) < 0.01);  // the mechanism clock starts at the docked start (t0)
}

TEST_CASE("ISS mission sequence: an undocking by hand before the undock epoch continues as the departure",
          "[iss][iss_mission][docking]") {
    test::spice_session();
    auto d = docking_test::DragonAtPort::place("ida2", 30.0, 0.0, 0.0, 0.0);
    auto& s = d.session;
    s.dock_in_place();
    sim::IssMissionSequence q;
    q.station_port = "ida2";
    q.undock_epoch = d.t0 + 1234.0;
    s.set_iss_mission(q);
    s.advance_to(d.t0 + 10.0, 100000);
    REQUIRE(s.undock());  // the player undocks early (FlightSim::undock)
    s.advance_to(d.t0 + 1300.0, 1000000);
    INFO(s.iss_mission_status());
    CHECK(s.iss_mission_leg() == IssMissionLeg::Departure);
    CHECK(s.iss_mission_status().rfind("Mission beendet", 0) != 0);
}
