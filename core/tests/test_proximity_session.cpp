// ISS i4: proximity operations in the flight session - start_proximity, its refusals, the docking attitude and the
// clearing paths (the approach campaigns: test_proximity_campaign.cpp).

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "proximity_test_support.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("start_proximity refusals", "[iss][proximity][session]") {
    test::spice_session();
    {  // no station
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}), /*with_station=*/false);
        CHECK_FALSE(d.session.start_proximity("ida2"));
        CHECK(d.session.proximity_status().find("keine Station") != std::string::npos);
    }
    {  // too far: 8 km behind
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-8000, 0, 0}));
        CHECK_FALSE(d.session.start_proximity("ida2"));
        CHECK(d.session.proximity_status().find("zu weit") != std::string::npos);
    }
    {  // unknown port
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}));
        CHECK_FALSE(d.session.start_proximity("pma1"));
        CHECK_FALSE(d.session.proximity_active());
        CHECK(d.session.proximity_status().find("pma1") != std::string::npos);
    }
    {  // inside the sphere without an on-axis start
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 150}));
        CHECK_FALSE(d.session.start_proximity("ida2"));
        CHECK(d.session.proximity_status().find("Sicherheitszone") != std::string::npos);
    }
    {  // commands without a flight
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}));
        CHECK_FALSE(d.session.proximity_hold());
        CHECK_FALSE(d.session.proximity_resume());
        CHECK_FALSE(d.session.proximity_abort());
        CHECK_FALSE(d.session.proximity_retreat());
        CHECK(d.session.proximity_status() == "Kein Anflug aktiv");
        CHECK_FALSE(d.session.proximity_phase());
        CHECK(d.session.proximity_events() == nullptr);
    }
}

TEST_CASE("start_proximity at Waypoint 0: docking attitude, cone open, ticking", "[iss][proximity][session]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5, 0, 397}), true, /*cone_open=*/false);
    auto& s = d.session;
    REQUIRE(s.start_proximity("ida2"));
    CHECK(s.body(0).vehicle().nose_cone_open());
    CHECK(s.autopilot().state == sim::AutopilotState::Proximity);
    CHECK(*s.proximity_phase() == gnc::ProximityPhase::Waypoint);
    s.advance_to(d.t0 + 240.0, 1000000);  // inside the 300 s hold
    const auto rel = *s.relative_port_state();
    CHECK(rel.misalignment_rad < 1.0 * constants::deg_to_rad);  // turned into the docking attitude
    CHECK(std::abs(rel.clocking_rad) < 1.0 * constants::deg_to_rad);
    CHECK(*s.proximity_phase() == gnc::ProximityPhase::Waypoint);
    CHECK(s.epoch() - d.t0 > 239.0);  // ticked through, not refused
}

TEST_CASE("Placing a body clears the proximity flight", "[iss][proximity][session]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5, 0, 397}));
    REQUIRE(d.session.start_proximity("ida2"));
    d.session.place_in_orbit(config::load_vehicle(proximity_test::config_dir, "crew_dragon"),
                             {d.session.body(0).state().position_gcrf_m, d.session.body(0).state().velocity_gcrf_mps},
                             d.session.epoch());
    CHECK_FALSE(d.session.proximity_active());
    CHECK(norm(d.session.translation_command()) == 0.0);
    CHECK(d.session.autopilot().state == sim::AutopilotState::Off);
}

TEST_CASE("Disengaging the autopilot ends the proximity flight", "[iss][proximity][session]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5, 0, 397}));
    REQUIRE(d.session.start_proximity("ida2"));
    d.session.disengage_autopilot();
    CHECK_FALSE(d.session.proximity_active());
    CHECK(norm(d.session.translation_command()) == 0.0);
}
