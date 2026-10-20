// Refuelling r5: the reference flights - session building blocks, the mission chain, the three missions.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "refuelling_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;
using sim::IssMissionLeg;

namespace {

constexpr double deg = constants::deg_to_rad;
bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

// The depot as an ordinary ship body in its orbit (not yet a station).
void place_depot_ship(refuelling_test::DepotSession& f) {
    f.session.place_in_orbit(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
}

}  // namespace

TEST_CASE("make_station turns a flown depot into the station with the ideal attitude", "[refuelling][station]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    place_depot_ship(f);
    REQUIRE_FALSE(f.session.station_index());
    REQUIRE(f.session.make_station(0) == 0);
    REQUIRE(f.session.station_index() == 0u);
    CHECK(f.session.autopilot().state == sim::AutopilotState::Off);
    // the same attitude add_station gives the same config at the same state
    refuelling_test::DepotSession g;
    g.session.add_station(refuelling_test::depot_config(g.depot_propellant_kg), g.depot, g.t0);
    f.session.advance_to(f.t0 + 60.0);
    g.session.advance_to(g.t0 + 60.0);
    const auto qa = f.session.body(0).state().attitude, qb = g.session.body(0).state().attitude;
    CHECK(std::abs(std::abs(qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z) - 1.0) < 1e-12);
    CHECK(norm(f.session.body(0).state().position_gcrf_m - g.session.body(0).state().position_gcrf_m) < 1e-3);
}

TEST_CASE("make_station refuses a stack, a second station and a vehicle without [stage.station]",
          "[refuelling][station]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(config::load_vehicle(refuelling_test::config_dir, "starship_depot"), f.depot, f.t0);
    CHECK_THROWS_AS(f.session.make_station(0), std::logic_error);  // booster + depot
    refuelling_test::DepotSession g;
    g.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), g.depot, g.t0);
    CHECK_THROWS_AS(g.session.make_station(0), std::logic_error);  // no [stage.station]
    refuelling_test::DepotSession h;
    place_depot_ship(h);
    h.session.make_station(0);
    CHECK_THROWS_AS(h.session.make_station(0), std::logic_error);  // already the station
}

TEST_CASE("release_tower refuses without a caught body", "[refuelling][catch]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    place_depot_ship(f);
    CHECK_FALSE(f.session.release_tower());  // no tower
    f.session.set_launch_site(
        sim::launch_site_from_config(config::load_launch_site(refuelling_test::config_dir, "starbase_olp1")));
    CHECK_FALSE(f.session.release_tower());  // tower Open
    CHECK(f.session.body_count() == 1u);
}

TEST_CASE("release_tower keeps the indices: booster caught, then lifted off; the ship flies on",
          "[refuelling][catch][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto m = config::load_ascent_mission(refuelling_test::config_dir, "starship_leo");
    sim::start_ascent_mission(f.session, m, refuelling_test::config_dir, f.t0);
    while (f.session.epoch() - f.t0 < 1200.0 &&
           !(f.session.tower() && f.session.tower()->phase() == sim::CatchPhase::Caught)) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    REQUIRE(f.session.tower()->phase() == sim::CatchPhase::Caught);
    REQUIRE(f.session.body_count() == 2u);
    const size_t ship = f.session.controlled_index();
    REQUIRE(ship == 1u);  // behind the booster: the erase shifts it
    const double ship_mass = f.session.body(ship).vehicle().mass_properties().mass_kg;
    // a station behind the booster in the list
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg),
                          refuelling_test::depot_state(f.earth, f.session.epoch()), f.session.epoch());
    REQUIRE(f.session.release_tower());
    CHECK(f.session.body_count() == 2u);
    CHECK(f.session.tower()->phase() == sim::CatchPhase::Open);
    CHECK(f.session.controlled_index() == 0u);
    CHECK(f.session.body(f.session.controlled_index()).vehicle().mass_properties().mass_kg ==
          Catch::Approx(ship_mass));
    REQUIRE(f.session.station_index());
    CHECK(*f.session.station_index() == 1u);
    CHECK(f.session.body(*f.session.station_index()).vehicle().stages().front().config.id == "depot");
    CHECK_FALSE(f.session.release_tower());
    f.session.advance_to(f.session.epoch() + 60.0);  // runs on without the booster
}

TEST_CASE("Leg marks record the chaser's mass and propellant at every leg change", "[refuelling][iss_mission]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    q.undock_epoch = f.session.epoch() + 30.0;
    f.session.set_iss_mission(q);
    const auto& marks = f.session.iss_mission_marks();
    REQUIRE(marks.size() == 1u);
    CHECK(marks[0].leg == IssMissionLeg::WaitUndock);
    CHECK(marks[0].chaser_main_propellant_kg == Catch::Approx(200.0e3).epsilon(1e-6));
    CHECK(marks[0].station_main_propellant_kg == Catch::Approx(600.0e3).epsilon(1e-6));
    f.fly([&] { return f.session.iss_mission_leg() == IssMissionLeg::Departure; }, 120.0);
    REQUIRE(marks.size() == 2u);
    CHECK(marks[1].leg == IssMissionLeg::Departure);
    CHECK(marks[1].epoch - marks[0].epoch == Catch::Approx(30.0).margin(0.01));
}
