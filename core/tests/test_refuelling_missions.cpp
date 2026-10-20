// Refuelling r5: the reference flights - session building blocks, the mission chain, the three missions.

#include <chrono>
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
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/mission_start.hpp"
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

namespace {

// The docked pair of r4 (tanker 200 t at the depot 600 t), Starbase as the return site.
struct DockedAtDepot : refuelling_test::TankerAtPort {
    explicit DockedAtDepot(const config::VehicleConfig& chaser = refuelling_test::tanker_config(200.0e3))
        : TankerAtPort({0.0, 0.0, 0.0}, {}, chaser) {
        session.set_launch_site(
            sim::launch_site_from_config(config::load_launch_site(refuelling_test::config_dir, "starbase_olp1")));
        session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
        session.dock_in_place();
    }
    sim::IssMissionSequence tanker_sequence() const {
        sim::IssMissionSequence q;
        q.station_port = "transfer_dock";
        q.transfer = sim::MissionTransfer::TankerReserve;
        gnc::DeorbitParameters p;
        p.search_window_s = sim::FlightSession::tanker_reserve_search_s;
        q.ship_return = p;
        return q;
    }
    IssMissionLeg leg() const { return session.iss_mission_leg(); }
};

}  // namespace

TEST_CASE("Docked tanker: transfer down to the reserve, undock, depart, deorbit planned", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f;
    f.session.set_iss_mission(f.tanker_sequence());
    REQUIRE(f.leg() == IssMissionLeg::Transfer);  // a docked start with a transfer begins with it
    f.fly([&] { return f.leg() == IssMissionLeg::Departure || f.leg() == IssMissionLeg::Ended; }, 3600.0, 10.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.leg() == IssMissionLeg::Departure);
    CHECK(f.session.transfer_state().phase == sim::TransferPhase::Complete);
    CHECK(f.session.transfer_state().delivered_kg[0] > 140.0e3);  // r4: 144.6 t LOX
    f.fly([&] { return f.leg() != IssMissionLeg::Departure; }, 4.0 * 3600.0, 10.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.leg() == IssMissionLeg::Return);
    REQUIRE(f.session.ship_return_plan());
    CHECK(f.session.ship_return_plan()->feasible);
    const auto& marks = f.session.iss_mission_marks();
    REQUIRE(marks.size() >= 3u);
    CHECK(marks.front().leg == IssMissionLeg::Transfer);
    if (verbose()) {
        for (const auto& m : marks) {
            std::printf("[r5] %-24s +%8.0f s  main %.1f t  station %.1f t\n", sim::to_string(m.leg),
                        m.epoch - marks.front().epoch, m.chaser_main_propellant_kg / 1e3,
                        m.station_main_propellant_kg / 1e3);
        }
    }
}

TEST_CASE("Docked tanker: the ship return flies to the catch", "[refuelling][iss_mission][ship][slow]") {
    test::spice_session();
    DockedAtDepot f;
    f.session.set_iss_mission(f.tanker_sequence());
    f.fly([&] { return f.leg() == IssMissionLeg::Complete || f.leg() == IssMissionLeg::Ended; }, 10.0 * 86400.0,
          600.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.leg() == IssMissionLeg::Complete);
    const auto& tanker = f.session.body(f.session.controlled_index());
    CHECK(tanker.caught());
    const double main_kg = tanker.vehicle().main_propellant_kg();
    std::printf("[r5] tanker caught: main tanks %.2f t, headers %.2f t\n", main_kg / 1e3,
                (tanker.vehicle().propellant_kg() - main_kg) / 1e3);
}

TEST_CASE("A stopped transfer ends the mission", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f;
    f.session.set_iss_mission(f.tanker_sequence());
    f.fly([&] { return f.session.transfer_state().phase == sim::TransferPhase::Flowing; }, 600.0, 10.0);
    REQUIRE(f.session.transfer_state().phase == sim::TransferPhase::Flowing);
    f.session.stop_transfer();
    f.fly([&] { return f.leg() != IssMissionLeg::Transfer; }, 60.0, 1.0);
    CHECK(f.leg() == IssMissionLeg::Ended);
    CHECK_THAT(f.session.iss_mission_status(), Catch::Matchers::StartsWith("Mission beendet"));
    CHECK(f.session.docked());  // no undocking after a failed transfer
}

TEST_CASE("Ship return refused ends the mission", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f;
    auto q = f.tanker_sequence();
    q.ship_return->search_window_s = 600.0;  // [test] no pass within reach in 10 min
    f.session.set_iss_mission(q);
    f.fly([&] { return f.leg() == IssMissionLeg::Return || f.leg() == IssMissionLeg::Ended; }, 6.0 * 3600.0, 10.0);
    CHECK(f.leg() == IssMissionLeg::Ended);
    CHECK_THAT(f.session.iss_mission_status(), Catch::Matchers::StartsWith("Mission beendet"));
}

TEST_CASE("WaitContinue coasts to its epoch and hands over to the continuation", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f(refuelling_test::ship_with("starship_hls", 1000.0e3));
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    q.transfer = sim::MissionTransfer::FillChaser;
    const time::Epoch at = f.session.epoch() + 2.0 * 86400.0;
    q.continue_epoch = at;
    time::Epoch called{};
    q.continuation = [&](sim::FlightSession& s) {
        called = s.epoch();
        return std::string();
    };
    f.session.set_iss_mission(q);
    const auto wall = std::chrono::steady_clock::now();
    f.fly([&] { return f.leg() == IssMissionLeg::Complete || f.leg() == IssMissionLeg::Ended; }, 3.0 * 86400.0,
          3600.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.leg() == IssMissionLeg::Complete);
    CHECK(std::abs(called - at) <= 0.5 * sim::FlightSession::tick_s + 1e-6);
    // the depot's 600 t went into the HLS (less chill-down and boil-off), its main tanks not full yet
    const auto& hls = f.session.body(f.session.controlled_index()).vehicle();
    CHECK(hls.main_propellant_kg() > 1000.0e3 + 600.0e3 - 30.0e3);  // [test] chill-down / settling-gas headroom
    CHECK(std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count() < 60.0);  // coasted
}

TEST_CASE("Continuation refusal ends the mission", "[refuelling][iss_mission]") {
    test::spice_session();
    DockedAtDepot f(refuelling_test::ship_with("starship_hls", 1000.0e3));
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    q.transfer = sim::MissionTransfer::FillChaser;
    q.continue_epoch = f.session.epoch() + 3.0 * 3600.0;
    q.continuation = [](sim::FlightSession&) { return std::string("Mondmission: zu spät"); };
    f.session.set_iss_mission(q);
    f.fly([&] { return f.leg() == IssMissionLeg::Complete || f.leg() == IssMissionLeg::Ended; }, 6.0 * 3600.0, 60.0);
    CHECK(f.leg() == IssMissionLeg::Ended);
    CHECK(f.session.iss_mission_status() == "Mission beendet: Mondmission: zu spät");
}

TEST_CASE("depot_launch starts the depot ascent through start_mission", "[refuelling][mission]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "depot_launch", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    CHECK(s.kind == config::MissionKind::DepotLaunch);
    CHECK(s.site_id == "starbase_olp1");
    CHECK(s.epoch == f.t0);  // the mission's epoch_utc
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Ascent);
    CHECK_FALSE(f.session.station_index());
}

TEST_CASE("depot_launch: pad -> orbit, the depot becomes the station, the booster is caught",
          "[refuelling][mission][ascent][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    REQUIRE(sim::start_mission(f.session, refuelling_test::config_dir, "depot_launch", f.t0, f.clock).refusal.empty());
    while (f.session.epoch() - f.t0 < 1800.0 && f.session.iss_mission_leg() == IssMissionLeg::Ascent) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    INFO(f.session.iss_mission_status());
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Complete);
    REQUIRE(f.session.station_index());
    const double insertion_s = f.session.iss_mission_marks().back().epoch - f.t0;
    const auto& depot = f.session.body(*f.session.station_index());
    const double fill = depot.vehicle().main_propellant_kg();
    const auto e = orbit::elements_from_state({depot.state().position_gcrf_m, depot.state().velocity_gcrf_mps},
                                              constants::earth_gm_m3ps2);
    const double rp = e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m;
    const double ra = e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m;
    while (f.session.epoch() - f.t0 < 1800.0 && f.session.tower()->phase() != sim::CatchPhase::Caught) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    CHECK(f.session.tower()->phase() == sim::CatchPhase::Caught);
    CHECK(rp > 185.0e3);  // [E] insertion within ~15 km of the 200 km target
    CHECK(ra < 215.0e3);
    CHECK(fill > 0.0);
    // one orbit as the station: ideal attitude, boil-off on
    const double before = f.session.body(*f.session.station_index()).vehicle().main_propellant_kg();
    f.session.advance_to(f.session.epoch() + 5400.0);
    const double boil_kg = before - f.session.body(*f.session.station_index()).vehicle().main_propellant_kg();
    CHECK(boil_kg > 0.0);
    std::printf("[r5] depot_launch: station %.0f s after liftoff, %.1f x %.1f km, initial fill %.1f t (main), "
                "boil-off %.1f kg in 90 min\n",
                insertion_s, rp / 1e3, ra / 1e3, fill / 1e3, boil_kg);
}
