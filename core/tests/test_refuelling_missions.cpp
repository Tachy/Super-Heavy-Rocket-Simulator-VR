// Refuelling r5: the reference flights - session building blocks, the mission chain, the three missions.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <vector>
#include <stdexcept>

#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "refuelling_measurements.hpp"
#include "refuelling_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/config/refuelling_reference.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/refuelling_mission.hpp"
#include "starsim/vehicle/boil_off.hpp"
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

// Review r5 I2: a tower whose booster catch was aborted (or missed) is re-armed for the ship as well.
TEST_CASE("release_tower re-arms a tower after an aborted booster catch", "[refuelling][catch][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto m = config::load_ascent_mission(refuelling_test::config_dir, "starship_leo");
    sim::start_ascent_mission(f.session, m, refuelling_test::config_dir, f.t0);
    while (f.session.epoch() - f.t0 < 300.0 || f.session.body_count() < 2) f.session.advance_to(f.session.epoch() + 1.0);
    REQUIRE(f.session.abort_landing(0));  // the booster gives up its return
    REQUIRE(f.session.tower()->phase() == sim::CatchPhase::Aborted);
    const size_t bodies = f.session.body_count();
    REQUIRE(f.session.release_tower());
    CHECK(f.session.tower()->phase() == sim::CatchPhase::Open);
    CHECK(f.session.body_count() == bodies);  // nothing caught: nothing lifted off
    CHECK_FALSE(f.session.release_tower());   // Open: nothing to do
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

// Refuelling f1: the tanker just inserted at 120 x 190 km (at its periapsis), 40 deg ahead of the depot (station),
// its mission at the end of the ascent leg with a raise to `to`.
struct RaiseFixture : refuelling_test::DepotSession {
    explicit RaiseFixture(double main_kg) {
        const double re = constants::wgs84_semi_major_axis_m;
        const auto ahead = refuelling_test::depot_state(earth, t0, 40.0 * deg);
        const double rp = re + 120e3, ra = re + 190e3;
        const double vp = std::sqrt(constants::earth_gm_m3ps2 * (2.0 / rp - 2.0 / (rp + ra)));
        const math::StateVector tanker{rp * math::normalized(ahead.position_m),
                                       vp * math::normalized(ahead.velocity_mps)};
        session.place_in_orbit(refuelling_test::tanker_config(main_kg), tanker, t0);
        session.add_station(refuelling_test::depot_config(depot_propellant_kg), depot, t0);
        sequence.from_launch = true;
        sequence.station_port = "transfer_dock";
        sequence.phasing.mission_limit_s = 48.0 * 3600.0;
    }
    void start() { session.set_iss_mission(sequence); }
    void fly_until(IssMissionLeg leg, double limit_s) {
        const time::Epoch end = session.epoch() + limit_s;
        while (session.epoch() < end && session.iss_mission_leg() != leg &&
               session.iss_mission_leg() != IssMissionLeg::Ended) {
            session.advance_to(session.epoch() + 10.0);
        }
    }
    std::pair<double, double> apsides() const {
        const auto& s = session.body(0).state();
        const auto el = orbit::elements_from_state({s.position_gcrf_m, s.velocity_gcrf_mps}, constants::earth_gm_m3ps2);
        return {el.semi_major_axis_m * (1.0 - el.eccentricity), el.semi_major_axis_m * (1.0 + el.eccentricity)};
    }
    sim::IssMissionSequence sequence;
};

}  // namespace

TEST_CASE("Raise: two burns to an orbit above the depot, then the phasing plan", "[refuelling][iss_mission]") {
    test::spice_session();
    const double re = constants::wgs84_semi_major_axis_m;
    RaiseFixture f(100e3);
    f.sequence.raise_to = mission::PhasingOrbit{re + 250e3, re + 270e3};
    f.start();
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Ascent);
    f.fly_until(IssMissionLeg::Rendezvous, 4.0 * 3600.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    const auto& marks = f.session.iss_mission_marks();
    CHECK(std::count_if(marks.begin(), marks.end(), [](const auto& m) { return m.leg == IssMissionLeg::Raise; }) == 2);
    // [E] two-body burns from osculating apsides: J2 short-period terms (~10 km in a) shift the raised orbit; the
    // phasing plan that follows runs in the full model from the live state
    const auto [rp, ra] = f.apsides();
    CHECK(rp == Catch::Approx(re + 250e3).margin(20e3));
    CHECK(ra == Catch::Approx(re + 270e3).margin(20e3));
    CHECK(rp > re + 200e3 + mission::PhasingParameters{}.coelliptic_height_m);  // above the depot: the mirrored case
    const double used_kg = 100e3 - marks.back().chaser_main_propellant_kg;
    std::printf("[f1] raise 120 x 190 -> %.1f x %.1f km: %.2f t main propellant\n", (rp - re) / 1e3, (ra - re) / 1e3,
                used_kg / 1e3);
    CHECK(used_kg < 12e3);  // ~44 + 21 m/s on ~310 t: ~6 t, plus the bare tanker's boil-off over ~3 h (~3.5 t)
}

TEST_CASE("Raise: a burn that runs dry ends the mission with its reason", "[refuelling][iss_mission]") {
    test::spice_session();
    const double re = constants::wgs84_semi_major_axis_m;
    RaiseFixture f(50.0);  // almost empty main tanks
    f.sequence.raise_to = mission::PhasingOrbit{re + 250e3, re + 270e3};
    f.start();
    f.fly_until(IssMissionLeg::Rendezvous, 2.0 * 3600.0);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Ended);
    CHECK(f.session.iss_mission_status().find("kein Treibstoff") != std::string::npos);
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
    refuelling_test::check_reference(fill, config::load_refuelling_reference(refuelling_test::config_dir).depot.initial_propellant_kg,
                                     "depot initial fill");
}

TEST_CASE("tanker_flight_orbit starts behind the depot and plans the phasing", "[refuelling][mission][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight_orbit", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    CHECK(s.kind == config::MissionKind::TankerFlight);
    CHECK(s.epoch == f.t0);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    REQUIRE(f.session.station_index());
    CHECK(f.session.body(*f.session.station_index()).vehicle().main_propellant_kg() == Catch::Approx(600.0e3));
    const auto m = config::load_refuelling_flight_mission(refuelling_test::config_dir, "tanker_flight_orbit");
    CHECK(f.session.body(f.session.controlled_index()).vehicle().main_propellant_kg() ==
          Catch::Approx(m.start_propellant_kg));
    CHECK(f.session.launch_site().latitude_rad ==
          Catch::Approx(config::load_launch_site(refuelling_test::config_dir, "starbase_olp1").latitude_rad));
}

TEST_CASE("tanker_flight finds a launch window into the depot plane within the uncrewed limit",
          "[refuelling][mission][launch_window]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    REQUIRE(s.launch_window);
    CHECK(s.launch_window->feasible);
    CHECK(s.epoch == s.launch_window->liftoff);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Ascent);
    REQUIRE(f.session.station_index());
    std::printf("[r5] tanker window: liftoff %.1f h after earliest, %d crossings rejected\n",
                (s.launch_window->liftoff - f.t0) / 3600.0, s.launch_window->rejected);
}

// Probe (f1 plan, Task 4): the planning boil-off rates of the phasing-orbit choice - the bare tanker with its residue
// and the insulated HLS with its residue after the ascent, orbit-averaged on the depot orbit at 2026-10-20.
TEST_CASE("Probe: planning boil-off of the tanker and the HLS", "[.f1_probe]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto sun = f.ephemeris.state_gcrf(ephemeris::Body::Sun, f.t0).position_m;
    for (const auto& [id, residue_kg] : {std::pair{std::string("starship_tanker"), 99.2e3},
                                         std::pair{std::string("starship_hls_launch"), 208.0e3}}) {
        const vehicle::Vehicle v(refuelling_test::ship_with(id, residue_kg), refuelling_test::config_dir);
        const auto rate = vehicle::orbit_average_boil_off(v, f.depot, {}, sun);
        std::printf("[f1 probe] %s with %.1f t main: %.2f t/day\n", id.c_str(), residue_kg / 1e3,
                    (rate.oxidizer_kgps + rate.fuel_kgps) * 86400.0 / 1e3);
    }
}

TEST_CASE("tanker_flight: an above-depot window is flown to docking", "[refuelling][mission][slow]") {
    test::spice_session();
    const double re = constants::wgs84_semi_major_axis_m;
    refuelling_test::DepotSession f;
    auto m = config::load_refuelling_flight_mission(refuelling_test::config_dir, "tanker_flight");
    m.phasing_candidates_m = {{220.0e3, 240.0e3}, {250.0e3, 270.0e3}};  // only orbits above the depot (f1)
    const auto s = sim::start_refuelling_flight_mission(f.session, refuelling_test::config_dir, m, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    REQUIRE(s.launch_window->phasing_orbit);
    REQUIRE(s.launch_window->phasing_orbit->periapsis_radius_m > re + 200.0e3);
    REQUIRE(s.launch_window->schedule.above);
    const auto start = std::chrono::steady_clock::now();
    while (f.session.epoch() - s.epoch < 60.0 * 3600.0 && f.session.iss_mission_leg() != IssMissionLeg::Docking &&
           f.session.iss_mission_leg() != IssMissionLeg::Docked && f.session.iss_mission_leg() != IssMissionLeg::Ended) {
        f.session.advance_to(f.session.epoch() + 60.0);
    }
    INFO(f.session.iss_mission_status());
    CHECK((f.session.iss_mission_leg() == IssMissionLeg::Docking || f.session.iss_mission_leg() == IssMissionLeg::Docked));
    const auto wp0 = refuelling_test::mark(f.session, IssMissionLeg::Proximity);
    REQUIRE(wp0);
    std::printf("[f1] above-depot flight: liftoff +%.1f h, orbit %.0f x %.0f km, Waypoint 0 %.1f h after liftoff "
                "(window %.1f h), main %.1f t at Waypoint 0, wall %.0f s\n",
                (s.epoch - f.t0) / 3600.0, (s.launch_window->phasing_orbit->periapsis_radius_m - re) / 1e3,
                (s.launch_window->phasing_orbit->apoapsis_radius_m - re) / 1e3, (wp0->epoch - s.epoch) / 3600.0,
                (s.launch_window->schedule.waypoint0_epoch - s.epoch) / 3600.0, wp0->chaser_main_propellant_kg / 1e3,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
}

TEST_CASE("Refuelling flights: the window's separation estimate matches the flown ascent",
          "[refuelling][mission][ascent][slow]") {
    test::spice_session();
    const std::string id = GENERATE(std::string("tanker_flight"), std::string("hls_refuel"));
    CAPTURE(id);
    refuelling_test::DepotSession f;
    const spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();  // hls_refuel: the lunar plane
    sim::FlightSession session{f.ephemeris, f.earth, refuelling_test::config_dir, {}, &moon};
    const auto m = config::load_refuelling_flight_mission(refuelling_test::config_dir, id);
    const auto s = sim::start_mission(session, refuelling_test::config_dir, id, f.t0, f.clock);
    REQUIRE(s.refusal.empty());
    const auto& w = *s.launch_window;
    while (session.iss_mission_leg() == IssMissionLeg::Ascent && session.epoch() - w.liftoff < 1200.0) {
        session.advance_to(session.epoch() + 1.0);
    }
    INFO(session.iss_mission_status());
    REQUIRE(session.iss_mission_leg() == IssMissionLeg::Raise);  // the 120 km insertion is raised at apoapsis
    const auto& marks = session.iss_mission_marks();  // Ascent, Raise
    REQUIRE(marks.size() >= 2u);
    const double seco_s = marks[1].epoch - w.liftoff;
    {
        const auto e = orbit::elements_from_state({session.body(session.controlled_index()).state().position_gcrf_m,
                                                   session.body(session.controlled_index()).state().velocity_gcrf_mps},
                                                  constants::earth_gm_m3ps2);
        const double ra = e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m;
        std::printf("[r5] at the end of the ascent leg: %s, %.1f x %.1f km\n", sim::to_string(marks[1].leg),
                    (e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m) / 1e3,
                    ra / 1e3);
        // the leg ends after the engines' tail-off (its ~12 m/s at 5 g lift the apoapsis by ~35 km)
        CHECK(ra == Catch::Approx(190.0e3).margin(5.0e3));
    }
    const auto& ship = session.body(session.controlled_index());
    const Vec3 r = ship.state().position_gcrf_m, v = ship.state().velocity_gcrf_mps;
    const Vec3 n = math::normalized(cross(r, v));
    const auto site = config::load_launch_site(refuelling_test::config_dir, "starbase_olp1");
    const Vec3 pad = f.earth.fixed_to_gcrf(w.liftoff).rotation *
                     frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const Vec3 pad_in_plane = pad - dot(pad, n) * n;
    const double arc = std::atan2(dot(cross(pad_in_plane, r), n), dot(pad_in_plane, r));
    // the estimate is the state at liftoff + insertion_s; the flown ship is at liftoff + seco_s
    const Vec3 r_est = w.separation_state_gcrf.position_m;
    const double angle = std::acos(std::clamp(dot(math::normalized(r), math::normalized(r_est)), -1.0, 1.0));
    std::printf("[r5] %s SECO %.1f s after liftoff (config %.1f), arc %.3f deg (config %.3f), %.3f deg from the "
                "estimate, main %.1f t, plane %.4f deg from the depot's\n",
                id.c_str(), seco_s, m.window.insertion_s, arc / deg, m.window.insertion_arc_rad / deg, angle / deg,
                ship.vehicle().main_propellant_kg() / 1e3,
                std::acos(std::clamp(dot(n, math::normalized(cross(session.body(*session.station_index())
                                                                           .state()
                                                                           .position_gcrf_m,
                                                                       session.body(*session.station_index())
                                                                           .state()
                                                                           .velocity_gcrf_mps))),
                                     -1.0, 1.0)) /
                    deg);
    CHECK(std::abs(seco_s - m.window.insertion_s) < 5.0);  // [E] calibrated (i6: Falcon 9 within 1 s)
    CHECK(angle < 0.1 * deg);                               // [E] i6: 0.004 deg in phase
}

namespace {

// Flies until the mission ends (advance_to coasts where nothing is ticked).
void fly_mission(refuelling_test::DepotSession& f, double limit_s) {
    const time::Epoch start = f.session.epoch();
    while (f.session.epoch() - start < limit_s && f.session.iss_mission_leg() != IssMissionLeg::Complete &&
           f.session.iss_mission_leg() != IssMissionLeg::Ended) {
        f.session.advance_to(f.session.epoch() + 600.0);
    }
    if (verbose()) {
        const auto& marks = f.session.iss_mission_marks();
        for (const auto& m : marks) {
            std::printf("[r5] %-24s +%8.0f s  mass %.1f t  main %.2f t  station %.1f t\n", sim::to_string(m.leg),
                        m.epoch - marks.front().epoch, m.chaser_mass_kg / 1e3, m.chaser_main_propellant_kg / 1e3,
                        m.station_main_propellant_kg / 1e3);
        }
    }
}

}  // namespace

TEST_CASE("tanker_flight_orbit: phasing orbit -> docked -> transfer -> caught at Starbase",
          "[refuelling][mission][docking][ship][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    REQUIRE(sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight_orbit", f.t0, f.clock)
                .refusal.empty());
    fly_mission(f, 12.0 * 86400.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Complete);
    const auto n = refuelling_test::tanker_numbers(f.session);
    refuelling_test::print("tanker_flight_orbit", n);
    CHECK(n.delivered_oxidizer_kg + n.delivered_fuel_kg > 0.0);
    CHECK(n.main_at_catch_kg >= 0.0);  // the reserve held: caught, not starved
    CHECK(f.session.body(f.session.controlled_index()).caught());
}

TEST_CASE("tanker_flight: pad -> booster caught -> docked -> transfer -> ship caught",
          "[refuelling][mission][docking][ship][ascent][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto s = sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight", f.t0, f.clock);
    REQUIRE(s.refusal.empty());
    // the ascent and the booster return are ticked: 1 s calls until the booster is caught
    while (f.session.epoch() - s.epoch < 1200.0 && f.session.tower()->phase() != sim::CatchPhase::Caught) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    REQUIRE(f.session.tower()->phase() == sim::CatchPhase::Caught);
    fly_mission(f, 12.0 * 86400.0);
    INFO(f.session.iss_mission_status());
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Complete);
    const auto n = refuelling_test::tanker_numbers(f.session);
    refuelling_test::print("tanker_flight", n);
    const auto ref = config::load_refuelling_reference(refuelling_test::config_dir).tanker;
    refuelling_test::check_reference(n.residue_after_ascent_kg, ref.residue_after_ascent_kg, "residue after ascent");
    refuelling_test::check_reference(n.rendezvous_kg, ref.rendezvous_kg, "rendezvous");
    refuelling_test::check_reference(n.delivered_oxidizer_kg, ref.delivered_oxidizer_kg, "delivered LOX");
    refuelling_test::check_reference(n.delivered_fuel_kg, ref.delivered_fuel_kg, "delivered CH4");
    refuelling_test::check_reference(n.chilldown_kg, ref.chilldown_kg, "chill");
    refuelling_test::check_reference(n.settling_gas_kg, ref.settling_gas_kg, "settling gas");
    refuelling_test::check_reference(n.main_at_undock_kg, ref.main_at_undock_kg, "main at undock");
    refuelling_test::check_reference(n.main_at_catch_kg, ref.main_at_catch_kg, "main at catch");
    CHECK(n.residue_after_ascent_kg > 0.0);
    CHECK(n.delivered_oxidizer_kg + n.delivered_fuel_kg > 0.0);
    CHECK(f.session.body(f.session.controlled_index()).caught());
    CHECK(f.session.body_count() == 2u);  // tanker + depot: the booster was lifted off the tower
}

// User 2026-10-20: the tanker inserts at a 120 km perigee (less propellant on the ascent; at 80 km the drag of the
// coast to apoapsis, ~60 m/s, brought it back down) and raises it to the phasing orbit's 170 km at the first apoapsis.
TEST_CASE("After the ascent the sequence raises the perigee at apoapsis, then plans the phasing",
          "[refuelling][iss_mission][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto depot = f.depot;
    f.session.place_in_orbit(refuelling_test::tanker_config(100.0e3),
                             sim::phasing_start_state(depot, 8.0 * deg, 120.0e3, 190.0e3), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), depot, f.t0);
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    q.from_launch = true;
    q.phasing.mission_limit_s = 48.0 * 3600.0;
    q.raise_to = mission::PhasingOrbit{constants::wgs84_semi_major_axis_m + 170.0e3,
                                       constants::wgs84_semi_major_axis_m + 190.0e3};
    f.session.set_iss_mission(q);
    double t = 0.0;
    while (t < 4.0 * 3600.0 && f.session.iss_mission_leg() != IssMissionLeg::Rendezvous &&
           f.session.iss_mission_leg() != IssMissionLeg::Ended) {
        f.session.advance_to(f.session.epoch() + 10.0);
        t += 10.0;
    }
    INFO(f.session.iss_mission_status());
    INFO(f.session.body(f.session.controlled_index()).breakup_reason());
    INFO("elapsed " << t << " s");
    REQUIRE(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    REQUIRE(refuelling_test::mark(f.session, IssMissionLeg::Raise));
    const auto& s = f.session.body(f.session.controlled_index()).state();
    const auto e = orbit::elements_from_state({s.position_gcrf_m, s.velocity_gcrf_mps}, constants::earth_gm_m3ps2);
    const double rp = e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m;
    CHECK(rp == Catch::Approx(170.0e3).margin(3.0e3));  // [E] the burn's execution and J2 osculation
    const double used_kg = 100.0e3 - refuelling_test::mark(f.session, IssMissionLeg::Rendezvous)->chaser_main_propellant_kg;
    std::printf("[r5] perigee raise 120 -> %.1f km: %.2f t main propellant (incl. boil-off)\n", rp / 1e3, used_kg / 1e3);
    CHECK(used_kg < 5.0e3);  // ~15 m/s on ~360 t
}

// Probe (user 2026-10-20: levers in the flight for the tanker's residue): the tanker ascent with one change at a
// time; prints the main-tank residue at insertion, the equivalent residue in the 170 x 190 km phasing orbit (a lower
// insertion perigee pays the raise at apoapsis on the belly Raptor, c ~3.4 km/s [E]) and whether the booster is caught.
TEST_CASE("Tanker ascent levers", "[.r5_probe]") {
    test::spice_session();
    struct Variant {
        const char* name;
        std::function<void(config::AscentMission&)> change;
        std::function<void(config::VehicleConfig&)> vehicle = [](config::VehicleConfig&) {};
    };
    // the ship's main tanks grown by kg (nose volume, at their O/F) with [E] 3 % of it as tank structure
    const auto bigger_tanks = [](double kg) {
        return [kg](config::VehicleConfig& v) {
            auto& ship = v.stages.back();
            double main = 0.0;
            for (const auto& t : ship.tanks) main += t.role == config::TankRole::Main ? t.capacity_kg : 0.0;
            for (auto& t : ship.tanks) {
                if (t.role != config::TankRole::Main) continue;
                t.capacity_kg += kg * t.capacity_kg / main;
                t.mass_kg = t.capacity_kg;
            }
            ship.dry_mass_kg += 0.03 * kg;
        };
    };
    const auto lighter = [](double kg) { return [kg](config::VehicleConfig& v) { v.stages.back().dry_mass_kg -= kg; }; };
    const auto none = [](config::AscentMission&) {};
    const auto kick = [](double k_deg, double rp_km) {
        return [k_deg, rp_km](config::AscentMission& m) {
            m.kick_angle_rad = k_deg * deg;
            m.periapsis_height_m = rp_km * 1e3;
        };
    };
    const auto booster = [](double dry_kg, double landing_kg) {
        return [dry_kg, landing_kg](config::VehicleConfig& v) {
            v.stages.front().dry_mass_kg += dry_kg;
            v.stages.front().landing_propellant_kg += landing_kg;
        };
    };
    const std::vector<Variant> variants = {
        {"baseline (repeat)", none},
        {"booster dry -20 t", none, booster(-20.0e3, 0.0)},
        {"booster dry -40 t", none, booster(-40.0e3, 0.0)},
        {"booster landing 41 t (-10)", none, booster(0.0, -10.0e3)},
        {"80 x 190 + ship dry -10 t", kick(14.0, 80.0), lighter(10.0e3)},
        {"80 x 190 + booster dry -40 t", kick(14.0, 80.0), booster(-40.0e3, 0.0)},
    };
    const double target_rp = constants::wgs84_semi_major_axis_m + 170.0e3;
    for (const auto& variant : variants) {
        refuelling_test::DepotSession f;
        auto m = config::load_ascent_mission(refuelling_test::config_dir, "tanker_ascent");
        variant.change(m);
        auto vehicle = config::load_vehicle(refuelling_test::config_dir, m.vehicle_id);
        variant.vehicle(vehicle);
        sim::start_ascent_mission(f.session, m, refuelling_test::config_dir, f.t0, &vehicle);
        double insertion_s = 0.0, main_kg = 0.0, rp = 0.0, ra = 0.0;
        while (f.session.epoch() - f.t0 < 1200.0) {
            f.session.advance_to(f.session.epoch() + 1.0);
            const auto& ship = f.session.body(f.session.controlled_index());
            if (insertion_s == 0.0 && f.session.autopilot().state == sim::AutopilotState::Orbit &&
                ship.vehicle().stages().size() == 1) {
                insertion_s = f.session.epoch() - f.t0;
                main_kg = ship.vehicle().main_propellant_kg();
                const auto e = orbit::elements_from_state({ship.state().position_gcrf_m, ship.state().velocity_gcrf_mps},
                                                          constants::earth_gm_m3ps2);
                rp = e.semi_major_axis_m * (1.0 - e.eccentricity);
                ra = e.semi_major_axis_m * (1.0 + e.eccentricity);
            }
            if (insertion_s > 0.0 && f.session.tower()->phase() != sim::CatchPhase::Open &&
                f.session.tower()->phase() != sim::CatchPhase::Standby && f.session.tower()->phase() != sim::CatchPhase::Closing) {
                break;
            }
        }
        double equivalent = main_kg;
        if (rp > 0.0 && rp < target_rp - 1.0e3) {  // raise the perigee at apoapsis to 170 km
            const double mu = constants::earth_gm_m3ps2;
            const double dv = std::sqrt(mu * (2.0 / ra - 2.0 / (target_rp + ra))) - std::sqrt(mu * (2.0 / ra - 2.0 / (rp + ra)));
            const double mass = f.session.body(f.session.controlled_index()).vehicle().mass_properties().mass_kg;
            equivalent = main_kg - mass * -std::expm1(-dv / 3400.0);
        }
        std::printf("[r5 probe] %-28s insertion %.0f s, %.1f x %.1f km, main %.1f t, in 170 x 190: %.1f t, booster %s\n",
                    variant.name, insertion_s, (rp - constants::wgs84_semi_major_axis_m) / 1e3,
                    (ra - constants::wgs84_semi_major_axis_m) / 1e3, main_kg / 1e3, equivalent / 1e3,
                    sim::to_string(f.session.tower()->phase()));
    }
}

namespace {

// The depot session with the Moon's orientation (lunar missions).
struct LunarDepotSession : refuelling_test::DepotSession {
    const spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession lunar{ephemeris, earth, refuelling_test::config_dir, {}, &moon};
};

}  // namespace

// Probe (r5 Task 8): the first lunar epoch from 2026-11-02 whose 26.5 deg parking plane reaches the Moon at arrival.
TEST_CASE("Probe: lunar epochs for a 26.5 deg depot plane", "[.r5_probe]") {
    test::spice_session();
    LunarDepotSession f;
    const auto m = config::load_lunar_landing_mission(refuelling_test::config_dir, "hls_lunar_llo_depot");
    const time::Epoch first = f.clock.from_utc({2026, 11, 2, 0, 0, 0.0});
    for (int k = 0; k <= 30; ++k) {
        const auto t = first + 86400.0 * k;
        const bool ok = sim::lunar_parking_state(f.lunar, m, t, 26.5 * deg).has_value();
        std::printf("[r5 probe] 2026-11-02 + %2d d: %s\n", k, ok ? "plane" : "-");
    }
}

TEST_CASE("continue_lunar_landing_mission plans like the start from the same placed vehicle",
          "[sim][lunar][refuelling]") {
    test::spice_session();
    LunarDepotSession f;
    const auto m = config::load_lunar_landing_mission(refuelling_test::config_dir, "hls_lunar_llo");
    const auto epoch = f.clock.from_utc(m.epoch_utc);
    const auto a = sim::start_lunar_landing_mission(f.lunar, refuelling_test::config_dir, m, epoch);
    INFO(a.refusal);
    REQUIRE(a.refusal.empty());
    const double tli_a = norm(f.lunar.lunar_outbound_plan()->tli.delta_v_gcrf_mps);
    sim::FlightSession b{f.ephemeris, f.earth, refuelling_test::config_dir, {}, &f.moon};
    sim::place_lunar_landing_mission(b, m, refuelling_test::config_dir, epoch);
    const auto c = sim::continue_lunar_landing_mission(b, refuelling_test::config_dir, m, epoch);
    INFO(c.refusal);
    REQUIRE(c.refusal.empty());
    CHECK(norm(b.lunar_outbound_plan()->tli.delta_v_gcrf_mps) == Catch::Approx(tli_a).epsilon(1e-9));
    CHECK(b.lunar_status() == f.lunar.lunar_status());
}

TEST_CASE("Continuation after the lunar epoch is refused", "[sim][lunar][refuelling]") {
    test::spice_session();
    LunarDepotSession f;
    const auto m = config::load_lunar_landing_mission(refuelling_test::config_dir, "hls_lunar_llo_depot");
    const auto epoch = f.clock.from_utc(m.epoch_utc);
    sim::place_lunar_landing_mission(f.lunar, m, refuelling_test::config_dir, epoch + 3600.0);
    const auto c = sim::continue_lunar_landing_mission(f.lunar, refuelling_test::config_dir, m, epoch);
    CHECK_THAT(c.refusal, Catch::Matchers::ContainsSubstring("zu spät"));
    CHECK(f.lunar.body_count() == 1u);  // nothing re-placed
}

TEST_CASE("hls_refuel starts with the depot in the lunar mission's parking plane", "[refuelling][mission][lunar]") {
    test::spice_session();
    LunarDepotSession f;
    const auto s = sim::start_mission(f.lunar, refuelling_test::config_dir, "hls_refuel_orbit", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    CHECK(s.kind == config::MissionKind::HlsRefuel);
    REQUIRE(f.lunar.iss_mission());
    REQUIRE(f.lunar.iss_mission()->continue_epoch);
    const auto lunar = config::load_lunar_landing_mission(refuelling_test::config_dir, "hls_lunar_llo_depot");
    const auto lunar_epoch = f.clock.from_utc(lunar.epoch_utc);
    CHECK(*f.lunar.iss_mission()->continue_epoch == lunar_epoch);
    // the depot's plane at the lunar epoch is the parking plane (J2 run back and forth): the depot alone, integrated
    sim::FlightSession alone{f.ephemeris, f.earth, refuelling_test::config_dir, {}, &f.moon};
    const auto& start_depot = f.lunar.body(*f.lunar.station_index());
    alone.add_station(config::top_stage_config(config::load_vehicle(refuelling_test::config_dir, "starship_depot")),
                      {start_depot.state().position_gcrf_m, start_depot.state().velocity_gcrf_mps}, f.lunar.epoch());
    while (alone.epoch() < lunar_epoch - 1.0) alone.advance_to(std::min(lunar_epoch, alone.epoch() + 3600.0));
    REQUIRE(alone.epoch() >= lunar_epoch - 1.0);
    const auto parking = sim::lunar_parking_state(alone, lunar, lunar_epoch, lunar.parking_inclination_rad);
    REQUIRE(parking);
    const auto& depot = alone.body(0).state();
    const Vec3 n_depot = math::normalized(cross(depot.position_gcrf_m, depot.velocity_gcrf_mps));
    const Vec3 n_parking = math::normalized(cross(parking->position_m, parking->velocity_mps));
    const double angle = std::acos(std::clamp(dot(n_depot, n_parking), -1.0, 1.0));
    std::printf("[r5] depot plane %.3f deg from the lunar parking plane at the lunar epoch\n", angle / deg);
    CHECK(angle < 0.3 * deg);  // [E] mean vs osculating elements, the J2 rates vs the integrated orbit
}

// r5 (found in hls_refuel): the filled HLS (1,585 t main) leaves the nearly empty depot (84 t). Its cold gas gives
// ~2.5 mm/s^2, the Coriolis term of the LVLH frame ~2 n v ~2 mm/s^2 per m/s: a burn that holds a fixed LVLH velocity
// never converged (157 t of gas in 11.5 h). The burn's target is the free CW drift after the impulse.
// r5 (found in hls_refuel_orbit: the light HLS bounced off the depot's drogues): after a failed capture the proximity
// flight waits for a go at Waypoint 2. An autonomous mission resumes up to max_capture_retries times, then ends; a
// retreat the pilot commanded waits for the pilot (review r5 I4), as with 0 retries (the ISS default).
TEST_CASE("A commanded retreat in an autonomous mission waits for the pilot",
          "[refuelling][iss_mission][proximity][docking]") {
    test::spice_session();
    const auto with_sequence = [](refuelling_test::TankerAtPort& f, int retries) {
        sim::IssMissionSequence q;
        q.station_port = "transfer_dock";
        q.max_capture_retries = retries;
        f.session.set_iss_mission(q);
        gnc::ProximityParameters p;
        p.start_waypoint = 2;
        REQUIRE(f.session.start_proximity("transfer_dock", p));
        f.session.mark_iss_leg_for_test(IssMissionLeg::Proximity);
    };
    const auto proximity_marks = [](const sim::FlightSession& s) {
        const auto& m = s.iss_mission_marks();
        return std::count_if(m.begin(), m.end(), [](const auto& x) { return x.leg == IssMissionLeg::Proximity; });
    };
    // (a failed capture's resume: the gnc test 'A failed capture retreats to Waypoint 2 and waits for resume' and
    // the hls_refuel_orbit campaign, where the light HLS bounces off the drogues once)
    {  // a commanded retreat with retries left: the pilot's decision stands
        refuelling_test::TankerAtPort f({20.0, 0.0, 0.0});
        with_sequence(f, 2);
        f.session.advance_to(f.session.epoch() + 60.0);
        REQUIRE(f.session.proximity_retreat());
        f.fly([&] { return f.session.proximity_waiting(); }, 4.0 * 3600.0, 5.0);
        REQUIRE(f.session.proximity_waiting());
        f.session.advance_to(f.session.epoch() + 10.0);
        CHECK(f.session.proximity_waiting());
        CHECK(f.session.iss_mission_leg() == IssMissionLeg::Proximity);
    }
}

// Review r5 I1: a deorbit called off after it was planned (the main tanks no longer hold the burn) ends the mission
// with the reason instead of waiting in Return forever.
TEST_CASE("A deorbit called off after the plan ends the mission", "[refuelling][iss_mission][deorbit]") {
    test::spice_session();
    DockedAtDepot f;
    f.session.set_iss_mission(f.tanker_sequence());
    f.fly([&] { return f.leg() == IssMissionLeg::Return || f.leg() == IssMissionLeg::Ended; }, 6.0 * 3600.0, 10.0);
    REQUIRE(f.leg() == IssMissionLeg::Return);
    // the tanks drained (a leak, or more boil-off than planned): the next re-plan calls the burn off
    const size_t ship = f.session.controlled_index();
    f.session.set_tank_mass(ship, 0, "lox_main", 400.0);
    f.session.set_tank_mass(ship, 0, "ch4_main", 100.0);
    f.fly([&] { return f.leg() != IssMissionLeg::Return; }, 9.0 * 86400.0, 600.0);
    std::printf("[r5] called-off deorbit: %s (abort: %s)\n", f.session.iss_mission_status().c_str(),
                f.session.burn_abort_reason().c_str());
    INFO(f.session.iss_mission_status());
    CHECK(f.leg() == IssMissionLeg::Ended);
    // at once, with the call-off's reason (before: days later "Schiff verloren")
    CHECK_THAT(f.session.iss_mission_status(), Catch::Matchers::StartsWith("Mission beendet: Zu wenig Treibstoff"));
    CHECK_FALSE(f.session.body(ship).caught());
}

TEST_CASE("The full HLS departs from the nearly empty depot", "[refuelling][docking][departure]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0}, {}, refuelling_test::ship_with("starship_hls", 1585.0e3), 84.0e3);
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    REQUIRE(f.session.undock());
    const double m0 = f.session.body(0).vehicle().main_propellant_kg();
    const double s = f.fly([&] { return f.session.departure_phase() == gnc::DeparturePhase::Complete; },
                           3.0 * 3600.0, 60.0);
    const double gas_kg = m0 - f.session.body(0).vehicle().main_propellant_kg();
    std::printf("[r5] full HLS departure: %.0f min, gas %.2f t\n", s / 60.0, gas_kg / 1e3);
    CHECK(f.session.departure_phase() == gnc::DeparturePhase::Complete);
    CHECK(gas_kg < 5.0e3);  // [E] the 600 t depot case: 2.4 t
}

// Probe (r5 Task 8 diagnostics): where the hls_refuel_orbit chain spends its wall time.
TEST_CASE("Probe: hls_refuel_orbit timeline", "[.r5_probe]") {
    test::spice_session();
    LunarDepotSession f;
    const auto s = sim::start_mission(f.lunar, refuelling_test::config_dir, "hls_refuel_orbit", f.t0, f.clock);
    REQUIRE(s.refusal.empty());
    const time::Epoch start = f.lunar.epoch();
    const auto wall0 = std::chrono::steady_clock::now();
    auto last = wall0;
    while (f.lunar.epoch() - start < 8.0 * 86400.0 && f.lunar.iss_mission_leg() != IssMissionLeg::Complete &&
           f.lunar.iss_mission_leg() != IssMissionLeg::Ended) {
        f.lunar.advance_to(f.lunar.epoch() + 600.0);
        const auto now = std::chrono::steady_clock::now();
        const double step_s = std::chrono::duration<double>(now - last).count();
        last = now;
        const double wall_s = std::chrono::duration<double>(now - wall0).count();
        if (step_s > 2.0) {
            std::printf("[probe] sim %7.2f h  wall %6.0f s (+%5.1f)  leg %-22s autopilot %-12s %s\n",
                        (f.lunar.epoch() - start) / 3600.0, wall_s, step_s, sim::to_string(f.lunar.iss_mission_leg()),
                        sim::to_string(f.lunar.autopilot().state), f.lunar.iss_mission_status().c_str());
            std::fflush(stdout);
        }
        if (wall_s > 900.0 || f.lunar.iss_mission_status().find("Freigabe") != std::string::npos) break;
    }
    std::printf("[probe] end: sim %.2f h, leg %s, %s\n", (f.lunar.epoch() - start) / 3600.0,
                sim::to_string(f.lunar.iss_mission_leg()), f.lunar.iss_mission_status().c_str());
}

TEST_CASE("hls_refuel_orbit: docked, filled, departed, the lunar mission continues with the real tank load",
          "[refuelling][mission][lunar][slow]") {
    test::spice_session();
    LunarDepotSession f;
    const auto s = sim::start_mission(f.lunar, refuelling_test::config_dir, "hls_refuel_orbit", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    const time::Epoch start = f.lunar.epoch();
    while (f.lunar.epoch() - start < 8.0 * 86400.0 && f.lunar.iss_mission_leg() != IssMissionLeg::Complete &&
           f.lunar.iss_mission_leg() != IssMissionLeg::Ended) {
        f.lunar.advance_to(f.lunar.epoch() + 600.0);
    }
    INFO(f.lunar.iss_mission_status());
    REQUIRE(f.lunar.iss_mission_leg() == IssMissionLeg::Complete);
    REQUIRE(f.lunar.lunar_outbound_plan());
    const auto& hls = f.lunar.body(f.lunar.controlled_index()).vehicle();
    const auto* transfer = refuelling_test::mark(f.lunar, IssMissionLeg::Transfer);
    const auto* departure = refuelling_test::mark(f.lunar, IssMissionLeg::Departure);
    REQUIRE(transfer);
    REQUIRE(departure);
    std::printf("[r5] hls_refuel_orbit: HLS main at docking %.1f t, after the fill %.1f t, at the lunar start %.1f t, "
                "depot left %.1f t, TLI %.0f m/s\n",
                transfer->chaser_main_propellant_kg / 1e3, departure->chaser_main_propellant_kg / 1e3,
                hls.main_propellant_kg() / 1e3,
                f.lunar.body(*f.lunar.station_index()).vehicle().main_propellant_kg() / 1e3,
                norm(f.lunar.lunar_outbound_plan()->tli.delta_v_gcrf_mps));
    CHECK(departure->chaser_main_propellant_kg > transfer->chaser_main_propellant_kg);
}

TEST_CASE("hls_refuel: pad -> docked -> filled -> lunar mission planned", "[refuelling][mission][lunar][ascent][slow]") {
    test::spice_session();
    LunarDepotSession f;
    const auto s = sim::start_mission(f.lunar, refuelling_test::config_dir, "hls_refuel", f.t0, f.clock);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    while (f.lunar.epoch() - s.epoch < 1200.0 && f.lunar.tower()->phase() != sim::CatchPhase::Caught) {
        f.lunar.advance_to(f.lunar.epoch() + 1.0);
    }
    const time::Epoch start = f.lunar.epoch();
    while (f.lunar.epoch() - start < 10.0 * 86400.0 && f.lunar.iss_mission_leg() != IssMissionLeg::Complete &&
           f.lunar.iss_mission_leg() != IssMissionLeg::Ended) {
        f.lunar.advance_to(f.lunar.epoch() + 600.0);
    }
    INFO(f.lunar.iss_mission_status());
    INFO("breakup: " << f.lunar.body(f.lunar.controlled_index()).breakup_reason() << ", tower "
                     << sim::to_string(f.lunar.tower()->phase()) << ", liftoff +"
                     << (f.lunar.epoch() - s.epoch) / 3600.0 << " h");
    for (const auto& m : f.lunar.iss_mission_marks()) {
        std::printf("[r5] hls_refuel %-24s +%9.0f s  main %.1f t  station %.1f t\n", sim::to_string(m.leg),
                    m.epoch - s.epoch, m.chaser_main_propellant_kg / 1e3, m.station_main_propellant_kg / 1e3);
    }
    REQUIRE(f.lunar.iss_mission_leg() == IssMissionLeg::Complete);
    REQUIRE(f.lunar.lunar_outbound_plan());
    const auto* raise = refuelling_test::mark(f.lunar, IssMissionLeg::Raise);
    const auto* rv = refuelling_test::mark(f.lunar, IssMissionLeg::Rendezvous);
    REQUIRE(rv);
    const auto* docked = refuelling_test::mark(f.lunar, IssMissionLeg::Docked);
    REQUIRE(docked);
    refuelling_test::check_reference(docked->chaser_main_propellant_kg,
                                     config::load_refuelling_reference(refuelling_test::config_dir).hls.main_at_docking_kg,
                                     "HLS main at docking");
    std::printf("[r5] hls_refuel: SECO %.1f s, HLS residue at insertion %.1f t, after the raise %.1f t, main at the "
                "lunar start %.1f t\n",
                raise ? raise->epoch - s.epoch : 0.0, raise ? raise->chaser_main_propellant_kg / 1e3 : 0.0,
                rv->chaser_main_propellant_kg / 1e3,
                f.lunar.body(f.lunar.controlled_index()).vehicle().main_propellant_kg() / 1e3);
}
