// Refuelling f3: settling before an in-orbit ignition (settling state, PMD tanks, gas ingestion, settling push).

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "refuelling_test_support.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/boil_off.hpp"
#include "starsim/vehicle/propellant_transfer.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// The tanker's ship stage alone with `lox_kg` / `ch4_kg` in its main tanks.
std::unique_ptr<vehicle::Vehicle> tanker(double lox_kg = 50.0e3, double ch4_kg = 15.0e3) {
    auto v = std::make_unique<vehicle::Vehicle>(config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker")),
                                                config_dir);
    v->set_tank_mass(0, "lox_main", lox_kg);
    v->set_tank_mass(0, "ch4_main", ch4_kg);
    return v;
}

// 200 km, body z radial, no rotation (as test_refuelling_transfer.cpp); `tide` false: far from any body.
vehicle::LiquidEnvironment orbit_liquid(const math::Vec3& specific_force_body_mps2, bool tide = true) {
    const double r = tide ? constants::wgs84_semi_major_axis_m + 200.0e3 : 1.0e15;
    return {specific_force_body_mps2, {}, {0.0, 0.0, r}, math::Quat{}};
}

// Every burn engine group of stage 0 commanded to run.
vehicle::VehicleCommand light_all(const vehicle::Vehicle& v) {
    vehicle::VehicleCommand c;
    for (const auto& e : v.stages()[0].engines) {
        if (!e.auxiliary()) c.groups.push_back({e.group_id, true, 1.0});
    }
    return c;
}

// A tanker coasting from the apoapsis (or periapsis) of a `periapsis_height_m` x 190 km orbit, no depot and no mission.
struct TankerInOrbit : refuelling_test::DepotSession {
    explicit TankerInOrbit(double main_kg, double periapsis_height_m = 120e3, double apoapsis_height_m = 190e3,
                           double ahead_deg = 0.0, bool at_periapsis = false) {
        const double re = constants::wgs84_semi_major_axis_m;
        const auto along = refuelling_test::depot_state(earth, t0, ahead_deg * constants::deg_to_rad);
        const double rp = re + periapsis_height_m, ra = re + apoapsis_height_m;
        const double r = at_periapsis ? rp : ra;
        const double v = std::sqrt(constants::earth_gm_m3ps2 * (2.0 / r - 2.0 / (rp + ra)));
        const math::StateVector s{r * math::normalized(along.position_m), v * math::normalized(along.velocity_mps)};
        session.place_in_orbit(refuelling_test::tanker_config(main_kg), s, t0);
    }
    const vehicle::Vehicle& vehicle() const { return session.body(0).vehicle(); }
    bool any_failed() const {
        for (const auto& e : vehicle().stages()[0].engines) {
            if (e.failed) return true;
        }
        return false;
    }
};

}  // namespace

TEST_CASE("Settling: a +x push settles the main tanks in sqrt(2L/a)", "[ignition_settling][vehicle]") {
    auto v = tanker();
    REQUIRE(v->needs_settling(0));
    v->stages()[0].settled = 0.0;
    const double a = 0.010;
    const auto liquid = orbit_liquid({a, 0.0, 0.0});
    const double t = vehicle::settling_time_s(v->settling_travel_m(0), a);
    CHECK(t == Catch::Approx(std::sqrt(2.0 * v->settling_travel_m(0) / a)));
    v->update_settling(liquid, 0.5 * t);
    CHECK(v->stages()[0].settled == Catch::Approx(0.5).margin(0.02));  // the tide adds < 2 %
    CHECK_FALSE(v->stage_settled(0));
    v->update_settling(liquid, 0.6 * t);
    CHECK(v->stage_settled(0));
}

TEST_CASE("Settling: without thrust the liquid leaves the outlets", "[ignition_settling][vehicle]") {
    auto v = tanker();
    const auto drag_nose_first = orbit_liquid({-2.0e-5, 0.0, 0.0});  // drag pushes the liquid forward
    v->update_settling(drag_nose_first, 60.0);
    CHECK(v->stages()[0].settled < 1.0);
    v->update_settling(drag_nose_first, 1.0e5);
    CHECK(v->stages()[0].settled == 0.0);
    v->stages()[0].settled = 0.4;
    v->update_settling(orbit_liquid({}, false), 1.0e5);  // no acceleration: the liquid stays where it is
    CHECK(v->stages()[0].settled == Catch::Approx(0.4));
}

TEST_CASE("Settling: PMD tanks are always covered, header feed needs no settling", "[ignition_settling][vehicle]") {
    auto v = tanker();
    for (const auto& t : v->stages()[0].tanks) {
        INFO(t.config.id);
        CHECK(t.config.propellant_management_device == (t.config.role == config::TankRole::Header));
    }
    v->select_feed(0, vehicle::FeedSource::Header);
    CHECK_FALSE(v->needs_settling(0));
    v->stages()[0].settled = 0.0;
    CHECK(v->stage_settled(0));
    v->select_feed(0, vehicle::FeedSource::Main);
    CHECK_FALSE(v->stage_settled(0));
    // main tanks dry: nothing to settle
    auto dry = tanker(0.0, 0.0);
    CHECK_FALSE(dry->needs_settling(0));
}

TEST_CASE("Settling: the travel is the ullage of the longest main tank, at least 1 m", "[ignition_settling][vehicle]") {
    auto v = tanker();
    double longest = 0.0;
    for (const auto& t : v->stages()[0].tanks) {
        if (t.config.role != config::TankRole::Main || !(t.mass_kg > 0.0)) continue;
        longest = std::max(longest, t.length_m * (1.0 - t.mass_kg / t.config.capacity_kg));
    }
    CHECK(longest > 10.0);
    CHECK(v->settling_travel_m(0) == Catch::Approx(longest));
    for (const auto& t : std::vector<vehicle::Tank>(v->stages()[0].tanks)) {
        if (t.config.role == config::TankRole::Main) v->set_tank_mass(0, t.config.id, t.config.capacity_kg);
    }
    CHECK(v->settling_travel_m(0) == Catch::Approx(vehicle::settling_min_travel_m));
}

TEST_CASE("Vehicle configs flag the PMD tanks", "[ignition_settling][config]") {
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    bool found = false;
    for (const auto& s : dragon.stages) {
        for (const auto& t : s.tanks) {
            if (t.id == "mmh_nto") {
                found = true;
                CHECK(t.propellant_management_device);
            }
        }
    }
    CHECK(found);
    for (const std::string id : {"starship_depot", "starship_block3", "starship_hls"}) {
        INFO(id);
        const auto config = config::load_vehicle(config_dir, id);  // kept alive across the loop
        for (const auto& t : config.stages.back().tanks) {
            CHECK(t.propellant_management_device == (t.role == config::TankRole::Header));
        }
    }
}

TEST_CASE("Ignition check: unsettled main feed fails the engines, header feed lights", "[ignition_settling][vehicle]") {
    auto v = tanker();
    v->set_ignition_check(true);
    v->stages()[0].settled = 0.0;
    v->update_engines(0.005, light_all(*v), 0.0);
    CHECK(v->gas_ingestion());
    bool any_lit = false, all_failed = true;
    for (const auto& e : v->stages()[0].engines) {
        if (e.auxiliary()) continue;
        any_lit = any_lit || e.mode != vehicle::EngineMode::Off;
        all_failed = all_failed && e.failed;
    }
    CHECK_FALSE(any_lit);
    CHECK(all_failed);
    v->clear_gas_ingestion();
    CHECK_FALSE(v->gas_ingestion());

    auto h = tanker();
    h->set_ignition_check(true);
    h->stages()[0].settled = 0.0;
    h->select_feed(0, vehicle::FeedSource::Header);
    h->update_engines(0.005, light_all(*h), 0.0);
    CHECK_FALSE(h->gas_ingestion());

    auto off = tanker();  // check inactive (not in orbit, or the fidelity flag off)
    off->stages()[0].settled = 0.0;
    off->update_engines(0.005, light_all(*off), 0.0);
    CHECK_FALSE(off->gas_ingestion());
}

TEST_CASE("Unsettled ignition in orbit: the engines ingest gas and fail", "[ignition_settling][sim]") {
    test::spice_session();
    TankerInOrbit f(100e3);
    CHECK(f.session.body(0).in_orbit());
    f.session.advance_to(f.session.epoch() + 2.0 * 3600.0);  // the liquid leaves the outlets
    REQUIRE_FALSE(f.vehicle().stage_settled(0));
    f.session.ignite();
    f.session.advance_to(f.session.epoch() + 1.0);
    CHECK(f.any_failed());
    CHECK_FALSE(f.vehicle().any_engine_producing_thrust());
    CHECK_THAT(f.session.burn_abort_reason(), Catch::Matchers::StartsWith("Gasansaugung"));
}

TEST_CASE("Settled ignition in orbit lights normally; the flag off never ingests", "[ignition_settling][sim]") {
    test::spice_session();
    TankerInOrbit f(100e3);
    REQUIRE(f.vehicle().stage_settled(0));  // just placed: settled
    f.session.ignite();
    f.session.advance_to(f.session.epoch() + 5.0);
    CHECK_FALSE(f.any_failed());
    CHECK(f.vehicle().any_engine_producing_thrust());

    TankerInOrbit g(100e3);
    g.session.set_ignition_settling(false);
    g.session.advance_to(g.session.epoch() + 2.0 * 3600.0);
    g.session.ignite();
    g.session.advance_to(g.session.epoch() + 5.0);
    CHECK_FALSE(g.any_failed());
    CHECK(g.vehicle().any_engine_producing_thrust());
}

TEST_CASE("Suborbital ignitions are not checked", "[ignition_settling][sim]") {
    test::spice_session();
    // a 1,500 km apoapsis arc that falls back (periapsis 2,000 km below the surface)
    TankerInOrbit f(100e3, -2000e3, 1500e3);
    CHECK_FALSE(f.session.body(0).in_orbit());
    f.session.advance_to(f.session.epoch() + 1200.0);
    REQUIRE_FALSE(f.vehicle().stage_settled(0));
    f.session.ignite();
    f.session.advance_to(f.session.epoch() + 5.0);
    CHECK_FALSE(f.any_failed());
    CHECK(f.vehicle().any_engine_producing_thrust());
}
TEST_CASE("BurnExecutor credit lowers the remaining delta-v without starting the burn", "[ignition_settling][burn_node]") {
    const gnc::BurnNode node{time::Epoch::from_tdb_seconds_past_j2000(8.4e8), {10.0, 0.0, 0.0}};
    const vehicle::FlightBody body(tanker(), vehicle::RigidBodyState{});
    gnc::BurnExecutor ex(node);
    ex.credit({0.8, 0.0, 0.0});
    const auto st = ex.update(body, node.epoch - 100.0, 0.005, 2.0e6, 650.0, 1.0);
    CHECK(st.remaining_dv_mps == Catch::Approx(9.2));
    CHECK_FALSE(st.ignite);
}

namespace {

// Advances in steps of at most `step_s` to `target`; true when the settling push was seen at a step's end.
bool run_to(sim::FlightSession& s, time::Epoch target, double step_s) {
    bool seen = false;
    while (s.epoch() < target - 0.01) {  // advance_to stops a fraction of a tick short
        s.advance_to(std::min(s.epoch() + step_s, target));
        seen = seen || s.settling();
    }
    return seen;
}

// Prograde along the current velocity, `ahead_s` from now.
gnc::BurnNode prograde(const sim::FlightSession& s, double ahead_s, double dv_mps) {
    return {s.epoch() + ahead_s, math::normalized(s.body(0).state().velocity_gcrf_mps) * dv_mps};
}

}  // namespace

TEST_CASE("The autopilot settles before a raise burn and the burn flies", "[ignition_settling][refuelling][iss_mission]") {
    test::spice_session();
    TankerInOrbit f(100e3, 120e3, 190e3, -10.0, true);  // as the f1 below-candidate cases
    sim::IssMissionSequence q;
    q.from_launch = true;
    q.station_port = "transfer_dock";
    q.phasing.mission_limit_s = 48.0 * 3600.0;
    q.raise_to = mission::PhasingOrbit{constants::wgs84_semi_major_axis_m + 170e3,
                                       constants::wgs84_semi_major_axis_m + 190e3};
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    f.session.advance_to(f.session.epoch() + 2.0 * 3600.0);  // the liquid leaves the outlets
    REQUIRE_FALSE(f.vehicle().stage_settled(0));
    const double lead = f.session.settling_lead_s();
    CHECK(lead > 30.0);
    CHECK(lead < 300.0);
    const double gas_estimate = f.session.settling_gas_kg();
    const double supply0 = f.vehicle().rcs_supply_kg();
    f.session.set_iss_mission(q);
    bool seen = false;
    const time::Epoch end = f.session.epoch() + 4.0 * 3600.0;
    while (f.session.epoch() < end && f.session.iss_mission_leg() != sim::IssMissionLeg::Ended &&
           f.session.iss_mission_leg() != sim::IssMissionLeg::Rendezvous) {
        f.session.advance_to(f.session.epoch() + 10.0);
        seen = seen || f.session.settling();
    }
    INFO(f.session.iss_mission_status());
    CHECK(seen);
    CHECK(f.session.iss_mission_leg() == sim::IssMissionLeg::Rendezvous);
    CHECK_FALSE(f.any_failed());
    CHECK(f.session.burn_abort_reason().empty());
    CHECK(gas_estimate > 0.0);
    CHECK(supply0 - f.vehicle().rcs_supply_kg() > gas_estimate);  // the raise burn's propellant on top of the gas
}

// The final states are not compared: the 10 s and 600 s runs end ~8 m/s apart with the flag off too (pre-existing).
TEST_CASE("Settling: the coast stops before the lead in 10 s and in 600 s steps", "[ignition_settling][sim]") {
    test::spice_session();
    TankerInOrbit fine(100e3), coarse(100e3);
    for (auto* f : {&fine, &coarse}) {
        run_to(f->session, f->t0 + 2.0 * 3600.0, 600.0);  // both at the same epoch
        REQUIRE_FALSE(f->vehicle().stage_settled(0));
        REQUIRE(f->session.execute_burn(prograde(f->session, 7200.0, 100.0)));
    }
    const time::Epoch end = fine.t0 + 2.0 * 3600.0 + 7200.0 + 600.0;
    CHECK(run_to(fine.session, end, 10.0));
    run_to(coarse.session, end, 600.0);
    for (auto* f : {&fine, &coarse}) {
        CHECK_FALSE(f->any_failed());
        CHECK(f->session.burn_abort_reason().empty());
        CHECK_FALSE(f->session.settling());
    }
}

TEST_CASE("Settling: a lead longer than the time left holds the ignition", "[ignition_settling][sim]") {
    test::spice_session();
    TankerInOrbit f(100e3);
    f.session.advance_to(f.session.epoch() + 2.0 * 3600.0);
    REQUIRE_FALSE(f.vehicle().stage_settled(0));
    const double lead = f.session.settling_lead_s();
    const auto node = prograde(f.session, 20.0, 100.0);
    REQUIRE(f.session.execute_burn(node));
    REQUIRE(lead > 40.0);
    std::optional<time::Epoch> lit;
    while (f.session.epoch() < node.epoch + 600.0) {
        f.session.advance_to(f.session.epoch() + 1.0);
        if (!lit && f.vehicle().any_engine_producing_thrust()) lit = f.session.epoch();
    }
    CHECK_FALSE(f.any_failed());
    CHECK(f.session.burn_abort_reason().empty());
    REQUIRE(lit);
    CHECK(*lit - node.epoch > 0.0);  // held past the planned ignition
    CHECK(*lit - node.epoch < sim::FlightSession::settling_hold_factor * lead);
}

TEST_CASE("Manual settle: the push runs until switched off and settles the tanks", "[ignition_settling][sim]") {
    test::spice_session();
    TankerInOrbit f(100e3);
    f.session.advance_to(f.session.epoch() + 2.0 * 3600.0);
    REQUIRE_FALSE(f.vehicle().stage_settled(0));
    const double lead = f.session.settling_lead_s();
    CHECK(f.session.settle(true));
    CHECK(f.session.settling());
    run_to(f.session, f.session.epoch() + lead, 10.0);
    CHECK(f.vehicle().stage_settled(0));
    CHECK(f.session.settle(false));
    CHECK_FALSE(f.session.settling());
}
