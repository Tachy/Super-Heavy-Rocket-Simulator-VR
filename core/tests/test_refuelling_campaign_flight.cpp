// Refuelling f2: the flown campaign - session building blocks, the station reboost, the driver.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "refuelling_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/refuelling_mission.hpp"
#include "test_support.hpp"

using namespace starsim;
using sim::IssMissionLeg;

namespace {

constexpr const char* tanker_stack = "starship_tanker";  // tanker_ascent's vehicle
constexpr const char* tanker_site = "starbase_olp1";     // tanker_ascent's site

sim::LaunchSite starbase() {
    return sim::launch_site_from_config(config::load_launch_site(refuelling_test::config_dir, tanker_site));
}

struct CountingSupervisor : sim::SessionSupervisor {
    std::optional<time::Epoch> wake;
    std::vector<time::Epoch> updates;
    void update(sim::FlightSession& s) override { updates.push_back(s.epoch()); }
    std::optional<time::Epoch> wake_epoch() const override { return wake; }
};

}  // namespace

TEST_CASE("keep_station: a launch keeps the station and removes every other body", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    f.session.advance_to(f.t0 + 600.0);
    const auto before = f.session.body(0).state();
    const double fill = f.session.body(0).vehicle().main_propellant_kg();
    f.session.launch_from_pad(tanker_stack, starbase(), 0.0, f.session.epoch(), true);
    REQUIRE(f.session.body_count() == 2u);
    CHECK(f.session.controlled_index() == 0u);
    REQUIRE(f.session.station_index() == std::optional<size_t>{1});
    CHECK(norm(f.session.body(1).state().position_gcrf_m - before.position_gcrf_m) < 1e-6);
    CHECK(f.session.body(1).vehicle().main_propellant_kg() == Catch::Approx(fill));
    CHECK(f.session.body(1).ideal_attitude());
    // a second launch with the station kept replaces the stack, not the station
    f.session.launch_from_pad(tanker_stack, starbase(), 0.0, f.session.epoch(), true);
    CHECK(f.session.body_count() == 2u);
    CHECK(f.session.station_index() == std::optional<size_t>{1});
}

TEST_CASE("keep_station refuses without a station or off the session epoch", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    CHECK_THROWS_AS(f.session.launch_from_pad(tanker_stack, starbase(), 0.0, f.t0, true), std::logic_error);
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    CHECK_THROWS_AS(f.session.launch_from_pad(tanker_stack, starbase(), 0.0, f.t0 + 60.0, true), std::logic_error);
    CHECK(f.session.body_count() == 1u);  // nothing dropped
    CHECK(f.session.station_index() == std::optional<size_t>{0});
}

TEST_CASE("keep_station on an orbit placement", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    f.session.place_in_orbit(refuelling_test::tanker_config(50.0e3), refuelling_test::depot_state(f.earth, f.t0, -0.1),
                             f.t0, true);
    REQUIRE(f.session.body_count() == 2u);
    CHECK(f.session.controlled_index() == 0u);
    CHECK(f.session.station_index() == std::optional<size_t>{1});
}

TEST_CASE("keep_only_station removes the chaser and the sequence", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, -30.0});  // body 0 tanker, body 1 depot
    sim::IssMissionSequence q;
    q.station_port = "transfer_dock";
    f.session.set_iss_mission(q);
    f.session.keep_only_station();
    REQUIRE(f.session.body_count() == 1u);
    CHECK(f.session.station_index() == std::optional<size_t>{0});
    CHECK(f.session.controlled_index() == 0u);
    CHECK_FALSE(f.session.iss_mission().has_value());
    f.session.advance_to(f.session.epoch() + 3600.0);  // the station alone coasts on
    sim::FlightSession empty(f.ephemeris, f.earth, refuelling_test::config_dir);
    CHECK_THROWS_AS(empty.keep_only_station(), std::logic_error);
}

TEST_CASE("place_vehicle places a copy with its tank load", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(42.0e3), f.depot, f.t0);
    const auto& body = f.session.body(0);
    sim::FlightSession side(f.ephemeris, f.earth, refuelling_test::config_dir);
    side.place_vehicle(body.vehicle().copy_upper(0), body.state(), f.t0);
    REQUIRE(side.body_count() == 1u);
    CHECK(side.body(0).vehicle().main_propellant_kg() == Catch::Approx(42.0e3).epsilon(1e-6));
    CHECK(side.epoch() == f.t0);
}

TEST_CASE("Supervisor: a time-warp call stops at its wake epoch", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    auto sup = std::make_shared<CountingSupervisor>();
    sup->wake = f.t0 + 1000.3;
    f.session.set_supervisor(sup);
    f.session.advance_to(f.t0 + 5.0 * 86400.0);
    const auto hit = std::find_if(sup->updates.begin(), sup->updates.end(),
                                  [&](time::Epoch e) { return e >= *sup->wake - sim::FlightSession::tick_s; });
    REQUIRE(hit != sup->updates.end());
    CHECK(std::abs(*hit - *sup->wake) <= sim::FlightSession::tick_s);
    CHECK(f.session.epoch() - f.t0 == Catch::Approx(5.0 * 86400.0).margin(1.0));
}

TEST_CASE("Supervisor: removed by clear_bodies and by any mission start", "[campaign_flown][refuelling]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    f.session.set_supervisor(std::make_shared<CountingSupervisor>());
    f.session.clear_bodies();
    CHECK(f.session.supervisor() == nullptr);
    f.session.set_supervisor(std::make_shared<CountingSupervisor>());
    sim::start_mission(f.session, refuelling_test::config_dir, "tanker_flight_orbit", f.t0, f.clock);
    CHECK(f.session.supervisor() == nullptr);
}

namespace {

double mean_height_km(const sim::FlightSession& s) { return s.station_mean_height_m() / 1000.0; }

// Advances in 60 s calls until the reboost ends or `limit_s`.
void fly_reboost(sim::FlightSession& s, double limit_s = 4.0 * 3600.0) {
    const auto t0 = s.epoch();
    while (s.station_reboost_active() && s.epoch() - t0 < limit_s) s.advance_to(s.epoch() + 60.0);
}

const std::vector<std::string> belly = {"sea_level_3"};

}  // namespace

TEST_CASE("Reboost: a low depot raises to 200 km in two burns and is the station again",
          "[campaign_flown][refuelling][reboost]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.add_station(refuelling_test::depot_config(400.0e3),
                          refuelling_test::depot_state(f.earth, f.t0, 0.0, 193.0e3), f.t0);
    const double fill0 = f.session.body(0).vehicle().main_propellant_kg();
    const double mass0 = f.session.body(0).vehicle().mass_properties().mass_kg;
    const double ve_raptor = sim::phasing_exhaust_velocity_mps(f.session.body(0).vehicle());
    const double ve_gas = refuelling_test::depot_config(0.0).stages.back().rcs.isp_s * constants::standard_gravity_mps2;
    const double start_km = mean_height_km(f.session);  // the energy mean (the synthetic 193 km state reads ~191 km)
    REQUIRE(f.session.start_station_reboost(200.0e3, belly));
    CHECK_FALSE(f.session.station_index().has_value());  // controlled for the burns
    fly_reboost(f.session);
    REQUIRE(f.session.last_reboost());
    const auto& r = *f.session.last_reboost();
    INFO(r.reason);
    CHECK(r.completed);
    CHECK(r.burns == 2);
    REQUIRE(f.session.station_index() == std::optional<size_t>{0});
    CHECK(f.session.body(0).ideal_attitude());
    CHECK(mean_height_km(f.session) == Catch::Approx(200.0).margin(1.0));  // [E]
    // propellant against the ideal Hohmann pair (vis-viva) at the exhaust velocity of what flew [E]: at 636 t the
    // belly Raptor's transients exceed 60 % of a ~2 m/s burn, so the cold gas flies it (f2 finding)
    const double mu = constants::earth_gm_m3ps2, re = constants::wgs84_semi_major_axis_m;
    const double r1 = re + start_km * 1000.0, r2 = re + 200.0e3, at = 0.5 * (r1 + r2);
    const double dv = std::sqrt(mu * (2 / r1 - 1 / at)) - std::sqrt(mu / r1) + std::sqrt(mu / r2) -
                      std::sqrt(mu * (2 / r2 - 1 / at));
    const double ideal_kg = mass0 * (1.0 - std::exp(-dv / (r.cold_gas ? ve_gas : ve_raptor)));
    std::printf("reboost: %d burns, cold gas %d, %.1f kg (ideal %.1f kg), %.2f h, mean height %.2f km\n",
                r.burns, r.cold_gas ? 1 : 0, r.propellant_kg, ideal_kg, (r.end - r.start) / 3600.0,
                mean_height_km(f.session));
    // the reboost's propellant plus the session's boil-off is what the tanks lost
    CHECK(r.propellant_kg <= fill0 - f.session.body(0).vehicle().main_propellant_kg() + 1.0);
    CHECK(r.propellant_kg > ideal_kg);
    CHECK(r.propellant_kg < 1.5 * ideal_kg + 200.0);  // + the turns and the settling gas [E]
}

TEST_CASE("Reboost refusals", "[campaign_flown][refuelling][reboost]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    CHECK_FALSE(f.session.start_station_reboost(200.0e3, belly));  // no station
    CHECK_FALSE(f.session.reboost_status().empty());
    f.session.add_station(refuelling_test::depot_config(400.0e3),
                          refuelling_test::depot_state(f.earth, f.t0, 0.0, 201.0e3), f.t0);
    CHECK_FALSE(f.session.start_station_reboost(200.0e3, belly));  // already above
    CHECK_FALSE(f.session.start_station_reboost(210.0e3, {}));     // no engine groups
    CHECK(f.session.station_index().has_value());
    refuelling_test::TankerAtPort g({0.0, 0.0, -30.0});
    CHECK_FALSE(g.session.start_station_reboost(260.0e3, belly));  // a chaser is present
}

TEST_CASE("Reboost: a refused burn leaves the depot a station", "[campaign_flown][refuelling][reboost]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    // a full depot, no such engine group and 17 m/s: the cold gas would take hours - the burn is refused after the
    // release (f2 probe: the depot's cold gas feeds from the full header tanks, so an empty main tank does not stop it)
    f.session.add_station(refuelling_test::depot_config(1500.0e3),
                          refuelling_test::depot_state(f.earth, f.t0, 0.0, 200.0e3), f.t0);
    REQUIRE(f.session.start_station_reboost(260.0e3, {"no_such_group"}));
    fly_reboost(f.session);
    CHECK_FALSE(f.session.station_reboost_active());
    REQUIRE(f.session.last_reboost());
    INFO(f.session.last_reboost()->reason);
    CHECK_FALSE(f.session.last_reboost()->completed);
    CHECK_FALSE(f.session.last_reboost()->reason.empty());
    REQUIRE(f.session.station_index() == std::optional<size_t>{0});
    CHECK(f.session.body(0).ideal_attitude());
    f.session.advance_to(f.session.epoch() + 3600.0);  // a station again: it coasts
}

TEST_CASE("A campaign tanker launches beside the kept depot and flies its chain to the raise",
          "[campaign_flown][refuelling][ascent]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto& dir = refuelling_test::config_dir;
    const auto m = config::load_refuelling_flight_mission(dir, "tanker_flight");
    const auto ascent = config::load_ascent_mission(dir, m.ascent_id);
    const auto site = config::load_launch_site(dir, ascent.site_id);
    f.session.add_station(refuelling_test::depot_config(300.0e3), f.depot, f.t0);
    const vehicle::Vehicle chaser(config::top_stage_config(config::load_vehicle(dir, ascent.vehicle_id)), dir);
    const auto p = sim::refuelling_window_parameters(m, ascent, sim::phasing_exhaust_velocity_mps(chaser),
                                                     sim::phasing_settling_gas_kg(chaser, f.session.ignition_settling()));
    const auto mean = orbit::station_mean_elements_from_state(f.depot, f.t0, constants::earth_gm_m3ps2,
                                                              constants::earth_j2, constants::wgs84_semi_major_axis_m,
                                                              f.earth);
    const auto pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const auto w = mission::plan_launch_window(pad, f.earth, sim::station_orbit(mean, f.earth), f.t0, p,
                                               constants::earth_gm_m3ps2);
    REQUIRE(w.feasible);
    // off the liftoff: refused, nothing changed
    CHECK_FALSE(sim::launch_campaign_flight(f.session, dir, m, w, p).refusal.empty());
    CHECK(f.session.body_count() == 1u);
    f.session.advance_to(w.liftoff);
    const auto s = sim::launch_campaign_flight(f.session, dir, m, w, p);
    REQUIRE(s.refusal.empty());
    REQUIRE(f.session.iss_mission());
    CHECK_FALSE(f.session.iss_mission()->ship_return.has_value());
    CHECK(f.session.iss_mission()->transfer == sim::MissionTransfer::TankerReserve);
    CHECK(f.session.station_index().has_value());
    while (f.session.iss_mission_leg() == IssMissionLeg::Ascent && f.session.epoch() - w.liftoff < 1200.0) {
        f.session.advance_to(f.session.epoch() + 1.0);
    }
    INFO(f.session.iss_mission_status());
    CHECK((f.session.iss_mission_leg() == IssMissionLeg::Raise ||
           f.session.iss_mission_leg() == IssMissionLeg::Rendezvous));
    CHECK(f.session.station_index().has_value());
}

TEST_CASE("Probe: reboost timeline", "[.f2_probe]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const double fill = std::getenv("PROBE_FILL") ? std::atof(std::getenv("PROBE_FILL")) : 400.0e3;
    f.session.add_station(refuelling_test::depot_config(fill),
                          refuelling_test::depot_state(f.earth, f.t0, 0.0, 193.0e3), f.t0);
    std::printf("start mean %.3f km\n", mean_height_km(f.session));
    std::printf("start %d: %s\n", f.session.start_station_reboost(200.0e3, belly) ? 1 : 0,
                f.session.reboost_status().c_str());
    const auto t0 = f.session.epoch();
    while (f.session.station_reboost_active() && f.session.epoch() - t0 < 4.0 * 3600.0) {
        f.session.advance_to(f.session.epoch() + 60.0);
        const auto& b = f.session.body(0);
        const auto el = orbit::elements_from_state({b.state().position_gcrf_m, b.state().velocity_gcrf_mps},
                                                   constants::earth_gm_m3ps2);
        const auto& ap = f.session.autopilot();
        std::printf("t %7.0f ap %d tti %8.1f rem %6.3f ign %d | hp %.2f ha %.2f km main %.1f kg\n", f.session.epoch() - t0,
                    static_cast<int>(ap.state), ap.burn.time_to_ignition_s, ap.burn.remaining_dv_mps, ap.burn.ignite ? 1 : 0,
                    (el.semi_major_axis_m * (1 - el.eccentricity) - constants::wgs84_semi_major_axis_m) / 1e3,
                    (el.semi_major_axis_m * (1 + el.eccentricity) - constants::wgs84_semi_major_axis_m) / 1e3,
                    b.vehicle().main_propellant_kg());
    }
    if (f.session.last_reboost()) std::printf("end: %s\n", f.session.last_reboost()->reason.c_str());
    if (f.session.station_index()) std::printf("end mean %.3f km\n", mean_height_km(f.session));
}
