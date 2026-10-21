// Refuelling r3: the tanker's phasing to Waypoint 0 below the depot, and the depot launch window from Starbase.

#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <set>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "refuelling_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/orbit/kepler.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

// The tanker in a 170 x 190 km orbit in the depot plane, `behind_deg` of argument of latitude behind it (perigee at
// the tanker's position).
math::StateVector tanker_behind(const refuelling_test::DepotSession& f, double behind_deg) {
    const auto depot_now = refuelling_test::depot_state(f.earth, f.t0, -behind_deg * deg);  // same plane, behind
    const double rp = constants::wgs84_semi_major_axis_m + 170.0e3, ra = constants::wgs84_semi_major_axis_m + 190.0e3;
    const double a = 0.5 * (rp + ra);
    const Vec3 r_hat = math::normalized(depot_now.position_m);
    const Vec3 v_hat = math::normalized(depot_now.velocity_mps - dot(depot_now.velocity_mps, r_hat) * r_hat);
    const double vp = std::sqrt(constants::earth_gm_m3ps2 * (2.0 / rp - 1.0 / a));
    return {rp * r_hat, vp * v_hat};
}

}  // namespace

TEST_CASE("A tanker plans the coelliptic phasing to the depot on Raptor and cold gas", "[refuelling][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    REQUIRE(f.session.plan_rendezvous({}));
    INFO(f.session.rendezvous_status());
    const auto* plan = f.session.rendezvous_plan();
    REQUIRE(plan);
    CHECK(plan->feasible);
    CHECK(plan->total_delta_v_mps > 10.0);
    CHECK(plan->total_delta_v_mps < 150.0);
    CHECK(plan->parameters.waypoint0_below_m == 400.0);  // the depot's target block
    CHECK(plan->parameters.chaser_drag_area_per_mass_m2pkg > 0.0);
    CHECK(plan->parameters.target_drag_area_per_mass_m2pkg > 0.0);  // the depot has no fixed drag area
}

TEST_CASE("Rendezvous burns are modelled at the throttle they fly", "[refuelling][rendezvous]") {
    // r3 minor M1: burns between the two Raptor minimum impulses fly at minimum throttle (start_burn) but were
    // modelled at full throttle
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    const double full_n = f.session.rendezvous_burn_model(std::numeric_limits<double>::infinity()).thrust_n;
    std::set<double> thrusts_n;
    for (double dv = 0.1; dv < 300.0; dv *= 1.05) {
        const double n = f.session.rendezvous_burn_model(dv).thrust_n;
        if (!thrusts_n.empty()) CHECK(n >= *thrusts_n.rbegin());  // never weaker for a larger burn
        thrusts_n.insert(n);
    }
    REQUIRE(thrusts_n.size() == 3);  // cold gas, the belly Raptor at minimum throttle, at full throttle
    CHECK(*thrusts_n.rbegin() == full_n);
    CHECK(*std::next(thrusts_n.begin()) < full_n);  // the Raptor at its minimum throttle
}

TEST_CASE("A tanker without main propellant is refused the rendezvous, no throw", "[refuelling][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(0.0), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    bool planned = true;
    REQUIRE_NOTHROW(planned = f.session.plan_rendezvous({}));
    CHECK_FALSE(planned);
    CHECK_FALSE(f.session.rendezvous_status().empty());
}

TEST_CASE("A tanker with a failed belly Raptor plans on the cold gas or is refused, no throw (review r3 I1)",
          "[refuelling][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    const auto& engines = f.session.body(0).vehicle().stages()[0].engines;
    for (size_t i = 0; i < engines.size(); ++i) {
        if (engines[i].group_id.ends_with("/sea_level_3")) f.session.fail_engine(0, 0, i);
    }
    bool planned = false;
    REQUIRE_NOTHROW(planned = f.session.plan_rendezvous({}));
    INFO(f.session.rendezvous_status());
    if (planned) {
        for (const auto& b : f.session.rendezvous_plan()->burns) CHECK(std::isfinite(norm(b.node.delta_v_gcrf_mps)));
    } else {
        CHECK_FALSE(f.session.rendezvous_status().empty());
        CHECK(f.session.rendezvous_status() != "Kein Rendezvous: keine Triebwerke für die Manöver");  // the cold gas is there
    }
}

TEST_CASE("The tanker flies the phasing to Waypoint 0 below the depot", "[refuelling][rendezvous]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    REQUIRE(f.session.plan_rendezvous({}));
    const double main_before = f.session.body(0).vehicle().rcs_supply_kg();
    for (double t = 0.0; t < 30.0 * 3600.0 && f.session.rendezvous_step() != sim::RendezvousStep::Waypoint0 &&
                         f.session.rendezvous_step() != sim::RendezvousStep::None;
         t += 60.0) {
        f.session.advance_to(f.session.epoch() + 60.0);
    }
    INFO(f.session.rendezvous_status());
    REQUIRE(f.session.rendezvous_step() == sim::RendezvousStep::Waypoint0);
    const auto& depot = f.session.body(1).state();
    const auto rel = orbit::lvlh_relative_state({depot.position_gcrf_m, depot.velocity_gcrf_mps},
                                                {f.session.body(0).state().position_gcrf_m,
                                                 f.session.body(0).state().velocity_gcrf_mps});
    CHECK(norm(rel.position_m - Vec3{0.0, 0.0, 400.0}) < 150.0);
    CHECK(norm(rel.velocity_mps) < 0.5);
    bool raptor = false, cold_gas = false;
    for (const auto& b : f.session.rendezvous_burns()) {
        (norm(b.node.delta_v_gcrf_mps) >= 7.0 ? raptor : cold_gas) = true;
    }
    CHECK(raptor);
    CHECK(cold_gas);
    const double used = main_before - f.session.body(0).vehicle().rcs_supply_kg();  // burns, RCS and boil-off
    CHECK(used > 0.0);
    std::printf("[r3] tanker phasing: %zu burns, main propellant used %.1f t, Waypoint 0 miss %.1f m, %.3f m/s\n",
                f.session.rendezvous_burns().size(), used / 1000.0, norm(rel.position_m - Vec3{0.0, 0.0, 400.0}),
                norm(rel.velocity_mps));
}

// The plane crosses the pad about once a day (inclination 26.5 deg just above the pad latitude 26.0: the two daily
// crossings merge), but the depot's phase moves ~90 deg per day while the 170 x 190 km tanker catches up only ~25 deg
// in the crew phasing limit of 28 h: most crossings are rejected (finding r3, open for the r6 campaign cadence).
TEST_CASE("Depot launch window from Starbase: the plane passes the pad about once a day, the phasing closes on few",
          "[refuelling][launch_window]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    const auto site = config::load_launch_site(refuelling_test::config_dir, "starbase_olp1");
    const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    mission::LaunchWindowParameters p;
    p.inclination_rad = 26.5 * deg;
    p.insertion_s = 520.0;  // [test] Starship to SECO
    p.insertion_arc_rad = 20.0 * deg;
    p.separation_periapsis_radius_m = constants::wgs84_semi_major_axis_m + 170.0e3;
    p.separation_apoapsis_radius_m = constants::wgs84_semi_major_axis_m + 190.0e3;
    p.search_s = 12.0 * 86400.0;  // [test] the crew default (3 days) finds none here
    const mission::StationStateAt depot = [&](time::Epoch t) {
        const double n =
            std::sqrt(constants::earth_gm_m3ps2 / std::pow(constants::wgs84_semi_major_axis_m + 200.0e3, 3));
        return refuelling_test::depot_state(f.earth, t, n * (t - f.t0));
    };
    const auto w1 = mission::plan_launch_window(pad, f.earth, depot, f.t0, p, constants::earth_gm_m3ps2);
    INFO(w1.reason);
    REQUIRE(w1.feasible);
    CHECK(w1.schedule.feasible);
    // crossings about a day apart: the window is the (rejected + 1)-th crossing after t0
    const double hours = (w1.liftoff - f.t0) / 3600.0;
    REQUIRE(w1.rejected >= 1);
    CHECK(hours > 20.0 * w1.rejected);
    CHECK(hours < 26.0 * (w1.rejected + 1));
    std::printf("[r3] depot window: liftoff %.1f h after t0, %d earlier crossings rejected (%.1f h per crossing)\n",
                hours, w1.rejected, hours / (w1.rejected + 1));
}

TEST_CASE("Tanker from its phasing orbit to hard capture at the depot", "[refuelling][rendezvous][docking][slow]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::tanker_config(200.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    REQUIRE(f.session.plan_rendezvous({}));
    sim::IssMissionSequence sequence;  // the generic legs: Rendezvous -> Proximity -> Docking -> Docked
    sequence.station_port = "transfer_dock";
    // as the refuelling missions (r5): a capture that bounces off is flown again (f2: with the Raptor flying the long
    // phasing burns the tanker arrived differently and its first capture bounced)
    sequence.max_capture_retries = 2;
    f.session.set_iss_mission(sequence);
    REQUIRE(f.session.iss_mission_leg() == sim::IssMissionLeg::Rendezvous);
    double t = 0.0;
    for (; t < 36.0 * 3600.0 && f.session.iss_mission_leg() != sim::IssMissionLeg::Docked &&
           f.session.iss_mission_leg() != sim::IssMissionLeg::Ended;
         t += 30.0) {
        f.session.advance_to(f.session.epoch() + 30.0);
    }
    INFO(f.session.iss_mission_status() << " | " << f.session.proximity_status() << " | "
                                        << f.session.docking_status());
    REQUIRE(f.session.iss_mission_leg() == sim::IssMissionLeg::Docked);
    REQUIRE(f.session.docked());
    std::printf("[r3] tanker campaign: docked after %.1f h, peak compression %.1f kN\n", t / 3600.0,
                f.session.docking_loads()->compression_n / 1000.0);
}

TEST_CASE("A long cold-gas rendezvous burn goes to the Raptor once its minimum impulse fits (f2)",
          "[refuelling][rendezvous][campaign_flown]") {
    // f2: the HLS's belly Raptor put every burn below ~25 m/s on the 3 kN cold gas; a 24 m/s braking burn took ~40 min
    test::spice_session();
    refuelling_test::DepotSession f;
    f.session.place_in_orbit(refuelling_test::ship_with("starship_hls_launch", 190.0e3), tanker_behind(f, 8.0), f.t0);
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    const double cold_gas_n = f.session.rendezvous_burn_model(0.5).thrust_n;  // a short burn: cold gas
    CHECK(cold_gas_n < 1.0e4);
    const double mass_kg = f.session.body(0).vehicle().mass_properties().mass_kg;
    const double longest_cold_gas_mps = sim::FlightSession::rendezvous_max_cold_gas_burn_s * cold_gas_n / mass_kg;
    CHECK(f.session.rendezvous_burn_model(0.9 * longest_cold_gas_mps).thrust_n == cold_gas_n);
    CHECK(f.session.rendezvous_burn_model(24.4).thrust_n > 1.0e6);  // the braking burn of the f2 campaign: Raptor
    // never weaker for a larger burn
    double last_n = 0.0;
    for (double dv = 0.1; dv < 300.0; dv *= 1.05) {
        const double n = f.session.rendezvous_burn_model(dv).thrust_n;
        CHECK(n >= last_n);
        last_n = n;
    }
}
