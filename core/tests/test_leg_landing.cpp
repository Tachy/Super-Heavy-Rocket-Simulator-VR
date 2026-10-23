// Falcon 9 first stage on its landing legs (milestone c2): deployment, hand-over of the ground contact, drop tests on
// the LZ-40 pad judged by the session ("gelandet", "umgekippt", "Aufprall").

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/sim/leg_landing_sim.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double landing_propellant_kg = 1000.0;  // what is left after a landing burn [E]

sim::LaunchSite lz40() {
    return sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40"));
}

struct Env {
    Env() { test::spice_session(); }
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
};

Env& env() {
    static Env e;
    return e;
}

// Falcon 9 first stage (dry + 1 t) whose lowest foot (deployed geometry) is `feet_height_m` above the pad, falling at
// `velocity_enu_mps`, tilted by `tilt_rad` about north (towards east), legs at `deployment`.
sim::FlightSession& drop(sim::FlightSession& session, double feet_height_m, const Vec3& velocity_enu_mps,
                         double tilt_rad = 0.0, double deployment = 1.0) {
    const auto cfg = test::booster_config("falcon9_block5", landing_propellant_kg);
    const auto t0 = env().time.from_utc({2026, 10, 13, 12, 0, 0.0});
    session.set_launch_site(lz40());
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const double foot = cfg.stages.front().landing_legs.foot_station_m;
    const double radius = cfg.stages.front().landing_legs.footprint_radius_m;
    const auto tilt = math::Quat::from_axis_angle({0.0, 1.0, 0.0}, tilt_rad);
    session.place_body(cfg, test::state_over_site(session.launch_site(), env().earth, t0,
                                                  {0.0, 0.0, feet_height_m + (cg - foot) * std::cos(tilt_rad) +
                                                                 radius * std::sin(std::abs(tilt_rad))},
                                                  velocity_enu_mps, tilt),
                       t0);
    session.set_leg_deployment(0, deployment);
    return session;
}


struct DropResult {
    sim::LegTouchdown touchdown = sim::LegTouchdown::None;
    double max_stroke_m = 0.0;
    double crush_work_j = 0.0;   // sum of crushed stroke x crush force
    double energy_lost_j = 0.0;  // kinetic + potential (relative to the ground) from the start to the end
    double offset_m = 0.0;       // horizontal CG offset at the end
};

double mechanical_energy_j(const sim::FlightSession& session) {
    const auto& b = session.body(0);
    const auto t = env().earth.fixed_to_gcrf(session.epoch());
    const Vec3 r = b.state().position_gcrf_m;
    const Vec3 v = b.state().velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), r);
    const auto enu = sim::site_enu_m(session.launch_site(), transpose(t.rotation) * r);
    const double m = b.vehicle().mass_properties().mass_kg;
    return 0.5 * m * dot(v, v) + m * constants::standard_gravity_mps2 * enu.z;
}

// Runs a placed drop until the legs' judgement (or 20 s); one advance_to per 0.1 s unless `one_call`.
DropResult run_drop(sim::FlightSession& session, bool one_call = false) {
    DropResult r;
    const double e0 = mechanical_energy_j(session);
    const auto t0 = session.epoch();
    if (one_call) {
        session.advance_to(t0 + 60.0);
    } else {
        while (session.leg_touchdown(0) == sim::LegTouchdown::None && session.epoch() - t0 < 20.0) {
            session.advance_to(session.epoch() + 0.1);
        }
    }
    r.touchdown = session.leg_touchdown(0);
    const auto crushed = session.leg_crushed_m(0);
    const double crush_force = session.body(0).vehicle().stages().front().config.landing_legs.crush_force_n;
    for (double c : crushed) {
        r.max_stroke_m = std::max(r.max_stroke_m, c);
        r.crush_work_j += c * crush_force;
    }
    r.energy_lost_j = e0 - mechanical_energy_j(session);
    const auto t = env().earth.fixed_to_gcrf(session.epoch());
    const auto enu = sim::site_enu_m(session.launch_site(), transpose(t.rotation) * session.body(0).state().position_gcrf_m);
    r.offset_m = std::hypot(enu.x, enu.y);
    std::printf("[legs] %-10s stroke %.3f m  crush work %.0f kJ  energy lost %.0f kJ  offset %.2f m\n",
                sim::to_string(r.touchdown), r.max_stroke_m, r.crush_work_j / 1e3, r.energy_lost_j / 1e3, r.offset_m);
    return r;
}

}  // namespace

TEST_CASE("Landing legs deploy in their deploy time and stay deployed", "[legs]") {
    const auto cfg = test::booster_config("falcon9_block5", landing_propellant_kg);
    vehicle::Vehicle v(cfg, STARSIM_CONFIG_DIR);
    REQUIRE(v.has_landing_legs());
    CHECK(v.leg_deployment() == 0.0);
    v.update_landing_legs(1.0);
    CHECK(v.leg_deployment() == 0.0);  // not commanded
    v.deploy_legs();
    const double t = cfg.stages.front().landing_legs.deploy_time_s;
    v.update_landing_legs(0.5 * t);
    CHECK(v.leg_deployment() == Catch::Approx(0.5));
    v.update_landing_legs(t);
    CHECK(v.leg_deployment() == 1.0);
    // A vehicle without legs ignores the command.
    vehicle::Vehicle starship(test::booster_config("starship_block3", 50000.0), STARSIM_CONFIG_DIR);
    starship.deploy_legs();
    starship.update_landing_legs(10.0);
    CHECK(starship.leg_deployment() == 0.0);
}

TEST_CASE("Stowed legs keep the bottom-point ground contact", "[legs][session]") {
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    // Bottom (engine bell exit) 0.1 m up, 2 m/s: touches at ~2.5 m/s (< 3 m/s). drop() places the deployed feet.
    const double foot = test::booster_config("falcon9_block5", landing_propellant_kg).stages.front().landing_legs.foot_station_m;
    drop(session, 0.1 + foot, {0.0, 0.0, -2.0}, 0.0, 0.0);
    session.advance_to(session.epoch() + 5.0);  // one call: the session must tick near the ground
    CHECK(session.body(0).resting());
    CHECK_FALSE(session.body(0).impacted());
}

TEST_CASE("A landed stage is held where it stands", "[legs][session]") {
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    drop(session, 0.0, {});
    vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(test::booster_config("falcon9_block5", landing_propellant_kg),
                                                                STARSIM_CONFIG_DIR),
                             session.body(0).state());
    const auto epoch = session.epoch();
    body.hold_landed(session.earth_orientation(), epoch);
    const auto& earth = session.earth_orientation();  // the frame the session flies in
    const Vec3 before = transpose(earth.fixed_to_gcrf(epoch).rotation) * body.state().position_gcrf_m;
    for (int i = 0; i < 100; ++i) body.step(session.environment(), epoch + 0.01 * i, 0.01, {});
    body.coast(session.environment(), epoch + 1.0, 59.0);
    const Vec3 after = transpose(earth.fixed_to_gcrf(epoch + 60.0).rotation) * body.state().position_gcrf_m;
    CHECK(norm(after - before) < 1e-6);
    CHECK(body.resting());
    CHECK_FALSE(body.caught());
}

TEST_CASE("Gentle upright touchdowns on the legs land", "[legs][session]") {
    for (double sink : {1.0, 3.0, 6.0}) {
        sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
        drop(session, 0.01, {0.0, 0.0, -sink});
        const auto r = run_drop(session);
        CHECK(r.touchdown == sim::LegTouchdown::Landed);
        CHECK(session.body(0).resting());
        CHECK_FALSE(session.body(0).impacted());
        if (sink >= 6.0) CHECK(r.max_stroke_m > 0.0);  // the crush cores take the energy
    }
}

TEST_CASE("Crush work accounts for the energy of a 6 m/s touchdown", "[legs][session]") {
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    drop(session, 0.01, {0.0, 0.0, -6.0});
    const auto r = run_drop(session);
    REQUIRE(r.touchdown == sim::LegTouchdown::Landed);
    CHECK(r.crush_work_j == Catch::Approx(r.energy_lost_j).epsilon(0.05));
}

TEST_CASE("A hard touchdown exhausts the crush cores", "[legs][session]") {
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    drop(session, 0.01, {0.0, 0.0, -10.0});
    CHECK(run_drop(session).touchdown == sim::LegTouchdown::Impact);
    CHECK(session.body(0).impacted());
}

TEST_CASE("Tilted touchdowns: 5 deg lands, steep ones are lost", "[legs][session]") {
    {
        sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
        drop(session, 0.05, {0.0, 0.0, -2.0}, 5.0 * deg);
        CHECK(run_drop(session).touchdown == sim::LegTouchdown::Landed);
    }
    {
        sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
        // 25 deg at 3 m/s: the lowest leg takes the whole blow and its crush core runs out (the stage is lost). At 2 m/s
        // the 6.5 kN nitrogen thrusters (follow-ups Task 7) now hold the attitude and it lands, 6.3 m off.
        drop(session, 0.05, {0.0, 0.0, -3.0}, 25.0 * deg);
        CHECK(run_drop(session).touchdown == sim::LegTouchdown::Impact);
        CHECK(session.body(0).impacted());
    }
    {
        // Beyond the tip-over angle on a foot (slowly, the core holds): it falls over.
        sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
        drop(session, 0.02, {0.0, 0.0, -0.3}, 32.0 * deg);
        CHECK(run_drop(session).touchdown == sim::LegTouchdown::TippedOver);
        CHECK(session.body(0).impacted());
    }
}

TEST_CASE("A sliding touchdown comes to rest on the legs", "[legs][session]") {
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    drop(session, 0.01, {2.0, 0.0, -2.0});
    const auto r = run_drop(session);
    CHECK(r.touchdown == sim::LegTouchdown::Landed);
    CHECK(r.offset_m < 2.0);
}

TEST_CASE("Half-deployed legs: the base strikes the pad", "[legs][session]") {
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    drop(session, 0.5, {0.0, 0.0, -2.0}, 0.0, 0.5);
    CHECK(run_drop(session).touchdown == sim::LegTouchdown::Impact);
}

TEST_CASE("A drop in one long advance is still judged", "[legs][session]") {
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    drop(session, 2.0, {0.0, 0.0, -3.0});
    CHECK(run_drop(session, true).touchdown == sim::LegTouchdown::Landed);
}
