// Falcon 9 first stage on its landing legs (milestone c2): deployment, hand-over of the ground contact, drop tests on
// the LZ-40 pad judged by the session ("gelandet", "umgekippt", "Aufprall").

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
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

// Falcon 9 first stage (dry + 1 t) whose feet (deployed geometry) are `feet_height_m` above the pad, falling at
// `velocity_enu_mps`, tilted by `tilt_rad` about north (towards east), legs at `deployment`.
sim::FlightSession& drop(sim::FlightSession& session, double feet_height_m, const Vec3& velocity_enu_mps,
                         double tilt_rad = 0.0, double deployment = 1.0) {
    const auto cfg = test::booster_config("falcon9_block5", landing_propellant_kg);
    const auto t0 = env().time.from_utc({2026, 10, 13, 12, 0, 0.0});
    session.set_launch_site(lz40());
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const double foot = cfg.stages.front().landing_legs.foot_station_m;
    const auto tilt = math::Quat::from_axis_angle({0.0, 1.0, 0.0}, tilt_rad);
    session.place_body(cfg, test::state_over_site(session.launch_site(), env().earth, t0,
                                                  {0.0, 0.0, cg - foot + feet_height_m}, velocity_enu_mps, tilt),
                       t0);
    session.set_leg_deployment(0, deployment);
    return session;
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
    body.hold_landed(env().earth, epoch);
    const Vec3 before = transpose(env().earth.fixed_to_gcrf(epoch).rotation) * body.state().position_gcrf_m;
    for (int i = 0; i < 100; ++i) body.step(session.environment(), epoch + 0.01 * i, 0.01, {});
    body.coast(session.environment(), epoch + 1.0, 59.0);
    const Vec3 after = transpose(env().earth.fixed_to_gcrf(epoch + 60.0).rotation) * body.state().position_gcrf_m;
    CHECK(norm(after - before) < 1e-6);
    CHECK(body.resting());
    CHECK_FALSE(body.caught());
}
