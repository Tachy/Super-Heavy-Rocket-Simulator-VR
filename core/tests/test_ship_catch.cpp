// The ship from the gate above Starbase: relight, flip and SCvx landing burn (3 -> 2 -> 1) into the tower's arms.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

const spice::SpiceEphemeris& spice_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

// The ship alone with `propellant_kg` in its main tanks (no header tanks modelled).
config::VehicleConfig ship_with(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    auto& ship = cfg.stages[0];
    double full = 0.0;
    for (const auto& tank : ship.tanks) full += tank.mass_kg;
    for (auto& tank : ship.tanks) tank.mass_kg *= propellant_kg / full;
    for (auto& f : ship.flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    return cfg;
}

struct Gate {
    Vec3 offset_enu_m{0.0, 0.0, 0.0};   // horizontal offset from the pad centre
    double height_m = 950.0;           // CG above the pad (flight 13 relit at ~0.95 km)
    Vec3 velocity_enu_mps{3.0, 0.0, -100.0};
    double alpha_deg = 80.0;           // belly first into the flow
    double propellant_kg = 30000.0;
    gnc::ShipLandingParameters parameters = gnc::ship_landing_parameters();
};

test::CatchRun fly_from_gate(const Gate& gate, double* flip_s = nullptr) {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto cfg = ship_with(gate.propellant_kg);
    auto state = test::state_over_site(session.launch_site(), earth(), t0, gate.offset_enu_m + Vec3{0.0, 0.0, gate.height_m},
                                       gate.velocity_enu_mps);
    const auto t = earth().fixed_to_gcrf(t0);
    const Vec3 v_rel = state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), gate.alpha_deg * deg,
                                                    0.0, cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    session.place_landing_ship(cfg, state, t0, gate.parameters);
    const auto run = test::fly_landing(session, earth(), t0, 120.0);
    const auto* g = session.ship_landing(0);
    if (g) std::printf("flip %.1f s, ends at %.0f m; ", g->flip_duration_s(), g->flip_end_height_m());
    std::printf("ship from the gate: %s, offset %.2f m, lateral %.2f m/s, sink %.2f m/s, first plan attempts %d, %d plans "
                "(%d failed), %.1f t propellant left\n",
                sim::to_string(run.tower_phase), run.offset_m, run.lateral_mps, run.sink_mps, g ? g->first_plan_attempts() : -1,
                g && g->scvx() ? g->scvx()->statistics().plans : 0, g && g->scvx() ? g->scvx()->statistics().failures : 0,
                session.body(0).vehicle().propellant_kg() / 1000.0);
    (void)flip_s;
    return run;
}

}  // namespace

TEST_CASE("Ship flips at the gate and is caught", "[ship][landing][catch]") {
    // gate back along the line pad -> tower (azimuth 315 deg), nose to the tower: the flip pushes towards it and the
    // side pins lie across the arms without a roll
    Gate g;
    const Vec3 to_tower{std::sin(315.0 * deg), std::cos(315.0 * deg), 0.0};
    g.offset_enu_m = to_tower * -220.0;
    g.velocity_enu_mps = to_tower * 3.0 + Vec3{0.0, 0.0, -100.0};
    const auto run = fly_from_gate(g);
    CHECK(run.caught_within_tolerance());
    CHECK_FALSE(run.tower_struck);
    CHECK(run.outcome == sim::LandingOutcome::Caught);
}
