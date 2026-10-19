// The ship from the gate above Starbase: relight, flip and SCvx landing burn (3 -> 2 -> 1) into the tower's arms.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/propellant_loading.hpp"
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
    config::fill_tanks(ship, propellant_kg);
    for (auto& f : ship.flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    return cfg;
}

struct Gate {
    Vec3 offset_enu_m{0.0, 0.0, 0.0};   // horizontal offset from the pad centre
    double height_m = 950.0;           // CG above the pad (flight 13 relit at ~0.95 km)
    Vec3 velocity_enu_mps{3.0, 0.0, -100.0};
    double alpha_deg = 80.0;           // belly first into the flow
    double bank_deg = 0.0;
    // Air velocity the attitude is built from (alpha, bank; the nose heads along its horizontal part): the entry
    // guidance turns the nose to the tower, a dispersed gate velocity must not turn it ("": the gate velocity).
    std::optional<Vec3> attitude_velocity_enu_mps;
    std::string failed_group;          // this engine fails 1 s into the flip ("": none)
    double propellant_kg = 35000.0;    // the ship's landing_propellant_kg [E]
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
    const auto attitude_state = test::state_over_site(session.launch_site(), earth(), t0,
                                                      gate.offset_enu_m + Vec3{0.0, 0.0, gate.height_m},
                                                      gate.attitude_velocity_enu_mps.value_or(gate.velocity_enu_mps));
    const Vec3 v_rel = attitude_state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), gate.alpha_deg * deg,
                                                    gate.bank_deg * deg, cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    session.place_landing_ship(cfg, state, t0, gate.parameters);
    if (!gate.failed_group.empty()) {
        session.advance_to(t0 + 1.0, 1000);
        const auto& engines = session.body(0).vehicle().stages().front().engines;
        for (size_t i = 0; i < engines.size(); ++i) {
            if (engines[i].group_id == gate.failed_group) session.fail_engine(0, 0, i);
        }
    }
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

// Gate back along the line pad -> tower (azimuth 315 deg), nose to the tower: the flip pushes towards it and the side
// pins lie across the arms without a roll.
static Gate nominal_gate() {
    Gate g;
    const Vec3 to_tower{std::sin(315.0 * deg), std::cos(315.0 * deg), 0.0};
    g.offset_enu_m = to_tower * -220.0;
    g.velocity_enu_mps = to_tower * 3.0 + Vec3{0.0, 0.0, -100.0};
    return g;
}

// Dispersed gate of the campaign (fixed seed): +-10 m per axis, +-10 m/s per axis, propellant 35 t + 0-3.5 t, alpha and
// bank +-5 deg; the nose to the tower.
static Gate dispersed_gate(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Gate g = nominal_gate();
    g.attitude_velocity_enu_mps = g.velocity_enu_mps;  // nose to the tower
    g.offset_enu_m += Vec3{10.0 * unit(rng), 10.0 * unit(rng), 0.0};
    g.height_m += 10.0 * unit(rng);
    g.velocity_enu_mps += Vec3{10.0 * unit(rng), 10.0 * unit(rng), 10.0 * unit(rng)};
    g.propellant_kg += 1750.0 * (1.0 + unit(rng));  // full headers + 0-3.5 t main-tank residue (the 5 % spread width)
    g.alpha_deg += 5.0 * unit(rng);
    g.bank_deg = 5.0 * unit(rng);
    return g;
}

TEST_CASE("Ship catch single case", "[.][ship-single]") {
    // Diagnostics: STARSIM_SHIP_SEED=<seed of the campaign, 1-20> (0: nominal gate).
    const char* seed_text = std::getenv("STARSIM_SHIP_SEED");
    const unsigned seed = seed_text ? static_cast<unsigned>(std::atoi(seed_text)) : 0u;
    const Gate g = seed ? dispersed_gate(seed) : nominal_gate();
    std::printf("seed %u: propellant %.0f kg, gate v %.1f,%.1f,%.1f m/s, height %.0f m\n", seed, g.propellant_kg,
                g.velocity_enu_mps.x, g.velocity_enu_mps.y, g.velocity_enu_mps.z, g.height_m);
    fly_from_gate(g);
}

TEST_CASE("Ship flips at the gate and is caught", "[ship][landing][catch]") {
    const auto run = fly_from_gate(nominal_gate());
    CHECK(run.caught_within_tolerance());
    CHECK_FALSE(run.tower_struck);
    CHECK(run.outcome == sim::LandingOutcome::Caught);
}

// Dispersed gate states (fixed seeds): the entry guidance's gate accuracy is metres (b2 campaign), the belly flop adds
// wind-free scatter; +-10 m per axis, +-10 m/s per axis, propellant 35 t + 0-3.5 t, alpha and bank +-5 deg. One case
// loses the first engine to be dropped (sea_level_3) in the flip: the switch to two engines comes early.
TEST_CASE("Ship catch campaign: gate dispersions and an engine failure", "[ship][catch][validation][slow]") {
    std::vector<Gate> cases;
    for (unsigned seed = 1; seed <= 20; ++seed) cases.push_back(dispersed_gate(seed));
    Gate failure = nominal_gate();
    failure.failed_group = "ship/sea_level_3";
    cases.push_back(failure);

    int caught = 0;
    for (size_t i = 0; i < cases.size(); ++i) {
        CAPTURE(i);
        std::printf("case %2zu: ", i);
        const auto run = fly_from_gate(cases[i]);
        std::fflush(stdout);
        CHECK_FALSE(run.tower_struck);  // never, in any case
        if (run.caught_within_tolerance()) {
            ++caught;
        } else {
            CHECK((run.outcome == sim::LandingOutcome::Splashdown || run.outcome == sim::LandingOutcome::Landed));
        }
    }
    std::printf("caught %d of %zu\n", caught, cases.size());
    CHECK(caught >= 19);  // >= 90 % of 21, as the booster
}
