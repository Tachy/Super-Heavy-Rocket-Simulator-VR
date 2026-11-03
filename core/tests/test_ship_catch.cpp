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
#include <catch2/generators/catch_generators.hpp>

#include "abort_trace.hpp"
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
    bool abort_in_flip = false;        // the pilot gives the catch up 1 s into the flip
    double propellant_kg = 35000.0;    // the ship's landing_propellant_kg [E]
    std::optional<gnc::ShipLandingParameters> parameters;  // none: from the ship's config
};

test::CatchRun fly_from_gate(const Gate& gate, double* flip_s = nullptr) {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
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
    if (gate.abort_in_flip) {
        session.advance_to(t0 + 1.0, 1000);
        session.abort_landing(0);
    }
    if (!gate.failed_group.empty()) {
        session.advance_to(t0 + 1.0, 1000);
        const auto& engines = session.body(0).vehicle().stages().front().engines;
        for (size_t i = 0; i < engines.size(); ++i) {
            if (engines[i].group_id == gate.failed_group) session.fail_engine(0, 0, i);
        }
    }
    auto run = test::fly_landing(session, earth(), t0, 120.0);
    run.result = session.ship_return_result(0);
    const auto* g = session.ship_landing(0);
    if (g) std::printf("flip %.1f s, ends at %.0f m; ", g->flip_duration_s(), g->flip_end_height_m());
    if (g && g->has_contact()) {
        std::printf("contact tilt %.1f deg, sink %.1f m/s; ", g->contact_tilt_rad() / deg, g->contact_sink_mps());
    }
    std::printf("ship from the gate: %s (%s), offset %.2f m, lateral %.2f m/s, sink %.2f m/s, first plan attempts %d, %d plans "
                "(%d failed), %.1f t propellant left\n",
                sim::to_string(run.tower_phase), sim::to_string(run.result), run.offset_m, run.lateral_mps, run.sink_mps, g ? g->first_plan_attempts() : -1,
                g && g->scvx() ? g->scvx()->statistics().plans : 0, g && g->scvx() ? g->scvx()->statistics().failures : 0,
                session.body(0).vehicle().propellant_kg() / 1000.0);
    (void)flip_s;
    return run;
}

}  // namespace

// Gate back along the line pad -> tower (the site's azimuth), nose to the tower: the flip pushes towards it and the side
// pins lie across the arms without a roll.
static Gate nominal_gate() {
    Gate g;
    const double azimuth = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2").tower->azimuth_rad;
    const Vec3 to_tower{std::sin(azimuth), std::cos(azimuth), 0.0};
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
    Gate g = seed ? dispersed_gate(seed) : nominal_gate();
    // STARSIM_SHIP_FAILED=<group>: that group fails in the flip (the campaign's last case: ship/sea_level_3, seed 0).
    if (const char* failed = std::getenv("STARSIM_SHIP_FAILED")) g.failed_group = failed;
    std::printf("seed %u: propellant %.0f kg, gate v %.1f,%.1f,%.1f m/s, height %.0f m\n", seed, g.propellant_kg,
                g.velocity_enu_mps.x, g.velocity_enu_mps.y, g.velocity_enu_mps.z, g.height_m);
    fly_from_gate(g);
}

// Crew escape calibration (plan Task 3): the lowest propellant load at the nominal gate that is still caught, in 1 t
// steps from the landing load down (config/crew_escape min_catch_propellant_kg = this + margin).
TEST_CASE("Ship catch: lowest catchable gate load", "[.][ship-single]") {
    double lowest_caught_kg = -1.0;
    for (double propellant_kg = 35000.0; propellant_kg >= 10000.0; propellant_kg -= 1000.0) {
        Gate g = nominal_gate();
        g.propellant_kg = propellant_kg;
        const auto run = fly_from_gate(g);
        const bool caught = run.caught_within_tolerance() && !run.tower_struck;
        std::printf("calibration: %.0f t -> %s\n", propellant_kg / 1000.0, caught ? "caught" : "missed");
        if (!caught) break;
        lowest_caught_kg = propellant_kg;
    }
    std::printf("calibration: lowest caught gate load %.0f t\n", lowest_caught_kg / 1000.0);
}

TEST_CASE("Ship flips at the gate and is caught", "[ship][landing][catch]") {
    const auto run = fly_from_gate(nominal_gate());
    CHECK(run.caught_within_tolerance());
    CHECK_FALSE(run.tower_struck);
    CHECK(run.outcome == sim::LandingOutcome::Caught);
}

// Ship catch robustness (2026-10-21): a catch given up comes down upright and slow, never tumbling, never on the tower.
TEST_CASE("Ship catch: an abort requested in the flip comes down upright", "[ship][landing][catch]") {
    Gate g = nominal_gate();
    g.abort_in_flip = true;
    const auto run = fly_from_gate(g);
    CHECK(run.result == sim::ReturnResult::UprightAbort);
    CHECK_FALSE(run.tower_struck);
}

// Diagnostics (STARSIM_ABORT_TRACE_DIR): ship aborts at falling CG heights above the pad, from the first in the flip down
// to below the arms' reach, with whether the water stayed required; for tools/validation/abort_clearance.py.
TEST_CASE("Ship abort clearance trace", "[.abort_clearance]") {
    // STARSIM_ABORT_HEIGHT=<m> flies that height alone, STARSIM_SHIP_SEED=<seed> a dispersed gate of the campaign.
    const char* height_text = std::getenv("STARSIM_ABORT_HEIGHT");
    const char* seed_text = std::getenv("STARSIM_SHIP_SEED");
    const unsigned seed = seed_text ? static_cast<unsigned>(std::atoi(seed_text)) : 0u;
    const double abort_height_m = height_text ? std::atof(height_text) : GENERATE(900.0, 600.0, 550.0, 500.0, 475.0, 450.0, 425.0, 400.0, 375.0, 350.0, 325.0, 300.0, 275.0, 250.0, 225.0, 200.0, 170.0);
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    Gate gate = seed ? dispersed_gate(seed) : nominal_gate();
    // STARSIM_GATE_HEIGHT=<m>: the gate height the entry guidance hands over at (800 m) instead of the catch cases' 950 m
    const char* gate_text = std::getenv("STARSIM_GATE_HEIGHT");
    if (gate_text) gate.height_m += std::atof(gate_text) - nominal_gate().height_m;
    const auto cfg = ship_with(gate.propellant_kg);
    auto state = test::state_over_site(session.launch_site(), earth(), t0, gate.offset_enu_m + Vec3{0.0, 0.0, gate.height_m},
                                       gate.velocity_enu_mps);
    const auto t = earth().fixed_to_gcrf(t0);
    const auto attitude_state = test::state_over_site(session.launch_site(), earth(), t0, gate.offset_enu_m + Vec3{0.0, 0.0, gate.height_m},
                                                      gate.attitude_velocity_enu_mps.value_or(gate.velocity_enu_mps));
    const Vec3 v_rel = attitude_state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), gate.alpha_deg * deg,
                                                    gate.bank_deg * deg, cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    session.place_landing_ship(cfg, state, t0, gate.parameters);
    test::AbortTrace trace("ship_g" + std::string(gate_text ? gate_text : "950") + "_s" + std::to_string(seed) + "_" + std::to_string(static_cast<int>(abort_height_m)));
    const auto& ship = session.body(0);
    bool aborted = false, abort_accepted = false, water_kept = false;
    for (double s = 0.0; s < 150.0 && !ship.impacted() && !ship.resting() && !ship.caught(); s += 0.05) {
        session.advance_to(t0 + s + 0.05, 100);
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const Vec3 p = sim::site_enu_m(session.launch_site(), transpose(tr.rotation) * ship.state().position_gcrf_m);
        if (!aborted && p.z < abort_height_m) {
            abort_accepted = session.abort_landing(0);
            aborted = true;
        }
        if (const auto* g = session.ship_landing(0); g && g->scvx()) water_kept = g->scvx()->sea_required();
        trace.sample(session, earth(), 0, s);
    }
    std::printf("[clearance] gate %s seed %u ship abort at %.0f m: accepted %d, water kept %d, %s / %s, struck %d, %.1f t left\n", gate_text ? gate_text : "950", seed, abort_height_m,
                abort_accepted ? 1 : 0, water_kept ? 1 : 0, sim::to_string(session.landing_outcome(0)), sim::to_string(session.ship_return_result(0)),
                session.tower()->tower_struck() ? 1 : 0, ship.vehicle().propellant_kg() / 1e3);
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

    int caught = 0, lost = 0;
    for (size_t i = 0; i < cases.size(); ++i) {
        CAPTURE(i);
        std::printf("case %2zu: ", i);
        const auto run = fly_from_gate(cases[i]);
        std::fflush(stdout);
        CHECK_FALSE(run.tower_struck);  // never, in any case
        if (run.caught_within_tolerance()) ++caught;
        if (run.result == sim::ReturnResult::Lost) ++lost;
        // every case ends caught or in an upright abort (not lost, not still flying when the run ends)
        CHECK((run.result == sim::ReturnResult::Caught || run.result == sim::ReturnResult::UprightAbort));
    }
    std::printf("caught %d of %zu, lost %d\n", caught, cases.size(), lost);
    // Ship catch robustness (2026-10-21): the engine-failure case may abort - upright; none lost (was >= 19 caught)
    CHECK(caught >= 20);
    CHECK(lost == 0);
}
