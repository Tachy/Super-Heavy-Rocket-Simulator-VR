// The booster placed at landing-burn ignition over Starbase flies the burn on SCvx plans and is caught.

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include <random>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double nominal_propellant_kg = 70.0e3;  // below the FAA EA's ~74 t at ignition (Task 1: 70.4 t)
constexpr double nominal_sink_mps = 355.0;        // [D] flights 13/14: altitude falls ~0.35-0.4 km/s at landing-burn ignition

const spice::SpiceEphemeris& ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

const spice::SpiceEphemeris& de440() { return ephemeris(); }

time::Epoch start() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
}

test::CatchRun fly(const Vec3& offset_enu_m, const Vec3& velocity_offset_mps) {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, offset_enu_m + Vec3{0.0, 0.0, h},
                                                          velocity_offset_mps + Vec3{0.0, 0.0, -nominal_sink_mps}),
                                    t0, gnc::BoosterReturnParameters{}, gnc::ReturnPhase::LandingBurn);
    const auto run = test::fly_landing(session, earth(), t0, 90.0);
    if (const auto* g = session.return_guidance(0); g && g->scvx()) {
        const auto& s = g->scvx()->statistics();
        std::printf("ignition at %.0f m: %s, offset %.2f m, lateral %.2f m/s, sink %.2f m/s, %d plans (%d failed), "
                    "max %.0f ms\n", h, sim::to_string(run.tower_phase), run.offset_m, run.lateral_mps, run.sink_mps,
                    s.plans, s.failures, s.max_solve_ms);
    } else {
        std::printf("ignition at %.0f m: no SCvx guidance (fallback)\n", h);
    }
    return run;
}

}  // namespace

TEST_CASE("SCvx flies a nominal landing burn onto the arms", "[gnc][catch]") {
    const auto run = fly(Vec3{20.0, -10.0, 0.0}, {});
    CHECK(run.caught_within_tolerance());
    CHECK(run.sink_mps < 2.0);
    CHECK_FALSE(run.tower_struck);
    CHECK(run.outcome == sim::LandingOutcome::Caught);
}

TEST_CASE("A commanded abort late in the landing burn ditches the booster at sea", "[gnc][catch][abort]") {
    // The pilot (or the tower) gives the catch up at a given CG height above the pad: the booster must reach the sea
    // beyond the shoreline, not hover over land until its propellant runs out.
    const double abort_height_m = GENERATE(1500.0, 800.0, 400.0);
    CAPTURE(abort_height_m);
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, {20.0, -10.0, h},
                                                          {0.0, 0.0, -nominal_sink_mps}),
                                    t0, gnc::BoosterReturnParameters{}, gnc::ReturnPhase::LandingBurn);
    const auto& booster = session.body(0);
    const auto& site = session.launch_site();
    bool aborted = false;
    double t = 0.0;
    for (; t < 150.0 && !booster.impacted() && !booster.resting() && !booster.caught(); t += 0.05) {
        session.advance_to(t0 + t, 100);
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const Vec3 p = sim::site_enu_m(site, transpose(tr.rotation) * booster.state().position_gcrf_m);
        if (!aborted && p.z < abort_height_m) {
            session.abort_landing(0);
            aborted = true;
        }
    }
    const auto tr = earth().fixed_to_gcrf(session.epoch());
    const Vec3 p = sim::site_enu_m(site, transpose(tr.rotation) * booster.state().position_gcrf_m);
    std::printf("[abort] at %.0f m: %s after %.1f s at %.0f m east / %.0f m north, %.1f t left\n", abort_height_m,
                sim::to_string(session.landing_outcome(0)), t, p.x, p.y, booster.vehicle().propellant_kg() / 1e3);
    CHECK(aborted);
    CHECK(session.landing_outcome(0) == sim::LandingOutcome::Splashdown);
    CHECK_FALSE(session.tower()->tower_struck());
}

TEST_CASE("An abort the guidance cannot fly leaves the tower's arms alone", "[gnc][catch][abort]") {
    // Review finding: without water to ditch in, or once the booster is down, the abort is refused - the arms must not
    // open under a booster that keeps aiming at them.
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
    site.sea.reset();  // a tower site without a coastline
    session.set_launch_site(site);
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    session.place_returning_booster(cfg, test::state_over_site(session.launch_site(), earth(), t0, {20.0, -10.0, h},
                                                               {0.0, 0.0, -nominal_sink_mps}),
                                    t0, gnc::BoosterReturnParameters{}, gnc::ReturnPhase::LandingBurn);
    session.advance_to(t0 + 1.0, 100);
    CHECK_FALSE(session.abort_landing(0));
    CHECK(session.tower()->phase() != sim::CatchPhase::Aborted);

    gnc::BoosterReturnGuidance landed(session.return_site(session.body(0).vehicle()), gnc::BoosterReturnParameters{});
    landed.skip_to(gnc::ReturnPhase::Landed);
    CHECK_FALSE(landed.request_abort());
}

TEST_CASE("SCvx landings are deterministic", "[gnc][catch]") {
    const auto a = fly({20.0, -10.0, 0.0}, {3.0, 0.0, 0.0});
    const auto b = fly({20.0, -10.0, 0.0}, {3.0, 0.0, 0.0});
    CHECK(a.offset_m == b.offset_m);
    CHECK(a.lateral_mps == b.lateral_mps);
    CHECK(a.sink_mps == b.sink_mps);
}

namespace {

struct Dispersion {
    Vec3 offset_m;              // lateral, site frame
    Vec3 velocity_mps;          // added to the nominal fall
    double mass_factor = 1.0;   // residual propellant
    math::Quat tilt;            // attitude error
    bool engine_failure = false;
};

// Fixed seeds; realistic dispersions at ignition after the grid-fin descent (spec, decision 2026-10-13): lateral
// +-10 m, velocity +-3 m/s per axis, residual propellant +-5 %, attitude +-3 deg about a random horizontal axis.
Dispersion dispersion(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Dispersion d;
    d.offset_m = {10.0 * unit(rng), 10.0 * unit(rng), 0.0};
    d.velocity_mps = {3.0 * unit(rng), 3.0 * unit(rng), 3.0 * unit(rng)};
    d.mass_factor = 1.0 + 0.05 * unit(rng);
    const double azimuth = constants::pi * unit(rng);
    d.tilt = math::Quat::from_axis_angle({std::cos(azimuth), std::sin(azimuth), 0.0}, 3.0 * constants::deg_to_rad * unit(rng));
    return d;
}

test::CatchRun fly_dispersed(const Dispersion& d) {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto parameters = gnc::BoosterReturnParameters{};
    const auto nominal = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, nominal, earth(), t0, nominal_sink_mps, parameters);
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg * d.mass_factor);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, d.offset_m + Vec3{0.0, 0.0, h},
                                                          d.velocity_mps + Vec3{0.0, 0.0, -nominal_sink_mps}, d.tilt),
                                    t0, parameters, gnc::ReturnPhase::LandingBurn);
    if (d.engine_failure) {
        // One middle-ring engine fails 1 s into the 13-engine phase.
        session.advance_to(t0 + 1.0, 1000);
        const auto& engines = session.body(0).vehicle().stages().front().engines;
        for (size_t i = 0; i < engines.size(); ++i) {
            if (engines[i].group_id == "booster/middle") {
                session.fail_engine(0, 0, i);
                break;
            }
        }
    }
    return test::fly_landing(session, earth(), t0, 90.0);
}

}  // namespace

TEST_CASE("SCvx catch campaign: dispersions and an engine failure", "[gnc][catch][validation]") {
    std::vector<Dispersion> cases;
    for (unsigned seed = 1; seed <= 20; ++seed) cases.push_back(dispersion(seed));
    Dispersion failure;
    failure.engine_failure = true;
    cases.push_back(failure);

    int caught = 0;
    for (size_t i = 0; i < cases.size(); ++i) {
        const auto run = fly_dispersed(cases[i]);
        CAPTURE(i);
        std::printf("case %2zu: %s / %s, offset %.2f m, lateral %.2f m/s, sink %.2f m/s\n", i,
                    sim::to_string(run.tower_phase), sim::to_string(run.outcome), run.offset_m, run.lateral_mps, run.sink_mps);
        std::fflush(stdout);
        CHECK_FALSE(run.tower_struck);  // never, in any case
        if (run.caught_within_tolerance()) {
            ++caught;
        } else {
            // Not caught: it must have ended as a splashdown or a soft touchdown without tower damage.
            CHECK((run.outcome == sim::LandingOutcome::Splashdown || run.outcome == sim::LandingOutcome::Landed));
        }
    }
    std::printf("caught %d of %zu\n", caught, cases.size());
    CHECK(caught >= 19);  // >= 90 % of 21
}

namespace {

// Places a block 3 booster in the coast 60 deg off in roll, 12 km above the site, and returns the angle between its
// body +y and the catch roll reference when the landing burn starts (-1: it never started).
double roll_error_at_ignition_deg() {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const auto rolled = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 60.0 * constants::deg_to_rad);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, {0.0, 0.0, 12000.0},
                                                          {0.0, 0.0, -450.0}, rolled),
                                    t0, gnc::BoosterReturnParameters{}, gnc::ReturnPhase::Coast);
    for (double t = 0.0; t < 120.0;) {
        t += sim::FlightSession::tick_s;
        session.advance_to(t0 + t, 10);
        const auto* status = session.return_status(0);
        if (!status || status->phase != gnc::ReturnPhase::LandingBurn) continue;
        const Vec3 body_y = session.body(0).state().attitude.rotate({0.0, 1.0, 0.0});
        return std::acos(std::clamp(dot(body_y, status->roll_reference_gcrf), -1.0, 1.0)) / constants::deg_to_rad;
    }
    return -1.0;
}

}  // namespace

// Recorded catches light the landing burn already rolled for the catch: the grid fins roll the booster on the way down.
TEST_CASE("A returning booster rolls its pins towards the arms before the landing burn", "[gnc][catch]") {
    const double roll_error_deg = roll_error_at_ignition_deg();
    CAPTURE(roll_error_deg);
    CHECK(roll_error_deg >= 0.0);  // the landing burn started
    CHECK(roll_error_deg < 5.0);
}
