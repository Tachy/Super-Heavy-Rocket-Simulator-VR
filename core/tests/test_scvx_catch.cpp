// The booster placed at landing-burn ignition over Starbase flies the burn on SCvx plans and is caught.

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include <random>

#include "abort_trace.hpp"
#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/catch_tower_sim.hpp"
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

// A booster at ignition has already rolled for the catch on the way down (recorded flights; "A returning booster rolls
// ..." below): state_over_site's body +z is north, turned about the vertical to the catch roll (body +z away from the tower).
math::Quat exact_catch_roll(const sim::LaunchSite& site) {
    return math::Quat::from_axis_angle({0.0, 0.0, 1.0}, -(site.tower->azimuth_rad + constants::pi));
}

// The fixtures' start: the catch roll with the roll error a flown return still has at ignition (the full return in
// test_booster_return.cpp stays within this bound).
math::Quat catch_roll(const sim::LaunchSite& site) {
    return math::Quat::from_axis_angle({0.0, 0.0, 1.0}, test::flown_ignition_roll_error_bound_deg * constants::deg_to_rad) * exact_catch_roll(site);
}

test::CatchRun fly(const Vec3& offset_enu_m, const Vec3& velocity_offset_mps) {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, offset_enu_m + Vec3{0.0, 0.0, h},
                                                          velocity_offset_mps + Vec3{0.0, 0.0, -nominal_sink_mps},
                                                          catch_roll(session.launch_site())),
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

TEST_CASE("A commanded abort late in the landing burn ditches the booster at sea", "[gnc][catch][abort][slow]") {
    // The pilot (or the tower) gives the catch up at a given CG height above the pad: the booster must reach the sea
    // beyond the shoreline, not hover over land until its propellant runs out.
    const double abort_height_m = GENERATE(1500.0, 800.0, 400.0);
    CAPTURE(abort_height_m);
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, {20.0, -10.0, h},
                                                          {0.0, 0.0, -nominal_sink_mps}, catch_roll(session.launch_site())),
                                    t0, gnc::BoosterReturnParameters{}, gnc::ReturnPhase::LandingBurn);
    const auto& booster = session.body(0);
    const auto& site = session.launch_site();
    bool aborted = false;
    bool water_kept = false;
    double t = 0.0;
    for (; t < 150.0 && !booster.impacted() && !booster.resting() && !booster.caught(); t += 0.05) {
        session.advance_to(t0 + t, 100);
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const Vec3 p = sim::site_enu_m(site, transpose(tr.rotation) * booster.state().position_gcrf_m);
        if (!aborted && p.z < abort_height_m) {
            session.abort_landing(0);
            aborted = true;
        }
        if (const auto* g = session.return_guidance(0); g && g->scvx()) water_kept = g->scvx()->sea_required();
    }
    const auto tr = earth().fixed_to_gcrf(session.epoch());
    const Vec3 p = sim::site_enu_m(site, transpose(tr.rotation) * booster.state().position_gcrf_m);
    std::printf("[abort] at %.0f m: %s after %.1f s at %.0f m east / %.0f m north, %.1f t left\n", abort_height_m,
                sim::to_string(session.landing_outcome(0)), t, p.x, p.y, booster.vehicle().propellant_kg() / 1e3);
    CHECK(aborted);
    CHECK_FALSE(session.tower()->tower_struck());
    // The reach estimate decides at the abort. From 1500 and 800 m the water is in reach, with propellant left. From 400 m
    // the flight there would end on the last propellant, or short of it (flown: 55 m short, 2.9 t): the water is given up
    // and the booster comes down soft on land instead.
    const bool water_expected = abort_height_m > 400.0;
    CHECK(water_kept == water_expected);
    if (water_expected) {
        CHECK(session.landing_outcome(0) == sim::LandingOutcome::Splashdown);
        CHECK(booster.vehicle().propellant_kg() > 1000.0);
    } else {
        CHECK(session.landing_outcome(0) == sim::LandingOutcome::Landed);
    }
}

// Diagnostics (STARSIM_ABORT_TRACE_DIR): the hull axis of the commanded aborts, for tools/validation/abort_clearance.py.
TEST_CASE("Booster abort clearance trace", "[.abort_clearance]") {
    const double abort_height_m = GENERATE(1500.0, 800.0, 400.0);
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, {20.0, -10.0, h},
                                                          {0.0, 0.0, -nominal_sink_mps}, catch_roll(session.launch_site())),
                                    t0, gnc::BoosterReturnParameters{}, gnc::ReturnPhase::LandingBurn);
    test::AbortTrace trace("booster_" + std::to_string(static_cast<int>(abort_height_m)));
    const auto& booster = session.body(0);
    bool aborted = false;
    for (double t = 0.0; t < 150.0 && !booster.impacted() && !booster.resting() && !booster.caught(); t += 0.05) {
        session.advance_to(t0 + t, 100);
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const Vec3 p = sim::site_enu_m(session.launch_site(), transpose(tr.rotation) * booster.state().position_gcrf_m);
        if (!aborted && p.z < abort_height_m) {
            session.abort_landing(0);
            aborted = true;
        }
        trace.sample(session, earth(), 0, t);
    }
    std::printf("[clearance] booster abort at %.0f m: %s, struck %d, %.1f t left\n", abort_height_m,
                sim::to_string(session.landing_outcome(0)), session.tower()->tower_struck() ? 1 : 0, booster.vehicle().propellant_kg() / 1e3);
}

TEST_CASE("An abort the guidance cannot fly leaves the tower's arms alone", "[gnc][catch][abort]") {
    // Review finding: without water to ditch in, or once the booster is down, the abort is refused - the arms must not
    // open under a booster that keeps aiming at them.
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2"));
    site.sea.reset();  // a tower site without a coastline
    session.set_launch_site(site);
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    session.place_returning_booster(cfg, test::state_over_site(session.launch_site(), earth(), t0, {20.0, -10.0, h},
                                                               {0.0, 0.0, -nominal_sink_mps}, catch_roll(session.launch_site())),
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
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    const auto parameters = gnc::BoosterReturnParameters{};
    const auto nominal = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, nominal, earth(), t0, nominal_sink_mps, parameters);
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg * d.mass_factor);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, d.offset_m + Vec3{0.0, 0.0, h},
                                                          d.velocity_mps + Vec3{0.0, 0.0, -nominal_sink_mps}, d.tilt * catch_roll(session.launch_site())),
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

TEST_CASE("SCvx catch campaign: dispersions and an engine failure", "[gnc][catch][validation][slow]") {
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
double roll_error_at_ignition_deg(double start_roll_deg = 60.0) {
    const auto t0 = start();
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    // Measured from the catch roll (body +z away from the tower).
    const auto rolled = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, start_roll_deg * constants::deg_to_rad) *
                        exact_catch_roll(session.launch_site());
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

// The fixtures start at the catch roll; a flown return arrives within the bound, either side.
TEST_CASE("SCvx catches with the residual roll error of a flown return", "[gnc][catch]") {
    const double sign = GENERATE(1.0, -1.0);
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    const auto residual = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, sign * test::flown_ignition_roll_error_bound_deg * constants::deg_to_rad);
    session.place_returning_booster(cfg, test::state_over_site(session.launch_site(), earth(), t0, {20.0, -10.0, h},
                                                               {0.0, 0.0, -nominal_sink_mps}, residual * exact_catch_roll(session.launch_site())),
                                    t0, gnc::BoosterReturnParameters{}, gnc::ReturnPhase::LandingBurn);
    const auto run = test::fly_landing(session, earth(), t0, 90.0);
    CHECK(run.caught_within_tolerance());
    CHECK_FALSE(run.tower_struck);
}

// The roll is one-way (the booster's fin away from the tower): the worst start, half a turn off, either side of it.
TEST_CASE("A booster half a turn off in roll still lights the burn rolled for the catch", "[gnc][catch]") {
    const double start_roll_deg = GENERATE(180.0, 179.0, -179.0);
    CAPTURE(start_roll_deg);
    const double roll_error_deg = roll_error_at_ignition_deg(start_roll_deg);
    CAPTURE(roll_error_deg);
    std::printf("[roll] start %.0f deg: %.2f deg at ignition\n", start_roll_deg, roll_error_deg);
    CHECK(roll_error_deg >= 0.0);
    CHECK(roll_error_deg < 5.0);
}

// The full guided landing burn ends with the fin gap at the tower.
TEST_CASE("The landing burn ends with the 90 deg fin away from the tower", "[gnc][catch]") {
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    const auto cfg = test::booster_config("starship_block3", nominal_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, nominal_sink_mps, gnc::BoosterReturnParameters{});
    session.place_returning_booster(cfg, test::state_over_site(session.launch_site(), earth(), t0, {20.0, -10.0, h},
                                                               {0.0, 0.0, -nominal_sink_mps}, catch_roll(session.launch_site())),
                                    t0, gnc::BoosterReturnParameters{}, gnc::ReturnPhase::LandingBurn);
    const auto run = test::fly_landing(session, earth(), t0, 90.0);
    CHECK(run.caught_within_tolerance());
    const auto tr = earth().fixed_to_gcrf(session.epoch());
    const auto& g = session.tower()->geometry();
    const Vec3 fin_90_site = transpose(g.site_to_itrf()) * transpose(tr.rotation) * session.body(0).state().attitude.rotate({0.0, 0.0, 1.0});
    CAPTURE(fin_90_site.x, fin_90_site.y, fin_90_site.z);
    CHECK(dot(fin_90_site, g.tower_direction()) < -std::cos(3.0 * constants::deg_to_rad));
}
