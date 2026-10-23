#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

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

// Block 3 booster 1.3 km over Starbase, falling at 250 m/s, 30 m east / 15 m south of the pad, engines off.
struct Fixture {
    time::Epoch t0;
    std::unique_ptr<sim::FlightSession> session;
    std::unique_ptr<vehicle::FlightBody> body;

    Fixture() {
        test::spice_session();
        t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
        session = std::make_unique<sim::FlightSession>(ephemeris(), earth(), STARSIM_CONFIG_DIR);
        const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
        session->set_launch_site(site);
        body = std::make_unique<vehicle::FlightBody>(
            std::make_unique<vehicle::Vehicle>(test::booster_config("starship_block3", 70000.0), STARSIM_CONFIG_DIR),
            test::state_over_site(site, earth(), t0, {30.0, -15.0, 1300.0}, {4.0, -2.0, -250.0}));
    }

    gnc::ScvxLandingParameters parameters() const {
        gnc::ScvxLandingParameters p;
        p.snapshot.phase_groups = {std::vector<std::string>{"booster/center", "booster/divert", "booster/middle"},
                             std::vector<std::string>{"booster/center", "booster/divert"},
                             std::vector<std::string>{"booster/center"}};
        return p;
    }
};

}  // namespace

TEST_CASE("SCvx landing guidance starts with a plan and commands the ignition engines", "[gnc][scvx_landing]") {
    Fixture f;
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters());
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    const auto cmd = guidance.update(*f.body, f.session->environment(), f.t0);
    CHECK(cmd.run_groups == std::vector<std::string>{"booster/center", "booster/divert", "booster/middle"});
    CHECK(cmd.throttle > 0.0);
    CHECK(cmd.throttle <= 1.0);
    CHECK_FALSE(cmd.abort);
    CHECK(cmd.time_to_go_s > 5.0);
    CHECK(guidance.statistics().plans == 1);
    std::printf("first plan: %.0f ms\n", guidance.statistics().last_solve_ms);
}

TEST_CASE("A re-plan from a snapshot takes effect half a second later", "[gnc][scvx_landing]") {
    Fixture f;
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters());
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    const auto& env = f.session->environment();
    guidance.update(*f.body, env, f.t0 + 1.0);  // snapshot (the body is not moved: a static re-plan)
    guidance.update(*f.body, env, f.t0 + 1.4);
    CHECK(guidance.statistics().plans == 1);
    guidance.update(*f.body, env, f.t0 + 1.5);
    CHECK(guidance.statistics().plans == 2);
}

TEST_CASE("Repeated failed re-plans switch to an abort", "[gnc][scvx_landing]") {
    Fixture f;
    int calls = 0;
    double abort_min_phase_s = -1.0;
    gnc::PlanFunction failing = [&calls, &abort_min_phase_s](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
        if (r.target.goal == gnc::DescentGoal::Abort) abort_min_phase_s = r.settings.limits.min_phase_s[0];
        auto plan = gnc::default_plan_function()(planner, r);
        if (calls++ > 0 && r.target.goal == gnc::DescentGoal::Catch) {  // re-plans fail: the catch is unreachable
            plan.converged = false;
            plan.virtual_control = 5.0;
        }
        return plan;
    };
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters(), failing);
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    const auto& env = f.session->environment();
    for (double t = 1.0; t <= 3.5 && !guidance.aborting(); t += 0.5) guidance.update(*f.body, env, f.t0 + t);
    CHECK(guidance.aborting());
    CHECK(guidance.statistics().failures == 2);
    CHECK(guidance.update(*f.body, env, f.t0 + 4.0).abort);
    for (double t = 4.5; t <= 6.0; t += 0.5) guidance.update(*f.body, env, f.t0 + t);
    // An abort re-plans mid-flight: no minimum phase durations (a forced second on 13 engines low down would climb).
    CHECK(abort_min_phase_s == 0.0);
}

TEST_CASE("An unreachable catch at ignition starts with an abort plan to the sea", "[gnc][scvx_landing]") {
    // Super Heavy has no legs: if the catch cannot be planned, the booster ditches at sea instead (arms open).
    Fixture f;
    gnc::PlanFunction no_catch = [](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
        auto plan = gnc::default_plan_function()(planner, r);
        if (r.target.goal == gnc::DescentGoal::Catch) plan.converged = false;
        return plan;
    };
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters(), no_catch);
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    CHECK(guidance.aborting());
    CHECK(guidance.update(*f.body, f.session->environment(), f.t0).abort);
    CHECK(guidance.statistics().plans == 2);
    CHECK(guidance.statistics().failures == 1);
}

TEST_CASE("Landing throttle delivers the thrust, counting engines still running down", "[gnc][scvx_landing]") {
    Fixture f;
    auto& body = *f.body;
    const auto& env = f.session->environment();
    vehicle::VehicleCommand all;
    all.groups = {{"booster/center", true, 1.0}, {"booster/divert", true, 1.0}, {"booster/middle", true, 1.0}};
    for (int i = 0; i < 800; ++i) body.step(env, f.t0 + i * 0.005, 0.005, all);  // 4 s: all 13 engines spooled up
    vehicle::VehicleCommand centre;
    centre.groups = {{"booster/center", true, 1.0}, {"booster/divert", false, 1.0}, {"booster/middle", false, 1.0}};
    for (int i = 0; i < 50; ++i) body.step(env, f.t0 + 4.0 + i * 0.005, 0.005, centre);  // middle engines half run down
    const std::vector<std::string> groups{"booster/center"};
    const double pressure = 1.0e5;
    double centre_full = 0.0, residual = 0.0;
    for (const auto& e : body.vehicle().stages().front().engines) {
        if (e.group_id == "booster/center") centre_full += e.spool * e.performance->thrust_n(1.0, pressure);
        else if (e.group_id == "booster/middle" || e.group_id == "booster/divert") residual += e.spool * e.performance->thrust_n(e.throttle, pressure);
    }
    REQUIRE(residual > 0.0);
    // Low throttle, where the nozzle's pressure loss (not proportional to the throttle) matters most.
    for (const double fraction : {0.6, 0.35}) {
        CAPTURE(fraction);
        const double thrust = residual + fraction * centre_full;
        const double throttle = gnc::landing_throttle(body.vehicle(), groups, thrust, pressure);
        double delivered = residual;
        for (const auto& e : body.vehicle().stages().front().engines) {
            if (e.group_id == "booster/center") delivered += e.spool * e.performance->thrust_n(throttle, pressure);
        }
        CHECK(std::abs(delivered - thrust) < 1e-9 * thrust);
    }
}

TEST_CASE("Slowly converging re-plans keep the catch and the previous plan", "[gnc][scvx_landing]") {
    Fixture f;
    int calls = 0;
    gnc::PlanFunction slow = [&calls](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
        auto plan = gnc::default_plan_function()(planner, r);
        if (calls++ > 0) {  // out of iterations, but the dynamics nearly hold: not an unreachable catch
            plan.converged = false;
            plan.virtual_control = 0.05;
        }
        return plan;
    };
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters(), slow);
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    const auto& env = f.session->environment();
    for (double t = 1.0; t <= 4.0; t += 0.5) guidance.update(*f.body, env, f.t0 + t);
    CHECK_FALSE(guidance.aborting());
    CHECK(guidance.statistics().failures >= 2);
}

TEST_CASE("Re-plans may draw on part of the propellant reserve, the first plan keeps it", "[gnc][scvx_landing]") {
    Fixture f;
    std::vector<double> soft_reserve_kg;
    gnc::PlanFunction recording = [&soft_reserve_kg](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
        soft_reserve_kg.push_back(r.settings.limits.soft_reserve_kg);
        return gnc::default_plan_function()(planner, r);
    };
    const auto parameters = f.parameters();
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), parameters, recording);
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    guidance.update(*f.body, f.session->environment(), f.t0 + 1.0);
    guidance.update(*f.body, f.session->environment(), f.t0 + 1.5);
    REQUIRE(soft_reserve_kg.size() == 2);
    CHECK(soft_reserve_kg[0] == 0.0);
    CHECK(soft_reserve_kg[1] == parameters.replan_reserve_fraction * parameters.snapshot.reserve_kg);
    CHECK(soft_reserve_kg[1] > 0.0);
}
