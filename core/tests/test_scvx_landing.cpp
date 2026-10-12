#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <memory>

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
        p.snapshot.ignition_groups = {"booster/center", "booster/middle"};
        p.snapshot.final_groups = {"booster/center"};
        return p;
    }
};

}  // namespace

TEST_CASE("SCvx landing guidance starts with a plan and commands the ignition engines", "[gnc][scvx_landing]") {
    Fixture f;
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters());
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    const auto cmd = guidance.update(*f.body, f.session->environment(), f.t0);
    CHECK(cmd.run_groups == std::vector<std::string>{"booster/center", "booster/middle"});
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
    gnc::PlanFunction failing = [&calls](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
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
}

TEST_CASE("Landing throttle counts the thrust of engines still running down", "[gnc][scvx_landing]") {
    Fixture f;
    auto& body = *f.body;
    const auto& env = f.session->environment();
    vehicle::VehicleCommand all;
    all.groups = {{"booster/center", true, 1.0}, {"booster/middle", true, 1.0}};
    for (int i = 0; i < 800; ++i) body.step(env, f.t0 + i * 0.005, 0.005, all);  // 4 s: all 13 engines spooled up
    vehicle::VehicleCommand centre;
    centre.groups = {{"booster/center", true, 1.0}, {"booster/middle", false, 1.0}};
    for (int i = 0; i < 50; ++i) body.step(env, f.t0 + 4.0 + i * 0.005, 0.005, centre);  // middle engines half run down
    const std::vector<std::string> groups{"booster/center"};
    const double pressure = 1.0e5;
    double centre_full = 0.0, residual = 0.0;
    for (const auto& e : body.vehicle().stages().front().engines) {
        if (e.group_id == "booster/center") centre_full += e.spool * e.performance->thrust_n(1.0, pressure);
        else if (e.group_id == "booster/middle") residual += e.spool * e.performance->thrust_n(e.throttle, pressure);
    }
    REQUIRE(residual > 0.0);
    const double thrust = residual + 0.6 * centre_full;
    CHECK(std::abs(gnc::landing_throttle(body.vehicle(), groups, thrust, pressure) - 0.6) < 1e-9);
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
