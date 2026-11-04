#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/math/constants.hpp"
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

// Block 3 booster 1.3 km above the catch's old CG target (arm tops 133 m, pins 63 m: 41.5 m higher than 95 / 66.5), falling at 250 m/s, 30 m east / 15 m south of the pad, engines off.
struct Fixture {
    time::Epoch t0;
    std::unique_ptr<sim::FlightSession> session;
    std::unique_ptr<vehicle::FlightBody> body;

    Fixture() {
        test::spice_session();
        t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
        session = std::make_unique<sim::FlightSession>(ephemeris(), earth(), STARSIM_CONFIG_DIR);
        const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2"));
        session->set_launch_site(site);
        body = std::make_unique<vehicle::FlightBody>(
            std::make_unique<vehicle::Vehicle>(test::booster_config("starship_block3", 70000.0), STARSIM_CONFIG_DIR),
            test::state_over_site(site, earth(), t0, {30.0, -15.0, 1341.5}, {4.0, -2.0, -250.0}));
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

// Task 11b: real-time play plans the first plan on the replanner's worker; the vehicle flies on meanwhile. Same snapshot,
// same request: the plan is the one start() makes at once, bit for bit (the commands from it are equal).
TEST_CASE("A first plan on the worker equals the one made at once", "[gnc][scvx_landing][realtime]") {
    Fixture f;
    const auto& env = f.session->environment();
    gnc::ScvxLandingGuidance at_once(f.session->return_site(f.body->vehicle()), f.parameters());
    REQUIRE(at_once.start(*f.body, env, f.t0));
    gnc::ScvxLandingGuidance background(f.session->return_site(f.body->vehicle()), f.parameters());
    background.begin(*f.body, env, f.t0, true);
    std::optional<bool> started;
    for (int k = 0; k < 20000 && !started; ++k) {  // the sim thread polls each tick until it is there (<= 100 s)
        started = background.poll(f.t0 + 0.005 * k);
        if (!started) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    REQUIRE(started.has_value());
    REQUIRE(*started);
    CHECK(background.plan_end_epoch() == at_once.plan_end_epoch());
    CHECK(background.statistics().plans == at_once.statistics().plans);
    const auto a = at_once.update(*f.body, env, f.t0 + 0.2);
    const auto b = background.update(*f.body, env, f.t0 + 0.2);
    CHECK(a.run_groups == b.run_groups);
    CHECK(a.throttle == b.throttle);
    CHECK(a.axis_gcrf.x == b.axis_gcrf.x);
    CHECK(a.axis_gcrf.y == b.axis_gcrf.y);
    CHECK(a.axis_gcrf.z == b.axis_gcrf.z);
    CHECK(a.time_to_go_s == b.time_to_go_s);
}

// Fix round 1 (I2): a first plan after a hold (the booster's unpowered fall) plans from the state the hold leads to and
// takes effect at the hand-over; a solve slower than the hold is waited for there (the synchronous fallback), so the sim
// time between snapshot and plan stays the hold. The plan equals the one made at once from the same hold, bit for bit.
TEST_CASE("A first plan slower than its hold is waited for at the hand-over", "[gnc][scvx_landing][realtime]") {
    Fixture f;
    const auto& env = f.session->environment();
    gnc::FirstPlanHold hold;
    hold.duration_s = 0.5;
    hold.engines_running = false;
    constexpr auto solve_delay = std::chrono::milliseconds(1500);  // an artificially slow solve: longer than the hold
    const auto slow = [plan = gnc::default_plan_function(), solve_delay](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
        std::this_thread::sleep_for(solve_delay);
        return plan(planner, r);
    };
    auto realtime = f.parameters();
    realtime.realtime = true;
    gnc::ScvxLandingGuidance at_once(f.session->return_site(f.body->vehicle()), f.parameters());
    at_once.begin(*f.body, env, f.t0, false, hold);
    REQUIRE_FALSE(at_once.poll(f.t0).has_value());  // decided, but only at the hand-over
    REQUIRE(at_once.poll(f.t0 + hold.duration_s) == std::optional<bool>(true));
    gnc::ScvxLandingGuidance background(f.session->return_site(f.body->vehicle()), realtime, slow);
    const auto wall = std::chrono::steady_clock::now();
    background.begin(*f.body, env, f.t0, true, hold);
    const auto early = background.poll(f.t0 + 0.2);  // before the hand-over: does not wait
    const double early_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall).count();
    CHECK_FALSE(early.has_value());
    CHECK(early_ms < 500.0);
    const auto started = background.poll(f.t0 + hold.duration_s);  // the hand-over: waits for the slow solve
    const double handover_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall).count();
    std::printf("slow first plan: poll before the hand-over %.1f ms, plan at the hand-over after %.0f ms\n", early_ms,
                handover_ms);
    REQUIRE(started == std::optional<bool>(true));
    CHECK(handover_ms >= 1400.0);
    CHECK(background.plan_end_epoch() == at_once.plan_end_epoch());
    CHECK(*at_once.plan_end_epoch() - (f.t0 + hold.duration_s) > 5.0);  // in effect from the hand-over
    const auto a = at_once.update(*f.body, env, f.t0 + hold.duration_s + 0.2);
    const auto b = background.update(*f.body, env, f.t0 + hold.duration_s + 0.2);
    CHECK(a.throttle == b.throttle);
    CHECK(a.axis_gcrf.x == b.axis_gcrf.x);
    CHECK(a.axis_gcrf.y == b.axis_gcrf.y);
    CHECK(a.axis_gcrf.z == b.axis_gcrf.z);
    CHECK(a.time_to_go_s == b.time_to_go_s);
}

// The hold's prediction: an unpowered fall keeps its mass and gains less than g x t of sink (the drag at 250 m/s here
// exceeds the weight: it slows by ~2 m/s); a held thrust burns propellant and brakes.
TEST_CASE("The first plan's hold flies the planning model forward", "[gnc][scvx_landing][realtime]") {
    Fixture f;
    const auto snapshot = gnc::take_descent_snapshot(*f.body, f.session->environment(), f.t0,
                                                     f.session->return_site(f.body->vehicle()),
                                                     gnc::plan_frame(f.session->return_site(f.body->vehicle())),
                                                     f.parameters().snapshot);
    const auto& x0 = snapshot.initial;
    const auto fall = gnc::predict_hold(snapshot.model, x0, {}, false, 1.0);
    CHECK(fall[gnc::descent_index::mass] == x0[gnc::descent_index::mass]);
    const double sink_gain = x0[gnc::descent_index::velocity] - fall[gnc::descent_index::velocity];  // x up
    CHECK(sink_gain < constants::standard_gravity_mps2);  // gravity less the drag
    CHECK(sink_gain > -10.0);
    const double drop_m = x0[gnc::descent_index::position] - fall[gnc::descent_index::position];
    CHECK(drop_m > 240.0);  // ~250 m/s over 1 s
    CHECK(drop_m < 260.0);
    const auto braked = gnc::predict_hold(snapshot.model, x0, {2.0e7, 0.0, 0.0}, true, 1.0);
    CHECK(braked[gnc::descent_index::mass] < x0[gnc::descent_index::mass]);
    CHECK(braked[gnc::descent_index::velocity] > fall[gnc::descent_index::velocity]);
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

TEST_CASE("Abort plans may draw on the whole reserve, the ship's header reserve stays hard", "[gnc][scvx_landing]") {
    Fixture f;
    std::vector<double> abort_soft_reserve_kg;
    gnc::PlanFunction no_catch = [&abort_soft_reserve_kg](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
        auto plan = gnc::default_plan_function()(planner, r);
        if (r.target.goal == gnc::DescentGoal::Catch) plan.converged = false;
        else abort_soft_reserve_kg.push_back(r.settings.limits.soft_reserve_kg);
        return plan;
    };
    const auto parameters = f.parameters();
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), parameters, no_catch);
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    REQUIRE(abort_soft_reserve_kg.size() == 1);
    CHECK(abort_soft_reserve_kg[0] == parameters.abort_reserve_fraction * parameters.snapshot.reserve_kg);
    CHECK(parameters.abort_reserve_fraction == 1.0);
    const auto block3 = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto ship = gnc::ship_landing_parameters(block3, block3.stages.at(1)).scvx;
    CHECK(ship.replan_reserve_fraction == 0.0);
    CHECK(ship.abort_reserve_fraction == 0.0);
}

TEST_CASE("A first plan that needs the final set beyond its tilt limit starts with the abort", "[gnc][scvx_landing]") {
    Fixture f;
    gnc::PlanFunction beyond = [](gnc::DescentPlanner& planner, const gnc::PlanRequest& r) {
        auto plan = gnc::default_plan_function()(planner, r);
        if (r.target.goal == gnc::DescentGoal::Catch) plan.planned_tilt_excess = 0.2;  // ~23 deg over the limit
        return plan;
    };
    gnc::ScvxLandingGuidance guidance(f.session->return_site(f.body->vehicle()), f.parameters(), beyond);
    REQUIRE(guidance.start(*f.body, f.session->environment(), f.t0));
    CHECK(guidance.aborting());
    CHECK(guidance.statistics().failures == 1);
}
