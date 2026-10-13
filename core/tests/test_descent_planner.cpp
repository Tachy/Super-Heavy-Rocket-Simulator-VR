#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/gnc/descent_planner.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Quat;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

// Block 3 booster with ~90 t of propellant left, model built like the flight will build it.
struct Booster {
    std::unique_ptr<vehicle::Vehicle> vehicle;
    environment::Us76Atmosphere atmosphere;
    DescentModel model;
};

std::unique_ptr<Booster> block3() {
    auto b = std::make_unique<Booster>();
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.resize(1);
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg *= 0.025;  // ~90 t left
    b->vehicle = std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR);
    DescentModelInputs in;
    in.vehicle = b->vehicle.get();
    in.ignition_groups = {"booster/center", "booster/middle"};
    in.final_groups = {"booster/center"};
    in.atmosphere = &b->atmosphere;
    in.site_height_m = 10.0;
    const double lat = 26.0 * deg;
    in.gravity_mps2 = {-9.79, 0.0, 0.0};
    in.earth_rate_radps = Vec3{std::sin(lat), 0.0, std::cos(lat)} * 7.2921150e-5;
    in.earth_center_m = {-6.374e6, 0.0, 0.0};
    b->model = make_descent_model(in);
    return b;
}

DescentTarget catch_target(const Booster& b, const Quat& initial_attitude) {
    DescentTarget t;
    t.goal = DescentGoal::Catch;
    const double pin_station = b.vehicle->stages()[0].config.catch_pins.station_m;
    const double cg = b.vehicle->mass_properties().cg_station_m;
    t.position_m = {96.0 - (pin_station - cg), 0.0, 0.0};  // pins 1 m above the 95 m arm tops
    t.velocity_mps = {-1.0, 0.0, 0.0};
    (void)initial_attitude;  // the catch ends upright with any roll (the tracking controller rolls the pins)
    return t;
}

ScvxSettings settings(const Booster& b) {
    ScvxSettings s;
    s.limits.min_mass_kg = b.vehicle->mass_properties().mass_kg - b.vehicle->propellant_kg() + 5000.0;
    return s;
}

// Ignition-like state: 1.2 km up, falling at 250 m/s, 30 m off to the east drifting 4 m/s, slightly tilted.
DescentState ignition_state(const Booster& b) {
    return make_state(b.vehicle->mass_properties().mass_kg, {1200.0, 30.0, -15.0}, {-250.0, 4.0, -2.0},
                      Quat::from_axis_angle({0.0, 0.0, 1.0}, 2.0 * deg), {});
}

}  // namespace

TEST_CASE("SCvx plans a catch from an ignition-like state", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto x0 = ignition_state(*b);
    const auto target = catch_target(*b, state_attitude(x0));
    const auto s = settings(*b);
    const auto start = std::chrono::steady_clock::now();
    const auto plan = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("scvx: %d iterations, vc %.2e, step %.2e, durations %.1f + %.1f s, %.0f ms\n", plan.iterations,
                    plan.virtual_control, plan.last_step, plan.reference.duration_s[0], plan.reference.duration_s[1], ms);
    }
    REQUIRE(plan.converged);
    // Constraints at the nodes.
    for (size_t k = 0; k < plan.reference.u.size(); ++k) {
        const Vec3 u = plan.reference.u[k];
        const int phase = k < static_cast<size_t>(s.nodes_per_phase - 1) ? 0 : 1;
        CHECK(norm(u) <= b->model.engine_sets[static_cast<size_t>(phase)].thrust_max_n * (1.0 + 1e-6) + 1.0);
        CHECK(std::hypot(u.y, u.z) <= std::tan(b->model.engine_sets[0].gimbal_max_rad) * u.x * (1.0 + 1e-6) + 1.0);
        const Vec3 w = state_rate(plan.reference.x[k]);
        CHECK(std::hypot(w.y, w.z) <= s.limits.rate_max_radps[static_cast<size_t>(phase)] + 1e-6);  // pitch/yaw
    }
    // Open loop: the plan's thrust flown through the nonlinear model ends at the catch point.
    DescentState x = x0;
    const int n = s.nodes_per_phase;
    for (int k = 0; k < 2 * n - 2; ++k) {
        const int phase = k < n - 1 ? 0 : 1;
        x = linearize_interval(b->model, phase, x, plan.reference.u[static_cast<size_t>(k)],
                               plan.reference.u[static_cast<size_t>(k + 1)],
                               plan.reference.duration_s[static_cast<size_t>(phase)], 1.0 / (n - 1), 20)
                .propagated;
    }
    CHECK(norm(state_position(x) - target.position_m) < 1.0);
    CHECK(norm(state_velocity(x) - target.velocity_mps) < 0.5);
}

TEST_CASE("SCvx planning is deterministic", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto x0 = ignition_state(*b);
    const auto target = catch_target(*b, state_attitude(x0));
    const auto a = plan_descent(b->model, x0, 0, target, settings(*b), nullptr, 0.0);
    const auto c = plan_descent(b->model, x0, 0, target, settings(*b), nullptr, 0.0);
    REQUIRE(a.reference.x.size() == c.reference.x.size());
    for (size_t k = 0; k < a.reference.x.size(); ++k) CHECK(a.reference.x[k] == c.reference.x[k]);
}

TEST_CASE("SCvx re-plans from the final phase with a warm start", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto x0 = ignition_state(*b);
    const auto target = catch_target(*b, state_attitude(x0));
    const auto s = settings(*b);
    const auto first = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(first.converged);
    const double t = first.reference.duration_s[0] + 1.0;  // one second into phase 1
    const auto again = plan_descent(b->model, first.state_at(t), 1, target, s, &first, t);
    CHECK(again.converged);
    CHECK(again.reference.duration_s[0] == 0.0);
    CHECK(again.iterations <= first.iterations);
}

TEST_CASE("Terminal attitude takes the quaternion sign nearest the start", "[gnc][descent]") {
    const Quat start = Quat::from_axis_angle({1.0, 0.0, 0.0}, 3.0);  // rolled almost half a turn
    const Quat target = Quat::from_axis_angle({1.0, 0.0, 0.0}, 3.14159265358979);
    const Quat chosen = nearest_sign(target, start);
    CHECK(chosen.w * start.w + chosen.x * start.x + chosen.y * start.y + chosen.z * start.z >= 0.0);
}

TEST_CASE("SCvx plans an abort to the sea clear of the tower", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto x0 = ignition_state(*b);
    DescentTarget target;
    target.goal = DescentGoal::Abort;
    target.ground_cg_height_m = b->vehicle->mass_properties().cg_station_m;
    target.sea_target_m = {0.0, 2000.0, 0.0};  // 2 km east
    target.keep_out_center_m = {0.0, 0.0, -30.0};
    target.keep_out_radius_m = 25.0;
    auto s = settings(*b);
    s.limits.glide_slope_rad = 0.0;
    const auto plan = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const auto end = plan.reference.x.back();
    CHECK(std::abs(end[descent_index::position] - target.ground_cg_height_m) < 1e-3);
    CHECK(state_position(end).y > 30.0);  // moved seaward
    for (const auto& x : plan.reference.x) {
        const Vec3 r = state_position(x) - target.keep_out_center_m;
        CHECK(std::hypot(r.y, r.z) >= target.keep_out_radius_m - 0.5);
    }
}

TEST_CASE("SCvx plans a catch whatever the initial roll", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    for (const double roll_deg : {90.0, -90.0, 20.0}) {
        CAPTURE(roll_deg);
        auto x0 = ignition_state(*b);
        const Quat q = Quat::from_axis_angle({1.0, 0.0, 0.0}, roll_deg * deg) * state_attitude(x0);
        x0 = make_state(x0[descent_index::mass], state_position(x0), state_velocity(x0), q, {});
        const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, q), s, nullptr, 0.0);
        CHECK(plan.converged);
    }
}

TEST_CASE("SCvx re-plans from states slightly outside the path cones", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    auto x0 = ignition_state(*b);
    x0[descent_index::rate + 1] = 0.12;  // above the 0.1 rad/s rate bound
    CHECK(plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0).converged);

    const auto first = plan_descent(b->model, ignition_state(*b), 0, catch_target(*b, state_attitude(ignition_state(*b))), s,
                                    nullptr, 0.0);
    REQUIRE(first.converged);
    const double t = first.reference.duration_s[0] + 1.0;
    auto x1 = first.state_at(t);
    const Quat tilted = Quat::from_axis_angle({0.0, 0.0, 1.0}, 11.0 * deg);  // beyond phase 1's 10 deg tilt
    x1 = make_state(x1[descent_index::mass], state_position(x1), state_velocity(x1), tilted, state_rate(x1));
    CHECK(plan_descent(b->model, x1, 1, catch_target(*b, tilted), s, &first, t).converged);
}

TEST_CASE("A re-plan does not stretch a nearly finished phase to the minimum duration", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto target = catch_target(*b, state_attitude(x0));
    const auto first = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(first.converged);
    const double t = first.reference.duration_s[0] - 0.5;  // half a second of the 13-engine phase left
    const auto again = plan_descent(b->model, first.state_at(t), 0, target, s, &first, t);
    REQUIRE(again.converged);
    CHECK(again.reference.duration_s[0] < 0.9);
}

TEST_CASE("Planning model mass flow matches the engines at minimum throttle", "[gnc][descent]") {
    const auto b = block3();
    const auto& set = b->model.engine_sets[1];  // 3 centre engines
    const auto x = make_state(3.0e5, {500.0, 0.0, 0.0}, {-50.0, 0.0, 0.0}, Quat{1.0, 0.0, 0.0, 0.0}, {});
    const double model_flow = -b->model.derivative(x, {set.thrust_min_n, 0.0, 0.0}, 1)[descent_index::mass];
    double true_flow = 0.0;
    for (const auto& e : b->vehicle->stages()[0].engines) {
        if (e.group_id == "booster/center") true_flow += e.performance->mass_flow_kgps(e.performance->config().throttle_min);
    }
    CHECK(std::abs(model_flow - true_flow) < 1e-6 * true_flow);
}

TEST_CASE("A planner reuses one solver per start phase and goal with unchanged results", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto target = catch_target(*b, state_attitude(x0));
    DescentPlanner planner;
    const auto first = planner.plan(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(first.converged);
    const double t = first.reference.duration_s[0] + 1.0;
    const auto again = planner.plan(b->model, first.state_at(t), 1, target, s, &first, t);
    const auto twice = planner.plan(b->model, first.state_at(t), 1, target, s, &first, t);
    CHECK(planner.solver_count() == 2);  // (phase 0, catch) and (phase 1, catch)
    const auto fresh = plan_descent(b->model, first.state_at(t), 1, target, s, &first, t);
    CHECK(twice.reference.x == fresh.reference.x);
    CHECK(again.reference.x == fresh.reference.x);
}

TEST_CASE("SCvx stops early when the virtual control stalls", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    // 200 m up at 250 m/s: no thrust can stop this before the catch point.
    const auto x0 = make_state(b->vehicle->mass_properties().mass_kg, {200.0, 0.0, 0.0}, {-250.0, 0.0, 0.0}, Quat{}, {});
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, Quat{}), s, nullptr, 0.0);
    CHECK_FALSE(plan.converged);
    CHECK(plan.iterations <= s.stall_iterations + 2);
}

namespace {

double tilt_of(const DescentState& x) {
    return 2.0 * std::asin(std::min(1.0, std::hypot(x[descent_index::attitude + 2], x[descent_index::attitude + 3])));
}

}  // namespace

TEST_CASE("The tilt limit tapers towards the catch", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const int last = static_cast<int>(plan.reference.x.size()) - 1;
    for (int from_end = 0; from_end < s.limits.tilt_taper_nodes; ++from_end) {
        const double limit = s.limits.terminal_tilt_rad +
                             (s.limits.tilt_max_rad[1] - s.limits.terminal_tilt_rad) * from_end / s.limits.tilt_taper_nodes;
        CHECK(tilt_of(plan.reference.x[static_cast<size_t>(last - from_end)]) <= limit + 1e-6);
    }
}

TEST_CASE("Thrust follows the engines' start-up at ignition", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    s.limits.phase0_initial_spool = 0.0;
    s.limits.phase0_startup_s = 3.0;
    const auto x0 = ignition_state(*b);
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const double t_max = b->model.engine_sets[0].thrust_max_n;
    for (int k = 0; k < s.nodes_per_phase - 1; ++k) {
        const double t = plan.node_time_s(k);
        // The bound uses the previous iterate's node times: a converged plan's durations differ by its last step.
        CHECK(norm(plan.reference.u[static_cast<size_t>(k)]) <= t_max * (std::min(1.0, t / 3.0) + 1e-3));
    }
}

TEST_CASE("An abort plan stays above the ground at every node", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    s.limits.glide_slope_rad = 0.0;
    DescentTarget target;
    target.goal = DescentGoal::Abort;
    target.ground_cg_height_m = b->vehicle->mass_properties().cg_station_m;
    target.sea_target_m = {0.0, 2000.0, 0.0};
    target.keep_out_center_m = {0.0, 0.0, -30.0};
    target.keep_out_radius_m = 25.0;
    // Low and slow: the cheapest path to the sea would skim the ground.
    const auto x0 = make_state(b->vehicle->mass_properties().mass_kg, {120.0, 0.0, 0.0}, {-20.0, 15.0, 0.0}, Quat{}, {});
    const auto plan = plan_descent(b->model, x0, 1, target, s, nullptr, 0.0);
    REQUIRE(plan.converged);
    for (const auto& x : plan.reference.x) CHECK(x[descent_index::position] >= target.ground_cg_height_m - 1e-3);
}

TEST_CASE("A warm start begins at the measured snapshot", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto target = catch_target(*b, state_attitude(x0));
    const auto first = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(first.converged);
    const double t = 2.0;
    auto snapshot = first.state_at(t);
    snapshot[descent_index::position + 1] += 20.0;  // 20 m east of the old plan
    snapshot[descent_index::velocity + 2] -= 3.0;
    const int phase = first.phase_at(t);
    const auto ref = warm_start_reference(first, t, phase, s.nodes_per_phase, snapshot);
    CHECK(ref.x[0] == snapshot);
    const auto again = plan_descent(b->model, snapshot, phase, target, s, &first, t);
    CHECK(again.converged);
}

TEST_CASE("SCvx re-plans while the booster rolls (no roll authority in the model)", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    auto x0 = ignition_state(*b);
    for (const double roll_rate : {0.01, 0.15}) {  // 0.15 rad/s: above the pitch/yaw rate bound
        CAPTURE(roll_rate);
        x0[descent_index::rate] = roll_rate;  // the tracking controller rolls the pins towards the arms
        const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
        CHECK(plan.converged);
    }
}

TEST_CASE("Catch plans fly the final engine set straight above the catch point", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);  // 30 m east, 15 m south: the divert happens on the ignition set
    const auto target = catch_target(*b, state_attitude(x0));
    const auto plan = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const double arms_m = 60.0;  // the hull slides down between the arms for the last ~60 m
    int inside = 0;
    for (size_t k = static_cast<size_t>(s.nodes_per_phase - 1); k < plan.reference.x.size(); ++k) {
        const auto& x = plan.reference.x[k];
        if (x[descent_index::position] > target.position_m.x + arms_m) continue;
        ++inside;
        CHECK(std::hypot(x[descent_index::position + 1], x[descent_index::position + 2]) <= 0.3);
    }
    CHECK(inside >= 2);
}


TEST_CASE("Catch plans stay in a vertical corridor while the hull passes the arms", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    // Within the campaign's ignition dispersion (+-10 m): larger offsets are partly left to the final set (soft).
    const auto x0 = make_state(b->vehicle->mass_properties().mass_kg, {1200.0, 8.0, -6.0}, {-250.0, 4.0, -2.0},
                               Quat::from_axis_angle({0.0, 0.0, 1.0}, 2.0 * deg), {});
    auto target = catch_target(*b, state_attitude(x0));
    target.corridor_top_m = target.position_m.x + 70.0;
    target.corridor_radius_m = 1.5;
    target.final_set_radius_m = 3.0;  // the final engine set flies above the catch point (divert on 13 engines)
    const auto plan = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(plan.converged);
    for (size_t k = static_cast<size_t>(s.nodes_per_phase - 1); k < plan.reference.x.size(); ++k) {
        const auto& x = plan.reference.x[k];
        CHECK(std::hypot(x[descent_index::position + 1], x[descent_index::position + 2]) <= target.final_set_radius_m + 1e-3);
    }
    int inside = 0;
    for (const auto& x : plan.reference.x) {
        if (x[descent_index::position] >= target.corridor_top_m - 2.0) continue;  // linearised about the last iterate
        ++inside;
        CHECK(std::hypot(x[descent_index::position + 1], x[descent_index::position + 2]) <= target.corridor_radius_m + 1e-3);
    }
    CHECK(inside >= 3);
}

TEST_CASE("A feasible plan at the iteration limit is flyable", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    s.fresh_max_iterations = 5;          // stopped before the fuel optimum
    s.final_mass_tolerance_kg = 1.0e-6;  // and before the final mass settles
    s.step_tolerance = 1.0e-9;
    const auto x0 = ignition_state(*b);
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
    CHECK(plan.iterations == 5);
    CHECK(plan.virtual_control < s.virtual_control_tolerance);
    CHECK(plan.nonlinear_defect < s.defect_tolerance);
    CHECK(plan.converged);
}

TEST_CASE("A re-plan from outside the corridor stays feasible and closes in", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    auto target = catch_target(*b, Quat{});
    target.corridor_top_m = target.position_m.x + 70.0;
    target.corridor_radius_m = 1.5;
    target.final_set_radius_m = 3.0;
    // On the final engines, 8 m off the catch axis 100 m above the catch point: no plan can reach the corridor at once.
    const auto x0 = make_state(b->vehicle->mass_properties().mass_kg - 30000.0, {target.position_m.x + 100.0, 8.0, 0.0},
                               {-15.0, 0.0, 0.0}, Quat{}, {});
    const auto plan = plan_descent(b->model, x0, 1, target, s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const auto& end = plan.reference.x.back();
    CHECK(std::hypot(end[descent_index::position + 1], end[descent_index::position + 2]) < 1e-3);  // still the catch point
}

TEST_CASE("A re-plan right after the engine switch tolerates the turn rate of the ignition set", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    const auto target = catch_target(*b, Quat{});
    // On the final engines at the switch of the catch profile (~600 m, ~72 m/s), tilted 8 deg and still turning at
    // 0.135 rad/s (above the final set's 0.1 rad/s): the three engines need longer than one node interval to stop that.
    const Quat tilted = Quat::from_axis_angle({0.0, 0.0, 1.0}, 8.0 * deg);
    const auto x0 = make_state(b->vehicle->mass_properties().mass_kg - 30000.0, {target.position_m.x + 600.0, 3.0, 0.0},
                               {-72.0, -2.0, 0.0}, tilted, {0.0, 0.0, 0.135});
    const auto plan = plan_descent(b->model, x0, 1, target, s, nullptr, 0.0);
    CHECK(plan.converged);
}

TEST_CASE("A centred ignition with engines still starting plans with corridor and soft limits", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    s.limits.phase0_initial_spool = 0.0;
    s.limits.phase0_startup_s = 3.0;
    auto target = catch_target(*b, Quat{});
    target.corridor_top_m = target.position_m.x + 70.0;
    target.corridor_radius_m = 1.5;
    target.final_set_radius_m = 3.0;
    for (const double height : {1450.0, 1500.0, 1550.0}) {  // ignition of the catch profile at 260 m/s
        CAPTURE(height);
        const auto x0 = make_state(b->vehicle->mass_properties().mass_kg, {height, 0.0, 0.0}, {-260.0, 0.0, 0.0}, Quat{}, {});
        const auto plan = plan_descent(b->model, x0, 0, target, s, nullptr, 0.0);
        CHECK(plan.converged);
    }
}

TEST_CASE("Planned thrust changes no faster than the engines can throttle", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const int boundary = s.nodes_per_phase - 1;
    for (int k = 0; k + 1 < static_cast<int>(plan.reference.u.size()); ++k) {
        if (k == boundary - 1) continue;  // the engine switch: the ignition set runs down
        const int phase = k < boundary ? 0 : 1;
        const auto& set = b->model.engine_sets[static_cast<size_t>(phase)];
        const double dt = plan.node_time_s(k + 1) - plan.node_time_s(k);
        const double change = std::abs(norm(plan.reference.u[static_cast<size_t>(k + 1)]) - norm(plan.reference.u[static_cast<size_t>(k)]));
        CAPTURE(k);
        CHECK(change <= set.throttle_rate_per_s * set.thrust_max_n * dt * 1.05 + 1.0e3);
    }
}

TEST_CASE("Plans end gently: thrust near the minimum at the catch", "[gnc][descent][scvx]") {
    // The three final engines exceed the weight even at minimum throttle and throttle at a limited rate: a plan
    // ending in hard braking would turn the booster upwards right after its end, before the pins reach the arms.
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const auto& set = b->model.engine_sets[1];
    CHECK(norm(plan.reference.u.back()) <= set.thrust_min_n + s.limits.terminal_thrust_margin * set.thrust_max_n + 1.0e3);
}

TEST_CASE("The engine switch happens with the booster upright and calm", "[gnc][descent][scvx]") {
    // Three engines turn the booster slowly: it must not still be tilting when the ignition set shuts down.
    const auto b = block3();
    const auto s = settings(*b);
    const auto x0 = ignition_state(*b);  // 30 m east, 15 m south: a divert on the ignition set
    const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, state_attitude(x0)), s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const auto& x = plan.reference.x[static_cast<size_t>(s.nodes_per_phase - 1)];
    CHECK(tilt_of(x) <= s.limits.switch_tilt_rad + 1e-3);
    const Vec3 w = state_rate(x);
    CHECK(std::hypot(w.y, w.z) <= s.limits.switch_rate_radps + 1e-3);
}

TEST_CASE("An abort plan keeps clear of the launch mount near the ground", "[gnc][descent][scvx]") {
    const auto b = block3();
    auto s = settings(*b);
    s.limits.glide_slope_rad = 0.0;
    DescentTarget target;
    target.goal = DescentGoal::Abort;
    target.ground_cg_height_m = b->vehicle->mass_properties().cg_station_m;
    target.sea_target_m = {0.0, 650.0, 0.0};  // the sea 650 m east
    target.keep_out_center_m = {0.0, -21.0, 21.0};  // tower north-west of the pad
    target.keep_out_radius_m = 18.0;
    target.mount_keep_out_radius_m = 20.0;  // launch mount on the pad centre (13 m + hull 4.5 m + margin)
    target.mount_top_cg_height_m = 20.0 + target.ground_cg_height_m + 5.0;
    // Low over the pad, slowly sinking: the cheapest path would leave from the mount sideways at its height.
    const auto x0 = make_state(b->vehicle->mass_properties().mass_kg - 30000.0, {120.0, 2.0, 0.0}, {-15.0, 0.0, 0.0}, Quat{}, {});
    const auto plan = plan_descent(b->model, x0, 1, target, s, nullptr, 0.0);
    REQUIRE(plan.converged);
    for (const auto& x : plan.reference.x) {
        if (x[descent_index::position] >= target.mount_top_cg_height_m - 1.0) continue;
        CHECK(std::hypot(x[descent_index::position + 1], x[descent_index::position + 2]) >= target.mount_keep_out_radius_m - 0.5);
    }
}

TEST_CASE("The ignition set hands over within the hand-over sink rates", "[gnc][descent][scvx]") {
    // Thirteen engines at minimum throttle far exceed the weight: running them down to a slow descent would make
    // any timing error a climb. The switch happens at a sink rate the final engines can take over.
    const auto b = block3();
    const auto s = settings(*b);
    for (const double height : {1000.0, 1300.0}) {
        CAPTURE(height);
        const auto x0 = make_state(b->vehicle->mass_properties().mass_kg, {height, 5.0, 0.0}, {-250.0, 0.0, 0.0}, Quat{}, {});
        const auto plan = plan_descent(b->model, x0, 0, catch_target(*b, Quat{}), s, nullptr, 0.0);
        REQUIRE(plan.converged);
        CHECK(plan.reference.x[static_cast<size_t>(s.nodes_per_phase - 1)][descent_index::velocity] <=
              -s.limits.switch_min_sink_mps + 1e-3);
        // ... and not much faster: the final engines are slow to steer, their segment stays short.
        CHECK(plan.reference.x[static_cast<size_t>(s.nodes_per_phase - 1)][descent_index::velocity] >=
              -s.limits.switch_max_sink_mps - 1e-3);
    }
}
