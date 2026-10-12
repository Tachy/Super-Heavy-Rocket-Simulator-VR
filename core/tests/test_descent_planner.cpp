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
        CHECK(norm(state_rate(plan.reference.x[k])) <= s.limits.rate_max_radps + 1e-6);
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
