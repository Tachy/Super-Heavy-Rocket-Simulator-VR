#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/gnc/descent_tracking.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Quat;
using math::Vec3;

namespace {

constexpr double mass_kg = 1.0e5;
constexpr double gravity_mps2 = 9.8;
constexpr double braking_mps2 = 5.0;  // net upward acceleration: thrust = m (g + a)

// No air, no mass flow, no Earth rotation: the vertical fall braked at a constant net acceleration.
DescentModel vacuum_model() {
    DescentModel m;
    m.gravity_mps2 = {-gravity_mps2, 0.0, 0.0};
    m.inertia_kg_m2 = {1.0e6, 1.0e7, 1.0e7};
    m.density_kg_m3 = {0.0, 0.0};
    m.speed_of_sound_mps = {340.0, 340.0};
    m.cd_by_mach = {0.0};
    m.crossflow_cd_by_mach = {0.0};
    return m;
}

// Two nodes per phase, phases of 2 s, 2 s, 0 s, on the analytic path h = 100 - 20 t + a t^2 / 2 (upright).
DescentPlan braking_plan() {
    const auto h = [](double t) { return 100.0 - 20.0 * t + 0.5 * braking_mps2 * t * t; };
    const auto v = [](double t) { return -20.0 + braking_mps2 * t; };
    DescentPlan p;
    p.nodes_per_phase = 2;
    p.converged = true;
    p.reference.duration_s = {2.0, 2.0, 0.0};
    for (const double t : {0.0, 2.0, 4.0, 4.0}) p.reference.x.push_back(make_state(mass_kg, {h(t), 0.0, 0.0}, {v(t), 0.0, 0.0}, Quat{}, {}));
    p.reference.u.assign(4, Vec3{mass_kg * (gravity_mps2 + braking_mps2), 0.0, 0.0});
    return p;
}

}  // namespace

TEST_CASE("The plan path follows the planned dynamics between nodes, not their chord", "[gnc][tracking]") {
    const auto plan = braking_plan();
    const auto model = vacuum_model();
    const auto path = plan_path(plan, model);
    REQUIRE(path.t_s.size() == path.x.size());
    REQUIRE(path.t_s.size() >= 2);
    CHECK(path.t_s.front() == 0.0);
    CHECK(path.t_s.back() == 4.0);
    for (size_t i = 1; i < path.t_s.size(); ++i) CHECK(path.t_s[i] - path.t_s[i - 1] <= plan_path_step_s + 1e-12);
    // Mid-interval the braking path lies a dt^2 / 8 = 2.5 m below the nodes' chord (85 m at t = 1 s).
    const auto chord = sample_plan(plan, model, 1.0, 1.0);
    const auto ref = sample_plan(plan, model, 1.0, 1.0, path);
    CHECK(std::abs(chord.position_m.x - 85.0) < 1e-9);
    CHECK(std::abs(ref.position_m.x - 82.5) < 5e-3);  // linear between 0.05 s samples: a h^2 / 8 ~ 1.6 mm
    CHECK(std::abs(ref.velocity_mps.x + 15.0) < 1e-9);
    CHECK(std::abs(ref.thrust_acceleration_mps2.x - (gravity_mps2 + braking_mps2)) < 1e-9);
    CHECK(ref.phase == 0);
    // In the second phase as well, and the path meets the last node.
    CHECK(std::abs(sample_plan(plan, model, 3.0, 1.0, path).position_m.x - 62.5) < 5e-3);
    CHECK(std::abs(path_state(path, 4.0)[descent_index::position] - 60.0) < 1e-9);
    // Past the end the reference keeps sinking from the last node, as without a path.
    const auto after = sample_plan(plan, model, 5.0, 1.0, path);
    CHECK(after.finished);
    CHECK(std::abs(after.position_m.x - 59.0) < 1e-9);
}

TEST_CASE("A plan path turns with the planned rate", "[gnc][tracking]") {
    // Braking at 5 m/s^2 net (braking_plan), the gimbaled thrust tilted so that its torque spins the body up about z at a constant rate of change:
    // the path's rate grows linearly, the chord's is the nodes' (zero at both ends of a phase that starts at rest).
    auto plan = braking_plan();
    const auto model = vacuum_model();
    const double lever_m = 20.0, side_n = 1.0e5;
    auto m = model;
    m.gimbal_lever_m = lever_m;
    for (auto& u : plan.reference.u) u.y = side_n;
    const auto path = plan_path(plan, m);
    const double alpha = lever_m * side_n / m.inertia_kg_m2.z;  // torque (-lever x side) about z / I
    const auto ref = sample_plan(plan, m, 1.0, 1.0, path);
    CHECK(std::abs(std::abs(ref.rate_plan_radps.z) - alpha * 1.0) < 1e-3 * alpha);
    CHECK(std::abs(std::abs(ref.angular_acceleration_plan_radps2.z) - alpha) < 1e-2 * alpha);
    CHECK(norm(sample_plan(plan, m, 1.0, 1.0).rate_plan_radps) == 0.0);
}

TEST_CASE("An empty plan path has the zero state and a plan without nodes builds none", "[gnc][tracking]") {
    const PlanPath empty;
    const auto x = path_state(empty, 1.0);
    for (int i = 0; i < 14; ++i) CHECK(x[i] == 0.0);
    DescentPlan plan;
    CHECK(plan_path(plan, vacuum_model()).t_s.empty());
}
