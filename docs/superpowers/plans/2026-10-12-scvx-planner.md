# SCvx Landing Planner (Plan 3a) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A tested offline 6-DOF successive-convexification planner that plans the booster's landing burn (13 -> 3 engines) to the tower catch point or to a sea abort target, on a reusable SOCP solver.

**Architecture:** `optim::SocpSolver` keeps ordering, symbolic factorisation and work arrays across solves of one structure. `gnc` gains four small units: `descent_model` (planning frame, 14-state rigid-body model, numeric Jacobians, model builder from a `Vehicle`), `descent_discretization` (first-order-hold propagation with sensitivities), `descent_problem` (convex subproblem in scaled deviation variables), `descent_planner` (SCvx iterations, initial guess, warm start, plan sampling). In-flight use (worker thread, tracking, abort handling, dispersion campaign) is Plan 3b.

**Tech Stack:** C++20 (MSVC 2022, CMake, Ninja), Catch2 v3.8.1, own `optim` SOCP solver.

**Spec:** `docs/superpowers/specs/2026-10-12-scvx-landing-design.md` (this plan covers its sections "Prerequisite" and "Guidance model"; "Execution in flight", "Tracking", "Abort to the sea" (except the abort subproblem), "Fallback" and the campaign tests follow in Plan 3b).

## Global Constraints

- Module order: `optim` uses only the standard library; `gnc/descent_*` may use `math`, `integrators`, `optim`, `config`, `vehicle`, `environment`, `aero`; nothing in `descent_*` may call SPICE or `ephemeris` (the planner must run on a worker thread later).
- The planner is deterministic: identical inputs give bit-identical plans (no threads, no randomness, no time-dependent logic inside).
- SI units, radians; frame + unit in names. Planning frame `plan`: origin at the pad centre on the ground, x up, y east, z north.
- Every constant with a comment (and [E] where it is an estimate/tuning value).
- Commits: Conventional Commits + the session's attribution lines.
- Build/test: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests` then `build/core/tests/starsim_core_tests.exe "<tag>"`; full `tools/build.ps1` at the end of each task.

## Review Focus

- A reference whose thrust passes near zero (e.g. the first guess at a node) -> the lower-bound linearisation divides by |u|; must stay finite (guard) and the plan must still converge; tested in Task 5 (`initial guess` with tiny thrust is clamped).
- A start state already inside the final (3-engine) phase -> phase 0 of zero duration must not break the discretisation (no division by the duration, Phi = I); tested in Task 6.
- Quaternion sign: the terminal attitude must be the sign nearer the initial attitude, otherwise SCvx is asked to rotate by ~360 deg; tested in Task 6 (catch target with roll from both sides).
- Identical sparsity structure across SCvx iterations (zero Jacobian entries kept as explicit zeros) -> the reusable solver must not throw a structure mismatch; tested in Task 4.
- Solver reuse after a failed solve (NumericalError) -> the next solve on the same object starts clean; tested in Task 1.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `core/include/starsim/optim/socp_solver.hpp`, `core/src/optim/socp_solver.cpp` | `SocpSolver` (reusable), `solve_socp` wrapper, finite checks, diagnostics |
| `core/include/starsim/optim/kkt_system.hpp`, `.cpp` | `set_values`, refinement stop ratio |
| `core/include/starsim/optim/socp_problem.hpp`, `.cpp` | `validate` finite data, `SocpResult::regularized_pivots` |
| `core/include/starsim/optim/socp_builder.hpp` | bounds-checked `VariableBlock::operator[]` |
| `core/include/starsim/gnc/descent_model.hpp`, `core/src/gnc/descent_model.cpp` | planning frame, `DescentState`, `DescentModel` (dynamics, Jacobians), `make_descent_model` |
| `core/include/starsim/gnc/descent_discretization.hpp`, `.cpp` | `linearize_interval` (FOH, RK4, sensitivities) |
| `core/include/starsim/gnc/descent_problem.hpp`, `.cpp` | `DescentTarget`, `DescentLimits`, `DescentWeights`, `DescentReference`, `DescentSubproblem`, `build_descent_subproblem` |
| `core/include/starsim/gnc/descent_planner.hpp`, `.cpp` | `ScvxSettings`, `DescentPlan` (+ sampling), `initial_guess`, `plan_descent` |
| `core/tests/test_socp_reuse.cpp`, `test_descent_model.cpp`, `test_descent_discretization.cpp`, `test_descent_problem.cpp`, `test_descent_planner.cpp` | tests |

---

### Task 1: Reusable SOCP solver and the review follow-ups

**Files:**
- Modify: `core/include/starsim/optim/socp_solver.hpp`, `core/src/optim/socp_solver.cpp`, `core/include/starsim/optim/kkt_system.hpp`, `core/src/optim/kkt_system.cpp`, `core/include/starsim/optim/socp_problem.hpp`, `core/src/optim/socp_problem.cpp`, `core/include/starsim/optim/socp_builder.hpp`
- Create: `core/tests/test_socp_reuse.cpp`; modify `core/tests/CMakeLists.txt`

**Interfaces:**
- Produces: `class optim::SocpSolver { explicit SocpSolver(const SocpProblem& structure, SocpSettings settings = {}); SocpResult solve(const SocpProblem& problem); }` (throws `std::invalid_argument` if `problem` has a different variable count, cone layout or sparsity pattern of A/G); `KktSystem::set_values(const CscMatrix& a, const CscMatrix& g)`; `SocpResult::regularized_pivots` (largest count over the iterations).

- [ ] **Step 1: Failing tests** - `core/tests/test_socp_reuse.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "starsim/optim/socp_builder.hpp"
#include "starsim/optim/socp_solver.hpp"

using namespace starsim::optim;

namespace {

// min ||x - p|| over the half-space x0 + x1 >= 1 (p as a parameter of h only).
SocpProblem projection(double px, double py) {
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    const auto t = b.add_variables(1, "t");
    b.set_objective(AffineExpr{}.add(t[0], 1.0));
    b.add_second_order_cone({AffineExpr{}.add(t[0], 1.0), AffineExpr{-px}.add(x[0], 1.0), AffineExpr{-py}.add(x[1], 1.0)});
    b.add_nonnegative(AffineExpr{-1.0}.add(x[0], 1.0).add(x[1], 1.0));
    return b.build();
}

}  // namespace

TEST_CASE("A reusable solver gives the same results as solve_socp", "[optim]") {
    SocpSolver solver(projection(0.0, 0.0));
    for (const auto& p : {std::pair{0.0, 0.0}, std::pair{-2.0, 0.5}, std::pair{3.0, 3.0}}) {
        const auto problem = projection(p.first, p.second);
        const auto a = solver.solve(problem);
        const auto b = solve_socp(problem);
        REQUIRE(a.status == SocpStatus::Optimal);
        CHECK(a.x == b.x);
        CHECK(a.iterations == b.iterations);
    }
}

TEST_CASE("A reusable solver rejects a different structure", "[optim]") {
    SocpSolver solver(projection(0.0, 0.0));
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    b.set_objective(AffineExpr{}.add(x[0], 1.0));
    b.add_nonnegative(AffineExpr{}.add(x[0], 1.0));
    CHECK_THROWS_AS(solver.solve(b.build()), std::invalid_argument);
}

TEST_CASE("A reusable solver recovers after a failed solve", "[optim]") {
    auto problem = projection(0.0, 0.0);
    SocpSolver solver(problem);
    auto broken = problem;
    broken.h[0] = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(solver.solve(broken), std::invalid_argument);  // validate rejects NaN data
    const auto r = solver.solve(problem);
    CHECK(r.status == SocpStatus::Optimal);
    CHECK(r.x == solve_socp(problem).x);
}

TEST_CASE("Variable blocks are bounds-checked", "[optim]") {
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    CHECK(x[1] == 1);
    CHECK_THROWS_AS(x[2], std::out_of_range);
}
```

Add `test_socp_reuse.cpp` to `core/tests/CMakeLists.txt`. Run: build. Expected: compile error (`SocpSolver` not declared).

- [ ] **Step 2: `KktSystem::set_values` and the refinement stop ratio**

`kkt_system.hpp`: public `void set_values(const CscMatrix& a, const CscMatrix& g);  // same patterns as in the constructor`; private `std::vector<int> a_positions_, g_positions_;  // value index of each A / G entry, then of its transpose`.

`kkt_system.cpp`, at the end of the constructor (after `matrix_` exists):

```cpp
    for (int j = 0; j < n_; ++j) {
        for (int q = a.col_start()[j]; q < a.col_start()[j + 1]; ++q) {
            a_positions_.push_back(value_position(matrix_, n_ + a.row_index()[q], j));
            a_positions_.push_back(value_position(matrix_, j, n_ + a.row_index()[q]));
        }
        for (int q = g.col_start()[j]; q < g.col_start()[j + 1]; ++q) {
            g_positions_.push_back(value_position(matrix_, z0 + g.row_index()[q], j));
            g_positions_.push_back(value_position(matrix_, j, z0 + g.row_index()[q]));
        }
    }
```

(`z0` is the local `n_ + p_` already defined there; move the loop after `matrix_ = CscMatrix(...)`.)

```cpp
void KktSystem::set_values(const CscMatrix& a, const CscMatrix& g) {
    auto& values = matrix_.values();
    for (size_t k = 0; k < a.values().size(); ++k) values[a_positions_[2 * k]] = values[a_positions_[2 * k + 1]] = a.values()[k];
    for (size_t k = 0; k < g.values().size(); ++k) values[g_positions_[2 * k]] = values[g_positions_[2 * k + 1]] = g.values()[k];
}
```

(The CSC value order of `a` is column-major, the same order the position vectors were filled in.)

Refinement: add `constexpr double refinement_stop_ratio = 5.0;  // a step must shrink the residual this much (Clarabel)` and rewrite the loop in `solve`:

```cpp
    double previous = std::numeric_limits<double>::infinity();
    for (int step = 0; step < refinement_steps_; ++step) {
        std::copy(rhs.begin(), rhs.end(), residual_.begin());
        matrix_.multiply_add(solution, residual_, -1.0);
        for (int j = 0; j < n_; ++j) residual_[j] += delta_ * solution[j];
        for (int i = 0; i < p_; ++i) residual_[n_ + i] -= delta_ * solution[n_ + i];
        const double size = norm_inf(residual_);
        if (size <= tolerance || size > previous / refinement_stop_ratio) break;
        previous = size;
        ldl_.solve(residual_);
        for (std::size_t i = 0; i < residual_.size(); ++i) solution[i] += residual_[i];
    }
```

(`#include <limits>`.)

- [ ] **Step 3: Finite checks, bounds check, diagnostics**
- `SocpProblem::validate`: after the size checks, `for` each value in `c`, `b`, `h`, `A.values()`, `G.values()`: `if (!std::isfinite(v)) fail("non-finite data");` (`#include <cmath>`).
- `SocpResult`: `int regularized_pivots = 0;  // most dynamically regularised KKT pivots in one iteration`.
- `VariableBlock::operator[]`: `if (i < 0 || i >= size) throw std::out_of_range("VariableBlock: index outside the block"); return offset + i;` (`#include <stdexcept>` in `socp_builder.hpp`).

- [ ] **Step 4: `SocpSolver`** - `socp_solver.hpp`:

```cpp
// Interior-point solver for a fixed problem structure: the ordering, the symbolic LDL^T factorisation and all
// work arrays are built once; solve() accepts any problem with the same variable count, cone layout and
// sparsity patterns of A and G (values, c, b, h may change). Not thread-safe; deterministic.
class SocpSolver {
public:
    explicit SocpSolver(const SocpProblem& structure, SocpSettings settings = {});
    ~SocpSolver();
    SocpSolver(SocpSolver&&) noexcept;
    SocpSolver& operator=(SocpSolver&&) noexcept;
    SocpResult solve(const SocpProblem& problem);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
```

(`#include <memory>`.) In `socp_solver.cpp`, refactor `InteriorPointSolver` into `SocpSolver::Impl`:
- The constructor takes `(const SocpProblem& structure, const SocpSettings& settings)`, validates `structure`, stores `settings_`, `n_, p_, m_`, `cones_ = structure.cones`, the patterns (`structure.A`, `structure.G` copies kept as `pattern_a_`, `pattern_g_`), builds `kkt_(structure.A, structure.G, cones_, ...)`, `scaling_(cones_)` and allocates all work vectors exactly as the old constructor did.
- `SocpResult solve(const SocpProblem& problem)`: `problem.validate()`; throw `std::invalid_argument("SocpSolver: different problem structure")` unless `problem.variables == n_`, `problem.A.rows() == p_`, `problem.G.rows() == m_`, the cone layouts are equal (`nonnegative` and `second_order`) and `col_start()`/`row_index()` of A and G equal those of the patterns. Then `original_ = &problem; scaled_ = problem; equilibration_ = equilibrate(scaled_, settings_.equilibration_passes); kkt_.set_values(scaled_.A, scaled_.G);` reset `tau_ = kappa_ = 1`, zero the iterate vectors, `max_regularized_ = 0`, and `return run();`.
- Replace every `scaled_.cones` in the class with `cones_`; `original_` becomes a `const SocpProblem*`.
- In `run()`: after each successful `kkt_.factor(scaling_)` (both in `initialize` and in the loop) `max_regularized_ = std::max(max_regularized_, kkt_.regularized_pivots());`; `store()` (or `finish`/the success paths) sets `result.regularized_pivots = max_regularized_`.
- Non-finite step: right after `const double alpha = ...` add `if (!std::isfinite(alpha) || !std::isfinite(combined_.tau)) return give_up(result, SocpStatus::NumericalError);` and likewise after `alpha_affine` for `affine_.tau`.
- Free function `solve_socp(problem, settings)` becomes `SocpSolver solver(problem, settings); return solver.solve(problem);`.
- Out-of-line `SocpSolver::SocpSolver(...) : impl_(std::make_unique<Impl>(structure, settings)) {}`, `~SocpSolver() = default;`, defaulted moves, `solve` forwards to `impl_->solve`.

- [ ] **Step 5: Run** - `build/core/tests/starsim_core_tests.exe "[optim]"`. Expected: all pass (old and new). Then `tools/build.ps1` (full) -> 100 %.

- [ ] **Step 6: Commit** - `git commit -m "feat: reusable SOCP solver for repeated solves of one structure"` (body: refinement stop ratio, finite checks, diagnostics).

---

### Task 2: Planning model

**Files:**
- Create: `core/include/starsim/gnc/descent_model.hpp`, `core/src/gnc/descent_model.cpp`, `core/tests/test_descent_model.cpp`; CMake lists.

**Interfaces:**
- Produces: `gnc::site_to_plan(Vec3)`, `gnc::plan_to_site(Vec3)`, `using gnc::DescentState = math::VecN<14>`, `namespace gnc::descent_index { mass=0, position=1, velocity=4, attitude=7, rate=11 }`, `struct gnc::EngineSetModel`, `struct gnc::DescentModel` with `derivative(x, thrust_body_n, phase)` and `jacobians(x, u, phase, Mat14x14& a, Mat14x3& b)` (`using Mat14x14 = std::array<double, 196>; using Mat14x3 = std::array<double, 42>;` row-major), `struct gnc::DescentModelInputs`, `gnc::DescentModel gnc::make_descent_model(const DescentModelInputs&)`, helpers `make_state(m, r, v, q, w)`, `state_position(x)`, `state_velocity(x)`, `state_attitude(x)`, `state_rate(x)`.

- [ ] **Step 1: Failing tests** - `core/tests/test_descent_model.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/gnc/descent_model.hpp"

using namespace starsim;
using namespace starsim::gnc;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using math::Quat;
using math::Vec3;

namespace {

// Simple model: one set of 10 MN (min 4 MN), Isp 330 s, lever 30 m, no air, no Earth rotation.
DescentModel simple_model() {
    DescentModel m;
    for (auto& set : m.engine_sets) set = {1.0e7, 4.0e6, 1.0 / (330.0 * 9.80665), 15.0 * 3.14159265358979 / 180.0};
    m.gimbal_lever_m = 30.0;
    m.inertia_kg_m2 = {1.0e7, 2.0e8, 2.0e8};
    m.reference_area_m2 = 63.6;
    m.cd_by_mach = {1.0, 1.0};
    m.density_kg_m3 = {0.0, 0.0};
    m.speed_of_sound_mps = {340.0, 340.0};
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

DescentState upright(double mass, const Vec3& r, const Vec3& v) { return make_state(mass, r, v, Quat{1.0, 0.0, 0.0, 0.0}, {}); }

}  // namespace

TEST_CASE("Planning frame maps site east-north-up to up-east-north", "[gnc][descent]") {
    const Vec3 enu{1.0, 2.0, 3.0};
    CHECK(site_to_plan(enu) == Vec3{3.0, 1.0, 2.0});
    CHECK(plan_to_site(site_to_plan(enu)) == enu);
}

TEST_CASE("Planning model: thrust, gravity, mass flow and gimbal torque", "[gnc][descent]") {
    const auto model = simple_model();
    const auto x = upright(5.0e5, {1000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0});
    const auto d = model.derivative(x, {8.0e6, 0.0, 0.0}, 0);
    CHECK_THAT(d[descent_index::mass], WithinRel(-8.0e6 / (330.0 * 9.80665), 1e-12));
    CHECK_THAT(d[descent_index::position], WithinAbs(-100.0, 1e-12));
    CHECK_THAT(d[descent_index::velocity], WithinRel(8.0e6 / 5.0e5 - 9.8, 1e-12));
    // Thrust tilted towards body +z (gimbal "pitch"): torque about +y = lever * u_z.
    const auto t = model.derivative(x, {8.0e6, 0.0, 1.0e5}, 0);
    CHECK_THAT(t[descent_index::rate + 1], WithinRel(30.0 * 1.0e5 / 2.0e8, 1e-12));
    CHECK_THAT(t[descent_index::rate + 2], WithinAbs(0.0, 1e-15));
}

TEST_CASE("Planning model: drag opposes the motion through the air", "[gnc][descent]") {
    auto model = simple_model();
    model.density_kg_m3 = {1.2, 1.2};
    const auto x = upright(5.0e5, {1000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0});
    const auto d = model.derivative(x, {0.0, 0.0, 0.0}, 0);
    // a = -9.8 + 0.5 rho v^2 A cd / m (upwards for a falling body).
    CHECK_THAT(d[descent_index::velocity], WithinRel(-9.8 + 0.5 * 1.2 * 1.0e4 * 63.6 / 5.0e5, 1e-9));
}

TEST_CASE("Planning model Jacobians predict small changes", "[gnc][descent]") {
    auto model = simple_model();
    model.density_kg_m3 = {1.2, 0.8};
    model.earth_rate_radps = {3.2e-5, 0.0, 6.5e-5};
    model.earth_center_m = {-6.37e6, 0.0, 0.0};
    const Quat q = Quat::from_axis_angle({0.0, 1.0, 1.0}, 0.1);
    const auto x = make_state(5.0e5, {800.0, 20.0, -30.0}, {-120.0, 5.0, 3.0}, q, {0.01, -0.02, 0.03});
    const Vec3 u{8.0e6, 2.0e5, -1.0e5};
    Mat14x14 a;
    Mat14x3 b;
    model.jacobians(x, u, 0, a, b);
    DescentState dx;
    for (int i = 0; i < 14; ++i) dx[i] = 1e-4 * (1.0 + std::abs(x[i])) * ((i % 3) - 1.0);
    const Vec3 du{1.0e3, -2.0e3, 5.0e2};
    const auto f0 = model.derivative(x, u, 0);
    const auto f1 = model.derivative(x + dx, u + du, 0);
    for (int i = 0; i < 14; ++i) {
        double predicted = 0.0;
        for (int j = 0; j < 14; ++j) predicted += a[i * 14 + j] * dx[j];
        predicted += b[i * 3] * du.x + b[i * 3 + 1] * du.y + b[i * 3 + 2] * du.z;
        const double actual = f1[i] - f0[i];
        CHECK(std::abs(predicted - actual) <= 1e-3 * std::abs(actual) + 1e-9);
    }
}
```

Run: build. Expected: compile error (`descent_model.hpp` missing).

- [ ] **Step 2: Implement** - `core/include/starsim/gnc/descent_model.hpp`

```cpp
#pragma once

#include <array>
#include <string>
#include <vector>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/math/vec_n.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::gnc {

// Planning frame of the landing guidance: origin at the pad centre on the ground, x up (geodetic vertical),
// y east, z north. Upright (body x along +x) is the identity attitude, so the tilt limit is a cone in the
// quaternion's (y, z) part.
inline math::Vec3 site_to_plan(const math::Vec3& enu) { return {enu.z, enu.x, enu.y}; }
inline math::Vec3 plan_to_site(const math::Vec3& p) { return {p.y, p.z, p.x}; }

// mass, position (3), velocity (3, relative to the ground), attitude quaternion (w, x, y, z; body -> plan),
// body angular rate (3, relative to the plan frame).
using DescentState = math::VecN<14>;
using Mat14x14 = std::array<double, 196>;  // row-major
using Mat14x3 = std::array<double, 42>;    // row-major

namespace descent_index {
constexpr int mass = 0;
constexpr int position = 1;
constexpr int velocity = 4;
constexpr int attitude = 7;
constexpr int rate = 11;
}  // namespace descent_index

DescentState make_state(double mass_kg, const math::Vec3& position_m, const math::Vec3& velocity_mps,
                        const math::Quat& attitude, const math::Vec3& rate_radps);
math::Vec3 state_position(const DescentState& x);
math::Vec3 state_velocity(const DescentState& x);
math::Quat state_attitude(const DescentState& x);
math::Vec3 state_rate(const DescentState& x);

// One engine set at sea-level pressure.
struct EngineSetModel {
    double thrust_max_n = 0.0;               // all engines at full throttle
    double thrust_min_n = 0.0;               // all engines at minimum throttle
    double mass_flow_per_thrust_s_m = 0.0;   // kg/s per N = 1 / (Isp g0)
    double gimbal_max_rad = 0.0;
};

// Planning model of the landing burn (documented simplifications): rigid body with constant inertia and CG,
// thrust at a gimbal point on the axis, mass flow proportional to thrust, tail-first axial drag along the
// motion through still air, gravity constant over the pad, Coriolis and centrifugal terms of the rotating
// plan frame; no grid fins, no wind. Air data and drag are tabulated so the model owns all its data (the
// planner can run on a worker thread).
struct DescentModel {
    std::array<EngineSetModel, 2> engine_sets;  // phase 0: ignition set, phase 1: final set
    double gimbal_lever_m = 0.0;                // CG above the gimbal point
    math::Vec3 inertia_kg_m2;                   // principal, body axes
    double reference_area_m2 = 0.0;
    std::vector<double> cd_by_mach;             // every mach_step from Mach 0
    double mach_step = 0.05;
    std::vector<double> density_kg_m3;         // every height_step_m above the pad
    std::vector<double> speed_of_sound_mps;
    double height_step_m = 50.0;
    math::Vec3 gravity_mps2;                    // gravitation at the pad, plan frame
    math::Vec3 earth_rate_radps;               // Earth rotation, plan frame
    math::Vec3 earth_center_m;                 // Earth centre in the plan frame

    DescentState derivative(const DescentState& x, const math::Vec3& thrust_body_n, int phase) const;
    // Central differences; a = d derivative / d x, b = d derivative / d thrust.
    void jacobians(const DescentState& x, const math::Vec3& thrust_body_n, int phase, Mat14x14& a, Mat14x3& b) const;
};

// Snapshot inputs for the model (all evaluated on the simulation thread).
struct DescentModelInputs {
    const vehicle::Vehicle* vehicle = nullptr;    // the booster (single stage)
    std::vector<std::string> ignition_groups;     // phase 0 engine set
    std::vector<std::string> final_groups;        // phase 1 engine set
    const environment::Atmosphere* atmosphere = nullptr;
    double site_height_m = 0.0;                   // pad ground above WGS84
    math::Vec3 gravity_mps2;                      // plan frame
    math::Vec3 earth_rate_radps;                  // plan frame
    math::Vec3 earth_center_m;                    // plan frame
};

DescentModel make_descent_model(const DescentModelInputs& inputs);

}  // namespace starsim::gnc
```

`core/src/gnc/descent_model.cpp`:

```cpp
#include "starsim/gnc/descent_model.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::gnc {

using math::Quat;
using math::Vec3;

namespace {

constexpr double table_height_m = 15000.0;   // air data tabulated up to this height above the pad
constexpr double table_mach = 4.0;           // drag tabulated up to this Mach number
constexpr double jacobian_relative_step = 1e-6;

// Linear interpolation in an equally spaced table starting at 0; clamped at both ends.
double interpolate(const std::vector<double>& table, double step, double at) {
    if (table.size() < 2) return table.empty() ? 0.0 : table[0];
    const double s = std::clamp(at / step, 0.0, static_cast<double>(table.size() - 1));
    const size_t i = std::min(static_cast<size_t>(s), table.size() - 2);
    const double f = s - static_cast<double>(i);
    return table[i] + (table[i + 1] - table[i]) * f;
}

}  // namespace

DescentState make_state(double mass_kg, const Vec3& r, const Vec3& v, const Quat& q, const Vec3& w) {
    return {{mass_kg, r.x, r.y, r.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z, w.x, w.y, w.z}};
}
Vec3 state_position(const DescentState& x) { return {x[1], x[2], x[3]}; }
Vec3 state_velocity(const DescentState& x) { return {x[4], x[5], x[6]}; }
Quat state_attitude(const DescentState& x) { return {x[7], x[8], x[9], x[10]}; }
Vec3 state_rate(const DescentState& x) { return {x[11], x[12], x[13]}; }

DescentState DescentModel::derivative(const DescentState& x, const Vec3& u, int phase) const {
    const double m = x[descent_index::mass];
    const Vec3 r = state_position(x);
    const Vec3 v = state_velocity(x);
    const Quat q = state_attitude(x);
    const Vec3 w = state_rate(x);
    Vec3 drag;
    const double speed = norm(v);
    if (speed > 1e-6) {
        const double height = std::max(r.x, 0.0);
        const double rho = interpolate(density_kg_m3, height_step_m, height);
        const double mach = speed / std::max(interpolate(speed_of_sound_mps, height_step_m, height), 1.0);
        drag = v * (-0.5 * rho * speed * reference_area_m2 * interpolate(cd_by_mach, mach_step, mach));
    }
    const Vec3 omega = earth_rate_radps;
    const Vec3 acceleration = q.rotate(u) / m + drag / m + gravity_mps2 - cross(omega, v) * 2.0 -
                              cross(omega, cross(omega, r - earth_center_m));
    const Quat q_dot = math::attitude_rate(q, w);
    const Vec3 lever{-gimbal_lever_m, 0.0, 0.0};  // gimbal point relative to the CG, body axes
    const Vec3 h{inertia_kg_m2.x * w.x, inertia_kg_m2.y * w.y, inertia_kg_m2.z * w.z};
    const Vec3 torque = cross(lever, u) - cross(w, h);
    return make_state(-engine_sets[static_cast<size_t>(phase)].mass_flow_per_thrust_s_m * norm(u), v, acceleration,
                      q_dot, {torque.x / inertia_kg_m2.x, torque.y / inertia_kg_m2.y, torque.z / inertia_kg_m2.z});
}

void DescentModel::jacobians(const DescentState& x, const Vec3& u, int phase, Mat14x14& a, Mat14x3& b) const {
    for (int j = 0; j < 14; ++j) {
        const double h = jacobian_relative_step * std::max(1.0, std::abs(x[j]));
        DescentState plus = x, minus = x;
        plus[j] += h;
        minus[j] -= h;
        const auto df = derivative(plus, u, phase) - derivative(minus, u, phase);
        for (int i = 0; i < 14; ++i) a[static_cast<size_t>(i * 14 + j)] = df[i] / (2.0 * h);
    }
    const double uu[3] = {u.x, u.y, u.z};
    for (int j = 0; j < 3; ++j) {
        const double h = jacobian_relative_step * std::max(1.0, norm(u));
        double up[3] = {uu[0], uu[1], uu[2]}, um[3] = {uu[0], uu[1], uu[2]};
        up[j] += h;
        um[j] -= h;
        const auto df = derivative(x, {up[0], up[1], up[2]}, phase) - derivative(x, {um[0], um[1], um[2]}, phase);
        for (int i = 0; i < 14; ++i) b[static_cast<size_t>(i * 3 + j)] = df[i] / (2.0 * h);
    }
}

DescentModel make_descent_model(const DescentModelInputs& in) {
    const auto& vehicle = *in.vehicle;
    const auto& stage = vehicle.stages().front();
    const double pressure = in.atmosphere->at(in.site_height_m).pressure_pa;
    DescentModel m;
    const std::vector<std::string>* groups[2] = {&in.ignition_groups, &in.final_groups};
    double gimbal_station = 0.0;
    int gimbal_count = 0;
    for (int p = 0; p < 2; ++p) {
        EngineSetModel set;
        double flow = 0.0;
        set.gimbal_max_rad = 0.0;
        for (const auto& e : stage.engines) {
            if (e.starved || std::find(groups[p]->begin(), groups[p]->end(), e.group_id) == groups[p]->end()) continue;
            const auto& c = e.performance->config();
            set.thrust_max_n += e.performance->thrust_n(1.0, pressure);
            set.thrust_min_n += e.performance->thrust_n(c.throttle_min, pressure);
            flow += e.performance->mass_flow_kgps(1.0);
            if (e.gimbaled) {
                set.gimbal_max_rad = set.gimbal_max_rad > 0.0 ? std::min(set.gimbal_max_rad, c.gimbal_max_rad) : c.gimbal_max_rad;
                gimbal_station += e.gimbal_point_m.x;
                ++gimbal_count;
            }
        }
        set.mass_flow_per_thrust_s_m = set.thrust_max_n > 0.0 ? flow / set.thrust_max_n : 0.0;
        m.engine_sets[static_cast<size_t>(p)] = set;
    }
    const auto mass = vehicle.mass_properties();
    m.gimbal_lever_m = mass.cg_station_m - (gimbal_count > 0 ? gimbal_station / gimbal_count : 0.0);
    m.inertia_kg_m2 = mass.inertia_kg_m2;
    m.reference_area_m2 = constants::pi * 0.25 * vehicle.diameter_m() * vehicle.diameter_m();
    const auto& drag = vehicle.aero().drag_curve(true);
    for (double mach = 0.0; mach <= table_mach + 1e-9; mach += m.mach_step) m.cd_by_mach.push_back(drag.cd(mach, true));
    for (double h = 0.0; h <= table_height_m + 1e-9; h += m.height_step_m) {
        const auto air = in.atmosphere->at(in.site_height_m + h);
        m.density_kg_m3.push_back(air.density_kg_m3);
        m.speed_of_sound_mps.push_back(air.speed_of_sound_mps);
    }
    m.gravity_mps2 = in.gravity_mps2;
    m.earth_rate_radps = in.earth_rate_radps;
    m.earth_center_m = in.earth_center_m;
    return m;
}

}  // namespace starsim::gnc
```

Add `src/gnc/descent_model.cpp` and `test_descent_model.cpp` to the CMake lists.

- [ ] **Step 3: Run** - `"[descent]"`. Expected: PASS.

- [ ] **Step 4: Commit** - `git commit -m "feat: 6-DOF planning model of the booster landing burn"`

---

### Task 3: First-order-hold discretisation with sensitivities

**Files:**
- Create: `core/include/starsim/gnc/descent_discretization.hpp`, `core/src/gnc/descent_discretization.cpp`, `core/tests/test_descent_discretization.cpp`; CMake lists.

**Interfaces:**
- Consumes: `DescentModel`, `DescentState`, `Mat14x14`, `Mat14x3`.
- Produces: `struct gnc::IntervalLinearization { Mat14x14 phi; Mat14x3 gamma_minus, gamma_plus; std::array<double, 14> sigma; DescentState propagated; }` and `IntervalLinearization gnc::linearize_interval(const DescentModel&, int phase, const DescentState& x0, const Vec3& u0, const Vec3& u1, double phase_duration_s, double dtau, int substeps)`.

Math (normalised time tau over the interval [0, dtau], phase duration s, FOH thrust u(tau) = u0 (1 - tau/dtau) + u1 tau/dtau):
`x' = s f(x, u)`, `Phi' = s A Phi` (Phi(0) = I), `Gm' = s A Gm + s B (1 - tau/dtau)`, `Gp' = s A Gp + s B tau/dtau`, `Sigma' = s A Sigma + f(x, u)`, all zero at 0 except Phi. Then for neighbouring trajectories `x1 ~ propagated + Phi dx0 + Gm du0 + Gp du1 + Sigma ds`.

- [ ] **Step 1: Failing tests** - `core/tests/test_descent_discretization.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/gnc/descent_discretization.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Quat;
using math::Vec3;

namespace {

DescentModel model() {
    DescentModel m;
    for (auto& set : m.engine_sets) set = {1.0e7, 4.0e6, 1.0 / (330.0 * 9.80665), 0.26};
    m.gimbal_lever_m = 30.0;
    m.inertia_kg_m2 = {1.0e7, 2.0e8, 2.0e8};
    m.reference_area_m2 = 63.6;
    m.cd_by_mach = {0.9, 1.1, 1.3};
    m.mach_step = 0.5;
    m.density_kg_m3 = {1.2, 1.0, 0.8};
    m.speed_of_sound_mps = {340.0, 335.0, 330.0};
    m.height_step_m = 500.0;
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

const DescentState x0 = make_state(5.0e5, {900.0, 20.0, -10.0}, {-150.0, 3.0, -2.0},
                                   Quat::from_axis_angle({0.0, 0.3, 1.0}, 0.05), {0.0, 0.01, -0.02});
const Vec3 u0{8.0e6, 1.0e5, 0.0};
const Vec3 u1{7.0e6, 0.0, -1.0e5};

}  // namespace

TEST_CASE("FOH propagation with zero perturbation reproduces the nonlinear state", "[gnc][descent]") {
    const auto m = model();
    const auto coarse = linearize_interval(m, 0, x0, u0, u1, 20.0, 0.1, 5);
    const auto fine = linearize_interval(m, 0, x0, u0, u1, 20.0, 0.1, 40);
    for (int i = 0; i < 14; ++i) CHECK(std::abs(coarse.propagated[i] - fine.propagated[i]) <= 1e-6 * (1.0 + std::abs(fine.propagated[i])));
}

TEST_CASE("FOH sensitivities predict a perturbed propagation", "[gnc][descent]") {
    const auto m = model();
    const double s = 20.0, dtau = 0.1;
    const auto ref = linearize_interval(m, 0, x0, u0, u1, s, dtau, 10);
    DescentState dx;
    for (int i = 0; i < 14; ++i) dx[i] = 1e-4 * (1.0 + std::abs(x0[i])) * ((i % 2) ? 1.0 : -1.0);
    const Vec3 du0{2.0e4, -1.0e4, 5.0e3}, du1{-1.0e4, 2.0e4, 0.0};
    const double ds = 0.02;
    const auto pert = linearize_interval(m, 0, x0 + dx, u0 + du0, u1 + du1, s + ds, dtau, 10);
    for (int i = 0; i < 14; ++i) {
        double predicted = ref.sigma[static_cast<size_t>(i)] * ds;
        for (int j = 0; j < 14; ++j) predicted += ref.phi[static_cast<size_t>(i * 14 + j)] * dx[j];
        const double dm[3] = {du0.x, du0.y, du0.z}, dp[3] = {du1.x, du1.y, du1.z};
        for (int j = 0; j < 3; ++j) {
            predicted += ref.gamma_minus[static_cast<size_t>(i * 3 + j)] * dm[j] + ref.gamma_plus[static_cast<size_t>(i * 3 + j)] * dp[j];
        }
        const double actual = pert.propagated[i] - ref.propagated[i];
        CHECK(std::abs(predicted - actual) <= 1e-2 * std::abs(actual) + 1e-8);
    }
}

TEST_CASE("A zero-length interval is the identity", "[gnc][descent]") {
    const auto lin = linearize_interval(model(), 0, x0, u0, u1, 0.0, 0.1, 5);
    for (int i = 0; i < 14; ++i) {
        CHECK(lin.propagated[i] == x0[i]);
        for (int j = 0; j < 14; ++j) CHECK(lin.phi[static_cast<size_t>(i * 14 + j)] == (i == j ? 1.0 : 0.0));
    }
}
```

- [ ] **Step 2: Implement**

`descent_discretization.hpp`:

```cpp
#pragma once

#include <array>

#include "starsim/gnc/descent_model.hpp"

namespace starsim::gnc {

// One interval of the first-order-hold discretisation and its sensitivities: for neighbouring trajectories
// x(end) ~ propagated + phi dx0 + gamma_minus du0 + gamma_plus du1 + sigma ds.
struct IntervalLinearization {
    Mat14x14 phi{};
    Mat14x3 gamma_minus{};
    Mat14x3 gamma_plus{};
    std::array<double, 14> sigma{};
    DescentState propagated;
};

// Propagates an interval of normalised length `dtau` of a phase lasting `phase_duration_s` with the thrust
// changing linearly from u0 to u1 (RK4, `substeps` steps), integrating the variational equations along.
IntervalLinearization linearize_interval(const DescentModel& model, int phase, const DescentState& x0,
                                         const math::Vec3& u0, const math::Vec3& u1, double phase_duration_s,
                                         double dtau, int substeps);

}  // namespace starsim::gnc
```

`descent_discretization.cpp`:

```cpp
#include "starsim/gnc/descent_discretization.hpp"

#include "starsim/integrators/rk4.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

// Packed state: x (14), phi (196), gamma_minus (42), gamma_plus (42), sigma (14).
constexpr size_t n_x = 14, o_phi = 14, o_gm = o_phi + 196, o_gp = o_gm + 42, o_sigma = o_gp + 42, n_total = o_sigma + 14;
using Packed = integrators::State<n_total>;

}  // namespace

IntervalLinearization linearize_interval(const DescentModel& model, int phase, const DescentState& x0, const Vec3& u0,
                                         const Vec3& u1, double s, double dtau, int substeps) {
    Packed y;
    for (size_t i = 0; i < n_x; ++i) y[i] = x0[static_cast<int>(i)];
    for (size_t i = 0; i < 14; ++i) y[o_phi + i * 14 + i] = 1.0;
    const integrators::OdeFunction<n_total> rhs = [&](double tau, const Packed& p) {
        const double lambda_plus = tau / dtau;
        const double lambda_minus = 1.0 - lambda_plus;
        const Vec3 u = u0 * lambda_minus + u1 * lambda_plus;
        DescentState x;
        for (int i = 0; i < 14; ++i) x[i] = p[static_cast<size_t>(i)];
        const DescentState f = model.derivative(x, u, phase);
        Mat14x14 a;
        Mat14x3 b;
        model.jacobians(x, u, phase, a, b);
        Packed d;
        for (int i = 0; i < 14; ++i) d[static_cast<size_t>(i)] = s * f[i];
        for (size_t i = 0; i < 14; ++i) {
            for (size_t c = 0; c < 14; ++c) {  // phi
                double sum = 0.0;
                for (size_t k = 0; k < 14; ++k) sum += a[i * 14 + k] * p[o_phi + k * 14 + c];
                d[o_phi + i * 14 + c] = s * sum;
            }
            for (size_t c = 0; c < 3; ++c) {  // gamma_minus, gamma_plus
                double gm = 0.0, gp = 0.0;
                for (size_t k = 0; k < 14; ++k) {
                    gm += a[i * 14 + k] * p[o_gm + k * 3 + c];
                    gp += a[i * 14 + k] * p[o_gp + k * 3 + c];
                }
                d[o_gm + i * 3 + c] = s * (gm + b[i * 3 + c] * lambda_minus);
                d[o_gp + i * 3 + c] = s * (gp + b[i * 3 + c] * lambda_plus);
            }
            double sg = 0.0;  // sigma
            for (size_t k = 0; k < 14; ++k) sg += a[i * 14 + k] * p[o_sigma + k];
            d[o_sigma + i] = s * sg + f[static_cast<int>(i)];
        }
        return d;
    };
    const double h = dtau / substeps;
    for (int k = 0; k < substeps; ++k) y = integrators::rk4_step<n_total>(rhs, k * h, y, h);

    IntervalLinearization out;
    for (int i = 0; i < 14; ++i) out.propagated[i] = y[static_cast<size_t>(i)];
    for (size_t i = 0; i < 196; ++i) out.phi[i] = y[o_phi + i];
    for (size_t i = 0; i < 42; ++i) {
        out.gamma_minus[i] = y[o_gm + i];
        out.gamma_plus[i] = y[o_gp + i];
    }
    for (size_t i = 0; i < 14; ++i) out.sigma[i] = y[o_sigma + i];
    return out;
}

}  // namespace starsim::gnc
```

Note: with `s = 0` the state and phi stay unchanged but sigma integrates `f` (sensitivity to the duration): the zero-length test checks only `propagated` and `phi`.

- [ ] **Step 3: Run** - `"[descent]"`. Expected: PASS. If the sensitivity test misses its 1 % only on the mass row, check `derivative` near `|u|` (the mass flow is `|u|`, smooth for `u != 0`).

- [ ] **Step 4: Commit** - `git commit -m "feat: first-order-hold discretisation with sensitivities for the landing planner"`

---

### Task 4: Convex subproblem

**Files:**
- Create: `core/include/starsim/gnc/descent_problem.hpp`, `core/src/gnc/descent_problem.cpp`, `core/tests/test_descent_problem.cpp`; CMake lists.

**Interfaces:**
- Consumes: `DescentModel`, `IntervalLinearization`, `optim::SocpBuilder`.
- Produces:
  - `enum class gnc::DescentGoal { Catch, Abort };`
  - `struct gnc::DescentTarget { DescentGoal goal; Vec3 position_m; Vec3 velocity_mps; math::Quat attitude; double ground_cg_height_m; Vec3 sea_target_m; Vec3 keep_out_center_m; double keep_out_radius_m; }` (plan frame; Catch uses position/velocity/attitude; Abort uses ground_cg_height_m, sea_target_m (y, z), keep-out)
  - `struct gnc::DescentLimits { std::array<double,2> tilt_max_rad{25 deg, 10 deg}; double rate_max_radps = 0.1; double glide_slope_rad = 30 deg; double min_mass_kg = 0; double max_phase_s = 120; std::array<double,2> min_phase_s{1.0, 1.0}; }`
  - `struct gnc::DescentWeights { double fuel = 1.0; double virtual_control = 1.0e3; double trust_region = 1.0e-1; double sea_distance = 1.0e-2; }`
  - `struct gnc::DescentReference { std::vector<DescentState> x; std::vector<Vec3> u; std::array<double,2> duration_s; }` (`x.size() == u.size() == 2 N - 1`)
  - `struct gnc::DescentSubproblem { optim::SocpProblem problem; ... ; DescentReference unpack(const std::vector<double>& solution) const; double virtual_control(const std::vector<double>& solution) const; double max_step(const std::vector<double>& solution) const; }`
  - `DescentSubproblem gnc::build_descent_subproblem(const DescentModel&, const DescentReference&, const std::vector<IntervalLinearization>&, const DescentState& initial, int start_phase, const DescentTarget&, const DescentLimits&, const DescentWeights&, int nodes_per_phase)`

Node layout: nodes `0 .. 2N-2`; intervals `0 .. N-2` belong to phase 0, `N-1 .. 2N-3` to phase 1; node `N-1` is shared (its thrust bounds are the intersection of both sets). Phase durations `s0, s1`; each interval has normalised length `1/(N-1)`. `start_phase = 1` fixes `s0 = 0`.

Variables (scaled deviations from the reference): `dx_k` (14 per node), `du_k` (3 per node), `ds_p` (2), virtual control `nu_k` (14 per interval) with absolute-value slacks `t_k` (14), trust-region epigraphs `eta_k` per node and `eta_s`, abort distance `d`. Physical value = reference + scale * deviation. Scales: mass 1e4 kg, position 10 m, velocity 10 m/s, quaternion 0.1, rate 0.05 rad/s, thrust 0.1 * thrust_max of phase 0, duration 1 s.

Rows:
- dynamics (per interval, each of the 14 rows divided by the state scale): `S dx_{k+1} - Phi S dx_k - Gm Su du_k - Gp Su du_{k+1} - Sigma ds_p - S nu_k + (xbar_{k+1} - propagated_k) = 0`
- initial: `dx_0 = (initial - xbar_0) / S`
- Catch terminal (node 2N-2): position, velocity, attitude, rate equal to the target (rate 0)
- Abort terminal: position x = ground_cg_height, velocity 0, rate 0; distance epigraph `d >= ||(r_y - sea_y, r_z - sea_z)||`; keep-out at every node: `n_k . (r_k - c)_horizontal >= R` with `n_k` the horizontal unit from `c` to the reference position (if the reference is at `c`, use `(0, 1, 0)`)
- thrust: `||u_k|| <= Tmax_k`; lower bound `(ubar_k / |ubar_k|) . u_k >= Tmin_k` (guard `|ubar_k| >= 1 N`, else use `(1, 0, 0)`)
- gimbal: `||(u_y, u_z)|| <= tan(gimbal_max) u_x`
- tilt: `||(q_y, q_z)|| <= sqrt((1 - cos(tilt_max_phase)) / 2)` (node N-1 and later: phase 1's limit)
- rate: `||omega_k|| <= rate_max`
- Catch glide slope: `tan(glide) ||(r - target)_yz|| <= (r - target)_x`
- mass: `m_k >= min_mass`
- durations: `min_phase_s <= s_p <= max_phase_s` (start_phase 1: `s0 = 0`)
- trust region: rotated cone `eta_k >= ||(dx_k, du_k)||^2` as `||(2 dx_k, 2 du_k, eta_k - 1)|| <= eta_k + 1`; same for `(ds0, ds1)` with `eta_s`
- virtual control: `t_k - nu_k >= 0`, `t_k + nu_k >= 0`

Objective: `- weights.fuel * dx_{last,mass} + weights.virtual_control * sum t + weights.trust_region * (sum eta_k + eta_s) + weights.sea_distance * d` (abort only).

- [ ] **Step 1: Failing tests** - `core/tests/test_descent_problem.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/gnc/descent_problem.hpp"
#include "starsim/optim/socp_solver.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Quat;
using math::Vec3;

namespace {

constexpr int N = 6;

DescentModel model() {
    DescentModel m;
    m.engine_sets[0] = {3.0e7, 1.2e7, 1.0 / (330.0 * 9.80665), 0.26};
    m.engine_sets[1] = {7.0e6, 2.8e6, 1.0 / (330.0 * 9.80665), 0.26};
    m.gimbal_lever_m = 30.0;
    m.inertia_kg_m2 = {1.0e7, 2.0e8, 2.0e8};
    m.reference_area_m2 = 63.6;
    m.cd_by_mach = {1.0};
    m.density_kg_m3 = {1.0};
    m.speed_of_sound_mps = {340.0};
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

DescentReference straight_reference(double a_scale) {
    DescentReference ref;
    ref.duration_s = {10.0, 8.0};
    for (int k = 0; k < 2 * N - 1; ++k) {
        const double f = static_cast<double>(k) / (2 * N - 2);
        ref.x.push_back(make_state(4.0e5 - 5.0e4 * f, {600.0 * (1.0 - f) + 30.0, 10.0 * (1.0 - f), 0.0},
                                   {-100.0 * (1.0 - f), 0.0, 0.0}, Quat{1.0, 0.0, 0.0, 0.0}, {}));
        ref.u.push_back({a_scale * (k < N ? 2.0e7 : 5.0e6), 0.0, 0.0});
    }
    return ref;
}

std::vector<IntervalLinearization> linearize(const DescentModel& m, const DescentReference& ref) {
    std::vector<IntervalLinearization> lin;
    for (int k = 0; k < 2 * N - 2; ++k) {
        const int phase = k < N - 1 ? 0 : 1;
        lin.push_back(linearize_interval(m, phase, ref.x[k], ref.u[k], ref.u[k + 1], ref.duration_s[phase], 1.0 / (N - 1), 2));
    }
    return lin;
}

DescentTarget catch_target() {
    DescentTarget t;
    t.goal = DescentGoal::Catch;
    t.position_m = {30.0, 0.0, 0.0};
    t.velocity_mps = {-1.0, 0.0, 0.0};
    t.attitude = Quat{1.0, 0.0, 0.0, 0.0};
    return t;
}

}  // namespace

TEST_CASE("The descent subproblem keeps its structure across references", "[gnc][descent]") {
    const auto m = model();
    const DescentLimits limits;
    const DescentWeights weights;
    const auto ref_a = straight_reference(1.0);
    const auto ref_b = straight_reference(0.9);
    const auto a = build_descent_subproblem(m, ref_a, linearize(m, ref_a), ref_a.x[0], 0, catch_target(), limits, weights, N);
    const auto b = build_descent_subproblem(m, ref_b, linearize(m, ref_b), ref_b.x[0], 0, catch_target(), limits, weights, N);
    CHECK(a.problem.A.col_start() == b.problem.A.col_start());
    CHECK(a.problem.A.row_index() == b.problem.A.row_index());
    CHECK(a.problem.G.col_start() == b.problem.G.col_start());
    CHECK(a.problem.G.row_index() == b.problem.G.row_index());
    optim::SocpSolver solver(a.problem);
    CHECK_NOTHROW(solver.solve(b.problem));
}

TEST_CASE("One SCvx subproblem is solvable and its solution unpacks", "[gnc][descent]") {
    const auto m = model();
    const auto ref = straight_reference(1.0);
    const auto sub = build_descent_subproblem(m, ref, linearize(m, ref), ref.x[0], 0, catch_target(), DescentLimits{},
                                              DescentWeights{}, N);
    const auto result = optim::solve_socp(sub.problem);
    REQUIRE(result.status == optim::SocpStatus::Optimal);
    const auto next = sub.unpack(result.x);
    REQUIRE(next.x.size() == ref.x.size());
    CHECK(std::abs(next.x.back()[descent_index::position] - 30.0) < 1e-6);  // terminal height (equality)
    CHECK(next.duration_s[0] >= 1.0);
    CHECK(sub.max_step(result.x) >= 0.0);
}

TEST_CASE("Starting in the final phase fixes the first phase to zero length", "[gnc][descent]") {
    const auto m = model();
    const auto ref = straight_reference(1.0);
    const auto sub = build_descent_subproblem(m, ref, linearize(m, ref), ref.x[0], 1, catch_target(), DescentLimits{},
                                              DescentWeights{}, N);
    const auto result = optim::solve_socp(sub.problem);
    REQUIRE(result.status == optim::SocpStatus::Optimal);
    CHECK(sub.unpack(result.x).duration_s[0] == 0.0);
}
```

(`unpack` sets `duration_s[0]` to exactly 0 when `start_phase == 1`.)

- [ ] **Step 2: Implement** - `descent_problem.hpp`

```cpp
#pragma once

#include <array>
#include <vector>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/optim/socp_builder.hpp"

namespace starsim::gnc {

enum class DescentGoal { Catch, Abort };

// End of the landing burn, plan frame.
struct DescentTarget {
    DescentGoal goal = DescentGoal::Catch;
    math::Vec3 position_m;        // Catch: CG position at the catch
    math::Vec3 velocity_mps;      // Catch: velocity at the catch (sink 1 m/s)
    math::Quat attitude;          // Catch: upright, pins across the arms (sign nearest the initial attitude)
    double ground_cg_height_m = 0.0;  // Abort: CG height at touchdown
    math::Vec3 sea_target_m;          // Abort: point at sea (y, z used)
    math::Vec3 keep_out_center_m;     // Abort: tower + mount, horizontal centre
    double keep_out_radius_m = 0.0;
};

struct DescentLimits {
    std::array<double, 2> tilt_max_rad{25.0 * 3.14159265358979 / 180.0, 10.0 * 3.14159265358979 / 180.0};  // per phase
    double rate_max_radps = 0.1;                                // [E] structural / controllability
    double glide_slope_rad = 30.0 * 3.14159265358979 / 180.0;  // above the horizontal, towards the catch point
    double min_mass_kg = 0.0;                                   // dry mass + reserve
    std::array<double, 2> min_phase_s{1.0, 1.0};
    double max_phase_s = 120.0;
};

struct DescentWeights {
    double fuel = 1.0;               // per 1e4 kg
    double virtual_control = 1.0e3;  // L1, scaled units
    double trust_region = 1.0e-1;    // squared scaled step
    double sea_distance = 1.0e-2;    // per metre (abort)
};

// Iterate of SCvx: states and thrust at 2N-1 nodes, phase durations.
struct DescentReference {
    std::vector<DescentState> x;
    std::vector<math::Vec3> u;  // body frame
    std::array<double, 2> duration_s{};
};

class DescentSubproblem {
public:
    optim::SocpProblem problem;

    DescentReference unpack(const std::vector<double>& solution) const;
    double virtual_control(const std::vector<double>& solution) const;  // L1, scaled
    double max_step(const std::vector<double>& solution) const;         // largest scaled node step

    // Internal layout (filled by build_descent_subproblem).
    DescentReference reference;
    int nodes = 0;
    int start_phase = 0;
    std::vector<optim::VariableBlock> dx, du, nu;
    optim::VariableBlock ds;
    std::array<double, 14> state_scale{};
    double thrust_scale = 1.0;
};

DescentSubproblem build_descent_subproblem(const DescentModel& model, const DescentReference& reference,
                                           const std::vector<IntervalLinearization>& linearization,
                                           const DescentState& initial, int start_phase, const DescentTarget& target,
                                           const DescentLimits& limits, const DescentWeights& weights, int nodes_per_phase);

}  // namespace starsim::gnc
```

`descent_problem.cpp`:

```cpp
#include "starsim/gnc/descent_problem.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;
using optim::AffineExpr;

namespace {

// Scales of the deviation variables (one unit of a variable = this much physical change).
constexpr double mass_scale_kg = 1.0e4;
constexpr double position_scale_m = 10.0;
constexpr double velocity_scale_mps = 10.0;
constexpr double attitude_scale = 0.1;
constexpr double rate_scale_radps = 0.05;
constexpr double thrust_scale_fraction = 0.1;  // of phase 0's maximum thrust
constexpr double duration_scale_s = 1.0;
constexpr double min_reference_thrust_n = 1.0;  // below: no direction for the lower-bound linearisation

std::array<double, 14> state_scales() {
    std::array<double, 14> s{};
    s[0] = mass_scale_kg;
    for (int i = 1; i < 4; ++i) s[static_cast<size_t>(i)] = position_scale_m;
    for (int i = 4; i < 7; ++i) s[static_cast<size_t>(i)] = velocity_scale_mps;
    for (int i = 7; i < 11; ++i) s[static_cast<size_t>(i)] = attitude_scale;
    for (int i = 11; i < 14; ++i) s[static_cast<size_t>(i)] = rate_scale_radps;
    return s;
}

}  // namespace

DescentSubproblem build_descent_subproblem(const DescentModel& model, const DescentReference& ref,
                                           const std::vector<IntervalLinearization>& lin, const DescentState& initial,
                                           int start_phase, const DescentTarget& target, const DescentLimits& limits,
                                           const DescentWeights& weights, int n) {
    DescentSubproblem sub;
    sub.reference = ref;
    sub.start_phase = start_phase;
    sub.nodes = 2 * n - 1;
    sub.state_scale = state_scales();
    sub.thrust_scale = thrust_scale_fraction * model.engine_sets[0].thrust_max_n;
    const auto& S = sub.state_scale;
    const double su = sub.thrust_scale;
    const int K = sub.nodes;
    const int boundary = n - 1;

    optim::SocpBuilder b;
    for (int k = 0; k < K; ++k) {
        sub.dx.push_back(b.add_variables(14, "dx"));
        sub.du.push_back(b.add_variables(3, "du"));
    }
    sub.ds = b.add_variables(2, "ds");
    std::vector<optim::VariableBlock> abs_nu;
    for (int k = 0; k < K - 1; ++k) {
        sub.nu.push_back(b.add_variables(14, "nu"));
        abs_nu.push_back(b.add_variables(14, "|nu|"));
    }
    const auto eta = b.add_variables(K, "eta");
    const auto eta_s = b.add_variables(1, "eta_s");
    const auto distance = b.add_variables(1, "sea_distance");

    // Physical value expressions.
    auto X = [&](int k, int i) { return AffineExpr{ref.x[static_cast<size_t>(k)][i]}.add(sub.dx[static_cast<size_t>(k)][i], S[static_cast<size_t>(i)]); };
    auto U = [&](int k, int j) {
        const Vec3& u = ref.u[static_cast<size_t>(k)];
        const double v[3] = {u.x, u.y, u.z};
        return AffineExpr{v[j]}.add(sub.du[static_cast<size_t>(k)][j], su);
    };
    auto phase_of_interval = [&](int k) { return k < boundary ? 0 : 1; };

    // Dynamics, rows divided by the state scale.
    for (int k = 0; k < K - 1; ++k) {
        const auto& L = lin[static_cast<size_t>(k)];
        const int p = phase_of_interval(k);
        for (int i = 0; i < 14; ++i) {
            const double inv = 1.0 / S[static_cast<size_t>(i)];
            AffineExpr row{(ref.x[static_cast<size_t>(k + 1)][i] - L.propagated[i]) * inv};
            row.add(sub.dx[static_cast<size_t>(k + 1)][i], 1.0);
            for (int j = 0; j < 14; ++j) row.add(sub.dx[static_cast<size_t>(k)][j], -L.phi[static_cast<size_t>(i * 14 + j)] * S[static_cast<size_t>(j)] * inv);
            for (int j = 0; j < 3; ++j) {
                row.add(sub.du[static_cast<size_t>(k)][j], -L.gamma_minus[static_cast<size_t>(i * 3 + j)] * su * inv);
                row.add(sub.du[static_cast<size_t>(k + 1)][j], -L.gamma_plus[static_cast<size_t>(i * 3 + j)] * su * inv);
            }
            row.add(sub.ds[p], -L.sigma[static_cast<size_t>(i)] * duration_scale_s * inv);
            row.add(sub.nu[static_cast<size_t>(k)][i], -1.0);
            b.add_equality(row);
            b.add_nonnegative(AffineExpr{}.add(abs_nu[static_cast<size_t>(k)][i], 1.0).add(sub.nu[static_cast<size_t>(k)][i], -1.0));
            b.add_nonnegative(AffineExpr{}.add(abs_nu[static_cast<size_t>(k)][i], 1.0).add(sub.nu[static_cast<size_t>(k)][i], 1.0));
        }
    }
    // Initial state.
    for (int i = 0; i < 14; ++i) {
        b.add_equality(AffineExpr{(ref.x[0][i] - initial[i]) / S[static_cast<size_t>(i)]}.add(sub.dx[0][i], 1.0));
    }
    // Terminal state.
    const int last = K - 1;
    if (target.goal == DescentGoal::Catch) {
        const double t[13] = {target.position_m.x, target.position_m.y, target.position_m.z,
                              target.velocity_mps.x, target.velocity_mps.y, target.velocity_mps.z,
                              target.attitude.w, target.attitude.x, target.attitude.y, target.attitude.z, 0.0, 0.0, 0.0};
        for (int i = 1; i < 14; ++i) b.add_equality(X(last, i).add_constant(-t[i - 1]));
    } else {
        b.add_equality(X(last, descent_index::position).add_constant(-target.ground_cg_height_m));
        for (int i = descent_index::velocity; i < descent_index::velocity + 3; ++i) b.add_equality(X(last, i));
        for (int i = descent_index::rate; i < 14; ++i) b.add_equality(X(last, i));
        b.add_second_order_cone({AffineExpr{}.add(distance[0], 1.0), X(last, 2).add_constant(-target.sea_target_m.y),
                                 X(last, 3).add_constant(-target.sea_target_m.z)});
    }
    // Path constraints.
    for (int k = 0; k < K; ++k) {
        const int p = k < boundary ? 0 : (k == boundary ? -1 : 1);  // -1: shared node
        const auto& e0 = model.engine_sets[0];
        const auto& e1 = model.engine_sets[1];
        const double t_max = p == 0 ? e0.thrust_max_n : p == 1 ? e1.thrust_max_n : std::min(e0.thrust_max_n, e1.thrust_max_n);
        const double t_min = p == 0 ? e0.thrust_min_n : p == 1 ? e1.thrust_min_n : std::max(e0.thrust_min_n, e1.thrust_min_n);
        const double gimbal = p == 0 ? e0.gimbal_max_rad : p == 1 ? e1.gimbal_max_rad : std::min(e0.gimbal_max_rad, e1.gimbal_max_rad);
        const double tilt = limits.tilt_max_rad[p == 0 ? 0 : 1];
        b.add_second_order_cone({AffineExpr{t_max}, U(k, 0), U(k, 1), U(k, 2)});
        const Vec3& ub = ref.u[static_cast<size_t>(k)];
        const Vec3 dir = norm(ub) >= min_reference_thrust_n ? ub / norm(ub) : Vec3{1.0, 0.0, 0.0};
        AffineExpr lower{-t_min};
        const double d[3] = {dir.x, dir.y, dir.z};
        for (int j = 0; j < 3; ++j) {
            const double v[3] = {ub.x, ub.y, ub.z};
            lower.add_constant(d[j] * v[j]).add(sub.du[static_cast<size_t>(k)][j], d[j] * su);
        }
        b.add_nonnegative(lower);
        AffineExpr axial = U(k, 0);
        for (auto& [index, c] : axial.terms) c *= std::tan(gimbal);
        axial.constant *= std::tan(gimbal);
        b.add_second_order_cone({axial, U(k, 1), U(k, 2)});
        b.add_second_order_cone({AffineExpr{std::sqrt(0.5 * (1.0 - std::cos(tilt)))}, X(k, descent_index::attitude + 2),
                                 X(k, descent_index::attitude + 3)});
        b.add_second_order_cone({AffineExpr{limits.rate_max_radps}, X(k, descent_index::rate), X(k, descent_index::rate + 1),
                                 X(k, descent_index::rate + 2)});
        b.add_nonnegative(X(k, descent_index::mass).add_constant(-limits.min_mass_kg));
        if (target.goal == DescentGoal::Catch) {
            const double tg = std::tan(limits.glide_slope_rad);
            AffineExpr hy = X(k, 2).add_constant(-target.position_m.y), hz = X(k, 3).add_constant(-target.position_m.z);
            for (auto* e : {&hy, &hz}) {
                for (auto& [index, c] : e->terms) c *= tg;
                e->constant *= tg;
            }
            b.add_second_order_cone({X(k, 1).add_constant(-target.position_m.x), hy, hz});
        } else {
            const Vec3 rb = state_position(ref.x[static_cast<size_t>(k)]) - target.keep_out_center_m;
            const double h = std::hypot(rb.y, rb.z);
            const double ny = h > 1e-6 ? rb.y / h : 1.0, nz = h > 1e-6 ? rb.z / h : 0.0;
            // n . (r - c) >= R with n the horizontal direction from the centre to the reference.
            const AffineExpr ry = X(k, 2), rz = X(k, 3);
            AffineExpr keep{-target.keep_out_radius_m - ny * target.keep_out_center_m.y - nz * target.keep_out_center_m.z};
            for (const auto& [term_index, coef] : ry.terms) keep.add(term_index, ny * coef);
            for (const auto& [term_index, coef] : rz.terms) keep.add(term_index, nz * coef);
            keep.add_constant(ny * ry.constant + nz * rz.constant);
            b.add_nonnegative(keep);
        }
        // Trust region: eta_k >= ||(dx_k, du_k)||^2 as a rotated cone.
        std::vector<AffineExpr> cone{AffineExpr{1.0}.add(eta[k], 1.0)};
        for (int i = 0; i < 14; ++i) cone.push_back(AffineExpr{}.add(sub.dx[static_cast<size_t>(k)][i], 2.0));
        for (int j = 0; j < 3; ++j) cone.push_back(AffineExpr{}.add(sub.du[static_cast<size_t>(k)][j], 2.0));
        cone.push_back(AffineExpr{-1.0}.add(eta[k], 1.0));
        b.add_second_order_cone(cone);
    }
    b.add_second_order_cone({AffineExpr{1.0}.add(eta_s[0], 1.0), AffineExpr{}.add(sub.ds[0], 2.0),
                             AffineExpr{}.add(sub.ds[1], 2.0), AffineExpr{-1.0}.add(eta_s[0], 1.0)});
    // Phase durations.
    for (int p = 0; p < 2; ++p) {
        AffineExpr s{ref.duration_s[static_cast<size_t>(p)]};
        s.add(sub.ds[p], duration_scale_s);
        if (p == 0 && start_phase == 1) {
            b.add_equality(s);
            continue;
        }
        b.add_nonnegative(AffineExpr{s}.add_constant(-limits.min_phase_s[static_cast<size_t>(p)]));
        AffineExpr upper{limits.max_phase_s - ref.duration_s[static_cast<size_t>(p)]};
        upper.add(sub.ds[p], -duration_scale_s);
        b.add_nonnegative(upper);
    }
    // Objective.
    AffineExpr objective;
    objective.add(sub.dx[static_cast<size_t>(last)][descent_index::mass], -weights.fuel * S[0] / mass_scale_kg);
    for (const auto& block : abs_nu) {
        for (int i = 0; i < 14; ++i) objective.add(block[i], weights.virtual_control);
    }
    for (int k = 0; k < K; ++k) objective.add(eta[k], weights.trust_region);
    objective.add(eta_s[0], weights.trust_region);
    if (target.goal == DescentGoal::Abort) objective.add(distance[0], weights.sea_distance);
    else b.add_nonnegative(AffineExpr{}.add(distance[0], 1.0));  // unused: keep the variable bounded
    b.set_objective(objective);
    sub.problem = b.build();
    return sub;
}

DescentReference DescentSubproblem::unpack(const std::vector<double>& s) const {
    DescentReference out = reference;
    for (int k = 0; k < nodes; ++k) {
        for (int i = 0; i < 14; ++i) out.x[static_cast<size_t>(k)][i] += state_scale[static_cast<size_t>(i)] * s[static_cast<size_t>(dx[static_cast<size_t>(k)][i])];
        Vec3& u = out.u[static_cast<size_t>(k)];
        u += Vec3{s[static_cast<size_t>(du[static_cast<size_t>(k)][0])], s[static_cast<size_t>(du[static_cast<size_t>(k)][1])],
                  s[static_cast<size_t>(du[static_cast<size_t>(k)][2])]} * thrust_scale;
    }
    for (int p = 0; p < 2; ++p) out.duration_s[static_cast<size_t>(p)] += duration_scale_s * s[static_cast<size_t>(ds[p])];
    if (start_phase == 1) out.duration_s[0] = 0.0;
    return out;
}

double DescentSubproblem::virtual_control(const std::vector<double>& s) const {
    double sum = 0.0;
    for (const auto& block : nu) {
        for (int i = 0; i < 14; ++i) sum += std::abs(s[static_cast<size_t>(block[i])]);
    }
    return sum;
}

double DescentSubproblem::max_step(const std::vector<double>& s) const {
    double largest = 0.0;
    for (int k = 0; k < nodes; ++k) {
        double sq = 0.0;
        for (int i = 0; i < 14; ++i) sq += std::pow(s[static_cast<size_t>(dx[static_cast<size_t>(k)][i])], 2);
        for (int j = 0; j < 3; ++j) sq += std::pow(s[static_cast<size_t>(du[static_cast<size_t>(k)][j])], 2);
        largest = std::max(largest, std::sqrt(sq));
    }
    return largest;
}

}  // namespace starsim::gnc
```

`AffineExpr` needs `add_constant(double) -> AffineExpr&` (add it in `socp_builder.hpp`: `AffineExpr& add_constant(double value) { constant += value; return *this; }`). `X(...)` returns by value, so chaining `X(k, i).add_constant(c)` yields a usable temporary.

The unused `distance` variable in Catch mode is fixed by `distance >= 0` and has zero cost; the structure is the same for every Catch subproblem (abort subproblems have their own solver).

- [ ] **Step 3: Run** - `"[descent]"`. Expected: PASS.

- [ ] **Step 4: Commit** - `git commit -m "feat: convex SCvx subproblem of the landing burn"`

---

### Task 5: SCvx planner

**Files:**
- Create: `core/include/starsim/gnc/descent_planner.hpp`, `core/src/gnc/descent_planner.cpp`, `core/tests/test_descent_planner.cpp`; CMake lists.

**Interfaces:**
- Consumes: Tasks 1-4.
- Produces:
  - `struct gnc::ScvxSettings { int nodes_per_phase = 15; int max_iterations = 10; int substeps = 4; double step_tolerance = 1e-2; double virtual_control_tolerance = 1e-4; DescentLimits limits; DescentWeights weights; }`
  - `struct gnc::DescentPlan { DescentReference reference; int nodes_per_phase; int start_phase; DescentGoal goal; bool converged; int iterations; double virtual_control; double last_step; optim::SocpStatus status; double duration_s() const; double node_time_s(int k) const; int phase_at(double t) const; DescentState state_at(double t) const; math::Vec3 thrust_at(double t) const; }`
  - `DescentReference gnc::initial_guess(const DescentModel&, const DescentState& initial, int start_phase, const DescentTarget&, int nodes_per_phase)`
  - `DescentPlan gnc::plan_descent(const DescentModel&, const DescentState& initial, int start_phase, const DescentTarget&, const ScvxSettings&, const DescentPlan* warm_start, double warm_start_elapsed_s)`
  - `math::Quat gnc::nearest_sign(const math::Quat& target, const math::Quat& reference)` (target or -target, whichever has a non-negative dot product with reference)

Initial guess: durations from constant decelerations (phase 0 brings the speed to 50 m/s at 0.8 of the set's thrust-to-weight minus g, phase 1 from 50 m/s to the target velocity; each at least `min_phase_s`, phase 0 zero if `start_phase == 1`); states interpolated linearly in position and velocity between initial and target (Catch: target position; Abort: the sea target's horizontal position at `ground_cg_height_m`), quaternion normalised-lerp from the initial attitude to the target attitude (Abort: upright), rate 0, mass decreasing with the mass flow of the guessed thrust; thrust `(m |g| + m * deceleration, 0, 0)` clamped to each node's bounds.

Warm start: sample the previous plan at `t = elapsed + new node times` with phase 0's duration reduced by the elapsed time (if the start phase is now 1, phase 0 has zero length).

Plan sampling: node times `t_k = s0 * k / (N-1)` for `k <= N-1`, `s0 + s1 * (k - N + 1) / (N - 1)` after; `state_at` interpolates linearly between nodes (quaternion normalised); `thrust_at` interpolates the FOH thrust; times before 0 / after the end clamp.

- [ ] **Step 1: Failing tests** - `core/tests/test_descent_planner.cpp`

```cpp
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

Booster block3() {
    Booster b;
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.resize(1);
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg *= 0.025;  // ~90 t left
    b.vehicle = std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR);
    DescentModelInputs in;
    in.vehicle = b.vehicle.get();
    in.ignition_groups = {"booster/center", "booster/middle"};
    in.final_groups = {"booster/center"};
    in.atmosphere = &b.atmosphere;
    in.site_height_m = 10.0;
    const double lat = 26.0 * deg;
    in.gravity_mps2 = {-9.79, 0.0, 0.0};
    in.earth_rate_radps = Vec3{std::sin(lat), 0.0, std::cos(lat)} * 7.2921150e-5;
    in.earth_center_m = {-6.374e6, 0.0, 0.0};
    b.model = make_descent_model(in);
    return b;
}

DescentTarget catch_target(const Booster& b, const Quat& initial_attitude) {
    DescentTarget t;
    t.goal = DescentGoal::Catch;
    const double pin_station = b.vehicle->stages()[0].config.catch_pins.station_m;
    const double cg = b.vehicle->mass_properties().cg_station_m;
    t.position_m = {96.0 - (pin_station - cg), 0.0, 0.0};  // pins 1 m above the 95 m arm tops
    t.velocity_mps = {-1.0, 0.0, 0.0};
    // Upright, roll 0 (the pin direction itself is checked in Plan 3b with the real tower geometry).
    t.attitude = nearest_sign(Quat{1.0, 0.0, 0.0, 0.0}, initial_attitude);
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
    const auto x0 = ignition_state(b);
    const auto target = catch_target(b, state_attitude(x0));
    const auto s = settings(b);
    const auto start = std::chrono::steady_clock::now();
    const auto plan = plan_descent(b.model, x0, 0, target, s, nullptr, 0.0);
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
        CHECK(norm(u) <= b.model.engine_sets[static_cast<size_t>(phase)].thrust_max_n * (1.0 + 1e-6) + 1.0);
        CHECK(std::hypot(u.y, u.z) <= std::tan(b.model.engine_sets[0].gimbal_max_rad) * u.x * (1.0 + 1e-6) + 1.0);
        CHECK(norm(state_rate(plan.reference.x[k])) <= s.limits.rate_max_radps + 1e-6);
    }
    // Open loop: the plan's thrust flown through the nonlinear model ends at the catch point.
    DescentState x = x0;
    const int n = s.nodes_per_phase;
    for (int k = 0; k < 2 * n - 2; ++k) {
        const int phase = k < n - 1 ? 0 : 1;
        x = linearize_interval(b.model, phase, x, plan.reference.u[static_cast<size_t>(k)], plan.reference.u[static_cast<size_t>(k + 1)],
                               plan.reference.duration_s[static_cast<size_t>(phase)], 1.0 / (n - 1), 20).propagated;
    }
    CHECK(norm(state_position(x) - target.position_m) < 1.0);
    CHECK(norm(state_velocity(x) - target.velocity_mps) < 0.5);
}

TEST_CASE("SCvx planning is deterministic", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto x0 = ignition_state(b);
    const auto target = catch_target(b, state_attitude(x0));
    const auto a = plan_descent(b.model, x0, 0, target, settings(b), nullptr, 0.0);
    const auto c = plan_descent(b.model, x0, 0, target, settings(b), nullptr, 0.0);
    REQUIRE(a.reference.x.size() == c.reference.x.size());
    for (size_t k = 0; k < a.reference.x.size(); ++k) CHECK(a.reference.x[k] == c.reference.x[k]);
}

TEST_CASE("SCvx re-plans from the final phase with a warm start", "[gnc][descent][scvx]") {
    const auto b = block3();
    const auto x0 = ignition_state(b);
    const auto target = catch_target(b, state_attitude(x0));
    const auto s = settings(b);
    const auto first = plan_descent(b.model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(first.converged);
    const double t = first.reference.duration_s[0] + 1.0;  // one second into phase 1
    const auto again = plan_descent(b.model, first.state_at(t), 1, target, s, &first, t);
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
    const auto x0 = ignition_state(b);
    DescentTarget target;
    target.goal = DescentGoal::Abort;
    target.ground_cg_height_m = b.vehicle->mass_properties().cg_station_m;
    target.sea_target_m = {0.0, 2000.0, 0.0};   // 2 km east
    target.keep_out_center_m = {0.0, 0.0, -30.0};
    target.keep_out_radius_m = 25.0;
    auto s = settings(b);
    s.limits.glide_slope_rad = 0.0;
    const auto plan = plan_descent(b.model, x0, 0, target, s, nullptr, 0.0);
    REQUIRE(plan.converged);
    const auto end = plan.reference.x.back();
    CHECK(std::abs(end[descent_index::position] - target.ground_cg_height_m) < 1e-3);
    CHECK(state_position(end).y > 30.0);  // moved seaward
    for (const auto& x : plan.reference.x) {
        const Vec3 r = state_position(x) - target.keep_out_center_m;
        CHECK(std::hypot(r.y, r.z) >= target.keep_out_radius_m - 0.5);
    }
}
```

- [ ] **Step 2: Implement** - `descent_planner.hpp`

```cpp
#pragma once

#include <vector>

#include "starsim/gnc/descent_problem.hpp"
#include "starsim/optim/socp_problem.hpp"

namespace starsim::gnc {

struct ScvxSettings {
    int nodes_per_phase = 15;
    int max_iterations = 10;
    int substeps = 4;                         // RK4 steps per interval in the discretisation
    double step_tolerance = 1e-2;             // largest scaled node step for convergence
    double virtual_control_tolerance = 1e-4;  // L1, scaled
    DescentLimits limits;
    DescentWeights weights;
};

struct DescentPlan {
    DescentReference reference;
    int nodes_per_phase = 0;
    int start_phase = 0;
    DescentGoal goal = DescentGoal::Catch;
    bool converged = false;
    int iterations = 0;
    double virtual_control = 0.0;
    double last_step = 0.0;
    optim::SocpStatus status = optim::SocpStatus::NumericalError;

    double duration_s() const { return reference.duration_s[0] + reference.duration_s[1]; }
    double node_time_s(int k) const;
    int phase_at(double t_s) const;
    DescentState state_at(double t_s) const;
    math::Vec3 thrust_at(double t_s) const;
};

math::Quat nearest_sign(const math::Quat& target, const math::Quat& reference);

DescentReference initial_guess(const DescentModel& model, const DescentState& initial, int start_phase,
                               const DescentTarget& target, const DescentLimits& limits, int nodes_per_phase);

// SCvx (penalised trust region): linearise about the iterate, solve the convex subproblem, take its solution as
// the next iterate; converged when the step and the virtual control are below tolerance. `warm_start` (may be
// null) is the previous plan, `warm_start_elapsed_s` the time since its start.
DescentPlan plan_descent(const DescentModel& model, const DescentState& initial, int start_phase,
                         const DescentTarget& target, const ScvxSettings& settings, const DescentPlan* warm_start,
                         double warm_start_elapsed_s);

}  // namespace starsim::gnc
```

`descent_planner.cpp`:

```cpp
#include "starsim/gnc/descent_planner.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/optim/socp_solver.hpp"

namespace starsim::gnc {

using math::Quat;
using math::Vec3;

namespace {

constexpr double gate_speed_mps = 50.0;       // guess: phase 0 brings the speed down to this (as the explicit guidance)
constexpr double guess_throttle = 0.8;        // guess: design throttle
constexpr double min_deceleration_mps2 = 1.0;

Quat nlerp(const Quat& a, const Quat& b, double f) {
    const Quat bb = nearest_sign(b, a);
    return Quat{a.w + (bb.w - a.w) * f, a.x + (bb.x - a.x) * f, a.y + (bb.y - a.y) * f, a.z + (bb.z - a.z) * f}.normalized();
}

}  // namespace

Quat nearest_sign(const Quat& target, const Quat& reference) {
    const double d = target.w * reference.w + target.x * reference.x + target.y * reference.y + target.z * reference.z;
    return d >= 0.0 ? target : Quat{-target.w, -target.x, -target.y, -target.z};
}

double DescentPlan::node_time_s(int k) const {
    const int n = nodes_per_phase;
    if (k <= n - 1) return reference.duration_s[0] * k / (n - 1);
    return reference.duration_s[0] + reference.duration_s[1] * (k - n + 1) / (n - 1);
}

int DescentPlan::phase_at(double t) const { return t < reference.duration_s[0] ? 0 : 1; }

namespace {

// Node interval containing t and the fraction within it.
std::pair<int, double> locate(const DescentPlan& plan, double t) {
    const int last = static_cast<int>(plan.reference.x.size()) - 1;
    if (t <= 0.0) return {0, 0.0};
    for (int k = 0; k < last; ++k) {
        const double a = plan.node_time_s(k), b = plan.node_time_s(k + 1);
        if (t <= b && b > a) return {k, (t - a) / (b - a)};
    }
    return {last - 1, 1.0};
}

}  // namespace

DescentState DescentPlan::state_at(double t) const {
    const auto [k, f] = locate(*this, t);
    const auto& a = reference.x[static_cast<size_t>(k)];
    const auto& b = reference.x[static_cast<size_t>(k + 1)];
    DescentState x = a * (1.0 - f) + b * f;
    const Quat q = nlerp(state_attitude(a), state_attitude(b), f);
    x[7] = q.w;
    x[8] = q.x;
    x[9] = q.y;
    x[10] = q.z;
    return x;
}

Vec3 DescentPlan::thrust_at(double t) const {
    const auto [k, f] = locate(*this, t);
    return reference.u[static_cast<size_t>(k)] * (1.0 - f) + reference.u[static_cast<size_t>(k + 1)] * f;
}

DescentReference initial_guess(const DescentModel& model, const DescentState& initial, int start_phase,
                               const DescentTarget& target, const DescentLimits& limits, int n) {
    const double m0 = initial[descent_index::mass];
    const double g = norm(model.gravity_mps2);
    const double speed0 = norm(state_velocity(initial));
    const double final_speed = target.goal == DescentGoal::Catch ? norm(target.velocity_mps) : 0.0;
    auto decel = [&](int p, double m) {
        return std::max(guess_throttle * model.engine_sets[static_cast<size_t>(p)].thrust_max_n / m - g, min_deceleration_mps2);
    };
    DescentReference ref;
    ref.duration_s[0] = start_phase == 1 ? 0.0 : std::max((speed0 - gate_speed_mps) / decel(0, m0), limits.min_phase_s[0]);
    ref.duration_s[1] = std::max((std::min(speed0, gate_speed_mps) - final_speed) / decel(1, m0), limits.min_phase_s[1]);
    const Vec3 r_end = target.goal == DescentGoal::Catch
                           ? target.position_m
                           : Vec3{target.ground_cg_height_m, target.sea_target_m.y, target.sea_target_m.z};
    const Vec3 v_end = target.goal == DescentGoal::Catch ? target.velocity_mps : Vec3{};
    const Quat q_end = target.goal == DescentGoal::Catch ? target.attitude : Quat{1.0, 0.0, 0.0, 0.0};
    const double total = ref.duration_s[0] + ref.duration_s[1];
    double mass = m0;
    for (int k = 0; k < 2 * n - 1; ++k) {
        const double t = k <= n - 1 ? ref.duration_s[0] * k / (n - 1) : ref.duration_s[0] + ref.duration_s[1] * (k - n + 1) / (n - 1);
        const double f = total > 0.0 ? t / total : 0.0;
        const int p = k < n - 1 ? 0 : 1;
        const auto& set = model.engine_sets[static_cast<size_t>(p)];
        const double thrust = std::clamp(mass * (g + decel(p, mass)), set.thrust_min_n, set.thrust_max_n);
        ref.x.push_back(make_state(mass, state_position(initial) * (1.0 - f) + r_end * f,
                                   state_velocity(initial) * (1.0 - f) + v_end * f, nlerp(state_attitude(initial), q_end, f), {}));
        ref.u.push_back({thrust, 0.0, 0.0});
        if (k < 2 * n - 2) {
            const double dt = (k < n - 1 ? ref.duration_s[0] : ref.duration_s[1]) / (n - 1);
            mass = std::max(mass - set.mass_flow_per_thrust_s_m * thrust * dt, limits.min_mass_kg);
        }
    }
    return ref;
}

namespace {

DescentReference shifted(const DescentPlan& previous, double elapsed, int start_phase, int n) {
    DescentReference ref;
    ref.duration_s[0] = start_phase == 1 ? 0.0 : std::max(previous.reference.duration_s[0] - elapsed, 0.0);
    ref.duration_s[1] = start_phase == 1 ? std::max(previous.duration_s() - elapsed, 0.0) : previous.reference.duration_s[1];
    for (int k = 0; k < 2 * n - 1; ++k) {
        const double t = k <= n - 1 ? ref.duration_s[0] * k / (n - 1) : ref.duration_s[0] + ref.duration_s[1] * (k - n + 1) / (n - 1);
        ref.x.push_back(previous.state_at(elapsed + t));
        ref.u.push_back(previous.thrust_at(elapsed + t));
    }
    return ref;
}

}  // namespace

DescentPlan plan_descent(const DescentModel& model, const DescentState& initial, int start_phase, const DescentTarget& target,
                         const ScvxSettings& settings, const DescentPlan* warm_start, double elapsed) {
    const int n = settings.nodes_per_phase;
    DescentPlan plan;
    plan.nodes_per_phase = n;
    plan.start_phase = start_phase;
    plan.goal = target.goal;
    plan.reference = warm_start ? shifted(*warm_start, elapsed, start_phase, n)
                                : initial_guess(model, initial, start_phase, target, settings.limits, n);
    std::optional<optim::SocpSolver> solver;
    for (int it = 0; it < settings.max_iterations; ++it) {
        std::vector<IntervalLinearization> lin;
        lin.reserve(static_cast<size_t>(2 * n - 2));
        for (int k = 0; k < 2 * n - 2; ++k) {
            const int p = k < n - 1 ? 0 : 1;
            lin.push_back(linearize_interval(model, p, plan.reference.x[static_cast<size_t>(k)], plan.reference.u[static_cast<size_t>(k)],
                                             plan.reference.u[static_cast<size_t>(k + 1)], plan.reference.duration_s[static_cast<size_t>(p)],
                                             1.0 / (n - 1), settings.substeps));
        }
        const auto sub = build_descent_subproblem(model, plan.reference, lin, initial, start_phase, target, settings.limits,
                                                  settings.weights, n);
        if (!solver) solver.emplace(sub.problem);
        const auto result = solver->solve(sub.problem);
        plan.status = result.status;
        plan.iterations = it + 1;
        if (result.status != optim::SocpStatus::Optimal) return plan;  // not converged; caller falls back / aborts
        plan.virtual_control = sub.virtual_control(result.x);
        plan.last_step = sub.max_step(result.x);
        plan.reference = sub.unpack(result.x);
        if (plan.last_step < settings.step_tolerance && plan.virtual_control < settings.virtual_control_tolerance) {
            plan.converged = true;
            break;
        }
    }
    return plan;
}

}  // namespace starsim::gnc
```

Add the sources and test to the CMake lists.

- [ ] **Step 3: Run** - `STARSIM_VERBOSE=1 build/core/tests/starsim_core_tests.exe "[scvx]"`.
Expected: all pass; the print shows iterations (<= 10) and the time per plan. If SCvx does not converge, diagnose in this order (systematic debugging, one change at a time, record in the ledger): (1) print `virtual_control`/`last_step` per iteration; (2) check the subproblem's terminal equalities are reachable (glide slope vs the initial position: 30 m off at 1170 m height is inside a 30 deg cone); (3) adjust `DescentWeights::trust_region` (larger = smaller steps) and `virtual_control`; (4) raise `max_iterations` only last. Tolerances of the tests stay as written.

- [ ] **Step 4: Commit** - `git commit -m "feat: SCvx landing-burn planner with warm start and plan sampling"` (body: iterations and time per plan from the verbose run).

---

### Task 6: Documentation and verification

**Files:** Modify `CLAUDE.md`, `docs/superpowers/specs/2026-10-12-scvx-landing-design.md` (record deviations, if any, under a "Refinements" heading).

- [ ] **Step 1:** `CLAUDE.md`, GNC paragraph: add "`gnc/descent_*` (M6, Plan 3a): 6-DOF SCvx planner for the landing burn in the plan frame (x up, y east, z north): `descent_model` (planning model + `make_descent_model`), `descent_discretization` (FOH + sensitivities), `descent_problem` (convex subproblem, scaled deviations, catch or sea-abort goal), `descent_planner` (`plan_descent`, warm start, sampling). Uses `optim::SocpSolver` (structure reused across iterations). Not yet flown (Plan 3b)." Known issues: measured iterations/time per plan from Task 5.
- [ ] **Step 2:** Full verification: `tools/build.ps1` -> 100 %; smoke test passes.
- [ ] **Step 3:** Commit `docs: SCvx landing planner`.
