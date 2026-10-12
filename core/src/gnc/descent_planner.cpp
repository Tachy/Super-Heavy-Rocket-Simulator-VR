#include "starsim/gnc/descent_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/optim/socp_solver.hpp"

namespace starsim::gnc {

using math::Quat;
using math::Vec3;

namespace {

constexpr double gate_speed_mps = 50.0;   // guess: phase 0 brings the speed down to this (as the explicit guidance)
constexpr double guess_throttle = 0.8;    // guess: design throttle
constexpr double min_deceleration_mps2 = 1.0;

Quat nlerp(const Quat& a, const Quat& b, double f) {
    const Quat bb = nearest_sign(b, a);
    return Quat{a.w + (bb.w - a.w) * f, a.x + (bb.x - a.x) * f, a.y + (bb.y - a.y) * f, a.z + (bb.z - a.z) * f}.normalized();
}

// Upright attitude that keeps the roll of q (the planning model has no roll authority).
Quat upright_with_roll(const Quat& q) {
    const double n = std::hypot(q.w, q.x);
    return n > 1e-9 ? Quat{q.w / n, q.x / n, 0.0, 0.0} : Quat{1.0, 0.0, 0.0, 0.0};
}

double guess_node_time(const std::array<double, 2>& duration, int k, int n) { return reference_node_time(duration, k, n); }

// Node interval containing t (zero-length intervals skipped) and the fraction within it.
std::pair<int, double> locate(const DescentPlan& plan, double t) {
    const int last = static_cast<int>(plan.reference.x.size()) - 1;
    for (int k = 0; k < last; ++k) {
        const double a = plan.node_time_s(k), b = plan.node_time_s(k + 1);
        if (b <= a) continue;
        if (t <= a) return {k, 0.0};
        if (t <= b) return {k, (t - a) / (b - a)};
    }
    return {last - 1, 1.0};
}

}  // namespace

DescentReference warm_start_reference(const DescentPlan& previous, double elapsed, int start_phase, int n,
                                      const DescentState& initial) {
    DescentReference ref;
    ref.duration_s[0] = start_phase == 1 ? 0.0 : std::max(previous.reference.duration_s[0] - elapsed, 0.0);
    ref.duration_s[1] = start_phase == 1 ? std::max(previous.duration_s() - elapsed, 0.0) : previous.reference.duration_s[1];
    for (int k = 0; k < 2 * n - 1; ++k) {
        const double t = guess_node_time(ref.duration_s, k, n);
        ref.x.push_back(t <= 0.0 ? initial : previous.state_at(elapsed + t));
        ref.u.push_back(previous.thrust_at(elapsed + t));
    }
    return ref;
}

Quat nearest_sign(const Quat& target, const Quat& reference) {
    const double d = target.w * reference.w + target.x * reference.x + target.y * reference.y + target.z * reference.z;
    return d >= 0.0 ? target : Quat{-target.w, -target.x, -target.y, -target.z};
}

double DescentPlan::node_time_s(int k) const { return guess_node_time(reference.duration_s, k, nodes_per_phase); }

int DescentPlan::phase_at(double t) const { return t < reference.duration_s[0] ? 0 : 1; }

DescentState DescentPlan::state_at(double t) const {
    const auto [k, f] = locate(*this, t);
    const auto& a = reference.x[static_cast<size_t>(k)];
    const auto& b = reference.x[static_cast<size_t>(k + 1)];
    DescentState x = a * (1.0 - f) + b * f;
    const Quat q = nlerp(state_attitude(a), state_attitude(b), f);
    x[descent_index::attitude] = q.w;
    x[descent_index::attitude + 1] = q.x;
    x[descent_index::attitude + 2] = q.y;
    x[descent_index::attitude + 3] = q.z;
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
    const Quat q_end = upright_with_roll(state_attitude(initial));
    const double total = ref.duration_s[0] + ref.duration_s[1];
    double mass = m0;
    for (int k = 0; k < 2 * n - 1; ++k) {
        const double f = total > 0.0 ? guess_node_time(ref.duration_s, k, n) / total : 0.0;
        const int p = k < n - 1 ? 0 : 1;
        const auto& set = model.engine_sets[static_cast<size_t>(p)];
        const double thrust = std::clamp(mass * (g + decel(p, mass)), set.thrust_min_n, set.thrust_max_n);
        ref.x.push_back(make_state(mass, state_position(initial) * (1.0 - f) + r_end * f,
                                   state_velocity(initial) * (1.0 - f) + v_end * f, nlerp(state_attitude(initial), q_end, f), {}));
        ref.u.push_back({thrust, 0.0, 0.0});
        if (k < 2 * n - 2) {
            const double dt = (k < n - 1 ? ref.duration_s[0] : ref.duration_s[1]) / (n - 1);
            mass = std::max(mass - set.mass_flow_per_thrust_s_m * (thrust + set.pressure_thrust_loss_n) * dt, limits.min_mass_kg);
        }
    }
    return ref;
}

DescentPlan plan_descent(const DescentModel& model, const DescentState& initial, int start_phase, const DescentTarget& target,
                         const ScvxSettings& settings, const DescentPlan* warm_start, double elapsed) {
    DescentPlanner planner;
    return planner.plan(model, initial, start_phase, target, settings, warm_start, elapsed);
}

optim::SocpResult DescentPlanner::solve(int start_phase, DescentGoal goal, const optim::SocpProblem& problem) {
    auto& solver = solvers_[{start_phase, goal}];
    if (solver) {
        try {
            return solver->solve(problem);
        } catch (const std::invalid_argument&) {
            // Another structure (e.g. another node count): rebuild below. Invalid data throws again there.
        }
    }
    solver = std::make_unique<optim::SocpSolver>(problem);
    return solver->solve(problem);
}

DescentPlan DescentPlanner::plan(const DescentModel& model, const DescentState& initial, int start_phase,
                                 const DescentTarget& target, const ScvxSettings& settings, const DescentPlan* warm_start,
                                 double elapsed) {
    const int n = settings.nodes_per_phase;
    const int max_iterations = warm_start ? settings.max_iterations : settings.fresh_max_iterations;
    DescentPlan plan;
    plan.nodes_per_phase = n;
    plan.start_phase = start_phase;
    plan.goal = target.goal;
    plan.reference = warm_start ? warm_start_reference(*warm_start, elapsed, start_phase, n, initial)
                                : initial_guess(model, initial, start_phase, target, settings.limits, n);
    // Minimum phase durations only shape a fresh plan: on a re-plan they would stretch a nearly finished phase
    // again and again (every replan cycle).
    DescentLimits limits = settings.limits;
    if (warm_start) limits.min_phase_s = {0.0, 0.0};
    std::vector<double> virtual_controls;
    const auto scale = descent_state_scales();
    bool candidate = false;  // the last solution met the convergence tests that need no propagation
    for (int it = 0;; ++it) {
        std::vector<IntervalLinearization> lin;
        lin.reserve(static_cast<size_t>(2 * n - 2));
        double defect = 0.0;
        for (int k = 0; k < 2 * n - 2; ++k) {
            const int p = k < n - 1 ? 0 : 1;
            lin.push_back(linearize_interval(model, p, plan.reference.x[static_cast<size_t>(k)],
                                             plan.reference.u[static_cast<size_t>(k)], plan.reference.u[static_cast<size_t>(k + 1)],
                                             plan.reference.duration_s[static_cast<size_t>(p)], 1.0 / (n - 1), settings.substeps));
            for (int i = 0; i < 14; ++i) {
                const double d = std::abs(plan.reference.x[static_cast<size_t>(k + 1)][i] - lin.back().propagated[i]) /
                                 scale[static_cast<size_t>(i)];
                defect = std::max(defect, d);
            }
        }
        plan.nonlinear_defect = defect;
        // The linearised dynamics holding (small virtual control) is not enough: the new iterate is propagated
        // through the nonlinear model (this linearisation) before it counts as converged.
        if (candidate && defect < settings.defect_tolerance) {
            plan.converged = true;
            break;
        }
        if (it == max_iterations) {
            // Out of iterations: a dynamically consistent plan (virtual control and nonlinear defects within
            // tolerance) is flyable even though it has not settled at the fuel optimum.
            plan.converged = it > 0 && plan.virtual_control < settings.virtual_control_tolerance &&
                             defect < settings.flyable_defect_tolerance;
            break;
        }
        const auto sub = build_descent_subproblem(model, plan.reference, lin, initial, start_phase, target, limits,
                                                  settings.weights, n);
        const auto result = solve(start_phase, target.goal, sub.problem);
        plan.status = result.status;
        plan.iterations = it + 1;
        if (result.status != optim::SocpStatus::Optimal) {
            // The subproblem failed (numerics): the current iterate still counts if it is flyable, otherwise the
            // caller falls back / aborts.
            plan.converged = it > 0 && plan.virtual_control < settings.virtual_control_tolerance &&
                             defect < settings.flyable_defect_tolerance;
            return plan;
        }
        const double previous_final_mass = plan.reference.x.back()[descent_index::mass];
        plan.virtual_control = sub.virtual_control(result.x);
        virtual_controls.push_back(plan.virtual_control);
        if (it >= settings.stall_iterations && plan.virtual_control > 10.0 * settings.virtual_control_tolerance &&
            plan.virtual_control > settings.stall_ratio * virtual_controls[static_cast<size_t>(it - settings.stall_iterations)]) {
            plan.last_step = sub.max_step(result.x);
            plan.reference = sub.unpack(result.x);
            return plan;  // stalled: not converged
        }
        plan.last_step = sub.max_step(result.x);
        plan.reference = sub.unpack(result.x);
        const bool dynamics_hold = plan.virtual_control < settings.virtual_control_tolerance;
        const bool settled = plan.last_step < settings.step_tolerance ||
                             std::abs(plan.reference.x.back()[descent_index::mass] - previous_final_mass) <
                                 settings.final_mass_tolerance_kg;
        candidate = dynamics_hold && settled;
    }
    return plan;
}

}  // namespace starsim::gnc
