#include "starsim/gnc/descent_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <utility>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/optim/socp_solver.hpp"

namespace starsim::gnc {

using math::Quat;
using math::Vec3;

namespace {

constexpr double guess_throttle = 0.8;    // guess: design throttle (each phase ends mid-way in its switch window)
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
    // Phases before start_phase are over; the start phase ends where it ended in the previous plan (later phases keep
    // their durations): what is left of it, or of all phases up to it if the engines switched before the plan said.
    DescentReference ref;
    double end = 0.0;    // previous plan: end of phase p
    double begin = 0.0;  // new plan: start of phase p, previous plan's time
    for (int p = 0; p < descent_phases; ++p) {
        end += previous.reference.duration_s[static_cast<size_t>(p)];
        if (p < start_phase) continue;
        if (p == start_phase) begin = elapsed;
        ref.duration_s[static_cast<size_t>(p)] = std::max(end - begin, 0.0);
        begin = std::max(end, begin);
    }
    for (int k = 0; k < descent_node_count(n); ++k) {
        const double t = reference_node_time(ref.duration_s, k, n);
        ref.x.push_back(t <= 0.0 ? initial : previous.state_at(elapsed + t));
        ref.u.push_back(previous.thrust_at(elapsed + t));
    }
    return ref;
}

Quat nearest_sign(const Quat& target, const Quat& reference) {
    const double d = target.w * reference.w + target.x * reference.x + target.y * reference.y + target.z * reference.z;
    return d >= 0.0 ? target : Quat{-target.w, -target.x, -target.y, -target.z};
}

double DescentPlan::node_time_s(int k) const { return reference_node_time(reference.duration_s, k, nodes_per_phase); }

double DescentPlan::duration_s() const {
    double total = 0.0;
    for (const double d : reference.duration_s) total += d;
    return total;
}

int DescentPlan::phase_at(double t) const {
    double end = 0.0;
    for (int p = 0; p + 1 < descent_phases; ++p) {
        end += reference.duration_s[static_cast<size_t>(p)];
        if (t < end) return p;
    }
    return descent_phases - 1;
}

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
    const double final_speed = target.goal != DescentGoal::Abort ? norm(target.velocity_mps) : 0.0;
    auto decel = [&](int p, double m) {
        return std::max(guess_throttle * model.engine_sets[static_cast<size_t>(p)].thrust_max_n / m - g, min_deceleration_mps2);
    };
    DescentReference ref;
    double speed = speed0;  // at the start of phase p
    for (int p = start_phase; p < descent_phases; ++p) {
        const auto& sw = limits.switches[static_cast<size_t>(std::min(p, descent_phases - 2))];
        const double end_speed = p + 1 < descent_phases ? std::min(speed, 0.5 * (sw.min_sink_mps + sw.max_sink_mps)) : final_speed;
        ref.duration_s[static_cast<size_t>(p)] = std::max((speed - end_speed) / decel(p, m0), limits.min_phase_s[static_cast<size_t>(p)]);
        speed = end_speed;
    }
    const Vec3 r_end = target.goal != DescentGoal::Abort
                           ? target.position_m
                           : Vec3{target.ground_cg_height_m, target.sea_target_m.y, target.sea_target_m.z};
    const Vec3 v_end = target.goal != DescentGoal::Abort ? target.velocity_mps : Vec3{};
    const Quat q_end = upright_with_roll(state_attitude(initial));
    double total = 0.0;
    for (const double d : ref.duration_s) total += d;
    double mass = m0;
    const int nodes = descent_node_count(n);
    for (int k = 0; k < nodes; ++k) {
        const double f = total > 0.0 ? reference_node_time(ref.duration_s, k, n) / total : 0.0;
        const int p = descent_node_phase(k, n);
        const auto& set = model.engine_sets[static_cast<size_t>(p)];
        const double thrust = std::clamp(mass * (g + decel(p, mass)), set.thrust_min_n, set.thrust_max_n);
        ref.x.push_back(make_state(mass, state_position(initial) * (1.0 - f) + r_end * f,
                                   state_velocity(initial) * (1.0 - f) + v_end * f, nlerp(state_attitude(initial), q_end, f), {}));
        ref.u.push_back({thrust, 0.0, 0.0});
        if (k < nodes - 1) {
            const double dt = ref.duration_s[static_cast<size_t>(k / (n - 1))] / (n - 1);
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
    if (warm_start) limits.min_phase_s = {};
    std::vector<double> virtual_controls;
    DescentWeights weights = settings.weights;
    const auto scale = descent_state_scales();
    bool candidate = false;  // the last solution met the convergence tests that need no propagation
    for (int it = 0;; ++it) {
        std::vector<IntervalLinearization> lin;
        const int intervals = descent_node_count(n) - 1;
        lin.reserve(static_cast<size_t>(intervals));
        double defect = 0.0;
        for (int k = 0; k < intervals; ++k) {
            const int p = k / (n - 1);
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
        // Adaptive trust region: a subproblem the solver cannot finish (numerics, usually a far step from a poor
        // iterate) is solved again with a tighter trust region; after a success it relaxes towards the setting.
        auto sub = build_descent_subproblem(model, plan.reference, lin, initial, start_phase, target, limits, weights, n);
        // A subproblem solution need not be optimal, only feasible: SCvx judges the iterate by its nonlinear defects.
        const auto usable = [&](const optim::SocpResult& r) {
            return r.status == optim::SocpStatus::Optimal ||
                   ((r.status == optim::SocpStatus::NumericalError || r.status == optim::SocpStatus::MaxIterations) &&
                    !r.x.empty() && r.primal_residual <= settings.usable_primal_residual);
        };
        auto result = solve(start_phase, target.goal, sub.problem);
        for (int retry = 0; !usable(result) && retry < settings.trust_region_retries; ++retry) {
            weights.trust_region *= settings.trust_region_growth;
            sub = build_descent_subproblem(model, plan.reference, lin, initial, start_phase, target, limits, weights, n);
            result = solve(start_phase, target.goal, sub.problem);
        }
        if (usable(result)) {
            weights.trust_region = std::max(settings.weights.trust_region, weights.trust_region / settings.trust_region_growth);
        }
        plan.status = result.status;
        plan.iterations = it + 1;
        if (std::getenv("STARSIM_DEBUG_SCVX_SOLVER")) {  // diagnostics: SOCP solver per SCvx iteration
            std::printf("  [socp] it %d status %d iters %d pivots %d reduced %d pres %.2e dres %.2e gap %.2e\n", it,
                        static_cast<int>(result.status), result.iterations, result.regularized_pivots,
                        result.reduced_accuracy, result.primal_residual, result.dual_residual, result.gap);
        }
        if (!usable(result)) {
            // The subproblem failed (numerics): the current iterate still counts if it is flyable, otherwise the
            // caller falls back / aborts.
            plan.converged = it > 0 && plan.virtual_control < settings.virtual_control_tolerance &&
                             defect < settings.flyable_defect_tolerance;
            return plan;
        }
        const double previous_final_mass = plan.reference.x.back()[descent_index::mass];
        plan.virtual_control = sub.virtual_control(result.x);
        virtual_controls.push_back(plan.virtual_control);
        if (it >= settings.stall_iterations && plan.virtual_control > settings.stall_virtual_control &&
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
