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
    // Also converged once the dynamics hold (virtual control) and the final mass changes less than this: near the
    // fuel optimum the trust region only lets SCvx creep (~70 kg per iteration), feasibility is what guidance needs.
    double final_mass_tolerance_kg = 100.0;
    // Largest nonlinear defect of an interval, scaled (1 cm in position): defects add up over the 2N-2 intervals,
    // so this keeps the flown plan within ~0.3 m of the target (5 cm per interval allowed > 1 m).
    double defect_tolerance = 1e-3;
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
    double nonlinear_defect = 0.0;  // largest scaled defect of the plan propagated through the model
    optim::SocpStatus status = optim::SocpStatus::NumericalError;

    double duration_s() const { return reference.duration_s[0] + reference.duration_s[1]; }
    double node_time_s(int k) const;
    int phase_at(double t_s) const;
    DescentState state_at(double t_s) const;   // linear between nodes, quaternion normalised
    math::Vec3 thrust_at(double t_s) const;    // first-order hold
};

// target or -target, whichever is nearer `reference` (a quaternion and its negative are the same attitude).
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
