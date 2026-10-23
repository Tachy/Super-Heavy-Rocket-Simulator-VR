#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "starsim/gnc/descent_problem.hpp"
#include "starsim/optim/socp_problem.hpp"
#include "starsim/optim/socp_solver.hpp"

namespace starsim::gnc {

struct ScvxSettings {
    int nodes_per_phase = 10;  // 28 nodes over the three phases (re-plans within the 1 s cycle)
    int max_iterations = 10;         // re-plan from a warm start (time budget)
    // Fresh plan from the straight-line guess (the simulation waits for it). [E] 60 (was 30): a Falcon 9 drone-ship
    // ignition 1.3 km short at 3.8 km (ASDS MECO campaign seed 107) converges in 42; at 30 the deck was refused.
    int fresh_max_iterations = 60;
    int substeps = 4;                         // RK4 steps per interval in the discretisation
    double step_tolerance = 1e-2;             // largest scaled node step for convergence
    double virtual_control_tolerance = 1e-4;  // L1, scaled
    // Also converged once the dynamics hold (virtual control) and the final mass changes less than this: near the
    // fuel optimum the trust region only lets SCvx creep (~70 kg per iteration), feasibility is what guidance needs.
    double final_mass_tolerance_kg = 100.0;
    // Largest nonlinear defect of an interval, scaled (1 cm in position): defects add up over the P(N-1) intervals,
    // so this keeps the flown plan within ~0.3 m of the target (5 cm per interval allowed > 1 m).
    double defect_tolerance = 1e-3;
    // At the iteration limit a plan within this defect (5 cm per interval) still counts as flyable: the next re-plan
    // starts from the measured state a second later.
    double flyable_defect_tolerance = 5e-3;
    // Adaptive trust region: a failed subproblem is solved again with the trust-region weight times growth.
    int trust_region_retries = 3;
    // A subproblem the solver could not finish to full accuracy is still used if it is feasible to this residual.
    double usable_primal_residual = 1e-6;
    double trust_region_growth = 10.0;
    // Not converging: stop once the virtual control has not shrunk below stall_ratio of its value stall_iterations
    // iterations earlier while still above stall_virtual_control (scaled: metre-size defects, an unreachable target;
    // keeps failing re-plans short without cutting slow but healthy convergence).
    int stall_iterations = 3;
    double stall_ratio = 0.5;
    double stall_virtual_control = 1.0;
    DescentLimits limits;
    DescentWeights weights;
    // Fresh plans: phase durations of a kinematic first guess (all 0: the straight-line guess). For approaches that
    // mostly fly horizontally (the lunar high gate: 3 km at 90 m/s), where the straight-line guess is far from
    // consistent and SCvx ends in a poor local optimum. [E] per vehicle.
    PhaseDurations kinematic_guess_s{};
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

    double duration_s() const;
    double node_time_s(int k) const;
    int phase_at(double t_s) const;
    DescentState state_at(double t_s) const;   // linear between nodes, quaternion normalised
    math::Vec3 thrust_at(double t_s) const;    // first-order hold
};

// target or -target, whichever is nearer `reference` (a quaternion and its negative are the same attitude).
math::Quat nearest_sign(const math::Quat& target, const math::Quat& reference);

DescentReference initial_guess(const DescentModel& model, const DescentState& initial, int start_phase,
                               const DescentTarget& target, const DescentLimits& limits, int nodes_per_phase);

// Kinematic guess over the given phase durations: a cubic (Hermite) path that matches position and velocity at both
// ends, its acceleration flown by the thrust (minus gravity and the frame terms; magnitude within each set's bounds,
// tilt within the phase limit), the attitude along the thrust (roll of the initial state), the mass from the flow.
DescentReference kinematic_guess(const DescentModel& model, const DescentState& initial, int start_phase,
                                 const DescentTarget& target, const DescentLimits& limits, int nodes_per_phase,
                                 const PhaseDurations& duration_s);

// Previous plan shifted by `elapsed_s` onto the node grid of a re-plan; nodes at time 0 are the measured `initial`
// state (the old plan's prediction there differs by the tracking error).
DescentReference warm_start_reference(const DescentPlan& previous, double elapsed_s, int start_phase, int nodes_per_phase,
                                      const DescentState& initial);

// SCvx (penalised trust region): linearise about the iterate, solve the convex subproblem, take its solution as
// the next iterate; converged when the step and the virtual control are below tolerance. `warm_start` (may be
// null) is the previous plan, `warm_start_elapsed_s` the time since its start.
DescentPlan plan_descent(const DescentModel& model, const DescentState& initial, int start_phase,
                         const DescentTarget& target, const ScvxSettings& settings, const DescentPlan* warm_start,
                         double warm_start_elapsed_s);

// SCvx with one reusable SOCP solver per (start phase, goal): the subproblem structure depends only on these (and
// the node count), so re-plans reuse ordering, symbolic factorisation and work arrays. Not thread-safe: one planner
// per thread. A non-converged plan may still carry status Optimal (the last subproblem solved): check `converged`.
class DescentPlanner {
public:
    DescentPlan plan(const DescentModel& model, const DescentState& initial, int start_phase, const DescentTarget& target,
                     const ScvxSettings& settings, const DescentPlan* warm_start, double warm_start_elapsed_s);
    size_t solver_count() const { return solvers_.size(); }

private:
    optim::SocpResult solve(int start_phase, DescentGoal goal, const optim::SocpProblem& problem);
    std::map<std::pair<int, DescentGoal>, std::unique_ptr<optim::SocpSolver>> solvers_;
};

}  // namespace starsim::gnc
