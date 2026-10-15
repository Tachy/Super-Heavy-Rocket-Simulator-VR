#include "starsim/mission/position_targeter.hpp"

#include <array>
#include <cmath>

#include "starsim/orbit/ballistic_propagator.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {
constexpr double jacobian_step_mps = 1e-3;  // finite-difference step of the delta-v columns
constexpr double max_step_mps = 500.0;      // Newton step limit (keeps a far first guess on the right arc)
constexpr int line_search_steps = 6;

// Solves [a b c] x = y by Cramer's rule; false when the columns are (nearly) dependent.
bool solve_columns(const std::array<Vec3, 3>& col, const Vec3& y, Vec3& x) {
    const double det = dot(col[0], cross(col[1], col[2]));
    const double scale = norm(col[0]) * norm(col[1]) * norm(col[2]);
    if (!std::isfinite(det) || std::abs(det) <= 1e-14 * scale) return false;
    x = Vec3{dot(y, cross(col[1], col[2])), dot(col[0], cross(y, col[2])), dot(col[0], cross(col[1], y))} / det;
    return true;
}

Vec3 unit_axis(int k) {
    return k == 0 ? Vec3{1.0, 0.0, 0.0} : (k == 1 ? Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0});
}
}  // namespace

PositionTargeter::PositionTargeter(const environment::GravityModel& earth_centred_gravity)
    : gravity_(earth_centred_gravity) {}

StateVector PositionTargeter::coast(const StateVector& state, time::Epoch from, time::Epoch to) const {
    if (to - from == 0.0) return state;
    return orbit::propagate_coast(gravity_, state, from, to).state;
}

PositionSolution PositionTargeter::solve(const StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                         const PositionTarget& target, const Vec3& initial_delta_v,
                                         double tolerance_m, int max_iterations) const {
    const StateVector at_burn = coast(state, epoch, burn_epoch);
    auto arrive = [&](const Vec3& dv) {
        StateVector s = at_burn;
        if (finite_) {
            s = apply_finite_burn(gravity_, at_burn, burn_epoch, dv, *finite_);
        } else {
            s.velocity_mps += dv;
        }
        return coast(s, burn_epoch, target.epoch);
    };
    PositionSolution solution;
    Vec3 dv = initial_delta_v;
    StateVector arrival = arrive(dv);
    Vec3 miss = arrival.position_m - target.position_gcrf_m;
    for (int it = 0; it < max_iterations && norm(miss) > tolerance_m; ++it) {
        solution.iterations = it + 1;
        // Jacobian d(arrival position)/d(dv), central differences.
        std::array<Vec3, 3> column;
        for (int k = 0; k < 3; ++k) {
            const Vec3 e = jacobian_step_mps * unit_axis(k);
            column[k] = (arrive(dv + e).position_m - arrive(dv - e).position_m) / (2.0 * jacobian_step_mps);
        }
        Vec3 step;
        if (!solve_columns(column, -1.0 * miss, step)) break;
        if (norm(step) > max_step_mps) step = step * (max_step_mps / norm(step));
        // Backtracking line search on the miss distance.
        double scale = 1.0;
        for (int ls = 0; ls < line_search_steps; ++ls, scale *= 0.5) {
            const StateVector trial = arrive(dv + scale * step);
            const Vec3 trial_miss = trial.position_m - target.position_gcrf_m;
            if (norm(trial_miss) < norm(miss) || ls == line_search_steps - 1) {
                dv += scale * step;
                arrival = trial;
                miss = trial_miss;
                break;
            }
        }
    }
    solution.converged = norm(miss) <= tolerance_m;
    solution.burn = {burn_epoch, dv};
    solution.arrival_gcrf = arrival;
    solution.miss_m = norm(miss);
    return solution;
}

}  // namespace starsim::mission
