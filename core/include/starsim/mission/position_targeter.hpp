#pragma once

#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// A GCRF position to be reached at a fixed epoch.
struct PositionTarget {
    math::Vec3 position_gcrf_m;
    time::Epoch epoch;
};

struct PositionSolution {
    bool converged = false;
    int iterations = 0;
    gnc::BurnNode burn;              // impulsive, at the burn epoch
    math::StateVector arrival_gcrf;  // coasted state at target.epoch (Earth-centred GCRF)
    double miss_m = 0.0;
};

// Fixed-time position targeting in the full force model: the impulsive delta-v at a burn epoch whose coast reaches
// a GCRF position at a fixed epoch (Newton on finite-difference sensitivities, backtracking line search).
class PositionTargeter {
public:
    explicit PositionTargeter(const environment::GravityModel& earth_centred_gravity);

    math::StateVector coast(const math::StateVector& state, time::Epoch from, time::Epoch to) const;

    PositionSolution solve(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                           const PositionTarget& target, const math::Vec3& initial_delta_v,
                           double tolerance_m = 10.0, int max_iterations = 30) const;

private:
    const environment::GravityModel& gravity_;
};

}  // namespace starsim::mission
