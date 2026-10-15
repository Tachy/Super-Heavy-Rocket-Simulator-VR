#pragma once

#include "starsim/environment/gravity.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// Engines of a burn as gnc::BurnExecutor flies it: constant vacuum thrust and mass flow.
struct FiniteBurnModel {
    double mass_kg = 0.0;  // at ignition
    double thrust_n = 0.0;
    double mass_flow_kgps = 0.0;
};

// A node flown as a finite burn: ignition half the rocket-equation duration before the node, thrust along the node's
// delta-v (gnc::BurnExecutor steers along the remaining delta-v, which keeps that direction) until it is gained.
// Returns the "equivalent impulsive" state at the node epoch: coasting it reproduces the trajectory after the cutoff.
// `at_node`: the coasted (pre-burn) state at the node epoch. The long burns from a parking orbit (TLI: ~7 min, ~28 deg
// of orbit) miss the Moon by thousands of km when targeted as impulses.
math::StateVector apply_finite_burn(const environment::GravityModel& gravity, const math::StateVector& at_node,
                                    time::Epoch node_epoch, const math::Vec3& delta_v_mps, const FiniteBurnModel& model);

}  // namespace starsim::mission
