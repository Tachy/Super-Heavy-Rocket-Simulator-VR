#pragma once

#include "starsim/gnc/entry_predictor.hpp"

namespace starsim::gnc {

// Lateral reach of a lifting entry flown at the bank magnitude `bank_rad` with reversals (the deorbit planner's crossrange
// limit): the crossrange a flight holding one bank sign gains over the reversals' mean path. Equilibrium glide with small
// heading change (the standard crossrange approximation): with u = v^2 / (g r), L cos(sigma) = m g (1 - u), the heading
// grows as d(psi) = -(L/D) sin(sigma) dv / v and the range as ds = -(L/D) cos(sigma) v dv / (g (1 - u)); the crossrange
// is the integral of psi ds. For a constant L/D from circular speed down it is (pi^2 / 48) (L/D)^2 sin(2 sigma) r.
// Here L/D follows the vehicle: the model's aero table at its alpha schedule, at the Mach of the equilibrium glide
// (the US76 height whose density carries the glide's lift at the model's mass), from `entry_speed_mps` (Earth-relative,
// at most circular) down. 0 when the bank gives no vertical lift or the table none.
double entry_crossrange_reach_m(const EntryPredictorModel& model, double bank_rad, double entry_speed_mps);

// The closed form for a constant L/D from circular speed (the check of the integral above).
double constant_lift_crossrange_reach_m(double lift_to_drag, double bank_rad, double radius_m);

}  // namespace starsim::gnc
