#pragma once

#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

// Cold gas an LVLH attitude hold spends against the gravity gradient (fix round 1 of planner Task 8, M2: the depot's
// attitude gas). A slender body held nose along +V-bar, dorsal to nadir, sits on the gravity gradient's unstable
// pitch equilibrium: within the hold deadband +-deadband_rad the torque 1.5 n^2 (I_yaw - I_roll) sin(2 theta) pushes it
// away from the reference and the RCS pushes it back. With the pitch error spread evenly over the deadband [E] the
// mean torque is 1.5 n^2 dI (1 - cos(2 deadband)) / (2 deadband); the gas is that over the torque per mass flow.
// Yaw (about the local vertical) has no gravity-gradient torque; roll's is (I_pitch - I_yaw) ~ 0 on an axisymmetric
// body. Not modelled: the aerodynamic torque (nose into the flow: none at zero error), the free limit cycle (a pulse of
// the minimum on-time per deadband crossing: grams per day).
double gravity_gradient_hold_gas_kgps(double inertia_difference_kg_m2, double orbital_rate_radps, double deadband_rad,
                                      double torque_per_mass_flow_nm_per_kgps);

// Pitch torque per propellant flow of the vehicle's RCS (body y axis about the current CG): a couple of the thrusters
// with the longest pitch arms forward and aft of the CG (as the allocator pairs them in vacuum), each at its own exhaust
// velocity (Isp g0). 0 without pitch thrusters.
double pitch_torque_per_mass_flow(const Vehicle& v);

// The vehicle's LVLH hold gas (its inertia and RCS now) in an orbit of rate `orbital_rate_radps`.
double lvlh_hold_gas_kgps(const Vehicle& v, double orbital_rate_radps, double deadband_rad);

}  // namespace starsim::vehicle
