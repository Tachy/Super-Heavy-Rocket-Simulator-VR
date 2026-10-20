#pragma once

// Refuelling r6: the campaign's orbit costs - the tanker's way to another depot altitude and back, the depot's drag.

namespace starsim::mission {

// Total of both burns of a Hohmann transfer between circular radii (>= 0, either direction).
double hohmann_delta_v_mps(double from_radius_m, double to_radius_m, double gm_m3ps2);

// Lowering the perigee of a circular orbit to `perigee_radius_m` (the deorbit burn, one impulse at apoapsis).
double deorbit_delta_v_mps(double radius_m, double perigee_radius_m, double gm_m3ps2);

// Propellant flow that makes up the drag of a body in a circular orbit: 0.5 rho v^2 Cd A / exhaust velocity
// (impulsive makeup, small delta-v: the rocket equation linearised).
double drag_makeup_kgps(double density_kg_m3, double speed_mps, double drag_coefficient, double area_m2,
                        double exhaust_velocity_mps);

}  // namespace starsim::mission
