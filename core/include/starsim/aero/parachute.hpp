#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::aero {

// Drag area C_D S (m^2) of ONE canopy `time_since_deploy_s` after line stretch. Each reefing stage, and then the final
// opening, fills from the previous area in fill_time_s with the area growing as t^2 (the filling-time model of Knacke,
// Parachute Recovery Systems Design Manual, NWC TP 6575 [public domain]) and holds for its hold_s. Massless canopy (spec
// d4: no added mass, no pendulum body), so the opening load is q C_D S with no opening-shock factor.
double parachute_drag_area_m2(const config::ParachuteConfig& parachute, double time_since_deploy_s);

// Drag of `drag_area_m2` (all canopies of a set together) on a riser point moving through the air at
// `point_air_velocity_body_mps`: -1/2 rho |v| v C_D S.
math::Vec3 parachute_force_body_n(double drag_area_m2, const math::Vec3& point_air_velocity_body_mps,
                                  double density_kg_m3);

}  // namespace starsim::aero
