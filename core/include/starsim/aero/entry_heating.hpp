#pragma once

namespace starsim::aero {

// Convective heat rate at the stagnation point (Sutton & Graves, NASA TR R-376, 1971): q = k sqrt(rho / R_n) v^3
// with k for Earth air (constants::sutton_graves_earth_si). Radiative heating is neglected (it matters above
// ~9 km/s: lunar and planetary returns). Returns 0 without air or without a nose radius.
double stagnation_heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m);

}  // namespace starsim::aero
