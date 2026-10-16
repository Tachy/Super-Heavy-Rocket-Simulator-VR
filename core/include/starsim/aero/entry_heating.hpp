#pragma once

namespace starsim::aero {

// Convective heat rate at the stagnation point (Sutton & Graves, NASA TR R-376, 1971): q = k sqrt(rho / R_n) v^3
// with k for Earth air (constants::sutton_graves_earth_si). Convective only; radiative heating (it matters above
// ~9 km/s: lunar and planetary returns): entry_heat_rate_w_m2. Returns 0 without air or without a nose radius.
double stagnation_heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m);

class RadiativeHeatingTable;

// Stagnation heat rate of an entry: convective (Sutton-Graves) plus, with a table, equilibrium radiative heating
// (NASA TM-102652). The radiative part matters above ~9 km/s (lunar returns): at 11 km/s, 60 km and a 5 m radius it is
// 6.8 MW/m^2 against ~2 MW/m^2 convective.
double entry_heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m,
                            const RadiativeHeatingTable* radiative);

}  // namespace starsim::aero
