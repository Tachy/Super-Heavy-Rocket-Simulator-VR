#include "starsim/aero/entry_heating.hpp"

#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::aero {

double stagnation_heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m) {
    if (density_kg_m3 <= 0.0 || nose_radius_m <= 0.0) return 0.0;
    return constants::sutton_graves_earth_si * std::sqrt(density_kg_m3 / nose_radius_m) * speed_mps * speed_mps * speed_mps;
}

}  // namespace starsim::aero
