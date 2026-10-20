#include "starsim/mission/refuelling_costs.hpp"

#include <cmath>

namespace starsim::mission {

double hohmann_delta_v_mps(double from_radius_m, double to_radius_m, double gm_m3ps2) {
    const double a = 0.5 * (from_radius_m + to_radius_m);
    const auto speed = [&](double r, double sma) { return std::sqrt(gm_m3ps2 * (2.0 / r - 1.0 / sma)); };
    return std::abs(speed(from_radius_m, a) - speed(from_radius_m, from_radius_m)) +
           std::abs(speed(to_radius_m, to_radius_m) - speed(to_radius_m, a));
}

double deorbit_delta_v_mps(double radius_m, double perigee_radius_m, double gm_m3ps2) {
    const double a = 0.5 * (radius_m + perigee_radius_m);
    return std::sqrt(gm_m3ps2 / radius_m) - std::sqrt(gm_m3ps2 * (2.0 / radius_m - 1.0 / a));
}

double drag_makeup_kgps(double density_kg_m3, double speed_mps, double drag_coefficient, double area_m2,
                        double exhaust_velocity_mps) {
    return 0.5 * density_kg_m3 * speed_mps * speed_mps * drag_coefficient * area_m2 / exhaust_velocity_mps;
}

}  // namespace starsim::mission
