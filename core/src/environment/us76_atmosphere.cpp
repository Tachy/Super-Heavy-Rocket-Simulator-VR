#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

// U.S. Standard Atmosphere 1976 (NOAA-S/T 76-1562) constants.
constexpr double gas_constant = 8314.32;            // R*, J / (kmol K)
constexpr double sea_level_molar_mass = 28.9644;    // M0, kg / kmol
constexpr double effective_earth_radius_m = 6356766.0;  // r0 for geopotential altitude
constexpr double heat_capacity_ratio = 1.4;
constexpr double boltzmann = 1.380649e-23;               // J/K
constexpr double collision_diameter_m = 3.65e-10;         // sigma, US76 mean collision diameter

struct Layer {
    double base_geopotential_m;
    double lapse_rate_k_per_m;
    double base_temperature_k;
    double base_pressure_pa;
};

// Table 4 of US76: layers up to 84.852 km geopotential (= 86 km geometric).
constexpr std::array<Layer, 7> layers{{
    {0.0, -0.0065, 288.15, 101325.0},
    {11000.0, 0.0, 216.65, 22632.06},
    {20000.0, 0.001, 216.65, 5474.889},
    {32000.0, 0.0028, 228.65, 868.0187},
    {47000.0, 0.0, 270.65, 110.9063},
    {51000.0, -0.0028, 270.65, 66.93887},
    {71000.0, -0.002, 214.65, 3.956420},
}};
constexpr double upper_limit_geopotential_m = 84852.0;

struct UpperPoint {
    double altitude_m;
    double temperature_k;
    double density_kg_m3;
};

// US76 tabulated values, geometric altitude (Table 1 / Appendix tables, selected points).
constexpr std::array<UpperPoint, 22> upper{{
    {86e3, 186.87, 6.958e-6},   {90e3, 186.87, 3.416e-6},   {100e3, 195.08, 5.604e-7},
    {110e3, 240.00, 9.708e-8},  {120e3, 360.00, 2.222e-8},  {130e3, 469.27, 8.152e-9},
    {140e3, 559.63, 3.831e-9},  {150e3, 634.39, 2.076e-9},  {160e3, 696.29, 1.233e-9},
    {180e3, 790.07, 5.194e-10}, {200e3, 854.56, 2.541e-10}, {250e3, 941.33, 6.073e-11},
    {300e3, 976.01, 1.916e-11}, {350e3, 990.06, 7.014e-12}, {400e3, 995.83, 2.803e-12},
    {450e3, 998.22, 1.184e-12}, {500e3, 999.24, 5.215e-13}, {600e3, 999.85, 1.137e-13},
    {700e3, 999.97, 3.070e-14}, {800e3, 999.99, 1.136e-14}, {900e3, 1000.0, 5.759e-15},
    {1000e3, 1000.0, 3.561e-15},
}};

// lambda = 1 / (sqrt(2) pi sigma^2 n), n = p / (k T)  (US76 eq. 21)
double mean_free_path(double pressure_pa, double temperature_k) {
    if (pressure_pa <= 0.0) return std::numeric_limits<double>::infinity();
    const double n = pressure_pa / (boltzmann * temperature_k);
    return 1.0 / (std::sqrt(2.0) * constants::pi * collision_diameter_m * collision_diameter_m * n);
}

double speed_of_sound(double temperature_k) {
    return std::sqrt(heat_capacity_ratio * gas_constant * temperature_k / sea_level_molar_mass);
}

AtmosphereState lower_atmosphere(double altitude_m) {
    const double h = effective_earth_radius_m * altitude_m / (effective_earth_radius_m + altitude_m);
    const Layer* layer = &layers[0];
    for (const auto& candidate : layers) {
        if (h >= candidate.base_geopotential_m) layer = &candidate;
    }
    const double dh = h - layer->base_geopotential_m;
    const double g0 = constants::standard_gravity_mps2;
    const double temperature = layer->base_temperature_k + layer->lapse_rate_k_per_m * dh;
    double pressure;
    if (layer->lapse_rate_k_per_m == 0.0) {
        pressure = layer->base_pressure_pa *
                   std::exp(-g0 * sea_level_molar_mass * dh / (gas_constant * layer->base_temperature_k));
    } else {
        pressure = layer->base_pressure_pa *
                   std::pow(layer->base_temperature_k / temperature,
                            g0 * sea_level_molar_mass / (gas_constant * layer->lapse_rate_k_per_m));
    }
    const double density = pressure * sea_level_molar_mass / (gas_constant * temperature);
    return {density, pressure, temperature, speed_of_sound(temperature), mean_free_path(pressure, temperature)};
}

AtmosphereState upper_atmosphere(double altitude_m) {
    if (altitude_m >= upper.back().altitude_m) {
        return {0.0, 0.0, upper.back().temperature_k, speed_of_sound(upper.back().temperature_k),
                std::numeric_limits<double>::infinity()};
    }
    const auto next = std::upper_bound(upper.begin(), upper.end(), altitude_m,
                                       [](double z, const UpperPoint& p) { return z < p.altitude_m; });
    const auto& hi = *next;
    const auto& lo = *(next - 1);
    const double f = (altitude_m - lo.altitude_m) / (hi.altitude_m - lo.altitude_m);
    const double density = std::exp(std::log(lo.density_kg_m3) + f * std::log(hi.density_kg_m3 / lo.density_kg_m3));
    const double temperature = lo.temperature_k + f * (hi.temperature_k - lo.temperature_k);
    const double pressure = density * gas_constant * temperature / sea_level_molar_mass;
    return {density, pressure, temperature, speed_of_sound(temperature), mean_free_path(pressure, temperature)};
}

}  // namespace

AtmosphereState Us76Atmosphere::at(double altitude_m) const {
    altitude_m = std::max(altitude_m, -5000.0);
    const double h = effective_earth_radius_m * altitude_m / (effective_earth_radius_m + altitude_m);
    return h < upper_limit_geopotential_m ? lower_atmosphere(altitude_m) : upper_atmosphere(altitude_m);
}

}  // namespace starsim::environment
