#pragma once

#include "starsim/config/vehicle_config.hpp"

namespace starsim::config {

// Loads a stage with `total_kg` of oxidizer + fuel: the header tanks first (up to their capacity, in their own
// ratio), the rest into the main tanks in their capacity ratio. Capacities are TankConfig::capacity_kg (the
// configured full loads); Rcs tanks are untouched. Without headers: every oxidizer and fuel tank scaled alike.
void fill_tanks(StageConfig& stage, double total_kg);

// The same per propellant: `oxidizer_kg` over the oxidizer tanks, `fuel_kg` over the fuel tanks, headers first.
void fill_tanks(StageConfig& stage, double oxidizer_kg, double fuel_kg);

}  // namespace starsim::config
