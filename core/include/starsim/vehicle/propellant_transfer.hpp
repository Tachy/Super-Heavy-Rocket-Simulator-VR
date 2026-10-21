#pragma once

#include <cstddef>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/boil_off.hpp"

namespace starsim::vehicle {

class Vehicle;

// Ship-to-ship propellant transfer physics (refuelling r4).

// Pressure-driven line flow: mdot = C_d A sqrt(2 rho dp) (orifice law; the valves close against back-flow: 0 for
// dp <= 0). The hydrostatic head under the settling push (~0.2 kPa over 20 m of LOX at 1e-4 g) is left out.
double line_mass_flow_kgps(double discharge_area_m2, double density_kg_m3, double pressure_difference_pa);

// C_d A of two coupled connectors: the smaller one governs.
double pair_discharge_area_m2(const config::TransferConnectorConfig& a, const config::TransferConnectorConfig& b);

// Where a tank's liquid lies relative to its outlet (the aft dome's low point, body -x).
struct OutletCheck {
    bool covered = false;
    double bond_number = 0.0;  // rho |a| r^2 / sigma at the tank centre
    double tilt_rad = 0.0;     // between the apparent gravity and -x
};

// The apparent gravity at the tank centre (on the axis, mid-way along its full-load length) settles the liquid when
// settled_weight(Bo) reaches 1 (Bo >= 10, r2); the outlet is covered when it is settled, within `max_tilt_rad` of -x
// and the tank holds liquid. A propellant without a surface tension counts as settled.
OutletCheck outlet_check(const Vehicle& vehicle, size_t stage_index, size_t tank_index, const LiquidEnvironment& liquid,
                         double max_tilt_rad);

// Time for liquid at rest to cross a tank of `length_m` under `acceleration_mps2`: sqrt(2 L / a) [E] (kinematic, no
// slosh or drainage); infinite without acceleration.
double settling_time_s(double length_m, double acceleration_mps2);

// Refuelling f3: the shortest travel of the liquid in a tank (a near-full tank would settle and unsettle at once) [E],
// and the largest tilt of the apparent gravity from -x that still covers an outlet (the r4 transfer's value).
constexpr double settling_min_travel_m = 1.0;
constexpr double settling_max_tilt_rad = 30.0 * constants::deg_to_rad;
// The settling push before an ignition starts this many settling times ahead [E].
constexpr double settling_lead_factor = 1.25;

// The cold-gas push that settles stage `stage_index` before an ignition (refuelling f3): the full pure +x translation
// capability; lead = settling_lead_factor x sqrt(2L/a), gas = its mass flow over the lead. Zero without a need to settle
// or without +x capability.
struct SettlingPush {
    double acceleration_mps2 = 0.0;
    double lead_s = 0.0;
    double gas_kg = 0.0;
};
SettlingPush settling_push(const Vehicle& vehicle, size_t stage_index);

}  // namespace starsim::vehicle
