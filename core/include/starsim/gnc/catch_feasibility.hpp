#pragma once

#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/report/report.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::gnc {

struct CatchFeasibility {
    bool feasible = true;
    report::Report reason;  // ok when feasible
};

// Crew escape: can the onboard computer still try the catch? Not when an engine of the flip groups has failed (the
// catch campaign's engine-failure case ends in the upright abort) or when the propellant the landing can burn (the
// header tanks at the engines' mixture ratio, landing_feed_propellant_kg, less the unusable header propellant) is below
// `catch_floor_kg` (ship_catch_propellant_floor_kg).
CatchFeasibility catch_feasibility(const vehicle::Vehicle& ship, const std::vector<std::string>& flip_groups,
                                   double catch_floor_kg);

// The least propellant a catch needs from the gate [D] (planner plan 2026-11-03 Task 4 fix round 2): a lower bound, no
// guidance can catch with less.
//  Stop - the ideal vertical stop of the CG from the gate sink v over the drop h to the catch at the catch sink v_f with
//         the strongest landing thrust (the first landing set at its planned maximum, ship_landing_parameters'
//         gate_flip.brake_accel_mps2, on the live mass): fall, then full braking a - g to the catch,
//         v_i^2 = ((v^2 + 2 g h)(a - g) + g v_f^2) / a, dv = (v_i - v_f) a / (a - g) (gravity loss of the burn included,
//         none of a real profile's).
//  Flip - the flip's thrust across the path (ship_flip_push's speed_mps on the live mass): impulse the vertical stop
//         cannot use (the air may stop the push, the propellant is spent).
// Rocket equation to the light landing mass (ship_landing_masses: dry + unusable header propellant) at the landing
// engines' sea-level Isp; the live mass at the gate is that plus the need (fixed point).
struct CatchPropellantNeed {
    double stop_dv_mps = 0.0;
    double flip_dv_mps = 0.0;
    double exhaust_velocity_mps = 0.0;
    double end_mass_kg = 0.0;
    double propellant_kg = 0.0;  // usable, above the unusable header propellant
};
// drop_m: the CG's fall from the gate to the catch; gate_sink_mps: at the gate; catch_sink_mps: at the catch.
CatchPropellantNeed ship_catch_propellant_need(const config::VehicleConfig& vehicle, const config::StageConfig& ship,
                                               double drop_m, double gate_sink_mps, double catch_sink_mps);

// The crewed ship's give-up floor [D]: the need from the nominal gate (gate_height_m above the site ground at the
// reference sink gate_sink_mps) to the catch (pins at the arm tops `arm_top_height_m`, the CG of the heavy landing mass
// below them; 0: no tower, the ground) times (1 + margin) [config crew_escape catch_propellant_margin].
double ship_catch_propellant_floor_kg(const config::VehicleConfig& vehicle, const config::StageConfig& ship,
                                      double gate_height_m, double gate_sink_mps, double arm_top_height_m, double margin);

}  // namespace starsim::gnc
