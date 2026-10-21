#pragma once

#include <string>

#include "starsim/environment/gravity.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::gnc {

struct DeorbitParameters {
    double entry_interface_height_m = 120000.0;                         // b2's entry interface
    double entry_flight_path_rad = -1.0 * constants::deg_to_rad;        // [E] b2's nominal (campaign +-0.2 deg)
    // [E] the plan flies this bank (both signs averaged: reversals), so that the entry guidance has range to spare
    // either way (b2 campaign: nominal ~45-50 deg at 7,500 km).
    double nominal_bank_rad = 50.0 * constants::deg_to_rad;
    double max_crossrange_m = 60000.0;     // [E] b2 campaign's dispersion: tested reach of the reversals
    double search_window_s = 2.0 * 86400.0;
    double lead_s = 600.0;                 // earliest burn after planning (turn to the burn attitude)
    double gate_height_m = 800.0;          // as EntryGuidanceParameters
    double sample_s = 60.0;                // coast sampling of the pass search
    // > 0: the burn is flown as a finite burn of this thrust centred on the node, along the node's fixed inertial
    // direction (the executor ignites half the duration early); 0: impulsive. A Draco deorbit lasts ~13 min (d3).
    double burn_thrust_n = 0.0;
    // > 0: the entry predictor's mass (a capsule drops its trunk after the burn); 0: the mass after the burn.
    double entry_mass_kg = 0.0;
    // Orbital drag of the coast, C_D A / m (US76 density, air co-rotating with the Earth); 0: drag-free. A tracked
    // value (estimate_drag_area_per_mass): drag-free, a 4.7-day coast at ~200 km put the pass ~80 km off its plan, out
    // of the reversals' 60 km reach (l5 Task 10, b4 from the aircatch orbit).
    double drag_area_per_mass_m2pkg = 0.0;
    // Refuelling r2: mean boil-off of the main tanks while the ship waits for its burn (vehicle::orbit_average_boil_off);
    // the propellant at the burn is the available propellant less this rate times the wait. 0: none. An offline
    // planning knob: FlightSession leaves it 0 and checks the boil-off by flying a copy ahead (boil_off_shortfall),
    // which also sets DeorbitPlan::boil_off_kg.
    double boil_off_kgps = 0.0;
};

struct DeorbitPlan {
    bool feasible = false;
    std::string reason;                   // German (UI), when not feasible
    BurnNode node;                        // retrograde, impulsive at node.epoch
    double delta_v_mps = 0.0;
    time::Epoch entry_interface_epoch;    // coasting after the impulse
    double entry_flight_path_rad = 0.0;
    double entry_speed_mps = 0.0;         // relative to the Earth
    double downrange_error_m = 0.0;       // nominal bank (> 0: overshoot)
    double crossrange_m = 0.0;            // nominal bank, both signs averaged
    double propellant_kg = 0.0;           // for the burn (rocket equation)
    double boil_off_kg = 0.0;             // boiled off until the burn (boil_off_kgps; the session: flown ahead)
    bool short_of_reserve = false;        // not feasible only because the landing reserve would be cut (plan complete)
    int passes_checked = 0;
};

// Deorbit burn of a ship in orbit for a return to `target` (onboard computer, simulation thread: it coasts with the
// full gravity model and the Earth orientation). Searches the passes in the window in time order (closest approach of
// the ground track to the target); for each, shoots the burn epoch and the retrograde delta-v so that the coast after
// the impulse reaches the entry interface at the entry flight-path angle and the entry predictor, flying the nominal
// bank, ends at the gate; the first pass whose predicted crossrange is within reach is taken. Refused (with a reason)
// when no pass is found, or the burn would leave less than `reserve_propellant_kg`.
// `mass_kg`: the ship now; `exhaust_velocity_mps`: of the deorbit engine set.
DeorbitPlan plan_deorbit(const math::StateVector& state_gcrf, time::Epoch epoch, const environment::GravityModel& gravity,
                         const frames::BodyOrientation& earth, EntryPredictorModel model, const EntryTarget& target,
                         double mass_kg, double exhaust_velocity_mps, double available_propellant_kg,
                         double reserve_propellant_kg, const DeorbitParameters& parameters = {});

// Coast with the planner's dynamics (gravity + drag at `drag_area_per_mass_m2pkg`).
math::StateVector coast_with_drag(const environment::GravityModel& gravity, const math::StateVector& state_gcrf,
                                  time::Epoch epoch, time::Epoch end, double drag_area_per_mass_m2pkg);

// Orbit determination of the drag (onboard tracking): the C_D A / m whose coast from the tracked state `reference`
// at `reference_epoch` arrives where the ship is now (along-track position: drag shows as a lead growing with t^2).
// Returns `guess` when the arc is too short to tell (the lead is below ~1 km).
double estimate_drag_area_per_mass(const environment::GravityModel& gravity, const math::StateVector& reference,
                                   time::Epoch reference_epoch, const math::StateVector& now, time::Epoch now_epoch,
                                   double guess_m2pkg);

}  // namespace starsim::gnc
