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

}  // namespace starsim::gnc
