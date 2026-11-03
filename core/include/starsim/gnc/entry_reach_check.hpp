#pragma once

#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/math/constants.hpp"

#include <vector>

namespace starsim::gnc {

// One dispersed entry interface: offsets from the planned entry and the aerodynamics / mass against the model.
struct EntryDispersion {
    double flight_path_rad = 0.0;  // > 0: shallower
    double speed_mps = 0.0;
    double range_m = 0.0;          // > 0: the interface further from the target (more range to fly)
    double cross_m = 0.0;          // > 0: the interface displaced to the right of the path
    double aero_factor = 1.0;      // lift and drag
    double mass_factor = 1.0;
};

// [E] The dispersed cases the campaigns fly, drawn as they draw them (same generator, seeds and order):
//  - the ship's entry guidance campaign (core/tests/test_entry_guidance_campaign.cpp, 11 cases): flight-path angle
//    +-0.2 deg, speed +-40 m/s, start range +-300 km, +-60 km across, crossflow and flap tables +-10 % (the crossflow
//    factor scales lift and drag together, as that table scales the normal force; the entry mass is known onboard);
//  (a capsule: none yet, see entry_reach_check). Not single-axis extremes, which no campaign flies: those refused every
//  ship pass beyond ~200 km and every Crew Dragon pass (planner plan Task 7 fix round 2).
std::vector<EntryDispersion> ship_entry_campaign_dispersions();

// What the deorbit planner checks a pass's entry against: the onboard entry guidance itself (its corrector, reversal
// rules, hand-over and corridor), flown ahead from the entry interface, and the entry-interface dispersions it is
// flown against in the campaigns.
struct EntryReachCheck {
    EntryGuidanceParameters guidance;   // the onboard guidance's parameters (entry_reach_check)
    config::HeatShieldConfig limits;    // heat rate and load limits (the crew's when lower); 0: not judged
    bool capsule = false;               // no terminal phase: the entry corrector flies to the gate
    // A capsule (no terminal phase): the gate's horizontal miss allowed - the splashdown zone's radius (the session sets
    // it); 0: the guidance's reversal deadband.
    double gate_tolerance_m = 0.0;
    // The dispersed entry interfaces the guidance must fly as well (default: the ship's entry guidance campaign; a
    // capsule: none, entry_reach_check), and the flight-path dispersion's size (the planner's aim step).
    std::vector<EntryDispersion> dispersions = ship_entry_campaign_dispersions();
    double flight_path_dispersion_rad = 0.2 * constants::deg_to_rad;
    // [E] The fly-ahead's corrector cycle (the guidance's: cycle_s, 2 s): a reversal is still timed within its cycle;
    // the bank magnitude is re-solved every 10 s instead of 2 s (planner plan Task 7 fix round 2: phases 0/90/180/270,
    // planned against flown peaks in the report).
    double cycle_s = 10.0;
    double step_s = 2.0;  // predictor step (as the planner's shots)
    bool measure_reach = true;  // EntryReachAssessment::reach_m by flying moved targets (two waves of 8 flights)
};

// The check that matches the onboard entry guidance (its parameters and its heat-shield / crew limits).
EntryReachCheck entry_reach_check(const EntryGuidance& guidance);
// The same from the guidance's parameters alone (no corridor limits): planner fixtures without a vehicle.
EntryReachCheck entry_reach_check(const EntryGuidanceParameters& parameters, bool capsule);

struct EntryReachAssessment {
    bool bracketed = false;        // every case (nominal and dispersed) flies to the target (assess_entry_reach)
    bool within_corridor = false;  // every case's flown peaks stay within the vehicle's limits
    const char* limiting_case = "nominal";  // the first case that does not fly
    bool nominal_flies = false;             // the nominal guided entry flies (gate, terminal, its corridor)
    int cases_flying = 0;                   // of the nominal and the dispersed cases
    // The nominal flies but only dispersed cases of one energy side fail (shallower + faster against the dispersions'
    // sizes): -1 the energy-high side (aim steeper), +1 the energy-low side (aim shallower); 0 otherwise.
    int aim_direction = 0;
    // The planner's crossrange (mean of the one-sign predictions, as DeorbitPlan::crossrange_m) and the reach: the
    // largest |crossrange| every case still flies to (targets moved further out across, to span / 64; measured only for
    // a flyable entry, 0 otherwise; with measure_reach false: |crossrange|).
    double crossrange_m = 0.0;
    double reach_m = 0.0;
    double reach_margin_m = 0.0;        // reach - |crossrange|
    double bank_rad = 0.0;              // the planner's shot bank (warm start)
    double first_sign = 0.0;            // nominal: the guidance's first bank sign (the one ending closer across)
    int reversals = 0;                  // nominal: bank reversals flown
    double reversal_time_s = -1.0;      // ... the last, from the entry interface
    double reversal_speed_mps = 0.0;    // ... its Earth-relative speed
    double handover_time_s = 0.0;       // nominal: hand-over to the terminal phase (the gate for a capsule)
    double terminal_miss_m = 0.0;       // nominal: the terminal steering's best miss from the hand-over
    // The planned entry: the nominal guided flight's peaks (flown, the whole entry - not one prediction's) and the
    // largest over the cases.
    double peak_heat_rate_w_m2 = 0.0;
    double peak_load_g = 0.0;
    double peak_dynamic_pressure_pa = 0.0;
    double worst_peak_heat_rate_w_m2 = 0.0;
    double worst_peak_load_g = 0.0;
    double worst_peak_dynamic_pressure_pa = 0.0;
};

// Checks the entry from the entry-interface state `entry` (Earth-fixed, bank ignored) by flying the onboard entry
// guidance ahead in its own predictor: entry_corrector_cycle every EntryReachCheck::cycle_s (the bank magnitude re-solved
// for the range - raised after a late reversal as in flight -, the reversal timed by its rule), the bank rate-limited
// between, the hand-over at the terminal Mach number once no reversal rolls (terminal_mach_latest at the latest), then
// the terminal steering's reach to the gate from there (its Levenberg-Marquardt step on the miss, without the nose
// sector). A case flies when it reaches the gate, the terminal steering nulls the miss from the hand-over
// (corrector_tolerance_m; a capsule: the gate missed by at most the reversal deadband) - range the corridor cut-back or a
// saturated bank gave up shows there - and its flown peaks stay within the vehicle's limits (heat rate and load of the
// heat shield / crew, the guidance's dynamic-pressure limit; the guidance itself cuts back at constraint_fraction). Every
// dispersed case (EntryReachCheck::dispersions) flies on its own (its model is the guidance's: the scale factors
// converged), all at once on worker threads. `bank_rad`: the planner's shot bank (the warm start and the mean crossrange).
EntryReachAssessment assess_entry_reach(const EntryPredictorModel& model, const EntryPredictorState& entry, double bank_rad,
                                        const EntryTarget& target, double gate_height_m, const EntryReachCheck& check);

}  // namespace starsim::gnc
