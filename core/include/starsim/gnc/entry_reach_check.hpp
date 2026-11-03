#pragma once

#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

class EntryGuidance;
struct EntryGuidanceParameters;

// What the deorbit planner checks a pass's entry against: the entry guidance's own reversal and hand-over rules, its
// corridor limits and the entry-interface dispersions it is designed for.
struct EntryReachCheck {
    // The last bank reversal must start before the hand-over: Mach terminal_mach (the ship's terminal phase takes over
    // there; a reversal still rolling stretches it to terminal_mach_latest) or, without a terminal phase (a capsule),
    // the guidance's min_reversal_speed_mps. 0: none of that kind.
    double handover_mach = 5.0;            // EntryGuidanceParameters::terminal_mach
    double min_reversal_speed_mps = 600.0; // EntryGuidanceParameters::min_reversal_speed_mps
    double reversal_deadband_m = 2000.0;   // EntryGuidanceParameters::reversal_crossrange_base_m
    // Corridor (EntryGuidance::violates): peak heat rate and load within constraint_fraction of the heat shield's
    // limits, dynamic pressure within its limit. 0: not checked.
    double max_heat_rate_w_m2 = 0.0;
    double max_load_g = 0.0;
    double max_dynamic_pressure_pa = 0.0;
    double constraint_fraction = 0.9;
    // [E] The entry-interface dispersions the guidance is flown against, each case alone (steeper; slower; aerodynamics
    // and mass up and down - lift and drag scaled together, as the truth's tables are) must still leave the target
    // bracketed. Defaults: the ship's entry guidance campaign (core/tests/test_entry_guidance_campaign.cpp: flight-path
    // angle +-0.2 deg, speed +-40 m/s, crossflow and flap tables +-10 %); a capsule: entry_reach_check.
    double dispersion_flight_path_rad = 0.2 * constants::deg_to_rad;
    double dispersion_speed_mps = 40.0;
    double dispersion_aero_fraction = 0.1;
    double dispersion_mass_fraction = 0.0;
    double step_s = 2.0;  // predictor step (as the planner's shots)
};

// The check that matches the onboard entry guidance (its parameters and its heat-shield / crew limits). A capsule is
// checked against its own campaign's dispersions (core/tests/test_capsule_entry.cpp: entry angle +-0.1 deg, mass +-3 %,
// truth aero +-10 %).
EntryReachCheck entry_reach_check(const EntryGuidance& guidance);
// The same from the guidance's parameters alone (no corridor limits): planner fixtures without a vehicle.
EntryReachCheck entry_reach_check(const EntryGuidanceParameters& parameters, bool capsule);

struct EntryReachAssessment {
    bool bracketed = false;        // every case (nominal and dispersed) can null the crossrange as the guidance flies it
    bool within_corridor = false;  // the planned entry stays within the corridor limits
    // The planner's crossrange (mean of the one-sign predictions, as DeorbitPlan::crossrange_m) and the reach it is
    // allowed: |crossrange| + the smallest margin over the cases (the margin of a case: how far the target lies inside
    // the crossrange interval its reversal window covers, less the guidance's deadband; < 0: outside).
    double crossrange_m = 0.0;
    double reach_m = 0.0;
    double nominal_margin_m = 0.0;
    double dispersion_reserve_m = 0.0;  // nominal margin less the smallest margin (what the dispersions use up)
    const char* limiting_case = "nominal";  // the case with the smallest margin
    double bank_rad = 0.0;              // nominal bank magnitude flown
    double first_sign = 0.0;            // the guidance's first bank sign (the one ending closer across)
    double reversal_time_s = -1.0;      // nominal: the reversal that ends on the target, from the entry interface
    double reversal_speed_mps = 0.0;    // ... its Earth-relative speed
    double handover_time_s = 0.0;       // nominal: the end of the reversal window
    double peak_heat_rate_w_m2 = 0.0;   // planned entry (reversal flown)
    double peak_load_g = 0.0;
    double peak_dynamic_pressure_pa = 0.0;
};

// Checks the entry from the entry-interface state `entry` (Earth-fixed, bank ignored) with the guidance's predictor, as
// the guidance flies it: the bank magnitude for the range (the mean downrange of the two signs on the target, solved per
// dispersed case; the nominal case flies `bank_rad`, the planner's shot), the first sign the one ending closer across,
// one reversal (flown at the model's bank rate: the reversal's loss) whose time ends on the target. The reversal window
// ends at the hand-over: holding the first sign to the hand-over and reversing there must end beyond the target, so the
// terminal phase never inherits a one-sided lateral need (its nose sector cannot turn the track - planner plan Task 7,
// ship return phase 90). Every dispersed case (EntryReachCheck::dispersion_*) must still bracket the target.
EntryReachAssessment assess_entry_reach(const EntryPredictorModel& model, const EntryPredictorState& entry, double bank_rad,
                                        const EntryTarget& target, double gate_height_m, const EntryReachCheck& check);

}  // namespace starsim::gnc
