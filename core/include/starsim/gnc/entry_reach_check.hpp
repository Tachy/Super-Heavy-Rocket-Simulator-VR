#pragma once

#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/math/constants.hpp"

#include <string>
#include <vector>

namespace starsim::gnc {

// One dispersed entry: the interface (a state - a deorbit burn error propagated to it -, or offsets from the planned
// entry) and the aerodynamics / mass against the model.
struct EntryDispersion {
    std::string name;  // German (UI: DeorbitPlan::reach_limiting_case)
    bool has_state = false;  // the interface below (Earth-fixed); the offsets then only describe it (the planner's aim)
    math::Vec3 position_itrf_m, velocity_itrf_mps;
    double flight_path_rad = 0.0;  // > 0: shallower
    double speed_mps = 0.0;
    double range_m = 0.0;          // > 0: the interface further from the target (more range to fly)
    double cross_m = 0.0;          // > 0: the interface displaced to the right of the path
    double aero_factor = 1.0;      // lift and drag (the vehicle's aerodynamics, or the air's density: both act alike)
    double mass_factor = 1.0;      // known onboard (the vehicle's mass)
};

// [E] The air and aerodynamics the entry guidance must absorb at the planned interface: lift and drag together
// +-`aero_fraction` (the ship's entry guidance campaign flies its crossflow table +-10 %, the capsule campaign its truth
// aerodynamics +-10 %; a density error scales both alike), each side alone; and a capsule's mass +-`mass_fraction`
// (capsule campaign: +-3 %, the RCS tank; known onboard). The deorbit burn's own errors are propagated to the interface
// by the planner (DeorbitParameters::burn_dispersion).
std::vector<EntryDispersion> entry_air_dispersions(double aero_fraction, double mass_fraction = 0.0);
inline constexpr double entry_aero_dispersion_fraction = 0.10;
inline constexpr double capsule_mass_dispersion_fraction = 0.03;

// What the deorbit planner checks a pass's entry against: the onboard entry guidance itself (its corrector, reversal
// rules, hand-over and corridor), flown ahead from the entry interface, and the dispersed entries it must fly too (the
// air; the deorbit planner adds its burn's errors propagated to the interface).
struct EntryReachCheck {
    EntryGuidanceParameters guidance;   // the onboard guidance's parameters (entry_reach_check)
    config::HeatShieldConfig limits;    // heat rate and load limits (the crew's when lower); 0: not judged
    bool capsule = false;               // no terminal phase: the entry corrector flies to the gate
    // A capsule (no terminal phase): the gate's horizontal miss allowed - the splashdown zone's radius (the session sets
    // it); 0: the guidance's reversal deadband.
    double gate_tolerance_m = 0.0;
    // The dispersed entries the guidance must fly as well: the air (entry_air_dispersions); the deorbit planner adds its
    // burn errors propagated to the interface. The flight-path step of the planner's aim search (its half).
    std::vector<EntryDispersion> dispersions = entry_air_dispersions(entry_aero_dispersion_fraction);
    double flight_path_dispersion_rad = 0.2 * constants::deg_to_rad;
    // [E] The fly-ahead's corrector cycle (the guidance's: cycle_s, 2 s): a reversal is still timed within its cycle;
    // the bank magnitude is re-solved every 10 s instead of 2 s (planner plan Task 7 fix round 2: phases 0/90/180/270,
    // planned against flown peaks in the report).
    double cycle_s = 10.0;
    // [E] The terminal steering's cycle flown ahead (the guidance's: terminal_cycle_s, 0.5 s; its step limit scaled to
    // the same steering rate). At 10 s near-identical entries (a 0.0004 deg burn-pointing case) handed over 1-8 km
    // across and the gate verdict flipped between them (planner plan Task 7 fix round 3).
    double terminal_cycle_s = 2.0;
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
    std::string limiting_case = "nominal";  // the first case that does not fly
    bool nominal_flies = false;             // the nominal guided entry flies (gate, terminal, its corridor)
    int cases = 0;                          // the nominal and the dispersed cases
    int cases_flying = 0;                   // of them
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
// between, the hand-over at the terminal Mach number once no reversal rolls (terminal_mach_latest at the latest:
// entry_terminal_handover), then the terminal steering flown ahead to the gate from there (its Levenberg-Marquardt step,
// entry_terminal_steering_step, every terminal_cycle_s, without the nose sector). A case flies when it reaches the gate, the terminal steering nulls the miss from the hand-over
// (corrector_tolerance_m; a capsule: the gate missed by at most the reversal deadband) - range the corridor cut-back or a
// saturated bank gave up shows there - and its flown peaks stay within the vehicle's limits (heat rate and load of the
// heat shield / crew, the guidance's dynamic-pressure limit; the guidance itself cuts back at constraint_fraction). Every
// dispersed case (EntryReachCheck::dispersions) flies on its own (the onboard model starts nominal and learns the
// truth's aero scale at the guidance's filter gain), on a pool of hardware-concurrency worker threads. `bank_rad`: the planner's shot bank (the warm start and the mean crossrange).
EntryReachAssessment assess_entry_reach(const EntryPredictorModel& model, const EntryPredictorState& entry, double bank_rad,
                                        const EntryTarget& target, double gate_height_m, const EntryReachCheck& check);

}  // namespace starsim::gnc
