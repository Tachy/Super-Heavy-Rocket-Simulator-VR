#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

enum class EntryPhase { Entry, Terminal, Gate, Exit };  // Exit: an aerocapture left the atmosphere

const char* to_string(EntryPhase phase);

struct EntryGuidanceParameters {
    // [E] above the target ground: the flip starts here (b3). Flight 13 relit at ~0.95 km / 101 m/s and flipped in
    // 2-3 s losing ~200 m (webcast telemetry); at 950 m the entry campaign's hardest case (4) misses by 212 m, at
    // 800 m all twelve reach the gate - open (b3 ledger).
    double gate_height_m = 800.0;
    double cycle_s = 2.0;            // re-prediction interval
    double max_bank_rad = 150.0 * constants::deg_to_rad;
    // Bank rate limits [E]: 1 deg/s hypersonic (faster departs in alpha, b1; seen again with weak flaps), 3 deg/s
    // below Mach 5 (dense air, more flap authority: reversals late in the entry must be quick), linear in between.
    double bank_rate_radps = 1.0 * constants::deg_to_rad;
    double fast_bank_rate_radps = 3.0 * constants::deg_to_rad;
    double fast_bank_mach_low = 5.0;
    double fast_bank_mach_high = 10.0;
    double constraint_fraction = 0.9;                      // of the heat-rate and load limits
    double max_dynamic_pressure_pa = 25000.0;              // [E]
    double terminal_mach = 5.0;                            // [E] hand-over to the terminal steering ...
    double terminal_mach_latest = 3.0;                     // [E] ... or here if a bank reversal was still running
    double terminal_min_alpha_rad = 50.0 * constants::deg_to_rad;        // [E] most lift (trimmed range, b1)
    double terminal_alpha_authority_rad = 20.0 * constants::deg_to_rad;  // [E] steering around the alpha schedule
    double terminal_cycle_s = 0.5;               // steering update (three predictions)
    double terminal_prediction_step_s = 0.5;
    double terminal_newton_step = 0.1;           // finite difference of the steering fraction
    double terminal_max_step = 0.3;              // [E] steering change per cycle (Levenberg-Marquardt step limit)
    // [E] the commanded attitude turns at most this fast in the terminal phase (as the bank rate below Mach 5)
    double terminal_slew_rate_radps = 3.0 * constants::deg_to_rad;
    // [E] with a nose direction in the target (EntryPredictorModel::nose_*): the lift direction - and the nose - turns
    // about the air velocity at this rate; while it turns the steering is held at the nose fraction (no lift to speak of)
    // the nose sector (EntryPredictorModel::nose_*) [E]
    double terminal_nose_fraction = 0.2;
    double terminal_nose_sector_rad = 10.0 * constants::deg_to_rad;  // at the gate (20 deg: the flip pushed past the arms)
    double terminal_nose_sector_wide_rad = 30.0 * constants::deg_to_rad;  // this high above the gate and more:
    double terminal_nose_sector_wide_height_m = 4000.0;                   // crossrange authority after the turn
    double terminal_nose_min_fraction = 0.02;
    double terminal_nose_max_fraction = 0.5;
    // 3 deg/s (b4 campaign: 13 of 13 caught; at 5 deg/s the flaps' roll limit cycle departed or drifted 3 of 13, b3's 5
    // deg/s was set before the prediction modelled the turn)
    double terminal_nose_turn_rate_radps = 3.0 * constants::deg_to_rad;
    // [E] the commanded attitude follows the turn at this multiple of its rate: at 2x the pitch-up to the turn's alpha
    // together with the turn saturated the flaps and set off a roll limit cycle (departures, b4 campaign)
    double terminal_nose_slew_factor = 1.5;
    double terminal_nose_turned_rad = 5.0 * constants::deg_to_rad;  // the turn is over within this
    double terminal_nose_max_turn_step_rad = 5.0 * constants::deg_to_rad;  // [E] steering angle change per cycle in the sector
    // Reversals: planned by the corrector (time of the next reversal for zero crossrange); without a bracket the
    // sign closer across wins if it gains more than this [E]. None below min_reversal_speed (belly flop's job).
    double reversal_crossrange_base_m = 2000.0;
    double min_reversal_speed_mps = 600.0;
    double min_reversal_interval_s = 20.0;    // [E] dwell between reversals
    double scale_filter_gain = 0.3;   // per cycle, measured / modelled aerodynamic acceleration
    double scale_min_acceleration_mps2 = 1.0;
    bool predictor_j2 = true;         // false: central gravity (test scenarios without J2)
    bool terminal_phase = true;       // false: no terminal steering (a capsule: bank to the gate)
    // > 0: below this speed the bank is held (no more corrector cycles). A capsule falling steeply at subsonic speed has
    // little lift authority left, and its bank from the vertical plane degenerates: the corrector swung to lift down
    // and back (d3 campaign: 0.4 km predicted at 690 m/s, 3.8 km flown). 0: never (the ship's terminal phase takes over).
    double hold_bank_below_speed_mps = 0.0;
    // Lunar l5: the predictor carries skip arcs (EntryPredictorModel::carry_skips) and looks this far ahead.
    bool carry_skips = false;
    double max_time_s = 3000.0;
    // > 0: aerocapture - one pass to this exit apoapsis height (no gate, no terminal phase).
    double aerocapture_apoapsis_m = 0.0;
    double target_inclination_rad = -1.0;  // aerocapture: reversals on the exit orbit's inclination (< 0: none)
    double inclination_deadband_rad = 0.2 * constants::deg_to_rad;  // [E]
    double exit_height_m = 130e3;  // aerocapture: phase Exit once above this after the dip below 80 km

    // A capsule (Crew Dragon, d3): gate at the drogue deployment height (~18,000 ft [S: NASA Crew-2]), bank rate
    // 10 deg/s [E] throughout (Dracos; Apollo rolled at up to ~20 deg/s), no terminal phase, reversals to lower speed.
    static EntryGuidanceParameters capsule() {
        EntryGuidanceParameters p;
        p.gate_height_m = 5500.0;
        p.bank_rate_radps = 10.0 * constants::deg_to_rad;
        p.fast_bank_rate_radps = 10.0 * constants::deg_to_rad;
        p.max_bank_rad = 165.0 * constants::deg_to_rad;
        p.terminal_phase = false;
        // no terminal phase removes the last crossrange: reversals keep going to lower speed with a narrower deadband [E]
        p.reversal_crossrange_base_m = 500.0;
        p.min_reversal_speed_mps = 500.0;
        p.hold_bank_below_speed_mps = 500.0;  // [E]
        return p;
    }
    // Lunar return (l5): the predictor carries skip arcs (Orion-like skip entry, Brunner & Lu 2008 principle) and
    // looks further ahead.
    static EntryGuidanceParameters lunar_return() {
        EntryGuidanceParameters p;
        p.carry_skips = true;
        p.max_time_s = 6000.0;
        return p;
    }
    // Aerocapture (l5, "Aircatch"): one pass, the bank magnitude by the predicted exit apoapsis (numerical
    // predictor-corrector, the principle of FNPAG [S: Lu, Cerimele, Tigges, Matz, AIAA 2015-1520]), constant bank per
    // cycle; reversals on the exit orbit's inclination; no terminal phase.
    static EntryGuidanceParameters aerocapture(double apoapsis_m, double inclination_rad) {
        EntryGuidanceParameters p;
        p.aerocapture_apoapsis_m = apoapsis_m;
        p.target_inclination_rad = inclination_rad;
        p.terminal_phase = false;
        p.max_time_s = 3000.0;
        return p;
    }
};

struct EntryCommand {
    EntryPhase phase = EntryPhase::Entry;
    double alpha_rad = 0.0;              // setpoint (entry phase)
    double bank_rad = 0.0;               // setpoint now (rate limited)
    math::Vec3 axis_gcrf;                // for AttitudeController::steer
    math::Vec3 roll_reference_gcrf;
    // Terminal phase: angular velocity of the commanded attitude (feedforward: a PD controller tracking the nose turn's
    // 5 deg/s ramp alone lags ~2 zeta w / wn = ~18 deg, which the belly flop loses as angle of attack)
    math::Vec3 rate_gcrf_radps;
    double downrange_error_m = 0.0;      // latest prediction (> 0: overshoot)
    double crossrange_m = 0.0;           // > 0: target right of the predicted track
    double heading_error_rad = 0.0;      // target azimuth - track azimuth (> 0: target to the right)
    bool constrained = false;            // bank limited by a corridor constraint
    double drag_scale = 1.0;
    double lift_scale = 1.0;
    double exit_apoapsis_height_m = 0.0;  // aerocapture: latest prediction (< 0: hyperbolic)
};

// Entry guidance of the ship (onboard computer), from the measured state each cycle:
//  Entry     - numeric predictor-corrector: the bank magnitude whose prediction ends at the target (warm-started
//              bracket + Illinois); peaks of heat rate, load and dynamic pressure beyond their limits cut the bank
//              back (range is given up, not the vehicle); bank reversals when the predicted crossrange overshoots a
//              deadband that narrows with the range to go; online drag / lift scale factors from the accelerometer.
//              (A heading-error corridor does not work here: the long glide on one bank sign curves the track, the
//              great-circle heading to the target differs by 15-20 deg on a trajectory that ends on it.)
//  Terminal  - below the hand-over Mach number (supersonic to the subsonic belly flop): like a skydiver, the belly
//              faces the flow (no sideslip) and the lift is pointed - its direction about the air velocity and its
//              size by the angle of attack (90 deg: none, the minimum alpha: most) - by a Newton step per cycle on
//              the predicted horizontal miss at the gate (two directions, three predictions).
//  Gate      - the gate height above the target is reached (b3 flips and lands).
class EntryGuidance {
public:
    // `ship`: the entering single-stage vehicle (flaps, heat shield); its current mass properties set the trim.
    // `radiative`: the predictor adds equilibrium radiative heating (lunar-return speeds); null: convective only.
    EntryGuidance(const vehicle::FlightBody& ship, const AlphaSchedule& schedule, EntryTarget target,
                  EntryGuidanceParameters parameters = {},
                  std::shared_ptr<const aero::RadiativeHeatingTable> radiative = nullptr);
    // A capsule (Vehicle::capsule_aero): fixed trim angle of attack, bank only (EntryAeroTable::capsule); the load limit
    // is the crew's ([stage.heat_shield] crew_load_limit_g) when lower than the structure's.
    EntryGuidance(const vehicle::FlightBody& capsule, EntryTarget target,
                  EntryGuidanceParameters parameters = EntryGuidanceParameters::capsule());
    bool capsule() const { return capsule_; }

    EntryCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    EntryPhase phase() const { return phase_; }
    const EntryTarget& target() const { return target_; }
    const EntryGuidanceParameters& parameters() const { return parameters_; }
    void skip_to(EntryPhase phase) { phase_ = phase; }

    struct Solution {
        double bank_rad = 0.0;  // magnitude
        EntryPrediction prediction;
        bool constrained = false;
        int predictions = 0;
    };
    // Corrector: bank magnitude in [0, max_bank] for zero downrange error with the bank sign `sign` (+-1), the
    // search starting at `warm_start_rad`; then the corridor constraints. A reversal planned in `state`
    // (reversal_time_s) is flown by the prediction.
    Solution solve(const EntryPredictorModel& model, const EntryPredictorState& state, double heat_load_j_m2,
                   double sign, double warm_start_rad) const;
    const EntryPredictorModel& model() const { return model_; }

private:
    void entry_cycle(const vehicle::FlightBody& body, const math::Vec3& r_itrf, const math::Vec3& v_itrf);
    void aerocapture_cycle(const vehicle::FlightBody& body, const EntryPredictorState& state);
    void terminal(const vehicle::FlightBody& body, const math::Mat3& itrf_to_gcrf, const math::Vec3& r_itrf,
                  const math::Vec3& v_itrf, double mach, double dt_s);
    // Horizontal miss at the gate predicted with a terminal steering vector (and the time to go).
    math::Vec3 terminal_miss(const vehicle::FlightBody& body, const math::Vec3& r_itrf, const math::Vec3& v_itrf,
                               const math::Vec3& steering, double* time_to_go_s) const;
    bool violates(const EntryPrediction& p) const;
    // Corrector miss (> 0: too much energy left - more bank).
    double miss(const EntryPrediction& p) const;
    bool aerocapture() const { return parameters_.aerocapture_apoapsis_m > 0.0; }
    // Aerocapture: |predicted exit inclination - target| (pi when the prediction does not exit).
    double inclination_error(const EntryPrediction& p) const;

    EntryTarget target_;
    EntryGuidanceParameters parameters_;
    EntryPredictorModel model_;
    config::HeatShieldConfig limits_;
    double belly_azimuth_rad_ = 0.0;
    bool capsule_ = false;
    void configure_model(const vehicle::FlightBody& body);
    EntryPhase phase_ = EntryPhase::Entry;
    bool started_ = false;
    bool dipped_ = false;  // aerocapture: has been below 80 km
    time::Epoch last_update_;
    time::Epoch last_cycle_;
    time::Epoch last_scale_update_;
    double bank_rad_ = 0.0;          // setpoint now (rate limited)
    double bank_target_rad_ = 0.0;   // corrector output with sign
    double bank_magnitude_rad_ = 0.0;
    double sign_ = 0.0;              // 0: not chosen yet
    double time_since_reversal_s_ = 1.0e9;
    EntryCommand command_;
    bool terminal_started_ = false;
    bool terminal_predicted_ = false;
    math::Quat terminal_attitude_gcrf_;  // commanded attitude in the terminal phase (slew limited)
    math::Vec3 terminal_steering_;        // velocity frame (steering_basis): lift direction x fraction (|.| <= 1)
    math::Vec3 terminal_lift_direction_;  // last lift direction (kept when the steering is ~0)
    bool terminal_basis_frozen_ = false;
    bool nose_turning_ = false;  // the lift direction still turns to the nose sector
    bool handed_over_ = false;  // the terminal phase follows the entry phase (not placed by skip_to)
    math::Vec3 terminal_e1_, terminal_e2_;  // steering basis (see EntryPredictorState::basis_frozen)
    math::Vec3 terminal_zem_itrf_;             // predicted horizontal miss at the gate
    double terminal_prediction_age_s_ = 0.0;
};

}  // namespace starsim::gnc
