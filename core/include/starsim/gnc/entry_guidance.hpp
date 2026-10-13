#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

enum class EntryPhase { Entry, Terminal, Gate };

const char* to_string(EntryPhase phase);

struct EntryGuidanceParameters {
    double gate_height_m = 950.0;    // [D] above the target ground: the flip starts here (b3); flight 13 relit at
                                     // ~0.95 km / 101 m/s, flipped in 2-3 s losing ~200 m (webcast telemetry)
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
    // Reversals: planned by the corrector (time of the next reversal for zero crossrange); without a bracket the
    // sign closer across wins if it gains more than this [E]. None below min_reversal_speed (belly flop's job).
    double reversal_crossrange_base_m = 2000.0;
    double min_reversal_speed_mps = 600.0;
    double min_reversal_interval_s = 20.0;    // [E] dwell between reversals
    double scale_filter_gain = 0.3;   // per cycle, measured / modelled aerodynamic acceleration
    double scale_min_acceleration_mps2 = 1.0;
    bool predictor_j2 = true;         // false: central gravity (test scenarios without J2)
};

struct EntryCommand {
    EntryPhase phase = EntryPhase::Entry;
    double alpha_rad = 0.0;              // setpoint (entry phase)
    double bank_rad = 0.0;               // setpoint now (rate limited)
    math::Vec3 axis_gcrf;                // for AttitudeController::steer
    math::Vec3 roll_reference_gcrf;
    double downrange_error_m = 0.0;      // latest prediction (> 0: overshoot)
    double crossrange_m = 0.0;           // > 0: target right of the predicted track
    double heading_error_rad = 0.0;      // target azimuth - track azimuth (> 0: target to the right)
    bool constrained = false;            // bank limited by a corridor constraint
    double drag_scale = 1.0;
    double lift_scale = 1.0;
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
    EntryGuidance(const vehicle::FlightBody& ship, const AlphaSchedule& schedule, EntryTarget target,
                  EntryGuidanceParameters parameters = {});

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
    void terminal(const vehicle::FlightBody& body, const math::Mat3& itrf_to_gcrf, const math::Vec3& r_itrf,
                  const math::Vec3& v_itrf, double mach, double dt_s);
    // Horizontal miss at the gate predicted with a terminal steering vector (and the time to go).
    math::Vec3 terminal_miss(const vehicle::FlightBody& body, const math::Vec3& r_itrf, const math::Vec3& v_itrf,
                               const math::Vec3& steering, double* time_to_go_s) const;
    bool violates(const EntryPrediction& p) const;

    EntryTarget target_;
    EntryGuidanceParameters parameters_;
    EntryPredictorModel model_;
    config::HeatShieldConfig limits_;
    double belly_azimuth_rad_ = 0.0;
    EntryPhase phase_ = EntryPhase::Entry;
    bool started_ = false;
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
    bool handed_over_ = false;  // the terminal phase follows the entry phase (not placed by skip_to)
    math::Vec3 terminal_e1_, terminal_e2_;  // steering basis (see EntryPredictorState::basis_frozen)
    math::Vec3 terminal_zem_itrf_;             // predicted horizontal miss at the gate
    double terminal_prediction_age_s_ = 0.0;
};

}  // namespace starsim::gnc
