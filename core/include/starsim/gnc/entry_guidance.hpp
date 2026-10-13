#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

enum class EntryPhase { Entry, BellyFlop, Gate };

const char* to_string(EntryPhase phase);

struct EntryGuidanceParameters {
    double gate_height_m = 1500.0;   // [E] above the target ground: the flip starts here (b3)
    double cycle_s = 2.0;            // re-prediction interval
    double max_bank_rad = 150.0 * constants::deg_to_rad;
    double bank_rate_radps = 1.0 * constants::deg_to_rad;  // faster bank changes depart in alpha (b1)
    double constraint_fraction = 0.9;                      // of the heat-rate and load limits
    double max_dynamic_pressure_pa = 25000.0;              // [E]
    double belly_flop_mach = 0.7;                          // [E] subsonic hand-over
    double max_tilt_rad = 15.0 * constants::deg_to_rad;    // [E] belly-flop steering
    double zem_gain = 3.0;
    // [E] the commanded attitude turns at most this fast in the belly flop (from the entry attitude at hand-over)
    double belly_flop_slew_rate_radps = 5.0 * constants::deg_to_rad;
    // [E] the nose turns onto the wanted acceleration only when it exceeds this (no heading jitter near the gate)
    double belly_flop_turn_min_acceleration_mps2 = 0.3;
    // Bank reversal when the predicted crossrange (the current bank sign held to the gate) overshoots to the side the
    // bank pushes by more than base + fraction * range to go [E]; none below min_reversal_speed (belly flop's job).
    double reversal_crossrange_base_m = 2000.0;
    double reversal_crossrange_fraction = 0.01;
    double min_reversal_speed_mps = 600.0;
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
//  BellyFlop - below the hand-over Mach number: belly down, tilt steering to the gate point.
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
    // search starting at `warm_start_rad`; then the corridor constraints.
    Solution solve(const EntryPredictorModel& model, const EntryPredictorState& state, double heat_load_j_m2,
                   double sign, double warm_start_rad) const;
    const EntryPredictorModel& model() const { return model_; }

private:
    void entry_cycle(const vehicle::FlightBody& body, const math::Vec3& r_itrf, const math::Vec3& v_itrf);
    void belly_flop(const vehicle::FlightBody& body, const math::Mat3& itrf_to_gcrf, const math::Vec3& r_itrf,
                    const math::Vec3& v_itrf, double height_m, double dt_s);
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
    double bank_rad_ = 0.0;          // setpoint now (rate limited)
    double bank_target_rad_ = 0.0;   // corrector output with sign
    double bank_magnitude_rad_ = 0.0;
    double sign_ = 0.0;              // 0: not chosen yet
    EntryCommand command_;
    bool flop_started_ = false;
    math::Quat flop_attitude_gcrf_;  // commanded attitude in the belly flop (slew limited)
    math::Vec3 nose_itrf_;           // nose direction chosen at hand-over (horizontal)
};

}  // namespace starsim::gnc
