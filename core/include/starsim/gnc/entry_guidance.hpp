#pragma once

#include <optional>

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
    // [E] a ship sinking faster than this at the gate hands over higher, gate_height_m ((sink / this)^2 - 1) above the
    // gate (stopping distance ~ v^2; flight 13 relit at ~101 m/s); the steering still aims at the gate point. l5 Task 10:
    // thin air (x0.88) reached the gate at 106 m/s, the flip ended ~130 m lower than nominal and the catch plan ran
    // into an arm with 22 t left; the same gate 100 m higher was caught (0: off).
    double gate_reference_sink_mps = 100.0;
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
    // None above this either (lunar_return(): the supercircular pull-out, where the reversal solutions jump between
    // skips and dives - Task 8 reversed at 9.7 km/s to -52 deg and was lost).
    double max_reversal_speed_mps = 1.0e9;
    double min_reversal_interval_s = 20.0;    // [E] dwell between reversals
    double reversal_max_downrange_error_m = 50e3;  // [E] no reversal check while the range is missed by more
    // > 0: profiled entries limit the bank setpoint's angular acceleration (bank_step_smooth), the predictor too
    double bank_acceleration_radps2 = 0.0;
    // Lateral margin (lunar_return(), l5 Task 10): each held sign aims its track this far beyond the target in its own
    // push direction, shrinking with the speed as ((v - end) / (lateral_margin_speed - end))^3 (a deadband funnel in
    // predictive form): a drift either way stays correctable by a later reversal. 0: aim at the target (LEO ship, capsule). Campaign seed 2
    // (truth lift x0.92): one reversal landed on the target, the held sign then drifted 90 km short in its push
    // direction, where neither holding (bank needed for range) nor reversing helps.
    double lateral_margin_m = 0.0;
    double lateral_margin_speed_mps = 9700.0;
    // The funnel closes at this speed: m(v) = margin ((v - end) / (speed - end))^3 above it, 0 below (0: v^3 to zero).
    double lateral_margin_end_speed_mps = 0.0;
    double scale_filter_gain = 0.3;   // per cycle, measured / modelled aerodynamic acceleration
    double scale_min_acceleration_mps2 = 1.0;
    // Terminal phase: no scale update when the flown attitude was this far off the commanded one at any tick of the
    // cycle (an attitude transient: the flaps' control deflections add aero the trimmed table does not hold - M8a
    // Task 7: the subsonic pitch-up to the belly-flop alpha, 5-9 deg off, measured drag 1.08-1.12 x the table and
    // taught a scale of 1.09 that the steady flight after it (0.996) did not confirm). [E] steady terminal flight stays
    // within ~0.3 deg.
    double scale_max_attitude_error_rad = 2.0 * constants::deg_to_rad;
    bool predictor_j2 = true;         // false: central gravity (test scenarios without J2)
    bool terminal_phase = true;       // false: no terminal steering (a capsule: bank to the gate)
    // > 0: below this speed the bank is held (no more corrector cycles). A capsule falling steeply at subsonic speed has
    // little lift authority left, and its bank from the vertical plane degenerates: the corrector swung to lift down
    // and back (d3 campaign: 0.4 km predicted at 690 m/s, 3.8 km flown). 0: never (the ship's terminal phase takes over).
    double hold_bank_below_speed_mps = 0.0;
    // Lunar l5: the predictor carries skip arcs (EntryPredictorModel::carry_skips) and looks this far ahead.
    bool carry_skips = false;
    // Direct entry: a prediction that skips (a carried arc) counts as an overshoot - the bank keeps the ship in the
    // air. l5 Task 10: a planned skip exiting at ~9.3 km/s (far above circular) reached the gate in the predictor;
    // in air x1.12 / lift x1.04 it rose to a 15,000 km apoapsis and came down 3,600 km short (seeds 4 / 11).
    bool skip_is_overshoot = false;
    double max_time_s = 3000.0;
    // Lunar l5 (user 2026-10-16): the predictor's bank falls linearly in energy to this at the gate
    // (EntryPredictorModel::final_bank_rad); the corrector solves the initial bank. < 0: constant bank.
    double final_bank_rad = -1.0;
    // Warm start of the first bank solution (magnitude). The profiled range is not monotone in the initial bank (low
    // skips between the exits and the captures): bracketing from 0 found a skip root that violated the limits and was
    // cut back to an exit (Task 8). A lunar return starts from the plan's nominal bank.
    double initial_bank_rad = 0.0;
    // > 0: aerocapture - one pass to this exit apoapsis height (no gate, no terminal phase).
    double aerocapture_apoapsis_m = 0.0;
    double target_inclination_rad = -1.0;  // aerocapture: reversals on the exit orbit's inclination (< 0: none)
    double inclination_deadband_rad = 0.2 * constants::deg_to_rad;  // [E]
    double exit_height_m = 130e3;  // aerocapture: phase Exit once above this after the lowest point

    // A capsule (Crew Dragon, d3): gate at the drogue deployment height (~18,000 ft [S: NASA Crew-2]), bank rate
    // 10 deg/s [E] throughout (Dracos; Apollo rolled at up to ~20 deg/s), no terminal phase, reversals to lower speed.
    static EntryGuidanceParameters capsule() {
        EntryGuidanceParameters p;
        p.gate_height_m = 5500.0;
        p.gate_reference_sink_mps = 0.0;  // the drogues open at their height
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
        p.skip_is_overshoot = true;
        p.final_bank_rad = 40.0 * constants::deg_to_rad;  // [E] probe 2026-10-16: the widest initial-bank window
        p.max_reversal_speed_mps = 8500.0;                 // [E] below the pull-out (relative)
        p.lateral_margin_m = 300e3;                        // [E] as EarthReturnTarget::lateral_margin_m at the interface
        // [E] l5 Task 10: the v^3 funnel held the track ~35 km out until ~2.5 km/s; the last reversal (2.2 km/s, a
        // ~75 s roll) did not finish before the terminal phase (seed 18: 10 km across at Mach 3.5, the fall never
        // steep enough for the nose turn). Closed at 3 km/s first; with the bank roll fed forward (2026-10-18) the
        // first reversal (~5 km/s) ends where it is predicted - 8 km beyond the target at a 3 km/s close, and the
        // second reversal came at 1.6-1.9 km/s, unfinished at the hand-over. Closed at 5 km/s (fixtures caught at 5,
        // missed at 3 / 4 / 6 km/s; direct campaign 20/21, every gate within 0.03 km).
        p.lateral_margin_end_speed_mps = 5000.0;
        // [E] the reversal at ~5 km/s and 17-20 kPa starts as a ramp: a step in the rate fed forward asked 4-16 x the
        // RCS in yaw, pinned an aft flap and departed (fixture); 0.1-1 deg/s^2 all caught both fixtures, 0.2 and 0.5
        // the direct campaign 20/21
        p.bank_acceleration_radps2 = 0.5 * constants::deg_to_rad;
        // [E] user 2026-10-17: 25 kPa (b2's estimate, no source) cut the bank back in dense or thin air and the
        // dispersed returns over- or undershot (campaign 3/11 -> 7/11 at 32 kPa); 0.9 x 6 g still bounds the load
        p.max_dynamic_pressure_pa = 32000.0;
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

// [E] below this dynamic pressure the entry's flaps rest in EntryCommand::flap_rest_rad (user 2026-10-18): their moment
// there is a few % of the RCS's and the allocation only wandered them.
inline constexpr double entry_flap_rest_dynamic_pressure_pa = 200.0;

struct EntryCommand {
    EntryPhase phase = EntryPhase::Entry;
    double alpha_rad = 0.0;              // setpoint (entry phase)
    double bank_rad = 0.0;               // setpoint now (rate limited)
    math::Vec3 axis_gcrf;                // for AttitudeController::steer
    math::Vec3 roll_reference_gcrf;
    // Terminal phase: angular velocity of the commanded attitude (feedforward: a PD controller tracking the nose turn's
    // 5 deg/s ramp alone lags ~2 zeta w / wn = ~18 deg, which the belly flop loses as angle of attack)
    math::Vec3 rate_gcrf_radps;
    // Flap rest position (AttitudeController::Feedforward::flap_rest_rad): the pitch trim of the coming entry at the
    // scheduled alpha (no public data on a thermally best rest angle, user 2026-10-18 [E]); empty for a capsule.
    std::vector<double> flap_rest_rad;
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
    // Crew escape: the onboard computer gives the ship up. From now on it aims at `sea` (gnc::sea_give_up_target) and
    // never hands over at a gate: the entry / terminal steering fly the belly flop down to the impact.
    void give_up(const EntryTarget& sea);
    bool given_up() const { return given_up_; }
    // The gate as the latest prediction reaches it (onboard prediction for the mission timeline): the cycle's epoch plus
    // the predicted time to the gate height - the entry corrector's solution or the terminal steering's prediction.
    // None before the first, when the prediction misses the gate, for an aerocapture or a given-up ship, and once there.
    std::optional<time::Epoch> gate_epoch() const {
        if (phase_ == EntryPhase::Gate || given_up_) return std::nullopt;
        return gate_epoch_;
    }

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
    double gate_trigger_height_m(double sink_mps) const;  // hand-over height above the target ground
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
    bool given_up_ = false;
    bool started_ = false;
    bool dipped_ = false;  // aerocapture: has been below 80 km or climbs again (the lowest point is behind)
    time::Epoch last_update_;
    time::Epoch last_cycle_;
    time::Epoch last_scale_update_;
    double scale_window_attitude_error_rad_ = 0.0;  // terminal phase: largest attitude error since the last scale update
    double bank_rad_ = 0.0;          // setpoint now (rate limited)
    int roll_direction_ = 0;         // profiled entry: the direction of a roll under way (bank_step_held)
    double flown_bank_rad_ = 0.0;    // diagnostics: the bank the vehicle flies (STARSIM_DEBUG_ENTRY)
    double bank_target_rad_ = 0.0;   // corrector output with sign
    double bank_magnitude_rad_ = 0.0;
    double sign_ = 0.0;              // 0: not chosen yet
    double time_since_reversal_s_ = 1.0e9;
    bool reversal_rolling_ = false;  // the last range-control reversal still rolls (no new one is checked)
    double pending_reversal_in_s_ = -1.0;  // a reversal timed within the cycle (< 0: none)
    double pending_reversal_bank_rad_ = 0.0;
    double bank_rate_radps_ = 0.0;   // the setpoint's bank rate (bank_step_smooth)
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
    std::optional<time::Epoch> gate_epoch_;  // latest predicted arrival at the gate height
};

}  // namespace starsim::gnc
