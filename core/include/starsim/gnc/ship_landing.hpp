#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/gnc/booster_return.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/gnc/upright_descent.hpp"
#include "starsim/gnc/vehicle_capability.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// The flip's push along the nose and the braking after it [D]:
//  Flip    - the flip set relights (thrust and gimbal authority ramp up linearly over its engines' startup time) and
//            turns the heavy ship rest-to-rest (time-optimal bang-bang at the ramped authority) through 90 deg,
//            belly-first -> engines down; its thrust along the axis at theta(t) from the horizontal gives
//            dv = int a s(t) cos(theta) dt and the travel x = int (t_f - t) a s(t) cos(theta) dt.
//  Tilt-in - the first landing set (planned maximum) tilts rest-to-rest to the phase's tilt limit at its gimbal
//            acceleration, capped by the phase's tilt-rate limit (flip_duration_s); the horizontal part a sin(theta)
//            brakes meanwhile.
//  Brake   - the rest at the full tilt: v^2 / (2 a sin(tilt)).
// lead = flip travel + tilt-in travel + braking distance. The ideal turn ends settled: no settle time (the guidance's
// settled-rate test passes at the turn's end), and the synchronous first plan adds no flight time. A horizontal speed
// v0 along the line at the gate (towards the tower > 0) adds v0 (t_flip + t_tilt-in) of travel and enters the braking
// (the speed after the tilt-in is v0 + dv - dv_tilt-in); the tilt-in travel stops where the speed reaches zero.
struct FlipModel {
    double flip_accel_mps2 = 0.0;       // the flip set at the flip throttle on the heavy ship, fully spooled
    double flip_alpha_radps2 = 0.0;     // its gimbal pitch acceleration, fully spooled
    double spool_s = 0.0;               // the flip engines' startup time (0: full thrust at once)
    double brake_accel_mps2 = 0.0;      // the first landing set at its planned maximum on the heavy ship
    double brake_alpha_radps2 = 0.0;    // its gimbal pitch acceleration there
    double brake_rate_max_radps = 0.0;  // the phase's tilt-rate limit (<= 0: none)
    double brake_tilt_rad = 0.0;        // the phase's tilt limit
    double initial_speed_mps = 0.0;     // v0: horizontal speed along the line at the gate (towards the tower > 0)
};

// Flip and landing of the ship at the tower (b3 of docs/superpowers/archive/specs/2026-10-13-ship-return-design.md).
// Production values come from the vehicle config: ship_landing_parameters(vehicle, stage). The member defaults are
// only test fixtures.
struct ShipLandingParameters {
    std::vector<std::string> flip_groups{"ship/sea_level_1", "ship/sea_level_2", "ship/sea_level_3"};
    double flip_throttle = 0.8;                                  // [K] flip_throttle
    double flip_tolerance_rad = 15.0 * constants::deg_to_rad;    // [K] axis from the vertical: the burn plans start
    double flip_settled_rate_radps = 0.2;                        // [K] ... once the flip's pitch/yaw rate is this low
    // The flip starts aerodynamically: aft flaps folded, forward flaps out - the tail loses its drag and falls (engines
    // down) while the engines spool up; below this tilt the attitude controller takes the flaps back [K flaps_until_deg].
    double flip_flaps_until_tilt_rad = 30.0 * constants::deg_to_rad;
    double first_plan_retry_s = 1.0;                             // a failed first plan is tried again this often
    // [D] the flip pushes the ship along its nose (thrust across the path while it turns) and the landing burn brakes
    // the speed it gained: the entry's gate lies this far back from the catch axis on the line tower -> pad, the nose
    // pointing to the tower. This static value (no horizontal speed at the gate, the heavy ship) is the deorbit
    // planners' aim; in flight the terminal entry guidance re-aims from the live state (ship_gate_lead_m).
    double gate_lead_m = 220.0;
    FlipModel gate_flip;              // the model behind gate_lead_m (initial_speed_mps 0)
    double gate_flip_mass_kg = 0.0;   // the mass its accelerations are for (the heavy landing mass)
    // Ship catch robustness (2026-10-21): the last-resort abort (gnc::upright_descent) after this many failed first
    // plans (neither the catch nor the sea in reach), or when an abort has no converged abort plan after the grace
    // time or leans the hull beyond the guard tilt.
    UprightDescentParameters upright_descent;
    int first_plans_before_descent = 3;                               // [E]
    double abort_plan_grace_s = 2.0;                                  // [E] two re-plan cycles
    // [E] abort plans fly up to ~24 deg (20 deg limit + thrust axis + tracking); seed 18 tumbled from 45 deg on
    double upright_guard_tilt_rad = 35.0 * constants::deg_to_rad;
    // [D] The upright descent runs the flip set (braking towards the landing pair's descent profile) until the pair at
    // its planned maximum (scvx throttle_margin below full) can still stop the sink within the height,
    // sqrt(v_f^2 + 2 (a_pair - g) h) at the live mass; then the pair (sticky) follows its profile to the ground.
    // [K] an abort switches to the descent once the usable propellant is below this times a vertical stop's need
    double descent_propellant_margin = 1.3;
    // The 3 -> 2 window's maximum is set at the gate from its height (ship_switch_sink_window); false: as given.
    bool switch_window_from_gate = false;
    ScvxLandingParameters scvx;                                  // ship preset: ship_landing_parameters()
};

// The ship's flip and landing burn from its config ([stage.recovery] ship keys, [stage.tanks] unusable propellant,
// the engines): engine sets 3 -> 2 (user 2026-10-13, flight 13/14 videos), tilt limits and the K values as configured;
// the tilt-rate limits, the 3 -> 2 sink window and the gate lead derived (see the functions below). Throws
// std::invalid_argument for a stage without the ship landing keys (config::StageConfig::ship_landing).
ShipLandingParameters ship_landing_parameters(const config::VehicleConfig& vehicle, const config::StageConfig& ship);
// The same for a flying ship: its source config and its front stage.
ShipLandingParameters ship_landing_parameters(const vehicle::Vehicle& ship);
// The flip's engine groups (stage-prefixed ids) of a ship stage; throws std::invalid_argument without the ship keys.
std::vector<std::string> ship_flip_groups(const config::StageConfig& ship);

// The masses the derivations use: heavy = the ship with full header tanks and empty main tanks (the landing propellant
// at the gate), light = dry with the header's unusable propellant only (the end of the burn).
struct ShipLandingMasses {
    double heavy_kg = 0.0;
    double light_kg = 0.0;
    double heavy_cg_station_m = 0.0;
    double heavy_pitch_inertia_kg_m2 = 0.0;
};
ShipLandingMasses ship_landing_masses(const config::VehicleConfig& vehicle, const config::StageConfig& ship);

// Tilt-rate limit of a phase [D]: the peak rate of a rest-to-rest turn through the phase's tilt allowance at the set's
// gimbal acceleration at minimum throttle, sqrt(alpha tilt_max), capped by the gimbal slew rate (the smaller).
double ship_phase_rate_limit_radps(const EngineSetThrust& set, double tilt_max_rad, double inertia_kg_m2);

// The 3 -> 2 switch's sink window [D]. Minimum: the outgoing set at minimum throttle exceeds the light ship's weight -
// it must switch while still sinking faster than what the set removes while it shuts down,
// (F_out,min / m_light - g) t_shutdown; a set that cannot exceed the weight needs no minimum (0). Maximum: the incoming
// set at its planned maximum (1 - throttle_margin) still stops the heavy ship within the remaining height,
// sqrt(2 (F_in,max (1 - margin) / m_heavy - g) height); 0 when it cannot hold the weight.
struct SinkWindow {
    double min_sink_mps = 0.0;
    double max_sink_mps = 0.0;
};
SinkWindow ship_switch_sink_window(const EngineSetThrust& outgoing, const EngineSetThrust& incoming, double light_mass_kg,
                                   double heavy_mass_kg, double height_m, double throttle_margin, double shutdown_s,
                                   double gravity_mps2);
// The window's maximum for the ship's own sets (ship_landing_masses) from `height_m`: the CG's height above the catch
// (or the ground) at the gate.
double ship_switch_max_sink_mps(const config::VehicleConfig& vehicle, const config::StageConfig& ship,
                                const ShipLandingParameters& parameters, double height_m);

struct FlipPush {
    double duration_s = 0.0;   // the flip
    double speed_mps = 0.0;    // gained in the flip
    double travel_m = 0.0;     // in the flip
    double tilt_in_s = 0.0;
    double tilt_in_m = 0.0;    // travel while tilting in
    double brake_speed_mps = 0.0;  // left when the full tilt is reached
    double brake_m = 0.0;
    double lead_m() const { return travel_m + tilt_in_m + brake_m; }
};
FlipPush ship_flip_push(const FlipModel& model);

// The ship's catch approach time constant [D] (DescentLimits::approach_time_constant_s): the time the final set's
// engines take to run their throttle range, (throttle_max - throttle_min) / throttle_rate (the slowest engine). Within
// it the thrust can go from the hover-ish minimum to full braking, so the plan's sink near the arms stays below
// height / tau: what is left can still be braked at the rate the engines can follow.
double ship_approach_time_constant_s(const config::VehicleConfig& vehicle, const config::StageConfig& ship,
                                     const std::vector<std::string>& groups);

// Where the ship's entry guidance aims: with a catch tower `gate_lead_m` back from the catch axis on the line
// tower -> pad, the nose to the tower: the flip pushes the ship towards the arms, the final approach comes from the open
// side between them and the side pins lie across them; else the pad centre.
EntryTarget ship_entry_target(const LandingSite& site, const ShipLandingParameters& parameters);
EntryTarget ship_entry_target(const LandingSite& site, double gate_lead_m);
// The gate lead from the live state [D]: the parameters' flip model (gate_flip) with the horizontal speed along the
// line `along_speed_mps` (towards the tower > 0) and the linear accelerations at `mass_kg` (same thrust; the angular
// accelerations stay the heavy ship's).
double ship_gate_lead_m(const ShipLandingParameters& parameters, double along_speed_mps, double mass_kg);
// The terminal entry guidance's re-aim (EntryGuidance::set_terminal_aim): the gate with ship_gate_lead_m for the
// predicted gate velocity (Earth-fixed) along the line to the tower and the live mass; the pad centre without a tower.
EntryTarget ship_gate_aim(const LandingSite& site, const ShipLandingParameters& parameters,
                          const math::Vec3& gate_velocity_itrf_mps, double mass_kg);

// From the gate (belly down, ~100 m/s sink):
//  Flip        - the sea-level engines relight at the flip throttle and the attitude controller turns the axis to the
//                local vertical (engines down; gimbals, flaps and RCS).
//  LandingBurn - once the axis is within the flip tolerance: SCvx plans to the catch on the ship's pins (re-plans,
//                abort to the sea after repeated unreachable catches - ScvxLandingGuidance); a failed first plan is
//                retried every first_plan_retry_s while the flip attitude and thrust are held.
//  Upright descent (either phase, sticky) - the last-resort abort: no plan reaches the catch or the sea, or an abort
//                plan is missing / leans the hull too far: gnc::upright_descent keeps the hull upright and slows it.
//  Landed      - pins on the arms, touchdown or impact: engines off.
// Commands as the booster's return guidance (ReturnCommand), so the session flies both the same way.
class ShipLandingGuidance {
public:
    ShipLandingGuidance(LandingSite site, ShipLandingParameters parameters);

    ReturnCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    ReturnPhase phase() const { return phase_; }
    void set_catch_contact(bool contact) { catch_contact_ = contact; }
    void set_realtime(bool realtime);
    const ScvxLandingGuidance* scvx() const { return scvx_.get(); }
    // Gives the catch up (pilot or tower command): the landing burn plans for the sea. False (refused) without a sea,
    // once down or with the pins on the arms.
    bool request_abort();
    int first_plan_attempts() const { return first_plan_attempts_; }
    // The catch the landing burn's plan flies to (its end); none before the first plan, once landed, while aborting
    // and in the upright descent (onboard prediction for the mission timeline).
    std::optional<time::Epoch> catch_epoch() const {
        if (phase_ != ReturnPhase::LandingBurn || !scvx_ || upright_descent_ || scvx_->aborting()) return std::nullopt;
        return scvx_->plan_end_epoch();
    }
    // The flip: from the gate until the axis is within the flip tolerance; height of the CG above the site then.
    double flip_duration_s() const { return flip_duration_s_; }
    double flip_end_height_m() const { return flip_end_height_m_; }
    // The last-resort abort flies (sticky): upright descent instead of plans.
    bool upright_descent_active() const { return upright_descent_; }
    // ... and has switched to the landing pair.
    bool descent_on_pair() const { return descent_on_pair_; }
    // First tick resting or impacted: axis tilt from the local vertical and sink speed (Earth-fixed).
    bool has_contact() const { return contact_; }
    double contact_tilt_rad() const { return contact_tilt_rad_; }
    double contact_sink_mps() const { return contact_sink_mps_; }

private:
    void print_gate_state(const char* label, const vehicle::FlightBody& body, const frames::FixedToInertial& transform,
                          time::Epoch epoch) const;
    // The upright descent's engines: the flip set while the pair at its planned maximum could not stop the sink within
    // the height, then the pair (sticky once it runs).
    bool descent_pair_reached(const vehicle::Vehicle& vehicle, double sink_mps, double height_m, double pressure_pa) const;
    const std::vector<std::string>& descent_groups(bool pair) const {
        return pair ? parameters_.scvx.snapshot.phase_groups[1] : parameters_.flip_groups;
    }
    void fly_upright_descent(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                             const frames::FixedToInertial& transform, const math::Vec3& site_up, ReturnCommand& cmd);

    LandingSite site_;
    ShipLandingParameters parameters_;
    ReturnPhase phase_ = ReturnPhase::Flip;
    std::unique_ptr<ScvxLandingGuidance> scvx_;
    bool started_ = false;
    time::Epoch next_attempt_;
    time::Epoch flip_start_;
    double flip_duration_s_ = 0.0;
    double flip_end_height_m_ = 0.0;
    int first_plan_attempts_ = 0;
    bool catch_contact_ = false;
    bool abort_requested_ = false;
    bool upright_descent_ = false;
    bool descent_on_pair_ = false;
    std::optional<time::Epoch> abort_since_;
    bool contact_ = false;
    double contact_tilt_rad_ = 0.0;
    double contact_sink_mps_ = 0.0;
};

}  // namespace starsim::gnc
