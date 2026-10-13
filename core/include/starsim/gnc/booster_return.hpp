#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/landing_site.hpp"
#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

enum class ReturnPhase { Flip, Boostback, Coast, EntryBurn, LandingBurn, Landed };

const char* to_string(ReturnPhase phase);

struct BoosterReturnParameters {
    // false (drone ship): no flip and no boostback - the stage coasts on its ascent arc from separation, turning engines
    // first on its RCS, and the entry burn steers the impact point.
    bool boostback = true;
    double flight_time_s = 230.0;  // planned time from boostback start to touchdown (Lambert arc shape)
    double flip_duration_s = 10.0; // [E] separation to boostback ignition (MECO planning; our flip takes ~10 s)
    // [D] MECO planning: boostback delta-v beyond the two-body Lambert arc - the drag-aware aim point lies ~60 m/s
    // further, finite burn and tail-off ~70 m/s more (identified on our full block 3 return: 1616 vs 1488 m/s).
    double boostback_loss_fraction = 0.086;
    std::vector<std::string> boostback_groups{"booster/center", "booster/divert", "booster/middle"};
    // Landing burn: starts with the inner engines (high deceleration close to the ground, small gravity
    // losses) and continues on the centre engines once these can finish it at the design throttle.
    std::vector<std::string> landing_ignition_groups{"booster/center", "booster/divert", "booster/middle"};
    std::vector<std::string> landing_groups{"booster/center"};
    // SCvx landing (V3 baseline 13 -> 5 -> 3): the divert set between the ignition and the centre engines.
    std::vector<std::string> landing_divert_groups{"booster/center", "booster/divert"};
    // Entry burn (Falcon 9; Super Heavy has none): lit while descending below entry_altitude_m, retrograde (relative
    // to the ground), until the burned delta-v reaches entry_delta_v_mps. It limits the descent's dynamic pressure and
    // heating. entry_altitude_m = 0: no entry burn.
    std::vector<std::string> entry_groups;
    double entry_altitude_m = 0.0;
    double entry_delta_v_mps = 0.0;
    double entry_throttle = 1.0;
    double landing_throttle = 0.8;  // design throttle of the inner-engine segment: the rest is control margin
    double final_throttle = 0.8;    // design throttle of the centre-engine segment (slower, time to settle)
    double gate_speed_mps = 50.0;   // sink rate at the hand-over to the centre engines
    double max_tilt_rad = 10.0 * 3.14159265358979 / 180.0;         // thrust from the vertical, centre engines
    double max_tilt_ignition_rad = 25.0 * 3.14159265358979 / 180.0; // with the inner engines (more gimbal authority)
    double flip_tolerance_rad = 15.0 * 3.14159265358979 / 180.0;
    // Landing burn on SCvx plans (catch sites only); the explicit laws above remain the fallback when the first
    // plan fails, and the guidance of sites without a tower.
    bool scvx_landing = true;
    ScvxLandingParameters scvx;
};

// Return parameters of a first stage: `base` with every key of a return profile that is set (engine sets, return flight
// time, entry burn, landing-burn profile; engine group ids get the stage prefix `stage_id`).
BoosterReturnParameters booster_return_parameters(const std::string& stage_id, const config::ReturnProfile& profile,
                                                  BoosterReturnParameters base = {});
// The same with the stage's [stage.recovery] profile (launch / landing site, catch tower).
BoosterReturnParameters booster_return_parameters(const config::StageConfig& stage, BoosterReturnParameters base = {});

struct ReturnCommand {
    ReturnPhase phase = ReturnPhase::Flip;
    std::vector<std::string> run_groups;  // engine groups that should run now
    double throttle = 1.0;
    math::Vec3 axis_gcrf;                 // desired body x-axis
    math::Vec3 rcs_force_gcrf;            // RCS translation, fraction of the maximum force (|f| <= 1)
    double impact_miss_m = 0.0;           // predicted horizontal miss of the landing site (ballistic)
    double time_to_go_s = 0.0;            // landing burn
    math::Vec3 roll_reference_gcrf;       // desired body +y (zero: free roll)
    bool scvx = false;                                          // landing burn flown on SCvx plans
    bool abort = false;                                         // catch given up: arms stay open, landing at sea
    std::optional<AttitudeController::Feedforward> feedforward; // planned rotation (SCvx)
    std::optional<math::Vec3> torque_body_nm;                   // SCvx lateral LQR: body torque (gimbals / RCS)
    std::vector<double> flap_rad;  // flaps commanded directly (ship flip; empty: the attitude controller sets them)
    bool deploy_legs = false;      // landing on legs: deploy them now (sticky)
};

// Return of a first stage to its launch site:
//  Flip      - turn towards the boostback direction with the centre engines at minimum throttle
//  Boostback - burn along v_go of a Lambert arc whose aim point is shifted until the drag-aware
//              prediction of that arc hits the site (shooting on the aim point); cutoff at v_go ~ 0
//  Coast     - engines first (stable for a low CG). In the atmosphere the grid fins hold an angle of
//              attack whose cross-flow force moves the predicted impact point onto the site; above it
//              RCS translation trims the impact point
//  Landing   - ZEM/ZEV (zero-effort miss / zero-effort velocity) guidance in the Earth-fixed frame to a
//              slow touchdown on the site, time-to-go from a constant-deceleration profile. Ignition
//              from a one-dimensional simulation of the burn along the flight path (drag, gravity, mass
//              flow, engine start-up, engine-set switch) at the design throttle.
// The tower catch is modelled as a touchdown on the pad (no tower geometry yet).
class BoosterReturnGuidance {
public:
    BoosterReturnGuidance(LandingSite site, BoosterReturnParameters parameters = {});

    ReturnCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    ReturnPhase phase() const { return phase_; }
    // Pins on the tower arms (reported by the tower each tick): the landing burn ends.
    void set_catch_contact(bool contact) { catch_contact_ = contact; }
    // A foot on the ground (reported by the leg landing each tick): the landing burn ends.
    void set_leg_contact(bool contact) { leg_contact_ = contact; }
    double entry_delta_v_mps() const { return entry_delta_v_mps_; }  // burned in the entry burn so far
    // Scenarios: start in a later phase (e.g. LandingBurn for a booster placed at ignition).
    void skip_to(ReturnPhase phase) {
        phase_ = phase;
        started_ = true;
    }
    void set_realtime(bool realtime);
    const ScvxLandingGuidance* scvx() const { return scvx_.get(); }

    // Retrograde burn still to come on the way down (the entry burn), modelled as an impulse where the fall first
    // crosses `altitude_m` above the ground height.
    struct PendingBurn {
        double altitude_m = 0.0;
        double delta_v_mps = 0.0;
    };
    // Drag-aware ballistic impact prediction (point mass, tail-first axial drag from the vehicle's table,
    // power off). `velocity_gcrf_mps` overrides the body's velocity (e.g. a planned arc).
    struct Impact {
        math::Vec3 position_itrf_m;
        math::Vec3 velocity_itrf_mps;  // relative to the rotating Earth
        double time_s = 0.0;
        bool valid = false;
    };
    static Impact predict_impact(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                 time::Epoch epoch, double ground_height_m, const math::Vec3* velocity_gcrf_mps = nullptr,
                                 const PendingBurn* burn = nullptr);

    // One-dimensional landing-burn simulation along the current flight path: height above the ground
    // where the speed reaches zero (negative: below ground) and the burn duration.
    struct BurnPrediction {
        double stop_height_m = 0.0;
        double duration_s = 0.0;
    };
    BurnPrediction predict_landing_burn(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                        time::Epoch epoch) const;

    // Propellant the booster needs at main-engine cutoff to return with `landing_propellant_kg` left at landing-burn
    // ignition; the ascent autopilot cuts off when the booster is down to it. Without a boostback: the run-down and the
    // entry burn only (the site does not matter). `stack` is the vehicle before
    // separation (its lowest stage is the booster) at `state_gcrf`. Sums the run-down of the engines running now,
    // the flip (flip_duration_s on the landing engines at minimum throttle, two-body coast) and the boostback: the
    // rocket equation (vacuum exhaust velocity) on the delta-v of the Lambert arc from the state after the flip to
    // the site at the end of flight_time_s, as the guidance flies it, plus boostback_loss_fraction (drag shift of the
    // aim point, finite burn). Without a Lambert arc only the landing propellant and the flip are counted.
    static double meco_propellant_kg(const vehicle::Vehicle& stack, const math::StateVector& state_gcrf,
                                     const frames::BodyOrientation& earth, time::Epoch epoch, const LandingSite& site,
                                     const BoosterReturnParameters& parameters, double landing_propellant_kg);

private:
    LandingSite site_;  // target of the return (moved to the sea point when ditching)
    bool ditch_ = false;  // explicit laws only at a catch site: the whole return aims beyond the shoreline
    BoosterReturnParameters parameters_;
    ReturnPhase phase_ = ReturnPhase::Flip;
    time::Epoch boostback_start_;
    bool started_ = false;
    math::Vec3 aim_offset_itrf_;  // horizontal shift of the Lambert aim point (drag compensation)
    bool final_engines_ = false;  // landing burn continues on the centre engines
    int tick_counter_ = 0;
    double impact_miss_m_ = 0.0;
    math::Vec3 trim_itrf_;  // RCS trim force fraction from the latest impact prediction
    math::Vec3 steer_itrf_;  // aerodynamic steering: lateral offset of the axis (tan alpha)
    bool catch_contact_ = false;
    double target_bottom_height_m_ = 0.0;  // vehicle bottom above the site ground at touchdown / catch
    std::unique_ptr<ScvxLandingGuidance> scvx_;  // landing burn on SCvx plans (null: explicit laws)
    bool landing_started_ = false;
    bool leg_contact_ = false;
    bool legs_deployed_ = false;          // deployment commanded
    bool entry_done_ = false;
    bool entry_predicted_ = false;         // the entry burn's own impact prediction has run
    double entry_delta_v_mps_ = 0.0;
    time::Epoch last_update_;              // previous update (entry burn integration)
    std::optional<time::Epoch> plan_end_;  // SCvx pad landing: end of the active plan

    void entry_burn(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch,
                    ReturnCommand& cmd);
    // The rest of the entry burn while it is still ahead or running (nullopt without one / once flown).
    std::optional<PendingBurn> pending_entry_burn() const;
    Impact predict(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch,
                   const math::Vec3* velocity_gcrf_mps = nullptr) const;

    void start_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    void explicit_landing(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch,
                          ReturnCommand& cmd);
};

}  // namespace starsim::gnc
