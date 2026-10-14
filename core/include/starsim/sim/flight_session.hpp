#pragma once

#include <filesystem>
#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/sim/catch_tower_sim.hpp"
#include "starsim/sim/leg_landing_sim.hpp"
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

// Normalised pilot input (HOTAS / keyboard): stick -1..1, throttle 0..1.
// pitch > 0: nose towards the launch heading (stick forward); yaw > 0: nose to the right of it.
struct PilotInput {
    double pitch = 0.0;
    double yaw = 0.0;
    double roll = 0.0;
    double throttle = 1.0;
};

enum class ControlMode {
    RateCommand,  // stick commands body rates; the controller drives the gimbals (stability augmentation)
    DirectGimbal  // stick commands gimbal angles directly
};

// Return: a ship after its deorbit burn (coast to the entry interface, entry, flip, catch).
enum class AutopilotState { Off, PreLaunch, Booster, UpperStage, Orbit, Burn, Return };

const char* to_string(AutopilotState state);

struct AutopilotStatus {
    AutopilotState state = AutopilotState::Off;
    gnc::GuidanceCommand guidance;
    gnc::BurnExecutor::Status burn;  // valid in state Burn
};

struct LaunchSite {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    std::optional<config::TowerConfig> tower;  // catch tower (stacks then stand on its launch mount)
    std::optional<config::SeaConfig> sea;      // coastline (aborted landings)
    double pad_radius_m = 0.0;                 // landing zone: concrete pad radius
    std::optional<contact::LandingSurface::Deck> deck;  // a drone ship: its deck (height_m: the deck), the sea around it
};

LaunchSite launch_site_from_config(const config::LaunchSiteConfig& site);
// A drone ship holding station at (latitude, longitude) as a landing zone: its deck at sea height + freeboard, the long
// axis along `heading_rad` (clockwise from north). `sea` marks the water beside the deck's side edge: the landing
// guidance's abort aims there.
LaunchSite drone_ship_site(const config::DroneShipConfig& ship, double latitude_rad, double longitude_rad,
                           double sea_height_m, double heading_rad);

enum class LandingOutcome;

// All flying bodies of one mission (the stack, and after staging booster and ship) on a common
// fixed tick. The pilot controls one body; separated bodies coast with their engines off.
// Not thread-safe (SPICE): use from the simulation thread only.
class FlightSession {
public:
    static constexpr double tick_s = 0.005;  // 200 Hz
    static constexpr double max_coast_s = 600.0;
    static constexpr double burn_preparation_s = 120.0;  // ticks resume this long before an ignition
    // [E] a returning ship gets its entry guidance this long before the entry interface (turn to the entry attitude
    // on RCS: ~1.5 min for the 125 deg from the burn attitude)
    static constexpr double entry_lead_s = 300.0;
    static constexpr double max_pitch_yaw_rate_radps = 5.0 * 3.14159265358979 / 180.0;
    static constexpr double max_roll_rate_radps = 10.0 * 3.14159265358979 / 180.0;

    // `gravity_config`: fidelity flags of the gravity model (default: Earth + J2, Moon, Sun).
    FlightSession(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& earth,
                  std::filesystem::path config_dir, environment::PointMassGravityConfig gravity_config = {});

    // Places a vehicle on a pad (engines off, clamped). `heading_rad`: direction of body +y.
    void launch_from_pad(const std::string& vehicle_id, const LaunchSite& site, double heading_rad,
                         time::Epoch epoch);
    // The same with a loaded (possibly modified, e.g. the mission's payload) vehicle configuration.
    void launch_from_pad(const config::VehicleConfig& config, const LaunchSite& site, double heading_rad,
                         time::Epoch epoch);
    // Places a vehicle (stages from `first_stage` upwards) on a given orbit, nose along the velocity.
    void place_in_orbit(const std::string& vehicle_id, const math::StateVector& state_gcrf, time::Epoch epoch,
                        size_t first_stage = 0);
    // Fidelity flag: catch towers of launch sites (default on). Takes effect at the next set_launch_site().
    void set_towers(bool enabled) { towers_enabled_ = enabled; }
    // Site of the mission (tower, landing target). launch_from_pad() sets it too.
    void set_launch_site(const LaunchSite& site);
    const LaunchSite& launch_site() const { return launch_site_; }
    // Landing zone of the mission: first stages that land on legs ("leg_landing") return to it, and the legs land on
    // its ground plane (with its shoreline). Set after the launch site (set_launch_site resets the leg landing).
    void set_landing_zone(const LaunchSite& zone);
    const std::optional<LaunchSite>& landing_zone() const { return landing_zone_; }
    // Places a vehicle with a free state (scenarios, tests); the body becomes the controlled one.
    void place_body(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch);
    // Landing site of a returning `booster`: catch target when the site has a tower and the booster has pins; a stage
    // with legs ("leg_landing") lands on the landing zone's pad; coastline when configured.
    gnc::LandingSite return_site(const vehicle::Vehicle& booster) const;
    // The lowest stage of `vehicle` gets return guidance after separation (booster return on, and either caught by a
    // tower at this site or landing on its legs on the landing zone).
    bool returns_after_staging(const vehicle::Vehicle& vehicle) const;
    // Return parameters of a first stage: the guidance's (set_booster_return) with the stage's return profile for this
    // mission's landing target (drone ship or not).
    gnc::BoosterReturnParameters return_parameters(const config::StageConfig& stage) const;
    const vehicle::FlightEnvironment& environment() const { return env_; }
    const CatchTowerSim* tower() const { return tower_.get(); }
    CatchTowerSim* tower() { return tower_.get(); }
    bool active() const { return !bodies_.empty(); }
    time::Epoch epoch() const { return epoch_; }

    // Integrates up to `target` (at most `max_ticks` steps); returns the epoch reached. Fixed 200 Hz ticks
    // while anything is commanded, adaptive coast segments (<= max_coast_s) otherwise.
    time::Epoch advance_to(time::Epoch target, int max_ticks = 4000);

    void set_input(const PilotInput& input) { input_ = input; }
    void set_control_mode(ControlMode mode) { mode_ = mode; }
    ControlMode control_mode() const { return mode_; }

    // Ascent autopilot: ignition, liftoff at full thrust, booster profile, MECO at the landing reserve,
    // hot staging, closed-loop upper-stage guidance to the target orbit, cutoff. Pilot input is ignored
    // while engaged.
    void engage_ascent_autopilot(const gnc::AscentTarget& target, const gnc::PitchProgram& program = {});
    void disengage_autopilot() { autopilot_.state = AutopilotState::Off; }
    // Return of the booster to the launch pad after staging (flip, boostback, landing burn). Takes effect
    // at the next staging; the booster is then guided independently of the controlled body.
    void set_booster_return(bool enabled, gnc::BoosterReturnParameters parameters = {});
    // Scenarios: a returning booster placed with a free state, guided by its return guidance from `start_phase`
    // (it becomes the controlled body; pilot input is ignored while it has return guidance).
    void place_returning_booster(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                 time::Epoch epoch, gnc::BoosterReturnParameters parameters, gnc::ReturnPhase start_phase);
    // Scenarios: a ship (single stage with flaps and a heat shield) placed with a free state flies its entry guidance
    // to the gate above the launch site's pad (b3 adds the flip and the catch). It becomes the controlled body; pilot
    // input is ignored while it has entry guidance.
    void place_entering_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch,
                             gnc::EntryGuidanceParameters parameters = {});
    const gnc::EntryGuidance* entry_guidance(size_t index) const { return bodies_.at(index).entry_guidance.get(); }
    // Scenarios: a ship placed at the gate (belly down) flips and lands on its landing guidance (catch at this site's
    // tower). An entering ship gets the same guidance when its entry guidance reaches the gate.
    void place_landing_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch,
                            gnc::ShipLandingParameters parameters = gnc::ship_landing_parameters());
    const gnc::ShipLandingGuidance* ship_landing(size_t index) const { return bodies_.at(index).ship_landing.get(); }
    // Return of the controlled ship from orbit to this site's tower (b4): the onboard computer plans the deorbit burn
    // (gnc::plan_deorbit) and flies it on the vehicle's deorbit engine set; the ship then coasts, gets its entry guidance
    // `entry_lead_s` before the entry interface and is caught by the flip and landing guidance. False if the plan is
    // refused (reason in ship_return_plan()) or the burn cannot be flown.
    bool plan_ship_return(const gnc::DeorbitParameters& parameters = {});
    const gnc::DeorbitPlan* ship_return_plan() const { return deorbit_plan_ ? &*deorbit_plan_ : nullptr; }
    // Entry guidance of a body (nullptr if it has none): its latest command (setpoint for the flight director).
    const gnc::EntryCommand* entry_status(size_t index) const;
    // Interactive real-time play: landing re-plans never hold up the simulation (default: off = deterministic).
    void set_realtime_planning(bool realtime);
    const gnc::BoosterReturnGuidance* return_guidance(size_t index) const { return bodies_.at(index).return_guidance.get(); }
    // Return guidance of a body (nullptr if it has none) and its latest command.
    const gnc::ReturnCommand* return_status(size_t index) const;
    const AutopilotStatus& autopilot() const { return autopilot_; }
    const gnc::AscentGuidance* ascent_guidance() const { return guidance_.get(); }
    double gm_earth_m3ps2() const { return gm_earth_; }

    // Manoeuvre execution on the controlled body (RCS orientation, timed ignition, closed-loop cutoff).
    // Burns smaller than the main engines' start + shutdown impulse (they would overshoot) fly on RCS translation
    // along +x (the thrusters' force and mass flow), when the vehicle has thrusters along +x; else they are rejected.
    bool execute_burn(const gnc::BurnNode& node);
    // On a given engine set (config group ids of the lowest stage with engines, e.g. the deorbit set): full throttle if
    // its start/shutdown impulse is at most a quarter of the burn, else minimum throttle; false if neither fits. The hull
    // is steered so that the set's thrust axis through the CG (gnc::thrust_axis_body) points along the burn.
    bool execute_burn(const gnc::BurnNode& node, const std::vector<std::string>& groups);
    // Plans and executes; false if not on an elliptic orbit or the burn is too small.
    bool plan_circularization_at_apoapsis();
    // Smallest delta-v the main engines of the controlled body can deliver (start + shutdown transients
    // of the weakest configuration: one engine group at minimum throttle).
    double minimum_burn_dv_mps() const;

    // Engine configuration used for a burn.
    struct BurnConfiguration {
        std::vector<std::string> groups;
        double throttle = 1.0;
        double thrust_n = 0.0;        // vacuum
        double mass_flow_kgps = 0.0;
        double shutdown_s = 0.0;
        double minimum_dv_mps = 0.0;  // impulse of the start + shutdown transients
        bool rcs = false;             // RCS translation (no main engines)
        math::Vec3 thrust_axis_body{1.0, 0.0, 0.0};  // through the CG at the start (asymmetric sets: tilted)
    };
    const BurnConfiguration& burn_configuration() const { return burn_config_; }
    const gnc::BurnExecutor* burn() const { return burn_ ? &*burn_ : nullptr; }

    // Coasting prediction of a body (no thrust): GCRF positions every `step_s`, until impact or `duration_s`.
    std::vector<math::Vec3> predict_trajectory(size_t index, double duration_s, double step_s) const;

    // Failure injection for tests and training.
    void fail_engine(size_t body_index, size_t stage_index, size_t engine_index);
    // Gives a returning body's landing or catch up (pilot's abort): it ditches at sea and the tower keeps its arms open.
    // Before the landing burn the abort is remembered and the burn plans for the sea. False when the guidance cannot
    // abort (not returning, no sea at the site, already down or on the arms): then nothing changes.
    bool abort_landing(size_t body_index);
    // Crew Dragon's nose cone (thrusters behind it fire only while it is open); no effect without one.
    void set_nose_cone(size_t body_index, bool open);
    // Landing legs of a body: start the deployment (guidance, pilot), or place them in a state (scenarios).
    void deploy_legs(size_t body_index);
    void set_leg_deployment(size_t body_index, double fraction);
    // Landing on legs: the surface the legs land on (default: the site's ground plane with its shoreline; a drone
    // ship's deck replaces it), and how a body's touchdown ended.
    void set_landing_surface(double latitude_rad, double longitude_rad, double height_m, contact::LandingSurface surface);
    const LegLandingSim& leg_landing() const { return *legs_; }
    LegTouchdown leg_touchdown(size_t body_index) const { return legs_->touchdown(*bodies_.at(body_index).body); }
    std::vector<double> leg_crushed_m(size_t body_index) const { return legs_->crushed_m(*bodies_.at(body_index).body); }
    // Dispersion injection (tests, training): an instantaneous velocity change of a body, e.g. the residual of a burn
    // (tail-off, misalignment) beyond what the closed-loop cutoff removes.
    void apply_velocity_error(size_t body_index, const math::Vec3& delta_v_gcrf_mps);
    // Truth-model error injection (tests, training): the hull's aerodynamic force of a body scaled by `factor`; the
    // onboard models (impact prediction, landing planner) keep the nominal tables.
    void set_aero_dispersion(size_t body_index, double factor);

    // Pilot actions on the controlled body.
    void ignite();             // start all engine groups of the lowest stage that still has engines
    void cutoff();             // stop all engines
    void release_clamps();
    void stage();              // separate the lowest stage; the upper part becomes the controlled body

    size_t body_count() const { return bodies_.size(); }
    size_t controlled_index() const { return controlled_; }
    const vehicle::FlightBody& body(size_t index) const { return *bodies_.at(index).body; }
    vehicle::FlightTelemetry telemetry(size_t index) const;
    // How the flight of body `index` ended so far (Flying while it is in the air).
    LandingOutcome landing_outcome(size_t index) const;
    // Seconds since liftoff (clamp release); before liftoff: since the vehicle was placed on the pad.
    double mission_time_s() const { return epoch_ - launch_epoch_; }

private:
    struct Flying {
        std::unique_ptr<vehicle::FlightBody> body;
        std::map<std::string, bool> running_groups;  // "<stage>/<group>" -> run
        std::unique_ptr<gnc::BoosterReturnGuidance> return_guidance;  // autonomous return (separated booster)
        gnc::ReturnCommand return_command;
        std::unique_ptr<gnc::EntryGuidance> entry_guidance;  // entering ship (scenario)
        gnc::EntryCommand entry_command;
        std::unique_ptr<gnc::ShipLandingGuidance> ship_landing;  // ship from the gate: flip and catch (its command:
                                                                 // return_command)
    };

    vehicle::VehicleCommand command_for(size_t index, double ambient_pressure_pa) const;
    void run_autopilot();
    void run_return_guidance(Flying& flying);
    void run_ship_landing(Flying& flying);
    void apply_return_command(const Flying& flying, const gnc::AttitudeController& controller,
                              double ambient_pressure_pa, vehicle::VehicleCommand& command) const;
    double coast_allowance_s() const;  // 0 if fixed ticks are needed now
    void start_entry_guidance(Flying& flying, const gnc::EntryGuidanceParameters& parameters);
    time::Epoch entry_start_epoch() const;  // returning ship: entry interface (coasting from now) - entry_lead_s
    std::vector<vehicle::FlightBody*> body_pointers() const;

    std::filesystem::path config_dir_;
    const frames::BodyOrientation& earth_;
    environment::PointMassGravity gravity_;
    environment::Us76Atmosphere atmosphere_;
    vehicle::FlightEnvironment env_;
    gnc::AttitudeController controller_;
    // Returning booster: faster attitude loop for the landing-burn divert (gimbal limits unchanged).
    gnc::AttitudeController return_controller_{gnc::AttitudeController::Gains{1.5, 0.9, true}};
    // Its coast phase (turning engines first on the RCS, d1): slews limited to 2 deg/s [E] - the RCS supply is finite
    // (Falcon 9 nitrogen); a full-authority turn spent it (and the momentum the air adds at 70 km) before the entry burn.
    gnc::AttitudeController return_coast_controller_{
        gnc::AttitudeController::Gains{1.5, 0.9, true, false, 0.0, 2.0 * constants::deg_to_rad}};
    // Entering ship: flaps + RCS, slow (the flaps move at ~20 deg/s: 0.8 rad/s limit-cycles in roll, b1). [E] actuator
    // delay 0.5 s (the flaps' slew): without it the belly flop's nose turn (5 deg/s) set off a roll limit cycle and the
    // ship departed (b4 campaign).
    gnc::AttitudeController entry_controller_{gnc::AttitudeController::Gains{0.5, 0.9, false, true, 0.5}};
    // Ship flip and landing burn: gimbals with flaps and RCS, faster than in the entry [E].
    // [E] actuator delay: the gimbals slew 30 deg (stop to stop) at 20 deg/s, half of that on average
    gnc::AttitudeController ship_landing_controller_{gnc::AttitudeController::Gains{1.5, 0.9, false, true, 0.75}};
    std::vector<Flying> bodies_;
    size_t controlled_ = 0;
    PilotInput input_;
    ControlMode mode_ = ControlMode::RateCommand;
    time::Epoch epoch_;
    time::Epoch launch_epoch_;
    LaunchSite launch_site_;
    std::optional<LaunchSite> landing_zone_;
    bool towers_enabled_ = true;
    bool realtime_planning_ = false;
    std::unique_ptr<CatchTowerSim> tower_;
    std::unique_ptr<LegLandingSim> legs_;  // always present (site ground plane by default)
    void reset_leg_landing();              // new bodies: no stale tracks
    void reset_mission_state();            // new flight: autopilot off, no ascent guidance, burn or ship return
    std::optional<gnc::BoosterReturnParameters> booster_return_;
    double gm_earth_;
    AutopilotStatus autopilot_;
    std::unique_ptr<gnc::AscentGuidance> guidance_;
    std::optional<gnc::BurnExecutor> burn_;
    BurnConfiguration burn_config_;
    math::Vec3 last_rcs_acceleration_;  // RCS translation of the controlled body in the last tick (GCRF)
    std::optional<gnc::DeorbitPlan> deorbit_plan_;  // last ship return plan (also when refused)
    bool ship_return_ = false;                      // the burn is the deorbit burn of a ship return
    gnc::DeorbitParameters deorbit_parameters_;
    time::Epoch entry_start_;                       // state Return: entry guidance from here

    // `forced`: only this set (config group ids), at full and at minimum throttle.
    std::vector<BurnConfiguration> burn_configurations(const std::vector<std::string>* forced = nullptr) const;
    bool start_burn(const gnc::BurnNode& node, const std::vector<BurnConfiguration>& configurations);
};

}  // namespace starsim::sim
