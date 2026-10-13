#pragma once

#include <filesystem>
#include "starsim/config/launch_site_config.hpp"
#include "starsim/sim/catch_tower_sim.hpp"
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

enum class AutopilotState { Off, PreLaunch, Booster, UpperStage, Orbit, Burn };

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
};

LaunchSite launch_site_from_config(const config::LaunchSiteConfig& site);

enum class LandingOutcome;

// All flying bodies of one mission (the stack, and after staging booster and ship) on a common
// fixed tick. The pilot controls one body; separated bodies coast with their engines off.
// Not thread-safe (SPICE): use from the simulation thread only.
class FlightSession {
public:
    static constexpr double tick_s = 0.005;  // 200 Hz
    static constexpr double max_coast_s = 600.0;
    static constexpr double burn_preparation_s = 120.0;  // ticks resume this long before an ignition
    static constexpr double max_pitch_yaw_rate_radps = 5.0 * 3.14159265358979 / 180.0;
    static constexpr double max_roll_rate_radps = 10.0 * 3.14159265358979 / 180.0;

    // `gravity_config`: fidelity flags of the gravity model (default: Earth + J2, Moon, Sun).
    FlightSession(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& earth,
                  std::filesystem::path config_dir, environment::PointMassGravityConfig gravity_config = {});

    // Places a vehicle on a pad (engines off, clamped). `heading_rad`: direction of body +y.
    void launch_from_pad(const std::string& vehicle_id, const LaunchSite& site, double heading_rad,
                         time::Epoch epoch);
    // Places a vehicle (stages from `first_stage` upwards) on a given orbit, nose along the velocity.
    void place_in_orbit(const std::string& vehicle_id, const math::StateVector& state_gcrf, time::Epoch epoch,
                        size_t first_stage = 0);
    // Fidelity flag: catch towers of launch sites (default on). Takes effect at the next set_launch_site().
    void set_towers(bool enabled) { towers_enabled_ = enabled; }
    // Site of the mission (tower, landing target). launch_from_pad() sets it too.
    void set_launch_site(const LaunchSite& site);
    const LaunchSite& launch_site() const { return launch_site_; }
    // Places a vehicle with a free state (scenarios, tests); the body becomes the controlled one.
    void place_body(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch);
    // Landing site of a returning `booster`: catch target when the site has a tower and the booster has pins,
    // coastline when configured.
    gnc::LandingSite return_site(const vehicle::Vehicle& booster) const;
    // The lowest stage of `vehicle` gets return guidance after separation (booster return on, caught by a tower at
    // this site).
    bool returns_after_staging(const vehicle::Vehicle& vehicle) const;
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
    // Interactive real-time play: landing re-plans never hold up the simulation (default: off = deterministic).
    void set_realtime_planning(bool realtime);
    const gnc::BoosterReturnGuidance* return_guidance(size_t index) const { return bodies_.at(index).return_guidance.get(); }
    // Return guidance of a body (nullptr if it has none) and its latest command.
    const gnc::ReturnCommand* return_status(size_t index) const;
    const AutopilotStatus& autopilot() const { return autopilot_; }
    const gnc::AscentGuidance* ascent_guidance() const { return guidance_.get(); }
    double gm_earth_m3ps2() const { return gm_earth_; }

    // Manoeuvre execution on the controlled body (RCS orientation, timed ignition, closed-loop cutoff).
    // Rejects burns smaller than the main engines' start + shutdown impulse (they would overshoot);
    // fine corrections need RCS translation (M9).
    bool execute_burn(const gnc::BurnNode& node);
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
    };
    const BurnConfiguration& burn_configuration() const { return burn_config_; }
    const gnc::BurnExecutor* burn() const { return burn_ ? &*burn_ : nullptr; }

    // Coasting prediction of a body (no thrust): GCRF positions every `step_s`, until impact or `duration_s`.
    std::vector<math::Vec3> predict_trajectory(size_t index, double duration_s, double step_s) const;

    // Failure injection for tests and training.
    void fail_engine(size_t body_index, size_t stage_index, size_t engine_index);

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
    };

    vehicle::VehicleCommand command_for(size_t index, double ambient_pressure_pa) const;
    void run_autopilot();
    void run_return_guidance(Flying& flying);
    double coast_allowance_s() const;  // 0 if fixed ticks are needed now
    std::vector<vehicle::FlightBody*> body_pointers() const;

    std::filesystem::path config_dir_;
    const frames::BodyOrientation& earth_;
    environment::PointMassGravity gravity_;
    environment::Us76Atmosphere atmosphere_;
    vehicle::FlightEnvironment env_;
    gnc::AttitudeController controller_;
    // Returning booster: faster attitude loop for the landing-burn divert (gimbal limits unchanged).
    gnc::AttitudeController return_controller_{gnc::AttitudeController::Gains{1.5, 0.9, true}};
    std::vector<Flying> bodies_;
    size_t controlled_ = 0;
    PilotInput input_;
    ControlMode mode_ = ControlMode::RateCommand;
    time::Epoch epoch_;
    time::Epoch launch_epoch_;
    LaunchSite launch_site_;
    bool towers_enabled_ = true;
    bool realtime_planning_ = false;
    std::unique_ptr<CatchTowerSim> tower_;
    std::optional<gnc::BoosterReturnParameters> booster_return_;
    double gm_earth_;
    AutopilotStatus autopilot_;
    std::unique_ptr<gnc::AscentGuidance> guidance_;
    std::optional<gnc::BurnExecutor> burn_;
    BurnConfiguration burn_config_;
    math::Vec3 last_rcs_acceleration_;  // RCS translation of the controlled body in the last tick (GCRF)

    std::vector<BurnConfiguration> burn_configurations() const;
};

}  // namespace starsim::sim
