#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/gnc/attitude_controller.hpp"
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
};

// All flying bodies of one mission (the stack, and after staging booster and ship) on a common
// fixed tick. The pilot controls one body; separated bodies coast with their engines off.
// Not thread-safe (SPICE): use from the simulation thread only.
class FlightSession {
public:
    static constexpr double tick_s = 0.005;  // 200 Hz
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
    bool active() const { return !bodies_.empty(); }
    time::Epoch epoch() const { return epoch_; }

    // Integrates whole ticks up to `target` (at most `max_ticks`); returns the epoch reached.
    time::Epoch advance_to(time::Epoch target, int max_ticks = 4000);

    void set_input(const PilotInput& input) { input_ = input; }
    void set_control_mode(ControlMode mode) { mode_ = mode; }
    ControlMode control_mode() const { return mode_; }

    // Ascent autopilot: ignition, liftoff at full thrust, booster profile, MECO at the landing reserve,
    // hot staging, closed-loop upper-stage guidance to the target orbit, cutoff. Pilot input is ignored
    // while engaged.
    void engage_ascent_autopilot(const gnc::AscentTarget& target, const gnc::PitchProgram& program = {});
    void disengage_autopilot() { autopilot_.state = AutopilotState::Off; }
    const AutopilotStatus& autopilot() const { return autopilot_; }
    const gnc::AscentGuidance* ascent_guidance() const { return guidance_.get(); }
    double gm_earth_m3ps2() const { return gm_earth_; }

    // Manoeuvre execution on the controlled body (RCS orientation, timed ignition, closed-loop cutoff).
    void execute_burn(const gnc::BurnNode& node);
    bool plan_circularization_at_apoapsis();  // plans and executes; false if not on an elliptic orbit
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
    // Seconds since liftoff (clamp release); before liftoff: since the vehicle was placed on the pad.
    double mission_time_s() const { return epoch_ - launch_epoch_; }

private:
    struct Flying {
        std::unique_ptr<vehicle::FlightBody> body;
        std::map<std::string, bool> running_groups;  // "<stage>/<group>" -> run
    };

    vehicle::VehicleCommand command_for(size_t index, double ambient_pressure_pa) const;
    void run_autopilot();

    std::filesystem::path config_dir_;
    const frames::BodyOrientation& earth_;
    environment::PointMassGravity gravity_;
    environment::Us76Atmosphere atmosphere_;
    vehicle::FlightEnvironment env_;
    gnc::AttitudeController controller_;
    std::vector<Flying> bodies_;
    size_t controlled_ = 0;
    PilotInput input_;
    ControlMode mode_ = ControlMode::RateCommand;
    time::Epoch epoch_;
    time::Epoch launch_epoch_;
    double gm_earth_;
    AutopilotStatus autopilot_;
    std::unique_ptr<gnc::AscentGuidance> guidance_;
    std::optional<gnc::BurnExecutor> burn_;
};

}  // namespace starsim::sim
