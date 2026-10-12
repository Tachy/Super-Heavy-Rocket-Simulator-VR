#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/aero/slender_body_aero.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/propulsion/engine_performance.hpp"

namespace starsim::vehicle {

// Body frame: x along the vehicle axis towards the nose, origin at the bottom of the lowest stage
// ("station" = x coordinate). y/z complete a right-handed frame; engine 0 of each ring sits at +y.

enum class EngineMode { Off, Starting, Running, ShuttingDown };

struct Engine {
    std::shared_ptr<const propulsion::EnginePerformance> performance;
    std::string group_id;
    math::Vec3 gimbal_point_m;  // body frame
    double ring_angle_rad = 0.0;
    bool gimbaled = false;
    bool relight = false;
    EngineMode mode = EngineMode::Off;
    double spool = 0.0;      // 0..1 thrust build-up during start / shutdown
    double throttle = 0.0;   // actual throttle (rate limited)
    double gimbal_pitch_rad = 0.0;  // deflection towards +z
    double gimbal_yaw_rad = 0.0;    // deflection towards +y
    bool has_run = false;
    bool starved = false;    // shut down by propellant depletion

    // Unit thrust direction (body frame) for the current gimbal angles.
    math::Vec3 thrust_direction() const;
    bool producing_thrust() const { return mode != EngineMode::Off && spool > 0.0; }
};

struct Tank {
    config::TankConfig config;
    double density_kg_m3 = 0.0;
    double area_m2 = 0.0;
    double mass_kg = 0.0;
};

struct Stage {
    config::StageConfig config;
    double base_station_m = 0.0;  // bottom of this stage in the vehicle body frame
    std::vector<Tank> tanks;
    std::vector<Engine> engines;
    double extra_mass_kg = 0.0;   // payload + fairing

    double propellant_kg(config::TankContents contents) const;
};

struct MassProperties {
    double mass_kg = 0.0;
    double cg_station_m = 0.0;
    math::Vec3 inertia_kg_m2;  // principal moments about the CG: (roll, pitch, yaw) = (Ixx, Iyy, Izz)
};

// Command for one simulation tick. Engine groups are addressed as "<stage>/<group>".
struct GroupCommand {
    std::string group;  // e.g. "booster/center"
    bool run = false;
    double throttle = 1.0;
};

struct VehicleCommand {
    std::vector<GroupCommand> groups;
    // Thrust-vector steering applied to every gimbaled, running engine (rad). Roll is realised by
    // tangential deflection of off-axis engines.
    double gimbal_pitch_rad = 0.0;
    double gimbal_yaw_rad = 0.0;
    double gimbal_roll_rad = 0.0;
};

// Stack of stages with engines, tanks and aerodynamics. Owns no kinematic state (see FlightBody).
class Vehicle {
public:
    explicit Vehicle(const config::VehicleConfig& config, const std::filesystem::path& config_dir);

    const std::string& id() const { return id_; }
    const std::vector<Stage>& stages() const { return stages_; }
    std::vector<Stage>& stages() { return stages_; }
    double length_m() const;
    double diameter_m() const;

    // Mass properties now, or `tau_s` into a tick with constant tank mass flows (RK4 sub-steps).
    MassProperties mass_properties(const std::vector<std::vector<double>>* flows = nullptr, double tau_s = 0.0) const;
    double propellant_kg() const;

    // Discrete engine update for one tick: start/stop transients, throttle and gimbal rate limits,
    // relight rules, flow-separation limits. Must be called once per tick before integration.
    void update_engines(double dt_s, const VehicleCommand& command, double ambient_pressure_pa);

    // Total propellant mass flow per tank (index: stage, tank) for the current engine state.
    std::vector<std::vector<double>> tank_mass_flows_kgps() const;
    // Applies a tick's propellant consumption; starves engines whose tanks ran dry.
    void consume_propellant(double dt_s);

    bool any_engine_producing_thrust() const;
    const aero::SlenderBodyAero& aero() const { return *aero_; }

    // Splits the stack below stage `index`: this vehicle keeps stages [0, index), the returned one
    // stages [index, n). Stations of the upper part are re-based to its own bottom.
    std::unique_ptr<Vehicle> separate(size_t index);

private:
    Vehicle() = default;
    void rebuild_aero();

    std::string id_;
    std::filesystem::path config_dir_;
    std::vector<Stage> stages_;
    std::unique_ptr<aero::SlenderBodyAero> aero_;
};

}  // namespace starsim::vehicle
