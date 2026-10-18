#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/aero/capsule_aero.hpp"
#include "starsim/aero/flap.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/aero/grid_fin.hpp"
#include "starsim/aero/slender_body_aero.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/propulsion/engine_performance.hpp"
#include "starsim/propulsion/rcs_allocator.hpp"
#include "starsim/propulsion/rcs_modulator.hpp"
#include "starsim/vehicle/liquid_settling.hpp"

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
    bool abort_only = false; // launch escape (data only): never lit by the session
    math::Vec3 axis_body{1.0, 0.0, 0.0};  // thrust axis without gimbal (canted engines: towards the hull axis)
    bool landing_only = false;           // landing thrusters: flown by landing guidance only
    // Launch escape or landing thrusters: never chosen for burns, never lit by ignite().
    bool auxiliary() const { return abort_only || landing_only; }

    // Unit thrust direction (body frame) for the current gimbal angles.
    math::Vec3 thrust_direction() const;
    bool producing_thrust() const { return mode != EngineMode::Off && spool > 0.0; }
    // Fixed and canted (the HLS landing thrusters): steered by differential throttling while running.
    bool throttle_steered() const { return !gimbaled && axis_body.x < 1.0; }
};

// Torque about the CG per unit of throttle (vacuum thrust at full throttle along the engine's fixed axis): what
// differential throttling of a fixed engine buys. Canted thrust lines miss the CG, so a ring of them only cancels by
// symmetry - each one alone turns the vehicle.
math::Vec3 throttle_torque_nm(const Engine& engine, double cg_station_m);

struct GridFin {
    aero::GridFinGeometry geometry;  // station in the vehicle body frame
    double max_deflection_rad = 0.0;
    double rate_rad_s = 0.0;
    double deflection_rad = 0.0;     // actual (rate limited)
};

// Reaction-control thruster: fixed thrust, on/off (constant feed pressure; user 2026-10-14).
struct RcsThruster {
    std::string id;
    math::Vec3 position_m;   // vehicle body frame
    math::Vec3 direction;    // unit; force on the vehicle
    double thrust_n = 0.0;
    double isp_s = 0.0;
    int tank = -1;           // index of the stage's Rcs tank; -1: ullage gas from the main tanks
    double output = 0.0;     // this tick: 1 firing, 0 off
    bool starved = false;    // its supply is empty
    bool behind_nose_cone = false;
    bool blocked = false;    // cannot fire now: behind a closed nose cone, or a payload stage still stacked
};

struct Flap {
    aero::FlapGeometry geometry;     // stations in the vehicle body frame
    double max_deflection_rad = 0.0;
    double rate_rad_s = 0.0;
    double deflection_rad = 0.0;     // actual (rate limited), 0 = folded leeward
};

enum class ParachuteState { Stowed, Open, Released, Torn };  // Open: inflating, reefed or full
const char* to_string(ParachuteState state);  // German (UI): "Fallschirm gerissen" for a torn set

// A parachute set on a stage (d4): `drag_area_m2` covers all its canopies.
struct Parachute {
    config::ParachuteConfig config;
    double attach_station_m = 0.0;   // vehicle body frame
    ParachuteState state = ParachuteState::Stowed;
    double deployed_s = 0.0;         // since line stretch
    double drag_area_m2 = 0.0;       // now, all canopies
    double peak_load_n = 0.0;        // per canopy, since deployment
};

struct Tank {
    config::TankConfig config;
    double density_kg_m3 = 0.0;
    double area_m2 = 0.0;
    double mass_kg = 0.0;
    double radius_m = 0.0;
    double length_m = 0.0;  // of the full load (capacity) in this cross-section; 0: no settling (a column)
    mutable std::shared_ptr<const SettlingTable> settling;  // fetched on the first tilted query
};

struct Stage {
    config::StageConfig config;
    double base_station_m = 0.0;  // bottom of this stage in the vehicle body frame
    std::vector<Tank> tanks;
    std::vector<Engine> engines;
    std::vector<GridFin> grid_fins;
    std::shared_ptr<const aero::GridFinTable> grid_fin_table;
    std::vector<Flap> flaps;
    std::shared_ptr<const aero::FlapTable> flap_table;
    std::vector<Parachute> parachutes;
    std::vector<RcsThruster> rcs;
    double extra_mass_kg = 0.0;   // payload + fairing

    double propellant_kg(config::TankContents contents) const;
};

struct MassProperties {
    double mass_kg = 0.0;
    double cg_station_m = 0.0;
    math::Vec3 cg_offset_m;    // lateral CG offset from the axis (0, y, z): a capsule's trim offset, else zero
    math::Vec3 inertia_kg_m2;  // principal moments about the CG: (roll, pitch, yaw) = (Ixx, Iyy, Izz)
    math::Vec3 cg_point_m() const { return {cg_station_m, cg_offset_m.y, cg_offset_m.z}; }
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
    // Differential throttling: added to the group throttle of each engine (stage order over all stages, then clamped to
    // the engine's range); empty: none.
    std::vector<double> engine_throttle_trim;
    // Reaction control: requested torque as a fraction (-1..1) of the capability per body axis (max_rcs_torque_nm).
    math::Vec3 rcs_torque_fraction;
    // Reaction control translation: requested force direction and magnitude as a fraction (|f| <= 1) of the
    // capability along that direction (rcs_force_capability), body axes.
    math::Vec3 rcs_force_fraction;
    // Keep the RCS net force what was asked for where the layout allows (pure couples): turns do not translate the
    // vehicle, a saturated torque is cut instead. Orbit manoeuvres set it (an unrequested push moves the orbit);
    // otherwise torque comes first.
    bool rcs_keep_force = false;
    // Grid fin deflections (rad), one per fin in stage order; missing entries mean neutral.
    std::vector<double> grid_fin_rad;
    // Flap deflections (rad), one per flap in stage order; missing entries hold the current deflection.
    std::vector<double> flap_rad;
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
    // Main propellant (oxidizer + fuel: the gauge); reaction-control supplies are rcs_propellant_kg().
    double propellant_kg() const;
    double rcs_propellant_kg() const;

    // Propellant settling (fidelity flag FlightEnvironment::propellant_settling): the liquid's "down" direction in
    // body axes - minus the specific force, lagged - shared by all oxidizer and fuel tanks; initially -x (settled aft
    // by thrust or on the pad). Below a threshold (coasts) the liquid holds its position.
    void settle_propellant(const math::Vec3& specific_force_body_mps2, double dt_s);
    const math::Vec3& liquid_down_body() const { return liquid_down_body_; }
    void set_liquid_down_body(const math::Vec3& down_body);  // normalised here

    // Discrete engine update for one tick: start/stop transients, throttle and gimbal rate limits,
    // relight rules, flow-separation limits. Must be called once per tick before integration.
    void update_engines(double dt_s, const VehicleCommand& command, double ambient_pressure_pa);
    // Grid fin actuators for one tick (deflection and rate limits).
    void update_grid_fins(double dt_s, const VehicleCommand& command);
    size_t grid_fin_count() const;
    // Landing legs of the lowest stage ([stage.landing_legs]): deploy() starts the deployment, which cannot be
    // reversed and takes deploy_time_s; leg_deployment() runs 0 (stowed) .. 1 (deployed, locked).
    bool has_landing_legs() const;
    void deploy_legs() { legs_deploying_ = has_landing_legs(); }
    bool legs_deploying() const { return legs_deploying_; }
    double leg_deployment() const { return leg_deployment_; }
    void set_leg_deployment(double fraction);  // scenarios: legs placed in a given state
    // Truth-model error injection: the hull's aerodynamic force (FlightBody) is scaled by this; aero() itself - what
    // the onboard models use - is unchanged.
    void set_hull_aero_scale(double factor) { hull_aero_scale_ = factor; }
    double hull_aero_scale() const { return hull_aero_scale_; }
    // Truth-model dispersion (tests, training): thrust and mass flow of every engine scaled by `factor` in the forces
    // the vehicle produces; the performance queries the guidance uses (thrust_n, current_performance) stay nominal.
    void set_thrust_factor(double factor) { thrust_factor_ = factor; }
    double thrust_factor() const { return thrust_factor_; }
    void update_landing_legs(double dt_s);
    // Parachute sets ([[stage.parachute]], d4). deploy opens the stowed sets of `role` ("drogue" / "main"); a released
    // or torn set stays so. update grows the open sets' drag areas (aero::parachute_drag_area_m2).
    bool deploy_parachutes(const std::string& role);
    void release_parachutes(const std::string& role);
    void update_parachutes(double dt_s);
    bool parachutes_out() const;
    bool parachute_torn() const;
    // Opening-load judgement (FlightBody): the peak per canopy; above the set's max_load_n the set tears.
    void record_parachute_load(size_t stage, size_t set, double load_per_canopy_n);
    // The lowest stage recovers by splashdown ([stage.recovery] method = "splashdown").
    bool splashdown_recovery() const;
    double max_splashdown_speed_mps() const;
    // Flap actuators for one tick (range [0, max] and rate limits).
    void update_flaps(double dt_s, const VehicleCommand& command);
    size_t flap_count() const;

    // Total propellant mass flow per tank (index: stage, tank) for the current engine state.
    std::vector<std::vector<double>> tank_mass_flows_kgps() const;
    // Applies a tick's propellant consumption; starves engines whose tanks ran dry.
    void consume_propellant(double dt_s);

    bool any_engine_producing_thrust() const;

    // Failure injection: engine `index` (in stage order) shuts down for good.
    void fail_engine(size_t stage_index, size_t engine_index);

    // Reaction control from the individual thrusters ([stage.rcs]) about the current CG: torque capability per body
    // axis (the smaller of + and -), and the force (with no torque) along a body direction with its mass flow.
    math::Vec3 max_rcs_torque_nm() const;
    struct RcsCapability {
        double force_n = 0.0;
        double mass_flow_kgps = 0.0;
    };
    RcsCapability rcs_force_capability(const math::Vec3& direction_body) const;
    // Allocation (torque first) and on/off pulses with a minimum on-time for one tick; must be called once per tick
    // before integration. The resulting torque (about the CG) and force of the firing thrusters, body axes:
    void update_rcs(double dt_s, const VehicleCommand& command);
    math::Vec3 rcs_torque_nm() const { return rcs_torque_; }
    math::Vec3 rcs_force_n() const { return rcs_force_; }
    // Duty (0..1) the allocator gave each thruster this tick, stage by stage in thruster order; empty before the first
    // update_rcs and after stop_rcs (all off).
    const std::vector<double>& rcs_duty() const { return rcs_duty_; }
    // All thrusters off and nothing owed (coast segments, impact; also after a separation).
    void stop_rcs();
    // Hinged nose cone of a stage (Crew Dragon): closed at construction; thrusters behind it are blocked while closed.
    bool has_nose_cone() const;
    // A payload vehicle (config::stack_payload) still on its launcher: a launcher stage below payload stages. A released
    // payload keeps its stages' payload flag, so this - not the flag - tells whether it flies alone.
    bool payload_stacked() const;
    bool nose_cone_open() const { return nose_cone_open_; }
    void set_nose_cone_open(bool open);
    const aero::SlenderBodyAero& aero() const { return *aero_; }
    // Blunt-capsule aerodynamics when the lowest stage has a capsule table (the capsule alone), else nullptr.
    const aero::CapsuleAero* capsule_aero() const { return capsule_aero_.get(); }
    // Hull aerodynamics for the flight state: the capsule's when present, else the slender body's.
    aero::AeroResult hull_aero(const math::Vec3& air_velocity_body_mps, const environment::AtmosphereState& air,
                               bool powered) const;

    // Splits the stack below stage `index`: this vehicle keeps stages [0, index), the returned one
    // stages [index, n). Stations of the upper part are re-based to its own bottom.
    std::unique_ptr<Vehicle> separate(size_t index);
    // A copy of stages [index, n) as they are now (tanks, nose cone), as separate() would split them off; this vehicle is
    // unchanged. Planning: the capsule a Crew Dragon will be after its trunk jettison.
    std::unique_ptr<Vehicle> copy_upper(size_t index) const;

private:
    Vehicle() = default;
    void rebuild_aero();
    const propulsion::RcsAllocator& rcs_allocator() const;  // for the current CG and the thrusters not starved
    void refresh_rcs_availability();  // starves thrusters whose supply ran dry, blocks those that cannot fire now

    std::string id_;
    std::filesystem::path config_dir_;
    std::vector<Stage> stages_;
    std::unique_ptr<aero::SlenderBodyAero> aero_;
    std::unique_ptr<aero::CapsuleAero> capsule_aero_;
    bool legs_deploying_ = false;
    double leg_deployment_ = 0.0;
    double hull_aero_scale_ = 1.0;
    double thrust_factor_ = 1.0;
    bool nose_cone_open_ = false;
    mutable std::optional<propulsion::RcsAllocator> rcs_allocator_;
    propulsion::RcsModulator rcs_modulator_;
    std::vector<double> rcs_duty_;
    math::Vec3 rcs_torque_;
    math::Vec3 rcs_force_;
    math::Vec3 liquid_down_body_{-1.0, 0.0, 0.0};
};

}  // namespace starsim::vehicle
