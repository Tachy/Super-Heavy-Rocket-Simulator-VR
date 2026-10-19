#pragma once

#include <filesystem>
#include <map>
#include <stdexcept>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "starsim/math/vec3.hpp"

// Vehicle definitions loaded from config/ (see config/README.md). Plain data, SI units, radians.

namespace starsim::config {

class ConfigError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct PropellantConfig {
    std::string id;
    double mixture_ratio = 0.0;  // oxidizer / fuel by mass
    double oxidizer_density_kg_m3 = 0.0;
    double fuel_density_kg_m3 = 0.0;
};

struct EngineConfig {
    std::string id;
    std::string name;
    double mixture_ratio = 0.0;
    double reference_pressure_pa = 0.0;
    double thrust_ref_n = 0.0;
    double isp_ref_s = 0.0;
    double nozzle_exit_area_m2 = 0.0;
    double max_ambient_pressure_pa = 0.0;
    double throttle_min = 0.0;
    double throttle_max = 1.0;
    double throttle_rate_per_s = 0.0;
    double gimbal_max_rad = 0.0;
    double gimbal_rate_rad_s = 0.0;
    double dry_mass_kg = 0.0;
    double startup_s = 0.0;
    double shutdown_s = 0.0;
};

enum class TankContents { Oxidizer, Fuel, Rcs };  // Rcs: a reaction-control supply (e.g. nitrogen)
// Main tanks feed the engines by default; header tanks (Starship) hold the landing propellant and feed the flip and
// landing burn once the stage selects them (vehicle::FeedSource).
enum class TankRole { Main, Header };

struct TankConfig {
    std::string id;
    TankContents contents = TankContents::Oxidizer;
    double mass_kg = 0.0;          // loaded propellant
    double base_station_m = 0.0;   // bottom of the tank, from the bottom of the stage
    double density_kg_m3 = 0.0;    // Rcs tanks only (the main propellants take theirs from the propellant)
    double capacity_kg = 0.0;      // full load: the configured mass_kg at config load (scenarios change mass_kg only)
    TankRole role = TankRole::Main;
    double diameter_m = 0.0;       // 0: the stage diameter (a header is a small tank inside the hull)
};

// One reaction-control thruster: fixed thrust, on/off.
struct RcsThrusterConfig {
    std::string id;
    math::Vec3 position_m;  // station along the stage axis from the stage bottom, y, z (body axes)
    math::Vec3 direction;   // unit; the force on the vehicle (the plume points the other way)
    double thrust_n = 0.0;
    bool behind_nose_cone = false;  // fires only with the stage's nose cone open
};

// [stage.rcs]: pods ([[stage.rcs.pod]]) are expanded into thrusters by the loader.
struct RcsConfig {
    double isp_s = 0.0;
    std::string propellant;            // "main" (the stage's oxidizer and fuel tanks: ullage gas) or an Rcs tank id
    double minimum_on_time_s = 0.0;    // shortest pulse (valve)
    std::vector<RcsThrusterConfig> thrusters;
};

struct EngineGroupConfig {
    std::string id;
    std::string engine_id;
    int count = 0;
    double ring_radius_m = 0.0;
    bool gimbal = false;
    bool relight = false;
    std::vector<double> angles_rad;  // explicit ring angles (one per engine); empty: evenly spaced from 0
    bool abort_only = false;         // launch escape (SuperDraco): data only, never lit by the session
    double station_m = 0.0;          // thrust point above the bottom of the stage (0: at the base)
    double cant_rad = 0.0;           // thrust tilted towards the axis (exhaust away from the hull); no gimbal then
    bool landing_only = false;       // landing thrusters (HLS): never a burn or ascent engine, flown by landing guidance
};

struct AeroConfig {
    double reference_diameter_m = 0.0;
    std::string ascent_drag_table;  // path relative to config/, empty if none
    std::string descent_drag_table; // tail-first flight (booster return); empty = mirror the ascent table
    std::string crossflow_table;    // hull cross-flow eta * C_dc over Mach; used when the stage flies alone
    double nose_length_m = 0.0;     // ogive nose (0 = flat top, e.g. a booster)
    std::string flap_table;         // flap normal-force table (mach, cn_max), path relative to config/
    double belly_azimuth_rad = 1.5 * 3.14159265358979323846;  // body direction of the heat shield (default -z)
    std::string entry_alpha_table;  // nominal entry angle of attack over Mach (mach, alpha_deg); entry guidance
    std::string capsule_table;      // blunt capsule (mach, alpha_deg, ca, cn, cm), used once the stage is the lowest
    std::string entry_profile = "ship";  // entry guidance phases: "ship" (terminal nose turn) or "capsule"
    double fixed_drag_area_m2 = 0.0;      // > 0: fixed C_D A drag (a station), replaces the slender-body aero
    double fixed_drag_coefficient = 0.0;
    double solar_area_m2 = 0.0;           // > 0: fixed area for solar radiation pressure (else the cylinder model)
};

// [stage.station.reference_orbit]: the synthetic station orbit without any OEM, elements in the true equator frame of
// reference_utc (UTC text, converted by the session).
struct StationReferenceOrbitConfig {
    double altitude_m = 0.0;  // semi-major axis minus the equatorial radius
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    std::string reference_utc;
};

// [[stage.docking_port]]: stage frame (x = station from the stage bottom, y/z body axes); `axis` is the outward unit
// vector, `up` the clocking reference perpendicular to it.
// IDSS: one side extends its soft-capture ring (active), the other stays retracted (passive) - ISS i5.
enum class SoftCaptureMode { Passive, Active };

struct DockingPortConfig {
    std::string id;
    std::string name;
    math::Vec3 position_m;
    math::Vec3 axis;
    math::Vec3 up;
    bool behind_nose_cone = false;  // covered while the stage's nose cone is closed (Crew Dragon)
    SoftCaptureMode soft_capture = SoftCaptureMode::Passive;  // TOML soft_capture = "active" | "passive"
};

// [[stage.collision]]: "capsule" = segment a-b with radius; "box" = centre a_m, body-aligned, half_extents_m.
struct CollisionPrimitiveConfig {
    std::string id;
    std::string kind;
    math::Vec3 a_m;
    math::Vec3 b_m;
    double radius_m = 0.0;
    math::Vec3 half_extents_m;
};

// One flap of a stage (Starship): flat plate hinged along the hull side, see aero/flap.hpp.
struct FlapConfig {
    std::string id;
    double hinge_azimuth_rad = 0.0;  // from body +y towards +z
    double station_fwd_m = 0.0;      // from the bottom of the stage
    double station_aft_m = 0.0;
    double span_m = 0.0;
    double area_m2 = 0.0;
    double max_deflection_rad = 0.0;
    double rate_rad_s = 0.0;
    double initial_deflection_rad = 0.0;
};

// Grid fins of a stage (aerodynamic control surfaces, e.g. for the booster's return).
struct GridFinConfig {
    std::vector<double> azimuths_rad;  // one fin per entry, from body +y towards +z
    double area_m2 = 0.0;              // frontal lattice area per fin
    double station_m = 0.0;            // hinge station from the bottom of the stage
    double radius_m = 0.0;             // radius of the fin's centre of pressure
    double max_deflection_rad = 0.0;
    double rate_rad_s = 0.0;
    std::string table;                 // normal- and axial-force table, path relative to config/
};

// Catch pins of a stage: hardpoints the tower arms catch (the returning booster).
struct CatchPinConfig {
    std::vector<double> azimuths_rad;  // one pin per entry, from body +y towards +z (like the grid fins)
    double station_m = 0.0;            // from the bottom of the stage
    double protrusion_m = 0.0;         // beyond the hull radius
    double max_load_n = 0.0;           // structural limit per pin
};

// Landing legs of a stage that lands on the ground or a deck (Falcon 9). Deployed geometry and the crush-core contact
// parameters per leg ([stage.landing_legs]); no legs when azimuths_rad is empty.
struct LandingLegConfig {
    std::vector<double> azimuths_rad;  // one leg per entry, from body +y towards +z
    double footprint_radius_m = 0.0;   // foot from the axis, deployed
    double foot_station_m = 0.0;       // foot station from the bottom of the stage (negative: below it), deployed
    double deploy_time_s = 0.0;
    double stroke_m = 0.0;             // crush-core stroke
    double crush_force_n = 0.0;        // the core crushes above this force
    double stiffness_n_m = 0.0;        // before the core crushes
    double damping_n_s_m = 0.0;
    double friction = 0.0;             // Coulomb, foot on the surface
    bool fixed = false;                // always deployed (HLS): no deployment, deploy_time_s unused
};

// Heat shield and structure of a vehicle that enters the atmosphere (the ship). Judged while the stage flies alone.
struct HeatShieldConfig {
    double nose_radius_m = 0.0;        // effective radius of the windward side for Sutton-Graves; 0 = none
    double max_heat_rate_w_m2 = 0.0;
    double max_heat_load_j_m2 = 0.0;
    double max_load_g = 0.0;           // aerodynamic acceleration (aero force / mass); 0 = not judged
    double crew_load_limit_g = 0.0;    // crew: warning above it and the entry guidance's load limit; 0 = none
};

// One reefing stage of a parachute: the canopy fills to `area_fraction` of its full drag area and holds there.
struct ReefingStageConfig {
    double area_fraction = 0.0;
    double hold_s = 0.0;
};

// A set of identical parachutes ([[stage.parachute]]; d4): massless drag areas on a riser point on the axis.
struct ParachuteConfig {
    std::string id;
    std::string role;                  // "drogue" or "main" (gnc::ParachuteSequencer)
    int count = 0;                     // canopies in the set
    double drag_area_m2 = 0.0;         // C_D S0 of one canopy, fully open
    double attach_station_m = 0.0;     // riser attachment on the axis, from the bottom of the stage
    double fill_time_s = 0.0;          // inflation of each reefing stage and of the final opening
    std::vector<ReefingStageConfig> reefing;  // ascending area fractions in (0, 1); empty: opens to full at once
    double max_load_n = 0.0;           // per canopy: beyond it the set tears ("Fallschirm gerissen")
    double deploy_height_m = 0.0;      // above the ground / sea
    double max_deploy_dynamic_pressure_pa = 0.0;  // the deployment waits above it; 0 = no check
};

// Return of a first stage (gnc::booster_return_parameters; empty / 0: the guidance's defaults, which are Super Heavy's).
// Engine groups are ids of the stage.
struct ReturnProfile {
    bool boostback = true;  // false: no flip and no boostback, the stage coasts on its ascent arc (drone ship)
    std::vector<std::string> boostback_engine_groups;
    std::vector<std::string> entry_engine_groups;
    std::vector<std::vector<std::string>> landing_engine_phases;  // 3 sets: ignition, divert, final
    std::vector<double> landing_phase_throttles;                  // design throttle per set (ignition timing)
    std::vector<double> landing_phase_max_tilt_rad;               // planned thrust tilt limit per set (gimbal authority)
    std::vector<std::pair<double, double>> landing_switch_sink_mps;  // per engine switch: sink window {min, max}
    double return_flight_time_s = 0.0;   // boostback start -> touchdown (Lambert arc)
    double boostback_loss_fraction = -1.0;  // MECO planning: boostback delta-v beyond the Lambert arc (< 0: default)
    double rcs_propellant_kg = -1.0;        // MECO planning: main-tank RCS gas spent before landing-burn ignition (< 0: default)
    double entry_burn_altitude_m = 0.0;  // entry burn ignition while descending (0: none)
    double entry_burn_delta_v_mps = 0.0;
    double entry_burn_throttle = 0.0;
};

struct StageConfig {
    std::string id;
    std::string name;
    double length_m = 0.0;
    double diameter_m = 0.0;
    double hull_top_diameter_m = 0.0;  // optional: the hull tapers linearly to this at the top (0: = diameter_m)
    double dry_mass_kg = 0.0;  // includes engines
    double dry_cg_station_m = 0.0;
    math::Vec3 dry_cg_offset_m;  // lateral dry CG offset (y, z) from the axis (capsule trim); x unused (0)
    double payload_mass_kg = 0.0;
    double fairing_mass_kg = 0.0;
    double jettison_after_separation_kg = 0.0;
    double reserve_propellant_kg = 0.0;  // MECO without return guidance
    // Returning booster: propellant to have left at landing-burn ignition; the ascent autopilot plans MECO from it
    // and the return's needs (0: MECO at reserve_propellant_kg).
    double landing_propellant_kg = 0.0;
    // [stage.recovery] method: "tower_catch" (Super Heavy: always caught, no legs), "leg_landing" (Falcon 9: on a
    // landing zone or a drone ship, chosen by the mission); empty: expendable.
    std::string recovery_method;
    // [stage.recovery] return of a first stage to the launch or landing site / a catch tower, and
    // [stage.recovery.drone_ship] the return to a drone ship (keys left out there come from the first profile).
    ReturnProfile return_profile;
    std::optional<ReturnProfile> drone_ship_return;
    const ReturnProfile& return_profile_for(bool drone_ship) const {
        return drone_ship && drone_ship_return ? *drone_ship_return : return_profile;
    }
    // [stage.recovery] deorbit_engine_groups: the engine set deorbit burns run on (empty: the burn picks one). The ship:
    // the belly sea-level Raptor alone, its thrust steered through the CG (user 2026-10-13).
    std::vector<std::string> deorbit_engine_groups;
    LandingLegConfig landing_legs;
    double radiation_pressure_coefficient = 1.3;  // Cr for solar radiation pressure
    RcsConfig rcs;                                 // reaction control (no thrusters: none)
    std::string separation;
    bool nose_cone = false;  // a hinged nose cone (Crew Dragon): thrusters behind it fire only while it is open
    bool payload = false;    // a payload vehicle's stage (config::stack_payload): its RCS waits until it flies alone
    std::vector<TankConfig> tanks;
    std::vector<EngineGroupConfig> engine_groups;
    AeroConfig aero;
    GridFinConfig grid_fins;  // empty azimuths: none
    CatchPinConfig catch_pins;  // empty azimuths: none
    std::vector<FlapConfig> flaps;
    HeatShieldConfig heat_shield;
    std::vector<ParachuteConfig> parachutes;  // [[stage.parachute]] (d4: Crew Dragon drogues and mains)
    // [stage.recovery] max_splashdown_speed_mps (method "splashdown", required there): sink rate at the water up to
    // which the capsule is "gewassert"; faster is an impact.
    double max_splashdown_speed_mps = 0.0;
    std::optional<math::Vec3> inertia_kg_m2;      // principal moments of the dry stage about its own CG (else a cylinder)
    std::optional<math::Vec3> body_in_lvlh_rad;   // a station: ideal LVLH attitude, yaw / pitch / roll (TEA)
    std::optional<StationReferenceOrbitConfig> reference_orbit;  // a station: synthetic orbit defaults
    std::vector<DockingPortConfig> docking_ports;
    std::vector<CollisionPrimitiveConfig> collision;
};

struct VehicleConfig {
    std::string id;
    std::string name;
    std::string display_name;
    PropellantConfig propellant;
    std::vector<StageConfig> stages;              // bottom to top
    std::map<std::string, EngineConfig> engines;  // all engines referenced by the stages

    const EngineConfig& engine(const std::string& id) const;
};

// Loads config/vehicles/<vehicle_id>.toml plus the referenced propellant and engine files.
VehicleConfig load_vehicle(const std::filesystem::path& config_dir, std::string_view vehicle_id);

// The launcher with a payload vehicle on top (a mission's [payload]): the payload's stages follow the launcher's, marked
// `payload`; the top launcher stage carries no payload mass. Throws ConfigError if a stage id occurs twice.
VehicleConfig stack_payload(VehicleConfig launcher, const VehicleConfig& payload);

// Numeric CSV table: '#' comment lines, one header line, then rows of numbers.
struct CsvTable {
    std::vector<std::string> columns;
    std::vector<std::vector<double>> rows;

    std::vector<double> column(std::string_view name) const;
};

CsvTable load_csv_table(const std::filesystem::path& path);

}  // namespace starsim::config
