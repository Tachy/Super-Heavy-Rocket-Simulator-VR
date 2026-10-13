#pragma once

#include <filesystem>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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

enum class TankContents { Oxidizer, Fuel };

struct TankConfig {
    std::string id;
    TankContents contents = TankContents::Oxidizer;
    double mass_kg = 0.0;          // loaded propellant
    double base_station_m = 0.0;   // bottom of the tank, from the bottom of the stage
};

struct EngineGroupConfig {
    std::string id;
    std::string engine_id;
    int count = 0;
    double ring_radius_m = 0.0;
    bool gimbal = false;
    bool relight = false;
    std::vector<double> angles_rad;  // explicit ring angles (one per engine); empty: evenly spaced from 0
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
};

// Heat shield and structure of a vehicle that enters the atmosphere (the ship). Judged while the stage flies alone.
struct HeatShieldConfig {
    double nose_radius_m = 0.0;        // effective radius of the windward side for Sutton-Graves; 0 = none
    double max_heat_rate_w_m2 = 0.0;
    double max_heat_load_j_m2 = 0.0;
    double max_load_g = 0.0;           // aerodynamic acceleration (aero force / mass); 0 = not judged
};

struct StageConfig {
    std::string id;
    std::string name;
    double length_m = 0.0;
    double diameter_m = 0.0;
    double dry_mass_kg = 0.0;  // includes engines
    double dry_cg_station_m = 0.0;
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
    // [stage.recovery] return of a first stage (gnc::booster_return_parameters; empty / 0: the guidance's defaults,
    // which are Super Heavy's). Engine groups are ids of this stage.
    std::vector<std::string> boostback_engine_groups;
    std::vector<std::string> entry_engine_groups;
    std::vector<std::vector<std::string>> landing_engine_phases;  // 3 sets: ignition, divert, final
    std::vector<double> landing_phase_throttles;                  // design throttle per set (ignition timing)
    std::vector<std::pair<double, double>> landing_switch_sink_mps;  // per engine switch: sink window {min, max}
    double return_flight_time_s = 0.0;   // boostback start -> touchdown (Lambert arc)
    double entry_burn_altitude_m = 0.0;  // entry burn ignition while descending (0: none)
    double entry_burn_delta_v_mps = 0.0;
    double entry_burn_throttle = 0.0;
    // [stage.recovery] deorbit_engine_groups: the engine set deorbit burns run on (empty: the burn picks one). The ship:
    // the belly sea-level Raptor alone, its thrust steered through the CG (user 2026-10-13).
    std::vector<std::string> deorbit_engine_groups;
    LandingLegConfig landing_legs;
    double radiation_pressure_coefficient = 1.3;  // Cr for solar radiation pressure
    int rcs_thruster_count = 0;                    // reaction control (cold gas)
    double rcs_thrust_n = 0.0;                     // per thruster
    std::string separation;
    std::vector<TankConfig> tanks;
    std::vector<EngineGroupConfig> engine_groups;
    AeroConfig aero;
    GridFinConfig grid_fins;  // empty azimuths: none
    CatchPinConfig catch_pins;  // empty azimuths: none
    std::vector<FlapConfig> flaps;
    HeatShieldConfig heat_shield;
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

// Numeric CSV table: '#' comment lines, one header line, then rows of numbers.
struct CsvTable {
    std::vector<std::string> columns;
    std::vector<std::vector<double>> rows;

    std::vector<double> column(std::string_view name) const;
};

CsvTable load_csv_table(const std::filesystem::path& path);

}  // namespace starsim::config
