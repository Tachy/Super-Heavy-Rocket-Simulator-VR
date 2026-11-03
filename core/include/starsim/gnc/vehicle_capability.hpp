#pragma once

#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"

// Vehicle capabilities the planners derive from the config instead of carrying their own numbers: the stage's pitch
// inertia, the thrust and gimbal authority of an engine set, the angular acceleration from gimbal or RCS, and the
// time of a bang-bang turn. Capability estimates for planning, not the flight dynamics (vehicle::Vehicle is that).

namespace starsim::gnc {

// Pitch moment of inertia of a stage flying alone, about its CG, propellant settled aft: the flight dynamics' own
// model (vehicle::settled_stage_mass_properties, the parts of Vehicle::mass_properties). propellant_kg: the main
// oxidizer + fuel load; headers and RCS tanks at their configured mass. The CG station of the same load (for the
// levers below) is vehicle::settled_stage_mass_properties(...).cg_station_m.
double stage_pitch_inertia_kg_m2(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                 double propellant_kg);

// An engine set at one ambient pressure. min_n / max_n: the whole set at the common minimum / maximum throttle (the
// highest throttle_min and the lowest throttle_max of its engines). gimbal_rad / gimbal_rate_radps: the smallest of
// its gimballed engines (0: none gimbals). lever_m: CG station minus the thrust station, the shortest of the gimballed
// groups. gimballed_min_n / gimballed_max_n: the gimballed engines' part of min_n / max_n (< 0: the whole set gimbals);
// kept per end, as a set of mixed engine types (Raptor SL gimballed + RVac fixed) has a different gimballed share at
// each throttle and pressure. gimballed_fraction: gimballed_max_n / max_n, for information.
struct EngineSetThrust {
    double min_n;
    double max_n;
    double gimbal_rad;
    double gimbal_rate_radps;
    double lever_m;
    double throttle_min = 0.0;
    double throttle_max = 1.0;
    double gimballed_fraction = 1.0;
    double gimballed_min_n = -1.0;
    double gimballed_max_n = -1.0;
};

// groups: engine group ids of `stage`, bare ("center") or with the stage prefix ("booster/center"), each once.
// cg_station_m: stage-local, metres from the stage bottom (the frame of the groups' station_m and of
// vehicle::settled_stage_mass_properties), not a station of a stacked vehicle. The engines' performance comes from
// vehicle.engines (EnginePerformance). Throws std::invalid_argument for an unknown or repeated group.
EngineSetThrust engine_set_thrust(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                  const std::vector<std::string>& groups, double ambient_pa, double cg_station_m);

// Pitch acceleration of full gimbal deflection at `throttle` (engine throttle, the gimballed thrust linear in it
// between the set's ends): F_gimballed sin(gimbal) lever / I.
double max_pitch_acceleration_radps2(const EngineSetThrust& set, double throttle, double inertia_kg_m2);

// Pitch acceleration of the stage's RCS: the vehicle's own capability (propulsion::RcsAllocator::max_torque_nm, the
// pure torque per axis with the weaker sign, as Vehicle::max_rcs_torque_nm) about the CG on the axis; the weaker of
// the two transverse axes, so the turn can be started and stopped about either. Thrusters behind a nose cone are left
// out. cg_station_m: stage-local (from the stage bottom, the frame of the thruster positions).
double rcs_pitch_acceleration_radps2(const config::StageConfig& stage, double inertia_kg_m2, double cg_station_m);

// Rest-to-rest turn through angle_rad: accelerate, coast at max_rate_radps, decelerate. max_rate_radps <= 0: no cap
// (2 sqrt(angle / accel)). Throws std::invalid_argument for accel_radps2 <= 0 (no authority: the turn never ends).
double flip_duration_s(double angle_rad, double accel_radps2, double max_rate_radps);

// The same rest-to-rest turn through total_rad: the time until the axis is within tolerance_rad of its end (a burn that
// lights inside a pointing tolerance). tolerance_rad >= total_rad: 0.
double turn_time_within_s(double total_rad, double tolerance_rad, double accel_radps2, double max_rate_radps);

// A free turn from rest (no braking): accelerate up to max_rate_radps (<= 0: no cap), then coast; the time to turn
// through angle_rad. Throws std::invalid_argument for accel_radps2 <= 0.
double free_turn_time_s(double angle_rad, double accel_radps2, double max_rate_radps);

// The time the slowest engine of a set runs its whole throttle range, (throttle_max - throttle_min) / throttle_rate
// (0: no engine with a throttle rate). groups: as engine_set_thrust.
double throttle_range_time_s(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                             const std::vector<std::string>& groups);

}  // namespace starsim::gnc
