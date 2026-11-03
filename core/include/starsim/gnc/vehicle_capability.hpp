#pragma once

#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"

// Vehicle capabilities the planners derive from the config instead of carrying their own numbers: the stage's pitch
// inertia, the thrust and gimbal authority of an engine set, the angular acceleration from gimbal or RCS, and the
// time of a bang-bang turn. Capability estimates for planning, not the flight dynamics (vehicle::Vehicle is that).

namespace starsim::gnc {

// Pitch moment of inertia of a stage flying alone about its centre: the dry stage (inertia_kg_m2 if set, else the
// vehicle's thin-walled cylinder) plus the propellant as the same cylinder of the hull [E] (no tank layout).
double stage_pitch_inertia_kg_m2(const config::StageConfig& stage, double propellant_kg);

// An engine set at one ambient pressure. min_n / max_n: the whole set at the common minimum / maximum throttle (the
// highest throttle_min and the lowest throttle_max of its engines). gimbal_rad / gimbal_rate_radps: the smallest of
// its gimballed engines (0: none gimbals). lever_m: CG station minus the thrust station, the shortest of the gimballed
// groups. gimballed_fraction: share of the set's thrust on gimballed engines (at full throttle).
struct EngineSetThrust {
    double min_n;
    double max_n;
    double gimbal_rad;
    double gimbal_rate_radps;
    double lever_m;
    double throttle_min = 0.0;
    double throttle_max = 1.0;
    double gimballed_fraction = 1.0;
};

// groups: engine group ids of `stage`, bare ("center") or with the stage prefix ("booster/center"). The engines'
// performance comes from vehicle.engines (EnginePerformance). Throws std::invalid_argument for an unknown group.
EngineSetThrust engine_set_thrust(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                  const std::vector<std::string>& groups, double ambient_pa, double cg_station_m);

// Pitch acceleration of full gimbal deflection at `throttle` (engine throttle, thrust linear in it between the set's
// ends): F_gimballed sin(gimbal) lever / I.
double max_pitch_acceleration_radps2(const EngineSetThrust& set, double throttle, double inertia_kg_m2);

// Pitch acceleration of the stage's RCS: the vehicle's own capability (propulsion::RcsAllocator::max_torque_nm, the
// pure torque per axis with the weaker sign, as Vehicle::max_rcs_torque_nm) about the CG on the axis; the weaker of
// the two transverse axes, so the turn can be started and stopped about either. Thrusters behind a nose cone are left
// out.
double rcs_pitch_acceleration_radps2(const config::StageConfig& stage, double inertia_kg_m2, double cg_station_m);

// Rest-to-rest turn through angle_rad: accelerate, coast at max_rate_radps, decelerate. max_rate_radps <= 0: no cap
// (2 sqrt(angle / accel)).
double flip_duration_s(double angle_rad, double accel_radps2, double max_rate_radps);

}  // namespace starsim::gnc
