#pragma once

// Constants and helpers shared by the booster-return guidance units (booster_return*.cpp). Private to core/src/gnc.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::gnc::booster_return_detail {

using math::Mat3;
using math::Vec3;

inline constexpr double touchdown_speed_mps = 1.0;
inline constexpr double trim_deadband_m = 2.0;
// Entry burn shooting along the ground track: the probe burn is this much longer (secant on the impact prediction).
inline constexpr double entry_delta_v_probe_mps = 20.0;
// Entry burn ignition shooting in the coast: the probe ignites this much higher.
inline constexpr double entry_altitude_probe_m = 1000.0;
inline constexpr double aero_steering_min_q_pa = 2000.0;  // below: RCS translation trim
inline constexpr double aero_steering_time_fraction = 0.5;
inline constexpr double aero_steering_max_alpha_rad = 8.0 * constants::deg_to_rad;  // [E] fin trim authority, heating
inline constexpr double minimum_time_to_go_s = 0.3;
// Centre-engine approach: sink rate follows touchdown speed + height / tau (an exponential approach, the V3 profile
// ending in a near hover; the same shape as the SCvx plans' approach limit), with a proportional sink-rate loop.
inline constexpr double approach_time_constant_s = 3.0;  // [E]
inline constexpr double approach_gain_per_s = 1.0;       // [E]
inline constexpr double touchdown_cutoff_height_m = 0.05; // engine cutoff when the legs / catch arms take the load
inline constexpr double divert_time_constant_s = 2.0;      // cross-track damping during the divert
inline constexpr double hold_time_constant_s = 4.0;        // position hold on the centre engines
inline constexpr double divert_hold_radius_m = 3.0;        // inside: position hold instead of braking profile
inline constexpr double catch_missed_height_m = 5.0;  // CG this far below the catch height: the arms were missed
inline constexpr double tilt_taper_height_m = 60.0;                           // below: tilt limit tapers off
inline constexpr double leg_deploy_margin_s = 5.0;      // [E] legs locked this long before the planned touchdown
inline constexpr double pad_cutoff_after_plan_s = 0.5;  // pad landing: engines off this long after the plan's end (spec)
inline constexpr double touchdown_tilt_rad = 4.0 * constants::deg_to_rad;     // tilt limit at touchdown
inline constexpr double entry_no_thrust_s = 5.0;  // [E] entry burn without thrust this long (engines out): it is over
// Online drag estimate (as the ship's entry guidance): filter gain per 0.2 s update, only with a clear signal.
inline constexpr double drag_scale_filter_gain = 0.1;          // [E]
inline constexpr double drag_scale_min_acceleration_mps2 = 1.0; // [E]

inline Vec3 up_at(double lat, double lon) { return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)}; }

// Rotation about z by angle a (Earth rotation over short horizons; precession/nutation negligible).
inline Mat3 rotation_z(double a) {
    Mat3 m = Mat3::identity();
    m(0, 0) = std::cos(a);
    m(0, 1) = -std::sin(a);
    m(1, 0) = std::sin(a);
    m(1, 1) = std::cos(a);
    return m;
}

struct EngineSet {
    double thrust_n = 0.0;        // full throttle at the given ambient pressure
    double vacuum_thrust_n = 0.0; // full throttle in vacuum (the pressure loss does not scale with the throttle)
    double mass_flow_kgps = 0.0;  // full throttle
    double startup_s = 0.0;
    double shutdown_s = 0.0;
};

inline EngineSet engine_set(const vehicle::Vehicle& v, const std::vector<std::string>& groups, double pressure_pa) {
    EngineSet set;
    for (const auto& stage : v.stages()) {
        for (const auto& e : stage.engines) {
            if (e.starved || std::find(groups.begin(), groups.end(), e.group_id) == groups.end()) continue;
            set.thrust_n += e.performance->thrust_n(1.0, pressure_pa);
            set.vacuum_thrust_n += e.performance->thrust_n(1.0, 0.0);
            set.mass_flow_kgps += e.performance->mass_flow_kgps(1.0);
            set.startup_s = std::max(set.startup_s, e.performance->config().startup_s);
            set.shutdown_s = std::max(set.shutdown_s, e.performance->config().shutdown_s);
        }
    }
    return set;
}

}  // namespace starsim::gnc::booster_return_detail
