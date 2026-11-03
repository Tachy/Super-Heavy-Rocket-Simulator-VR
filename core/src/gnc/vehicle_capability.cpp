#include "starsim/gnc/vehicle_capability.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "starsim/math/vec3.hpp"
#include "starsim/propulsion/engine_performance.hpp"
#include "starsim/propulsion/rcs_allocator.hpp"
#include "starsim/vehicle/stage_inertia.hpp"

namespace starsim::gnc {

namespace {

const config::EngineGroupConfig& find_group(const config::StageConfig& stage, const std::string& key) {
    const std::string prefix = stage.id + "/";
    const std::string id = key.rfind(prefix, 0) == 0 ? key.substr(prefix.size()) : key;
    for (const auto& g : stage.engine_groups) {
        if (g.id == id) return g;
    }
    throw std::invalid_argument("vehicle_capability: stage '" + stage.id + "' has no engine group '" + key + "'");
}

}  // namespace

double stage_pitch_inertia_kg_m2(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                 double propellant_kg) {
    return vehicle::settled_stage_mass_properties(vehicle, stage, propellant_kg).pitch_inertia_kg_m2;
}

EngineSetThrust engine_set_thrust(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                  const std::vector<std::string>& groups, double ambient_pa, double cg_station_m) {
    constexpr double inf = std::numeric_limits<double>::infinity();
    struct Member {
        const config::EngineGroupConfig* group;
        propulsion::EnginePerformance engine;
    };
    std::vector<Member> members;
    double throttle_min = 0.0, throttle_max = inf;
    for (const auto& key : groups) {
        const auto& g = find_group(stage, key);
        for (const auto& m : members) {
            if (m.group == &g) {
                throw std::invalid_argument("vehicle_capability: engine group '" + key + "' listed twice");
            }
        }
        members.push_back({&g, propulsion::EnginePerformance(vehicle.engine(g.engine_id))});
        throttle_min = std::max(throttle_min, members.back().engine.config().throttle_min);
        throttle_max = std::min(throttle_max, members.back().engine.config().throttle_max);
    }
    EngineSetThrust set{0.0, 0.0, inf, inf, inf, throttle_min, members.empty() ? 1.0 : throttle_max, 1.0};
    double gimballed_min_n = 0.0, gimballed_max_n = 0.0;
    for (const auto& m : members) {
        const double n = m.group->count;
        const double low_n = n * m.engine.thrust_n(set.throttle_min, ambient_pa);
        const double full_n = n * m.engine.thrust_n(set.throttle_max, ambient_pa);
        set.min_n += low_n;
        set.max_n += full_n;
        if (!m.group->gimbal) continue;
        gimballed_min_n += low_n;
        gimballed_max_n += full_n;
        set.gimbal_rad = std::min(set.gimbal_rad, m.engine.config().gimbal_max_rad);
        set.gimbal_rate_radps = std::min(set.gimbal_rate_radps, m.engine.config().gimbal_rate_rad_s);
        set.lever_m = std::min(set.lever_m, cg_station_m - m.group->station_m);
    }
    if (gimballed_max_n <= 0.0) {
        set.gimbal_rad = 0.0;
        set.gimbal_rate_radps = 0.0;
        set.lever_m = 0.0;
        set.gimballed_fraction = 0.0;
        set.gimballed_min_n = 0.0;
        set.gimballed_max_n = 0.0;
    } else {
        set.gimballed_fraction = gimballed_max_n / set.max_n;
        set.gimballed_min_n = gimballed_min_n;
        set.gimballed_max_n = gimballed_max_n;
    }
    return set;
}

double max_pitch_acceleration_radps2(const EngineSetThrust& set, double throttle, double inertia_kg_m2) {
    // Thrust is linear in the throttle (EnginePerformance::thrust_n): interpolate the gimballed engines' thrust between
    // the set's ends (a set built by hand without the gimballed ends gimbals as a whole).
    const double low_n = set.gimballed_min_n < 0.0 ? set.min_n : set.gimballed_min_n;
    const double high_n = set.gimballed_max_n < 0.0 ? set.max_n : set.gimballed_max_n;
    const double span = set.throttle_max - set.throttle_min;
    const double t = std::clamp(throttle, set.throttle_min, set.throttle_max);
    const double thrust_n = span > 0.0 ? low_n + (t - set.throttle_min) / span * (high_n - low_n) : high_n;
    return thrust_n * std::sin(set.gimbal_rad) * set.lever_m / inertia_kg_m2;
}

double rcs_pitch_acceleration_radps2(const config::StageConfig& stage, double inertia_kg_m2, double cg_station_m) {
    // The vehicle's own capability (Vehicle::max_rcs_torque_nm): the allocator's pure torque per axis, the weaker sign.
    std::vector<propulsion::RcsThrusterGeometry> thrusters;
    for (const auto& t : stage.rcs.thrusters) {
        if (!t.behind_nose_cone) thrusters.push_back({t.position_m, t.direction, t.thrust_n});
    }
    const math::Vec3 torque_nm = propulsion::RcsAllocator(std::move(thrusters), cg_station_m).max_torque_nm();
    return std::min(torque_nm.y, torque_nm.z) / inertia_kg_m2;
}

double flip_duration_s(double angle_rad, double accel_radps2, double max_rate_radps) {
    if (!(accel_radps2 > 0.0)) {
        throw std::invalid_argument("flip_duration_s: angular acceleration must be > 0 (no turn authority)");
    }
    const double angle = std::abs(angle_rad);
    if (max_rate_radps <= 0.0 || max_rate_radps * max_rate_radps >= angle * accel_radps2) {
        return 2.0 * std::sqrt(angle / accel_radps2);  // triangle: the cap is never reached
    }
    // Trapezoid: two ramps of w / a each cover w^2 / a, the rest coasts at w.
    return angle / max_rate_radps + max_rate_radps / accel_radps2;
}

double turn_time_within_s(double total_rad, double tolerance_rad, double accel_radps2, double max_rate_radps) {
    const double total = std::abs(total_rad);
    const double reach = total - std::abs(tolerance_rad);
    if (reach <= 0.0) return 0.0;
    const double duration = flip_duration_s(total, accel_radps2, max_rate_radps);
    const bool capped = max_rate_radps > 0.0 && max_rate_radps * max_rate_radps < total * accel_radps2;
    // angle covered by the acceleration ramp (and, symmetric, by the braking ramp)
    const double ramp = capped ? 0.5 * max_rate_radps * max_rate_radps / accel_radps2 : 0.5 * total;
    if (reach <= ramp) return std::sqrt(2.0 * reach / accel_radps2);
    if (reach <= total - ramp) return max_rate_radps / accel_radps2 + (reach - ramp) / max_rate_radps;
    return duration - std::sqrt(2.0 * (total - reach) / accel_radps2);
}

double free_turn_time_s(double angle_rad, double accel_radps2, double max_rate_radps) {
    if (!(accel_radps2 > 0.0)) {
        throw std::invalid_argument("free_turn_time_s: angular acceleration must be > 0 (no turn authority)");
    }
    const double angle = std::abs(angle_rad);
    const double ramp = max_rate_radps > 0.0 ? 0.5 * max_rate_radps * max_rate_radps / accel_radps2 : angle;
    if (angle <= ramp) return std::sqrt(2.0 * angle / accel_radps2);
    return max_rate_radps / accel_radps2 + (angle - ramp) / max_rate_radps;
}

double landing_phase_rate_limit_radps(const EngineSetThrust& set, double tilt_max_rad, double inertia_kg_m2) {
    const double alpha = max_pitch_acceleration_radps2(set, set.throttle_min, inertia_kg_m2);
    return std::min(set.gimbal_rate_radps, std::sqrt(alpha * tilt_max_rad));
}

double throttle_range_time_s(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                             const std::vector<std::string>& groups) {
    double time_s = 0.0;
    for (const auto& key : groups) {
        const auto& e = vehicle.engine(find_group(stage, key).engine_id);
        if (e.throttle_rate_per_s > 0.0) time_s = std::max(time_s, (e.throttle_max - e.throttle_min) / e.throttle_rate_per_s);
    }
    return time_s;
}

}  // namespace starsim::gnc
