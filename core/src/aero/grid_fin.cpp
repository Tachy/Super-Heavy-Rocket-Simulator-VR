#include "starsim/aero/grid_fin.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::aero {

using math::Vec3;

namespace {

double interpolate(const std::vector<double>& x, const std::vector<double>& y, double at) {
    if (at <= x.front()) return y.front();
    if (at >= x.back()) return y.back();
    const auto upper = std::upper_bound(x.begin(), x.end(), at);
    const size_t i = static_cast<size_t>(upper - x.begin());
    const double f = (at - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + f * (y[i] - y[i - 1]);
}

struct LocalFlow {
    double speed = 0.0;
    double dynamic_pressure = 0.0;
    double mach = 0.0;
    Vec3 direction;  // of the fin's motion through the air
};

LocalFlow local_flow(const Vec3& fin_velocity, const FinAirData& air) {
    LocalFlow flow;
    flow.speed = norm(fin_velocity);
    if (flow.speed < 1e-6 || air.density_kg_m3 <= 0.0) return flow;
    flow.dynamic_pressure = 0.5 * air.density_kg_m3 * flow.speed * flow.speed * (1.0 - air.rarefaction);
    flow.mach = air.speed_of_sound_mps > 0.0 ? flow.speed / air.speed_of_sound_mps : 0.0;
    flow.direction = fin_velocity / flow.speed;
    return flow;
}

}  // namespace

GridFinTable::GridFinTable(const config::CsvTable& table)
    : mach_(table.column("mach")), slope_(table.column("cn_alpha_per_rad")) {
    for (double deg : table.column("limit_deg")) limit_rad_.push_back(deg * constants::deg_to_rad);
    if (!std::is_sorted(mach_.begin(), mach_.end()) || mach_.size() < 2) {
        throw config::ConfigError("grid fin table: mach column must be ascending with >= 2 rows");
    }
}

double GridFinTable::normal_force_slope_per_rad(double mach) const {
    return empty() ? 0.0 : interpolate(mach_, slope_, mach);
}

double GridFinTable::limit_rad(double mach) const { return empty() ? 0.0 : interpolate(mach_, limit_rad_, mach); }

Vec3 grid_fin_radial(const GridFinGeometry& fin) {
    return {0.0, std::cos(fin.azimuth_rad), std::sin(fin.azimuth_rad)};
}

Vec3 grid_fin_tangential(const GridFinGeometry& fin) {
    return {0.0, -std::sin(fin.azimuth_rad), std::cos(fin.azimuth_rad)};
}

GridFinForce grid_fin_force(const GridFinGeometry& fin, double deflection_rad, const Vec3& fin_velocity_body_mps,
                            const FinAirData& air, const GridFinTable& table) {
    GridFinForce result;
    result.application_point_m = Vec3{fin.station_m, 0.0, 0.0} + grid_fin_radial(fin) * fin.radius_m;
    const LocalFlow flow = local_flow(fin_velocity_body_mps, air);
    if (flow.dynamic_pressure <= 0.0) return result;
    const Vec3 normal =
        grid_fin_tangential(fin) * std::cos(deflection_rad) + Vec3{1.0, 0.0, 0.0} * std::sin(deflection_rad);
    const double alpha = std::asin(std::clamp(dot(flow.direction, normal), -1.0, 1.0));
    const double limited = std::copysign(std::min(std::abs(alpha), table.limit_rad(flow.mach)), alpha);
    const double coefficient = table.normal_force_slope_per_rad(flow.mach) * std::sin(limited);
    result.force_body_n = normal * (-flow.dynamic_pressure * fin.area_m2 * coefficient);
    return result;
}

double grid_fin_effectiveness(const GridFinGeometry& fin, const Vec3& fin_velocity_body_mps, const FinAirData& air,
                              const GridFinTable& table) {
    const LocalFlow flow = local_flow(fin_velocity_body_mps, air);
    if (flow.dynamic_pressure <= 0.0) return 0.0;
    // alpha ~ u.t + d (u.e_x)  =>  dF_t/dd = -q S slope (u.e_x)
    return -flow.dynamic_pressure * fin.area_m2 * table.normal_force_slope_per_rad(flow.mach) * flow.direction.x;
}

}  // namespace starsim::aero
