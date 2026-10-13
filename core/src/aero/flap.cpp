#include "starsim/aero/flap.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::aero {

using math::Vec3;

namespace {

constexpr double free_molecular_normal_coefficient = 2.2;  // [E] diffuse reflection, as SlenderBodyAero

Vec3 radial(double azimuth_rad) { return {0.0, std::cos(azimuth_rad), std::sin(azimuth_rad)}; }

double interpolate(const std::vector<double>& x, const std::vector<double>& y, double at) {
    if (at <= x.front()) return y.front();
    if (at >= x.back()) return y.back();
    const auto upper = std::upper_bound(x.begin(), x.end(), at);
    const size_t i = static_cast<size_t>(upper - x.begin());
    const double f = (at - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + f * (y[i] - y[i - 1]);
}

}  // namespace

FlapTable::FlapTable(const config::CsvTable& table) : mach_(table.column("mach")), cn_max_(table.column("cn_max")) {
    if (!std::is_sorted(mach_.begin(), mach_.end()) || mach_.size() < 2) {
        throw config::ConfigError("flap table: mach column must be ascending with >= 2 rows");
    }
}

double FlapTable::cn_max(double mach) const { return empty() ? 0.0 : interpolate(mach_, cn_max_, mach); }

Vec3 flap_span_direction(const FlapGeometry& flap, double deflection_rad) {
    const Vec3 e_r = radial(flap.hinge_azimuth_rad);
    Vec3 stow = cross(Vec3{1.0, 0.0, 0.0}, e_r);  // hull tangent at the hinge
    if (dot(stow, radial(flap.belly_azimuth_rad)) > 0.0) stow = stow * -1.0;
    return stow * std::cos(deflection_rad) + e_r * std::sin(deflection_rad);
}

Vec3 flap_normal(const FlapGeometry& flap, double deflection_rad) {
    return cross(Vec3{1.0, 0.0, 0.0}, flap_span_direction(flap, deflection_rad));
}

Vec3 flap_centroid(const FlapGeometry& flap, double deflection_rad) {
    return Vec3{0.5 * (flap.station_fwd_m + flap.station_aft_m), 0.0, 0.0} +
           radial(flap.hinge_azimuth_rad) * flap.hull_radius_m + flap_span_direction(flap, deflection_rad) * (0.5 * flap.span_m);
}

FlapForce flap_force(const FlapGeometry& flap, double deflection_rad, const Vec3& velocity_body_mps,
                     const FinAirData& air, const FlapTable& table) {
    FlapForce out;
    out.application_point_m = flap_centroid(flap, deflection_rad);
    const double speed = norm(velocity_body_mps);
    if (speed < 1e-6 || air.density_kg_m3 <= 0.0) return out;
    if (dot(velocity_body_mps, radial(flap.belly_azimuth_rad)) <= 0.0) return out;  // in the hull's lee
    const Vec3 n = flap_normal(flap, deflection_rad);
    const double normal_speed = dot(velocity_body_mps, n);
    const double sin_theta = std::abs(normal_speed) / speed;
    const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
    const double w = air.rarefaction;
    const double cn =
        ((1.0 - w) * table.cn_max(mach) + w * free_molecular_normal_coefficient) * sin_theta * sin_theta;
    const double q = 0.5 * air.density_kg_m3 * speed * speed;
    out.force_body_n = n * (-std::copysign(cn * q * flap.area_m2, normal_speed));
    return out;
}

}  // namespace starsim::aero
