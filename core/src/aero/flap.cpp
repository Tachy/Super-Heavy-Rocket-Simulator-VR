#include "starsim/aero/flap.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::aero {

using math::Vec3;

namespace {

constexpr double free_molecular_normal_coefficient = 2.2;  // [E] diffuse reflection, as SlenderBodyAero
constexpr int shadow_edge_iterations = 20;                 // bisection: 90 deg / 2^20

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

Vec3 flap_hinge_direction(const FlapGeometry& flap) {
    return Vec3{std::cos(flap.hinge_lean_rad), 0.0, 0.0} - radial(flap.hinge_azimuth_rad) * std::sin(flap.hinge_lean_rad);
}

Vec3 flap_span_direction(const FlapGeometry& flap, double deflection_rad) {
    const Vec3 e_r = radial(flap.hinge_azimuth_rad);
    Vec3 stow = cross(Vec3{1.0, 0.0, 0.0}, e_r);  // hull tangent at the hinge
    if (dot(stow, radial(flap.belly_azimuth_rad)) > 0.0) stow = stow * -1.0;
    const Vec3 broadside = Vec3{std::sin(flap.hinge_lean_rad), 0.0, 0.0} + e_r * std::cos(flap.hinge_lean_rad);
    return stow * std::cos(deflection_rad) + broadside * std::sin(deflection_rad);
}

Vec3 flap_normal(const FlapGeometry& flap, double deflection_rad) {
    return cross(flap_hinge_direction(flap), flap_span_direction(flap, deflection_rad));
}

Vec3 flap_centroid(const FlapGeometry& flap, double deflection_rad) {
    const double hinge_length_m = (flap.station_fwd_m - flap.station_aft_m) / std::cos(flap.hinge_lean_rad);
    const double u_m = flap.centroid_u_m.value_or(0.5 * hinge_length_m);
    const double v_m = flap.centroid_v_m.value_or(0.5 * flap.span_m);
    return Vec3{flap.station_aft_m, 0.0, 0.0} +
           radial(flap.hinge_azimuth_rad) * flap.hinge_radius_aft_m.value_or(flap.hull_radius_m) +
           flap_hinge_direction(flap) * u_m + flap_span_direction(flap, deflection_rad) * v_m;
}

double flap_exposed_fraction(const FlapGeometry& flap, double deflection_rad, const Vec3& velocity_body_mps) {
    const double cross_speed = std::hypot(velocity_body_mps.y, velocity_body_mps.z);
    if (cross_speed <= 0.0) return 1.0;
    const double upstream_y = velocity_body_mps.y / cross_speed, upstream_z = velocity_body_mps.z / cross_speed;
    const Vec3 c = flap_centroid(flap, deflection_rad);
    const double upstream_m = c.y * upstream_y + c.z * upstream_z;
    const double upstream_station_shift_m = std::max(-upstream_m, 0.0) * velocity_body_mps.x / cross_speed;
    const double hull_radius_m = std::max(flap.centroid_hull_radius_m.value_or(flap.hull_radius_m) +
                                              flap.centroid_hull_taper * upstream_station_shift_m, 0.0);
    const double beyond_edge_m = std::abs(c.y * upstream_z - c.z * upstream_y) - hull_radius_m;
    return std::clamp(std::max(beyond_edge_m, upstream_m) / (0.5 * flap.span_m), 0.0, 1.0);
}

double flap_edge_on_deflection_rad(const FlapGeometry& flap, const Vec3& velocity_body_mps) {
    // n(d) . v = a cos(d) + b sin(d) with a = n(0) . v, b = n(90 deg) . v.
    const double a = dot(flap_normal(flap, 0.0), velocity_body_mps);
    const double b = dot(flap_normal(flap, 0.5 * constants::pi), velocity_body_mps);
    if (a == 0.0 && b == 0.0) return 0.0;
    double d = std::atan2(-a, b);
    if (d < 0.0) d += constants::pi;
    return d <= 0.5 * constants::pi ? d : 0.0;
}

double flap_lowest_usable_deflection_rad(const FlapGeometry& flap, double max_deflection_rad, const Vec3& velocity_body_mps) {
    double lo = std::min(flap_edge_on_deflection_rad(flap, velocity_body_mps), max_deflection_rad);
    if (!flap_in_hull_shadow(flap, lo, velocity_body_mps)) return lo;
    double hi = max_deflection_rad;
    if (flap_in_hull_shadow(flap, hi, velocity_body_mps)) return 0.0;
    for (int i = 0; i < shadow_edge_iterations; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (flap_in_hull_shadow(flap, mid, velocity_body_mps)) lo = mid; else hi = mid;
    }
    return hi;
}

FlapForce flap_force(const FlapGeometry& flap, double deflection_rad, const Vec3& velocity_body_mps,
                     const FinAirData& air, const FlapTable& table) {
    FlapForce out;
    out.application_point_m = flap_centroid(flap, deflection_rad);
    const double speed = norm(velocity_body_mps);
    if (speed < 1e-6 || air.density_kg_m3 <= 0.0) return out;
    const double exposed = flap_exposed_fraction(flap, deflection_rad, velocity_body_mps);
    if (exposed <= 0.0) return out;
    const Vec3 n = flap_normal(flap, deflection_rad);
    const double normal_speed = dot(velocity_body_mps, n);
    const double sin_theta = std::abs(normal_speed) / speed;
    const double mach = air.speed_of_sound_mps > 0.0 ? speed / air.speed_of_sound_mps : 0.0;
    const double w = air.rarefaction;
    const double cn =
        ((1.0 - w) * table.cn_max(mach) + w * free_molecular_normal_coefficient) * sin_theta * sin_theta;
    const double q = 0.5 * air.density_kg_m3 * speed * speed;
    out.force_body_n = n * (-std::copysign(exposed * cn * q * flap.area_m2, normal_speed));
    return out;
}

}  // namespace starsim::aero
