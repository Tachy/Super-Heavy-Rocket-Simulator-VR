#include "starsim/aero/slender_body_aero.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::aero {

namespace {

constexpr double free_molecular_drag_coefficient = 2.2;  // [E] diffuse reflection, per projected area

double interpolate(const std::vector<double>& x, const std::vector<double>& y, double at) {
    if (at <= x.front()) return y.front();
    if (at >= x.back()) return y.back();
    const auto upper = std::upper_bound(x.begin(), x.end(), at);
    const size_t i = static_cast<size_t>(upper - x.begin());
    const double f = (at - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + f * (y[i] - y[i - 1]);
}

}  // namespace

double rarefaction_weight(double knudsen) {
    if (knudsen <= 0.01) return 0.0;
    if (knudsen >= 10.0) return 1.0;
    const double s = std::sin(constants::pi / 8.0 * (3.0 + std::log10(knudsen)));
    return s * s;
}

AxialDragCurve::AxialDragCurve(const config::CsvTable& table)
    : mach_(table.column("mach")),
      cd_power_on_(table.column("cd_axial_power_on")),
      cd_power_off_(table.column("cd_axial_power_off")) {
    if (!std::is_sorted(mach_.begin(), mach_.end()) || mach_.size() < 2) {
        throw config::ConfigError("drag table: mach column must be ascending with >= 2 rows");
    }
}

AxialDragCurve AxialDragCurve::plus(const std::vector<double>& mach, const std::vector<double>& delta_cd) const {
    if (mach.empty()) return *this;
    std::vector<double> points = mach_;
    points.insert(points.end(), mach.begin(), mach.end());
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    AxialDragCurve sum;
    sum.mach_ = points;
    for (const double m : points) {
        const double delta = interpolate(mach, delta_cd, m);
        sum.cd_power_on_.push_back(interpolate(mach_, cd_power_on_, m) + delta);
        sum.cd_power_off_.push_back(interpolate(mach_, cd_power_off_, m) + delta);
    }
    return sum;
}

CrossflowCurve::CrossflowCurve(const config::CsvTable& table)
    : mach_(table.column("mach")), cd_(table.column("crossflow_cd")) {
    if (!std::is_sorted(mach_.begin(), mach_.end()) || mach_.size() < 2) {
        throw config::ConfigError("cross-flow table: mach column must be ascending with >= 2 rows");
    }
}

double CrossflowCurve::cd(double mach) const { return empty() ? 0.0 : interpolate(mach_, cd_, mach); }

double SlenderBodyAero::continuum_crossflow_cd(double mach) const {
    return crossflow_.empty() ? generic_crossflow_efficiency * generic_crossflow_drag_coefficient : crossflow_.cd(mach);
}

double SlenderBodyAero::potential_station_m() const {
    return geometry_.length_m - ogive_cp_fraction * geometry_.nose_length_m;
}

double AxialDragCurve::cd(double mach, bool powered) const {
    if (empty()) return 0.0;
    return interpolate(mach_, powered ? cd_power_on_ : cd_power_off_, mach);
}

SlenderBodyAero::SlenderBodyAero(SlenderBodyGeometry geometry, AxialDragCurve drag, AxialDragCurve tail_first_drag,
                                 CrossflowCurve crossflow)
    : geometry_(geometry),
      drag_(std::move(drag)),
      tail_first_drag_(std::move(tail_first_drag)),
      crossflow_(std::move(crossflow)),
      reference_area_m2_(constants::pi * 0.25 * geometry.diameter_m * geometry.diameter_m) {}

AeroResult SlenderBodyAero::compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3,
                                    double speed_of_sound_mps, bool powered, double mean_free_path_m) const {
    AeroResult result;
    const double speed = norm(air_velocity_body_mps);
    result.normal_force_station_m = 0.5 * geometry_.length_m;
    if (speed < 1e-6 || density_kg_m3 <= 0.0) {
        return result;
    }
    result.mach = speed_of_sound_mps > 0.0 ? speed / speed_of_sound_mps : 0.0;
    result.dynamic_pressure_pa = 0.5 * density_kg_m3 * speed * speed;
    const double q_area = result.dynamic_pressure_pa * reference_area_m2_;
    result.knudsen = mean_free_path_m / geometry_.length_m;
    result.rarefaction = rarefaction_weight(result.knudsen);
    const double w = result.rarefaction;

    // Oncoming flow relative to the body is -air_velocity. Angle of attack between +x and the motion.
    const double axial_speed = air_velocity_body_mps.x;
    const math::Vec3 cross_velocity{0.0, air_velocity_body_mps.y, air_velocity_body_mps.z};
    const double cross_speed = norm(cross_velocity);
    const double alpha = std::atan2(cross_speed, axial_speed);  // 0 = nose first, pi = tail first
    result.angle_of_attack_rad = alpha;

    // Axial force opposes the axial component of motion.
    const double cos_a = std::cos(alpha);
    const double cd = drag_curve(axial_speed < 0.0).cd(result.mach, powered);
    const double c_axial = ((1.0 - w) * cd + w * free_molecular_drag_coefficient) * cos_a * cos_a;
    const double axial_force = -std::copysign(c_axial * q_area, axial_speed);

    // Normal force opposes the cross-flow velocity.
    const double sin_a = std::sin(alpha);
    const double potential =
        (1.0 - w) * (geometry_.nose_length_m > 0.0 ? std::abs(std::sin(2.0 * alpha) * std::cos(0.5 * alpha)) : 0.0);
    const double planform_ratio = geometry_.length_m * geometry_.diameter_m / reference_area_m2_;
    const double crossflow = (1.0 - w) * continuum_crossflow_cd(result.mach) + w * free_molecular_drag_coefficient;
    const double viscous = crossflow * planform_ratio * sin_a * sin_a;
    const double normal_force = (potential + viscous) * q_area;

    result.force_body_n = {axial_force, 0.0, 0.0};
    if (cross_speed > 1e-9) {
        result.force_body_n += cross_velocity * (-normal_force / cross_speed);
    }
    // Centre of pressure of the normal force: weighted potential (nose) and viscous (mid-length) parts.
    const double nose_station = potential_station_m();
    const double total = potential + viscous;
    if (total > 0.0) {
        result.normal_force_station_m = (potential * nose_station + viscous * 0.5 * geometry_.length_m) / total;
    }
    return result;
}

}  // namespace starsim::aero
