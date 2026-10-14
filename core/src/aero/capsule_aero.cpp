#include "starsim/aero/capsule_aero.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::aero {

using math::Vec3;

namespace {

constexpr double max_trim_theta_rad = 60.0 * constants::deg_to_rad;
constexpr double trim_scan_step_rad = 0.25 * constants::deg_to_rad;

// Index i and weight w with x between grid[i] and grid[i + 1] (clamped at the ends).
void bracket(const std::vector<double>& grid, double x, size_t& i, double& w) {
    if (grid.size() < 2 || x <= grid.front()) {
        i = 0;
        w = 0.0;
        return;
    }
    if (x >= grid.back()) {
        i = grid.size() - 2;
        w = 1.0;
        return;
    }
    i = static_cast<size_t>(std::upper_bound(grid.begin(), grid.end(), x) - grid.begin()) - 1;
    w = (x - grid[i]) / (grid[i + 1] - grid[i]);
}

double bilinear(const std::vector<std::vector<double>>& v, size_t i, double wi, size_t j, double wj) {
    const size_t i1 = std::min(i + 1, v.size() - 1);
    const size_t j1 = std::min(j + 1, v[i].size() - 1);
    const double a = v[i][j] + wj * (v[i][j1] - v[i][j]);
    const double b = v[i1][j] + wj * (v[i1][j1] - v[i1][j]);
    return a + wi * (b - a);
}

// Unit air velocity at theta in the plane of rho.
Vec3 velocity_at(double theta_rad, const Vec3& rho) {
    return Vec3{-std::cos(theta_rad), 0.0, 0.0} + rho * std::sin(theta_rad);
}

Vec3 radial_of(const Vec3& cg_body_m) {
    const Vec3 o{0.0, cg_body_m.y, cg_body_m.z};
    return norm(o) < 1e-12 ? Vec3{0.0, 1.0, 0.0} : math::normalized(o);
}

}  // namespace

CapsuleAeroTable load_capsule_aero_table(const std::filesystem::path& csv) {
    const auto t = config::load_csv_table(csv);
    const auto mach = t.column("mach");
    const auto theta_deg = t.column("alpha_deg");
    const auto ca = t.column("ca");
    const auto cn = t.column("cn");
    const auto cm = t.column("cm");
    CapsuleAeroTable table;
    for (size_t r = 0; r < mach.size(); ++r) {
        if (table.mach.empty() || mach[r] != table.mach.back()) {
            if (!table.mach.empty() && mach[r] <= table.mach.back()) {
                throw config::ConfigError(csv.string() + ": Mach blocks must ascend");
            }
            table.mach.push_back(mach[r]);
            table.ca.emplace_back();
            table.cn.emplace_back();
            table.cm.emplace_back();
        }
        const size_t block = table.mach.size() - 1;
        const size_t j = table.ca[block].size();
        const double theta = theta_deg[r] * constants::deg_to_rad;
        if (block == 0) {
            if (j > 0 && theta <= table.theta_rad.back()) throw config::ConfigError(csv.string() + ": theta must ascend");
            table.theta_rad.push_back(theta);
        } else if (j >= table.theta_rad.size() || std::abs(theta - table.theta_rad[j]) > 1e-12) {
            throw config::ConfigError(csv.string() + ": every Mach block needs the same theta grid");
        }
        table.ca[block].push_back(ca[r]);
        table.cn[block].push_back(cn[r]);
        table.cm[block].push_back(cm[r]);
    }
    for (const auto& row : table.ca) {
        if (row.size() != table.theta_rad.size()) throw config::ConfigError(csv.string() + ": ragged theta grid");
    }
    if (table.mach.empty() || table.theta_rad.size() < 2) throw config::ConfigError(csv.string() + ": empty table");
    return table;
}

CapsuleAero::CapsuleAero(CapsuleAeroTable table, double reference_diameter_m, double apex_station_m)
    : table_(std::move(table)),
      diameter_m_(reference_diameter_m),
      apex_station_m_(apex_station_m),
      reference_area_m2_(constants::pi * reference_diameter_m * reference_diameter_m / 4.0) {}

CapsuleAero::Coefficients CapsuleAero::coefficients(double mach, double theta_rad) const {
    size_t i = 0, j = 0;
    double wi = 0.0, wj = 0.0;
    bracket(table_.mach, mach, i, wi);
    bracket(table_.theta_rad, theta_rad, j, wj);
    return {bilinear(table_.ca, i, wi, j, wj), bilinear(table_.cn, i, wi, j, wj), bilinear(table_.cm, i, wi, j, wj)};
}

AeroResult CapsuleAero::compute(const Vec3& u, double density_kg_m3, double speed_of_sound_mps) const {
    AeroResult r;
    r.normal_force_station_m = apex_station_m_;
    const double speed = norm(u);
    if (speed < 1e-6) return r;
    const Vec3 x{1.0, 0.0, 0.0};
    const Vec3 radial = u - x * u.x;
    const Vec3 rho = norm(radial) > 1e-9 * speed ? radial / norm(radial) : Vec3{0.0, 1.0, 0.0};
    const double theta = std::acos(std::clamp(-u.x / speed, -1.0, 1.0));
    const double mach = speed_of_sound_mps > 0.0 ? speed / speed_of_sound_mps : 0.0;
    const double q = 0.5 * density_kg_m3 * speed * speed;
    const auto c = coefficients(mach, theta);
    r.force_body_n = (x * c.ca - rho * c.cn) * (q * reference_area_m2_);
    r.moment_body_nm = cross(x, rho) * (q * reference_area_m2_ * diameter_m_ * c.cm);
    r.mach = mach;
    r.dynamic_pressure_pa = q;
    r.angle_of_attack_rad = theta;
    return r;
}

double CapsuleAero::pitch_moment_about(double mach, double theta_rad, const Vec3& cg_body_m) const {
    const Vec3 x{1.0, 0.0, 0.0};
    const Vec3 rho = radial_of(cg_body_m);
    const auto c = coefficients(mach, theta_rad);
    // per unit q S: force, moment about the apex (/ D) and about the CG
    const Vec3 f = x * c.ca - rho * c.cn;
    const Vec3 m_apex = cross(x, rho) * (diameter_m_ * c.cm);
    const Vec3 arm = cg_body_m - x * apex_station_m_;
    const Vec3 m_cg = m_apex - cross(arm, f);
    return dot(m_cg, cross(x, rho)) / diameter_m_;
}

CapsuleTrim CapsuleAero::trim(double mach, const Vec3& cg_body_m) const {
    CapsuleTrim t;
    double lo = -1.0, hi = -1.0;
    double previous = pitch_moment_about(mach, 0.0, cg_body_m);
    for (double th = trim_scan_step_rad; th <= max_trim_theta_rad + 1e-12; th += trim_scan_step_rad) {
        const double now = pitch_moment_about(mach, th, cg_body_m);
        if (previous > 0.0 && now <= 0.0) {
            lo = th - trim_scan_step_rad;
            hi = th;
            break;
        }
        previous = now;
    }
    if (lo < 0.0) return t;
    for (int k = 0; k < 40; ++k) {
        const double mid = 0.5 * (lo + hi);
        (pitch_moment_about(mach, mid, cg_body_m) > 0.0 ? lo : hi) = mid;
    }
    t.found = true;
    t.theta_rad = 0.5 * (lo + hi);
    const Vec3 rho = radial_of(cg_body_m);
    t.air_velocity_body_unit = velocity_at(t.theta_rad, rho);
    const auto c = coefficients(mach, t.theta_rad);
    const Vec3 f = Vec3{c.ca, 0.0, 0.0} - rho * c.cn;
    t.cd = -dot(f, t.air_velocity_body_unit);
    const Vec3 lift = f + t.air_velocity_body_unit * t.cd;
    t.cl = norm(lift);
    t.lift_to_drag = t.cd > 0.0 ? t.cl / t.cd : 0.0;
    t.lift_body_unit = t.cl > 1e-12 ? lift / t.cl : Vec3{};
    return t;
}

}  // namespace starsim::aero
