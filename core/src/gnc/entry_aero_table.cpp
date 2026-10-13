#include "starsim/gnc/entry_aero_table.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/aero/flap.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

double interpolate(const std::vector<double>& x, const std::vector<double>& y, double at) {
    if (at <= x.front()) return y.front();
    if (at >= x.back()) return y.back();
    const size_t i = static_cast<size_t>(std::upper_bound(x.begin(), x.end(), at) - x.begin());
    const double f = (at - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + f * (y[i] - y[i - 1]);
}

// Reference air for the coefficients (continuum; only ratios to the dynamic pressure are kept).
constexpr double reference_density_kg_m3 = 1.0e-3;
constexpr double reference_speed_of_sound_mps = 300.0;
constexpr double alpha_grid_min_deg = 40.0;  // terminal steering table
constexpr double alpha_grid_max_deg = 140.0;
constexpr double alpha_grid_step_deg = 5.0;
const std::vector<double> mach_grid{0.3, 0.5, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5, 2.0, 3.0,
                                    4.0, 6.0, 8.0, 10.0, 15.0, 20.0, 25.0, 30.0};

struct FlapSetting {
    const vehicle::Flap* flap;
    bool forward;  // forward pair (centroid ahead of the CG)
};

}  // namespace

double AlphaSchedule::at(double mach_number) const { return interpolate(mach, alpha_rad, mach_number); }

AlphaSchedule load_alpha_schedule(const std::filesystem::path& config_dir, const std::string& table) {
    const auto csv = config::load_csv_table(config_dir / table);
    AlphaSchedule s;
    s.mach = csv.column("mach");
    for (double a : csv.column("alpha_deg")) s.alpha_rad.push_back(a * constants::deg_to_rad);
    return s;
}

EntryAeroTable::EntryAeroTable(const vehicle::Vehicle& ship, const AlphaSchedule& schedule) : schedule_(schedule) {
    const auto& stage = ship.stages().front();
    const double cg = ship.mass_properties().cg_station_m;
    const double belly_azimuth = stage.config.aero.belly_azimuth_rad;
    const Vec3 belly{0.0, std::cos(belly_azimuth), std::sin(belly_azimuth)};
    const Vec3 pitch_axis = cross(Vec3{1.0, 0.0, 0.0}, belly);  // nose-up about this axis turns the nose to the lee
    std::vector<FlapSetting> flaps;
    for (const auto& f : stage.flaps) {
        const double centroid = 0.5 * (f.geometry.station_fwd_m + f.geometry.station_aft_m);
        flaps.push_back({&f, centroid > cg});
    }

    // Total force (body axes) at Mach number and angle of attack (belly towards the flow), flaps in pitch trim.
    const auto trimmed_force = [&](double mach, double alpha) {
        const double speed = mach * reference_speed_of_sound_mps;
        const Vec3 u = Vec3{std::cos(alpha), 0.0, 0.0} + belly * std::sin(alpha);  // air velocity direction
        const Vec3 v_body = u * speed;
        const auto hull = ship.aero().compute(v_body, reference_density_kg_m3, reference_speed_of_sound_mps, false, 0.0);
        const aero::FinAirData air{reference_density_kg_m3, reference_speed_of_sound_mps, 0.0};

        // Force and pitch moment with the forward pair at mid + delta, the aft pair at mid - delta.
        const auto evaluate = [&](double delta, Vec3& force) {
            force = hull.force_body_n;
            Vec3 moment = cross(Vec3{hull.normal_force_station_m - cg, 0.0, 0.0}, hull.force_body_n);
            if (stage.flap_table) {
                for (const auto& s : flaps) {
                    const double max = s.flap->max_deflection_rad;
                    const double d = std::clamp(0.5 * max + (s.forward ? delta : -delta), 0.0, max);
                    const auto f = aero::flap_force(s.flap->geometry, d, v_body, air, *stage.flap_table);
                    force += f.force_body_n;
                    moment += cross(f.application_point_m - Vec3{cg, 0.0, 0.0}, f.force_body_n);
                }
            }
            return dot(moment, pitch_axis);
        };
        const double half_range = constants::pi / 2.0;
        Vec3 force;
        double lo = -half_range, hi = half_range;
        const double m_lo = evaluate(lo, force), m_hi = evaluate(hi, force);
        double delta = std::abs(m_lo) < std::abs(m_hi) ? lo : hi;
        if ((m_lo < 0.0) != (m_hi < 0.0)) {
            for (int i = 0; i < 50; ++i) {
                const double mid = 0.5 * (lo + hi);
                if ((evaluate(mid, force) < 0.0) == (m_lo < 0.0)) lo = mid; else hi = mid;
            }
            delta = 0.5 * (lo + hi);
        }
        evaluate(delta, force);
        return force / (0.5 * reference_density_kg_m3 * speed * speed);
    };

    for (double mach : mach_grid) {
        const double alpha = schedule.at(mach);
        const Vec3 u = Vec3{std::cos(alpha), 0.0, 0.0} + belly * std::sin(alpha);
        const Vec3 force = trimmed_force(mach, alpha);  // per dynamic pressure
        const Vec3 x{1.0, 0.0, 0.0};
        const Vec3 l = math::normalized(x - u * dot(x, u));  // lift direction in the alpha plane
        mach_.push_back(mach);
        cd_area_m2_.push_back(-dot(force, u));
        cl_area_m2_.push_back(dot(force, l));
    }
    for (double a = alpha_grid_min_deg; a <= alpha_grid_max_deg + 1e-9; a += alpha_grid_step_deg) {
        alpha_grid_rad_.push_back(a * constants::deg_to_rad);
    }
    for (double mach : mach_grid) {
        std::vector<double> cd, cl;
        for (double alpha : alpha_grid_rad_) {
            const Vec3 force = trimmed_force(mach, alpha);
            const double fx = force.x, fb = dot(force, belly);
            // air velocity u = cos(a) x + sin(a) b; lift direction l = sin(a) x - cos(a) b
            cd.push_back(-(fx * std::cos(alpha) + fb * std::sin(alpha)));
            cl.push_back(fx * std::sin(alpha) - fb * std::cos(alpha));
        }
        cd_grid_m2_.push_back(cd);
        cl_grid_m2_.push_back(cl);
    }
}

double EntryAeroTable::bilinear(const std::vector<std::vector<double>>& grid, double mach, double alpha_rad) const {
    if (mach <= mach_.front()) return interpolate(alpha_grid_rad_, grid.front(), alpha_rad);
    if (mach >= mach_.back()) return interpolate(alpha_grid_rad_, grid.back(), alpha_rad);
    const size_t i = static_cast<size_t>(std::upper_bound(mach_.begin(), mach_.end(), mach) - mach_.begin());
    const double f = (mach - mach_[i - 1]) / (mach_[i] - mach_[i - 1]);
    return (1.0 - f) * interpolate(alpha_grid_rad_, grid[i - 1], alpha_rad) + f * interpolate(alpha_grid_rad_, grid[i], alpha_rad);
}
double EntryAeroTable::cd_area_m2(double mach, double alpha_rad) const { return bilinear(cd_grid_m2_, mach, alpha_rad); }
double EntryAeroTable::cl_area_m2(double mach, double alpha_rad) const { return bilinear(cl_grid_m2_, mach, alpha_rad); }

double EntryAeroTable::cd_area_m2(double mach) const { return interpolate(mach_, cd_area_m2_, mach); }
double EntryAeroTable::cl_area_m2(double mach) const { return interpolate(mach_, cl_area_m2_, mach); }

}  // namespace starsim::gnc
