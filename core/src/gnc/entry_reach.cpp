#include "starsim/gnc/entry_reach.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

namespace {

// The glide is integrated in ln v from the entry speed down to this fraction of it: below, the range left is ~v^2 and
// adds nothing (the integral's tail beyond 1e-3 is < 1e-5 of it).
constexpr double lowest_speed_fraction = 1.0e-3;
constexpr int steps = 800;                // in ln v (the result changes < 1e-4 from 800 to 3200 steps)
constexpr int mach_iterations = 4;        // glide height <-> Mach (the table's lift changes slowly with Mach)
constexpr double highest_glide_m = 200e3; // search band of the glide height (US76)

// US76 height whose density is `density_kg_m3` (bisection on the monotonic density; clamped to the band).
double height_of_density(const environment::Us76Atmosphere& air, double density_kg_m3) {
    double lo = 0.0, hi = highest_glide_m;
    if (air.at(lo).density_kg_m3 <= density_kg_m3) return lo;
    for (int i = 0; i < 40; ++i) {
        const double mid = 0.5 * (lo + hi);
        (air.at(mid).density_kg_m3 > density_kg_m3 ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

}  // namespace

double entry_crossrange_reach_m(const EntryPredictorModel& model, double bank_rad, double entry_speed_mps) {
    const double cos_bank = std::cos(bank_rad), sin_bank = std::abs(std::sin(bank_rad));
    if (!(cos_bank > 0.0) || model.aero.empty() || !(model.mass_kg > 0.0) || !(entry_speed_mps > 0.0)) return 0.0;
    const double r = model.earth_radius_m;
    const double g = model.gm_m3ps2 / (r * r);
    const double circular_mps = std::sqrt(model.gm_m3ps2 / r);
    const double v0 = std::min(entry_speed_mps, circular_mps);
    const double dx = -std::log(lowest_speed_fraction) / steps;  // step in -ln v
    const environment::Us76Atmosphere air;
    const double sea_level_sound_mps = air.at(0.0).speed_of_sound_mps;
    double psi = 0.0, crossrange = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double v = v0 * std::exp(-(i + 0.5) * dx);
        const double u = v * v / (circular_mps * circular_mps);
        const double lift_per_mass = g * (1.0 - u) / cos_bank;  // equilibrium glide
        double mach = v / sea_level_sound_mps;  // first guess, then the glide height's
        for (int k = 0; k < mach_iterations; ++k) {
            const double lift_area = model.aero.cl_area_m2(mach);
            if (!(lift_area > 0.0)) return 0.0;
            const double density = 2.0 * model.mass_kg * lift_per_mass / (v * v * lift_area);
            mach = v / air.at(height_of_density(air, density)).speed_of_sound_mps;
        }
        const double drag_area = model.aero.cd_area_m2(mach);
        if (!(drag_area > 0.0)) return 0.0;
        const double lift_to_drag = model.aero.cl_area_m2(mach) / drag_area;
        const double dpsi = lift_to_drag * sin_bank * dx;
        const double ds = lift_to_drag * cos_bank * v * v * dx / (g * (1.0 - u));
        crossrange += (psi + 0.5 * dpsi) * ds;
        psi += dpsi;
    }
    return crossrange;
}

double constant_lift_crossrange_reach_m(double lift_to_drag, double bank_rad, double radius_m) {
    return constants::pi * constants::pi / 48.0 * lift_to_drag * lift_to_drag * std::sin(2.0 * std::abs(bank_rad)) *
           radius_m;
}

}  // namespace starsim::gnc
