#pragma once

#include <algorithm>

#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Where the entry should end: the gate lies `gate_height_m` (guidance parameter) above this ground point.
struct EntryTarget {
    math::Vec3 position_itrf_m;    // gate point projected to the ground
    double ground_height_m = 0.0;  // WGS84 height of that ground
    // Horizontal unit vector the nose should point along at the gate (a tower catch: to the tower), zero: free. The
    // terminal phase turns the nose to it once the fall is steep (EntryPredictorModel::nose_*).
    math::Vec3 nose_direction_itrf;
};

// The onboard computer's point-mass model of the entering ship.
struct EntryPredictorModel {
    EntryAeroTable aero;
    double mass_kg = 0.0;
    double nose_radius_m = 0.0;   // Sutton-Graves (heat rate prediction); 0: no heating predicted
    double drag_scale = 1.0;      // online corrections (measured / modelled aerodynamic acceleration)
    double lift_scale = 1.0;
    // Bank rate limit over Mach: `bank_rate_radps` above `fast_bank_mach_high`, `fast_bank_rate_radps` below
    // `fast_bank_mach_low`, linear in between (see EntryGuidanceParameters).
    double bank_rate_radps = 1.0 * constants::deg_to_rad;
    double fast_bank_rate_radps = 1.0 * constants::deg_to_rad;
    double fast_bank_mach_low = 5.0;
    double fast_bank_mach_high = 10.0;
    double bank_rate_at(double mach) const {
        const double f = std::clamp((fast_bank_mach_high - mach) / (fast_bank_mach_high - fast_bank_mach_low), 0.0, 1.0);
        return bank_rate_radps + f * (fast_bank_rate_radps - bank_rate_radps);
    }
    // Terminal steering (EntryPredictorState::steered): angle of attack from the steering fraction f in [0, 1],
    // piecewise linear and continuous: 0 -> 90 deg (no lift, so the steering vector is continuous through zero),
    // 0.5 -> the entry's alpha schedule (the entry's own prediction), 1 -> `terminal_alpha_authority_rad` below it
    // (most lift, at least the minimum alpha).
    double terminal_min_alpha_rad = 50.0 * constants::deg_to_rad;          // [E] trimmed range (b1)
    double terminal_alpha_authority_rad = 20.0 * constants::deg_to_rad;    // [E]
    double steering_freeze_gamma_rad = -70.0 * constants::deg_to_rad;      // [E]
    // With a nose direction in the target: when the basis freezes the steering turns into the nose sector (the nearest
    // edge) at nose_fraction (alpha ~86 deg: little lift, the nose turns about the air velocity; room to steer either
    // way); after that it stays within the sector about the nose direction and between nose_min_fraction and
    // nose_max_fraction (at most the belly flop's alpha: more lift pointed back to the tower in a -70 deg fall points
    // down and pulls the ship out of trim) - nose_sector_steering [E]. The sector narrows from nose_sector_wide_rad
    // (nose_sector_wide_height_m and more above the gate: crossrange authority) to nose_sector_rad at the gate (the flip
    // must push along the tower line) [E].
    double nose_fraction = 0.2;
    double nose_sector_rad = 10.0 * constants::deg_to_rad;
    double nose_sector_wide_rad = 30.0 * constants::deg_to_rad;
    double nose_sector_wide_height_m = 4000.0;
    // The lift direction turns into the sector about the air velocity at this rate, the fraction held at nose_fraction
    // until it is within nose_turned_rad (as the guidance flies it: an instant turn left the prediction blind to the
    // lift swept during the up to ~25 s turn - gate overshoots of 0.4-0.8 km, b4 campaign).
    double nose_turn_rate_radps = 3.0 * constants::deg_to_rad;
    double nose_turned_rad = 5.0 * constants::deg_to_rad;
    double nose_sector_at(double height_above_gate_m) const {
        const double f = std::clamp(height_above_gate_m / nose_sector_wide_height_m, 0.0, 1.0);
        return nose_sector_rad + f * (nose_sector_wide_rad - nose_sector_rad);
    }
    double nose_min_fraction = 0.02;
    double nose_max_fraction = 0.5;
    double steering_alpha(double fraction, double mach) const {
        const double f = std::clamp(fraction, 0.0, 1.0);
        const double nominal = std::min(aero.alpha_rad(mach), 0.5 * constants::pi);
        const double most_lift = std::max(nominal - terminal_alpha_authority_rad, terminal_min_alpha_rad);
        return f <= 0.5 ? 0.5 * constants::pi + (nominal - 0.5 * constants::pi) * (f / 0.5)
                        : nominal + (most_lift - nominal) * ((f - 0.5) / 0.5);
    }
    double steering_fraction(double alpha_rad, double mach) const {
        const double nominal = std::min(aero.alpha_rad(mach), 0.5 * constants::pi);
        const double most_lift = std::max(nominal - terminal_alpha_authority_rad, terminal_min_alpha_rad);
        if (alpha_rad >= nominal) {
            const double span = 0.5 * constants::pi - nominal;
            return span > 1e-9 ? std::clamp(0.5 * (0.5 * constants::pi - alpha_rad) / span, 0.0, 0.5) : 0.5;
        }
        const double span = nominal - most_lift;
        return span > 1e-9 ? std::clamp(0.5 + 0.5 * (nominal - alpha_rad) / span, 0.5, 1.0) : 0.5;
    }
    double gm_m3ps2 = constants::earth_gm_m3ps2;
    double j2 = constants::earth_j2;
    double earth_radius_m = constants::wgs84_semi_major_axis_m;
    double omega_radps = constants::earth_rotation_rate_radps;  // about ITRF z
};

struct EntryPredictorState {
    math::Vec3 position_itrf_m;
    math::Vec3 velocity_itrf_mps;  // relative to the rotating Earth
    double bank_rad = 0.0;         // current bank (signed, entry_attitude convention)
    // Terminal steering instead of the bank: the lift in the velocity frame (x: towards "up" perpendicular to the
    // air velocity, y: to the side, see steering_basis; z unused) times the steering fraction (|.| <= 1), held to the
    // gate like a bank; coefficients at that fraction's alpha.
    bool steered = false;
    math::Vec3 steering;
    // The steering basis follows the velocity (like a bank) until the flight path is steeper than
    // `steering_freeze_gamma_rad`, then it stays fixed in the Earth frame (a near vertical fall has no defined "up
    // perpendicular to the velocity"). A start already past that carries its frozen basis.
    bool basis_frozen = false;
    math::Vec3 basis_e1, basis_e2;
    // Frozen basis with a nose direction: the lift direction flown now (unit, along e1 / e2; zero: the steering's).
    math::Vec3 lift_direction;
    double reversal_time_s = -1.0; // the bank command changes sign at this time from the start (< 0: never)
};

struct EntryPrediction {
    math::Vec3 end_position_itrf_m;
    math::Vec3 end_velocity_itrf_mps;
    double time_s = 0.0;
    // Miss: > 0 overshoot / target to the right. Start farther than 50 km from the target: range start -> end minus
    // range start -> target, and the target's distance from the great circle start -> end; closer: the horizontal
    // miss at the end along / across the track there.
    double downrange_error_m = 0.0;
    double crossrange_m = 0.0;
    double peak_heat_rate_w_m2 = 0.0;
    double heat_load_j_m2 = 0.0;     // including the load so far
    double peak_load_g = 0.0;        // aerodynamic acceleration / g0
    double peak_dynamic_pressure_pa = 0.0;
    bool reached_gate = false;       // false: skipped out (above 130 km after dipping below 80 km) or time limit
};

// Integrates the entry of a point mass from `start` to the gate height above the target ground (classical RK4,
// `step_s`), in the Earth-fixed frame: central gravity + J2, Coriolis and centrifugal terms, US 1976 air, constant
// mass. Drag along -v and lift perpendicular to v, rotated about v by the bank from the local vertical plane; the
// bank moves from start.bank_rad to `bank_command_rad` at the model's bank rate limit. Pure (no SPICE): worker-safe.
EntryPrediction predict_entry(const EntryPredictorModel& model, const EntryPredictorState& start,
                              double bank_command_rad, const EntryTarget& target, double gate_height_m,
                              double heat_load_so_far_j_m2 = 0.0, double step_s = 1.0, double max_time_s = 3000.0);

// Steering basis perpendicular to the air velocity u (unit): e1 = the local vertical's component perpendicular to u
// (north's when the fall is nearly vertical), e2 = u x e1. A bank sigma (entry_attitude) is the lift
// e1 cos(sigma) - e2 sin(sigma).
void steering_basis(const math::Vec3& up, const math::Vec3& u, math::Vec3& e1, math::Vec3& e2);

// The steering vector (components along e1, e2) of the horizontal direction `nose`, `fraction` long.
math::Vec3 nose_steering(const math::Vec3& nose, const math::Vec3& e1, const math::Vec3& e2, double fraction);

// Where the nose turn ends when the basis freezes: the direction of p turned into the sector (its nearest edge), at
// `fraction` (the nose direction itself when p is zero).
math::Vec3 nose_turn_steering(const math::Vec3& p, const math::Vec3& nose, const math::Vec3& e1, const math::Vec3& e2,
                              double sector_rad, double fraction);

// The steering vector p (components along e1, e2) turned into the sector of +-sector_rad about the horizontal
// direction `nose`, its length clamped to [min_fraction, max_fraction].
math::Vec3 nose_sector_steering(const math::Vec3& p, const math::Vec3& nose, const math::Vec3& e1, const math::Vec3& e2,
                                double sector_rad, double min_fraction, double max_fraction);

// Great-circle ground range between the directions of two Earth-fixed points (sphere of the WGS84 semi-major axis).
double ground_range_m(const math::Vec3& a_itrf_m, const math::Vec3& b_itrf_m);

}  // namespace starsim::gnc
