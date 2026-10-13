#pragma once

#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Where the entry should end: the gate lies `gate_height_m` (guidance parameter) above this ground point.
struct EntryTarget {
    math::Vec3 position_itrf_m;    // gate point projected to the ground
    double ground_height_m = 0.0;  // WGS84 height of that ground
};

// The onboard computer's point-mass model of the entering ship.
struct EntryPredictorModel {
    EntryAeroTable aero;
    double mass_kg = 0.0;
    double nose_radius_m = 0.0;   // Sutton-Graves (heat rate prediction); 0: no heating predicted
    double drag_scale = 1.0;      // online corrections (measured / modelled aerodynamic acceleration)
    double lift_scale = 1.0;
    double bank_rate_radps = 1.0 * constants::deg_to_rad;
    double gm_m3ps2 = constants::earth_gm_m3ps2;
    double j2 = constants::earth_j2;
    double earth_radius_m = constants::wgs84_semi_major_axis_m;
    double omega_radps = constants::earth_rotation_rate_radps;  // about ITRF z
};

struct EntryPredictorState {
    math::Vec3 position_itrf_m;
    math::Vec3 velocity_itrf_mps;  // relative to the rotating Earth
    double bank_rad = 0.0;         // current bank (signed, entry_attitude convention)
};

struct EntryPrediction {
    math::Vec3 end_position_itrf_m;
    math::Vec3 end_velocity_itrf_mps;
    double time_s = 0.0;
    double downrange_error_m = 0.0;  // range start -> end minus range start -> target; > 0: overshoot
    double crossrange_m = 0.0;       // target's distance from the great circle start -> end; > 0: target to the right
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

// Great-circle ground range between the directions of two Earth-fixed points (sphere of the WGS84 semi-major axis).
double ground_range_m(const math::Vec3& a_itrf_m, const math::Vec3& b_itrf_m);

}  // namespace starsim::gnc
