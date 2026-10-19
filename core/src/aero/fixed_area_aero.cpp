#include "starsim/aero/fixed_area_aero.hpp"

#include <cmath>

namespace starsim::aero {

AeroResult FixedAreaAero::compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3,
                                  double speed_of_sound_mps) const {
    AeroResult r;
    const double speed = norm(air_velocity_body_mps);
    r.dynamic_pressure_pa = 0.5 * density_kg_m3 * speed * speed;
    r.mach = speed_of_sound_mps > 0.0 ? speed / speed_of_sound_mps : 0.0;
    r.rarefaction = 1.0;
    if (speed <= 0.0) return r;
    // As SlenderBodyAero: air_velocity_body is the body's motion relative to the air (FlightBody passes
    // relative_velocity); the drag opposes it.
    r.force_body_n = air_velocity_body_mps * (-r.dynamic_pressure_pa * drag_coefficient * drag_area_m2 / speed);
    return r;
}

}  // namespace starsim::aero
