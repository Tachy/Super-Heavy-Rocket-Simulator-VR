#pragma once

#include <cstddef>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/thermal_radiation.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

// Refuelling r2: propellant boil-off from a quasi-steady heat balance on the hull. Every hull tank of a stage with
// [stage.thermal] (a main tank of the stage's diameter) is a cylinder of its full-load length, split into
// boil_off_strips strips around the circumference. Per strip: absorbed (Sun, Earth albedo, Earth infrared) =
// emissivity sigma T^4 + (T - T_boil) / R_wall. The leaks (T - T_boil) / R_wall over the tank's strips are summed;
// that heat (clamped at 0: a tank losing heat through its wall does not condense) over the latent heat is the vented
// mass flow.
// Deliberate simplifications [docs/subsystems.md]: no skin heat capacity and no conduction around the circumference;
// no tank pressure, temperature, subcooling or stratification; a tank's whole wall heat reaches its liquid whatever
// the fill level; domes, ends, header and RCS tanks take no heat; no shadows of other vehicles or of the flaps.
inline constexpr int boil_off_strips = 36;

struct BoilOffRate {
    double oxidizer_kgps = 0.0;
    double fuel_kgps = 0.0;
};

struct TankBoilOff {
    size_t stage_index = 0;
    size_t tank_index = 0;
    double heat_w = 0.0;          // into the propellant (>= 0)
    double mass_flow_kgps = 0.0;  // vented
};

// A cylinder section of the hull along the body x axis.
struct HullSection {
    double radius_m = 0.0;
    double length_m = 0.0;
};

// Wall heat flux into the propellant (W/m^2; negative when the wall loses heat) of a strip absorbing
// `absorbed_w_m2`: solves emissivity sigma T^4 + (T - boiling_point) / resistance = absorbed for the skin temperature.
double strip_heat_leak_w_m2(double absorbed_w_m2, const config::ThermalSurfaceConfig& surface, double boiling_point_k);

// Net heat (W, may be negative) through the wall of a section. Strips within the shield's half width of
// `belly_azimuth_rad` (azimuth from body +y towards +z) use the shield surface. `body_to_frame` turns body axes into
// the environment's frame.
double section_heat_w(const HullSection& section, const config::ThermalConfig& thermal, double belly_azimuth_rad,
                      double boiling_point_k, const environment::RadiationEnvironment& env,
                      const math::Quat& body_to_frame);

// Heat and vented flow of every boiling hull tank (holding propellant) of the vehicle's stages with [stage.thermal].
std::vector<TankBoilOff> boil_off_rates(const Vehicle& vehicle, const environment::RadiationEnvironment& env,
                                        const math::Quat& body_to_frame);

// The flows summed by contents.
BoilOffRate total_boil_off(const Vehicle& vehicle, const std::vector<TankBoilOff>& tanks);

}  // namespace starsim::vehicle
