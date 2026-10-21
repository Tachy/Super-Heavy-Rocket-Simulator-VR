#pragma once

#include <cstddef>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/thermal_radiation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

// Refuelling r2: propellant boil-off from a quasi-steady heat balance on the hull. Every hull tank of a stage with
// [stage.thermal] (a main tank of the stage's diameter) is a cylinder of its full-load length, split into
// boil_off_strips strips around the circumference. Per strip: absorbed (Sun, Earth albedo, Earth infrared) =
// emissivity sigma T^4 + (T - T_boil) / R_wall - on its dry part (the liquid not wetting it) through the ullage gas
// as well. The leaks over the tank's strips are summed; that heat (clamped at 0: a tank losing heat through its wall
// does not condense) over the latent heat is the vented mass flow. Where the liquid lies: settled by the apparent
// gravity or spread over the walls by surface tension, by the Bond number (Task 5b).
// Deliberate simplifications [docs/subsystems.md]: no skin heat capacity and no conduction around the circumference;
// no tank pressure, temperature, subcooling or stratification; the dry wall's leak is vented like boil-off; domes,
// ends, header and RCS tanks take no heat; no shadows of other vehicles or of the flaps.
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
// the environment's frame. `wetted` (one fraction per strip; nullptr: all wet): the dry part of a strip passes its
// heat through the ullage gas as well (ThermalConfig::ullage_resistance_m2k_w).
double section_heat_w(const HullSection& section, const config::ThermalConfig& thermal, double belly_azimuth_rad,
                      double boiling_point_k, const environment::RadiationEnvironment& env,
                      const math::Quat& body_to_frame, const std::vector<double>* wetted = nullptr);

// What moves the liquid in the tanks (Task 5b): the non-gravitational specific force (drag, thrust; body axes), the
// body's rotation and its place in the Earth's field (tide). Euler and Coriolis terms are left out (slow rotation).
struct LiquidEnvironment {
    math::Vec3 specific_force_body_mps2;
    math::Vec3 angular_velocity_body_radps;
    math::Vec3 position_gcrf_m;  // of the free-falling reference point (the body's CG, or a docked pair's compound CG)
    math::Quat body_to_gcrf;
    math::Vec3 cg_offset_body_m;  // refuelling r4: this body's CG from the reference point (docked: the compound CG)
    // f3: the body the vehicle orbits (its tide): the Earth by default, the Moon in lunar orbit
    math::Vec3 centre_gcrf_m;
    double gm_m3ps2 = constants::earth_gm_m3ps2;
};

// Apparent gravity on liquid `offset_body_m` from the CG (body axes): the orbited body's tide minus the specific force
// plus the centrifugal acceleration of the rotation; tide and rotation act about the reference point.
math::Vec3 apparent_gravity_body_mps2(const LiquidEnvironment& liquid, const math::Vec3& offset_body_m);

// How far the liquid follows the apparent gravity, by the Bond number rho a r^2 / sigma: 0 at Bo <= 1 (surface tension
// spreads the wetting liquid over the walls - Centaur's zero-g coasts, AIAA 2007-5845), 1 at Bo >= 10 (settled) [E],
// log-linear between.
double settled_weight(double bond_number);

// Wetted fraction of each strip of a cylinder (radius 1, length `aspect` radii) holding `fill` of its volume, the
// liquid settled towards `down_body` below a flat surface (domes left out).
std::vector<double> settled_wetted_fractions(double fill, const math::Vec3& down_body, double aspect);

// Heat and vented flow of every boiling hull tank (holding propellant) of the vehicle's stages with [stage.thermal].
// `liquid`: where the liquid lies decides the wetted wall (Bond number); nullptr: all walls wet (conservative).
std::vector<TankBoilOff> boil_off_rates(const Vehicle& vehicle, const environment::RadiationEnvironment& env,
                                        const math::Quat& body_to_frame, const LiquidEnvironment* liquid = nullptr);

// The flows summed by contents.
BoilOffRate total_boil_off(const Vehicle& vehicle, const std::vector<TankBoilOff>& tanks);

// Mean vented flow over one two-body orbit about the Earth (the r6 campaign calculator's daily losses): the vehicle
// held at `body_in_lvlh` (body -> LVLH of frames/lvlh.hpp: x along the velocity, z nadir), the Sun fixed at
// `sun_position_gcrf_m` over the orbit, Earth shadow only, tank masses as they are.
BoilOffRate orbit_average_boil_off(const Vehicle& vehicle, const math::StateVector& orbit_gcrf,
                                   const math::Quat& body_in_lvlh, const math::Vec3& sun_position_gcrf_m,
                                   int samples = 360);
}  // namespace starsim::vehicle
