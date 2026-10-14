#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

// Attitude control of an entering capsule (Crew Dragon) on its RCS. The offset CG trims the capsule at an angle of
// attack, and the aerodynamics hold it there; the thrusters never fight that trim. They roll the lift vector about the
// air velocity to the commanded bank and damp the pitch / yaw rates of the trim oscillation. In thin air (no restoring
// moment yet) they hold the full trim attitude instead.
struct CapsuleControlGains {
    double bank_rate_max_radps = 10.0 * constants::deg_to_rad;  // [E] (Apollo rolled at up to ~20 deg/s)
    double bank_gain_per_s = 0.5;          // [E] roll rate command per bank error
    double rate_time_constant_s = 1.0;     // [E] torque = I (w_cmd - w) / tau
    double damping_deadband_radps = 1.0 * constants::deg_to_rad;    // [E] pitch / yaw rates below this are left alone
    double roll_rate_deadband_radps = 0.5 * constants::deg_to_rad;  // [E]
    double bank_deadband_rad = 2.0 * constants::deg_to_rad;         // [E]
    double aero_hold_dynamic_pressure_pa = 200.0;  // [E] below: hold the full trim attitude
    double hold_rate_max_radps = 2.0 * constants::deg_to_rad;       // [E] slew rate of the thin-air hold
    double hold_gain_per_s = 0.5;                                   // [E]
    double hold_deadband_rad = 2.0 * constants::deg_to_rad;         // [E]
};

// Bank of the capsule now: the direction of its aerodynamic lift (the force perpendicular to the air velocity) in the
// steering basis of the local vertical (entry_attitude convention: lift = e1 cos(bank) - e2 sin(bank)). In vacuum the
// lift direction of the trim (aero::CapsuleTrim::lift_body_unit) stands in for the force.
double capsule_bank_rad(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

// Body torque fractions (VehicleCommand::rcs_torque_fraction) for this tick that fly `bank_command_rad`. The bank moves
// without wrapping: a reversal always rolls through lift up (|bank| < 90 deg), never through lift down.
math::Vec3 capsule_entry_torque_fraction(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                         time::Epoch epoch, double bank_command_rad, const CapsuleControlGains& gains = {});

// Attitude (body -> GCRF) of the capsule at its trim (at `mach`, about the current CG) with the given bank.
math::Quat capsule_trim_attitude(const vehicle::Vehicle& capsule, const math::Vec3& air_velocity_gcrf,
                                 const math::Vec3& up_gcrf, double bank_rad, double mach = 20.0);

}  // namespace starsim::gnc
