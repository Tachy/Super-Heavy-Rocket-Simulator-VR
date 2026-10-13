#pragma once

#include "starsim/math/quat.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Attitude setpoint of a belly-first entry, for AttitudeController::steer.
struct EntryAttitude {
    math::Vec3 axis_gcrf;            // desired body x-axis (nose)
    math::Vec3 roll_reference_gcrf;  // where body +y must point
};

// From the velocity relative to the air, the local vertical, the angle of attack alpha (between the body axis and
// the velocity, the belly leading) and the bank angle (the lift vector rotated right-handed about the velocity
// from the vertical plane). belly_azimuth_rad: body direction of the heat shield (from +y towards +z).
// Lift direction l = up perpendicular to the velocity u, rotated by the bank; nose x = cos(alpha) u + sin(alpha) l;
// belly b = sin(alpha) u - cos(alpha) l.
EntryAttitude entry_attitude(const math::Vec3& air_velocity_gcrf, const math::Vec3& up_gcrf, double alpha_rad,
                             double bank_rad, double belly_azimuth_rad);

// The same setpoint as a body -> GCRF rotation (initial states, tests).
math::Quat entry_attitude_quaternion(const math::Vec3& air_velocity_gcrf, const math::Vec3& up_gcrf, double alpha_rad,
                                     double bank_rad, double belly_azimuth_rad);

}  // namespace starsim::gnc
