#pragma once

#include "starsim/math/quat.hpp"
#include "starsim/math/state_vector.hpp"

namespace starsim::gnc {

// Attitude modes of a vehicle in orbit, flown on RCS by the session's orbit controller (d2: Crew Dragon).
// Free: no attitude control (the vehicle coasts). Inertial: hold a body attitude fixed in GCRF. Lvlh: hold an attitude
// fixed in the local-vertical/local-horizontal frame. Direction: body +x along a fixed GCRF direction (roll: body +y
// towards the orbit normal). Sun: body +x at the Sun (the session fills direction_gcrf each tick).
enum class AttitudeModeKind { Free, Inertial, Lvlh, Direction, Sun };

struct AttitudeMode {
    AttitudeModeKind kind = AttitudeModeKind::Free;
    math::Quat attitude_gcrf;   // Inertial
    math::Quat body_in_lvlh;    // Lvlh (identity: nose along the velocity, +z to nadir)
    math::Vec3 direction_gcrf;  // Direction, Sun
};

// What the attitude controller steers to: body +x along axis_gcrf, body +y towards roll_reference_gcrf, while the
// target turns at rate_gcrf_radps (fed forward).
struct AttitudeTarget {
    math::Vec3 axis_gcrf;
    math::Vec3 roll_reference_gcrf;
    math::Vec3 rate_gcrf_radps;
};

// LVLH frame (CCSDS / ISS convention) as an attitude: z to nadir (-r), y against the orbit normal (-r x v), x = y x z
// (along the velocity on a circular orbit).
math::Quat lvlh_attitude_gcrf(const math::StateVector& state);
AttitudeTarget attitude_target(const AttitudeMode& mode, const math::StateVector& state);
AttitudeMode inertial_hold(const math::Quat& attitude_gcrf);
AttitudeMode lvlh_hold(const math::Quat& body_in_lvlh = {});

}  // namespace starsim::gnc
