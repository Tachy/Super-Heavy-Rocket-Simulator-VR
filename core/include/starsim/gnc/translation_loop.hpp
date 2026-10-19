#pragma once

#include <array>

#include "starsim/gnc/proximity_geometry.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Translation velocity loop and the approach laws of proximity operations (ISS i4). The laws return desired
// relative velocities; the loop turns the velocity error into RCS translation fractions.

// Pure translation authority along the six axes of the frame the loop works in (m/s^2): +x, -x, +y, -y, +z, -z.
struct AxisAcceleration {
    std::array<double, 6> mps2{};
    double along(int axis, double sign) const { return mps2[2 * axis + (sign < 0.0 ? 1 : 0)]; }
};

// Desired relative velocity -> translation fractions per axis with a hysteresis deadband per axis (a pulse train
// starts above start_error_mps and runs until the error is below stop_error_mps), fraction = error / (a * gain_time)
// clamped to [-1, 1]; the vector is scaled to norm <= 1.
class TranslationLoop {
public:
    math::Vec3 command(const math::Vec3& desired_mps, const math::Vec3& velocity_mps, const AxisAcceleration& authority,
                       const TranslationLoopParameters& p);
    void reset() { firing_ = {false, false, false}; }
    bool firing() const { return firing_[0] || firing_[1] || firing_[2]; }

private:
    std::array<bool, 3> firing_{false, false, false};
};

// Closing speed (> 0 towards the port) on the axis at axial distance x: between Waypoint 2 and the port linear from
// final_speed_start_mps to contact_speed_mps; farther out clamp(x / approach_time_constant_s, final_speed_start_mps,
// approach_speed_far_mps).
double axial_closing_speed_mps(double x_m, const ProximityParameters& p);

// Desired velocity in the target port frame on the axial approach: x = -closing speed, lateral = -offset *
// lateral_gain_per_s, the lateral vector limited to lateral_speed_max_mps.
math::Vec3 approach_velocity_port(const math::Vec3& position_port_m, const ProximityParameters& p);

// Position hold: -(position - hold) / hold_time_constant_s, limited to hold_speed_max_mps (any frame).
math::Vec3 hold_velocity(const math::Vec3& position_m, const math::Vec3& hold_m, const ProximityParameters& p);

}  // namespace starsim::gnc
