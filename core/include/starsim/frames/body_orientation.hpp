#pragma once

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::frames {

// Rotation from a body-fixed frame to GCRF, including its time derivative so that
// velocities transform correctly: v_gcrf = R * v_fixed + dR/dt * r_fixed.
struct FixedToInertial {
    math::Mat3 rotation;
    math::Mat3 rotation_rate_per_s;

    math::Vec3 position(const math::Vec3& fixed_m) const { return rotation * fixed_m; }

    math::Vec3 velocity(const math::Vec3& fixed_m, const math::Vec3& fixed_velocity_mps) const {
        return rotation * fixed_velocity_mps + rotation_rate_per_s * fixed_m;
    }
};

// Orientation of a rotating body (Earth: ITRF; Moon: principal-axes frame) relative to GCRF axes.
class BodyOrientation {
public:
    virtual ~BodyOrientation() = default;

    virtual FixedToInertial fixed_to_gcrf(time::Epoch epoch) const = 0;
};

// Rotation vector of a body-fixed frame in GCRF axes from the transform's rate (omega_x = dR/dt * R^T).
inline math::Vec3 rotation_vector(const FixedToInertial& transform) {
    const math::Mat3 w = transform.rotation_rate_per_s * transpose(transform.rotation);
    return {0.5 * (w(2, 1) - w(1, 2)), 0.5 * (w(0, 2) - w(2, 0)), 0.5 * (w(1, 0) - w(0, 1))};
}

}  // namespace starsim::frames
