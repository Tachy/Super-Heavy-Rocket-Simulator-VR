#include "starsim/contact/docking_interface.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::contact {

namespace {
// Regularised Coulomb friction opposing the tangential velocity, bounded by mu * normal.
math::Vec3 friction(const math::Vec3& v_tangential, double normal_n, const DockingInterfaceParameters& p) {
    const double speed = norm(v_tangential);
    if (speed <= 0.0 || normal_n <= 0.0) return {};
    const double scale = std::min(1.0, speed / p.friction_slip_mps);
    return -(p.friction_coefficient * normal_n * scale / speed) * v_tangential;
}
}  // namespace

InterfaceContact docking_interface_contact(const DockingPose& pose, double ring_extension_m,
                                           const DockingInterfaceParameters& p) {
    InterfaceContact c;
    const math::Vec3 xc = column(pose.chaser_axes, 0);  // chaser docking axis (points towards the passive port)
    c.ring_centre_m = pose.position_m + ring_extension_m * xc;
    const math::Vec3 ring_velocity = pose.velocity_mps + cross(pose.angular_rate_radps, ring_extension_m * xc);
    c.capture_radius_m = p.petal_clearance_m + 2.0 * p.petal_height_m * std::tan(p.petal_lean_rad);
    const double gap = c.ring_centre_m.x;  // ring face to ring face along the passive axis (the ring conforms in tilt)
    const math::Vec3 lateral{0.0, c.ring_centre_m.y, c.ring_centre_m.z};
    const double offset = norm(lateral);
    const double overlap = 2.0 * p.petal_height_m;
    if (gap >= overlap) return c;  // petals not yet meshing

    const math::Vec3 x{1.0, 0.0, 0.0};
    const math::Vec3 v_lateral{0.0, ring_velocity.y, ring_velocity.z};
    if (offset > c.capture_radius_m) {
        // tips on the opposing petals: an axial contact at the tip plane, no centring
        const double depth = overlap - gap;
        const double normal = std::max(0.0, p.ring_stiffness_n_per_m * depth - p.ring_damping_ns_per_m * ring_velocity.x);
        c.tip_strike = c.any_contact = normal > 0.0;
        c.force_port_n = normal * x + friction(v_lateral, normal, p);
        c.compression_n = normal;
        return c;
    }
    // petal cone: the allowed offset grows with the gap. The petals guide the light active ring, which gives way on its
    // platform (lateral stiffness ring_lateral_stiffness_n_per_m) up to ring_lateral_range_m; only beyond that the stiff
    // petal contact reaches the vehicle. The cone normal also pushes the ring (and the vehicle) out: tan(lean) of it.
    const double clearance = p.petal_clearance_m + std::max(gap, 0.0) * std::tan(p.petal_lean_rad);
    if (offset > clearance) {
        const double penetration = offset - clearance;  // lateral deflection the ring must take
        const math::Vec3 radial = lateral / offset;
        const double v_radial = dot(ring_velocity, radial);
        double lateral_n = p.ring_lateral_stiffness_n_per_m * std::min(penetration, p.ring_lateral_range_m);
        if (penetration > p.ring_lateral_range_m) {  // the platform at its stop: the stiff petal contact
            const double depth = (penetration - p.ring_lateral_range_m) * std::cos(p.petal_lean_rad);
            lateral_n += std::cos(p.petal_lean_rad) *
                         std::max(0.0, p.petal_stiffness_n_per_m * depth + p.petal_damping_ns_per_m * v_radial);
        }
        c.petal_contact = lateral_n > 0.0;
        c.any_contact = c.any_contact || c.petal_contact;
        const double axial_n = lateral_n * std::tan(p.petal_lean_rad);
        c.force_port_n += -lateral_n * radial + axial_n * x +
                          friction(ring_velocity - v_radial * radial, lateral_n / std::cos(p.petal_lean_rad), p);
        c.compression_n += axial_n;
    }
    if (gap < 0.0) {
        const double normal = std::max(0.0, -p.ring_stiffness_n_per_m * gap - p.ring_damping_ns_per_m * ring_velocity.x);
        c.ring_contact = normal > 0.0;
        c.any_contact = c.any_contact || c.ring_contact;
        c.force_port_n += normal * x + friction(v_lateral, normal, p);
        c.compression_n += normal;
    }
    // latch condition: faces within the gap, centred, tilt within the ring's range, roll within the strikers'
    const double tilt = std::acos(std::clamp(-xc.x, -1.0, 1.0));  // mated: x_c = -x
    const math::Vec3 zc = column(pose.chaser_axes, 2);
    const double roll = std::atan2(-zc.y, zc.z);  // the chaser's up against the passive up about x
    // the conformed ring is centred while its deflection stays within its lateral range
    const double deflection = std::max(0.0, offset - p.petal_clearance_m);
    c.latch_ready = gap <= p.latch_gap_m && deflection <= p.ring_lateral_range_m && tilt <= p.ring_tilt_max_rad &&
                    std::abs(roll) <= p.latch_roll_max_rad;
    return c;
}

}  // namespace starsim::contact
