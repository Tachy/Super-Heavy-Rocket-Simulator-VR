#include "starsim/contact/probe_drogue_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace starsim::contact {

using math::Vec3;

namespace {

Vec3 friction(const Vec3& v_tangential, double normal_n, const ProbeDrogueParameters& p) {
    const double speed = norm(v_tangential);
    if (speed <= 0.0 || normal_n <= 0.0) return {};
    const double scale = std::min(1.0, speed / p.friction_slip_mps);
    return -(p.friction_coefficient * normal_n * scale / speed) * v_tangential;
}

Vec3 in_port_frame(const Vec3& point_stage_m, const config::DockingPortConfig& port) {
    const Vec3 d = point_stage_m - port.position_m;
    return {dot(d, port.axis), dot(d, cross(port.up, port.axis)), dot(d, port.up)};
}

}  // namespace

ProbeDrogueParameters probe_drogue_parameters(const config::DockingPortConfig& chaser,
                                              const config::DockingPortConfig& target) {
    ProbeDrogueParameters p;
    for (const auto& e : chaser.elements_m) p.probes_m.push_back(in_port_frame(e, chaser));
    for (const auto& e : target.elements_m) p.drogues_m.push_back(in_port_frame(e, target));
    return p;
}

InterfaceContact probe_drogue_contact(const DockingPose& pose, double extension_m, const ProbeDrogueParameters& p) {
    InterfaceContact c;
    c.capture_radius_m = p.mouth_radius_m - p.head_radius_m;
    const Vec3 x{1.0, 0.0, 0.0};
    const Vec3 xc = column(pose.chaser_axes, 0);  // the probes point along it (towards the passive port)
    const double length = p.cone_depth_m + extension_m;
    const double tan_cone = (p.mouth_radius_m - p.seat_radius_m) / p.cone_depth_m;
    const double cos_cone = 1.0 / std::sqrt(1.0 + tan_cone * tan_cone), sin_cone = tan_cone * cos_cone;
    std::vector<Vec3> heads, forces;
    bool seated = false;
    for (const auto& probe : p.probes_m) {
        const Vec3 head = pose.position_m + pose.chaser_axes * probe + length * xc;
        const Vec3 v = pose.velocity_mps + cross(pose.angular_rate_radps, head - pose.position_m);
        heads.push_back(head);
        c.gap_m = std::min(c.gap_m, head.x);
        const Vec3* mouth = &p.drogues_m.front();
        for (const auto& d : p.drogues_m) {
            if (std::hypot(head.y - d.y, head.z - d.z) < std::hypot(head.y - mouth->y, head.z - mouth->z)) mouth = &d;
        }
        const Vec3 lateral{0.0, head.y - mouth->y, head.z - mouth->z};
        const double r = norm(lateral);
        const double depth = -head.x;
        Vec3 f;
        if (depth > 0.0 && r > c.capture_radius_m) {  // beside the mouth: the plate
            const double normal = std::max(0.0, p.plate_stiffness_n_per_m * depth - p.plate_damping_ns_per_m * v.x);
            f = normal * x + friction({0.0, v.y, v.z}, normal, p);
            c.tip_strike = c.tip_strike || normal > 0.0;
        } else if (depth > 0.0) {
            const double allowed = p.mouth_radius_m - tan_cone * std::min(depth, p.cone_depth_m) - p.head_radius_m;
            if (r > allowed && r > 0.0) {  // the cone wall: normal towards the axis and out of the cone
                const Vec3 radial = lateral / r;
                const Vec3 n = -cos_cone * radial + sin_cone * x;
                const double vn = dot(v, n);
                const double fn =
                    std::max(0.0, p.wall_stiffness_n_per_m * (r - allowed) * cos_cone - p.wall_damping_ns_per_m * vn);
                f += fn * n + friction(v - vn * n, fn, p);
                c.petal_contact = c.petal_contact || fn > 0.0;
            }
            if (depth > p.cone_depth_m) {  // on the seat: the probe's attenuator
                const double fn = std::max(0.0, p.seat_stiffness_n_per_m * (depth - p.cone_depth_m) -
                                                    p.seat_damping_ns_per_m * v.x);
                f += fn * x + friction({0.0, v.y, v.z}, fn, p);
                c.ring_contact = c.ring_contact || fn > 0.0;
            }
            seated = seated ||
                     (depth >= p.cone_depth_m - p.latch_gap_m && r <= p.seat_radius_m - p.head_radius_m + p.latch_gap_m);
        }
        c.any_contact = c.any_contact || norm(f) > 0.0;
        c.compression_n += std::max(0.0, f.x);
        forces.push_back(f);
    }
    Vec3 centre;
    for (const auto& h : heads) centre += h;
    c.ring_centre_m = centre / static_cast<double>(heads.size());
    for (size_t i = 0; i < heads.size(); ++i) {
        c.force_port_n += forces[i];
        c.torque_port_nm += cross(heads[i] - c.ring_centre_m, forces[i]);
    }
    const double tilt = std::acos(std::clamp(-xc.x, -1.0, 1.0));
    const Vec3 zc = column(pose.chaser_axes, 2);
    const double roll = std::atan2(-zc.y, zc.z);
    c.latch_ready = seated && tilt <= p.latch_tilt_max_rad && std::abs(roll) <= p.latch_roll_max_rad;
    return c;
}

}  // namespace starsim::contact
