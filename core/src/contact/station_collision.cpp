#include "starsim/contact/station_collision.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::contact {

using math::Vec3;

std::vector<math::Vec3> hull_sample_points(const config::VehicleConfig& vehicle, const StationCollisionParameters& p) {
    std::vector<math::Vec3> points;
    double base = 0.0;
    for (size_t i = 0; i < vehicle.stages.size(); ++i) {
        const auto& s = vehicle.stages[i];
        double top = base + s.length_m;
        const bool top_stage = i + 1 == vehicle.stages.size();
        // a nose port (axis along +x: Crew Dragon) ends the hull below it - the lowest one if there are several;
        // dorsal ports (the ships) do not
        if (top_stage) {
            for (const auto& port : s.docking_ports) {
                if (port.axis.x > 0.5) top = std::min(top, base + port.position_m.x - p.port_clearance_m);
            }
        }
        const double r0 = 0.5 * s.diameter_m;
        const double r1 = 0.5 * (s.hull_top_diameter_m > 0.0 ? s.hull_top_diameter_m : s.diameter_m);
        const int rings = static_cast<int>(std::floor((top - base) / p.ring_spacing_m + 1e-9));
        for (int k = 0; k <= rings; ++k) {
            const double x = std::min(base + k * p.ring_spacing_m, top);
            const double r = r0 + (r1 - r0) * (x - base) / s.length_m;
            for (int j = 0; j < p.points_per_ring; ++j) {
                const double phi = 2.0 * constants::pi * j / p.points_per_ring;
                points.push_back({x, r * std::cos(phi), r * std::sin(phi)});
            }
        }
        // landing-leg struts (HLS): from the hull at leg_root_station_m to the deployed foot (refuelling r3: the legs
        // must clear the depot)
        const auto& legs = s.landing_legs;
        for (const double az : legs.azimuths_rad) {
            const Vec3 dir{0.0, std::cos(az), std::sin(az)};
            const Vec3 root = Vec3{base + p.leg_root_station_m, 0.0, 0.0} + r0 * dir;
            const Vec3 foot = Vec3{base + legs.foot_station_m, 0.0, 0.0} + legs.footprint_radius_m * dir;
            const int n = std::max(1, static_cast<int>(std::ceil(norm(foot - root) / p.ring_spacing_m)));
            for (int k = 0; k <= n; ++k) points.push_back(root + (static_cast<double>(k) / n) * (foot - root));
        }
        base += s.length_m;
    }
    return points;
}

StationShape station_shape(const config::VehicleConfig& station) {
    StationShape shape;
    double base = 0.0;
    for (const auto& s : station.stages) {
        const math::Vec3 offset{base, 0.0, 0.0};
        for (const auto& c : s.collision) {
            if (c.kind == "capsule") {
                shape.capsules.push_back({c.a_m + offset, c.b_m + offset, c.radius_m});
            } else {
                shape.boxes.push_back({c.a_m + offset, math::Mat3::identity(), c.half_extents_m});
            }
        }
        base += s.length_m;
    }
    return shape;
}

HullContact hull_contact(const StationShape& shape, const std::vector<math::Vec3>& points,
                         const std::vector<math::Vec3>& velocities, const math::Vec3& cg,
                         const StationCollisionParameters& p) {
    HullContact contact;
    for (size_t i = 0; i < points.size(); ++i) {
        Penetration deepest;
        for (const auto& c : shape.capsules) {
            const auto pen = point_in_capsule(c.a_m, c.b_m, c.radius_m, points[i]);
            if (pen.inside && pen.depth_m > deepest.depth_m) deepest = pen;
        }
        for (const auto& b : shape.boxes) {
            const auto pen = point_in_box(b, points[i]);
            if (pen.inside && pen.depth_m > deepest.depth_m) deepest = pen;
        }
        if (!deepest.inside) continue;
        const double vn = dot(velocities[i], deepest.normal);
        const double normal = std::max(0.0, p.stiffness_n_per_m * deepest.depth_m - p.damping_ns_per_m * vn);
        const math::Vec3 f = normal * deepest.normal;
        contact.any = true;
        contact.max_depth_m = std::max(contact.max_depth_m, deepest.depth_m);
        contact.force_station_n += f;
        contact.torque_station_nm += cross(points[i] - cg, f);
    }
    return contact;
}

}  // namespace starsim::contact
