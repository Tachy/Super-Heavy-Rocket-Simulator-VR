#include "starsim/contact/landing_legs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "starsim/math/constants.hpp"

namespace starsim::contact {

using math::Vec3;

namespace {

// Below this slip friction grows linearly (explicit friction stability, as for the catch pins: mu N / v_reg * dt / m
// per foot ~ 0.6 * 7e4 N / 0.1 m/s * 0.005 s / 7e3 kg ~ 0.3 < 2).
constexpr double friction_regularization_mps = 0.1;
constexpr int base_samples = 12;

Vec3 radial(double azimuth_rad) { return {0.0, std::cos(azimuth_rad), std::sin(azimuth_rad)}; }

constexpr double min_axis_cosine = 0.1;  // [E] a surface that steep to the stroke axis gives no meaningful stroke

}  // namespace

double foot_compression_along_axis_m(double depth_m, const Vec3& n, const Vec3& axis) {
    const double c = dot(n, axis);
    return depth_m > 0.0 && c > min_axis_cosine ? depth_m / c : 0.0;
}

LegGeometry make_leg_geometry(const config::LandingLegConfig& legs, double stage_base_station_m, double hull_radius_m) {
    LegGeometry g;
    g.config = legs;
    if (legs.azimuths_rad.empty()) return g;
    for (const double azimuth : legs.azimuths_rad) {
        g.feet.push_back(Vec3{stage_base_station_m + legs.foot_station_m, 0.0, 0.0} + radial(azimuth) * legs.footprint_radius_m);
    }
    for (int k = 0; k < base_samples; ++k) {
        g.base_ring.push_back(Vec3{stage_base_station_m, 0.0, 0.0} +
                              radial(constants::two_pi * k / base_samples) * hull_radius_m);
    }
    return g;
}

LegContact leg_contact(const LegGeometry& legs, std::vector<double>& crushed_m, const SitePose& pose,
                       const LandingSurface& surface, const FrictionLimit* friction_limit,
                       std::vector<FootStick>* stick) {
    LegContact result;
    crushed_m.resize(legs.feet.size(), 0.0);
    result.foot_compression_m.assign(legs.feet.size(), 0.0);
    const bool stick_slip = friction_limit && stick;
    if (stick) stick->resize(legs.feet.size());
    const auto release = [&](size_t i) {
        if (stick) (*stick)[i].anchored = false;
    };
    const Vec3 station_axis = pose.body_to_site * Vec3{1.0, 0.0, 0.0};
    const auto& c = legs.config;
    auto site_point = [&](const Vec3& b) { return pose.cg_m + pose.body_to_site * (b - Vec3{pose.cg_station_m, 0.0, 0.0}); };
    // Each foot's pad on the support under its disc (a point foot for foot_pad_radius_m = 0). A foot above the
    // surface's upper bound touches nothing: its query is skipped (the same result - no depth, not touching).
    std::vector<SurfaceSupport> supports;
    supports.reserve(legs.feet.size());
    for (const auto& foot : legs.feet) {
        const Vec3 p = site_point(foot);
        if (p.z >= surface.below_disc_upper_bound(p.x, p.y, c.foot_pad_radius_m)) {
            supports.push_back({-std::numeric_limits<double>::infinity(), false, {0.0, 0.0, 1.0}});
        } else {
            supports.push_back(surface.below_disc(p.x, p.y, c.foot_pad_radius_m));
        }
    }
    // Feet that touch supported ground (they share the friction limit).
    int touching = 0;
    for (size_t i = 0; i < legs.feet.size(); ++i) {
        const Vec3 p = site_point(legs.feet[i]);
        const auto& support = supports[i];
        if (!support.water && (support.height_m - p.z) * support.normal.z > 0.0) ++touching;
    }
    // Effective mass of the body at a point for a push along `t` (site frame): 1 / (1/m + (r x t) . I^-1 (r x t)).
    auto effective_mass = [&](const Vec3& r, const Vec3& t) {
        const Vec3 a = transpose(pose.body_to_site) * cross(r, t);  // body axes
        const auto& inertia = friction_limit->inertia_body_kg_m2;
        const double rotational = a.x * a.x / inertia.x + a.y * a.y / inertia.y + a.z * a.z / inertia.z;
        return 1.0 / (1.0 / friction_limit->mass_kg + rotational);
    };
    for (size_t i = 0; i < legs.feet.size(); ++i) {
        const Vec3 p = site_point(legs.feet[i]);
        const auto& support = supports[i];
        const Vec3 n = support.normal;
        const double depth = (support.height_m - p.z) * n.z;  // penetration along the normal (flat: n.z = 1)
        if (depth <= 0.0) {
            release(i);
            continue;
        }
        result.foot_compression_m[i] = support.water ? 0.0 : foot_compression_along_axis_m(depth, n, station_axis);
        if (support.water) {
            result.water = true;
            release(i);
            continue;
        }
        const Vec3 v = pose.velocity_mps + cross(pose.angular_velocity_radps, p - pose.cg_m);
        const double v_n = dot(v, n);
        double f_n = c.stiffness_n_m * (depth - crushed_m[i]) - c.damping_n_s_m * v_n;
        if (f_n <= 0.0) {
            release(i);
            continue;
        }
        if (f_n > c.crush_force_n) {  // the core crushes: the elastic part stays at the crush force
            crushed_m[i] = std::max(crushed_m[i], depth - c.crush_force_n / c.stiffness_n_m);
            f_n = c.crush_force_n;
        }
        if (crushed_m[i] > c.stroke_m) result.stroke_exhausted = true;
        const Vec3 slip = v - n * v_n;  // in the surface (flat: (v.x, v.y, 0) exactly)
        // Caps a slip force to what stops the foot's slip within the tick (explicit stability, shared by the feet).
        const auto cap_to_stop = [&](Vec3 force) {
            if (norm(slip) > 0.0) {
                const double stop = effective_mass(p - pose.cg_m, slip / norm(slip)) * norm(slip) /
                                    (friction_limit->dt_s * std::max(touching, 1));
                if (norm(force) > stop) force = force * (stop / norm(force));
            }
            return force;
        };
        Vec3 friction;
        if (stick_slip) {
            auto& s = (*stick)[i];
            if (!s.anchored) {
                s.anchored = true;
                s.anchor_m = p;
            }
            // Stick spring tuned to the smallest effective mass over the tangent plane (the stiffest stable choice).
            const Vec3 t1 = normalized(std::abs(n.x) < 0.9 ? cross(n, Vec3{1.0, 0.0, 0.0}) : cross(n, Vec3{0.0, 1.0, 0.0}));
            const Vec3 t2 = cross(n, t1);
            const double mass = std::min(effective_mass(p - pose.cg_m, t1), effective_mass(p - pose.cg_m, t2)) /
                                std::max(touching, 1);
            const double omega = stick_omega_dt / friction_limit->dt_s;
            const double k = mass * omega * omega, damping = 2.0 * mass * omega;
            const Vec3 offset = p - s.anchor_m;
            const Vec3 d = offset - n * dot(offset, n);  // tangential
            friction = d * -k - slip * damping;
            const double limit = c.friction * f_n;
            if (norm(friction) > limit) {  // slips: Coulomb force, the anchor follows the pad
                friction = cap_to_stop(friction * (limit / norm(friction)));
                s.anchor_m = p + friction / k;  // the new tangential offset p - anchor is -friction / k
            }
        } else {
            friction = slip * (-c.friction * f_n / std::max(norm(slip), friction_regularization_mps));
            if (friction_limit) friction = cap_to_stop(friction);
        }
        const Vec3 f = n * f_n + friction;
        result.force_n += f;
        result.torque_nm += cross(p - pose.cg_m, f);
        result.max_foot_force_n = std::max(result.max_foot_force_n, f_n);
        ++result.feet_in_contact;
    }
    for (const auto& b : legs.base_ring) {
        const Vec3 p = site_point(b);
        if (p.z >= surface.height_below_upper_bound(p.x, p.y)) continue;
        const auto support = surface.height_below(p.x, p.y);
        if (p.z < support.height_m) {
            if (support.water) result.water = true;
            else result.base_contact = true;
        }
    }
    return result;
}

}  // namespace starsim::contact
