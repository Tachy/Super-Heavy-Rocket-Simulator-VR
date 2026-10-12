#include "starsim/contact/pin_contact.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::contact {

using math::Vec3;

namespace {

// Contact parameters, tuned for the 200 Hz RK4 tick (contact period ~0.4 s for a 275 t booster on two pins).
constexpr double pin_stiffness_n_m = 4.0e7;    // [E] per pin: ~3.4 cm compression under a 275 t booster
constexpr double pin_damping_n_s_m = 1.0e6;    // [E] per pin: damping ratio ~0.2
constexpr double hull_stiffness_n_m = 4.0e7;   // [E] whole hull ring
constexpr double hull_damping_n_s_m = 1.0e6;   // [E]
constexpr double friction_coefficient = 0.3;   // [E] dry steel on steel
// Below this slip friction grows linearly. The explicit (start-of-tick) friction is stable only while
// mu N / v_reg * dt / m_eff < 2; at a pin m_eff ~ 7e4 kg and mu N ~ 4e5 N, so 0.1 m/s gives ~0.3
// (0.01 m/s gave ~3: the slip reversed every tick).
constexpr double friction_regularization_mps = 0.1;
constexpr int pin_samples = 3;
constexpr int hull_samples = 16;

Vec3 radial(double azimuth_rad) { return {0.0, std::cos(azimuth_rad), std::sin(azimuth_rad)}; }

}  // namespace

bool CatchContact::all_pins_in_contact() const {
    return !pin_points_in_contact.empty() &&
           std::all_of(pin_points_in_contact.begin(), pin_points_in_contact.end(), [](int n) { return n > 0; });
}

bool CatchContact::any_contact() const { return norm(force_n) > 0.0; }

CatchBodyGeometry make_catch_geometry(const config::CatchPinConfig& pins, double stage_base_station_m,
                                      double hull_radius_m, double arm_thickness_m) {
    CatchBodyGeometry g;
    if (pins.azimuths_rad.empty()) return g;
    g.pin_station_m = stage_base_station_m + pins.station_m;
    g.max_pin_load_n = pins.max_load_n;
    for (const double azimuth : pins.azimuths_rad) {
        std::vector<Vec3> points;
        for (int k = 0; k < pin_samples; ++k) {
            const double r = hull_radius_m + pins.protrusion_m * (k + 0.5) / pin_samples;
            points.push_back(Vec3{g.pin_station_m, 0.0, 0.0} + radial(azimuth) * r);
        }
        g.pins.push_back(points);
    }
    const double hull_station = g.pin_station_m - 0.5 * arm_thickness_m;
    for (int k = 0; k < hull_samples; ++k) {
        g.hull.push_back(Vec3{hull_station, 0.0, 0.0} + radial(constants::two_pi * k / hull_samples) * hull_radius_m);
    }
    return g;
}

CatchContact catch_contact(const CatchBodyGeometry& body, const SitePose& pose, const std::array<ArmContactState, 2>& arms) {
    CatchContact result;
    result.pin_points_in_contact.assign(body.pins.size(), 0);
    auto site_point = [&](const Vec3& b) { return pose.cg_m + pose.body_to_site * (b - Vec3{pose.cg_station_m, 0.0, 0.0}); };
    // Contact force of one sample point on an arm (zero if not touching).
    auto force_at = [&](const Vec3& p, double k, double c, const ArmContactState& arm) -> Vec3 {
        const auto pen = point_in_box(arm.box, p);
        if (!pen.inside) return {};
        const Vec3 body_velocity = pose.velocity_mps + cross(pose.angular_velocity_radps, p - pose.cg_m);
        const Vec3 arm_velocity = Vec3{0.0, 0.0, arm.carriage_velocity_mps} +
                                  cross(Vec3{0.0, 0.0, arm.yaw_rate_radps}, p - arm.hinge_m);
        const Vec3 v_rel = body_velocity - arm_velocity;
        const double v_n = dot(v_rel, pen.normal);
        const double f_n = k * pen.depth_m - c * v_n;
        if (f_n <= 0.0) return {};
        const Vec3 slip = v_rel - pen.normal * v_n;
        return pen.normal * f_n - slip * (friction_coefficient * f_n / std::max(norm(slip), friction_regularization_mps));
    };
    auto apply = [&](const Vec3& p, const Vec3& f, size_t arm_index) {
        result.force_n += f;
        result.torque_nm += cross(p - pose.cg_m, f);
        result.vertical_load_n += f.z;
        result.hull_push_n[arm_index] += std::max(dot(-f, arms[arm_index].outward), 0.0);
    };
    for (size_t j = 0; j < body.pins.size(); ++j) {
        double load = 0.0;
        for (const auto& b : body.pins[j]) {
            const Vec3 p = site_point(b);
            for (size_t i = 0; i < arms.size(); ++i) {
                const Vec3 f = force_at(p, pin_stiffness_n_m / pin_samples, pin_damping_n_s_m / pin_samples, arms[i]);
                if (norm(f) == 0.0) continue;
                ++result.pin_points_in_contact[j];
                load += norm(f);
                apply(p, f, i);
            }
        }
        result.max_pin_load_n = std::max(result.max_pin_load_n, load);
    }
    for (const auto& b : body.hull) {
        const Vec3 p = site_point(b);
        for (size_t i = 0; i < arms.size(); ++i) {
            const Vec3 f = force_at(p, hull_stiffness_n_m / hull_samples, hull_damping_n_s_m / hull_samples, arms[i]);
            if (norm(f) > 0.0) apply(p, f, i);
        }
    }
    return result;
}

}  // namespace starsim::contact
