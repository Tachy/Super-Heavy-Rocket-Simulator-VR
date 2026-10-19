#include "starsim/gnc/translation_loop.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

namespace {

math::Vec3 limit_norm(const math::Vec3& v, double max_norm) {
    const double n = norm(v);
    return n > max_norm ? v * (max_norm / n) : v;
}

}  // namespace

math::Vec3 TranslationLoop::command(const math::Vec3& desired, const math::Vec3& velocity,
                                    const AxisAcceleration& authority, const TranslationLoopParameters& p) {
    const math::Vec3 e = desired - velocity;
    const double err[3] = {e.x, e.y, e.z};
    double f[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        const double a = std::abs(err[i]);
        if (!firing_[i] && a > p.start_error_mps) firing_[i] = true;
        if (firing_[i] && a < p.stop_error_mps) firing_[i] = false;
        const double acc = authority.along(i, err[i]);
        if (firing_[i] && acc > 0.0) f[i] = std::clamp(err[i] / (acc * p.gain_time_s), -1.0, 1.0);
    }
    return limit_norm({f[0], f[1], f[2]}, 1.0);
}

double axial_closing_speed_mps(double x, const ProximityParameters& p) {
    if (x <= p.waypoint2_range_m) {
        const double s = std::clamp(x / p.waypoint2_range_m, 0.0, 1.0);
        return p.contact_speed_mps + (p.final_speed_start_mps - p.contact_speed_mps) * s;
    }
    return std::clamp(x / p.approach_time_constant_s, p.final_speed_start_mps, p.approach_speed_far_mps);
}

math::Vec3 approach_velocity_port(const math::Vec3& position_port_m, const ProximityParameters& p) {
    const math::Vec3 lateral =
        limit_norm({0.0, -position_port_m.y * p.lateral_gain_per_s, -position_port_m.z * p.lateral_gain_per_s},
                   p.lateral_speed_max_mps);
    return {-axial_closing_speed_mps(position_port_m.x, p), lateral.y, lateral.z};
}

math::Vec3 hold_velocity(const math::Vec3& position_m, const math::Vec3& hold_m, const ProximityParameters& p) {
    return limit_norm((position_m - hold_m) * (-1.0 / p.hold_time_constant_s), p.hold_speed_max_mps);
}

}  // namespace starsim::gnc
