#include "starsim/sim/crew_escape.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::sim {

const char* to_string(CrewOutcome outcome) {
    switch (outcome) {
        case CrewOutcome::Flying: return "im Flug";
        case CrewOutcome::Rescued: return "gerettet";
        case CrewOutcome::Lost: return "verloren";
    }
    return "";
}

double hull_clearance_m(const math::Vec3& bottom, const math::Vec3& top, double radius_m, const math::Vec3& point) {
    const math::Vec3 axis = top - bottom;
    const double length2 = dot(axis, axis);
    const double s = length2 > 0.0 ? std::clamp(dot(point - bottom, axis) / length2, 0.0, 1.0) : 0.0;
    return norm(point - (bottom + axis * s)) - radius_m;
}

double plume_clearance_m(const math::Vec3& apex, const math::Vec3& axis, double half_angle_rad, double length_m,
                         const math::Vec3& point) {
    const math::Vec3 d = point - apex;
    const double along = dot(d, axis);
    const double radial = norm(d - axis * along);
    if (along >= 0.0 && along <= length_m && radial <= along * std::tan(half_angle_rad)) return 0.0;
    if (along < 0.0) return norm(d);  // ahead of the engines (beside or above the ship)
    // beside the cone: distance to its surface line in the plane of the point (beyond its end: to the end disk rim)
    const double c = std::cos(half_angle_rad), s = std::sin(half_angle_rad);
    const double along_clamped = std::min(along * c * c + radial * s * c, length_m);
    const double r_surface = along_clamped * std::tan(half_angle_rad);
    return std::hypot(along - along_clamped, radial - r_surface);
}

bool LoadJudge::exceeded(double load_g, double dt_s, double max_g, double max_s) {
    above_s_ = load_g > max_g ? above_s_ + dt_s : 0.0;
    return above_s_ > max_s;
}

}  // namespace starsim::sim
