#include "starsim/gnc/landing_site.hpp"

#include <cmath>

namespace starsim::gnc {

using math::Vec3;

Vec3 catch_roll_reference(const Vec3& across, const Vec3& up, double pin_azimuth_rad, const Vec3& current_body_y) {
    const Vec3 y = across * std::cos(pin_azimuth_rad) - cross(up, across) * std::sin(pin_azimuth_rad);
    return dot(y, current_body_y) < 0.0 ? -y : y;
}

Vec3 catch_roll_reference(const CatchTarget& target, const Vec3& up, const Vec3& current_body_y) {
    if (!target.away_from_tower_azimuth_rad) {
        return catch_roll_reference(target.across_itrf, up, target.pin_azimuth_rad, current_body_y);
    }
    // The body direction at this azimuth, y cos a + (up x y) sin a, along -toward (toward = across x up).
    const double a = *target.away_from_tower_azimuth_rad;
    const Vec3 toward = cross(target.across_itrf, up);
    return target.across_itrf * std::sin(a) - toward * std::cos(a);
}

}  // namespace starsim::gnc
