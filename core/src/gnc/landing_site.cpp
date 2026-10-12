#include "starsim/gnc/landing_site.hpp"

#include <cmath>

namespace starsim::gnc {

using math::Vec3;

Vec3 catch_roll_reference(const Vec3& across, const Vec3& up, double pin_azimuth_rad, const Vec3& current_body_y) {
    const Vec3 y = across * std::cos(pin_azimuth_rad) - cross(up, across) * std::sin(pin_azimuth_rad);
    return dot(y, current_body_y) < 0.0 ? -y : y;
}

}  // namespace starsim::gnc
