#include "starsim/environment/eclipse.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

double occulted_fraction(const math::Vec3& to_source_m, double source_radius_m, const math::Vec3& to_occluder_m,
                         double occluder_radius_m) {
    const double source_distance = norm(to_source_m);
    const double occluder_distance = norm(to_occluder_m);
    if (occluder_distance <= occluder_radius_m) {
        return 1.0;  // observer inside the occluder
    }
    if (occluder_distance > source_distance) {
        return 0.0;  // occluder behind the source
    }
    const double a = std::asin(std::min(1.0, source_radius_m / source_distance));  // apparent source radius
    const double b = std::asin(occluder_radius_m / occluder_distance);             // apparent occluder radius
    // Angular separation via atan2: well conditioned for the small angles of eclipses.
    const double c = std::atan2(norm(cross(to_source_m, to_occluder_m)), dot(to_source_m, to_occluder_m));

    if (c >= a + b) {
        return 0.0;  // no overlap
    }
    if (c <= b - a) {
        return 1.0;  // total
    }
    if (c <= a - b) {
        return (b * b) / (a * a);  // annular
    }
    // Partial overlap of two disks (planar approximation in angular coordinates).
    const double x = (c * c + a * a - b * b) / (2.0 * c);
    const double y = std::sqrt(std::max(0.0, a * a - x * x));
    const double area = a * a * std::acos(std::clamp(x / a, -1.0, 1.0)) +
                        b * b * std::acos(std::clamp((c - x) / b, -1.0, 1.0)) - c * y;
    return std::clamp(area / (constants::pi * a * a), 0.0, 1.0);
}

}  // namespace starsim::environment
