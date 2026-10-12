#include "starsim/frames/geodetic.hpp"

#include <cmath>

namespace starsim::frames {

math::Vec3 geodetic_to_fixed(const GeodeticPosition& geodetic, const Ellipsoid& ellipsoid) {
    const double e2 = ellipsoid.first_eccentricity_squared();
    const double sin_lat = std::sin(geodetic.latitude_rad);
    const double cos_lat = std::cos(geodetic.latitude_rad);
    // Prime vertical radius of curvature.
    const double n = ellipsoid.semi_major_axis_m / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    return {(n + geodetic.height_m) * cos_lat * std::cos(geodetic.longitude_rad),
            (n + geodetic.height_m) * cos_lat * std::sin(geodetic.longitude_rad),
            (n * (1.0 - e2) + geodetic.height_m) * sin_lat};
}

GeodeticPosition fixed_to_geodetic(const math::Vec3& fixed_m, const Ellipsoid& ellipsoid) {
    const double a = ellipsoid.semi_major_axis_m;
    const double b = ellipsoid.semi_minor_axis_m();
    const double e2 = ellipsoid.first_eccentricity_squared();
    const double ep2 = (a * a - b * b) / (b * b);  // second eccentricity squared

    const double x = fixed_m.x;
    const double y = fixed_m.y;
    const double z = fixed_m.z;
    const double p = std::hypot(x, y);

    GeodeticPosition result;
    result.longitude_rad = std::atan2(y, x);

    if (p < 1e-9) {  // on the rotation axis
        result.latitude_rad = std::copysign(constants::pi / 2.0, z);
        result.height_m = std::abs(z) - b;
        return result;
    }

    // Heikkinen (1982), as given in Zhu (1994), "Conversion of Earth-centered Earth-fixed
    // coordinates to geodetic coordinates", IEEE Trans. Aerosp. Electron. Syst. 30(3).
    const double z2 = z * z;
    const double f = 54.0 * b * b * z2;
    const double g = p * p + (1.0 - e2) * z2 - e2 * (a * a - b * b);
    const double c = e2 * e2 * f * p * p / (g * g * g);
    const double s = std::cbrt(1.0 + c + std::sqrt(c * c + 2.0 * c));
    const double k = s + 1.0 + 1.0 / s;
    const double big_p = f / (3.0 * k * k * g * g);
    const double q = std::sqrt(1.0 + 2.0 * e2 * e2 * big_p);
    const double r0 = -(big_p * e2 * p) / (1.0 + q) +
                      std::sqrt(0.5 * a * a * (1.0 + 1.0 / q) - big_p * (1.0 - e2) * z2 / (q * (1.0 + q)) -
                                0.5 * big_p * p * p);
    const double t = p - e2 * r0;
    const double u = std::sqrt(t * t + z2);
    const double v = std::sqrt(t * t + (1.0 - e2) * z2);
    const double z0 = b * b * z / (a * v);

    result.height_m = u * (1.0 - b * b / (a * v));
    result.latitude_rad = std::atan2(z + ep2 * z0, p);
    return result;
}

}  // namespace starsim::frames
