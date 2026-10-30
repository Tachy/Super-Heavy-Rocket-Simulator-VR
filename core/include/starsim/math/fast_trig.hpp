#pragma once

#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::math {

// atan(x) to 2e-8 rad (max error, tested against std::atan): the argument is reduced to |t| <= tan(pi / 8) (one
// division, or two for |x| > 1) and the odd Taylor series is summed to t^15 (remainder t^17 / 17 < 2e-8).
// About 3x faster than std::atan2 on MSVC; for samplers that call it millions of times (terrain lookups from a
// direction), where 3 cm on the Moon are far below the data's pixel.
inline double fast_atan(double x) {
    constexpr double tan_pi_8 = 0.41421356237309503;
    const double ax = std::fabs(x);
    const bool inverted = ax > 1.0;
    double t = inverted ? 1.0 / ax : ax;
    double offset = 0.0;
    if (t > tan_pi_8) {
        t = (t - 1.0) / (t + 1.0);
        offset = 0.25 * constants::pi;
    }
    const double z = t * t;
    double r = t * (1.0 + z * (-1.0 / 3.0 + z * (1.0 / 5.0 + z * (-1.0 / 7.0 + z * (1.0 / 9.0 + z * (-1.0 / 11.0 +
                    z * (1.0 / 13.0 - z / 15.0)))))));
    r += offset;
    if (inverted) r = 0.5 * constants::pi - r;
    return std::copysign(r, x);
}

// atan2(y, x) with the accuracy of fast_atan; (0, 0) gives 0.
inline double fast_atan2(double y, double x) {
    const double ax = std::fabs(x), ay = std::fabs(y);
    if (ax >= ay) {
        if (ax == 0.0) return 0.0;
        const double a = fast_atan(y / x);
        return x > 0.0 ? a : a + std::copysign(constants::pi, y);
    }
    return std::copysign(0.5 * constants::pi, y) - fast_atan(x / y);
}

// asin(z) for z in [-1, 1] (clamped), same accuracy: atan2(z, sqrt((1 - z)(1 + z))).
inline double fast_asin(double z) {
    if (z >= 1.0) return 0.5 * constants::pi;
    if (z <= -1.0) return -0.5 * constants::pi;
    return fast_atan2(z, std::sqrt((1.0 - z) * (1.0 + z)));
}

}  // namespace starsim::math
