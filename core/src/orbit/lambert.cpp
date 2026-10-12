#include "starsim/orbit/lambert.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::orbit {

using math::Vec3;

namespace {

double stumpff_c(double z) {
    if (z > 1e-6) return (1.0 - std::cos(std::sqrt(z))) / z;
    if (z < -1e-6) return (std::cosh(std::sqrt(-z)) - 1.0) / (-z);
    return 0.5 - z / 24.0;
}

double stumpff_s(double z) {
    if (z > 1e-6) {
        const double s = std::sqrt(z);
        return (s - std::sin(s)) / (s * s * s);
    }
    if (z < -1e-6) {
        const double s = std::sqrt(-z);
        return (std::sinh(s) - s) / (s * s * s);
    }
    return 1.0 / 6.0 - z / 120.0;
}

}  // namespace

std::optional<LambertSolution> solve_lambert(const Vec3& r1, const Vec3& r2, double tof, double gm,
                                             const Vec3& reference_normal) {
    const double r1n = norm(r1);
    const double r2n = norm(r2);
    double dtheta = std::acos(std::clamp(dot(r1, r2) / (r1n * r2n), -1.0, 1.0));
    if (dot(cross(r1, r2), reference_normal) < 0.0) {
        dtheta = constants::two_pi - dtheta;  // long way round keeps the motion on the reference side
    }
    if (std::abs(std::sin(dtheta)) < 1e-8) {
        return std::nullopt;
    }
    const double a = std::sin(dtheta) * std::sqrt(r1n * r2n / (1.0 - std::cos(dtheta)));
    auto y_of = [&](double z) { return r1n + r2n + a * (z * stumpff_s(z) - 1.0) / std::sqrt(stumpff_c(z)); };
    auto f_of = [&](double z) {
        const double y = y_of(z);
        return std::pow(y / stumpff_c(z), 1.5) * stumpff_s(z) + a * std::sqrt(y) - std::sqrt(gm) * tof;
    };

    // Bracket the root on the single-revolution branch z < (2 pi)^2 by scanning valid z (y >= 0) for the
    // first sign change of F (F rises with z there), then bisect.
    const double z_max = constants::two_pi * constants::two_pi - 1e-9;
    const double z_min = -2000.0;  // hyperbolic transfers down to very short flight times
    constexpr int samples = 4000;
    double lo = 0.0, hi = 0.0;
    bool bracketed = false;
    double previous_z = 0.0;
    bool previous_valid = false;
    for (int i = 0; i <= samples && !bracketed; ++i) {
        const double z = z_min + (z_max - z_min) * i / samples;
        if (y_of(z) < 0.0) {
            previous_valid = false;
            continue;
        }
        const double value = f_of(z);
        if (previous_valid && f_of(previous_z) <= 0.0 && value > 0.0) {
            lo = previous_z;
            hi = z;
            bracketed = true;
        } else if (!previous_valid && value > 0.0) {
            // First valid sample already positive: the root lies between the y = 0 edge and here.
            double invalid = z - (z_max - z_min) / samples;
            double valid = z;
            for (int k = 0; k < 100; ++k) {
                const double mid = 0.5 * (invalid + valid);
                if (y_of(mid) < 0.0) {
                    invalid = mid;
                } else {
                    valid = mid;
                }
            }
            lo = valid;
            hi = z;
            bracketed = f_of(lo) <= 0.0;
        }
        previous_z = z;
        previous_valid = true;
    }
    if (!bracketed) {
        return std::nullopt;
    }
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (y_of(mid) < 0.0 || f_of(mid) < 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
        if (hi - lo < 1e-12 * std::max(1.0, std::abs(mid))) break;
    }
    const double z = 0.5 * (lo + hi);
    const double y = y_of(z);
    const double f = 1.0 - y / r1n;
    const double g = a * std::sqrt(y / gm);
    const double g_dot = 1.0 - y / r2n;
    return LambertSolution{(r2 - f * r1) / g, (g_dot * r2 - r1) / g};
}

}  // namespace starsim::orbit
