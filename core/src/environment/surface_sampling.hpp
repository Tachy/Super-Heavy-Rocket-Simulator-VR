#pragma once

#include <cmath>

#include "starsim/config/moon_surface_config.hpp"

// Sampling helpers shared by the procedural surface populations (craters, rocks): Poisson counts and the inverse
// cumulative size-frequency law, driven by uniform numbers from the integer hash (no library random numbers).

namespace starsim::environment::sampling {

// Poisson count by inversion of the cumulative distribution at the uniform number u in [0, 1).
inline int poisson(double u, double lambda) {
    double p = std::exp(-lambda), cdf = p;
    int n = 0;
    while (u > cdf) {
        ++n;
        p *= lambda / n;
        if (p <= cdf * 1e-17) break;  // no further mass (u within rounding of 1)
        cdf += p;
    }
    return n;
}

// Diameter D whose cumulative density N(>D) equals cumulative_per_m2.
inline double inverse_cumulative(const config::SizeFrequency& sfd, double cumulative_per_m2) {
    if (sfd.form == "power") {
        return sfd.reference_diameter_m * std::pow(cumulative_per_m2 / sfd.density_per_m2, -1.0 / sfd.exponent);
    }
    return -std::log(cumulative_per_m2 / sfd.density_per_m2) / sfd.exponent;  // "exponential"
}

}  // namespace starsim::environment::sampling
