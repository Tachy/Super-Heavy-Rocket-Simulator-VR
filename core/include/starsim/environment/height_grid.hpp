#pragma once

#include <cstdint>
#include <vector>

namespace starsim::environment {

// Heights above a reference surface on a regular grid, stored as the LOLA products store them (int16 x scale).
struct HeightGrid {
    int lines = 0;
    int samples = 0;
    double dn_scale_m = 0.0;
    std::vector<std::int16_t> dn;  // row-major, lines x samples

    double at(int line, int sample) const {
        return dn_scale_m * dn[static_cast<size_t>(line) * static_cast<size_t>(samples) + static_cast<size_t>(sample)];
    }
    // Bilinear inside [0, lines-1] x [0, samples-1] (nodes exactly).
    double bilinear(double line, double sample) const;
};

}  // namespace starsim::environment
