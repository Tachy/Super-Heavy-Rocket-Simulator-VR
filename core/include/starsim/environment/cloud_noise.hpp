#pragma once

// Tileable 3D noise textures for the sub-grid cloud shapes (M8 weather and clouds), after Schneider 2015 ("The
// Real-time Volumetric Cloudscapes of Horizon Zero Dawn", SIGGRAPH Advances in Real-Time Rendering) and Hillaire's
// TileableVolumeNoise. Deterministic (fixed seed, integer hashing, no library random), plain C++, built in parallel over
// z slices. The shader (game/shaders/clouds.gdshaderinc) reads them; the coverage rule is the core's.
//
// Shape texture (shape_size^3 RGBA8, x fastest, then y, then z):
//   R: Perlin-Worley = remap(perlin, 0, 1, worley_fbm(4), 1) = wf + perlin (1 - wf), perlin = Perlin fBm of lattice
//      periods 8, 16, 32, 64 (gain 0.5) mapped to [0, 1], then rank-equalised (a monotone remap to a uniform
//      distribution: the shapes are unchanged, the coverage resolution is 1/256 everywhere);
//   G, B, A: worley_fbm(4), worley_fbm(8), worley_fbm(16) (erosion octaves, another seed than R).
// Detail texture (detail_size^3 RGBA8): R, G, B worley_fbm(2), worley_fbm(4), worley_fbm(8); A unused (0).
// worley_fbm(c) = 0.625 W(c) + 0.25 W(2c) + 0.125 W(4c), W(c) = 1 - min(1, F1) with F1 the distance (in cells) to the
// nearest feature point of a c^3 cell grid that wraps (one point per cell).
//
// Coverage rule: coverage_threshold[i] = t with fraction(R > t) = i / 255 for the R channel sampled TRILINEARLY at
// uniform random positions (what the shader reads: magnified, any scale and offset, wrapping, no mipmaps), from 8
// jittered samples per texel cell binned at 1/4096. A layer with cloud fraction cf is cloud where the filtered shape
// value exceeds t(cf), so the covered fraction equals cf; the in-cloud water content is the grid mean / cf (the
// shader's part). Filtering narrows the value distribution: a table from the texel histogram gave filtered fractions
// 0.0978 / 0.4994 / 0.9068 for the entries 26 / 128 / 230 (0.1020 / 0.5020 / 0.9020); this table gives 0.1020 /
// 0.5017 / 0.9021 (2e6 random positions, task 4 fix round 1). Mipmapped (minified) sampling would narrow it further,
// so the shader samples the shape without mipmaps. Cells whose 8 texels share a code filter to that code exactly
// (point masses, handled apart). t[0] > 1 (nothing), t[255] < 0 (everything).

#include <array>
#include <cstdint>
#include <vector>

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

struct CloudNoise {
    int shape_size = 128, detail_size = 32;
    std::vector<std::uint8_t> shape_rgba, detail_rgba;
    std::array<float, 256> coverage_threshold{};
};

CloudNoise generate_cloud_noise();

// The tileable primitives (coordinates in lattice / cell units, period in the same units; exact repetition with it).
double tileable_perlin(const math::Vec3& p, int period);                 // about [-1, 1]
double tileable_worley(const math::Vec3& p, int period, std::uint32_t seed);  // W = 1 - min(1, F1), [0, 1]

}  // namespace starsim::environment
