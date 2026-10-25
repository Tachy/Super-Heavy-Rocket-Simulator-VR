#pragma once

// Tileable 3D noise textures for the sub-grid cloud shapes (M8 weather and clouds), after Schneider 2015 ("The
// Real-time Volumetric Cloudscapes of Horizon Zero Dawn", SIGGRAPH Advances in Real-Time Rendering) and Hillaire's
// TileableVolumeNoise. Deterministic (fixed seed, integer hashing, no library random), plain C++, built in parallel over
// z slices. The shader (game/shaders/clouds.gdshaderinc) reads them; the coverage rule is the core's.
//
// Shape texture (shape_size^3 RGBA8, x fastest, then y, then z):
//   R: Perlin-Worley = remap(perlin, 0, 1, worley_fbm(4), 1) = wf + perlin (1 - wf), perlin = Perlin fBm of lattice
//      periods 8, 16, 32, 64 with the fractal gain 2^-H (below) mapped to [0, 1], then rank-equalised (a monotone remap
//      to a uniform distribution: the shapes are unchanged, the coverage resolution is 1/256 everywhere);
//   G, B: worley_fbm(4), worley_fbm(8) (erosion octaves, another seed than R; not read by the shader);
//   A: the edge-width noise n: Perlin fBm of lattice periods 16, 32, 64 (gain 0.5, another gradient seed),
//      rank-equalised (soft edges below).
// Detail texture (detail_size^3 RGBA8): R, G, B the single Worley octaves W(2), W(4), W(8); A unused (0). The detail
// value d = (R + g G + g^2 B) / (1 + g + g^2) with the fractal gain g = 2^-H (below; detail_weights), detail_mean its
// mean over the texture (which is the filtered mean too).
// worley_fbm(c) = 0.625 W(c) + 0.25 W(2c) + 0.125 W(4c), W(c) = 1 - min(1, F1) with F1 the distance (in cells) to the
// nearest feature point of a c^3 cell grid that wraps (one point per cell).
//
// Fractal shape value (task 8 fix round 1, irregular edges at every distance): cloud perimeters are fractal from metres
// to hundreds of km with the area-perimeter dimension D = 1.35 (constants.hpp cloud_perimeter_fractal_dimension, Lovejoy
// 1982). The level sets of a fractional Brownian field of Hurst exponent H have dimension 2 - H, so the shape value
// continues as fBm with H = 2 - D = 0.65 below the shape texture: the value the coverage rule thresholds is
//   v = s + sum_k a_k (d_k - detail_mean),  k = 0 .. cloud_detail_levels - 1,
// s the filtered shape R, d_k the detail value at the isotropic tile T_k = cloud_detail_tile_m / 4^k (cells of T_k / 2
// to T_k / 8: 500 m down to 8 m), a_k = A 4^(-H k) (the fBm gain per factor 4 in scale). A continues the shape's own
// spectrum: the horizontal increment variance of v at the lag cloud_detail_join_lag_m equals the shape's at its finest
// Perlin period r0 = 500 m scaled by fBm, Var(r0) (r / r0)^(2H) (increment variances of independent parts add), times
// cloud_detail_amplitude_calibration (the measured perimeter dimension is D = 1.35 with it).
// Band limit: an octave finer than the pixel aliases, so level k has the weight
//   w_k = clamp(log2(l_k / f) - 1, 0, 1),  l_k = T_k / 4 (its middle cell), f the pixel footprint [m]
// (full where the middle cell spans >= 4 pixels, none at <= 2); the levels fade finest first and never overlap, so
// the detail state L = sum w_k in [0, levels] says it all: level k enters with clamp(L - k, 0, 1). Faded levels
// contribute their mean (0): the silhouette stays irregular down to the pixel, and the tables below keep the water.
//
// Coverage rule: coverage_threshold[L][i] = t with fraction(v > t) = i / 255 for the value v of the detail state L at
// uniform random positions with the R channel sampled TRILINEARLY (what the shader reads: magnified, any scale and
// offset, wrapping, no mipmaps). State 0 (the shape alone): from 8 jittered samples per texel cell binned at 1/4096, the
// cells whose 8 texels share a code as point masses (filtered fractions 0.1020 / 0.5017 / 0.9021 for the entries
// 26 / 128 / 230 with the earlier gain 0.5, 2e6 random positions, task 4 fix round 1; a texel histogram gave 0.0978 /
// 0.4994 / 0.9068). States 1 .. levels: from 2 jittered samples per texel cell of the composite, binned at 1/4096.
// The shader interpolates the rows linearly at a fractional state. A layer with cloud fraction cf is cloud where v
// exceeds t(cf), so the covered fraction equals cf at every state. t[L][0] > every value, t[L][255] < every value.
//
// Soft edges (task 8 amendment; fix round 1: varying width): the in-cloud water does not jump at the coverage boundary
// but rises over the cloud's edge zone (entrainment): with the excess e = v - t(cf), the density is
//   beta = beta_mean / m(cf) x ramp(e / w(n)),   ramp(x) = smoothstep(0, 1, x) = x^2 (3 - 2x) on [0, 1],
// zero where e <= 0 (the covered area fraction stays cf), full in the cloud's interior. The edge width varies along the
// perimeter (constants.hpp cloud_edge_width_min_m .. _max_m, log-uniform by the width noise n of the shape's A channel):
//   w(n) = w_min (w_max / w_min)^n,
// w_min / w_max the median horizontal increment of v (all levels) over those widths (the value change across the zone).
// Wide zones are the wispy fringes of thin, gradual transmittance, narrow ones the sharp cauliflower edges.
// m[L](cf) = E[ramp((v - t[L](cf)) / w(n))] over the joint (v, n) distribution of the state (binned at 1/2048 x 1/16),
// <= cf: the mean over the cell is the field's grid mean at every state and edge width (the "alpha" at the edge is real
// transmittance of less water, not a fudge).
//
// Blue noise (task 8 amendment, no grain): a size^2 tileable rank mask (void-and-cluster, Ulichney 1993, "The
// void-and-cluster method for dither array generation", Proc. SPIE 1913, 332; Gaussian energy sigma 1.5 on the
// torus), byte = rank x 256 / size^2 (each value equally often). The shader offsets the raymarch start per pixel by it
// (and per frame by the golden ratio): the error is pushed to high spatial frequencies, where it is least visible.

#include <array>
#include <cstdint>
#include <vector>

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// [E] Scales of the sub-grid shapes in the shader (game/shaders/clouds.gdshaderinc, change both together): the shape
// tile 32 km across (Perlin octaves of 4 km to 500 m, Worley cells of 8 km) and 16 km high; the detail tile of level 0
// 1 km (each further level 4 x smaller), three levels (cells down to 8 m: finer ones only matter inside ~1 km, where
// the half-resolution pixel is ~2 m - and the cost per sample grows with each level).
inline constexpr double cloud_shape_tile_horizontal_m = 32000.0;
inline constexpr double cloud_shape_tile_vertical_m = 16000.0;
inline constexpr double cloud_detail_tile_m = 1000.0;
inline constexpr int cloud_detail_levels = 3;
inline constexpr double cloud_detail_level_ratio = 4.0;
// Lag where the detail amplitude A joins the shape's spectrum (header): the middle of the detail scales.
inline constexpr double cloud_detail_join_lag_m = 62.5;
// [calibrated] Factor on the spectral continuation of A (header): D = 2 - H holds for Gaussian fields; the Worley
// octaves are not Gaussian, and the continuation alone gives perimeters of D = 1.22 (x 1.5: 1.27, x 2: 1.31, x 2.5:
// 1.350, x 3: 1.37; box counting of test_cloud_optics "Cloud perimeters have the observed fractal dimension"). The
// factor makes the measured D Lovejoy's 1.35 - calibrated on the observable, not on a look.
inline constexpr double cloud_detail_amplitude_calibration = 2.5;
inline constexpr int cloud_detail_states = cloud_detail_levels + 1;

struct CloudNoise {
    int shape_size = 128, detail_size = 32, blue_noise_size = 64;
    std::vector<std::uint8_t> shape_rgba, detail_rgba;
    std::array<std::array<float, 256>, cloud_detail_states> coverage_threshold{};  // [state][code] (header)
    std::array<std::array<float, 256>, cloud_detail_states> edge_mean_factor{};    // [state][code] m (header)
    std::array<float, cloud_detail_levels> level_amplitude{};                      // a_k (header)
    std::array<float, 3> detail_weights{};                                         // of R, G, B in d (header)
    float detail_mean = 0.0f;                                                      // mean of d (header)
    float edge_width_min_value = 0.0f, edge_width_max_value = 0.0f;                // w_min, w_max (header)
    std::vector<std::uint8_t> blue_noise;  // blue_noise_size^2 R8, row-major
};

CloudNoise generate_cloud_noise();

// The Hurst exponent of the shapes, 2 - D (header).
double cloud_shape_hurst_exponent();

// The edge ramp (header): smoothstep of x, 0 for x <= 0, 1 for x >= 1.
double cloud_edge_ramp(double x);

// The edge width in shape-value units for the width noise n in [0, 1] (header).
double cloud_edge_width_value(const CloudNoise& noise, double width_noise);

// Weight of detail level k at the pixel footprint footprint_m, and the detail state L = sum of the weights of the
// first max_levels levels (header).
double cloud_detail_level_weight(int level, double footprint_m);
double cloud_detail_state(double footprint_m, int max_levels = cloud_detail_levels);

// GPU-style trilinear sample (wrapping, texel centres at integer + 0.5) of one channel of a size^3 RGBA8 texture at
// texel coordinates, in [0, 1].
double sample_noise_trilinear(const std::vector<std::uint8_t>& rgba, int size, int channel, const math::Vec3& texel);

// The detail value d (header) at detail-tile coordinates (1 = one tile; wraps).
double cloud_detail_value(const CloudNoise& noise, const math::Vec3& tile);

// The fractal shape value v (header) of the filtered shape value s at the isotropic position position_m [m] (the
// detail levels' coordinates position_m / T_k) for the detail state L.
double cloud_shape_value(const CloudNoise& noise, double shape_value, const math::Vec3& position_m,
                         double detail_state);

// The threshold t and the edge mean factor m of the state (interpolated between the rows at a fractional state, as
// the shader's linear filter) and the code of the cloud fraction.
double cloud_coverage_threshold(const CloudNoise& noise, double detail_state, int code);
double cloud_edge_mean_factor(const CloudNoise& noise, double detail_state, int code);

// Void-and-cluster blue-noise rank mask (header) of size x size, deterministic.
std::vector<std::uint8_t> generate_blue_noise(int size);

// The tileable primitives (coordinates in lattice / cell units, period in the same units; exact repetition with it).
double tileable_perlin(const math::Vec3& p, int period, std::uint32_t seed = 0u);  // about [-1, 1]
double tileable_worley(const math::Vec3& p, int period, std::uint32_t seed);       // W = 1 - min(1, F1), [0, 1]

}  // namespace starsim::environment
