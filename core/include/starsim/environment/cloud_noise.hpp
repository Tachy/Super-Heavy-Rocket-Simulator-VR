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
//   v = s + sum_k e_k(a_k (d_k - detail_mean)),  k = 0 .. cloud_detail_levels - 1,
// e_k(p) = p for p < 0, cloud_detail_lift[k] x p above (cloud_detail_erosion; P = sum_k a_k (d_k - mean) the signed
// sum),
// s the filtered shape R, d_k the detail value at the isotropic tile T_k = cloud_detail_tile_m / 4^k (cells of T_k / 2
// to T_k / 8: 500 m down to 8 m), a_k = A 4^(-H k) (the fBm gain per factor 4 in scale). A continues the shape's own
// spectrum: the horizontal increment variance of v at the lag cloud_detail_join_lag_m equals the shape's at its finest
// Perlin period r0 = 500 m scaled by fBm, Var(r0) (r / r0)^(2H) (increment variances of independent parts add), times
// cloud_detail_amplitude_calibration (calibration below).
// Erosion (task 5 of the static-clouds plan, user 2026-10-26: each cumulus is ONE connected body; fix round 1: with
// its cauliflower lobes): each level lowers the value at full strength but lifts it only by its share - level 0 (the
// 100 - 500 m bulges) by half, the finer levels by a quarter - so the detail erodes the shape's bodies from their rims
// inward, keeps their convex lobes and lifts few beads beside them. The shape's low octaves decide where a cloud is;
// the tables below are those of this value, so the covered fraction and the water stay. Measured (test_cloud_optics "A
// cumulus is one connected body": a 1.5 km cumulus layer at cf 0.3, detail level 0 / all levels): the signed detail P
// left 0.46 % / 0.48 % of the cloud volume in fragments under (200 m)^3 and 1.25 % / 1.89 % of the top view; the
// per-level lift 0.135 % / 0.122 % and 0.94 % / 0.90 %, perimeter dimension D = 1.278.
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
// w_min / w_max the median horizontal increment of the signed s + P (all levels) over those widths (the value change
// across a rim; the eroded v is flat where P is cut, its median would make every edge ~2.7 x steeper).
// Wide zones are the wispy fringes of thin, gradual transmittance, narrow ones the sharp cauliflower edges.
// m[L](cf) = E[ramp((v - t[L](cf)) / w(n))] over the joint (v, n) distribution of the state (binned at 1/2048 x 1/16),
// <= cf: the mean over the cell is the field's grid mean at every state and edge width (the "alpha" at the edge is real
// transmittance of less water, not a fudge).
//
// Shape prefilter (task 10 fix round 1, anti-aliasing): the shape value s is read without mipmaps (above), which is
// right while the shape's octaves span pixels; at a pixel footprint of kilometres (the Earth from ~8 600 km: ~7 km) its
// Perlin octaves (4 km to 500 m) are point-sampled - moire, speckle and a wrong local cover. Beyond the start footprint
// f_s (cloud_shape_prefilter_start_m) the shape is read from its box-filtered mip chain (shape_mips: 2 x 2 x 2 texel
// means, rounded, down to one texel) at the level
//   lambda(f) = clamp(log2(f / f_s), 0, 1) x log2(f / texel),  texel = tile / shape_size (250 m), at most
//   cloud_shape_prefilter_levels,
// the mips blended linearly in lambda (as the GPU's trilinear mip filter): the octaves finer than the pixel enter with
// their mean, as the detail levels' band limit does. lambda is 0 up to f_s and continuous (no pop); from 2 f_s on it is
// the standard mip level of the footprint. Against the level's own threshold prefilter_threshold[lambda](cf) (covered
// fraction cf, from uniform random positions of the blended value) and edge mean factor prefilter_edge_mean[lambda](cf)
// (the joint (v, n) distribution of the level): the covered area stays cf and the mean water the grid mean. Rows at
// lambda = r / cloud_shape_prefilter_rows_per_level, linear between them; row 0 is the state-0 row (the unfiltered
// shape). The detail is gone long before f_s (level 0 fades out at 125 m), so the prefilter acts on the shape alone.
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
// octaves are not Gaussian. With the signed detail the continuation alone gave perimeters of D = 1.22 (x 2.5: 1.350);
// the per-level lift (header) removes part of the rims' outward bumps: x 2.5 gives D = 1.278 with cloud_detail_lift
// 0.5 / 0.25 / 0.25 (1.291 with a lift of 0.25 on the summed detail, 1.267 with erosion only; box counting of
// test_cloud_optics "Cloud perimeters have the observed fractal dimension"), inside the test's 1.25 .. 1.45. Larger
// factors steepen the state-0 edge widths (the signed increments): x 3 broke the base contour of cloud_check section 9
// (VR 6 px vs 3) - calibrated on the observables, not on a look.
inline constexpr double cloud_detail_amplitude_calibration = 2.5;
inline constexpr int cloud_detail_states = cloud_detail_levels + 1;
// [E] Share of each detail level's upward part that is kept (header, erosion; static clouds Task 5 fix round 1).
// Level 0 (cells 500 .. 125 m: the 100 - 500 m bulges, the cauliflower lobes on rims and tops) keeps half, so a body
// keeps its convex relief; levels 1 and 2 (the bead scale, ~1 march pixel at a few km) keep a quarter, so they lift
// few beads beside a body; their downward part erodes it at full strength from the rim inward. Measured (test "A
// cumulus is one connected body", cf 0.3, states 1 / 3): fragments 0.135 % / 0.122 % of the volume, 0.94 % / 0.90 %
// of the top view; level 0 at 0.6 / 0.8: 1.10 % of the top view at state 1 (bound 1 %), volume 0.19 % / 0.33 %.
// 0 everywhere would leave the rims where the detail points up on the shape's own gradient alone (soft: steepness p90
// 41 m).
inline constexpr std::array<double, cloud_detail_levels> cloud_detail_lift = {0.5, 0.25, 0.25};
// Level k's contribution to the shape value for its signed term p_k (header): p_k below 0, cloud_detail_lift[k] x p_k
// above.
inline constexpr double cloud_detail_erosion(int level, double p) {
    return p < 0.0 ? p : cloud_detail_lift[static_cast<std::size_t>(level)] * p;
}

// Cloud types (spec decision 5; ERA5 / GFS carry none, so the type follows from the layer data at the point): ice-
// dominated cloud is cirrus (thin sheets: the shape tile cloud_ice_tile_vertical_m high, no fibre direction - the
// snapshot has no wind, user decision 2026-10-25); a deck (cloud fraction >= cloud_stratiform_fraction that does not
// thicken upward) is stratiform (stratus / stratocumulus: the shape tile stretched horizontally by
// cloud_stratiform_stretch - a flat sheet with an undulating top and few, wide gaps); everything else is cumulus (the
// shape tile itself, its low octaves' lobes, eroded from the rim by the detail). The stretches are affine maps of the
// shape coordinates, the detail stays isotropic: the value statistics and so the coverage and water tables are the
// same for every type (test "Coverage and water are kept for every cloud type").
// The types are weights, not a switch (fix round 1: no seam, no pop - spec anti-goals): each rule passes through a
// smoothstep band, and the shader mixes the types' edge ramps by the weights (each ramp has the table mean m, so the
// mix keeps the water; the covered area grows inside a band, where two patterns overlap). The stratiform weight
// belongs to a layer centre (its fraction and that of the centre above); between the centres around a point it is
// their mean weighted by (1 - x) and x times each centre's fraction - the type of the cloud that is there: under a
// deck's base centre the empty centre below has no say (a plain linear mix made that interval half cumulus and moved
// cloud_far_check's deck luminance from 9.1 % to 11.1 %). Like every field value it is blended between the snapshots:
// continuous in position and in time.
// [E] Ice share of the extinction above which the cloud is cirrus: the majority phase; the band half-width.
inline constexpr double cloud_cirrus_ice_share = 0.5;
inline constexpr double cloud_cirrus_ice_band = 0.1;
// [E] Cloud fraction from which a layer is a deck: stratus / stratocumulus cover their grid cell almost fully (broken
// cumulus fields stay below; the 0.9 of the spec); the band half-width.
inline constexpr double cloud_stratiform_fraction = 0.9;
inline constexpr double cloud_stratiform_fraction_band = 0.05;
// [E] Thickening upward (fraction above minus fraction) over which a deck turns convective (cumulus): from 0 to this.
inline constexpr double cloud_stratiform_thickening_band = 0.05;
// [E] Horizontal stretch of the stratiform shape tile: 4 x 32 km across at the cumulus tile's 16 km height - features
// 8 x wider than deep (decks are an order of magnitude wider than thick; their gaps are kilometres wide).
inline constexpr double cloud_stratiform_stretch = 4.0;
// [E] Vertical shape tile of cirrus (the shader's CLOUD_ICE_TILE_V_M): 2 km instead of 16 (cirrus and altostratus are
// layered, often a few hundred metres thick).
inline constexpr double cloud_ice_tile_vertical_m = 2000.0;

enum class CloudType { Cumulus, Stratiform, Cirrus };

struct CloudTypeWeights {
    double cumulus = 1.0, stratiform = 0.0, cirrus = 0.0;  // sum 1
};

// The stratiform weight of a layer centre (above) with the cloud fraction fraction and fraction_above that of the
// centre above it: smoothstep across cloud_stratiform_fraction +- band times 1 - smoothstep of the thickening.
double cloud_stratiform_weight(double fraction, double fraction_above);
// The stratiform weight at x in [0, 1] between the centre below a point (fraction_below) and the one above it
// (fraction_above), fraction_above_next that of the centre above that one: the two centres' weights, weighted by
// (1 - x) fraction_below and x fraction_above (above; 0 where both are empty).
double cloud_stratiform_weight_between(double fraction_below, double fraction_above, double fraction_above_next,
                                       double x);
// The type weights at a point (above) of the stratiform weight there (interpolated between the centres) and the ice
// share of the extinction: cirrus = smoothstep across cloud_cirrus_ice_share +- band, stratiform the rest times the
// stratiform weight, cumulus what remains.
CloudTypeWeights cloud_type_weights(double stratiform_weight, double ice_share);

struct CloudNoise;

// The shape texel coordinates of the isotropic position [m] for the type's tile (cumulus: the shape tile; stratiform:
// stretched horizontally; cirrus: the ice tile).
math::Vec3 cloud_shape_texel(const CloudNoise& noise, CloudType type, const math::Vec3& position_m);
// [E] Shape prefilter (header): the start footprint f_s - the largest that still filters the Earth seen from the
// scaled-space limit fully (fully filtered from 2 f_s; there the march pixel is >= 7 km: 8 600 km at 8e-4 rad and
// more), above the footprints from the ground and an ascent below ~30 km (the farthest cloud sample ~1 100 km away at
// the 1600 x 900 check's 3.1e-3 rad half-resolution pixel: ~3.4 km). Views under it read the unfiltered shape exactly
// (ruling 2026-10-26: shapes unchanged at normal distances); higher views (the limb from an ascent or from orbit) are
// prefiltered where their footprint exceeds f_s - there the shape aliases as at the hand-over. The highest level
// (2: 32^3 texels of 1 km; deeper mips keep only a few features per horizontal slice of the tile, and a cloud layer
// seen from afar cuts many slices: their cover, off cf, showed as rings around the sub-camera point and moved the
// cf 0.4 hand-over to 10.7 % with level 4, 7.5 % with 2 - beyond the level the shader's sub-pixel offsets average the
// footprint, clouds.gdshaderinc) and the table rows per level (quarter levels: the blend of two mips is not the mean of
// their tables - covered fraction / water 2.3 % / 36 % off at 4.5 with whole-level rows).
inline constexpr double cloud_shape_prefilter_start_m = 3500.0;
inline constexpr int cloud_shape_prefilter_levels = 2;
inline constexpr int cloud_shape_prefilter_rows_per_level = 4;
inline constexpr int cloud_shape_prefilter_rows = cloud_shape_prefilter_levels * cloud_shape_prefilter_rows_per_level + 1;

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
    // Shape prefilter (header): mip levels 1 .. log2(shape_size) of shape_rgba (sizes shape_size / 2 .. 1), and the
    // threshold and edge mean factor per row [row][code] (row 0 = state 0).
    std::vector<std::vector<std::uint8_t>> shape_mips;
    std::array<std::array<float, 256>, cloud_shape_prefilter_rows> prefilter_threshold{};
    std::array<std::array<float, 256>, cloud_shape_prefilter_rows> prefilter_edge_mean{};
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

// Shape prefilter (header): the level lambda at the pixel footprint [m] for a shape texture of shape_size texels; one
// channel of the shape at level-0 texel coordinates, the mips blended linearly in the level (level 0: exactly
// sample_noise_trilinear of shape_rgba); the level's threshold and edge mean factor of a code (linear between rows).
double cloud_shape_prefilter_level(double footprint_m, int shape_size = 128);
double sample_shape_prefiltered(const CloudNoise& noise, int channel, const math::Vec3& texel, double level);
double cloud_prefilter_threshold(const CloudNoise& noise, double level, int code);
double cloud_prefilter_edge_mean(const CloudNoise& noise, double level, int code);

// Void-and-cluster blue-noise rank mask (header) of size x size, deterministic.
std::vector<std::uint8_t> generate_blue_noise(int size);

// The tileable primitives (coordinates in lattice / cell units, period in the same units; exact repetition with it).
double tileable_perlin(const math::Vec3& p, int period, std::uint32_t seed = 0u);  // about [-1, 1]
double tileable_worley(const math::Vec3& p, int period, std::uint32_t seed);       // W = 1 - min(1, F1), [0, 1]

}  // namespace starsim::environment
