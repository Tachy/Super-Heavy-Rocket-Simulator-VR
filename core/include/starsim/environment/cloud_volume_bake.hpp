#pragma once

// The cloud world volume's per-cell density bake, CPU reference (world-volume plan Task 4; spec section A "Baked
// density"): the GPU bake (game/shaders/cloud_volume_bake.gdshaderinc) is checked against it - change both together.
//
// A cell is the parallelepiped centre + u e_x + v e_y + w e_z, u, v, w in [-1/2, 1/2] (e_x, e_y the horizontal edges,
// e_z the vertical one, Earth-local render axes as cloud_render_field.hpp). Its bake averages the point model
// (cloud_render_field.hpp, no detail) over taps at the centres of an n_h x n_h x n_v grid of sub-boxes. Stored:
//   rho     = the mean of the taps' extinction (the cell's band-limited mean extinction);
//   sigma_c = the mean of the taps' in-cloud extinction (grid mean / edge mean factor m: the extinction where the edge
//             ramp is 1), so 0 <= rho <= sigma_c and rho / sigma_c is the cell's cloud amount (in [0, 1]);
// both 0 in a clear cell. (Grid mean / cf, the column terms' "in-cloud", is smaller than a cloud interior's
// grid mean / m - m < cf - and a cell inside a cloud would have rho > sigma_c.)
//
// Tap rule (cloud_cell_bake_taps; controller ruling T4 Q2, measured on a cf 0.4 slab against a 16^3 stratified point
// mean at a 10 m footprint, test_cloud_volume_bake.cpp):
//   n_h = clamp(ceil(h / cloud_bake_horizontal_spacing_m), cloud_bake_min_horizontal_taps, cap(h)),
//   cap = cloud_bake_max_horizontal_taps (cloud_bake_widest_cell_horizontal_taps beyond cloud_bake_widest_cell_m),
//   n_v = max(cloud_bake_min_vertical_taps, ceil(v / 5 m)),
//   lambda = clamp(log2((h / n_h) / shape texel), 0, cloud_shape_prefilter_levels)
// (the taps' spacing beyond the shape's 250 m texel is band-limited with the box-filtered mips and their prefilter
// tables, the far march's band limit). Per level (h x v -> taps, lambda): 50 x 25 and 100 x 50 -> 4 x 4 x 16 = 256;
// 200 / 400 / 800 x 164 -> 4 x 4 x 33 = 528; 1600 -> 8 x 8 x 33 = 2112; 3200 -> 2112 at lambda 0.68; 6400 ->
// 16 x 16 x 33 = 8448 at lambda 0.68.
// - The vertical taps matter most: the 50 m base transition, the 30 m flat-base ramp and the 20-40 m edges (4 taps up a
//   164 m cell missed the dense mean by 0.1 sigma_c, 16 by up to 0.03, 33 by 0.011).
// - Accuracy tiers: cells up to cloud_bake_dense_cell_max_m (levels 0-2) are the point model's cell mean within
//   0.02 sigma_c. Coarser cells (levels 3-7) are a deliberate band-limit simplification: their mean is held to the
//   point model's water at weather-cell scale (0.25 deg: within 0.4 %; 4 x 4 taps at the mips' band limit missed it by
//   3 %, 8 x 8 at level 7 by 0.3 %, hence its 16 x 16), not per cell - there a cell scatters about the dense mean
//   (rms <= 0.017 sigma_c, max 0.07 at level 6; cumulus and cirrus slabs). The mips' tables keep the cover and
//   water statistically, not per cell (a cell's mean of the thresholded unfiltered field is not a function of its
//   box-filtered shape value); coarse cells are seen from 100 km and more at about a pixel's size, where an unbiased
//   scatter is texture, not a coverage error.

#include <span>
#include <vector>

#include "starsim/environment/cloud_noise.hpp"
#include "starsim/environment/cloud_render_field.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// [E] Tap rule (header, measured): a tap every 200 m across (4 to 8 per axis, 16 at 6.4 km), every 5 m up (at
// least 16); the dense accuracy tier ends at 200 m cells (level 2).
inline constexpr double cloud_bake_horizontal_spacing_m = 200.0;
inline constexpr int cloud_bake_min_horizontal_taps = 4;
// The cap per horizontal axis: 8, and 16 for cells wider than cloud_bake_widest_cell_m (level 7, 6.4 km; the threshold
// lies between levels 6 and 7 so a rounded edge length cannot flip it): with 8 the level-7 taps sit at the mips'
// lambda 1.68 and the weather-cell water missed by 0.3 %; level 6 keeps it with 8 at lambda 0.68.
inline constexpr int cloud_bake_max_horizontal_taps = 8;
inline constexpr double cloud_bake_widest_cell_m = 4800.0;
inline constexpr int cloud_bake_widest_cell_horizontal_taps = 16;
inline constexpr double cloud_bake_vertical_spacing_m = 5.0;
inline constexpr int cloud_bake_min_vertical_taps = 16;
inline constexpr double cloud_bake_dense_cell_max_m = 200.0;
// [E] Relative slack of the tap counts: a cell up to 1 % longer than its level's nominal size keeps the level's taps
// (real edges differ by up to ~0.4 %: the AEQD's azimuthal scale, the height above the projection sphere).
inline constexpr double cloud_bake_tap_count_slack = 0.01;

struct CloudCellBox {
    math::Vec3 centre_earth_m;
    math::Vec3 edge_x_m, edge_y_m, edge_z_m;  // full edge vectors
};

// The box of a cell of horizontal size horizontal_m and vertical size vertical_m at centre_earth_m: e_x east, e_y north
// (of the geodetic normal there; e_x = the +X axis at a pole), e_z the geodetic normal.
CloudCellBox cloud_cell_box(const math::Vec3& centre_earth_m, double horizontal_m, double vertical_m);

// Taps per axis and the prefilter level of each tap (header).
struct CloudBakeTaps {
    int horizontal = 1, vertical = 1;
    double prefilter_level = 0.0;
};

// The tap rule (header) for a cell horizontal_m across; shape_size the shape texture's texels per axis.
CloudBakeTaps cloud_cell_bake_taps(double horizontal_m, double vertical_m, int shape_size = 128);

struct CloudCellBake {
    double mean_extinction_per_m = 0.0;      // rho
    double in_cloud_extinction_per_m = 0.0;  // sigma_c
};

CloudCellBake bake_cloud_cell(const CloudRenderField& field, const CloudNoise& noise, const CloudShapeFrame& frame,
                              const CloudCellBox& box, const CloudBakeTaps& taps);
// With the tap rule of the box's horizontal size (the longer of |e_x|, |e_y|).
CloudCellBake bake_cloud_cell(const CloudRenderField& field, const CloudNoise& noise, const CloudShapeFrame& frame,
                              const CloudCellBox& box);
CloudCellBake bake_cloud_cell(const CloudRenderField& field, const CloudNoise& noise, const CloudShapeFrame& frame,
                              const math::Vec3& centre_earth_m, double cell_horizontal_m, double cell_vertical_m);

// Many cells (tap rule per box) in parallel on the build threads (parallel_rows.hpp).
std::vector<CloudCellBake> bake_cloud_cells(const CloudRenderField& field, const CloudNoise& noise,
                                            const CloudShapeFrame& frame, std::span<const CloudCellBox> boxes);

}  // namespace starsim::environment
