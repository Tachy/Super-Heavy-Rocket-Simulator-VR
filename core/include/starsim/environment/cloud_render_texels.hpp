#pragma once

// The GPU form of a weather snapshot (M8 weather and clouds, Task 8): what game/shaders/clouds.gdshaderinc samples
// (change both together). Same grid and layout as WeatherSnapshot::texels() (32 layers x rows x columns RGBA8,
// layer-major), different channels:
//   R: cloud fraction, the snapshot's code (code / 255);
//   G, B: the grid-mean extinction [1/m] of liquid + ice (cloud_extinction_per_m of the decoded water contents) as one
//         16-bit LINEAR number, G the high and B the low byte: beta = (256 G + B) x cloud_texel_extinction_step_per_m().
//         Hardware (bi/tri)linear filtering of the two UNORM8 channels is linear in that number, so the GPU interpolates
//         the extinction exactly as WeatherSnapshot::at() interpolates the water contents (the log water codes of the
//         cache form would interpolate geometrically: a 250 m ramp at a deck edge would lose about half its depth).
//         Full scale: the extinction of both codes at their maximum (weather_water_code_max_kg_m3 liquid + ice,
//         0.818 /m); step 1.25e-5 /m (an optical depth of 0.0125 per km). Liquid and ice share the phase function
//         (cloud_optics.hpp), so the shader needs only their summed extinction.
//   A: the IN-CLOUD optical depth from the column top (20 km) down to the layer centre, log-coded (code 0 = none, code c
//      in 1..255 = min x (max / min)^((c - 1) / 254), min 0.01, max 1e4: half a code step <= 2.8 %). In-cloud
//      extinction at a centre = grid mean / cloud fraction (0 where the fraction is 0: no cloud is drawn there, the
//      coverage rule of cloud_noise.hpp), linear (trapezoids) between consecutive centres, held from the top centre to
//      20 km. The shader takes the depth of a point from the centre above it plus the trapezoid down to the point, and
//      the column (the diffuse field's tau*) from layer 0; maximum overlap: the cloudy parts of the layers lie in one
//      column (the noise is vertically coherent). Layers below the ground are zero in the snapshot and add nothing.
// Ice share (task 8 amendment, cloud types): a second texture, R8 of the same grid: ice extinction / (liquid + ice)
// extinction x 255 (0 where there is none). The shader shapes ice-dominated cloud as thin sheets (clouds.gdshaderinc).
// Linear filtering mixes the share with clear neighbours' 0 at the edge of an ice cloud: the shapes blend there.
//
// Vertical profile rule (task 8 amendment, cloud types): between two layer centres (value v_b below, v_a above, cloud
// fractions f_b, f_a, normalised position x in [0, 1] over the gap) the shader does not take the linear interpolation
// alone but
//   v(x) = (1 - x) v_b + x v_a + b (step(x) - x) max(v_a - v_b, 0),   b = clamp(1 - f_b / f_a, 0, 1)^2,
// step(x) a linear ramp from 0 to 1 over cloud_base_transition_m centred on the middle of the gap. Where a cloud starts
// above a clear layer (f_b = 0: b = 1) that is a flat, sharp base in the middle of the gap (convective bases at the
// lifting condensation level are flat); where the cloud only thickens upward (0 < f_b < f_a) partly so; elsewhere
// (thinning upward, the tops) the linear taper of the field stays, so the covered area shrinks toward a top: rounded
// tops (cumuliform) out of the same noise. Stratiform decks (cloud fraction ~1 through the deck) stay layered, flat
// based. The term integrates to zero over the gap (step and x both have the mean 1/2): the column's water path and
// cloud amount are the field's. Applied to the cloud fraction and to the grid-mean extinction with the same b.

#include <cstdint>
#include <vector>

#include "starsim/environment/weather_snapshot.hpp"

namespace starsim::environment {

// The vertical profile rule (header) at x in [0, 1] of a gap of gap_m between two layer centres.
double cloud_profile_value(double value_below, double value_above, double fraction_below, double fraction_above,
                           double x, double gap_m);

// The ice share texture (header): rows x columns x 32 layers R8, layer-major like the texels.
std::vector<std::uint8_t> cloud_render_ice_texels(const WeatherSnapshot& snapshot);

inline constexpr double cloud_depth_code_min = 0.01;
inline constexpr double cloud_depth_code_max = 1e4;
std::uint8_t encode_cloud_optical_depth(double optical_depth);
double decode_cloud_optical_depth(std::uint8_t code);

// Extinction per unit of the 16-bit number in G, B (full scale / 65535) and the decode of one texel's two bytes.
double cloud_texel_extinction_step_per_m();
double cloud_texel_extinction_per_m(std::uint8_t high, std::uint8_t low);

// Column texture (Task 9, light under the clouds): RG8 of the same grid. R: the GRID-MEAN optical depth from the column
// top (20 km) down to the layer centre (grid-mean extinction at the centres, trapezoids between them, held from the
// top centre to 20 km - cloud_column_depths_above at the centre), coded as the A channel (encode_cloud_optical_depth);
// G: the largest cloud fraction code from the top down to the centre (the column's maximum-overlap cover). The shader
// takes a point's column from the centre above plus the trapezoid of the grid-mean extinction down to it, its cover the
// larger of the centre's and the point's fraction: the sky factor and the Sun's diffuse light (cloud_optics.hpp).
std::vector<std::uint8_t> cloud_render_column_texels(const WeatherSnapshot& snapshot);

// Base texture (static clouds Task 7: cumulus bases at the lifting condensation level): RG32F, rows x columns, one
// texel (two floats) per cell, no layers. R: the cell's flat cloud base, ground + lifting condensation level as a
// height above the ELLIPSOID (m; the shader knows heights above the ellipsoid); G: the extinction scale of the base's layer
// (cloud_lcl_extinction_scale, >= 1), 0 = no base, the cell's cloud is not cut - the level is unknown, saturated (fog)
// or not below cloud_boundary_layer_top_m, or condensate (liquid or ice) remains in a layer wholly under the layer that
// holds the base (the snapshot build moved the liquid but not the ice: ice under the level keeps the column uncut).
// Read with nearest texel fetches (a base is not an interpolable number across a "none" neighbour).
std::vector<float> cloud_render_base_texels(const WeatherSnapshot& snapshot);

// The flat base's cut of the cloud FRACTION at height_m over a base at base_m: 0 below it, a linear ramp up to 1 over
// cloud_lcl_ramp_m, 1 above - and 1 from the top of the layer that holds the base on (the ramp ends there, so the cut
// stays inside one layer; NaN base: 1).
double cloud_lcl_cut_factor(double height_m, double base_m);
// The rendered share of the grid-mean EXTINCTION at height_m: the cut times scale inside the base's layer, 1 above
// (NaN base or scale <= 0: 1).
double cloud_lcl_extinction_factor(double height_m, double base_m, double scale);
// The scale that keeps a cell's rendered column: the integral of the renderer's grid-mean extinction profile (the
// vertical profile rule between the layer centres, as the shader's cloud_snapshot_cell at the column) over the
// above-ground part of the base's layer, divided by the integral of that profile times the cut (Simpson's rule on the
// pieces between their kinks, steps of at most cloud_lcl_ramp_m / 8: exact up to rounding); 1 where the layer holds no
// cloud above the base. The layer's stored mean is its water over the whole layer (the snapshot build keeps the part
// above the level as such a mean), so what the cut removes under the base is put back above it, in proportion to the
// profile there.
double cloud_lcl_extinction_scale(const WeatherSnapshot& snapshot, int row, int column, double base_m);

// The grid-mean vertical optical depth of a cell from its ground to 20 km as the snapshot's layers hold it: the sum of
// each layer's decoded extinction (the render texels' 16-bit code) times its above-ground thickness (checks: the
// rendered column of the flat base against the core's).
double cloud_render_column_optical_depth(const WeatherSnapshot& snapshot, int row, int column);

// The GPU form of the snapshot's texels (layout above).
std::vector<std::uint8_t> cloud_render_texels(const WeatherSnapshot& snapshot);

// The height above which the rendered field holds no cloud anywhere (static clouds Task 9b: the cloud shadows on the
// air need no samples above it): the centre of the layer above the highest layer with a cloud fraction and condensate
// in any cell (the profile between the centres reaches zero there), the field's top weather_top_m for the top layer
// (held to it), 0 without cloud.
double cloud_render_top_m(const WeatherSnapshot& snapshot);

}  // namespace starsim::environment
