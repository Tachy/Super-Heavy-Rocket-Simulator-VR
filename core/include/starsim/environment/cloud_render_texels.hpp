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

// Column texture (Task 9, light under the clouds; Task 7, the cloud base): RGBA8 of the same grid. R: the GRID-MEAN
// optical depth from the column top (20 km) down to the layer centre (grid-mean extinction at the centres, trapezoids
// between them, held from the top centre to 20 km - cloud_column_depths_above at the centre), coded as the A channel
// (encode_cloud_optical_depth); G: the largest cloud fraction code from the top down to the centre (the column's
// maximum-overlap cover). The shader takes a point's column from the centre above plus the trapezoid of the grid-mean
// extinction down to it, its cover the larger of the centre's and the point's fraction: the sky factor and the Sun's
// diffuse light (cloud_optics.hpp). B, A (layer 0 only, zero above): the 16-bit base of the cell's boundary-layer cloud,
// the lifting condensation level as a height above the ELLIPSOID (ground + lcl_m; the shader knows heights above the
// ellipsoid), B the high and A the low byte of encode_cloud_lcl_base; 0 = none (fog, unknown level, a level not below
// cloud_boundary_layer_top_m: the cell's cloud is not cut). Read with nearest texel fetches (a code is not an
// interpolable number across a "none" neighbour). The shader multiplies the cloud fraction and the extinction by
// cloud_lcl_cut_factor, the flat base of cumulus (the cloud above the level was lifted there by the snapshot build).
std::vector<std::uint8_t> cloud_render_column_texels(const WeatherSnapshot& snapshot);

// The 16-bit code of a cloud base height above the ellipsoid: 0 = none, else 1 + round of (height + offset) / span x 65534
// clamped (offset 500 m, span 21 km: 0.32 m per code, covers the geoid, the highest ground + 3 km and the 20 km top).
inline constexpr double cloud_lcl_code_offset_m = 500.0;
inline constexpr double cloud_lcl_code_span_m = 21000.0;
std::uint16_t encode_cloud_lcl_base(double base_height_m);
// The base height of a code; NaN for 0.
double decode_cloud_lcl_base(std::uint16_t code);

// The share of the cloud that remains at height_m over a flat base at base_m: 0 below it, a linear ramp up to 1 over
// cloud_lcl_ramp_m, 1 above (NaN base: 1).
double cloud_lcl_cut_factor(double height_m, double base_m);

// The GPU form of the snapshot's texels (layout above).
std::vector<std::uint8_t> cloud_render_texels(const WeatherSnapshot& snapshot);

}  // namespace starsim::environment
