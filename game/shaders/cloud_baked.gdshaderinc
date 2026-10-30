// The baked cloud volume ("Weg 2", 2026-10-29; the clouds' only march since 2026-10-30): the clouds' shape and light
// precomputed once in a world-fixed volume around an anchor (the camera when the bake started) - static clouds, independent of the view direction - and
// each frame a cheap march through it. GLSL 450 only - compiled by CloudBaked (game/world/weather/cloud_baked.gd) after
// CloudEffect's march files (cloud_pass.gdshaderinc's main calls cloud_baked_shade), never included by a Godot shader.
//
// Grid: K nested levels (a clipmap) of N x N columns about the anchor, level k with the horizontal cell c0 2^k and the
// vertical cell dz_k over the band [z_lo, z_hi]. Horizontal coordinates: x = q . axis_x, y = q . axis_y of the
// model-space point q (Earth-local x ATM_STRETCH: the ellipsoid a sphere of radius a), with axis_x / axis_y unit and
// perpendicular to the anchor's up - linear along a straight ray; vertical: the model height |q| - a. All levels are
// stacked along one 3-D texture's depth, each between two guard slices (no cloud).
// Volumes (each set):
//   A  RG16F   x: the signed edge coordinate - (shape value + detail - threshold) / edge width: the clouds' soft edge
//              ramp (clouds.gdshaderinc) is smoothstep(clamp(x, 0, 1)); interpolated trilinearly it keeps the silhouette crisp at sub-cell
//              precision; y: the extinction scale (1/km): extinction = y x ramp (cell extinction / edge mean)
//   B  RGBA8   sqrt-coded light: x the Sun's transmittance T_s (MC model exp(-tau_s); closed form exp(-k tau_s)), y the
//              Sun's multiple-scattering source J_ms / J_MAX, z the sky's source J_sky / J_MAX (both for the view from
//              the anchor), w unused
//   O  R8      occupancy per block of 4 x 4 x 4 cells (the block or a neighbouring cell may hold cloud): the empty skip
//   S  RGBA16F 2-D array, one layer per volume level at CLOUD_BAKED_SHADOW_SUPERSAMPLE x the columns (shadow step,
//              2026-10-30): the line toward the Sun from the band's floor at each column through the baked shapes (the
//              field's grid mean beyond the levels): x T = exp(-tau), y tau (clamped), z / w tau x the lowest / highest
//              cloudy height on the line (km; premultiplied: a clear neighbour does not move them). The cloud shadow
//              maps (CLOUD_SHADOW_BAKED_FILL, CloudShadowEffect), the air shadow (cloud_baked_air_sun) and the Sun probe
//              (CLOUD_BAKED_SUN_PROBE: the vehicles' and the eye's direct Sun) read it.
// The phase function is applied per frame (per ray: one cosine), J = P(theta) T_s + J_ms.
// Flat levels (2.5D, 2026-10-29; step 3): levels m0, m0 + 1, ... of N_f x N_f columns (N_f up to CloudBaked
// FLAT_COLUMNS_MAX), each doubling the cell (c0 2^m), over only the spherical cap the camera can see (the far side is
// never baked or fetched). Below 40 km m0 = K (6.4 km: beyond the volume); above, m0 is the footprint's level at the
// nearest cloud, so from orbit the footprint, not a level's extent, picks the cell (a level holds 2 / pixel angle
// columns), and the flat levels take over from a volume sample whose level the clipmap's extent made coarser
// (cloud_baked_flat_weight_at). A level's columns no camera of the geometry's height range can fetch are not baked.
// Coordinates: azimuthal equidistant about flat_up0, (X, Y) = a theta (unit tangent . axis_x, . axis_y), theta the
// angle from up0 (any cap up to the whole globe). Per column (RGBA16F 2-D arrays, one layer per flat level):
//   F  x: the pattern P (the cell's share of shape taps with cloud), y: the grid-mean vertical depth, z / w: the far
//         field's cloud height (km) for a nadir / a grazing view x the depth (premultiplied: a clear neighbour does not
//         lower them; CLOUD_BAKED_FLAT_GRAZING_MU)
//   T  x: the column's nadir cover C_col, y: the optical top x C_col (both shared by the two sets)
// Per pixel (step 4, 5): F and T reconstructed by the cubic B-spline (no cell drawn as a square), the point where the
// ray crosses the far field's height for its view (up to four heights-only fetches, then the full one);
// the far field's cover / opacity rule (slant beam cover, alpha = C_v (1 - exp(-tau / mu))); the shapes' prefiltered
// pattern of that cover at the pixel's footprint (anti-aliased edges one pixel wide at every distance; two readings
// blended: no repeating tile) with the volume's edge ramp as the depth inside the shapes; the column-model light
// evaluated at the pixel for its own view (cloud_far.gdshaderinc: plane-parallel slab, Monte Carlo table, the shapes'
// 3D gain) with that depth.
// Kernels: CLOUD_BAKED_BAND (the field's lowest and highest cloudy layer over the region: the band),
// CLOUD_BAKED_DENSITY (A), CLOUD_BAKED_OCCUPANCY (O), CLOUD_BAKED_LIGHT (B; marches toward the Sun through A),
// CLOUD_BAKED_FLAT (F and T of one flat level, a range of rows), CLOUD_BAKED_SHADOW (S of one volume level, a range
// of rows), CLOUD_BAKED_MARCH (with CLOUD_MARCH; CLOUD_BAKED_FADE: two sets marched and cross-faded - a re-bake's swap;
// CLOUD_BAKED_FAR_FALLBACK: the far field beyond the flat cap), CLOUD_SHADOW_BAKED_FILL (the shadow maps; with
// CLOUD_BAKED_SUN_PROBE the Sun probe instead).
// The code, in this order (each part's own header says what it holds):

#include "res://shaders/cloud_baked_grid.gdshaderinc"
#include "res://shaders/cloud_baked_shape.gdshaderinc"
#include "res://shaders/cloud_baked_sun.gdshaderinc"
#include "res://shaders/cloud_baked_bake.gdshaderinc"
#include "res://shaders/cloud_baked_light_bake.gdshaderinc"
#include "res://shaders/cloud_baked_shadow_bake.gdshaderinc"
#include "res://shaders/cloud_baked_flat_bake.gdshaderinc"
#include "res://shaders/cloud_baked_volume.gdshaderinc"
#include "res://shaders/cloud_baked_flat.gdshaderinc"
#include "res://shaders/cloud_baked_march.gdshaderinc"
#include "res://shaders/cloud_baked_fill.gdshaderinc"
