// Clouds from the weather field (M8 weather and clouds, Task 8): the field lookup with the time blend and the vertical
// profile rule, the height layers, the coverage-preserving sub-grid shapes with soft edges, the extinction, the light
// march toward the Sun and the scattering sources - the GPU port of core/include/starsim/environment/cloud_optics.hpp,
// cloud_noise.hpp, cloud_render_texels.hpp and weather_snapshot.hpp (change them together). Include
// atmosphere.gdshaderinc first (the floating-origin globals atmosphere_camera_hi/lo).
// Written in the common subset of the Godot shading language and GLSL 450: the cloud pass (CloudEffect) compiles it as
// a compute shader (CloudShaderPort turns the `global uniform` lines into a uniform block and bindings).
//
// Frames and numerics: positions are Earth-local render axes (ITRF (x, y, z) -> (y, z, x), polar axis +Y), metres.
// Points near the camera are passed camera-relative (offset_local = point - camera), exact in float32; heights come from
// the camera's height cloud_camera_height_m (float64 on the CPU) plus the offset in the stretched model space (the
// WGS84 ellipsoid -> the sphere of radius a, as atmosphere.gdshaderinc): height = |M p| - a, within 0.34 % of the
// geodetic height (68 m at 20 km against 250 m / 1 km layers). Latitude: geodetic from tan(phi) = (a/b)^2 z / s (exact
// on the ellipsoid, < 1e-5 rad off at 20 km); longitude atan2.
//
// Field (CloudLayer uploads the GPU form of two snapshots, cloud_render_texels.hpp): x = longitude column (wraps),
// y = latitude row (+90 deg at row 0, -90 deg at the last row, both inclusive), z = height layer; RGBA8: R cloud
// fraction, G/B the 16-bit linear grid-mean extinction, A the log-coded in-cloud optical depth from 20 km down to the
// layer centre; a second R8 texture the ice share of the extinction. Values belong to texel centres (layer centres in
// height). Horizontally the hardware filters bilinearly; vertically the two centres around a point are fetched apart
// and joined by the vertical profile rule (cloud_profile_value: flat bases, the water path kept); below the centre of
// layer 0 and above that of layer 31 the end values hold (the core's at()), zero above 20 km. The two snapshots may
// differ in size (the 1-deg analog beside a 0.25-deg ERA5 hour). Cloud bases at the lifting condensation level
// (cloud_lcl_cut; static clouds Task 7): the snapshot build has moved a column's boundary-layer condensate above its
// level; the fraction and the extinction are also cut with a ramp of CLOUD_LCL_RAMP_M over the level's flat base (the
// column texture's layer 0), so cumulus bases are flat and sharp and nothing hangs below them.
//
// Sub-grid shapes (cloud_noise.hpp): the fractal value v = s + sum_k e_k(w_k a_k (d_k - mean(d))) - the shape
// noise's R channel s, sampled TRILINEARLY WITHOUT MIPMAPS (what the tables assume), eroded by the detail levels k
// (e_k(p) = p below 0, cloud_level_lift[k] x p above: the detail erodes a body from its rim, level 0 keeps its lobes
// and the finer levels lift few beads beside it; isotropic tiles
// CLOUD_DETAIL_TILE_M / 4^k, fBm gain: the perimeters have Lovejoy's D = 1.35), each band-limited by the pixel
// footprint f (w_k = clamp(log2(l_k / f) - 1, 0, 1), l_k = tile / 4: an octave below the pixel enters with its mean 0,
// so the silhouette stays irregular down to the pixel at every distance) - against the threshold t[L](cf) of the detail
// state L = sum w_k: density = mean / m[L](cf) x ramp((v - t) / w(n)) (the soft edge ramp, mean-preserving at every
// state: m = edge mean factor table; the edge width w(n) = w_min (w_max / w_min)^n varies along the perimeter by the
// width noise n of the shape's A channel - wispy fringes and sharp edges); the covered area fraction stays cf. The
// tables are 256 x states textures, linear between the state rows. The cloud types (core cloud_noise.hpp
// cloud_type_weights: cirrus where ice dominates, stratiform for a deck that does not thicken upward, else cumulus)
// are smooth weights; each type reads the shape at its coordinates - cumulus the shape tile, stratiform the tile
// stretched horizontally by CLOUD_STRATIFORM_STRETCH, cirrus thin sheets (vertical tile CLOUD_ICE_TILE_V_M) - with the
// same detail and tables (affine maps keep the value statistics), and the types' edge ramps are mixed by the weights
// (each has the mean m: the water is kept; no seam, no pop). No detail beyond the footprint: the same optical depth
// (no pop). Shape coordinates
// are anisotropic (S_H across, S_V up: q = p / S_H + up(p) h(p) (1 / S_V - 1 / S_H), up = p / |p|), detail
// isotropic; all Earth-fixed, so every eye and every
// frame sees the same shapes. The camera's coordinates arrive wrapped to [0, 1) from the CPU (float64); level k's
// origin is fract(4^k x level 0's) (exact: 4^k is an integer).
//
// Shape prefilter (Task 10 fix round 1, cloud_noise.hpp, anti-aliasing): beyond the start footprint
// CLOUD_SHAPE_PREFILTER_START_M of the PIXEL (shape_footprint_m; the light march's interval does not count) the shape
// is read from its box-filtered mips at the level lambda = clamp(log2(f / f_s), 0, 1) log2(f / texel) (at most
// CLOUD_SHAPE_PREFILTER_LEVELS; between level 0, the unfiltered texture, and 1 the two are mixed), against the level's
// threshold and edge mean factor tables (the covered area stays cf, the mean water the grid mean), read at a sub-pixel
// offset per ray (cloud_subpixel_offset: the temporal mean averages the pixel's footprint). Compiled only with
// CLOUD_SHAPE_PREFILTER: the cloud pass runs that kernel variant only when a footprint on the frame's clouds can exceed
// f_s (CloudEffect), else the unfiltered kernel - bit for bit the code before the prefilter (one more branch alone
// moved results by an ulp through the compiler).

// ---- Globals (project.godot [shader_globals], CloudLayer).
global uniform sampler3D cloud_field_before : filter_linear, repeat_enable;
global uniform sampler3D cloud_field_after : filter_linear, repeat_enable;
global uniform sampler3D cloud_ice_before : filter_linear, repeat_enable;
global uniform sampler3D cloud_ice_after : filter_linear, repeat_enable;
// The column texture (RGBA8, core cloud_render_column_texels): the light under the clouds (R depth, G cover per layer)
// and in layer 0 the cloud base at the lifting condensation level (B, A). Read with texelFetch.
global uniform sampler3D cloud_column_before : filter_nearest, repeat_enable;
global uniform sampler3D cloud_column_after : filter_nearest, repeat_enable;
global uniform vec3 cloud_field_before_size;  // (columns, rows, layers)
global uniform vec3 cloud_field_after_size;
global uniform float cloud_blend;  // weight of cloud_field_after
global uniform sampler3D cloud_noise_shape : filter_linear, repeat_enable;
global uniform sampler3D cloud_noise_detail : filter_linear, repeat_enable;
// 256 x CLOUD_DETAIL_STATES, R = t[state](code / 255) and m[state](code / 255).
global uniform sampler2D cloud_coverage_threshold : filter_linear, repeat_disable;
global uniform sampler2D cloud_edge_mean : filter_linear, repeat_disable;
global uniform sampler3D cloud_noise_shape_mips : filter_linear_mipmap, repeat_enable;  // shape mip levels 1..
global uniform sampler2D cloud_prefilter_threshold : filter_linear, repeat_disable;  // 256 x rows (core)
global uniform sampler2D cloud_prefilter_edge_mean : filter_linear, repeat_disable;  // 256 x rows (core)
global uniform vec2 cloud_edge_width;       // (w_min, w_max) in shape-value units
global uniform bool cloud_enabled;
global uniform float cloud_camera_height_m;  // |M camera| - a, float64 on the CPU
global uniform vec3 cloud_noise_origin;      // shape coordinates of the camera, wrapped to [0, 1)
global uniform vec3 cloud_ice_origin;        // ice-sheet shape coordinates of the camera, wrapped
global uniform vec3 cloud_stratiform_origin; // stratiform (stretched) shape coordinates of the camera, wrapped
global uniform vec3 cloud_detail_origin;     // level-0 detail coordinates of the camera, wrapped
global uniform float cloud_detail_mean;      // mean of the detail value over the texture (core)
global uniform vec3 cloud_detail_weights;    // weights of R, G, B in the detail value (core)
global uniform vec3 cloud_level_amplitude;   // a_k of the detail levels (core)
global uniform vec3 cloud_level_lift;        // kept upward share of each detail level (core cloud_detail_lift)
global uniform int cloud_detail_levels;      // levels used (quality preset; 0 = the shape alone)
global uniform vec4 cloud_phase;             // Jendersie & d'Eon: (hg_g, draine_g, draine_alpha, draine_weight)
global uniform float cloud_asymmetry_g;      // g of that phase function (closed form)
global uniform float cloud_extinction_step_per_m;  // per unit of the 16-bit G/B number
global uniform vec2 cloud_depth_code;        // (min, max) of the A channel's log code
// The light march (cloud_optics.hpp CloudLightMarch, quality preset): (first interval m, ratio, near length m, unused),
// (near samples, far samples).
global uniform vec4 cloud_light_march;
global uniform ivec2 cloud_light_samples;
// The multiple-scattering model (fidelity flag): 1 = the Monte Carlo transfer table (cloud_transfer_table.hpp, the
// reference; the lights' source attenuated with exp(-w_D tau)), 0 = the closed form (TMS on the delta-Eddington
// source function, scaled depths) for comparison.
global uniform int cloud_scattering_model;
// The transfer table's GPU form (cloud_transfer_gpu_texels): x = mu0 cell x theta, y = b cell x psi, z = a; RGBA the
// cell's (mu0, b), (mu0 + 1, b), (mu0, b + 1), (mu0 + 1, b + 1) entries. Node counts (depth, mu0, theta, psi), the depth
// map (scale, ln ratio, K) and the theta map (theta0, log, linear) as in the core.
global uniform sampler3D cloud_transfer : filter_linear, repeat_disable;
global uniform vec4 cloud_transfer_nodes;
global uniform vec3 cloud_transfer_depth_map;
global uniform vec3 cloud_transfer_theta_map;
// Checks only (cloud_check.gd; 0 in the game): 1 = the background transmittance T_bg, 2 = the Sun's scattered radiance
// per unit Sun irradiance without the air and the sky (both opaque, both of the GRID-MEAN field without the sub-grid
// shapes: the core's CPU references have no shapes); 3 = the coverage (1 cloud, 0 clear) where the ray crosses the height
// cloud_debug_height_m (the GPU's filtered coverage against the core's table); 4 = T_bg with the shapes; 6 = the full
// image (light, sky, air, composite) on the grid-mean field without the shapes (Task 10: the far field's physics);
// 7 = as 2 with the sub-grid shapes (the light cache against the light march on broken cloud, static clouds Task 1).
global uniform int cloud_debug_output;
global uniform float cloud_debug_height_m;

// [S] WGS84 equatorial radius (core constants.hpp wgs84_a_m); the stretch is ATM_STRETCH (atmosphere.gdshaderinc).
const float CLOUD_A_M = 6378137.0;
// (a / b)^2 of WGS84: geodetic latitude from the position.
const float CLOUD_AB2 = 1.0067394967422765;
const float CLOUD_PI = 3.14159265358979;
// Height layers (core weather_snapshot.hpp): 16 x 250 m from 0 to 4 km, 16 x 1 km from 4 to 20 km.
const float CLOUD_FINE_LAYER_M = 250.0;
const float CLOUD_COARSE_LAYER_M = 1000.0;
const float CLOUD_FINE_TOP_M = 4000.0;
const float CLOUD_TOP_M = 20000.0;
const float CLOUD_LAYERS = 32.0;
// Sub-grid shape scales (core cloud_noise.hpp cloud_shape_tile_horizontal_m / _vertical_m / cloud_detail_tile_m).
const float CLOUD_SHAPE_TILE_H_M = 32000.0;
const float CLOUD_SHAPE_TILE_V_M = 16000.0;
const float CLOUD_DETAIL_TILE_M = 1000.0;  // level 0; level k: / 4^k
const int CLOUD_DETAIL_LEVELS = 3;          // core cloud_detail_levels
const float CLOUD_DETAIL_STATES = 4.0;      // core cloud_detail_states (the table rows)
// Shape prefilter (core cloud_noise.hpp cloud_shape_prefilter_*; texels of the shape texture across the tile).
const float CLOUD_SHAPE_PREFILTER_START_M = 3500.0;
const float CLOUD_SHAPE_PREFILTER_LEVELS = 2.0;
const float CLOUD_SHAPE_PREFILTER_ROWS_PER_LEVEL = 4.0;
const float CLOUD_SHAPE_PREFILTER_ROWS = 9.0;
const float CLOUD_SHAPE_TEXELS = 128.0;
// [E] Ice-dominated cloud: thin sheets - the shape tile 2 km high instead of 16 (cirrus and altostratus are layered,
// often a few hundred metres thick; no fibre direction: the snapshot has no wind, user decision 2026-10-25).
const float CLOUD_ICE_TILE_V_M = 2000.0;
// Cloud types (core cloud_noise.hpp CloudType, cloud_type_weights; cloud_cirrus_ice_share / _band,
// cloud_stratiform_fraction / _fraction_band / _thickening_band, cloud_stratiform_stretch; smoke_test.gd compares
// them with the core).
const int CLOUD_TYPE_CUMULUS = 0;
const int CLOUD_TYPE_STRATIFORM = 1;
const int CLOUD_TYPE_CIRRUS = 2;
const float CLOUD_CIRRUS_ICE_SHARE = 0.5;
const float CLOUD_CIRRUS_ICE_BAND = 0.1;
const float CLOUD_STRATIFORM_FRACTION = 0.9;
const float CLOUD_STRATIFORM_FRACTION_BAND = 0.05;
const float CLOUD_STRATIFORM_THICKENING_BAND = 0.05;
const float CLOUD_STRATIFORM_STRETCH = 4.0;
// A cloud fraction below half a code step counts as none (no division by ~0).
const float CLOUD_MIN_FRACTION = 0.5 / 255.0;  // core constants.hpp cloud_min_cover
// The direct beam's cover 1 - (1 - f)^n of a broken layer (core cloud_beam_cover; constants.hpp cloud_overlap_*): n = 1 +
// a (1 - f) + b max(cot e - k, 0), fitted to the shapes' shadows (cloud_check.gd section 7).
const float CLOUD_OVERLAP_VERTICAL = 0.66;
const float CLOUD_OVERLAP_SLANT = 0.21;
const float CLOUD_OVERLAP_ONSET_COT = 1.8;
// Base transition of the vertical profile rule (core constants.hpp cloud_base_transition_m).
const float CLOUD_BASE_TRANSITION_M = 50.0;
// The flat cloud base at the lifting condensation level (core constants.hpp cloud_lcl_ramp_m, cloud_render_texels.hpp
// encode_cloud_lcl_base): the ramp from no cloud to the cloud, the 16-bit base code's offset and span, and the height
// above which no base can lie (highest ground 6 km + cloud_boundary_layer_top_m + the ramp, rounded up: no fetch above).
const float CLOUD_LCL_RAMP_M = 30.0;
const float CLOUD_LCL_CODE_OFFSET_M = 500.0;
const float CLOUD_LCL_CODE_SPAN_M = 21000.0;
const float CLOUD_LCL_CUT_TOP_M = 9500.0;
// The entry-surface cosine (core cloud_optics.cpp): regulariser and smallest cosine.
const float CLOUD_ENTRY_REGULARISER = 1.0;
const float CLOUD_ENTRY_COSINE_MIN = 0.02;

// ---- Height layers.
float cloud_layer_coordinate(float height_m) {
	float s = height_m < CLOUD_FINE_TOP_M ? height_m / CLOUD_FINE_LAYER_M
			: 16.0 + (height_m - CLOUD_FINE_TOP_M) / CLOUD_COARSE_LAYER_M;
	return clamp(s, 0.0, CLOUD_LAYERS);
}

float cloud_layer_centre_m(float layer) {
	return layer < 16.0 ? (layer + 0.5) * CLOUD_FINE_LAYER_M
			: CLOUD_FINE_TOP_M + (layer - 16.0 + 0.5) * CLOUD_COARSE_LAYER_M;
}

// ---- Geometry: (latitude, longitude) of an Earth-local point; its height from the camera-relative offset.
vec2 cloud_lat_lon(vec3 p_local) {
	float s = length(p_local.xz);
	return vec2(atan(CLOUD_AB2 * p_local.y, s), atan(p_local.x, p_local.z));
}

// The camera in model space: radius r_c = a + h_c, its up and the model-space ray (unit) with the length factor of
// the stretch (model distance = distance x scale).
struct CloudRay {
	vec3 dir_local;   // unit, Earth-local render axes
	float scale;      // |M dir|
	float mu;         // cosine of the model ray to the camera's model up
	float r_c;
};

CloudRay cloud_ray(vec3 dir_local) {
	CloudRay ray;
	ray.dir_local = dir_local;
	vec3 vm = dir_local * ATM_STRETCH;
	ray.scale = length(vm);
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	ray.mu = dot(up_c, vm / ray.scale);
	ray.r_c = CLOUD_A_M + cloud_camera_height_m;
	return ray;
}

// Height above the ellipsoid (model space) at distance d along the ray: h + d'(d' + 2 r mu) / (r_d + r), never the
// difference of two large radii.
float cloud_height_along(CloudRay ray, float d) {
	float dm = d * ray.scale;
	float r_d = sqrt(max(ray.r_c * ray.r_c + dm * dm + 2.0 * ray.r_c * ray.mu * dm, 0.0));
	return cloud_camera_height_m + dm * (dm + 2.0 * ray.r_c * ray.mu) / (r_d + ray.r_c);
}

// d(height) / d(distance) along the ray at distance d (the sine of the local elevation).
float cloud_height_slope(CloudRay ray, float d) {
	float dm = d * ray.scale;
	float r_d = sqrt(max(ray.r_c * ray.r_c + dm * dm + 2.0 * ray.r_c * ray.mu * dm, 1.0));
	return ray.scale * (dm + ray.r_c * ray.mu) / r_d;
}

// Distances (true metres) where the ray crosses the sphere of height h_s: x = entry, y = exit; x > y: no crossing.
vec2 cloud_sphere_crossing(CloudRay ray, float h_s) {
	float r_s = CLOUD_A_M + h_s;
	float b = ray.r_c * ray.mu;
	float disc = b * b - (cloud_camera_height_m - h_s) * (ray.r_c + r_s);
	if (disc < 0.0) {
		return vec2(1.0, -1.0);
	}
	float q = sqrt(disc);
	return vec2(-b - q, -b + q) / ray.scale;
}

// ---- Field lookup.
struct CloudCell {
	float fraction;     // cloud fraction
	float extinction;   // grid-mean extinction [1/m]
	float ice;          // ice share of the extinction
	float stratiform;   // stratiform weight (core cloud_stratiform_weight_between; the type weights)
	float depth_above;  // in-cloud optical depth from 20 km down to the layer centre above the point
	float in_cloud_above;  // in-cloud extinction (mean / fraction) at that centre
};

vec3 cloud_field_uvw(vec2 lat_lon, float layer, vec3 size) {
	float u = lat_lon.y / (2.0 * CLOUD_PI) + 0.5 / size.x;  // column c's centre at longitude c x 360 / columns
	float y = clamp((0.5 * CLOUD_PI - lat_lon.x) / (CLOUD_PI / (size.y - 1.0)), 0.0, size.y - 1.0);
	return vec3(u, (y + 0.5) / size.y, (layer + 0.5) / CLOUD_LAYERS);
}

float cloud_decode_extinction(vec4 t) {
	return (t.g * 65280.0 + t.b * 255.0) * cloud_extinction_step_per_m;  // (256 G + B) codes, UNORM8 = code / 255
}

float cloud_decode_depth(float a) {
	float code = floor(a * 255.0 + 0.5);
	return code < 0.5 ? 0.0 : cloud_depth_code.x * pow(cloud_depth_code.y / cloud_depth_code.x, (code - 1.0) / 254.0);
}

// The vertical profile rule (core cloud_profile_value): x in [0, 1] over a gap of gap_m between two centres.
float cloud_profile_value(float value_below, float value_above, float fraction_below, float fraction_above, float x,
		float gap_m) {
	float linear = (1.0 - x) * value_below + x * value_above;
	if (!(fraction_above > 0.0) || !(value_above > value_below)) {
		return linear;
	}
	float b = clamp(1.0 - fraction_below / fraction_above, 0.0, 1.0);
	b *= b;
	float half_width = 0.5 * CLOUD_BASE_TRANSITION_M / gap_m;
	float step_x = clamp((x - 0.5) / (2.0 * half_width) + 0.5, 0.0, 1.0);
	return linear + b * (step_x - x) * (value_above - value_below);
}

// The stratiform weight of a layer centre (core cloud_stratiform_weight): its fraction across the deck band, times
// 1 - the thickening upward (the fraction of the centre above minus its own) across its band.
float cloud_stratiform_weight(float fraction, float fraction_above) {
	float deck = smoothstep(CLOUD_STRATIFORM_FRACTION - CLOUD_STRATIFORM_FRACTION_BAND,
			CLOUD_STRATIFORM_FRACTION + CLOUD_STRATIFORM_FRACTION_BAND, fraction);
	return deck * (1.0 - smoothstep(0.0, CLOUD_STRATIFORM_THICKENING_BAND, fraction_above - fraction));
}

// The share of the cloud left at height_m over the flat base of each of the four columns around (lat, lon), mixed
// bilinearly like the field's own values (core cloud_lcl_cut_factor per column): a column without a base (fog, unknown
// level, no boundary-layer cloud: code 0) is not cut. The base is read from the column texture's layer 0 (B high byte,
// A low byte of the 16-bit code), nearest texels.
float cloud_lcl_cut(sampler3D column, vec3 size, vec2 lat_lon, float height_m) {
	if (height_m > CLOUD_LCL_CUT_TOP_M) {
		return 1.0;
	}
	float x = lat_lon.y / (2.0 * CLOUD_PI) * size.x;  // column c's centre at longitude c x 360 / columns
	float y = clamp((0.5 * CLOUD_PI - lat_lon.x) / (CLOUD_PI / (size.y - 1.0)), 0.0, size.y - 1.0);
	float x0 = floor(x);
	float y0 = min(floor(y), size.y - 2.0);
	vec2 f = vec2(x - x0, y - y0);
	int columns = int(size.x);
	int c0 = int(mod(x0, size.x));
	int c1 = c0 + 1 < columns ? c0 + 1 : 0;
	int r0 = int(y0);
	vec4 codes = vec4(0.0);
	vec4 t = texelFetch(column, ivec3(c0, r0, 0), 0);
	codes.x = floor(t.b * 255.0 + 0.5) * 256.0 + floor(t.a * 255.0 + 0.5);
	t = texelFetch(column, ivec3(c1, r0, 0), 0);
	codes.y = floor(t.b * 255.0 + 0.5) * 256.0 + floor(t.a * 255.0 + 0.5);
	t = texelFetch(column, ivec3(c0, r0 + 1, 0), 0);
	codes.z = floor(t.b * 255.0 + 0.5) * 256.0 + floor(t.a * 255.0 + 0.5);
	t = texelFetch(column, ivec3(c1, r0 + 1, 0), 0);
	codes.w = floor(t.b * 255.0 + 0.5) * 256.0 + floor(t.a * 255.0 + 0.5);
	vec4 bases = (codes - 1.0) / 65534.0 * CLOUD_LCL_CODE_SPAN_M - CLOUD_LCL_CODE_OFFSET_M;
	vec4 cut = clamp((vec4(height_m) - bases) / CLOUD_LCL_RAMP_M, 0.0, 1.0);
	cut = mix(vec4(1.0), cut, step(0.5, codes));  // code 0: no base, no cut
	return mix(mix(cut.x, cut.y, f.x), mix(cut.z, cut.w, f.x), f.y);
}

// One snapshot at a point: the centres k (below) and k + 1 (above) around layer coordinate s = layer - 0.5.
CloudCell cloud_snapshot_cell(sampler3D field, sampler3D ice_share, sampler3D column, vec3 size, vec2 lat_lon,
		float height_m) {
	float s = cloud_layer_coordinate(height_m) - 0.5;
	float k = clamp(floor(s), 0.0, CLOUD_LAYERS - 2.0);
	float x = clamp(s - k, 0.0, 1.0);  // below centre 0: x = 0 (the end value); above centre 31: x = 1
	vec4 below = texture(field, cloud_field_uvw(lat_lon, k, size));
	vec4 above = texture(field, cloud_field_uvw(lat_lon, k + 1.0, size));
	float gap = cloud_layer_centre_m(k + 1.0) - cloud_layer_centre_m(k);
	float beta_b = cloud_decode_extinction(below);
	float beta_a = cloud_decode_extinction(above);
	CloudCell c;
	c.fraction = cloud_profile_value(below.r, above.r, below.r, above.r, x, gap);
	c.extinction = cloud_profile_value(beta_b, beta_a, below.r, above.r, x, gap);
	float lcl_cut = cloud_lcl_cut(column, size, lat_lon, height_m);  // the flat base at the lifting condensation level
	c.fraction *= lcl_cut;
	c.extinction *= lcl_cut;
	c.ice = texture(ice_share, cloud_field_uvw(lat_lon, clamp(s, 0.0, CLOUD_LAYERS - 1.0), size)).r;
	// The stratiform weight of the centres k and k + 1 weighted by (1 - x) and x times their fractions (core
	// cloud_stratiform_weight_between; continuous through the centres); centre k + 1's needs the fraction of k + 2 -
	// fetched only where centre k + 1 can be a deck.
	float above_next = above.r;
	if (above.r > CLOUD_STRATIFORM_FRACTION - CLOUD_STRATIFORM_FRACTION_BAND && k + 2.0 < CLOUD_LAYERS) {
		above_next = texture(field, cloud_field_uvw(lat_lon, k + 2.0, size)).r;
	}
	float share_below = (1.0 - x) * below.r;
	float share_above = x * above.r;
	c.stratiform = share_below + share_above > 0.0
			? (share_below * cloud_stratiform_weight(below.r, above.r)
					+ share_above * cloud_stratiform_weight(above.r, above_next)) / (share_below + share_above)
			: 0.0;
	// The centre above the point (above centre 31: that centre itself, held to 20 km).
	bool top = s >= CLOUD_LAYERS - 1.0;
	vec4 centre = top ? above : (x <= 0.0 && s < 0.0 ? below : above);
	c.depth_above = cloud_decode_depth(centre.a);
	c.in_cloud_above = centre.r >= CLOUD_MIN_FRACTION ? cloud_decode_extinction(centre) / centre.r : 0.0;
	return c;
}

// The blended field at (lat, lon, height): cloud fraction, grid-mean extinction, ice share and the column terms;
// 0 above 20 km.
CloudCell cloud_field(vec2 lat_lon, float height_m) {
	CloudCell c;
	c.fraction = 0.0;
	c.extinction = 0.0;
	c.ice = 0.0;
	c.stratiform = 0.0;
	c.depth_above = 0.0;
	c.in_cloud_above = 0.0;
	if (height_m > CLOUD_TOP_M) {
		return c;
	}
	if (cloud_blend < 1.0) {
		CloudCell b = cloud_snapshot_cell(cloud_field_before, cloud_ice_before, cloud_column_before,
				cloud_field_before_size, lat_lon, height_m);
		float w = 1.0 - cloud_blend;
		c.fraction += w * b.fraction;
		c.extinction += w * b.extinction;
		c.ice += w * b.ice;
		c.stratiform += w * b.stratiform;
		c.depth_above += w * b.depth_above;
		c.in_cloud_above += w * b.in_cloud_above;
	}
	if (cloud_blend > 0.0) {
		CloudCell a = cloud_snapshot_cell(cloud_field_after, cloud_ice_after, cloud_column_after,
				cloud_field_after_size, lat_lon, height_m);
		c.fraction += cloud_blend * a.fraction;
		c.extinction += cloud_blend * a.extinction;
		c.ice += cloud_blend * a.ice;
		c.stratiform += cloud_blend * a.stratiform;
		c.depth_above += cloud_blend * a.depth_above;
		c.in_cloud_above += cloud_blend * a.in_cloud_above;
	}
	return c;
}

// The whole column's in-cloud optical depth (the diffuse field's tau*): the depth at centre 0 plus centre 0's in-cloud
// extinction held down to the ellipsoid (cloud_render_texels.hpp).
float cloud_column_depth(vec2 lat_lon) {
	float result = 0.0;
	if (cloud_blend < 1.0) {
		vec4 t = texture(cloud_field_before, cloud_field_uvw(lat_lon, 0.0, cloud_field_before_size));
		float in_cloud = t.r >= CLOUD_MIN_FRACTION ? cloud_decode_extinction(t) / t.r : 0.0;
		result += (1.0 - cloud_blend) * (cloud_decode_depth(t.a) + in_cloud * cloud_layer_centre_m(0.0));
	}
	if (cloud_blend > 0.0) {
		vec4 t = texture(cloud_field_after, cloud_field_uvw(lat_lon, 0.0, cloud_field_after_size));
		float in_cloud = t.r >= CLOUD_MIN_FRACTION ? cloud_decode_extinction(t) / t.r : 0.0;
		result += cloud_blend * (cloud_decode_depth(t.a) + in_cloud * cloud_layer_centre_m(0.0));
	}
	return result;
}

// The column terms at a point of in-cloud extinction beta_point: x = in-cloud optical depth from the column top (20 km)
// down to the point (the centre above plus the trapezoid down to the point), y = the whole column.
vec2 cloud_column_depths(CloudCell cell, vec2 lat_lon, float height_m, float beta_point) {
	float s = cloud_layer_coordinate(height_m) - 0.5;
	vec2 depth;
	if (s >= CLOUD_LAYERS - 1.0) {
		depth.x = beta_point * max(CLOUD_TOP_M - height_m, 0.0);  // held from the top centre to 20 km
	} else {
		float above = clamp(floor(s) + 1.0, 0.0, CLOUD_LAYERS - 1.0);
		depth.x = cell.depth_above
				+ 0.5 * (beta_point + cell.in_cloud_above) * max(cloud_layer_centre_m(above) - height_m, 0.0);
	}
	depth.y = max(cloud_column_depth(lat_lon), depth.x);
	return depth;
}

// ---- Sub-grid shapes.
// Shape coordinates of the point at camera + offset (height_m its height) for the tiles tile_h_m across and tile_v_m
// up; origin the camera's (header).
vec3 cloud_shape_coordinates_for(vec3 offset_local, float height_m, vec3 origin, float tile_h_m, float tile_v_m) {
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	vec3 up_p = normalize(camera + offset_local);
	vec3 up_c = normalize(camera);
	float k = 1.0 / tile_v_m - 1.0 / tile_h_m;
	return origin + offset_local / tile_h_m + k * (up_p * height_m - up_c * cloud_camera_height_m);
}

vec3 cloud_shape_coordinates(vec3 offset_local, float height_m) {
	return cloud_shape_coordinates_for(offset_local, height_m, cloud_noise_origin, CLOUD_SHAPE_TILE_H_M,
			CLOUD_SHAPE_TILE_V_M);
}

// The type weights at a point (core cloud_type_weights), indexed by CLOUD_TYPE_*: cirrus across the ice band,
// stratiform the rest times the cell's stratiform weight, cumulus what remains; sum 1.
vec3 cloud_type_weights(CloudCell cell) {
	float cirrus = smoothstep(CLOUD_CIRRUS_ICE_SHARE - CLOUD_CIRRUS_ICE_BAND,
			CLOUD_CIRRUS_ICE_SHARE + CLOUD_CIRRUS_ICE_BAND, cell.ice);
	float stratiform = (1.0 - cirrus) * clamp(cell.stratiform, 0.0, 1.0);
	return vec3(1.0 - cirrus - stratiform, stratiform, cirrus);
}

// The shape coordinates of the type (core cloud_shape_texel) at camera + offset_local.
vec3 cloud_type_shape_coordinates(int type, vec3 offset_local, float height_m) {
	if (type == CLOUD_TYPE_CIRRUS) {
		return cloud_shape_coordinates_for(offset_local, height_m, cloud_ice_origin, CLOUD_SHAPE_TILE_H_M,
				CLOUD_ICE_TILE_V_M);
	}
	if (type == CLOUD_TYPE_STRATIFORM) {
		return cloud_shape_coordinates_for(offset_local, height_m, cloud_stratiform_origin,
				CLOUD_SHAPE_TILE_H_M * CLOUD_STRATIFORM_STRETCH, CLOUD_SHAPE_TILE_V_M);
	}
	return cloud_shape_coordinates(offset_local, height_m);
}

// The table texel of a cloud fraction (columns: codes) and a detail state (rows), linear between both.
vec2 cloud_table_uv(float fraction, float state) {
	return vec2((fraction * 255.0 + 0.5) / 256.0, (state + 0.5) / CLOUD_DETAIL_STATES);
}

float cloud_threshold(float fraction, float state) {
	return texture(cloud_coverage_threshold, cloud_table_uv(fraction, state)).r;
}

// Coverage of the shape alone (state 0): 1 where it is cloud for the cloud fraction, else 0 (checks).
float cloud_coverage(vec3 shape_q, float fraction) {
	return texture(cloud_noise_shape, shape_q).r > cloud_threshold(fraction, 0.0) ? 1.0 : 0.0;
}

// The shape prefilter's level at the pixel footprint [m] (core cloud_shape_prefilter_level): 0 up to the start.
float cloud_shape_prefilter_level(float footprint_m) {
	if (footprint_m <= CLOUD_SHAPE_PREFILTER_START_M) {
		return 0.0;
	}
	float onset = min(log2(footprint_m / CLOUD_SHAPE_PREFILTER_START_M), 1.0);
	return clamp(onset * log2(footprint_m * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M), 0.0,
			CLOUD_SHAPE_PREFILTER_LEVELS);
}

// The shape at its prefilter level (core sample_shape_prefiltered): mip 1 is the mips texture's level 0.
vec4 cloud_shape_prefiltered(vec3 shape_q, float level) {
	if (level < 1.0) {
		return mix(texture(cloud_noise_shape, shape_q), textureLod(cloud_noise_shape_mips, shape_q, 0.0), level);
	}
	return textureLod(cloud_noise_shape_mips, shape_q, level - 1.0);
}

// The prefilter tables' texel of a cloud fraction (columns: codes) and a level (rows), linear between both.
vec2 cloud_prefilter_uv(float fraction, float level) {
	return vec2((fraction * 255.0 + 0.5) / 256.0,
			(level * CLOUD_SHAPE_PREFILTER_ROWS_PER_LEVEL + 0.5) / CLOUD_SHAPE_PREFILTER_ROWS);
}

// The soft edge ramp (core cloud_edge_ramp, cloud_edge_width_value): smoothstep of the excess over the edge width of
// the width noise n.
float cloud_edge_ramp(float value, float threshold, float width_noise) {
	float w = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, clamp(width_noise, 0.0, 1.0));
	float x = clamp((value - threshold) / max(w, 1.0e-6), 0.0, 1.0);
	return x * x * (3.0 - 2.0 * x);
}

// The detail state L at the pixel footprint [m] (core cloud_detail_state, capped by the preset's levels).
float cloud_detail_state(float footprint_m) {
	float state = 0.0;
	float middle_cell_m = CLOUD_DETAIL_TILE_M / 4.0;
	for (int k = 0; k < CLOUD_DETAIL_LEVELS; k++) {
		if (k >= cloud_detail_levels) {
			break;
		}
		state += clamp(log2(middle_cell_m / max(footprint_m, 1.0e-3)) - 1.0, 0.0, 1.0);
		middle_cell_m *= 0.25;
	}
	return state;
}

// The detail levels' eroded sum (core cloud_shape_value: v = s + sum_k cloud_detail_erosion(k, p_k)) at camera +
// offset_local for the state.
float cloud_detail_perturbation(vec3 offset_local, float state) {
	float p = 0.0;
	float scale = 1.0;
	for (int k = 0; k < CLOUD_DETAIL_LEVELS; k++) {
		float w = clamp(state - float(k), 0.0, 1.0);
		if (w <= 0.0) {
			break;
		}
		vec3 q = fract(cloud_detail_origin * scale) + offset_local * (scale / CLOUD_DETAIL_TILE_M);
		float d = dot(texture(cloud_noise_detail, q).rgb, cloud_detail_weights);
		float term = w * cloud_level_amplitude[k] * (d - cloud_detail_mean);
		p += term < 0.0 ? term : cloud_level_lift[k] * term;  // erosion (core cloud_detail_erosion)
		scale *= 4.0;
	}
	return p;
}

#ifdef CLOUD_SHAPE_PREFILTER
// Four uniform numbers in [0, 1) of the ray (set by the cloud pass per pixel and frame: blue noise + R4 sequence).
vec4 cloud_shape_subpixel = vec4(0.5);
#endif

// Sub-pixel offsets (header): from f_s on the pixel spans several of the shape's features (and the 32 km tile's period
// beats with the pixel grid; the mips stop at level 2): the shape is read at a horizontal offset in the local tangent
// plane drawn per ray from the pixel's tent filter (half-width f per axis, faded in from f_s to 2 f_s: the ray stands
// for a random point of its pixel's footprint), and the cloud pass's temporal mean averages the pixel's transmittance
// over it - the coverage-preserving mean of the thresholded shape, in transmittance (a mean density would make broken
// cloud a fog). The cell (fraction, water) stays the true point's.
vec3 cloud_subpixel_offset(vec3 offset_local, float footprint_m) {
	float half_width = footprint_m * clamp(log2(footprint_m / CLOUD_SHAPE_PREFILTER_START_M), 0.0, 1.0);
	if (half_width <= 0.0) {
		return offset_local;
	}
	vec3 up = normalize(atmosphere_camera_hi + atmosphere_camera_lo + offset_local);
	// Orthonormal tangent basis (Duff et al. 2017, "Building an Orthonormal Basis, Revisited").
	float sign_z = up.z >= 0.0 ? 1.0 : -1.0;
	float a = -1.0 / (sign_z + up.z);
	float b = up.x * up.y * a;
	vec3 t1 = vec3(1.0 + sign_z * up.x * up.x * a, sign_z * b, -sign_z * up.x);
	vec3 t2 = vec3(b, sign_z + up.y * up.y * a, -up.y);
#ifdef CLOUD_SHAPE_PREFILTER
	vec2 u = cloud_shape_subpixel.xy + cloud_shape_subpixel.zw - 1.0;  // tent on [-1, 1]
#else
	vec2 u = vec2(0.0);
#endif
	return offset_local + (t1 * u.x + t2 * u.y) * half_width;
}

// The shapes at the prefilter level (> 0; header) for the pixel footprint footprint_m: the detail is long gone there
// (its state is 0 beyond 125 m).
float cloud_extinction_prefiltered(CloudCell cell, vec3 offset_local, float height_m, float footprint_m, float level) {
	vec2 uv = cloud_prefilter_uv(cell.fraction, level);
	float threshold = texture(cloud_prefilter_threshold, uv).r;
	vec3 shifted = cloud_subpixel_offset(offset_local, footprint_m);
	vec3 types = cloud_type_weights(cell);
	float ramp = 0.0;
	for (int type = 0; type < 3; type++) {  // the types' ramps mixed by their weights (one lookup outside the bands)
		if (types[type] > 0.0) {
			vec4 s = cloud_shape_prefiltered(cloud_type_shape_coordinates(type, shifted, height_m), level);
			ramp += types[type] * cloud_edge_ramp(s.r, threshold, s.a);
		}
	}
	if (ramp <= 0.0) {
		return 0.0;
	}
	float m = texture(cloud_prefilter_edge_mean, uv).r;
	return cell.extinction / max(m, 1.0e-4) * ramp;
}

// Extinction [1/m] (unscaled) at camera + offset_local with its height, from the field cell already looked up there;
// the detail band-limited at footprint_m (the pixel footprint there or larger; large: the shape alone), the shape
// prefiltered at the pixel footprint shape_footprint_m (0: never; header). in_cloud: the in-cloud mean extinction
// mean / cf of the covered area (the column terms: the A channel's convention, core cloud_render_texels.hpp; one
// convention, so the column depth is continuous through the layer centres).
float cloud_extinction_in_cell(CloudCell cell, vec3 offset_local, float height_m, float footprint_m,
		float shape_footprint_m, out float in_cloud) {
	in_cloud = 0.0;
	if (cloud_debug_output == 1 || cloud_debug_output == 2 || cloud_debug_output == 6) {  // checks: the grid mean
		in_cloud = cell.extinction;
		return cell.extinction;
	}
	if (cell.fraction < CLOUD_MIN_FRACTION || cell.extinction <= 0.0) {
		return 0.0;
	}
	in_cloud = cell.extinction / cell.fraction;
#ifdef CLOUD_SHAPE_PREFILTER
	float prefilter = cloud_shape_prefilter_level(shape_footprint_m);
	if (prefilter > 0.0) {
		return cloud_extinction_prefiltered(cell, offset_local, height_m, shape_footprint_m, prefilter);
	}
#endif
	float state = cloud_detail_state(footprint_m);
	float threshold = cloud_threshold(cell.fraction, state);
	float detail = state > 0.0 ? cloud_detail_perturbation(offset_local, state) : 0.0;
	vec3 types = cloud_type_weights(cell);
	float ramp = 0.0;
	for (int type = 0; type < 3; type++) {  // the types' ramps mixed by their weights (one lookup outside the bands)
		if (types[type] > 0.0) {
			vec4 s = texture(cloud_noise_shape, cloud_type_shape_coordinates(type, offset_local, height_m));
			ramp += types[type] * cloud_edge_ramp(s.r + detail, threshold, s.a);
		}
	}
	if (ramp <= 0.0) {
		return 0.0;
	}
	float m = texture(cloud_edge_mean, cloud_table_uv(cell.fraction, state)).r;
	return cell.extinction / max(m, 1.0e-4) * ramp;
}

float cloud_extinction_at(vec3 offset_local, float height_m, float footprint_m, float shape_footprint_m,
		out vec2 lat_lon, out float in_cloud) {
	lat_lon = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_local);
	return cloud_extinction_in_cell(cloud_field(lat_lon, height_m), offset_local, height_m, footprint_m,
			shape_footprint_m, in_cloud);
}

// Grid-mean extinction (no shapes) at camera + offset_local: the far part of the light march.
float cloud_mean_extinction_at(vec3 offset_local, float height_m) {
	return cloud_field(cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_local), height_m).extinction;
}

// Height of the absolute Earth-local point p (Task 9 / 10 callers; far points only: float32 absolute positions).
float cloud_height_of(vec3 p_earth_m) {
	return length(p_earth_m * ATM_STRETCH) - CLOUD_A_M;
}

// Extinction [1/m] at an absolute Earth-local point (Tasks 9 / 10): the shapes, the detail band-limited and the shape
// prefiltered at the pixel footprint footprint_m.
float cloud_density(vec3 p_earth_m, float footprint_m) {
	vec3 offset = p_earth_m - atmosphere_camera_hi - atmosphere_camera_lo;
	vec2 lat_lon;
	float in_cloud;
	return cloud_extinction_at(offset, cloud_height_of(p_earth_m), footprint_m, footprint_m, lat_lon, in_cloud);
}

// ---- The light march (core cloud_light_optical_depth): from camera + offset_local toward dir_local; near_length in
// near_samples geometric intervals (first interval, ratio: cloud_light_march) through the shapes, then far_samples
// equal intervals to the cloud top (at most CLOUD_SUN_PATH_MAX_M) through the grid-mean field. One sample per interval
// at the fraction jitter, weighted by the interval: stratified, unbiased over the jitter. The detail is band-limited at
// the larger of the pixel footprint there (pixel_angle x (camera_distance_m + s)) and the interval (finer octaves
// average out over it; the tables keep the mean); pixel_angle large: the shape alone. The shape is prefiltered at the
// pixel footprint of shape_pixel_angle alone (0: never). Heights along the straight line (curvature + s^2 / 2a).
// cloud_light_optical_depth_with: the intervals (march, as cloud_light_march) and samples (near, far) given, and the
// detail's footprint never below footprint_floor_m (the light cache: its voxel; cloud_light_cache.gdshaderinc).
const float CLOUD_SUN_PATH_MAX_M = 300000.0;
float cloud_light_optical_depth_with(vec3 offset_local, float height_m, float camera_distance_m, vec3 dir_local,
		float jitter, float pixel_angle, float shape_pixel_angle, vec4 march, ivec2 samples, float footprint_floor_m) {
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset_local;
	vec3 up = normalize(p * ATM_STRETCH);
	float climb = dot(up, normalize(dir_local * ATM_STRETCH));
	float to_top = climb > 1.0e-3 ? max(CLOUD_TOP_M - height_m, 0.0) / climb : CLOUD_SUN_PATH_MAX_M;
	to_top = min(to_top, CLOUD_SUN_PATH_MAX_M);
	float near_length = min(march.z, to_top);
	float tau = 0.0;
	float start = 0.0;
	float length_i = march.x * near_length / march.z;  // the intervals scale with the length
	for (int i = 0; i < samples.x; i++) {
		float s = start + jitter * length_i;
		float h = height_m + s * climb + s * s / (2.0 * CLOUD_A_M);
		if (h <= CLOUD_TOP_M) {
			vec2 lat_lon;
			float in_cloud;
			float footprint = max(max(pixel_angle * (camera_distance_m + s), length_i), footprint_floor_m);
			tau += cloud_extinction_at(offset_local + dir_local * s, h, footprint,
					shape_pixel_angle * (camera_distance_m + s), lat_lon, in_cloud) * length_i;
		}
		start += length_i;
		length_i *= march.y;
	}
	if (samples.y > 0 && to_top > near_length) {
		float step_m = (to_top - near_length) / float(samples.y);
		for (int i = 0; i < samples.y; i++) {
			float s = near_length + (float(i) + jitter) * step_m;
			float h = height_m + s * climb + s * s / (2.0 * CLOUD_A_M);
			if (h <= CLOUD_TOP_M) {
				tau += cloud_mean_extinction_at(offset_local + dir_local * s, h) * step_m;
			}
		}
	}
	return tau;
}

// The light march of the quality preset (cloud_light_march, cloud_light_samples).
float cloud_light_optical_depth(vec3 offset_local, float height_m, float camera_distance_m, vec3 dir_local,
		float jitter, float pixel_angle, float shape_pixel_angle) {
	return cloud_light_optical_depth_with(offset_local, height_m, camera_distance_m, dir_local, jitter, pixel_angle,
			shape_pixel_angle, cloud_light_march, cloud_light_samples, 0.0);
}

// Transmittance of the direct Sun (Beer-Lambert, unscaled extinction, as the core's cloud_sun_transmittance) from the
// absolute Earth-local point toward sun_dir (Tasks 9 / 10): the light march of the shape alone at the jitter 0.5.
const float CLOUD_NO_DETAIL_ANGLE = 1.0e3;
float cloud_sun_transmittance(vec3 p_earth_m, vec3 sun_dir) {
	float h = cloud_height_of(p_earth_m);
	if (h > CLOUD_TOP_M && dot(normalize(p_earth_m), sun_dir) >= 0.0) {
		return 1.0;
	}
	vec3 offset = p_earth_m - atmosphere_camera_hi - atmosphere_camera_lo;
	return exp(-cloud_light_optical_depth(offset, h, length(offset), sun_dir, 0.5, CLOUD_NO_DETAIL_ANGLE, 0.0));
}

// ---- Optics (cloud_optics.hpp).
float cloud_henyey_greenstein(float one_minus_mu, float g) {
	// 1 + g^2 - 2 g mu written as (1 - g)^2 + 2 g (1 - mu): no cancellation in the forward peak (g ~0.995).
	float denom = (1.0 - g) * (1.0 - g) + 2.0 * g * one_minus_mu;
	return (1.0 - g * g) / (4.0 * CLOUD_PI * denom * sqrt(denom));
}

// p(mu) of Jendersie & d'Eon 2023 (HG + Draine), per steradian; one_minus_mu = 1 - cos theta (passed separately for
// the forward peak's precision).
float cloud_phase_function(float mu, float one_minus_mu) {
	float g_hg = cloud_phase.x;
	float g_d = cloud_phase.y;
	float alpha = cloud_phase.z;
	float w = cloud_phase.w;
	float draine = cloud_henyey_greenstein(one_minus_mu, g_d) * (1.0 + alpha * mu * mu)
			/ (1.0 + alpha * (1.0 + 2.0 * g_d * g_d) / 3.0);
	return (1.0 - w) * cloud_henyey_greenstein(one_minus_mu, g_hg) + w * draine;
}

// Delta-Eddington constants of the phase function: x = f, y = g', z = gamma1.
vec3 cloud_delta_eddington() {
	float g = cloud_asymmetry_g;
	float g_scaled = g / (1.0 + g);
	return vec3(g * g, g_scaled, 0.75 * (1.0 - g_scaled));
}

float cloud_transport_scale() {
	return 1.0 - cloud_asymmetry_g * cloud_asymmetry_g;
}

// The Sun's scattering source J (cloud_scattering_source: TMS on the delta-Eddington source function) per unit Sun
// irradiance and unit unscaled optical depth. cos_theta: Sun propagation . view propagation (1 = looking into the Sun),
// one_minus_cos = 1 - cos_theta; view_up_cos: the view propagation direction (toward the camera) . the column axis;
// depths unscaled.
float cloud_scattering_source(float cos_theta, float one_minus_cos, float view_up_cos, float sun_cos_zenith,
		float sun_optical_depth, float depth_from_top, float column_optical_depth) {
	if (sun_cos_zenith <= 0.0) {
		return 0.0;
	}
	vec3 de = cloud_delta_eddington();
	float k = 1.0 - de.x;
	float mu0 = min(1.0, sun_cos_zenith);
	float column = k * max(0.0, column_optical_depth);
	float t = clamp(k * depth_from_top, 0.0, column);
	float gamma1 = de.z;
	float gamma3 = (2.0 - 3.0 * de.y * mu0) / 4.0;
	float r = (gamma1 * column + (gamma3 - gamma1 * mu0) * (1.0 - exp(-column / mu0))) / (1.0 + gamma1 * column);
	float a0 = mu0 * (r + 2.0 * gamma1 * mu0 + 1.0 - 2.0 * gamma3);
	float a1 = 2.0 * gamma1 * mu0 * (r - 1.0);
	float a2 = -mu0 * (2.0 * gamma1 * mu0 + 1.0 - 2.0 * gamma3);
	float b0 = mu0 * (r - 1.0);
	float b1 = mu0;
	float e = exp(-t / mu0);
	float sigma = a0 + a1 * t + a2 * e;
	float net = b0 + b1 * e;
	float i0 = sigma / (2.0 * CLOUD_PI);
	float i1 = 3.0 * net / (4.0 * CLOUD_PI);
	return k * (i0 + de.y * view_up_cos * i1)
			+ cloud_phase_function(cos_theta, one_minus_cos) * exp(-k * max(0.0, sun_optical_depth));
}

// The entry-surface cosine (core cloud_entry_cosine, cloud_optics.hpp "3D Sun light").
float cloud_entry_cosine(float sun_cos_zenith, float sun_optical_depth, float vertical_depth_from_top) {
	float mu0 = clamp(sun_cos_zenith, -1.0, 1.0);
	float ratio = (max(0.0, vertical_depth_from_top) + CLOUD_ENTRY_REGULARISER * mu0)
			/ (max(0.0, sun_optical_depth) + CLOUD_ENTRY_REGULARISER);
	return clamp(ratio, max(mu0, CLOUD_ENTRY_COSINE_MIN), 1.0);
}

// The Sun's source with the 3D Sun depth (core cloud_scattering_source_3d): light_dir toward the light, view_prop the
// propagation toward the camera, up the local (geodetic) up, all unit; depths unscaled.
// The entry surface's normal: up turned toward the light by acos(mu0) - acos(mu_e) (core entry_normal).
vec3 cloud_entry_normal(vec3 light_dir, vec3 up, float mu0, float mu_e) {
	vec3 normal = up;
	vec3 across = light_dir - up * mu0;
	float across_norm = length(across);
	if (across_norm > 1.0e-6) {
		float turn = acos(mu0) - acos(mu_e);
		normal = up * cos(turn) + across * (sin(turn) / across_norm);
	}
	return normal;
}

float cloud_scattering_source_3d(vec3 light_dir, vec3 view_prop, vec3 up, float sun_optical_depth,
		float vertical_depth_from_top, float column_optical_depth) {
	float mu0 = clamp(dot(light_dir, up), -1.0, 1.0);
	float mu_e = cloud_entry_cosine(mu0, sun_optical_depth, vertical_depth_from_top);
	vec3 normal = cloud_entry_normal(light_dir, up, mu0, mu_e);
	float tau_s = max(0.0, sun_optical_depth);
	float depth = mu_e * tau_s;
	vec3 diff = light_dir + view_prop;  // |s + v|^2 / 2 = 1 + s.v = 1 - cos(theta), theta between the propagations
	float one_minus_cos = 0.5 * dot(diff, diff);
	return cloud_scattering_source(1.0 - one_minus_cos, one_minus_cos, dot(view_prop, normal), mu_e, tau_s, depth,
			max(column_optical_depth, depth));
}

// Node coordinate of a depth on the transfer table's a / b axis (core cloud_transfer_depth_coordinate).
float cloud_transfer_depth_coordinate(float depth) {
	vec3 m = cloud_transfer_depth_map;
	float last = cloud_transfer_nodes.x - 2.0;
	float a = max(depth, 0.0);
	float a_last = m.x * (exp(last * m.y) - 1.0);
	if (a <= a_last) {
		return log(1.0 + a / m.x) / m.y;
	}
	// Linear in x = 1 / (1 + K a) from the last finite node to infinity.
	return last + 1.0 - (1.0 + m.z * a_last) / (1.0 + m.z * a);
}

// The tabulated source of the orders >= 2 (core cloud_transfer_lookup): one trilinear fetch (theta, psi, a) returns
// the four (mu0, b) corners of the cell, mixed here.
float cloud_transfer_lookup(float depth_from_top, float depth_from_bottom, float mu0, float theta, float psi) {
	vec4 n = cloud_transfer_nodes;
	vec3 tm = cloud_transfer_theta_map;
	float ua = clamp(cloud_transfer_depth_coordinate(depth_from_top), 0.0, n.x - 1.0);
	float ub = clamp(cloud_transfer_depth_coordinate(depth_from_bottom), 0.0, n.x - 1.0);
	float um = clamp(sqrt(clamp(mu0, 0.0, 1.0)) * n.y - 1.0, 0.0, n.y - 1.0);  // node i: ((i + 1) / n)^2
	float ut = clamp(tm.y * log(1.0 + theta / tm.x) + tm.z * theta, 0.0, n.z - 1.0);
	float up = clamp(psi / CLOUD_PI * (n.w - 1.0), 0.0, n.w - 1.0);
	float ib = min(floor(ub), n.x - 2.0);
	float im = min(floor(um), n.y - 2.0);
	vec3 size = vec3((n.y - 1.0) * n.z, (n.x - 1.0) * n.w, n.x);
	vec4 c = texture(cloud_transfer, (vec3(im * n.z + ut, ib * n.w + up, ua) + 0.5) / size);
	float fm = um - im;
	return clamp(mu0, 0.0, 1.0) * mix(mix(c.r, c.g, fm), mix(c.b, c.a, fm), ub - ib);  // the table holds J / mu0
}

// The light's source with the Monte Carlo table (core cloud_scattering_source_mc): exact single scattering plus the
// tabulated orders >= 2, per unit irradiance and unit optical depth; the view ray attenuates it with exp(-w_D tau_v)
// (w_D = cloud_phase.w; core cloud_transfer_view_extinction).
float cloud_scattering_source_mc(vec3 light_dir, vec3 view_prop, vec3 up, float sun_optical_depth,
		float vertical_depth_from_top, float column_optical_depth) {
	float mu0 = clamp(dot(light_dir, up), -1.0, 1.0);
	float mu_e = cloud_entry_cosine(mu0, sun_optical_depth, vertical_depth_from_top);
	vec3 normal = cloud_entry_normal(light_dir, up, mu0, mu_e);
	float tau_s = max(0.0, sun_optical_depth);
	float depth = mu_e * tau_s;
	float column = max(column_optical_depth, depth);
	vec3 chord = view_prop + light_dir;  // view propagation - Sun propagation: |chord| = 2 sin(theta / 2)
	float one_minus_cos = 0.5 * dot(chord, chord);
	float theta = 2.0 * asin(min(1.0, 0.5 * length(chord)));
	// psi: the view propagation's azimuth about the Sun's propagation s, from the normal's part across s.
	vec3 s = -light_dir;
	vec3 e1 = normal - s * dot(normal, s);
	float psi = 0.0;
	float e1_norm = length(e1);
	if (e1_norm > 1.0e-6) {
		e1 /= e1_norm;
		vec3 across = view_prop - s * dot(view_prop, s);
		float c1 = dot(across, e1);
		psi = atan(length(across - e1 * c1), c1);
	}
	return cloud_phase_function(1.0 - one_minus_cos, one_minus_cos) * exp(-tau_s)
			+ cloud_transfer_lookup(depth, column - depth, mu_e, theta, psi);
}

// The sky's source (cloud_sky_scattering_source) per unit horizontal sky irradiance on the column top.
float cloud_sky_scattering_source(float view_up_cos, float depth_from_top, float column_optical_depth) {
	vec3 de = cloud_delta_eddington();
	float k = 1.0 - de.x;
	float column = k * max(0.0, column_optical_depth);
	float t = clamp(k * depth_from_top, 0.0, column);
	float transmittance = 1.0 / (1.0 + de.z * column);
	float sigma = 2.0 - transmittance - 2.0 * de.z * transmittance * t;
	float i0 = sigma / (2.0 * CLOUD_PI);
	float i1 = -3.0 * transmittance / (4.0 * CLOUD_PI);
	return k * (i0 + de.y * view_up_cos * i1);
}

// ---- Light under the clouds (Task 9; core cloud_render_column_texels, cloud_optics.hpp cloud_column_depths_above,
// cloud_sky_factor and cloud_sun_diffuse_transmittance - change them together). The grid-mean column above a point:
// the column texture's depth at the layer centre above (bilinear on the decoded corners: linear in the depth, as the
// CPU's linear interpolation of the field) plus the trapezoid of the grid-mean extinction down to the point (the
// field's vertical profile rule at the point), held from the top centre to 20 km; its cover C the larger of the
// texture's largest cloud fraction above the centre and the point's fraction (maximum overlap). Independent columns:
// clear on 1 - C, the depth / C on C (no plane-parallel bias for broken cloud). The sky's diffuse transmittance is
// 1 - C + C x the two-stream one, the Sun's diffuse light C x (the Monte Carlo slab's total - its direct part).
// The slab's total beam transmittance (core CloudSlabTransmittanceLut): x = depth node, y = cosine node.
global uniform sampler2D cloud_slab_transmittance : filter_linear, repeat_disable;
// (depth scale, ln depth ratio, depth max, cosine min); x <= 0: no table (delta-Eddington's diffuse T stands in).
global uniform vec4 cloud_slab_transmittance_map;

// Grid-mean depth from 20 km to layer centre `layer` at (lat, lon) and the cover above it: the four texels around,
// decoded, bilinear.
vec2 cloud_column_texel(sampler3D column, vec3 size, vec2 lat_lon, float layer) {
	float x = lat_lon.y / (2.0 * CLOUD_PI) * size.x;  // column c's centre at longitude c x 360 / columns
	float y = clamp((0.5 * CLOUD_PI - lat_lon.x) / (CLOUD_PI / (size.y - 1.0)), 0.0, size.y - 1.0);
	float x0 = floor(x);
	float y0 = min(floor(y), size.y - 2.0);
	vec2 f = vec2(x - x0, y - y0);
	int columns = int(size.x);
	int c0 = int(mod(x0, size.x));
	int c1 = c0 + 1 < columns ? c0 + 1 : 0;
	int r0 = int(y0);
	int z = int(layer);
	// RGBA8: R the depth code, G the cover (the cloud base in B, A: cloud_lcl_cut).
	vec2 t00 = texelFetch(column, ivec3(c0, r0, z), 0).rg;
	vec2 t10 = texelFetch(column, ivec3(c1, r0, z), 0).rg;
	vec2 t01 = texelFetch(column, ivec3(c0, r0 + 1, z), 0).rg;
	vec2 t11 = texelFetch(column, ivec3(c1, r0 + 1, z), 0).rg;
	vec2 d00 = vec2(cloud_decode_depth(t00.x), t00.y);
	vec2 d10 = vec2(cloud_decode_depth(t10.x), t10.y);
	vec2 d01 = vec2(cloud_decode_depth(t01.x), t01.y);
	vec2 d11 = vec2(cloud_decode_depth(t11.x), t11.y);
	return mix(mix(d00, d10, f.x), mix(d01, d11, f.x), f.y);
}

// One snapshot's grid-mean column above (lat, lon, height): x the depth, y the cover.
vec2 cloud_snapshot_column(sampler3D field, sampler3D column, vec3 size, vec2 lat_lon, float height_m) {
	float s = cloud_layer_coordinate(height_m) - 0.5;
	float k = clamp(floor(s), 0.0, CLOUD_LAYERS - 2.0);
	float x = clamp(s - k, 0.0, 1.0);
	vec4 below = texture(field, cloud_field_uvw(lat_lon, k, size));
	vec4 above = texture(field, cloud_field_uvw(lat_lon, k + 1.0, size));
	float beta_b = cloud_decode_extinction(below);
	float beta_a = cloud_decode_extinction(above);
	float gap = cloud_layer_centre_m(k + 1.0) - cloud_layer_centre_m(k);
	float beta = cloud_profile_value(beta_b, beta_a, below.r, above.r, x, gap);
	float fraction = cloud_profile_value(below.r, above.r, below.r, above.r, x, gap);
	if (s >= CLOUD_LAYERS - 1.0) {
		return vec2(beta_a * max(CLOUD_TOP_M - height_m, 0.0), above.r);  // held from the top centre to 20 km
	}
	float centre = clamp(floor(s) + 1.0, 0.0, CLOUD_LAYERS - 1.0);  // the centre above (0 below centre 0)
	float beta_centre = centre > k ? beta_a : beta_b;
	vec2 at_centre = cloud_column_texel(column, size, lat_lon, centre);
	return vec2(at_centre.x + 0.5 * (beta + beta_centre) * max(cloud_layer_centre_m(centre) - height_m, 0.0),
			max(at_centre.y, fraction));
}

// The blended column above (lat, lon, height) (core cloud_column_depths_above): x the grid-mean depth, y the cover.
vec2 cloud_column_mean_depth(vec2 lat_lon, float height_m) {
	vec2 depth = vec2(0.0);
	if (height_m > CLOUD_TOP_M) {
		return depth;
	}
	if (cloud_blend < 1.0) {
		depth += (1.0 - cloud_blend) * cloud_snapshot_column(cloud_field_before, cloud_column_before,
				cloud_field_before_size, lat_lon, height_m);
	}
	if (cloud_blend > 0.0) {
		depth += cloud_blend * cloud_snapshot_column(cloud_field_after, cloud_column_after,
				cloud_field_after_size, lat_lon, height_m);
	}
	return depth;
}

// The cover to use (core CloudColumnDepths::cover): 0 without cloud, at least half a fraction code step with it.
float cloud_column_cover(vec2 column) {
	return column.x > 0.0 ? clamp(column.y, CLOUD_MIN_FRACTION, 1.0) : 0.0;
}

// The cover of the direct beam of cloud fraction f for the Sun's cosine mu0 to the vertical (core cloud_beam_cover; f is
// the fraction as stored: 0 without cloud).
float cloud_beam_cover_of(float fraction, float mu0) {
	if (fraction <= 0.0) {
		return 0.0;
	}
	float f = clamp(fraction, CLOUD_MIN_FRACTION, 1.0);
	if (f >= 1.0) {
		return 1.0;
	}
	float m = clamp(mu0, 0.02, 1.0);
	float cot = sqrt(max(1.0 - m * m, 0.0)) / m;
	float n = 1.0 + CLOUD_OVERLAP_VERTICAL * (1.0 - f) + CLOUD_OVERLAP_SLANT * max(cot - CLOUD_OVERLAP_ONSET_COT, 0.0);
	return 1.0 - pow(1.0 - f, n);
}

// The beam's cover of a column (x the grid-mean depth, y the largest fraction): 0 without cloud.
float cloud_column_beam_cover(vec2 column, float mu0) {
	return column.x > 0.0 ? cloud_beam_cover_of(column.y, mu0) : 0.0;
}

// The sky's diffuse transmittance (core two_stream_diffuse_transmittance; cloud_sky_factor).
float cloud_two_stream_diffuse(float optical_depth) {
	return 1.0 / (1.0 + 0.75 * (1.0 - cloud_asymmetry_g) * max(optical_depth, 0.0));
}

// The slab's total beam transmittance (core cloud_slab_total_transmittance): the table bilinear, the diffusion
// asymptote beyond its last depth.
float cloud_slab_total_transmittance(float optical_depth, float mu0) {
	if (!(optical_depth > 0.0)) {
		return 1.0;
	}
	if (mu0 <= 0.0) {
		return 0.0;
	}
	vec4 m = cloud_slab_transmittance_map;
	if (m.x <= 0.0) {
		return cloud_two_stream_diffuse(optical_depth);
	}
	vec2 size = vec2(textureSize(cloud_slab_transmittance, 0));
	float x = clamp(log(min(optical_depth, m.z) / m.x + 1.0) / m.y, 0.0, size.x - 1.0);
	float y = clamp((mu0 - m.w) / (1.0 - m.w) * (size.y - 1.0), 0.0, size.y - 1.0);
	float t = texture(cloud_slab_transmittance, (vec2(x, y) + 0.5) / size).r;
	if (optical_depth > m.z) {
		float k = 0.75 * (1.0 - cloud_asymmetry_g);
		t *= (1.0 + k * m.z) / (1.0 + k * optical_depth);
	}
	return t;
}

// The Sun's light diffused through the column (core cloud_sun_diffuse_transmittance) per unit of its horizontal
// irradiance: the cloudy part's slab total minus its direct transmittance, times the cover.
float cloud_sun_diffuse(vec2 column, float mu0) {
	float cover = cloud_column_beam_cover(column, mu0);
	if (mu0 <= 0.0 || cover <= 0.0) {
		return 0.0;
	}
	float tau = column.x / cover;
	return cover * max(cloud_slab_total_transmittance(tau, mu0) - exp(-tau / mu0), 0.0);
}

// The sky's diffuse transmittance of the column (core cloud_sky_factor).
float cloud_sky_factor(vec2 column) {
	float cover = cloud_column_cover(column);
	return cover > 0.0 ? 1.0 - cover + cover * cloud_two_stream_diffuse(column.x / cover) : 1.0;
}

// The direct beam through the column slanted to mu0 (the far tier and the Moon).
float cloud_column_direct(vec2 column, float mu0) {
	float cover = cloud_column_beam_cover(column, mu0);
	return cover > 0.0 ? 1.0 - cover + cover * exp(-column.x / (cover * max(mu0, 0.02))) : 1.0;
}

// The geodetic up (ellipsoid normal) at an Earth-local point (render axes, polar axis +Y).
vec3 cloud_geodetic_up(vec3 p_local) {
	return normalize(vec3(p_local.x, p_local.y * CLOUD_AB2, p_local.z));
}
