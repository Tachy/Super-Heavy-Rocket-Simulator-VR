// The baked cloud volume ("Weg 2", 2026-10-29; the default march since 2026-10-30): the clouds' shape and light precomputed once in a world-fixed
// volume around an anchor (the camera when the bake started) - static clouds, independent of the view direction - and
// each frame a cheap march through it. GLSL 450 only - compiled by CloudBaked (game/world/weather/cloud_baked.gd) after
// CloudEffect's march files (cloud_pass.gdshaderinc's main calls cloud_baked_shade), never included by a Godot shader.
//
// Grid: K nested levels (a clipmap) of N x N columns about the anchor, level k with the horizontal cell c0 2^k and the
// vertical cell dz_k over the band [z_lo, z_hi]. Horizontal coordinates: x = q . axis_x, y = q . axis_y of the
// model-space point q (Earth-local x ATM_STRETCH: the ellipsoid a sphere of radius a), with axis_x / axis_y unit and
// perpendicular to the anchor's up - linear along a straight ray; vertical: the model height |q| - a. All levels are
// stacked along one 3-D texture's depth, each between two guard slices (no cloud).
// Volumes (each set):
//   A  RG16F   x: the signed edge coordinate - (shape value + detail - threshold) / edge width: the old march's soft edge
//              ramp is smoothstep(clamp(x, 0, 1)); interpolated trilinearly it keeps the silhouette crisp at sub-cell
//              precision; y: the extinction scale (1/km): extinction = y x ramp (cell extinction / edge mean)
//   B  RGBA8   sqrt-coded light: x the Sun's transmittance T_s (MC model exp(-tau_s); closed form exp(-k tau_s)), y the
//              Sun's multiple-scattering source J_ms / J_MAX, z the sky's source J_sky / J_MAX (both for the view from
//              the anchor), w unused
//   O  R8      occupancy per block of 4 x 4 x 4 cells (the block or a neighbouring cell may hold cloud): the empty skip
//   S  RGBA16F 2-D array, one layer per volume level at CLOUD_BAKED_SHADOW_SUPERSAMPLE x the columns (shadow step,
//              2026-10-30): the line toward the Sun from the band's floor at each column through the baked shapes (the
//              field's grid mean beyond the levels): x T = exp(-tau), y tau (clamped), z / w tau x the lowest / highest
//              cloudy height on the line (km; premultiplied: a clear neighbour does not move them). The cloud shadow
//              maps (CLOUD_SHADOW_BAKED_FILL, CloudShadowEffect) and the air shadow (cloud_baked_air_sun) read it.
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
// CLOUD_BAKED_FLAT (F and T of one flat level, a range of rows),
// CLOUD_BAKED_MARCH (with CLOUD_MARCH; CLOUD_BAKED_FADE: two sets marched and cross-faded - a re-bake's swap).

#if defined(CLOUD_BAKED_DENSITY) || defined(CLOUD_BAKED_OCCUPANCY) || defined(CLOUD_BAKED_LIGHT) || defined(CLOUD_BAKED_BAND) 		|| defined(CLOUD_BAKED_FLAT) || defined(CLOUD_BAKED_SHADOW)
#define CLOUD_BAKED_BUILD
#endif
#if defined(CLOUD_BAKED_BUILD) || defined(CLOUD_BAKED_MARCH) || defined(CLOUD_SHADOW_BAKED_FILL)

#ifdef CLOUD_BAKED_MARCH
#define CLOUD_BAKED_GRID_BINDING 53
#elif defined(CLOUD_SHADOW_BAKED_FILL)
#define CLOUD_BAKED_GRID_BINDING 2
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
#else
#define CLOUD_BAKED_GRID_BINDING 0
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
#endif

// One set's grid (CloudBaked._set_floats): std140, vec4 only.
struct CloudBakedSet {
	vec4 up0;      // xyz: model-space unit up at the anchor; w: columns per side N
	vec4 axis_x;   // xyz: model-space horizontal axis (unit); w: level 0's horizontal cell c0 (m)
	vec4 axis_y;   // xyz: up0 x axis_x; w: levels K
	vec4 band;     // x: z_lo, y: z_hi (model heights, m); z, w: the camera's grid x, y (m; float64 on the CPU)
	vec4 anchor;   // xyz: the anchor's offset from the camera (Earth-local, m); w: A / B texture depth (slices)
	vec4 dz[2];    // per level: the vertical cell (m)
	vec4 slab[2];  // per level: the texture slice of its lower guard (its cells follow)
	vec4 oslab[2]; // per level: the occupancy texture's first slice
	vec4 onz[2];   // per level: occupancy blocks along the height
	vec4 misc;     // x: occupancy texture depth, y: (set 0) the cross-fade weight of set 1 (CLOUD_BAKED_FADE), z: the
	               // first volume level baked and marched (the finer ones dropped with the altitude), w: -
	vec4 outer;    // x: flat levels, y: the baked cap's angle from up0 (rad), z: the first flat level m0 (absolute: cell
	               // c0 2^m0), w: the per-pixel entry's first guess of the cloud top's height (m)
	// The flat levels' own frame (re-baked less often than the volume): their up, axes (model space, unit) and the
	// anchor their light was baked for (offset from the camera, Earth-local m); w: flat_up0 the columns per side N_f,
	// flat_axis_x the pixel angle (rad) and flat_axis_y the camera height (m) of the geometry's bake, flat_anchor the
	// drift allowance (m) - the bake's rule which columns can be fetched (cloud_baked_flat_needed).
	vec4 flat_up0;
	vec4 flat_axis_x;
	vec4 flat_axis_y;
	vec4 flat_anchor;
};

layout(set = 1, binding = CLOUD_BAKED_GRID_BINDING, std140) uniform CloudBakedGrid {
	CloudBakedSet sets[2];  // the march: 0 the shown set, 1 the set fading in; the bake: 0 the set being baked
	ivec4 build;            // the bake: x pass (level), y first slice of this dispatch, z slices, w: model (1 MC table)
} baked;

// [E] The signed edge coordinate's range (ramp widths): beyond it the value is clamped (stays far from the edge).
const float CLOUD_BAKED_X_MAX = 24.0;
// [E] The sqrt code's range of the sources (per unit irradiance; J of a thick cloud stays below ~1).
const float CLOUD_BAKED_J_MAX = 2.0;
// Occupancy block: cells per block horizontally and vertically.
const int CLOUD_BAKED_BLOCK = 4;
const float CLOUD_BAKED_PER_KM = 1000.0;

float cloud_baked_ramp(float x) {
	x = clamp(x, 0.0, 1.0);
	return x * x * (3.0 - 2.0 * x);
}

// The level holding grid point (gx, gy) (m): the finest whose interior (one cell inside its edge) contains it; K when
// none does.
int cloud_baked_level(int si, float gx, float gy) {
	float r = max(abs(gx), abs(gy));
	float inner = (0.5 * baked.sets[si].up0.w - 1.0) * baked.sets[si].axis_x.w;
	int k = r <= inner ? 0 : int(ceil(log2(r / inner)));
	return min(k, int(baked.sets[si].axis_y.w));
}

// The normalised texture coordinate of grid point (gx, gy, height h) in level k.
vec3 cloud_baked_uvw(int si, int k, float gx, float gy, float h) {
	float n = baked.sets[si].up0.w;
	float c = baked.sets[si].axis_x.w * exp2(float(k));
	float dz = baked.sets[si].dz[k >> 2][k & 3];
	float nz = (baked.sets[si].band.y - baked.sets[si].band.x) / dz;
	vec2 g = clamp(vec2(gx, gy) / c + 0.5 * n, vec2(0.5), vec2(n - 0.5));
	float z = clamp((h - baked.sets[si].band.x) / dz, -0.5, nz + 0.5);
	return vec3(g / n, (baked.sets[si].slab[k >> 2][k & 3] + 1.0 + z) / baked.sets[si].anchor.w);
}

// The extinction (1/m) of a fetched A texel.
float cloud_baked_extinction(vec2 a) {
	return a.y * cloud_baked_ramp(a.x) / CLOUD_BAKED_PER_KM;
}

// [E] The level rule (user 2026-10-29): the data's resolution follows the distance continuously - a sample takes the
// continuous level l = max(log2(r / inner_0), log2(CLOUD_BAKED_LEVEL_DROP x footprint / c_0)): the finest level that
// both holds the point (its clipmap extent, r = max(|x|, |y|) from the anchor) and is not finer than the pixel footprint
// at the sample (cell ~ footprint, like a trilinear mip choice). Its integer part picks level ceil(l); over the last
// cloud_baked_band() of each level (in octaves) the sample blends toward the next (volume and flat levels alike, the
// volume's last into the flat levels). Levels finer than the footprint at the nearest possible cloud are neither baked
// nor marched (CloudBaked.first_level). On the ground and on a deck the extent's level is always the coarser one.
const float CLOUD_BAKED_LEVEL_DROP = 1.0;
// The band (octaves) grows with the camera's height from CLOUD_BAKED_LEVEL_BAND_LOW (the ground and the decks: the
// clipmap's own hand-overs switch, far away and grazing - the approved look; any band there cost ~0.17 ms) to
// CLOUD_BAKED_LEVEL_BAND_HIGH (a whole octave: every sample mixes its two levels, a trilinear choice) between
// CLOUD_BAKED_LEVEL_BAND_HEIGHTS_M: from above, a level's 256 columns end long before the footprint would need the next
// (at the ISS level 3, 800 m, holds +-102 km where the footprint asks for it out to ~1 000 km), so the extent decides
// and a narrow band showed every level's border as a square.
const float CLOUD_BAKED_LEVEL_BAND_LOW = 0.0;
const float CLOUD_BAKED_LEVEL_BAND_HIGH = 1.0;
const vec2 CLOUD_BAKED_LEVEL_BAND_HEIGHTS_M = vec2(20000.0, 60000.0);
// The volume's hand-over to the flat levels blends over at least this band (no seam at the horizon's 410 km).
const float CLOUD_BAKED_FLAT_SEAM_BAND = 0.25;

float cloud_baked_band() {
	return mix(CLOUD_BAKED_LEVEL_BAND_LOW, CLOUD_BAKED_LEVEL_BAND_HIGH,
			smoothstep(CLOUD_BAKED_LEVEL_BAND_HEIGHTS_M.x, CLOUD_BAKED_LEVEL_BAND_HEIGHTS_M.y, cloud_camera_height_m));
}

// The continuous level l of a point at clipmap distance r (m, max norm) and pixel footprint (m).
float cloud_baked_lod(int si, float r, float footprint_m) {
	float c0 = baked.sets[si].axis_x.w;
	float inner0 = (0.5 * baked.sets[si].up0.w - 1.0) * c0;
	return log2(max(max(r, 1.0) / inner0, max(CLOUD_BAKED_LEVEL_DROP * footprint_m, 1.0e-3) / c0));
}

// The volume level of continuous level l (never a dropped one, below k_min) and the blend weight toward the next
// (0 at the volume's last level: the flat levels' hand-over takes it).
int cloud_baked_level_of(int si, float l, int k_min, out float blend) {
	int k = max(int(ceil(l)), k_min);
	float band = cloud_baked_band();
	blend = band > 1.0e-3 ? smoothstep(1.0 - band, 1.0, l - float(k - 1)) : 0.0;
	if (k + 1 >= int(baked.sets[si].axis_y.w)) {
		blend = 0.0;
	}
#ifdef CLOUD_BAKED_NO_BLEND
	blend = 0.0;  // (diagnostics: the levels switched, not blended)
#endif
	return k;
}

// ---- Flat outer levels: shared ----
// The flat levels' coordinates of the Earth-local point p (from the Earth's centre): x, y = (X, Y) (m, azimuthal
// equidistant about up0), z = theta (rad) from up0.
vec3 cloud_baked_flat_xy(int si, vec3 p) {
	vec3 nrm = normalize(p * ATM_STRETCH);
	float cx = dot(nrm, baked.sets[si].flat_axis_x.xyz);
	float cy = dot(nrm, baked.sets[si].flat_axis_y.xyz);
	float s = length(vec2(cx, cy));
	float theta = atan(s, dot(nrm, baked.sets[si].flat_up0.xyz));
	float f = s > 1.0e-9 ? CLOUD_A_M * theta / s : CLOUD_A_M;
	return vec3(cx * f, cy * f, theta);
}

// The model-space unit radial of flat coordinates (X, Y) (m).
vec3 cloud_baked_flat_radial(int si, vec2 g) {
	float r = length(g);
	float theta = r / CLOUD_A_M;
	vec3 tangent = r > 1.0e-3 ? (g.x * baked.sets[si].flat_axis_x.xyz + g.y * baked.sets[si].flat_axis_y.xyz) / r
			: baked.sets[si].flat_axis_x.xyz;
	return cos(theta) * baked.sets[si].flat_up0.xyz + sin(theta) * tangent;
}

// The largest cloud fraction f whose nadir beam cover cloud_beam_cover_of(f, 1) = 1 - (1 - f)^(1 + v (1 - f)) is c
// (v CLOUD_OVERLAP_VERTICAL; fixed-point iterations, ~1e-3).
float cloud_baked_fraction_of(float c) {
	c = clamp(c, 0.0, 0.9999);
	float f = c;
	for (int i = 0; i < 4; i++) {
		f = 1.0 - pow(1.0 - c, 1.0 / (1.0 + CLOUD_OVERLAP_VERTICAL * (1.0 - f)));
	}
	return f;
}

// The edge coordinate whose ramp is r (0 < r < 1): the inverse smoothstep.
float cloud_baked_unramp(float r) {
	return 0.5 - sin(asin(1.0 - 2.0 * r) / 3.0);
}

// [E] The flat levels' tile breaking (step 5; step 4's domain warp sheared the shapes into marbled streaks): over the
// hundreds of km of an orbit view the 32 km shape tile's period drew a regular lattice of dots. The shape is read twice
// - at q and turned (CLOUD_BAKED_BLEND_ROTATION, offset) - and the two readings are blended by a large-scale weight
// (the shape's level-1 mip read at CLOUD_BAKED_BLEND_SCALE of q, mapped through its interquartile range: about a
// quarter of the area each reading alone, half a blend), histogram-preserving (Heitz & Neyret 2018: the blend's
// deviation from the median rescaled by 1 / sqrt(w^2 + (1 - w)^2)) - so the pattern's statistics, hence the prefilter
// thresholds and the mean cover, are kept, and no shear: the shapes stay isotropic.
const float CLOUD_BAKED_BLEND_SCALE = 0.1931;
const mat3 CLOUD_BAKED_BLEND_ROTATION = mat3(0.8130186879, 0.5112918472, -0.2785341274, -0.4537591358,
		0.8561682215, 0.2471408976, 0.3648331945, -0.0745427634, 0.9280841107);
const vec3 CLOUD_BAKED_BLEND_OFFSET = vec3(0.379, 0.617, 0.233);

// The flat levels' heights (step 5; step 4 entered at the column's highest cloudy layer's top: where a thin upper layer
// set in, the entry - and with it the pattern's point - jumped by kilometres along a line, a visible seam). Per column
// the far field's own placement (cloud_far_layer: cloud_far_depth_height of the seen depth) for a nadir view and for
// a view of cosine CLOUD_BAKED_FLAT_GRAZING_MU (the march interpolates in the view's cosine between them), and the
// optical top (the height where the depth from the top is CLOUD_BAKED_FLAT_TOP_DEPTH of the column's): continuous
// in the field.
const float CLOUD_BAKED_FLAT_GRAZING_MU = 0.2;
const float CLOUD_BAKED_FLAT_TOP_DEPTH = 0.05;

// The blend weight of the second reading at shape coordinate q (tile units).
float cloud_baked_blend_weight(vec3 q) {
	float n = textureLod(cloud_noise_shape_mips, q * CLOUD_BAKED_BLEND_SCALE + vec3(0.137, 0.713, 0.371), 0.0).r;
	float q25 = texture(cloud_prefilter_threshold, cloud_prefilter_uv(0.75, 1.0)).r;
	float q75 = texture(cloud_prefilter_threshold, cloud_prefilter_uv(0.25, 1.0)).r;
	return smoothstep(q25, q75, n);
}

// The shape at q (prefilter level) blended with its turned reading by weight w: x the value, y the edge-width noise.
vec2 cloud_baked_shape(vec3 q, float level, float w, float median) {
	vec4 s1 = cloud_shape_prefiltered(q, level);
	if (w <= 0.0) {
		return s1.ra;
	}
	vec4 s2 = cloud_shape_prefiltered(CLOUD_BAKED_BLEND_ROTATION * q + CLOUD_BAKED_BLEND_OFFSET, level);
	float a = 1.0 - w;
	return vec2(median + (a * (s1.r - median) + w * (s2.r - median)) * inversesqrt(a * a + w * w), mix(s1.a, s2.a, w));
}

// ---- The Sun's transmittance from the bake (S; shadow step 2026-10-30) ----------------------------------------------
// [E] S columns per volume column and axis: level 0's 100 m cells give 50 m shadow texels (the old near map: 62.5 m);
// the shapes' sub-cell edges survive through the edge coordinate's interpolation in the march toward the Sun.
const int CLOUD_BAKED_SHADOW_SUPERSAMPLE = 2;
// [E] The optical depth S keeps at most (exp(-64): black) - the premultiplied heights stay within half-float range.
const float CLOUD_BAKED_SHADOW_TAU_MAX = 64.0;

// The ray from camera + offset (Earth-local, m) along dir (unit, Earth-local): cloud_ray with the point's own height
// and up (distances along it in Earth-local metres, as cloud_sphere_crossing / cloud_height_along return them).
CloudRay cloud_baked_point_ray(vec3 offset, vec3 dir) {
	CloudRay ray = cloud_ray(dir);
	vec3 pm = (atmosphere_camera_hi + atmosphere_camera_lo + offset) * ATM_STRETCH;
	ray.r_c = length(pm);
	ray.h_c = ray.r_c - CLOUD_A_M;
	ray.mu = dot(pm / ray.r_c, dir * ATM_STRETCH / ray.scale);
	return ray;
}

// The S texture coordinate (normalised x, y; the layer) of grid point (gx, gy) (m) in volume level k.
vec3 cloud_baked_s_uvl(int si, int k, float gx, float gy) {
	float n = baked.sets[si].up0.w * float(CLOUD_BAKED_SHADOW_SUPERSAMPLE);
	float c = baked.sets[si].axis_x.w * exp2(float(k)) / float(CLOUD_BAKED_SHADOW_SUPERSAMPLE);
	vec2 g = clamp(vec2(gx, gy) / c + 0.5 * n, vec2(0.5), vec2(n - 0.5));
	return vec3(g / n, float(k));
}

#if defined(CLOUD_BAKED_MARCH) || defined(CLOUD_SHADOW_BAKED_FILL)
#ifdef CLOUD_BAKED_MARCH
layout(set = 1, binding = 61) uniform sampler2DArray baked_s0;  // linear
#ifdef CLOUD_BAKED_FADE
layout(set = 1, binding = 62) uniform sampler2DArray baked_s1;
#define CLOUD_BAKED_S_TWO_SETS
#endif
#else
layout(set = 1, binding = 3) uniform sampler2DArray baked_s0;  // linear
layout(set = 1, binding = 4) uniform sampler2DArray baked_s1;  // (set 0's again without a cross-fade)
#define CLOUD_BAKED_S_TWO_SETS
#endif

vec4 cloud_baked_s(int si, vec3 uvl) {
#ifdef CLOUD_BAKED_S_TWO_SETS
	if (si == 1) {
		return textureLod(baked_s1, uvl, 0.0);
	}
#endif
	return textureLod(baked_s0, uvl, 0.0);
}

// The line toward the Sun (sun: unit, Earth-local) through camera + offset in set si's S, read where it rises through
// the band's floor (the closest approach to it when the line passes above the floor: a grazing Sun): x its cloud
// transmittance (below its clouds), y / z the lowest / highest cloudy height on it (m), w the floor point's distance
// along the line from the point (m, negative behind it). False where that point lies beyond the volume levels (or all
// of them are dropped): no shapes baked there. footprint_m: the lookup's footprint at the floor (m; 0: the finest
// level holding the point) - a coarser level whose S cell covers it (each level's S is baked through its own or
// coarser shapes: a prefiltered shadow), so sparse samples of a far stretch do not alias the shapes' shadows.
bool cloud_baked_shadow_line(int si, vec3 offset, vec3 sun, float footprint_m, out vec4 line) {
	CloudRay ray = cloud_baked_point_ray(offset, sun);
	vec2 floor_c = cloud_sphere_crossing(ray, baked.sets[si].band.x);
	float t0 = floor_c.x <= floor_c.y ? floor_c.y : -ray.r_c * ray.mu / ray.scale;
	vec3 q = offset + sun * t0;
	float gx = baked.sets[si].band.z + dot(q, baked.sets[si].axis_x.xyz * ATM_STRETCH);
	float gy = baked.sets[si].band.w + dot(q, baked.sets[si].axis_y.xyz * ATM_STRETCH);
	int levels = int(baked.sets[si].axis_y.w);
	int k = max(cloud_baked_level(si, gx, gy), int(baked.sets[si].misc.z));
	// the footprint's continuous level (blended between its two levels: no step between pixel rows)
	float s_cell0 = baked.sets[si].axis_x.w / float(CLOUD_BAKED_SHADOW_SUPERSAMPLE);
	float l = footprint_m > s_cell0 ? log2(footprint_m / s_cell0) : 0.0;
	int k0 = max(k, int(floor(l)));
	float blend = k0 > k || l >= float(k) ? fract(l) : 0.0;
	if (k0 >= levels) {
		line = vec4(1.0, 0.0, 0.0, t0);
		return false;
	}
	vec4 s = cloud_baked_s(si, cloud_baked_s_uvl(si, k0, gx, gy));
	if (blend > 0.0 && k0 + 1 < levels) {
		s = mix(s, cloud_baked_s(si, cloud_baked_s_uvl(si, k0 + 1, gx, gy)), blend);
	}
	float tau = max(s.y, 1.0e-6);
	line = vec4(clamp(s.x, 0.0, 1.0), 1000.0 * s.z / tau, 1000.0 * s.w / tau, t0);
	return true;
}

// The Sun's cloud transmittance at height h (m) on a line of cloud_baked_shadow_line: the whole line's below its lowest
// cloud, 1 above its highest, exp(-tau) scaled linearly in the height between (the old maps' rule inside a cloud).
float cloud_baked_shadow_at_height(vec4 line, float h) {
	if (line.x >= 1.0 || h <= line.y) {
		return line.x;
	}
	if (h >= line.z) {
		return 1.0;
	}
	return pow(max(line.x, 1.0e-30), (line.z - h) / max(line.z - line.y, 1.0));
}
#endif

#endif

// ---- The bake ------------------------------------------------------------------------------------------------------
#ifdef CLOUD_BAKED_BUILD

// The camera offset (Earth-local, m) of grid point (x, y) (m) at model height h.
vec3 cloud_baked_offset_of(int si, float x, float y, float h) {
	float r = CLOUD_A_M + h;
	vec3 q = x * baked.sets[si].axis_x.xyz + y * baked.sets[si].axis_y.xyz + sqrt(max(r * r - x * x - y * y, 0.0)) * baked.sets[si].up0.xyz;
	return q / ATM_STRETCH - (atmosphere_camera_hi + atmosphere_camera_lo);
}

// The voxel of this invocation in level baked.build.x: its cell (i, j, layer) and texel; false outside the level.
bool cloud_baked_voxel(int si, out ivec3 cell, out ivec3 texel) {
	int k = baked.build.x;
	texel = ivec3(gl_GlobalInvocationID.xy, int(gl_GlobalInvocationID.z) + baked.build.y);
	int n = int(baked.sets[si].up0.w);
	int nz = int(round((baked.sets[si].band.y - baked.sets[si].band.x) / baked.sets[si].dz[k >> 2][k & 3]));
	cell = ivec3(texel.xy, texel.z);
	texel.z += int(baked.sets[si].slab[k >> 2][k & 3]) + 1;
	return texel.x < n && texel.y < n && cell.z < nz && int(gl_GlobalInvocationID.z) < baked.build.z;
}

// The grid point (m) and height of a cell's centre in level k.
vec3 cloud_baked_cell_centre(int si, int k, ivec3 cell) {
	float c = baked.sets[si].axis_x.w * exp2(float(k));
	float dz = baked.sets[si].dz[k >> 2][k & 3];
	return vec3((vec2(cell.xy) + 0.5 - 0.5 * baked.sets[si].up0.w) * c, baked.sets[si].band.x + (float(cell.z) + 0.5) * dz);
}
#endif

#ifdef CLOUD_BAKED_BAND
layout(set = 1, binding = 1, std430) buffer CloudBakedBand {
	int lowest;   // the lowest field layer with cloud (atomicMin; 32: none)
	int highest;  // the highest (atomicMax; -1: none)
} baked_band;
// Columns per axis of the scan over the outermost level (the field's 0.25 deg cells are ~28 km: a column every ~13 km).
const int CLOUD_BAKED_BAND_COLUMNS = 64;

void main() {
	const int si = 0;
	ivec2 id = ivec2(gl_GlobalInvocationID.xy);
	if (id.x >= CLOUD_BAKED_BAND_COLUMNS || id.y >= CLOUD_BAKED_BAND_COLUMNS) {
		return;
	}
	float half_size = 0.5 * baked.sets[si].up0.w * baked.sets[si].axis_x.w * exp2(baked.sets[si].axis_y.w - 1.0);
	vec2 xy = ((vec2(id) + 0.5) / float(CLOUD_BAKED_BAND_COLUMNS) * 2.0 - 1.0) * half_size;
	for (int layer = 0; layer < int(CLOUD_LAYERS); layer++) {
		float h = cloud_layer_centre_m(float(layer));
		vec3 offset = cloud_baked_offset_of(si, xy.x, xy.y, h);
		CloudCell cell = cloud_field(cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset), h);
		if (cell.extinction > 0.0) {
			atomicMin(baked_band.lowest, layer);
			atomicMax(baked_band.highest, layer);
		}
	}
}
#endif

#ifdef CLOUD_BAKED_DENSITY
layout(rg16f, set = 1, binding = 1) uniform writeonly image3D baked_a_out;

// The old march's soft-edge ramp at camera + offset (cloud_extinction_in_cell's terms, detail band-limited at
// footprint_m): x the ramp sum (types mixed by their weights), y the largest signed edge coordinate of the types
// present, z the extinction scale (1/m), w 1 where the field has cloud here. extra_state: detail levels the march adds
// per sample (CLOUD_BAKED_DETAIL: level 1 in level 0) - the threshold and the edge mean are those of the state with
// them (their erosion is the march's), so the march's sum keeps the cloud fraction (fix 1: with the bake's own state's
// threshold the march's erosion shrank every near outline by tens of metres and tore its rim into fragments).
vec4 cloud_baked_ramp_at(vec3 offset, float h, float footprint_m, float extra_state) {
	vec2 lat_lon = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset);
	CloudCell cell = cloud_field(lat_lon, h);
	if (cell.fraction < CLOUD_MIN_FRACTION || cell.extinction <= 0.0) {
		return vec4(0.0, -CLOUD_BAKED_X_MAX, 0.0, 0.0);
	}
	float state = cloud_detail_state(footprint_m);
	float state_t = min(state + extra_state, float(cloud_detail_levels));
	float threshold = cloud_threshold(cell.fraction, state_t);
	vec3 types = cloud_type_weights(cell);
	float detail = state > 0.0 ? cloud_detail_perturbation(offset, state) : 0.0;
	float ramp = 0.0;
	float x_max = -CLOUD_BAKED_X_MAX;
	for (int type = 0; type < 3; type++) {
		if (types[type] > 0.0) {
			vec4 shape = texture(cloud_noise_shape, cloud_type_shape_coordinates(type, offset, h));
			float w = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, clamp(shape.a, 0.0, 1.0));
			float x = (shape.r + detail - threshold) / max(w, 1.0e-6);
			ramp += types[type] * cloud_baked_ramp(x);
			x_max = max(x_max, x);
		}
	}
	float m = texture(cloud_edge_mean, cloud_table_uv(cell.fraction, state_t)).r;
	return vec4(ramp, x_max, cell.extinction / max(m, 1.0e-4), 1.0);
}

void main() {
	const int si = 0;
	ivec3 cell;
	ivec3 texel;
	if (!cloud_baked_voxel(si, cell, texel)) {
		return;
	}
	int k = baked.build.x;
	vec3 g = cloud_baked_cell_centre(si, k, cell);
	float c = baked.sets[si].axis_x.w * exp2(float(k));
	float dz = baked.sets[si].dz[k >> 2][k & 3];
	// The voxel's box average of the ramp (its coverage kept, not a mean field value): taps spaced at most about half
	// a shape texel - horizontally 125 m (cumulus texels 250 m), vertically 60 m (125 m), 20 m where the field holds
	// ice (cirrus texels 16 m high) - capped. A voxel whose field has no cloud at its bottom, middle and top is clear.
	vec3 offset_c = cloud_baked_offset_of(si, g.x, g.y, g.z);
	vec2 lat_lon_c = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_c);
	CloudCell lo_cell = cloud_field(lat_lon_c, g.z - 0.5 * dz);
	CloudCell mid_cell = cloud_field(lat_lon_c, g.z);
	CloudCell hi_cell = cloud_field(lat_lon_c, g.z + 0.5 * dz);
	if (lo_cell.extinction <= 0.0 && mid_cell.extinction <= 0.0 && hi_cell.extinction <= 0.0) {
		imageStore(baked_a_out, texel, vec4(-CLOUD_BAKED_X_MAX, 0.0, 0.0, 0.0));
		return;
	}
	bool ice = max(max(lo_cell.ice, mid_cell.ice), hi_cell.ice) > 0.0;
	int taps_h = clamp(int(ceil(c / 125.0)), 1, 3);
	int taps_v = clamp(int(ceil(dz / (ice ? 20.0 : 60.0))), 1, ice ? 16 : 8);
	float ramp = 0.0;
	float x_max = -CLOUD_BAKED_X_MAX;
	float scale = 0.0;
	float scale_n = 0.0;
	// The march's per-sample detail level (level 1 in level 0): its full weight (the march's at any footprint it has
	// in level 0, <= ~13 m at divisor 2), counted in the threshold's state.
#ifdef CLOUD_BAKED_DETAIL
	float extra_state = k == 0 && cloud_detail_levels > 1 ? 1.0 : 0.0;
#else
	const float extra_state = 0.0;
#endif
	for (int kz = 0; kz < taps_v; kz++) {
		for (int ky = 0; ky < taps_h; ky++) {
			for (int kx = 0; kx < taps_h; kx++) {
				vec3 f = (vec3(kx, ky, kz) + 0.5) / vec3(taps_h, taps_h, taps_v) - 0.5;
				vec3 p = g + f * vec3(c, c, dz);
				vec4 r = cloud_baked_ramp_at(cloud_baked_offset_of(si, p.x, p.y, p.z), p.z, 0.5 * c, extra_state);
				ramp += r.x;
				x_max = max(x_max, r.y);
				scale += r.z;
				scale_n += r.w;
			}
		}
	}
	float n = float(taps_h * taps_h * taps_v);
	ramp /= n;
	// The signed coordinate: inside the ramp's range its inverse (the ramp sum is what the march must reproduce), at
	// the ends the largest type's own (continuous: the distance to the edge for the interpolation).
	float x = ramp <= 0.0 ? min(x_max, 0.0) : (ramp >= 1.0 ? max(x_max, 1.0) : cloud_baked_unramp(ramp));
	// The scale: the mean over the taps with field cloud (the clear taps lower the ramp's mean instead).
	float scale_mean = scale_n > 0.0 ? scale / scale_n : 0.0;
	imageStore(baked_a_out, texel, vec4(clamp(x, -CLOUD_BAKED_X_MAX, CLOUD_BAKED_X_MAX),
			scale_mean * CLOUD_BAKED_PER_KM, 0.0, 0.0));
}
#endif

#ifdef CLOUD_BAKED_OCCUPANCY
layout(r8, set = 1, binding = 1) uniform writeonly image3D baked_o_out;
layout(set = 1, binding = 2) uniform sampler3D baked_a_in;  // nearest

void main() {
	const int si = 0;
	int k = baked.build.x;
	int n = int(baked.sets[si].up0.w);
	int nb = n / CLOUD_BAKED_BLOCK;
	int nz = int(round((baked.sets[si].band.y - baked.sets[si].band.x) / baked.sets[si].dz[k >> 2][k & 3]));
	int nbz = int(baked.sets[si].onz[k >> 2][k & 3]);
	ivec3 b = ivec3(gl_GlobalInvocationID.xy, int(gl_GlobalInvocationID.z) + baked.build.y);
	if (b.x >= nb || b.y >= nb || b.z >= nbz || int(gl_GlobalInvocationID.z) >= baked.build.z) {
		return;
	}
	// The block's cells and one cell around (the trilinear fetch's reach): any with extinction scale > 0 and x > 0
	// (a trilinear mix can only be cloud where a corner is).
	bool occupied = false;
	ivec3 lo = b * CLOUD_BAKED_BLOCK - 1;
	ivec3 hi = b * CLOUD_BAKED_BLOCK + CLOUD_BAKED_BLOCK;
	int slab = int(baked.sets[si].slab[k >> 2][k & 3]) + 1;
	for (int z = lo.z; z <= hi.z && !occupied; z++) {
		if (z < 0 || z >= nz) {
			continue;
		}
		for (int y = max(lo.y, 0); y <= min(hi.y, n - 1) && !occupied; y++) {
			for (int x = max(lo.x, 0); x <= min(hi.x, n - 1); x++) {
				vec2 a = texelFetch(baked_a_in, ivec3(x, y, slab + z), 0).rg;
				if (a.x > 0.0 && a.y > 0.0) {
					occupied = true;
					break;
				}
			}
		}
	}
	imageStore(baked_o_out, ivec3(b.xy, int(baked.sets[si].oslab[k >> 2][k & 3]) + b.z), vec4(occupied ? 1.0 : 0.0));
}
#endif

#ifdef CLOUD_BAKED_LIGHT
layout(rgba8, set = 1, binding = 1) uniform writeonly image3D baked_b_out;
layout(set = 1, binding = 2) uniform sampler3D baked_a_in;  // linear
layout(set = 1, binding = 3) uniform sampler3D baked_o_in;  // nearest

// [E] Samples of the Sun's path: CLOUD_BAKED_SUN_NEAR_SAMPLES through the baked shapes over its first
// CLOUD_BAKED_SUN_SHAPE_PATH_M (quadratic spacing: dense at the voxel, the first interval L / n^2), then
// CLOUD_BAKED_SUN_FAR_SAMPLES equal intervals through the field's grid mean to the band's top (at most
// CLOUD_BAKED_SUN_PATH_MAX_M). The split is the light model of the old march and the core
// (cloud_light_optical_depth: near_m 6 km through the shapes, the presets' light_near_m, then the grid mean): through
// the shapes all the way, a grazing Sun path (low Sun, broken cirrus: 10-100 km in the layer) found the shapes' gaps
// and the baked cirrus was 1.14 x the old march's brightness per alpha toward a 5 deg Sun (step 7).
const int CLOUD_BAKED_SUN_NEAR_SAMPLES = 24;
const int CLOUD_BAKED_SUN_FAR_SAMPLES = 8;
const float CLOUD_BAKED_SUN_SHAPE_PATH_M = 6000.0;
const float CLOUD_BAKED_SUN_PATH_MAX_M = 300000.0;

// The optical depth toward the Sun from camera + offset (height h): the baked volume's extinction where a level holds
// the point over the near part, the field's grid mean beyond; up to the band's top (0 when the path meets the ground:
// Earth's shadow is atm_sunlit's / the Sun's transmittance's).
float cloud_baked_sun_depth(int si, vec3 offset, vec3 sun) {
	vec3 axis_x = baked.sets[si].axis_x.xyz * ATM_STRETCH;
	vec3 axis_y = baked.sets[si].axis_y.xyz * ATM_STRETCH;
	CloudRay ray = cloud_ray(sun);
	// cloud_ray is the camera's ray: the start's height and up from the point instead
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset;
	vec3 pm = p * ATM_STRETCH;
	ray.r_c = length(pm);
	ray.h_c = ray.r_c - CLOUD_A_M;
	ray.mu = dot(pm / ray.r_c, sun * ATM_STRETCH / ray.scale);
	vec2 top = cloud_sphere_crossing(ray, baked.sets[si].band.y);
	float length_m = min(max(top.y, 0.0), CLOUD_BAKED_SUN_PATH_MAX_M);
	if (length_m <= 0.0) {
		return 0.0;
	}
	vec2 ground = cloud_sphere_crossing(ray, 0.0);
	if (ground.x <= ground.y && ground.x > 0.0 && ground.x < length_m) {
		return 1.0e4;  // below the horizon: no direct Sun (the light is the sky's)
	}
	float gx0 = baked.sets[si].band.z + dot(offset, axis_x);
	float gy0 = baked.sets[si].band.w + dot(offset, axis_y);
	float gdx = dot(sun, axis_x);
	float gdy = dot(sun, axis_y);
	float tau = 0.0;
	float near_m = min(length_m, CLOUD_BAKED_SUN_SHAPE_PATH_M);
	float n = float(CLOUD_BAKED_SUN_NEAR_SAMPLES);
	for (int i = 0; i < CLOUD_BAKED_SUN_NEAR_SAMPLES; i++) {
		float a = near_m * float(i * i) / (n * n);
		float b = near_m * float((i + 1) * (i + 1)) / (n * n);
		float t = 0.5 * (a + b);
		float h = cloud_height_along(ray, t);
		float gx = gx0 + t * gdx;
		float gy = gy0 + t * gdy;
		int k = max(cloud_baked_level(si, gx, gy), int(baked.sets[si].misc.z));
		float beta;
		if (k < int(baked.sets[si].axis_y.w)) {
			beta = cloud_baked_extinction(texture(baked_a_in, cloud_baked_uvw(si, k, gx, gy, h)).rg);
		} else {
			beta = cloud_mean_extinction_at(offset + sun * t, h);
		}
		tau += beta * (b - a);
	}
	if (length_m > near_m) {
		float step_m = (length_m - near_m) / float(CLOUD_BAKED_SUN_FAR_SAMPLES);
		for (int i = 0; i < CLOUD_BAKED_SUN_FAR_SAMPLES; i++) {
			float t = near_m + (float(i) + 0.5) * step_m;
			tau += cloud_mean_extinction_at(offset + sun * t, cloud_height_along(ray, t)) * step_m;
		}
	}
	return tau;
}

void main() {
	const int si = 0;
	ivec3 cell;
	ivec3 texel;
	if (!cloud_baked_voxel(si, cell, texel)) {
		return;
	}
	int k = baked.build.x;
	// Voxels outside every occupied block (and its neighbours') are never read where there is cloud: no light.
	ivec3 b = cell / CLOUD_BAKED_BLOCK;
	int nb = int(baked.sets[si].up0.w) / CLOUD_BAKED_BLOCK;
	int nbz = int(baked.sets[si].onz[k >> 2][k & 3]);
	bool near_cloud = false;
	for (int z = max(b.z - 1, 0); z <= min(b.z + 1, nbz - 1); z++) {
		for (int y = max(b.y - 1, 0); y <= min(b.y + 1, nb - 1); y++) {
			for (int x = max(b.x - 1, 0); x <= min(b.x + 1, nb - 1); x++) {
				near_cloud = near_cloud || texelFetch(baked_o_in, ivec3(x, y, int(baked.sets[si].oslab[k >> 2][k & 3]) + z), 0).r > 0.5;
			}
		}
	}
	if (!near_cloud) {
		imageStore(baked_b_out, texel, vec4(1.0, 0.0, 0.0, 0.0));
		return;
	}
	vec3 g = cloud_baked_cell_centre(si, k, cell);
	vec3 offset = cloud_baked_offset_of(si, g.x, g.y, g.z);
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset;
	vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);  // the ellipsoid's normal (geodetic up)
	vec3 sun = normalize(atmosphere_sun_direction);
	float tau_s = cloud_baked_sun_depth(si, offset, sun);
	// The column terms (the old march's: the field's grid-mean columns) and the view from the anchor.
	vec2 lat_lon = cloud_lat_lon(p);
	CloudCell c = cloud_field(lat_lon, g.z);
	float in_cloud = c.fraction >= CLOUD_MIN_FRACTION ? c.extinction / c.fraction : 0.0;
	vec2 column = cloud_column_depths(c, lat_lon, g.z, in_cloud);
	vec3 to_anchor = baked.sets[si].anchor.xyz - offset;
	vec3 view_prop = length(to_anchor) > 1.0 ? normalize(to_anchor) : normal;
	vec3 diff = sun + view_prop;
	float one_minus_cos = 0.5 * dot(diff, diff);
	float phase = cloud_phase_function(1.0 - one_minus_cos, one_minus_cos);
	float transmittance;
	float j_sun;
	if (baked.build.w == 1) {
		transmittance = exp(-tau_s);
		j_sun = cloud_scattering_source_mc(sun, view_prop, normal, tau_s, column.x, column.y);
	} else {
		transmittance = exp(-cloud_transport_scale() * tau_s);
		j_sun = cloud_scattering_source_3d(sun, view_prop, normal, tau_s, column.x, column.y);
	}
	float j_ms = max(j_sun - phase * transmittance, 0.0);
	float j_sky = cloud_sky_scattering_source(dot(view_prop, normal), column.x, column.y);
	// The edge width noise (the shapes' A channel, types mixed) for the march's per-sample detail.
	vec3 types = cloud_type_weights(c);
	float width_noise = 0.0;
	for (int type = 0; type < 3; type++) {
		if (types[type] > 0.0) {
			width_noise += types[type] * texture(cloud_noise_shape, cloud_type_shape_coordinates(type, offset, g.z)).a;
		}
	}
	imageStore(baked_b_out, texel, vec4(sqrt(clamp(transmittance, 0.0, 1.0)),
			sqrt(clamp(j_ms / CLOUD_BAKED_J_MAX, 0.0, 1.0)), sqrt(clamp(j_sky / CLOUD_BAKED_J_MAX, 0.0, 1.0)),
			clamp(width_noise, 0.0, 1.0)));
}
#endif

#ifdef CLOUD_BAKED_SHADOW
// S of one volume level, a range of rows (baked.build: x the level, y the first row, z the rows). Per column the line
// toward the Sun from the band's floor to its top (at most CLOUD_BAKED_SHADOW_PATH_MAX_M) through the baked shapes -
// the extinction the march draws, in this level or the coarser one holding each point (a prefiltered shadow: the
// whole level is baked, so a coarser level's S also serves the air shadow's far, sparse samples inside the finer
// levels' extent) - and the field's grid mean beyond the levels (as the light bake's far part). The light bake's own T_s (B.x) is no shadow volume: it exists only around occupied
// blocks (clear voxels store 1) and takes the shapes over the first 6 km only (the core's light model for a cloud's
// own light); a cloud's shadow is the line through its shapes, however long.
// March: CLOUD_BAKED_SHADOW_COARSE intervals, each looked up in the occupancy blocks; an interval whose block or a
// neighbouring interval's may hold cloud is sampled CLOUD_BAKED_SHADOW_FINE times through A (midpoint rule), the
// others are clear; intervals beyond the levels take one grid-mean sample. Stops once tau > CLOUD_BAKED_SHADOW_TAU_STOP.
// A line meeting the ground before its top (the Sun below the floor's horizon) stores no cloud: that light is the
// Earth's shadow (the atmosphere's), not the clouds'.
layout(rgba16f, set = 1, binding = 1) uniform writeonly image2DArray baked_s_out;
layout(set = 1, binding = 2) uniform sampler3D baked_a_in;  // linear
layout(set = 1, binding = 3) uniform sampler3D baked_o_in;  // nearest

// [E] Coarse intervals per line (a 2 km layer at 45 deg: ~45 m apart through the 0-16.8 km ERA5 band ~190 m, below the
// occupancy block's 400 m), fine samples per interval near occupied blocks, the optical depth to stop at, the longest
// line (a grazing Sun; the light bake's CLOUD_BAKED_SUN_PATH_MAX_M).
const int CLOUD_BAKED_SHADOW_COARSE = 128;
const int CLOUD_BAKED_SHADOW_FINE = 8;
const float CLOUD_BAKED_SHADOW_TAU_STOP = 30.0;
const float CLOUD_BAKED_SHADOW_PATH_MAX_M = 300000.0;

// What the line meets at grid point (gx, gy), height h: 0 clear (an empty occupancy block, or outside the band), 1 a
// block that may hold cloud, 2 beyond the volume levels.
int cloud_baked_shadow_kind(int si, float gx, float gy, float h) {
	int levels = int(baked.sets[si].axis_y.w);
	int k = max(cloud_baked_level(si, gx, gy), baked.build.x);
	if (k >= levels) {
		return 2;
	}
	float n = baked.sets[si].up0.w;
	float c = baked.sets[si].axis_x.w * exp2(float(k));
	float dz = baked.sets[si].dz[k >> 2][k & 3];
	float z = (h - baked.sets[si].band.x) / dz;
	int nbz = int(baked.sets[si].onz[k >> 2][k & 3]);
	if (z < 0.0 || int(z) / CLOUD_BAKED_BLOCK >= nbz) {
		return 0;
	}
	int nb = int(n) / CLOUD_BAKED_BLOCK;
	ivec2 b = clamp(ivec2(floor(vec2(gx, gy) / c + 0.5 * n)) / CLOUD_BAKED_BLOCK, ivec2(0), ivec2(nb - 1));
	return texelFetch(baked_o_in, ivec3(b, int(baked.sets[si].oslab[k >> 2][k & 3]) + int(z) / CLOUD_BAKED_BLOCK), 0).r
			> 0.5 ? 1 : 0;
}

// The extinction (1/m) the line meets at camera + offset (grid point gx, gy; height h): the baked shapes in the finest
// level holding the point, the grid mean beyond the levels.
float cloud_baked_shadow_beta(int si, vec3 offset, float gx, float gy, float h) {
	int k = max(cloud_baked_level(si, gx, gy), baked.build.x);
	if (k >= int(baked.sets[si].axis_y.w)) {
		return cloud_mean_extinction_at(offset, h);
	}
	return cloud_baked_extinction(texture(baked_a_in, cloud_baked_uvw(si, k, gx, gy, h)).rg);
}

void main() {
	const int si = 0;
	int k = baked.build.x;
	int ns = int(baked.sets[si].up0.w) * CLOUD_BAKED_SHADOW_SUPERSAMPLE;
	ivec2 col = ivec2(gl_GlobalInvocationID.x, int(gl_GlobalInvocationID.y) + baked.build.y);
	if (col.x >= ns || col.y >= ns || int(gl_GlobalInvocationID.y) >= baked.build.z) {
		return;
	}
	ivec3 texel = ivec3(col, k);
	float c = baked.sets[si].axis_x.w * exp2(float(k)) / float(CLOUD_BAKED_SHADOW_SUPERSAMPLE);
	vec2 g = (vec2(col) + 0.5 - 0.5 * float(ns)) * c;
	float z_lo = baked.sets[si].band.x;
	vec3 offset = cloud_baked_offset_of(si, g.x, g.y, z_lo);
	vec3 sun = normalize(atmosphere_sun_direction);
	CloudRay ray = cloud_baked_point_ray(offset, sun);
	float length_m = min(max(cloud_sphere_crossing(ray, baked.sets[si].band.y).y, 0.0), CLOUD_BAKED_SHADOW_PATH_MAX_M);
	vec2 ground = cloud_sphere_crossing(ray, -1.0);
	if (length_m <= 0.0 || (ground.x <= ground.y && ground.x > 0.0 && ground.x < length_m)) {
		imageStore(baked_s_out, texel, vec4(1.0, 0.0, 0.0, 0.0));
		return;
	}
	float gdx = dot(sun, baked.sets[si].axis_x.xyz * ATM_STRETCH);
	float gdy = dot(sun, baked.sets[si].axis_y.xyz * ATM_STRETCH);
	float dt = length_m / float(CLOUD_BAKED_SHADOW_COARSE);
	float tau = 0.0;
	float h_lo = 1.0e9;
	float h_hi = -1.0e9;
	int kind_prev = 0;
	float t_c = 0.5 * dt;
	int kind = cloud_baked_shadow_kind(si, g.x + t_c * gdx, g.y + t_c * gdy, cloud_height_along(ray, t_c));
	for (int i = 0; i < CLOUD_BAKED_SHADOW_COARSE; i++) {
		int kind_next = 0;
		if (i + 1 < CLOUD_BAKED_SHADOW_COARSE) {
			float t_n = (float(i) + 1.5) * dt;
			kind_next = cloud_baked_shadow_kind(si, g.x + t_n * gdx, g.y + t_n * gdy, cloud_height_along(ray, t_n));
		}
		if (kind == 2) {
			float t = (float(i) + 0.5) * dt;
			float h = cloud_height_along(ray, t);
			float beta = cloud_mean_extinction_at(offset + sun * t, h);
			if (beta > 0.0) {
				tau += beta * dt;
				h_lo = min(h_lo, h);
				h_hi = max(h_hi, h);
			}
		} else if (kind_prev == 1 || kind == 1 || kind_next == 1) {
			float step_m = dt / float(CLOUD_BAKED_SHADOW_FINE);
			for (int j = 0; j < CLOUD_BAKED_SHADOW_FINE; j++) {
				float t = float(i) * dt + (float(j) + 0.5) * step_m;
				float h = cloud_height_along(ray, t);
				float beta = cloud_baked_shadow_beta(si, offset + sun * t, g.x + t * gdx, g.y + t * gdy, h);
				if (beta > 0.0) {
					tau += beta * step_m;
					h_lo = min(h_lo, h);
					h_hi = max(h_hi, h);
				}
			}
		}
		if (tau > CLOUD_BAKED_SHADOW_TAU_STOP) {
			break;
		}
		kind_prev = kind;
		kind = kind_next;
	}
	if (tau <= 0.0) {
		imageStore(baked_s_out, texel, vec4(1.0, 0.0, 0.0, 0.0));
		return;
	}
	float tau_c = min(tau, CLOUD_BAKED_SHADOW_TAU_MAX);
	imageStore(baked_s_out, texel, vec4(exp(-tau), tau_c, tau_c * h_lo / 1000.0, tau_c * h_hi / 1000.0));
}
#endif

#ifdef CLOUD_BAKED_FLAT
// [E] The flat geometry's shape taps per column (the bake's cost: from orbit every level holds ~2 / pixel angle columns
// across): 3 x 3 at one field point up to CLOUD_BAKED_FLAT_TAP_CELLS_M.y of cell (CloudBaked FLAT_TAP_CELLS_M; 2 x 2
// left a deck mottled at 800 m cells: the 250 m shape texels undersampled), beyond 4 x 4 at 2 x 2 field points (the
// field's 0.25 deg cells: 28 km). (.x: unused.)
const vec2 CLOUD_BAKED_FLAT_TAP_CELLS_M = vec2(1600.0, 14000.0);
// [E] The flat geometry's height range (CloudBaked FLAT_HEIGHT_RANGE) and the pixel angle's (a re-bake beyond either):
// a level's columns are baked only where some camera in that range (and within the drift) can need them.
const float CLOUD_BAKED_FLAT_HEIGHT_RANGE = 1.25;
const float CLOUD_BAKED_FLAT_PIXEL_RANGE = 1.25;
// [E] Octaves of margin of that rule (the band blends a level in from 1 octave below it at most).
const float CLOUD_BAKED_FLAT_SKIP_MARGIN = 0.5;

// The flat level's column (absolute level m = m0 + j) at grid g (m, azimuthal equidistant): false when no camera of the
// geometry's height range and drift can fetch it (beyond the cap; or inside the finer level's interior while the
// footprint there stays at least CLOUD_BAKED_FLAT_SKIP_MARGIN below where this level blends in) - written empty.
bool cloud_baked_flat_needed(int si, int j, vec2 g, float c) {
	if ((length(g) - 0.71 * c) / CLOUD_A_M > baked.sets[si].outer.y) {
		return false;
	}
	if (j == 0) {
		return true;
	}
	float n = baked.sets[si].flat_up0.w;
	float inner_finer = (0.5 * n - 1.0) * 0.5 * c;  // the finer level's interior
	if (max(abs(g.x), abs(g.y)) + 2.0 * c >= inner_finer) {
		return true;
	}
	// The largest footprint any camera of the range can fetch the column with: the farthest camera (the highest, the
	// drift away) at the most grazing view (the lowest, the drift toward; the fetch's 1 / sqrt(mu), mu >= 0.04).
	float a = CLOUD_A_M;
	float h_b = baked.sets[si].flat_axis_y.w;
	float drift = baked.sets[si].flat_anchor.w;
	float r = length(g);
	float h_hi = h_b * CLOUD_BAKED_FLAT_HEIGHT_RANGE + 2000.0;
	float h_lo = max(h_b / CLOUD_BAKED_FLAT_HEIGHT_RANGE - 2000.0, 0.0);
	float th_far = (r + drift) / a;
	float d_max = length(vec2((a + h_hi) * sin(th_far), (a + h_hi) * cos(th_far) - a));
	float th_near = max(r - drift, 0.0) / a;
	vec2 v = vec2((a + h_lo) * sin(th_near), (a + h_lo) * cos(th_near) - (a + CLOUD_TOP_M));
	float mu = max(v.y / max(length(v), 1.0), 0.04);
	float footprint = CLOUD_BAKED_FLAT_PIXEL_RANGE * baked.sets[si].flat_axis_x.w * d_max / sqrt(mu);
	float l = log2(max(footprint, 1.0e-3) / baked.sets[si].axis_x.w);
	float m = baked.sets[si].outer.z + float(j);
	return l >= m - 2.0 - CLOUD_BAKED_FLAT_SKIP_MARGIN;
}
#endif

#ifdef CLOUD_BAKED_FLAT
// The flat levels' geometry (world-fixed: shared by the sets until the visible cap, the height range or the weather
// changes; step 3, user ruling 2026-10-29: the old whole-Earth far field's cover is the reference). The cell's mean is
// the far field's column rule (cloud_far.gdshaderinc cloud_far_layer): the field's column (cloud_column_mean_depth: the
// grid-mean vertical depth tau and the largest cloud fraction f) at the cell's field points, the cover C_col =
// cloud_beam_cover_of(f, 1) (the march takes it to the view's slant as the far field does), the in-cloud depth
// tau / C_col - thin cloud translucent, haze invisible. The shapes only place that cover inside the cell: the pattern P
// is the share of the cell's taps where the shapes' prefiltered value (at the taps' spacing) exceeds the prefilter
// threshold of C_col (the old far tier's cloud_far_cover: the pattern covers the share C_col of the layer, so its mean
// is C_col); from CLOUD_BAKED_FLAT_PATTERN_CELLS_M.x to .y of cell P fades to C_col itself (the far field's even
// cover: the 32 km tile's period drew a lattice of dots across 12.8 km cells).
//   F  x P, y the grid-mean depth tau, z / w the cloud top / base x P (premultiplied by P: the bilinear fetch of a clear
//      neighbour does not lower them)
//   T  x C_col, y the cloud top (the light kernel's: its divisor and view point where P = 0)
// (Step 2 / early step 3 took the cover from the shape taps themselves, the union over all layers with any ramp > 0:
// cirrus and thin edges counted as cover - too much white against the far field; baked-outer2-report.md.)
layout(rgba16f, set = 1, binding = 1) uniform writeonly image2DArray baked_f_out;
layout(rg16f, set = 1, binding = 2) uniform writeonly image2DArray baked_t_out;

// [E] From this cell on (m) every second pattern tap (a checkerboard) reads the shape unaligned - turned and scaled as
// the old far tier's second reading (cloud_pass.gdshaderinc CLOUD_FAR_COVER_UNALIGNED_*, not part of the bake kernels'
// source): over the hundreds of km of an orbit view the 32 km tile's period drew a regular lattice of dots.
const float CLOUD_BAKED_FLAT_UNALIGNED_CELL_M = 1500.0;
const mat3 CLOUD_BAKED_FLAT_UNALIGNED_ROTATION = mat3(0.8130186879, 0.5112918472, -0.2785341274, -0.4537591358,
		0.8561682215, 0.2471408976, 0.3648331945, -0.0745427634, 0.9280841107);
const float CLOUD_BAKED_FLAT_UNALIGNED_SCALE = 1.37;
// [E] The taps' offset in a cell: a smooth field (wavelength CLOUD_BAKED_FLAT_JITTER_M, amplitude
// CLOUD_BAKED_FLAT_JITTER of the tap spacing) - neighbouring cells sample the same relative positions (a per-cell
// random jitter gave each cell a random cover: salt noise), while over the hundreds of km of an orbit view the taps
// drift against the 32 km shape tile (regular taps aliased it into a lattice of holes).
const float CLOUD_BAKED_FLAT_JITTER_M = 37000.0;
const float CLOUD_BAKED_FLAT_JITTER = 0.4;
// [E] The pattern's fade to the even cover over the cell (m; above).
const vec2 CLOUD_BAKED_FLAT_PATTERN_CELLS_M = vec2(6400.0, 12800.0);
// [E] A pattern tap's step softened over this much of the prefiltered shape value (no single-tap speckle).
const float CLOUD_BAKED_FLAT_TAP_SOFT = 0.03;
// The height of a column whose bilinear profile has no cloud while its B-spline has (m; a trace at the edge).
const float CLOUD_BAKED_FLAT_DEFAULT_HEIGHT_M = 2000.0;

#ifdef CLOUD_BAKED_FLAT_TAP_STATS
// (diagnostics: step 2's shape-tap cover of the cell - F = (its cover, the share of it from taps of nadir opacity
// < 0.1, < 0.3, its mean opacity) - the march shows it, CLOUD_BAKED_FLAT_TAP_STATS there too)
vec4 cloud_baked_flat_tap_stats(int si, vec2 g, float c, ivec2 cell) {
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float state = cloud_detail_state(0.5 * c);
	const int taps = 3;
	float cover = 0.0;
	float thin1 = 0.0;
	float thin3 = 0.0;
	float opacity = 0.0;
	for (int t = 0; t < taps * taps; t++) {
		vec2 u = (vec2(t % taps, t / taps) + 0.5) / float(taps) - 0.5;
		vec3 radial = cloud_baked_flat_radial(si, g + u * c) / ATM_STRETCH;
		vec2 lat_lon = cloud_lat_lon(CLOUD_A_M * radial);
		float x_max = -CLOUD_BAKED_X_MAX;
		float tau = 0.0;
		for (int layer = 0; layer < int(CLOUD_LAYERS); layer++) {
			float h_c = cloud_layer_centre_m(float(layer));
			CloudCell cl = cloud_field(lat_lon, h_c);
			if (cl.fraction < CLOUD_MIN_FRACTION || cl.extinction <= 0.0) {
				continue;
			}
			float bottom = cloud_layer_bottom_m(float(layer));
			float thick = cloud_layer_bottom_m(float(layer) + 1.0) - bottom;
			float threshold = cloud_threshold(cl.fraction, state);
			float scale = cl.extinction / max(texture(cloud_edge_mean, cloud_table_uv(cl.fraction, state)).r, 1.0e-4);
			vec3 types = cloud_type_weights(cl);
			vec3 offset = CLOUD_A_M * radial - camera + h_c * radial;
			float ramp = 0.0;
			for (int type = 0; type < 3; type++) {
				if (types[type] > 0.0) {
					vec4 shape = texture(cloud_noise_shape, cloud_type_shape_coordinates(type, offset, h_c));
					float w = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, clamp(shape.a, 0.0, 1.0));
					float x = (shape.r - threshold) / max(w, 1.0e-6);
					ramp += types[type] * cloud_baked_ramp(x);
					x_max = max(x_max, x);
				}
			}
			tau += scale * ramp * thick;
		}
		float r = tau > 0.0 ? cloud_baked_ramp(x_max) : 0.0;
		if (r > 0.0) {
			float a = 1.0 - exp(-tau / r);
			cover += r;
			thin1 += a < 0.1 ? r : 0.0;
			thin3 += a < 0.3 ? r : 0.0;
			opacity += r * a;
		}
	}
	float nt = float(taps * taps);
	return cover > 0.0 ? vec4(cover / nt, thin1 / cover, thin3 / cover, opacity / nt) : vec4(0.0);
}
#endif

void main() {
	const int si = 0;
	int j = baked.build.x;  // the flat level (0: level m0)
	int n = int(baked.sets[si].flat_up0.w);
	ivec2 cell = ivec2(gl_GlobalInvocationID.x, int(gl_GlobalInvocationID.y) + baked.build.y);
	if (cell.x >= n || cell.y >= n || int(gl_GlobalInvocationID.y) >= baked.build.z) {
		return;
	}
	ivec3 texel = ivec3(cell, j);
	float c = baked.sets[si].axis_x.w * exp2(baked.sets[si].outer.z + float(j));
	vec2 g = (vec2(cell) + 0.5 - 0.5 * float(n)) * c;
	if (!cloud_baked_flat_needed(si, j, g, c)) {
		imageStore(baked_f_out, texel, vec4(0.0));
		imageStore(baked_t_out, texel, vec4(0.0));
		return;
	}
#ifdef CLOUD_BAKED_FLAT_TAP_STATS
	imageStore(baked_f_out, texel, cloud_baked_flat_tap_stats(si, g, c, cell));
	imageStore(baked_t_out, texel, vec4(1.0, 5000.0, 0.0, 0.0));
	return;
#endif
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	int fp = c <= CLOUD_BAKED_FLAT_TAP_CELLS_M.y ? 1 : 2;  // field points per axis
	// The column over the field points: the grid-mean depth, the largest fraction; the far field's heights (each
	// weighted by its point's depth: continuous where a point has no cloud); the pattern's types (the layers' depth-
	// weighted mix: no switch where another layer's fraction becomes the largest).
	vec2 column = vec2(0.0);
	vec3 heights = vec3(0.0);  // depth x (nadir height, grazing height, top)
	vec3 types = vec3(0.0);
	float type_weight = 0.0;
	for (int fy = 0; fy < fp; fy++) {
		for (int fx = 0; fx < fp; fx++) {
			vec3 radial_f = cloud_baked_flat_radial(si, g + ((vec2(fx, fy) + 0.5) / float(fp) - 0.5) * c) / ATM_STRETCH;
			vec2 lat_lon = cloud_lat_lon(CLOUD_A_M * radial_f);
			vec2 column_p = cloud_column_mean_depth_smooth(lat_lon, 0.0);  // (B-spline: no field-cell squares, step 4)
			column += column_p;
			if (!(column_p.x > 0.0)) {
				continue;
			}
			// the far field's placement (cloud_far_layer: the height where the depth from the top reaches the
			// transmittance-weighted seen depth) for a nadir and a grazing view, and the optical top
			vec2 column_b = cloud_column_mean_depth(lat_lon, 0.0);  // (the profile's own bilinear column)
			if (column_b.x > 0.0) {
				heights += column_p.x * vec3(cloud_far_depth_height(lat_lon, cloud_far_seen_depth(column_b, 1.0)),
						cloud_far_depth_height(lat_lon, cloud_far_seen_depth(column_b, CLOUD_BAKED_FLAT_GRAZING_MU)),
						cloud_far_depth_height(lat_lon, CLOUD_BAKED_FLAT_TOP_DEPTH * column_b.x));
			} else {
				heights += column_p.x * vec3(CLOUD_BAKED_FLAT_DEFAULT_HEIGHT_M);
			}
			for (int layer = 0; layer < int(CLOUD_LAYERS); layer++) {
				float h_c = cloud_layer_centre_m(float(layer));
				CloudCell cl = cloud_field(lat_lon, h_c);
				if (cl.fraction < CLOUD_MIN_FRACTION || cl.extinction <= 0.0) {
					continue;
				}
				float w = cl.extinction * (cloud_layer_bottom_m(float(layer) + 1.0) - cloud_layer_bottom_m(float(layer)));
				types += w * cloud_type_weights(cl);
				type_weight += w;
			}
		}
	}
	float points = float(fp * fp);
	float depth = column.x / points;
	float c_col = depth > 0.0 ? cloud_beam_cover_of(column.y / points, 1.0) : 0.0;
	if (!(c_col > 0.0)) {
		imageStore(baked_f_out, texel, vec4(0.0));
		imageStore(baked_t_out, texel, vec4(0.0));
		return;
	}
	heights /= column.x;
	types = type_weight > 0.0 ? types / type_weight : vec3(1.0, 0.0, 0.0);
	float h_pattern = heights.x;
	// The pattern (CLOUD_BAKED_FLAT_PATTERN_*): taps x taps over the cell, the shapes prefiltered at the taps' spacing.
	float w_pattern = 1.0 - smoothstep(CLOUD_BAKED_FLAT_PATTERN_CELLS_M.x, CLOUD_BAKED_FLAT_PATTERN_CELLS_M.y, c);
	float pattern = c_col;
#ifdef CLOUD_BAKED_DIAG_NO_BAKE_PATTERN
	w_pattern = 0.0;
#endif
	if (w_pattern > 0.0 && c_col < 1.0) {
		int taps = c <= CLOUD_BAKED_FLAT_TAP_CELLS_M.y ? 3 : 4;
		float spacing = c / float(taps);
		float level = clamp(log2(spacing * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M), 0.0, CLOUD_SHAPE_PREFILTER_LEVELS);
		float threshold = texture(cloud_prefilter_threshold, cloud_prefilter_uv(c_col, level)).r;
		float median = texture(cloud_prefilter_threshold, cloud_prefilter_uv(0.5, level)).r;
		bool unaligned = c >= CLOUD_BAKED_FLAT_UNALIGNED_CELL_M;
		float sum = 0.0;
		for (int ty = 0; ty < taps; ty++) {
			for (int tx = 0; tx < taps; tx++) {
				vec2 ph = g / CLOUD_BAKED_FLAT_JITTER_M + 0.37 * vec2(tx, ty) + 0.61 * vec2(ty, tx);
				vec2 jitter = 0.5 + CLOUD_BAKED_FLAT_JITTER * vec2(sin(6.2832 * (ph.x + 0.71 * ph.y)),
						sin(6.2832 * (ph.y - 0.53 * ph.x) + 1.7));
				vec3 radial = cloud_baked_flat_radial(si, g + ((vec2(tx, ty) + jitter) / float(taps) - 0.5) * c)
						/ ATM_STRETCH;
				vec3 offset = (CLOUD_A_M + h_pattern) * radial - camera;
				bool turned = unaligned && ((tx + ty) & 1) == 1;
				float covered = 0.0;
				for (int type = 0; type < 3; type++) {
					if (types[type] > 0.0) {
						vec3 q = cloud_type_shape_coordinates(type, offset, h_pattern);
						if (turned) {
							q = CLOUD_BAKED_FLAT_UNALIGNED_ROTATION * q / CLOUD_BAKED_FLAT_UNALIGNED_SCALE;
						}
						float s = cloud_baked_shape(q, level, cloud_baked_blend_weight(q), median).x;
						covered += types[type] * smoothstep(threshold - CLOUD_BAKED_FLAT_TAP_SOFT,
								threshold + CLOUD_BAKED_FLAT_TAP_SOFT, s);
					}
				}
				sum += covered;
			}
		}
		pattern = mix(c_col, sum / float(taps * taps), w_pattern);
	}
	// (heights in km: x the depth stays within RGBA16F's range for any column)
	imageStore(baked_f_out, texel, vec4(pattern, depth, 1.0e-3 * heights.x * depth, 1.0e-3 * heights.y * depth));
	imageStore(baked_t_out, texel, vec4(c_col, heights.z * c_col, 0.0, 0.0));
}
#endif

// ---- The march -----------------------------------------------------------------------------------------------------
#ifdef CLOUD_BAKED_MARCH
layout(set = 1, binding = 50) uniform sampler3D baked_a0;  // linear
layout(set = 1, binding = 51) uniform sampler3D baked_b0;  // linear
layout(set = 1, binding = 52) uniform sampler3D baked_o0;  // nearest
#ifdef CLOUD_BAKED_FADE
layout(set = 1, binding = 54) uniform sampler3D baked_a1;
layout(set = 1, binding = 55) uniform sampler3D baked_b1;
layout(set = 1, binding = 56) uniform sampler3D baked_o1;
#endif

// The set's volumes (fetch helpers instead of sampler parameters: si is a constant at each call).
vec2 cloud_baked_a(int si, vec3 uvw) {
#ifdef CLOUD_BAKED_FADE
	if (si == 1) {
		return texture(baked_a1, uvw).rg;
	}
#endif
	return texture(baked_a0, uvw).rg;
}

vec4 cloud_baked_b(int si, vec3 uvw) {
#ifdef CLOUD_BAKED_FADE
	if (si == 1) {
		return texture(baked_b1, uvw);
	}
#endif
	return texture(baked_b0, uvw);
}

float cloud_baked_o(int si, ivec3 texel) {
#ifdef CLOUD_BAKED_FADE
	if (si == 1) {
		return texelFetch(baked_o1, texel, 0).r;
	}
#endif
	return texelFetch(baked_o0, texel, 0).r;
}

#ifndef CLOUD_BAKED_SHADOW_OFF
// The air shadow's T_c at camera + offset (height m): the line toward the Sun through the point read from S (both sets
// by the cross-fade's weight while a new set fades in). False beyond the volume levels (the caller's column).
bool cloud_baked_air_sun(vec3 offset_local, float height_m, vec3 sun_dir, out float t) {
	vec4 line;
	bool inside = cloud_baked_shadow_line(0, offset_local, sun_dir, cloud_baked_air_footprint_m, line);
	t = cloud_baked_shadow_at_height(line, height_m);
#ifdef CLOUD_BAKED_FADE
	vec4 line1;
	if (cloud_baked_shadow_line(1, offset_local, sun_dir, cloud_baked_air_footprint_m, line1)) {
		float t1 = cloud_baked_shadow_at_height(line1, height_m);
		t = inside ? mix(t, t1, baked.sets[0].misc.y) : t1;
		inside = true;
	}
#endif
	return inside;
}
#endif

float cloud_baked_flat_weight_at(int si, float l);

// A march point in one set's levels (cloud_baked_lod): its level k (blend toward k + 1, texture coordinates in both)
// and keep = 1 - the flat levels' weight (the volume's share).
struct CloudBakedPoint {
	int k;
	float blend;
	float keep;
	vec3 uvw;
	vec3 g;  // the grid point and height (the next level's coordinates only while blending)
};

CloudBakedPoint cloud_baked_point(int si, float gx, float gy, float h, float footprint_m, int k_min) {
	CloudBakedPoint q;
	float l = cloud_baked_lod(si, max(abs(gx), abs(gy)), footprint_m);
	q.keep = 1.0 - cloud_baked_flat_weight_at(si, l);  // (the takeover: region by region, cloud_baked_set)
	q.k = cloud_baked_level_of(si, l, k_min, q.blend);
	int levels = int(baked.sets[si].axis_y.w);
	q.uvw = cloud_baked_uvw(si, min(q.k, levels - 1), gx, gy, h);
	q.g = vec3(gx, gy, h);
	return q;
}

// The light (B, still sqrt-coded) at a march point, blended as its extinction.
vec4 cloud_baked_b2(int si, CloudBakedPoint q) {
	vec4 b = cloud_baked_b(si, q.uvw);
	return q.blend > 0.0 ? mix(b, cloud_baked_b(si, cloud_baked_uvw(si, q.k + 1, q.g.x, q.g.y, q.g.z)), q.blend) : b;
}

// The extinction (1/m) at a march point (its level and blend, camera offset, pixel footprint): the baked field plus, with
// CLOUD_BAKED_DETAIL in level 0, one fetch of the detail's level 1 (62.5 m cells; the bake holds level 0) at the
// pixel's band limit near the edge - the old march's erosion term (cloud_detail_perturbation) on the edge coordinate.
// gap_x: how far the point lies outside any cloud the per-sample detail could add, in edge widths (the signed edge
// coordinate below the detail's reach; <= 0 within it) - the clear march's distance estimate.
float cloud_baked_beta_gap(int si, CloudBakedPoint q, vec3 offset, float footprint_m, out float gap_x) {
	int k = q.k;
	vec3 uvw = q.uvw;
	vec2 a = cloud_baked_a(si, uvw);
	if (q.blend > 0.0) {
		a = mix(a, cloud_baked_a(si, cloud_baked_uvw(si, k + 1, q.g.x, q.g.y, q.g.z)), q.blend);  // the level rule's blend
	}
	gap_x = a.y > 0.0 ? -a.x : CLOUD_BAKED_X_MAX;
#ifdef CLOUD_BAKED_DETAIL
	// The detail's reach in edge widths: down (erosion) its full amplitude, up only its lifted part at the detail's
	// largest value 1 (cloud_detail_lift_bound's term: ~1.8 widths against ~14 down).
	float reach = abs(cloud_level_amplitude[1]) / max(cloud_edge_width.x, 1.0e-6);
	float lift_reach = cloud_level_lift[1] * max(cloud_level_amplitude[1], 0.0) * (1.0 - cloud_detail_mean)
			/ max(cloud_edge_width.x, 1.0e-6);
	if (k == 0) {
		gap_x -= lift_reach;
	}
	if (k == 0 && a.y > 0.0 && a.x > -lift_reach && a.x < 1.0 + reach) {
		float w1 = clamp(log2(CLOUD_DETAIL_TILE_M / 16.0 / max(footprint_m, 1.0e-3)) - 1.0, 0.0, 1.0);
		if (w1 > 0.0 && cloud_detail_levels > 1) {
			float width_noise = cloud_baked_b2(si, q).w;
			float width = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, width_noise);
			vec3 q = fract(cloud_detail_origin * 4.0) + offset * (4.0 / CLOUD_DETAIL_TILE_M);
			float d = dot(texture(cloud_noise_detail, q).rgb, cloud_detail_weights);
			float term = w1 * cloud_level_amplitude[1] * (d - cloud_detail_mean);
			a.x += (term < 0.0 ? term : cloud_level_lift[1] * term) / max(width, 1.0e-6);
		}
	}
#endif
	return cloud_baked_extinction(a) * q.keep;
}

float cloud_baked_beta(int si, CloudBakedPoint q, vec3 offset, float footprint_m) {
	float gap_x;
	return cloud_baked_beta_gap(si, q, offset, footprint_m, gap_x);
}

// [E] Steps per ray at most; the shortest step (m); the bisections of a cloud entry; the in-cloud step weight limit
// (the old march's CLOUD_MAX_STEP_WEIGHT: a step's share of the pixel at most 0.15 / T).
const int CLOUD_BAKED_MAX_STEPS = 768;
const float CLOUD_BAKED_MIN_STEP_M = 4.0;
const int CLOUD_BAKED_BISECTIONS = 3;
const float CLOUD_BAKED_STEP_WEIGHT = 0.15;
// [E] The clear march near an edge (fix 1): metres per edge width of the distance estimate (the edge width is the
// shape value's median increment over 20-40 m, constants.hpp cloud_edge_width_min_m) and the shortest clear step;
// the in-cloud step's limit as a fraction of the cell (half a cell left wood-grain contours and facets in the near
// rims: the trapezoid over a thin lens changes with the step's phase); both only while the ray's scaled transmittance
// is above CLOUD_BAKED_FINE_TRANSMITTANCE (behind a dense part the coarse steps' errors are not seen). (Measured:
// 10 m / 8 m and 1/8 cell, the finest tried, looked the same as these.)
const float CLOUD_BAKED_EDGE_STEP_M = 20.0;
const float CLOUD_BAKED_MIN_CLEAR_STEP_M = 16.0;
const float CLOUD_BAKED_FINE_STEP_CELLS = 0.25;
const float CLOUD_BAKED_COARSE_STEP_CELLS = 0.5;
const float CLOUD_BAKED_FINE_TRANSMITTANCE = 0.3;
// [E] How far past a block's exit a skip lands (m, relative to the cell): the next sample lies inside the next block.
const float CLOUD_BAKED_EXIT_NUDGE = 0.01;
// [E] The band's height tolerance (m) for the float32 heights at its crossings.
const float CLOUD_BAKED_BAND_TOLERANCE_M = 2.0;

// The ray's state in one set's grid: its grid coordinates are linear in the distance.
struct CloudBakedRay {
	float gx0;
	float gy0;
	float gdx;
	float gdy;
};

CloudBakedRay cloud_baked_ray(int si, vec3 eye, vec3 dir) {
	CloudBakedRay r;
	vec3 axis_x = baked.sets[si].axis_x.xyz * ATM_STRETCH;
	vec3 axis_y = baked.sets[si].axis_y.xyz * ATM_STRETCH;
	r.gx0 = baked.sets[si].band.z + dot(eye, axis_x);
	r.gy0 = baked.sets[si].band.w + dot(eye, axis_y);
	r.gdx = dot(dir, axis_x);
	r.gdy = dot(dir, axis_y);
	return r;
}

// ---- The flat levels in the march ----
layout(set = 1, binding = 57) uniform sampler2DArray baked_f0;  // linear
layout(set = 1, binding = 58) uniform sampler2DArray baked_t0;  // linear (T: C_col, top)
#ifdef CLOUD_BAKED_FADE
layout(set = 1, binding = 59) uniform sampler2DArray baked_f1;
layout(set = 1, binding = 60) uniform sampler2DArray baked_t1;
#endif

// [E] How far outside the fetched column's heights the entry may lie (m; float32 heights at the shell crossings).
const float CLOUD_BAKED_FLAT_ENTRY_TOLERANCE_M = 50.0;
// [E] ... plus this per metre of distance (the float32 shell crossings from thousands of km: streaks of rejected
// entries along the image rows at 1e-5).
const float CLOUD_BAKED_FLAT_ENTRY_TOLERANCE_PER_M = 1.0e-4;
// [E] Behind a volume transmittance below this the flat levels are not fetched (< 1 % of their light).
const float CLOUD_BAKED_FLAT_MIN_TRANSMITTANCE = 0.01;
// [E] The takeover (step 3; the brief: the footprint-matched flat levels beat a volume level its clipmap's extent made
// too coarse): from the camera height CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M.x to .y the flat levels replace the volume,
// above it they draw every cloud (CloudBaked VOLUME_TOP_M: the volume then neither baked nor marched); below .x the
// volume levels are unchanged. A takeover only where the extent made the volume's level coarser than the footprint's
// kept the volume's square about the nadir visible (a coarse voxel level's trilinear edge coordinate closes a deck's
// sub-voxel gaps: a whiter deck than the far field's cover). Diagnostics: define CLOUD_BAKED_FLAT_NO_TAKEOVER keeps the
// volume at every height (step 2's hand-over at its last level only).
const vec2 CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M = vec2(40000.0, 60000.0);
// [E] The sub-cell pattern (step 3; at every cell size since step 4): the pixel's cover is the share of 2 x 2 taps over
// its footprint (x CLOUD_FAR_COVER_FOOTPRINT_SCALE, / mu along the view) where the shapes' prefiltered value at the
// taps' mip exceeds the prefilter threshold of the cell's (B-spline reconstructed) cover - the shapes' pattern covering
// that share of the layer (its mean the cell's cover), its edges as sharp as the footprint at every distance; the shape
// coordinates domain-warped (cloud_baked_warp: no repeating tile). Faded out over CLOUD_BAKED_FLAT_PATTERN_FADE_M of
// footprint (the old far tier's CLOUD_FAR_COVER_FADE_M: from a quarter of the tile on the taps average the layer's
// mean; the scaled-space far field takes over beyond). The tap's step is softened over CLOUD_BAKED_FLAT_PATTERN_SOFT of
// the shape value (no single-tap speckle). (Step 3 faded it in from cells 2 x the footprint and out over 2-4 km
// footprints: the cells' bilinear / smoothed cover alone drew blurred blobs and field-cell squares from 600 km up.)
const vec2 CLOUD_BAKED_FLAT_PATTERN_FADE_M = vec2(8000.0, 16000.0);
// [E] The pattern's fade toward the scaled-space far field (camera heights, m; it takes over at the camera distance
// CelestialBody.SCALED_SPACE_DISTANCE_M = 15 000 km from the Earth's centre, ~8 620 km up): its cover has no sub-cell
// pattern, so the flat levels lose theirs before the switch (step 5: no pop).
const vec2 CLOUD_BAKED_FLAT_SWITCH_FADE_M = vec2(5500000.0, 8000000.0);

vec4 cloud_baked_flat_tex(int si, int which, vec3 uvl) {
#ifdef CLOUD_BAKED_FADE
	if (si == 1) {
		return which == 0 ? texture(baked_f1, uvl) : texture(baked_t1, uvl);
	}
#endif
	return which == 0 ? texture(baked_f0, uvl) : texture(baked_t0, uvl);
}

// A flat level's texture (which: 0 F, 1 T) reconstructed by the cubic B-spline at uvl (n columns per side): four
// bilinear fetches at the B-spline's weighted positions (C2, non-negative weights: the cells' mean kept; step 4 - the
// bilinear's creases and plateaus drew every cell as a square where it spans several pixels).
vec4 cloud_baked_flat_bspline(int si, int which, vec3 uvl, float n) {
	vec2 x = uvl.xy * n - 0.5;
	vec2 i = floor(x);
	vec2 t = x - i;
	vec4 wx = cloud_bspline_weights(t.x);
	vec4 wy = cloud_bspline_weights(t.y);
	vec2 g0 = vec2(wx.x + wx.y, wy.x + wy.y);
	vec2 g1 = vec2(wx.z + wx.w, wy.z + wy.w);
	vec2 h0 = (i - 0.5 + vec2(wx.y, wy.y) / g0) / n;
	vec2 h1 = (i + 1.5 + vec2(wx.w, wy.w) / g1) / n;
	return g0.y * (g0.x * cloud_baked_flat_tex(si, which, vec3(h0.x, h0.y, uvl.z))
					+ g1.x * cloud_baked_flat_tex(si, which, vec3(h1.x, h0.y, uvl.z)))
			+ g1.y * (g0.x * cloud_baked_flat_tex(si, which, vec3(h0.x, h1.y, uvl.z))
					+ g1.x * cloud_baked_flat_tex(si, which, vec3(h1.x, h1.y, uvl.z)));
}

// The flat levels' weight at a volume sample of continuous level l (cloud_baked_lod; 0 the volume alone, 1 the flat
// levels alone; the volume's extinction scaled by 1 - w, the flat levels' alpha by w): the hand-over beyond the
// volume's last level K - 1 (across the same band as any level to the next). The takeover is a cross-fade of the
// renders (cloud_baked_takeover_weight).
float cloud_baked_flat_weight_at(int si, float l) {
	if (baked.sets[si].outer.x < 0.5) {
		return 0.0;
	}
	// (diagnostics, STARSIM_BAKED_DEFINES: the flat levels alone / the volume alone)
#ifdef CLOUD_BAKED_FLAT_ONLY
	return 1.0;
#endif
#ifdef CLOUD_BAKED_VOLUME_ONLY
	return 0.0;
#endif
	float last = baked.sets[si].axis_y.w - 1.0;
	return smoothstep(last - max(cloud_baked_band(), CLOUD_BAKED_FLAT_SEAM_BAND), last, l);
}

// The takeover's weight (camera height; 0 below CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M.x, 1 above .y): the two renders -
// the volume with the flat levels' hand-over behind it, and the flat levels alone - switched region by region by it
// (cloud_baked_takeover_local; cirrus check
// 2026-10-30: scaling the volume's extinction by 1 - w kept its dense shapes nearly opaque at w 0.5 while the flat
// levels' half-alpha shapes filled the gaps - grey translucent shapes at 50 km, the lower half's mean alpha 0.40
// against 0.31 (volume) / 0.34 (flat) alone).
float cloud_baked_takeover_weight(int si) {
	if (baked.sets[si].outer.x < 0.5) {
		return 0.0;
	}
#ifdef CLOUD_BAKED_FLAT_ONLY
	return 1.0;
#endif
#ifdef CLOUD_BAKED_VOLUME_ONLY
	return 0.0;
#endif
#ifdef CLOUD_BAKED_FLAT_NO_TAKEOVER
	return 0.0;
#endif
	if (baked.sets[si].outer.z >= baked.sets[si].axis_y.w) {
		return 0.0;
	}
	return smoothstep(CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M.x, CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M.y,
			cloud_camera_height_m);
}

// [E] The takeover's regions: a value noise of the Earth-fixed point (cells of CLOUD_BAKED_TAKEOVER_CELL_M, smooth
// trilinear between hashed corners) against the takeover's weight, each region switching over CLOUD_BAKED_TAKEOVER_SOFT
// of the noise. A plain cross-fade of the two renders showed both shape sets (uncorrelated: the flat levels' tile
// breaking) at half alpha everywhere - grey translucent shapes at 50 km; per region one render is drawn, the mean over
// the regions still follows the weight (0 / 1 at the band's ends).
const float CLOUD_BAKED_TAKEOVER_CELL_M = 20000.0;
const float CLOUD_BAKED_TAKEOVER_SOFT = 0.08;

float cloud_baked_hash(ivec3 c) {
	uint h = uint(c.x) * 0x8da6b343u ^ uint(c.y) * 0xd8163841u ^ uint(c.z) * 0xcb1ab31fu;
	h ^= h >> 16;
	h *= 0x7feb352du;
	h ^= h >> 15;
	h *= 0x846ca68bu;
	h ^= h >> 16;
	return float(h) * (1.0 / 4294967295.0);
}

// The local takeover weight at the Earth-local point p (from the Earth's centre, m) for the band's weight w.
float cloud_baked_takeover_local(vec3 p, float w) {
	if (w <= 0.0 || w >= 1.0) {
		return w;
	}
	vec3 x = p / CLOUD_BAKED_TAKEOVER_CELL_M;
	vec3 i = floor(x);
	vec3 f = x - i;
	f = f * f * (3.0 - 2.0 * f);
	ivec3 b = ivec3(i);
	float n = mix(mix(mix(cloud_baked_hash(b), cloud_baked_hash(b + ivec3(1, 0, 0)), f.x),
					mix(cloud_baked_hash(b + ivec3(0, 1, 0)), cloud_baked_hash(b + ivec3(1, 1, 0)), f.x), f.y),
			mix(mix(cloud_baked_hash(b + ivec3(0, 0, 1)), cloud_baked_hash(b + ivec3(1, 0, 1)), f.x),
					mix(cloud_baked_hash(b + ivec3(0, 1, 1)), cloud_baked_hash(b + ivec3(1, 1, 1)), f.x), f.y), f.z);
	n = smoothstep(0.15, 0.85, n);  // (the trilinear mix of uniform corners clusters about 0.5: spread toward uniform)
	float s = CLOUD_BAKED_TAKEOVER_SOFT;
	float level = mix(-s, 1.0 + s, w);
	return smoothstep(n - s, n + s, level);
}

// The footprint's continuous level (cell c0 2^l = the footprint; the flat levels' rule).
float cloud_baked_lod_fp(int si, float footprint_m) {
	return log2(max(CLOUD_BAKED_LEVEL_DROP * footprint_m, 1.0e-3) / baked.sets[si].axis_x.w);
}

// True while the takeover can reach this set's volume samples (the flat levels start finer than the volume's end and the
// camera is high enough): the flat levels are then shaded from the ray's start.
bool cloud_baked_flat_takeover(int si) {
#ifdef CLOUD_BAKED_FLAT_NO_TAKEOVER
	return false;
#endif
	return baked.sets[si].outer.z < baked.sets[si].axis_y.w && cloud_camera_height_m > CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M.x;
}

// The distance along the ray where the flat levels' weight starts (x, y linear in it, the footprint too).
float cloud_baked_flat_start(int si, CloudBakedRay g, float pixel_angle) {
#ifdef CLOUD_BAKED_FLAT_ONLY
	return 0.0;
#endif
	if (cloud_baked_flat_takeover(si)) {
		return 0.0;
	}
	float l0 = baked.sets[si].axis_y.w - 1.0 - max(cloud_baked_band(), CLOUD_BAKED_FLAT_SEAM_BAND);
	float r0 = (0.5 * baked.sets[si].up0.w - 1.0) * baked.sets[si].axis_x.w * exp2(l0);
	float t = 1.0e30;
	if (max(abs(g.gx0), abs(g.gy0)) >= r0) {
		t = 0.0;
	} else {
		if (g.gdx != 0.0) {
			t = min(t, ((g.gdx > 0.0 ? r0 : -r0) - g.gx0) / g.gdx);
		}
		if (g.gdy != 0.0) {
			t = min(t, ((g.gdy > 0.0 ? r0 : -r0) - g.gy0) / g.gdy);
		}
	}
	return min(t, baked.sets[si].axis_x.w * exp2(l0) / (CLOUD_BAKED_LEVEL_DROP * max(pixel_angle, 1.0e-9)));
}

// The flat levels at the Earth-local point p (from the Earth's centre) for a pixel footprint (m): F and T (the cubic
// B-spline, cloud_baked_flat_bspline), the level rule's l = max(the footprint's, the levels' extent: m0 + log2(r /
// inner)) - level ceil(l), at least m0, blended toward the next over cloud_baked_band() (at least
// CLOUD_BAKED_FLAT_SEAM_BAND). heights_only: F alone. false beyond the baked cap (the far side is never fetched).
float cloud_baked_diag_level = 0.0;
bool cloud_baked_flat_fetch(int si, vec3 p, float footprint_m, bool heights_only, out vec4 f, out vec2 t) {
	f = vec4(0.0);
	t = vec2(0.0);
	int flat_levels = int(baked.sets[si].outer.x + 0.5);
	if (flat_levels <= 0) {
		return false;
	}
	vec3 xyt = cloud_baked_flat_xy(si, p);
	if (xyt.z > baked.sets[si].outer.y) {
		return false;
	}
	float n = baked.sets[si].flat_up0.w;
	float c0 = baked.sets[si].axis_x.w;
	float m0 = baked.sets[si].outer.z;
	float inner = (0.5 * n - 1.0) * c0 * exp2(m0);
	float kf = max(cloud_baked_lod_fp(si, footprint_m), m0 + log2(max(max(abs(xyt.x), abs(xyt.y)), 1.0) / inner));
	int k0 = max(int(ceil(kf)), int(m0));
	float band = max(cloud_baked_band(), CLOUD_BAKED_FLAT_SEAM_BAND);  // (the flat levels' own hand-overs: always a band)
	float blend = smoothstep(1.0 - band, 1.0, kf - float(k0 - 1));
	int j = k0 - int(m0);
	if (j >= flat_levels - 1) {
		j = flat_levels - 1;
		blend = 0.0;
	}
	float c = c0 * exp2(m0 + float(j));
	cloud_baked_diag_level = float(j) + blend;
	vec3 uvl = vec3(xyt.xy / (c * n) + 0.5, float(j));
	f = cloud_baked_flat_bspline(si, 0, uvl, n);
	if (!heights_only) {
		t = cloud_baked_flat_bspline(si, 1, uvl, n).xy;
	}
	if (blend > 0.0) {
		uvl = vec3(xyt.xy / (2.0 * c * n) + 0.5, float(j + 1));
		f = mix(f, cloud_baked_flat_bspline(si, 0, uvl, n), blend);
		if (!heights_only) {
			t = mix(t, cloud_baked_flat_bspline(si, 1, uvl, n).xy, blend);
		}
	}
	return true;
}

// [E] The thickness inside the shapes (step 5; step 4's shapes were uniform paper cut-outs): the in-cloud depth at a tap
// is the column's times k = (K0 + (1 - K0) r) / (K0 + (1 - K0) E[r | covered]), r the volume's soft edge ramp of the
// shape's excess over the threshold (cloud_edge_ramp: the edge width from the shape's width noise) and E[r | covered]
// = the prefiltered edge mean / the cover (cloud_prefilter_edge_mean: the volume levels' own normalisation) - thick
// cores, margins at K0 of the mean (translucent where the cloud is thin), the depth's mean over the covered taps the
// column's (the far field's water); the volume levels below the takeover draw the same ramp in 3-D. Toward a closed
// cover (CLOUD_BAKED_FLAT_DECK_COVER) the ramp is 1 nearly everywhere: k moves to a texture about the median, x 1 +
// CLOUD_BAKED_FLAT_DEPTH_TEXTURE (s - s_50) within CLOUD_BAKED_FLAT_DEPTH_RANGE (step 4's deck texture).
const float CLOUD_BAKED_FLAT_MARGIN_DEPTH = 0.3;
const vec2 CLOUD_BAKED_FLAT_DECK_COVER = vec2(0.7, 0.95);
const float CLOUD_BAKED_FLAT_DEPTH_TEXTURE = 3.0;
const vec2 CLOUD_BAKED_FLAT_DEPTH_RANGE = vec2(0.25, 1.75);
// [E] The pattern's anti-aliasing (step 5): a tap covers clamp((s - threshold) / g + 0.5) of its area, g the shape's
// change across a tap (the taps' differences: the pixel footprint's), at least CLOUD_BAKED_FLAT_AA_MIN - the edge one
// pixel wide at every distance, sharp, not a binary staircase.
const float CLOUD_BAKED_FLAT_AA_MIN = 0.004;

// One reading of the sub-cell pattern: the 2 x 2 taps (t1 along, t2 across; along_m x across_m) about p, the shape
// scaled up by `scale` (the coordinates divided by it; the tap spacing in the scaled shape picks its prefilter level);
// `cover` the cell's cover along the view, tau the in-cloud vertical depth, mu the view's cosine. x the covered share,
// y the mean opacity, z the covered taps' mean depth factor k (CLOUD_BAKED_FLAT_MARGIN_DEPTH).
vec3 cloud_baked_flat_pattern_at(vec3 p, vec3 t1, vec3 t2, float along_m, float across_m, float spacing_m, float scale,
		float cover, float tau, float mu) {
	float level = clamp(log2(spacing_m / scale * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M), 0.0,
			CLOUD_SHAPE_PREFILTER_LEVELS);
	vec2 uv = cloud_prefilter_uv(clamp(cover, 0.004, 0.996), level);
	float threshold = texture(cloud_prefilter_threshold, uv).r;
	float median = texture(cloud_prefilter_threshold, cloud_prefilter_uv(0.5, level)).r;
	float ramp_mean = clamp(texture(cloud_prefilter_edge_mean, uv).r / max(cover, 1.0e-3), 0.05, 1.0);
	float deck = smoothstep(CLOUD_BAKED_FLAT_DECK_COVER.x, CLOUD_BAKED_FLAT_DECK_COVER.y, cover);
	float k_norm = CLOUD_BAKED_FLAT_MARGIN_DEPTH + (1.0 - CLOUD_BAKED_FLAT_MARGIN_DEPTH) * ramp_mean;
	float unit = 1.0 / (CLOUD_SHAPE_TILE_H_M * CLOUD_FAR_COVER_UNALIGNED_SCALE * scale);
	vec3 q0 = CLOUD_FAR_COVER_UNALIGNED_ROTATION * p * unit;
	float w = cloud_baked_blend_weight(q0);  // (smooth: once per pixel)
	vec2 s[4];
	for (int i = 0; i < 4; i++) {
		vec2 u = (vec2(i & 1, i >> 1) + 0.5) * 0.5 - 0.5;
		vec3 q = q0 + CLOUD_FAR_COVER_UNALIGNED_ROTATION * (t1 * (u.x * along_m) + t2 * (u.y * across_m)) * unit;
		s[i] = cloud_baked_shape(q, level, w, median);
	}
	float aa = max(length(vec2(s[1].x - s[0].x + s[3].x - s[2].x, s[2].x - s[0].x + s[3].x - s[1].x)) * 0.5,
			CLOUD_BAKED_FLAT_AA_MIN);
	vec3 result = vec3(0.0);
	for (int i = 0; i < 4; i++) {
		float covered = cover >= 0.996 ? 1.0 : clamp((s[i].x - threshold) / aa + 0.5, 0.0, 1.0);
		float k_ramp = (CLOUD_BAKED_FLAT_MARGIN_DEPTH + (1.0 - CLOUD_BAKED_FLAT_MARGIN_DEPTH)
				* cloud_edge_ramp(s[i].x, threshold, s[i].y)) / k_norm;
		float k_deck = clamp(1.0 + CLOUD_BAKED_FLAT_DEPTH_TEXTURE * (s[i].x - median), CLOUD_BAKED_FLAT_DEPTH_RANGE.x,
				CLOUD_BAKED_FLAT_DEPTH_RANGE.y);
		float k = mix(k_ramp, k_deck, deck);
		result += vec3(covered, covered * (1.0 - exp(-tau * k / mu)), covered * k);
	}
	result.z = result.x > 0.0 ? result.z / result.x : 1.0;
	result.xy *= 0.25;
	return result;
}

// The sub-cell pattern (CLOUD_BAKED_FLAT_PATTERN_*) at the Earth-local point p (geodetic up `up`) for a view dir of
// |cosine| mu, a pixel footprint, the cell's cover along the view (its mean) and in-cloud depth tau: x the pixel's
// cover, y its opacity, z the depth factor of its cloud (cloud_baked_flat_pattern_at). Where the taps' spacing passes
// the shapes' last prefilter level (1 km texels) the shape is read scaled up by powers of two (two octaves cross-faded
// by the spacing's fraction; each with the cover's mean): its features stay a few pixels wide at every distance (step 4).
vec3 cloud_baked_flat_pattern(vec3 p, vec3 up, vec3 dir, float mu, float footprint_m, float cover, float tau) {
	if (cover <= 0.004) {
		return vec3(cover, cover * (1.0 - exp(-tau / mu)), 1.0);
	}
	vec3 along = dir - up * dot(dir, up);
	float along_norm = length(along);
	vec3 t1 = along_norm > 1.0e-6 ? along / along_norm
			: normalize(cross(up, abs(up.y) < 0.9 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0)));
	vec3 t2 = cross(up, t1);
	float across_m = footprint_m * CLOUD_FAR_COVER_FOOTPRINT_SCALE;
	float along_m = min(across_m / mu, max(CLOUD_FAR_COVER_ALONG_MAX_M, across_m));
	float spacing_m = 0.5 * max(along_m, across_m);
	float octave = max(log2(spacing_m * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M) - CLOUD_SHAPE_PREFILTER_LEVELS, 0.0);
	float k = floor(octave);
	float w = octave - k;
	vec3 result = cloud_baked_flat_pattern_at(p, t1, t2, along_m, across_m, spacing_m, exp2(k), cover, tau, mu);
	if (w > 0.0) {
		result = mix(result, cloud_baked_flat_pattern_at(p, t1, t2, along_m, across_m, spacing_m, exp2(k + 1.0), cover,
				tau, mu), w);
	}
	return result;
}

// The flat levels' height at a fetched F for a view of cosine mu (CLOUD_BAKED_FLAT_GRAZING_MU).
float cloud_baked_flat_height(vec4 f, float mu) {
	return 1000.0 * mix(f.w, f.z, clamp((mu - CLOUD_BAKED_FLAT_GRAZING_MU) / (1.0 - CLOUD_BAKED_FLAT_GRAZING_MU), 0.0,
			1.0)) / f.y;
}

// [E] The Earth's shadow edge on the clouds (step 5): within CLOUD_BAKED_FLAT_TERMINATOR_MU of the Sun's horizon at
// the light's point the column's light is the mean of the slab at its optical top, its point and as far below (the
// edge passes the column's heights over a band of Sun elevations: a single height drew it as one sharp line, kinked
// where the height changed).
const float CLOUD_BAKED_FLAT_TERMINATOR_MU = 0.08;
// [E] The height of the sphere the per-pixel pattern is read on (m; the pattern's parallax is that of a layer at this
// height, the light and the occlusion distance the column's own).
const float CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M = 2000.0;
// [E] A cloud behind the scene but with its optical top in front is drawn at this share of the scene's distance.
const float CLOUD_BAKED_FLAT_SCENE_PULL = 0.9995;
const float CLOUD_BAKED_FLAT_SCENE_DEPTH_M = 3000.0;
// [E] The height crossing's steps (heights-only fetches) and its tolerance (m).
const int CLOUD_BAKED_FLAT_HEIGHT_STEPS = 4;
const float CLOUD_BAKED_FLAT_HEIGHT_TOLERANCE_M = 10.0;

// The flat levels along the ray from t_from (inside the 20 km shell): the far field's placement (cloud_far_layer) - the
// point where the ray crosses the column's seen-depth height (cloud_baked_flat_height; a first fetch where the ray
// crosses the guess height, then the fetched height's crossing until bracketed, then false position), the light evaluated there. The pixel's cover and
// opacity (the far field's rule, cloud_far.gdshaderinc cloud_far_layer): the cell's cover C seen along the view as the
// beam cover's slant growth 1 - (1 - C)^(1 + s), s = CLOUD_OVERLAP_SLANT max(cot - CLOUD_OVERLAP_ONSET_COT, 0)
// (cloud_beam_cover_of's slant term; its vertical term is in C already: the shapes' projected cover), the water kept
// (in-cloud depth tau_e C / C_v), alpha = C_v (1 - exp(-tau / mu)) - thin cloud translucent; the sub-cell pattern at
// the footprint (cover, opacity and the depth inside the shapes); the light the far field's column model at the pixel
// for its own view (cloud_far_cloudy_radiance) with the shapes' depth. Out: premultiplied colour with the air in front,
// alpha, the distance; false: none.
bool cloud_baked_flat_shade(int si, CloudRay ray, vec3 dir, vec3 eye, float t_from, float scene_m, float pixel_angle,
		out vec3 color, out float alpha, out float dist) {
	color = vec3(0.0);
	alpha = 0.0;
	dist = 0.0;
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float t1 = t_from;
	float h1 = cloud_height_along(ray, t1);
	if (h1 > CLOUD_TOP_M - CLOUD_BAKED_BAND_TOLERANCE_M) {
		vec2 x = cloud_sphere_crossing(ray, baked.sets[si].outer.w);
		if (x.x > x.y || x.x < t1) {
			return false;
		}
		t1 = x.x;
	}
	// the scene in front of the shell's entry: by the same vertical rule as the crossing behind the scene below (step 7:
	// near the limb the scene's depth - grazing, half resolution - landed in front of the entry along rows and columns
	// of the depth buffer, a striped staircase of holes in the clouds at 3 000 km; a descending ray meets the shell
	// before any ground under it)
	if (t1 >= scene_m && (t1 - scene_m) * max(abs(dot(dir, cloud_geodetic_up(camera + eye + dir * t1))),
			CLOUD_FAR_MU_MIN) > CLOUD_BAKED_FLAT_SCENE_DEPTH_M) {
		return false;
	}
	vec4 f;
	vec2 t_col;
	float t_hit = t1;
	vec3 p = camera + eye + dir * t_hit;
	// the root of g(t) = height(t) - h(p(t)): fixed-point steps (the fetched height's crossing) until the root is
	// bracketed (a point on each side of the height surface), then false position (steep height changes: the fixed point
	// alone oscillated - smeared shapes)
	float t_near = t_from;
	float g_near = 0.0;
	float t_far = 0.0;
	float g_far = 0.0;
	bool bracketed = false;
	bool have_near = false;
	for (int i = 0; i < CLOUD_BAKED_FLAT_HEIGHT_STEPS; i++) {
		if (!cloud_baked_flat_fetch(si, p, pixel_angle * t_hit, true, f, t_col)) {
			return false;
		}
		if (!(f.y > 1.0e-6)) {
			break;
		}
		float h = cloud_baked_flat_height(f, max(abs(dot(dir, cloud_geodetic_up(p))), CLOUD_FAR_MU_MIN));
		float g = cloud_height_along(ray, t_hit) - h;
		if (abs(g) < CLOUD_BAKED_FLAT_HEIGHT_TOLERANCE_M) {
			break;
		}
		if ((cloud_camera_height_m > h) == (g > 0.0)) {
			t_near = t_hit;  // still on the camera's side
			g_near = g;
			have_near = true;
		} else {
			t_far = t_hit;
			g_far = g;
			bracketed = have_near;
		}
		float t_next;
		if (bracketed) {
			t_next = t_near - g_near * (t_far - t_near) / (g_far - g_near);
		} else {
			vec2 x = cloud_sphere_crossing(ray, h);
			if (x.x > x.y) {
				return false;  // (the ray passes above the height: the far field's crossing has none either)
			}
			t_next = cloud_camera_height_m > h ? x.x : x.y;
			if (t_next < 0.0) {
				return false;
			}
		}
		t_hit = max(t_next, t_from);
		p = camera + eye + dir * t_hit;
	}
	if (t_hit < 0.0) {
		return false;
	}
	// Behind the Earth (VR bugs 2026-10-30): a crossing beyond the ray's own crossing of the model sphere (sea level)
	// lies behind the Earth's curvature - from below a deck the far side's crossing of the column's height (x.y, the
	// ray through the Earth) passed where the scene's depth let it (the 3 km vertical rule at grazing rays, no depth
	// at the horizon's sky / ground blocks) and drew far cap parts just under the horizon line.
	vec2 ground = cloud_sphere_crossing(ray, 0.0);
	bool behind_earth = ground.x <= ground.y && ground.x > 0.0 && t_hit > ground.x;
#ifndef CLOUD_BAKED_DIAG_BEHIND_EARTH
	if (behind_earth) {
		return false;
	}
#endif
	vec3 up = cloud_geodetic_up(p);
	float mu = max(abs(dot(dir, up)), CLOUD_FAR_MU_MIN);
	float footprint = pixel_angle * t_hit / sqrt(max(mu, 0.04));
	if (!cloud_baked_flat_fetch(si, p, footprint, false, f, t_col)) {
		return false;
	}
	if (t_hit >= scene_m) {
		// behind the scene (the column's seen height below high terrain - the field's layers over mountains): drawn on
		// the scene's point while the column's optical top is in front of it or the point lies less than
		// CLOUD_BAKED_FLAT_SCENE_DEPTH_M (vertically) behind the scene (the far field ignores the terrain; a rejection cut
		// the clouds along the depth buffer's steps)
		vec2 x = cloud_sphere_crossing(ray, t_col.x > 0.0 ? t_col.y / t_col.x : 0.0);
		bool top_in_front = x.x <= x.y && (cloud_camera_height_m > t_col.y / max(t_col.x, 1.0e-6) ? x.x : x.y) < scene_m;
		if (!top_in_front && (t_hit - scene_m) * mu > CLOUD_BAKED_FLAT_SCENE_DEPTH_M) {
			return false;
		}
		t_hit = scene_m * CLOUD_BAKED_FLAT_SCENE_PULL;
		p = camera + eye + dir * t_hit;
	}
	float c_col = t_col.x;  // the column rule's nadir cover (the cell's mean of the pattern f.x)
	if (c_col < CLOUD_MIN_FRACTION || !(f.y > 0.0)) {
		return false;
	}
	// the far field's beam cover at the view's slant: 1 - (1 - f)^(n_v + s) = 1 - (1 - C_col)^(1 + s / n_v), applied to
	// the pattern (0 and 1 kept, its mean the far field's); the in-cloud depth the grid mean / that cover
	float fraction = cloud_baked_fraction_of(c_col);
	float cover_v = cloud_beam_cover_of(fraction, mu);
	float n_v = 1.0 + CLOUD_OVERLAP_VERTICAL * (1.0 - fraction);
	float slant = CLOUD_OVERLAP_SLANT * max(sqrt(max(1.0 - mu * mu, 0.0)) / mu - CLOUD_OVERLAP_ONSET_COT, 0.0);
	float cov = 1.0 - pow(max(1.0 - clamp(f.x, 0.0, 1.0), 0.0), 1.0 + slant / n_v);
	float tau_v = f.y / max(cover_v, CLOUD_MIN_FRACTION);
	alpha = cov * (1.0 - exp(-tau_v / mu));
	float tau_light = tau_v;
	// the pattern faded out with the footprint and toward the scaled-space far field (no pattern there: no pop at the
	// switch, CLOUD_BAKED_FLAT_SWITCH_FADE_M)
	float w_pattern = (1.0 - smoothstep(CLOUD_BAKED_FLAT_PATTERN_FADE_M.x, CLOUD_BAKED_FLAT_PATTERN_FADE_M.y, footprint))
			* (1.0 - smoothstep(CLOUD_BAKED_FLAT_SWITCH_FADE_M.x, CLOUD_BAKED_FLAT_SWITCH_FADE_M.y, cloud_camera_height_m));
#ifdef CLOUD_BAKED_DIAG_NO_PIXEL_PATTERN
	w_pattern = 0.0;
#endif
	if (w_pattern > 0.0) {
		// the pattern read on a sphere of fixed height (CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M), not at p: where the
		// column's height rises steeply against the view the shapes were dragged along it (smeared)
		vec2 x_p = cloud_sphere_crossing(ray, CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M);
		float t_p = cloud_camera_height_m > CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M ? x_p.x : x_p.y;
		vec3 p_pattern = x_p.x <= x_p.y && t_p > 0.0 ? camera + eye + dir * t_p : p;
		vec3 pattern = cloud_baked_flat_pattern(p_pattern, up, dir, mu, footprint, cov, tau_v);
		cov = mix(cov, pattern.x, w_pattern);
		alpha = mix(alpha, pattern.y, w_pattern);
		tau_light *= mix(1.0, pattern.z, w_pattern);
	}
	if (!(alpha > 1.0e-6)) {
		return false;
	}
#ifdef CLOUD_BAKED_DIAG_BEHIND_EARTH
	if (behind_earth) {
		color = vec3(alpha, 0.0, 0.0);  // (diagnostic: the old behaviour's hits behind the Earth in red)
		dist = t_hit;
		return true;
	}
#endif
	vec3 radiance = cloud_far_cloudy_radiance(p, up, -dir, tau_light, fraction, vec3(1.0));
	// near the Earth's shadow edge: the slab's light at three heights (CLOUD_BAKED_FLAT_TERMINATOR_MU)
	vec3 pm = p * ATM_STRETCH;
	float mu_s = dot(normalize(pm), normalize(atmosphere_sun_direction * ATM_STRETCH));
	float mu_h = -sqrt(max(1.0 - CLOUD_A_M * CLOUD_A_M / dot(pm, pm), 0.0));
	if (abs(mu_s - mu_h) < CLOUD_BAKED_FLAT_TERMINATOR_MU && t_col.x > 0.0) {
		float h_p = cloud_height_along(ray, t_hit);
		float above = clamp(t_col.y / t_col.x - h_p, 0.0, 6000.0);
		if (above > 10.0) {
			radiance = (radiance + cloud_far_cloudy_radiance(p + up * above, up, -dir, tau_light, fraction, vec3(1.0))
					+ cloud_far_cloudy_radiance(p - up * min(above, max(h_p, 0.0)), up, -dir, tau_light, fraction,
							vec3(1.0))) / 3.0;
		}
	}
	vec3 air_transmittance;
	vec3 air_light = atm_to_point_offset(eye + dir * t_hit, air_transmittance);
	color = air_transmittance * radiance * cov + alpha * air_light;
#ifdef CLOUD_BAKED_DIAG_LEVEL
	color = alpha * vec3(fract(cloud_baked_diag_level), floor(cloud_baked_diag_level) / 3.0, t_col.y / 10000.0);
#endif
	dist = t_hit;
	return true;
}

// [E] Beyond the flat geometry's cap the far field's column draws the clouds (step 6: a camera that outran the re-bake -
// the free camera moves up to its distance to the surface per second, a re-bake takes ~0.3 s - saw the clouds end at
// the shown geometry's cap, a circle inside the disc): cloud_far_layer (the scaled-space Earth's: the same cover rule
// and light) at the ray's ground point, blended in over CLOUD_BAKED_FLAT_FAR_BAND_RAD up to
// CLOUD_BAKED_FLAT_FAR_EDGE_CELLS of the last level's cells inside the cap (the bake's columns end 0.71 cell beyond it,
// the B-spline reads 2 cells around a point). In a settled view the visible cap ends FLAT_DRIFT_RAD (2 deg) inside the
// band: no pixel pays for it.
const float CLOUD_BAKED_FLAT_FAR_BAND_RAD = 0.0087;
const float CLOUD_BAKED_FLAT_FAR_EDGE_CELLS = 3.0;

// The far field's weight (0..1) for the ray in this set: 0 without a ground crossing.
float cloud_baked_flat_far_weight(int si, CloudRay ray, vec3 dir, vec3 eye) {
	vec2 ground = cloud_sphere_crossing(ray, 0.0);
	if (baked.sets[si].outer.x < 0.5 || !(ground.x <= ground.y && ground.x > 0.0)) {
		return 0.0;
	}
	vec3 g = atmosphere_camera_hi + atmosphere_camera_lo + eye + dir * ground.x;
	float c_last = baked.sets[si].axis_x.w * exp2(baked.sets[si].outer.z + baked.sets[si].outer.x - 1.0);
	float edge = baked.sets[si].outer.y - CLOUD_BAKED_FLAT_FAR_EDGE_CELLS * c_last / CLOUD_A_M;
	return smoothstep(edge - CLOUD_BAKED_FLAT_FAR_BAND_RAD, edge, cloud_baked_flat_xy(si, g).z);
}

// The far field's layer for the ray (premultiplied colour with the air in front, alpha) at the distance dist (the
// ray's crossing of CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M); none where the scene is in front by more than
// CLOUD_BAKED_FLAT_SCENE_DEPTH_M (vertically). Evaluated once per pixel after the sets (cloud_baked_shade), and only in
// the march kernels with CLOUD_BAKED_FAR_FALLBACK (CloudBaked: above FLAT_FINE_HEIGHT_M - a camera below moves at most
// ~40 km/s, the 2 deg drift allowance lasts > 5 s against a re-bake's ~0.3 s): its code raised the march's register
// peak - ground +0.14 ms even when not taken (inside the set's function ground / deck +0.15 / +0.5 ms).
vec4 cloud_baked_flat_far_layer(CloudRay ray, vec3 dir, vec3 eye, float scene_m, out float dist) {
	dist = 0.0;
	vec2 ground = cloud_sphere_crossing(ray, 0.0);
	if (!(ground.x <= ground.y && ground.x > 0.0)) {
		return vec4(0.0);
	}
	vec3 g = atmosphere_camera_hi + atmosphere_camera_lo + eye + dir * ground.x;
	vec2 x = cloud_sphere_crossing(ray, CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M);
	dist = x.x <= x.y && x.x > 0.0 ? x.x : ground.x;
	float mu = max(abs(dot(dir, cloud_geodetic_up(g))), CLOUD_FAR_MU_MIN);
	if ((dist - scene_m) * mu > CLOUD_BAKED_FLAT_SCENE_DEPTH_M) {
		return vec4(0.0);
	}
	return cloud_far_layer(g, vec3(1.0));
}

// The march of one set: dir (Earth-local unit) from eye over [d0, d1] of the band. Out: radiance (before the air in
// front), the background's transmittance and the alpha-weighted distance.
void cloud_baked_march(int si, CloudRay ray, vec3 dir,
		vec3 eye, float d0, float d1, out vec3 radiance, out float transmittance_bg, out float weighted) {
	radiance = vec3(0.0);
	transmittance_bg = 1.0;
	weighted = 0.0;
	CloudBakedRay g = cloud_baked_ray(si, eye, dir);
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float k_sc = cloud_transport_scale();
	bool table = cloud_scattering_model == 1;
	float w_light = table ? cloud_phase.w : k_sc;
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	bool sun_ray = dot(dir, sun_dir) > cos(CLOUD_SUN_RAY_ANGLE);
	vec3 diff = sun_dir - dir;  // Sun propagation (-sun_dir) . view propagation (-dir): the scattering angle
	float one_minus_cos = 0.5 * dot(diff, diff);
	float phase = cloud_phase_function(1.0 - one_minus_cos, one_minus_cos);
	float transmittance_sc = 1.0;
	float transmittance_l = 1.0;
	vec3 sun_irradiance = vec3(-1.0);  // set at the first cloud sample (one air lookup per ray)
	vec3 sky_irradiance = vec3(0.0);
	int levels = int(baked.sets[si].axis_y.w);
	float c0 = baked.sets[si].axis_x.w;
	float n = baked.sets[si].up0.w;
	float pixel_angle = cloud_view.params.w;
	int k_min = int(baked.sets[si].misc.z);
	float t = d0;
	float t_clear = d0;     // the last clear sample (an entry is bisected from it)
	bool inside = false;    // the last sample had extinction: the next steps are in-cloud steps
	float beta_last = 0.0;  // in cloud: the extinction and light at t
	vec4 light_last = vec4(0.0);
	for (int i = 0; i < CLOUD_BAKED_MAX_STEPS && t < d1; i++) {
		float h = cloud_height_along(ray, t);
		// (a tolerance: at the band's crossings the float32 height lands either side - a ray starting on the shell
		// was sent to the shell's far crossing, skipping the band)
		if (h > baked.sets[si].band.y + CLOUD_BAKED_BAND_TOLERANCE_M
				|| h < baked.sets[si].band.x - CLOUD_BAKED_BAND_TOLERANCE_M) {
			// Outside the band (a ray grazing its bottom): on to where it enters again.
			CLOUD_COUNT_STEP(false)
			vec2 x = cloud_sphere_crossing(ray, h > baked.sets[si].band.y ? baked.sets[si].band.y : baked.sets[si].band.x);
			float enter = x.x > t ? x.x : (x.y > t ? x.y : 1.0e30);
			if (enter >= d1) {
				break;
			}
			t = enter + 1.0;
			t_clear = t;
			inside = false;
			continue;
		}
		float gx = g.gx0 + t * g.gdx;
		float gy = g.gy0 + t * g.gdy;
		// The level rule (cloud_baked_lod: the clipmap's extent and the footprint, never a dropped level; blended toward
		// the next level over its last cloud_baked_band()) and the hand-over to the flat levels: the extinction
		// scaled by 1 - w; the flat levels' alone from w = 1 on (w grows along the ray: x, y and the footprint grow with t
		// from the anchor).
		CloudBakedPoint q = cloud_baked_point(si, gx, gy, h, pixel_angle * t, k_min);
		if (q.keep <= 0.0) {
			break;
		}
		int k = q.k;
		if (k >= levels) {
			break;  // beyond the outermost level: the flat levels (cloud_baked_set)
		}
		float c = c0 * exp2(float(k));
		int kq = k >> 2;
		int kr = k & 3;
		float dz = baked.sets[si].dz[kq][kr];
		float slope = cloud_height_slope(ray, t);
		float cell_step = min(c, dz / max(abs(slope), 1.0e-3));
		if (!inside) {
			// The block's occupancy: an empty block is skipped to its exit (x, y linear; the height shells exact).
			vec3 cg = vec3(gx / c + 0.5 * n, gy / c + 0.5 * n, (h - baked.sets[si].band.x) / dz);
			ivec3 b = ivec3(floor(cg / float(CLOUD_BAKED_BLOCK)));
			float occ = cloud_baked_o(si, ivec3(b.xy, int(baked.sets[si].oslab[kq][kr])
					+ clamp(b.z, 0, int(baked.sets[si].onz[kq][kr]) - 1)));
			if (occ < 0.5 && q.blend > 0.0) {
				// blending: the next level's block too (its cells may reach where this level's are clear)
				int k1 = k + 1;
				float c1 = 2.0 * c;
				float dz1 = baked.sets[si].dz[k1 >> 2][k1 & 3];
				ivec3 b1 = ivec3(floor(vec3(gx / c1 + 0.5 * n, gy / c1 + 0.5 * n, (h - baked.sets[si].band.x) / dz1)
						/ float(CLOUD_BAKED_BLOCK)));
				occ = cloud_baked_o(si, ivec3(b1.xy, int(baked.sets[si].oslab[k1 >> 2][k1 & 3])
						+ clamp(b1.z, 0, int(baked.sets[si].onz[k1 >> 2][k1 & 3]) - 1)));
			}
			if (occ < 0.5) {
				CLOUD_COUNT_STEP(false)
				float bs = float(CLOUD_BAKED_BLOCK);
				float exit_m = 1.0e30;
				if (g.gdx != 0.0) {
					float edge = ((float(b.x) + (g.gdx > 0.0 ? 1.0 : 0.0)) * bs - 0.5 * n) * c;
					exit_m = min(exit_m, (edge - gx) / g.gdx);
				}
				if (g.gdy != 0.0) {
					float edge = ((float(b.y) + (g.gdy > 0.0 ? 1.0 : 0.0)) * bs - 0.5 * n) * c;
					exit_m = min(exit_m, (edge - gy) / g.gdy);
				}
				// The block's height exit by the ray's direction (cirrus check): rising, the top shell's forward crossing;
				// descending, the bottom's first crossing, else (it turns back up above the bottom) the top's. Taken by
				// the direction, never by "the first root ahead of t": the roots' float32 rounding (~1 m at 6 378 km
				// radii) against cloud_height_along's put a ray just under the top shell past both roots, so the
				// skip went to an x / y face kilometres away - straight through the sheet (pixel holes in cirrus
				// overhead, square rings along the blocks). A root just behind t means the shell is here (exit 0).
				float z_lo = baked.sets[si].band.x + float(b.z) * bs * dz;
				vec2 hi = cloud_sphere_crossing(ray, z_lo + bs * dz);
				float next_z = hi.y;
				if (slope < 0.0) {
					vec2 lo = cloud_sphere_crossing(ray, z_lo);
					if (lo.x <= lo.y && lo.x > t - CLOUD_BAKED_BAND_TOLERANCE_M) {
						next_z = lo.x;
					}
				}
				exit_m = min(exit_m, max(next_z, t) - t);
				t += max(exit_m, 0.0) + max(CLOUD_BAKED_EXIT_NUDGE * c, 1.0);
				t_clear = t;
				continue;
			}
			// An occupied block: a sample per cell far from any edge, nearer the edge coordinate's distance estimate (fix 1:
			// one sample per 100 m cell missed the rims' thin parts ray by ray - holes, speckles and stairs at near
			// outlines), never below the pixel's footprint.
			float gap_x;
			float beta = cloud_baked_beta_gap(si, q, eye + dir * t, pixel_angle * t, gap_x);
			if (beta <= 0.0) {
				CLOUD_COUNT_STEP(false)
				t_clear = t;
				t += transmittance_sc > CLOUD_BAKED_FINE_TRANSMITTANCE
						? clamp(gap_x * CLOUD_BAKED_EDGE_STEP_M, max(CLOUD_BAKED_MIN_CLEAR_STEP_M, pixel_angle * t), cell_step)
						: cell_step;
				continue;
			}
			// The entry: bisected between the last clear sample and this one (the outline found, not sampled).
			float lo_t = t_clear;
			float hi_t = t;
			for (int j = 0; j < CLOUD_BAKED_BISECTIONS; j++) {
				float mid = 0.5 * (lo_t + hi_t);
				float hm = cloud_height_along(ray, mid);
				// (the sample's level and blend: the outline's position, not its level, is searched)
				CloudBakedPoint qm = q;
				qm.g = vec3(g.gx0 + mid * g.gdx, g.gy0 + mid * g.gdy, hm);
				qm.uvw = cloud_baked_uvw(si, k, qm.g.x, qm.g.y, hm);
				float bm = cloud_baked_beta(si, qm, eye + dir * mid, pixel_angle * mid);
				if (bm > 0.0) {
					hi_t = mid;
					beta = bm;
				} else {
					lo_t = mid;
				}
			}
			// The in-cloud steps start at the entry's cloudy end, its extinction and light known.
			t = hi_t;
			beta_last = beta;
			float h_e = cloud_height_along(ray, t);
			light_last = cloud_baked_b2(si, cloud_baked_point(si, g.gx0 + t * g.gdx, g.gy0 + t * g.gdy, h_e,
					pixel_angle * t, k_min));
			light_last *= light_last;  // the sqrt code
			inside = true;
			continue;
		}
		// In cloud: a step of at most the step weight (the old march's dense limit) and a quarter / half a cell
		// (CLOUD_BAKED_FINE_STEP_CELLS), integrated by the trapezoid over the extinction at its ends (a layer thinner than
		// the step still counts).
		CLOUD_COUNT_STEP(true)
		float allowed = CLOUD_BAKED_STEP_WEIGHT / transmittance_sc;
		float dense = allowed < 1.0 ? -log(1.0 - allowed) / (k_sc * max(beta_last, 1.0e-7)) : 1.0e30;
		float step_m = clamp(dense, CLOUD_BAKED_MIN_STEP_M, (transmittance_sc > CLOUD_BAKED_FINE_TRANSMITTANCE
				? CLOUD_BAKED_FINE_STEP_CELLS : CLOUD_BAKED_COARSE_STEP_CELLS) * cell_step);
		step_m = min(step_m, d1 - t);
		float te = t + step_m;
		float he = cloud_height_along(ray, te);
		// (the step's start's level, blend and flat weight, as the volume's march took the start's level)
		CloudBakedPoint qe = q;
		qe.g = vec3(g.gx0 + te * g.gdx, g.gy0 + te * g.gdy, he);
		qe.uvw = cloud_baked_uvw(si, k, qe.g.x, qe.g.y, he);
		float beta_e = cloud_baked_beta(si, qe, eye + dir * te, pixel_angle * te);
		vec4 light_e = light_last;
		if (beta_e > 0.0) {
			light_e = cloud_baked_b2(si, qe);
			light_e *= light_e;  // the sqrt code
		}
		float beta = 0.5 * (beta_last + beta_e);
		vec4 light = (beta_last * light_last + beta_e * light_e) / max(beta_last + beta_e, 1.0e-12);
		float d = t + 0.5 * step_m;
		t = te;
		beta_last = beta_e;
		light_last = light_e;
		if (beta_e <= 0.0) {
			inside = false;
			t_clear = te;
		}
		if (sun_irradiance.x < 0.0) {
			vec3 p = camera + eye + dir * d;
			vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);
			sun_irradiance = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
			float eclipse_x = clamp((d - CLOUD_ECLIPSE_NEAR_M) / (CLOUD_ECLIPSE_FAR_M - CLOUD_ECLIPSE_NEAR_M), 0.0,
					1.0);
			sky_irradiance = ATM_PI * atm_sky_light(p, normal)
					* mix(cloud_group_eclipse_near, cloud_group_eclipse_far, eclipse_x);
#ifdef CLOUD_DIAG_SUN_ONLY
			sky_irradiance = vec3(0.0);
#endif
#ifdef CLOUD_DIAG_SKY_ONLY
			sun_irradiance = vec3(0.0);
#endif
		}
		float tau = beta * step_m;
		float scaled_step = exp(-k_sc * tau);
		float light_step = exp(-w_light * tau);
		float true_step = exp(-tau);
		if (transmittance_sc >= CLOUD_MIN_SCALED_TRANSMITTANCE) {
			float j_sun = phase * light.x + CLOUD_BAKED_J_MAX * light.y;
			radiance += transmittance_sc * sky_irradiance * (CLOUD_BAKED_J_MAX * light.z) * (1.0 - scaled_step) / k_sc
					+ transmittance_l * sun_irradiance * j_sun * (1.0 - light_step) / w_light;
		}
		transmittance_sc *= scaled_step;
		transmittance_l *= light_step;
		float next_bg = transmittance_bg * true_step;
		weighted += d * (transmittance_bg - next_bg);
		transmittance_bg = next_bg;
		if (transmittance_sc < CLOUD_MIN_SCALED_TRANSMITTANCE
				&& !(sun_ray && transmittance_bg > CLOUD_SUN_MIN_TRANSMITTANCE)) {
			break;
		}
	}
}

// One set's clouds along the ray: the volume's march over its band segment [d0, d1] (volume: the segment exists; faded
// out toward the flat levels) and the flat levels behind it. Out: premultiplied colour with the air in front, the
// background's transmittance and the alpha-weighted cloud distance.
void cloud_baked_set(int si, CloudRay ray, vec3 dir, vec3 eye, float d0, float d1, bool volume, float scene_m,
		out vec3 color, out float transmittance, out float cloud_m, out float far_w) {
	color = vec3(0.0);
	transmittance = 1.0;
	cloud_m = 0.0;
	far_w = 0.0;
	float alpha = 0.0;
	if (volume && int(baked.sets[si].misc.z) < int(baked.sets[si].axis_y.w) && cloud_baked_takeover_weight(si) < 1.0) {
		vec3 radiance;
		float weighted;
		cloud_baked_march(si, ray, dir, eye, d0, d1, radiance, transmittance, weighted);
		alpha = 1.0 - transmittance;
		cloud_m = alpha > 1.0e-6 ? weighted / alpha : 0.0;
		if (alpha > 1.0e-6) {
			vec3 air_transmittance;
			vec3 air_light = atm_to_point_offset(eye + dir * cloud_m, air_transmittance);
			color = air_transmittance * radiance + alpha * air_light;
		}
	}
	// The flat levels from where their weight starts, inside the 20 km shell: behind the volume by the hand-over's
	// weight (render A), alone (render B), A and B cross-faded by the takeover's weight.
	float wt = cloud_baked_takeover_weight(si);
	if (baked.sets[si].outer.x < 0.5 || (transmittance < CLOUD_BAKED_FLAT_MIN_TRANSMITTANCE && wt <= 0.0)) {
		return;
	}
#ifdef CLOUD_BAKED_VOLUME_ONLY
	return;
#endif
	float pixel_angle = cloud_view.params.w;
	CloudBakedRay g = cloud_baked_ray(si, eye, dir);
	float t_from = cloud_baked_flat_start(si, g, pixel_angle);
	vec2 shell = cloud_sphere_crossing(ray, CLOUD_TOP_M);
	vec3 color_f = vec3(0.0);
	float alpha_f = 0.0;
	float dist_f = 0.0;
	// beyond the flat geometry's cap the far field's column takes over (its share far_w: cloud_baked_shade adds it)
#ifdef CLOUD_BAKED_FAR_FALLBACK
	far_w = cloud_baked_flat_far_weight(si, ray, dir, eye);
#endif
	if (far_w >= 1.0 || shell.x > shell.y || shell.y <= t_from || t_from >= scene_m
			|| !cloud_baked_flat_shade(si, ray, dir, eye, max(t_from, shell.x), scene_m, pixel_angle, color_f, alpha_f,
					dist_f)) {
		color_f = vec3(0.0);
		alpha_f = 0.0;
	}
	color_f *= 1.0 - far_w;
	alpha_f *= 1.0 - far_w;
	if (alpha_f > 0.0 && transmittance >= CLOUD_BAKED_FLAT_MIN_TRANSMITTANCE) {
		float w = cloud_baked_flat_weight_at(si, cloud_baked_lod(si, max(abs(g.gx0 + dist_f * g.gdx),
				abs(g.gy0 + dist_f * g.gdy)), pixel_angle * dist_f));
		float alpha_w = alpha_f * w;
		float alpha_all = alpha + transmittance * alpha_w;
		cloud_m = alpha_all > 1.0e-6 ? (alpha * cloud_m + transmittance * alpha_w * dist_f) / alpha_all : 0.0;
		color += transmittance * w * color_f;
		transmittance *= 1.0 - alpha_w;
	}
	if (wt > 0.0) {
		// Region by region (cloud_baked_takeover_local): each ground patch shows one render, its edge soft.
		vec2 x2 = cloud_sphere_crossing(ray, CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M);
		float t_p = x2.x <= x2.y && x2.x > 0.0 ? x2.x : (x2.y > 0.0 ? x2.y : CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M.y);
		wt = cloud_baked_takeover_local(atmosphere_camera_hi + atmosphere_camera_lo + eye + dir * t_p, wt);
		float alpha_a = 1.0 - transmittance;
		float alpha_mix = mix(alpha_a, alpha_f, wt);
		cloud_m = alpha_mix > 1.0e-6 ? mix(alpha_a * cloud_m, alpha_f * dist_f, wt) / alpha_mix : 0.0;
		color = mix(color, color_f, wt);
		transmittance = 1.0 - alpha_mix;
	}
}

// One ray of the baked volume and its flat outer levels: dir (Earth-local unit) from eye up to scene_m. Out: the stored
// colour (cloud_stored_color: premultiplied colour with the air in front and the air shadow's loss, T in .a) and the
// alpha-weighted cloud distance.
void cloud_baked_shade(vec3 dir, vec3 eye, float scene_m, out vec4 color_t, out float cloud_m) {
#ifdef CLOUD_BAKED_DIAG_NO_SCENE
	scene_m = 1.0e30;  // (diagnostic: the scene's depth ignored)
#endif
	cloud_m = 0.0;
	color_t = vec4(0.0, 0.0, 0.0, 1.0);
	if (!cloud_enabled || !atmosphere_enabled) {
		return;
	}
	const int si = 0;
	CloudRay ray = cloud_ray_from(dir, eye);
#ifdef CLOUD_BAKED_FAR_REFERENCE
	{
		// (diagnostics: the scaled-space far field's cloud layer, cloud_far.gdshaderinc cloud_far_layer, at the ray's
		// ground point instead of the baked clouds - the flat levels' cover reference at any height)
		vec2 ground = cloud_sphere_crossing(ray, 0.0);
		if (ground.x <= ground.y && ground.x > 0.0) {
			vec4 layer = cloud_far_layer(atmosphere_camera_hi + atmosphere_camera_lo + eye + dir * ground.x, vec3(1.0));
			cloud_m = layer.a > 1.0e-6 ? ground.x : 0.0;
			color_t = cloud_stored_color(layer.rgb, 1.0 - layer.a);
		}
		return;
	}
#endif
#ifdef CLOUD_BAKED_FLAT_TAP_STATS
	{
		// (diagnostics: the flat cell's step 2 tap statistics at the ray's 5 km crossing, raw: rgb = cover, the share of
		// it from taps of opacity < 0.1, < 0.3; a = 1 - the mean opacity)
		vec2 x = cloud_sphere_crossing(ray, 5000.0);
		vec4 f;
		vec2 t_col;
		if (x.x <= x.y && x.x > 0.0 && cloud_baked_flat_fetch(si, atmosphere_camera_hi + atmosphere_camera_lo + eye
				+ dir * x.x, cloud_view.params.w * x.x, true, f, t_col)) {
			color_t = vec4(f.xyz, 1.0 - f.w);
		}
		return;
	}
#endif
	vec2 top = cloud_sphere_crossing(ray, baked.sets[si].band.y);
	vec3 color = vec3(0.0);
	float transmittance = 1.0;
	float d0 = 0.0;
	float d1 = 0.0;
	if (top.x <= top.y && top.y > 0.0) {
		d0 = max(top.x, 0.0);
		d1 = top.y;
		vec2 bottom = cloud_sphere_crossing(ray, baked.sets[si].band.x);
		if (bottom.x <= bottom.y) {
			if (ray.h_c >= baked.sets[si].band.x) {
				if (bottom.x > 0.0) {
					d1 = min(d1, bottom.x);
				}
			} else {
				d0 = max(d0, bottom.y);
			}
		}
		d1 = min(d1, scene_m);
		if (cloud_march_cut_m > 0.0) {
			d1 = min(d1, cloud_march_cut_m);
		}
#ifdef CLOUD_BAKED_DEBUG
		{
			// debug: the grid map's round trip at the band segment's middle (m of error / 10 in red), the field there
			float tm = min(0.5 * (max(top.x, 0.0) + top.y), 20000.0);
			CloudBakedRay gd = cloud_baked_ray(si, eye, dir);
			float gx = gd.gx0 + tm * gd.gdx;
			float gy = gd.gy0 + tm * gd.gdy;
			float hm = cloud_height_along(ray, tm);
			float r = CLOUD_A_M + hm;
			vec3 q = gx * baked.sets[si].axis_x.xyz + gy * baked.sets[si].axis_y.xyz
					+ sqrt(max(r * r - gx * gx - gy * gy, 0.0)) * baked.sets[si].up0.xyz;
			vec3 o = q / ATM_STRETCH - (atmosphere_camera_hi + atmosphere_camera_lo);
			float err = length(o - (eye + dir * tm));
			color_t = vec4(err / 10.0, hm / 20000.0, 0.0, 0.0);
			int kd = cloud_baked_level(si, gx, gy);
			if (cloud_debug_height_m < 0.0 && kd < int(baked.sets[si].axis_y.w)) {
				// the baked field on the sphere of height -cloud_debug_height_m: ramp (red), scale (green), level (blue)
				vec2 hc = cloud_sphere_crossing(ray, -cloud_debug_height_m);
				float th = hc.x > 0.0 ? hc.x : hc.y;
				float gx2 = gd.gx0 + th * gd.gdx;
				float gy2 = gd.gy0 + th * gd.gdy;
				int k2 = max(cloud_baked_level(si, gx2, gy2), int(baked.sets[si].misc.z));
				vec2 a = texture(baked_a0, cloud_baked_uvw(si, min(k2, 5), gx2, gy2, -cloud_debug_height_m)).rg;
				color_t = vec4(cloud_baked_ramp(a.x), a.y / 100.0, float(k2) / 6.0, 0.0);
			}
			cloud_m = tm;
			return;
		}
#endif
	}
	bool volume = d1 > d0;
	float far_w;
	cloud_baked_set(0, ray, dir, eye, d0, d1, volume, scene_m, color, transmittance, cloud_m, far_w);
#ifdef CLOUD_BAKED_FADE
	{
		// A re-bake's swap: the set fading in too, the two renders cross-faded (premultiplied).
		float w = baked.sets[0].misc.y;
		vec2 top1 = cloud_sphere_crossing(ray, baked.sets[1].band.y);
		float e0 = 0.0;
		float e1 = 0.0;
		if (top1.x <= top1.y && top1.y > 0.0) {
			e0 = max(top1.x, 0.0);
			e1 = top1.y;
			vec2 bottom1 = cloud_sphere_crossing(ray, baked.sets[1].band.x);
			if (bottom1.x <= bottom1.y) {
				if (ray.h_c >= baked.sets[1].band.x) {
					if (bottom1.x > 0.0) {
						e1 = min(e1, bottom1.x);
					}
				} else {
					e0 = max(e0, bottom1.y);
				}
			}
			e1 = min(e1, scene_m);
			if (cloud_march_cut_m > 0.0) {
				e1 = min(e1, cloud_march_cut_m);
			}
		}
		vec3 color1;
		float transmittance1;
		float cloud_m1;
		float far_w1;
		cloud_baked_set(1, ray, dir, eye, e0, e1, e1 > e0, scene_m, color1, transmittance1, cloud_m1, far_w1);
		far_w = mix(far_w, far_w1, w);
		float a0 = 1.0 - transmittance;
		float a1 = 1.0 - transmittance1;
		float a = mix(a0, a1, w);
		cloud_m = a > 1.0e-6 ? mix(a0 * cloud_m, a1 * cloud_m1, w) / a : 0.0;
		color = mix(color, color1, w);
		transmittance = mix(transmittance, transmittance1, w);
	}
#endif
#ifdef CLOUD_BAKED_FAR_FALLBACK
	if (far_w > 0.0) {
		// beyond the flat geometry's cap (cloud_baked_flat_far_weight): the far field's share, behind the rest
		float dist_far;
		vec4 far = cloud_baked_flat_far_layer(ray, dir, eye, scene_m, dist_far);
		float a_far = transmittance * far_w * far.a;
		float a_rest = 1.0 - transmittance;
		cloud_m = a_rest + a_far > 1.0e-6 ? (a_rest * cloud_m + a_far * dist_far) / (a_rest + a_far) : 0.0;
		color += transmittance * far_w * far.rgb;
		transmittance -= a_far;
	}
#endif
	float alpha = 1.0 - transmittance;
#ifdef CLOUD_BAKED_DEBUG_ALPHA
	color_t = vec4(alpha, isnan(transmittance) ? 1.0 : 0.0, dot(color, vec3(1.0)) > 0.0 ? 0.5 : 0.0, 0.0);
	return;
#endif
#ifndef CLOUD_ABLATE_AIR_SHADOW
	if (cloud_debug_output == 0) {
		color -= cloud_air_shadow_loss(ray, dir, eye, scene_m, alpha > 1.0e-6 ? cloud_m : 1.0e30, transmittance);
	}
#endif
	color_t = cloud_stored_color(color, transmittance);
}
#endif

// ---- The cloud shadow maps from the bake (CloudShadowEffect; shadow step 2026-10-30) ------------------------------
#ifdef CLOUD_SHADOW_BAKED_FILL
// One map texel per invocation, the texel semantics of cloud_shadow.gdshaderinc (cloud_light.gdshaderinc reads it
// unchanged): along the texel's line toward the Sun (t from the map plane) the first and last t with cloud, the optical
// depth between, the cover (1: the shapes). The line is read from S once (cloud_baked_shadow_line; two sets mixed by
// the cross-fade's weight while a new set fades in); its cloudy heights give t_front / t_back. Where the line's floor
// point lies beyond the volume levels: the field's slanted column there (cloud_column_direct, as the maps' far rule).
layout(set = 1, binding = 0, std140) uniform CloudShadowFillBlock {
	vec4 anchor;  // xyz anchor - camera (m), w half extent (m)
	vec4 e1;      // xyz map axis (unit), w -
	vec4 e2;      // xyz map axis (unit), w the map's size (texels)
	vec4 sun;     // xyz the Sun direction (unit), w 1: a cross-fade's second set is bound (baked.sets[0].misc.y its weight)
} shadow;

layout(rgba32f, set = 1, binding = 1) uniform writeonly image2D out_map;

// t (m from the map plane) where the line rises through height h (its closest approach when it stays above h).
float cloud_shadow_fill_t(CloudRay ray, float h) {
	vec2 c = cloud_sphere_crossing(ray, h);
	return c.x <= c.y ? c.y : -ray.r_c * ray.mu / ray.scale;
}

#ifdef CLOUD_SHADOW_BAKED_DIAG
// Checks only (STARSIM_BAKED_DEFINES=CLOUD_SHADOW_BAKED_DIAG; render_check --shadow_dump): the texel holds instead
// x the map's transmittance for a receiver on the ground (height 0) of this line, y the reference - a dense march
// (CLOUD_SHADOW_DIAG_SAMPLES) from that ground point toward the Sun through the baked shapes (A, set 0; the grid mean
// beyond the levels), z the bake's own light T_s (B.x^2) half a level-0 cell below the line's lowest cloud (-1: no
// cloud), w the horizontal distance of the line's optical-depth-weighted cloud point from the ground point over its
// height (tan of the Sun's zenith for a straight line over a flat Earth; -1: no cloud; -2: the line serves no ground
// receiver - it passes above the ground or meets it beyond 140 km from the camera).
layout(set = 1, binding = 5) uniform sampler3D baked_a_diag;  // linear
layout(set = 1, binding = 6) uniform sampler3D baked_b_diag;  // linear
const int CLOUD_SHADOW_DIAG_SAMPLES = 2048;

float cloud_shadow_diag_beta(vec3 offset, float h) {
	vec3 axis_x = baked.sets[0].axis_x.xyz * ATM_STRETCH;
	vec3 axis_y = baked.sets[0].axis_y.xyz * ATM_STRETCH;
	float gx = baked.sets[0].band.z + dot(offset, axis_x);
	float gy = baked.sets[0].band.w + dot(offset, axis_y);
	int k = max(cloud_baked_level(0, gx, gy), int(baked.sets[0].misc.z));
	if (k >= int(baked.sets[0].axis_y.w)) {
		return cloud_mean_extinction_at(offset, h);
	}
	return cloud_baked_extinction(texture(baked_a_diag, cloud_baked_uvw(0, k, gx, gy, h)).rg);
}

vec4 cloud_shadow_fill_diag(CloudRay ray, vec3 origin, vec3 s, vec4 texel) {
	float t_g = cloud_shadow_fill_t(ray, 0.0);
	float fraction = clamp((texel.y - t_g) / max(texel.y - texel.x, 1.0e-3), 0.0, 1.0);
	float t_map = exp(-texel.z * fraction);
	vec3 g = origin + s * t_g;
	CloudRay rg = cloud_baked_point_ray(g, s);
	float length_m = min(max(cloud_sphere_crossing(rg, baked.sets[0].band.y).y, 0.0), 300000.0);
	float dt = length_m / float(CLOUD_SHADOW_DIAG_SAMPLES);
	float tau = 0.0;
	float h_lo = 1.0e9;
	float t_lo = 0.0;
	vec3 cloud_point = vec3(0.0);
	for (int i = 0; i < CLOUD_SHADOW_DIAG_SAMPLES; i++) {
		float t = (float(i) + 0.5) * dt;
		float h = cloud_height_along(rg, t);
		float beta = cloud_shadow_diag_beta(g + s * t, h);
		if (beta > 0.0) {
			tau += beta * dt;
			cloud_point += beta * dt * vec3(t, h, 0.0);
			if (h < h_lo) {
				h_lo = h;
				t_lo = t;
			}
		}
	}
	// -2: the line meets no ground receiver the map serves (none, or its ground point beyond the far map's use,
	// CloudLayer.SHADOW_FAR_FADE_M's 140 km from the camera: the lines grazing the Earth toward the terminator)
	vec2 ground_c = cloud_sphere_crossing(ray, 0.0);
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	float flag = ground_c.x <= ground_c.y && length(g - up_c * dot(g, up_c)) < 140000.0 ? -1.0 : -2.0;
	if (tau <= 0.0 || flag < -1.5) {
		return vec4(t_map, exp(-tau), -1.0, flag);
	}
	cloud_point /= tau;
	// the bake's light just below the lowest cloud on the line
	float t_b = max(t_lo - 0.5 * baked.sets[0].dz[0][0] / max(rg.mu, 0.05), 0.0);
	vec3 pb = g + s * t_b;
	float hb = cloud_height_along(rg, t_b);
	vec3 axis_x = baked.sets[0].axis_x.xyz * ATM_STRETCH;
	vec3 axis_y = baked.sets[0].axis_y.xyz * ATM_STRETCH;
	float gx = baked.sets[0].band.z + dot(pb, axis_x);
	float gy = baked.sets[0].band.w + dot(pb, axis_y);
	int k = max(cloud_baked_level(0, gx, gy), int(baked.sets[0].misc.z));
	float t_bake = -1.0;
	if (k < int(baked.sets[0].axis_y.w)) {
		float b = texture(baked_b_diag, cloud_baked_uvw(0, k, gx, gy, hb)).x;
		t_bake = b * b;
	}
	// horizontal distance of the cloud_point point from the ground point over its height
	vec3 pc = g + s * cloud_point.x;
	vec3 up = normalize((atmosphere_camera_hi + atmosphere_camera_lo + g) * ATM_STRETCH);
	vec3 d = (pc - g) * ATM_STRETCH;
	float horizontal = length(d - up * dot(d, up));
	return vec4(t_map, exp(-tau), t_bake, horizontal / max(cloud_point.y, 1.0));
}
#endif

void main() {
	int size = int(shadow.e2.w);
	ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
	if (texel.x >= size || texel.y >= size) {
		return;
	}
	float half_extent = shadow.anchor.w;
	float texel_m = 2.0 * half_extent / float(size);
	vec2 uv = (vec2(texel) + 0.5) * texel_m - half_extent;
	vec3 s = shadow.sun.xyz;
	vec3 origin = shadow.anchor.xyz + uv.x * shadow.e1.xyz + uv.y * shadow.e2.xyz;  // t = 0, from the camera
	CloudRay ray = cloud_baked_point_ray(origin, s);
	vec4 line;
	bool inside = cloud_baked_shadow_line(0, origin, s, 0.0, line);
	float w = baked.sets[0].misc.y;
	if (shadow.sun.w > 0.5 && w > 0.0) {
		vec4 line1;
		if (cloud_baked_shadow_line(1, origin, s, 0.0, line1) && inside) {
			// heights weighted by each set's optical depth (a clear set's do not count)
			float tau0 = -log(max(line.x, 1.0e-30)) * (1.0 - w);
			float tau1 = -log(max(line1.x, 1.0e-30)) * w;
			float t_mix = mix(line.x, line1.x, w);
			if (tau0 + tau1 > 0.0) {
				line.yz = (tau0 * line.yz + tau1 * line1.yz) / (tau0 + tau1);
			}
			line.x = t_mix;
		}
	}
	float od;
	float h_lo;
	float h_hi;
	if (inside) {
		if (line.x >= 1.0) {
			float t_top = cloud_shadow_fill_t(ray, baked.sets[0].band.y);
			vec4 clear = vec4(t_top, t_top, 0.0, 1.0);
#ifdef CLOUD_SHADOW_BAKED_DIAG
			clear = cloud_shadow_fill_diag(ray, origin, s, clear);
#endif
			imageStore(out_map, texel, clear);
			return;
		}
		od = -log(max(line.x, 1.0e-30));
		h_lo = line.y;
		h_hi = max(line.z, line.y + 1.0);
	} else {
		vec3 q = origin + s * line.w;
		vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + q;
		float mu0 = dot(cloud_geodetic_up(p), s);
		float analytic = cloud_column_direct(cloud_column_mean_depth(cloud_lat_lon(p), baked.sets[0].band.x), mu0);
		od = -log(max(analytic, 1.0e-30));
		h_lo = baked.sets[0].band.x;
		h_hi = baked.sets[0].band.y;
	}
	vec4 result = vec4(cloud_shadow_fill_t(ray, h_lo), cloud_shadow_fill_t(ray, h_hi), od, 1.0);
#ifdef CLOUD_SHADOW_BAKED_DIAG
	result = cloud_shadow_fill_diag(ray, origin, s, result);
#endif
	imageStore(out_map, texel, result);
}
#endif
