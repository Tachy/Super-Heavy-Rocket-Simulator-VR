// The baked cloud volume prototype ("Weg 2", 2026-10-29): the clouds' shape and light precomputed once in a world-fixed
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
// The phase function is applied per frame (per ray: one cosine), J = P(theta) T_s + J_ms.
// Kernels: CLOUD_BAKED_BAND (the field's lowest and highest cloudy layer over the region: the band),
// CLOUD_BAKED_DENSITY (A), CLOUD_BAKED_OCCUPANCY (O), CLOUD_BAKED_LIGHT (B; marches toward the Sun through A),
// CLOUD_BAKED_MARCH (with CLOUD_MARCH; CLOUD_BAKED_FADE: two sets marched and cross-faded - a re-bake's swap).

#if defined(CLOUD_BAKED_DENSITY) || defined(CLOUD_BAKED_OCCUPANCY) || defined(CLOUD_BAKED_LIGHT) || defined(CLOUD_BAKED_BAND)
#define CLOUD_BAKED_BUILD
#endif
#if defined(CLOUD_BAKED_BUILD) || defined(CLOUD_BAKED_MARCH)

#ifdef CLOUD_BAKED_MARCH
#define CLOUD_BAKED_GRID_BINDING 53
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
	vec4 misc;     // x: occupancy texture depth, y: (set 0) the cross-fade weight of set 1 (CLOUD_BAKED_FADE), z, w: -
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

// The edge coordinate whose ramp is r (0 < r < 1): the inverse smoothstep.
float cloud_baked_unramp(float r) {
	return 0.5 - sin(asin(1.0 - 2.0 * r) / 3.0);
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

// [E] Samples of the Sun's path (quadratic spacing: dense at the voxel, the first interval L / n^2), its cap.
const int CLOUD_BAKED_SUN_SAMPLES = 32;
const float CLOUD_BAKED_SUN_PATH_MAX_M = 300000.0;

// The optical depth toward the Sun from camera + offset (height h): the baked volume's extinction where a level holds
// the point, the field's grid mean beyond; up to the band's top (0 when the path meets the ground: Earth's shadow is
// atm_sunlit's / the Sun's transmittance's).
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
	float n = float(CLOUD_BAKED_SUN_SAMPLES);
	for (int i = 0; i < CLOUD_BAKED_SUN_SAMPLES; i++) {
		float a = length_m * float(i * i) / (n * n);
		float b = length_m * float((i + 1) * (i + 1)) / (n * n);
		float t = 0.5 * (a + b);
		float h = cloud_height_along(ray, t);
		float gx = gx0 + t * gdx;
		float gy = gy0 + t * gdy;
		int k = cloud_baked_level(si, gx, gy);
		float beta;
		if (k < int(baked.sets[si].axis_y.w)) {
			beta = cloud_baked_extinction(texture(baked_a_in, cloud_baked_uvw(si, k, gx, gy, h)).rg);
		} else {
			beta = cloud_mean_extinction_at(offset + sun * t, h);
		}
		tau += beta * (b - a);
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

// The extinction (1/m) at a march point (uvw in level k, camera offset, pixel footprint): the baked field plus, with
// CLOUD_BAKED_DETAIL in level 0, one fetch of the detail's level 1 (62.5 m cells; the bake holds level 0) at the
// pixel's band limit near the edge - the old march's erosion term (cloud_detail_perturbation) on the edge coordinate.
// gap_x: how far the point lies outside any cloud the per-sample detail could add, in edge widths (the signed edge
// coordinate below the detail's reach; <= 0 within it) - the clear march's distance estimate.
float cloud_baked_beta_gap(int si, int k, vec3 uvw, vec3 offset, float footprint_m, out float gap_x) {
	vec2 a = cloud_baked_a(si, uvw);
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
			float width_noise = cloud_baked_b(si, uvw).w;
			float width = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, width_noise);
			vec3 q = fract(cloud_detail_origin * 4.0) + offset * (4.0 / CLOUD_DETAIL_TILE_M);
			float d = dot(texture(cloud_noise_detail, q).rgb, cloud_detail_weights);
			float term = w1 * cloud_level_amplitude[1] * (d - cloud_detail_mean);
			a.x += (term < 0.0 ? term : cloud_level_lift[1] * term) / max(width, 1.0e-6);
		}
	}
#endif
	return cloud_baked_extinction(a);
}

float cloud_baked_beta(int si, int k, vec3 uvw, vec3 offset, float footprint_m) {
	float gap_x;
	return cloud_baked_beta_gap(si, k, uvw, offset, footprint_m, gap_x);
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
		int k = cloud_baked_level(si, gx, gy);
		if (k >= levels) {
			break;  // beyond the outermost level: no cloud in the prototype
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
				float z_lo = baked.sets[si].band.x + float(b.z) * bs * dz;
				vec2 lo = cloud_sphere_crossing(ray, z_lo);
				vec2 hi = cloud_sphere_crossing(ray, z_lo + bs * dz);
				float next_lo = lo.x > t ? lo.x : (lo.y > t ? lo.y : 1.0e30);
				float next_hi = hi.x > t ? hi.x : (hi.y > t ? hi.y : 1.0e30);
				exit_m = min(exit_m, min(next_lo, next_hi) - t);
				t += max(exit_m, 0.0) + max(CLOUD_BAKED_EXIT_NUDGE * c, 1.0);
				t_clear = t;
				continue;
			}
			// An occupied block: a sample per cell far from any edge, nearer the edge coordinate's distance estimate (fix 1:
			// one sample per 100 m cell missed the rims' thin parts ray by ray - holes, speckles and stairs at near
			// outlines), never below the pixel's footprint.
			float gap_x;
			float beta = cloud_baked_beta_gap(si, k, cloud_baked_uvw(si, k, gx, gy, h), eye + dir * t, pixel_angle * t,
					gap_x);
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
				float bm = cloud_baked_beta(si, k, cloud_baked_uvw(si, k, g.gx0 + mid * g.gdx, g.gy0 + mid * g.gdy, hm),
						eye + dir * mid, pixel_angle * mid);
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
			light_last = cloud_baked_b(si, cloud_baked_uvw(si, k, g.gx0 + t * g.gdx, g.gy0 + t * g.gdy, h_e));
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
		vec3 uvw = cloud_baked_uvw(si, k, g.gx0 + te * g.gdx, g.gy0 + te * g.gdy, he);
		float beta_e = cloud_baked_beta(si, k, uvw, eye + dir * te, pixel_angle * te);
		vec4 light_e = light_last;
		if (beta_e > 0.0) {
			light_e = cloud_baked_b(si, uvw);
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

// One ray of the baked volume: dir (Earth-local unit) from eye up to scene_m. Out: the stored colour
// (cloud_stored_color: premultiplied colour with the air in front and the air shadow's loss, T in .a) and the
// alpha-weighted cloud distance.
void cloud_baked_shade(vec3 dir, vec3 eye, float scene_m, out vec4 color_t, out float cloud_m) {
	cloud_m = 0.0;
	color_t = vec4(0.0, 0.0, 0.0, 1.0);
	if (!cloud_enabled || !atmosphere_enabled) {
		return;
	}
	const int si = 0;
	CloudRay ray = cloud_ray_from(dir, eye);
	vec2 top = cloud_sphere_crossing(ray, baked.sets[si].band.y);
	vec3 color = vec3(0.0);
	float transmittance = 1.0;
	float alpha = 0.0;
	if (top.x <= top.y && top.y > 0.0) {
		float d0 = max(top.x, 0.0);
		float d1 = top.y;
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
				int k2 = cloud_baked_level(si, gx2, gy2);
				vec2 a = texture(baked_a0, cloud_baked_uvw(si, min(k2, 5), gx2, gy2, -cloud_debug_height_m)).rg;
				color_t = vec4(cloud_baked_ramp(a.x), a.y / 100.0, float(k2) / 6.0, 0.0);
			}
			cloud_m = tm;
			return;
		}
#endif
		if (d1 > d0) {
			vec3 radiance;
			float weighted;
			cloud_baked_march(0, ray, dir, eye, d0, d1, radiance, transmittance,
					weighted);
#ifdef CLOUD_BAKED_FADE
			// A re-bake's swap: the set fading in marched too, the two renders cross-faded (premultiplied).
			float w = baked.sets[0].misc.y;
			vec3 radiance1;
			float transmittance1;
			float weighted1;
			cloud_baked_march(1, ray, dir, eye, d0, d1, radiance1,
					transmittance1, weighted1);
			radiance = mix(radiance, radiance1, w);
			transmittance = mix(transmittance, transmittance1, w);
			weighted = mix(weighted, weighted1, w);
#endif
			alpha = 1.0 - transmittance;
			cloud_m = alpha > 1.0e-6 ? weighted / alpha : 0.0;
#ifdef CLOUD_BAKED_DEBUG_ALPHA
			color_t = vec4(alpha, isnan(transmittance) ? 1.0 : 0.0, dot(radiance, vec3(1.0)) > 0.0 ? 0.5 : 0.0, 0.0);
			return;
#endif
			if (alpha > 1.0e-6) {
				vec3 air_transmittance;
				vec3 air_light = atm_to_point_offset(eye + dir * cloud_m, air_transmittance);
				color = air_transmittance * radiance + alpha * air_light;
			}
		}
	}
#ifndef CLOUD_ABLATE_AIR_SHADOW
	if (cloud_debug_output == 0) {
		color -= cloud_air_shadow_loss(ray, dir, eye, scene_m, alpha > 1.0e-6 ? cloud_m : 1.0e30, transmittance);
	}
#endif
	color_t = cloud_stored_color(color, transmittance);
}
#endif
