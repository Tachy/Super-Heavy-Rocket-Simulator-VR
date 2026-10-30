// The baked volume's sets: their grid block (CloudBaked._set_floats), the clipmap's level rule, texture coordinates
// and the edge coordinate's extinction - shared by the bake, the march and the shadow maps' fill
// (cloud_baked.gdshaderinc).

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
	               // first volume level marched (the finer ones dropped with the altitude; all of them above 62 km), w:
	               // the first level with shapes and S (<= z: above 62 km the shadows' levels are baked, not marched)
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

#endif
