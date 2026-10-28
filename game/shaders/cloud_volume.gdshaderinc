// The cloud world volume's GPU side (world-volume plan Task 5; spec B; CloudWorldVolume): the CloudVolume block, the
// grid of the clipmap (CloudWorldVolumeLayout's azimuthal-equidistant frame, height above WGS84) evaluated relative to
// the camera, and the lookups. GLSL 450 only, compiled after the march's files (clouds.gdshaderinc: cloud_blend,
// CLOUD_A_M) - never included by a Godot shader.
//
// Numerics (preflight C22): no absolute float32 position or grid value is ever differenced. The CPU (float64) gives per
// frame the camera's grid point (hi + lo), geodetic normal n_c, height h_c, prime vertical radius N_c and its polar
// coordinates about the frame origin; the GPU works on the differences to the camera alone - the geodetic normal by
// the fixed point n = normalize(p + e^2 N(n) n_y Y) written for n - n_c (contraction e^2 per step), the azimuthal
// equidistant map through the sinc / x-cosec series in theta^2 (theta <= 0.3 rad: a re-frame at 1000 km plus level 7's
// 820 km; truncation < 1e-9) - and adds the camera's grid point last (one float32 rounding of the result).
//
// Layout of a level's textures (CloudWorldVolume): toroidal (texel = cell mod size), sampled with repeat; density
// RGBA16F (1/km): x rho, y sigma_c of channel pair 0 (slot 0's snapshot), z, w those of pair 1 (slot 1); light RGBA16F
// (Task 6); the max-extinction mip R16F (1/km) and the Chebyshev distance field R8 (blocks / 255) at
// 1 / CLOUD_VOLUME_MIP_DIV resolution. Index 8 is the transient spare level of a re-frame.

#ifndef CLOUD_VOLUME_BLOCK_BINDING
#define CLOUD_VOLUME_BLOCK_BINDING 40
#endif

// CloudWorldVolumeLayout's constants (change together).
const int CLOUD_VOLUME_LEVELS = 8;
const int CLOUD_VOLUME_SPARE = 8;  // the spare's texture index
const int CLOUD_VOLUME_CELLS_H = 256;
const int CLOUD_VOLUME_CELLS_V = 128;
const int CLOUD_VOLUME_WINDOWED_LEVELS = 2;
const float CLOUD_VOLUME_SHELL_BOTTOM_M = -500.0;
const float CLOUD_VOLUME_FADE_CELLS = 8.0;
const int CLOUD_VOLUME_MIP_DIV = 4;
const ivec3 CLOUD_VOLUME_SIZE = ivec3(256, 256, 128);
const ivec3 CLOUD_VOLUME_MIP_SIZE = ivec3(64, 64, 32);
const float CLOUD_VOLUME_CELL_H_M[8] = float[8](50.0, 100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0);
const float CLOUD_VOLUME_CELL_V_M[8] = float[8](25.0, 50.0, 164.0625, 164.0625, 164.0625, 164.0625, 164.0625,
		164.0625);
// [S] WGS84 first eccentricity squared 2 f - f^2 (core constants.hpp; CloudWorldVolumeLayout.E2).
const float CLOUD_VOLUME_E2 = 0.0066943799901413165;
// Iterations of the camera-relative geodetic fixed point (error e^2^k x the angle: 5 steps < 1e-10 rad at 0.3 rad).
const int CLOUD_VOLUME_GEODETIC_ITERATIONS = 5;
// Stored extinction unit: 1/km (a half float keeps 3 digits from 6e-5 /km up).
const float CLOUD_VOLUME_PER_KM = 1000.0;
// R8 UNORM holds the distance (blocks) / this.
const float CLOUD_VOLUME_DISTANCE_SCALE = 255.0;
// [E] The largest ratio of a grid length to the true length it stands for: the azimuthal scale theta / sin(theta)
// (1.015 at 0.3 rad) times the Gaussian sphere's radius over the ellipsoid's smallest radius of curvature (1.007) -
// a true step this much shorter than the grid's free distance stays inside the free box (cloud_volume_free_m).
const float CLOUD_VOLUME_MAX_SCALE = 1.03;
// Level flags (CloudWorldVolume.globals_bytes).
const int CLOUD_VOLUME_ACTIVE = 1;
const int CLOUD_VOLUME_READY = 2;
const int CLOUD_VOLUME_VERTICAL_FADE = 4;
const int CLOUD_VOLUME_CLAMPED_BOTTOM = 8;
const int CLOUD_VOLUME_FRAME_SHIFT = 4;

// One frame's camera terms (float64 on the CPU, CloudWorldVolume.frame_floats). The camera's geodetic normal n_c is
// split into its float32 part n_h (|n_h| = 1 + its rounding) and the rest; the reference grid point g_r is
// R (n_h . east, n_h . north) t / sin(t), t = atan2(|(n_h . east, n_h . north)|, n_h . n0) - exactly what the inverse
// map's series give for n_h, so g_r cancels out of grid = g_r + (g(n_p) - g_r) - and the forward map works from the
// unit normal at g_r (its difference to n_c in `unit`).
struct CloudVolumeFrame {
	vec4 grid_hi;    // xyz: (g_r, h_c) float32 part; w: the projection sphere's radius R
	vec4 grid_lo;    // xyz: its remainder; w: 0
	vec4 normal;     // xyz: n_h; w: the camera's prime vertical radius N_c
	vec4 normal_lo;  // xyz: n_c - n_h; w: |n_h|^2 - 1
	vec4 up0;        // xyz: the frame origin's normal n0; w: N_c + h_c
	vec4 east;       // xyz: the frame's east; w: 0
	vec4 north;      // xyz: the frame's north; w: 0
	vec4 polar;      // n_h . east, n_h . north, n_h . n0, |(n_h . east, n_h . north)|
	vec4 unit;       // xyz: the unit normal at g_r minus n_c; w: 0
};

layout(set = 1, binding = CLOUD_VOLUME_BLOCK_BINDING, std140) uniform CloudVolume {
	CloudVolumeFrame frames[2];  // per level its frame (flags): two during a re-frame
	ivec4 levels[9];             // xyz: the window's origin (cells); w: flags (frame << CLOUD_VOLUME_FRAME_SHIFT)
	vec4 spare;                  // x: the spare's level (-1: none), y: its cross-fade weight (0 while built)
} cloud_volume;

// sinc(t) = sin(t) / t and t / sin(t) as series in t^2, and their differences between t_p and t_c given p2 = t_p^2,
// c2 = t_c^2 and d2 = p2 - c2 (exact differences: never sinc(t_p) - sinc(t_c) of two rounded values).
float cloud_volume_sinc(float t2) {
	return 1.0 + t2 * (-1.0 / 6.0 + t2 * (1.0 / 120.0 + t2 * (-1.0 / 5040.0 + t2 / 362880.0)));
}

float cloud_volume_sinc_delta(float p2, float c2, float d2) {
	float s1 = p2 + c2;
	float s2 = p2 * p2 + p2 * c2 + c2 * c2;
	float s3 = (p2 * p2 + c2 * c2) * s1;
	return d2 * (-1.0 / 6.0 + s1 / 120.0 - s2 / 5040.0 + s3 / 362880.0);
}

float cloud_volume_cosec(float t2) {
	return 1.0 + t2 * (1.0 / 6.0 + t2 * (7.0 / 360.0 + t2 * (31.0 / 15120.0 + t2 * 127.0 / 604800.0)));
}

float cloud_volume_cosec_delta(float p2, float c2, float d2) {
	float s1 = p2 + c2;
	float s2 = p2 * p2 + p2 * c2 + c2 * c2;
	float s3 = (p2 * p2 + c2 * c2) * s1;
	return d2 * (1.0 / 6.0 + 7.0 * s1 / 360.0 + 31.0 * s2 / 15120.0 + 127.0 * s3 / 604800.0);
}

// N(n) - N(n_c) of the normal n = n_c + dn (prime vertical radius a / sqrt(1 - e^2 n_y^2), its difference in closed
// form: (B - A) / (sqrt(A) sqrt(B) (sqrt(A) + sqrt(B))) with B - A = e^2 dn_y (2 n_c,y + dn_y)).
float cloud_volume_dprime(vec3 n_c, vec3 dn) {
	float ny = n_c.y + dn.y;
	float a = sqrt(1.0 - CLOUD_VOLUME_E2 * ny * ny);
	float b = sqrt(1.0 - CLOUD_VOLUME_E2 * n_c.y * n_c.y);
	return CLOUD_A_M * CLOUD_VOLUME_E2 * dn.y * (2.0 * n_c.y + dn.y) / (a * b * (a + b));
}

// The grid offset (m) from the camera's grid point of the point camera + offset_m (Earth-local), in frame f.
vec3 cloud_volume_grid_delta(int f, vec3 offset_m) {
	CloudVolumeFrame fr = cloud_volume.frames[f];
	vec3 n_h = fr.normal.xyz;
	vec3 n_lo = fr.normal_lo.xyz;
	float big_n = fr.normal.w;
	float l = fr.up0.w;
	// The geodetic normal n_p and height relative to the camera: v = p + e^2 N(n) n_y Y = (N + h) n at the solution; at
	// the camera v = l n_c, so v = l n_h + w with w = offset + l (n_c - n_h) + e^2 (N_p n_p,y - N_c n_c,y) Y.
	vec3 base = offset_m + l * n_lo;
	vec3 dn = vec3(0.0);  // n_p - n_h
	float dprime = 0.0;   // N_p - N_c
	float q2m1 = 0.0;     // |n_h + w / l|^2 - 1
	float q = 1.0;
	for (int i = 0; i < CLOUD_VOLUME_GEODETIC_ITERATIONS; i++) {
		vec3 w = base;
		w.y += CLOUD_VOLUME_E2 * (dprime * (n_h.y + dn.y) + big_n * (dn.y - n_lo.y));
		vec3 wl = w / l;
		q2m1 = fr.normal_lo.w + 2.0 * dot(n_h, wl) + dot(wl, wl);
		q = sqrt(1.0 + q2m1);
		dn = n_h * (-q2m1 / (q * (1.0 + q))) + wl / q;
		dprime = cloud_volume_dprime(n_h, dn - n_lo);
	}
	float dh = l * q2m1 / (q + 1.0) - dprime;  // |v| - l - (N_p - N_c)
	// The azimuthal-equidistant map g = R (a, b) t / sin(t), (a, b) = (n . east, n . north), t = atan2(|(a, b)|, n . n0).
	float da = dot(dn, fr.east.xyz);
	float db = dot(dn, fr.north.xyz);
	float dz = dot(dn, fr.up0.xyz);
	vec4 pc = fr.polar;
	float ap = pc.x + da;
	float bp = pc.y + db;
	float rp = length(vec2(ap, bp));
	float zp = pc.z + dz;
	float rs = rp + pc.w;
	float dr = rs > 0.0 ? (2.0 * (pc.x * da + pc.y * db) + da * da + db * db) / rs : 0.0;
	float dt = atan(dr * pc.z - dz * pc.w, zp * pc.z + rp * pc.w);  // t_p - t_c
	float tc = atan(pc.w, pc.z);
	float tp = tc + dt;
	float c2 = tc * tc;
	float p2 = tp * tp;
	float d2 = dt * (tp + tc);
	float r = fr.grid_hi.w;
	vec2 dxy = r * (vec2(da, db) * cloud_volume_cosec(p2) + pc.xy * cloud_volume_cosec_delta(p2, c2, d2));
	return vec3(dxy, dh);
}

// The grid point (m: x east, y north along the projection sphere, z the height above WGS84) of camera + offset in frame
// f; within 1e-3 of the containing level's cell of CloudWorldVolumeLayout.grid_of (cloud_check.gd section 16).
vec3 cloud_volume_grid_in(int f, vec3 offset_from_camera_m) {
	CloudVolumeFrame fr = cloud_volume.frames[f];
	return fr.grid_hi.xyz + (fr.grid_lo.xyz + cloud_volume_grid_delta(f, offset_from_camera_m));
}

vec3 cloud_volume_grid(vec3 offset_from_camera_m) {
	return cloud_volume_grid_in(0, offset_from_camera_m);
}

// The Earth-local offset from the camera of the grid point camera grid + dg (frame f) and its geodetic normal (normal).
vec3 cloud_volume_offset_of(int f, vec3 dg, out vec3 normal) {
	CloudVolumeFrame fr = cloud_volume.frames[f];
	float r = fr.grid_hi.w;
	vec2 gc = fr.grid_hi.xy + fr.grid_lo.xy;
	vec2 gp = gc + dg.xy;
	float c2 = dot(gc, gc) / (r * r);
	float p2 = dot(gp, gp) / (r * r);
	float d2 = (2.0 * dot(gc, dg.xy) + dot(dg.xy, dg.xy)) / (r * r);  // t_p^2 - t_c^2
	float tc = sqrt(c2);
	float tp = sqrt(p2);
	float dt = tp + tc > 0.0 ? d2 / (tp + tc) : 0.0;
	// n = n0 cos(t) + (east, north) . g sinc(t) / R: the difference to the unit normal at g_r term by term, then to n_c.
	float dcos = -2.0 * sin(tc + 0.5 * dt) * sin(0.5 * dt);
	vec2 side = (cloud_volume_sinc(p2) * dg.xy + cloud_volume_sinc_delta(p2, c2, d2) * gc) / r;
	vec3 dn = fr.up0.xyz * dcos + fr.east.xyz * side.x + fr.north.xyz * side.y + fr.unit.xyz;  // n_p - n_c
	vec3 n_c = fr.normal.xyz;
	normal = n_c + (fr.normal_lo.xyz + dn);
	float dprime = cloud_volume_dprime(n_c, dn);
	float h_c = fr.grid_hi.z + fr.grid_lo.z;
	vec3 d = vec3(1.0, 1.0 - CLOUD_VOLUME_E2, 1.0);
	// p - p_c = D (N n - N_c n_c) + h n - h_c n_c, D the ellipsoid's polar factor.
	return d * ((fr.normal.w + dprime) * dn + dprime * n_c) + h_c * dn + dg.z * normal;
}

// Continuous cell coordinates (global cells) of a grid point at level k (cell i spans [i, i + 1]).
vec3 cloud_volume_cells(int k, vec3 grid_m) {
	return vec3(grid_m.x / CLOUD_VOLUME_CELL_H_M[k], grid_m.y / CLOUD_VOLUME_CELL_H_M[k],
			(grid_m.z - CLOUD_VOLUME_SHELL_BOTTOM_M) / CLOUD_VOLUME_CELL_V_M[k]);
}

// c mod size in [0, size) (GLSL leaves % of a negative operand undefined).
ivec3 cloud_volume_wrap(ivec3 c, ivec3 size) {
	return c - size * ivec3(floor(vec3(c) / vec3(size)));
}

int cloud_volume_wrap1(int c, int size) {
	return c - size * int(floor(float(c) / float(size)));
}

// The level's weight at a grid point (CloudWorldVolumeLayout.fade_weight_at): 1 inside, 0 at the outermost cell,
// linear over CLOUD_VOLUME_FADE_CELLS; levels 0-1 also at their vertical window edges, except a clamped bottom.
float cloud_volume_fade(int index, int k, vec3 grid_m) {
	ivec4 lv = cloud_volume.levels[index];
	vec3 u = cloud_volume_cells(k, grid_m) - vec3(lv.xyz);
	float d = min(min(u.x, float(CLOUD_VOLUME_CELLS_H) - u.x), min(u.y, float(CLOUD_VOLUME_CELLS_H) - u.y));
	if ((lv.w & CLOUD_VOLUME_VERTICAL_FADE) != 0) {
		d = min(d, float(CLOUD_VOLUME_CELLS_V) - u.z);
		if ((lv.w & CLOUD_VOLUME_CLAMPED_BOTTOM) == 0) {
			d = min(d, u.z);
		}
	}
	return clamp((d - 0.5) / CLOUD_VOLUME_FADE_CELLS, 0.0, 1.0);
}

// The finest shown level (active and ready, in frame 0) whose window holds the grid point (frame 0), its weight in
// fade; -1 when none (the march's fallback). A level of the other frame during a re-frame needs that frame's grid
// (cloud_volume_grid_in(1, ...)): it is skipped here.
int cloud_volume_level(vec3 grid_m, out float fade) {
	for (int k = 0; k < CLOUD_VOLUME_LEVELS; k++) {
		int flags = cloud_volume.levels[k].w;
		if ((flags & (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)) != (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)
				|| (flags >> CLOUD_VOLUME_FRAME_SHIFT) != 0) {
			continue;
		}
		float w = cloud_volume_fade(k, k, grid_m);
		if (w > 0.0) {
			fade = w;
			return k;
		}
	}
	fade = 0.0;
	return -1;
}

// The texture coordinate of a grid point in texture `index` (level k): window-local cells plus the window origin's
// texel (toroidal; the repeat sampler wraps), levels 2-7 clamped to the shell vertically (not toroidal there).
vec3 cloud_volume_uvw(int index, int k, vec3 grid_m) {
	ivec3 o = cloud_volume.levels[index].xyz;
	vec3 t = cloud_volume_cells(k, grid_m) - vec3(o) + vec3(cloud_volume_wrap(o, CLOUD_VOLUME_SIZE));
	if (k >= CLOUD_VOLUME_WINDOWED_LEVELS) {
		t.z = clamp(t.z, 0.5, float(CLOUD_VOLUME_CELLS_V) - 0.5);
	}
	return t / vec3(CLOUD_VOLUME_SIZE);
}

#ifdef CLOUD_VOLUME_LOOKUP
// The lookup's textures (set 1): density / light linear + repeat, the max mip and the distance field nearest.
#define CLOUD_VOLUME_TEXTURES(name, first) \
	layout(set = 1, binding = first + 0) uniform sampler3D name##0; \
	layout(set = 1, binding = first + 1) uniform sampler3D name##1; \
	layout(set = 1, binding = first + 2) uniform sampler3D name##2; \
	layout(set = 1, binding = first + 3) uniform sampler3D name##3; \
	layout(set = 1, binding = first + 4) uniform sampler3D name##4; \
	layout(set = 1, binding = first + 5) uniform sampler3D name##5; \
	layout(set = 1, binding = first + 6) uniform sampler3D name##6; \
	layout(set = 1, binding = first + 7) uniform sampler3D name##7; \
	layout(set = 1, binding = first + 8) uniform sampler3D name##8;
CLOUD_VOLUME_TEXTURES(cloud_volume_density_tex, CLOUD_VOLUME_BLOCK_BINDING + 1)
CLOUD_VOLUME_TEXTURES(cloud_volume_light_tex, CLOUD_VOLUME_BLOCK_BINDING + 10)
CLOUD_VOLUME_TEXTURES(cloud_volume_max_tex, CLOUD_VOLUME_BLOCK_BINDING + 19)
CLOUD_VOLUME_TEXTURES(cloud_volume_distance_tex, CLOUD_VOLUME_BLOCK_BINDING + 28)

#define CLOUD_VOLUME_PICK(name, index, call) \
	switch (index) { \
		case 0: return call(name##0); case 1: return call(name##1); case 2: return call(name##2); \
		case 3: return call(name##3); case 4: return call(name##4); case 5: return call(name##5); \
		case 6: return call(name##6); case 7: return call(name##7); default: return call(name##8); \
	}

// Texture `index` (a level, or CLOUD_VOLUME_SPARE) of level k at a grid point.
vec4 cloud_volume_fetch_density(int index, int k, vec3 grid_m) {
	vec3 uvw = cloud_volume_uvw(index, k, grid_m);
#define CLOUD_VOLUME_SAMPLE(t) texture(t, uvw) / CLOUD_VOLUME_PER_KM
	CLOUD_VOLUME_PICK(cloud_volume_density_tex, index, CLOUD_VOLUME_SAMPLE)
#undef CLOUD_VOLUME_SAMPLE
}

vec4 cloud_volume_fetch_light(int index, int k, vec3 grid_m) {
	vec3 uvw = cloud_volume_uvw(index, k, grid_m);
#define CLOUD_VOLUME_SAMPLE(t) texture(t, uvw)
	CLOUD_VOLUME_PICK(cloud_volume_light_tex, index, CLOUD_VOLUME_SAMPLE)
#undef CLOUD_VOLUME_SAMPLE
}

// The mip texel (block) holding a grid point of level k in texture `index`.
ivec3 cloud_volume_block(int k, vec3 grid_m) {
	return ivec3(floor(cloud_volume_cells(k, grid_m) / float(CLOUD_VOLUME_MIP_DIV)));
}

float cloud_volume_fetch_max(int index, ivec3 texel) {
#define CLOUD_VOLUME_SAMPLE(t) texelFetch(t, texel, 0).r / CLOUD_VOLUME_PER_KM
	CLOUD_VOLUME_PICK(cloud_volume_max_tex, index, CLOUD_VOLUME_SAMPLE)
#undef CLOUD_VOLUME_SAMPLE
}

float cloud_volume_fetch_distance(int index, ivec3 texel) {
#define CLOUD_VOLUME_SAMPLE(t) round(texelFetch(t, texel, 0).r * CLOUD_VOLUME_DISTANCE_SCALE)
	CLOUD_VOLUME_PICK(cloud_volume_distance_tex, index, CLOUD_VOLUME_SAMPLE)
#undef CLOUD_VOLUME_SAMPLE
}

// (rho, sigma_c) [1/m] at a grid point of `level` (cloud_volume_level): both snapshots blended by cloud_blend, faded
// toward level + 1 by 1 - fade.
vec2 cloud_volume_density(vec3 grid_m, int level, float fade) {
	vec4 v = cloud_volume_fetch_density(level, level, grid_m);
	if (fade < 1.0 && level + 1 < CLOUD_VOLUME_LEVELS) {
		v = mix(cloud_volume_fetch_density(level + 1, level + 1, grid_m), v, fade);
	}
	return vec2(mix(v.x, v.z, cloud_blend), mix(v.y, v.w, cloud_blend));
}

// The light terms (tau_sun, tau_below, tau_above, tau_moon; Task 6) at a grid point of `level`, faded as the density.
vec4 cloud_volume_light(vec3 grid_m, int level, float fade) {
	vec4 v = cloud_volume_fetch_light(level, level, grid_m);
	if (fade < 1.0 && level + 1 < CLOUD_VOLUME_LEVELS) {
		v = mix(cloud_volume_fetch_light(level + 1, level + 1, grid_m), v, fade);
	}
	return v;
}

// The max-extinction bound [1/m] of the block holding the grid point: no sample of the level's density there (any
// blend, the trilinear filter, the sub-cell field up to sigma_c) exceeds it (CLOUD_VOLUME_OCCUPANCY).
float cloud_volume_max_extinction(vec3 grid_m, int level) {
	return cloud_volume_fetch_max(level, cloud_volume_wrap(cloud_volume_block(level, grid_m), CLOUD_VOLUME_MIP_SIZE));
}

// Free space (true metres) around the grid point: the box of blocks closer than the distance field's value (Chebyshev,
// blocks) holds no density, clipped to the level's window (beyond it the texels hold other cells), shrunk by the grid's
// largest scale. 0 in an occupied block.
float cloud_volume_free_m(vec3 grid_m, int level) {
	vec3 cells = cloud_volume_cells(level, grid_m);
	ivec3 block = ivec3(floor(cells / float(CLOUD_VOLUME_MIP_DIV)));
	float d = cloud_volume_fetch_distance(level, cloud_volume_wrap(block, CLOUD_VOLUME_MIP_SIZE));
	if (d < 1.0) {
		return 0.0;
	}
	vec3 lo = (vec3(block) - (d - 1.0)) * float(CLOUD_VOLUME_MIP_DIV);
	vec3 hi = (vec3(block) + d) * float(CLOUD_VOLUME_MIP_DIV);
	vec3 o = vec3(cloud_volume.levels[level].xyz);
	lo = max(lo, o);
	hi = min(hi, o + vec3(CLOUD_VOLUME_SIZE));
	vec3 m = max(min(cells - lo, hi - cells), vec3(0.0))
			* vec3(CLOUD_VOLUME_CELL_H_M[level], CLOUD_VOLUME_CELL_H_M[level], CLOUD_VOLUME_CELL_V_M[level]);
	return min(min(m.x, m.y), m.z) / CLOUD_VOLUME_MAX_SCALE;
}
#endif
