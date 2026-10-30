// The baked volume's Sun transmittance S read back: the line toward the Sun through a point and its transmittance at
// a height - the march's air shadow, the shadow maps' fill and the Sun probe (cloud_baked.gdshaderinc).

#if defined(CLOUD_BAKED_BUILD) || defined(CLOUD_BAKED_MARCH) || defined(CLOUD_SHADOW_BAKED_FILL)
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
	int k = max(cloud_baked_level(si, gx, gy), int(baked.sets[si].misc.w));
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
