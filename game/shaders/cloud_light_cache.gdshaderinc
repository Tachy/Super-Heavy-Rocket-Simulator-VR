// The light cache (static clouds Tasks 1-2): the optical depth toward the Sun tau_light (the quantity
// cloud_light_optical_depth returns) at the voxel centres of camera-anchored, Earth-fixed cascades over the cloud
// shell, filled progressively by the CLOUD_LIGHT_FILL kernel (CloudLightCache, scheduled by CloudLightCacheSchedule)
// and only looked up while marching (cloud_pass.gdshaderinc cloud_light_source): the light a cloud sample receives
// depends on the Sun, the weather field and the point - not on the camera. GLSL 450 through CloudShaderPort (its
// `global uniform`s in set 0); include atmosphere.gdshaderinc and clouds.gdshaderinc first.
//
// The frame: the tangent plane at cloud_light_cache_origin (east / north axes; Earth-fixed, a ground point). A point
// maps to its plane coordinates (offset from the origin along east, along north) and its height (the march's model
// height, cloud_light_cache_shell.xy: the shell). Cascade k's voxel (i, j, layer) holds the point at plane coordinates
// ((i + 0.5) dx, (j + 0.5) dx) whose height is the layer centre's (the fill solves for that height along the plane's
// up, so fill and lookup agree exactly at the voxel centres). Global indices (i, j) are Earth-fixed; a cascade stores
// its window toroidally at texel (i mod N, j mod N) - the repeat sampler interpolates across the wrap - in its own
// texture per generation (cloud_light_cache_a<k>: slot 0, _b<k>: slot 1). Five cascades, x2 apart (8 .. 128 km).
// Weights: inside cascade k's camera square (half size radius around the ground point) weight 1, falling linearly to 0
// over `band` voxels at its edge and at the edge of the voxel centres its slots hold; the remaining weight goes to the
// next cascade, after the last to the per-sample march (no seam). Within a cascade the two generations cross-fade by
// the slot weight. Voxel values belong to voxel centres; hardware trilinear between them.
// The fill marches toward the light with a converged, deterministic sample set (CloudLightCache.SAMPLES near + far,
// the stratified midpoints: jitter 0.5), the detail band-limited at least at the voxel's horizontal size (finer
// octaves cannot be held by the voxels; the tables keep their mean), the field mixed at the generation's blend.

// ---- Globals (CloudLayer: the fidelity flag; CloudEffect: the rest, from the cache's state on the render thread).
// tau_light per cascade k and generation slot (a: 0, b: 1).
global uniform sampler3D cloud_light_cache_a0 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_b0 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_a1 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_b1 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_a2 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_b2 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_a3 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_b3 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_a4 : filter_linear, repeat_enable;
global uniform sampler3D cloud_light_cache_b4 : filter_linear, repeat_enable;
global uniform bool cloud_light_cache_enabled;  // fidelity flag (false: the light march everywhere, the reference)
global uniform bool cloud_light_cache_valid;    // some cascade holds a filled region for this frame
global uniform vec3 cloud_light_cache_origin;   // the frame's origin - the camera (Earth-local axes, m; float64 on CPU)
global uniform vec3 cloud_light_cache_east;     // the frame's horizontal axes (unit, Earth-local)
global uniform vec3 cloud_light_cache_north;
global uniform vec4 cloud_light_cache_shell;    // (bottom height m, top height m, texels across N, unused)
// The hybrid cascades' near march: (first interval m, ratio, length m, samples).
global uniform vec4 cloud_light_cache_near;
// Per cascade (columns): [0] (dx m, 1: hybrid / 0: pure, layers, weight of slot 1); [1] / [2] the voxels slot 0 / 1
// holds (x0, y0, x1, y1: global indices, end exclusive; empty: x1 <= x0); [3] (the ground point x, y in voxel units,
// the camera square's half size, the band; voxels).
global uniform mat4 cloud_light_cache_cascade0;
global uniform mat4 cloud_light_cache_cascade1;
global uniform mat4 cloud_light_cache_cascade2;
global uniform mat4 cloud_light_cache_cascade3;
global uniform mat4 cloud_light_cache_cascade4;

// Weight 0 .. 1 of a held rectangle at global voxel coordinate g: 0 at the edge of its voxel centres' range, 1 band
// voxels inside it (0 for an empty rectangle).
float cloud_cache_rect_weight(vec4 held, vec2 g, float band) {
	vec2 lo = g - (held.xy + 0.5);
	vec2 hi = (held.zw - 0.5) - g;
	return clamp(min(min(lo.x, lo.y), min(hi.x, hi.y)) / band, 0.0, 1.0);
}

// The march's model height (cloud_height_along's form) of the point camera + offset.
float cloud_offset_height(vec3 offset) {
	vec3 m = offset * ATM_STRETCH;
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	float r_c = CLOUD_A_M + cloud_camera_height_m;
	float q = dot(m, m) + 2.0 * r_c * dot(up_c, m);
	return cloud_camera_height_m + q / (sqrt(max(r_c * r_c + q, 0.0)) + r_c);
}

// Adds cascade c's share to result (x: the weighted cached tau_light so far, y: the weight of the hybrid cascades
// whose near part the caller marches, z: the weight still open). The cascade is chosen at the sample (its plane
// coordinates `plane`, m); a pure cascade (c[0].y = 0) is read at the sample (height fraction hf over the shell), a
// hybrid one (c[0].y = 1) at the point the near march ends (plane_near, hf_near) - and only where it holds that point.
void cloud_cache_cascade(mat4 c, sampler3D slot0, sampler3D slot1, vec2 plane, float hf, vec2 plane_near,
		float hf_near, inout vec3 result) {
	vec2 g = plane / c[0].x;  // global voxel coordinates (voxel i spans [i, i + 1))
	vec2 d = abs(g - c[3].xy);
	float e = clamp((c[3].z - max(d.x, d.y)) / c[3].w, 0.0, 1.0);
	bool hybrid = c[0].y > 0.5;
	vec2 gf = hybrid ? plane_near / c[0].x : g;  // where the cascade is read
	float w1 = c[0].w;
	if (w1 < 1.0) {
		e = min(e, cloud_cache_rect_weight(c[1], gf, c[3].w));
	}
	if (w1 > 0.0) {
		e = min(e, cloud_cache_rect_weight(c[2], gf, c[3].w));
	}
	if (e <= 0.0) {
		return;
	}
	float layers = c[0].z;
	vec3 f = vec3(fract(gf / cloud_light_cache_shell.z),
			clamp((hybrid ? hf_near : hf) * layers, 0.5, layers - 0.5) / layers);
	float tau = w1 < 1.0 ? textureLod(slot0, f, 0.0).r : 0.0;
	if (w1 > 0.0) {
		tau = mix(tau, textureLod(slot1, f, 0.0).r, w1);
	}
	float w = result.z * e;
	result.x += w * tau;
	result.y += hybrid ? w : 0.0;
	result.z -= w;
}

// tau_light toward the Sun (light_dir) at camera + offset_local with its height from the cascades: x the weighted
// cached optical depth, y the weight of the hybrid cascades (their tau starts cloud_light_cache_near.z sunward: the
// caller adds y x the near march over that length through the shapes), z the weight left for the per-sample march
// (1: the cache holds nothing here - off, not filled, outside every cascade - and the caller's result is the march's;
// 0: the cache alone). tau_light = x + y * near march + z * march.
vec3 cloud_cached_light_depth(vec3 offset_local, float height_m, vec3 light_dir) {
	vec3 result = vec3(0.0, 0.0, 1.0);
	if (!cloud_light_cache_enabled || !cloud_light_cache_valid) {
		return result;
	}
	vec3 rel = offset_local - cloud_light_cache_origin;
	vec2 plane = vec2(dot(rel, cloud_light_cache_east), dot(rel, cloud_light_cache_north));
	float span = cloud_light_cache_shell.y - cloud_light_cache_shell.x;
	float hf = (height_m - cloud_light_cache_shell.x) / span;
	vec3 near_offset = offset_local + light_dir * cloud_light_cache_near.z;
	vec3 rel_near = rel + light_dir * cloud_light_cache_near.z;
	vec2 plane_near = vec2(dot(rel_near, cloud_light_cache_east), dot(rel_near, cloud_light_cache_north));
	float hf_near = (cloud_offset_height(near_offset) - cloud_light_cache_shell.x) / span;
	cloud_cache_cascade(cloud_light_cache_cascade0, cloud_light_cache_a0, cloud_light_cache_b0, plane, hf, plane_near,
			hf_near, result);
	if (result.z > 0.0) {
		cloud_cache_cascade(cloud_light_cache_cascade1, cloud_light_cache_a1, cloud_light_cache_b1, plane, hf,
				plane_near, hf_near, result);
	}
	if (result.z > 0.0) {
		cloud_cache_cascade(cloud_light_cache_cascade2, cloud_light_cache_a2, cloud_light_cache_b2, plane, hf,
				plane_near, hf_near, result);
	}
	if (result.z > 0.0) {
		cloud_cache_cascade(cloud_light_cache_cascade3, cloud_light_cache_a3, cloud_light_cache_b3, plane, hf,
				plane_near, hf_near, result);
	}
	if (result.z > 0.0) {
		cloud_cache_cascade(cloud_light_cache_cascade4, cloud_light_cache_a4, cloud_light_cache_b4, plane, hf,
				plane_near, hf_near, result);
	}
	return result;
}

// The near part of a hybrid cascade's light path: from the sample cloud_light_cache_near.z toward the light through
// the shapes in cloud_light_cache_near.w geometric intervals (first .x, ratio .y), the stratified midpoints (jitter
// 0.5: deterministic, as the fill); the detail band-limited at the pixel footprint as the per-sample march.
float cloud_near_light_depth(vec3 offset_local, float height_m, float camera_distance_m, vec3 light_dir,
		float pixel_angle) {
	return cloud_light_optical_depth_with(offset_local, height_m, camera_distance_m, light_dir, 0.5, pixel_angle,
			pixel_angle, vec4(cloud_light_cache_near.xyz, 0.0), ivec2(int(cloud_light_cache_near.w), 0), 0.0);
}

// =====================================================================================================================
#ifdef CLOUD_LIGHT_FILL

layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;

// One fill job (CloudLightCache; Earth-local axes, m).
layout(set = 1, binding = 0, std140) uniform CloudLightFill {
	vec4 origin;   // xyz: the frame's origin - the camera; w: the detail's footprint floor (the voxel's size across)
	vec4 east;     // xyz: unit; w: dx (m)
	vec4 north;    // xyz: unit
	vec4 up;       // xyz: the frame's up (the geodetic up at the origin)
	vec4 shell;    // (bottom height m, top height m, the cascade's layers, unused)
	ivec4 region;  // the job's first voxel (i, j: global indices; layer), w: texels across N
	ivec4 count;   // the job's voxels (x, y, layers)
	vec4 light;    // xyz: unit direction toward the light
	vec4 march;    // the intervals (as cloud_light_march)
	ivec4 samples; // xy: near, far samples
} cloud_fill;

#ifdef CLOUD_LIGHT_FILL_R16F
layout(r16f, set = 1, binding = 1) uniform writeonly image3D out_light_depth;
#else
layout(r32f, set = 1, binding = 1) uniform writeonly image3D out_light_depth;
#endif

// Floor division (GLSL's % and / are undefined / truncating for negative operands); exact for |i| < 2^24.
int cloud_fill_floor_div(int i, int n) {
	return int(floor(float(i) / float(n)));
}

void main() {
	ivec3 id = ivec3(gl_GlobalInvocationID);
	if (any(greaterThanEqual(id, cloud_fill.count.xyz))) {
		return;
	}
	ivec3 voxel = cloud_fill.region.xyz + id;
	int n = cloud_fill.region.w;
	float dx = cloud_fill.east.w;
	float x = (float(voxel.x) + 0.5) * dx;
	float y = (float(voxel.y) + 0.5) * dx;
	float h = mix(cloud_fill.shell.x, cloud_fill.shell.y, (float(voxel.z) + 0.5) / cloud_fill.shell.z);
	vec3 base = cloud_fill.origin.xyz + cloud_fill.east.xyz * x + cloud_fill.north.xyz * y;
	// The point above the plane point whose model height is h: the sphere's drop as the start, then Newton steps on
	// the exact height (d height / d t = the plane's up against the local up, >= cos 9 deg within REFRAME_M; the
	// sphere vs the ellipsoid start error of < 0.2 km shrinks below a millimetre in three steps).
	float t = h + (x * x + y * y) / (2.0 * (CLOUD_A_M + h));
	for (int k = 0; k < 3; k++) {
		t += h - cloud_offset_height(base + cloud_fill.up.xyz * t);
	}
	vec3 offset = base + cloud_fill.up.xyz * t;
	float tau = cloud_light_optical_depth_with(offset, h, 0.0, cloud_fill.light.xyz, 0.5, 0.0, 0.0, cloud_fill.march,
			cloud_fill.samples.xy, cloud_fill.origin.w);
	ivec3 texel = ivec3(voxel.x - n * cloud_fill_floor_div(voxel.x, n), voxel.y - n * cloud_fill_floor_div(voxel.y, n),
			voxel.z);
	imageStore(out_light_depth, texel, vec4(tau, 0.0, 0.0, 0.0));
}

#endif
