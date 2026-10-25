// The cloud shadow maps' compute kernel (M8 weather and clouds, Task 9): GLSL 450 only - compiled by CloudShadowEffect
// (game/world/weather/cloud_shadow_effect.gd) through CloudShaderPort after atmosphere.gdshaderinc and
// clouds.gdshaderinc, never included by a Godot shader. Lookup and frame: shaders/cloud_light.gdshaderinc.
//
// One texel = one line toward the Sun through the map plane's point A + u e1 + v e2 (A the anchor, offsets from the
// camera, Earth-local render axes), parameter t along the Sun direction s. The line's part inside the cloud shell
// (from where it leaves the sphere CLOUD_SHADOW_BOTTOM_M below the ellipsoid on the Sun's side - or enters the 20 km
// sphere if it misses it - to where it leaves the 20 km sphere) is marched twice: CLOUD_SHADOW_COARSE samples of the
// grid-mean field find the cloudy part (every 20 km / 64 = 312 m of height, as the layers' profile is >= 500 m wide
// around a cloudy 250 m centre), then CLOUD_SHADOW_FINE samples over it (one coarse step added on both sides) with the
// map's density: the shapes of the visible clouds (near map; detail band-limited at the larger of the texel and the
// step, the shapes' mean for steps beyond CLOUD_SHADOW_SHAPE_STEP_M; cover 1) or the grid mean (far map; cover = the
// beam's cover of the largest cloud fraction on the line, cloud_beam_cover_of). Midpoint rule.
// Out: (t of the first cloudy sample's start, t of the last one's end, optical depth, cover); a clear line (t_hi,
// t_hi, 0, 1).

#ifdef CLOUD_SHADOW_MAP

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(set = 1, binding = 0, std140) uniform CloudShadowBlock {
	vec4 anchor;  // xyz anchor - camera (m), w half extent (m)
	vec4 e1;      // xyz map axis, w 1: the shapes (near map), 0: the grid mean (far map)
	vec4 e2;      // xyz map axis, w the map's size (texels)
	vec4 sun;     // xyz the Sun direction (unit)
} shadow;

layout(rgba32f, set = 1, binding = 1) uniform writeonly image2D out_map;

const int CLOUD_SHADOW_COARSE = 64;
const int CLOUD_SHADOW_FINE = 64;
const float CLOUD_SHADOW_BOTTOM_M = -500.0;
// [E] A fine step longer than this sees the shapes' mean in part (fully at 4x): the shape noise's cells are ~250 m,
// point samples at kilometre steps would only add noise (the coverage tables keep the mean).
const float CLOUD_SHADOW_SHAPE_STEP_M = 500.0;

// The line's parameters where it crosses the sphere of height h (model space): x entry, y exit; x > y: none.
vec2 cloud_shadow_crossing(vec3 p_model, float p_height_m, vec3 s_model, float h_m) {
	float a2 = dot(s_model, s_model);
	float b = dot(p_model, s_model);
	float r = CLOUD_A_M + h_m;
	// |p|^2 - r^2 as (h_p - h)(|p| + r): no difference of two large squares.
	float c = (p_height_m - h_m) * (length(p_model) + r);
	float disc = b * b - a2 * c;
	if (disc < 0.0) {
		return vec2(1.0, -1.0);
	}
	float q = sqrt(disc);
	return vec2(-b - q, -b + q) / a2;
}

// x the extinction (shapes or grid mean), y the cover there: 1 for the shapes, else the beam's cover of the cell's
// fraction (cloud_beam_cover_of: the cell mean's shadow as the shapes cast it); the shapes' mean part (steps beyond
// CLOUD_SHADOW_SHAPE_STEP_M) mixes toward it. sun: the Sun direction (unit).
vec2 cloud_shadow_density(vec3 offset, vec3 sun, float step_m, float texel_m, bool shapes) {
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset;
	float h = cloud_height_of(p);
	if (h > CLOUD_TOP_M) {
		return vec2(0.0);
	}
	CloudCell cell = cloud_field(cloud_lat_lon(p), h);
	float mean = cell.extinction;
	float beam = cloud_beam_cover_of(cell.fraction, dot(cloud_geodetic_up(p), sun));
	if (!shapes || mean <= 0.0) {
		return vec2(mean, beam);
	}
	float w = smoothstep(CLOUD_SHADOW_SHAPE_STEP_M, 4.0 * CLOUD_SHADOW_SHAPE_STEP_M, step_m);
	if (w >= 1.0) {
		return vec2(mean, beam);  // the grid mean's own independent column (as the far map), not an overcast
	}
	float in_cloud;
	float shaped = cloud_extinction_in_cell(cell, offset, h, max(texel_m, step_m), in_cloud);
	return vec2(mix(shaped, mean, w), mix(1.0, beam, w));
}

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
	bool shapes = shadow.e1.w > 0.5;

	vec3 p_model = (atmosphere_camera_hi + atmosphere_camera_lo + origin) * ATM_STRETCH;
	float p_height = length(p_model) - CLOUD_A_M;
	vec3 s_model = s * ATM_STRETCH;
	vec2 top = cloud_shadow_crossing(p_model, p_height, s_model, CLOUD_TOP_M);
	if (top.x > top.y) {
		imageStore(out_map, texel, vec4(0.0, 0.0, 0.0, 1.0));
		return;
	}
	vec2 bottom = cloud_shadow_crossing(p_model, p_height, s_model, CLOUD_SHADOW_BOTTOM_M);
	float t_lo = bottom.x <= bottom.y ? max(bottom.y, top.x) : top.x;
	float t_hi = top.y;
	if (t_hi <= t_lo) {
		imageStore(out_map, texel, vec4(t_hi, t_hi, 0.0, 1.0));
		return;
	}

	// 1. The cloudy part of the line (grid mean).
	float dt = (t_hi - t_lo) / float(CLOUD_SHADOW_COARSE);
	int first = -1;
	int last = -1;
	for (int i = 0; i < CLOUD_SHADOW_COARSE; i++) {
		float t = t_lo + (float(i) + 0.5) * dt;
		if (cloud_shadow_density(origin + s * t, s, dt, texel_m, false).x > 0.0) {
			if (first < 0) {
				first = i;
			}
			last = i;
		}
	}
	if (first < 0) {
		imageStore(out_map, texel, vec4(t_hi, t_hi, 0.0, 1.0));
		return;
	}
	// 2. Its optical depth with the map's density.
	float a = t_lo + float(max(first - 1, 0)) * dt;
	float b = t_lo + float(min(last + 2, CLOUD_SHADOW_COARSE)) * dt;
	float step_m = (b - a) / float(CLOUD_SHADOW_FINE);
	float depth = 0.0;
	float cover = 0.0;  // the largest cover seen (the shapes: 1; the shapes' mean part: the cell fraction)
	float front = b;
	float back = a;
	for (int i = 0; i < CLOUD_SHADOW_FINE; i++) {
		float t = a + (float(i) + 0.5) * step_m;
		vec2 beta = cloud_shadow_density(origin + s * t, s, step_m, texel_m, shapes);
		if (beta.x > 0.0) {
			depth += beta.x * step_m;
			cover = max(cover, beta.y);
			front = min(front, t - 0.5 * step_m);
			back = t + 0.5 * step_m;
		}
	}
	if (depth <= 0.0) {
		imageStore(out_map, texel, vec4(t_hi, t_hi, 0.0, 1.0));
		return;
	}
	imageStore(out_map, texel, vec4(front, back, depth, cover));
}

#endif
