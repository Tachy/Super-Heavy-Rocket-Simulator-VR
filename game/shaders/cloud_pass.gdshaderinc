// The cloud pass's compute kernels (M8 weather and clouds, Task 8): GLSL 450 only - compiled by CloudEffect
// (game/world/weather/cloud_effect.gd) through CloudShaderPort, never included by a Godot shader. One of CLOUD_MARCH,
// CLOUD_TEMPORAL, CLOUD_UPSAMPLE is defined per kernel; CLOUD_MARCH also gets atmosphere.gdshaderinc and
// clouds.gdshaderinc (translated: their `global uniform`s in set 0).
//
// Per view (each VR eye apart, the same kernels):
// 1. CLOUD_MARCH at reduced resolution (the quality preset's scale, 1/2): one ray per low-resolution pixel through the
//    centre of its block of full-resolution pixels; the scene depth of the block in a checkerboard: the nearest of the
//    block on even pixels, the farthest on odd ones (both depths are represented for the upsampling). The raymarch
//    (steps, light, compositing) as described in cloud_layer.gdshader's header. The start jitter is a blue-noise value
//    per low-resolution pixel (cloud_noise.hpp, 64 x 64 tiled) offset per frame by the golden ratio (a low-discrepancy
//    sequence in time): a function of the pixel and the frame alone - the same for both eyes. Out: premultiplied
//    colour + alpha, and (march distance used, transmittance-weighted cloud distance).
// 2. CLOUD_TEMPORAL: exponential accumulation with reprojection, per eye: the pixel's point at its cloud distance
//    (the far sky's 1e7 m where there is no cloud: rotation only) moved by the camera's motion (Earth-fixed, float64 on
//    the CPU) into the previous frame's clip space of the same eye; the history there (bilinear) is clamped to the
//    current 3 x 3 neighbourhood's min / max (no ghosting) and blended with the new value by the preset's weight; no
//    history (first frame, outside the previous view, after a reset): the new value alone.
// 3. CLOUD_UPSAMPLE to full resolution: the four nearest low-resolution pixels, bilinear weights x a depth weight
//    exp(-CLOUD_DEPTH_SHARPNESS |ln(D / D_i)|) (D the full-resolution scene distance, D_i the sample's march distance):
//    a cloud in front of a mountain does not bleed over it and the sky between peaks keeps its cloud. Out: the
//    full-resolution image the composite quad (cloud_layer.gdshader) blends over the scene.

#if defined(CLOUD_MARCH) || defined(CLOUD_TEMPORAL) || defined(CLOUD_UPSAMPLE)

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

// The view's parameters (CloudEffect): matrices of this frame and the previous frame's clip transform of the same eye.
layout(set = 1, binding = 0, std140) uniform CloudView {
	mat4 inv_projection;       // clip (the renderer's: reversed Z in [0, 1], 1 = near; y down) -> view space
	mat4 view_to_world;        // the eye's transform (render world; the floating origin is the camera)
	mat4 world_to_earth;       // render world axes -> Earth-local axes (rotation)
	mat4 prev_clip_from_earth; // previous frame: Earth-local offset from the previous camera -> this eye's clip
	vec4 sizes;                // full width, full height, low width, low height
	vec4 camera_delta;         // xyz: camera now - camera before (Earth-local, m); w: 1 = history valid
	vec4 params;               // x: frame jitter offset [0, 1), y: temporal weight, z: debug output, w: unused
} cloud_view;

// Distance [m] from the camera of a full-resolution pixel's depth (reversed Z: 1 near, 0 far = no geometry).
float cloud_scene_distance(vec2 pixel_centre, float depth) {
	if (depth <= 0.0) {
		return 1.0e30;
	}
	vec2 ndc = pixel_centre / cloud_view.sizes.xy * 2.0 - 1.0;  // the renderer's projection: y down (flipped)
	vec4 v = cloud_view.inv_projection * vec4(ndc, depth, 1.0);
	return length(v.xyz / v.w);
}

// Earth-local unit direction of the ray through a full-resolution position.
vec3 cloud_ray_earth(vec2 pixel_centre) {
	vec2 ndc = pixel_centre / cloud_view.sizes.xy * 2.0 - 1.0;  // the renderer's projection: y down (flipped)
	vec4 v = cloud_view.inv_projection * vec4(ndc, 0.5, 1.0);  // in front of the camera in both depth conventions
	vec3 dir_view = normalize(v.xyz / v.w);
	vec3 dir_world = normalize(mat3(cloud_view.view_to_world) * dir_view);
	return normalize(mat3(cloud_view.world_to_earth) * dir_world);
}

#endif

// =====================================================================================================================
#ifdef CLOUD_MARCH

layout(set = 1, binding = 1) uniform sampler2D scene_depth;  // full resolution, nearest
layout(rgba16f, set = 1, binding = 2) uniform writeonly image2D out_color;
layout(rg32f, set = 1, binding = 3) uniform writeonly image2D out_depth;

// [E] Path toward the light from an in-cloud sample in the near (shape) part (cloud_light_march.z: the preset).
// [E] The coarse and fine steps are fractions of the distance from this distance on (nearer: CLOUD_MIN_STEP_M).
const float CLOUD_NEAR_START_M = 1000.0;
// [E] Smallest step (a camera inside a cloud: the mean free path of a thick cumulus is ~15 m).
const float CLOUD_MIN_STEP_M = 10.0;
// Lower end of the shell: below every ground (the Dead Sea shore -430 m); the field holds layer 0 below 0 m.
const float CLOUD_BOTTOM_M = -500.0;
const float CLOUD_MIN_SCALED_TRANSMITTANCE = 0.01;
// [E] The air's light at the cloud is re-evaluated along a ray after 500 m or 5 % of the distance (the transmitted
// Sun changes by < 1 % over that at a 10-deg Sun; the eclipse's shadow edge is ~50 km wide).
const float CLOUD_LIGHT_REFRESH_M = 500.0;
const float CLOUD_LIGHT_REFRESH_FRACTION = 0.05;

global uniform sampler2D cloud_blue_noise : filter_nearest, repeat_enable;  // 64 x 64 R8 (cloud_noise.hpp)
global uniform int cloud_steps;        // coarse steps spanning the segment (quality preset)
global uniform int cloud_max_steps;    // cap on the steps per pixel (coarse + fine)
global uniform float cloud_fine_step_fraction;  // fine step / distance inside the field's cloudy region

// Scattering source of one light (Sun or Moon) at a sample: J per unit irradiance normal to the light.
float cloud_light_source(vec3 light_dir, vec3 normal, vec3 dir, vec3 offset, float height_m, float distance_m,
		vec2 column, float jitter) {
	float tau_light = cloud_light_optical_depth(offset, height_m, distance_m, light_dir, jitter, true);
	return cloud_scattering_source_3d(light_dir, -dir, normal, tau_light, column.x, column.y);
}

// The raymarch of one ray (dir: Earth-local unit) up to scene_m; out: premultiplied colour, alpha, weighted distance.
void cloud_march(vec3 dir, float scene_m, float jitter, out vec3 color, out float alpha, out float cloud_m) {
	color = vec3(0.0);
	alpha = 0.0;
	cloud_m = 0.0;
	CloudRay ray = cloud_ray(dir);
	if (cloud_debug_output == 3) {
		vec2 crossing = cloud_sphere_crossing(ray, cloud_debug_height_m);
		float d_h = crossing.x > 0.0 ? crossing.x : crossing.y;
		if (crossing.x > crossing.y || d_h <= 0.0) {
			return;
		}
		vec3 offset_h = dir * d_h;
		vec2 lat_lon_h = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_h);
		CloudCell cell = cloud_field(lat_lon_h, cloud_debug_height_m);
		color = vec3(cloud_coverage(cloud_shape_coordinates(offset_h, cloud_debug_height_m), cell.fraction));
		alpha = 1.0;
		return;
	}
	vec2 top = cloud_sphere_crossing(ray, CLOUD_TOP_M);
	if (top.x > top.y || top.y <= 0.0) {
		return;
	}
	float d0 = max(top.x, 0.0);
	float d1 = top.y;
	vec2 bottom = cloud_sphere_crossing(ray, CLOUD_BOTTOM_M);
	if (bottom.x <= bottom.y) {
		if (cloud_camera_height_m >= CLOUD_BOTTOM_M) {
			if (bottom.x > 0.0) {
				d1 = min(d1, bottom.x);
			}
		} else {
			d0 = max(d0, bottom.y);
		}
	}
	d1 = min(d1, scene_m);
	if (d1 <= d0) {
		return;
	}

	float k = cloud_transport_scale();
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	vec3 moon_dir = normalize(atmosphere_moon_direction);
	bool moonlit = atm_moonlit();
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float start = max(d0, CLOUD_NEAR_START_M);
	int n = max(cloud_steps, 1);
	// Coarse step: the fraction of the distance that spans [start, d1] in n steps; at most one height layer.
	float coarse_rel = d1 > start ? exp(log(d1 / start) / float(n)) - 1.0 : 1.0;
	float transmittance_bg = 1.0;   // exp(-tau): the background
	float transmittance_sc = 1.0;   // exp(-(1 - f) tau): the scattered light
	vec3 radiance = vec3(0.0);
	float weighted_distance = 0.0;
	float light_at_m = -1.0e30;
	vec3 sun_irradiance = vec3(0.0);
	vec3 sky_irradiance = vec3(0.0);
	vec3 moon_irradiance = vec3(0.0);
	vec3 moon_sky_irradiance = vec3(0.0);
	float a = d0;
	for (int i = 0; i < cloud_max_steps && a < d1; i++) {
		float h_a = cloud_height_along(ray, a);
		float climb = abs(cloud_height_slope(ray, a));
		float layer_m = h_a < CLOUD_FINE_TOP_M ? CLOUD_FINE_LAYER_M : CLOUD_COARSE_LAYER_M;
		float step_m = min(max(a, start) * coarse_rel, layer_m / max(climb, 1.0e-3));
		step_m = max(step_m, CLOUD_MIN_STEP_M);
		// Inside the field's cloudy region the step shrinks to the fine fraction of the distance: the sub-grid shapes
		// are resolved (a coarse step would hit or miss a cumulus edge per pixel: grain).
		float d = a + jitter * step_m;
		vec2 lat_lon = cloud_lat_lon(camera + dir * d);
		CloudCell cell = cloud_field(lat_lon, cloud_height_along(ray, d));
		if (cell.extinction > 0.0) {
			// Never finer than what still reaches the segment's end within the remaining steps (no far cloud is cut).
			float remaining = float(max(cloud_max_steps - i, 1));
			float reach = max(a, start) * (exp(log(max(d1, start) / max(a, start)) / remaining) - 1.0);
			float fine = max(max(max(a, start) * cloud_fine_step_fraction, reach), CLOUD_MIN_STEP_M);
			if (fine < step_m) {
				step_m = fine;
				d = a + jitter * step_m;
				lat_lon = cloud_lat_lon(camera + dir * d);
				cell = cloud_field(lat_lon, cloud_height_along(ray, d));
			}
		}
		float b = min(a + step_m, d1);
		d = min(d, b);
		float a_this = a;
		a = b;
		if (cell.extinction <= 0.0) {
			continue;
		}
		vec3 offset = dir * d;
		float height = cloud_height_along(ray, d);
		float in_cloud;
		float beta = cloud_extinction_in_cell(cell, offset, height, d, cloud_debug_output == 0 || cloud_debug_output == 4,
				in_cloud);
		if (beta <= 0.0) {
			continue;
		}
		float tau = beta * (b - a_this);
		vec3 p = camera + offset;
		vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);  // the ellipsoid's normal (geodetic up)
		vec2 column = cloud_column_depths(cell, lat_lon, height, in_cloud);
		// The light march's jitter decorrelated from step to step (golden-ratio sequence): the samples of one ray
		// average its noise instead of sharing one light-march error.
		float light_jitter = fract(jitter + float(i) * 0.6180339887);
		vec3 source;
		if (cloud_debug_output == 2) {
			source = vec3(cloud_light_source(sun_dir, normal, dir, offset, height, d, column, light_jitter));
		} else {
			// The air's light at the cloud (Sun and sky irradiance), refreshed along the ray every
			// CLOUD_LIGHT_REFRESH_FRACTION of the distance (at least CLOUD_LIGHT_REFRESH_M): it changes over kilometres.
			if (abs(d - light_at_m) > max(CLOUD_LIGHT_REFRESH_M, CLOUD_LIGHT_REFRESH_FRACTION * d)) {
				light_at_m = d;
				sun_irradiance = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
				sky_irradiance = ATM_PI * atm_sky_light(p, normal);
				if (moonlit) {
					moon_irradiance = ATM_PI * atm_moon_at(p);
					moon_sky_irradiance = ATM_PI * atm_moon_sky_light(p, normal);
				}
			}
			float sky_source = cloud_sky_scattering_source(-dot(dir, normal), column.x, column.y);
			source = sky_irradiance * sky_source;
			if (dot(sun_irradiance, sun_irradiance) > 0.0) {
				source += sun_irradiance * cloud_light_source(sun_dir, normal, dir, offset, height, d, column, light_jitter);
			}
			if (moonlit) {
				source += moon_irradiance * cloud_light_source(moon_dir, normal, dir, offset, height, d, column, light_jitter)
						+ moon_sky_irradiance * sky_source;
			}
		}
		float scaled_step = exp(-k * tau);
		radiance += transmittance_sc * source * (1.0 - scaled_step) / k;
		transmittance_sc *= scaled_step;
		float next_bg = transmittance_bg * exp(-tau);
		weighted_distance += d * (transmittance_bg - next_bg);
		transmittance_bg = next_bg;
		if (transmittance_sc < CLOUD_MIN_SCALED_TRANSMITTANCE) {
			break;
		}
	}

	if (cloud_debug_output == 1 || cloud_debug_output == 4) {
		color = vec3(transmittance_bg);
		alpha = 1.0;
		return;
	}
	if (cloud_debug_output == 2) {
		color = radiance;
		alpha = 1.0;
		return;
	}
	alpha = 1.0 - transmittance_bg;
	if (alpha <= 1.0e-6 && dot(radiance, radiance) <= 0.0) {
		alpha = 0.0;
		return;
	}
	vec3 air_transmittance = vec3(1.0);
	vec3 air_light = vec3(0.0);
	if (alpha > 1.0e-6) {
		cloud_m = weighted_distance / alpha;
		air_light = atm_to_point_offset(dir * cloud_m, air_transmittance);
	}
	color = air_transmittance * radiance + alpha * air_light;
}

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 low = ivec2(cloud_view.sizes.zw);
	if (pixel.x >= low.x || pixel.y >= low.y) {
		return;
	}
	ivec2 full = ivec2(cloud_view.sizes.xy);
	// The block of full-resolution pixels this pixel covers, and its centre.
	vec2 scale = cloud_view.sizes.xy / cloud_view.sizes.zw;
	ivec2 first = ivec2(vec2(pixel) * scale);
	ivec2 last = min(ivec2(vec2(pixel + 1) * scale) - 1, full - 1);
	vec2 centre = (vec2(pixel) + 0.5) * scale;
	bool nearest = ((pixel.x + pixel.y) & 1) == 0;
	float scene_m = nearest ? 1.0e30 : 0.0;
	for (int y = first.y; y <= last.y; y++) {
		for (int x = first.x; x <= last.x; x++) {
			float dist = cloud_scene_distance(vec2(x, y) + 0.5, texelFetch(scene_depth, ivec2(x, y), 0).r);
			scene_m = nearest ? min(scene_m, dist) : max(scene_m, dist);
		}
	}
	vec3 color = vec3(0.0);
	float alpha = 0.0;
	float cloud_m = 0.0;
	if (cloud_enabled && atmosphere_enabled) {
		float noise = texelFetch(cloud_blue_noise, pixel % 64, 0).r;
		float jitter = fract(noise + cloud_view.params.x);
		cloud_march(cloud_ray_earth(centre), scene_m, jitter, color, alpha, cloud_m);
	}
	// Half-float range (a lit cloud seen through a dark-adapted eye) and no NaN into the history.
	color = clamp(color, vec3(0.0), vec3(65000.0));
	if (any(isnan(color)) || isnan(alpha)) {
		color = vec3(0.0);
		alpha = 0.0;
	}
	imageStore(out_color, pixel, vec4(color, clamp(alpha, 0.0, 1.0)));
	imageStore(out_depth, pixel, vec4(scene_m, cloud_m, 0.0, 0.0));
}

#endif

// =====================================================================================================================
#ifdef CLOUD_TEMPORAL

layout(set = 1, binding = 1) uniform sampler2D current_color;  // nearest
layout(set = 1, binding = 2) uniform sampler2D current_depth;  // nearest
layout(set = 1, binding = 3) uniform sampler2D history_color;  // linear, clamp
layout(rgba16f, set = 1, binding = 4) uniform writeonly image2D out_history;

// Distance where a pixel without cloud is reprojected (the far sky: rotation only).
const float CLOUD_SKY_REPROJECT_M = 1.0e7;

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 low = ivec2(cloud_view.sizes.zw);
	if (pixel.x >= low.x || pixel.y >= low.y) {
		return;
	}
	vec4 current = texelFetch(current_color, pixel, 0);
	float weight = cloud_view.params.y;
	if (cloud_view.camera_delta.w < 0.5 || weight >= 1.0) {
		imageStore(out_history, pixel, current);
		return;
	}
	// The neighbourhood's range.
	vec4 lo = current;
	vec4 hi = current;
	for (int dy = -1; dy <= 1; dy++) {
		for (int dx = -1; dx <= 1; dx++) {
			vec4 c = texelFetch(current_color, clamp(pixel + ivec2(dx, dy), ivec2(0), low - 1), 0);
			lo = min(lo, c);
			hi = max(hi, c);
		}
	}
	// Reprojection of the pixel's point into the previous frame of this eye.
	vec2 scale = cloud_view.sizes.xy / cloud_view.sizes.zw;
	vec3 dir = cloud_ray_earth((vec2(pixel) + 0.5) * scale);
	float cloud_m = texelFetch(current_depth, pixel, 0).g;
	float distance_m = cloud_m > 0.0 ? cloud_m : CLOUD_SKY_REPROJECT_M;
	vec3 previous_offset = dir * distance_m + cloud_view.camera_delta.xyz;
	vec4 clip = cloud_view.prev_clip_from_earth * vec4(previous_offset, 1.0);
	vec4 result = current;
	if (clip.w > 0.0) {
		vec2 ndc = clip.xy / clip.w;
		vec2 uv = ndc * 0.5 + 0.5;
		if (all(greaterThanEqual(uv, vec2(0.0))) && all(lessThanEqual(uv, vec2(1.0)))) {
			vec4 history = clamp(texture(history_color, uv), lo, hi);
			result = mix(history, current, weight);
		}
	}
	imageStore(out_history, pixel, result);
}

#endif

// =====================================================================================================================
#ifdef CLOUD_UPSAMPLE

layout(set = 1, binding = 1) uniform sampler2D scene_depth;    // full resolution, nearest
layout(set = 1, binding = 2) uniform sampler2D low_color;      // the accumulated history, nearest
layout(set = 1, binding = 3) uniform sampler2D low_depth;      // the march distances, nearest
layout(rgba16f, set = 1, binding = 4) uniform writeonly image2D out_image;

// [E] Sharpness of the depth weight: a factor e^-1 per 1/4 of a e-fold distance ratio (|ln(D / D_i)| = 0.25: a
// sample 28 % nearer or farther weighs 37 %); equal distances (the sky, a far cloud deck over the sea) weigh 1.
const float CLOUD_DEPTH_SHARPNESS = 4.0;

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 full = ivec2(cloud_view.sizes.xy);
	if (pixel.x >= full.x || pixel.y >= full.y) {
		return;
	}
	ivec2 low = ivec2(cloud_view.sizes.zw);
	float scene_m = min(cloud_scene_distance(vec2(pixel) + 0.5, texelFetch(scene_depth, pixel, 0).r), 1.0e20);
	vec2 position = (vec2(pixel) + 0.5) * cloud_view.sizes.zw / cloud_view.sizes.xy - 0.5;
	ivec2 base = ivec2(floor(position));
	vec2 f = position - vec2(base);
	vec4 sum = vec4(0.0);
	float weights = 0.0;
	vec4 best = vec4(0.0);
	float best_error = 1.0e30;
	for (int j = 0; j <= 1; j++) {
		for (int i = 0; i <= 1; i++) {
			ivec2 q = clamp(base + ivec2(i, j), ivec2(0), low - 1);
			vec4 c = texelFetch(low_color, q, 0);
			float d = min(texelFetch(low_depth, q, 0).r, 1.0e20);
			float error = abs(log(max(scene_m, 1.0) / max(d, 1.0)));
			float w = (i == 0 ? 1.0 - f.x : f.x) * (j == 0 ? 1.0 - f.y : f.y) * exp(-CLOUD_DEPTH_SHARPNESS * error);
			sum += w * c;
			weights += w;
			if (error < best_error) {
				best_error = error;
				best = c;
			}
		}
	}
	imageStore(out_image, pixel, weights > 1.0e-4 ? sum / weights : best);
}

#endif
