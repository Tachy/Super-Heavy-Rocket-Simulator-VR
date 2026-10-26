// The cloud pass's compute kernels (M8 weather and clouds, Task 8): GLSL 450 only - compiled by CloudEffect
// (game/world/weather/cloud_effect.gd) through CloudShaderPort, never included by a Godot shader. One of CLOUD_MARCH,
// CLOUD_TEMPORAL, CLOUD_UPSAMPLE is defined per kernel; CLOUD_MARCH also gets atmosphere.gdshaderinc and
// clouds.gdshaderinc (translated: their `global uniform`s in set 0).
//
// The march and the temporal accumulation run once per frame in the centre view (VR: the head, a frustum covering both
// eyes, CloudCentreView; the desktop: the view itself), the upsample once per eye (VR performance part 2: two marches
// with their own jitter and history showed different noise and blur per eye):
// 1. CLOUD_MARCH at reduced resolution (the quality preset's divisor): one ray per low-resolution pixel through the
//    centre of its block of full-resolution pixels; the scene depth of the block in a checkerboard: the nearest of the
//    block on even pixels, the farthest on odd ones (both depths are represented for the upsampling) - unless the
//    block's distances lie within CLOUD_ONE_SURFACE_SPREAD of each other (one surface: the ground seen from far, a
//    slope): then the mean, the centre ray's own distance (Task 10). The raymarch
//    (steps, light, compositing) as described in cloud_layer.gdshader's header. The start jitter is a blue-noise value
//    per low-resolution pixel (cloud_noise.hpp, 64 x 64 tiled) offset per frame by the golden ratio (a low-discrepancy
//    sequence in time): a function of the pixel and the frame alone - the same for both eyes. Out: premultiplied
//    colour + alpha, and (march distance used, transmittance-weighted cloud distance).
// 2. CLOUD_TEMPORAL: accumulation with reprojection, per eye: the pixel's point at its cloud distance (the far sky's
//    1e7 m where there is no cloud: rotation only) moved by the camera's motion (Earth-fixed, float64 on the CPU) into
//    the previous frame's clip space of the same eye; the history there (bilinear) is clamped to the current 3 x 3
//    neighbourhood's min / max (no ghosting) and blended with the new value by the weight 1 / n, n the frames the
//    history holds (+1, its age, kept per pixel beside it) down to the preset's floor weight: a running mean that
//    converges (an exponential mean of weight w keeps a noise floor of sqrt(w / (2 - w)) of one frame's noise: 23 % at
//    0.1), with the floor's memory at most. A history the clamp moved (the content changed) keeps at most
//    CLOUD_CLAMPED_AGE frames; no history (first frame, outside the previous view, after a reset): the new value alone.
// 3. CLOUD_UPSAMPLE to full resolution, per eye: the desktop's view (its own centre) takes the four nearest
//    low-resolution pixels, bilinear weights x a depth weight exp(-CLOUD_DEPTH_SHARPNESS |ln(D / D_i)|) (D the
//    full-resolution scene distance, D_i the sample's march distance): a cloud in front of a mountain does not bleed
//    over it and the sky between peaks keeps its cloud. A VR eye reprojects its ray into the centre image - at its
//    scene point, then at the taps' cloud distance (the disparity of the cloud) - and gathers a 4 x 4 tent (soft
//    alpha edges from the march's noise, the same in both eyes) with the same depth weight. Out: the full-resolution
//    image the composite quad (cloud_layer.gdshader) blends over the scene.

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
	vec4 params;               // x: frame jitter [0, 1), y: temporal weight, z: debug output, w: march pixel angle [rad]
	vec4 eye_earth;            // xyz: the eye's offset from the head (camera), Earth-local axes (m); 0 on the desktop;
	                           // w: the frame index mod 4096 (the sub-pixel shape offsets' sequence)
	vec4 temporal;             // x: the history's clip - 0: the neighbourhood's min / max; > 0: its mean +- x standard
	                           // deviations (VR: noise inside the range keeps the history; CloudLayer.PRESETS)
} cloud_view;

// Distance [m] from the camera (the head) of a pixel's depth (reversed Z: 1 near, 0 far = no geometry) through a
// projection's inverse and its image size.
float cloud_scene_distance_in(mat4 inv_projection, vec2 size, vec2 pixel_centre, float depth) {
	if (depth <= 0.0) {
		return 1.0e30;
	}
	vec2 ndc = pixel_centre / size * 2.0 - 1.0;  // the renderer's projection: y down (flipped)
	vec4 v = inv_projection * vec4(ndc, depth, 1.0);
	return length(v.xyz / v.w);
}

// Distance [m] from the camera of a full-resolution pixel's depth of this view.
float cloud_scene_distance(vec2 pixel_centre, float depth) {
	return cloud_scene_distance_in(cloud_view.inv_projection, cloud_view.sizes.xy, pixel_centre, depth);
}

// Head-space (view) unit direction of the ray through a full-resolution position.
vec3 cloud_ray_view(vec2 pixel_centre) {
	vec2 ndc = pixel_centre / cloud_view.sizes.xy * 2.0 - 1.0;  // the renderer's projection: y down (flipped)
	// The renderer's per-eye projection already contains the eye offset (inverse: clip -> head space), so one unprojected
	// point is not on the eye's ray. Two depths of the same pixel are: their difference is the eye's ray direction.
	vec4 a = cloud_view.inv_projection * vec4(ndc, 0.5, 1.0);
	vec4 b = cloud_view.inv_projection * vec4(ndc, 1.0, 1.0);
	return normalize(a.xyz / a.w - b.xyz / b.w);
}

// Earth-local unit direction of the ray through a full-resolution position.
vec3 cloud_ray_earth(vec2 pixel_centre) {
	vec3 dir_world = normalize(mat3(cloud_view.view_to_world) * cloud_ray_view(pixel_centre));
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
// [E] A block whose nearest and farthest scene distances differ by less than this (ln of their ratio) shows one
// surface: its centre ray is marched to the block's mean distance. The checkerboard's nearest depth cut that ray short
// by up to the block's depth extent - from 8 600 km a slanted 2 x 2 block of ground spans several km, so every other
// pixel lost the low clouds in front of the ground (Task 10). An occluder's edge (a mountain against the sky or the
// far terrain) spans far more and keeps the checkerboard. 0.05: the upsampling's depth weight
// exp(-CLOUD_DEPTH_SHARPNESS x 0.05) = 0.82 cannot tell such depths apart either.
const float CLOUD_ONE_SURFACE_SPREAD = 0.05;
// [E] Largest weight of one step in the pixel's scattered light, T_sc (1 - exp(-(1 - f) tau_step)): a heavier step is
// marched again with steps of this weight (one point sample - its light march included - would decide the pixel over
// the whole step: grain at cloud surfaces and at bases seen at a grazing angle; with steps of tau ~1 instead the pass
// cost 6 ms more at the descent's worst second). Behind T_sc < this weight any step is fine.
const float CLOUD_MAX_STEP_WEIGHT = 0.15;
// [E] The air's light at the cloud is re-evaluated along a ray after 500 m or 5 % of the distance (the transmitted
// Sun changes by < 1 % over that at a 10-deg Sun; the eclipse's shadow edge is ~50 km wide).
const float CLOUD_LIGHT_REFRESH_M = 500.0;
const float CLOUD_LIGHT_REFRESH_FRACTION = 0.05;
// [E] The eclipse's factor on the sky light (atm_sky_eclipse_factor: costly - evaluated at every light refresh it took
// the descent's p99 from 14.5 to 25 ms): once per 8 x 8 work group at two points of the group's centre ray, 5 km and
// 150 km away, linear in the distance between them (held beyond). The penumbra spans ~3500 km from first contact to
// totality (~3 % of obscuration per 100 km), the umbra ~200 km: neighbouring pixels share the factor.
const float CLOUD_ECLIPSE_NEAR_M = 5000.0;
const float CLOUD_ECLIPSE_FAR_M = 150000.0;
shared vec3 cloud_group_eclipse_near;
shared vec3 cloud_group_eclipse_far;

global uniform sampler2D cloud_blue_noise : filter_nearest, repeat_enable;  // 64 x 64 R8 (cloud_noise.hpp)
global uniform int cloud_steps;        // coarse steps spanning the segment (quality preset)
global uniform int cloud_max_steps;    // cap on the steps per pixel (coarse + fine)
global uniform float cloud_fine_step_fraction;  // fine step / distance inside the field's cloudy region

// Scattering source of one light (Sun or Moon) at a sample: J per unit irradiance normal to the light. The Sun's
// optical depth from the light cache where it holds the sample (cloud_light_cache.gdshaderinc), else - and always for
// the Moon - the light march (one call site: outside the cache the result is the march's, bit for bit).
float cloud_light_source(vec3 light_dir, vec3 normal, vec3 dir, vec3 offset, float height_m, float distance_m,
		vec2 column, float jitter, float pixel_angle, bool sun) {
	float tau_light = sun ? cloud_cached_light_depth(offset, height_m) : -1.0;
	if (tau_light < 0.0) {
		tau_light = cloud_light_optical_depth(offset, height_m, distance_m, light_dir, jitter, pixel_angle,
				pixel_angle);
	}
	if (cloud_scattering_model == 1) {
		return cloud_scattering_source_mc(light_dir, -dir, normal, tau_light, column.x, column.y);
	}
	return cloud_scattering_source_3d(light_dir, -dir, normal, tau_light, column.x, column.y);
}

// The raymarch of one ray (dir: Earth-local unit) up to scene_m; out: premultiplied colour, alpha, weighted distance.
void cloud_march(vec3 dir, float scene_m, float jitter, vec3 eclipse_near, vec3 eclipse_far, out vec3 color,
		out float alpha, out float cloud_m) {
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
		vec3 offset_h = cloud_view.eye_earth.xyz + dir * d_h;
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
	vec3 eye = cloud_view.eye_earth.xyz;  // each eye marches from its own position (heights: the head's, <= 3 cm apart)
	float pixel_angle = cloud_view.params.w;  // the detail's band limit: footprint = pixel_angle x distance
	float start = max(d0, CLOUD_NEAR_START_M);
	int n = max(cloud_steps, 1);
	// Coarse step: the fraction of the distance that spans [start, d1] in n steps; at most one height layer.
	float coarse_rel = d1 > start ? exp(log(d1 / start) / float(n)) - 1.0 : 1.0;
	float transmittance_bg = 1.0;   // exp(-tau): the background
	float transmittance_sc = 1.0;   // exp(-(1 - f) tau): the scattered light
	// exp(-w_D tau): the lights' scattered light with the transfer table (cloud_scattering_model 1; the table's source
	// leaves the phase function's HG peak in the view ray, core cloud_transfer_view_extinction)
	float transmittance_tab = 1.0;
	float w_tab = cloud_phase.w;
	vec3 radiance = vec3(0.0);
	float weighted_distance = 0.0;
	float light_at_m = -1.0e30;
	vec3 sun_irradiance = vec3(0.0);
	vec3 sky_irradiance = vec3(0.0);
	vec3 moon_irradiance = vec3(0.0);
	vec3 moon_sky_irradiance = vec3(0.0);
	float dense_step_m = 1.0e30;  // the step limit after a dense sample (CLOUD_MAX_STEP_WEIGHT)
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
		vec2 lat_lon = cloud_lat_lon(camera + eye + dir * d);
		CloudCell cell = cloud_field(lat_lon, cloud_height_along(ray, d));
		if (cell.extinction > 0.0) {
			// Never finer than what still reaches the segment's end within the remaining steps (no far cloud is cut).
			float remaining = float(max(cloud_max_steps - i, 1));
			float reach = max(a, start) * (exp(log(max(d1, start) / max(a, start)) / remaining) - 1.0);
			// The dense-cloud limit overrides the reach (behind an opaque surface little of the segment is seen).
			float fine = min(max(max(max(a, start) * cloud_fine_step_fraction, reach), CLOUD_MIN_STEP_M), dense_step_m);
			if (fine < step_m) {
				step_m = fine;
				d = a + jitter * step_m;
				lat_lon = cloud_lat_lon(camera + eye + dir * d);
				cell = cloud_field(lat_lon, cloud_height_along(ray, d));
			}
		}
		float b = min(a + step_m, d1);
		d = min(d, b);
		float a_this = a;
		a = b;
		if (cell.extinction <= 0.0) {
			dense_step_m = min(2.0 * dense_step_m, 1.0e30);
			continue;
		}
		vec3 offset = eye + dir * d;  // from the head: the ray starts at the eye
		float height = cloud_height_along(ray, d);
		float in_cloud;
		float beta = cloud_extinction_in_cell(cell, offset, height, pixel_angle * d, pixel_angle * d, in_cloud);
		if (beta <= 0.0) {
			dense_step_m = min(2.0 * dense_step_m, 1.0e30);  // clear air: the steps grow back
			continue;
		}
		// A step weighing more than CLOUD_MAX_STEP_WEIGHT in the pixel is done again with steps of that weight. The
		// step's intended length is compared, not b - a_this: far from the camera (a float32 distance of 8.6e6 m has a
		// 1 m ulp) the rounded difference exceeded the 0.1 % margin again and again - the same step was retried until
		// the step budget ran out and the ray never reached the cloud (Task 10: low decks seen from the scaled-space
		// limit, the horizon from orbit).
		float allowed = CLOUD_MAX_STEP_WEIGHT / transmittance_sc;
		float refined = allowed < 1.0 ? max(-log(1.0 - allowed) / (k * beta), CLOUD_MIN_STEP_M) : 1.0e30;
		if (min(step_m, d1 - a_this) > 1.001 * refined) {
			dense_step_m = refined;
			a = a_this;
			continue;
		}
		dense_step_m = max(refined, CLOUD_MIN_STEP_M);  // the limit at this density (grows as the pixel turns opaque)
		float tau = beta * (b - a_this);
		vec3 p = camera + offset;
		vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);  // the ellipsoid's normal (geodetic up)
		vec2 column = cloud_column_depths(cell, lat_lon, height, in_cloud);
		// The light march's jitter decorrelated from step to step (golden-ratio sequence): the samples of one ray
		// average its noise instead of sharing one light-march error.
		float light_jitter = fract(jitter + float(i) * 0.6180339887);
		vec3 source = vec3(0.0);        // the sky's (scaled problem: attenuated on the scaled depths)
		vec3 light_source = vec3(0.0);  // the Sun's and the Moon's (cloud_light_source)
		if (cloud_debug_output == 2 || cloud_debug_output == 7) {
			light_source = vec3(cloud_light_source(sun_dir, normal, dir, offset, height, d, column, light_jitter,
					pixel_angle, true));
		} else {
			// The air's light at the cloud (Sun and sky irradiance), refreshed along the ray every
			// CLOUD_LIGHT_REFRESH_FRACTION of the distance (at least CLOUD_LIGHT_REFRESH_M): it changes over kilometres.
			if (abs(d - light_at_m) > max(CLOUD_LIGHT_REFRESH_M, CLOUD_LIGHT_REFRESH_FRACTION * d)) {
				light_at_m = d;
				sun_irradiance = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
				float eclipse_x = clamp((d - CLOUD_ECLIPSE_NEAR_M) / (CLOUD_ECLIPSE_FAR_M - CLOUD_ECLIPSE_NEAR_M), 0.0,
						1.0);
				sky_irradiance = ATM_PI * atm_sky_light(p, normal) * mix(eclipse_near, eclipse_far, eclipse_x);
				if (moonlit) {
					moon_irradiance = ATM_PI * atm_moon_at(p);
					moon_sky_irradiance = ATM_PI * atm_moon_sky_light(p, normal);
				}
			}
			float sky_source = cloud_sky_scattering_source(-dot(dir, normal), column.x, column.y);
			source = sky_irradiance * sky_source;
			if (dot(sun_irradiance, sun_irradiance) > 0.0) {
				light_source += sun_irradiance
						* cloud_light_source(sun_dir, normal, dir, offset, height, d, column, light_jitter,
						pixel_angle, true);
			}
			if (moonlit) {
				light_source += moon_irradiance
						* cloud_light_source(moon_dir, normal, dir, offset, height, d, column, light_jitter,
						pixel_angle, false);
				source += moon_sky_irradiance * sky_source;
			}
		}
		float scaled_step = exp(-k * tau);
		float true_step = exp(-tau);
		if (cloud_scattering_model == 1) {
			// The lights' source with the table (exact single scattering + the tabulated Draine part of the higher
			// orders) is attenuated with exp(-w_D tau); the sky's stays in the scaled problem.
			float tab_step = exp(-w_tab * tau);
			radiance += transmittance_sc * source * (1.0 - scaled_step) / k
					+ transmittance_tab * light_source * (1.0 - tab_step) / w_tab;
			transmittance_tab *= tab_step;
		} else {
			radiance += transmittance_sc * (source + light_source) * (1.0 - scaled_step) / k;
		}
		transmittance_sc *= scaled_step;
		float next_bg = transmittance_bg * true_step;
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
	if (cloud_debug_output == 2 || cloud_debug_output == 7) {
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
		air_light = atm_to_point_offset(eye + dir * cloud_m, air_transmittance);
	}
	color = air_transmittance * radiance + alpha * air_light;
}

#ifdef CLOUD_CENTRE
// The centre view has no depth buffer of its own: its rays take the scene distance from an eye's (VR performance
// part 2).
layout(set = 1, binding = 4) uniform sampler2D scene_depth_right;  // `scene_depth` is the left eye's
layout(set = 1, binding = 5, std140) uniform CloudEyes {
	mat4 clip_from_head[2];  // the eyes' projections (the renderer's: eye offset and rotation inside)
	mat4 inv_projection[2];
	vec4 eye_sizes;          // xy: the eyes' full-resolution size
} cloud_eyes;

// [E] Where a centre ray is projected into an eye to find its depth texels: far, so the texel is the eye's look along
// the same direction; nearer geometry is then read with a parallax of at most IPD/2 / D (the distance from the head
// differs by at most IPD/2: cloud_vr_stereo_test), which the eyes' own upsample (their depth) absorbs at full
// resolution.
const float CLOUD_CENTRE_PROBE_M = 1.0e6;
// [E] Samples per axis of an eye's depth texels under one centre pixel (strided beyond).
const int CLOUD_CENTRE_GATHER_MAX = 6;

// The ndc of a head-space direction's far point in an eye; w <= 0 (behind the eye): far outside.
vec2 cloud_eye_ndc(int eye, vec3 dir_view) {
	vec4 c = cloud_eyes.clip_from_head[eye] * vec4(dir_view * CLOUD_CENTRE_PROBE_M, 1.0);
	return c.w > 0.0 ? c.xy / c.w : vec2(1.0e9);
}

// Nearest, farthest, sum and count of the scene distances under a low-resolution centre pixel: read in the eye whose
// ndc holds the pixel's centre ray (the left one first), over the eye texels its corners project to.
void cloud_block_distances(ivec2 pixel, vec2 scale, inout float near_m, inout float far_m, inout float sum_m,
		inout float count) {
	vec3 dir = cloud_ray_view(min((vec2(pixel) + 0.5) * scale, cloud_view.sizes.xy));
	int eye = all(lessThanEqual(abs(cloud_eye_ndc(0, dir)), vec2(1.0))) ? 0 : 1;
	vec2 size = cloud_eyes.eye_sizes.xy;
	vec2 lo = vec2(1.0e30);
	vec2 hi = vec2(-1.0e30);
	for (int j = 0; j <= 1; j++) {
		for (int i = 0; i <= 1; i++) {
			vec2 corner = min(vec2(pixel + ivec2(i, j)) * scale, cloud_view.sizes.xy);
			vec2 px = (clamp(cloud_eye_ndc(eye, cloud_ray_view(corner)), vec2(-1.0), vec2(1.0)) * 0.5 + 0.5) * size;
			lo = min(lo, px);
			hi = max(hi, px);
		}
	}
	ivec2 full = ivec2(size);
	ivec2 first = clamp(ivec2(floor(lo)), ivec2(0), full - 1);
	ivec2 last = clamp(ivec2(ceil(hi)) - 1, first, full - 1);
	ivec2 stride = max((last - first + CLOUD_CENTRE_GATHER_MAX) / CLOUD_CENTRE_GATHER_MAX, ivec2(1));
	for (int y = first.y; y <= last.y; y += stride.y) {
		for (int x = first.x; x <= last.x; x += stride.x) {
			float depth = eye == 0 ? texelFetch(scene_depth, ivec2(x, y), 0).r
					: texelFetch(scene_depth_right, ivec2(x, y), 0).r;
			float dist = cloud_scene_distance_in(cloud_eyes.inv_projection[eye], size, vec2(x, y) + 0.5, depth);
			near_m = min(near_m, dist);
			far_m = max(far_m, dist);
			sum_m += dist;
			count += 1.0;
		}
	}
}
#else
// Nearest, farthest, sum and count of the scene distances of the block of full-resolution pixels this pixel covers.
void cloud_block_distances(ivec2 pixel, vec2 scale, inout float near_m, inout float far_m, inout float sum_m,
		inout float count) {
	ivec2 full = ivec2(cloud_view.sizes.xy);
	ivec2 first = ivec2(vec2(pixel) * scale);
	ivec2 last = min(ivec2(vec2(pixel + 1) * scale) - 1, full - 1);
	for (int y = first.y; y <= last.y; y++) {
		for (int x = first.x; x <= last.x; x++) {
			float dist = cloud_scene_distance(vec2(x, y) + 0.5, texelFetch(scene_depth, ivec2(x, y), 0).r);
			near_m = min(near_m, dist);
			far_m = max(far_m, dist);
			sum_m += dist;
			count += 1.0;
		}
	}
}
#endif

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 low = ivec2(cloud_view.sizes.zw);
	vec2 scale = cloud_view.sizes.xy / cloud_view.sizes.zw;
	// The work group's eclipse factors on the sky light (CLOUD_ECLIPSE_NEAR_M / _FAR_M), before any invocation leaves.
	if (gl_LocalInvocationIndex == 0u) {
		vec3 near_factor = vec3(1.0);
		vec3 far_factor = vec3(1.0);
		if (cloud_enabled && atmosphere_enabled && atmosphere_eclipse_segments > 0) {
			vec2 group_centre = (vec2(gl_WorkGroupID.xy * gl_WorkGroupSize.xy) + 0.5 * vec2(gl_WorkGroupSize.xy)) * scale;
			vec3 group_dir = cloud_ray_earth(min(group_centre, cloud_view.sizes.xy - 0.5));
			vec3 camera_point = atmosphere_camera_hi + atmosphere_camera_lo + cloud_view.eye_earth.xyz;
			// Probe points above the ground: a ray that meets it is probed short of it.
			vec2 ground = cloud_sphere_crossing(cloud_ray(group_dir), 0.0);
			float limit_m = ground.x <= ground.y && ground.x > 0.0 ? ground.x : CLOUD_ECLIPSE_FAR_M;
			near_factor = atm_sky_eclipse_factor(camera_point + group_dir * min(CLOUD_ECLIPSE_NEAR_M, limit_m));
			far_factor = atm_sky_eclipse_factor(camera_point + group_dir * min(CLOUD_ECLIPSE_FAR_M, limit_m));
		}
		cloud_group_eclipse_near = near_factor;
		cloud_group_eclipse_far = far_factor;
	}
	barrier();
	if (pixel.x >= low.x || pixel.y >= low.y) {
		return;
	}
	vec2 centre = (vec2(pixel) + 0.5) * scale;
	bool nearest = ((pixel.x + pixel.y) & 1) == 0;
	float near_m = 1.0e30;
	float far_m = 0.0;
	float sum_m = 0.0;
	float count = 0.0;
	cloud_block_distances(pixel, scale, near_m, far_m, sum_m, count);
	float scene_m = nearest ? near_m : far_m;
	if (far_m < 1.0e29 && far_m <= near_m * exp(CLOUD_ONE_SURFACE_SPREAD)) {
		scene_m = sum_m / count;  // one surface: the centre ray's distance
	}
	vec3 color = vec3(0.0);
	float alpha = 0.0;
	float cloud_m = 0.0;
	if (cloud_enabled && atmosphere_enabled) {
		float noise = texelFetch(cloud_blue_noise, pixel % 64, 0).r;
		float jitter = fract(noise + cloud_view.params.x);
#ifdef CLOUD_SHAPE_PREFILTER
		// The sub-pixel shape offset (clouds.gdshaderinc cloud_subpixel_offset): blue noise at four shifted texels, moved
		// per frame along the R4 sequence (Roberts 2018: 1 / phi_4^k) of the frame index.
		float frame = cloud_view.eye_earth.w;
		cloud_shape_subpixel = fract(vec4(texelFetch(cloud_blue_noise, (pixel + ivec2(17, 41)) % 64, 0).r,
				texelFetch(cloud_blue_noise, (pixel + ivec2(43, 29)) % 64, 0).r,
				texelFetch(cloud_blue_noise, (pixel + ivec2(5, 53)) % 64, 0).r,
				texelFetch(cloud_blue_noise, (pixel + ivec2(31, 11)) % 64, 0).r)
				+ frame * vec4(0.8566748839, 0.7338918708, 0.6287067210, 0.5385972572));
#endif
		cloud_march(cloud_ray_earth(centre), scene_m, jitter, cloud_group_eclipse_near, cloud_group_eclipse_far, color,
				alpha, cloud_m);
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
layout(set = 1, binding = 5) uniform sampler2D history_age;    // nearest, clamp: frames the history holds
layout(r16f, set = 1, binding = 6) uniform writeonly image2D out_age;

// Distance where a pixel without cloud is reprojected (the far sky: rotation only).
const float CLOUD_SKY_REPROJECT_M = 1.0e7;
// [E] Frames a history keeps when the neighbourhood clamp moved it (changed content: a short memory, no ghost).
const float CLOUD_CLAMPED_AGE = 4.0;

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 low = ivec2(cloud_view.sizes.zw);
	if (pixel.x >= low.x || pixel.y >= low.y) {
		return;
	}
	vec4 current = texelFetch(current_color, pixel, 0);
	float weight_floor = cloud_view.params.y;
	if (cloud_view.camera_delta.w < 0.5 || weight_floor >= 1.0) {
		imageStore(out_history, pixel, current);
		imageStore(out_age, pixel, vec4(1.0));
		return;
	}
	// The neighbourhood's range: its min / max, or (clip > 0) its mean +- clip standard deviations (variance clipping,
	// Salvi 2016: the march's noise stays inside, a real change of the content leaves it).
	vec4 lo = current;
	vec4 hi = current;
	vec4 m1 = vec4(0.0);
	vec4 m2 = vec4(0.0);
	for (int dy = -1; dy <= 1; dy++) {
		for (int dx = -1; dx <= 1; dx++) {
			vec4 c = texelFetch(current_color, clamp(pixel + ivec2(dx, dy), ivec2(0), low - 1), 0);
			lo = min(lo, c);
			hi = max(hi, c);
			m1 += c;
			m2 += c * c;
		}
	}
	float clip_gamma = cloud_view.temporal.x;
	if (clip_gamma > 0.0) {
		m1 /= 9.0;
		vec4 sigma = sqrt(max(m2 / 9.0 - m1 * m1, vec4(0.0)));
		lo = m1 - clip_gamma * sigma;
		hi = m1 + clip_gamma * sigma;
	}
	// Reprojection of the pixel's point into the previous frame of this eye.
	vec2 scale = cloud_view.sizes.xy / cloud_view.sizes.zw;
	vec3 dir = cloud_ray_earth((vec2(pixel) + 0.5) * scale);
	float cloud_m = texelFetch(current_depth, pixel, 0).g;
	float distance_m = cloud_m > 0.0 ? cloud_m : CLOUD_SKY_REPROJECT_M;
	vec3 previous_offset = cloud_view.eye_earth.xyz + dir * distance_m + cloud_view.camera_delta.xyz;
	vec4 clip = cloud_view.prev_clip_from_earth * vec4(previous_offset, 1.0);
	vec4 result = current;
	float age = 1.0;
	if (clip.w > 0.0) {
		vec2 ndc = clip.xy / clip.w;
		vec2 uv = ndc * 0.5 + 0.5;
		if (all(greaterThanEqual(uv, vec2(0.0))) && all(lessThanEqual(uv, vec2(1.0)))) {
			vec4 history = texture(history_color, uv);
			vec4 clamped = clamp(history, lo, hi);
			float held = texture(history_age, uv).r;
			if (any(notEqual(clamped, history))) {
				held = min(held, CLOUD_CLAMPED_AGE);
			}
			age = min(held + 1.0, 1.0 / weight_floor);
			result = mix(clamped, current, 1.0 / age);
		}
	}
	imageStore(out_history, pixel, result);
	imageStore(out_age, pixel, vec4(age));
}

#endif

// =====================================================================================================================
#ifdef CLOUD_UPSAMPLE

layout(set = 1, binding = 1) uniform sampler2D scene_depth;    // full resolution, nearest
layout(set = 1, binding = 2) uniform sampler2D low_color;      // the accumulated history (of the centre view), nearest
layout(set = 1, binding = 3) uniform sampler2D low_depth;      // the march distances (of the centre view), nearest
layout(rgba16f, set = 1, binding = 4) uniform writeonly image2D out_image;
// The centre view the history belongs to (CloudEffect): its transform from Earth-local offsets of the head to its clip
// space, its sizes (virtual full, low) and params: x the mode (1 = an eye reads the centre (VR), 0 = the view is the
// centre: the desktop's own pixel grid, the 2 x 2 gather of before), y the centre pixel's angle [rad].
layout(set = 1, binding = 5, std140) uniform CloudCentre {
	mat4 clip_from_earth;
	vec4 sizes;
	vec4 params;
} cloud_centre;

// [E] Sharpness of the depth weight: a factor e^-1 per 1/4 of a e-fold distance ratio (|ln(D / D_i)| = 0.25: a
// sample 28 % nearer or farther weighs 37 %); equal distances (the sky, a far cloud deck over the sea) weigh 1.
const float CLOUD_DEPTH_SHARPNESS = 4.0;
// The eyes' tent gather (user 2026-10-26: the march's noise may stay but must end in soft alpha edges, not grain):
// weights max(0, 1 - |d| / R) per axis over the 4 x 4 nearest centre pixels. Display only: the history stays
// unfiltered (a filter fed back would blur more every frame). R spans CLOUD_EDGE_SOFT_M at the cloud's distance - a
// cloud's frayed edge is about as wide in metres near and far, so it narrows on screen with distance (a fixed screen
// radius blurred far clouds as much as near ones, user 2026-10-26) - between bilinear (1) and the soft maximum (2):
// with the VR centre's ~1.8 mrad pixels soft up to ~5.5 km, bilinear beyond ~11 km.
// [E] Width of a cloud's soft edge [m] (the frayed rim of a cumulus; cloud_vr_stereo_test mirrors these three).
const float CLOUD_EDGE_SOFT_M = 20.0;
const float CLOUD_TENT_RADIUS_MIN = 1.0;
const float CLOUD_TENT_RADIUS_MAX = 2.0;
// Distance where an eye pixel without scene geometry is reprojected into the centre (the far sky: rotation only).
const float CLOUD_UPSAMPLE_SKY_M = 1.0e7;

float cloud_depth_weight(float scene_m, float d) {
	return exp(-CLOUD_DEPTH_SHARPNESS * abs(log(max(scene_m, 1.0) / max(d, 1.0))));
}

// The centre's low-resolution position (texel units, texel centres at +0.5) of an Earth-local offset from the head,
// clamped to the image (an eye's edge pixel outside the centre reads its edge).
vec2 cloud_centre_position(vec3 offset) {
	vec4 clip = cloud_centre.clip_from_earth * vec4(offset, 1.0);
	vec2 ndc = clip.w > 0.0 ? clip.xy / clip.w : vec2(0.0);
	vec2 low = cloud_centre.sizes.zw;
	return clamp((ndc * 0.5 + 0.5) * low, vec2(0.5), low - 0.5);
}

// The tent gather at a centre position: premultiplied colour + alpha weighted by the tent and the depth weight; `cloud`
// gets the alpha-weighted cloud distance of the taps (0 without cloud).
// The tent radius [centre pixels] for a cloud at `cloud_m` (CLOUD_EDGE_SOFT_M; params.y: the centre pixel's angle).
float cloud_tent_radius(float cloud_m) {
	return clamp(CLOUD_EDGE_SOFT_M / (max(cloud_m, 1.0) * max(cloud_centre.params.y, 1.0e-9)), CLOUD_TENT_RADIUS_MIN,
			CLOUD_TENT_RADIUS_MAX);
}

vec4 cloud_tent_gather(vec2 position, float scene_m, float radius, out float cloud) {
	ivec2 low = ivec2(cloud_centre.sizes.zw);
	ivec2 base = ivec2(floor(position - 0.5)) - 1;
	vec4 sum = vec4(0.0);
	float weights = 0.0;
	float cloud_sum = 0.0;
	float cloud_weights = 0.0;
	vec4 best = vec4(0.0);
	float best_error = 1.0e30;
	for (int j = 0; j <= 3; j++) {
		for (int i = 0; i <= 3; i++) {
			ivec2 q = clamp(base + ivec2(i, j), ivec2(0), low - 1);
			vec2 d = abs(vec2(base + ivec2(i, j)) + 0.5 - position);
			float tent = max(0.0, 1.0 - d.x / radius) * max(0.0, 1.0 - d.y / radius);
			vec4 c = texelFetch(low_color, q, 0);
			vec2 depths = texelFetch(low_depth, q, 0).rg;
			float march_m = min(depths.r, 1.0e20);
			float error = abs(log(max(scene_m, 1.0) / max(march_m, 1.0)));
			float w = tent * exp(-CLOUD_DEPTH_SHARPNESS * error);
			sum += w * c;
			weights += w;
			if (depths.g > 0.0) {
				cloud_sum += w * c.a * depths.g;
				cloud_weights += w * c.a;
			}
			if (tent > 0.0 && error < best_error) {
				best_error = error;
				best = c;
			}
		}
	}
	cloud = cloud_weights > 1.0e-6 ? cloud_sum / cloud_weights : 0.0;
	return weights > 1.0e-4 ? sum / weights : best;
}

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 full = ivec2(cloud_view.sizes.xy);
	if (pixel.x >= full.x || pixel.y >= full.y) {
		return;
	}
	float scene_m = min(cloud_scene_distance(vec2(pixel) + 0.5, texelFetch(scene_depth, pixel, 0).r), 1.0e20);
	if (cloud_centre.params.x > 0.5) {
		// An eye reads the shared centre image at its own ray: first at its scene point, then again at the taps' cloud
		// distance (the cloud's own disparity; cloud_vr_stereo_test).
		vec3 dir = cloud_ray_earth(vec2(pixel) + 0.5);
		vec3 eye = cloud_view.eye_earth.xyz;
		float cloud_m;
		cloud_tent_gather(cloud_centre_position(eye + dir * min(scene_m, CLOUD_UPSAMPLE_SKY_M)), scene_m,
				CLOUD_TENT_RADIUS_MAX, cloud_m);
		float at_m = cloud_m > 0.0 ? min(cloud_m, scene_m) : min(scene_m, CLOUD_UPSAMPLE_SKY_M);
		float radius = cloud_m > 0.0 ? cloud_tent_radius(cloud_m) : CLOUD_TENT_RADIUS_MAX;
		imageStore(out_image, pixel, cloud_tent_gather(cloud_centre_position(eye + dir * at_m), scene_m, radius,
				cloud_m));
		return;
	}
	ivec2 low = ivec2(cloud_view.sizes.zw);
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
