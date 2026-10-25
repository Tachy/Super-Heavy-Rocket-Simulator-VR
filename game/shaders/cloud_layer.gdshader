shader_type spatial;
// Volumetric clouds near the camera (M8 weather and clouds, Task 8): a full-screen quad (CloudLayer) drawn first in the
// transparent pass (plumes and the other transparent objects over it), raymarching the cloud shell (-0.5 to 20 km above
// the ellipsoid) between the camera and the opaque scene's depth. Per pixel:
// - Steps: outside the field's cloudy region coarse steps growing with the distance (the fraction that spans the
//   segment from 1 km to its end in cloud_steps steps; at most one height layer: 250 m below 4 km, 1 km above),
//   inside it fine steps (cloud_fine_step_fraction of the distance, >= 10 m: a cumulus edge is resolved instead of hit
//   or missed per pixel), at most cloud_max_steps per pixel. Each sample sits at a jittered point of its step (one
//   hash value per screen pixel and frame, cloud_jitter) - a function of the pixel alone, the same for both VR eyes (and
//   the cloud shapes are Earth-fixed: both eyes see the same clouds). No temporal accumulation.
// - Extinction: clouds.gdshaderinc (field, coverage-preserving shapes, faded detail). The background behind the cloud
//   is seen through exp(-tau) (Beer-Lambert of the unscaled extinction: the core's cloud_sun_transmittance along the
//   ray); the scattered light is attenuated on the scaled depths exp(-(1 - f) tau) (cloud_optics.hpp).
// - Light per sample: the Sun transmitted through the air to the sample (atm_sun_at x the eclipse's unocculted part) x
//   the Sun's scattering source (TMS; the Sun's optical depth through the clouds from cloud_light_steps samples over
//   CLOUD_LIGHT_PATH_M), plus the clear sky's horizontal irradiance there (atm_sky_light) x the sky source; at night the
//   Moon the same way (atmosphere_moon_light). Each step integrates the source exactly over its constant extinction:
//   L += T J (1 - exp(-(1 - f) dtau)) / (1 - f). Early exit when the scaled transmittance falls below 0.01.
// - Compositing (premultiplied): the clouds' light seen through the air to their transmittance-weighted mean distance
//   d_c (atm_to_point_offset: T_c, S_c) and the background through the cloud:
//     final = T_bg x scene + (1 - T_bg) S_c + T_c L,
//   i.e. the air in front of the cloud is kept, the scene's own air light behind it is dimmed with it (exact for cloud
//   light at one distance; a deck seen at a slant spreads over its depth).
render_mode unshaded, blend_premul_alpha, depth_draw_never, depth_test_disabled, cull_disabled, shadows_disabled,
		fog_disabled, skip_vertex_transform;

#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/clouds.gdshaderinc"

uniform sampler2D depth_texture : hint_depth_texture, filter_nearest, repeat_disable;

global uniform int cloud_steps;        // coarse steps spanning the segment (quality preset)
global uniform int cloud_max_steps;    // cap on the steps per pixel (coarse + fine)
global uniform float cloud_fine_step_fraction;  // fine step / distance inside the field's cloudy region
global uniform int cloud_light_steps;  // samples toward the Sun / Moon per in-cloud sample
global uniform int cloud_frame;        // frame counter for the jitter

// [E] Path toward the light from an in-cloud sample: 6 km (a 1-km deck at a 10-deg Sun); the source's attenuation
// exp(-(1 - f) tau_s) is below 1 % after tau_s 20 (1.3 km of a thick cumulus).
const float CLOUD_LIGHT_PATH_M = 6000.0;
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

void vertex() {
	POSITION = vec4(VERTEX.xy, 0.5, 1.0);  // the quad covers the screen in every view (depth test off)
}

// The ray start's jitter: a value in [0, 1) per screen pixel and frame from an integer hash (PCG, Jarzynski & Olano
// 2020, "Hash Functions for GPU Rendering", JCGT 9(3)). White noise: fine grain at cloud edges (interleaved gradient
// noise drew diagonal hatching there).
float cloud_jitter(vec2 pixel) {
	uint v = uint(pixel.x) * 1973u + uint(pixel.y) * 9277u + uint(cloud_frame) * 26699u;
	v = v * 747796405u + 2891336453u;
	v = ((v >> ((v >> 28u) + 4u)) ^ v) * 277803737u;
	v = (v >> 22u) ^ v;
	return float(v) / 4294967296.0;
}

// Scattering source of one light (Sun or Moon) at a sample: J per unit irradiance, or 0 below the local horizon.
float cloud_light_source(vec3 light_dir, vec3 normal, vec3 dir, vec3 offset, float height_m, vec2 column, float jitter) {
	float mu0 = dot(normal, light_dir);
	if (mu0 <= 0.0) {
		return 0.0;
	}
	float tau_light = cloud_optical_depth_toward(offset, height_m, light_dir, CLOUD_LIGHT_PATH_M, cloud_light_steps,
			jitter);
	vec3 diff = light_dir - dir;
	float one_minus_cos = 0.5 * dot(diff, diff);
	return cloud_scattering_source(1.0 - one_minus_cos, one_minus_cos, -dot(dir, normal), mu0, tau_light, column.x,
			column.y);
}

bool cloud_shade(vec2 screen_uv, vec2 frag_coord, float depth, mat4 inv_projection, mat4 inv_view, out vec3 color,
		out float alpha) {
	color = vec3(0.0);
	alpha = 0.0;
	if (!cloud_enabled || !atmosphere_enabled) {
		return false;
	}
	vec2 ndc_xy = screen_uv * 2.0 - 1.0;
	vec4 near_view = inv_projection * vec4(ndc_xy, 1.0, 1.0);  // reversed Z: 1 is the near plane
	vec3 view_dir = normalize(near_view.xyz / near_view.w);
	float scene_m = 1.0e30;  // the sky: no geometry
	if (depth > 0.0) {
		vec4 scene_view = inv_projection * vec4(ndc_xy, depth, 1.0);
		scene_m = length(scene_view.xyz / scene_view.w);
	}
	vec3 dir = normalize(atmosphere_world_to_earth * normalize((inv_view * vec4(view_dir, 0.0)).xyz));

	// The segment inside the shell, before the scene.
	CloudRay ray = cloud_ray(dir);
	if (cloud_debug_output == 3) {
		vec2 crossing = cloud_sphere_crossing(ray, cloud_debug_height_m);
		float d_h = crossing.x > 0.0 ? crossing.x : crossing.y;
		if (crossing.x > crossing.y || d_h <= 0.0) {
			return false;
		}
		vec3 offset_h = dir * d_h;
		vec2 lat_lon_h = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_h);
		CloudCell cell = cloud_field(lat_lon_h, cloud_debug_height_m);
		color = vec3(cloud_coverage(cloud_shape_coordinates(offset_h, cloud_debug_height_m), cell.fraction));
		alpha = 1.0;
		return true;
	}
	vec2 top = cloud_sphere_crossing(ray, CLOUD_TOP_M);
	if (top.x > top.y || top.y <= 0.0) {
		return false;
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
		return false;
	}

	float jitter = cloud_jitter(frag_coord);
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
		float step = min(max(a, start) * coarse_rel, layer_m / max(climb, 1.0e-3));
		step = max(step, CLOUD_MIN_STEP_M);
		// Inside the field's cloudy region the step shrinks to the fine fraction of the distance: the sub-grid shapes
		// are resolved (a coarse step would hit or miss a cumulus edge per pixel: grain).
		float d = a + jitter * step;
		vec2 lat_lon = cloud_lat_lon(camera + dir * d);
		CloudCell cell = cloud_field(lat_lon, cloud_height_along(ray, d));
		if (cell.extinction > 0.0) {
			// Never finer than what still reaches the segment's end within the remaining steps (no far cloud is cut).
			float remaining = float(max(cloud_max_steps - i, 1));
			float reach = max(a, start) * (exp(log(max(d1, start) / max(a, start)) / remaining) - 1.0);
			float fine = max(max(max(a, start) * cloud_fine_step_fraction, reach), CLOUD_MIN_STEP_M);
			if (fine < step) {
				step = fine;
				d = a + jitter * step;
				lat_lon = cloud_lat_lon(camera + dir * d);
				cell = cloud_field(lat_lon, cloud_height_along(ray, d));
			}
		}
		float b = min(a + step, d1);
		d = min(d, b);
		float a_this = a;
		a = b;
		if (cell.extinction <= 0.0) {
			continue;
		}
		vec3 offset = dir * d;
		float height = cloud_height_along(ray, d);
		float in_cloud;
		float beta = cloud_extinction_in_cell(cell, offset, height, d, cloud_debug_output == 0, in_cloud);
		if (beta <= 0.0) {
			continue;
		}
		float tau = beta * (b - a_this);
		vec3 p = camera + offset;
		vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);  // the ellipsoid's normal (geodetic up)
		vec2 column = cloud_column_depths(lat_lon, height, in_cloud);
		float view_up = -dot(dir, normal);
		vec3 source;
		if (cloud_debug_output == 2) {
			source = vec3(cloud_light_source(sun_dir, normal, dir, offset, height, column, jitter));
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
			float sky_source = cloud_sky_scattering_source(view_up, column.x, column.y);
			source = sun_irradiance * cloud_light_source(sun_dir, normal, dir, offset, height, column, jitter)
					+ sky_irradiance * sky_source;
			if (moonlit) {
				source += moon_irradiance * cloud_light_source(moon_dir, normal, dir, offset, height, column, jitter)
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

	if (cloud_debug_output == 1) {
		color = vec3(transmittance_bg);
		alpha = 1.0;
		return true;
	}
	if (cloud_debug_output == 2) {
		color = radiance;
		alpha = 1.0;
		return true;
	}
	alpha = 1.0 - transmittance_bg;
	if (alpha <= 1.0e-6 && dot(radiance, radiance) <= 0.0) {
		return false;
	}
	vec3 air_transmittance = vec3(1.0);
	vec3 air_light = vec3(0.0);
	if (alpha > 1.0e-6) {
		air_light = atm_to_point_offset(dir * (weighted_distance / alpha), air_transmittance);
	}
	color = air_transmittance * radiance + alpha * air_light;
	return true;
}

void fragment() {
	vec3 color;
	float alpha;
	if (!cloud_shade(SCREEN_UV, FRAGCOORD.xy, texture(depth_texture, SCREEN_UV).r, INV_PROJECTION_MATRIX,
			INV_VIEW_MATRIX, color, alpha)) {
		discard;
	}
	ALBEDO = color;
	ALPHA = alpha;
}
