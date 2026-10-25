shader_type sky;
// The Earth's sky from the precomputed atmosphere (atmosphere.gdshaderinc), for every pixel without geometry, from the
// camera's true position: the same function on the pad, during the ascent and in orbit (no switch). Rays into the
// ground show the model ground (the WGS84 ellipsoid lowered to the model's ground radius, below every camera and
// every drawn surface): the day texture lit by the transmitted Sun and the sky light, seen through the atmosphere. In
// the radiance-cubemap pass this ground gives Godot's ambient and reflections the ground bounce and the earthshine;
// in the background pass the terrain and the Earth mesh normally cover it.
// Until the tables are uploaded (atmosphere_enabled false) the sky is black, as before the atmosphere existed.

#include "res://shaders/atmosphere.gdshaderinc"

uniform sampler2D day_texture : source_color, filter_linear_mipmap, repeat_enable;
uniform bool day_texture_bound = false;
// Albedo without the day texture: the precomputation's ground albedo, atmosphere_ground_albedo (the tables' parameters,
// config/atmosphere/earth_optics.toml).

// The model ground seen along v from the camera, lit and attenuated (scene units).
vec3 ground_radiance(vec3 camera_local, vec3 view_local) {
	vec3 v = normalize(view_local * ATM_STRETCH);
	vec3 up;
	float r;
	float rho;
	atm_camera_model(camera_local, up, r, rho);
	vec3 camera_model = up * r;
	float t;
	if (r > atmosphere_top_radius_m) {
		// From space: the closest approach of the ray, then back to the ground sphere (well conditioned far out).
		float along = dot(camera_model, v);
		float b = length(cross(camera_model, v));
		float R = atmosphere_bottom_radius_m;
		t = -along - sqrt(max((R - b) * (R + b), 0.0));
	} else {
		t = atm_distance_to_bottom(r, rho, dot(up, v));
	}
	vec3 point_model = camera_model + v * t;
	vec3 point_local = point_model / ATM_STRETCH;
	vec3 normal_local = normalize(point_model * ATM_STRETCH);  // geodetic up = M^T x model radial
	vec3 albedo = vec3(atmosphere_ground_albedo);
	if (day_texture_bound) {
		// Equirectangular texture (EllipsoidMesh): u = (lon + 180 deg) / 360 deg, v = (90 deg - lat) / 180 deg; ITRF
		// x = local z, ITRF y = local x, the geodetic latitude from the normal.
		float lon = atan(point_local.x, point_local.z);
		float lat = asin(clamp(normal_local.y, -1.0, 1.0));
		albedo = textureLod(day_texture, vec2(lon / (2.0 * ATM_PI) + 0.5, 0.5 - lat / ATM_PI), 0.0).rgb;  // no mip jump at the lon seam
	}
	vec3 sky_light;
	vec3 sun_light = atm_sun_and_sky(point_local, normal_local, sky_light) * atm_sunlit(point_local);  // Moon shadow
	sky_light *= atm_sky_eclipse_factor(point_local);
	vec3 transmittance;
	vec3 in_scatter = atm_to_point_model(v * t, transmittance);  // camera-relative: the hit's offset along the ray
	return albedo * (sun_light + sky_light) * transmittance + in_scatter;
}

void sky() {
	if (!atmosphere_enabled) {
		COLOR = vec3(0.0);
	} else {
		vec3 camera_local = atm_camera_local();
		vec3 view_local = normalize(atmosphere_world_to_earth * EYEDIR);
		// A ray into the model ground shows the ground (both passes), every other ray the sky.
		vec3 v = normalize(view_local * ATM_STRETCH);
		vec3 up;
		float r;
		float rho;
		atm_camera_model(camera_local, up, r, rho);
		bool hits_ground;
		if (r > atmosphere_top_radius_m) {
			vec3 camera_model = up * r;
			float R = atmosphere_bottom_radius_m;
			hits_ground = dot(camera_model, v) < 0.0 && length(cross(camera_model, v)) < R;
		} else {
			hits_ground = atm_ray_intersects_ground(r, rho, dot(up, v));
		}
		if (hits_ground) {
			COLOR = ground_radiance(camera_local, view_local);
		} else {
			vec3 transmittance;
			COLOR = atm_sky(camera_local, view_local, transmittance);
		}
	}
}
