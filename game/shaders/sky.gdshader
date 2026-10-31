shader_type sky;
// The Earth's sky from the precomputed atmosphere (atmosphere.gdshaderinc), for every pixel without geometry, from the
// camera's true position: the same function on the pad, during the ascent and in orbit (no switch). Rays into the
// ground show the model ground (the WGS84 ellipsoid lowered to the model's ground radius, below every camera and
// every drawn surface): the day texture lit by the transmitted Sun and the sky light, seen through the atmosphere. In
// the radiance-cubemap pass this ground gives Godot's ambient and reflections the ground bounce and the earthshine;
// in the background pass the terrain and the Earth mesh normally cover it. The cube is drawn anew only after a write to
// this material (World._update_sky_radiance, SkyRadianceRefresh).
// At night (atmosphere_moon_light > 0) the Moon lights the sky and the model ground as a second, weak source.
// Until the tables are uploaded (atmosphere_enabled false) the sky is black, as before the atmosphere existed.

#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/moon_regolith.gdshaderinc"

uniform sampler2D day_texture : source_color, filter_linear_mipmap, repeat_enable;
uniform bool day_texture_bound = false;
// Albedo without the day texture: the precomputation's ground albedo, atmosphere_ground_albedo (the tables' parameters,
// config/atmosphere/earth_optics.toml).

// Written to mark the material updated: only that makes Godot redraw the cube. No value is read.
uniform float radiance_serial = 0.0;
// The lunar ground in the cube, for an eye near the Moon (radius 0: none): a sphere of the mean texture albedo under the
// Sun and the earthshine, in front of the Earth. Set with each refresh (render world axes).
uniform float radiance_moon_radius_m = 0.0;
uniform float radiance_moon_distance_m = 0.0;
uniform vec3 radiance_moon_zenith = vec3(0.0, 1.0, 0.0);  // from the Moon's centre to the eye
uniform vec3 radiance_moon_to_sun = vec3(1.0, 0.0, 0.0);
uniform vec3 radiance_moon_to_earth = vec3(1.0, 0.0, 0.0);
uniform float radiance_moon_albedo = 0.3;
// The parts of the Sun's and the Earth's discs the eye sees, for the ground around it (RadianceFarEarth.ground_fraction):
// the terrain's shadows at the eye stand for the ground in view.
uniform float radiance_moon_sun_visible = 1.0;
uniform float radiance_moon_earth_visible = 1.0;
// Below this angular radius the Earth's disc (with its air) is drawn this large in the cube at the same flux
// (RadianceFarEarth.MIN_RADIUS_RAD, the rationale there) with a smooth radial profile (radiance_earth_profile).
// Near the Moon (radiance_earth_flat) it is a grey disc of the earthshine the regolith and the eye adaptation get
// (moon_earthshine, Atmosphere.earthshine_relative), not the cloudless model Earth (0.37 x that, measured): one earthshine,
// x radiance_earth_flux (the part of the true disc the eye sees). A disc that would cross the cube's lunar horizon is drawn
// lifted clear of it around radiance_earth_drawn: only the true fraction decides, not where the wide disc is cut.
uniform float radiance_earth_min_radius_rad = 0.07;
uniform float radiance_earth_flux = 1.0;
uniform bool radiance_earth_flat = false;
uniform vec3 radiance_earth_drawn = vec3(0.0, 1.0, 0.0);  // from the eye, render world axes
// The light under the clouds over the clear sky's by channel (Atmosphere.cloud_radiance_factors), 1 with weather off,
// for the
// sky's half and the ground's: the ground's light holds the direct Sun, the sky's does not. In the cube because Godot's
// ambient_light_energy does not reach a sky's cube.
uniform vec3 radiance_cloud_sky_factor = vec3(1.0);
uniform vec3 radiance_cloud_ground_factor = vec3(1.0);

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
	vec3 moon_light = atm_moonlit() ? atm_moon_light(point_local, normal_local) : vec3(0.0);
	vec3 transmittance;
	vec3 in_scatter = atm_to_point_model(v * t, transmittance);  // camera-relative: the hit's offset along the ray
	return albedo * (sun_light + sky_light + moon_light) * transmittance + in_scatter;
}

// The lunar ground along the unit world ray v; false: the ray misses it.
bool radiance_moon_ground(vec3 v, out vec3 color) {
	color = vec3(0.0);
	if (radiance_moon_radius_m <= 0.0) {
		return false;
	}
	float mu = dot(radiance_moon_zenith, v);
	float d = radiance_moon_distance_m;
	float R = radiance_moon_radius_m;
	float c = (d - R) * (d + R);  // d^2 - R^2 without the cancellation near the ground
	float disc = d * d * mu * mu - c;
	if (mu >= 0.0 || disc < 0.0) {
		return false;
	}
	float t = c / (-d * mu + sqrt(disc));
	vec3 n = normalize(radiance_moon_zenith * d + v * t);
	float light = moon_regolith_scale * (regolith_reflectance(n, radiance_moon_to_sun, -v) * radiance_moon_sun_visible
			* moon_sun_scene + regolith_reflectance(n, radiance_moon_to_earth, -v) * radiance_moon_earth_visible
			* moon_earthshine);
	color = vec3(radiance_moon_albedo * light * atmosphere_exposure_gain);
	return true;
}

// The widened disc's radiance over a uniform disc's of that radius, at the same flux: 3 (1 - x)^2 for x = (1 - cos angle) /
// (1 - cos radius), which integrates exactly to the uniform disc's solid angle. About a Gaussian of one texel's sigma (radius
// 2.5 texels): a hard edge makes the texels the disc covers jump as it moves.
float radiance_earth_profile(float cos_angle) {
	float x = clamp((1.0 - cos_angle) / (1.0 - cos(radiance_earth_min_radius_rad)), 0.0, 1.0);
	return 3.0 * (1.0 - x) * (1.0 - x);
}

void sky() {
	vec3 lunar_ground;
	if (!atmosphere_enabled) {
		COLOR = vec3(0.0);
	} else if (AT_CUBEMAP_PASS && radiance_moon_ground(EYEDIR, lunar_ground)) {
		COLOR = lunar_ground;
	} else if (AT_CUBEMAP_PASS && radiance_earth_flat) {
		// A white plate facing the disc reads L x solid angle / pi = moon_earthshine x the gain, grey as the regolith's light.
		float solid_angle = 2.0 * ATM_PI * (1.0 - cos(radiance_earth_min_radius_rad));
		COLOR = vec3(radiance_earth_flux * moon_earthshine * atmosphere_exposure_gain * ATM_PI / solid_angle
				* radiance_earth_profile(dot(EYEDIR, radiance_earth_drawn)));
	} else {
		vec3 camera_local = atm_camera_local();
		vec3 view_local = normalize(atmosphere_world_to_earth * EYEDIR);
		// A ray into the model ground shows the ground (both passes), every other ray the sky.
		vec3 v = normalize(view_local * ATM_STRETCH);
		vec3 up;
		float r;
		float rho;
		atm_camera_model(camera_local, up, r, rho);
		// The cube's far Earth: a small disc widened to a steady size, the flux kept.
		float flux_factor = 1.0;
		if (AT_CUBEMAP_PASS && r > atmosphere_top_radius_m) {
			float radius_rad = asin(atmosphere_top_radius_m / r);
			float angle = acos(clamp(-dot(up, v), -1.0, 1.0));
			if (radius_rad < radiance_earth_min_radius_rad && angle < radiance_earth_min_radius_rad) {
				float shrink = radius_rad / radiance_earth_min_radius_rad;
				vec3 side = v - up * dot(up, v);
				float side_length = length(side);
				side = side_length > 1.0e-6 ? side / side_length : vec3(0.0);
				v = normalize(-up * cos(angle * shrink) + side * sin(angle * shrink));
				view_local = normalize(v / ATM_STRETCH);
				flux_factor = shrink * shrink * radiance_earth_profile(cos(angle));
			}
		}
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
			if (atm_moonlit()) {
				COLOR += atm_moon_sky(camera_local, view_local);  // the moonlit sky (night only)
			}
		}
		COLOR *= flux_factor;
		if (AT_CUBEMAP_PASS) {
			COLOR *= hits_ground ? radiance_cloud_ground_factor : radiance_cloud_sky_factor;
		}
	}
}
