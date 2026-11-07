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
#include "res://shaders/water.gdshaderinc"

uniform sampler2D day_texture : source_color, filter_linear_mipmap, repeat_enable;
uniform bool day_texture_bound = false;
// Water fraction (255 = sea) on the day texture's grid (data/textures/earth_water.png); unbound: black, no sea.
uniform sampler2D water_texture : filter_linear_mipmap, repeat_enable, hint_default_black;
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

// The albedo of the imagery's ground around the eye by horizontal distance and azimuth (GroundAlbedoNear: the mean in the
// cells of RADIANCE_LOCAL_SECTORS azimuth sectors x the annuli of RADIANCE_LOCAL_RING_EDGES_M = GroundAlbedoNear.SECTORS,
// RING_EDGES_M; sector-major; valid 0: none, the global day texture alone). The global texel (~7 km) averages the land with
// the bay and the Gulf (0.108 at Starbase against 0.20 of the ground within a hundred metres) and so leaves a wall's bounce
// 30-40 % low; the cube is drawn at the eye, so the ground near it, on the side a face looks at, counts most. The cell values
// hold at the rings' mid-distances and the sectors' centres and are interpolated linearly in between; beyond the last ring's
// outer edge (radiance_local_albedo_near_m, horizontal distance) the ground blends to the texture by radiance_local_albedo_far_m.
const int RADIANCE_LOCAL_SECTORS = 8;
const int RADIANCE_LOCAL_RINGS = 5;
const float RADIANCE_LOCAL_RING_EDGES_M[RADIANCE_LOCAL_RINGS + 1] = float[](0.0, 25.0, 100.0, 250.0, 700.0, 1500.0);
uniform float radiance_local_albedo_valid = 0.0;
uniform vec3 radiance_local_albedo[RADIANCE_LOCAL_SECTORS * RADIANCE_LOCAL_RINGS];
uniform float radiance_local_albedo_near_m = 1500.0;
uniform float radiance_local_albedo_far_m = 6000.0;

// One sector's ring values at the horizontal distance r: held inside the first mid-distance and beyond the last, linear between.
vec3 radiance_local_sector_albedo(int sector, float r) {
	int base = sector * RADIANCE_LOCAL_RINGS;
	if (r <= 0.5 * (RADIANCE_LOCAL_RING_EDGES_M[0] + RADIANCE_LOCAL_RING_EDGES_M[1])) {
		return radiance_local_albedo[base];
	}
	for (int k = 0; k + 1 < RADIANCE_LOCAL_RINGS; k++) {
		float mid0 = 0.5 * (RADIANCE_LOCAL_RING_EDGES_M[k] + RADIANCE_LOCAL_RING_EDGES_M[k + 1]);
		float mid1 = 0.5 * (RADIANCE_LOCAL_RING_EDGES_M[k + 1] + RADIANCE_LOCAL_RING_EDGES_M[k + 2]);
		if (r <= mid1) {
			return mix(radiance_local_albedo[base + k], radiance_local_albedo[base + k + 1], (r - mid0) / (mid1 - mid0));
		}
	}
	return radiance_local_albedo[base + RADIANCE_LOCAL_RINGS - 1];
}

// The local ground albedo at the horizontal distance r (m) and azimuth (rad, from north toward east) from the eye.
vec3 radiance_local_ground_albedo(float r, float azimuth) {
	float u = mod(azimuth / (2.0 * ATM_PI / float(RADIANCE_LOCAL_SECTORS)), float(RADIANCE_LOCAL_SECTORS));
	int s0 = int(floor(u)) % RADIANCE_LOCAL_SECTORS;
	int s1 = (s0 + 1) % RADIANCE_LOCAL_SECTORS;
	return mix(radiance_local_sector_albedo(s0, r), radiance_local_sector_albedo(s1, r), fract(u));
}

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
	if (radiance_local_albedo_valid > 0.5) {
		// The hit's horizontal offset from the eye in the eye's east / north (the pole is the local y axis: ITRF z): the
		// profile is built by horizontal distance (GroundAlbedoNear.ring_albedo), not the slant ray length t.
		vec3 east = normalize(cross(vec3(0.0, 1.0, 0.0), up));
		vec3 north = cross(up, east);
		vec2 offset = vec2(dot(v, east), dot(v, north)) * t;
		float r_h = length(offset);
		albedo = mix(radiance_local_ground_albedo(r_h, r_h > 0.0 ? atan(offset.x, offset.y) : 0.0), albedo,
				smoothstep(radiance_local_albedo_near_m, radiance_local_albedo_far_m, r_h));
	}
	vec3 sky_light;
	vec3 sun_light = atm_sun_and_sky(point_local, normal_local, sky_light) * atm_sunlit(point_local);  // Moon shadow
	sky_light *= atm_sky_eclipse_factor(point_local);
	vec3 moon_light = atm_moonlit() ? atm_moon_light(point_local, normal_local) : vec3(0.0);
	vec3 day = albedo * (sun_light + sky_light + moon_light);
	// The sea (look fixes T3): the calm-sea model of the terrain and the Earth mesh (water.gdshaderinc, flat normal) where
	// the global water fraction says water - the water-leaving reflectance under (1 - F), the Fresnel sky, the glint.
	float water = 0.0;
	if (day_texture_bound) {
		float lon = atan(point_local.x, point_local.z);
		float lat = asin(clamp(normal_local.y, -1.0, 1.0));
		water = textureLod(water_texture, vec2(lon / (2.0 * ATM_PI) + 0.5, 0.5 - lat / ATM_PI), 0.0).r;
	}
	if (water > 0.0) {
		vec3 offset_local = (v * t) / ATM_STRETCH;
		vec3 to_camera = -normalize(view_local);
		vec3 reflected_dir = water_reflected_direction(normal_local, normal_local, to_camera);
		vec3 reflected_transmittance;
		vec3 sky_reflected = atm_sky_at_offset(offset_local, reflected_dir, reflected_transmittance);
		vec3 sea = water_radiance(albedo, normal_local, normal_local, 0.0, normalize(atmosphere_sun_direction), to_camera,
				atm_sunlit(point_local), atm_sun_at(point_local), sky_light, sky_reflected);
		if (atm_moonlit()) {
			sea += water_radiance(albedo, normal_local, normal_local, 0.0, normalize(atmosphere_moon_direction), to_camera,
					1.0, atm_moon_at(point_local), atm_moon_sky_light(point_local, normal_local),
					atm_moon_sky_at_offset(offset_local, reflected_dir));
		}
		day = mix(day, sea, water);
	}
	vec3 transmittance;
	vec3 in_scatter = atm_to_point_model(v * t, transmittance);  // camera-relative: the hit's offset along the ray
	return day * transmittance + in_scatter;
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
