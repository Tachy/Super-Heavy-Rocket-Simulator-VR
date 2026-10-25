// The Earth's surface for the ellipsoid mesh (scaled space), shared by earth.gdshader and earth_clouds.gdshader:
// day texture lit by the Sun transmitted through the atmosphere plus the sky light, city lights on the night side, Moon
// shadow (solar eclipses); seen through the atmosphere (aerial perspective: color x T + in-scattering,
// atmosphere.gdshaderinc). All positions are in the body's local render frame (metres, unscaled), so floating origin
// and scaled-space rendering do not affect the lighting; the camera is the true Earth-local camera of the atmosphere
// globals (not the scaled mesh's position), so the hand-over to the terrain quadtree shows the same atmosphere.
// Until the atmosphere tables are loaded (atmosphere_enabled false): the vacuum lighting albedo x cos x day_exposure.
// With EARTH_CLOUDS defined (earth_clouds.gdshader; M8 weather and clouds, Task 10; World draws it while
// CloudLayer.far_enabled: weather shown, the camera beyond the scaled-space limit, where the cloud pass is off): the
// ground lit through the cloud column and the cloud layer over it (cloud_far.gdshaderinc). Without it the preprocessed
// source is the pre-Task-10 earth.gdshader: weather off draws every pixel as before (cloud_far_check.gd compares bit
// for bit; one shader with a run-time flag changed the code generation and ~1400 pixels by up to 4 half-float ulps).
#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"
#ifdef EARTH_CLOUDS
#include "res://shaders/clouds.gdshaderinc"
#include "res://shaders/cloud_light.gdshaderinc"
#include "res://shaders/cloud_far.gdshaderinc"
#endif

uniform sampler2D day_texture : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform sampler2D night_texture : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform vec3 sun_position_local;   // apparent, from the Earth's centre
uniform vec3 moon_position_local;  // apparent, from the Earth's centre
uniform float sun_radius_m = 695700000.0;
uniform float moon_radius_m = 1737400.0;
// City lights before the atmosphere tables are loaded (display value of a white texel); with them: scene-referred.
uniform float night_lights_intensity = 1.2;
global uniform float earth_city_lights_cd_m2;
uniform float day_exposure;  // set from Atmosphere.DAY_EXPOSURE (world.gd): the one exposure of every surface
// Ground patches store vertices relative to their centre; this restores body-local metres.
instance uniform vec3 vertex_offset = vec3(0.0);

varying vec3 local_position;
varying vec3 local_normal;
varying vec3 sky_eclipse_factor;  // atm_sky_eclipse_factor per vertex (it varies over tens of km)

void vertex() {
	local_position = VERTEX + vertex_offset;
	local_normal = NORMAL;
	sky_eclipse_factor = atmosphere_enabled ? atm_sky_eclipse_factor(local_position) : vec3(1.0);
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = sun_position_local - local_position;
	float cos_sun = dot(n, normalize(to_sun));
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, moon_position_local - local_position, moon_radius_m);
	vec3 albedo = texture(day_texture, UV).rgb;
	// City lights switch on through civil twilight (sun 0..6 deg below the horizon).
	float night_factor = smoothstep(0.0, -0.1, cos_sun);
	vec3 night = texture(night_texture, UV).rgb;
	night = night * night * night_factor;  // the squared texel suppresses the dark-sky noise

	if (atmosphere_enabled) {
		// The direct Sun x the local sunlit fraction; the sky light with the eclipse sky's factor (the dark sky inside
		// the Moon's shadow, the lit air around it: atm_sky_eclipse_factor).
		// The mesh's vertices are ~156 km apart, the 2024 umbra ~200 km wide: inside the Moon's shadow (penumbra and
		// umbra) the factor per fragment, elsewhere the vertex value (review M4; the mesh is drawn from > 1.5e7 m only,
		// the extra cost only during an eclipse).
		vec3 eclipse_factor = sunlit < 1.0 ? atm_sky_eclipse_factor(local_position) : sky_eclipse_factor;
		vec3 sky_light;
		vec3 sun_light = atm_sun_and_sky(local_position, n, sky_light);
		vec3 moon_light = atm_moonlit() ? atm_moon_light(local_position, n) : vec3(0.0);
		vec3 day = albedo * (sun_light * sunlit + sky_light * eclipse_factor + moon_light);
#ifdef EARTH_CLOUDS
		// The light through the cloud column (cloud_ground_light's column tier, as the terrain beyond 140 km): the
		// direct Sun x its transmittance, the sky x its diffuse transmittance, the Sun's light diffused through the
		// column as an isotropic downward radiance x the upper sky's view factor; the Moon likewise. The clear-sky
		// expression above stays untouched, so the variant without EARTH_CLOUDS is the shader as before.
		{
			vec3 camera_offset = local_position - atmosphere_camera_hi - atmosphere_camera_lo;
			float upper = 0.5 * (1.0 + dot(n, cloud_geodetic_up(local_position)));
			vec4 cloud = cloud_ground_light(camera_offset, normalize(atmosphere_sun_direction), false);
			vec3 glow = atm_sun_at(local_position) * (max(cloud.w, 0.0) * cloud.y * sunlit * upper);
			vec3 moon_clouded = moon_light;
			if (atm_moonlit()) {
				vec3 moon_dir = normalize(atmosphere_moon_direction);
				vec4 moon_cloud = cloud_ground_light(camera_offset, moon_dir, false);
				vec3 moon_normal = atm_moon_at(local_position);
				moon_clouded = moon_normal * (max(dot(n, moon_dir), 0.0) * moon_cloud.x)
						+ atm_moon_sky_light(local_position, n) * moon_cloud.z
						+ moon_normal * (max(moon_cloud.w, 0.0) * moon_cloud.y * upper);
			}
			day = albedo * (sun_light * (cloud.x * sunlit) + sky_light * (cloud.z * eclipse_factor) + moon_clouded
					+ glow);
		}
#endif
		// Camera-relative offset: the mesh is far from the camera whenever it is drawn (the terrain takes over near
		// the ground), so the difference of the absolute float32 positions is accurate enough here.
		vec3 offset = (local_position - atmosphere_camera_hi) - atmosphere_camera_lo;
		vec3 transmittance;
		vec3 in_scatter = atm_to_point_offset(offset, transmittance);
		// City lights: scene-referred (user ruling 2026-10-25, review I6), a luminance through the eye model like every
		// other: the texel x earth_city_lights_cd_m2 (Atmosphere.CITY_LIGHTS_PEAK_CD_M2) x the scene value per cd/m^2.
		vec3 city = night * (earth_city_lights_cd_m2 * atmosphere_scene_per_cd_m2);
		ALBEDO = (day + city) * transmittance + in_scatter;
#ifdef EARTH_CLOUDS
		vec4 layer = cloud_far_layer(local_position, eclipse_factor);  // premultiplied, as the cloud pass
		ALBEDO = layer.rgb + (1.0 - layer.a) * ALBEDO;
#endif
	} else {
		ALBEDO = albedo * max(cos_sun, 0.0) * sunlit * day_exposure + night * night_lights_intensity;
	}
}
