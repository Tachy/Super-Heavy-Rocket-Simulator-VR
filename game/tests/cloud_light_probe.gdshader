shader_type spatial;
// Checks only (cloud_check.gd section 7): evaluates the surfaces' cloud light (cloud_light.gdshaderinc, the functions
// terrain_earth.gdshader calls) at receivers given as camera-relative offsets (Earth-local render axes) in a float
// texture, one receiver per pixel of a full-screen quad. No terrain geometry: the receivers can lie anywhere around the
// camera, at the ranges of the shadow maps' hand-overs (9-11 km, 120-140 km). Output channel 0: (the distance rule's
// direct transmittance, the near map alone, the far map alone); channel 1: (the column slanted to the Sun, the Sun's
// diffuse share, the sky's diffuse transmittance).
render_mode unshaded, cull_disabled, depth_test_disabled;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/clouds.gdshaderinc"
#include "res://shaders/cloud_light.gdshaderinc"

uniform sampler2D receivers : filter_nearest, repeat_disable;
uniform int channel = 0;

void vertex() {
	POSITION = vec4(VERTEX.xy, 1.0, 1.0);
}

void fragment() {
	vec3 offset = texelFetch(receivers, ivec2(FRAGCOORD.xy), 0).xyz;
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset;
	vec3 sun = normalize(atmosphere_sun_direction);
	vec3 up = cloud_geodetic_up(p);
	float mu0 = dot(up, sun);
	vec2 column = cloud_column_mean_depth(cloud_lat_lon(p), cloud_height_of(p));
	if (channel == 0) {
		float composed = cloud_shadow_transmittance(offset, column, mu0);
		float near = cloud_shadow_map_transmittance(cloud_shadow_near, cloud_shadow_near_anchor, cloud_shadow_extent.x,
				offset);
		float far = cloud_shadow_map_transmittance(cloud_shadow_far, cloud_shadow_far_anchor, cloud_shadow_extent.y,
				offset);
		ALBEDO = vec3(composed, near, far);
	} else {
		ALBEDO = vec3(cloud_column_direct(column, mu0), cloud_sun_diffuse(column, mu0), cloud_sky_factor(column));
	}
}
