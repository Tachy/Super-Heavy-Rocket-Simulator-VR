shader_type spatial;
// Checks only (cloud_far_check.gd section 1): the far field's slab radiance of the Sun (cloud_far_light_radiance, the
// function earth.gdshader calls) per unit Sun irradiance, one geometry per pixel of a full-screen quad. params (one
// texel per pixel): x the slab's vertical optical depth, y the Sun's cosine to the vertical, z the view's, w the
// relative azimuth (0: the view on the Sun's side - forward), the frame of the core's slab_reflected_radiance_mc.
render_mode unshaded, cull_disabled, depth_test_disabled;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/clouds.gdshaderinc"
#include "res://shaders/cloud_far.gdshaderinc"

uniform sampler2D params : filter_nearest, repeat_disable;

void vertex() {
	POSITION = vec4(VERTEX.xy, 1.0, 1.0);
}

void fragment() {
	vec4 p = texelFetch(params, ivec2(FRAGCOORD.xy), 0);
	float s0 = sqrt(max(1.0 - p.y * p.y, 0.0));
	float sv = sqrt(max(1.0 - p.z * p.z, 0.0));
	vec3 up = vec3(0.0, 0.0, 1.0);
	vec3 light_dir = vec3(-s0, 0.0, p.y);  // toward the Sun: its propagation is (s0, 0, -mu0)
	vec3 view_prop = vec3(sv * cos(p.w), sv * sin(p.w), p.z);
	ALBEDO = vec3(cloud_far_light_radiance(p.x, light_dir, view_prop, up));
}
