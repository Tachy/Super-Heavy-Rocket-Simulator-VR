shader_type spatial;
// Checks only (cloud_far_check.gd section 1): the far field's slab radiance of the Sun (cloud_far_light_radiance, the
// function earth.gdshader calls) per unit Sun irradiance, one geometry per pixel of a full-screen quad. params (one
// texel per pixel): x the slab's vertical optical depth, y the Sun's cosine to the vertical, z the view's, w the
// relative azimuth (0: the view on the Sun's side - forward), the frame of the core's slab_reflected_radiance_mc.
// mode 1 (section 1b, look fixes T7b): cloud_far_bright_top_transmittance, params x the air's Beer transmittance (one
// value in all channels), y the view's cosine, z the cover, w the in-cloud depth; out r.
// mode 2 (section 1c): cloud_far_meso_cover, params x the cell's cover, y the longitude (rad), z sin(latitude), w the
// footprint (m); out r the local cover.
// mode 3 (section 1d): the cloud-to-cloud spread (cloud_spread_factor) at a point on the sphere, params y the
// longitude (rad), z sin(latitude); out r the factor.
render_mode unshaded, cull_disabled, depth_test_disabled;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/clouds.gdshaderinc"
#include "res://shaders/cloud_far.gdshaderinc"

uniform sampler2D params : filter_nearest, repeat_disable;
uniform int mode = 0;

void vertex() {
	POSITION = vec4(VERTEX.xy, 1.0, 1.0);
}

void fragment() {
	vec4 p = texelFetch(params, ivec2(FRAGCOORD.xy), 0);
	if (mode == 1) {
		ALBEDO = vec3(cloud_far_bright_top_transmittance(vec3(p.x), p.y, p.z, p.w).r);
	} else if (mode == 2) {
		float c = sqrt(max(1.0 - p.z * p.z, 0.0));
		vec3 point = CLOUD_A_M * vec3(c * cos(p.y), c * sin(p.y), p.z);
		ALBEDO = vec3(cloud_far_meso_cover(point, p.x, p.w));
	} else if (mode == 3) {
		float c = sqrt(max(1.0 - p.z * p.z, 0.0));
		ALBEDO = vec3(cloud_spread_factor(CLOUD_A_M * vec3(c * cos(p.y), c * sin(p.y), p.z)));
	} else {
		float s0 = sqrt(max(1.0 - p.y * p.y, 0.0));
		float sv = sqrt(max(1.0 - p.z * p.z, 0.0));
		vec3 up = vec3(0.0, 0.0, 1.0);
		vec3 light_dir = vec3(-s0, 0.0, p.y);  // toward the Sun: its propagation is (s0, 0, -mu0)
		vec3 view_prop = vec3(sv * cos(p.w), sv * sin(p.w), p.z);
		ALBEDO = vec3(cloud_far_light_radiance(p.x, light_dir, view_prop, up));
	}
}
