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
// mode 4 (section 1e, look fixes T7c): the shapes' cover seen straight down through a block of uniform cloud fraction
// (the volume's density rule at state 0: shape > cloud_threshold), params x the fraction, y the block's depth (m), z the
// type (CLOUD_TYPE_*), w the column's index (a point of the R2 sequence on the sphere, the block's base at 1 km); out r 1
// where any height of the block exceeds the threshold of the fraction as stored, g the same with the volume's depth-
// aware fraction (cloud_volume_fraction).
render_mode unshaded, cull_disabled, depth_test_disabled;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/clouds.gdshaderinc"
#include "res://shaders/cloud_far.gdshaderinc"

uniform sampler2D params : filter_nearest, repeat_disable;
uniform int mode = 0;
// mode 4: the block's in-cloud vertical depth and the opacity that counts as cloud (a satellite cloud mask's ~0.05).
const float PROBE_BLOCK_TAU = 10.0;
const float PROBE_COVER_ALPHA = 0.05;

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
	} else if (mode == 4) {
		float lon = fract(p.w * 0.7548776662) * 6.2831853 - 3.1415927;
		float z = fract(p.w * 0.5698402910) * 2.0 - 1.0;
		vec3 radial = vec3(sqrt(max(1.0 - z * z, 0.0)) * vec3(cos(lon), sin(lon), 0.0)) + vec3(0.0, 0.0, z);
		int type = int(p.z + 0.5);
		float tile_v = type == CLOUD_TYPE_CIRRUS ? CLOUD_ICE_TILE_V_M : CLOUD_SHAPE_TILE_V_M;
		float step_m = 0.5 * tile_v / CLOUD_SHAPE_TEXELS;
		int n = max(int(ceil(p.y / step_m)), 1);
		float f_vol = cloud_volume_fraction(p.x, p.y, type);
		vec2 thr = vec2(cloud_threshold(p.x, 0.0), cloud_threshold(f_vol, 0.0));
		// the in-cloud extinction of a block of in-cloud depth PROBE_BLOCK_TAU at the fraction (the volume's: the grid
		// mean over the edge ramp's mean)
		vec2 beta = PROBE_BLOCK_TAU / p.y * vec2(p.x, p.x)
				/ max(vec2(texture(cloud_edge_mean, cloud_table_uv(p.x, 0.0)).r,
				texture(cloud_edge_mean, cloud_table_uv(f_vol, 0.0)).r), vec2(1.0e-4));
		vec2 tau = vec2(0.0);
		for (int i = 0; i < n; i++) {
			float h = 1000.0 + (float(i) + 0.5) * p.y / float(n);
			vec4 s = texture(cloud_noise_shape, cloud_type_shape_coordinates_world(type, (CLOUD_A_M + h) * radial, h));
			tau += beta * vec2(cloud_edge_ramp(s.r, thr.x, s.a), cloud_edge_ramp(s.r, thr.y, s.a)) * p.y / float(n);
		}
		vec2 alpha = 1.0 - exp(-tau);
		ALBEDO = vec3(step(PROBE_COVER_ALPHA, alpha.x), step(PROBE_COVER_ALPHA, alpha.y), alpha.y);
	} else {
		float s0 = sqrt(max(1.0 - p.y * p.y, 0.0));
		float sv = sqrt(max(1.0 - p.z * p.z, 0.0));
		vec3 up = vec3(0.0, 0.0, 1.0);
		vec3 light_dir = vec3(-s0, 0.0, p.y);  // toward the Sun: its propagation is (s0, 0, -mu0)
		vec3 view_prop = vec3(sv * cos(p.w), sv * sin(p.w), p.z);
		ALBEDO = vec3(cloud_far_light_radiance(p.x, light_dir, view_prop, up));
	}
}
