shader_type spatial;
// Engine bells of the detailed vehicle models (game/vehicle/vehicle_models.gd): a dark metal with regen ribs from a mipmapped
// map (one rib per repeat, addressed by the azimuth about the engine axis X): the slope tilts the normal, the slope energy the
// mips lost widens the GGX alpha, so a far bell gets rougher and does not shimmer. Lit like LitAerial, opaque.

#include "res://shaders/atmosphere.gdshaderinc"

uniform vec3 albedo : source_color = vec3(0.25);
uniform float base_roughness = 0.55;
uniform float metallic_value = 0.3;
uniform sampler2D rib_map : filter_linear_mipmap_anisotropic, repeat_enable;
uniform float rib_count = 0.0;     // ribs around the bell
uniform float rib_slope = 0.3;     // [E] largest tilt of the ribs
uniform float exit_x_m = -2.1;     // ribs from the exit plane ...
uniform float rib_top_x_m = 0.0;   // ... up to here (local X, thrust +X)

varying vec3 world_offset;
varying vec3 local_pos;
varying vec3 local_normal;
varying vec3 view_normal;
varying vec3 view_tangent;

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	local_pos = VERTEX;
	local_normal = NORMAL;
	// In the vertex stage: MODEL_MATRIX holds the MultiMesh instance's transform, the fragment's model matrix does not.
	mat3 to_view = mat3(VIEW_MATRIX) * mat3(MODEL_MATRIX);
	view_normal = to_view * NORMAL;
	view_tangent = to_view * vec3(0.0, -VERTEX.z, VERTEX.y);
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;

	vec3 n = normalize(local_normal);
	float r = max(length(local_pos.yz), 1e-4);
	// Azimuth derivatives from the position, not from atan: no seam in the mip choice.
	float u = atan(local_pos.z, local_pos.y) / TAU * rib_count;
	vec2 dp_dx = dFdx(local_pos.yz);
	vec2 dp_dy = dFdy(local_pos.yz);
	float scale = rib_count / (TAU * r * r);
	float du_dx = (local_pos.y * dp_dx.y - local_pos.z * dp_dx.x) * scale;
	float du_dy = (local_pos.y * dp_dy.y - local_pos.z * dp_dy.x) * scale;
	vec4 t = textureGrad(rib_map, vec2(u, 0.5), vec2(du_dx, 0.0), vec2(du_dy, 0.0));
	vec3 radial = vec3(0.0, local_pos.y, local_pos.z) / r;
	float on = step(0.0, dot(n, radial)) * step(exit_x_m - 0.01, local_pos.x) * step(local_pos.x, rib_top_x_m)
			* step(0.5, rib_count);
	float slope = (t.r * 2.0 - 1.0) * rib_slope * on;
	float lost = max(t.g * rib_slope * rib_slope - slope * slope, 0.0) * on;
	vec3 n_out = normalize(normalize(view_normal) - normalize(view_tangent) * slope);
	float alpha = base_roughness * base_roughness;

	ALBEDO = albedo * colour_factor;
	ROUGHNESS = clamp(sqrt(sqrt(alpha * alpha + lost)), 0.04, 1.0);
	METALLIC = metallic_value;
	NORMAL = n_out;
}
