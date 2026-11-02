shader_type spatial;
// A painted vehicle part with its baked surface (tools/structures/surface_textures.py: Falcon 9, Crew Dragon): lit_aerial + a detail map (RGB x gain multiplies the paint, A scales its roughness), and optionally a
// wear map (one channel: how much of the soot's albedo and roughness is mixed in). The mesh's UVs are metres; each map repeats over its period. All maps are linear data, mipped by the game with box averages and sampled
// with trilinear + anisotropic filtering, so the mean holds at every distance; nothing is view dependent, both eyes of a stereo frame sample the same maps with their own derivatives. No normal map; the geometry's own normal variance widens the specular lobe (normal_filter).

#include "res://shaders/atmosphere.gdshaderinc"

uniform vec4 albedo : source_color = vec4(1.0);
uniform float roughness : hint_range(0.0, 1.0) = 1.0;
uniform float metallic : hint_range(0.0, 1.0) = 0.0;
uniform sampler2D detail_map : hint_default_white, filter_linear_mipmap_anisotropic, repeat_enable;
uniform vec2 detail_period_m = vec2(1.0);
uniform float detail_gain = 1.0;
uniform sampler2D wear_map : hint_default_black, filter_linear_mipmap_anisotropic, repeat_enable;
uniform vec2 wear_period_m = vec2(1.0);
uniform vec3 wear_albedo = vec3(0.05);
uniform float wear_roughness = 0.85;
uniform float specular = 0.5;
uniform float normal_filter = 1.0;  // widens the specular lobe by the pixel's normal variance (Kaplanyan, Hill, Patney, Lefohn 2016, sigma^2 0.25); 0: off (diagnostics)
uniform float footprint_scale = 1.0;  // 1 always; tile_shimmer_check.gd: its supersampled reference evaluates the SAME mips, only sampled more densely

varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	vec2 detail_uv = UV / detail_period_m;
	vec2 wear_uv = UV / wear_period_m;
	vec4 detail = textureGrad(detail_map, detail_uv, dFdx(detail_uv) * footprint_scale, dFdy(detail_uv) * footprint_scale);
	float wear = textureGrad(wear_map, wear_uv, dFdx(wear_uv) * footprint_scale, dFdy(wear_uv) * footprint_scale).r;
	vec3 painted = albedo.rgb * detail.rgb * detail_gain;
	ALBEDO = mix(painted, wear_albedo, wear) * colour_factor;
	float base_roughness = mix(roughness * detail.a, wear_roughness, wear);
	vec3 dn_x = dFdx(NORMAL);
	vec3 dn_y = dFdy(NORMAL);
	float curvature = dot(dn_x, dn_x) + dot(dn_y, dn_y);
	ROUGHNESS = sqrt(clamp(base_roughness * base_roughness + normal_filter * min(2.0 * 0.25 * curvature, 0.5 /* [E] clamp */), 0.0, 1.0));
	METALLIC = metallic * (1.0 - wear);
	SPECULAR = specular;
}
