shader_type spatial;
// Steel of the Pad B tower members (LatticeBuilder; game/world/structures/tower_material.gd): the CC0 ambientCG Metal032
// colour map projected triplanar in the member's own scaled frame (the mesh's unit length times the instance scale, so
// the texel size does not depend on the member's length), plus procedural grime: value noise that darkens and roughens
// patches, with a different offset per instance (INSTANCE_ID, stable: no floating-origin dependence). Seen through the
// atmosphere like the other structures (atm_fog, as lit_aerial.gdshader). Without the texture (use_texture = false) a
// plain tinted steel with the same grime.

#include "res://shaders/atmosphere.gdshaderinc"

uniform sampler2D steel_albedo : source_color, filter_linear_mipmap, repeat_enable;
uniform bool use_texture = false;
uniform vec4 tint : source_color = vec4(1.0);
uniform float texel_scale_m = 2.0;     // the texture repeats every 2 m
uniform float grime_scale_m = 2.5;     // noise feature size
uniform float grime_strength = 0.45;   // darkening at the dirtiest
uniform float roughness_clean = 0.40;
uniform float roughness_dirty = 0.80;
uniform float metallic = 0.65;

varying vec3 member_position;  // VERTEX x instance scale (metres along the member's own axes)
varying vec3 member_normal;
varying vec3 instance_offset;
varying vec3 world_offset;     // fragment - the render origin (= the atmosphere's camera), world axes

float hash13(vec3 p) {
	p = fract(p * 0.1031);
	p += dot(p, p.zyx + 31.32);
	return fract((p.x + p.y) * p.z);
}

float value_noise(vec3 p) {
	vec3 i = floor(p);
	vec3 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	return mix(mix(mix(hash13(i), hash13(i + vec3(1, 0, 0)), f.x),
			mix(hash13(i + vec3(0, 1, 0)), hash13(i + vec3(1, 1, 0)), f.x), f.y),
			mix(mix(hash13(i + vec3(0, 0, 1)), hash13(i + vec3(1, 0, 1)), f.x),
			mix(hash13(i + vec3(0, 1, 1)), hash13(i + vec3(1, 1, 1)), f.x), f.y), f.z);
}

void vertex() {
	vec3 scale = vec3(length(MODEL_MATRIX[0].xyz), length(MODEL_MATRIX[1].xyz), length(MODEL_MATRIX[2].xyz));
	member_position = VERTEX * scale;
	member_normal = NORMAL;
	float id = float(INSTANCE_ID);
	instance_offset = vec3(fract(id * 0.618034), fract(id * 0.414214), fract(id * 0.732051)) * 97.0;
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	vec3 colour = tint.rgb;
	if (use_texture) {
		vec3 w = pow(abs(member_normal), vec3(4.0));
		w /= w.x + w.y + w.z;
		vec3 p = member_position / texel_scale_m;
		vec3 texel = texture(steel_albedo, p.yz).rgb * w.x + texture(steel_albedo, p.xz).rgb * w.y
				+ texture(steel_albedo, p.xy).rgb * w.z;
		colour *= texel;
	}
	vec3 q = (member_position + instance_offset) / grime_scale_m;
	float grime = smoothstep(0.35, 0.8, 0.65 * value_noise(q) + 0.35 * value_noise(q * 3.1));
	ALBEDO = colour * (1.0 - grime_strength * grime) * colour_factor;
	ROUGHNESS = mix(roughness_clean, roughness_dirty, grime);
	METALLIC = metallic * (1.0 - 0.5 * grime);
}
