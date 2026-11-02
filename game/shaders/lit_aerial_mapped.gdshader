shader_type spatial;
// lit_aerial.gdshader with a baked albedo map (the ISS's cells, panel lines; tools/structures/iss_textures.py): the mesh UVs are metres, the map repeats over map_period_m. Mipped by the
// game (box averages), sampled with trilinear + anisotropic filtering: nothing view dependent, both eyes of a stereo frame sample the same map with their own derivatives.

#include "res://shaders/atmosphere.gdshaderinc"

uniform vec4 albedo : source_color = vec4(1.0);
uniform float roughness : hint_range(0.0, 1.0) = 1.0;
uniform float metallic : hint_range(0.0, 1.0) = 0.0;
uniform sampler2D albedo_map : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform vec2 map_period_m = vec2(1.0);

varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	ALBEDO = albedo.rgb * texture(albedo_map, UV / map_period_m).rgb * colour_factor;
	ROUGHNESS = roughness;
	METALLIC = metallic;
	SPECULAR = 0.5;
}
