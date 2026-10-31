shader_type spatial;
// Shell of the Pad B launch mount (tan steel panels in courses): the generated panel map (tools/structures/
// mount_textures.py, repeated every tile_m metres of the shell's UV) lit like lit_aerial.gdshader and seen through the
// atmosphere (aerial perspective as Godot's FOG output).

#include "res://shaders/atmosphere.gdshaderinc"

uniform sampler2D panels : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform float tile_m = 6.4;
uniform float roughness : hint_range(0.0, 1.0) = 0.75;
uniform float metallic : hint_range(0.0, 1.0) = 0.15;

varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	ALBEDO = texture(panels, UV / tile_m).rgb * colour_factor;
	ROUGHNESS = roughness;
	METALLIC = metallic;
	SPECULAR = 0.5;
}
