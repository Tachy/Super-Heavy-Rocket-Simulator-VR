shader_type spatial;
render_mode cull_disabled;
// Open steel lattice: members along the bay edges and both diagonals of every bay, gaps cut out; seen through the
// atmosphere like the other structures (atm_fog, as lit_aerial.gdshader).

#include "res://shaders/atmosphere.gdshaderinc"

uniform vec4 steel : source_color = vec4(0.32, 0.33, 0.35, 1.0);
uniform float bay_m = 6.0;
uniform float member_fraction = 0.035;

varying vec3 local_position;
varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes

void vertex() {
	local_position = VERTEX;
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec2 p = vec2(local_position.x + local_position.z, local_position.y) / bay_m;
	vec2 f = fract(p);
	float edge = min(min(f.x, 1.0 - f.x), min(f.y, 1.0 - f.y));
	float diagonal = min(abs(f.x - f.y), abs(f.x + f.y - 1.0)) * 0.7071;
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	ALBEDO = steel.rgb * colour_factor;
	METALLIC = 0.6;
	ROUGHNESS = 0.5;
	ALPHA = step(min(edge, diagonal), member_fraction);
	ALPHA_SCISSOR_THRESHOLD = 0.5;
}
