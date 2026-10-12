shader_type spatial;
render_mode cull_disabled;
// Open steel lattice: members along the bay edges and both diagonals of every bay, gaps cut out.

uniform vec4 steel : source_color = vec4(0.32, 0.33, 0.35, 1.0);
uniform float bay_m = 6.0;
uniform float member_fraction = 0.035;

varying vec3 local_position;

void vertex() {
	local_position = VERTEX;
}

void fragment() {
	vec2 p = vec2(local_position.x + local_position.z, local_position.y) / bay_m;
	vec2 f = fract(p);
	float edge = min(min(f.x, 1.0 - f.x), min(f.y, 1.0 - f.y));
	float diagonal = min(abs(f.x - f.y), abs(f.x + f.y - 1.0)) * 0.7071;
	ALBEDO = steel.rgb;
	METALLIC = 0.6;
	ROUGHNESS = 0.5;
	ALPHA = step(min(edge, diagonal), member_fraction);
	ALPHA_SCISSOR_THRESHOLD = 0.5;
}
