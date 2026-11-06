shader_type spatial;
// Plate-clad plinth of the Pad B tower (a unit box scaled per instance, LatticeBuilder): the mount's generated plate map
// (mount_panels.png: courses and plates with rust-tan seam lines) projected on the box in metres of the box's own scaled frame
// (the dominant axis of the face: a side face takes height / width, the roof x / z), so the seams keep their size whatever the
// box's dimensions; tinted to the plinth's albedo (the map's mean divided out, so the tint IS the albedo). Seen through the
// atmosphere as the other structures (atm_fog, as tower_steel.gdshader).

#include "res://shaders/atmosphere.gdshaderinc"

uniform sampler2D panels : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform vec4 tint : source_color = vec4(0.46, 0.43, 0.38, 1.0);
uniform vec3 panels_mean = vec3(0.3, 0.27, 0.2);  // linear mean of the map (set from the image)
uniform float tile_m = 12.0;                      // the map repeats every tile_m metres
uniform float roughness = 0.8;
uniform float metallic = 0.0;

varying vec3 box_position;  // VERTEX x instance scale (metres), the box's lower corner at the origin along y
varying vec3 box_normal;
varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes

void vertex() {
	vec3 scale = vec3(length(MODEL_MATRIX[0].xyz), length(MODEL_MATRIX[1].xyz), length(MODEL_MATRIX[2].xyz));
	box_position = VERTEX * scale + vec3(0.5 * scale.x, 0.5 * scale.y, 0.5 * scale.z);
	box_normal = NORMAL;
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	vec3 n = abs(box_normal);
	vec2 uv = box_position.xy;
	if (n.y > n.x && n.y > n.z) {
		uv = box_position.xz;
	} else if (n.x > n.z) {
		uv = box_position.zy;
	}
	vec3 texel = texture(panels, uv / tile_m).rgb;
	ALBEDO = tint.rgb * texel / panels_mean * colour_factor;
	ROUGHNESS = roughness;
	METALLIC = metallic;
	SPECULAR = 0.5;
}
