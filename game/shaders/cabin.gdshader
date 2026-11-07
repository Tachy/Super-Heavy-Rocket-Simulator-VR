shader_type spatial;
// Matte / painted cabin surfaces of the Crew Dragon (CockpitView): lit_aerial's look (Godot PBR under the Sun's light, aerial
// perspective as FOG) plus two cabin terms. (1) ambient_visibility scales the sky's ambient light and reflections (AO): the sky
// does not reach the inside of a closed capsule, only the Sun through the windows and the cabin lamps do; a small remainder
// stands in for the bounced light of the white walls (look pass [call]). (2) Panel seams: thin dark lines on a grid in the
// part's own coordinates (surface detail by shader, not geometry), strength seam_depth (0 = none).

#include "res://shaders/atmosphere.gdshaderinc"

uniform vec4 albedo : source_color = vec4(1.0);
uniform float roughness : hint_range(0.0, 1.0) = 0.85;
uniform float metallic : hint_range(0.0, 1.0) = 0.0;
uniform float specular : hint_range(0.0, 1.0) = 0.4;
uniform float ambient_visibility : hint_range(0.0, 1.0) = 0.2;
uniform float fill = 0.0;  // scene value of the cabin's bounced light (E / pi x scene_per_cd_m2), set by CockpitView each frame
uniform float seam_depth : hint_range(0.0, 1.0) = 0.0;
uniform vec2 seam_pitch_m = vec2(0.42, 0.30);
uniform float seam_width_m = 0.0015;

varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes
varying vec3 local_position;
varying vec3 local_normal;

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	local_position = VERTEX;
	local_normal = NORMAL;
}

float seam(vec3 p, vec3 n) {
	// Two grids along the two axes the surface is not facing.
	vec3 a = abs(n);
	vec2 q = a.x > 0.7 ? p.yz : (a.y > 0.7 ? p.xz : p.xy);
	vec2 cell = q / seam_pitch_m;
	vec2 d = abs(fract(cell + 0.5) - 0.5) * seam_pitch_m;
	float w = max(seam_width_m, 1.0e-5);
	vec2 line = 1.0 - smoothstep(vec2(0.5 * w), vec2(1.5 * w), d);
	return max(line.x, line.y);
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	float line = seam_depth > 0.0 ? seam(local_position, normalize(local_normal)) : 0.0;
	line *= smoothstep(0.12, 0.45, abs(dot(normalize(NORMAL), normalize(VIEW))));  // the 1.5 mm seams alias to moire at grazing angles (pilot_eye ceiling): fade them out there
	ALBEDO = albedo.rgb * (1.0 - seam_depth * 0.55 * line) * colour_factor;
	ROUGHNESS = roughness;
	METALLIC = metallic;
	SPECULAR = specular;
	EMISSION = albedo.rgb * (1.0 - metallic) * fill;
	AO = ambient_visibility;
	AO_LIGHT_AFFECT = 0.0;
}
