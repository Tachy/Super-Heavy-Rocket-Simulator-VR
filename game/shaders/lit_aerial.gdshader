shader_type spatial;
// Lit surfaces of vehicles and structures (vehicles, tower parts, pads, drone ship; game/world/lit_aerial.gd): the
// StandardMaterial3D look they had (Godot's PBR lighting: the Sun's DirectionalLight, the sky's ambient and
// reflections), seen through the atmosphere - aerial perspective as Godot's FOG output (atm_fog,
// atmosphere.gdshaderinc), so a rocket climbing away from a ground camera fades into the sky. At zero distance the look
// is unchanged (T = 1, nothing in-scattered).

#include "res://shaders/atmosphere.gdshaderinc"

uniform vec4 albedo : source_color = vec4(1.0);
uniform float roughness : hint_range(0.0, 1.0) = 1.0;
uniform float metallic : hint_range(0.0, 1.0) = 0.0;
uniform float specular : hint_range(0.0, 1.0) = 0.5;  // StandardMaterial3D's default
uniform vec4 emission : source_color = vec4(0.0, 0.0, 0.0, 1.0);

varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	ALBEDO = albedo.rgb * colour_factor;
	ROUGHNESS = roughness;
	METALLIC = metallic;
	SPECULAR = specular;
	EMISSION = emission.rgb * colour_factor;
}
