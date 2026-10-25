shader_type spatial;
// Engine plume (vehicle_view.gd): additive, self-luminous glow - the look of its former unshaded StandardMaterial3D,
// attenuated by the atmosphere's transmittance between the camera and the fragment only [call, M8 atmosphere spec]:
// an additive layer adds no in-scattering of its own (the sky behind it already carries it).
render_mode unshaded, blend_add, cull_disabled, shadows_disabled;

#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/vision.gdshaderinc"

uniform vec4 color : source_color = vec4(1.0, 0.55, 0.25, 0.55);

varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec3 transmittance = vec3(1.0);
	if (atmosphere_enabled) {
		atm_to_point_offset(atmosphere_world_to_earth * world_offset, transmittance);
	}
	// Display-referred (one rule for emitters without a physical luminance - plumes, city lights, the stars' own
	// brightness: game/world/eye_adaptation.gd): not multiplied by the eye adaptation's gain, and past night vision's
	// rod step unchanged [call 2026-10-25: a bright source, seen by the cones in colour].
	ALBEDO = vision_display_referred(color.rgb * transmittance);
	ALPHA = color.a;
}
