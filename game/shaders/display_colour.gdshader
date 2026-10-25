shader_type spatial;
// A flat display-referred colour for UI markers in the 3D scene (OrbitLine): unshaded, alpha-blended, and past night
// vision's rod step unchanged (vision.gdshaderinc) - Godot's built-in StandardMaterial3D cannot take that inverse.
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled, fog_disabled;

#include "res://shaders/vision.gdshaderinc"

uniform vec4 colour : source_color = vec4(1.0);

void fragment() {
	ALBEDO = vision_display_referred(colour.rgb);
	ALPHA = colour.a;
}
