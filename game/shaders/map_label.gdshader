shader_type spatial;
// Map-view label (MapLabel: a TextMesh): a camera-facing text of constant screen size drawn over everything - Label3D's
// billboard + fixed_size + no_depth_test, with the code of Godot's BaseMaterial3D for those flags - in a
// display-referred colour past night vision's rod step unchanged (vision.gdshaderinc; Label3D cannot take that
// inverse: its colours are 8-bit vertex colours, its material cannot be overridden).
render_mode unshaded, blend_mix, depth_test_disabled, depth_draw_never, cull_disabled, shadows_disabled, fog_disabled;

#include "res://shaders/vision.gdshaderinc"

uniform vec4 colour : source_color = vec4(1.0);

void vertex() {
	MODELVIEW_MATRIX = VIEW_MATRIX * mat4(MAIN_CAM_INV_VIEW_MATRIX[0], MAIN_CAM_INV_VIEW_MATRIX[1],
			MAIN_CAM_INV_VIEW_MATRIX[2], MODEL_MATRIX[3]);
	// Scale by depth (stereo: the full distance, as BaseMaterial3D).
	float sc = IS_MULTIVIEW ? length(MODELVIEW_MATRIX[3].xyz) : -MODELVIEW_MATRIX[3].z;
	MODELVIEW_MATRIX[0] *= sc;
	MODELVIEW_MATRIX[1] *= sc;
	MODELVIEW_MATRIX[2] *= sc;
}

void fragment() {
	ALBEDO = vision_display_referred(colour.rgb);
	ALPHA = colour.a;
}
