shader_type spatial;
// Window glass of the cabin: nearly clear, a faint specular sheen; drawn in the transparent pass without depth writes so the
// reveal behind stays visible (CockpitView).
render_mode blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform float opacity : hint_range(0.0, 1.0) = 0.06;
uniform vec3 tint = vec3(0.6, 0.65, 0.7);  // the windows' faint grey; the player's raised visor: its copper coating (Task 9 VR fix B)

void fragment() {
	ALBEDO = tint;
	ALPHA = opacity;
	ROUGHNESS = 0.05;
	SPECULAR = 0.8;
}
