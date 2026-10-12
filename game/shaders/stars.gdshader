shader_type spatial;
// Star points: vertex colour = B-V colour, vertex alpha = relative brightness.
render_mode unshaded, cull_disabled, depth_draw_never, blend_add;

uniform float point_scale = 2.4;

void vertex() {
	POINT_SIZE = clamp(point_scale * sqrt(COLOR.a) * 2.0, 1.0, 5.0);
}

void fragment() {
	ALBEDO = COLOR.rgb * clamp(COLOR.a, 0.04, 1.0);
}
