shader_type spatial;
// Orbit line as a camera-facing ribbon of constant pixel width with soft edges (game/vehicle/trajectory_view.gd): a
// plain line strip is one aliased pixel wide and looks stepped. Depth-tested against the scene, so vehicles and the
// Earth cover it. Behind the Earth's limb it is cut analytically with the true geometry (the scaled-space mapping moves
// far points closer, so the depth buffer alone cannot decide), and where the line of sight grazes the atmosphere it
// fades out.
// Vertex data (camera-relative, rebuilt every frame): VERTEX = this end (render space, scaled), CUSTOM0.xyz = a point
// further along the segment (same space), UV.x = true distance from the camera [m], UV.y = side (-1 / +1).
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled, fog_disabled;

#include "res://shaders/ribbon.gdshaderinc"

uniform vec4 line_color : source_color = vec4(0.3, 1.0, 0.45, 1.0);
uniform float width_px = 2.5;
uniform vec3 earth_center_world = vec3(0.0);  // true Earth centre relative to the render origin [m]
uniform float earth_radius_m = 6371000.0;
uniform float atmosphere_height_m = 100000.0;

varying float true_distance_m;
varying float side;

void vertex() {
	vec3 here = (MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz;
	vec3 ahead = (MODELVIEW_MATRIX * vec4(CUSTOM0.xyz, 1.0)).xyz;
	POSITION = PROJECTION_MATRIX * vec4(ribbon_corner(here, ahead, UV.y, width_px, PROJECTION_MATRIX, VIEWPORT_SIZE), 1.0);
	true_distance_m = UV.x;
	side = UV.y;
}

void fragment() {
	float visibility = limb_visibility(normalize(VERTEX), true_distance_m,
			(VIEW_MATRIX * vec4(earth_center_world, 1.0)).xyz, earth_radius_m, atmosphere_height_m);
	if (visibility < 0.0) {
		discard;
	}
	ALBEDO = line_color.rgb;
	ALPHA = line_color.a * ribbon_edge(side) * visibility;
}
