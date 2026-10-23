shader_type spatial;
// Orbit line as a camera-facing ribbon of constant pixel width with soft edges (game/vehicle/trajectory_view.gd): a
// plain line strip is one aliased pixel wide and looks stepped. Depth-tested against the scene, so vehicles and the
// Earth cover it. Behind the Earth's limb it is cut analytically with the true geometry (the scaled-space mapping moves
// far points closer, so the depth buffer alone cannot decide), and where the line of sight grazes the atmosphere it
// fades out.
// Vertex data: VERTEX = this end (render space, scaled), CUSTOM0.xyz = a point further along the segment (same space),
// UV.x = true distance from the camera [m], UV.y = side of the ribbon (-1 / +1).
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled, fog_disabled;

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
	// Perpendicular to the segment and to the line of sight; works for points behind the camera too (no division by w).
	vec3 across = cross(ahead - here, here);
	float across_length = length(across);
	across = across_length > 0.0 ? across / across_length : vec3(0.0);
	float metres_per_px = 2.0 * max(-here.z, 1.0e-3) / (PROJECTION_MATRIX[1][1] * VIEWPORT_SIZE.y);
	here += across * UV.y * 0.5 * width_px * metres_per_px;
	POSITION = PROJECTION_MATRIX * vec4(here, 1.0);
	true_distance_m = UV.x;
	side = UV.y;
}

void fragment() {
	float alpha = line_color.a * (1.0 - smoothstep(0.5, 1.0, abs(side)));  // soft edges
	// Line of sight from the eye to this point of the line (true distance) against the Earth sphere.
	vec3 direction = normalize(VERTEX);
	vec3 centre = (VIEW_MATRIX * vec4(earth_center_world, 1.0)).xyz;
	float along = dot(direction, centre);
	if (along > 0.0 && along < true_distance_m) {  // the closest approach lies between the eye and the point
		float grazing_height_m = length(direction * along - centre) - earth_radius_m;
		if (grazing_height_m < 0.0) {
			discard;  // behind the limb
		}
		alpha *= smoothstep(0.0, atmosphere_height_m, grazing_height_m);  // seen through the atmosphere
	}
	ALBEDO = line_color.rgb;
	ALPHA = alpha;
}
