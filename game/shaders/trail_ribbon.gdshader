shader_type spatial;
// Flown trail (game/vehicle/flight_trail_view.gd): Earth-fixed samples stay in the mesh (rebuilt only when a sample is
// added); this shader turns them with the Earth, places them relative to the camera with the scaled-space mapping of the
// bodies (CelestialBody.SCALED_SPACE_DISTANCE_M) and draws the same ribbon as the orbit line. The trail fades in from
// fade_in_start_m behind the body. float32 Earth-fixed positions are good to ~1 m: hidden inside the first kilometre.
// Vertex data: VERTEX = this sample (Earth-fixed m in render axes), CUSTOM0.xyz = a point further along the segment,
// UV.y = side (-1 / +1), UV2.x = length along the trail from its oldest sample [m].
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled, fog_disabled;

#include "res://shaders/ribbon.gdshaderinc"

uniform vec4 line_color : source_color = vec4(1.0, 1.0, 1.0, 0.85);
uniform float width_px = 2.0;
uniform vec3 earth_center_world = vec3(0.0);  // true Earth centre relative to the render origin [m]
uniform mat3 fixed_to_world = mat3(1.0);  // Earth orientation, render axes to render axes (Frames.basis_to_render)
uniform float scaled_space_distance_m = 1.5e7;
uniform float earth_radius_m = 6371000.0;
uniform float atmosphere_height_m = 100000.0;
uniform float head_path_m = 0.0;  // length along the path from the body to the newest sample
uniform float newest_path_m = 0.0;  // length along the trail from the oldest to the newest sample
uniform float fade_in_start_m = 1000.0;
uniform float fade_in_length_m = 2000.0;

varying float true_distance_m;
varying float side;
varying float path_m;

vec3 placed_world(vec3 fixed_m, out float distance_m) {
	vec3 world = earth_center_world + fixed_to_world * fixed_m;
	distance_m = length(world);
	return distance_m > scaled_space_distance_m ? world * (scaled_space_distance_m / distance_m) : world;
}

void vertex() {
	float distance_m;
	float ahead_distance_m;
	vec3 here = (VIEW_MATRIX * vec4(placed_world(VERTEX, distance_m), 1.0)).xyz;
	vec3 ahead = (VIEW_MATRIX * vec4(placed_world(CUSTOM0.xyz, ahead_distance_m), 1.0)).xyz;
	POSITION = PROJECTION_MATRIX * vec4(ribbon_corner(here, ahead, UV.y, width_px, PROJECTION_MATRIX, VIEWPORT_SIZE), 1.0);
	true_distance_m = distance_m;
	side = UV.y;
	path_m = head_path_m + newest_path_m - UV2.x;  // along the path back from the body
}

void fragment() {
	float visibility = limb_visibility(normalize(VERTEX), true_distance_m,
			(VIEW_MATRIX * vec4(earth_center_world, 1.0)).xyz, earth_radius_m, atmosphere_height_m);
	if (visibility < 0.0) {
		discard;
	}
	float fade_in = smoothstep(fade_in_start_m, fade_in_start_m + fade_in_length_m, path_m);
	ALBEDO = line_color.rgb;
	ALPHA = line_color.a * ribbon_edge(side) * visibility * fade_in;
}
