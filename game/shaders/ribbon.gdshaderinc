// Shared parts of the line shaders (orbit_ribbon.gdshader, trail_ribbon.gdshader): a camera-facing ribbon of constant
// pixel width with soft edges, the cut behind the Earth's limb and the fade where the line of sight grazes the atmosphere.

// View-space position of a ribbon corner: `here` widened sideways by `side` (-1 / +1) x half the width, perpendicular
// to the segment towards `ahead` and to the line of sight (also for points behind the camera: no division by w).
vec3 ribbon_corner(vec3 here, vec3 ahead, float side, float width_px, mat4 projection, vec2 viewport_size) {
	vec3 across = cross(ahead - here, here);
	float across_length = length(across);
	across = across_length > 0.0 ? across / across_length : vec3(0.0);
	float metres_per_px = 2.0 * max(-here.z, 1.0e-3) / (projection[1][1] * viewport_size.y);
	return here + across * side * 0.5 * width_px * metres_per_px;
}

// Opacity of the soft ribbon edge.
float ribbon_edge(float side) {
	return 1.0 - smoothstep(0.5, 1.0, abs(side));
}

// Visibility of a point seen along `direction` (view space, unit) at `true_distance_m`, against the Earth sphere centred
// at `centre` (view space, true metres): 0 behind the limb (-1 to discard), fading where the line of sight passes
// within `atmosphere_height_m` of the surface, 1 otherwise.
float limb_visibility(vec3 direction, float true_distance_m, vec3 centre, float radius_m, float atmosphere_height_m) {
	float along = dot(direction, centre);
	if (along > 0.0 && along < true_distance_m) {  // the closest approach lies between the eye and the point
		float grazing_height_m = length(direction * along - centre) - radius_m;
		if (grazing_height_m < 0.0) {
			return -1.0;  // behind the limb
		}
		return smoothstep(0.0, atmosphere_height_m, grazing_height_m);
	}
	return 1.0;
}
