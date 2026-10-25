// Astronomical refraction of the Earth's atmosphere for the camera (core environment::refraction_lift_table through
// StarsimCore.atmospheric_refraction, uploaded by world.gd): a source's geometric direction from the camera is turned
// up in its vertical plane by the lift of its true elevation. Sun disc, Moon and stars; near the horizon the lift
// falls by ~1/5 across a disc (the flattened setting Sun), the disc stays visible ~2-3 min after its geometric set.
// One wavelength (550 nm): no dispersion. The fragments behind see the sky along the refracted (apparent) ray.

global uniform sampler2D refraction_lift : filter_linear, repeat_disable;  // R32F: lift (rad) at the grid samples
global uniform vec4 refraction_grid;  // true elevation min (rad), span (rad), sample count, 1 = on
global uniform vec3 refraction_up;    // render world unit vector: the camera's geodetic vertical

// Lift (rad) of a source at `true_elevation` (rad): grid min + span (i / (n - 1))^2, the lift of min below it (that
// part is behind the Earth), 0 above min + span.
float refraction_lift_rad(float true_elevation) {
	if (refraction_grid.w < 0.5 || refraction_grid.y <= 0.0) {
		return 0.0;
	}
	float x = (true_elevation - refraction_grid.x) / refraction_grid.y;
	if (x > 1.0) {
		return 0.0;
	}
	float n = refraction_grid.z;
	float u = sqrt(max(x, 0.0)) * (n - 1.0);
	return texture(refraction_lift, vec2((u + 0.5) / n, 0.5)).r;
}

// Where the refracted image of a world point is drawn for a camera at `camera_world` (both render world space).
vec3 refracted_world_point(vec3 world_point, vec3 camera_world) {
	vec3 offset = world_point - camera_world;
	float distance = length(offset);
	vec3 direction = offset / distance;
	float sin_elevation = clamp(dot(direction, refraction_up), -1.0, 1.0);
	float lift = refraction_lift_rad(asin(sin_elevation));
	vec3 towards_up = refraction_up - sin_elevation * direction;  // in the vertical plane, normal to the direction
	float towards_up_length = length(towards_up);
	if (lift == 0.0 || towards_up_length < 1.0e-6) {
		return world_point;
	}
	direction = cos(lift) * direction + sin(lift) * towards_up / towards_up_length;
	return camera_world + direction * distance;
}
