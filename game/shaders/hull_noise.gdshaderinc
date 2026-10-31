// Helpers of the hull shader. The heat shield tiles are no longer computed here: they come from the baked, mipmapped maps of
// tools/structures/hull_tiles.py (vehicle_hull.gdshader samples them). What stays is the hash / value noise of the steel's
// per-ring tint and large-scale mottle.

// Smooth value noise in 3D (dents, tints), 0 .. 1.
float hull_hash3(vec3 p) {
	p = fract(p * 0.1031);
	p += dot(p, p.zyx + 31.32);
	return fract((p.x + p.y) * p.z);
}

float hull_noise3(vec3 x) {
	vec3 i = floor(x);
	vec3 f = fract(x);
	f = f * f * (3.0 - 2.0 * f);
	float n000 = hull_hash3(i);
	float n100 = hull_hash3(i + vec3(1, 0, 0));
	float n010 = hull_hash3(i + vec3(0, 1, 0));
	float n110 = hull_hash3(i + vec3(1, 1, 0));
	float n001 = hull_hash3(i + vec3(0, 0, 1));
	float n101 = hull_hash3(i + vec3(1, 0, 1));
	float n011 = hull_hash3(i + vec3(0, 1, 1));
	float n111 = hull_hash3(i + vec3(1, 1, 1));
	return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
			mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y), f.z);
}
