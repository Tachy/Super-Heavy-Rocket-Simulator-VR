// Fraction of a light source's disk hidden by an occluding sphere, seen from the fragment.
// Mirror of core/src/environment/eclipse.cpp (Montenbruck & Gill 3.85-3.90); keep both in sync.
// Inputs are vectors FROM the observer in metres.
float occulted_fraction(vec3 to_source, float source_radius, vec3 to_occluder, float occluder_radius) {
	float source_distance = length(to_source);
	float occluder_distance = length(to_occluder);
	if (occluder_distance <= occluder_radius) {
		return 1.0;
	}
	if (occluder_distance > source_distance) {
		return 0.0;
	}
	float a = asin(min(1.0, source_radius / source_distance));
	float b = asin(occluder_radius / occluder_distance);
	vec3 s = to_source / source_distance;
	vec3 o = to_occluder / occluder_distance;
	float c = atan(length(cross(s, o)), dot(s, o));  // well conditioned for small angles
	if (c >= a + b) {
		return 0.0;
	}
	if (c <= b - a) {
		return 1.0;
	}
	if (c <= a - b) {
		return (b * b) / (a * a);
	}
	float x = (c * c + a * a - b * b) / (2.0 * c);
	float y = sqrt(max(0.0, a * a - x * x));
	float area = a * a * acos(clamp(x / a, -1.0, 1.0)) + b * b * acos(clamp((c - x) / b, -1.0, 1.0)) - c * y;
	return clamp(area / (PI * a * a), 0.0, 1.0);
}
