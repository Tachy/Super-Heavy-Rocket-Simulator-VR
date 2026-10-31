// Heat-shield tiles as an analytic function of the surface (research-models 7.3): axial hexagon cells of a 2D surface
// coordinate in metres, the distance to the cell edge for the gap, a bevel direction, a per-tile hash. Evaluated per
// pixel from the surface alone, so both eyes of a stereo frame see the same pattern; the pixel footprint `px` (metres,
// from the derivatives of the surface position) box-filters the gap and fades every detail to its mean colour as the
// cell shrinks below a few pixels (the analytic pattern's mip chain: no shimmer).

const float TILE_SQRT3 = 1.7320508;

struct Tile {
	vec2 id;        // integer-valued lattice id of the cell (exact hash input)
	vec2 offset_m;  // from the cell centre, surface metres
	float edge_m;   // distance to the nearest cell edge (m), 0 on the edge
};

// Hexagon cells with `across_flats_m` between opposite flats; flats perpendicular to the first coordinate axis.
Tile tile_cell(vec2 p_m, float across_flats_m) {
	vec2 p = p_m / across_flats_m;
	vec2 r = vec2(1.0, TILE_SQRT3);
	vec2 h = r * 0.5;
	vec2 a = mod(p, r) - h;
	vec2 b = mod(p - h, r) - h;
	bool first = dot(a, a) < dot(b, b);
	vec2 g = first ? a : b;  // from the nearest centre of the two interleaved rectangular lattices
	Tile t;
	// Lattice integers (no float difference of two large numbers): 2 x cell index, +1 on the shifted lattice.
	t.id = first ? 2.0 * floor(p / r) : 2.0 * floor((p - h) / r) + 1.0;
	t.offset_m = g * across_flats_m;
	float flats = max(abs(g.x), max(abs(dot(g, vec2(0.5, 0.5 * TILE_SQRT3))), abs(dot(g, vec2(-0.5, 0.5 * TILE_SQRT3)))));
	t.edge_m = (0.5 - flats) * across_flats_m;
	return t;
}

float tile_hash(vec2 id) {
	vec3 p = fract(vec3(id.xyx) * 0.1031);
	p += dot(p, p.yzx + 33.33);
	return fract((p.x + p.y) * p.z);
}

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

// Box-filtered coverage of a stripe of half width `half_m` at distance `d_m`, seen through a pixel of `px` metres.
float stripe_cover(float d_m, float half_m, float px) {
	return clamp((half_m - d_m) / max(px, 1e-5) + 0.5, 0.0, 1.0);
}
