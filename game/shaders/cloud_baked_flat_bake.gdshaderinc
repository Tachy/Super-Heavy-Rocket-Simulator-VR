// The bake of the flat levels' columns (F, T: pattern, cover, depth, heights; cloud_baked.gdshaderinc).

#ifdef CLOUD_BAKED_FLAT
// [E] The flat geometry's shape taps per column (the bake's cost: from orbit every level holds ~2 / pixel angle columns
// across): 3 x 3 at one field point up to CLOUD_BAKED_FLAT_TAP_CELLS_M.y of cell (CloudBaked FLAT_TAP_CELLS_M; 2 x 2
// left a deck mottled at 800 m cells: the 250 m shape texels undersampled), beyond 4 x 4 at 2 x 2 field points (the
// field's 0.25 deg cells: 28 km). (.x: unused.)
const vec2 CLOUD_BAKED_FLAT_TAP_CELLS_M = vec2(1600.0, 14000.0);
// [E] The flat geometry's height range (CloudBaked FLAT_HEIGHT_RANGE) and the pixel angle's (a re-bake beyond either):
// a level's columns are baked only where some camera in that range (and within the drift) can need them.
const float CLOUD_BAKED_FLAT_HEIGHT_RANGE = 1.25;
const float CLOUD_BAKED_FLAT_PIXEL_RANGE = 1.25;
// [E] Octaves of margin of that rule (the band blends a level in from 1 octave below it at most).
const float CLOUD_BAKED_FLAT_SKIP_MARGIN = 0.5;

// The flat level's column (absolute level m = m0 + j) at grid g (m, azimuthal equidistant): false when no camera of the
// geometry's height range and drift can fetch it (beyond the cap; or inside the finer level's interior while the
// footprint there stays at least CLOUD_BAKED_FLAT_SKIP_MARGIN below where this level blends in) - written empty.
bool cloud_baked_flat_needed(int si, int j, vec2 g, float c) {
	if ((length(g) - 0.71 * c) / CLOUD_A_M > baked.sets[si].outer.y) {
		return false;
	}
	if (j == 0) {
		return true;
	}
	float n = baked.sets[si].flat_up0.w;
	float inner_finer = (0.5 * n - 1.0) * 0.5 * c;  // the finer level's interior
	if (max(abs(g.x), abs(g.y)) + 2.0 * c >= inner_finer) {
		return true;
	}
	// The largest footprint any camera of the range can fetch the column with: the farthest camera (the highest, the
	// drift away) at the most grazing view (the lowest, the drift toward; the fetch's 1 / sqrt(mu), mu >= 0.04).
	float a = CLOUD_A_M;
	float h_b = baked.sets[si].flat_axis_y.w;
	float drift = baked.sets[si].flat_anchor.w;
	float r = length(g);
	float h_hi = h_b * CLOUD_BAKED_FLAT_HEIGHT_RANGE + 2000.0;
	float h_lo = max(h_b / CLOUD_BAKED_FLAT_HEIGHT_RANGE - 2000.0, 0.0);
	float th_far = (r + drift) / a;
	float d_max = length(vec2((a + h_hi) * sin(th_far), (a + h_hi) * cos(th_far) - a));
	float th_near = max(r - drift, 0.0) / a;
	vec2 v = vec2((a + h_lo) * sin(th_near), (a + h_lo) * cos(th_near) - (a + CLOUD_TOP_M));
	float mu = max(v.y / max(length(v), 1.0), 0.04);
	float footprint = CLOUD_BAKED_FLAT_PIXEL_RANGE * baked.sets[si].flat_axis_x.w * d_max / sqrt(mu);
	float l = log2(max(footprint, 1.0e-3) / baked.sets[si].axis_x.w);
	float m = baked.sets[si].outer.z + float(j);
	return l >= m - 2.0 - CLOUD_BAKED_FLAT_SKIP_MARGIN;
}
#endif

#ifdef CLOUD_BAKED_FLAT
// The flat levels' geometry (world-fixed: shared by the sets until the visible cap, the height range or the weather
// changes; step 3, user ruling 2026-10-29: the old whole-Earth far field's cover is the reference). The cell's mean is
// the far field's column rule (cloud_far.gdshaderinc cloud_far_layer): the field's column (cloud_column_mean_depth: the
// grid-mean vertical depth tau and the largest cloud fraction f) at the cell's field points, the cover C_col =
// cloud_beam_cover_of(f, 1) (the march takes it to the view's slant as the far field does), the in-cloud depth
// tau / C_col - thin cloud translucent, haze invisible. The shapes only place that cover inside the cell: the pattern P
// is the share of the cell's taps where the shapes' prefiltered value (at the taps' spacing) exceeds the prefilter
// threshold of C_col (the old far tier's cloud_far_cover: the pattern covers the share C_col of the layer, so its mean
// is C_col); from CLOUD_BAKED_FLAT_PATTERN_CELLS_M.x to .y of cell P fades to C_col itself (the far field's even
// cover: the 32 km tile's period drew a lattice of dots across 12.8 km cells).
//   F  x P, y the grid-mean depth tau, z / w the cloud top / base x P (premultiplied by P: the bilinear fetch of a clear
//      neighbour does not lower them)
//   T  x C_col, y the cloud top (the light kernel's: its divisor and view point where P = 0)
// (Step 2 / early step 3 took the cover from the shape taps themselves, the union over all layers with any ramp > 0:
// cirrus and thin edges counted as cover - too much white against the far field; baked-outer2-report.md.)
layout(rgba16f, set = 1, binding = 1) uniform writeonly image2DArray baked_f_out;
layout(rg16f, set = 1, binding = 2) uniform writeonly image2DArray baked_t_out;

// [E] From this cell on (m) every second pattern tap (a checkerboard) reads the shape unaligned - turned and scaled as
// the old far tier's second reading (cloud_pass.gdshaderinc CLOUD_FAR_COVER_UNALIGNED_*, not part of the bake kernels'
// source): over the hundreds of km of an orbit view the 32 km tile's period drew a regular lattice of dots.
const float CLOUD_BAKED_FLAT_UNALIGNED_CELL_M = 1500.0;
const mat3 CLOUD_BAKED_FLAT_UNALIGNED_ROTATION = mat3(0.8130186879, 0.5112918472, -0.2785341274, -0.4537591358,
		0.8561682215, 0.2471408976, 0.3648331945, -0.0745427634, 0.9280841107);
const float CLOUD_BAKED_FLAT_UNALIGNED_SCALE = 1.37;
// [E] The taps' offset in a cell: a smooth field (wavelength CLOUD_BAKED_FLAT_JITTER_M, amplitude
// CLOUD_BAKED_FLAT_JITTER of the tap spacing) - neighbouring cells sample the same relative positions (a per-cell
// random jitter gave each cell a random cover: salt noise), while over the hundreds of km of an orbit view the taps
// drift against the 32 km shape tile (regular taps aliased it into a lattice of holes).
const float CLOUD_BAKED_FLAT_JITTER_M = 37000.0;
const float CLOUD_BAKED_FLAT_JITTER = 0.4;
// [E] The pattern's fade to the even cover over the cell (m; above).
const vec2 CLOUD_BAKED_FLAT_PATTERN_CELLS_M = vec2(6400.0, 12800.0);
// [E] A pattern tap's step softened over this much of the prefiltered shape value (no single-tap speckle).
const float CLOUD_BAKED_FLAT_TAP_SOFT = 0.03;
// The height of a column whose bilinear profile has no cloud while its B-spline has (m; a trace at the edge).
const float CLOUD_BAKED_FLAT_DEFAULT_HEIGHT_M = 2000.0;

#ifdef CLOUD_BAKED_FLAT_TAP_STATS
// (diagnostics: step 2's shape-tap cover of the cell - F = (its cover, the share of it from taps of nadir opacity
// < 0.1, < 0.3, its mean opacity) - the march shows it, CLOUD_BAKED_FLAT_TAP_STATS there too)
vec4 cloud_baked_flat_tap_stats(int si, vec2 g, float c, ivec2 cell) {
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float state = cloud_detail_state(0.5 * c);
	const int taps = 3;
	float cover = 0.0;
	float thin1 = 0.0;
	float thin3 = 0.0;
	float opacity = 0.0;
	for (int t = 0; t < taps * taps; t++) {
		vec2 u = (vec2(t % taps, t / taps) + 0.5) / float(taps) - 0.5;
		vec3 radial = cloud_baked_flat_radial(si, g + u * c) / ATM_STRETCH;
		vec2 lat_lon = cloud_lat_lon(CLOUD_A_M * radial);
		float x_max = -CLOUD_BAKED_X_MAX;
		float tau = 0.0;
		for (int layer = 0; layer < int(CLOUD_LAYERS); layer++) {
			float h_c = cloud_layer_centre_m(float(layer));
			CloudCell cl = cloud_field(lat_lon, h_c);
			if (cl.fraction < CLOUD_MIN_FRACTION || cl.extinction <= 0.0) {
				continue;
			}
			float bottom = cloud_layer_bottom_m(float(layer));
			float thick = cloud_layer_bottom_m(float(layer) + 1.0) - bottom;
			float threshold = cloud_threshold(cl.fraction, state);
			float scale = cl.extinction / max(texture(cloud_edge_mean, cloud_table_uv(cl.fraction, state)).r, 1.0e-4);
			vec3 types = cloud_type_weights(cl);
			vec3 offset = CLOUD_A_M * radial - camera + h_c * radial;
			float ramp = 0.0;
			for (int type = 0; type < 3; type++) {
				if (types[type] > 0.0) {
					vec4 shape = texture(cloud_noise_shape, cloud_type_shape_coordinates(type, offset, h_c));
					float w = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, clamp(shape.a, 0.0, 1.0));
					float x = (shape.r - threshold) / max(w, 1.0e-6);
					ramp += types[type] * cloud_baked_ramp(x);
					x_max = max(x_max, x);
				}
			}
			tau += scale * ramp * thick;
		}
		float r = tau > 0.0 ? cloud_baked_ramp(x_max) : 0.0;
		if (r > 0.0) {
			float a = 1.0 - exp(-tau / r);
			cover += r;
			thin1 += a < 0.1 ? r : 0.0;
			thin3 += a < 0.3 ? r : 0.0;
			opacity += r * a;
		}
	}
	float nt = float(taps * taps);
	return cover > 0.0 ? vec4(cover / nt, thin1 / cover, thin3 / cover, opacity / nt) : vec4(0.0);
}
#endif

void main() {
	const int si = 0;
	int j = baked.build.x;  // the flat level (0: level m0)
	int n = int(baked.sets[si].flat_up0.w);
	ivec2 cell = ivec2(gl_GlobalInvocationID.x, int(gl_GlobalInvocationID.y) + baked.build.y);
	if (cell.x >= n || cell.y >= n || int(gl_GlobalInvocationID.y) >= baked.build.z) {
		return;
	}
	ivec3 texel = ivec3(cell, j);
	float c = baked.sets[si].axis_x.w * exp2(baked.sets[si].outer.z + float(j));
	vec2 g = (vec2(cell) + 0.5 - 0.5 * float(n)) * c;
	if (!cloud_baked_flat_needed(si, j, g, c)) {
		imageStore(baked_f_out, texel, vec4(0.0));
		imageStore(baked_t_out, texel, vec4(0.0));
		return;
	}
#ifdef CLOUD_BAKED_FLAT_TAP_STATS
	imageStore(baked_f_out, texel, cloud_baked_flat_tap_stats(si, g, c, cell));
	imageStore(baked_t_out, texel, vec4(1.0, 5000.0, 0.0, 0.0));
	return;
#endif
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	int fp = c <= CLOUD_BAKED_FLAT_TAP_CELLS_M.y ? 1 : 2;  // field points per axis
	// The column over the field points: the grid-mean depth, the largest fraction; the far field's heights (each
	// weighted by its point's depth: continuous where a point has no cloud); the pattern's types (the layers' depth-
	// weighted mix: no switch where another layer's fraction becomes the largest).
	vec2 column = vec2(0.0);
	vec3 heights = vec3(0.0);  // depth x (nadir height, grazing height, top)
	vec3 types = vec3(0.0);
	float type_weight = 0.0;
	for (int fy = 0; fy < fp; fy++) {
		for (int fx = 0; fx < fp; fx++) {
			vec3 radial_f = cloud_baked_flat_radial(si, g + ((vec2(fx, fy) + 0.5) / float(fp) - 0.5) * c) / ATM_STRETCH;
			vec2 lat_lon = cloud_lat_lon(CLOUD_A_M * radial_f);
			vec2 column_p = cloud_column_mean_depth_smooth(lat_lon, 0.0);  // (B-spline: no field-cell squares, step 4)
			column += column_p;
			if (!(column_p.x > 0.0)) {
				continue;
			}
			// the far field's placement (cloud_far_layer: the height where the depth from the top reaches the
			// transmittance-weighted seen depth) for a nadir and a grazing view, and the optical top
			vec2 column_b = cloud_column_mean_depth(lat_lon, 0.0);  // (the profile's own bilinear column)
			if (column_b.x > 0.0) {
				heights += column_p.x * vec3(cloud_far_depth_height(lat_lon, cloud_far_seen_depth(column_b, 1.0)),
						cloud_far_depth_height(lat_lon, cloud_far_seen_depth(column_b, CLOUD_BAKED_FLAT_GRAZING_MU)),
						cloud_far_depth_height(lat_lon, CLOUD_BAKED_FLAT_TOP_DEPTH * column_b.x));
			} else {
				heights += column_p.x * vec3(CLOUD_BAKED_FLAT_DEFAULT_HEIGHT_M);
			}
			for (int layer = 0; layer < int(CLOUD_LAYERS); layer++) {
				float h_c = cloud_layer_centre_m(float(layer));
				CloudCell cl = cloud_field(lat_lon, h_c);
				if (cl.fraction < CLOUD_MIN_FRACTION || cl.extinction <= 0.0) {
					continue;
				}
				float w = cl.extinction * (cloud_layer_bottom_m(float(layer) + 1.0) - cloud_layer_bottom_m(float(layer)));
				types += w * cloud_type_weights(cl);
				type_weight += w;
			}
		}
	}
	float points = float(fp * fp);
	float depth = column.x / points;
	float c_col = depth > 0.0 ? cloud_beam_cover_of(column.y / points, 1.0) : 0.0;
	if (!(c_col > 0.0)) {
		imageStore(baked_f_out, texel, vec4(0.0));
		imageStore(baked_t_out, texel, vec4(0.0));
		return;
	}
	heights /= column.x;
	types = type_weight > 0.0 ? types / type_weight : vec3(1.0, 0.0, 0.0);
	float h_pattern = heights.x;
	// The pattern (CLOUD_BAKED_FLAT_PATTERN_*): taps x taps over the cell, the shapes prefiltered at the taps' spacing.
	float w_pattern = 1.0 - smoothstep(CLOUD_BAKED_FLAT_PATTERN_CELLS_M.x, CLOUD_BAKED_FLAT_PATTERN_CELLS_M.y, c);
	float pattern = c_col;
#ifdef CLOUD_BAKED_DIAG_NO_BAKE_PATTERN
	w_pattern = 0.0;
#endif
	if (w_pattern > 0.0 && c_col < 1.0) {
		int taps = c <= CLOUD_BAKED_FLAT_TAP_CELLS_M.y ? 3 : 4;
		float spacing = c / float(taps);
		float level = clamp(log2(spacing * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M), 0.0, CLOUD_SHAPE_PREFILTER_LEVELS);
		float threshold = texture(cloud_prefilter_threshold, cloud_prefilter_uv(c_col, level)).r;
		float median = texture(cloud_prefilter_threshold, cloud_prefilter_uv(0.5, level)).r;
		bool unaligned = c >= CLOUD_BAKED_FLAT_UNALIGNED_CELL_M;
		float sum = 0.0;
		for (int ty = 0; ty < taps; ty++) {
			for (int tx = 0; tx < taps; tx++) {
				vec2 ph = g / CLOUD_BAKED_FLAT_JITTER_M + 0.37 * vec2(tx, ty) + 0.61 * vec2(ty, tx);
				vec2 jitter = 0.5 + CLOUD_BAKED_FLAT_JITTER * vec2(sin(6.2832 * (ph.x + 0.71 * ph.y)),
						sin(6.2832 * (ph.y - 0.53 * ph.x) + 1.7));
				vec3 radial = cloud_baked_flat_radial(si, g + ((vec2(tx, ty) + jitter) / float(taps) - 0.5) * c)
						/ ATM_STRETCH;
				vec3 offset = (CLOUD_A_M + h_pattern) * radial - camera;
				bool turned = unaligned && ((tx + ty) & 1) == 1;
				float covered = 0.0;
				for (int type = 0; type < 3; type++) {
					if (types[type] > 0.0) {
						vec3 q = cloud_type_shape_coordinates(type, offset, h_pattern);
						if (turned) {
							q = CLOUD_BAKED_FLAT_UNALIGNED_ROTATION * q / CLOUD_BAKED_FLAT_UNALIGNED_SCALE;
						}
						float s = cloud_baked_shape(q, level, cloud_baked_blend_weight(q), median).x;
						covered += types[type] * smoothstep(threshold - CLOUD_BAKED_FLAT_TAP_SOFT,
								threshold + CLOUD_BAKED_FLAT_TAP_SOFT, s);
					}
				}
				sum += covered;
			}
		}
		pattern = mix(c_col, sum / float(taps * taps), w_pattern);
	}
	// (heights in km: x the depth stays within RGBA16F's range for any column)
	imageStore(baked_f_out, texel, vec4(pattern, depth, 1.0e-3 * heights.x * depth, 1.0e-3 * heights.y * depth));
	imageStore(baked_t_out, texel, vec4(c_col, heights.z * c_col, 0.0, 0.0));
}
#endif
