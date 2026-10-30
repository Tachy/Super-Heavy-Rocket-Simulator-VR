// The bake of the shapes: the common voxel helpers, the band kernel, the density (A) and the occupancy (O) bakes
// (cloud_baked.gdshaderinc).

// ---- The bake ------------------------------------------------------------------------------------------------------
#ifdef CLOUD_BAKED_BUILD

// The camera offset (Earth-local, m) of grid point (x, y) (m) at model height h.
vec3 cloud_baked_offset_of(int si, float x, float y, float h) {
	float r = CLOUD_A_M + h;
	vec3 q = x * baked.sets[si].axis_x.xyz + y * baked.sets[si].axis_y.xyz + sqrt(max(r * r - x * x - y * y, 0.0)) * baked.sets[si].up0.xyz;
	return q / ATM_STRETCH - (atmosphere_camera_hi + atmosphere_camera_lo);
}

// The voxel of this invocation in level baked.build.x: its cell (i, j, layer) and texel; false outside the level.
bool cloud_baked_voxel(int si, out ivec3 cell, out ivec3 texel) {
	int k = baked.build.x;
	texel = ivec3(gl_GlobalInvocationID.xy, int(gl_GlobalInvocationID.z) + baked.build.y);
	int n = int(baked.sets[si].up0.w);
	int nz = int(round((baked.sets[si].band.y - baked.sets[si].band.x) / baked.sets[si].dz[k >> 2][k & 3]));
	cell = ivec3(texel.xy, texel.z);
	texel.z += int(baked.sets[si].slab[k >> 2][k & 3]) + 1;
	return texel.x < n && texel.y < n && cell.z < nz && int(gl_GlobalInvocationID.z) < baked.build.z;
}

// The grid point (m) and height of a cell's centre in level k.
vec3 cloud_baked_cell_centre(int si, int k, ivec3 cell) {
	float c = baked.sets[si].axis_x.w * exp2(float(k));
	float dz = baked.sets[si].dz[k >> 2][k & 3];
	return vec3((vec2(cell.xy) + 0.5 - 0.5 * baked.sets[si].up0.w) * c, baked.sets[si].band.x + (float(cell.z) + 0.5) * dz);
}
#endif

#ifdef CLOUD_BAKED_BAND
layout(set = 1, binding = 1, std430) buffer CloudBakedBand {
	int lowest;   // the lowest field layer with cloud (atomicMin; 32: none)
	int highest;  // the highest (atomicMax; -1: none)
} baked_band;
// Columns per axis of the scan over the outermost level (the field's 0.25 deg cells are ~28 km: a column every ~13 km).
const int CLOUD_BAKED_BAND_COLUMNS = 64;

void main() {
	const int si = 0;
	ivec2 id = ivec2(gl_GlobalInvocationID.xy);
	if (id.x >= CLOUD_BAKED_BAND_COLUMNS || id.y >= CLOUD_BAKED_BAND_COLUMNS) {
		return;
	}
	float half_size = 0.5 * baked.sets[si].up0.w * baked.sets[si].axis_x.w * exp2(baked.sets[si].axis_y.w - 1.0);
	vec2 xy = ((vec2(id) + 0.5) / float(CLOUD_BAKED_BAND_COLUMNS) * 2.0 - 1.0) * half_size;
	for (int layer = 0; layer < int(CLOUD_LAYERS); layer++) {
		float h = cloud_layer_centre_m(float(layer));
		vec3 offset = cloud_baked_offset_of(si, xy.x, xy.y, h);
		CloudCell cell = cloud_field(cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset), h);
		if (cell.extinction > 0.0) {
			atomicMin(baked_band.lowest, layer);
			atomicMax(baked_band.highest, layer);
		}
	}
}
#endif

#ifdef CLOUD_BAKED_DENSITY
layout(rg16f, set = 1, binding = 1) uniform writeonly image3D baked_a_out;

// The old march's soft-edge ramp at camera + offset (cloud_extinction_in_cell's terms, detail band-limited at
// footprint_m): x the ramp sum (types mixed by their weights), y the largest signed edge coordinate of the types
// present, z the extinction scale (1/m), w 1 where the field has cloud here. extra_state: detail levels the march adds
// per sample (CLOUD_BAKED_DETAIL: level 1 in level 0) - the threshold and the edge mean are those of the state with
// them (their erosion is the march's), so the march's sum keeps the cloud fraction (fix 1: with the bake's own state's
// threshold the march's erosion shrank every near outline by tens of metres and tore its rim into fragments).
vec4 cloud_baked_ramp_at(vec3 offset, float h, float footprint_m, float extra_state) {
	vec2 lat_lon = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset);
	CloudCell cell = cloud_field(lat_lon, h);
	if (cell.fraction < CLOUD_MIN_FRACTION || cell.extinction <= 0.0) {
		return vec4(0.0, -CLOUD_BAKED_X_MAX, 0.0, 0.0);
	}
	float state = cloud_detail_state(footprint_m);
	float state_t = min(state + extra_state, float(cloud_detail_levels));
	float threshold = cloud_threshold(cell.fraction, state_t);
	vec3 types = cloud_type_weights(cell);
	float detail = state > 0.0 ? cloud_detail_perturbation(offset, state) : 0.0;
	float ramp = 0.0;
	float x_max = -CLOUD_BAKED_X_MAX;
	for (int type = 0; type < 3; type++) {
		if (types[type] > 0.0) {
			vec4 shape = texture(cloud_noise_shape, cloud_type_shape_coordinates(type, offset, h));
			float w = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, clamp(shape.a, 0.0, 1.0));
			float x = (shape.r + detail - threshold) / max(w, 1.0e-6);
			ramp += types[type] * cloud_baked_ramp(x);
			x_max = max(x_max, x);
		}
	}
	float m = texture(cloud_edge_mean, cloud_table_uv(cell.fraction, state_t)).r;
	return vec4(ramp, x_max, cell.extinction / max(m, 1.0e-4), 1.0);
}

void main() {
	const int si = 0;
	ivec3 cell;
	ivec3 texel;
	if (!cloud_baked_voxel(si, cell, texel)) {
		return;
	}
	int k = baked.build.x;
	vec3 g = cloud_baked_cell_centre(si, k, cell);
	float c = baked.sets[si].axis_x.w * exp2(float(k));
	float dz = baked.sets[si].dz[k >> 2][k & 3];
	// The voxel's box average of the ramp (its coverage kept, not a mean field value): taps spaced at most about half
	// a shape texel - horizontally 125 m (cumulus texels 250 m), vertically 60 m (125 m), 20 m where the field holds
	// ice (cirrus texels 16 m high) - capped. A voxel whose field has no cloud at its bottom, middle and top is clear.
	vec3 offset_c = cloud_baked_offset_of(si, g.x, g.y, g.z);
	vec2 lat_lon_c = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_c);
	CloudCell lo_cell = cloud_field(lat_lon_c, g.z - 0.5 * dz);
	CloudCell mid_cell = cloud_field(lat_lon_c, g.z);
	CloudCell hi_cell = cloud_field(lat_lon_c, g.z + 0.5 * dz);
	if (lo_cell.extinction <= 0.0 && mid_cell.extinction <= 0.0 && hi_cell.extinction <= 0.0) {
		imageStore(baked_a_out, texel, vec4(-CLOUD_BAKED_X_MAX, 0.0, 0.0, 0.0));
		return;
	}
	bool ice = max(max(lo_cell.ice, mid_cell.ice), hi_cell.ice) > 0.0;
	int taps_h = clamp(int(ceil(c / 125.0)), 1, 3);
	int taps_v = clamp(int(ceil(dz / (ice ? 20.0 : 60.0))), 1, ice ? 16 : 8);
	float ramp = 0.0;
	float x_max = -CLOUD_BAKED_X_MAX;
	float scale = 0.0;
	float scale_n = 0.0;
	// The march's per-sample detail level (level 1 in level 0): its full weight (the march's at any footprint it has
	// in level 0, <= ~13 m at divisor 2), counted in the threshold's state.
#ifdef CLOUD_BAKED_DETAIL
	float extra_state = k == 0 && cloud_detail_levels > 1 ? 1.0 : 0.0;
#else
	const float extra_state = 0.0;
#endif
	for (int kz = 0; kz < taps_v; kz++) {
		for (int ky = 0; ky < taps_h; ky++) {
			for (int kx = 0; kx < taps_h; kx++) {
				vec3 f = (vec3(kx, ky, kz) + 0.5) / vec3(taps_h, taps_h, taps_v) - 0.5;
				vec3 p = g + f * vec3(c, c, dz);
				vec4 r = cloud_baked_ramp_at(cloud_baked_offset_of(si, p.x, p.y, p.z), p.z, 0.5 * c, extra_state);
				ramp += r.x;
				x_max = max(x_max, r.y);
				scale += r.z;
				scale_n += r.w;
			}
		}
	}
	float n = float(taps_h * taps_h * taps_v);
	ramp /= n;
	// The signed coordinate: inside the ramp's range its inverse (the ramp sum is what the march must reproduce), at
	// the ends the largest type's own (continuous: the distance to the edge for the interpolation).
	float x = ramp <= 0.0 ? min(x_max, 0.0) : (ramp >= 1.0 ? max(x_max, 1.0) : cloud_baked_unramp(ramp));
	// The scale: the mean over the taps with field cloud (the clear taps lower the ramp's mean instead).
	float scale_mean = scale_n > 0.0 ? scale / scale_n : 0.0;
	imageStore(baked_a_out, texel, vec4(clamp(x, -CLOUD_BAKED_X_MAX, CLOUD_BAKED_X_MAX),
			scale_mean * CLOUD_BAKED_PER_KM, 0.0, 0.0));
}
#endif

#ifdef CLOUD_BAKED_OCCUPANCY
layout(r8, set = 1, binding = 1) uniform writeonly image3D baked_o_out;
layout(set = 1, binding = 2) uniform sampler3D baked_a_in;  // nearest

void main() {
	const int si = 0;
	int k = baked.build.x;
	int n = int(baked.sets[si].up0.w);
	int nb = n / CLOUD_BAKED_BLOCK;
	int nz = int(round((baked.sets[si].band.y - baked.sets[si].band.x) / baked.sets[si].dz[k >> 2][k & 3]));
	int nbz = int(baked.sets[si].onz[k >> 2][k & 3]);
	ivec3 b = ivec3(gl_GlobalInvocationID.xy, int(gl_GlobalInvocationID.z) + baked.build.y);
	if (b.x >= nb || b.y >= nb || b.z >= nbz || int(gl_GlobalInvocationID.z) >= baked.build.z) {
		return;
	}
	// The block's cells and one cell around (the trilinear fetch's reach): any with extinction scale > 0 and x > 0
	// (a trilinear mix can only be cloud where a corner is).
	bool occupied = false;
	ivec3 lo = b * CLOUD_BAKED_BLOCK - 1;
	ivec3 hi = b * CLOUD_BAKED_BLOCK + CLOUD_BAKED_BLOCK;
	int slab = int(baked.sets[si].slab[k >> 2][k & 3]) + 1;
	for (int z = lo.z; z <= hi.z && !occupied; z++) {
		if (z < 0 || z >= nz) {
			continue;
		}
		for (int y = max(lo.y, 0); y <= min(hi.y, n - 1) && !occupied; y++) {
			for (int x = max(lo.x, 0); x <= min(hi.x, n - 1); x++) {
				vec2 a = texelFetch(baked_a_in, ivec3(x, y, slab + z), 0).rg;
				if (a.x > 0.0 && a.y > 0.0) {
					occupied = true;
					break;
				}
			}
		}
	}
	imageStore(baked_o_out, ivec3(b.xy, int(baked.sets[si].oslab[k >> 2][k & 3]) + b.z), vec4(occupied ? 1.0 : 0.0));
}
#endif
