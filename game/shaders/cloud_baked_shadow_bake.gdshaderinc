// The bake of S, the Sun's transmittance per column of each volume level (cloud_baked.gdshaderinc).

#ifdef CLOUD_BAKED_SHADOW
// S of one volume level, a range of rows (baked.build: x the level, y the first row, z the rows). Per column the line
// toward the Sun from the band's floor to its top (at most CLOUD_BAKED_SHADOW_PATH_MAX_M) through the baked shapes -
// the extinction the march draws, in this level or the coarser one holding each point (a prefiltered shadow: the
// whole level is baked, so a coarser level's S also serves the air shadow's far, sparse samples inside the finer
// levels' extent) - and the field's grid mean beyond the levels (as the light bake's far part). The light bake's own T_s (B.x) is no shadow volume: it exists only around occupied
// blocks (clear voxels store 1) and takes the shapes over the first 6 km only (the core's light model for a cloud's
// own light); a cloud's shadow is the line through its shapes, however long.
// March: CLOUD_BAKED_SHADOW_COARSE intervals, each looked up in the occupancy blocks; an interval whose block or a
// neighbouring interval's may hold cloud is sampled CLOUD_BAKED_SHADOW_FINE times through A (midpoint rule), the
// others are clear; intervals beyond the levels take one grid-mean sample. Stops once tau > CLOUD_BAKED_SHADOW_TAU_STOP.
// A line meeting the ground before its top (the Sun below the floor's horizon) stores no cloud: that light is the
// Earth's shadow (the atmosphere's), not the clouds'.
layout(rgba16f, set = 1, binding = 1) uniform writeonly image2DArray baked_s_out;
layout(set = 1, binding = 2) uniform sampler3D baked_a_in;  // linear
layout(set = 1, binding = 3) uniform sampler3D baked_o_in;  // nearest

// [E] Coarse intervals per line (a 2 km layer at 45 deg: ~45 m apart through the 0-16.8 km ERA5 band ~190 m, below the
// occupancy block's 400 m), fine samples per interval near occupied blocks, the optical depth to stop at, the longest
// line (a grazing Sun; the light bake's CLOUD_BAKED_SUN_PATH_MAX_M).
const int CLOUD_BAKED_SHADOW_COARSE = 128;
const int CLOUD_BAKED_SHADOW_FINE = 8;
const float CLOUD_BAKED_SHADOW_TAU_STOP = 30.0;
const float CLOUD_BAKED_SHADOW_PATH_MAX_M = 300000.0;

// What the line meets at grid point (gx, gy), height h: 0 clear (an empty occupancy block, or outside the band), 1 a
// block that may hold cloud, 2 beyond the volume levels.
int cloud_baked_shadow_kind(int si, float gx, float gy, float h) {
	int levels = int(baked.sets[si].axis_y.w);
	int k = max(cloud_baked_level(si, gx, gy), baked.build.x);
	if (k >= levels) {
		return 2;
	}
	float n = baked.sets[si].up0.w;
	float c = baked.sets[si].axis_x.w * exp2(float(k));
	float dz = baked.sets[si].dz[k >> 2][k & 3];
	float z = (h - baked.sets[si].band.x) / dz;
	int nbz = int(baked.sets[si].onz[k >> 2][k & 3]);
	if (z < 0.0 || int(z) / CLOUD_BAKED_BLOCK >= nbz) {
		return 0;
	}
	int nb = int(n) / CLOUD_BAKED_BLOCK;
	ivec2 b = clamp(ivec2(floor(vec2(gx, gy) / c + 0.5 * n)) / CLOUD_BAKED_BLOCK, ivec2(0), ivec2(nb - 1));
	return texelFetch(baked_o_in, ivec3(b, int(baked.sets[si].oslab[k >> 2][k & 3]) + int(z) / CLOUD_BAKED_BLOCK), 0).r
			> 0.5 ? 1 : 0;
}

// The extinction (1/m) the line meets at camera + offset (grid point gx, gy; height h): the baked shapes in the finest
// level holding the point, the grid mean beyond the levels.
float cloud_baked_shadow_beta(int si, vec3 offset, float gx, float gy, float h) {
	int k = max(cloud_baked_level(si, gx, gy), baked.build.x);
	if (k >= int(baked.sets[si].axis_y.w)) {
		return cloud_mean_extinction_at(offset, h);
	}
	return cloud_baked_extinction(texture(baked_a_in, cloud_baked_uvw(si, k, gx, gy, h)).rg);
}

void main() {
	const int si = 0;
	int k = baked.build.x;
	int ns = int(baked.sets[si].up0.w) * CLOUD_BAKED_SHADOW_SUPERSAMPLE;
	ivec2 col = ivec2(gl_GlobalInvocationID.x, int(gl_GlobalInvocationID.y) + baked.build.y);
	if (col.x >= ns || col.y >= ns || int(gl_GlobalInvocationID.y) >= baked.build.z) {
		return;
	}
	ivec3 texel = ivec3(col, k);
	float c = baked.sets[si].axis_x.w * exp2(float(k)) / float(CLOUD_BAKED_SHADOW_SUPERSAMPLE);
	vec2 g = (vec2(col) + 0.5 - 0.5 * float(ns)) * c;
	float z_lo = baked.sets[si].band.x;
	vec3 offset = cloud_baked_offset_of(si, g.x, g.y, z_lo);
	vec3 sun = normalize(atmosphere_sun_direction);
	CloudRay ray = cloud_baked_point_ray(offset, sun);
	float length_m = min(max(cloud_sphere_crossing(ray, baked.sets[si].band.y).y, 0.0), CLOUD_BAKED_SHADOW_PATH_MAX_M);
	vec2 ground = cloud_sphere_crossing(ray, -1.0);
	if (length_m <= 0.0 || (ground.x <= ground.y && ground.x > 0.0 && ground.x < length_m)) {
		imageStore(baked_s_out, texel, vec4(1.0, 0.0, 0.0, 0.0));
		return;
	}
	float gdx = dot(sun, baked.sets[si].axis_x.xyz * ATM_STRETCH);
	float gdy = dot(sun, baked.sets[si].axis_y.xyz * ATM_STRETCH);
	float dt = length_m / float(CLOUD_BAKED_SHADOW_COARSE);
	float tau = 0.0;
	float h_lo = 1.0e9;
	float h_hi = -1.0e9;
	int kind_prev = 0;
	float t_c = 0.5 * dt;
	int kind = cloud_baked_shadow_kind(si, g.x + t_c * gdx, g.y + t_c * gdy, cloud_height_along(ray, t_c));
	for (int i = 0; i < CLOUD_BAKED_SHADOW_COARSE; i++) {
		int kind_next = 0;
		if (i + 1 < CLOUD_BAKED_SHADOW_COARSE) {
			float t_n = (float(i) + 1.5) * dt;
			kind_next = cloud_baked_shadow_kind(si, g.x + t_n * gdx, g.y + t_n * gdy, cloud_height_along(ray, t_n));
		}
		if (kind == 2) {
			float t = (float(i) + 0.5) * dt;
			float h = cloud_height_along(ray, t);
			float beta = cloud_mean_extinction_at(offset + sun * t, h);
			if (beta > 0.0) {
				tau += beta * dt;
				h_lo = min(h_lo, h);
				h_hi = max(h_hi, h);
			}
		} else if (kind_prev == 1 || kind == 1 || kind_next == 1) {
			float step_m = dt / float(CLOUD_BAKED_SHADOW_FINE);
			for (int j = 0; j < CLOUD_BAKED_SHADOW_FINE; j++) {
				float t = float(i) * dt + (float(j) + 0.5) * step_m;
				float h = cloud_height_along(ray, t);
				float beta = cloud_baked_shadow_beta(si, offset + sun * t, g.x + t * gdx, g.y + t * gdy, h);
				if (beta > 0.0) {
					tau += beta * step_m;
					h_lo = min(h_lo, h);
					h_hi = max(h_hi, h);
				}
			}
		}
		if (tau > CLOUD_BAKED_SHADOW_TAU_STOP) {
			break;
		}
		kind_prev = kind;
		kind = kind_next;
	}
	if (tau <= 0.0) {
		imageStore(baked_s_out, texel, vec4(1.0, 0.0, 0.0, 0.0));
		return;
	}
	float tau_c = min(tau, CLOUD_BAKED_SHADOW_TAU_MAX);
	imageStore(baked_s_out, texel, vec4(exp(-tau), tau_c, tau_c * h_lo / 1000.0, tau_c * h_hi / 1000.0));
}
#endif
