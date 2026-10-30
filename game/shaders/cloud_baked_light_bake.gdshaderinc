// The bake of the light (B): the Sun's transmittance and sources per voxel (cloud_baked.gdshaderinc).

#ifdef CLOUD_BAKED_LIGHT
layout(rgba8, set = 1, binding = 1) uniform writeonly image3D baked_b_out;
layout(set = 1, binding = 2) uniform sampler3D baked_a_in;  // linear
layout(set = 1, binding = 3) uniform sampler3D baked_o_in;  // nearest

// [E] Samples of the Sun's path: CLOUD_BAKED_SUN_NEAR_SAMPLES through the baked shapes over its first
// CLOUD_BAKED_SUN_SHAPE_PATH_M (quadratic spacing: dense at the voxel, the first interval L / n^2), then
// CLOUD_BAKED_SUN_FAR_SAMPLES equal intervals through the field's grid mean to the band's top (at most
// CLOUD_BAKED_SUN_PATH_MAX_M). The split is the light model of the old march and the core
// (cloud_light_optical_depth: near_m 6 km through the shapes, the presets' light_near_m, then the grid mean): through
// the shapes all the way, a grazing Sun path (low Sun, broken cirrus: 10-100 km in the layer) found the shapes' gaps
// and the baked cirrus was 1.14 x the old march's brightness per alpha toward a 5 deg Sun (step 7).
const int CLOUD_BAKED_SUN_NEAR_SAMPLES = 24;
const int CLOUD_BAKED_SUN_FAR_SAMPLES = 8;
const float CLOUD_BAKED_SUN_SHAPE_PATH_M = 6000.0;
const float CLOUD_BAKED_SUN_PATH_MAX_M = 300000.0;

// The optical depth toward the Sun from camera + offset (height h): the baked volume's extinction where a level holds
// the point over the near part, the field's grid mean beyond; up to the band's top (0 when the path meets the ground:
// Earth's shadow is atm_sunlit's / the Sun's transmittance's).
float cloud_baked_sun_depth(int si, vec3 offset, vec3 sun) {
	vec3 axis_x = baked.sets[si].axis_x.xyz * ATM_STRETCH;
	vec3 axis_y = baked.sets[si].axis_y.xyz * ATM_STRETCH;
	CloudRay ray = cloud_ray(sun);
	// cloud_ray is the camera's ray: the start's height and up from the point instead
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset;
	vec3 pm = p * ATM_STRETCH;
	ray.r_c = length(pm);
	ray.h_c = ray.r_c - CLOUD_A_M;
	ray.mu = dot(pm / ray.r_c, sun * ATM_STRETCH / ray.scale);
	vec2 top = cloud_sphere_crossing(ray, baked.sets[si].band.y);
	float length_m = min(max(top.y, 0.0), CLOUD_BAKED_SUN_PATH_MAX_M);
	if (length_m <= 0.0) {
		return 0.0;
	}
	vec2 ground = cloud_sphere_crossing(ray, 0.0);
	if (ground.x <= ground.y && ground.x > 0.0 && ground.x < length_m) {
		return 1.0e4;  // below the horizon: no direct Sun (the light is the sky's)
	}
	float gx0 = baked.sets[si].band.z + dot(offset, axis_x);
	float gy0 = baked.sets[si].band.w + dot(offset, axis_y);
	float gdx = dot(sun, axis_x);
	float gdy = dot(sun, axis_y);
	float tau = 0.0;
	float near_m = min(length_m, CLOUD_BAKED_SUN_SHAPE_PATH_M);
	float n = float(CLOUD_BAKED_SUN_NEAR_SAMPLES);
	for (int i = 0; i < CLOUD_BAKED_SUN_NEAR_SAMPLES; i++) {
		float a = near_m * float(i * i) / (n * n);
		float b = near_m * float((i + 1) * (i + 1)) / (n * n);
		float t = 0.5 * (a + b);
		float h = cloud_height_along(ray, t);
		float gx = gx0 + t * gdx;
		float gy = gy0 + t * gdy;
		int k = max(cloud_baked_level(si, gx, gy), int(baked.sets[si].misc.z));
		float beta;
		if (k < int(baked.sets[si].axis_y.w)) {
			beta = cloud_baked_extinction(texture(baked_a_in, cloud_baked_uvw(si, k, gx, gy, h)).rg);
		} else {
			beta = cloud_mean_extinction_at(offset + sun * t, h);
		}
		tau += beta * (b - a);
	}
	if (length_m > near_m) {
		float step_m = (length_m - near_m) / float(CLOUD_BAKED_SUN_FAR_SAMPLES);
		for (int i = 0; i < CLOUD_BAKED_SUN_FAR_SAMPLES; i++) {
			float t = near_m + (float(i) + 0.5) * step_m;
			tau += cloud_mean_extinction_at(offset + sun * t, cloud_height_along(ray, t)) * step_m;
		}
	}
	return tau;
}

void main() {
	const int si = 0;
	ivec3 cell;
	ivec3 texel;
	if (!cloud_baked_voxel(si, cell, texel)) {
		return;
	}
	int k = baked.build.x;
	// Voxels outside every occupied block (and its neighbours') are never read where there is cloud: no light.
	ivec3 b = cell / CLOUD_BAKED_BLOCK;
	int nb = int(baked.sets[si].up0.w) / CLOUD_BAKED_BLOCK;
	int nbz = int(baked.sets[si].onz[k >> 2][k & 3]);
	bool near_cloud = false;
	for (int z = max(b.z - 1, 0); z <= min(b.z + 1, nbz - 1); z++) {
		for (int y = max(b.y - 1, 0); y <= min(b.y + 1, nb - 1); y++) {
			for (int x = max(b.x - 1, 0); x <= min(b.x + 1, nb - 1); x++) {
				near_cloud = near_cloud || texelFetch(baked_o_in, ivec3(x, y, int(baked.sets[si].oslab[k >> 2][k & 3]) + z), 0).r > 0.5;
			}
		}
	}
	if (!near_cloud) {
		imageStore(baked_b_out, texel, vec4(1.0, 0.0, 0.0, 0.0));
		return;
	}
	vec3 g = cloud_baked_cell_centre(si, k, cell);
	vec3 offset = cloud_baked_offset_of(si, g.x, g.y, g.z);
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset;
	vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);  // the ellipsoid's normal (geodetic up)
	vec3 sun = normalize(atmosphere_sun_direction);
	float tau_s = cloud_baked_sun_depth(si, offset, sun);
	// The column terms (the old march's: the field's grid-mean columns) and the view from the anchor.
	vec2 lat_lon = cloud_lat_lon(p);
	CloudCell c = cloud_field(lat_lon, g.z);
	float in_cloud = c.fraction >= CLOUD_MIN_FRACTION ? c.extinction / c.fraction : 0.0;
	vec2 column = cloud_column_depths(c, lat_lon, g.z, in_cloud);
	vec3 to_anchor = baked.sets[si].anchor.xyz - offset;
	vec3 view_prop = length(to_anchor) > 1.0 ? normalize(to_anchor) : normal;
	vec3 diff = sun + view_prop;
	float one_minus_cos = 0.5 * dot(diff, diff);
	float phase = cloud_phase_function(1.0 - one_minus_cos, one_minus_cos);
	float transmittance;
	float j_sun;
	if (baked.build.w == 1) {
		transmittance = exp(-tau_s);
		j_sun = cloud_scattering_source_mc(sun, view_prop, normal, tau_s, column.x, column.y);
	} else {
		transmittance = exp(-cloud_transport_scale() * tau_s);
		j_sun = cloud_scattering_source_3d(sun, view_prop, normal, tau_s, column.x, column.y);
	}
	float j_ms = max(j_sun - phase * transmittance, 0.0);
	float j_sky = cloud_sky_scattering_source(dot(view_prop, normal), column.x, column.y);
	// The edge width noise (the shapes' A channel, types mixed) for the march's per-sample detail.
	vec3 types = cloud_type_weights(c);
	float width_noise = 0.0;
	for (int type = 0; type < 3; type++) {
		if (types[type] > 0.0) {
			width_noise += types[type] * texture(cloud_noise_shape, cloud_type_shape_coordinates(type, offset, g.z)).a;
		}
	}
	imageStore(baked_b_out, texel, vec4(sqrt(clamp(transmittance, 0.0, 1.0)),
			sqrt(clamp(j_ms / CLOUD_BAKED_J_MAX, 0.0, 1.0)), sqrt(clamp(j_sky / CLOUD_BAKED_J_MAX, 0.0, 1.0)),
			clamp(width_noise, 0.0, 1.0)));
}
#endif
