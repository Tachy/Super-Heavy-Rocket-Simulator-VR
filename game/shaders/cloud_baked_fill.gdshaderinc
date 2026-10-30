// The readers of S on the surfaces' side: the Sun probe (the vehicles' and the eye's direct Sun) and the cloud
// shadow maps' fill (CloudShadowEffect; cloud_baked.gdshaderinc).

// ---- The vehicles' and the eye's direct Sun from the bake (CloudShadowEffect's probe, 2026-10-30) ----------------
#if defined(CLOUD_SHADOW_BAKED_FILL) && defined(CLOUD_BAKED_SUN_PROBE)
// One point per invocation (one work group of 64): the Sun's cloud transmittance at camera + the point's offset from
// S, as the air shadow reads it (cloud_baked_air_sun, at the finest level holding the point); -1 beyond the volume
// levels (the caller takes the CPU's grid mean there); 1 above the cloud shell.
layout(std430, set = 1, binding = 0) readonly buffer CloudSunProbeIn {
	vec4 sun;       // xyz the Sun direction (Earth-local, unit), w 1: a cross-fade's second set is bound
	vec4 count;     // x the points
	vec4 points[];  // xyz the point's offset from the camera (Earth-local render axes, m)
} probe_in;
layout(std430, set = 1, binding = 1) writeonly buffer CloudSunProbeOut {
	float t[];
} probe_out;

void main() {
	uint i = gl_LocalInvocationIndex;
	if (i >= uint(probe_in.count.x)) {
		return;
	}
	vec3 offset = probe_in.points[i].xyz;
	vec3 s = probe_in.sun.xyz;
	float h = cloud_height_of(atmosphere_camera_hi + atmosphere_camera_lo + offset);
	if (h > CLOUD_TOP_M) {
		probe_out.t[i] = 1.0;
		return;
	}
	vec4 line;
	bool inside = cloud_baked_shadow_line(0, offset, s, 0.0, line);
	float t = cloud_baked_shadow_at_height(line, h);
	float w = baked.sets[0].misc.y;
	if (probe_in.sun.w > 0.5 && w > 0.0) {
		vec4 line1;
		if (cloud_baked_shadow_line(1, offset, s, 0.0, line1)) {
			float t1 = cloud_baked_shadow_at_height(line1, h);
			t = inside ? mix(t, t1, w) : t1;
			inside = true;
		}
	}
	probe_out.t[i] = inside ? t : -1.0;
}
#endif

// ---- The cloud shadow maps from the bake (CloudShadowEffect; shadow step 2026-10-30) ------------------------------
#if defined(CLOUD_SHADOW_BAKED_FILL) && !defined(CLOUD_BAKED_SUN_PROBE)
// One map texel per invocation, the texel semantics cloud_light.gdshaderinc reads (its header): along the texel's line toward the Sun (t from the map plane) the first and last t with cloud, the optical
// depth between, the cover (1: the shapes). The line is read from S once (cloud_baked_shadow_line; two sets mixed by
// the cross-fade's weight while a new set fades in); its cloudy heights give t_front / t_back. Where the line's floor
// point lies beyond the volume levels: the field's slanted column there (cloud_column_direct, as the maps' far rule).
layout(set = 1, binding = 0, std140) uniform CloudShadowFillBlock {
	vec4 anchor;  // xyz anchor - camera (m), w half extent (m)
	vec4 e1;      // xyz map axis (unit), w -
	vec4 e2;      // xyz map axis (unit), w the map's size (texels)
	vec4 sun;     // xyz the Sun direction (unit), w 1: a cross-fade's second set is bound (baked.sets[0].misc.y its weight)
} shadow;

layout(rgba32f, set = 1, binding = 1) uniform writeonly image2D out_map;

// t (m from the map plane) where the line rises through height h (its closest approach when it stays above h).
float cloud_shadow_fill_t(CloudRay ray, float h) {
	vec2 c = cloud_sphere_crossing(ray, h);
	return c.x <= c.y ? c.y : -ray.r_c * ray.mu / ray.scale;
}

#ifdef CLOUD_SHADOW_BAKED_DIAG
// Checks only (STARSIM_BAKED_DEFINES=CLOUD_SHADOW_BAKED_DIAG; render_check --shadow_dump): the texel holds instead
// x the map's transmittance for a receiver on the ground (height 0) of this line, y the reference - a dense march
// (CLOUD_SHADOW_DIAG_SAMPLES) from that ground point toward the Sun through the baked shapes (A, set 0; the grid mean
// beyond the levels), z the bake's own light T_s (B.x^2) half a level-0 cell below the line's lowest cloud (-1: no
// cloud), w the horizontal distance of the line's optical-depth-weighted cloud point from the ground point over its
// height (tan of the Sun's zenith for a straight line over a flat Earth; -1: no cloud; -2: the line serves no ground
// receiver - it passes above the ground or meets it beyond 140 km from the camera).
layout(set = 1, binding = 5) uniform sampler3D baked_a_diag;  // linear
layout(set = 1, binding = 6) uniform sampler3D baked_b_diag;  // linear
const int CLOUD_SHADOW_DIAG_SAMPLES = 2048;

float cloud_shadow_diag_beta(vec3 offset, float h) {
	vec3 axis_x = baked.sets[0].axis_x.xyz * ATM_STRETCH;
	vec3 axis_y = baked.sets[0].axis_y.xyz * ATM_STRETCH;
	float gx = baked.sets[0].band.z + dot(offset, axis_x);
	float gy = baked.sets[0].band.w + dot(offset, axis_y);
	int k = max(cloud_baked_level(0, gx, gy), int(baked.sets[0].misc.w));
	if (k >= int(baked.sets[0].axis_y.w)) {
		return cloud_mean_extinction_at(offset, h);
	}
	return cloud_baked_extinction(texture(baked_a_diag, cloud_baked_uvw(0, k, gx, gy, h)).rg);
}

vec4 cloud_shadow_fill_diag(CloudRay ray, vec3 origin, vec3 s, vec4 texel) {
	float t_g = cloud_shadow_fill_t(ray, 0.0);
	float fraction = clamp((texel.y - t_g) / max(texel.y - texel.x, 1.0e-3), 0.0, 1.0);
	float t_map = exp(-texel.z * fraction);
	vec3 g = origin + s * t_g;
	CloudRay rg = cloud_baked_point_ray(g, s);
	float length_m = min(max(cloud_sphere_crossing(rg, baked.sets[0].band.y).y, 0.0), 300000.0);
	float dt = length_m / float(CLOUD_SHADOW_DIAG_SAMPLES);
	float tau = 0.0;
	float h_lo = 1.0e9;
	float t_lo = 0.0;
	vec3 cloud_point = vec3(0.0);
	for (int i = 0; i < CLOUD_SHADOW_DIAG_SAMPLES; i++) {
		float t = (float(i) + 0.5) * dt;
		float h = cloud_height_along(rg, t);
		float beta = cloud_shadow_diag_beta(g + s * t, h);
		if (beta > 0.0) {
			tau += beta * dt;
			cloud_point += beta * dt * vec3(t, h, 0.0);
			if (h < h_lo) {
				h_lo = h;
				t_lo = t;
			}
		}
	}
	// -2: the line meets no ground receiver the map serves (none, or its ground point beyond the far map's use,
	// CloudLayer.SHADOW_FAR_FADE_M's 140 km from the camera: the lines grazing the Earth toward the terminator)
	vec2 ground_c = cloud_sphere_crossing(ray, 0.0);
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	float flag = ground_c.x <= ground_c.y && length(g - up_c * dot(g, up_c)) < 140000.0 ? -1.0 : -2.0;
	if (tau <= 0.0 || flag < -1.5) {
		return vec4(t_map, exp(-tau), -1.0, flag);
	}
	cloud_point /= tau;
	// the bake's light just below the lowest cloud on the line
	float t_b = max(t_lo - 0.5 * baked.sets[0].dz[0][0] / max(rg.mu, 0.05), 0.0);
	vec3 pb = g + s * t_b;
	float hb = cloud_height_along(rg, t_b);
	vec3 axis_x = baked.sets[0].axis_x.xyz * ATM_STRETCH;
	vec3 axis_y = baked.sets[0].axis_y.xyz * ATM_STRETCH;
	float gx = baked.sets[0].band.z + dot(pb, axis_x);
	float gy = baked.sets[0].band.w + dot(pb, axis_y);
	int k = max(cloud_baked_level(0, gx, gy), int(baked.sets[0].misc.z));
	float t_bake = -1.0;
	if (k < int(baked.sets[0].axis_y.w)) {
		float b = texture(baked_b_diag, cloud_baked_uvw(0, k, gx, gy, hb)).x;
		t_bake = b * b;
	}
	// horizontal distance of the cloud_point point from the ground point over its height
	vec3 pc = g + s * cloud_point.x;
	vec3 up = normalize((atmosphere_camera_hi + atmosphere_camera_lo + g) * ATM_STRETCH);
	vec3 d = (pc - g) * ATM_STRETCH;
	float horizontal = length(d - up * dot(d, up));
	return vec4(t_map, exp(-tau), t_bake, horizontal / max(cloud_point.y, 1.0));
}
#endif

void main() {
	int size = int(shadow.e2.w);
	ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
	if (texel.x >= size || texel.y >= size) {
		return;
	}
	float half_extent = shadow.anchor.w;
	float texel_m = 2.0 * half_extent / float(size);
	vec2 uv = (vec2(texel) + 0.5) * texel_m - half_extent;
	vec3 s = shadow.sun.xyz;
	vec3 origin = shadow.anchor.xyz + uv.x * shadow.e1.xyz + uv.y * shadow.e2.xyz;  // t = 0, from the camera
	CloudRay ray = cloud_baked_point_ray(origin, s);
	vec4 line;
	bool inside = cloud_baked_shadow_line(0, origin, s, 0.0, line);
	float w = baked.sets[0].misc.y;
	if (shadow.sun.w > 0.5 && w > 0.0) {
		vec4 line1;
		if (cloud_baked_shadow_line(1, origin, s, 0.0, line1) && inside) {
			// heights weighted by each set's optical depth (a clear set's do not count)
			float tau0 = -log(max(line.x, 1.0e-30)) * (1.0 - w);
			float tau1 = -log(max(line1.x, 1.0e-30)) * w;
			float t_mix = mix(line.x, line1.x, w);
			if (tau0 + tau1 > 0.0) {
				line.yz = (tau0 * line.yz + tau1 * line1.yz) / (tau0 + tau1);
			}
			line.x = t_mix;
		}
	}
	float od;
	float h_lo;
	float h_hi;
	if (inside) {
		if (line.x >= 1.0) {
			float t_top = cloud_shadow_fill_t(ray, baked.sets[0].band.y);
			vec4 clear = vec4(t_top, t_top, 0.0, 1.0);
#ifdef CLOUD_SHADOW_BAKED_DIAG
			clear = cloud_shadow_fill_diag(ray, origin, s, clear);
#endif
			imageStore(out_map, texel, clear);
			return;
		}
		od = -log(max(line.x, 1.0e-30));
		h_lo = line.y;
		h_hi = max(line.z, line.y + 1.0);
	} else {
		vec3 q = origin + s * line.w;
		vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + q;
		float mu0 = dot(cloud_geodetic_up(p), s);
		float analytic = cloud_column_direct(cloud_column_mean_depth(cloud_lat_lon(p), baked.sets[0].band.x), mu0);
		od = -log(max(analytic, 1.0e-30));
		h_lo = baked.sets[0].band.x;
		h_hi = baked.sets[0].band.y;
	}
	vec4 result = vec4(cloud_shadow_fill_t(ray, h_lo), cloud_shadow_fill_t(ray, h_hi), od, 1.0);
#ifdef CLOUD_SHADOW_BAKED_DIAG
	result = cloud_shadow_fill_diag(ray, origin, s, result);
#endif
	imageStore(out_map, texel, result);
}
#endif
