// The baked volume's shared shape helpers: the flat levels' azimuthal-equidistant coordinates, the edge ramp's inverse
// and the tile-breaking shape read (two readings blended, histogram-preserving) - the flat bake's and the march's
// (cloud_baked.gdshaderinc).

#if defined(CLOUD_BAKED_BUILD) || defined(CLOUD_BAKED_MARCH) || defined(CLOUD_SHADOW_BAKED_FILL)
// ---- Flat outer levels: shared ----
// The flat levels' coordinates of the Earth-local point p (from the Earth's centre): x, y = (X, Y) (m, azimuthal
// equidistant about up0), z = theta (rad) from up0.
vec3 cloud_baked_flat_xy(int si, vec3 p) {
	vec3 nrm = normalize(p * ATM_STRETCH);
	float cx = dot(nrm, baked.sets[si].flat_axis_x.xyz);
	float cy = dot(nrm, baked.sets[si].flat_axis_y.xyz);
	float s = length(vec2(cx, cy));
	float theta = atan(s, dot(nrm, baked.sets[si].flat_up0.xyz));
	float f = s > 1.0e-9 ? CLOUD_A_M * theta / s : CLOUD_A_M;
	return vec3(cx * f, cy * f, theta);
}

// The model-space unit radial of flat coordinates (X, Y) (m).
vec3 cloud_baked_flat_radial(int si, vec2 g) {
	float r = length(g);
	float theta = r / CLOUD_A_M;
	vec3 tangent = r > 1.0e-3 ? (g.x * baked.sets[si].flat_axis_x.xyz + g.y * baked.sets[si].flat_axis_y.xyz) / r
			: baked.sets[si].flat_axis_x.xyz;
	return cos(theta) * baked.sets[si].flat_up0.xyz + sin(theta) * tangent;
}

// The largest cloud fraction f whose nadir beam cover cloud_beam_cover_of(f, 1) = 1 - (1 - f)^(1 + v (1 - f)) is c
// (v CLOUD_OVERLAP_VERTICAL; fixed-point iterations, ~1e-3).
float cloud_baked_fraction_of(float c) {
	c = clamp(c, 0.0, 0.9999);
	float f = c;
	for (int i = 0; i < 4; i++) {
		f = 1.0 - pow(1.0 - c, 1.0 / (1.0 + CLOUD_OVERLAP_VERTICAL * (1.0 - f)));
	}
	return f;
}

// The edge coordinate whose ramp is r (0 < r < 1): the inverse smoothstep.
float cloud_baked_unramp(float r) {
	return 0.5 - sin(asin(1.0 - 2.0 * r) / 3.0);
}

// [E] The flat levels' tile breaking (step 5; step 4's domain warp sheared the shapes into marbled streaks): over the
// hundreds of km of an orbit view the 32 km shape tile's period drew a regular lattice of dots. The shape is read twice
// - at q and turned (CLOUD_BAKED_BLEND_ROTATION, offset) - and the two readings are blended by a large-scale weight
// (the shape's level-1 mip read at CLOUD_BAKED_BLEND_SCALE of q, mapped through its interquartile range: about a
// quarter of the area each reading alone, half a blend), histogram-preserving (Heitz & Neyret 2018: the blend's
// deviation from the median rescaled by 1 / sqrt(w^2 + (1 - w)^2)) - so the pattern's statistics, hence the prefilter
// thresholds and the mean cover, are kept, and no shear: the shapes stay isotropic.
const float CLOUD_BAKED_BLEND_SCALE = 0.1931;
const mat3 CLOUD_BAKED_BLEND_ROTATION = mat3(0.8130186879, 0.5112918472, -0.2785341274, -0.4537591358,
		0.8561682215, 0.2471408976, 0.3648331945, -0.0745427634, 0.9280841107);
const vec3 CLOUD_BAKED_BLEND_OFFSET = vec3(0.379, 0.617, 0.233);

// The flat levels' heights (step 5; step 4 entered at the column's highest cloudy layer's top: where a thin upper layer
// set in, the entry - and with it the pattern's point - jumped by kilometres along a line, a visible seam). Per column
// the far field's own placement (cloud_far_layer: cloud_far_depth_height of the seen depth) for a nadir view and for
// a view of cosine CLOUD_BAKED_FLAT_GRAZING_MU (the march interpolates in the view's cosine between them), and the
// optical top (the height where the depth from the top is CLOUD_BAKED_FLAT_TOP_DEPTH of the column's): continuous
// in the field.
const float CLOUD_BAKED_FLAT_GRAZING_MU = 0.2;
const float CLOUD_BAKED_FLAT_TOP_DEPTH = 0.05;

// The blend weight of the second reading at shape coordinate q (tile units).
float cloud_baked_blend_weight(vec3 q) {
	float n = textureLod(cloud_noise_shape_mips, q * CLOUD_BAKED_BLEND_SCALE + vec3(0.137, 0.713, 0.371), 0.0).r;
	float q25 = texture(cloud_prefilter_threshold, cloud_prefilter_uv(0.75, 1.0)).r;
	float q75 = texture(cloud_prefilter_threshold, cloud_prefilter_uv(0.25, 1.0)).r;
	return smoothstep(q25, q75, n);
}

// The shape at q (prefilter level) blended with its turned reading by weight w: x the value, y the edge-width noise.
vec2 cloud_baked_shape(vec3 q, float level, float w, float median) {
	vec4 s1 = cloud_shape_prefiltered(q, level);
	if (w <= 0.0) {
		return s1.ra;
	}
	vec4 s2 = cloud_shape_prefiltered(CLOUD_BAKED_BLEND_ROTATION * q + CLOUD_BAKED_BLEND_OFFSET, level);
	float a = 1.0 - w;
	return vec2(median + (a * (s1.r - median) + w * (s2.r - median)) * inversesqrt(a * a + w * w), mix(s1.a, s2.a, w));
}

#endif
