// The flat levels in the march: their fetches (B-spline), the level and takeover rules, the per-pixel pattern, the
// crossing of the column's height and its light, the far field beyond the cap (cloud_baked.gdshaderinc).

#ifdef CLOUD_BAKED_MARCH
// ---- The flat levels in the march ----
layout(set = 1, binding = 57) uniform sampler2DArray baked_f0;  // linear
layout(set = 1, binding = 58) uniform sampler2DArray baked_t0;  // linear (T: C_col, top)
#ifdef CLOUD_BAKED_FADE
layout(set = 1, binding = 59) uniform sampler2DArray baked_f1;
layout(set = 1, binding = 60) uniform sampler2DArray baked_t1;
#endif

// [E] Behind a volume transmittance below this the flat levels are not fetched (< 1 % of their light).
const float CLOUD_BAKED_FLAT_MIN_TRANSMITTANCE = 0.01;
// [E] The sub-cell pattern (step 3; at every cell size since step 4): the pixel's cover is the share of 2 x 2 taps over
// its footprint (x CLOUD_FAR_COVER_FOOTPRINT_SCALE, / mu along the view) where the shapes' prefiltered value at the
// taps' mip exceeds the prefilter threshold of the cell's (B-spline reconstructed) cover - the shapes' pattern covering
// that share of the layer (its mean the cell's cover), its edges as sharp as the footprint at every distance; the shape
// coordinates domain-warped (cloud_baked_warp: no repeating tile). Faded out over CLOUD_BAKED_FLAT_PATTERN_FADE_M of
// footprint (the old far tier's CLOUD_FAR_COVER_FADE_M: from a quarter of the tile on the taps average the layer's
// mean; the scaled-space far field takes over beyond). The tap's step is softened over CLOUD_BAKED_FLAT_PATTERN_SOFT of
// the shape value (no single-tap speckle). (Step 3 faded it in from cells 2 x the footprint and out over 2-4 km
// footprints: the cells' bilinear / smoothed cover alone drew blurred blobs and field-cell squares from 600 km up.)
const vec2 CLOUD_BAKED_FLAT_PATTERN_FADE_M = vec2(8000.0, 16000.0);
// [E] The pattern's fade toward the scaled-space far field (camera heights, m; it takes over at the camera distance
// CelestialBody.SCALED_SPACE_DISTANCE_M = 15 000 km from the Earth's centre, ~8 620 km up): its cover has no sub-cell
// pattern, so the flat levels lose theirs before the switch (step 5: no pop).
const vec2 CLOUD_BAKED_FLAT_SWITCH_FADE_M = vec2(5500000.0, 8000000.0);

vec4 cloud_baked_flat_tex(int si, int which, vec3 uvl) {
#ifdef CLOUD_BAKED_FADE
	if (si == 1) {
		return which == 0 ? texture(baked_f1, uvl) : texture(baked_t1, uvl);
	}
#endif
	return which == 0 ? texture(baked_f0, uvl) : texture(baked_t0, uvl);
}

// A flat level's texture (which: 0 F, 1 T) reconstructed by the cubic B-spline at uvl (n columns per side): four
// bilinear fetches at the B-spline's weighted positions (C2, non-negative weights: the cells' mean kept; step 4 - the
// bilinear's creases and plateaus drew every cell as a square where it spans several pixels).
vec4 cloud_baked_flat_bspline(int si, int which, vec3 uvl, float n) {
	vec2 x = uvl.xy * n - 0.5;
	vec2 i = floor(x);
	vec2 t = x - i;
	vec4 wx = cloud_bspline_weights(t.x);
	vec4 wy = cloud_bspline_weights(t.y);
	vec2 g0 = vec2(wx.x + wx.y, wy.x + wy.y);
	vec2 g1 = vec2(wx.z + wx.w, wy.z + wy.w);
	vec2 h0 = (i - 0.5 + vec2(wx.y, wy.y) / g0) / n;
	vec2 h1 = (i + 1.5 + vec2(wx.w, wy.w) / g1) / n;
	return g0.y * (g0.x * cloud_baked_flat_tex(si, which, vec3(h0.x, h0.y, uvl.z))
					+ g1.x * cloud_baked_flat_tex(si, which, vec3(h1.x, h0.y, uvl.z)))
			+ g1.y * (g0.x * cloud_baked_flat_tex(si, which, vec3(h0.x, h1.y, uvl.z))
					+ g1.x * cloud_baked_flat_tex(si, which, vec3(h1.x, h1.y, uvl.z)));
}

// The flat levels' weight at a volume sample of continuous level l (cloud_baked_lod; 0 the volume alone, 1 the flat
// levels alone; the volume's extinction scaled by 1 - w, the flat levels' alpha by w): the hand-over beyond the
// volume's last level K - 1 (across the same band as any level to the next).
float cloud_baked_flat_weight_at(int si, float l) {
	if (baked.sets[si].outer.x < 0.5) {
		return 0.0;
	}
	// (diagnostics, STARSIM_BAKED_DEFINES: the flat levels alone / the volume alone)
#ifdef CLOUD_BAKED_FLAT_ONLY
	return 1.0;
#endif
#ifdef CLOUD_BAKED_VOLUME_ONLY
	return 0.0;
#endif
	float last = baked.sets[si].axis_y.w - 1.0;
	return smoothstep(last - max(cloud_baked_band(), CLOUD_BAKED_FLAT_SEAM_BAND), last, l);
}

// The footprint's continuous level (cell c0 2^l = the footprint; the flat levels' rule).
float cloud_baked_lod_fp(int si, float footprint_m) {
	return log2(max(CLOUD_BAKED_LEVEL_DROP * footprint_m, 1.0e-3) / baked.sets[si].axis_x.w);
}

// The distance along the ray where the flat levels' weight (cloud_baked_flat_weight_at) first becomes > 0: the level
// l(t) = cloud_baked_lod(max-norm radius, footprint) reaches last - band, the radius and the footprint both linear in t
// (the first of the two crossings; 0 where the ray starts beyond the radius). The same band and terms as the weight and
// the volume's keep, so the flat levels start exactly where the volume begins to give way - at every camera height.
float cloud_baked_flat_start(int si, CloudBakedRay g, float pixel_angle) {
#ifdef CLOUD_BAKED_FLAT_ONLY
	return 0.0;
#endif
	float l0 = baked.sets[si].axis_y.w - 1.0 - max(cloud_baked_band(), CLOUD_BAKED_FLAT_SEAM_BAND);
	float r0 = (0.5 * baked.sets[si].up0.w - 1.0) * baked.sets[si].axis_x.w * exp2(l0);
	float t = 1.0e30;
	if (max(abs(g.gx0), abs(g.gy0)) >= r0) {
		t = 0.0;
	} else {
		if (g.gdx != 0.0) {
			t = min(t, ((g.gdx > 0.0 ? r0 : -r0) - g.gx0) / g.gdx);
		}
		if (g.gdy != 0.0) {
			t = min(t, ((g.gdy > 0.0 ? r0 : -r0) - g.gy0) / g.gdy);
		}
	}
	return min(t, baked.sets[si].axis_x.w * exp2(l0) / (CLOUD_BAKED_LEVEL_DROP * max(pixel_angle, 1.0e-9)));
}

// The flat levels at the Earth-local point p (from the Earth's centre) for a pixel footprint (m): F and T (the cubic
// B-spline, cloud_baked_flat_bspline), the level rule's l = max(the footprint's, the levels' extent: m0 + log2(r /
// inner)) - level ceil(l), at least m0, blended toward the next over cloud_baked_band() (at least
// CLOUD_BAKED_FLAT_SEAM_BAND). heights_only: F alone. false beyond the baked cap (the far side is never fetched).
float cloud_baked_diag_level = 0.0;
bool cloud_baked_flat_fetch(int si, vec3 p, float footprint_m, bool heights_only, out vec4 f, out vec2 t) {
	f = vec4(0.0);
	t = vec2(0.0);
	int flat_levels = int(baked.sets[si].outer.x + 0.5);
	if (flat_levels <= 0) {
		return false;
	}
	vec3 xyt = cloud_baked_flat_xy(si, p);
	if (xyt.z > baked.sets[si].outer.y) {
		return false;
	}
	float n = baked.sets[si].flat_up0.w;
	float c0 = baked.sets[si].axis_x.w;
	float m0 = baked.sets[si].outer.z;
	float inner = (0.5 * n - 1.0) * c0 * exp2(m0);
	float kf = max(cloud_baked_lod_fp(si, footprint_m), m0 + log2(max(max(abs(xyt.x), abs(xyt.y)), 1.0) / inner));
	int k0 = max(int(ceil(kf)), int(m0));
	float band = max(cloud_baked_band(), CLOUD_BAKED_FLAT_SEAM_BAND);  // (the flat levels' own hand-overs: always a band)
	float blend = smoothstep(1.0 - band, 1.0, kf - float(k0 - 1));
	int j = k0 - int(m0);
	if (j >= flat_levels - 1) {
		j = flat_levels - 1;
		blend = 0.0;
	}
	float c = c0 * exp2(m0 + float(j));
	cloud_baked_diag_level = float(j) + blend;
	vec3 uvl = vec3(xyt.xy / (c * n) + 0.5, float(j));
	f = cloud_baked_flat_bspline(si, 0, uvl, n);
	if (!heights_only) {
		t = cloud_baked_flat_bspline(si, 1, uvl, n).xy;
	}
	if (blend > 0.0) {
		uvl = vec3(xyt.xy / (2.0 * c * n) + 0.5, float(j + 1));
		f = mix(f, cloud_baked_flat_bspline(si, 0, uvl, n), blend);
		if (!heights_only) {
			t = mix(t, cloud_baked_flat_bspline(si, 1, uvl, n).xy, blend);
		}
	}
	return true;
}

// [E] The thickness inside the shapes (step 5; step 4's shapes were uniform paper cut-outs): the in-cloud depth at a tap
// is the column's times k = (K0 + (1 - K0) r) / (K0 + (1 - K0) E[r | covered]), r the volume's soft edge ramp of the
// shape's excess over the threshold (cloud_edge_ramp: the edge width from the shape's width noise) and E[r | covered]
// = the prefiltered edge mean / the cover (cloud_prefilter_edge_mean: the volume levels' own normalisation) - thick
// cores, margins at K0 of the mean (translucent where the cloud is thin), the depth's mean over the covered taps the
// column's (the far field's water); the volume levels below the takeover draw the same ramp in 3-D. Toward a closed
// cover (CLOUD_BAKED_FLAT_DECK_COVER) the ramp is 1 nearly everywhere: k moves to a texture about the median, x 1 +
// CLOUD_BAKED_FLAT_DEPTH_TEXTURE (s - s_50) within CLOUD_BAKED_FLAT_DEPTH_RANGE (step 4's deck texture).
const float CLOUD_BAKED_FLAT_MARGIN_DEPTH = 0.3;
const vec2 CLOUD_BAKED_FLAT_DECK_COVER = vec2(0.7, 0.95);
const float CLOUD_BAKED_FLAT_DEPTH_TEXTURE = 3.0;
const vec2 CLOUD_BAKED_FLAT_DEPTH_RANGE = vec2(0.25, 1.75);
// [E] The pattern's anti-aliasing (step 5): a tap covers clamp((s - threshold) / g + 0.5) of its area, g the shape's
// change across a tap (the taps' differences: the pixel footprint's), at least CLOUD_BAKED_FLAT_AA_MIN - the edge one
// pixel wide at every distance, sharp, not a binary staircase.
const float CLOUD_BAKED_FLAT_AA_MIN = 0.004;
// [E] The pattern taps' spacing (m) over which the cloud-to-cloud spread (features of 2-4 km) gives way to its effective
// thickness (CLOUD_FAR_TAU_SPREAD_CHI).
const vec2 CLOUD_BAKED_FLAT_SPREAD_FADE_M = vec2(500.0, 2000.0);

// One reading of the sub-cell pattern: the 2 x 2 taps (t1 along, t2 across; along_m x across_m) about p, the shape
// scaled up by `scale` (the coordinates divided by it; the tap spacing in the scaled shape picks its prefilter level);
// `cover` the cell's cover along the view, tau the in-cloud vertical depth, mu the view's cosine. x the covered share,
// y the mean opacity, z the covered taps' mean depth factor k (CLOUD_BAKED_FLAT_MARGIN_DEPTH).
vec3 cloud_baked_flat_pattern_at(vec3 p, vec3 t1, vec3 t2, float along_m, float across_m, float spacing_m, float scale,
		float cover, float tau, float mu) {
	float level = clamp(log2(spacing_m / scale * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M), 0.0,
			CLOUD_SHAPE_PREFILTER_LEVELS);
	vec2 uv = cloud_prefilter_uv(clamp(cover, 0.004, 0.996), level);
	float threshold = texture(cloud_prefilter_threshold, uv).r;
	float median = texture(cloud_prefilter_threshold, cloud_prefilter_uv(0.5, level)).r;
	float ramp_mean = clamp(texture(cloud_prefilter_edge_mean, uv).r / max(cover, 1.0e-3), 0.05, 1.0);
	float deck = smoothstep(CLOUD_BAKED_FLAT_DECK_COVER.x, CLOUD_BAKED_FLAT_DECK_COVER.y, cover);
	float k_norm = CLOUD_BAKED_FLAT_MARGIN_DEPTH + (1.0 - CLOUD_BAKED_FLAT_MARGIN_DEPTH) * ramp_mean;
	float unit = 1.0 / (CLOUD_SHAPE_TILE_H_M * CLOUD_FAR_COVER_UNALIGNED_SCALE * scale);
	vec3 q0 = CLOUD_FAR_COVER_UNALIGNED_ROTATION * p * unit;
	float w = cloud_baked_blend_weight(q0);  // (smooth: once per pixel)
	// the broken cloud's cloud-to-cloud spread as the volume's cumulus (cloud_spread_factor, look fixes T7b; the decks'
	// texture below without it), toward the effective thickness where the taps outgrow its few-km features
	float spread = cloud_far_subgrid ? mix(cloud_spread_factor(p), CLOUD_FAR_TAU_SPREAD_CHI,
			smoothstep(CLOUD_BAKED_FLAT_SPREAD_FADE_M.x, CLOUD_BAKED_FLAT_SPREAD_FADE_M.y, spacing_m)) : 1.0;
	vec2 s[4];
	for (int i = 0; i < 4; i++) {
		vec2 u = (vec2(i & 1, i >> 1) + 0.5) * 0.5 - 0.5;
		vec3 q = q0 + CLOUD_FAR_COVER_UNALIGNED_ROTATION * (t1 * (u.x * along_m) + t2 * (u.y * across_m)) * unit;
		s[i] = cloud_baked_shape(q, level, w, median);
	}
	float aa = max(length(vec2(s[1].x - s[0].x + s[3].x - s[2].x, s[2].x - s[0].x + s[3].x - s[1].x)) * 0.5,
			CLOUD_BAKED_FLAT_AA_MIN);
	vec3 result = vec3(0.0);
	for (int i = 0; i < 4; i++) {
		float covered = cover >= 0.996 ? 1.0 : clamp((s[i].x - threshold) / aa + 0.5, 0.0, 1.0);
		float k_ramp = (CLOUD_BAKED_FLAT_MARGIN_DEPTH + (1.0 - CLOUD_BAKED_FLAT_MARGIN_DEPTH)
				* cloud_edge_ramp(s[i].x, threshold, s[i].y)) / k_norm;
		float k_deck = clamp(1.0 + CLOUD_BAKED_FLAT_DEPTH_TEXTURE * (s[i].x - median), CLOUD_BAKED_FLAT_DEPTH_RANGE.x,
				CLOUD_BAKED_FLAT_DEPTH_RANGE.y);
		float k = mix(k_ramp * spread, k_deck, deck);
		result += vec3(covered, covered * (1.0 - exp(-tau * k / mu)), covered * k);
	}
	result.z = result.x > 0.0 ? result.z / result.x : 1.0;
	result.xy *= 0.25;
	return result;
}

// The sub-cell pattern (CLOUD_BAKED_FLAT_PATTERN_*) at the Earth-local point p (geodetic up `up`) for a view dir of
// |cosine| mu, a pixel footprint, the cell's cover along the view (its mean) and in-cloud depth tau: x the pixel's
// cover, y its opacity, z the depth factor of its cloud (cloud_baked_flat_pattern_at). Where the taps' spacing passes
// the shapes' last prefilter level (1 km texels) the shape is read scaled up by powers of two (two octaves cross-faded
// by the spacing's fraction; each with the cover's mean): its features stay a few pixels wide at every distance (step 4).
vec3 cloud_baked_flat_pattern(vec3 p, vec3 up, vec3 dir, float mu, float footprint_m, float cover, float tau) {
	if (cover <= 0.004) {
		return vec3(cover, cover * (1.0 - exp(-tau / mu)), 1.0);
	}
	vec3 along = dir - up * dot(dir, up);
	float along_norm = length(along);
	vec3 t1 = along_norm > 1.0e-6 ? along / along_norm
			: normalize(cross(up, abs(up.y) < 0.9 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0)));
	vec3 t2 = cross(up, t1);
	float across_m = footprint_m * CLOUD_FAR_COVER_FOOTPRINT_SCALE;
	float along_m = min(across_m / mu, max(CLOUD_FAR_COVER_ALONG_MAX_M, across_m));
	float spacing_m = 0.5 * max(along_m, across_m);
	float octave = max(log2(spacing_m * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M) - CLOUD_SHAPE_PREFILTER_LEVELS, 0.0);
	float k = floor(octave);
	float w = octave - k;
	vec3 result = cloud_baked_flat_pattern_at(p, t1, t2, along_m, across_m, spacing_m, exp2(k), cover, tau, mu);
	if (w > 0.0) {
		result = mix(result, cloud_baked_flat_pattern_at(p, t1, t2, along_m, across_m, spacing_m, exp2(k + 1.0), cover,
				tau, mu), w);
	}
	return result;
}

// The flat levels' height at a fetched F for a view of cosine mu (CLOUD_BAKED_FLAT_GRAZING_MU).
float cloud_baked_flat_height(vec4 f, float mu) {
	return 1000.0 * mix(f.w, f.z, clamp((mu - CLOUD_BAKED_FLAT_GRAZING_MU) / (1.0 - CLOUD_BAKED_FLAT_GRAZING_MU), 0.0,
			1.0)) / f.y;
}

// [E] The Earth's shadow edge on the clouds (step 5): within CLOUD_BAKED_FLAT_TERMINATOR_MU of the Sun's horizon at
// the light's point the column's light is the mean of the slab at its optical top, its point and as far below (the
// edge passes the column's heights over a band of Sun elevations: a single height drew it as one sharp line, kinked
// where the height changed).
const float CLOUD_BAKED_FLAT_TERMINATOR_MU = 0.08;
// [E] The height of the sphere the per-pixel pattern is read on (m; the pattern's parallax is that of a layer at this
// height, the light and the occlusion distance the column's own).
const float CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M = 2000.0;
// [E] A cloud behind the scene but with its optical top in front is drawn at this share of the scene's distance.
const float CLOUD_BAKED_FLAT_SCENE_PULL = 0.9995;
const float CLOUD_BAKED_FLAT_SCENE_DEPTH_M = 3000.0;
// [E] The height crossing's steps (heights-only fetches) and its tolerance (m).
const int CLOUD_BAKED_FLAT_HEIGHT_STEPS = 4;
const float CLOUD_BAKED_FLAT_HEIGHT_TOLERANCE_M = 10.0;

// [E] Camera heights (m) over which the air above a bright top (cloud_far_bright_top_transmittance) is applied: none
// from the ground and in the cloud layer, fully where the flat levels take over (40 km). The volume and the flat levels
// both use cloud_baked_bright_top_weight (look fixes T7b, T7 review I1: the flat levels took it unconditionally, so a
// ground camera's horizon clouds and a base seen from below got a term of a top lit from above - at grazing slant
// paths T_tot / T up to ~46x in blue).
const vec2 CLOUD_BAKED_BRIGHT_TOP_HEIGHT_M = vec2(20000.0, 40000.0);

// The bright-top term's weight (0..1) for a view of a cloud with mu_down = -dot(dir, up) at the cloud (> 0: seen from
// above): the camera-height fade times a view from above.
float cloud_baked_bright_top_weight(float mu_down) {
	return smoothstep(CLOUD_BAKED_BRIGHT_TOP_HEIGHT_M.x, CLOUD_BAKED_BRIGHT_TOP_HEIGHT_M.y, cloud_camera_height_m)
			* step(CLOUD_FAR_MU_MIN, mu_down);
}

// The flat levels along the ray from t_from (inside the 20 km shell): the far field's placement (cloud_far_layer) - the
// point where the ray crosses the column's seen-depth height (cloud_baked_flat_height; a first fetch where the ray
// crosses the guess height, then the fetched height's crossing until bracketed, then false position), the light evaluated there. The pixel's cover and
// opacity (the far field's rule, cloud_far.gdshaderinc cloud_far_layer): the cell's cover C seen along the view as the
// beam cover's slant growth 1 - (1 - C)^(1 + s), s = CLOUD_OVERLAP_SLANT max(cot - CLOUD_OVERLAP_ONSET_COT, 0)
// (cloud_beam_cover_of's slant term; its vertical term is in C already: the shapes' projected cover), the water kept
// (in-cloud depth tau_e C / C_v), alpha = C_v (1 - exp(-tau / mu)) - thin cloud translucent; the sub-cell pattern at
// the footprint (cover, opacity and the depth inside the shapes); the light the far field's column model at the pixel
// for its own view (cloud_far_cloudy_radiance) with the shapes' depth. Out: premultiplied colour with the air in front,
// alpha, the distance; false: none.
bool cloud_baked_flat_shade(int si, CloudRay ray, vec3 dir, vec3 eye, float t_from, float scene_m, float pixel_angle,
		out vec3 color, out float alpha, out float dist) {
	color = vec3(0.0);
	alpha = 0.0;
	dist = 0.0;
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float t1 = t_from;
	float h1 = cloud_height_along(ray, t1);
	if (h1 > CLOUD_TOP_M - CLOUD_BAKED_BAND_TOLERANCE_M) {
		vec2 x = cloud_sphere_crossing(ray, baked.sets[si].outer.w);
		if (x.x > x.y || x.x < t1) {
			return false;
		}
		t1 = x.x;
	}
	// the scene in front of the shell's entry: by the same vertical rule as the crossing behind the scene below (step 7:
	// near the limb the scene's depth - grazing, half resolution - landed in front of the entry along rows and columns
	// of the depth buffer, a striped staircase of holes in the clouds at 3 000 km; a descending ray meets the shell
	// before any ground under it)
	if (t1 >= scene_m && (t1 - scene_m) * max(abs(dot(dir, cloud_geodetic_up(camera + eye + dir * t1))),
			CLOUD_FAR_MU_MIN) > CLOUD_BAKED_FLAT_SCENE_DEPTH_M) {
		return false;
	}
	vec4 f;
	vec2 t_col;
	float t_hit = t1;
	vec3 p = camera + eye + dir * t_hit;
	// the root of g(t) = height(t) - h(p(t)): fixed-point steps (the fetched height's crossing) until the root is
	// bracketed (a point on each side of the height surface), then false position (steep height changes: the fixed point
	// alone oscillated - smeared shapes)
	float t_near = t_from;
	float g_near = 0.0;
	float t_far = 0.0;
	float g_far = 0.0;
	bool bracketed = false;
	bool have_near = false;
	for (int i = 0; i < CLOUD_BAKED_FLAT_HEIGHT_STEPS; i++) {
		if (!cloud_baked_flat_fetch(si, p, pixel_angle * t_hit, true, f, t_col)) {
			return false;
		}
		if (!(f.y > 1.0e-6)) {
			break;
		}
		float h = cloud_baked_flat_height(f, max(abs(dot(dir, cloud_geodetic_up(p))), CLOUD_FAR_MU_MIN));
		float g = cloud_height_along(ray, t_hit) - h;
		if (abs(g) < CLOUD_BAKED_FLAT_HEIGHT_TOLERANCE_M) {
			break;
		}
		if ((cloud_camera_height_m > h) == (g > 0.0)) {
			t_near = t_hit;  // still on the camera's side
			g_near = g;
			have_near = true;
		} else {
			t_far = t_hit;
			g_far = g;
			bracketed = have_near;
		}
		float t_next;
		if (bracketed) {
			t_next = t_near - g_near * (t_far - t_near) / (g_far - g_near);
		} else {
			vec2 x = cloud_sphere_crossing(ray, h);
			if (x.x > x.y) {
				return false;  // (the ray passes above the height: the far field's crossing has none either)
			}
			t_next = cloud_camera_height_m > h ? x.x : x.y;
			if (t_next < 0.0) {
				return false;
			}
		}
		t_hit = max(t_next, t_from);
		p = camera + eye + dir * t_hit;
	}
	if (t_hit < 0.0) {
		return false;
	}
	// Behind the Earth (VR bugs 2026-10-30): a crossing beyond the ray's own crossing of the model sphere (sea level)
	// lies behind the Earth's curvature - from below a deck the far side's crossing of the column's height (x.y, the
	// ray through the Earth) passed where the scene's depth let it (the 3 km vertical rule at grazing rays, no depth
	// at the horizon's sky / ground blocks) and drew far cap parts just under the horizon line.
	vec2 ground = cloud_sphere_crossing(ray, 0.0);
	bool behind_earth = ground.x <= ground.y && ground.x > 0.0 && t_hit > ground.x;
#ifndef CLOUD_BAKED_DIAG_BEHIND_EARTH
	if (behind_earth) {
		return false;
	}
#endif
	vec3 up = cloud_geodetic_up(p);
	float mu = max(abs(dot(dir, up)), CLOUD_FAR_MU_MIN);
	float footprint = pixel_angle * t_hit / sqrt(max(mu, 0.04));
	if (!cloud_baked_flat_fetch(si, p, footprint, false, f, t_col)) {
		return false;
	}
	if (t_hit >= scene_m) {
		// behind the scene (the column's seen height below high terrain - the field's layers over mountains): drawn on
		// the scene's point while the column's optical top is in front of it or the point lies less than
		// CLOUD_BAKED_FLAT_SCENE_DEPTH_M (vertically) behind the scene (the far field ignores the terrain; a rejection cut
		// the clouds along the depth buffer's steps)
		vec2 x = cloud_sphere_crossing(ray, t_col.x > 0.0 ? t_col.y / t_col.x : 0.0);
		bool top_in_front = x.x <= x.y && (cloud_camera_height_m > t_col.y / max(t_col.x, 1.0e-6) ? x.x : x.y) < scene_m;
		if (!top_in_front && (t_hit - scene_m) * mu > CLOUD_BAKED_FLAT_SCENE_DEPTH_M) {
			return false;
		}
		t_hit = scene_m * CLOUD_BAKED_FLAT_SCENE_PULL;
		p = camera + eye + dir * t_hit;
	}
	float c_col = t_col.x;  // the column rule's nadir cover (the cell's mean of the pattern f.x)
	if (c_col < CLOUD_MIN_FRACTION || !(f.y > 0.0)) {
		return false;
	}
	// the far field's beam cover at the view's slant: 1 - (1 - f)^(n_v + s) = 1 - (1 - C_col)^(1 + s / n_v), applied to
	// the pattern (0 and 1 kept, its mean the far field's); the in-cloud depth the grid mean / that cover
	float fraction = cloud_baked_fraction_of(c_col);
	float cover_v = cloud_beam_cover_of(fraction, mu);
	float n_v = 1.0 + CLOUD_OVERLAP_VERTICAL * (1.0 - fraction);
	float slant = CLOUD_OVERLAP_SLANT * max(sqrt(max(1.0 - mu * mu, 0.0)) / mu - CLOUD_OVERLAP_ONSET_COT, 0.0);
	float cov = 1.0 - pow(max(1.0 - clamp(f.x, 0.0, 1.0), 0.0), 1.0 + slant / n_v);
	float tau_v = f.y / max(cover_v, CLOUD_MIN_FRACTION);
	// the pattern faded out with the footprint and toward the scaled-space far field (no pattern there: no pop at the
	// switch, CLOUD_BAKED_FLAT_SWITCH_FADE_M)
	float w_pattern = (1.0 - smoothstep(CLOUD_BAKED_FLAT_PATTERN_FADE_M.x, CLOUD_BAKED_FLAT_PATTERN_FADE_M.y, footprint))
			* (1.0 - smoothstep(CLOUD_BAKED_FLAT_SWITCH_FADE_M.x, CLOUD_BAKED_FLAT_SWITCH_FADE_M.y, cloud_camera_height_m));
#ifdef CLOUD_BAKED_DIAG_NO_PIXEL_PATTERN
	w_pattern = 0.0;
#endif
	// where the shapes' pattern has faded, the far field's sub-grid structure (look fixes T7 B, cloud_far.gdshaderinc):
	// the even cover as mesoscale cells, the slab at the effective thickness of the in-cloud spread
	if (cloud_far_subgrid && w_pattern < 1.0) {
		cov = mix(cloud_far_meso_cover(p, cov, footprint), cov, w_pattern);
		tau_v *= mix(CLOUD_FAR_TAU_SPREAD_CHI, 1.0, w_pattern);
	}
	alpha = cov * (1.0 - exp(-tau_v / mu));
	float tau_light = tau_v;
	if (w_pattern > 0.0) {
		// the pattern read on a sphere of fixed height (CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M), not at p: where the
		// column's height rises steeply against the view the shapes were dragged along it (smeared)
		vec2 x_p = cloud_sphere_crossing(ray, CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M);
		float t_p = cloud_camera_height_m > CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M ? x_p.x : x_p.y;
		vec3 p_pattern = x_p.x <= x_p.y && t_p > 0.0 ? camera + eye + dir * t_p : p;
		vec3 pattern = cloud_baked_flat_pattern(p_pattern, up, dir, mu, footprint, cov, tau_v);
		cov = mix(cov, pattern.x, w_pattern);
		alpha = mix(alpha, pattern.y, w_pattern);
		tau_light *= mix(1.0, pattern.z, w_pattern);
	}
	if (!(alpha > 1.0e-6)) {
		return false;
	}
#ifdef CLOUD_BAKED_DIAG_BEHIND_EARTH
	if (behind_earth) {
		color = vec3(alpha, 0.0, 0.0);  // (diagnostic: the old behaviour's hits behind the Earth in red)
		dist = t_hit;
		return true;
	}
#endif
	vec3 radiance = cloud_far_cloudy_radiance(p, up, -dir, tau_light, fraction, vec3(1.0));
	// near the Earth's shadow edge: the slab's light at three heights (CLOUD_BAKED_FLAT_TERMINATOR_MU)
	vec3 pm = p * ATM_STRETCH;
	float mu_s = dot(normalize(pm), normalize(atmosphere_sun_direction * ATM_STRETCH));
	float mu_h = -sqrt(max(1.0 - CLOUD_A_M * CLOUD_A_M / dot(pm, pm), 0.0));
	if (abs(mu_s - mu_h) < CLOUD_BAKED_FLAT_TERMINATOR_MU && t_col.x > 0.0) {
		float h_p = cloud_height_along(ray, t_hit);
		float above = clamp(t_col.y / t_col.x - h_p, 0.0, 6000.0);
		if (above > 10.0) {
			radiance = (radiance + cloud_far_cloudy_radiance(p + up * above, up, -dir, tau_light, fraction, vec3(1.0))
					+ cloud_far_cloudy_radiance(p - up * min(above, max(h_p, 0.0)), up, -dir, tau_light, fraction,
							vec3(1.0))) / 3.0;
		}
	}
	vec3 air_transmittance;
	vec3 air_light = atm_to_point_offset(eye + dir * t_hit, air_transmittance);
	// the air above the bright top (cloud_far_bright_top_transmittance, look fixes T7 step 3), gated as the volume's:
	// a view from above, faded in with the camera's height (cloud_baked_bright_top_weight; none from the ground)
	vec3 seen = air_transmittance;
	float w_top = cloud_baked_bright_top_weight(-dot(dir, up));
	if (w_top > 0.0) {
		seen = mix(air_transmittance, cloud_far_bright_top_transmittance(air_transmittance, mu, cov, tau_light), w_top);
	}
	color = seen * radiance * cov + alpha * air_light;
#ifdef CLOUD_BAKED_DIAG_LEVEL
	color = alpha * vec3(fract(cloud_baked_diag_level), floor(cloud_baked_diag_level) / 3.0, t_col.y / 10000.0);
#endif
	dist = t_hit;
	return true;
}

// [E] Beyond the flat geometry's cap the far field's column draws the clouds (step 6: a camera that outran the re-bake -
// the free camera moves up to its distance to the surface per second, a re-bake takes ~0.3 s - saw the clouds end at
// the shown geometry's cap, a circle inside the disc): cloud_far_layer (the scaled-space Earth's: the same cover rule
// and light) at the ray's ground point, blended in over CLOUD_BAKED_FLAT_FAR_BAND_RAD up to
// CLOUD_BAKED_FLAT_FAR_EDGE_CELLS of the last level's cells inside the cap (the bake's columns end 0.71 cell beyond it,
// the B-spline reads 2 cells around a point). In a settled view the visible cap ends FLAT_DRIFT_RAD (2 deg) inside the
// band: no pixel pays for it.
const float CLOUD_BAKED_FLAT_FAR_BAND_RAD = 0.0087;
const float CLOUD_BAKED_FLAT_FAR_EDGE_CELLS = 3.0;

// The far field's weight (0..1) for the ray in this set: 0 without a ground crossing.
float cloud_baked_flat_far_weight(int si, CloudRay ray, vec3 dir, vec3 eye) {
	vec2 ground = cloud_sphere_crossing(ray, 0.0);
	if (baked.sets[si].outer.x < 0.5 || !(ground.x <= ground.y && ground.x > 0.0)) {
		return 0.0;
	}
	vec3 g = atmosphere_camera_hi + atmosphere_camera_lo + eye + dir * ground.x;
	float c_last = baked.sets[si].axis_x.w * exp2(baked.sets[si].outer.z + baked.sets[si].outer.x - 1.0);
	float edge = baked.sets[si].outer.y - CLOUD_BAKED_FLAT_FAR_EDGE_CELLS * c_last / CLOUD_A_M;
	return smoothstep(edge - CLOUD_BAKED_FLAT_FAR_BAND_RAD, edge, cloud_baked_flat_xy(si, g).z);
}

// The far field's layer for the ray (premultiplied colour with the air in front, alpha) at the distance dist (the
// ray's crossing of CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M); none where the scene is in front by more than
// CLOUD_BAKED_FLAT_SCENE_DEPTH_M (vertically). Evaluated once per pixel after the sets (cloud_baked_shade), and only in
// the march kernels with CLOUD_BAKED_FAR_FALLBACK (CloudBaked: above FLAT_FINE_HEIGHT_M - a camera below moves at most
// ~40 km/s, the 2 deg drift allowance lasts > 5 s against a re-bake's ~0.3 s): its code raised the march's register
// peak - ground +0.14 ms even when not taken (inside the set's function ground / deck +0.15 / +0.5 ms).
vec4 cloud_baked_flat_far_layer(CloudRay ray, vec3 dir, vec3 eye, float scene_m, float pixel_angle, out float dist) {
	dist = 0.0;
	vec2 ground = cloud_sphere_crossing(ray, 0.0);
	if (!(ground.x <= ground.y && ground.x > 0.0)) {
		return vec4(0.0);
	}
	vec3 g = atmosphere_camera_hi + atmosphere_camera_lo + eye + dir * ground.x;
	vec2 x = cloud_sphere_crossing(ray, CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M);
	dist = x.x <= x.y && x.x > 0.0 ? x.x : ground.x;
	float mu = max(abs(dot(dir, cloud_geodetic_up(g))), CLOUD_FAR_MU_MIN);
	if ((dist - scene_m) * mu > CLOUD_BAKED_FLAT_SCENE_DEPTH_M) {
		return vec4(0.0);
	}
	return cloud_far_layer(g, vec3(1.0), pixel_angle * ground.x / sqrt(max(mu, 0.04)));
}

#endif
