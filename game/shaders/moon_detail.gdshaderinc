// Procedural regolith detail and the statistics of the unresolved lunar surface (moon procedural surface plan Task 10),
// for terrain_moon.gdshader and the test patch game/tests/moon_detail_patch.gdshader. The statistics mirror the core's
// environment/unresolved_shading.hpp (keep in sync; moon_surface_check.gd renders a patch against
// StarsimCore.moon_unresolved_light_factors / moon_residual_theta_bar). MoonDetailShading.configure sets the material
// uniforms below (TerrainQuadtree copies the template to every node and sets the per-node ones).
//
// What a fragment gets (docs/subsystems/moon-surface.md, "Regolith shading"):
// - Residual Hapke mean slope: S_res^2 = S^2 - drawn(L_eff), S^2 = pi tan^2(theta_bar of the globe's fit), drawn(L) the
//   mean square slope the node's mesh draws at vertex spacing L (core table), L_eff = spacing x 2^morph (the morph
//   slides a node to its parent's surface: continuous across splits).
// - Sun factor: Smith's G1 for the drawn relief finer than the horizon map's grid (it casts no shadow here), the
//   unresolved rocks' albedo share, cast shadows and lit faces (Boolean model); earthshine factor: the albedo share
//   only (the earthshine keeps the horizon maps alone).
// - The baked regolith slope maps (s0..s3, Task 8 and the Task 10 fix round), each holding two components: the
//   self-affine regolith band and the craters of the core's law with D in the band. Each component is faded in by the
//   node spacing (the share of its variance the mesh does not draw: the band's spectral transfer for the regolith, the
//   core's crater_drawn_fraction for the craters) and out by the mip chain (pixel footprint), stochastic hex tiling
//   (random offset per triangle-grid vertex, variance-preserving blend). The texture relief casts no shadows, so it does not tilt the
//   Hapke normal: it modulates each light's radiance by 1 + tanh(k . grad h), k = d ln R / d grad h of the
//   single-scattering law at the mesh normal. The modulation is odd in grad h, so a symmetric slope field leaves the
//   mean radiance unchanged (the slopes stay in theta_bar's budget): first-order exact for gentle slopes, saturating to
//   lit / unlit facets under a low Sun.

// [S] Moon procedural surface spec: geometry from ~1-2 px projected size (ROCK_PIXEL_THRESHOLD = 1.5 px, [E]).
const float DETAIL_PI = 3.14159265358979;
const float DETAIL_LN2 = 0.693147180559945;
// [D] Hex lattice period in cells (the hash repeats after DETAIL_HEX_PERIOD cells; per scale 256 x tile / 2).
const int DETAIL_HEX_PERIOD = 256;
// [D] The core's tables (unresolved_shading.hpp): drawn_table_samples, lit_gain_samples.
const int DETAIL_DRAWN_SAMPLES = 97;
const int DETAIL_GAIN_SAMPLES = 46;
// MoonDetailShading.WEIGHT_SAMPLES / WEIGHT_PER_OCTAVE (texture weights from 1 mm, 4 per octave, to 16 m).
const int DETAIL_SCALES = 4;
const int DETAIL_WEIGHT_SAMPLES = 57;
const int DETAIL_WEIGHT_PER_OCTAVE = 4;

uniform bool detail_shading = false;  // detail on and the statistics present (MoonDetailShading)
uniform bool detail_textures = false;  // the baked maps present
// The baked slope maps (regolith dh/du, dh/dv, craters dh/du, dh/dv; MoonDetailShading.slope_image), half float,
// mipmapped.
uniform sampler2D detail_slope_s0 : filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D detail_slope_s1 : filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D detail_slope_s2 : filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D detail_slope_s3 : filter_linear_mipmap_anisotropic, repeat_enable;
// Per scale: tile (m), hex cell (m), wavelength_min_m, wavelength_max_m (moon_detail.toml).
uniform vec4 detail_scale[4];
uniform float detail_weight_table[228];  // regolith: DETAIL_SCALES x DETAIL_WEIGHT_SAMPLES
uniform float detail_crater_weight_table[228];  // craters: the same layout
uniform float detail_total_slope_variance = 0.0;
uniform float detail_drawn_table[97];
uniform vec2 detail_drawn_axis = vec2(1.0e-3, 4.0);  // drawn_table_min_spacing_m, drawn_table_per_octave
// Rocks: cover coefficient, exponent q (cover = coef (D1^q - D0^q)), D_min, D_max; eta, rho, gain step, gain floor.
uniform vec4 detail_rock_law = vec4(0.0);
uniform vec4 detail_rock_shape = vec4(0.0, 1.0, 1.0, 0.0);
uniform float detail_rock_lit_gain[46];
// The abundance factor (LunarRocks::abundance_factor), equirectangular like moon_color (u = (lon + 180) / 360).
uniform sampler2D detail_rock_abundance : filter_linear, repeat_enable;
// Rocks drawn as geometry (Task 11, MoonRockLayer): from moon_rock_threshold_px pixels of projected size
// (ROCK_PIXEL_THRESHOLD x rock_threshold_scale; 0: no rock geometry, every rock statistical) at moon_rock_pixel_angle
// (rad per pixel of the active render target), distances from the eye (the head centre in VR); a rock rises out of the
// ground over the growth band [d, 2 d] (d = the threshold diameter at its distance), mirrored here by the statistical
// weight 1 - log2(D / d) (core rock_statistical_cover).
global uniform float moon_rock_pixel_angle;
global uniform float moon_rock_threshold_px;
// Per node: the smallest rock diameter the node's rock list holds (MoonRockLayer; 1e30: none drawn). The threshold
// diameter is never below it, so a rock the node does not draw stays statistical.
uniform float rock_node_min_m = 1.0e30;
// The drawn rocks' cast shadows are statistical on the ground (until the near-field shadow map, Task 12: false there).
uniform bool rock_geometry_shadows_statistical = true;
// Tests (moon_surface_check's rock patch): > 0 fixes the threshold diameter (no distance, no node limit).
uniform float rock_geometry_override_m = -1.0;
// Per node (TerrainQuadtree): the nominal vertex spacing; per scale the node corner's texture coordinate modulo the
// tile (xy, m) and its skewed hex coordinate modulo DETAIL_HEX_PERIOD (zw); the horizon map's grid spacing.
uniform float detail_cell_m = 1.0;
uniform vec4 detail_origin[4];
uniform float detail_horizon_spacing_m = 1.0e30;

// drawn(L): the core's table, linear in log2 L, 0 beyond.
float detail_drawn(float spacing_m) {
	float x = log2(max(spacing_m, 1.0e-30) / detail_drawn_axis.x) * detail_drawn_axis.y;
	if (x >= float(DETAIL_DRAWN_SAMPLES - 1)) {
		return 0.0;
	}
	x = max(x, 0.0);
	int k = min(int(x), DETAIL_DRAWN_SAMPLES - 2);
	return mix(detail_drawn_table[k], detail_drawn_table[k + 1], x - float(k));
}

// Residual Hapke mean slope (rad) for the effective spacing.
float detail_theta_bar(float spacing_m) {
	return atan(sqrt(max(detail_total_slope_variance - detail_drawn(spacing_m), 0.0) / DETAIL_PI));
}

// erf, Abramowitz & Stegun 7.1.26 (|error| < 1.5e-7).
float detail_erf(float x) {
	float s = sign(x);
	float a = abs(x);
	float t = 1.0 / (1.0 + 0.3275911 * a);
	float y = 1.0 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t
			* exp(-a * a);
	return s * y;
}

// Smith's G1 (core smith_lit_fraction).
float detail_smith(float tan_elevation, float variance) {
	if (tan_elevation <= 0.0) {
		return 0.0;
	}
	if (variance <= 0.0) {
		return 1.0;
	}
	float a = tan_elevation / sqrt(variance);
	float lambda = 0.5 * (detail_erf(a) - 1.0) + exp(-a * a) / (2.0 * a * sqrt(DETAIL_PI));
	return 1.0 / (1.0 + lambda);
}

float detail_rock_cover(float d_lo, float d_hi) {
	float lo = max(d_lo, detail_rock_law.z);
	float hi = min(d_hi, detail_rock_law.w);
	if (hi <= lo || detail_rock_law.x == 0.0) {
		return 0.0;
	}
	return detail_rock_law.x * (pow(hi, detail_rock_law.y) - pow(lo, detail_rock_law.y));
}

// Statistical rock cover at the threshold diameter d (core rock_statistical_cover): every rock below d, the growth band
// [d, 2 d] weighted 1 - x, x = log2(D / d) (closed form over the power law), nothing above.
float detail_rock_statistical_cover(float d) {
	float cover = detail_rock_cover(detail_rock_law.z, d);
	if (!(d < detail_rock_law.w) || detail_rock_law.x == 0.0 || d <= 0.0) {
		return cover;
	}
	float x0 = max(0.0, log2(detail_rock_law.z / d));
	float x1 = min(1.0, log2(detail_rock_law.w / d));
	if (x1 <= x0) {
		return cover;
	}
	float q = detail_rock_law.y;
	float k = q * DETAIL_LN2;
	float integral;
	if (abs(k) < 1.0e-6) {
		integral = (x1 - 0.5 * x1 * x1) - (x0 - 0.5 * x0 * x0);
	} else {
		integral = exp(k * x1) * ((1.0 - x1) / k + 1.0 / (k * k)) - exp(k * x0) * ((1.0 - x0) / k + 1.0 / (k * k));
	}
	return cover + detail_rock_law.x * q * DETAIL_LN2 * pow(d, q) * integral;
}

// The density factor of the threshold (MoonRockLayer.abundance_threshold_factor): above the mean population
// (abundance > 1) the threshold grows as abundance^(1 / b) (b the law's cumulative exponent, 2 - q), so the number of
// drawn rocks per pixel area stays the mean population's: a predictable load rule, boulder fields draw their smaller
// rocks statistically. 1 at abundance <= 1.
float detail_rock_abundance_factor(float abundance) {
	return abundance > 1.0 ? pow(abundance, 1.0 / (2.0 - detail_rock_law.y)) : 1.0;
}

// The threshold diameter of a fragment or rock at distance_m from the eye at the local abundance factor on a node
// whose rock list starts at node_min_m (1e30: no rock geometry).
float detail_rock_threshold_m(float distance_m, float node_min_m, float abundance) {
	if (rock_geometry_override_m > 0.0) {
		return rock_geometry_override_m;
	}
	if (moon_rock_threshold_px <= 0.0) {
		return 1.0e30;
	}
	return max(moon_rock_threshold_px * moon_rock_pixel_angle * distance_m * detail_rock_abundance_factor(abundance),
			node_min_m);
}

float detail_gain(float elevation) {
	float x = clamp(elevation / detail_rock_shape.z, 0.0, float(DETAIL_GAIN_SAMPLES - 1));
	int k = min(int(x), DETAIL_GAIN_SAMPLES - 2);
	return mix(detail_rock_lit_gain[k], detail_rock_lit_gain[k + 1], x - float(k));
}

// (sun, earth) factors of the ground (core unresolved_light_factors). geometry_min_m: the threshold diameter
// (detail_rock_threshold_m; 1e30: every rock statistical); elevation: the Sun's macroscopic elevation (rad).
vec2 detail_light_factors(float abundance, float geometry_min_m, float spacing_m, float horizon_spacing_m,
		float elevation) {
	float rho = detail_rock_shape.y;
	float c_all = abundance * detail_rock_cover(detail_rock_law.z, detail_rock_law.w);
	float c = abundance * detail_rock_statistical_cover(geometry_min_m);
	// The drawn rocks' share; their cast shadows stay statistical on the visible ground (until Task 12).
	float c_drawn = rock_geometry_shadows_statistical ? max(c_all - c, 0.0) : 0.0;
	float norm = exp(-c_all) + (1.0 - exp(-c_all)) * rho;
	float rock_share = 1.0 - exp(-c);
	float earth = (exp(-c) + rock_share * rho) / norm;
	float se = sin(elevation);
	if (se <= 0.0) {
		return vec2(0.0, earth);
	}
	float unshadowed = max(detail_drawn(spacing_m) - detail_drawn(horizon_spacing_m), 0.0);
	float lit = detail_smith(tan(elevation), unshadowed);
	float cot = sqrt(max(1.0 - se * se, 0.0)) / se;
	float eta = detail_rock_shape.x;
	float sigma = 0.5 * (sqrt(1.0 + 4.0 * eta * eta * cot * cot) - 1.0);
	float sun = lit * (exp(-c * (1.0 + sigma) - c_drawn * sigma) + rock_share * rho * detail_gain(elevation)) / norm;
	return vec2(sun, earth);
}

// Amplitude weights (regolith, craters) of texture scale i at node spacing L (MoonDetailShading.texture_weight /
// crater_weight, tabulated at DETAIL_WEIGHT_PER_OCTAVE per octave from 1 mm; linear in log2 L, clamped): the texture
// carries the share of each component the mesh does not draw.
vec2 detail_scale_weight(int i, float spacing_m) {
	float x = clamp(log2(spacing_m / 1.0e-3) * float(DETAIL_WEIGHT_PER_OCTAVE), 0.0, float(DETAIL_WEIGHT_SAMPLES - 1));
	int k = min(int(x), DETAIL_WEIGHT_SAMPLES - 2);
	int base = i * DETAIL_WEIGHT_SAMPLES;
	float f = x - float(k);
	return vec2(mix(detail_weight_table[base + k], detail_weight_table[base + k + 1], f),
			mix(detail_crater_weight_table[base + k], detail_crater_weight_table[base + k + 1], f));
}

uint detail_hash(ivec2 c, int scale) {
	uint h = uint(c.x) * 0x8da6b343u ^ uint(c.y) * 0xd8163841u ^ uint(scale + 1) * 0xcb1ab31fu;
	h ^= h >> 16u;
	h *= 0x7feb352du;
	h ^= h >> 15u;
	h *= 0x846ca68bu;
	h ^= h >> 16u;
	return h;
}

// Height gradients (in texture metres, (u, v) = the face's (s, t) axes) of scale i, regolith (xy) and craters (zw):
// stochastic hex tiling of its slope map. uv: the texture coordinate (m), skew: the skewed hex coordinate,
// duv_dx / duv_dy: screen derivatives of uv.
vec4 detail_scale_slope(int i, vec2 uv, vec2 skew, vec2 duv_dx, vec2 duv_dy) {
	float tile = detail_scale[i].x;
	vec2 base = floor(skew);
	vec2 f = skew - base;
	float z = 1.0 - f.x - f.y;
	vec3 w;
	ivec2 v0;
	ivec2 v1;
	ivec2 v2;
	ivec2 b = ivec2(base);
	if (z > 0.0) {
		w = vec3(z, f.y, f.x);
		v0 = b;
		v1 = b + ivec2(0, 1);
		v2 = b + ivec2(1, 0);
	} else {
		w = vec3(-z, 1.0 - f.y, 1.0 - f.x);
		v0 = b + ivec2(1, 1);
		v1 = b + ivec2(1, 0);
		v2 = b + ivec2(0, 1);
	}
	ivec2 verts[3] = ivec2[3](v0, v1, v2);
	vec2 dx = duv_dx / tile;
	vec2 dy = duv_dy / tile;
	vec4 sum = vec4(0.0);
	for (int k = 0; k < 3; k++) {
		ivec2 c = ((verts[k] % DETAIL_HEX_PERIOD) + DETAIL_HEX_PERIOD) % DETAIL_HEX_PERIOD;
		uint h = detail_hash(c, i);
		vec2 offset = vec2(float(h & 0xffffu), float(h >> 16u)) / 65536.0;
		vec2 t = uv / tile + offset;
		vec4 slope;
		if (i == 0) {
			slope = textureGrad(detail_slope_s0, t, dx, dy);
		} else if (i == 1) {
			slope = textureGrad(detail_slope_s1, t, dx, dy);
		} else if (i == 2) {
			slope = textureGrad(detail_slope_s2, t, dx, dy);
		} else {
			slope = textureGrad(detail_slope_s3, t, dx, dy);
		}
		sum += w[k] * slope;
	}
	// Variance-preserving blend of the (zero-mean) slopes.
	return sum * inversesqrt(max(dot(w, w), 1.0e-6));
}

// The regolith texture's height gradient on the surface (render-frame vector, tangential): grid = the node grid
// position (CUSTOM2), position = fragment - camera (for the derivatives), n = the mesh normal, spacing_m = L_eff.
// The derivatives are taken here, in uniform control flow (call it unconditionally).
vec3 detail_texture_gradient(vec2 grid, vec3 position, vec3 n, float spacing_m) {
	vec2 local = grid * detail_cell_m;  // m from the node corner along (s, t)
	vec2 dl_dx = dFdx(local);
	vec2 dl_dy = dFdy(local);
	vec3 dp_dx = dFdx(position);
	vec3 dp_dy = dFdy(position);
	if (!detail_shading || !detail_textures) {
		return vec3(0.0);
	}
	// Surface gradients of the texture coordinates (cotangent frame): grad u, grad v in the render frame.
	vec3 p2 = cross(dp_dy, n);
	vec3 p1 = cross(n, dp_dx);
	float det = dot(dp_dx, p2);
	if (abs(det) < 1.0e-20) {
		return vec3(0.0);
	}
	vec3 grad_u = (p2 * dl_dx.x + p1 * dl_dy.x) / det;
	vec3 grad_v = (p2 * dl_dx.y + p1 * dl_dy.y) / det;
	vec2 slope = vec2(0.0);
	const mat2 HEX_SKEW = mat2(vec2(1.0, 0.0), vec2(-0.57735027, 1.15470054));
	for (int i = 0; i < DETAIL_SCALES; i++) {
		vec2 weight = detail_scale_weight(i, spacing_m);
		// [E] Below 0.1 a component adds < 0.016 rms slope (s2's regolith 0.16 x 0.1), invisible: skip the fetches when
		// both are below.
		if (max(weight.x, weight.y) < 0.1) {
			continue;
		}
		// Once even the footprint's short axis (the anisotropic filter's mip) spans the band's longest wavelength, the
		// mip has averaged the band's slopes away (< 1 % of its variance left): skip it.
		float footprint = min(length(dl_dx), length(dl_dy));
		if (footprint > detail_scale[i].w) {
			continue;
		}
		vec2 uv = detail_origin[i].xy + local;
		vec2 skew = detail_origin[i].zw + HEX_SKEW * local / detail_scale[i].y;
		vec4 s = detail_scale_slope(i, uv, skew, dl_dx, dl_dy);
		slope += weight.x * s.xy + weight.y * s.zw;
	}
	return slope.x * grad_u + slope.y * grad_v;
}

// 1 + tanh(k . grad h): k = (light + eye) / (mu0 + mu) - light / mu0 (Lommel-Seeliger d ln R / d grad h at the mesh
// normal: grad h tilts the normal to n - grad h).
float detail_modulation(vec3 gradient, vec3 n, vec3 to_light, vec3 to_eye) {
	float mu0 = max(dot(n, to_light), 1.0e-3);
	float mu = max(dot(n, to_eye), 1.0e-3);
	vec3 k = (to_light + to_eye) / (mu0 + mu) - to_light / mu0;
	return 1.0 + tanh(dot(k, gradient));
}
