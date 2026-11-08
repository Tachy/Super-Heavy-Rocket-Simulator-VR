class_name GlareSpread
extends RefCounted
## Physical glare spread (look fixes T13, user 2026-11-08): the scattering of the eye's own optics, drawn over the image.
## It replaces Godot's glow (capped at luminance 12, independent of the source and of the eye; T10 diagnosis).
##
## Model. The veil luminance over the retinal image is the sources' illuminance convolved with the CIE 146:2002 general
## disability glare function
##     L_veil / E_gl = f(theta) = 10 / theta^3 + (5 / theta^2 + 0.1 p / theta) (1 + (A / 62.5)^4) + 0.0025 p     [1/sr]
## [S: CIE 146:2002 "CIE equations for disability glare"; theta in degrees, valid 0.1 .. 100 deg; A the age in years, p the
## eye pigmentation (0.5, the CIE average); constants in GlareSpreadKernel, which tools/glare_kernel_fit.py writes].
## - Sources: only the light the display cannot show is spread [call: the viewer's own eye already scatters what the
##   display emits; Mantiuk et al. 2009 "Modeling a generic tone-mapping operator" style display glare]: per pixel
##   max(0, Y - SOURCE_WHITE) of the pixel's colour, in scene units at the frame's global gain (SOURCE_WHITE =
##   Atmosphere.DAY_EXPOSURE, the white plate). The Sun's disc (clamped in fp16) is masked out and gets its ANALYTIC veil
##   instead (sun_veil_cd_m2: the form of Atmosphere.glare_veil per pixel, in and out of frame).
## - Shown: each source's veil at its source's local adaptation factor (source_factor; T13 fix 1).
## - Spread: f is a sum of Gaussians (GlareSpreadKernel.WEIGHTS, sigma_k = 0.1 deg x 2^k, non-negative weights fitted offline,
##   max error 46 % at 0.1 deg, <= 3 % over 0.5 .. 30 deg) evaluated on a pyramid: level k has a texel of sigma_k /
##   TEXELS_PER_SIGMA (every level blurs with the same sigma in texels), the levels are built with a linear-tent filter
##   (energy and first moment kept at every step: a moving source does not make the veil pump), blurred, weighted and
##   summed coarse to fine with bilinear up-sampling (GlareSpreadEffect; this class is the CPU mirror and the model).
## The unit of a level texel is the mean source luminance M (scene units) over its solid angle: M_k = E_sum / dOmega_k.
## Small-angle (planar) geometry at the pixels-per-degree of the view axis; a wide view stretches the angles by up to
## 1 / cos^2 at its edge (open point in eye-adaptation.md).

## Fidelity flag: the spread on (default) or off (user argument --glare_spread=off: no spread, the Sun's veil is off too;
## Godot's glow stays off unless --glow=on).
static var enabled := _enabled_from_arguments(OS.get_cmdline_user_args())

## [call] Texels of a level per sigma of its Gaussian (the instructions: texel <= sigma_k / 1.5); with a base reduction of
## floor(ppd / (TEXELS_PER_SIGMA / sigma_0)) the actual ratio is 1.5 .. 3.
const TEXELS_PER_SIGMA := 1.5
## [call] Gaussian taps reach this many sigmas.
const TAP_SIGMAS := 3.0
## [S] the scene value of the display's white plate: sources below it are shown by the display itself.
const SOURCE_WHITE := Atmosphere.DAY_EXPOSURE
## [call] Largest scene value of a pixel (as the field meter and the shaders clamp: fp16 range).
const MAX_SCENE := 65000.0
const LUMINANCE_WEIGHTS := LocalAdaptation.LUMINANCE_WEIGHTS

## [S: LocalAdaptation.PLATEAU_SCENE] where the local adaptation shows a locally adapted region's mean (scene units).
const PLATEAU_SCENE := LocalAdaptation.PLATEAU_SCENE

enum Form { CIE_146, STILES_HOLLADAY }


static func _enabled_from_arguments(arguments: PackedStringArray) -> bool:
	for argument in arguments:
		if argument == "--glare_spread=off":
			return false
	return true


## (1 + (A / 62.5)^4): the age term of the CIE 146 function.
static func age_factor(age_years := GlareSpreadKernel.AGE_YEARS) -> float:
	return 1.0 + pow(age_years / 62.5, 4.0)


## f(theta) (1/sr), theta in degrees (0.1 .. 100 valid).
static func cie146(theta_deg: float) -> float:
	var p := GlareSpreadKernel.PIGMENTATION
	return 10.0 / pow(theta_deg, 3.0) + (5.0 / (theta_deg * theta_deg) + 0.1 * p / theta_deg) * age_factor() + 0.0025 * p


## The form of Atmosphere.glare_veil (Stiles-Holladay, 10 / theta^2), 1/sr.
static func stiles_holladay(theta_deg: float) -> float:
	return Atmosphere.GLARE_VEIL_COEFFICIENT / (theta_deg * theta_deg)


## The kernel the pyramid evaluates: sum_k w_k g_k(theta), g_k the normalised 2D Gaussian per sr.
static func fitted(theta_deg: float) -> float:
	var sum := 0.0
	for k in range(GlareSpreadKernel.WEIGHTS.size()):
		var sigma := sigma_deg(k)
		var sigma_rad := deg_to_rad(sigma)
		sum += float(GlareSpreadKernel.WEIGHTS[k]) * exp(-0.5 * pow(theta_deg / sigma, 2.0)) / (TAU * sigma_rad * sigma_rad)
	return sum


static func sigma_deg(level: int) -> float:
	return GlareSpreadKernel.SIGMA0_DEG * pow(2.0, level)


## Planar energy of the fitted kernel between two angles (fraction of the source's illuminance, analytic).
static func fitted_energy(min_deg: float, max_deg: float) -> float:
	var sum := 0.0
	for k in range(GlareSpreadKernel.WEIGHTS.size()):
		var s2 := 2.0 * sigma_deg(k) * sigma_deg(k)
		sum += float(GlareSpreadKernel.WEIGHTS[k]) * (exp(-min_deg * min_deg / s2) - exp(-max_deg * max_deg / s2))
	return sum


## Planar energy of f between two angles (numeric, log-spaced trapezoid).
static func cie146_energy(min_deg: float, max_deg: float, form := Form.CIE_146) -> float:
	var steps := 4000
	var sum := 0.0
	var previous_t := min_deg
	var previous_v := (cie146(min_deg) if form == Form.CIE_146 else stiles_holladay(min_deg)) * deg_to_rad(min_deg)
	for i in range(1, steps + 1):
		var t := min_deg * pow(max_deg / min_deg, float(i) / steps)
		var v := (cie146(t) if form == Form.CIE_146 else stiles_holladay(t)) * deg_to_rad(t)
		sum += 0.5 * (previous_v + v) * deg_to_rad(t - previous_t) * TAU
		previous_t = t
		previous_v = v
	return sum


## The Sun's veil at an angle theta from the line of sight (a pixel's ray), cd/m^2: f(max(theta, GLARE_MIN_ANGLE_DEG)) x
## E_gl, E_gl = normal illuminance x cos theta (the plane normal to the line of sight; nothing at 90 deg or more), times
## the visibility (Atmosphere.last_veil_visibility). Form.STILES_HOLLADAY is Atmosphere.glare_veil's form (1/theta^2).
static func sun_veil_cd_m2(normal_lux: float, theta_deg: float, visibility: float, form := Form.CIE_146) -> float:
	var cos_theta := cos(deg_to_rad(theta_deg))
	if cos_theta <= 0.0 or normal_lux <= 0.0:
		return 0.0
	var t := maxf(theta_deg, Atmosphere.GLARE_MIN_ANGLE_DEG)
	var f := cie146(t) if form == Form.CIE_146 else stiles_holladay(t)
	return f * normal_lux * cos_theta * visibility


# ---------------------------------------------------------------------------------------------------------------------
# The pyramid (CPU mirror of GlareSpreadEffect; one channel; images as row-major PackedFloat32Array).
# ---------------------------------------------------------------------------------------------------------------------

## Pixels per degree on the view axis (vertical).
static func pixels_per_degree(height_px: float, projection_y_scale: float) -> float:
	return LocalAdaptation.pixels_per_degree(height_px, projection_y_scale)


## The base reduction (px per level-0 texel): the largest integer that keeps a level-0 texel at or under sigma_0 /
## TEXELS_PER_SIGMA (1 when the frame is coarser than that).
static func base_factor(ppd: float) -> int:
	return maxi(1, int(floor(ppd * GlareSpreadKernel.SIGMA0_DEG / TEXELS_PER_SIGMA)))


## sigma of every level's Gaussian in texels (the same for all levels: texel and sigma double together).
static func sigma_texels(ppd: float) -> float:
	return GlareSpreadKernel.SIGMA0_DEG * ppd / float(base_factor(ppd))


## [call, measured] The variance (texels^2 of the blurred level) the pyramid's own filters add: the tent [1, 3, 3, 1] / 8 of
## the down-sampling (0.25 summed over the levels) and the chained bilinear up-sampling (0.22). The Gaussian taps take it out
## of the target variance, so the whole is the sigma_k Gaussian (without it the veil was 20 % high at 3 deg at 6.6 px/deg).
const RECONSTRUCTION_VARIANCE := 0.47
## [call] The narrowest blur taps (texels): at a sigma_k near the reconstruction's own spread the blur is the identity.
const MIN_BLUR_SIGMA := 0.2
## [call] The final bilinear read of level 0 at base factors above 1 (a tent of the pixel/texel ratio): variance, texels^2.
const RECONSTRUCTION_VARIANCE_FINAL := 0.17


## The pyramid's plan for a view: {"factor": f0, "merged": k0, "weights": per level, "sigmas": per level (taps' sigma, texels)}.
## A frame coarser than sigma_0 / TEXELS_PER_SIGMA (under 15 px/deg: every desktop view) cannot hold the narrow Gaussians on
## its texels: the levels whose sigma is under TEXELS_PER_SIGMA texels (k <= k0) are merged into level 0 as ONE Gaussian
## (the weights added, the variances matched: a sub-pixel core the display cannot resolve either) and the coarser levels
## k0 + j follow at texels of 2^j px, so every level j >= 1 keeps sigma >= TEXELS_PER_SIGMA texels. Level 0 is not
## reconstructed by up-sampling (no variance to take out); the levels above it are (RECONSTRUCTION_VARIANCE).
static func plan(ppd: float) -> Dictionary:
	var factor := base_factor(ppd)
	var k0 := 0
	while factor == 1 and GlareSpreadKernel.SIGMA0_DEG * pow(2.0, k0) * ppd < TEXELS_PER_SIGMA \
			and k0 < GlareSpreadKernel.WEIGHTS.size() - 2:
		k0 += 1
	var weights: Array = []
	var sigmas: Array = []
	var w_sum := 0.0
	var var_sum := 0.0
	for k in range(k0 + 1):
		var w := float(GlareSpreadKernel.WEIGHTS[k])
		w_sum += w
		var_sum += w * pow(sigma_deg(k), 2.0)
	weights.append(w_sum)
	var first := sqrt(var_sum / w_sum) * ppd / float(factor)
	sigmas.append(maxf(sqrt(maxf(first * first - (RECONSTRUCTION_VARIANCE_FINAL if factor > 1 else 0.0), 0.0)), MIN_BLUR_SIGMA))
	var texel_sigma := GlareSpreadKernel.SIGMA0_DEG * pow(2.0, k0) * ppd / float(factor)
	for k in range(k0 + 1, GlareSpreadKernel.WEIGHTS.size()):
		weights.append(float(GlareSpreadKernel.WEIGHTS[k]))
		sigmas.append(maxf(sqrt(maxf(texel_sigma * texel_sigma - RECONSTRUCTION_VARIANCE, 0.0)), MIN_BLUR_SIGMA))
	return {"factor": factor, "merged": k0, "weights": weights, "sigmas": sigmas}


static func blur_radius(sigma_texels_value: float) -> int:
	return maxi(1, int(ceil(TAP_SIGMAS * sigma_texels_value)))


static func level_sizes(width: int, height: int, factor: int, count := -1) -> Array[Vector2i]:
	var sizes: Array[Vector2i] = []
	var size := Vector2i((width - 1) / factor + 1, (height - 1) / factor + 1)
	for k in range(GlareSpreadKernel.WEIGHTS.size() if count < 0 else count):
		sizes.append(size)
		size = Vector2i((size.x + 1) / 2, (size.y + 1) / 2)
	return sizes


## Level 0 from the source image (per pixel: the excess luminance, one channel; ratio: dOmega_px / dOmega_axis, empty = 1):
## M_0(j) = sum_px tent(px - c_j) excess ratio / f0^2 with the tent of half-width f0 around the texel centre c_j =
## (j + 0.5) f0 (a partition of unity: the energy is kept).
static func source_level(excess: PackedFloat32Array, ratio: PackedFloat32Array, width: int, height: int, factor: int,
		size: Vector2i) -> PackedFloat32Array:
	var out := PackedFloat32Array()
	out.resize(size.x * size.y)
	var f := float(factor)
	for ty in range(size.y):
		var cy := (ty + 0.5) * f
		for tx in range(size.x):
			var cx := (tx + 0.5) * f
			var sum := 0.0
			for y in range(int(ceil(cy - f - 0.5)), int(floor(cy + f - 0.5)) + 1):
				if y < 0 or y >= height:
					continue
				var wy := 1.0 - absf(y + 0.5 - cy) / f
				if wy <= 0.0:
					continue
				for x in range(int(ceil(cx - f - 0.5)), int(floor(cx + f - 0.5)) + 1):
					if x < 0 or x >= width:
						continue
					var wx := 1.0 - absf(x + 0.5 - cx) / f
					if wx <= 0.0:
						continue
					var i := y * width + x
					sum += wx * wy * excess[i] * (ratio[i] if not ratio.is_empty() else 1.0)
			out[ty * size.x + tx] = sum / (f * f)
	return out


## Level k + 1 from level k: the linear-tent filter [1, 3, 3, 1] / 8 per axis around the texel centre between two source
## texels (zero outside). It keeps the energy and the first moment: a source's centroid does not depend on the grid phase.
## The tent of a source texel at the frame edge reaches one output texel beyond it: that share is folded back into the edge
## texel (down_weights), so a source near the edge loses no energy at the coarse levels.
static func down_weights(in_count: int, out_count: int) -> Array:
	var rows := []
	for t in range(out_count):
		var row := PackedFloat32Array()
		row.resize(in_count)
		for d in range(4):
			var x := 2 * t - 1 + d
			if x >= 0 and x < in_count:
				row[x] += [1.0, 3.0, 3.0, 1.0][d] / 8.0
		if t == 0:
			row[0] += 1.0 / 8.0
		if t == out_count - 1 and in_count == 2 * out_count:
			row[in_count - 1] += 1.0 / 8.0
		rows.append(row)
	return rows


static func down(level: PackedFloat32Array, size: Vector2i, out_size: Vector2i) -> PackedFloat32Array:
	var wx := down_weights(size.x, out_size.x)
	var wy := down_weights(size.y, out_size.y)
	var tmp := PackedFloat32Array()
	tmp.resize(size.y * out_size.x)
	for y in range(size.y):
		for tx in range(out_size.x):
			var row: PackedFloat32Array = wx[tx]
			var sum := 0.0
			for x in range(maxi(0, 2 * tx - 1), mini(size.x, 2 * tx + 3)):
				sum += row[x] * level[y * size.x + x]
			tmp[y * out_size.x + tx] = sum
	var out := PackedFloat32Array()
	out.resize(out_size.x * out_size.y)
	for ty in range(out_size.y):
		var row: PackedFloat32Array = wy[ty]
		for tx in range(out_size.x):
			var sum := 0.0
			for y in range(maxi(0, 2 * ty - 1), mini(size.y, 2 * ty + 3)):
				sum += row[y] * tmp[y * out_size.x + tx]
			out[ty * out_size.x + tx] = sum
	return out


static func blur_taps(sigma_texels_value: float) -> PackedFloat32Array:
	var radius := blur_radius(sigma_texels_value)
	var taps := PackedFloat32Array()
	var total := 0.0
	for d in range(-radius, radius + 1):
		var v := exp(-0.5 * d * d / (sigma_texels_value * sigma_texels_value))
		taps.append(v)
		total += v
	for i in range(taps.size()):
		taps[i] /= total
	return taps


## Separable blur with the (normalised) Gaussian taps, zero outside the image.
static func blur(level: PackedFloat32Array, size: Vector2i, taps: PackedFloat32Array) -> PackedFloat32Array:
	var radius := (taps.size() - 1) / 2
	var tmp := PackedFloat32Array()
	tmp.resize(level.size())
	for y in range(size.y):
		for x in range(size.x):
			var sum := 0.0
			for d in range(-radius, radius + 1):
				var xx := x + d
				if xx >= 0 and xx < size.x:
					sum += taps[d + radius] * level[y * size.x + xx]
			tmp[y * size.x + x] = sum
	var out := PackedFloat32Array()
	out.resize(level.size())
	for y in range(size.y):
		for x in range(size.x):
			var sum := 0.0
			for d in range(-radius, radius + 1):
				var yy := y + d
				if yy >= 0 and yy < size.y:
					sum += taps[d + radius] * tmp[yy * size.x + x]
			out[y * size.x + x] = sum
	return out


## Bilinear sample of an image at the fractional texel index (u, v) (texel i at index i), edges clamped.
static func bilinear(image: PackedFloat32Array, size: Vector2i, u: float, v: float) -> float:
	var x0 := int(floor(u))
	var y0 := int(floor(v))
	var fx := u - x0
	var fy := v - y0
	var xa := clampi(x0, 0, size.x - 1)
	var xb := clampi(x0 + 1, 0, size.x - 1)
	var ya := clampi(y0, 0, size.y - 1)
	var yb := clampi(y0 + 1, 0, size.y - 1)
	return (1.0 - fy) * ((1.0 - fx) * image[ya * size.x + xa] + fx * image[ya * size.x + xb]) \
			+ fy * ((1.0 - fx) * image[yb * size.x + xa] + fx * image[yb * size.x + xb])


## The whole pyramid: the veil image at level 0 (A_0, scene units) of the excess image. Returns {"a0", "size", "factor"}.
static func spread(excess: PackedFloat32Array, ratio: PackedFloat32Array, width: int, height: int,
		ppd: float) -> Dictionary:
	var the_plan := plan(ppd)
	var factor: int = the_plan["factor"]
	var sizes := level_sizes(width, height, factor, the_plan["weights"].size())
	var levels: Array[PackedFloat32Array] = [source_level(excess, ratio, width, height, factor, sizes[0])]
	for k in range(1, sizes.size()):
		levels.append(down(levels[k - 1], sizes[k - 1], sizes[k]))
	var accumulated := PackedFloat32Array()
	for k in range(sizes.size() - 1, -1, -1):
		var blurred := blur(levels[k], sizes[k], blur_taps(float(the_plan["sigmas"][k])))
		var weight := float(the_plan["weights"][k])
		for i in range(blurred.size()):
			blurred[i] *= weight
		if k < sizes.size() - 1:
			for y in range(sizes[k].y):
				for x in range(sizes[k].x):
					blurred[y * sizes[k].x + x] += bilinear(accumulated, sizes[k + 1], (x - 0.5) * 0.5, (y - 0.5) * 0.5)
		accumulated = blurred
	return {"a0": accumulated, "size": sizes[0], "factor": factor}


## The veil at a full-resolution pixel (bilinear in A_0).
static func veil_at(result: Dictionary, x: int, y: int) -> float:
	var factor := float(result["factor"])
	return bilinear(result["a0"], result["size"], (x + 0.5) / factor - 0.5, (y + 0.5) / factor - 0.5)


## The local adaptation factor a source pixel of scene luminance y_scene (at the global gain) is shown at, and with it
## its veil (look fixes T13 fix 1, user / orchestrator 2026-11-08). The veil is light on the retina, tied to its source:
## the eye adapted to the full Moon / the Earth / a lamp sees the scatter around it relative to that source - the source
## and its halo are one framework [E: Gilchrist et al. 1999 anchoring, the local adaptation's own basis
## (LocalAdaptation.PLATEAU_SCENE)], so the halo is compressed with the disc and keeps its physical ratio to it (CIE 146:
## 1e-3 of the full Moon's disc at 1 deg, 1.6e-4 at 2 deg). [call] The factor is the local adaptation's Weber-range form
## c = PLATEAU / y (a region adapted to y shows its mean at the plateau; LocalAdaptation), floored at 1 / g (the cone gain's
## floor 1: the Sun's disc, a lamp filament stay display white) and at most 1 (the step only lowers the gain). Measured
## alternatives (`.superpowers/sdd/2026-11-07-look-fixes/t13-fix1-report.md`): the veil added at the global gain (T13)
## showed the halo at 0.1-0.5 of the disc at 1 deg (a glowing ball); the veil fed into the local adaptation's pooling
## flattened the smooth halo to the plateau (a dark ring at the limb, a bright ring at 1-2 deg). The field meter still
## takes the full veil (the disability glare raises the adaptation). inverse_gain: 1 / g; 1 without the local adaptation.
static func source_factor(y_scene: float, inverse_gain: float) -> float:
	if y_scene <= 0.0:
		return 1.0
	return clampf(PLATEAU_SCENE / y_scene, minf(inverse_gain, 1.0), 1.0)


## The source term of a pixel of scene luminance y_scene (colour kept: the caller scales rgb by this over y_scene).
static func excess_luminance(y_scene: float) -> float:
	return maxf(minf(y_scene, MAX_SCENE) - SOURCE_WHITE, 0.0)
