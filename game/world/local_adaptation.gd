class_name LocalAdaptation
extends RefCounted
## Local adaptation (look fixes T10, grid-phase invariant since T13; user 2026-11-08): small bright objects - the Earth seen
## from the lunar night, the full Moon in the Earth's night sky, lamps in a dark cabin - keep their detail while the global
## field rule stays for the rest of the view. CPU mirror of LocalAdaptationEffect's shaders (tests, offline numbers); the
## two must stay identical.
##
## The eye adapts locally: the adapting luminance of a retinal point is pooled over a small neighbourhood, a Gaussian
## of sigma 0.428 deg [S: Vangorp, Myszkowski, Graf & Mantiuk 2015, "A model of local adaptation", ACM TOG 34(6) 166,
## Eq. 10; their Experiment 3: the adapting field levels off at a disc of ~0.5 deg diameter, rings beyond ~6 deg have no
## measurable effect]. Light adaptation of the cones is fast (Pattanaik et al. 2000: ~0.1 s neural, seconds for the
## pigment), dark adaptation slow (minutes): so a bright patch the eye looks at lowers the gain there within a glance, while
## a dark patch in a bright view is NOT brightened (the global gain already is the darker-adapted one). The step therefore
## only ever lowers the gain:
##     g_px = min(g, G(L_eff)),   L_eff = L_loc x DAY_EXPOSURE / (PLATEAU x d^2)   (in Atmosphere's scene units)
## g the global display gain (cone gain x brightness factor), G the same cone gain function the global rule uses
## (EyeAdaptation.cone_gain: Ferwerda's t.v.i., in [1, GAIN_MAX]), L_loc the pooled local luminance (cd/m^2). In the Weber
## range G = L_ref / L_eff, so a locally adapted region shows its local mean at the scene value PLATEAU [E: anchoring,
## Gilchrist et al. 1999 as in the anchored rule - the bright region is the anchor of its own neighbourhood and is seen
## as a white surface of reflectance EyeAdaptation.ANCHOR_WHITE under the TOA Sun: PLATEAU = ANCHOR_WHITE x DAY_EXPOSURE,
## below the display white, so its detail (contrast within ~0.4 deg) stays visible]; the display value of a region rises
## with L_loc up to PLATEAU, stays there, and rises again where G reaches its floor 1 (L_eff above the white plate under the
## TOA Sun: the Sun's disc, a lamp filament stay display white) - monotonic in L_loc, no inversion.
## Geometry [call]: one block size per frame, from view 0's pixels per degree at the image CENTRE, used by both eyes. For a
## wide rectilinear frustum the pixels per degree grow toward the edge, so the pooling is wider in degrees at the periphery.
## A region whose global display value is below PLATEAU is untouched (pad noon, lunar day, the Earth from orbit by day).
##
## Pooling = a piecewise-linear bilateral filter [S: Durand & Dorsey 2002, "Fast bilateral filtering for the display of
## high-dynamic-range images", SIGGRAPH 2002 (the range-sampled, spatially downsampled evaluation); Paris & Durand 2006],
## edge-aware so there is no dark ring around a bright disc and no bright limb (call: a display requirement - Vangorp's plain
## Gaussian would darken a 0.4 deg ring of the dark field and brighten the limb, the classic local tone-mapping halo).
## Log-luminance BANDS j (centres c_j, spacing <= RANGE_SIGMA_DECADES, from LOWEST_BAND_RATIO x PLATEAU up to the fp16
## clamp; a pixel beyond the ends uses the end band, the lowest band takes every darker pixel at weight 1). For each band
## the range weight of every FULL-RESOLUTION pixel q is a Gaussian of (log L_q - c_j): sigma RANGE_SIGMA_DECADES for a
## neighbour at or below the band, BRIGHT_NEIGHBOUR_SIGMA_DECADES for a neighbour above it [call: a pixel dimmer than a
## bright disc, e.g. the disc's antialiased edge pixels, shares the disc's factor instead of staying at c = 1 as a white
## sparkle; a pixel 4+ decades under (the dark field) is still unlike it]. w_j L and w_j are box-downsampled (area exact,
## linear luminance) to the block grid, blurred by a separable Gaussian (sigma POOL_SIGMA_DEG, at least BLOCKS_PER_SIGMA
## texels, radius KERNEL_RADIUS_SIGMAS sigmas, in-bounds taps renormalised): box x Gaussian is shift invariant up to the
## kernel's aliasing ~exp(-2 pi^2 (sigma/texel)^2) (< 1e-19 at 1.5), where the old block grid with its hard-ish like-block
## share and joint bilateral upsampling varied x1.1-1.9 for a 10-32 px disc moving over the grid. A pixel bilinearly reads
## the blurred band maps of its own two neighbouring bands (its own log L, linear in log L): L_loc = (1-t) R_j + t R_(j+1),
## R_j = Lc_j (blur(w_j L / Lc_j) / blur(w_j)), and the share S = (1-t) blur(w_j) + t blur(w_(j+1)) (the neighbourhood's
## Gaussian-weighted fraction that looks like the pixel). A pixel unlike everything near it (an emitter drawn after the
## opaque pass, a sub-block glint: S below MIN_WEIGHT_SHARE) keeps c = 1; between MIN_WEIGHT_SHARE and
## FULL_WEIGHT_SHARE_RATIO x it log c blends in by a smoothstep. No temporal low-pass (it cannot follow a moving object:
## a ghost behind a 30 deg/s head turn).

## [S: Vangorp et al. 2015 Eq. 10, sigma_1] The pooling Gaussian's sigma (deg of visual angle).
const POOL_SIGMA_DEG := 0.428
## [S: Durand & Dorsey 2002: sigma_r = 0.4 in log10 luminance] the range sigma of the edge-aware pooling, also the largest
## band spacing (Durand: sample the range at about sigma_r).
const RANGE_SIGMA_DECADES := 0.4
## [E: Gilchrist et al. 1999, see above] The scene value a locally adapted region shows its local mean at.
const PLATEAU_SCENE := EyeAdaptation.ANCHOR_WHITE * Atmosphere.DAY_EXPOSURE
## [call] The pooling sigma in texels the block size aims at (at least this many).
const BLOCKS_PER_SIGMA := 1.5
## [call] The blur kernel's radius in sigmas (3: the truncation jump exp(-4.5) = 1 %).
const KERNEL_RADIUS_SIGMAS := 3.0
## [call] Largest block edge (px); the block size is a power of two up to it.
const MAX_BLOCK := 32
## [call] The lowest band's centre as a share of PLATEAU_SCENE: regions below it (display 1/16 of the plateau) are never
## compressed by a brighter neighbourhood (4+ decades under any compressible region).
const LOWEST_BAND_RATIO := 1.0 / 16.0
## [call] sigma (log10) of the range weight of a neighbour BRIGHTER than the band (see above).
const BRIGHT_NEIGHBOUR_SIGMA_DECADES := 1.0
## [call] A pixel whose share of like pixels in the pooling neighbourhood is below this keeps c = 1.
const MIN_WEIGHT_SHARE := 0.01
## [call] Above MIN_WEIGHT_SHARE x this the like share counts fully; between the two the factor blends in (log domain)
## from 1, so a pixel crossing the threshold under motion changes c continuously.
const FULL_WEIGHT_SHARE_RATIO := 4.0
## [call] A band whose blurred weight is below this is treated as empty (its ratio is the band centre).
const EMPTY_WEIGHT := 1.0e-9
## [call] The largest scene luminance a pixel takes (as the field meter and the shaders clamp: fp16 range).
const MAX_SCENE_LUMINANCE := 65000.0
## [S: ITU-R BT.709] luminance weights.
const LUMINANCE_WEIGHTS := Vector3(0.2126, 0.7152, 0.0722)

## Fidelity flag: local adaptation on (default) or off (user argument --local_adaptation=off: the global rule alone).
static var enabled := _enabled_from_arguments(OS.get_cmdline_user_args())


static func _enabled_from_arguments(arguments: PackedStringArray) -> bool:
	for argument in arguments:
		if argument == "--local_adaptation=off":
			return false
	return true


## The block edge (px, a power of two in [1, MAX_BLOCK]) that puts at least BLOCKS_PER_SIGMA blocks into the pooling
## sigma at this many pixels per degree.
static func block_size(pixels_per_degree: float) -> int:
	var sigma_px := POOL_SIGMA_DEG * pixels_per_degree
	var block := 1
	while block * 2 <= MAX_BLOCK and sigma_px / float(block * 2) >= BLOCKS_PER_SIGMA:
		block *= 2
	return block


## The box downsample's sample step (px): 1 = every pixel (area exact; shift invariance needs it).
static func sample_step(_block: int) -> int:
	return 1


## Pixels per degree at the image centre of a view: half the image height x the projection's y scale (cot of half the
## vertical field for a symmetric frustum), per radian.
static func pixels_per_degree(height_px: float, projection_y_scale: float) -> float:
	return 0.5 * height_px * projection_y_scale * PI / 180.0


## log2 of the lowest band's centre (scene units).
static func band_lowest_log2() -> float:
	return log(PLATEAU_SCENE * LOWEST_BAND_RATIO) / log(2.0)


## Number of bands (even: two per RGBA texture): spacing at most RANGE_SIGMA_DECADES from the lowest centre to the fp16 clamp.
static func band_count() -> int:
	var span := log(MAX_SCENE_LUMINANCE) / log(2.0) - band_lowest_log2()
	var n := int(ceil(span / (RANGE_SIGMA_DECADES * log(10.0) / log(2.0)))) + 1
	return n + (n % 2)


## The band spacing in log2 units (the bands are spread evenly over the span).
static func band_step_log2() -> float:
	return (log(MAX_SCENE_LUMINANCE) / log(2.0) - band_lowest_log2()) / float(band_count() - 1)


## L_eff per L_loc: DAY_EXPOSURE / (PLATEAU x d^2) written as (scene value per cd/m^2 at gain 1) x L_ref / PLATEAU.
static func effective_per_local(scene_per_cd_m2: float, display_gain: float, reference_cd_m2: float) -> float:
	if display_gain <= 0.0:
		return 0.0
	return scene_per_cd_m2 / display_gain * reference_cd_m2 / PLATEAU_SCENE


## The local factor c = min(1, G(L_eff) / g) for a pooled local luminance (cd/m^2).
static func factor(local_cd_m2: float, display_gain: float, reference_cd_m2: float, effective_per_local_value: float) -> float:
	if display_gain <= 0.0 or local_cd_m2 <= 0.0:
		return 1.0
	var gain := EyeAdaptation.cone_gain(local_cd_m2 * effective_per_local_value, reference_cd_m2)
	return minf(1.0, gain / display_gain)


## One half of the 1-D blur kernel, taps[i] = exp(-i^2 / (2 sigma^2)), i = 0 .. radius.
static func blur_taps(sigma_blocks: float) -> PackedFloat64Array:
	var radius := int(ceil(KERNEL_RADIUS_SIGMAS * sigma_blocks))
	var taps := PackedFloat64Array()
	for i in range(radius + 1):
		taps.append(exp(-float(i * i) / (2.0 * sigma_blocks * sigma_blocks)))
	return taps


## Separable Gaussian blur of a lw x lh plane, in-bounds taps renormalised (the shader's blur pass, both directions).
static func _blur_plane(plane: PackedFloat64Array, lw: int, lh: int, taps: PackedFloat64Array) -> PackedFloat64Array:
	var radius := taps.size() - 1
	var tmp := PackedFloat64Array()
	tmp.resize(lw * lh)
	for y in range(lh):
		for x in range(lw):
			var s := 0.0
			var m := 0.0
			for i in range(maxi(-radius, -x), mini(radius, lw - 1 - x) + 1):
				var g := taps[absi(i)]
				s += g * plane[y * lw + x + i]
				m += g
			tmp[y * lw + x] = s / m
	var out := PackedFloat64Array()
	out.resize(lw * lh)
	for y in range(lh):
		for x in range(lw):
			var s := 0.0
			var m := 0.0
			for i in range(maxi(-radius, -y), mini(radius, lh - 1 - y) + 1):
				var g := taps[absi(i)]
				s += g * tmp[(y + i) * lw + x]
				m += g
			out[y * lw + x] = s / m
	return out


## CPU mirror of the whole step on a luminance image (row-major scene luminances, width x height): returns the factor c
## per pixel. parameters: scene_per_cd_m2, display_gain, reference_cd_m2, block (px), sigma_blocks. final: the image after
## the transparent pass the factor is applied to (empty: the same image; the bands are built from image).
static func factors(image: PackedFloat64Array, width: int, height: int, parameters: Dictionary,
		final := PackedFloat64Array()) -> PackedFloat64Array:
	if final.is_empty():
		final = image
	var spc: float = parameters["scene_per_cd_m2"]
	var gain: float = parameters["display_gain"]
	var reference: float = parameters["reference_cd_m2"]
	var block: int = parameters["block"]
	var sigma_blocks: float = parameters["sigma_blocks"]
	var lw := (width - 1) / block + 1
	var lh := (height - 1) / block + 1
	var count := band_count()
	var c0 := band_lowest_log2()
	var bstep := band_step_log2()
	var inv_lo := range_inverse_two_sigma_squared()
	var inv_hi := range_inverse_two_sigma_squared(BRIGHT_NEIGHBOUR_SIGMA_DECADES)
	var scale := PackedFloat64Array()  # 1 / Lc_j
	for k in range(count):
		scale.append(pow(2.0, -(c0 + float(k) * bstep)))
	# 1. per band: the range weights of every pixel, box-downsampled (area exact)
	var num: Array[PackedFloat64Array] = []
	var den: Array[PackedFloat64Array] = []
	for k in range(count):
		var a := PackedFloat64Array()
		a.resize(lw * lh)
		num.append(a)
		var b := PackedFloat64Array()
		b.resize(lw * lh)
		den.append(b)
	var step := sample_step(block)
	var accum_l := PackedFloat64Array()
	var accum_w := PackedFloat64Array()
	accum_l.resize(count)
	accum_w.resize(count)
	for ty in range(lh):
		for tx in range(lw):
			accum_l.fill(0.0)
			accum_w.fill(0.0)
			var n := 0
			for j in range(step / 2, block, step):
				for i in range(step / 2, block, step):
					var x := tx * block + i
					var y := ty * block + j
					if x >= width or y >= height:
						continue
					var lum := clampf(image[y * width + x], 0.0, MAX_SCENE_LUMINANCE)
					var lx := log(maxf(lum, 1.0e-30)) / log(2.0)
					for k in range(count):
						var d := lx - (c0 + float(k) * bstep)
						if k == 0:
							d = maxf(d, 0.0)
						var w := exp(-d * d * (inv_hi if d > 0.0 else inv_lo))
						accum_w[k] += w
						accum_l[k] += w * lum * scale[k]
					n += 1
			for k in range(count):
				num[k][ty * lw + tx] = accum_l[k] / float(n)
				den[k][ty * lw + tx] = accum_w[k] / float(n)
	# 2. blur every band's two planes
	var taps := blur_taps(sigma_blocks)
	for k in range(count):
		num[k] = _blur_plane(num[k], lw, lh, taps)
		den[k] = _blur_plane(den[k], lw, lh, taps)
	# 3. per pixel: its own two bands, bilinear in space
	var k_eff := effective_per_local(spc, gain, reference)
	var out := PackedFloat64Array()
	out.resize(width * height)
	for y in range(height):
		for x in range(width):
			out[y * width + x] = 1.0
			var own := final[y * width + x]
			if not (own > 0.0):
				continue
			var lx := log(minf(own, MAX_SCENE_LUMINANCE)) / log(2.0)
			var s := clampf((lx - c0) / bstep, 0.0, float(count - 1))
			var band := mini(int(floor(s)), count - 2)
			var t := s - float(band)
			var tcx := (float(x) + 0.5) / float(block) - 0.5
			var tcy := (float(y) + 0.5) / float(block) - 0.5
			var bx := int(floor(tcx))
			var by := int(floor(tcy))
			var fx := tcx - float(bx)
			var fy := tcy - float(by)
			var x0 := clampi(bx, 0, lw - 1)
			var x1 := clampi(bx + 1, 0, lw - 1)
			var y0 := clampi(by, 0, lh - 1)
			var y1 := clampi(by + 1, 0, lh - 1)
			var loc := 0.0
			var share := 0.0
			for m in range(2):
				var kk := band + m
				var weight_t := (1.0 - t) if m == 0 else t
				var a := lerpf(lerpf(num[kk][y0 * lw + x0], num[kk][y0 * lw + x1], fx),
						lerpf(num[kk][y1 * lw + x0], num[kk][y1 * lw + x1], fx), fy)
				var w := lerpf(lerpf(den[kk][y0 * lw + x0], den[kk][y0 * lw + x1], fx),
						lerpf(den[kk][y1 * lw + x0], den[kk][y1 * lw + x1], fx), fy)
				var centre := 1.0 / scale[kk]
				loc += weight_t * (centre * a / w if w > EMPTY_WEIGHT else centre)
				share += weight_t * w
			var c_loc := factor(loc / spc, gain, reference, k_eff)
			out[y * width + x] = pow(2.0, smooth_share(share) * log(c_loc) / log(2.0))
	return out


## The cones' part of a pixel after the post step (VisionEffect: p = M rgb, M = C + rod_scale ROD_TINT w_s^T): C M^-1.
static func cone_part(rod_scale: float, cone: Basis) -> Basis:
	var tint := EyeAdaptation.ROD_TINT * rod_scale
	var w := EyeAdaptation.SCOTOPIC_WEIGHTS
	var m := Basis(cone.x + tint * w.x, cone.y + tint * w.y, cone.z + tint * w.z)
	return cone * m.inverse()


## A post-step pixel under the local factor c: the cones' part x c, the rods' part x c^2. [E: rod saturation - the rod
## signal saturates at a few hundred scotopic cd/m^2 (Aguilar & Stiles 1954), so a region the cones adapt to locally,
## photopic by construction (its local mean at PLATEAU), has no rod signal]. [call: the SHAPE c^2 - only "the rods' share
## vanishes under strong compression and the step is continuous at c = 1" is justified; any steeper-than-c law that is 1 at
## c = 1 and small by c ~ 1e-2 would do, c^2 is the simplest].
static func apply_pixel(p: Vector3, c: float, cone_part_basis: Basis) -> Vector3:
	var cones := cone_part_basis * p
	return c * (cones + c * (p - cones))


## The blend of log c in from 0 (c = 1) with the like share (smoothstep from MIN_WEIGHT_SHARE to
## FULL_WEIGHT_SHARE_RATIO x it).
static func smooth_share(share: float) -> float:
	var t := clampf((share / MIN_WEIGHT_SHARE - 1.0) / (FULL_WEIGHT_SHARE_RATIO - 1.0), 0.0, 1.0)
	return t * t * (3.0 - 2.0 * t)


## 1 / (2 sigma_r^2) for log2 luminance differences (sigma in log10 decades).
static func range_inverse_two_sigma_squared(sigma_decades := RANGE_SIGMA_DECADES) -> float:
	var sigma_log2 := sigma_decades * log(10.0) / log(2.0)
	return 1.0 / (2.0 * sigma_log2 * sigma_log2)
