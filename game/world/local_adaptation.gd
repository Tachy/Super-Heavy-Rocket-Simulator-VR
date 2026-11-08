class_name LocalAdaptation
extends RefCounted
## Local adaptation (look fixes T10, user 2026-11-08): small bright objects - the Earth seen from the lunar night, the
## full Moon in the Earth's night sky, lamps in a dark cabin - keep their detail while the global field rule (rule D,
## Atmosphere.field_adaptation_luminance) stays for the rest of the view. CPU mirror of LocalAdaptationEffect's shaders
## (tests, offline numbers); the two must stay identical.
##
## The eye adapts locally: the adapting luminance of a retinal point is pooled over a small neighbourhood, a Gaussian
## of sigma 0.428 deg (and a narrower one) [S: Vangorp, Myszkowski, Graf & Mantiuk 2015, "A model of local adaptation",
## ACM TOG 34(6) 166, Eq. 10; their Experiment 3: the adapting field levels off at a disc of ~0.5 deg diameter, rings
## beyond ~6 deg have no measurable effect]. Light adaptation of the cones is fast (Pattanaik et al. 2000: ~0.1 s neural,
## seconds for the pigment), dark adaptation slow (minutes): so a bright patch the eye looks at lowers the gain there
## within a glance, while a dark patch in a bright view is NOT brightened (the global gain already is the darker-adapted
## one). The step therefore only ever lowers the gain:
##     g_px = min(g, G(L_eff)),   L_eff = L_loc x DAY_EXPOSURE / (PLATEAU x d^2)   (in Atmosphere's scene units)
## g the global display gain (cone gain x brightness factor), G the same cone gain function the global rule uses
## (EyeAdaptation.cone_gain: Ferwerda's t.v.i., in [1, GAIN_MAX]), L_loc the pooled local luminance (cd/m^2). In the Weber
## range G = L_ref / L_eff, so a locally adapted region shows its local mean at the scene value PLATEAU [E: anchoring,
## Gilchrist et al. 1999 as in the anchored rule - the bright region is the anchor of its own neighbourhood and is seen
## as a white surface of reflectance EyeAdaptation.ANCHOR_WHITE under the TOA Sun, ANCHOR_WHITE x DAY_EXPOSURE = 1.35:
## below AgX's shoulder (sRGB ~213), so its detail (contrast within ~0.4 deg) stays visible]; the display value
## of a region rises with L_loc up to PLATEAU, stays there, and rises again where G reaches its floor 1 (L_eff above the
## white plate under the TOA Sun: the Sun's disc, a lamp filament stay display white) - monotonic in L_loc, no inversion.
## A region whose global display value is below PLATEAU is untouched (pad noon, lunar day, the Earth from orbit by day).
## Pooling [call, a display requirement, not a property of the eye]: in log luminance and edge-aware (a bilateral
## filter, Durand & Dorsey 2002, "Fast bilateral filtering for the display of high-dynamic-range images", SIGGRAPH 2002:
## range sigma 0.4 in log10) - Vangorp's plain Gaussian would darken a 0.4 deg ring of the dark field around a bright
## disc and brighten its limb (the classic local tone-mapping halo); the physically real maladaptation there is a
## threshold effect, not a visible ring. Image pipeline: the opaque image (with the cloud composite) is averaged into
## blocks of BLOCK x BLOCK pixels (mean log2 luminance), the blocks are bilaterally pooled (sigma POOL_SIGMA_DEG in
## blocks, range RANGE_SIGMA_DECADES), the factor log2 c is evaluated per block and brought back to the pixels by a joint
## bilateral upsampling (Kopf et al. 2007, "Joint bilateral upsampling", SIGGRAPH 2007) against each pixel's own
## luminance (UPSAMPLE_SIGMA_BLOCKS, the same range sigma; 4 x 4 blocks); a pixel unlike every block near it (an emitter
## drawn after the opaque pass, a sub-block glint) keeps c = 1.

## [S: Vangorp et al. 2015 Eq. 10, sigma_1] The pooling Gaussian's sigma (deg of visual angle).
const POOL_SIGMA_DEG := 0.428
## [S: Durand & Dorsey 2002, sec. 5.2: sigma_r = 0.4 in log10 luminance] the range sigma of the edge-aware pooling.
const RANGE_SIGMA_DECADES := 0.4
## [E: Gilchrist et al. 1999, see above] The scene value a locally adapted region shows its local mean at.
const PLATEAU_SCENE := EyeAdaptation.ANCHOR_WHITE * Atmosphere.DAY_EXPOSURE
## [call] The pooling sigma in blocks the block size aims at (at least this many; 1.5 still resolves the Gaussian and
## keeps the headset's pooling at block 8, radius 4: T10 cost).
const BLOCKS_PER_SIGMA := 1.5
## [call] The pooling kernel's radius in sigmas (2: 95 % of a 2-D Gaussian's weight within it).
const KERNEL_RADIUS_SIGMAS := 2.0
## [call] Largest block edge (px); the block size is a power of two up to it.
const MAX_BLOCK := 32
## [call] The joint upsampling's spatial sigma (blocks) over its 4 x 4 block neighbourhood.
const UPSAMPLE_SIGMA_BLOCKS := 1.0
## [call] A pixel whose range-weighted share of the upsampling's spatial weight is below this keeps c = 1 (no block
## near it is like it: an emitter or a sub-block highlight).
const MIN_WEIGHT_SHARE := 0.01
## [S: the field meter's LOG_MIN, AdaptationMeter] luminances below 1e-6 cd/m^2 count as this floor in the log pooling.
const LOG_FLOOR_CD_M2 := 1.0e-6
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


## The block average's sample step (px): every pixel up to 4 x 4 per block, beyond that a 4 x 4 grid (block 8: every
## second pixel per axis) [call: T10 cost - the full-resolution read was the step's largest part].
static func sample_step(block: int) -> int:
	return maxi(1, block / 4)


## Pixels per degree at the image centre of a view: half the image height x the projection's y scale (cot of half the
## vertical field for a symmetric frustum), per radian.
static func pixels_per_degree(height_px: float, projection_y_scale: float) -> float:
	return 0.5 * height_px * projection_y_scale * PI / 180.0


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


## CPU mirror of the whole step on a luminance image (row-major scene luminances, width x height): returns the factor c
## per pixel. parameters: scene_per_cd_m2, display_gain, reference_cd_m2, block (px), sigma_blocks. final: the image after
## the transparent pass the factor is applied to (empty: the same image; the map is built from image).
static func factors(image: PackedFloat64Array, width: int, height: int, parameters: Dictionary,
		final := PackedFloat64Array()) -> PackedFloat64Array:
	if final.is_empty():
		final = image
	var spc: float = parameters["scene_per_cd_m2"]
	var gain: float = parameters["display_gain"]
	var reference: float = parameters["reference_cd_m2"]
	var block: int = parameters["block"]
	var sigma_blocks: float = parameters["sigma_blocks"]
	var floor_log := log2_floor(spc)
	var lw := (width - 1) / block + 1
	var lh := (height - 1) / block + 1
	# 1. blocks: mean log2 luminance
	var blocks := PackedFloat64Array()
	blocks.resize(lw * lh)
	for ty in range(lh):
		for tx in range(lw):
			var s := 0.0
			var n := 0
			var step := sample_step(block)
			for j in range(step / 2, block, step):
				for i in range(step / 2, block, step):
					var x := tx * block + i
					var y := ty * block + j
					if x < width and y < height:
						s += log2_of(image[y * width + x], floor_log)
						n += 1
			blocks[ty * lw + tx] = s / float(n)
	# 2. bilateral pooling and the factor per block
	var k_eff := effective_per_local(spc, gain, reference)
	var range_inv := range_inverse_two_sigma_squared()
	var radius := int(ceil(KERNEL_RADIUS_SIGMAS * sigma_blocks))
	var spatial_inv := 1.0 / (2.0 * sigma_blocks * sigma_blocks)
	var log_c := PackedFloat64Array()
	log_c.resize(lw * lh)
	for ty in range(lh):
		for tx in range(lw):
			var c0 := blocks[ty * lw + tx]
			var sw := 0.0
			var s := 0.0
			for dy in range(-radius, radius + 1):
				for dx in range(-radius, radius + 1):
					var qx := tx + dx
					var qy := ty + dy
					if qx < 0 or qy < 0 or qx >= lw or qy >= lh:
						continue
					var v := blocks[qy * lw + qx]
					var w := exp(-float(dx * dx + dy * dy) * spatial_inv - (v - c0) * (v - c0) * range_inv)
					sw += w
					s += w * v
			var pooled := s / sw
			var local_cd_m2 := pow(2.0, pooled) / spc if pooled > floor_log + 0.5 else 0.0
			log_c[ty * lw + tx] = log(factor(local_cd_m2, gain, reference, k_eff)) / log(2.0)
	# 3. joint bilateral upsampling against the pixel's own luminance
	var out := PackedFloat64Array()
	out.resize(width * height)
	var up_inv := 1.0 / (2.0 * UPSAMPLE_SIGMA_BLOCKS * UPSAMPLE_SIGMA_BLOCKS)
	for y in range(height):
		for x in range(width):
			out[y * width + x] = 1.0
			var own := final[y * width + x]
			if not (own > 0.0):
				continue
			var l := log2_of(own, floor_log)
			var tcx := (float(x) + 0.5) / float(block) - 0.5
			var tcy := (float(y) + 0.5) / float(block) - 0.5
			var bx := int(floor(tcx))
			var by := int(floor(tcy))
			var sw := 0.0
			var ss := 0.0
			var s := 0.0
			for iy in range(-1, 3):
				for ix in range(-1, 3):
					var qx := clampi(bx + ix, 0, lw - 1)
					var qy := clampi(by + iy, 0, lh - 1)
					var dx := float(bx + ix) - tcx
					var dy := float(by + iy) - tcy
					var spatial := exp(-(dx * dx + dy * dy) * up_inv)
					var v := blocks[qy * lw + qx]
					var w := spatial * exp(-(v - l) * (v - l) * range_inv)
					ss += spatial
					sw += w
					s += w * log_c[qy * lw + qx]
			if sw >= MIN_WEIGHT_SHARE * ss:
				out[y * width + x] = pow(2.0, s / sw)
	return out


## The cones' part of a pixel after the post step (VisionEffect: p = M rgb, M = C + rod_scale ROD_TINT w_s^T): C M^-1.
static func cone_part(rod_scale: float, cone: Basis) -> Basis:
	var tint := EyeAdaptation.ROD_TINT * rod_scale
	var w := EyeAdaptation.SCOTOPIC_WEIGHTS
	var m := Basis(cone.x + tint * w.x, cone.y + tint * w.y, cone.z + tint * w.z)
	return cone * m.inverse()


## A post-step pixel under the local factor c: the cones' part x c, the rods' part x c^2 [E: rod saturation - the rod
## signal saturates at a few hundred scotopic cd/m^2 (Aguilar & Stiles 1954), so a region the cones adapt to locally,
## photopic by construction (its local mean at PLATEAU), has no rod signal; c^2: continuous at c = 1].
static func apply_pixel(p: Vector3, c: float, cone_part_basis: Basis) -> Vector3:
	var cones := cone_part_basis * p
	return c * (cones + c * (p - cones))


## log2 of the floor luminance in scene units.
static func log2_floor(scene_per_cd_m2: float) -> float:
	return log(maxf(LOG_FLOOR_CD_M2 * scene_per_cd_m2, 1e-30)) / log(2.0)


static func log2_of(scene_luminance: float, floor_log: float) -> float:
	return maxf(log(scene_luminance) / log(2.0), floor_log) if scene_luminance > 0.0 else floor_log


## 1 / (2 sigma_r^2) for log2 luminance differences.
static func range_inverse_two_sigma_squared() -> float:
	var sigma_log2 := RANGE_SIGMA_DECADES * log(10.0) / log(2.0)
	return 1.0 / (2.0 * sigma_log2 * sigma_log2)
