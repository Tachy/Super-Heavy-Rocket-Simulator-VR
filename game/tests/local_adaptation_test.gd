# Local adaptation (LocalAdaptation, look fixes T10; eye-adaptation.md "Local adaptation"): headless, exit code 1 on
# failure. The CPU mirror of LocalAdaptationEffect's shaders:
# - the factor: a region whose global display value is at most PLATEAU is untouched (c = 1; pad noon's sky); a brighter
#   one shows its local mean at PLATEAU in the Weber range (1e-9); the displayed local mean is monotonic in the local
#   luminance over 1e-3 .. 1e10 cd/m^2 and at the gain floor (the Sun's disc) is L x scene-per-cd/m^2 at gain 1 (display
#   white);
# - the block size: a power of two, at least BLOCKS_PER_SIGMA blocks per pooling sigma (desktop and VR);
# - an image: the Earth seen from the lunar night (a 2 deg disc of 1e4 cd/m^2 with +-40 % detail over ground at
#   0.05 cd/m^2, lunar-night gains): the disc's displayed mean within 25 % of PLATEAU, its detail ratios kept within 5 %,
#   the ground and the black sky around it untouched (c = 1 exactly: no dark halo), no bright rim (the limb's displayed
#   values at most 1.25 x the disc's mean), an emitter drawn after the opaque pass over the black sky keeps c = 1;
# - temporal stability (T10 fix round 1): a disc of 10 .. 32 px (1.25 .. 4 blocks, 0.3 .. 1 deg on the headset) moved over
#   the block grid in half-pixel steps: the displayed flux ratio and the peak pixel vary by at most TRANSLATION_BOUND (the
#   area-exact linear block means and the smooth like-block blend; the sub-block 6 px disc is the documented limit and is
#   only printed); a soft glare-like gradient has a monotonic factor (no ring) and an image whose size is not a multiple
#   of the block (disc at the right / bottom edge) stays finite and bounded;
# - one source of truth: the shaders' constants are generated from EyeAdaptation / LocalAdaptation (the placeholders are
#   all resolved, the generated values equal the constants); the CPU-vs-GPU readback check is
#   local_adaptation_gpu_check.gd;
# - the flag: --local_adaptation=off.
# godot --headless --path game --script res://tests/local_adaptation_test.gd
extends SceneTree

## Lunar night under the Earth (render_check, 2026-11-08): display gain 7557 x brightness 0.368, scene per cd/m^2 at gain 1
## DAY_EXPOSURE pi / 127 000 lux, the white plate under the TOA Sun L_ref = 127 000 / pi.
const TOA_LUX := 127000.0
const NIGHT_GAIN := 7557.0 * 0.368
const DISC_CD_M2 := 1.0e4
const GROUND_CD_M2 := 0.05
## Largest max / min of the displayed flux ratio and of the peak displayed pixel while a 10 .. 32 px disc moves (measured
## 1.86 / 1.84 at 10 px, 1.5 / 1.6 at 16 px, 1.2 / 1.2 at 24 px).
const TRANSLATION_BOUND := 1.05  # gate: displayed flux and peak vary by < 5 % (measured 1.01 - 1.03)

var _failures := 0


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("local_adaptation_test: %-56s %s %s" % [label, detail, "ok" if ok else "FAIL"])


func _initialize() -> void:
	var base := Atmosphere.DAY_EXPOSURE * PI / TOA_LUX
	var reference := TOA_LUX / PI
	# The factor.
	var noon_gain := 1.1
	var k_noon := LocalAdaptation.effective_per_local(base * noon_gain, noon_gain, reference)
	var sky := 3136.0  # pad noon's sky band (eye-adaptation.md)
	_check("pad noon sky untouched", LocalAdaptation.factor(sky, noon_gain, reference, k_noon) == 1.0,
			"global display %.3f" % (sky * base * noon_gain))
	var k_night := LocalAdaptation.effective_per_local(base * NIGHT_GAIN, NIGHT_GAIN, reference)
	var shown := DISC_CD_M2 * base * NIGHT_GAIN * LocalAdaptation.factor(DISC_CD_M2, NIGHT_GAIN, reference, k_night)
	_check("bright region at the plateau", absf(shown / LocalAdaptation.PLATEAU_SCENE - 1.0) < 1.0e-9,
			"%.6f (global %.0f)" % [shown, DISC_CD_M2 * base * NIGHT_GAIN])
	var previous := 0.0
	var monotonic := true
	var l := 1.0e-3
	while l <= 1.0e10:
		var d := l * base * NIGHT_GAIN * LocalAdaptation.factor(l, NIGHT_GAIN, reference, k_night)
		monotonic = monotonic and d >= previous * (1.0 - 1.0e-12)
		previous = d
		l *= 1.1
	_check("displayed local mean monotonic 1e-3 .. 1e10 cd/m^2", monotonic, "last %.1f" % previous)
	var sun := 1.6e9
	var sun_shown := sun * base * NIGHT_GAIN * LocalAdaptation.factor(sun, NIGHT_GAIN, reference, k_night)
	_check("Sun disc at the gain floor (display white)", absf(sun_shown / (sun * base) - 1.0) < 1.0e-9
			and sun_shown > 16.29, "%.0f" % sun_shown)
	_check("fixed exposure: no compression", LocalAdaptation.factor(DISC_CD_M2, 1.0, reference,
			LocalAdaptation.effective_per_local(base, 1.0, reference)) == 1.0, "")

	# The block size.
	for case in [[900.0, 1.0 / tan(deg_to_rad(50.0))], [1080.0, 1.0 / tan(deg_to_rad(25.0))], [4250.0, 0.85]]:
		var ppd := LocalAdaptation.pixels_per_degree(case[0], case[1])
		var block := LocalAdaptation.block_size(ppd)
		var sigma_blocks := LocalAdaptation.POOL_SIGMA_DEG * ppd / float(block)
		_check("block size %d px high" % int(case[0]), (block & (block - 1)) == 0 and block <= LocalAdaptation.MAX_BLOCK
				and (sigma_blocks >= LocalAdaptation.BLOCKS_PER_SIGMA or block == 1),
				"%.1f px/deg, block %d, sigma %.2f blocks" % [ppd, block, sigma_blocks])

	# The post step's rods: a pixel p = C rgb + s ROD_TINT (w_s . rgb) keeps p at c = 1 and goes to c C rgb (no rods)
	# under strong compression (lunar night: rod scale 0.22, chroma 0.77).
	var cone := EyeAdaptation.cone_map(0.7654, Basis.IDENTITY)
	var rods := 0.22
	var rgb := Vector3(0.8, 1.0, 1.3)
	var p := cone * rgb + EyeAdaptation.ROD_TINT * rods * EyeAdaptation.SCOTOPIC_WEIGHTS.dot(rgb)
	var part := LocalAdaptation.cone_part(rods, cone)
	_check("c = 1 leaves the pixel", (LocalAdaptation.apply_pixel(p, 1.0, part) - p).length() < 1.0e-12 * p.length(), "")
	var small := 1.0e-3
	var compressed := LocalAdaptation.apply_pixel(p, small, part)
	_check("strong compression: the cones' image x c, rods gone", (compressed - small * (cone * rgb)).length()
			< 2.0e-3 * small * (cone * rgb).length(), "rods were %.0f %% of Y" % (100.0 *
			LocalAdaptation.LUMINANCE_WEIGHTS.dot(p - cone * rgb) / LocalAdaptation.LUMINANCE_WEIGHTS.dot(p)))
	_image_case(base, reference, 2)
	_image_case(base, reference, LocalAdaptation.block_size(20.0))  # the block size the step picks at 20 px/deg (4)
	_translation_cases(base, reference)
	_gradient_case(base, reference)
	_odd_size_case(base, reference)
	_generated_constants_case()
	_check("flag from the arguments", not LocalAdaptation._enabled_from_arguments(
			PackedStringArray(["--local_adaptation=off"])) and LocalAdaptation._enabled_from_arguments(
			PackedStringArray(["--protect_adaptation"])), "")
	quit(1 if _failures > 0 else 0)


## The Earth from the lunar night on a 160 x 120 image at 20 px/deg (a 2 deg disc: radius 20 px), block 2 (sigma 4.3
## blocks; and the step's own block size): the sky above row 70 black, the ground below at GROUND_CD_M2, the disc centred at (80, 35).
func _image_case(base: float, reference: float, block: int) -> void:
	var w := 160
	var h := 120
	var ppd := 20.0
	var spc := base * NIGHT_GAIN
	var image := PackedFloat64Array()
	image.resize(w * h)
	var inside := []
	var ring := []
	var limb := []
	for y in range(h):
		for x in range(w):
			var r := Vector2(x + 0.5 - 80.0, y + 0.5 - 35.0).length()
			var lum := GROUND_CD_M2 if y >= 70 else 0.0
			if r <= 20.0:
				lum = DISC_CD_M2 * (1.0 + 0.4 * sin(0.9 * x) * cos(0.7 * y))  # detail on the disc
				if r < 12.0:
					inside.append(y * w + x)
				elif r > 18.0:
					limb.append(y * w + x)
			elif r < 26.0:
				ring.append(y * w + x)
			image[y * w + x] = lum * spc
	var parameters := {"scene_per_cd_m2": spc, "display_gain": NIGHT_GAIN, "reference_cd_m2": reference, "block": block,
			"sigma_blocks": LocalAdaptation.POOL_SIGMA_DEG * ppd / float(block)}
	var c := LocalAdaptation.factors(image, w, h, parameters)
	var log_sum := 0.0
	for i in inside:
		log_sum += log(image[i] * c[i])
	var mean_shown := exp(log_sum / inside.size())
	_check("block %d: disc's displayed mean near the plateau" % block, absf(mean_shown / LocalAdaptation.PLATEAU_SCENE - 1.0) < 0.25,
			"%.3f (global %.0f)" % [mean_shown, DISC_CD_M2 * spc])
	var worst := 0.0
	for k in range(0, inside.size() - 1, 7):
		var a: int = inside[k]
		var b: int = inside[k + 1]
		worst = maxf(worst, absf((image[a] * c[a]) / (image[b] * c[b]) / (image[a] / image[b]) - 1.0))
	_check("disc detail ratios kept", worst < 0.05, "worst %.4f" % worst)
	var ring_ok := true
	for i in ring:
		ring_ok = ring_ok and c[i] == 1.0
	_check("no halo: ground and sky beside the disc untouched", ring_ok, "%d pixels" % ring.size())
	var rim := 0.0
	for i in limb:
		rim = maxf(rim, image[i] * c[i])
	var inside_max := 0.0
	for i in inside:
		inside_max = maxf(inside_max, image[i] * c[i])
	_check("no bright rim at the limb", rim <= 1.25 * inside_max, "limb max %.3f, inside max %.3f" % [rim, inside_max])
	# An emitter (drawn after the opaque pass) on the black sky beside the disc.
	var final := image.duplicate()
	var emitter := 40 * w + 110  # 30 px right of the disc's centre, above the ground
	final[emitter] = 5.0
	var c_final := LocalAdaptation.factors(image, w, h, parameters, final)
	_check("emitter over the black sky keeps c = 1", c_final[emitter] == 1.0, "c %.4f" % c_final[emitter])


## A disc of diameter px (antialiased by 4 x 4 supersampling, 1e4 cd/m^2 over a 1e-3 cd/m^2 sky) centred at (cx, cy).
func _disc_image(w: int, h: int, diameter: float, cx: float, cy: float, spc: float) -> PackedFloat64Array:
	var image := PackedFloat64Array()
	image.resize(w * h)
	for y in range(h):
		for x in range(w):
			var cover := 0.0
			for sy in range(4):
				for sx in range(4):
					var px := x + (sx + 0.5) / 4.0 - cx
					var py := y + (sy + 0.5) / 4.0 - cy
					if px * px + py * py <= diameter * diameter / 4.0:
						cover += 1.0 / 16.0
			image[y * w + x] = (DISC_CD_M2 * cover + 1.0e-3 * (1.0 - cover)) * spc
	return image


## Headset numbers (31.5 px/deg, block 8): the disc moves 0 .. 8 px diagonally in half-pixel steps.
func _translation_cases(base: float, reference: float) -> void:
	var ppd := 31.5
	var block := LocalAdaptation.block_size(ppd)
	var spc := base * NIGHT_GAIN
	var parameters := {"scene_per_cd_m2": spc, "display_gain": NIGHT_GAIN, "reference_cd_m2": reference, "block": block,
			"sigma_blocks": LocalAdaptation.POOL_SIGMA_DEG * ppd / float(block)}
	for diameter in [6.0, 9.5, 16.0, 24.0, 32.0]:
		var flux_low := INF
		var flux_high := 0.0
		var peak_low := INF
		var peak_high := 0.0
		for step in range(34):  # 0 .. 8 px in half-pixel steps: diagonal (17), then along x only (17)
			var shift := 0.5 * (step % 17)
			var image := _disc_image(96, 96, diameter, 40.0 + shift, 40.0 + (0.6 * shift if step < 17 else 0.0), spc)
			var c := LocalAdaptation.factors(image, 96, 96, parameters)
			var flux_in := 0.0
			var flux_out := 0.0
			var peak := 0.0
			for i in range(image.size()):
				flux_in += image[i]
				flux_out += image[i] * c[i]
				peak = maxf(peak, image[i] * c[i])
			flux_low = minf(flux_low, flux_out / flux_in)
			flux_high = maxf(flux_high, flux_out / flux_in)
			peak_low = minf(peak_low, peak)
			peak_high = maxf(peak_high, peak)
		var detail := "flux ratio x%.2f, peak x%.2f (%.2f .. %.2f)" % [flux_high / flux_low, peak_high / peak_low, peak_low, peak_high]
		if diameter < 8.0:  # sub-block: the documented limit (open point), printed only
			print("local_adaptation_test: sub-block %.0f px disc, not gated: %s" % [diameter, detail])
		else:
			_check("disc %.1f px moving over the grid: bounded variation" % diameter,
					flux_high / flux_low <= TRANSLATION_BOUND and peak_high / peak_low <= TRANSLATION_BOUND, detail)


## A glare-like gradient (core 1e4 cd/m^2 falling as 1/r^2 to the sky's 1e-3 over 60 px): the factor must rise
## monotonically away from the core (no ring, no overshoot) and stay in (0, 1].
func _gradient_case(base: float, reference: float) -> void:
	var w := 160
	var h := 64
	var ppd := 31.5
	var block := LocalAdaptation.block_size(ppd)
	var spc := base * NIGHT_GAIN
	var image := PackedFloat64Array()
	image.resize(w * h)
	for y in range(h):
		for x in range(w):
			var r := maxf(Vector2(x + 0.5 - 30.0, y + 0.5 - 32.0).length(), 8.0)
			image[y * w + x] = maxf(DISC_CD_M2 * 64.0 / (r * r), 1.0e-3) * spc
	var parameters := {"scene_per_cd_m2": spc, "display_gain": NIGHT_GAIN, "reference_cd_m2": reference, "block": block,
			"sigma_blocks": LocalAdaptation.POOL_SIGMA_DEG * ppd / float(block)}
	var c := LocalAdaptation.factors(image, w, h, parameters)
	var worst_drop := 0.0
	var bounded := true
	var previous := c[32 * w + 38]
	for x in range(38, w):
		var value: float = c[32 * w + x]
		bounded = bounded and value > 0.0 and value <= 1.0
		worst_drop = maxf(worst_drop, previous - value)
		previous = maxf(previous, value)
	_check("soft gradient: factor rises outward, no ring", bounded and worst_drop <= 0.05 * c[32 * w + w - 1],
			"worst drop %.4f, c %.4f -> %.4f" % [worst_drop, c[32 * w + 38], c[32 * w + w - 1]])


## 150 x 93 px (not a multiple of the block) with a 20 px disc against the right and bottom edge: the edge clamps.
func _odd_size_case(base: float, reference: float) -> void:
	var w := 150
	var h := 93
	var ppd := 31.5
	var block := LocalAdaptation.block_size(ppd)
	var spc := base * NIGHT_GAIN
	var image := _disc_image(w, h, 20.0, 138.0, 83.0, spc)
	var parameters := {"scene_per_cd_m2": spc, "display_gain": NIGHT_GAIN, "reference_cd_m2": reference, "block": block,
			"sigma_blocks": LocalAdaptation.POOL_SIGMA_DEG * ppd / float(block)}
	var c := LocalAdaptation.factors(image, w, h, parameters)
	var ok := true
	for value in c:
		ok = ok and is_finite(value) and value > 0.0 and value <= 1.0
	var centre := image[83 * w + 138] * c[83 * w + 138]
	_check("odd image size, disc at the edge", ok and centre < 3.0 * LocalAdaptation.PLATEAU_SCENE and c[5 * w + 5] == 1.0,
			"centre displayed %.3f" % centre)


## The shaders are generated from the constants (no hand-copied numbers): no placeholder is left and a few generated
## literals equal the constants they come from.
func _generated_constants_case() -> void:
	var left := false
	var sources := [LocalAdaptationEffect.down_shader_source(), LocalAdaptationEffect.blur_shader_source(),
			LocalAdaptationEffect.apply_shader_source()]
	for source: String in sources:
		left = left or source.contains("@")
	var pool: String = sources[2]
	_check("shaders generated, no placeholder left", not left, "")
	_check("apply shader carries the constants", pool.contains(LocalAdaptationEffect.glsl_float(EyeAdaptation.CONE_MID_SLOPE))
			and pool.contains(LocalAdaptationEffect.glsl_float(EyeAdaptation.CONE_WEBER_OFFSET))
			and pool.contains(LocalAdaptationEffect.glsl_float(EyeAdaptation.CONE_LOG_LOW)), "")
	_check("glsl_float round trip", is_equal_approx(LocalAdaptationEffect.glsl_float(0.249).to_float(), 0.249)
			and LocalAdaptationEffect.glsl_float(3.0).contains("."), LocalAdaptationEffect.glsl_float(3.0))
