# The physical glare spread (GlareSpread, look fixes T13), CPU mirror; headless, exit code 1 on failure.
# - the kernel (sum of Gaussians, tools/glare_kernel_fit.py) against the CIE 146:2002 function: <= 3 % from 0.5 to 30 deg,
#   <= 20 % at 0.2 deg (the 1 / theta^3 core below 0.3 deg is beyond a Gaussian sum); the energy over 0.1 .. 60 deg within 5 %;
# - the source threshold: nothing at or below the display's white (a display-white UI pixel spreads nothing), excess above;
# - the pyramid against the direct sum over the source pixels (disc of 0.5 deg): <= 8 % at 1.5 .. 5 deg, 10 % at 10 deg
#   (the 10 deg point of this 16 deg wide test frame lies 6 deg from its edge);
# - shift invariance: a 0.5 deg disc moved in sub-pixel steps: the pyramid/direct ratio at 2 and 5 deg varies < 3 %;
# - energy: a point source's level-0 energy is the source's;
# - the shaders' placeholders; the veil on a source goes with the local adaptation's compression (self_compression);
# - the Sun's analytic veil: the form of Atmosphere.glare_veil (Stiles-Holladay) at the view axis equals the scalar; the
#   CIE 146 form's ratio to it is reported; behind the eye nothing; held below the Sun's own radius.
# godot --headless --path game --script res://tests/glare_spread_test.gd
extends SceneTree

var PPD := 20.0  ## px/deg of the case (_pyramid loops over 6.6 (desktop), 20, 47 (base factor 3))
const WIDTH := 320
const HEIGHT := 120

var _failures := 0


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("glare_spread_test: %-56s %s %s" % [label, detail, "ok" if ok else "FAIL"])


func _initialize() -> void:
	_kernel()
	_threshold()
	_pyramid()
	_sun()
	_shaders()
	print("glare_spread_test: %s" % ("FAIL" if _failures > 0 else "PASS"))
	quit(1 if _failures > 0 else 0)


func _kernel() -> void:
	for theta: float in [0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 30.0]:
		var error := GlareSpread.fitted(theta) / GlareSpread.cie146(theta) - 1.0
		var tolerance := 0.20 if theta < 0.3 else 0.03
		_check("kernel at %.1f deg" % theta, absf(error) <= tolerance,
				"CIE %s, fit %s, %+.1f %% (tolerance %.0f %%)" % [GlareSpread.cie146(theta), GlareSpread.fitted(theta),
				100.0 * error, 100.0 * tolerance])
	_check("fit report max error stored", GlareSpreadKernel.FIT_MAX_ERROR < 0.5, "%.3f at 0.1 deg" % GlareSpreadKernel.FIT_MAX_ERROR)
	var cie := GlareSpread.cie146_energy(0.1, 60.0)
	var fit := GlareSpread.fitted_energy(0.1, 60.0)
	_check("energy 0.1 .. 60 deg", absf(fit / cie - 1.0) < 0.05, "CIE %.4f, fit %.4f (%+.1f %%)" % [cie, fit, 100.0 * (fit / cie - 1.0)])
	var a := 0.0
	for w: float in GlareSpreadKernel.WEIGHTS:
		a += w
	_check("weights non-negative", GlareSpreadKernel.WEIGHTS.all(func(w: float) -> bool: return w >= 0.0), "sum %.4f" % a)
	var shared := GlareSpread.cie146_energy(0.5, 10.0, GlareSpread.Form.STILES_HOLLADAY)
	print("glare_spread_test: info - Stiles-Holladay energy 0.5 .. 10 deg %.4f, CIE 146 %.4f" % [shared, GlareSpread.cie146_energy(0.5, 10.0)])


func _threshold() -> void:
	_check("display-white UI pixel spreads nothing", GlareSpread.excess_luminance(1.0) == 0.0
			and GlareSpread.excess_luminance(GlareSpread.SOURCE_WHITE) == 0.0, "")
	_check("excess above the white", is_equal_approx(GlareSpread.excess_luminance(151.5), 150.0), "")
	var image := PackedFloat32Array()
	image.resize(WIDTH * HEIGHT)
	for i in range(image.size()):
		image[i] = GlareSpread.excess_luminance(1.2)  # a white page
	var result := GlareSpread.spread(image, PackedFloat32Array(), WIDTH, HEIGHT, PPD)
	var peak := 0.0
	for v: float in result["a0"]:
		peak = maxf(peak, v)
	_check("image of display-white pixels: veil 0", peak == 0.0, "max %s" % peak)


## A disc of the given diameter (deg), antialiased by 4 x 4 supersampling; excess in scene units.
func _disc(cx: float, cy: float, diameter_deg: float, value: float) -> PackedFloat32Array:
	var image := PackedFloat32Array()
	image.resize(WIDTH * HEIGHT)
	var radius := 0.5 * diameter_deg * PPD
	for y in range(maxi(0, int(cy - radius) - 2), mini(HEIGHT, int(cy + radius) + 3)):
		for x in range(maxi(0, int(cx - radius) - 2), mini(WIDTH, int(cx + radius) + 3)):
			var inside := 0
			for sy in range(4):
				for sx in range(4):
					var dx := x + (sx + 0.5) / 4.0 - cx
					var dy := y + (sy + 0.5) / 4.0 - cy
					if dx * dx + dy * dy <= radius * radius:
						inside += 1
			image[y * WIDTH + x] = value * inside / 16.0
	return image


## The direct sum of the fitted kernel over the source pixels at pixel (px, py): sum excess dOmega fitted(theta).
func _direct(image: PackedFloat32Array, px: int, py: int) -> float:
	var d_omega := pow(deg_to_rad(1.0 / PPD), 2.0)
	var sum := 0.0
	for y in range(HEIGHT):
		for x in range(mini(WIDTH, 120)):  # the disc lies within 120 px of the left edge
			var e := image[y * WIDTH + x]
			if e == 0.0:
				continue
			var theta := sqrt(pow(x - px, 2.0) + pow(y - py, 2.0)) / PPD
			sum += e * d_omega * GlareSpread.fitted(maxf(theta, 0.05))
	return sum


func _pyramid() -> void:
	for ppd in [6.6, 20.0, 47.0]:
		PPD = ppd
		_pyramid_case()


func _pyramid_case() -> void:
	var cx := 40.0
	var cy := 60.0
	var image := _disc(cx, cy, 0.5, 1000.0)
	var result := GlareSpread.spread(image, PackedFloat32Array(), WIDTH, HEIGHT, PPD)
	# Energy: sum of the level-0 texels x texel solid angle = the source energy (up to the Gaussian reaching past the edge).
	var source := 0.0
	for v: float in image:
		source += v
	var f0 := float(result["factor"])
	var level0 := GlareSpread.source_level(image, PackedFloat32Array(), WIDTH, HEIGHT, int(f0), result["size"])
	var level_sum := 0.0
	for v: float in level0:
		level_sum += v
	_check("level-0 energy = source energy, %.1f px/deg" % PPD, absf(level_sum * f0 * f0 / source - 1.0) < 1.0e-5, "%.6f" % (level_sum * f0 * f0 / source))
	var angles := [1.5, 2.0, 3.0, 5.0, 10.0] if PPD < 40.0 else [0.5, 1.0, 1.5]
	for theta: float in angles:
		var px := int(round(cx + theta * PPD))
		var pyramid := GlareSpread.veil_at(result, px, int(cy))
		var direct := _direct(image, px, int(cy))
		_check("pyramid vs direct sum at %.1f deg, %.1f px/deg" % [theta, PPD], absf(pyramid / direct - 1.0) < (0.10 if theta > 8.0 else 0.08),
				"%s / %s (%+.1f %%)" % [pyramid, direct, 100.0 * (pyramid / direct - 1.0)])
	# Shift invariance: the ratio pyramid / direct at 2 and 5 deg over sub-pixel phases of the disc.
	var shifts := [2.0, 5.0] if PPD < 40.0 else [0.5, 1.0]
	var lows := {}
	var highs := {}
	var sums := {}
	for t: float in shifts:
		lows[t] = INF
		highs[t] = -INF
		sums[t] = 0.0
	for phase in range(4):
		var s := phase / 4.0
		var moved := _disc(cx + s, cy + 0.6 * s, 0.5, 1000.0)
		var moved_result := GlareSpread.spread(moved, PackedFloat32Array(), WIDTH, HEIGHT, PPD)
		for theta: float in shifts:
			var px := int(round(cx + theta * PPD))
			var ratio := GlareSpread.veil_at(moved_result, px, int(cy)) / _direct(moved, px, int(cy))
			lows[theta] = minf(lows[theta], ratio)
			highs[theta] = maxf(highs[theta], ratio)
			sums[theta] += ratio
	for theta: float in shifts:
		var variation: float = (highs[theta] - lows[theta]) / (sums[theta] / 4.0)
		_check("moving 0.5 deg disc: veil at %.1f deg, %.1f px/deg" % [theta, PPD], variation < 0.03,
				"pyramid / direct %.4f .. %.4f, variation %.2f %%" % [lows[theta], highs[theta], 100.0 * variation])


func _shaders() -> void:
	var left := 0
	for source: String in GlareSpreadEffect.shader_sources():
		left += source.count("@")
	_check("no placeholder left in the shaders", left == 0, "%d" % left)
	_check("self compression: no source = 1", GlareSpread.self_compression(0.0) == 1.0, "")
	# T13: the veil factor across a limb - sky (shown 0) keeps the veil, the disc (shown at the plateau) is compressed,
	# a half-covered limb pixel lies between (no dip below either side)
	var sky_f := GlareSpread.veil_factor(1000.0, 0.0)
	var limb_f := GlareSpread.veil_factor(1000.0, 0.5 * GlareSpread.COVERAGE_SHARE * GlareSpread.PLATEAU_SCENE)
	var disc_f := GlareSpread.veil_factor(1000.0, GlareSpread.PLATEAU_SCENE)
	_check("veil factor: sky 1, disc compressed, limb between", sky_f == 1.0 and absf(disc_f
			- GlareSpread.self_compression(1000.0)) < 1e-12 and limb_f < sky_f and limb_f > disc_f,
			"%.4f / %.4f / %.4f" % [sky_f, limb_f, disc_f])
	_check("self compression: a source of 1000 goes to the plateau",
			absf(GlareSpread.self_compression(1000.0) * (GlareSpread.SOURCE_WHITE + 1000.0) - GlareSpread.PLATEAU_SCENE) < 1.0e-9,
			"%.5f" % GlareSpread.self_compression(1000.0))
	_check("self compression: a weak source (excess 0.01) is not compressed above 1",
			GlareSpread.self_compression(0.01) <= 1.0, "%.4f" % GlareSpread.self_compression(0.01))


func _sun() -> void:
	var atmosphere := Atmosphere.new()
	atmosphere.last_sun_normal_lux = 1.27e5
	atmosphere.adaptation_surround.last_visibility = 1.0
	for theta: float in [3.0, 10.0, 25.0]:
		var projection := Projection.create_perspective(60.0, 16.0 / 9.0, 0.05, 1000.0)
		var t := deg_to_rad(theta)
		var view := {"head": Basis.IDENTITY, "eyes": [[Basis.IDENTITY, projection]], "sun": Vector3(sin(t), 0.0, -cos(t))}
		var scalar := atmosphere.glare_veil(view, [0, 0]).x
		var mine := GlareSpread.sun_veil_cd_m2(1.27e5, theta, atmosphere.last_veil_visibility, GlareSpread.Form.STILES_HOLLADAY)
		_check("Sun veil (Stiles-Holladay) = glare_veil at %.0f deg" % theta, absf(mine / scalar - 1.0) < 1.0e-9,
				"%.1f / %.1f cd/m^2, visibility %.2f" % [mine, scalar, atmosphere.last_veil_visibility])
		var cie := GlareSpread.sun_veil_cd_m2(1.27e5, theta, atmosphere.last_veil_visibility)
		print("glare_spread_test: info - %.0f deg: CIE 146 veil %.1f cd/m^2 = %.2f x the scalar's Stiles-Holladay" % [theta, cie, cie / scalar])
	_check("Sun veil behind the eye = 0", GlareSpread.sun_veil_cd_m2(1.27e5, 95.0, 1.0) == 0.0, "")
	_check("Sun veil held inside the disc's radius", is_equal_approx(GlareSpread.sun_veil_cd_m2(1.27e5, 0.0, 1.0),
			GlareSpread.sun_veil_cd_m2(1.27e5, Atmosphere.GLARE_MIN_ANGLE_DEG, 1.0)
			/ cos(deg_to_rad(Atmosphere.GLARE_MIN_ANGLE_DEG)) * 1.0), "")
