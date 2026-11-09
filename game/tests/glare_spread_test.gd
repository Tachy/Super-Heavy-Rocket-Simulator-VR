# The physical glare spread (GlareSpread, look fixes T13), CPU mirror; headless, exit code 1 on failure.
# - the kernel (sum of Gaussians, tools/glare_kernel_fit.py) against the CIE 146:2002 function: <= 3 % from 0.5 to 30 deg,
#   <= 20 % at 0.2 deg (the 1 / theta^3 core below 0.3 deg is beyond a Gaussian sum); the energy over 0.1 .. 60 deg within 5 %;
# - the source threshold: nothing at or below the display's white (a display-white UI pixel spreads nothing), excess above;
# - the pyramid against the direct sum over the source pixels (disc of 0.5 deg): <= 8 % at 1.5 .. 5 deg, 10 % at 10 deg
#   (the 10 deg point of this 16 deg wide test frame lies 6 deg from its edge);
# - shift invariance: a 0.5 deg disc moved in sub-pixel steps: the pyramid/direct ratio at 2 and 5 deg varies < 3 %;
# - energy: a point source's level-0 energy is the source's;
# - the shaders' placeholders; each source's veil at its local adaptation factor (source_factor);
# - the Sun's analytic veil: the form of Atmosphere.glare_veil (Stiles-Holladay) at the view axis equals the scalar; the
#   CIE 146 form's ratio to it is reported; behind the eye nothing; held below the Sun's own radius;
# - the view frame from the render projection (final review I1): an eye pair canted +-10 deg (and parallel, and the
#   desktop view) with Vulkan's y flip and the eye offset in it: the Sun's centre from the ray parameters and the Sun in
#   the eye's axes lands within 1 px / 0.05 deg of its true projection in both eyes (the old path, raw ray parameters
#   plus the eye's rotation applied a second time, is reported and must miss by > 1 px when canted).
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
	_canted()
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
	# T13 fix 1: each source's veil is shown at its source's local factor (GlareSpread.source_factor)
	var plateau := GlareSpread.PLATEAU_SCENE
	_check("source factor: a source of 1000 goes to the plateau",
			absf(GlareSpread.source_factor(1000.0, 1.0e-6) * 1000.0 - plateau) < 1.0e-9,
			"%.6f" % GlareSpread.source_factor(1000.0, 1.0e-6))
	_check("source factor: floored at 1 / g (the Sun stays display white)",
			GlareSpread.source_factor(1.0e9, 0.01) == 0.01, "%.4f" % GlareSpread.source_factor(1.0e9, 0.01))
	_check("source factor: never above 1 (a source below the plateau, no source)",
			GlareSpread.source_factor(0.5 * plateau, 1.0e-3) == 1.0 and GlareSpread.source_factor(0.0, 1.0e-3) == 1.0, "")
	_check("source factor: 1 without the local adaptation (inverse gain 1)",
			GlareSpread.source_factor(1.0e4, 1.0) == 1.0, "")
	# The full Moon (0.259 deg radius): its halo keeps the physical ratio to its disc, veil / disc = f(theta) x Omega
	# (CIE 146, young eye): ~1e-3 at 1 deg, ~1.6e-4 at 2 deg, ~2e-5 at 5 deg - a slight glow, no ball.
	var omega := PI * pow(deg_to_rad(0.259), 2.0)
	var ratios := []
	for theta: float in [1.0, 2.0, 5.0]:
		ratios.append(GlareSpread.cie146(theta) * omega)
	_check("full Moon: shown halo / disc at 1 / 2 / 5 deg",
			absf(ratios[0] / 9.7e-4 - 1.0) < 0.05 and absf(ratios[1] / 1.65e-4 - 1.0) < 0.05
			and absf(ratios[2] / 1.9e-5 - 1.0) < 0.05 and ratios[0] > ratios[1] and ratios[1] > ratios[2],
			"%.7f / %.7f / %.7f" % ratios)


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
	# The Sun's veil is shown at its disc's local factor PLATEAU / Y_disc (no floor): its halo keeps CIE 146 x Omega_Sun of the
	# disc's plateau value (1e-3 at 1 deg), like the Moon's and the Earth's.
	var disc := atmosphere.sun_disc_scene(0.0035)
	atmosphere.adaptation_surround.last_visibility = 1.0
	var factor := GlareSpread.source_factor(disc, 0.0)
	_check("Sun disc scene luminance ~ 1.9e9 cd/m^2 x scene per cd", absf(disc / (1.9e9 * 0.0035) - 1.0) < 0.1, "%.3g" % disc)
	_check("Sun factor = PLATEAU / Y_disc, far below the cone floor", absf(factor * disc / GlareSpread.PLATEAU_SCENE - 1.0) < 1.0e-9
			and factor < 1.0e-5, "%.3g" % factor)
	var halo_1deg := GlareSpread.sun_veil_cd_m2(1.27e5, 1.0, 1.0) * 0.0035 * factor / GlareSpread.PLATEAU_SCENE
	_check("Sun halo at 1 deg / disc = CIE 146 x Omega_Sun (1.0e-3, x0.5 .. 2)", halo_1deg > 0.5e-3 and halo_1deg < 2.0e-3,
			"%.3g" % halo_1deg)
	_check("Sun veil behind the eye = 0", GlareSpread.sun_veil_cd_m2(1.27e5, 95.0, 1.0) == 0.0, "")
	_check("Sun veil held inside the disc's radius", is_equal_approx(GlareSpread.sun_veil_cd_m2(1.27e5, 0.0, 1.0),
			GlareSpread.sun_veil_cd_m2(1.27e5, Atmosphere.GLARE_MIN_ANGLE_DEG, 1.0)
			/ cos(deg_to_rad(Atmosphere.GLARE_MIN_ANGLE_DEG)) * 1.0), "")


## The NDC (x, y up) a direction in the eye's axes falls on under ray parameters (a, b, c, d): ray (x a + b, y c + d, -1).
static func _ndc_of(ray: PackedFloat32Array, eye_dir: Vector3) -> Vector2:
	return Vector2((eye_dir.x / -eye_dir.z - ray[1]) / ray[0], (eye_dir.y / -eye_dir.z - ray[3]) / ray[2])


func _canted() -> void:
	const SIZE := Vector2i(2560, 2560)
	var correction := Projection.create_depth_correction(true)
	var head := Basis.from_euler(Vector3(0.2, 0.7, 0.1))
	var sun_head := Vector3(0.15, 0.1, -1.0).normalized()
	var sun_world := head * sun_head
	for cant_deg: float in [10.0, 0.0, -1.0]:  # -1: the desktop view (one view, no eye offset)
		var views := 1 if cant_deg < 0.0 else 2
		for view in range(views):
			var side := -1.0 if view == 0 else 1.0
			var eye := Transform3D.IDENTITY
			var p_eye := Projection.create_perspective(60.0, 16.0 / 9.0, 0.05, 1000.0)
			if views == 2:
				# The outer half wider (a Pimax-like asymmetric frustum), the eye yawed outward by the cant.
				var outer := 1.4 * 0.05
				var inner := 0.9 * 0.05
				p_eye = Projection.create_frustum(-outer if side < 0.0 else -inner, inner if side < 0.0 else outer,
						-1.1 * 0.05, 1.0 * 0.05, 0.05, 1000.0)
				eye = Transform3D(Basis(Vector3.UP, -side * deg_to_rad(cant_deg)), Vector3(side * 0.032, 0.0, 0.0))
			var render := correction * p_eye * Projection(eye.affine_inverse())  # RenderSceneDataRD.get_view_projection
			var clip := render * Vector4(sun_head.x, sun_head.y, sun_head.z, 0.0)
			var truth := Vector2(clip.x / clip.w, -clip.y / clip.w)  # NDC y up (the correction flips y)
			var frame := GlareSpreadEffect.view_frame(render, head, sun_world, SIZE)
			var got := _ndc_of(frame[0], frame[2])
			var px := ((got - truth) * Vector2(SIZE) * 0.5).length()
			var eye_axes: Basis = GlareSpreadEffect.eye_frame(render)["eye_in_head"]
			var ray: PackedFloat32Array = frame[0]
			var ray_world := (head * eye_axes * Vector3(truth.x * ray[0] + ray[1], truth.y * ray[2] + ray[3], -1.0)).normalized()
			var angle_deg := rad_to_deg(ray_world.angle_to(sun_world))
			var label := "desktop" if views == 1 else "cant %s%.0f deg eye %d" % ["+" if side < 0.0 else "-", cant_deg, view]
			_check("view frame: Sun centre, %s" % label, px < 1.0 and angle_deg < 0.05,
					"%.3f px, %.4f deg" % [px, angle_deg])
			if views == 2 and cant_deg > 0.0:
				# The old path: the raw ray parameters (rotation still inside) and the Sun rotated into the eye's axes.
				var raw := PackedFloat32Array([1.0 / render.x.x, render.z.x / render.x.x, 1.0 / absf(render.y.y),
						render.z.y / render.y.y])
				var old := _ndc_of(raw, (head * eye.basis).transposed() * sun_world)
				var old_px := ((old - truth) * Vector2(SIZE) * 0.5).length()
				_check("view frame: old double rotation misses, %s" % label, old_px > 1.0, "%.1f px" % old_px)
