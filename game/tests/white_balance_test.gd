# The eye's chromatic adaptation to the Sun's white (EyeAdaptation.white_balance, look fixes T2; eye-adaptation.md):
# - the display's own white (RGB grey, D65) is left alone (identity within 1e-6);
# - a white Lambertian surface under the TOA Sun (the colour of the core's solar spectrum, as the game takes it from the
#   atmosphere tables) is drawn grey at its luminance (channels within 1e-6 relative): a sunlit cloud seen from space is
#   white [S: crew photographs and DSCOVR EPIC natural colour show sunlit cloud tops neutral];
# - the TOA Sun itself is warmer than D65 (B/R 0.80-0.90 [D: the core's spectrum in linear sRGB 0.856]) - the cause of
#   the salmon clouds before (start view: cloud radiance B/R 0.79-0.84 measured, look fixes T2 report);
# - light reddened by the air stays warm: the Sun through Rayleigh air mass 4 at 700 hPa (B/R 0.51 colourimetric) is
#   still clearly warm after the balance (B/R < 0.75) - the terminator and the sunset keep their colour;
# - the post step's map and the display-referred inverse (vision.gdshaderinc, Sherman-Morrison) cancel with rods,
#   chroma < 1 and the white balance together (within 1e-6).
# Headless: godot --headless --path game --script res://tests/white_balance_test.gd
extends SceneTree

const TOLERANCE := 1.0e-6

var _failures := 0


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	_check_identity()
	var core := StarsimCore.new()
	if not core.open(Paths.data_dir().path_join("spice")):
		push_error("SPICE kernels missing")
		quit(1)
		return
	var atmosphere := Atmosphere.new()
	atmosphere.start_loading(core, Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	if not atmosphere.wait_until_loaded():
		push_error("atmosphere tables not loaded: " + atmosphere.status_text())
		quit(1)
		return
	var sun := Vector3(atmosphere.parameters["solar_illuminance_rgb"]) \
			* Vector3(atmosphere.parameters["sun_radiance_to_luminance"])
	_check_sun(sun)
	_check_reddened(sun)
	_check_inverse(EyeAdaptation.white_balance(sun))
	atmosphere.shutdown()
	print("white_balance_test: %s (%d failures)" % ["PASS" if _failures == 0 else "FAIL", _failures])
	quit(1 if _failures > 0 else 0)


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("white_balance_test: %-44s %s %s" % [label, detail, "ok" if ok else "FAIL"])


func _check_identity() -> void:
	var w := EyeAdaptation.white_balance(Vector3(2.0, 2.0, 2.0))
	var worst := 0.0
	for k in range(3):
		var d: Vector3 = (w[k] - Basis.IDENTITY[k]).abs()
		worst = maxf(worst, maxf(d.x, maxf(d.y, d.z)))
	_check("D65 grey: identity", worst < TOLERANCE, "largest element error %s" % worst)


func _check_sun(sun: Vector3) -> void:
	var ratio := sun.z / sun.x
	_check("TOA Sun warmer than D65", ratio > 0.80 and ratio < 0.90, "linear sRGB %s, B/R %.4f" % [sun / sun.y, ratio])
	var plate := EyeAdaptation.white_balance(sun) * sun
	var y := EyeAdaptation.LUMINANCE_WEIGHTS.dot(sun)
	var worst := 0.0
	for k in range(3):
		worst = maxf(worst, absf(plate[k] / y - 1.0))
	_check("sunlit white -> grey at its luminance", worst < TOLERANCE,
			"balanced %s, luminance %.6f, largest channel error %s" % [plate / y, y, worst])


func _check_reddened(sun: Vector3) -> void:
	# The Sun's colour through Rayleigh air mass 4 at 700 hPa (optical depth 0.0088 lambda_um^-4.05 at sea level
	# [S: Bodhaine et al. 1999 fit]) at the tables' channel wavelengths (680, 550, 440 nm).
	var lambdas := Vector3(0.680, 0.550, 0.440)
	var t := Vector3.ZERO
	for k in range(3):
		t[k] = exp(-4.0 * 0.7 * 0.0088 * pow(lambdas[k], -4.05))
	var reddened := sun * t
	var balanced := EyeAdaptation.white_balance(sun) * reddened
	var ratio := balanced.z / balanced.x
	_check("air-reddened Sun stays warm", ratio < 0.75 and ratio > 0.3,
			"B/R %.3f before, %.3f balanced" % [reddened.z / reddened.x, ratio])


func _check_inverse(white: Basis) -> void:
	var scale := 0.3
	var chroma_factor := 0.7
	var cone := EyeAdaptation.cone_map(chroma_factor, white)
	var cone_inverse := cone.inverse()
	var tint_q := cone_inverse * EyeAdaptation.ROD_TINT
	var u := tint_q * (scale / (1.0 + scale * EyeAdaptation.SCOTOPIC_WEIGHTS.dot(tint_q)))
	var worst := 0.0
	for c in [Vector3(1, 0, 0), Vector3(0.2, 0.5, 0.9), Vector3(3.0, 1.0, 0.1)]:
		# the emitter's inverse (vision.gdshaderinc), then the post step (VisionEffect)
		var q: Vector3 = cone_inverse * c
		var written := q - u * EyeAdaptation.SCOTOPIC_WEIGHTS.dot(q)
		var shown := EyeAdaptation.post_step(written, scale, chroma_factor, white)
		worst = maxf(worst, (shown - c).length() / c.length())
	_check("post step o display-referred inverse = id", worst < TOLERANCE, "largest relative error %s" % worst)
