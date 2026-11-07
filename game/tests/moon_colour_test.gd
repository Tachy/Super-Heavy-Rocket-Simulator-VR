# The regolith colour factor (look fixes T5b, Atmosphere.moon_albedo_tint / moon_grey_tint): headless, exit code 1 on failure.
# godot --headless --path game --script res://tests/moon_colour_test.gd
extends SceneTree

var _failures := 0


func _near(name_: String, actual: float, expected: float, tolerance: float) -> void:
	if not (absf(actual - expected) <= tolerance):
		print("moon_colour_test: FAIL %s: %s, expected %s +- %s" % [name_, actual, expected, tolerance])
		_failures += 1


func _initialize() -> void:
	var w := EyeAdaptation.LUMINANCE_WEIGHTS
	var mean := Atmosphere.MOON_TEXTURE_MEAN_RGB
	var tint := Atmosphere.moon_albedo_tint()
	var corrected := mean * tint
	# The texture's mean colour becomes the measured reflectance colour (B/R 0.66, G/R 0.85) ...
	_near("G/R", corrected.y / corrected.x, 0.85, 1e-9)
	_near("B/R", corrected.z / corrected.x, 0.66, 1e-9)
	# ... at the unchanged mean luminance (the calibrated albedo, regolith_scale and disc calibration stay).
	_near("mean luminance kept", w.dot(corrected), w.dot(mean), 1e-9)
	_near("mean luminance is the calibrated albedo", w.dot(mean), Atmosphere.MOON_MEAN_TEXTURE_ALBEDO, 0.002)
	_near("texture B/R (data)", mean.z / mean.x, 0.894, 0.002)
	# The grey users: luminance 1, same ratios.
	var grey := Atmosphere.moon_grey_tint()
	_near("grey luminance 1", w.dot(grey), 1.0, 1e-9)
	_near("grey B/R", grey.z / grey.x, 0.66, 1e-9)
	_near("grey G/R", grey.y / grey.x, 0.85, 1e-9)
	# A grey texel (a mosaic without colour) takes the grey tint's colour exactly in ratio.
	# The flag off: the former look.
	Atmosphere.moon_colour_correction = false
	_check_off()
	Atmosphere.moon_colour_correction = true
	print("moon_colour_test: %s" % ("PASS" if _failures == 0 else "%d FAILED" % _failures))
	quit(1 if _failures > 0 else 0)


func _check_off() -> void:
	if Atmosphere.moon_albedo_tint() != Vector3.ONE or Atmosphere.moon_grey_tint() != Vector3.ONE:
		print("moon_colour_test: FAIL flag off is not neutral")
		_failures += 1
