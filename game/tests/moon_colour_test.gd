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
	_check_field_tint(w, grey)
	# The flag off: the former look.
	Atmosphere.moon_colour_correction = false
	_check_off()
	Atmosphere.moon_colour_correction = true
	print("moon_colour_test: %s" % ("PASS" if _failures == 0 else "%d FAILED" % _failures))
	quit(1 if _failures > 0 else 0)


## The adapting field near the Moon (Atmosphere.moon_field_tint, bug hunt 2026-11-08): the regolith's colour by
## MOON_FIELD_MEAN_WEIGHT x the share 1 - cos(theta) of the field the Moon fills, luminance 1 throughout.
func _check_field_tint(w: Vector3, grey: Vector3) -> void:
	var r := Atmosphere.MOON_RADIUS_M
	var k := Atmosphere.MOON_FIELD_MEAN_WEIGHT
	var ground := Atmosphere.moon_field_tint(r, r)  # an eye on the ground: share 1
	_near("field on the ground B/R", ground.z / ground.x, (1.0 + k * (grey.z - 1.0)) / (1.0 + k * (grey.x - 1.0)), 1e-9)
	_near("field on the ground B/R value", ground.z / ground.x, 0.786, 0.002)
	_near("field on the ground luminance", w.dot(ground), 1.0, 1e-9)
	var share := 1.0 - sqrt(1.0 - pow(r / (r + 300000.0), 2.0))  # 300 km: theta 58.5 deg
	_near("field share 300 km", share, 0.477, 0.001)
	var orbit := Atmosphere.moon_field_tint(r + 300000.0, r)
	_near("field 300 km B/R", orbit.z / orbit.x,
			(1.0 + k * share * (grey.z - 1.0)) / (1.0 + k * share * (grey.x - 1.0)), 1e-9)
	_near("field 300 km luminance", w.dot(orbit), 1.0, 1e-9)
	var earth := Atmosphere.moon_field_tint(384400000.0, r)  # from the Earth: share 1e-5, the full Moon unchanged
	_near("field from the Earth B/R", earth.z / earth.x, 1.0, 1e-4)
	Atmosphere.moon_field_adaptation = false
	if Atmosphere.moon_field_tint(r, r) != Vector3.ONE:
		print("moon_colour_test: FAIL moon_field_adaptation off is not neutral")
		_failures += 1
	Atmosphere.moon_field_adaptation = true


func _check_off() -> void:
	if Atmosphere.moon_field_tint(Atmosphere.MOON_RADIUS_M, Atmosphere.MOON_RADIUS_M) != Vector3.ONE:
		print("moon_colour_test: FAIL field tint with the colour factor off is not neutral")
		_failures += 1
	if Atmosphere.moon_albedo_tint() != Vector3.ONE or Atmosphere.moon_grey_tint() != Vector3.ONE:
		print("moon_colour_test: FAIL flag off is not neutral")
		_failures += 1
