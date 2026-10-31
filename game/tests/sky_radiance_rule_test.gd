# The radiance refresh rule (SkyRadianceRefresh): headless, exit code 1 on failure.
# godot --headless --path game --script res://tests/sky_radiance_rule_test.gd
extends SceneTree

var _failures := 0


func _check(name_: String, actual: bool, expected: bool) -> void:
	if actual != expected:
		print("sky_radiance_rule_test: FAIL %s: %s" % [name_, actual])
		_failures += 1


func _initialize() -> void:
	var up := Vector3.UP
	var sun := Vector3.RIGHT
	var rule := SkyRadianceRefresh.new()
	# Arguments: tables, Earth zenith, height, Moon zenith, Moon height, Sun, scale, Moon light, Sun fraction.
	_check("first state", rule.due(false, up, 10.0, up, -1.0, sun, 1.0, 0.0, 1.0), true)
	_check("nothing changed", rule.due(false, up, 10.0, up, -1.0, sun, 1.0, 0.0, 1.0), false)
	_check("tables loaded", rule.due(true, up, 10.0, up, -1.0, sun, 1.0, 0.0, 1.0), true)
	_check("no repeat", rule.due(true, up, 10.0, up, -1.0, sun, 1.0, 0.0, 1.0), false)

	var small := deg_to_rad(0.05)
	var large := deg_to_rad(0.15)
	_check("Sun 0.05 deg", rule.due(true, up, 10.0, up, -1.0, sun.rotated(Vector3.UP, small), 1.0, 0.0, 1.0), false)
	_check("Sun 0.15 deg", rule.due(true, up, 10.0, up, -1.0, sun.rotated(Vector3.UP, large), 1.0, 0.0, 1.0), true)
	sun = sun.rotated(Vector3.UP, large)
	var turned := up.rotated(Vector3.RIGHT, large)
	_check("zenith 0.15 deg", rule.due(true, turned, 10.0, up, -1.0, sun, 1.0, 0.0, 1.0), true)
	_check("opposite zenith (a site jump)", rule.due(true, -turned, 10.0, up, -1.0, sun, 1.0, 0.0, 1.0), true)
	up = -turned
	_check("100 m up", rule.due(true, up, 110.0, up, -1.0, sun, 1.0, 0.0, 1.0), false)
	_check("300 m up", rule.due(true, up, 310.0, up, -1.0, sun, 1.0, 0.0, 1.0), true)
	_check("1 % of 400 km", rule.due(true, up, 400000.0, up, -1.0, sun, 1.0, 0.0, 1.0), true)
	_check("1 % of 400 km, again", rule.due(true, up, 404000.0, up, -1.0, sun, 1.0, 0.0, 1.0), false)
	_check("gain 1 %", rule.due(true, up, 400000.0, up, -1.0, sun, 1.01, 0.0, 1.0), false)
	_check("gain 3 %", rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 0.0, 1.0), true)
	_check("the Moon lights up", rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, 1.0), true)
	_check("eclipse 0.5 %", rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, 0.995), false)
	_check("eclipse 2 %", rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, 0.98), true)
	_check("near the Moon", rule.due(true, up, 400000.0, up, 5000.0, sun, 1.03, 1.0e-6, 0.98), true)
	_check("Moon height +100 m", rule.due(true, up, 400000.0, up, 5100.0, sun, 1.03, 1.0e-6, 0.98), false)
	_check("Moon height +400 m", rule.due(true, up, 400000.0, up, 5400.0, sun, 1.03, 1.0e-6, 0.98), true)
	_check("Moon zenith ignored far away", rule.due(true, up, 400000.0, -up, -1.0, sun, 1.03, 1.0e-6, 0.98), true)
	_check("no longer near: stable", rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, 0.98), false)
	print("sky_radiance_rule_test: %s" % ("PASS" if _failures == 0 else "FAIL"))
	quit(1 if _failures > 0 else 0)
