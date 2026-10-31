# The radiance refresh rule (SkyRadianceRefresh) and the far Earth's geometry (RadianceFarEarth): headless, exit code 1 on
# failure.
# godot --headless --path game --script res://tests/sky_radiance_rule_test.gd
extends SceneTree

const CLEAR := [1.0, 1.0, 1.0, 1.0, 1.0]

var _failures := 0


func _check(name_: String, actual: bool, expected: bool) -> void:
	if actual != expected:
		print("sky_radiance_rule_test: FAIL %s: %s" % [name_, actual])
		_failures += 1


func _near(name_: String, actual: float, expected: float, tolerance: float) -> void:
	if absf(actual - expected) > tolerance:
		print("sky_radiance_rule_test: FAIL %s: %f, expected %f" % [name_, actual, expected])
		_failures += 1


func _initialize() -> void:
	var up := Vector3.UP
	var sun := Vector3.RIGHT
	var rule := SkyRadianceRefresh.new()
	# Arguments: tables, Earth zenith, height, Moon zenith, Moon height, Sun, scale, Moon light, fractions.
	_check("first state", rule.due(false, up, 10.0, up, -1.0, sun, 1.0, 0.0, CLEAR), true)
	_check("nothing changed", rule.due(false, up, 10.0, up, -1.0, sun, 1.0, 0.0, CLEAR), false)
	_check("tables loaded", rule.due(true, up, 10.0, up, -1.0, sun, 1.0, 0.0, CLEAR), true)
	_check("no repeat", rule.due(true, up, 10.0, up, -1.0, sun, 1.0, 0.0, CLEAR), false)

	var small := deg_to_rad(0.02)
	var large := deg_to_rad(0.05)
	_check("Sun 0.02 deg", rule.due(true, up, 10.0, up, -1.0, sun.rotated(Vector3.UP, small), 1.0, 0.0, CLEAR), false)
	_check("Sun 0.05 deg (a 2 % step in the twilight)", rule.due(true, up, 10.0, up, -1.0, sun.rotated(Vector3.UP, large), 1.0,
			0.0, CLEAR), true)
	sun = sun.rotated(Vector3.UP, large)
	var turned := up.rotated(Vector3.RIGHT, large)
	_check("zenith 0.05 deg", rule.due(true, turned, 10.0, up, -1.0, sun, 1.0, 0.0, CLEAR), true)
	_check("opposite zenith (a site jump)", rule.due(true, -turned, 10.0, up, -1.0, sun, 1.0, 0.0, CLEAR), true)
	up = -turned
	_check("100 m up", rule.due(true, up, 110.0, up, -1.0, sun, 1.0, 0.0, CLEAR), false)
	_check("300 m up", rule.due(true, up, 310.0, up, -1.0, sun, 1.0, 0.0, CLEAR), true)
	_check("1 % of 400 km", rule.due(true, up, 400000.0, up, -1.0, sun, 1.0, 0.0, CLEAR), true)
	_check("1 % of 400 km, again", rule.due(true, up, 404000.0, up, -1.0, sun, 1.0, 0.0, CLEAR), false)
	_check("gain 1 %", rule.due(true, up, 400000.0, up, -1.0, sun, 1.01, 0.0, CLEAR), false)
	_check("gain 3 %", rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 0.0, CLEAR), true)
	_check("the Moon lights up", rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, CLEAR), true)
	# Each fraction: the eclipse's solar disc, the Earth's disc, both lights on the lunar ground, the cloud factor.
	for k in range(CLEAR.size()):
		var state := CLEAR.duplicate()
		state[k] = 0.995
		_check("fraction %d by 0.5 %%" % k, rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, state), false)
		state[k] = 0.98
		_check("fraction %d by 2 %%" % k, rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, state), true)
		_check("fraction %d back" % k, rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, CLEAR), true)
	_check("near the Moon", rule.due(true, up, 400000.0, up, 5000.0, sun, 1.03, 1.0e-6, CLEAR), true)
	_check("Moon height +100 m", rule.due(true, up, 400000.0, up, 5100.0, sun, 1.03, 1.0e-6, CLEAR), false)
	_check("Moon height +400 m", rule.due(true, up, 400000.0, up, 5400.0, sun, 1.03, 1.0e-6, CLEAR), true)
	_check("leaving the Moon's neighbourhood", rule.due(true, up, 400000.0, -up, -1.0, sun, 1.03, 1.0e-6, CLEAR), true)
	_check("far away the Moon's zenith does not count", rule.due(true, up, 400000.0, up, -1.0, sun, 1.03, 1.0e-6, CLEAR), false)

	_test_far_earth()
	print("sky_radiance_rule_test: %s" % ("PASS" if _failures == 0 else "FAIL"))
	quit(1 if _failures > 0 else 0)


func _test_far_earth() -> void:
	# An eye on the Moon's sphere: the ground fills the lower half; the Earth at elevation e.
	var radius := 1.0e6
	var to_moon := Vector3(0.0, -radius, 0.0)
	var margin := RadianceFarEarth.MIN_RADIUS_RAD
	for elevation_deg in [-1.0, 1.0, 3.0, 10.0]:
		var e := deg_to_rad(elevation_deg)
		var to_earth := Vector3(cos(e), sin(e), 0.0) * 1.0e8
		var lift := RadianceFarEarth.lift_rad(to_earth, to_moon, radius)
		_near("lift at %+.0f deg" % elevation_deg, lift, maxf(margin - e, 0.0), 1.0e-6)
		var drawn := RadianceFarEarth.lifted_direction(to_earth, to_moon, lift)
		_near("the disc stands clear at %+.0f deg" % elevation_deg, asin(drawn.y), maxf(margin, e), 1.0e-6)
	# The ground takes the eye's shadow near it, none from orbit.
	_near("on the ground", RadianceFarEarth.ground_fraction(0.25, 20.0), 0.25, 1.0e-9)
	_near("in orbit", RadianceFarEarth.ground_fraction(0.0, 100000.0), 1.0, 1.0e-9)
	_near("half way", RadianceFarEarth.ground_fraction(0.0, 5500.0), 0.5, 1.0e-9)
