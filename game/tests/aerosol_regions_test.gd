# The observer's maritime aerosol weight (AerosolRegions, look fixes T9b; atmosphere-scattering.md):
# - the footprint radius: 50 km minimum, then 0.5 x the distance to the horizon, growing with the height;
# - a synthetic map (left half water): weight 1 over the water half, 0 over the land half, 0.5 at the seam with a large
#   footprint (the coarsest pyramid level), continuous in height across the level choice (< 1e-3 per 100 m);
# - longitude wrap at +-180 (equal weight at both edges, no jump across the date line);
# - a missing map falls back to the continental set alone (FALLBACK_WEIGHT, no NaN);
# - orientation of the real map data/textures/earth_water.png (skipped if the git-ignored data is absent): a known ocean
#   point (mid-Pacific) has a high maritime weight, known continental points (central Asia, Sahara) a low one - a map
#   flipped in longitude or latitude fails.
# Headless: godot --headless --path game --script res://tests/aerosol_regions_test.gd
extends SceneTree

const TOL := 1.0e-6
## [call] Orientation gates on the real map at the ground: ocean >= 0.9, continental interior <= 0.1.
const OCEAN_MIN := 0.9
const LAND_MAX := 0.1
## [call] Continuity in height: largest change of the weight per 100 m step.
const HEIGHT_STEP_M := 100.0
const MAX_STEP := 1.0e-3

var _failures := 0


func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		print("FAIL ", what)
	else:
		print("ok   ", what)


func _synthetic() -> AerosolRegions:
	var image := Image.create(256, 128, false, Image.FORMAT_L8)
	for y in 128:
		for x in 256:
			var water := 1.0 if x < 128 else 0.0  # lon -180..0 water, 0..180 land
			image.set_pixel(x, y, Color(water, water, water))
	var r := AerosolRegions.new()
	r.set_map(image)
	return r


func _init() -> void:
	# Footprint.
	_check(is_equal_approx(AerosolRegions.footprint_radius_m(0.0), AerosolRegions.FOOTPRINT_MIN_M), "footprint at 0 m is the minimum")
	_check(is_equal_approx(AerosolRegions.footprint_radius_m(-50.0), AerosolRegions.FOOTPRINT_MIN_M), "footprint below 0 m is the minimum")
	var h := 400000.0
	var expected := 0.5 * sqrt(2.0 * AerosolRegions.EARTH_A_M * h + h * h)
	_check(absf(AerosolRegions.footprint_radius_m(h) - expected) < 1.0e-6 * expected, "footprint at 400 km = 0.5 x horizon distance")
	var prev := 0.0
	var mono := true
	for k in 12:
		var f := AerosolRegions.footprint_radius_m(pow(10.0, 2.0 + 0.5 * k))
		mono = mono and f >= prev
		prev = f
	_check(mono, "footprint non-decreasing in height")

	# Synthetic map: halves, seam, level choice.
	var r := _synthetic()
	_check(r.has_map(), "synthetic map built")
	_check(absf(r.weight(0.0, -90.0, 0.0) - 1.0) < 1.0e-3, "water half 1 at 0 m")
	_check(absf(r.weight(0.0, 90.0, 0.0) - 0.0) < 1.0e-3, "land half 0 at 0 m")
	_check(absf(r.weight(0.0, 0.0, 0.0) - 0.5) < 0.1, "seam ~0.5 at 0 m")
	_check(absf(r.weight(0.0, 0.0, 2.0e7) - 0.5) < 0.02, "seam 0.5 with a large footprint (coarsest level)")
	var worst := 0.0
	var last := r.weight(0.0, -2.0, 0.0)
	var height := HEIGHT_STEP_M
	while height <= 2.0e6:
		var w := r.weight(0.0, -2.0, height)
		worst = maxf(worst, absf(w - last))
		last = w
		height += HEIGHT_STEP_M
	_check(worst < MAX_STEP, "continuous in height across the level choice (largest step %s)" % String.num(worst, 6))
	_check(absf(r.weight(0.0, 180.0, 0.0) - r.weight(0.0, -180.0, 0.0)) < TOL, "longitude 180 = -180")
	_check(absf(r.weight(10.0, 179.99, 0.0) - r.weight(10.0, -179.99, 0.0)) < 0.05, "no jump across the date line (0.02 deg apart)")
	_check(absf(r.weight(10.0, 540.0 - 90.0, 0.0) - r.weight(10.0, 90.0, 0.0)) < TOL, "longitude wraps beyond 360 deg")
	_check(is_finite(r.weight(90.0, 10.0, 0.0)) and is_finite(r.weight(-90.0, 10.0, 0.0)), "poles clamp, finite")

	# Missing map.
	var none := AerosolRegions.from_file("res://does_not_exist_water.png")
	_check(not none.has_map(), "missing map: no pyramid")
	_check(none.weight(10.0, 20.0, 1000.0) == AerosolRegions.FALLBACK_WEIGHT, "missing map: the fallback weight")
	_check(AerosolRegions.FALLBACK_WEIGHT == 0.0, "fallback is the continental set")

	# Real map orientation.
	var path := ProjectSettings.globalize_path("res://").path_join("../data/textures/earth_water.png").simplify_path()
	if not FileAccess.file_exists(path):
		print("SKIP real-map orientation: ", path, " not present (git-ignored data)")
	else:
		var real := AerosolRegions.from_file(path)
		_check(real.has_map(), "real map loaded")
		var pacific := real.weight(0.0, -150.0, 0.0)
		var asia := real.weight(45.0, 85.0, 0.0)
		var sahara := real.weight(25.0, 15.0, 0.0)
		print("     mid-Pacific %.3f, central Asia %.3f, Sahara %.3f" % [pacific, asia, sahara])
		_check(pacific >= OCEAN_MIN, "mid-Pacific (0, -150) maritime weight high")
		_check(asia <= LAND_MAX, "central Asia (45, 85) maritime weight low")
		_check(sahara <= LAND_MAX, "Sahara (25, 15) maritime weight low")

	print("aerosol_regions_test: ", "PASS" if _failures == 0 else "%d FAILED" % _failures)
	quit(1 if _failures > 0 else 0)
