# Headless test of the Sun on the lunar terrain at every height: MoonSunHorizon (the terrain's horizon, the exact
# relief pre-check) and Atmosphere.sun_visible_near_moon (the combination with the Earth's eclipse). Needs the lunar
# terrain data (skips without). Run: godot --headless --path game --script res://tests/moon_sun_test.gd
extends SceneTree

const IM2_LAT_DEG := -84.79
const IM2_LON_DEG := 29.19
const AU_M := 149597870700.0
## Earth - Moon distance for the eclipse geometry (m, mean).
const EARTH_MOON_M := 384400000.0
## Sweep step of the Sun's elevation across a rim (deg).
const RIM_STEP_DEG := 0.1

var _failures := 0
var _core := StarsimCore.new()


func _initialize() -> void:
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	var present: Dictionary = _core.load_terrain(config_dir)
	if not present.get("moon", false):
		print("moon_sun_test: skipped, no lunar terrain data")
		quit(0)
		return
	_check_nan_fallback()
	_check_relief_bound_exact()
	_check_below_ground_clamp()
	_check_rim_sweep()
	_check_height_continuity()
	_check_earth_eclipse()
	_check_orbit()
	_report_cost()
	print("moon_sun_test: %s" % ("PASSED" if _failures == 0 else "%d FAILED" % _failures))
	quit(0 if _failures == 0 else 1)


## Moon-fixed position of a point height_m above the terrain at lat / lon (deg).
func _site(lat_deg: float, lon_deg: float, height_m: float) -> Vector3:
	var ground: Dictionary = _core.surface_samples("moon", PackedFloat64Array([lat_deg]), PackedFloat64Array([lon_deg]))
	return _up(lat_deg, lon_deg) * (Atmosphere.MOON_RADIUS_M + float(ground["height_m"][0]) + height_m)


static func _up(lat_deg: float, lon_deg: float) -> Vector3:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	return Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))


## Local north at an up vector (the pole's longitude fixes east like MoonSunHorizon).
static func _north(up: Vector3) -> Vector3:
	var lon := atan2(up.y, up.x)
	return up.cross(Vector3(-sin(lon), cos(lon), 0.0))


## The Sun at 1 AU in the horizontal direction `toward`, raised to elevation_rad above the local horizontal.
static func _sun_at(point: Vector3, up: Vector3, toward: Vector3, elevation_rad: float) -> Vector3:
	return point + (cos(elevation_rad) * toward + sin(elevation_rad) * up) * AU_M


## Without terrain (a core that has none) and for the Earth: NAN, the callers keep the old value.
func _check_nan_fallback() -> void:
	var bare := StarsimCore.new()
	var point := _site(IM2_LAT_DEG, IM2_LON_DEG, 2.0)
	var sun := point + Vector3(0.0, 0.0, AU_M)
	_check("no terrain -> NAN", is_nan(MoonSunHorizon.sun_visible_fraction(bare, point, sun)), "")
	_check("max height: Earth / no terrain NAN, Moon finite", is_nan(_core.terrain_max_height_m("earth"))
			and is_nan(bare.terrain_max_height_m("moon")) and _core.terrain_max_height_m("moon") > 5000.0,
			"moon %.0f m" % _core.terrain_max_height_m("moon"))
	# NAN terrain fraction: the old mean-sphere / ground-sphere value, bit for bit.
	var moon := Vector3(EARTH_MOON_M, 0.0, 0.0)
	var p := moon + point
	var s := moon + sun
	var mean := 0.37
	var old := Atmosphere.moon_visible_fraction(p, s, moon, mean)
	_check("NAN terrain fraction -> moon_visible_fraction",
			Atmosphere.sun_visible_near_moon(p, s, moon, mean, NAN) == old, "%.4f" % old)


## The pre-check's bound is exact: no marched horizon from a point at height h exceeds
## -acos((R + max) / (R + h)) in any sector (sites over the relief, heights above its top).
func _check_relief_bound_exact() -> void:
	var max_height_m: float = _core.terrain_max_height_m("moon")
	var worst := -INF
	var violations := 0
	var sites := [Vector2(IM2_LAT_DEG, IM2_LON_DEG), Vector2(-89.5, 0.0), Vector2(-85.5, 31.0), Vector2(0.0, 0.0),
			Vector2(-60.0, -150.0), Vector2(20.0, 100.0), Vector2(70.0, -40.0)]
	for site: Vector2 in sites:
		for extra_m: float in [100.0, 1000.0, 20000.0, 100000.0, 400000.0]:
			var height_m := max_height_m + extra_m
			var bound := MoonSunHorizon.relief_horizon_bound_rad(height_m, max_height_m)
			for k in 16:
				var horizon: float = _core.horizon_elevation_rad("moon", site.x, site.y, height_m, k * TAU / 16.0)
				worst = maxf(worst, horizon - bound)
				if horizon > bound + 1e-9:
					violations += 1
	_check("relief bound holds for every marched horizon", violations == 0,
			"max(horizon - bound) %s rad over 7 sites x 5 heights x 16 sectors (max height %.0f m)" % [str(worst), max_height_m])
	var inside := MoonSunHorizon.relief_horizon_bound_rad(max_height_m * 0.5, max_height_m)
	_check("no bound at or below the relief's top", is_inf(inside), "")


## A point below the terrain surface (a vehicle origin in contact) is marched from just above it.
func _check_below_ground_clamp() -> void:
	var lat := IM2_LAT_DEG
	var lon := IM2_LON_DEG
	var up := _up(lat, lon)
	var north := _north(up)
	var above := _site(lat, lon, MoonSunHorizon.MIN_HEIGHT_ABOVE_GROUND_M)
	var below := _site(lat, lon, -3.0)
	var ground: Dictionary = _core.surface_samples("moon", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var height_m := float(ground["height_m"][0]) + MoonSunHorizon.MIN_HEIGHT_ABOVE_GROUND_M
	var horizon: float = _core.horizon_elevation_rad("moon", lat, lon, height_m, 0.0)
	var f_above := MoonSunHorizon.sun_visible_fraction(_core, above, _sun_at(above, up, north, horizon))
	var f_below := MoonSunHorizon.sun_visible_fraction(_core, below, _sun_at(below, up, north, horizon))
	_check("point under the ground marched from just above it", absf(f_above - 0.5) < 0.02
			and absf(f_below - f_above) < 1e-6, "above %.4f, 3 m below the ground %.4f" % [f_above, f_below])


## A ground point facing a high rim: the Sun's elevation swept through the rim's horizon in its azimuth is 0 below,
## 1 above, monotonic and continuous (no step above the circular segment's steepest share).
func _check_rim_sweep() -> void:
	var best_lat := 0.0
	var best_lon := 0.0
	var best_sector := 0
	var best_horizon := -INF
	var lat := -89.9
	while lat <= -84.0:
		for lon in range(-180, 180, 30):
			var height_m: float = _core.surface_samples("moon", PackedFloat64Array([lat]),
					PackedFloat64Array([float(lon)]))["height_m"][0] + 2.0
			for k in 16:
				var h: float = _core.horizon_elevation_rad("moon", lat, float(lon), height_m, k * TAU / 16.0)
				if h > best_horizon:
					best_horizon = h
					best_lat = lat
					best_lon = float(lon)
					best_sector = k
		lat += 0.7
	var up := _up(best_lat, best_lon)
	var point := _site(best_lat, best_lon, 2.0)
	var lon_rad := deg_to_rad(best_lon)
	var east := Vector3(-sin(lon_rad), cos(lon_rad), 0.0)
	var azimuth := best_sector * TAU / 16.0
	var toward := cos(azimuth) * _north(up) + sin(azimuth) * east
	var steps := int(6.0 / RIM_STEP_DEG)
	var previous := -1.0
	var max_step := 0.0
	var monotonic := true
	var first := 0.0
	var last := 0.0
	for i in steps + 1:
		var elevation := best_horizon + deg_to_rad(-3.0 + i * RIM_STEP_DEG)
		var f := MoonSunHorizon.sun_visible_fraction(_core, point, _sun_at(point, up, toward, elevation))
		if i == 0:
			first = f
		last = f
		if previous >= 0.0:
			monotonic = monotonic and f >= previous - 1e-9
			max_step = maxf(max_step, f - previous)
		previous = f
	# Steepest slope of the circular segment: 2 / pi per disc radius.
	var radius_rad := asin(Atmosphere.SUN_RADIUS_M / AU_M)
	var allowed := 2.0 / PI * deg_to_rad(RIM_STEP_DEG) / radius_rad + 1e-6
	_check("rim sweep: 0 below, 1 above, monotonic, continuous", first == 0.0 and last == 1.0 and monotonic
			and max_step <= allowed, "rim %.2f deg at (%.1f, %.0f) sector %d; max step %.3f of %.3f allowed" % [
			rad_to_deg(best_horizon), best_lat, best_lon, best_sector, max_step, allowed])


## From 2 m to 200 km over one site, the Sun fixed just below the ground's horizon: the fraction rises from 0 to 1
## without a jump (the pre-check's exit at the relief's top included), and far up equals the mean sphere's value.
func _check_height_continuity() -> void:
	var up := _up(IM2_LAT_DEG, IM2_LON_DEG)
	var north := _north(up)
	var ground: Dictionary = _core.surface_samples("moon", PackedFloat64Array([IM2_LAT_DEG]),
			PackedFloat64Array([IM2_LON_DEG]))
	var ground_m := float(ground["height_m"][0])
	var horizon_2m: float = _core.horizon_elevation_rad("moon", IM2_LAT_DEG, IM2_LON_DEG, ground_m + 2.0, 0.0)
	var elevation := horizon_2m - deg_to_rad(0.3)  # a disc radius (0.267 deg) and a bit below the 2 m horizon
	var previous := NAN
	var max_jump := 0.0
	var h := 2.0
	var first := 0.0
	var last := 0.0
	while h <= 200000.0:
		var point := up * (Atmosphere.MOON_RADIUS_M + ground_m + h)
		var f := MoonSunHorizon.sun_visible_fraction(_core, point, _sun_at(point, up, north, elevation))
		if is_nan(previous):
			first = f
		else:
			max_jump = maxf(max_jump, absf(f - previous))
		previous = f
		last = f
		h *= 1.05
	var far := up * (Atmosphere.MOON_RADIUS_M + 200000.0)
	var mean := 1.0 - Atmosphere.occulted_fraction(_sun_at(far, up, north, elevation) - far, Atmosphere.SUN_RADIUS_M,
			-far, Atmosphere.MOON_RADIUS_M)
	_check("height sweep 2 m - 200 km continuous", first == 0.0 and absf(last - mean) < 1e-9 and max_jump < 0.1,
			"first %.3f, last %.3f (mean sphere %.3f), max step %.4f" % [first, last, mean, max_jump])


## Lunar eclipse near the ground: a point 2 m above the IM-2 ground, the Sun 30 deg up (far above the terrain
## horizon), the Earth on the line to the Sun (umbra), and offset so its limb crosses the Sun's centre (penumbra).
func _check_earth_eclipse() -> void:
	var up := _up(IM2_LAT_DEG, IM2_LON_DEG)
	var north := _north(up)
	var point_fixed := _site(IM2_LAT_DEG, IM2_LON_DEG, 2.0)
	var sun_fixed := _sun_at(point_fixed, up, north, deg_to_rad(30.0))
	var terrain := MoonSunHorizon.sun_visible_fraction(_core, point_fixed, sun_fixed)
	var s_dir := (sun_fixed - point_fixed).normalized()
	var across := s_dir.cross(up).normalized()  # perpendicular to the Sun line
	var earth_radius_rad := asin(Atmosphere.EARTH_MEAN_RADIUS_M / EARTH_MOON_M)
	var sun_radius_rad := asin(Atmosphere.SUN_RADIUS_M / AU_M)
	var results := []
	for offset_rad: float in [0.0, earth_radius_rad]:  # Earth's centre on the line / its limb on the Sun's centre
		# Earth-centred vectors: the Earth at the origin, the point EARTH_MOON_M from it at the angle offset_rad from the Sun.
		var to_sun := s_dir
		var earth_to_point := -(cos(offset_rad) * to_sun + sin(offset_rad) * across) * EARTH_MOON_M
		var point := earth_to_point
		var sun := point + (sun_fixed - point_fixed)
		var moon := point - point_fixed
		var f := Atmosphere.sun_visible_near_moon(point, sun, moon, 1.0, terrain)
		var expected := 1.0 - Atmosphere.occulted_fraction(sun - point, Atmosphere.SUN_RADIUS_M, -point,
				Atmosphere.EARTH_MEAN_RADIUS_M)
		results.append([f, expected])
	var umbra: Array = results[0]
	var limb: Array = results[1]
	_check("terrain Sun: 30 deg up -> 1", terrain == 1.0, "%.4f" % terrain)
	_check("Earth eclipse near the ground: umbra", umbra[0] < 1e-6 and absf(umbra[0] - umbra[1]) < 1e-9,
			"fraction %s" % umbra[0])
	# Physical anchor: a straight limb through the disc's centre hides half of it (the Earth's limb is 3.6 disc radii
	# wide in angle: its curvature moves that by 3 percent: sagitta r^2 / 2R = 0.037 deg of the 0.53 deg disc).
	_check("Earth eclipse near the ground: penumbra", absf(limb[0] - limb[1]) < 1e-9 and absf(limb[0] - 0.5) < 0.04
			and limb[0] < 0.99, "fraction %.4f (expected %.4f, ~0.5 at the limb; Sun %.3f / Earth %.3f deg)" % [
			limb[0], limb[1], rad_to_deg(sun_radius_rad), rad_to_deg(earth_radius_rad)])


## Lunar orbit (100 km): the terrain's fraction against the mean sphere's from the geometry - the Sun's disc above
## the relief bound -> 1, the Sun's disc deep below the lowest possible horizon -> 0, both as the mean sphere says.
func _check_orbit() -> void:
	var up := _up(IM2_LAT_DEG, IM2_LON_DEG)
	var north := _north(up)
	var height_m := 100000.0
	var point := up * (Atmosphere.MOON_RADIUS_M + height_m)
	var dip := acos(Atmosphere.MOON_RADIUS_M / (Atmosphere.MOON_RADIUS_M + height_m))
	var radius_rad := asin(Atmosphere.SUN_RADIUS_M / AU_M)
	var cases := []
	for elevation: float in [-dip + deg_to_rad(2.0), -dip - deg_to_rad(4.0)]:
		var sun := _sun_at(point, up, north, elevation)
		var terrain := MoonSunHorizon.sun_visible_fraction(_core, point, sun)
		var mean := 1.0 - Atmosphere.occulted_fraction(sun - point, Atmosphere.SUN_RADIUS_M, -point,
				Atmosphere.MOON_RADIUS_M)
		cases.append([terrain, mean])
	_check("100 km orbit: Sun 2 deg above the sphere's horizon", cases[0][0] == 1.0 and cases[0][1] == 1.0,
			"terrain %.4f, mean sphere %.4f (disc radius %.3f deg)" % [cases[0][0], cases[0][1], rad_to_deg(radius_rad)])
	_check("100 km orbit: Sun 4 deg below the sphere's horizon", cases[1][0] == 0.0 and cases[1][1] == 0.0,
			"terrain %.4f, mean sphere %.4f" % [cases[1][0], cases[1][1]])


## Cost of one fraction on the main thread (two marches) over rough highlands near IM-2 with the Sun at the local
## horizon (no pre-check exit): printed, not asserted.
func _report_cost() -> void:
	for height_m: float in [2.0, 5000.0, 20000.0, 100000.0, 1000000.0]:
		var up := _up(-85.5, 31.0)
		var point := up * (Atmosphere.MOON_RADIUS_M + height_m + 2000.0)
		var elevation := -acos(clampf((Atmosphere.MOON_RADIUS_M + 3000.0) / point.length(), 0.0, 1.0))
		var toward := _north(up).rotated(up, 0.2)  # between two sectors: two marches
		var sun := _sun_at(point, up, toward, elevation)
		MoonSunHorizon.sun_visible_fraction(_core, point, sun)
		var started := Time.get_ticks_usec()
		var f := 0.0
		for i in 10:
			f = MoonSunHorizon.sun_visible_fraction(_core, point, sun)
		print("cost: height %8.0f m: %.3f ms per fraction (%.3f)" % [height_m, (Time.get_ticks_usec() - started) / 10000.0, f])


func _check(label: String, ok: bool, detail: String) -> void:
	print("%s %s%s" % ["ok  " if ok else "FAIL", label, (": " + detail) if detail != "" else ""])
	if not ok:
		_failures += 1
