# GPU acceptance check of the clouds' look over a climb (look fixes T7c fix 3, user 2026-11-08: no visible switch or
# edge in the clouds at any height, view or Sun angle). The game's own scene (main.tscn) with its tone map, eye
# adaptation off (a fixed exposure, as render_check --fixed_exposure), the shipped analog snapshot of each epoch. For
# three sites (the Gulf's broken cloud, the Atlantic's cumulus, the Peruvian stratocumulus deck) and five views (straight
# down, 30 / 45 / 60 deg zenith angle at the aim point with a high Sun, 45 deg at a low Sun) the camera climbs along one
# fixed line of sight to the aim point (FLYUP_AIM_HEIGHT_M over the site; the camera south of it, looking north) from
# FLYUP_HEIGHTS_M.x to .y in steps of FLYUP_STEP_RATIO. Per step: 4 x 4 ground regions spread over the middle of the
# lower view (their blocks the same ground size in both images) - the mean |dL| of their luma (sRGB display codes,
# Rec. 709) and the regions' mean luma change per km of climb, each within the recorded state (FLYUP_BASELINE, at least
# the targets FLYUP_REGION_CODES / FLYUP_GLOBAL_PER_KM): a regression guard.
# The climb starts above every cloud top (the analog's band ends below 17 km): below it the camera enters the clouds and
# the view changes physically.
# Needs a GPU (no --headless), ~15-25 min: godot --path game --script res://tests/cloud_flyup_check.gd
#   [-- --save=<dir>] (every step's PNG) [--sites=gulf,cumulus,deck] [--views=0,30,45,60,low] [--ratio=<f>]
# Exit code 1 on failure. A [slow]-tier acceptance check (docs/setup.md "Cloud fly-up acceptance check").
extends SceneTree

## [E] The climb (m): from above the analog's cloud tops (band <= 16.8 km) to beyond the ISS (the volume's 410 km
## extent from above, the flat levels' last octaves); the height ratio per step (~26 steps).
const FLYUP_HEIGHTS_M := Vector2(20000.0, 2000000.0)
const FLYUP_STEP_RATIO := 1.2
## [E] The aim point's height (m): inside the low clouds' layer.
const FLYUP_AIM_HEIGHT_M := 1500.0
## [E] The gates (user / orchestrator 2026-11-08): per step the 4 x 4 regions' mean |dL| below ~4 sRGB codes, the global
## luma change below 1.5 % per km.
const FLYUP_REGION_CODES := 4.0
const FLYUP_GLOBAL_PER_KM := 0.015
## The recorded state (user 2026-11-08: today's clouds accepted; the check guards it): per sequence (site_view) the
## worst step's region change (codes) and global change (fraction per km) measured 2026-11-08 after T7c fix 3. A
## sequence gates at its baseline + FLYUP_BASELINE_MARGIN_CODES (and x FLYUP_BASELINE_GLOBAL_FACTOR), at least the
## targets above; the target misses are listed. [E] margin: a rerun moved the worst steps by -0.3 to -2.3 codes (the bake's state per run; cumulus straight down 10.2 -> 8.0).
const FLYUP_BASELINE_MARGIN_CODES := 2.0
const FLYUP_BASELINE_GLOBAL_FACTOR := 1.5
const FLYUP_BASELINE := {
	"gulf_0": Vector2(9.99, 0.01224),
	"gulf_30": Vector2(5.02, 0.00648),
	"gulf_45": Vector2(5.07, 0.00352),
	"gulf_60": Vector2(6.01, 0.00164),
	"gulf_low": Vector2(3.49, 0.00506),
	"cumulus_0": Vector2(10.24, 0.01180),
	"cumulus_30": Vector2(12.58, 0.01599),
	"cumulus_45": Vector2(8.40, 0.01263),
	"cumulus_60": Vector2(9.42, 0.01032),
	"cumulus_low": Vector2(5.70, 0.00342),
	"deck_0": Vector2(7.97, 0.00311),
	"deck_30": Vector2(5.93, 0.00253),
	"deck_45": Vector2(5.54, 0.00299),
	"deck_60": Vector2(8.34, 0.00449),
	"deck_low": Vector2(7.06, 0.00376),
}
## [E] The regions: a 4 x 4 grid over the middle FLYUP_GRID_SPAN of the lower view (its half-widths at the aim point's
## distance), each a square block of FLYUP_BLOCK_SHARE of the lower view's height across.
const FLYUP_GRID_SPAN := 0.6
const FLYUP_BLOCK_SHARE := 0.12
## Sites: name, (lat, lon), the high-Sun and the low-Sun epoch (UTC; the analog snapshot of the epoch's month and hour).
const FLYUP_SITES := {
	"gulf": ["Gulf 26 N 97 W", Vector2(26.0, -97.157), "2025-11-07T15:00:00", "2025-11-07T22:50:00"],
	"cumulus": ["cumulus 15 N 45 W", Vector2(15.0, -45.0), "2025-11-07T14:30:00", "2025-11-07T20:20:00"],
	"deck": ["Sc deck 20 S 80 W", Vector2(-20.0, -80.0), "2025-11-07T15:00:00", "2025-11-07T22:40:00"],
}
const SETTLE_FRAMES := 8
const MAX_FRAMES := 6000
const FOV_DEG := 60.0
const WGS84_A_M := 6378137.0
const WGS84_E2 := 6.69437999014e-3

var _main: Node
var _world: World
var _failures := 0
var _samples := 0
var _save_dir := ""
var _sites: PackedStringArray = ["gulf", "cumulus", "deck"]
var _views: PackedStringArray = ["0", "30", "45", "60", "low"]
var _ratio := FLYUP_STEP_RATIO
var _worst_region := 0.0
var _worst_global := 0.0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--save="):
			_save_dir = arg.trim_prefix("--save=")
		elif arg.begins_with("--sites="):
			_sites = arg.trim_prefix("--sites=").split(",", false)
		elif arg.begins_with("--views="):
			_views = arg.trim_prefix("--views=").split(",", false)
		elif arg.begins_with("--ratio="):
			_ratio = maxf(float(arg.trim_prefix("--ratio=")), 1.01)
	_run.call_deferred()


func _run() -> void:
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_world = _main.get_node("World")
	_world.weather.enabled = false
	for i in range(MAX_FRAMES):  # the cloud kernels built before anything is rendered
		await process_frame
		if i > 2 and _world.clouds.kernels_status() == "":
			break
	for i in range(MAX_FRAMES):
		await process_frame
		if _world.atmosphere.ready and _world.clouds.noise_ready and _world.clouds.transfer_ready:
			break
	if not (_world.atmosphere.ready and _world.clouds.noise_ready and _world.clouds.transfer_ready):
		_fail_and_quit("atmosphere / cloud noise / transfer table not ready")
		return
	_hide_ui(_main)
	_world.atmosphere.eye_adaptation_enabled = false
	_world.clock.warp = 0.0
	_world.set_focus(World.FOCUS_EARTH)
	var heights: Array[float] = []
	var h := FLYUP_HEIGHTS_M.x
	while h < FLYUP_HEIGHTS_M.y * 1.0001:
		heights.append(h)
		h = minf(h * _ratio, FLYUP_HEIGHTS_M.y) if h < FLYUP_HEIGHTS_M.y else h * 2.0
	for key in _sites:
		if not FLYUP_SITES.has(key):
			_fail_and_quit("unknown site %s" % key)
			return
		var site: Array = FLYUP_SITES[key]
		for view in _views:
			var low := view == "low"
			var zenith := 45.0 if low else float(view)
			var utc: String = site[3] if low else site[2]
			if not _show_analog(utc):
				_fail_and_quit("no analog snapshot for %s" % utc)
				return
			_world.clock.set_time(_world.core.utc_to_tdb_s(utc))
			await _climb("%s, %s" % [site[0], "45 deg, low Sun" if low else "%s deg" % view], key + "_" + view,
					site[1], zenith, heights)
	print("cloud_flyup_check: worst step %.2f codes (gate %.1f), worst global %.3f %% per km (gate %.1f)" % [
			_worst_region, FLYUP_REGION_CODES, _worst_global * 100.0, FLYUP_GLOBAL_PER_KM * 100.0])
	print("cloud_flyup_check: %s (%d samples, %d failures)" % ["PASS" if _failures == 0 else "FAIL", _samples,
			_failures])
	quit(1 if _failures > 0 else 0)


## One climb: the camera on the line of sight that leaves the aim point (p, FLYUP_AIM_HEIGHT_M up) at zenith_deg toward
## the south, at each height; one check per sequence (its worst step), every step printed. Each step's regions are the
## lower view's (_regions); both images are measured through their own camera (_pixel_of).
func _climb(label: String, tag: String, p: Vector2, zenith_deg: float, heights: Array[float]) -> void:
	var prev: Dictionary = {}
	var prev_height := 0.0
	var worst := Vector2.ZERO
	var worst_at := ""
	print("  %s:" % label)
	for h in heights:
		var distance := _place(p, zenith_deg, h)
		await _settle()
		var image := _frame()
		if _save_dir != "":
			image.save_png(_save_dir.path_join("flyup_%s_%04.0f.png" % [tag, h * 1e-3]))
		if not prev.is_empty():
			# the lower view's regions seen now: blocks of the same ground size
			var radius := maxi(int(round(float(prev["radius"]) * float(prev["distance"]) / distance)), 1)
			var sum_d := 0.0
			var sum_a := 0.0
			var sum_b := 0.0
			var n := 0
			for i in range((prev["points"] as Array).size()):
				var pb := _pixel_of(prev["points"][i])
				if not _inside(image, pb, radius):
					continue
				var la: float = prev["luma"][i]
				var lb := _block_luma(image, pb, radius)
				sum_d += absf(lb - la)
				sum_a += la
				sum_b += lb
				n += 1
			var region := sum_d / maxi(n, 1)
			var per_km := absf(sum_b - sum_a) / maxf(sum_a, 1.0e-6) / ((h - prev_height) * 1e-3)
			print("    %7.1f -> %7.1f km: regions %.2f codes, global %.3f %% per km (mean %.1f codes, %d regions)" % [
					prev_height * 1e-3, h * 1e-3, region, per_km * 100.0, sum_a / maxi(n, 1), n])
			if n < 8:
				print("    (fewer than 8 regions in view)")
			if region > worst.x:
				worst.x = region
				worst_at = "%.0f -> %.0f km" % [prev_height * 1e-3, h * 1e-3]
			worst.y = maxf(worst.y, per_km)
		prev = _regions(image, p, distance)
		prev_height = h
	_worst_region = maxf(_worst_region, worst.x)
	_worst_global = maxf(_worst_global, worst.y)
	var gate := Vector2(FLYUP_REGION_CODES, FLYUP_GLOBAL_PER_KM)
	if FLYUP_BASELINE.has(tag):
		var base: Vector2 = FLYUP_BASELINE[tag]
		gate = Vector2(maxf(gate.x, base.x + FLYUP_BASELINE_MARGIN_CODES), maxf(gate.y, base.y * FLYUP_BASELINE_GLOBAL_FACTOR))
	if worst.x > FLYUP_REGION_CODES or worst.y > FLYUP_GLOBAL_PER_KM:
		print("    (above the target %.1f codes / %.1f %% per km - the recorded state, weather-clouds.md open points)" % [
				FLYUP_REGION_CODES, FLYUP_GLOBAL_PER_KM * 100.0])
	_check("climb %.0f -> %.0f km, %s: per step the 4 x 4 regions within %.2f codes, the global change within %.3f %% per km"
			% [heights[0] * 1e-3, heights[-1] * 1e-3, label, gate.x, gate.y * 100.0],
			worst.x <= gate.x and worst.y <= gate.y,
			"worst regions %.2f codes (%s), worst global %.3f %% per km" % [worst.x, worst_at, worst.y * 100.0])


## The 4 x 4 ground regions of a view (aim point distance m): points spread over FLYUP_GRID_SPAN of its middle about the
## aim point (along the view stretched by the ground's foreshortening), their block luma (radius FLYUP_BLOCK_SHARE of
## the image height across) in this image; {points, luma, radius, distance}; points out of the image dropped.
func _regions(image: Image, p: Vector2, distance: float) -> Dictionary:
	var aim := _itrf_at(p.x, p.y, FLYUP_AIM_HEIGHT_M)
	var radius := maxi(int(round(image.get_height() * FLYUP_BLOCK_SHARE * 0.5)), 2)
	var points: Array[Vector3] = []
	var luma: Array[float] = []
	var c := _pixel_of(aim)
	var half := image.get_height() * 0.5 * FLYUP_GRID_SPAN
	for i in range(4):
		for j in range(4):
			# a pixel grid about the aim point's pixel, taken to the aim point's height sphere
			var px := c + Vector2(half * ((j + 0.5) / 2.0 - 1.0), half * ((i + 0.5) / 2.0 - 1.0))
			var q := _ground_of(px, aim.length())
			if q == Vector3.ZERO or not _inside(image, px, radius):
				continue
			points.append(q)
			luma.append(_block_luma(image, px, radius))
	return {"points": points, "luma": luma, "radius": radius, "distance": distance}


## The Earth-fixed point where the pixel's ray meets the sphere of radius r (ZERO: it misses).
func _ground_of(px: Vector2, r: float) -> Vector3:
	var dir_render := _world.camera.project_ray_normal(px)
	var dir_gcrf := Frames.from_render(dir_render).normalized()
	var o := _world.camera.position_gcrf_m
	var b := o.dot(dir_gcrf)
	var disc := b * b - (o.length_squared() - r * r)
	if disc < 0.0:
		return Vector3.ZERO
	var t := -b - sqrt(disc)
	if t <= 0.0:
		return Vector3.ZERO
	return _world.earth.fixed_to_gcrf.inverse() * (o + dir_gcrf * t)


## Places the camera at height h on the line of sight from the aim point (p, FLYUP_AIM_HEIGHT_M up; a sphere of
## WGS84_A_M) at zenith_deg toward the south, looking at it; returns the distance to the aim point (m).
func _place(p: Vector2, zenith_deg: float, h: float) -> float:
	var g := _geometry(zenith_deg, h)
	_world.set_ground_site({"id": "custom", "name": "custom", "body": "earth",
			"latitude_deg": p.x - rad_to_deg(g.y), "longitude_deg": p.y, "height_m": 0.0, "eye_height_m": h})
	_world.camera.fov = FOV_DEG
	_world.camera.azimuth = 0.0
	_world.camera.elevation = -(PI * 0.5 - (deg_to_rad(zenith_deg) - g.y))
	return g.x


## (distance to the aim point, central angle aim -> camera) for a camera at height h on the line of sight of zenith_deg.
static func _geometry(zenith_deg: float, h: float) -> Vector2:
	var z := deg_to_rad(zenith_deg)
	var r_p := WGS84_A_M + FLYUP_AIM_HEIGHT_M
	var r_c := WGS84_A_M + h
	var distance := -r_p * cos(z) + sqrt(r_p * r_p * cos(z) * cos(z) - r_p * r_p + r_c * r_c)
	return Vector2(distance, asin(clampf(distance * sin(z) / r_c, -1.0, 1.0)))


static func _inside(image: Image, c: Vector2, radius: int) -> bool:
	return c.x - radius >= 0 and c.y - radius >= 0 and c.x + radius < image.get_width() \
			and c.y + radius < image.get_height()


## The mean Rec. 709 luma (sRGB codes 0-255) of the (2 r + 1)^2 block about c (every second pixel).
static func _block_luma(image: Image, c: Vector2, radius: int) -> float:
	var sum := 0.0
	var n := 0
	var step := maxi(radius / 24, 1)
	for y in range(int(round(c.y)) - radius, int(round(c.y)) + radius + 1, step):
		for x in range(int(round(c.x)) - radius, int(round(c.x)) + radius + 1, step):
			var col := image.get_pixel(x, y)
			sum += 0.2126 * col.r + 0.7152 * col.g + 0.0722 * col.b
			n += 1
	return 255.0 * sum / maxf(n, 1)


func _show_analog(utc: String) -> bool:
	var unix := Time.get_unix_time_from_datetime_string(utc)
	var file: String = _world.weather.analog_file(unix / 3600.0 + WeatherService.HOURS_1900_TO_1970)
	if file == "":
		return false
	var snapshot: Dictionary = _world.core.weather_load_snapshot(file)
	if not snapshot.get("ok", false):
		return false
	_world.clouds.show_now(snapshot, snapshot, 0.0)
	_world.core.weather_set_active(file, file, 0.0)
	return true


## Waits until the terrain and the clouds have settled (cloud_far_check's rule).
func _settle() -> void:
	var idle := 0
	for i in range(MAX_FRAMES):
		await process_frame
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		var uploading: bool = _world.clouds.state()["uploading"]
		var baked := _world.clouds.effect.baked
		var baking: bool = ShaderGlobals.value(&"cloud_enabled") and (baked == null or not baked.settled())
		idle = idle + 1 if loading == 0 and not uploading and not baking else 0
		if idle >= SETTLE_FRAMES and i >= 2 * SETTLE_FRAMES:
			break


func _frame() -> Image:
	var image := root.get_texture().get_image()
	image.convert(Image.FORMAT_RGBA8)
	return image


func _pixel_of(itrf: Vector3) -> Vector2:
	var q := _world.earth.fixed_to_gcrf * itrf - _world.camera.position_gcrf_m
	var dir := Frames.to_render(q).normalized()
	return _world.camera.unproject_position(_world.camera.global_position + dir * 1000.0)


## Earth-fixed (ITRF) point at geodetic (lat, lon) deg, h m above the ellipsoid.
static func _itrf_at(lat_deg: float, lon_deg: float, h: float) -> Vector3:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	var n := WGS84_A_M / sqrt(1.0 - WGS84_E2 * sin(lat) * sin(lat))
	return Vector3((n + h) * cos(lat) * cos(lon), (n + h) * cos(lat) * sin(lon), (n * (1.0 - WGS84_E2) + h) * sin(lat))


func _hide_ui(node: Node) -> void:
	for child in node.get_children():
		if child is CanvasLayer:
			(child as CanvasLayer).visible = false
		elif child is Control:
			(child as Control).visible = false
		else:
			_hide_ui(child)


func _check(label: String, ok: bool, detail: String) -> void:
	_samples += 1
	if not ok:
		_failures += 1
	print("  %s  %s: %s" % ["ok  " if ok else "FAIL", label, detail])


func _fail_and_quit(message: String) -> void:
	print("cloud_flyup_check: FAIL - " + message)
	quit(1)
