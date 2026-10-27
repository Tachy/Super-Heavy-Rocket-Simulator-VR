# GPU check of the cloud march's step aliasing (static clouds Task 5c): the game's main scene, the shipped analog
# weather of 2024-06-15 21:00 UTC, the VR quality preset (deterministic march, light cache on). Each view is rendered
# with the preset's steps and with REFERENCE_STEP_FACTOR times as many (the converged march along the ray), and the
# cloud layer's own image (CloudEffect.output) must match the reference's:
# - the grain - the high-pass energy at the march-pixel scale: the mean |Laplacian| of the premultiplied luminance at a
#   spacing of one march pixel, over the pixels whose alpha and whose four neighbours' exceed ALPHA_MIN, relative to
#   their mean luminance; and the same on alpha - within GRAIN_TOLERANCE of the reference's: no more (no static
#   salt-and-pepper of a fixed step phase hitting or skipping thin / broken cloud) and no less (no blur);
# - the covered pixels (alpha > ALPHA_MIN), the mean alpha and the mean premultiplied luminance of the whole image
#   within MEAN_TOLERANCE (no cloud lost or gained, no brightness bias of the step's light sample);
# - the RMS difference of the premultiplied luminance over the cloud mask (either image's alpha > ALPHA_MIN), relative
#   to the reference's mean there, below RMS_TOLERANCE (the same picture, not only the same statistics).
# Views: the Cape (thin layers on long oblique paths, the user's t7_cape), the ground at Starbase, a deck seen from 9 km
# and the cirrus seen from 9 km off Peru (t5_cirrus). With the preset the cloud image of the next frame is bit-identical
# (static clouds; render_check's repeat_identical compares the whole frame, whose sea differs between frames at the Cape
# without any cloud change).
# Needs a GPU (no --headless): godot --xr-mode off --path game --script res://tests/cloud_grain_check.gd
# [-- --only=cape] (one view). Exit code 1 on failure.
extends SceneTree

const UTC := "2024-06-15T21:00:00"
## The views (render_check's: t3_render.sh ground / deck, the Cape view of the user's t7_cape, t5_cirrus).
const VIEWS := [
	{"name": "cape", "lat": 28.5729, "lon": -80.649, "azimuth": 90.0, "elevation": 20.0},
	{"name": "ground", "lat": 25.997, "lon": -97.157, "azimuth": 90.0, "elevation": 25.0},
	{"name": "deck", "lat": 25.997, "lon": -97.157, "azimuth": 0.0, "elevation": -10.0, "eye_height": 9000.0},
	{"name": "cirrus", "lat": -18.0, "lon": -78.0, "azimuth": 0.0, "elevation": 15.0, "eye_height": 9000.0},
]
## [E] The reference march: 8 x the preset's coarse and maximum steps (768 / 1536 for VR) - the controller's
## dbg_cape_vr_steps8x, where the grain is gone.
const REFERENCE_STEP_FACTOR := 8
## [E] Allowed relative deviation of the grain (luminance and alpha) from the reference's (brief: ours <= reference x
## 1.25; the same band below: no blur).
const GRAIN_TOLERANCE := 0.25
## [E] Allowed relative deviation of the covered pixels, the mean alpha and the mean luminance (fix round 1: a few per
## cent of cloud or brightness is the smallest change seen side by side in the renders).
const MEAN_TOLERANCE := 0.03
## [E] Largest RMS luminance difference over the cloud mask relative to the reference's mean there (fix round 1): the
## step aliasing of Task 5c's base measured 0.14-0.17 (ground, deck, cirrus); the reference's finer steps near the
## camera band-limit the detail more finely (CLOUD_DETAIL_STEP_FRACTION of the step), which leaves lobe-scale shading
## differences of up to ~0.09 on the ground view's near cumulus by design.
const RMS_TOLERANCE := 0.1
## [E] Cloud pixels: alpha above this (the thin layers' grain lives in alpha 0.1-0.6).
const ALPHA_MIN := 0.1
const FRAMES_TO_SETTLE := 6
const MAX_FRAMES := 6000

var _main: Node
var _world: World
var _failures := 0
var _only := ""


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--only="):
			_only = arg.trim_prefix("--only=")
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_run.call_deferred()


func _run() -> void:
	await process_frame
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	_world.clouds.quality = CloudLayer.Quality.VR
	var unix := Time.get_unix_time_from_datetime_string(UTC)
	var file: String = _world.weather.analog_file(unix / 3600.0 + WeatherService.HOURS_1900_TO_1970)
	if file == "":
		_fail_and_quit("no analog snapshot for %s" % UTC)
		return
	var snapshot: Dictionary = _world.core.weather_load_snapshot(file)
	if not snapshot.get("ok", false):
		_fail_and_quit("analog snapshot not loaded: %s" % snapshot.get("error", ""))
		return
	_world.clouds.wait_until_ready()
	_world.clouds.show_now(snapshot, snapshot, 0.0)
	_world.core.weather_set_active(file, file, 0.0)
	_world.clock.set_time(_world.core.utc_to_tdb_s(UTC))
	_world.atmosphere.eye_adaptation_enabled = false
	_world.set_focus(World.FOCUS_EARTH)
	var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.VR]
	var divisor := int(preset["divisor"])
	print("cloud_grain_check: %s, VR preset (steps %d / %d, divisor %d), reference x%d" % [file, preset["steps"],
			preset["max_steps"], divisor, REFERENCE_STEP_FACTOR])
	for view: Dictionary in VIEWS:
		if _only != "" and view["name"] != _only:
			continue
		var name: String = view["name"]
		_place(view)
		if not await _settle():
			_fail_and_quit("%s: the scene did not settle" % name)
			return
		var first := _world.clouds.effect.output.get_layer_data(0)
		await process_frame
		var second := _world.clouds.effect.output.get_layer_data(0)
		_check("%s: the cloud image of the next frame bit-identical" % name,
				first != null and second != null and first.get_data() == second.get_data(), "")
		ShaderGlobals.set_value(&"cloud_steps", int(preset["steps"]) * REFERENCE_STEP_FACTOR)
		ShaderGlobals.set_value(&"cloud_max_steps", int(preset["max_steps"]) * REFERENCE_STEP_FACTOR)
		var reference_settled: bool = await _settle()
		var reference_image := _world.clouds.effect.output.get_layer_data(0)
		ShaderGlobals.set_value(&"cloud_steps", int(preset["steps"]))
		ShaderGlobals.set_value(&"cloud_max_steps", int(preset["max_steps"]))
		if not reference_settled or first == null or reference_image == null:
			_fail_and_quit("%s: the x%d reference did not settle" % [name, REFERENCE_STEP_FACTOR])
			return
		_compare(name, _Picture.new(first), _Picture.new(reference_image), divisor)
	print("cloud_grain_check: %s" % ("PASS" if _failures == 0 else "FAIL (%d)" % _failures))
	quit(0 if _failures == 0 else 1)


## The view's checks of ours against the reference (header).
func _compare(name: String, ours: _Picture, reference: _Picture, spacing: int) -> void:
	var g := ours.grain(spacing)
	var g_ref := reference.grain(spacing)
	var ratio := g.x / maxf(g_ref.x, 1.0e-9)
	var ratio_alpha := g.z / maxf(g_ref.z, 1.0e-9)
	_check("%s: grain within %.0f %% of the x%d reference" % [name, GRAIN_TOLERANCE * 100.0, REFERENCE_STEP_FACTOR],
			absf(ratio - 1.0) <= GRAIN_TOLERANCE, "%.4f vs %.4f (ratio %.3f; over %d / %d pixels)" % [g.x, g_ref.x,
			ratio, int(g.y), int(g_ref.y)])
	_check("%s: alpha grain within %.0f %%" % [name, GRAIN_TOLERANCE * 100.0],
			absf(ratio_alpha - 1.0) <= GRAIN_TOLERANCE, "%.4f vs %.4f (ratio %.3f)" % [g.z, g_ref.z, ratio_alpha])
	_check_mean("%s: covered pixels" % name, ours.covered, reference.covered)
	_check_mean("%s: mean alpha" % name, ours.alpha_mean, reference.alpha_mean)
	_check_mean("%s: mean luminance" % name, ours.luminance_mean, reference.luminance_mean)
	var rms := ours.rms_against(reference)
	_check("%s: RMS luminance difference over the cloud mask below %.2f" % [name, RMS_TOLERANCE], rms <= RMS_TOLERANCE,
			"%.4f" % rms)


func _check_mean(label: String, ours: float, reference: float) -> void:
	var deviation := ours / maxf(reference, 1.0e-12) - 1.0
	_check("%s within %.0f %%" % [label, MEAN_TOLERANCE * 100.0], absf(deviation) <= MEAN_TOLERANCE,
			"%.5f vs %.5f (%+.2f %%)" % [ours, reference, deviation * 100.0])


## A ground view at the view's site (the eye eye_height above the core's surface, 2 m by default).
func _place(view: Dictionary) -> void:
	var lat: float = view["lat"]
	var lon: float = view["lon"]
	var samples: Dictionary = _world.core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var site := {"id": "grain_" + view["name"], "name": view["name"], "body": "earth", "latitude_deg": lat,
			"longitude_deg": lon, "height_m": samples["height_m"][0] if bool(samples["present"]) else 0.0}
	if view.has("eye_height"):
		site["eye_height_m"] = view["eye_height"]
	_world.set_ground_site(site)
	_world.camera.azimuth = deg_to_rad(view["azimuth"])
	_world.camera.elevation = deg_to_rad(view["elevation"])


## Waits until the terrain is loaded, the clouds' field is shown and the light cache is complete for FRAMES_TO_SETTLE
## frames (false after MAX_FRAMES).
func _settle() -> bool:
	var settled := 0
	for frame in range(MAX_FRAMES):
		await process_frame
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		var clouds: Dictionary = _world.clouds.state()
		var ready: bool = loading == 0 and not _world.atmosphere.loading and _world.clouds.noise_ready \
				and not clouds["uploading"] and clouds["tracking"] and _world.clouds.light_cache.settled()
		settled = settled + 1 if ready else 0
		if settled >= FRAMES_TO_SETTLE:
			return true
	return false


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("  %s  %s: %s" % ["ok  " if ok else "FAIL", label, detail])


func _fail_and_quit(message: String) -> void:
	print("cloud_grain_check: FAIL - " + message)
	quit(1)


## The cloud layer's image as premultiplied luminance and alpha, with its whole-image means.
class _Picture:
	var w: int
	var h: int
	var luminance := PackedFloat32Array()
	var alpha := PackedFloat32Array()
	var covered := 0.0         # pixels with alpha > ALPHA_MIN
	var alpha_mean := 0.0
	var luminance_mean := 0.0

	func _init(image: Image) -> void:
		w = image.get_width()
		h = image.get_height()
		luminance.resize(w * h)
		alpha.resize(w * h)
		for y in range(h):
			for x in range(w):
				var c := image.get_pixel(x, y)
				var i := y * w + x
				luminance[i] = 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b
				alpha[i] = c.a
				covered += 1.0 if c.a > ALPHA_MIN else 0.0
				alpha_mean += c.a
				luminance_mean += luminance[i]
		alpha_mean /= float(w * h)
		luminance_mean /= float(w * h)

	## (x) the luminance grain (header), (y) the pixels it was taken over, (z) the same on alpha (not normalised).
	func grain(spacing: int) -> Vector3:
		var high := 0.0
		var high_alpha := 0.0
		var mean := 0.0
		var count := 0
		for y in range(spacing, h - spacing):
			for x in range(spacing, w - spacing):
				var i := y * w + x
				var n := PackedInt32Array([i - spacing, i + spacing, i - spacing * w, i + spacing * w])
				if alpha[i] <= ALPHA_MIN or alpha[n[0]] <= ALPHA_MIN or alpha[n[1]] <= ALPHA_MIN \
						or alpha[n[2]] <= ALPHA_MIN or alpha[n[3]] <= ALPHA_MIN:
					continue
				high += absf(luminance[i] - 0.25 * (luminance[n[0]] + luminance[n[1]] + luminance[n[2]]
						+ luminance[n[3]]))
				high_alpha += absf(alpha[i] - 0.25 * (alpha[n[0]] + alpha[n[1]] + alpha[n[2]] + alpha[n[3]]))
				mean += luminance[i]
				count += 1
		if count == 0 or mean <= 0.0:
			return Vector3.ZERO
		return Vector3(high / mean, count, high_alpha / count)

	## The RMS luminance difference over the pixels where either image has alpha > ALPHA_MIN, relative to the other's
	## (the reference's) mean luminance there.
	func rms_against(other: _Picture) -> float:
		if other.w != w or other.h != h:
			return INF
		var sum_sq := 0.0
		var sum_ref := 0.0
		var count := 0
		for i in range(w * h):
			if alpha[i] <= ALPHA_MIN and other.alpha[i] <= ALPHA_MIN:
				continue
			var diff := luminance[i] - other.luminance[i]
			sum_sq += diff * diff
			sum_ref += other.luminance[i]
			count += 1
		if count == 0 or sum_ref <= 0.0:
			return 0.0
		return sqrt(sum_sq / count) / (sum_ref / count)
