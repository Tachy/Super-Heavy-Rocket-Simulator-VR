# GPU check (windowed, no --headless), a TOOL not a test: the headless runner never runs it.
# The free camera descending toward the sunlit Moon: one image per altitude (nadir-oblique view toward the horizon),
# each checked for graphics errors (NaN / Inf pixels shown as black or magenta, sky-coloured or black holes in the
# ground, a frame that does not settle). Bug hunt 2026-11-08 (look fixes): "below ~350 km over the Moon only graphics
# errors" - that was a stale GDExtension DLL (fixed by the rebuild), kept as the descent regression check; it also prints
# the adopted white and the regolith's colour (game scene = render_check, same pipeline).
# Run: godot --path game --script res://tests/moon_descent_check.gd -- [--out=<dir>] [--alts=1000,400,350,300,100,10]
#      [--utc=2026-11-07T14:00:00] [--sun_el=30] [--below=20]
#      [--over=sun|earth] (sun: where the Sun stands --sun_el high; earth: the sub-Earth point)
#      [--nadir] (straight down instead of --below the horizon)
#      [--stereo=res://tests/data/pimax_dream_air_projections.json] (the VR multiview path, both eye images judged)
#      [--weather=analog|off] (analog: the game's default weather on, the shipped analog snapshot)
#      [--fly[=<rate>]]: a continuous flight instead of settled stops - from the first altitude along the view at
#      <rate> x the altitude per second (the HOTAS's full throttle: 1), an image at each further altitude crossed, no
#      waiting for the terrain (what the player sees).
extends SceneTree

const SETTLE_FRAMES := 30  ## frames without terrain loading before an image is taken
const MAX_FRAMES := 3000  ## per altitude: give up (FAIL) after this many frames
## [call] a ground pixel darker than this (sRGB 0..1, all channels) on the sunlit side counts as a hole / NaN black
const HOLE_LEVEL := 0.02
## [call] share of the central ground box that may be hole-black (crater shadows at sun_el 30: < 1 % measured)
const HOLE_SHARE_MAX := 0.15

var _args := {}
var _main: Node
var _world: World
var _alts: PackedFloat64Array
var _next := 0
var _frames := 0
var _idle := 0
var _step_frames := 0
var _failures := 0
var _out := "res://../screenshots/look_fixes/bug_moon"
var _position := Vector3.ZERO  ## the camera (GCRF, m)
var _basis := Basis.IDENTITY  ## its orientation (render axes)
var _look := Vector3.ZERO  ## its view direction (GCRF)
var _last_delta := 0.0
var _stereo: StereoMockXr  ## --stereo: the stand-in headset
var _capture_wait := -1  ## --stereo: frames waited for the eye images (-1: none requested)


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_alts = String(_args.get("alts", "1000,400,350,300,100,10")).split_floats(",", false)
	_out = ProjectSettings.globalize_path(String(_args.get("out", _out)))
	DirAccess.make_dir_recursive_absolute(_out)
	if _args.has("stereo"):  # --stereo=<projections json>: the VR path (multiview) through the stand-in headset
		_stereo = StereoMockXr.new()
		if not _stereo.load(String(_args["stereo"])):
			push_error("moon_descent_check: --stereo cannot be loaded")
			quit(1)
			return
		XRServer.add_interface(_stereo)
		_stereo.initialize()
		XRServer.primary_interface = _stereo
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(delta: float) -> bool:
	_last_delta = delta
	_frames += 1
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	if _frames == 3:
		_world.clock.set_time(_world.core.utc_to_tdb_s(_args.get("utc", "2026-11-07T14:00:00")))
		_world.set_focus(World.FOCUS_FREE)
		if _args.get("weather", "analog") == "analog":
			_show_analog_weather()
		_place(_alts[0])
		return false
	if _frames < 3:
		return false
	_step_frames += 1
	if _args.has("fly"):
		return _fly()
	var loading: int = _world.moon_terrain.stats["nodes_loading"] + _world.earth_terrain.stats["nodes_loading"]
	_idle = _idle + 1 if loading == 0 and not _world.atmosphere.loading else 0
	if _idle < SETTLE_FRAMES and _step_frames < MAX_FRAMES:
		_place(_alts[_next])  # held in place (the free camera has no command; re-placed against drift)
		return false
	if _idle < SETTLE_FRAMES:
		_fail("%.0f km: terrain did not settle in %d frames" % [_alts[_next], MAX_FRAMES])
	if not _images_ready():
		_place(_alts[_next])
		return false
	_judge(_alts[_next])
	_next += 1
	if _next >= _alts.size():
		print("[moon_descent_check] %s (%d failures)" % ["OK" if _failures == 0 else "FAIL", _failures])
		quit(1 if _failures > 0 else 0)
		return true
	_idle = 0
	_step_frames = 0
	_place(_alts[_next])
	return false


## The free camera `alt_km` above the Moon's mean sphere over the point where the Sun stands --sun_el deg high, looking
## along the Sun's azimuth (down-Sun) --below deg below the horizon.
func _place(alt_km: float) -> void:
	_world.update_world()
	var t: float = _world.clock.tdb_s
	var moon: Vector3 = _world.moon_gcrf_m
	var sun_dir: Vector3 = (_world.core.body_position_gcrf_m("sun", t) - moon).normalized()
	var sun_el := deg_to_rad(float(_args.get("sun_el", "30")))
	var side := sun_dir.cross(Vector3(0, 0, 1)).normalized()  # rotate the up vector away from the subsolar point
	var up := sun_dir.rotated(side, PI / 2.0 - sun_el).normalized()
	if _args.get("over", "sun") == "earth":  # --over=earth: over the sub-Earth point (the near side; at new Moon its night)
		up = (-moon).normalized()
	var position := moon + up * (World.MOON_RADIUS_M + alt_km * 1000.0)
	var horizontal := (-sun_dir + up * sun_dir.dot(up)).normalized()  # down-Sun along the ground
	# --below: degrees below the geometric horizon (its dip acos(R / (R + h)) grows with the altitude)
	var dip := acos(World.MOON_RADIUS_M / (World.MOON_RADIUS_M + alt_km * 1000.0))
	var pitch := -dip - deg_to_rad(float(_args.get("below", "20")))
	var look := (horizontal * cos(pitch) + up * sin(pitch)).normalized()
	var basis := Basis.looking_at(Frames.to_render(look), Frames.to_render(up))
	if _args.has("nadir"):  # --nadir: straight down (image top toward down-Sun); the whole frame is ground below ~350 km
		look = -up
		basis = Basis.looking_at(Frames.to_render(look), Frames.to_render(horizontal))
	_position = position
	_basis = basis
	_look = look
	_hold()


func _hold() -> void:
	_world.free_pose.reset_to(_position, _basis, _world.earth.fixed_to_gcrf, _world.moon.fixed_to_gcrf,
			_world.moon_gcrf_m)
	_world.camera.position_gcrf_m = _position
	_world.camera.free_basis = _basis


## --fly: one frame of the flight (the camera moved along its view by rate x altitude x the frame time); an image at each
## target altitude crossed.
func _fly() -> bool:
	var rate := float(_args["fly"]) if String(_args["fly"]).is_valid_float() else 1.0
	var alt_m := (_position - _world.moon_gcrf_m).length() - World.MOON_RADIUS_M
	if _next == 0:
		_next = 1  # the first altitude is the start
	if _next < _alts.size() and alt_m <= _alts[_next] * 1000.0:
		if not _images_ready():
			_hold()
			return false
		_judge(alt_m / 1000.0)
		_next += 1
	if _next >= _alts.size() or _step_frames > 20 * MAX_FRAMES:
		print("[moon_descent_check] %s (%d failures)" % ["OK" if _failures == 0 else "FAIL", _failures])
		quit(1 if _failures > 0 else 0)
		return true
	_position += _look * maxf(alt_m, 1000.0) * rate * minf(_last_delta, 0.1)
	_hold()
	return false


## The game's weather (on by default): the shipped analog snapshot of the instant, pinned (as render_check --weather=analog).
func _show_analog_weather() -> void:
	var unix := Time.get_unix_time_from_datetime_string(_args.get("utc", "2026-11-07T14:00:00"))
	var file: String = _world.weather.analog_file(unix / 3600.0 + WeatherService.HOURS_1900_TO_1970)
	var snapshot: Dictionary = _world.core.weather_load_snapshot(file) if file != "" else {}
	if not snapshot.get("ok", false):
		_fail("no analog weather snapshot for --utc (%s)" % file)
		return
	_world.clouds.wait_until_ready()
	_world.clouds.show_now(snapshot, snapshot, 0.0)
	_world.core.weather_set_active(file, file, 0.0)
	print("[moon_descent_check] weather from %s" % file)


## --stereo: true once the eye images of a frame drawn at the held pose are read back (desktop: always).
func _images_ready() -> bool:
	if _stereo == null:
		return true
	if _capture_wait < 0:
		_stereo.captured = []
		_stereo.capture_requested = true
		_capture_wait = 0
		return false
	_capture_wait += 1
	if _stereo.captured.is_empty() and _capture_wait < 10:
		if not _stereo.capture_requested:
			_stereo.capture_requested = true
		return false
	_capture_wait = -1
	return true


func _judge(alt_km: float) -> void:
	var images: Array[Image] = []
	if _stereo != null:
		images = _stereo.captured.duplicate()
		if images.is_empty():
			_fail("%.0f km: no eye image (%s)" % [alt_km, _stereo.capture_error])
			return
	else:
		images = [root.get_viewport().get_texture().get_image()]
	for view in range(images.size()):
		var suffix := "_eye%d" % view if _stereo != null else ""
		_judge_image(images[view], alt_km, "%s/descent_%04.0fkm%s.png" % [_out, alt_km, suffix])


func _judge_image(image: Image, alt_km: float, path: String) -> void:
	if image.is_compressed() or image.get_format() == Image.FORMAT_RGBAH:
		image.convert(Image.FORMAT_RGBA8)  # (an HDR read-back shows its values clipped)
	image.save_png(path)
	image.convert(Image.FORMAT_RGBF)
	var w := image.get_width()
	var h := image.get_height()
	var holes := 0
	var magenta := 0
	var ground := 0
	var sum := Vector3.ZERO
	# The central box (20 % of width and height, <= ~20 deg across even in the headset's wide view) is lunar ground in
	# every placement: the view centre is --below (20) deg under the horizon, or the nadir (the Moon's disc >= 39 deg in
	# radius below 1000 km).
	for y in range(int(h * 0.4), int(h * 0.6), 2):
		for x in range(int(w * 0.4), int(w * 0.6), 2):
			var c := image.get_pixel(x, y)
			ground += 1
			sum += Vector3(c.r, c.g, c.b)
			if c.r < HOLE_LEVEL and c.g < HOLE_LEVEL and c.b < HOLE_LEVEL:
				holes += 1
			if c.r > 0.6 and c.b > 0.6 and c.g < 0.25:
				magenta += 1
	var mean := sum / maxf(ground, 1)
	var hole_share := float(holes) / maxf(ground, 1)
	var a := _world.atmosphere
	print("[moon_descent_check] %.0f km: adopted white %s (B/R %.4f), D %.4f" % [alt_km, a.field_illuminant_rgb,
			a.field_illuminant_rgb.z / maxf(a.field_illuminant_rgb.x, 1e-9), a.white_degree])
	print("[moon_descent_check] %.0f km: %d frames, ground mean sRGB (%.3f, %.3f, %.3f), black share %.4f, magenta %d, gain %.3f, saved %s"
			% [alt_km, _step_frames, mean.x, mean.y, mean.z, hole_share, magenta,
			_world.atmosphere.exposure_gain, path])
	if hole_share > HOLE_SHARE_MAX:
		_fail("%.0f km: %.1f %% of the ground is black (> %.0f %%)" % [alt_km, hole_share * 100, HOLE_SHARE_MAX * 100])
	if magenta > 0:
		_fail("%.0f km: %d magenta pixels" % [alt_km, magenta])
	if mean.x < 0.05 and mean.y < 0.05 and mean.z < 0.05:
		_fail("%.0f km: the sunlit ground renders black" % alt_km)


func _finalize() -> void:
	if _stereo != null:  # the XR server must not keep the stand-in while the engine shuts down
		XRServer.primary_interface = null
		_stereo.uninitialize()
		XRServer.remove_interface(_stereo)


func _fail(what: String) -> void:
	_failures += 1
	print("[moon_descent_check] FAIL: ", what)
