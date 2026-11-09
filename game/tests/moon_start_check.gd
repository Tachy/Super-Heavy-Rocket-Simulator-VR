# GPU check (windowed, no --headless), a TOOL not a test: the headless runner never runs it.
# The first frames after an arrival on the Moon's ground (bug 2026-11-09, user in VR: right after the start most of the
# view covered by dark surfaces with dense noisy vertical streaks). The game scene (main.tscn) boots, the free camera
# is put --alt_m over the sunlit lunar ground at once - no waiting for the terrain, which then loads for tens of seconds
# from the coarse roots down - and every frame of the first --frames is judged:
# - eye over the drawn ground: the shown camera at least SurfaceClearance.MIN_CLEARANCE_M - 1 cm above the surface the
#   terrain draws now (TerrainQuadtree.drawn_height_m; the bug: the coarse loading surface lay above the eye, which saw
#   the nodes' skirts and inner sides from below);
# - ground holes: the share of black pixels (all sRGB channels < HOLE_LEVEL) in the image's lower GROUND_ROWS (the view
#   is --below deg under the horizon: lunar ground there) at most HOLE_SHARE_MAX;
# - speckle: pixels whose sRGB luminance stands out from both horizontal neighbours by SPECKLE_STEP the same way, at
#   most SPECKLE_MAX of the image (the bug's noise: the float32 texture coordinate of a coarse node near the eye).
# Run: godot --path game --script res://tests/moon_start_check.gd -- [--out=<dir>] [--frames=240] [--alt_m=2]
#      [--utc=2026-11-09T05:55:00] [--sun_el=20] [--below=8] [--yaws=0,180] (deg from down-Sun; one arrival per yaw,
#      each at its own site SITE_STEP_DEG further round) [--near_shadow=on|off] [--rocks=on|off]
#      [--stereo=res://tests/data/pimax_dream_air_projections.json] [--save=all|bad|none] [--weather=off|analog]
# Exit 0: every frame passes; 1: a failure (listed).
extends SceneTree

## [call, measured 2026-11-09 (report bug-moon-start)] speckle share of a clean ground view: 0.007-0.012; the bug's
## noise 0.022-0.07
const SPECKLE_STEP := 0.18
const SPECKLE_MAX := 0.018
## [call] a ground pixel darker than this (sRGB, all channels) is a hole; the lower rows judged; the share allowed
const HOLE_LEVEL := 0.02
const GROUND_ROWS := 0.3
const HOLE_SHARE_MAX := 0.15
## [call] tolerance of the eye's clearance over the drawn ground (m; the clearance itself: SurfaceClearance)
const CLEARANCE_TOLERANCE_M := 0.01
## [E] each yaw arrives at its own site, this far round the circle where the Sun stands --sun_el high (a fresh tree
## there below level ~3: the quadtree keeps the nodes it built near the earlier site)
const SITE_STEP_DEG := 40.0

var _args := {}
var _main: Node
var _world: World
var _frames := 0
var _judged := 0
var _failures := 0
var _worst := {"speckle": 0.0, "holes": 0.0, "clearance_m": INF}
var _out := "res://../screenshots/look_fixes/bugs-2026-11-09/moon_start"
var _position := Vector3.ZERO
var _basis := Basis.IDENTITY
var _stereo: StereoMockXr
var _yaws: PackedFloat64Array
var _yaw_index := 0
var _phase_frame := 0  ## frames since the current arrival
var _arrived_ms := 0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_out = ProjectSettings.globalize_path(String(_args.get("out", _out)))
	DirAccess.make_dir_recursive_absolute(_out)
	_yaws = String(_args.get("yaws", "0,180")).split_floats(",", false)
	if _args.get("near_shadow", "on") == "off":
		MoonNearShadow.enabled = false
	if _args.get("rocks", "on") == "off":
		MoonRockLayer.geometry_enabled = false
	if _args.has("stereo"):
		_stereo = StereoMockXr.new()
		if not _stereo.load(String(_args["stereo"])):
			push_error("moon_start_check: --stereo cannot be loaded")
			quit(1)
			return
		XRServer.add_interface(_stereo)
		_stereo.initialize()
		XRServer.primary_interface = _stereo
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	_frames += 1
	_world = _main.get_node("World")
	if not _world.booted:
		return false
	_world.clock.warp = 0.0
	if _phase_frame == 0 and _yaw_index == 0 and _arrived_ms == 0:
		_world.weather.enabled = _args.get("weather", "off") != "off"
		_world.clock.set_time(_world.core.utc_to_tdb_s(_args.get("utc", "2026-11-09T05:55:00")))
		_world.set_focus(World.FOCUS_FREE)
		_arrive()
		return false
	if _phase_frame > 0:
		_judge_clearance()  # the shown camera of the frame just drawn (the world placed it after this script's last step)
	if _args.has("terrain_off"):  # diagnostics: --terrain_off=<bool uniform>[,...] set false on every Moon node
		for param in String(_args["terrain_off"]).split(","):
			_world.moon_terrain.set_material_parameter(StringName(param), false)
	_phase_frame += 1
	_hold()
	if _stereo != null:
		if not _stereo.captured.is_empty():
			var images: Array[Image] = _stereo.captured.duplicate()
			_stereo.captured = []
			for view in range(images.size()):
				_judge(images[view], "_eye%d" % view)
		_stereo.capture_requested = true
	else:
		_judge(root.get_viewport().get_texture().get_image(), "")
	if _phase_frame < int(_args.get("frames", "240")):
		return false
	_yaw_index += 1
	if _yaw_index < _yaws.size():
		_arrive()
		return false
	print("[moon_start_check] %s (%d failures in %d images; worst speckle %.4f, ground holes %.4f, clearance over the drawn ground %.3f m)"
			% ["OK" if _failures == 0 else "FAIL", _failures, _judged, _worst["speckle"], _worst["holes"],
			_worst["clearance_m"]])
	quit(1 if _failures > 0 else 0)
	return true


## The free camera --alt_m over the lunar terrain where the Sun stands --sun_el high, looking the current yaw (deg from
## down-Sun), --below deg under the horizon.
func _arrive() -> void:
	_world.update_world()
	var t: float = _world.clock.tdb_s
	var moon: Vector3 = _world.moon_gcrf_m
	var sun_dir: Vector3 = (_world.core.body_position_gcrf_m("sun", t) - moon).normalized()
	var sun_el := deg_to_rad(float(_args.get("sun_el", "20")))
	var side := sun_dir.cross(Vector3(0, 0, 1)).normalized()
	var up := sun_dir.rotated(side, PI / 2.0 - sun_el).rotated(sun_dir, deg_to_rad(SITE_STEP_DEG * _yaw_index)).normalized()
	var fixed: Vector3 = _world.moon.fixed_to_gcrf.transposed() * up
	var lat := rad_to_deg(asin(clampf(fixed.z, -1.0, 1.0)))
	var lon := rad_to_deg(atan2(fixed.y, fixed.x))
	var ground: Dictionary = _world.core.surface_samples("moon", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var ground_m: float = ground["height_m"][0] if bool(ground.get("present", false)) else 0.0
	_position = moon + up * (World.MOON_RADIUS_M + ground_m + float(_args.get("alt_m", "2")))
	var yaw := _yaws[_yaw_index]
	var horizontal := (-sun_dir + up * sun_dir.dot(up)).normalized().rotated(up, deg_to_rad(yaw))
	var pitch := -deg_to_rad(float(_args.get("below", "8")))
	var look := (horizontal * cos(pitch) + up * sin(pitch)).normalized()
	_basis = Basis.looking_at(Frames.to_render(look), Frames.to_render(up))
	_phase_frame = 0
	_arrived_ms = Time.get_ticks_msec()
	print("[moon_start_check] arrival at frame %d: lat %.4f lon %.4f ground %.1f m, alt %s m, yaw %.0f, near shadow %s"
			% [_frames, lat, lon, ground_m, _args.get("alt_m", "2"), yaw, MoonNearShadow.enabled])
	_hold()


func _hold() -> void:
	_world.free_pose.reset_to(_position, _basis, _world.earth.fixed_to_gcrf, _world.moon.fixed_to_gcrf, _world.moon_gcrf_m)
	_world.camera.position_gcrf_m = _position
	_world.camera.free_basis = _basis


## Speckle share of an image: pixels standing out from both horizontal neighbours by SPECKLE_STEP the same way.
static func speckle_share(image: Image) -> float:
	var w := image.get_width()
	var h := image.get_height()
	var count := 0
	var total := 0
	for y in range(0, h, 2):
		var a := _luma(image.get_pixel(0, y))
		var b := _luma(image.get_pixel(1, y))
		for x in range(2, w):
			var c := _luma(image.get_pixel(x, y))
			var d1 := b - a
			var d2 := b - c
			if (d1 > SPECKLE_STEP and d2 > SPECKLE_STEP) or (d1 < -SPECKLE_STEP and d2 < -SPECKLE_STEP):
				count += 1
			total += 1
			a = b
			b = c
	return float(count) / maxf(total, 1)


## Share of black pixels (all channels < HOLE_LEVEL) in the image's lower GROUND_ROWS.
static func hole_share(image: Image) -> float:
	var w := image.get_width()
	var h := image.get_height()
	var holes := 0
	var total := 0
	for y in range(int(h * (1.0 - GROUND_ROWS)), h, 2):
		for x in range(0, w, 2):
			var c := image.get_pixel(x, y)
			if c.r < HOLE_LEVEL and c.g < HOLE_LEVEL and c.b < HOLE_LEVEL:
				holes += 1
			total += 1
	return float(holes) / maxf(total, 1)


static func _luma(c: Color) -> float:
	return 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b


func _judge(image: Image, suffix: String) -> void:
	if image.is_compressed() or image.get_format() == Image.FORMAT_RGBAH:
		image.convert(Image.FORMAT_RGBA8)
	var small := image.duplicate() as Image
	if small.get_width() > 640:  # the metrics on <= 640 px wide, nearest (fast; a dot stays a dot)
		small.resize(640, int(640.0 * small.get_height() / small.get_width()), Image.INTERPOLATE_NEAREST)
	small.convert(Image.FORMAT_RGBF)
	var speckle := speckle_share(small)
	var holes := hole_share(small)
	_judged += 1
	_worst["speckle"] = maxf(_worst["speckle"], speckle)
	_worst["holes"] = maxf(_worst["holes"], holes)
	var bad := speckle > SPECKLE_MAX or holes > HOLE_SHARE_MAX
	var save: String = _args.get("save", "bad")
	var path := ""
	var n := _phase_frame
	if save == "all" or (save == "bad" and bad) or (save != "none" and (n <= 3 or n % 30 == 0)):
		path = "%s/yaw%03.0f_frame_%03d%s.png" % [_out, _yaws[_yaw_index], n, suffix]
		image.save_png(path)
	var stats: Dictionary = _world.moon_terrain.stats
	print("[moon_start_check] yaw %.0f %.1f s frame %d%s: speckle %.4f, holes %.4f, terrain loading %d deepest %d, near shadow %s %s"
			% [_yaws[_yaw_index], (Time.get_ticks_msec() - _arrived_ms) * 1.0e-3, n, suffix, speckle, holes,
			int(stats.get("nodes_loading", 0)), int(stats.get("max_drawn_level", 0)), _world.moon_near_shadow.active, path])
	if speckle > SPECKLE_MAX:
		_fail("yaw %.0f frame %d%s: speckle share %.4f > %.4f" % [_yaws[_yaw_index], n, suffix, speckle, SPECKLE_MAX])
	if holes > HOLE_SHARE_MAX:
		_fail("yaw %.0f frame %d%s: %.1f %% of the ground rows black (> %.0f %%)" % [_yaws[_yaw_index], n, suffix,
				holes * 100.0, HOLE_SHARE_MAX * 100.0])


## The shown camera over the surface the terrain draws (both as the world left them in the last frame: this script's
## step runs before the scene's nodes, its _hold re-places the unlifted pose).
func _judge_clearance() -> void:
	var p: Vector3 = _world.camera.position_gcrf_m - _world.moon_gcrf_m
	var fixed: Vector3 = _world.moon.fixed_to_gcrf.transposed() * p
	var geo := SurfaceClearance.geodetic(fixed, false)
	if not _world.moon_terrain.has_method(&"drawn_height_m"):  # a tree before the fix (A runs)
		return
	var drawn: float = _world.moon_terrain.call(&"drawn_height_m", CubeSphere.geodetic_direction(geo.x, geo.y))
	if is_nan(drawn):
		return
	var clearance := geo.z - drawn
	_worst["clearance_m"] = minf(_worst["clearance_m"], clearance)
	if clearance < SurfaceClearance.MIN_CLEARANCE_M - CLEARANCE_TOLERANCE_M:
		_fail("yaw %.0f frame %d: the eye %.2f m over the drawn ground (< %.2f m; drawn %.2f m, eye %.2f m, lift %.2f m)"
				% [_yaws[_yaw_index], _phase_frame, clearance, SurfaceClearance.MIN_CLEARANCE_M, drawn, geo.z,
				_world.get("_drawn_lift_m")])


func _fail(what: String) -> void:
	_failures += 1
	print("[moon_start_check] FAIL: ", what)


func _finalize() -> void:
	if _stereo != null:
		XRServer.primary_interface = null
		_stereo.uninitialize()
		XRServer.remove_interface(_stereo)
