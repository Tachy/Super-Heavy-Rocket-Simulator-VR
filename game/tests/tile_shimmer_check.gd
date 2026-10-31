# Heat-shield tile shimmer check (Starship prototype plan, task 3): the ship's tiled belly in a REAL flight session (ship
# return, entry state, in the vacuum of 60 km: no aerial perspective between camera and hull) seen from 5 to 500 m (distance
# to the surface) with
# the VR-like focal length (1500 px: FOV 33.4 deg on the 900 px window; --render_scale scales the pixels), exposure fixed.
# Per distance:
#  - mean luminance L of a 1 m x 1 m patch of the belly (linearised display values; the patch is physical, so it holds
#    many tiles near and a few pixels far), and the ratio max / min of L over all distances (limit RATIO_LIMIT: the tile
#    detail must fade to its mean colour, not brighten or darken with distance);
#  - flicker: sigma / mean of L over STEPS frames of a 0.3 px head motion (the pattern slides over the pixel grid): the
#    shimmer a moving head would see;
#  - HF: mean |dL| of neighbouring pixels in the patch / L (pattern contrast per pixel; it must fall with distance).
# godot --path game --script res://tests/tile_shimmer_check.gd -- [--utc=2026-10-31T13:10:00] [--render_scale=1.0]
# Exit code 1 when the ratio is above the limit.
extends SceneTree

const DISTANCES_M := [5.0, 10.0, 20.0, 40.0, 80.0, 160.0, 320.0, 500.0]
const FOV_DEG := 33.4
const FOCAL_PX := 1500.0
const PATCH_M := 1.0
const STEPS := 12
const STEP_RAD := 0.3 / FOCAL_PX  ## 0.3 px of head motion per frame at any distance
const FRAMES_SETTLE := 6
const RATIO_LIMIT := 1.10
const STATION_M := 20.0
## The camera aims at the axis; the hull is its radius closer (the ship's barrel, config diameter 9 m).
const HULL_RADIUS_M := 4.5
const MISSION_ARGS := {"mission": "starship_leo", "ship_return": "560", "t_plus": "5200", "utc": "2026-10-31T13:10:00"}

var _args := {}
var _main: Node
var _world: World
var _rig: VehicleRig
var _sim: FlightSim
var _frames := 0
var _ready_to_run := false
var _distance_index := 0
var _step := -1
var _wait := 0
var _lum: Array[float] = []
var _hf: Array[float] = []
var _results: Array[Dictionary] = []


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	if _args.has("render_scale"):
		root.scaling_3d_mode = Viewport.SCALING_3D_MODE_BILINEAR
		root.scaling_3d_scale = float(_args["render_scale"])


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	_frames += 1
	if not _ready_to_run:
		if _frames < 10 or _world.atmosphere.loading:
			return false
		TimePanel.show_frame_rate = false
		_world.atmosphere.eye_adaptation_enabled = false
		var rig_args := MISSION_ARGS.duplicate()
		rig_args.merge(_args, true)
		_rig = VehicleRig.new(_world, rig_args)
		_sim = _rig.start()
		_rig.fly_to_target(_sim)
		_rig.hide_ui(_main)
		_ready_to_run = true
		return false
	return _measure()


func _measure() -> bool:
	if _distance_index >= DISTANCES_M.size():
		return _report()
	var distance: float = DISTANCES_M[_distance_index]
	if _step < 0:  # a new distance: place, let the frame settle
		_rig.place(_sim, 1, STATION_M, Vector3(0.0, 0.0, -(distance + HULL_RADIUS_M)), FOV_DEG)
		_wait += 1
		if _wait >= FRAMES_SETTLE:
			_step = 0
			_wait = 0
			_lum.clear()
			_hf.clear()
		return false
	_rig.place(_sim, 1, STATION_M, Vector3(0.0, 0.0, -(distance + HULL_RADIUS_M)), FOV_DEG, NAN, _step * STEP_RAD)
	_wait += 1
	if _wait < 3:
		return false
	_wait = 0
	var image := root.get_viewport().get_texture().get_image()
	var window := _patch_stats(image, distance)
	_lum.append(window[0])
	_hf.append(window[1])
	_step += 1
	if _step >= STEPS:
		var mean := _mean(_lum)
		var variance := 0.0
		for value in _lum:
			variance += (value - mean) * (value - mean)
		_results.append({"distance_m": distance, "luminance": mean, "flicker": sqrt(variance / _lum.size()) / maxf(mean, 1e-9),
				"hf": _mean(_hf), "patch_px": window[2]})
		if _distance_index == 0:
			image.save_png(ProjectSettings.globalize_path("res://").path_join("../screenshots/vehicles/shimmer_5m.png"))
		_distance_index += 1
		_step = -1
	return false


## [mean luminance, mean neighbour difference / luminance, patch size px] of the central physical patch.
func _patch_stats(image: Image, distance_m: float) -> Array:
	var half := int(clampf(0.5 * PATCH_M * FOCAL_PX / distance_m * root.scaling_3d_scale, 1.5, 300.0))
	var cx := image.get_width() / 2
	var cy := image.get_height() / 2
	var total := 0.0
	var diff := 0.0
	var count := 0
	for y in range(cy - half, cy + half):
		var previous := -1.0
		for x in range(cx - half, cx + half):
			var l := _linear_luminance(image.get_pixel(x, y))
			total += l
			count += 1
			if previous >= 0.0:
				diff += absf(l - previous)
			previous = l
	var mean := total / maxi(count, 1)
	return [mean, diff / maxi(count, 1) / maxf(mean, 1e-9), 2 * half]


static func _linear_luminance(c: Color) -> float:
	var l := c.srgb_to_linear()
	return 0.2126 * l.r + 0.7152 * l.g + 0.0722 * l.b


static func _mean(values: Array[float]) -> float:
	var sum := 0.0
	for v in values:
		sum += v
	return sum / maxi(values.size(), 1)


func _report() -> bool:
	var lo := INF
	var hi := 0.0
	var worst_flicker := 0.0
	for r in _results:
		print("tile_shimmer_check: %5.0f m  patch %3d px  L %.5f  flicker %.4f  HF %.4f" % [r["distance_m"], r["patch_px"],
				r["luminance"], r["flicker"], r["hf"]])
		lo = minf(lo, r["luminance"])
		hi = maxf(hi, r["luminance"])
		worst_flicker = maxf(worst_flicker, r["flicker"])
	var ratio := hi / maxf(lo, 1e-9)
	print("tile_shimmer_check: luminance ratio max/min %.3f (limit %.2f), worst flicker %.4f -> %s" % [ratio, RATIO_LIMIT,
			worst_flicker, "PASS" if ratio <= RATIO_LIMIT else "FAIL"])
	quit(0 if ratio <= RATIO_LIMIT else 1)
	return true
