# Desktop frame time of the cockpit with the left screen's feed on and off (Task 7; the VR budget check follows in Task 9):
# the Crew Dragon mission at T-60 in the cockpit, the root viewport at the VR pixel proxy (--render_scale, default 1.65 as
# docs/subsystems/vr-performance.md), then per state the median / p95 of the frame interval, the root viewport's GPU time and the
# feed viewport's own GPU time. States: off (no feed renders), 30 Hz at 1024 x 640, 30 Hz at 512 x 320, 60 Hz at 1024 x 640.
# Needs a GPU (no --headless) and the generated Dragon models.
# Run: godot --path game --script res://tests/screen_perf.gd -- [--render_scale=1.65] [--frames=300] [--camera=chase]
extends SceneTree

const SETTLE_FRAMES := 150
const STATES := [
	{"name": "off", "hz": 0.0, "size": Vector2i(1024, 640)},
	{"name": "30 Hz 1024x640", "hz": 30.0, "size": Vector2i(1024, 640)},
	{"name": "30 Hz 512x320", "hz": 30.0, "size": Vector2i(512, 320)},
	{"name": "60 Hz 1024x640", "hz": 60.0, "size": Vector2i(1024, 640)},
]

var _args := {}
var _main: Node
var _world: World
var _frames := 0
var _started := false
var _state := -1
var _state_frames := 0
var _frames_per_state := 300
var _intervals: Array[float] = []
var _root_gpu: Array[float] = []
var _feed_gpu: Array[float] = []
var _last_usec := 0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_frames_per_state = int(_args.get("frames", 300))
	root.scaling_3d_mode = Viewport.SCALING_3D_MODE_BILINEAR
	root.scaling_3d_scale = float(_args.get("render_scale", 1.65))
	RenderingServer.viewport_set_measure_render_time(root.get_viewport_rid(), true)
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	_world = _main.get_node_or_null("World")
	if _world == null or not _world.booted:
		return false
	_frames += 1
	if not _started:
		if _frames < 5:
			return false
		_world.weather.enabled = false
		_world.start_mission("falcon9_crew_dragon")
		_started = true
		_frames = 0
		return false
	if not _world.flight.sim.is_active() or not _world.cockpit_active:
		if _frames > 3000:
			print("screen_perf: the cockpit did not start")
			quit(1)
			return true
		return false
	if _state < 0:
		if _frames < SETTLE_FRAMES:
			return false
		RenderingServer.viewport_set_measure_render_time(_world.screen_camera.viewport.get_viewport_rid(), true)
		_world.screen_camera.select(String(_args.get("camera", "chase")))
		_next_state()
		return false
	var now := Time.get_ticks_usec()
	if _state_frames > 20:  # the first frames of a state are its warm-up
		_intervals.append((now - _last_usec) * 0.001)
		_root_gpu.append(RenderingServer.viewport_get_measured_render_time_gpu(root.get_viewport_rid()))
		_feed_gpu.append(RenderingServer.viewport_get_measured_render_time_gpu(_world.screen_camera.viewport.get_viewport_rid()))
	_last_usec = now
	_state_frames += 1
	if _state_frames >= _frames_per_state:
		_report()
		if _state + 1 >= STATES.size():
			quit(0)
			return true
		_next_state()
	return false


func _next_state() -> void:
	_state += 1
	var s: Dictionary = STATES[_state]
	_world.screen_camera.update_hz = maxf(float(s["hz"]), 0.0001)
	_world.screen_camera.size_px = s["size"]
	_world.screen_camera.render_always = float(s["hz"]) > 0.0
	_world.screen_camera.viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	_state_frames = 0
	_intervals.clear()
	_root_gpu.clear()
	_feed_gpu.clear()


func _report() -> void:
	var s: Dictionary = STATES[_state]
	print("screen_perf: %-16s frame p50 %.2f p95 %.2f ms | root GPU p50 %.2f p95 %.2f ms | feed GPU mean of the rendered frames %.2f max %.2f ms | %d renders" % [
			s["name"], _pct(_intervals, 0.5), _pct(_intervals, 0.95), _pct(_root_gpu, 0.5), _pct(_root_gpu, 0.95),
			_mean_nonzero(_feed_gpu), _pct(_feed_gpu, 1.0), _world.screen_camera.render_count])


func _pct(values: Array[float], q: float) -> float:
	if values.is_empty():
		return 0.0
	var sorted := values.duplicate()
	sorted.sort()
	return sorted[mini(int(sorted.size() * q), sorted.size() - 1)]


func _mean_nonzero(values: Array[float]) -> float:
	var sum := 0.0
	var count := 0
	for v in values:
		if v > 0.0:
			sum += v
			count += 1
	return sum / count if count > 0 else 0.0
