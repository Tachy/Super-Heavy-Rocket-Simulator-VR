# Cost of the detailed vehicle models on the pad (Starship prototype plan, task 3): per view the frame cost with the stack's
# view shown and hidden (GPU render time, CPU render time, main-thread process time: median over FRAMES frames, REPEATS
# repeats because single runs misled), draw calls and primitives the stack adds. Needs a GPU (no --headless).
# godot --path game --script res://tests/vehicle_cost_check.gd -- [--scale=1,2] [--frames=120] [--repeats=3]
# Reads, does not judge; the numbers go into docs/subsystems/vehicle-models.md.
extends SceneTree

## [name, station aimed at (m), camera distance from the axis (m)]: frame-filling close-up of the ship's belly, the stack
## at the tower, far.
const VIEWS := [["close-up (10 m from the hull)", 95.0, 14.5], ["stack 100 m", 62.0, 100.0], ["stack 400 m", 62.0, 400.0]]
const FOV_DEG := 33.4
const FRAMES_TO_SETTLE := 8

var _world: World
var _rig: VehicleRig
var _sim: FlightSim
var _frames := 120
var _repeats := 3
var _args := {}
var _main: Node
var _started := false
var _loop_started := false


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_frames = int(_args.get("frames", "120"))
	_repeats = int(_args.get("repeats", "3"))
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	if not _started and Engine.get_process_frames() > 10 and not _world.atmosphere.loading:
		_started = true
		TimePanel.show_frame_rate = false
		_world.atmosphere.eye_adaptation_enabled = false
		_rig = VehicleRig.new(_world, {"mission": "starship_leo", "t_plus": "-30", "utc": "2026-10-31T21:00:00"})
		_sim = _rig.start()
		_rig.fly_to_target(_sim)
		_rig.hide_ui(_main)
		RenderingServer.viewport_set_measure_render_time(root.get_viewport_rid(), true)
		if not _loop_started:
			_loop_started = true
			_run()
	return false


func _run() -> void:
	var scales: Array[float] = []
	for text: String in String(_args.get("scale", "1,2")).split(","):
		scales.append(float(text))
	await _n(10)  # the flight session creates its views on the next frames
	var view := _world.flight.views[0]
	for scale in scales:
		root.scaling_3d_scale = scale
		print("=== render scale %.1f per axis (%.1fx pixels), %s ===" % [scale, scale * scale, root.size])
		for spec: Array in VIEWS:
			_rig.place(_sim, 0, spec[1], Vector3(0.0, 0.0, -spec[2]), FOV_DEG)
			await _n(FRAMES_TO_SETTLE)
			var rows := []
			for repeat in _repeats:
				rows.append([await _measure(view, true), await _measure(view, false)])
			print("view %s:" % spec[0])
			for r in rows.size():
				var s: Dictionary = rows[r][0]
				var h: Dictionary = rows[r][1]
				print("  repeat %d: GPU %.3f / %.3f ms (shown / hidden), CPU render %.3f / %.3f ms, main process %.3f / %.3f ms"
						% [r + 1, s["gpu"], h["gpu"], s["cpu"], h["cpu"], s["process"], h["process"]])
			print("  stack adds %d draw calls, %d primitives" % [rows[0][0]["calls"] - rows[0][1]["calls"],
					rows[0][0]["primitives"] - rows[0][1]["primitives"]])
	root.scaling_3d_scale = 1.0
	quit(0)


func _measure(view: VehicleView, shown: bool) -> Dictionary:
	view._stage_root.visible = shown
	await _n(FRAMES_TO_SETTLE)
	var gpu: Array[float] = []
	var cpu: Array[float] = []
	var process: Array[float] = []
	var vp := root.get_viewport_rid()
	for i in _frames:
		await process_frame
		gpu.append(RenderingServer.viewport_get_measured_render_time_gpu(vp))
		cpu.append(RenderingServer.viewport_get_measured_render_time_cpu(vp))
		process.append(1000.0 * Performance.get_monitor(Performance.TIME_PROCESS))
	view._stage_root.visible = true
	return {"gpu": _median(gpu), "cpu": _median(cpu), "process": _median(process),
			"calls": int(RenderingServer.get_rendering_info(RenderingServer.RENDERING_INFO_TOTAL_DRAW_CALLS_IN_FRAME)),
			"primitives": int(RenderingServer.get_rendering_info(RenderingServer.RENDERING_INFO_TOTAL_PRIMITIVES_IN_FRAME))}


func _median(values: Array[float]) -> float:
	var sorted := values.duplicate()
	sorted.sort()
	return sorted[sorted.size() / 2]


func _n(count: int) -> void:
	for i in count:
		await process_frame
