# Cost of the detailed vehicle models on the pad (Starship prototype plan, task 3 + final fix): per view and render scale the
# frame cost (GPU render time, CPU render time: median over FRAMES frames, REPEATS repeats with the variants interleaved
# because single runs misled) of
#   hull    - the models with the hull shader (the shipped look),
#   steel   - the SAME meshes with the old LitAerial steel (isolates the shader),
#   cyl     - the old cylinders (the vehicle without models),
#   hidden  - the vehicle's view hidden (what the stack hides: sky / terrain behind it).
# Views: frame-filling close-up of the ship's belly (the worst case for a pixel shader), the stack at 100 m and 400 m.
# Scales 1 and 2 per axis (2 is the engine maximum: 3200 x 1800), the terrain settled first (nothing loading for TERRAIN_IDLE_FRAMES frames).
# Needs a GPU (no --headless). Reads, does not judge; the numbers go into docs/subsystems/vehicle-models.md.
# godot --path game --script res://tests/vehicle_cost_check.gd -- [--scale=1,2] [--frames=120] [--repeats=3]
extends SceneTree

## [name, station aimed at (m), camera distance from the axis (m)]
const VIEWS := [["close-up (10 m from the hull)", 95.0, 14.5], ["stack 100 m", 62.0, 100.0], ["stack 400 m", 62.0, 400.0]]
const VARIANTS := ["hull", "steel", "cyl", "hidden"]
const FOV_DEG := 33.4
const FRAMES_TO_SETTLE := 8
const TERRAIN_IDLE_FRAMES := 30
const TERRAIN_MAX_FRAMES := 1800
## One eye of the Pimax Dream Air render target (CLAUDE.md memory: 5294 x 4250 px per eye, two eyes).
const VR_MEGAPIXELS := 2.0 * 5294.0 * 4250.0 / 1.0e6

var _world: World
var _rig: VehicleRig
var _sim: FlightSim
var _frames := 120
var _repeats := 3
var _args := {}
var _main: Node
var _started := false
var _loop_started := false
var _terrain_idle := 0
var _original_materials := {}  ## MeshInstance3D -> its hull material, while the "steel" variant is shown


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
		scales.append(minf(float(text), 2.0))  # Viewport.scaling_3d_scale tops out at 2
	await _n(10)  # the flight session creates its views on the next frames
	var view := _world.flight.views[0]
	await _settle_terrain()
	for scale in scales:
		root.scaling_3d_scale = scale
		var megapixels := root.size.x * scale * root.size.y * scale / 1.0e6
		print("=== render scale %.1f per axis (%.1f MP), %s ===" % [scale, megapixels, root.size])
		for spec: Array in VIEWS:
			_rig.place(_sim, 0, spec[1], Vector3(0.0, 0.0, -spec[2]), FOV_DEG)
			await _n(FRAMES_TO_SETTLE)
			var rows := {}
			for variant: String in VARIANTS:
				rows[variant] = []
			for repeat in _repeats:
				for variant: String in VARIANTS:
					await _show(view, variant)
					rows[variant].append(await _measure(view))
					await _restore(view, variant)
			print("view %s:" % spec[0])
			for variant: String in VARIANTS:
				var gpu: Array[float] = []
				var cpu: Array[float] = []
				for r: Dictionary in rows[variant]:
					gpu.append(r["gpu"])
					cpu.append(r["cpu"])
				print("  %-6s GPU %s ms (median %.3f), CPU render median %.3f ms, %d calls, %d primitives" % [variant,
						" / ".join(gpu.map(func(g: float) -> String: return "%.3f" % g)), _median(gpu), _median(cpu),
						rows[variant][0]["calls"], rows[variant][0]["primitives"]])
			var hull := _median(rows["hull"].map(func(r: Dictionary) -> float: return r["gpu"]))
			var steel := _median(rows["steel"].map(func(r: Dictionary) -> float: return r["gpu"]))
			var cyl := _median(rows["cyl"].map(func(r: Dictionary) -> float: return r["gpu"]))
			var hidden := _median(rows["hidden"].map(func(r: Dictionary) -> float: return r["gpu"]))
			print("  deltas (ms): hull shader vs old steel on the same meshes %+.3f (%+.3f ms/MP); models vs cylinders %+.3f;"
					% [hull - steel, (hull - steel) / megapixels, hull - cyl] + " models vs hidden %+.3f" % (hull - hidden))
	root.scaling_3d_scale = 1.0
	print("VR extrapolation (NOT measured): the hull-shader delta per MP of the frame-filling close-up x %.1f MP of both eyes"
			% VR_MEGAPIXELS)
	quit(0)


func _settle_terrain() -> void:
	var frames := 0
	while _terrain_idle < TERRAIN_IDLE_FRAMES and frames < TERRAIN_MAX_FRAMES:
		await process_frame
		frames += 1
		var loading: int = _world.earth_terrain.stats["nodes_loading"] + _world.moon_terrain.stats["nodes_loading"]
		_terrain_idle = _terrain_idle + 1 if loading == 0 and not _world.atmosphere.loading else 0
	print("terrain settled after %d frames (idle %d)" % [frames, _terrain_idle])


## Puts the view into a variant; _restore undoes it.
func _show(view: VehicleView, variant: String) -> void:
	match variant:
		"steel":
			_original_materials.clear()
			var hull_materials: Array = view._models["materials"].values()
			for node: MeshInstance3D in view._stage_root.find_children("*", "MeshInstance3D", true, false):
				if node.material_override in hull_materials:
					_original_materials[node] = node.material_override
					node.material_override = view._steel
		"cyl":
			VehicleModels._cache[VehicleModels.PROTOTYPE_VEHICLE_ID] = {}
			view._built_signature = ""  # the view's next update rebuilds without models
		"hidden":
			view._stage_root.visible = false
	await _n(FRAMES_TO_SETTLE)


func _restore(view: VehicleView, variant: String) -> void:
	match variant:
		"steel":
			for node: MeshInstance3D in _original_materials:
				node.material_override = _original_materials[node]
			_original_materials.clear()
		"cyl":
			VehicleModels.clear_cache()  # reloads the models (the cached materials are new objects: nothing else holds them)
			view._built_signature = ""
		"hidden":
			view._stage_root.visible = true
	await _n(FRAMES_TO_SETTLE)


func _measure(_view: VehicleView) -> Dictionary:
	var gpu: Array[float] = []
	var cpu: Array[float] = []
	var vp := root.get_viewport_rid()
	for i in _frames:
		await process_frame
		gpu.append(RenderingServer.viewport_get_measured_render_time_gpu(vp))
		cpu.append(RenderingServer.viewport_get_measured_render_time_cpu(vp))
	return {"gpu": _median(gpu), "cpu": _median(cpu),
			"calls": int(RenderingServer.get_rendering_info(RenderingServer.RENDERING_INFO_TOTAL_DRAW_CALLS_IN_FRAME)),
			"primitives": int(RenderingServer.get_rendering_info(RenderingServer.RENDERING_INFO_TOTAL_PRIMITIVES_IN_FRAME))}


func _median(values: Array) -> float:
	var sorted := values.duplicate()
	sorted.sort()
	return sorted[sorted.size() / 2]


func _n(count: int) -> void:
	for i in count:
		await process_frame
