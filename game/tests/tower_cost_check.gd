# Cost of the Pad B tower (LatticeBuilder): build time of TowerView, and per view the frame cost with and without the
# tower (the node hidden): GPU render time, CPU render time, main-thread process time (median over FRAMES frames,
# REPEATS repeats for the noise), draw calls and triangles the tower adds. Needs a GPU (no --headless) and
# data/models/starbase_olp2.
# godot --path game --script res://tests/tower_cost_check.gd -- [--scale=1,2] [--frames=120] [--repeats=3]
# Reads, does not judge; the numbers go into docs/subsystems/launch-structures.md.
extends SceneTree

const SITE_LAT_DEG := 25.99686
const SITE_LON_DEG := -97.15394
const SITE_HEIGHT_M := -21.11
const CAMERA_AZIMUTH_DEG := 135.0
const TOWER_AZIMUTH_DEG := 315.0
const EYE_HEIGHT_M := 15.0
const START_UTC := "2026-10-12T14:10:00"
const FOV_DEG := 60.0
## [name, distance from the tower (m)]: frame-filling close-up, mid range, the far tier.
const VIEWS := [["close-up", 60.0], ["300 m", 300.0], ["1 km", 1000.0], ["6 km (far tier)", 6000.0]]
const FRAMES_TO_SETTLE := 5
const TERRAIN_MAX_FRAMES := 600
const BUILD_REPEATS := 5

var _world: World
var _frames := 120
var _repeats := 3


func _initialize() -> void:
	var args := {}
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_frames = int(args.get("frames", "120"))
	_repeats = int(args.get("repeats", "3"))
	var scales: Array[float] = []
	for text: String in String(args.get("scale", "1,2")).split(","):
		scales.append(float(text))
	root.add_child(load("res://main.tscn").instantiate())
	_run(scales)


func _run(scales: Array[float]) -> void:
	await process_frame
	var main := root.get_child(root.get_child_count() - 1)
	_world = main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	TimePanel.show_frame_rate = false
	_world.clock.set_time(_world.core.utc_to_tdb_s(START_UTC))
	_world.atmosphere.eye_adaptation_enabled = false
	_world.set_focus(World.FOCUS_EARTH)
	var tower := _world.towers[0] if not _world.towers.is_empty() else null
	if tower == null or tower.get_node_or_null("Lattice") == null:
		print("tower_cost_check: no Pad B lattice (models missing?)")
		quit(1)
		return
	RenderingServer.viewport_set_measure_render_time(root.get_viewport_rid(), true)

	var builds: Array[float] = []
	for i in range(BUILD_REPEATS):
		var start := Time.get_ticks_usec()
		tower.build(tower._geometry)
		builds.append(0.001 * (Time.get_ticks_usec() - start))
	builds.sort()
	print("build: TowerView.build (models loaded, MultiMeshes + merged meshes): median %.1f ms, min %.1f, max %.1f (%d runs)"
			% [builds[builds.size() / 2], builds[0], builds[-1], BUILD_REPEATS])
	var lattice_children := 0
	for section in tower.get_node("Lattice").get_children():
		lattice_children += section.get_child_count()
	print("lattice: %d sections, %d tier nodes" % [tower.get_node("Lattice").get_child_count(), lattice_children])

	for scale in scales:
		root.scaling_3d_scale = scale
		print("=== render scale %.1f per axis (%.1fx pixels), %s ===" % [scale, scale * scale, root.size])
		for view: Array in VIEWS:
			await _place(tower, view[1])
			var rows := []
			for repeat in range(_repeats):
				var shown := await _measure(tower, true)
				var hidden := await _measure(tower, false)
				rows.append([shown, hidden])
			print("view %s (%.0f m):" % [view[0], view[1]])
			for r in range(rows.size()):
				var s: Dictionary = rows[r][0]
				var h: Dictionary = rows[r][1]
				print("  repeat %d: GPU %.3f / %.3f ms (tower / hidden), CPU render %.3f / %.3f ms, main process %.3f / %.3f ms"
						% [r + 1, s["gpu"], h["gpu"], s["cpu"], h["cpu"], s["process"], h["process"]])
			var s0: Dictionary = rows[0][0]
			var h0: Dictionary = rows[0][1]
			print("  tower adds %d draw calls, %d primitives" % [s0["calls"] - h0["calls"], s0["primitives"] - h0["primitives"]])
	root.scaling_3d_scale = 1.0
	quit(0)


## Median GPU / CPU render time (ms), main-thread process time (ms) and the draw calls / primitives of the last frame.
func _measure(tower: Node3D, shown: bool) -> Dictionary:
	tower.visible = shown
	await _frames_n(FRAMES_TO_SETTLE)
	var gpu: Array[float] = []
	var cpu: Array[float] = []
	var process: Array[float] = []
	var vp := root.get_viewport_rid()
	for i in range(_frames):
		await process_frame
		gpu.append(RenderingServer.viewport_get_measured_render_time_gpu(vp))
		cpu.append(RenderingServer.viewport_get_measured_render_time_cpu(vp))
		process.append(1000.0 * Performance.get_monitor(Performance.TIME_PROCESS))
	tower.visible = true
	return {"gpu": _median(gpu), "cpu": _median(cpu), "process": _median(process),
			"calls": int(RenderingServer.get_rendering_info(RenderingServer.RENDERING_INFO_TOTAL_DRAW_CALLS_IN_FRAME)),
			"primitives": int(RenderingServer.get_rendering_info(RenderingServer.RENDERING_INFO_TOTAL_PRIMITIVES_IN_FRAME))}


func _median(values: Array[float]) -> float:
	var sorted := values.duplicate()
	sorted.sort()
	return sorted[sorted.size() / 2]


func _frames_n(count: int) -> void:
	for i in range(count):
		await process_frame


## The camera `distance` m from the tower along the pad -> camera azimuth, aimed at the tower's middle.
func _place(_tower: Node3D, distance: float) -> void:
	var azimuth := deg_to_rad(CAMERA_AZIMUTH_DEG)
	var along_m := distance - 30.0  # the tower stands 30 m from the pad centre on the opposite side
	var lat := SITE_LAT_DEG + rad_to_deg(along_m * cos(azimuth) / World.EARTH_SEMI_MAJOR_AXIS_M)
	var lon := SITE_LON_DEG + rad_to_deg(along_m * sin(azimuth)
			/ (World.EARTH_SEMI_MAJOR_AXIS_M * cos(deg_to_rad(SITE_LAT_DEG))))
	var samples: Dictionary = _world.core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var height: float = samples["height_m"][0] if bool(samples["present"]) else SITE_HEIGHT_M
	_world.set_ground_site({"id": "custom", "name": "custom", "body": "earth", "latitude_deg": lat,
			"longitude_deg": lon, "height_m": height, "eye_height_m": EYE_HEIGHT_M})
	_world.update_world()
	_world.camera.fov = FOV_DEG
	_world.camera.azimuth = deg_to_rad(TOWER_AZIMUTH_DEG)
	_world.camera.elevation = atan2(70.0 - EYE_HEIGHT_M, distance)
	_world.atmosphere.snap_adaptation()
	var idle := 0
	var frames := 0
	while idle < FRAMES_TO_SETTLE and frames < TERRAIN_MAX_FRAMES:
		await process_frame
		frames += 1
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		idle = idle + 1 if loading == 0 and not _world.atmosphere.loading else 0
