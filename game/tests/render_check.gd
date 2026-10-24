# Renders the main scene at a fixed instant and camera, saves a PNG and quits (needs a GPU, no --headless).
# godot --path game --script res://tests/render_check.gd -- --utc=2024-04-08T18:42:00 --focus=earth
# Vehicle: --vehicle=starship_block3 --site=starbase_olp1 [--fly=40] (ignite, liftoff at 3 s, fly N s)
#          [--autopilot] (ascent autopilot instead of manual) [--circularize] (plan node afterwards)
# Sea: --sea_lat=32.5 --sea_lon=-61 [--eye_height=10] ground view at sea, the eye above the geoid (with --azimuth etc.)
# Catch tower: [--tower_arms_deg=0] draws the arms at a fixed opening (0 = closed).
# Webcast overlay: --webcast [--mission=starship_leo] (start the mission at --utc) [--t_plus=75] (advance to T+75 s)
#          or [--at_event=entry_interface,<offset s>] (to an event of the mission timeline, recorded or predicted)
#          [--wait_on_pad] (a launch window's wait on the pad: T-) [--view=1] (Tab presses: switch the viewed object)
#          [--legacy_hud] (F1: the old HUD too)
#     [--yaw=0.6] [--pitch=-0.3] [--distance=2.2e7] [--azimuth=90] [--elevation=5] [--fov=50] --out=C:/tmp/x.png
extends SceneTree

const FRAMES_TO_SETTLE := 6
## Upper bound on the wait for the streamed terrain (frames).
const TERRAIN_MAX_FRAMES := 6000
## The flight session's fixed tick (200 Hz, sim::FlightSession): --t_plus aims a tick late (the grid need not hit T-0).
const SIM_TICK_S := 0.005

var _args := {}
var _main: Node
var _frames := 0
var _terrain_idle := 0  ## consecutive frames without terrain loading


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	var world: World = _main.get_node("World")
	world.clock.warp = 0.0
	if _frames == 0:
		world.clock.set_time(world.core.utc_to_tdb_s(_args.get("utc", "2024-04-08T18:42:00")))
		world.set_focus(_args.get("focus", World.FOCUS_EARTH))
		if _args.has("lat"):
			world.set_ground_site({"id": "custom", "name": "custom", "latitude_deg": float(_args["lat"]),
					"longitude_deg": float(_args["lon"]), "height_m": float(_args.get("height", "0"))})
		if _args.has("sea_lat"):
			_view_at_sea(world)
		var cam := world.camera
		if _args.has("yaw"): cam.yaw = float(_args["yaw"])
		if _args.has("pitch"): cam.pitch = float(_args["pitch"])
		if _args.has("distance"): cam.distance_m = float(_args["distance"])
		if _args.has("azimuth"): cam.azimuth = deg_to_rad(float(_args["azimuth"]))
		if _args.has("elevation"): cam.elevation = deg_to_rad(float(_args["elevation"]))
		if _args.has("fov"): cam.fov = float(_args["fov"])
		if _args.has("look_at_moon"):
			_aim_at_moon(world)
		if _args.has("view_from"):
			_view_from(world, _args["view_from"])
		if _args.has("vehicle"):
			_fly_vehicle(world)
		if _args.has("webcast"):
			_fly_mission(world)
			cam = world.camera  # the mission's focus switch resets the camera: the requested view again
			if _args.has("yaw"): cam.yaw = float(_args["yaw"])
			if _args.has("pitch"): cam.pitch = float(_args["pitch"])
			if _args.has("distance"): cam.distance_m = float(_args["distance"])
		if _args.has("tower_arms_deg"):
			for view in world.towers:
				view.preview_opening_rad = deg_to_rad(float(_args["tower_arms_deg"]))
				view.update_state({})
	_frames += 1
	# The streamed terrain loads over many frames: wait until nothing is loading (or give up after TERRAIN_MAX_FRAMES).
	var terrain := world.earth_terrain
	_terrain_idle = _terrain_idle + 1 if terrain.stats["nodes_loading"] == 0 else 0
	if _frames >= FRAMES_TO_SETTLE and (_terrain_idle >= FRAMES_TO_SETTLE or _frames >= TERRAIN_MAX_FRAMES):
		print("render_check: terrain after %d frames: %s" % [_frames, terrain.stats])
		if _args.has("webcast"):
			var arc := (_main.get("overlay_host") as OverlayHost).overlay.timeline_arc
			print("render_check: timeline scale ", arc.time_scale, " now ", arc.now_tdb_s, " events ", arc.events.size())
		var out: String = _args.get("out", "user://render_check.png")
		var error := root.get_viewport().get_texture().get_image().save_png(out)
		print("render_check: saved %s (error %d)" % [out, error])
		return true  # quit
	return false


# Orbit camera: look at the focus from the direction of the Sun or the Moon.
func _view_from(world: World, body: String) -> void:
	world.update_world()
	var direction := world.sun_apparent_from_earth_m if body == "sun" else world.moon_apparent_from_earth_m
	var u := Frames.to_render(direction).normalized()
	world.camera.pitch = -asin(u.y)
	world.camera.yaw = atan2(u.x, u.z)


# Launch a vehicle at the given instant, fly it with neutral controls (rate command holds attitude).
func _fly_vehicle(world: World) -> void:
	var site: Dictionary = world.sites[0]
	for candidate: Dictionary in world.sites:
		if candidate["id"] == _args.get("site", ""):
			site = candidate
	var t0 := world.clock.tdb_s
	world.flight.manual_control = true  # the script flies by its own commands, not the external view's autopilot
	world.launch_vehicle(_args["vehicle"], site)
	world.clock.warp = 0.0
	var sim := world.flight.sim
	sim.wait_idle(60.0)  # the launch runs on the simulation thread
	var fly := float(_args.get("fly", "0"))
	if fly > 0.0:
		if _args.has("autopilot"):
			sim.engage_ascent_autopilot(200.0, 200.0, 26.5, true)
		else:
			sim.ignite()
			_advance(sim, t0 + 3.0)
			sim.release_clamps()
		_advance(sim, t0 + 3.0 + fly)
		world.clock.set_time(t0 + 3.0 + fly)
		if _args.has("circularize"):
			sim.plan_circularization()
			sim.wait_idle(60.0)
		if _args.has("burn"):  # --burn=<prograde m/s>,<node in s>,<capture after s>
			var b := (_args["burn"] as String).split(",")
			sim.plan_burn(float(b[1]), float(b[0]), 0.0, 0.0)
			sim.wait_idle(60.0)
			var t_capture := t0 + 3.0 + fly + float(b[2])
			_advance(sim, t_capture)
			world.clock.set_time(t_capture)
	var cam := world.camera
	if _args.has("yaw"): cam.yaw = float(_args["yaw"])
	if _args.has("pitch"): cam.pitch = float(_args["pitch"])
	if _args.has("distance"): cam.distance_m = float(_args["distance"])
	print("render_check: telemetry ", sim.telemetry(sim.controlled_index()))


# Starts a mission at the clock's instant and flies it to the requested mission time or timeline event.
func _fly_mission(world: World) -> void:
	var sim := world.flight.sim
	var id := sim.start_mission(_args.get("mission", "starship_leo"), world.clock.tdb_s, _args.has("wait_on_pad"))
	sim.wait_idle(600.0)
	print("render_check: mission start ", sim.result(id))
	world.flight.launch_site_id = sim.mission_site_id()
	world.set_focus(World.FOCUS_VEHICLE)
	world.clock.warp = 0.0
	var target := _mission_target(sim)
	if _args.has("at_event"):  # predicted events move while flying: approach, then aim again
		_advance(sim, target - 600.0)
		target = _mission_target(sim)
	_advance(sim, target)
	target = maxf(target, _mission_target(sim))  # T-0 is the recorded liftoff once it happened
	_advance(sim, target)
	world.clock.set_time(sim.advance_to(target))
	for i in int(_args.get("view", "0")):
		world.view_target.cycle(sim, 1)
	if _args.has("legacy_hud"):
		(_main.get("legacy_hud") as CanvasLayer).visible = true
	(_main.get("overlay_host") as OverlayHost).overlay.snap_timeline()
	var index := world.view_target.resolve(sim)
	print("render_check: mission clock ", sim.mission_timeline().get("clock_s"), "  viewed ", index, "  telemetry ",
			sim.telemetry(index))


## TDB of --t_plus after T-0 (default 75 s), or of --at_event=<kind>[,<offset s>] (its first occurrence).
func _mission_target(sim: FlightSim) -> float:
	var timeline: Dictionary = sim.mission_timeline()
	if _args.has("at_event"):
		var spec := (_args["at_event"] as String).split(",")
		for e: Dictionary in timeline.get("events", []):
			if e["kind"] == spec[0]:
				return float(e["tdb_s"]) + (float(spec[1]) if spec.size() > 1 else 0.0)
		print("render_check: no event %s in %s" % [spec[0], timeline.get("events", []).map(func(e): return e["kind"])])
	var t0 = timeline.get("t0_tdb_s")
	var zero := float(t0) if t0 != null else sim.mission_start_tdb_s()
	return zero + float(_args.get("t_plus", "75")) + SIM_TICK_S  # the sim stops on its tick grid, never just short


# The simulation thread advances in chunks; wait until it has reached the target.
func _advance(sim: FlightSim, target: float) -> void:
	for i in 1000:
		sim.advance_to(target)
		sim.wait_idle(5.0)
		if sim.advance_to(target) >= target - 0.01 or not sim.is_active():
			return


# Ground view at sea: the eye --eye_height (default 10 m) above the physics' surface at --sea_lat / --sea_lon (the
# geoid at sea; the ellipsoid without the Earth terrain).
func _view_at_sea(world: World) -> void:
	var lat := float(_args["sea_lat"])
	var lon := float(_args["sea_lon"])
	var samples: Dictionary = world.core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var surface_m: float = samples["height_m"][0] if bool(samples["present"]) else 0.0
	var water: bool = bool(samples["present"]) and samples["water"][0] != 0
	print("render_check: sea view at %.4f, %.4f: surface %.2f m, water %s" % [lat, lon, surface_m, water])
	world.set_ground_site({"id": "sea", "name": "sea", "latitude_deg": lat, "longitude_deg": lon, "height_m": surface_m,
			"eye_height_m": float(_args.get("eye_height", "10"))})


# Ground camera: point at the Moon.
func _aim_at_moon(world: World) -> void:
	world.update_world()
	var cam := world.camera
	var to_moon := world.moon_gcrf_m - cam.position_gcrf_m
	var up := cam.site_up_gcrf
	var north := cam.site_north_gcrf
	var east := cam.site_east_gcrf
	var d := to_moon.normalized()
	cam.elevation = asin(clampf(d.dot(up), -1.0, 1.0))
	cam.azimuth = atan2(d.dot(east), d.dot(north))
