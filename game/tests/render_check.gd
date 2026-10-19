# Renders the main scene at a fixed instant and camera, saves a PNG and quits (needs a GPU, no --headless).
# godot --path game --script res://tests/render_check.gd -- --utc=2024-04-08T18:42:00 --focus=earth
# Vehicle: --vehicle=starship_block3 --site=starbase_olp1 [--fly=40] (ignite, liftoff at 3 s, fly N s)
#          [--autopilot] (ascent autopilot instead of manual) [--circularize] (plan node afterwards)
# Catch tower: [--tower_arms_deg=0] draws the arms at a fixed opening (0 = closed).
#     [--yaw=0.6] [--pitch=-0.3] [--distance=2.2e7] [--azimuth=90] [--elevation=5] [--fov=50] --out=C:/tmp/x.png
extends SceneTree

const FRAMES_TO_SETTLE := 6

var _args := {}
var _main: Node
var _frames := 0


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
		if _args.has("tower_arms_deg"):
			for view in world.towers:
				view.preview_opening_rad = deg_to_rad(float(_args["tower_arms_deg"]))
				view.update_state({})
	_frames += 1
	if _frames == FRAMES_TO_SETTLE:
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


# The simulation thread advances in chunks; wait until it has reached the target.
func _advance(sim: FlightSim, target: float) -> void:
	for i in 1000:
		sim.advance_to(target)
		sim.wait_idle(5.0)
		if sim.advance_to(target) >= target - 0.01 or not sim.is_active():
			return


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
