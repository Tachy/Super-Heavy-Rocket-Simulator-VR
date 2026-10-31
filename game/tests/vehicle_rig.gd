class_name VehicleRig
extends RefCounted
## Test rig of the vehicle look scripts (vehicle_look_shots.gd, tile_shimmer_check.gd, vehicle_cost_check.gd): a REAL flight
## session (mission started, flown to a mission time or event; optionally the ship's return planned from orbit) and the
## orbit camera placed relative to a body (body frame: x station, y / z hull), aimed at a station of it.

const SIM_TICK_S := 0.005

var world: World
var args := {}
var aim := AimedTarget.new()
var _zero_override := NAN


## The orbit camera's target: a station of the viewed body instead of its CG.
class AimedTarget extends ViewTarget:
	var aim_station_m := NAN
	## Aim point off the axis (body frame, from the station): lets a close view centre on a fin out of the hull.
	var aim_offset_body := Vector3.ZERO

	func cg_gcrf_m(sim: FlightSim, index: int) -> Vector3:
		var at := station_gcrf_m(sim, index, sim.body_cg_station_m(index) if is_nan(aim_station_m) else aim_station_m)
		return at + Basis(sim.body_attitude_gcrf(index)) * aim_offset_body


func _init(world_: World, args_: Dictionary) -> void:
	world = world_
	args = args_


## Starts --mission (default starship_leo) at --utc, with --ship_return=<s> flies the ascent that long and plans the ship's
## return (the mission clock then counts from there). Returns the sim.
func start() -> FlightSim:
	world.clock.set_time(world.core.utc_to_tdb_s(args.get("utc", "2024-04-08T18:42:00")))
	var sim := world.flight.sim
	var id := sim.start_mission(args.get("mission", "starship_leo"), world.clock.tdb_s, false)
	sim.wait_idle(600.0)
	print("vehicle_rig: mission start ", sim.result(id))
	world.flight.launch_site_id = sim.mission_site_id()
	world.view_target = aim
	world.set_focus(World.FOCUS_VEHICLE)
	world.clock.warp = 0.0
	if args.has("ship_return"):
		advance(sim, zero_tdb_s(sim) + float(args["ship_return"]))
		var plan := sim.plan_ship_return()
		sim.wait_idle(600.0)
		print("vehicle_rig: plan_ship_return ", sim.result(plan))
		_zero_override = sim.advance_to(zero_tdb_s(sim))
	return sim


## Flies to --t_plus (s after the mission's zero) or --at_event=<kind>,<offset s> and sets the clock there.
func fly_to_target(sim: FlightSim) -> void:
	var target := target_tdb_s(sim)
	advance(sim, target)
	world.clock.set_time(sim.advance_to(target))


## T-0 of the mission (or, after --ship_return, the instant the return was planned).
func zero_tdb_s(sim: FlightSim) -> float:
	if not is_nan(_zero_override):
		return _zero_override
	var t0 = sim.mission_timeline().get("t0_tdb_s")
	return float(t0) if t0 != null else sim.mission_start_tdb_s()


func target_tdb_s(sim: FlightSim) -> float:
	var zero := zero_tdb_s(sim)
	if args.has("at_event"):
		var spec := String(args["at_event"]).split(",")
		for e: Dictionary in sim.mission_timeline().get("events", []):
			if e["kind"] == spec[0]:
				return float(e["tdb_s"]) + (float(spec[1]) if spec.size() > 1 else 0.0)
		print("vehicle_rig: no event ", spec[0])
	return zero + float(args.get("t_plus", "75")) + SIM_TICK_S


func advance(sim: FlightSim, target: float) -> void:
	for i in 1000:
		sim.advance_to(target)
		sim.wait_idle(5.0)
		if sim.advance_to(target) >= target - 0.01 or not sim.is_active():
			return


func print_bodies(sim: FlightSim) -> void:
	for i in sim.body_count():
		var stages: Array = sim.body_stages(i)
		print("vehicle_rig: body %d serial %d stages %s cg %.2f m, %d flaps, %d fins, altitude %.0f m" % [i,
				sim.body_serial(i), stages.map(func(s): return "%s@%.1f" % [s["id"], s["base_station_m"]]),
				sim.body_cg_station_m(i), sim.body_flaps(i).size(), sim.body_grid_fins(i).size(),
				sim.telemetry(i).get("altitude_m", -1.0)])
		print("    flaps ", sim.body_flaps(i).map(func(f): return snappedf(f["deflection_deg"], 0.1)),
				" fins ", sim.body_grid_fins(i).map(func(f): return snappedf(f["deflection_deg"], 0.1)))


## The viewed body: Tab through the bodies, as the game does.
func view_body(sim: FlightSim, index: int) -> void:
	for i in sim.body_count():
		if world.view_target.resolve(sim) == index:
			return
		world.view_target.cycle(sim, 1)


## Orbit camera at `offset_body` (m, body frame) from `station_m` of body `index`, aimed at that station; with `sun_turn_deg`
## not NAN instead from the Sun's side, turned about the vehicle axis by that angle, at the offset's distance; `yaw_step_rad`
## turns the camera about the target on top (a sub-pixel head motion).
func place(sim: FlightSim, index: int, station_m: float, offset_body: Vector3, fov_deg: float, sun_turn_deg := NAN,
		yaw_step_rad := 0.0, aim_offset_body := Vector3.ZERO) -> void:
	view_body(sim, index)
	var attitude := Basis(sim.body_attitude_gcrf(index))
	var anchor := sim.body_position_gcrf_m(index) + attitude * Vector3(station_m - sim.body_cg_station_m(index), 0.0, 0.0)
	aim.aim_station_m = station_m
	aim.aim_offset_body = aim_offset_body
	var from_target := attitude * offset_body
	if not is_nan(sun_turn_deg):
		var sun := (world.sun_apparent_from_earth_m - anchor).normalized()
		from_target = Quaternion(attitude * Vector3.RIGHT, deg_to_rad(sun_turn_deg)) * sun * offset_body.length()
	var direction: Vector3 = world.camera._frame.inverse() * Frames.to_render(from_target).normalized()
	world.camera.pitch = -asin(clampf(direction.y, -1.0, 1.0))
	world.camera.yaw = atan2(direction.x, direction.z) + yaw_step_rad
	world.camera.distance_m = from_target.length()
	world.camera.fov = fov_deg
	world.atmosphere.snap_adaptation()


func hide_ui(main: Node) -> void:
	for child in main.get_children():
		if child is CanvasLayer:
			child.visible = false
