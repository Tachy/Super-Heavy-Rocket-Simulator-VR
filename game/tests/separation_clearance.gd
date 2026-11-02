# Falcon 9 stage separation on a flown mission: does the DRAWN second stage (Merlin Vacuum bell, aft cone, hull's base rim) cut
# through the DRAWN interstage wall, and when (docs/subsystems/falcon9.md "Stack")? The physics has no collision between the
# parts; this measures what the viewer sees. Rings of the bell against the open cylinder, in the first stage's frame.
#  - inside: sample points in the cylinder's volume; outside: points beside the wall's height range, off the cylinder.
#  - the wall cuts the drawn parts while the same ring set has points on both sides of it at the wall's height.
#  - clearance: smallest distance of any point to the wall (a segment in station and radius), signed only by the state above.
# godot --headless --path game --script res://tests/separation_clearance.gd -- [--mission=falcon9_crew_dragon] [--seconds=12]
extends SceneTree

const STEP_S := 0.25
const AZIMUTHS := 48
const BELL_RINGS := 12
## detail/falcon9_block5.toml, config/vehicles/falcon9_block5.toml (a measuring script: values named, not read).
const GIMBAL_STATION_M := -0.6
const BELL_LENGTH_M := 3.4
const BELL_TOP_RADIUS_M := 0.69
const BELL_EXIT_RADIUS_M := 1.65
const BELL_CONTOUR_EXPONENT := 0.84
const SKIRT_RADIUS_M := 1.75
const HULL_RADIUS_M := 1.83
const WALL_THICKNESS_M := 0.04
const INTERSTAGE_BASE_M := 41.2
const INTERSTAGE_TOP_M := 47.7

var _args := {}
var _main: Node
var _world: World
var _started := false


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	if not _started and Engine.get_process_frames() > 10 and not _world.atmosphere.loading:
		_started = true
		_run()
	return false


func _run() -> void:
	var rig_args := {"mission": _args.get("mission", "falcon9_crew_dragon"), "utc": "2024-04-08T15:00:00"}
	var rig := VehicleRig.new(_world, rig_args)
	var sim := rig.start()
	var zero := rig.zero_tdb_s(sim)
	var t := 100.0
	while sim.body_count() < 2 and t < 300.0:
		t += STEP_S
		rig.advance(sim, zero + t)
	if sim.body_count() < 2:
		print("separation_clearance: no separation by T+300 s")
		quit(1)
		return
	var sep := t
	print("separation_clearance: two bodies at T+%.2f s" % sep)
	var seconds := float(_args.get("seconds", "12"))
	var first_cut := NAN
	var last_cut := NAN
	var min_clearance := INF
	var min_t := NAN
	while t <= sep + seconds:
		var row := _measure(sim)
		if row.is_empty():
			quit(1)
			return
		if row["cut"]:
			first_cut = t - sep if is_nan(first_cut) else first_cut
			last_cut = t - sep
		elif is_nan(first_cut) and row["clearance_m"] < min_clearance:
			min_clearance = row["clearance_m"]
			min_t = t - sep
		print("separation_clearance: dt %6.2f  T+%7.2f  cut %-5s  inside %4d  through %4d (deepest %6.3f m, %s)  clearance %6.3f m  exit above rim %7.3f m  tilt %6.2f deg  lateral %6.3f m  axial %6.3f m/s" % [
				t - sep, t, row["cut"], row["inside"], row["through"], row["deepest_m"], row["part"], row["clearance_m"],
				row["exit_above_rim_m"], row["tilt_deg"], row["lateral_m"], row["axial_mps"]])
		t += STEP_S
		rig.advance(sim, zero + t)
	print("separation_clearance: RESULT cut from dt %s s to dt %s s; before the cut smallest clearance %.3f m at dt %s s" % [
			first_cut, last_cut, min_clearance, min_t])
	quit(0)


func _measure(sim: FlightSim) -> Dictionary:
	var lower := -1
	var upper := -1
	var base := 0.0
	for i in sim.body_count():
		for s: Dictionary in sim.body_stages(i):
			if s["id"] == "first_stage":
				lower = i
			elif s["id"] == "second_stage":
				upper = i
				base = s["base_station_m"]
	if lower < 0 or upper < 0:
		print("separation_clearance: bodies not found")
		return {}
	var a_low := Basis(sim.body_attitude_gcrf(lower))
	var a_up := Basis(sim.body_attitude_gcrf(upper))
	var axis_low := a_low * Vector3.RIGHT
	var axis_up := a_up * Vector3.RIGHT
	var p_low: Vector3 = sim.body_position_gcrf_m(lower) - axis_low * sim.body_cg_station_m(lower)
	var p_up: Vector3 = sim.body_position_gcrf_m(upper) + axis_up * (base - sim.body_cg_station_m(upper))
	var rings: Array = [[0.0, HULL_RADIUS_M, "hull"], [0.0, SKIRT_RADIUS_M, "skirt"]]
	for k in BELL_RINGS + 1:
		var f := float(k) / BELL_RINGS
		rings.append([GIMBAL_STATION_M - f * BELL_LENGTH_M,
				BELL_TOP_RADIUS_M + (BELL_EXIT_RADIUS_M - BELL_TOP_RADIUS_M) * pow(f, BELL_CONTOUR_EXPONENT), "bell"])
	var wall_r := HULL_RADIUS_M - WALL_THICKNESS_M
	var u_y := a_up * Vector3.UP
	var u_z := a_up * Vector3.BACK
	var inside := 0
	var through := 0
	var deepest := 0.0
	var part := "-"
	var clearance := INF
	for ring: Array in rings:
		var hull_rim: bool = ring[2] == "hull"  # stands above the rim at the start, off the cylinder by its own radius
		for j in AZIMUTHS:
			var az := TAU * j / AZIMUTHS
			var p: Vector3 = p_up + axis_up * ring[0] + (u_y * cos(az) + u_z * sin(az)) * ring[1]
			var d := p - p_low
			var x := d.dot(axis_low)
			var r := (d - axis_low * x).length()
			var at_wall_height := x >= INTERSTAGE_BASE_M and x <= INTERSTAGE_TOP_M
			clearance = minf(clearance, Vector2(r - wall_r, x - clampf(x, INTERSTAGE_BASE_M, INTERSTAGE_TOP_M)).length())
			if at_wall_height and r < wall_r:
				inside += 1
			elif at_wall_height:
				through += 1
				if r - wall_r > deepest:
					deepest = r - wall_r
					part = ring[2]
			elif hull_rim:
				pass
	var exit_centre: Vector3 = p_up + axis_up * (GIMBAL_STATION_M - BELL_LENGTH_M)
	var from_low := exit_centre - p_low
	var along := from_low.dot(axis_low)
	var relative: Vector3 = sim.body_velocity_gcrf_mps(upper) - sim.body_velocity_gcrf_mps(lower)
	return {"cut": inside > 0 and through > 0, "inside": inside, "through": through, "deepest_m": deepest, "part": part,
			"clearance_m": clearance, "exit_above_rim_m": along - INTERSTAGE_TOP_M,
			"tilt_deg": rad_to_deg(axis_low.angle_to(axis_up)), "lateral_m": (from_low - axis_low * along).length(),
			"axial_mps": relative.dot(axis_low)}
