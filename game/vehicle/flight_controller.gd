class_name FlightController
extends Node
## Owns the C++ flight session: pilot input (HOTAS + keyboard), pilot actions, vehicle views.
## Keyboard: W/S pitch, A/D yaw, Q/E roll, Shift/Ctrl throttle, I ignition, L release clamps,
## K cutoff, G stage separation, M control mode, P ascent autopilot (config/missions/leo_ascent.toml),
## O circularize at apoapsis, R return the ship to the site's tower (config/missions/ship_return_starbase.toml),
## X autopilot off. HOTAS (stick + throttle found by name): config/input_hotas.toml.

const THROTTLE_KEY_RATE := 0.5  # per second

var sim := FlightSim.new()
var launch_site_id := ""  ## site of the current flight (its tower shows the flight's arm state)
var throttle := 1.0
var rate_mode := true
var views: Array[VehicleView] = []
var trajectory := TrajectoryView.new()
var navball: NavBall
var overlay: FlightOverlay
var ascent_target := {}
var last_event := ""
## Launch posted to the simulation thread and not yet finished: {id, site_id, text}.
var _pending_launch := {}

var _stick := {}     ## [stick] table of config/input_hotas.toml
var _throttle := {}  ## [throttle] table
var _joydump := OS.get_cmdline_user_args().has("--joydump")
## The throttle lever reports 0 (= 50 %) until it is first moved; only then does it take over.
var lever_armed := false
var _lever_initial := NAN


func setup(config_dir: String) -> void:
	sim.setup(config_dir)
	if not sim.command_finished.is_connected(_on_command_finished):
		sim.command_finished.connect(_on_command_finished)
	ascent_target = TomlLite.load_file(config_dir.path_join("missions/leo_ascent.toml")).get("target", {})
	var mapping := TomlLite.load_file(config_dir.path_join("input_hotas.toml"))
	_stick = mapping.get("stick", {})
	_throttle = mapping.get("throttle", {})


func launch(vehicle_id: String, site: Dictionary, tdb_s: float, heading_deg := 90.0) -> bool:
	for view in views:
		view.queue_free()
	views.clear()
	throttle = 1.0
	var id := sim.launch_from_site(vehicle_id, site["id"], heading_deg, tdb_s)
	launch_site_id = ""
	_pending_launch = {"id": id, "site_id": site["id"], "text": "%s auf %s" % [vehicle_id, site["name"]]}
	last_event = "Start wird vorbereitet" if id > 0 else "Start fehlgeschlagen"
	return id > 0


## Outcome of a command on the simulation thread (emitted from FlightSim.advance_to / wait_idle).
func _on_command_finished(id: int, command: String, ok: bool, _message: String) -> void:
	match command:
		"launch_from_site":
			if id != _pending_launch.get("id", 0):
				return
			launch_site_id = _pending_launch["site_id"] if ok else ""
			last_event = _pending_launch["text"] if ok else "Start fehlgeschlagen"
			_pending_launch = {}
		"plan_circularization":
			last_event = "Zirkularisierung geplant" if ok \
					else "Kein Manöver: keine Ellipsenbahn oder Δv zu klein für die Haupttriebwerke"
		"plan_ship_return":
			if ok:
				last_event = "Rückkehr geplant: Deorbit-Brennvorgang, Eintritt, Fang am Turm"
			else:
				last_event = "Keine Rückkehr: %s" % sim.autopilot_status().get("return", {}).get("reason", "kein Schiff im Orbit")


## Advances the simulation to `tdb_s`; returns the time actually reached (the sim may lag at high warp).
func advance(tdb_s: float, delta: float) -> float:
	if not sim.is_active():
		return tdb_s
	_read_input(delta)
	return sim.advance_to(tdb_s)


func place_views(parent: Node3D, camera_gcrf_m: Vector3) -> void:
	if trajectory.get_parent() == null:
		parent.add_child(trajectory)
	trajectory.update_view(sim, camera_gcrf_m, get_process_delta_time())
	_update_instruments()
	if not sim.is_active():
		return
	while views.size() < sim.body_count():
		var view := VehicleView.new()
		view.body_index = views.size()
		parent.add_child(view)
		views.append(view)
	for view in views:
		view.update_view(sim, camera_gcrf_m)


func controlled_position_gcrf() -> Vector3:
	return sim.body_position_gcrf_m(sim.controlled_index()) if sim.is_active() else Vector3.ZERO


func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo or not sim.is_active():
		return
	match key.keycode:
		KEY_I:
			sim.ignite()
			last_event = "Zündung"
		KEY_L:
			sim.release_clamps()
			last_event = "Klammern gelöst"
		KEY_K:
			sim.cutoff()
			last_event = "Triebwerke aus"
		KEY_G:
			sim.stage()
			last_event = "Stufentrennung"
		KEY_P:
			sim.set_booster_return(true)  # the booster flies back and is caught after staging
			sim.engage_ascent_autopilot(ascent_target.get("periapsis_km", 200.0), ascent_target.get("apoapsis_km", 200.0),
					ascent_target.get("inclination_deg", 26.5), ascent_target.get("northerly", true))
			last_event = "Aufstiegs-Autopilot aktiv"
		KEY_O:
			sim.plan_circularization()  # outcome: _on_command_finished
			last_event = "Zirkularisierung wird geplant"
		KEY_R:
			sim.plan_ship_return()  # outcome: _on_command_finished
			last_event = "Rückkehr wird geplant"
		KEY_X:
			sim.disengage_autopilot()
			last_event = "Autopilot aus"
		KEY_M:
			rate_mode = not rate_mode
			sim.set_rate_mode(rate_mode)
			last_event = "Steuerung: %s" % ("Ratensteuerung (SAS)" if rate_mode else "Direkt (Gimbal)")


## Feeds NavBall and flight director: local horizon, velocity vector, commanded direction.
func _update_instruments() -> void:
	if navball == null or overlay == null:
		return
	navball.visible = sim.is_active()
	if not sim.is_active():
		overlay.target_gcrf = Vector3.ZERO
		overlay.prograde_gcrf = Vector3.ZERO
		return
	var index := sim.controlled_index()
	var position := sim.body_position_gcrf_m(index)
	var velocity := sim.body_velocity_gcrf_mps(index)
	var up := position.normalized()
	var north := (Vector3(0, 0, 1) - up * up.z).normalized()
	# Below ~70 km the surface-relative velocity matters (aerodynamics), above it the inertial one (orbit).
	var relative := velocity - Vector3(0, 0, 7.2921150e-5).cross(position)
	var prograde := (relative if position.length() - 6378137.0 < 70000.0 else velocity).normalized()
	var status: Dictionary = sim.autopilot_status()
	var target: Vector3 = status.get("direction", Vector3.ZERO) if status.get("has_direction", false) else Vector3.ZERO
	navball.attitude = sim.body_attitude_gcrf(index)
	navball.up_gcrf = up
	navball.north_gcrf = north
	navball.prograde_gcrf = prograde if velocity.length() > 1.0 else Vector3.ZERO
	navball.target_gcrf = target
	overlay.prograde_gcrf = navball.prograde_gcrf
	overlay.target_gcrf = target


func _read_input(delta: float) -> void:
	var pitch := _key_axis(KEY_S, KEY_W)
	var yaw := _key_axis(KEY_A, KEY_D)
	var roll := _key_axis(KEY_Q, KEY_E)
	if Input.is_key_pressed(KEY_SHIFT):
		throttle = minf(throttle + THROTTLE_KEY_RATE * delta, 1.0)
	if Input.is_key_pressed(KEY_CTRL):
		throttle = maxf(throttle - THROTTLE_KEY_RATE * delta, 0.0)

	var stick := _find_device(_stick.get("device_name", ""))
	if stick >= 0:
		pitch += _axis(stick, _stick, "pitch")
		yaw += _axis(stick, _stick, "yaw")
		roll += _axis(stick, _stick, "roll")
	var lever := _find_device(_throttle.get("device_name", ""))
	if lever >= 0:
		var lever_value := _axis(lever, _throttle, "throttle", false)
		if is_nan(_lever_initial):
			_lever_initial = lever_value
		elif not lever_armed and absf(lever_value - _lever_initial) > 0.02:
			lever_armed = true
			last_event = "Schubhebel aktiv"
		if lever_armed:
			throttle = (lever_value + 1.0) * 0.5
	if _joydump:
		for device in Input.get_connected_joypads():
			var values := PackedStringArray()
			for axis in 6:
				values.append("%d:%+.2f" % [axis, Input.get_joy_axis(device, axis)])
			print("[joydump] %d %s  %s" % [device, Input.get_joy_name(device), " ".join(values)])
	sim.set_input(clampf(pitch, -1.0, 1.0), clampf(yaw, -1.0, 1.0), clampf(roll, -1.0, 1.0), throttle)


## Index of the first connected joypad whose name contains `name_part`, or -1.
static func _find_device(name_part: String) -> int:
	if name_part.is_empty():
		return -1
	for device in Input.get_connected_joypads():
		var raw_name := str(Input.get_joy_info(device).get("raw_name", ""))
		if Input.get_joy_name(device).contains(name_part) or raw_name.contains(name_part):
			return device
	return -1


static func _axis(device: int, mapping: Dictionary, role: String, apply_deadzone := true) -> float:
	var axis := int(mapping.get(role + "_axis", -1))
	if axis < 0:
		return 0.0
	var value := Input.get_joy_axis(device, axis)
	if mapping.get(role + "_invert", false):
		value = -value
	if apply_deadzone and absf(value) < float(mapping.get("deadzone", 0.04)):
		return 0.0
	return value


static func _key_axis(negative: Key, positive: Key) -> float:
	return (1.0 if Input.is_key_pressed(positive) else 0.0) - (1.0 if Input.is_key_pressed(negative) else 0.0)
