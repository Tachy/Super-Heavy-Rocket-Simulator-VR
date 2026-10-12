# Drives the interactive flight path in real time with injected key presses and logs, once per second:
# clock, mission time, altitude, throttle, clamped/impacted - to diagnose "vehicle does not lift off".
# godot --path game --script res://tests/flight_check.gd [-- --vehicle=starship_block3 --seconds=20]
extends SceneTree

var _main: Node
var _elapsed := 0.0
var _next_log := 0.0
var _args := {}
var _step := 0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _press(key: Key) -> void:
	var event := InputEventKey.new()
	event.keycode = key
	event.pressed = true
	Input.parse_input_event(event)
	var release := event.duplicate()
	release.pressed = false
	Input.parse_input_event(release)


func _process(delta: float) -> bool:
	var world: World = _main.get_node("World")
	_elapsed += delta
	if _step == 0 and _elapsed > 0.5:
		world.launch_vehicle(_args.get("vehicle", "starship_block3"), world.sites[0])
		print("[flight_check] launched; joypads: ", Input.get_connected_joypads().map(func(d): return Input.get_joy_name(d)))
		_step = 1
	elif _step == 1 and _elapsed > 1.5:
		_press(KEY_I)
		print("[flight_check] pressed I")
		_step = 2
	elif _step == 2 and _elapsed > 5.5:
		_press(KEY_L)
		print("[flight_check] pressed L")
		_step = 3
	if _step >= 1 and _elapsed >= _next_log:
		_next_log += 1.0
		var sim := world.flight.sim
		var t: Dictionary = sim.telemetry(sim.controlled_index())
		print("[flight_check] real %5.1f s  clock %.3f  warp %.0f  mission %6.2f  alt %8.1f m  vz %6.1f  thr %.2f  eng %d  clamped %s  impacted %s  fps %d" % [
			_elapsed, world.clock.tdb_s, world.clock.warp, sim.mission_time_s(), t.get("altitude_m", 0.0),
			t.get("vertical_speed_mps", 0.0), world.flight.throttle, t.get("engines_running", 0), t.get("clamped"),
			t.get("impacted"), Engine.get_frames_per_second()])
	return _elapsed > float(_args.get("seconds", "16"))
