# Drives the interactive launch path in real time and logs, once per second: clock, mission time, T clock,
# autopilot state, altitude, throttle, clamped/impacted - to diagnose "vehicle does not lift off".
# The external view flies on the autopilot (user 2026-10-22): a free launch counts down and lifts off at T-0 on the
# ascent autopilot, and the pilot keys are inactive. The manual-flight check (I ignition, L release clamps) returns with
# the cockpit, where manual control will live.
# godot --path game --script res://tests/flight_check.gd [-- --vehicle=starship_block3 --countdown=10 --seconds=30]
#       [--weather=on|off] (overrides the saved "Wetter" switch for this run only) [--perf-log] (PerfLog; needs a window)
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
		if _args.has("weather"):
			world.weather.enabled = _args["weather"] != "off"
		world.launch_vehicle(_args.get("vehicle", "starship_block3"), world.sites[0], float(_args.get("countdown", "10")))
		world.flight.sim.wait_idle(60.0)  # the launch runs on the simulation thread; its completion engages the autopilot
		print("[flight_check] launched; joypads: ", Input.get_connected_joypads().map(func(d): return Input.get_joy_name(d)))
		_step = 1
	elif _step == 1 and _elapsed > 1.5:
		_press(KEY_L)  # inactive in the external view: the clamps stay until the autopilot releases them at T-0
		print("[flight_check] pressed L (expected: no effect)")
		_step = 2
	if _step >= 1 and _elapsed >= _next_log:
		_next_log += 1.0
		var sim := world.flight.sim
		var t: Dictionary = sim.telemetry(sim.controlled_index())
		var clock = sim.mission_timeline().get("clock_s")
		print("[flight_check] real %5.1f s  clock %.3f  warp %.0f  mission %6.2f  T %s  ap %s  alt %8.1f m  vz %6.1f  thr %.2f  eng %d  clamped %s  impacted %s  fps %d" % [
			_elapsed, world.clock.tdb_s, world.clock.warp, sim.mission_time_s(),
			"-" if clock == null else "%+.1f" % float(clock), sim.autopilot_status().get("state", ""),
			t.get("altitude_m", 0.0), t.get("vertical_speed_mps", 0.0), world.flight.throttle, t.get("engines_running", 0),
			t.get("clamped"), t.get("impacted"), Engine.get_frames_per_second()])
	return _elapsed > float(_args.get("seconds", "30"))
