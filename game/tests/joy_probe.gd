# Lists connected joypads and records which axes move (range and time of first movement), so the
# HOTAS mapping in config/input_hotas.toml can be derived. Move the axes one after another.
# godot --path game --script res://tests/joy_probe.gd -- --seconds=30
# With --buttons it prints every joystick button press (device, 0-based index) instead of the axis table, for [buttons]:
# godot --path game --script res://tests/joy_probe.gd -- --buttons --seconds=30
extends SceneTree

const MOVEMENT := 0.3
const BUTTON_COUNT := 32

var _elapsed := 0.0
var _seconds := 3.0
var _min := {}
var _max := {}
var _first := {}
var _start := {}
var _buttons := false
var _held := {}  ## "device:button" -> held at the last frame
var _pressed_seen := {}  ## "device:button" -> press count


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--seconds="):
			_seconds = float(arg.split("=")[1])
		elif arg == "--buttons":
			_buttons = true
	print("[joy_probe] recording %.0f s - %s" % [_seconds, "press the buttons now" if _buttons else "move the axes now"])


func _process(delta: float) -> bool:
	_elapsed += delta
	if _buttons:
		return _process_buttons()
	for device in Input.get_connected_joypads():
		for axis in 10:
			var key := "%d:%d" % [device, axis]
			var value := Input.get_joy_axis(device, axis)
			if not _start.has(key):
				_start[key] = value
			_min[key] = minf(_min.get(key, value), value)
			_max[key] = maxf(_max.get(key, value), value)
			if not _first.has(key) and absf(value - _start[key]) > MOVEMENT:
				_first[key] = _elapsed
	if _elapsed < _seconds:
		return false
	for device in Input.get_connected_joypads():
		print("device %d: '%s' (%s)" % [device, Input.get_joy_name(device), Input.get_joy_info(device).get("raw_name", "")])
		for axis in 10:
			var key := "%d:%d" % [device, axis]
			if _max[key] - _min[key] > 0.05:
				print("  axis %d: range %+.2f .. %+.2f, now %+.2f, first moved at %s" % [axis, _min[key], _max[key],
						Input.get_joy_axis(device, axis), ("%.1f s" % _first[key]) if _first.has(key) else "-"])
	return true


## Prints each button press once (rising edge) and a summary at the end.
func _process_buttons() -> bool:
	for device in Input.get_connected_joypads():
		for button in BUTTON_COUNT:
			var key := "%d:%d" % [device, button]
			var down := Input.is_joy_button_pressed(device, button)
			if down and not _held.get(key, false):
				_pressed_seen[key] = _pressed_seen.get(key, 0) + 1
				print("[joy_probe] %.1f s  device %d '%s'  button %d pressed" % [_elapsed, device, Input.get_joy_name(device), button])
			_held[key] = down
	if _elapsed < _seconds:
		return false
	for key: String in _pressed_seen:
		print("summary: device:button %s pressed %d x" % [key, _pressed_seen[key]])
	return true
