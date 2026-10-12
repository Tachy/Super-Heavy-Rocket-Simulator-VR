# Lists connected joypads and records which axes move (range and time of first movement), so the
# HOTAS mapping in config/input_hotas.toml can be derived. Move the axes one after another.
# godot --path game --script res://tests/joy_probe.gd -- --seconds=30
extends SceneTree

const MOVEMENT := 0.3

var _elapsed := 0.0
var _seconds := 3.0
var _min := {}
var _max := {}
var _first := {}
var _start := {}


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--seconds="):
			_seconds = float(arg.split("=")[1])
	print("[joy_probe] recording %.0f s - move the axes now" % _seconds)


func _process(delta: float) -> bool:
	_elapsed += delta
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
