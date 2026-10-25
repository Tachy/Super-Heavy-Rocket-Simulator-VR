# CloudLayer's cross-fade (M8 weather and clouds, Task 8; controller ruling: no pop on a pair or source change, only two
# field textures resident). Headless: godot --headless --path game --script res://tests/cloud_layer_test.gd
# Small fake snapshots (2 x 3 cells x 32 layers) stand in for the service's dictionaries. Checked every frame:
# - the weight of slot 1 moves by at most dt / FADE_S (no jump of the visible mix);
# - a slot's content changes only while that slot has no weight;
# - the pair ends in its real blend and then tracks the service's blend; weather off fades out the same way.
extends SceneTree

const DT := 1.0 / 60.0

var _failures := 0
var _layer: CloudLayer


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	var core := StarsimCore.new()
	_layer = CloudLayer.new()
	root.add_child(_layer)
	_layer.setup(core)
	_layer.wait_until_ready()
	var a := _fake("a")
	var b := _fake("b")
	var c := _fake("c")
	var d := _fake("d")
	var camera := Vector3(6378137.0 + 100.0, 0.0, 0.0)

	# From nothing to the pair (a, b) at blend 0.3: fades in, ends at 0.3.
	_layer.set_target(a, b)
	_check(_drive(camera, 0.3, 8.0), "first pair: continuous")
	_check(_state_is_pair(["era5|a|2", "era5|b|2"], 0.3), "first pair resident at blend 0.3: %s" % _layer.state())
	# Tracking: the blend follows the service at once.
	_layer.update(camera, 0.35, DT)
	_check(_state_is_pair(["era5|a|2", "era5|b|2"], 0.35), "tracks the blend")

	# The hour rolls over: (b, c) at blend 0.02, b is resident - only c is uploaded.
	_layer.set_target(b, c)
	_check(_drive(camera, 0.02, 8.0), "next hour: continuous")
	_check(_state_is_pair(["era5|b|2", "era5|c|2"], 0.02), "next hour resident: %s" % _layer.state())

	# (b, c) -> (d, a): no snapshot shared; the dominant new one (a at blend 0.8) comes first.
	_layer.set_target(d, a)
	_check(_drive(camera, 0.8, 12.0), "source change: continuous")
	_check(_state_is_pair(["era5|d|2", "era5|a|2"], 0.8), "source change resident: %s" % _layer.state())

	# Again a pair sharing nothing (an analog -> ERA5 switch), now with the old pair's second snapshot visible.
	_layer.set_target(b, c)
	_check(_drive(camera, 0.6, 12.0), "disjoint pair: continuous")
	_check(_state_is_pair(["era5|b|2", "era5|c|2"], 0.6), "disjoint pair resident: %s" % _layer.state())

	# Weather off: both empty, faded out.
	_layer.set_target({}, {})
	_check(_drive(camera, 0.0, 8.0), "weather off: continuous")
	_check(_state_is(["", ""], -1.0), "weather off: nothing resident: %s" % _layer.state())

	_layer.shutdown()
	print("CLOUD LAYER TEST %s" % ("PASSED" if _failures == 0 else "FAILED (%d)" % _failures))
	quit(1 if _failures > 0 else 0)


func _fake(key: String) -> Dictionary:
	var texels := PackedByteArray()
	texels.resize(2 * 3 * 32 * 4)
	return {"ok": true, "kind": "era5", "cache_key": key, "rows": 2, "columns": 3, "layers": 32,
			"cloud_texels": texels}


## Runs frames for up to `seconds`, waiting for each upload; false on a jump or a change of a weighted slot.
func _drive(camera: Vector3, blend: float, seconds: float) -> bool:
	var ok := true
	var frames := int(seconds / DT)
	for i in range(frames):
		var before: Dictionary = _layer.state()
		_layer.update(camera, blend, DT)
		var after: Dictionary = _layer.state()
		var w0 := float(before["w1"])
		var w1 := float(after["w1"])
		if absf(w1 - w0) > DT / CloudLayer.FADE_S + 1e-9 and not bool(before["tracking"]):
			print("  jump %.4f -> %.4f at frame %d" % [w0, w1, i])
			ok = false
		for s in range(2):
			var weight := w0 if s == 1 else 1.0 - w0
			if before["slots"][s] != after["slots"][s] and weight > 1e-9:
				print("  slot %d changed with weight %.4f at frame %d" % [s, weight, i])
				ok = false
		if after["uploading"]:
			OS.delay_msec(1)
	return ok


func _state_is(slots: Array, w1: float) -> bool:
	var s: Dictionary = _layer.state()
	return s["slots"] == slots and (w1 < 0.0 or is_equal_approx(float(s["w1"]), w1)) and bool(s["tracking"])


## The pair in either slot order with the visible blend (weight of the second snapshot).
func _state_is_pair(pair: Array, blend: float) -> bool:
	var s: Dictionary = _layer.state()
	if s["slots"] == pair:
		return is_equal_approx(float(s["w1"]), blend) and bool(s["tracking"])
	if s["slots"] == [pair[1], pair[0]]:
		return is_equal_approx(1.0 - float(s["w1"]), blend) and bool(s["tracking"])
	return false


func _check(ok: bool, label: String) -> void:
	if not ok:
		_failures += 1
	print("  %s  %s" % ["ok  " if ok else "FAIL", label])
