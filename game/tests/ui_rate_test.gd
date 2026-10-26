# Headless test of UiRate (text and round gauges at 15 Hz; a clock jump or invalidate() updates at once).
# Run: godot --headless --path game --script res://tests/ui_rate_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	var rate := UiRate.new()
	var frame := 1.0 / 90.0
	var t := 1000.0
	_check(rate.due(frame, t, 1.0), "first frame: due")
	t += frame
	_check(not rate.due(frame, t, 1.0), "next 90 Hz frame: not due")
	var due_count := 0
	for i in 90:  # one second at 90 Hz
		t += frame
		if rate.due(frame, t, 1.0):
			due_count += 1
	_check(due_count >= 14 and due_count <= 16, "15 updates per second (%d)" % due_count)
	t += frame
	rate.due(frame, t, 1.0)
	t += 3600.0  # a clock jump
	_check(rate.due(frame, t, 1.0), "clock jump: due at once")
	t += frame * 100.0
	rate.due(1.0, t, 100.0)
	t += frame * 100.0
	_check(not rate.due(frame, t, 100.0), "time warp 100: the next frame is not due")
	rate.invalidate()
	_check(rate.due(frame, t, 100.0), "invalidate(): due at once")
	print("UI RATE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
