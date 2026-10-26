# Headless test of FrameProfiler (the perf log's named main-thread sections).
# Run: godot --headless --path game --script res://tests/frame_profiler_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	FrameProfiler.enabled = false
	FrameProfiler.begin(&"weather")
	_busy_ms(2.0)
	FrameProfiler.end(&"weather")
	_check(FrameProfiler.take_frame().is_empty(), "disabled: nothing recorded")

	FrameProfiler.enabled = true
	FrameProfiler.begin(&"weather")
	_busy_ms(2.0)
	FrameProfiler.end(&"weather")
	var frame := FrameProfiler.take_frame()
	var usec := int(frame.get(&"weather", 0))
	_check(usec >= 1500 and usec <= 20000, "a 2 ms section reports 1500-20000 usec (%d)" % usec)
	_check(FrameProfiler.take_frame().is_empty(), "take_frame clears")

	FrameProfiler.begin(&"clouds")
	_busy_ms(1.0)
	FrameProfiler.end(&"clouds")
	FrameProfiler.begin(&"clouds")
	_busy_ms(1.0)
	FrameProfiler.end(&"clouds")
	usec = int(FrameProfiler.take_frame().get(&"clouds", 0))
	_check(usec >= 1500, "two pairs of one section add up (%d usec)" % usec)
	_check(FrameProfiler.SECTIONS.has(&"weather") and FrameProfiler.SECTIONS.has(&"ui_hud"), "fixed section list")
	FrameProfiler.enabled = false
	print("FRAME PROFILER TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _busy_ms(ms: float) -> void:
	var end := Time.get_ticks_usec() + int(ms * 1000.0)
	while Time.get_ticks_usec() < end:
		pass


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
