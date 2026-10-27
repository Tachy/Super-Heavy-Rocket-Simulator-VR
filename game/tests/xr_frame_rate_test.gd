extends SceneTree
## The VR frame-rate rule (cloud view cache plan Task 1, user 2026-10-27: "mit Wetter immer 45 fps, ohne 90"):
## XrManager.target_hz by the weather switch, applied only in VR (the desktop keeps its frame rate).
## godot --headless --path game --script res://tests/xr_frame_rate_test.gd

const XR_MANAGER := preload("res://xr/xr_manager.gd")

var _failures := 0


func _initialize() -> void:
	_check(XR_MANAGER.target_hz(true) == 45.0, "weather on: 45 Hz (%.1f)" % XR_MANAGER.target_hz(true))
	_check(XR_MANAGER.target_hz(false) == 90.0, "weather off: 90 Hz (%.1f)" % XR_MANAGER.target_hz(false))
	var manager: Node = XR_MANAGER.new()
	root.add_child(manager)  # headless: desktop mode
	var before := Engine.max_fps
	manager.apply_frame_rate(true)
	manager.apply_frame_rate(false)
	_check(Engine.max_fps == before, "desktop: the frame rate is left alone (%d)" % Engine.max_fps)
	manager.queue_free()
	print("xr_frame_rate_test: %s" % ("PASS" if _failures == 0 else "FAIL (%d)" % _failures))
	quit(1 if _failures > 0 else 0)


func _check(ok: bool, message: String) -> void:
	if ok:
		print("  ok    %s" % message)
	else:
		_failures += 1
		print("  FAIL  %s" % message)
