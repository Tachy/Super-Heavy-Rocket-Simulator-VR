# Desktop run of the free camera: enters it from the Earth view, flies it by a synthetic command, checks the pose and the
# panel; no HOTAS needed. Run (GPU, windowed): godot --path game --script res://tests/free_camera_run.gd
extends SceneTree

var _frames := 0
var _world: World
var _start := Vector3.ZERO
var _failures := 0


func _initialize() -> void:
	var main: Node = load("res://main.tscn").instantiate()
	root.add_child(main)
	_world = main.get("world")


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames == 5:
		_world.set_focus(World.FOCUS_FREE)
	elif _frames == 8:
		_start = _world.camera.position_gcrf_m
		_check(_world.camera.mode == OrbitCamera.Mode.FREE, "free mode entered")
		_check(_world.free_camera_text.begins_with("Freie Kamera"), "axis readout shown")
		_world.free_pose.position_frame_m += Vector3(1000.0, 0.0, 0.0)
	elif _frames == 12:
		_check((_world.camera.position_gcrf_m - _start).length() > 500.0, "camera follows the free pose")
		_world.set_focus(World.FOCUS_MOON)
	elif _frames == 16:
		_check(_world.camera.mode == OrbitCamera.Mode.ORBIT and _world.free_camera_text == "", "back to the normal view")
		print("[free_camera_run] %s" % ("OK" if _failures == 0 else "FAIL"))
		quit(1 if _failures > 0 else 0)
	return false


func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		print("[free_camera_run] FAIL: ", what)
