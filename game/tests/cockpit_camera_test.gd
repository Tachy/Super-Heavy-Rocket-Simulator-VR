# Headless test of the cockpit camera mode (Task 6): the pure parts (look limits, HotasButtons edges, config) and the
# World in a real session: a Dragon mission starts in the cockpit, the head follows the capsule body to 1 mm over 100
# frames and through the staging, toggle_cockpit twice restores the pose and calls _view_jumped twice, the toggle under
# warp x100 is stable, a Starship mission starts external and the toggle logs once and changes nothing. Needs the generated
# models (python tools/structures/build_models.py --vehicle crew_dragon) and the built GDExtension.
# Run: godot --headless --path game --script res://tests/cockpit_camera_test.gd
extends SceneTree

const POSE_TOLERANCE_M := 0.001  # plan Task 6
const FRAMES_CHECKED := 100
const WARP := 100.0
const STAGING_MISSION_S := 175.0  # past the first stage separation (cockpit_view_test uses T+170 s)
const TIMEOUT_S := 280.0

var _failures := 0
var _main: Node
var _world: World
var _step := ""
var _frames := 0
var _real_s := 0.0
var _checked := 0
var _worst_m := 0.0
var _toggle_pose := {}
var _jumps := 0
var _staged_checks := 0


func _initialize() -> void:
	_check_look_limits()
	_check_buttons()
	_check_pose_math()
	if not VehicleModels.load_for("crew_dragon").has("cabin"):
		_check(false, "crew_dragon models with the cabin generated (%s)" % VehicleModels.fallback_reason("crew_dragon"))
		_finish()
		return
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_step = "boot"


func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		print("FAIL: ", what)
	else:
		print("ok: ", what)


func _finish() -> bool:
	print("COCKPIT CAMERA TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)
	return true


# ---- pure parts ----

func _check_look_limits() -> void:
	var camera := CockpitCamera.new()
	camera.look(Vector2(10.0, 10.0))
	_check(is_equal_approx(camera.look_yaw_rad, deg_to_rad(170.0)) and is_equal_approx(camera.look_pitch_rad, deg_to_rad(80.0)), "look clamps at +170 deg yaw, +80 deg pitch")
	camera.look(Vector2(-20.0, -20.0))
	_check(is_equal_approx(camera.look_yaw_rad, -deg_to_rad(170.0)) and is_equal_approx(camera.look_pitch_rad, -deg_to_rad(80.0)), "look clamps at -170 deg yaw, -80 deg pitch")
	camera.look(Vector2(0.01, -0.02))
	_check(is_equal_approx(camera.look_pitch_rad, -deg_to_rad(80.0) + 0.01), "small moves accumulate")
	camera.reset_look()
	_check(camera.look_yaw_rad == 0.0 and camera.look_pitch_rad == 0.0, "F12 reset_look centres the view")


func _check_buttons() -> void:
	var buttons := HotasButtons.new()
	buttons.configure({"stick_device": "T16000M", "cockpit_toggle": 2, "camera_next": 3})
	var held := {"cockpit_toggle": false, "camera_next": false}
	buttons.set_reader(func(role: String) -> bool: return held[role])
	var script := [  # [cockpit_toggle, camera_next] per frame -> expected pressed [toggle, next]
		[false, false, false, false], [true, false, true, false], [true, false, false, false], [true, true, false, true],
		[false, true, false, false], [false, false, false, false], [true, false, true, false]]
	var ok := true
	for row: Array in script:
		held["cockpit_toggle"] = row[0]
		held["camera_next"] = row[1]
		buttons.update()
		ok = ok and buttons.pressed("cockpit_toggle") == row[2] and buttons.pressed("camera_next") == row[3]
	_check(ok, "HotasButtons: pressed() is true on the frame of the press only")
	_check(buttons.held("cockpit_toggle") and not buttons.held("camera_next") and not buttons.pressed("nothing"), "held() and an unknown role")
	var config: Dictionary = TomlLite.load_file(Paths.config_dir().path_join("input_hotas.toml")).get("buttons", {})
	var loaded := HotasButtons.new()
	loaded.configure(config)
	_check(loaded.button_index("cockpit_toggle") == 2 and loaded.button_index("camera_next") == 3 and loaded.button_index("stick_device") == -1, "config [buttons]: stick_device T16000M, cockpit_toggle 2, camera_next 3")


func _check_pose_math() -> void:
	var config := CockpitConfig.for_vehicle("crew_dragon")
	var camera := CockpitCamera.new()
	var attitude := Basis(Quaternion(Vector3(0.3, 0.5, 0.2).normalized(), 1.1))
	camera.set_body_pose(Vector3(1.0e6, -2.0e6, 3.0e6), attitude, 12.0, 20.0, config)
	var expected := Vector3(1.0e6, -2.0e6, 3.0e6) + attitude * (Vector3(8.0, 0.0, 0.0) + config.eye.origin)
	_check(camera.position_gcrf_m.distance_to(expected) < 1.0e-9, "set_body_pose: body + attitude * (base - CG + eye)")
	var gaze := -camera.basis_render().z
	_check(gaze.distance_to(Frames.to_render(attitude * (-config.eye.basis.z))) < 1.0e-12, "gaze = attitude * eye forward, in render axes")
	camera.look(Vector2(0.0, PI / 2.0))
	camera.set_body_pose(Vector3.ZERO, Basis.IDENTITY, 0.0, 0.0, config)
	var seat_forward := -config.eye.basis.z
	var up := config.eye.basis.y
	var turned := -camera.basis_gcrf.z
	_check(turned.distance_to(Basis(up, PI / 2.0) * seat_forward) < 1.0e-12, "positive yaw look turns the gaze about the seat's up axis")


# ---- the World in a real session ----

func _process(delta: float) -> bool:
	if _main == null:
		return true
	_real_s += delta
	if _real_s > TIMEOUT_S:
		_check(false, "timeout in step '%s'" % _step)
		return _finish()
	_world = _main.get_node_or_null("World")
	if _world == null or not _world.booted:
		return false
	_frames += 1
	match _step:
		"boot":
			if _frames < 5:
				return false
			_world.weather.enabled = false
			_world.start_mission("falcon9_crew_dragon")
			_step = "dragon_start"
			_frames = 0
		"dragon_start":
			if _world.flight.sim.is_active() and _world.cockpit_active and _world.flight.views.size() > 0:
				_check(_world.camera.mode == OrbitCamera.Mode.COCKPIT, "a Dragon mission starts in the cockpit (camera mode COCKPIT)")
				_step = "follow"
				_frames = 0
			elif _frames > 3000:
				_check(false, "Dragon mission did not start in the cockpit (active %s, cockpit %s)" % [_world.flight.sim.is_active(), _world.cockpit_active])
				return _finish()
		"follow":
			_world.clock.warp = 1.0
			_measure()
			if _checked >= FRAMES_CHECKED:
				_check(_worst_m < POSE_TOLERANCE_M, "head at body + attitude * (eye - CG) over %d frames (worst %.6f m)" % [_checked, _worst_m])
				_step = "toggle"
		"toggle":
			_toggle_round_trip()
			_step = "warp"
			_checked = 0
			_worst_m = 0.0
		"warp":
			_world.clock.warp = WARP
			_measure()
			if _world.flight.sim.body_count() > 1 and _staged_checks == 0:
				print("  staged at mission time %.1f s (%d bodies)" % [_world.flight.sim.mission_time_s(), _world.flight.sim.body_count()])
				_staged_checks = 1
				_world.toggle_cockpit()  # under warp x100: out and back in
				_world.toggle_cockpit()
				_check(_world.cockpit_active and _pose_finite(), "toggling twice under warp x100 after the staging leaves the cockpit on, pose finite")
			if _staged_checks > 0:
				_staged_checks += 1
			if _staged_checks > FRAMES_CHECKED:
				_check(_worst_m < POSE_TOLERANCE_M, "head follows the capsule body at warp x%d through the staging (%d frames, worst %.6f m)" % [WARP, _checked, _worst_m])
				_world.clock.warp = 0.0
				_world.start_mission("starship_leo")
				_step = "starship_start"
				_frames = 0
			elif _world.flight.sim.mission_time_s() > STAGING_MISSION_S and _staged_checks == 0:
				_check(false, "no staging seen by T+%.0f s" % STAGING_MISSION_S)
				return _finish()
		"starship_start":
			if _frames > 3000:
				_check(false, "Starship mission did not start")
				return _finish()
			if _starship_flying():
				_check(not _world.cockpit_active and _world.camera.mode != OrbitCamera.Mode.COCKPIT, "a Starship mission starts external")
				var jumps := _world.view_jump_count
				var refusals := _world.cockpit_refusal_count
				_world.toggle_cockpit()
				_check(not _world.cockpit_active and _world.view_jump_count == jumps and _world.cockpit_refusal_count == refusals + 1, "toggle on a Starship stays external and logs once")
				return _finish()
	return false


## Per frame: the cockpit camera's position against the scene graph (the eye node sits at the floating origin: the camera).
func _measure() -> void:
	if not _world.cockpit_active:
		return
	var eye := _eye_node()
	if eye == null:
		return
	var error := eye.global_position.length()  # views are placed relative to the camera position of this frame
	_worst_m = maxf(_worst_m, error)
	_checked += 1


func _starship_flying() -> bool:
	var sim := _world.flight.sim
	if not sim.is_active() or sim.body_count() == 0 or sim.controlled_index() < 0 or sim.controlled_index() >= sim.body_count():
		return false
	var stages: Array = sim.body_stages(sim.controlled_index())
	return not stages.is_empty() and String(stages[0].get("vehicle_id", "")).begins_with("starship")


## Updates until the head stops moving: the simulation snapshot trails the held clock for a few of them.
func _settle() -> void:
	var previous := Vector3.INF
	for i in 50:
		_world.flight.sim.wait_idle(5.0)
		_world.update_world()
		if _world.camera.position_gcrf_m.distance_to(previous) < 1.0e-9:
			return
		previous = _world.camera.position_gcrf_m


func _eye_node() -> Node3D:
	for view in _world.flight.views:
		if view.cockpit != null and view.visible:
			return view.cockpit.eye_node
	return null


func _pose_finite() -> bool:
	var p := _world.camera.position_gcrf_m
	return is_finite(p.x) and is_finite(p.y) and is_finite(p.z) and _world.camera.free_basis.is_finite()


## Two toggles with the clock held: the same head pose and two _view_jumped calls.
func _toggle_round_trip() -> void:
	_world.clock.warp = 0.0
	_world.camera.free_basis = _world.camera.free_basis  # (explicit: the pose below is the one of the held frame)
	_settle()
	var position := _world.camera.position_gcrf_m
	var basis := _world.camera.free_basis
	var jumps := _world.view_jump_count
	_world.toggle_cockpit()
	_check(not _world.cockpit_active and _world.camera.mode != OrbitCamera.Mode.COCKPIT, "toggle_cockpit leaves the cockpit")
	_settle()
	_world.toggle_cockpit()
	_check(_world.cockpit_active, "toggle_cockpit again re-enters it")
	_settle()
	_check(_world.view_jump_count == jumps + 2, "two toggles call _view_jumped twice (%d)" % (_world.view_jump_count - jumps))
	print("  round trip: position change %.6f m, basis equal %s" % [_world.camera.position_gcrf_m.distance_to(position), _world.camera.free_basis.is_equal_approx(basis)])
	_check(_world.camera.position_gcrf_m.distance_to(position) < POSE_TOLERANCE_M and _world.camera.free_basis.is_equal_approx(basis), "the same head pose after the round trip")
	_world.cockpit_camera.look(Vector2(5.0, 5.0))
	_world.camera.free_look_rad = Vector3.ZERO
	_world.update_world()
	_check(is_equal_approx(_world.cockpit_camera.look_yaw_rad, deg_to_rad(170.0)) and is_equal_approx(_world.cockpit_camera.look_pitch_rad, deg_to_rad(80.0)), "world look stays clamped")
	var key := InputEventKey.new()
	key.keycode = KEY_F12
	key.pressed = true
	_world._input(key)
	_check(_world.cockpit_camera.look_yaw_rad == 0.0 and _world.cockpit_camera.look_pitch_rad == 0.0, "F12 resets the look in the world")
