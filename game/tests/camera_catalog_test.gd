# Headless test of the left screen's external cameras (Task 7): CameraCatalog against a stub simulation (offered ids follow the
# bodies, `requires`, fall-out to the default, config-order cycling, mount pose = body pose * mount to 1 mm) and in a real
# falcon9_crew_dragon session through main.tscn (T-60 list, mount pose against the scene graph, the first stage's camera stays
# through the staging, ScreenCamera renders while the cabin is active and is switched by HotasButtons "camera_next").
# Needs the generated Dragon models and the built GDExtension.
# Run: godot --headless --path game --script res://tests/camera_catalog_test.gd
extends SceneTree

const POSE_TOLERANCE_M := 0.001  # plan Task 7
const STAGING_MISSION_S := 175.0
const WARP := 100.0
const TIMEOUT_S := 280.0

var _failures := 0
var _main: Node
var _world: World
var _step := ""
var _frames := 0
var _real_s := 0.0
var _staged_frames := 0
var _selected_before := ""


## A simulation with the getters CameraCatalog reads. `bodies`: [{"stages": [{id, vehicle_id, base_station_m}], "position": V3,
## "attitude": Quaternion, "cg": float, "capsule": Dictionary}]
class StubSim:
	extends RefCounted
	var bodies: Array[Dictionary] = []
	var active := true

	func is_active() -> bool:
		return active

	func body_count() -> int:
		return bodies.size()

	func body_stages(i: int) -> Array:
		return bodies[i]["stages"]

	func body_position_gcrf_m(i: int) -> Vector3:
		return bodies[i]["position"]

	func body_attitude_gcrf(i: int) -> Quaternion:
		return bodies[i]["attitude"]

	func body_cg_station_m(i: int) -> float:
		return bodies[i]["cg"]

	func capsule_state(i: int) -> Dictionary:
		return bodies[i].get("capsule", {})


func _initialize() -> void:
	_check_catalog_logic()
	_check_screen_logic()
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
	print("CAMERA CATALOG TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)
	return true


func _same(a: Array, b: Array) -> bool:
	return str(a) == str(b)


func _stack(nose_open := false) -> Dictionary:
	return {"stages": [
			{"id": "first_stage", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 0.0},
			{"id": "second_stage", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 47.7},
			{"id": "trunk", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 62.0},
			{"id": "capsule", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 65.25}],
			"position": Vector3(6.4e6, 1.0e5, -2.0e5), "attitude": Quaternion(Vector3(0.2, 0.7, 0.4).normalized(), 0.9), "cg": 21.5,
			"capsule": {"present": true, "nose_cone_open": nose_open}}


func _check_catalog_logic() -> void:
	var config := CockpitConfig.for_vehicle("crew_dragon")
	_check(config != null and config.cameras.size() == 5, "five [[camera]] entries in the Crew Dragon cabin")
	_check(_same(CameraCatalog.all_ids(config), ["chase", "stage1_aft", "stage2_engine", "dragon_nose", "dragon_trunk"]), "config order chase, stage1_aft, stage2_engine, dragon_nose, dragon_trunk")
	var sim := StubSim.new()
	sim.bodies.append(_stack())
	_check(_same(CameraCatalog.available(sim, config), ["chase", "stage1_aft", "stage2_engine", "dragon_trunk"]), "a closed-nose stack offers chase, stage1_aft, stage2_engine, dragon_trunk (no dragon_nose)")
	sim.bodies[0] = _stack(true)
	_check(CameraCatalog.available(sim, config).has("dragon_nose"), "dragon_nose is offered while nose_cone_open")
	_check(CameraCatalog.available(sim) == CameraCatalog.available(sim, config), "available(sim) uses the Crew Dragon cabin by default")
	# staging: the first stage flies on alone, the second stage + Dragon go on
	sim.bodies[0] = {"stages": [
			{"id": "second_stage", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 0.0},
			{"id": "trunk", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 14.3},
			{"id": "capsule", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 17.55}],
			"position": Vector3(6.5e6, 0.0, 0.0), "attitude": Quaternion.IDENTITY, "cg": 8.0, "capsule": {"present": true}}
	sim.bodies.append({"stages": [{"id": "first_stage", "vehicle_id": "falcon9_block5", "base_station_m": 0.0}],
			"position": Vector3(6.4e6, 0.0, 0.0), "attitude": Quaternion.IDENTITY, "cg": 15.0})
	_check(CameraCatalog.available(sim, config).has("stage1_aft"), "after staging stage1_aft stays while the first stage body exists")
	sim.bodies.remove_at(1)
	_check(not CameraCatalog.available(sim, config).has("stage1_aft"), "stage1_aft vanishes once the first stage body is gone")
	sim.active = false
	_check(CameraCatalog.available(sim, config).is_empty(), "nothing is offered without an active simulation")
	# mount pose = body pose * mount, to 1 mm
	sim.active = true
	sim.bodies.clear()
	var body := _stack()
	sim.bodies.append(body)
	var worst_m := 0.0
	for id: String in ["chase", "stage1_aft", "stage2_engine", "dragon_trunk"]:
		var entry := CameraCatalog.entry_of(id, config)
		var stage_base_m := 0.0
		for stage: Dictionary in body["stages"]:
			if stage["id"] == entry["stage"]:
				stage_base_m = stage["base_station_m"]
		var attitude := Basis(body["attitude"] as Quaternion)
		var mount := Vector3(entry["mount_m"][0], entry["mount_m"][1], entry["mount_m"][2])
		var expected: Vector3 = (body["position"] as Vector3) + attitude * (Vector3(stage_base_m - float(body["cg"]), 0.0, 0.0) + mount)
		var pose := CameraCatalog.mount_pose_gcrf(sim, id, config)
		worst_m = maxf(worst_m, pose.origin.distance_to(expected))
		var look := Vector3(entry["look"][0], entry["look"][1], entry["look"][2]).normalized()
		_check(absf(pose.basis.determinant() - 1.0) < 1.0e-9, "%s: a proper rotation" % id)
		_check((-pose.basis.z).distance_to(attitude * look) < 1.0e-9, "%s: -Z looks along attitude * look" % id)
	_check(worst_m < POSE_TOLERANCE_M, "mount pose = body position + attitude * (stage base - CG + mount) (worst %.9f m)" % worst_m)
	_check(CameraCatalog.mount_pose_gcrf(sim, "nonexistent", config) == Transform3D.IDENTITY, "an unknown id has the identity pose")


func _check_screen_logic() -> void:
	var config := CockpitConfig.for_vehicle("crew_dragon")
	var screen := ScreenCamera.new()
	var sim := StubSim.new()
	sim.bodies.append(_stack())
	screen.update(sim, config, null, Vector3.ZERO, 0.0, false)
	_check(screen.current_id == "chase" and screen.available().size() == 4, "ScreenCamera starts on chase with four cameras offered")
	var order: Array[String] = []
	for i in 5:
		screen.next()
		order.append(screen.current_id)
	_check(_same(order, ["stage1_aft", "stage2_engine", "dragon_trunk", "chase", "stage1_aft"]), "next() cycles in config order over the offered ids only (%s)" % [order])
	screen.select("dragon_trunk")
	_check(screen.current_id == "dragon_trunk", "select() takes an id")
	screen.select("nonsense")
	_check(screen.current_id == "dragon_trunk", "select() ignores an unknown id")
	sim.bodies[0]["stages"].remove_at(0)  # the first stage and the second... (first stage gone from the stack)
	screen.select("stage1_aft")
	screen.update(sim, config, null, Vector3.ZERO, 0.0, false)
	_check(screen.current_id == "chase", "the current camera falling out switches to chase")
	screen.free()


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
			_step = "start"
			_frames = 0
		"start":
			if _world.flight.sim.is_active() and _world.cockpit_active and _world.flight.views.size() > 0 and _frames > 10:
				_session_checks()
				_step = "staging"
				_frames = 0
			elif _frames > 3000:
				_check(false, "Dragon mission did not start in the cockpit")
				return _finish()
		"staging":
			_world.clock.warp = WARP
			if _world.flight.sim.body_count() > 1:
				_staged_frames += 1
				if _staged_frames > 5:
					_world.clock.warp = 0.0
					var ids := CameraCatalog.available(_world.flight.sim)
					_check(ids.has("stage1_aft") and ids.has("chase"), "after the staging (%d bodies, T+%.0f s) stage1_aft and chase are offered (%s)" % [_world.flight.sim.body_count(), _world.flight.sim.mission_time_s(), ids])
					_check(_world.screen_camera.current_id == _selected_before, "the screen stays on its camera through the staging (%s)" % _selected_before)
					_world.screen_camera.select("stage1_aft")
					_world.update_world()
					_check(_world.screen_camera.current_id == "stage1_aft", "the first stage's camera can be selected after the staging")
					return _finish()
			elif _world.flight.sim.mission_time_s() > STAGING_MISSION_S:
				_check(false, "no staging seen by T+%.0f s" % STAGING_MISSION_S)
				return _finish()
	return false


func _session_checks() -> void:
	var sim := _world.flight.sim
	var ids := CameraCatalog.available(sim)
	_check(_same(ids, ["chase", "stage1_aft", "stage2_engine", "dragon_trunk"]), "at the start chase, stage1_aft, stage2_engine, dragon_trunk are offered, not dragon_nose (%s)" % [ids])
	# mount pose against the scene graph: the capsule stage node carries dragon_trunk (the node sits at the floating origin frame)
	_world.clock.warp = 0.0
	for i in 20:
		sim.wait_idle(5.0)
		_world.update_world()
	var stage_node: Node3D = null
	for view in _world.flight.views:
		if view.cockpit != null:
			stage_node = view.cockpit.get_parent() as Node3D
	_check(stage_node != null, "the cabin hangs on the capsule stage node")
	if stage_node != null:
		var entry := CameraCatalog.entry_of("dragon_trunk")
		var mount := Vector3(entry["mount_m"][0], entry["mount_m"][1], entry["mount_m"][2])
		var pose := CameraCatalog.mount_pose_gcrf(sim, "dragon_trunk")
		var node_render: Vector3 = stage_node.global_transform * mount  # the stage node's axes are the capsule frame (like the eye node)
		var expected_render := Frames.to_render(pose.origin - _world.camera.position_gcrf_m)
		_check(node_render.distance_to(expected_render) < POSE_TOLERANCE_M, "dragon_trunk mount = capsule stage node * mount within 1 mm (%.6f m)" % node_render.distance_to(expected_render))
	# ScreenCamera in the live World: renders while the cockpit is the view, stops outside it, camera_next steps through the list
	var screen := _world.screen_camera
	var rendered := screen.render_count
	for i in 5:
		_world.update_world()
		OS.delay_msec(40)
	_check(screen.render_count > rendered, "the feed renders while the cockpit is active (%d -> %d)" % [rendered, screen.render_count])
	_check(screen.viewport.render_target_update_mode != SubViewport.UPDATE_DISABLED or screen.render_count > rendered, "the feed viewport is updating")
	var first := screen.current_id
	_world.hotas_buttons.set_reader(func(role: String) -> bool: return role == "camera_next")
	_world.update_world()
	_world.hotas_buttons.set_reader(func(_role: String) -> bool: return false)
	_world.update_world()
	_check(screen.current_id != first and screen.available().has(screen.current_id), "camera_next steps to the next camera (%s -> %s)" % [first, screen.current_id])
	_selected_before = screen.current_id
	_world.toggle_cockpit()
	rendered = screen.render_count
	for i in 5:
		_world.update_world()
		OS.delay_msec(40)
	_check(screen.render_count == rendered and screen.viewport.render_target_update_mode == SubViewport.UPDATE_DISABLED, "no feed render outside the cockpit")
	_world.toggle_cockpit()
