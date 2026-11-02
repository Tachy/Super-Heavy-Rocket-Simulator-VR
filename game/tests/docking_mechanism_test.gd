# The Crew Dragon's docking mechanism view against the IDD numbers and the closed nose. Needs the generated models (python tools/structures/build_models.py --vehicle crew_dragon).
# godot --headless --path game --script res://tests/docking_mechanism_test.gd
extends SceneTree

const RING_TOLERANCE_M := 0.005  # Task 6 brief
const ATTACH_TOLERANCE_M := 0.001
const TILT_LIMIT_DEG := 4.0  # IDSS envelope's misalignment, Task 6 brief; the ring's conforming range is also checked

var _failures := 0
var _view: DockingMechanismView


func _initialize() -> void:
	VehicleModels.clear_cache()
	var models := VehicleModels.load_for("crew_dragon")
	if models.is_empty() or not models.has("mechanism"):
		_check(false, "crew_dragon: mechanism models generated (%s)" % VehicleModels.fallback_reason("crew_dragon"))
		quit(1)
		return
	_view = DockingMechanismView.new()
	root.add_child(_view)
	_view.build(models, func(_part: String) -> Material: return null)
	var idss: Dictionary = models["manifest"]["idss"]
	_check_port(idss)
	_check_retracted_inside_nose(idss)
	_check_extension(idss)
	_check_hooks(idss)
	_check_pins(idss)
	_check_struts(idss)
	_view.free()
	print("DOCKING MECHANISM TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		print("FAIL: ", what)
	else:
		print("ok: ", what)


func _check_port(idss: Dictionary) -> void:
	_check(is_equal_approx(_view.position.x, float(idss["port_position_m"][0])), "the view's origin lies on the config's port (%.3f); test_idss_view_values.cpp, test_dragon_spec.py: on the drawn flange" % _view.position.x)


## Every vertex of every part, retracted with hooks open and closed: inside the closed nose's inner surface and the seal ring, above the recess floor, inside the tunnel below the flange.
func _check_retracted_inside_nose(capsule: Dictionary) -> void:
	_view.set_state(0.0, Vector2.ZERO, 0.0)
	var worst := INF
	var worst_where := ""
	for f in [0.0, 1.0]:
		_view.set_state(0.0, Vector2.ZERO, f)
		for mesh_instance: MeshInstance3D in _view.find_children("*", "MeshInstance3D", true, false):
			var to_view := _to_view(mesh_instance)
			for v: Vector3 in mesh_instance.mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]:
				var p := _view.transform * (to_view * v)
				var clearance := _nose_clearance(p, capsule)
				if clearance < worst:
					worst = clearance
					worst_where = "%s x %.3f r %.3f" % [mesh_instance.name, p.x, Vector2(p.y, p.z).length()]
	_check(worst > 0.0, "retracted: all parts inside the closed nose, least clearance %.3f m at %s" % [worst, worst_where])
	var lowest := INF
	for i in _view.strut_count():
		lowest = minf(lowest, _view.strut_base_m(i).x)
	_check(is_equal_approx(lowest, float(capsule["strut_base_x_m"])), "struts stand on the recess floor (%.3f m below the port)" % -lowest)


func _to_view(node: Node3D) -> Transform3D:
	var t := node.transform
	var parent := node.get_parent() as Node3D
	while parent != null and parent != _view:
		t = parent.transform * t
		parent = parent.get_parent() as Node3D
	return t


## Clearance from the nose's inside: under the seal ring's plane only what lies beyond the seal's inner edge can touch it; above that plane the opening of the seal and the inner cone bound the radius,
## its flat end the station. p in body stations. (The view's Transform: the mechanism's origin at the port.)
func _nose_clearance(p: Vector3, capsule: Dictionary) -> float:
	var r := Vector2(p.y, p.z).length()
	var seal: Array = capsule["nose_seal_m"]
	var cone: Array = capsule["nose_inner_cone_m"]
	if p.x < float(seal[0]):
		return float(seal[0]) - p.x if r > float(seal[1]) else float(seal[1]) - r + float(seal[0]) - p.x
	var a: Array = cone[0]
	var b: Array = cone[1]
	var cone_r := lerpf(float(a[1]), float(b[1]), clampf((p.x - float(a[0])) / (float(b[0]) - float(a[0])), 0.0, 1.0))
	return minf(minf(float(seal[1]), cone_r) - r, float(cone[-1][0]) - p.x)


func _check_extension(idss: Dictionary) -> void:
	var full := float(idss["ring_extension_m"])
	_view.set_state(full, Vector2.ZERO, 0.0)
	_check(absf(_view.ring_plane_x_m() - full) <= RING_TOLERANCE_M, "full extension: ring plane %.4f m above the port (physics %.3f)" % [_view.ring_plane_x_m(), full])
	var ring := _view.get_node("ring") as Node3D
	_check(absf(ring.position.y) + absf(ring.position.z) < 1e-6, "the ring stays on the port axis")
	_view.set_state(0.0, Vector2.ZERO, 0.0)
	_check(absf(_view.ring_plane_x_m() + float(idss["scs_recess_m"])) < 1e-6, "retracted: the SCS plane 6 mm below the HCS plane")
	var petal := _view.get_node("ring/petal_0") as Node3D
	_check(petal != null and _view.get_node("ring/petal_2") != null, "three petals on the ring")
	var direction: Vector3 = _view.transform.basis * (petal.basis * Vector3(0.0, 1.0, 0.0))
	var expected := _direction(idss, float(idss["petal_theta_deg"][0]))
	_check(direction.distance_to(expected) < 1e-5, "petal 0 at the IDD's angle")


func _direction(idss: Dictionary, theta: float) -> Vector3:
	var up := DockingMechanismView._vec(idss["port_up"])
	var axis := DockingMechanismView._vec(idss["port_axis"])
	var right := up.cross(axis)
	return up * cos(deg_to_rad(theta)) + right * sin(deg_to_rad(theta))


## 12 hooks at theta 15 + 30 k, radius 663.5; closing moves the leading edge from the IDD's open to its closed station; never above the 18 mm envelope.
func _check_hooks(idss: Dictionary) -> void:
	var count := 0
	var worst_hook := 0.0
	for k in 12:
		var hook := _view.get_node_or_null("hook_%d" % k) as Node3D
		if hook == null:
			continue
		count += 1
		var theta := 15.0 + 30.0 * k  # IDD Fig 3.2.3-1, typed here on purpose: the test's own copy of the document
		var e := _direction(idss, theta)
		var along := Vector3.RIGHT.cross(e)
		var pivot_of: Array = idss["hook_pivot_m"]
		var want_at := e * 0.6635 + along * float(pivot_of[0]) + Vector3.RIGHT * float(pivot_of[1])
		worst_hook = maxf(worst_hook, hook.position.distance_to(want_at))
	_check(count == 12, "12 hooks")
	_check(worst_hook < 1e-4, "12 hooks on radius 663.5 mm at theta 15 + 30 k deg (worst %.5f m)" % worst_hook)
	var outline: Array = idss["hook_active_mm"]
	for f: float in [0.0, 1.0]:
		var t := _view.hook_transform(0, f)
		_check(t.basis.is_conformal(), "hook %.0f: rigid pose" % f)
		var e0 := _direction(idss, 15.0)
		var along0 := Vector3.RIGHT.cross(e0)
		var edge := -INF
		var top := -INF
		for point: Array in outline:
			var p := t * Vector3(float(point[1]) * 0.001, 0.0, float(point[0]) * 0.001) - e0 * 0.6635
			edge = maxf(edge, p.dot(along0))
			top = maxf(top, p.x)
		var want := float(idss["hook_active_edge_open_m"]) if f == 0.0 else float(idss["hook_active_edge_closed_m"])
		_check(absf(edge - want) < 0.0015, "hook at %.0f: leading edge %.4f m from the latch centre (IDD %.4f)" % [f, edge, want])
		_check(top <= 0.018 + 1e-4, "hook at %.0f: top %.4f m above the HCS plane, IDD envelope 0.018" % [f, top])
		_check(top >= 0.0160, "hook at %.0f: top near the IDD's 17.5-17.8 mm" % f)


func _check_pins(idss: Dictionary) -> void:
	var r0 := 0.5 * float(idss["pin_circle_diameter_m"])
	for i in 2:
		var pin := _view.get_node("guide_pin_%d" % i) as Node3D
		var e := _direction(idss, float(idss["pin_theta_deg"][i]))
		_check(pin.position.distance_to(e * r0) < 1e-5, "guide pin %d on the 1375 mm circle at theta %.2f" % [i, float(idss["pin_theta_deg"][i])])


## The struts reach the ring at every state: retracted, full, the 4 deg envelope and the ring's 6 deg range; the three tubes keep overlapping.
func _check_struts(idss: Dictionary) -> void:
	var full := float(idss["ring_extension_m"])
	var limit := deg_to_rad(float(idss["ring_tilt_max_deg"]))
	var tube := float(idss["strut_tube_length_m"])
	var limit4 := deg_to_rad(TILT_LIMIT_DEG)
	_check(_view.strut_count() == 6, "six struts")
	var states := [[0.0, Vector2.ZERO], [full, Vector2.ZERO], [full, Vector2(limit4, 0.0)], [full, Vector2(0.0, -limit4)], [full, Vector2(limit4, limit4)], [full, Vector2(limit, 0.0)], [0.5 * full, Vector2(-limit4, limit4)]]
	var worst := 0.0
	var longest := 0.0
	var shortest := INF
	for s: Array in states:
		_view.set_state(float(s[0]), s[1] as Vector2, 0.0)
		for i in _view.strut_count():
			worst = maxf(worst, _view.strut_rod_end_m(i).distance_to(_view.strut_top_m(i)))
			worst = maxf(worst, _view.strut_barrel_start_m(i).distance_to(_view.strut_base_m(i)))
			var length := _view.strut_base_m(i).distance_to(_view.strut_top_m(i))
			longest = maxf(longest, length)
			shortest = minf(shortest, length)
	_check(worst < ATTACH_TOLERANCE_M, "struts stay attached at both ends up to the tilt limits (worst %.5f m)" % worst)
	_check(longest <= 3.0 * tube, "the three tubes cover the longest stroke (%.3f m, tubes %.3f m)" % [longest, 3.0 * tube])
	_check(shortest >= tube, "retracted they fit (%.3f m, one tube %.3f m)" % [shortest, tube])
	_view.set_state(full, Vector2(limit4, 0.0), 0.0)
	var ring := _view.get_node("ring") as Node3D
	var tilted := rad_to_deg((ring.basis * Vector3.RIGHT).angle_to(Vector3.RIGHT))
	_check(absf(tilted - TILT_LIMIT_DEG) < 1e-3, "the tilt command is the ring's tilt (%.2f deg)" % tilted)
