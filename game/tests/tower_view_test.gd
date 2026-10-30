# Headless test of TowerView with the Pad B lattice and chopstick (needs data/models/starbase_olp2 from
# tools/structures/build_models.py; without them the model cases are skipped and only the fallback is checked).
# (The headless renderer keeps no MultiMesh transforms: instance counts are read, layout comes from LatticeBuilder.)
# Run: godot --headless --path game --script res://tests/tower_view_test.gd
extends SceneTree

const MODEL_SITE := "starbase_olp1"
const OTHER_SITE := "lc39a"  ## keeps its boxes
const STATES: Array[Dictionary] = [
	{},
	{"opening_rad": 0.113, "carriage_offset_m": 0.0},
	{"opening_rad": 0.70, "carriage_offset_m": 1.25},
	{"opening_rad": 0.0, "carriage_offset_m": -0.4},
]

var _failures := 0


func _initialize() -> void:
	var models := StructureModels.load_for(MODEL_SITE)
	_check(StructureModels.load_for(OTHER_SITE).is_empty(), "a site other than the prototype's gets no models")
	var boxes := _view(OTHER_SITE)
	_check(_count(boxes, "MeshInstance3D") > 0 and _count(boxes, "MultiMeshInstance3D") == 0, "other site: boxes only")
	if models.is_empty():
		print("  (no models: lattice cases skipped; run python tools/structures/build_models.py)")
		var fallback := _view(MODEL_SITE)
		_check(_count(fallback, "MultiMeshInstance3D") == 0, "without models the prototype site keeps its boxes")
	else:
		_check_lattice(models, boxes)
	print("TOWER VIEW TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check_lattice(models: Dictionary, boxes: TowerView) -> void:
	var view := _view(MODEL_SITE)
	var sections: Array = models["detail"]["tower.sections"]
	var lattice := view.get_node("Lattice")
	_check(lattice.get_child_count() == sections.size(), "one node per section")
	var top_m := 0.0
	var instances := 0
	for i in range(sections.size()):
		var section: Dictionary = sections[i]
		var node := lattice.get_node("section_%d" % i)
		var panels := maxi(1, roundi(float(section["height_m"]) / float(section["pitch_m"])))
		var expected := panels * 3 + (1 if i == sections.size() - 1 else 0)  # 2 diagonals + 1 horizontal, top cap
		_check((node.get_node("legs") as MultiMeshInstance3D).multimesh.instance_count == 4, "section %d: 4 legs" % i)
		for face in range(4):
			var multimesh := (node.get_node("face_%d" % face) as MultiMeshInstance3D).multimesh
			_check(multimesh.instance_count == expected, "section %d face %d: %d braces" % [i, face, expected])
			instances += multimesh.instance_count
		var half := 0.5 * float(section["face_width_m"]) - 0.5 * 1.6  # leg centre line: profiles.leg size_m
		for member: Array in LatticeBuilder.face_members(section, half, i == sections.size() - 1):
			top_m = maxf(top_m, node.position.y + maxf(member[0].y, member[1].y))
		_check((node.get_node("shadow_proxy") as MeshInstance3D).cast_shadow
				== GeometryInstance3D.SHADOW_CASTING_SETTING_SHADOWS_ONLY, "section %d: shadow proxy" % i)
		_check((node.get_node("face_0") as MultiMeshInstance3D).cast_shadow
				== GeometryInstance3D.SHADOW_CASTING_SETTING_OFF, "section %d: members cast no shadow" % i)
	var detail_height: float = models["detail"]["tower"]["height_m"]
	_check(absf(top_m - detail_height) < 0.01 * detail_height,
			"lattice top %.2f m = detail height %.2f m" % [top_m, detail_height])
	print("  lattice: %d sections, %d brace instances" % [sections.size(), instances])

	var again := _view(MODEL_SITE)
	again.build(_geometry())
	_check(_count(again, "MultiMeshInstance3D") == _count(view, "MultiMeshInstance3D"), "second build: no duplicates")
	_check(again.get_child_count() == view.get_child_count(), "second build: same child count")

	# Arm pivots follow the physics state exactly like the boxes' pivots.
	_check(_pivot(view, 0).get_node_or_null("Chopstick") != null, "+across arm is the chopstick")
	_check(_pivot(boxes, 0).get_node_or_null("Chopstick") == null, "box path has no chopstick")
	var second_arm := _pivot(view, 1).get_child(0) as MeshInstance3D
	_check(second_arm != null and second_arm.mesh is BoxMesh, "-across arm stays a box")
	for state in STATES:
		view.update_state(state)
		boxes.update_state(state)
		for arm in range(2):
			var a := _pivot(view, arm)
			var b := _pivot(boxes, arm)
			_check(a.transform.is_equal_approx(b.transform), "state %s arm %d: pivot equals the box path" % [state, arm])
	# The chopstick's hinge is the pivot origin and it points along the pivot's forward axis.
	var chopstick := _pivot(view, 0).get_node("Chopstick")
	var bounds := AABB()
	for part in ["truss", "rail"]:
		var mesh_node := chopstick.get_node(part) as MeshInstance3D
		bounds = bounds.merge(chopstick.transform * mesh_node.get_aabb()) if bounds.has_surface() else \
				chopstick.transform * mesh_node.get_aabb()
	_check(absf(bounds.position.z) < 0.05 and absf(bounds.end.z - 26.0) < 0.05,
			"chopstick: hinge at 0, tip at 26 m along +Z")
	_check(absf(bounds.end.y) < 0.05 and absf(bounds.size.y - 8.75) < 0.05,
			"chopstick: top rails at Y = 0, 8.75 m deep below")
	_check(absf(bounds.size.x - 3.0) < 0.05, "chopstick: 3 m wide")


func _pivot(view: TowerView, arm: int) -> Node3D:
	return view.get_child(view.get_child_count() - 2 + arm) as Node3D


func _view(site_id: String) -> TowerView:
	var view := TowerView.new()
	view.site_id = site_id
	root.add_child(view)
	view.build(_geometry())
	return view


func _count(node: Node, class_name_: String) -> int:
	return node.find_children("*", class_name_, true, false).size()


## starbase_olp1's tower as FlightSim.site_tower reports it (site frame), hand-computed.
func _geometry() -> Dictionary:
	var azimuth := deg_to_rad(315.0)
	var toward := Vector3(sin(azimuth), cos(azimuth), 0.0)
	var across := Vector3(-toward.y, toward.x, 0.0)
	var up := Vector3(0.0, 0.0, 1.0)
	var centre := toward * 30.0 + up * 73.0
	return {
		"present": true,
		"origin_itrf_m": Vector3(-5.5e6, -0.7e6, 2.8e6),
		"east": Vector3(0.0, 1.0, 0.0), "north": Vector3(0.0, 0.0, 1.0), "up": Vector3(1.0, 0.0, 0.0),
		"tower_direction": toward, "across": across,
		"tower_center_m": centre, "tower_size_m": Vector3(12.0, 12.0, 146.0),
		"mount_height_m": 20.0, "mount_inner_radius_m": 5.5, "mount_outer_radius_m": 13.0,
		"arm_length_m": 36.0, "arm_width_m": 2.0, "arm_thickness_m": 2.5, "arm_open_rad": deg_to_rad(40.0),
		"hinge_plus_m": toward * 24.0 + across * 2.8 + up * 45.0,
		"hinge_minus_m": toward * 24.0 - across * 2.8 + up * 45.0,
	}


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
