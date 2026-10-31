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
	var rows := 0
	for i in range(sections.size()):
		var section: Dictionary = sections[i]
		var node := lattice.get_node("section_%d" % i)
		var panels := maxi(1, roundi(float(section["height_m"]) / float(section["pitch_m"])))
		rows += panels
		var expected := panels * 3 + (1 if i == sections.size() - 1 else 0)  # 2 diagonals + 1 horizontal, top cap
		_check((node.get_node("legs") as MultiMeshInstance3D).multimesh.instance_count == 4, "section %d: 4 legs" % i)
		for face in range(4):
			var multimesh := (node.get_node("face_%d" % face) as MultiMeshInstance3D).multimesh
			_check(multimesh.instance_count == expected, "section %d face %d: %d braces" % [i, face, expected])
			instances += multimesh.instance_count
		var leg_size: float = models["profiles"]["leg"].get_aabb().size.x
		var half := LatticeBuilder.leg_line_m(float(section["face_width_m"]), leg_size)
		for member: Array in LatticeBuilder.face_members(section, half, i == sections.size() - 1):
			top_m = maxf(top_m, node.position.y + maxf(member[0].y, member[1].y))
		_check_hand_over(node, section, i, sections.size(), LatticeBuilder.total_height_m(models["detail"]))
		_check(node.get_node_or_null("shadow_proxy") == null, "section %d: no shadow proxy (the Sun casts no shadows)" % i)
		_check((node.get_node("face_0") as MultiMeshInstance3D).cast_shadow
				== GeometryInstance3D.SHADOW_CASTING_SETTING_OFF, "section %d: members cast no shadow" % i)
	_check(rows >= 34 and rows <= 38, "about 35 brace rows over the tower (pitch ~4 m): %d" % rows)
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
		var part_bounds: AABB = chopstick.transform * mesh_node.get_aabb()
		bounds = bounds.merge(part_bounds) if bounds.has_surface() else part_bounds
	var arm: Dictionary = models["detail"]["chopstick"]
	_check(absf(bounds.position.z) < 0.05 and absf(bounds.end.z - float(arm["length_m"])) < 0.05,
			"chopstick: hinge at 0, tip at %.1f m along +Z" % float(arm["length_m"]))
	_check(absf(bounds.end.y) < 0.05 and absf(bounds.size.y - float(arm["depth_m"])) < 0.05,
			"chopstick: top rails at Y = 0, %.2f m deep below" % float(arm["depth_m"]))
	_check(absf(bounds.size.x - float(arm["width_m"])) < 0.05, "chopstick: %.1f m wide" % float(arm["width_m"]))
	var paint_material := (chopstick.get_node("truss") as MeshInstance3D).material_override as ShaderMaterial
	var paint := paint_material.get_shader_parameter("albedo") as Color
	_check(paint.v < 0.1, "chopstick: matte black paint (value %.3f)" % paint.v)


## Tiers of one section: near MultiMeshes up to NEAR_END_M, the merged mesh to MID_END_M, the lattice shader beyond;
## decks / column bundle / roof cap / foot ring as dark solids.
func _check_hand_over(node: Node, section: Dictionary, index: int, count: int, total_m: float) -> void:
	var panels := LatticeBuilder.panel_count(section)
	var decks := node.get_node("decks") as MultiMeshInstance3D
	var expected := panels + 1 + (2 if index == count - 1 else 0) + (4 if index == 0 else 0)
	_check(decks.multimesh.instance_count == expected, "section %d: %d solids (decks, column, roof, ring)" % [index, expected])
	_check((node.get_node_or_null("roof") != null) == (index == count - 1), "section %d: pyramid frame only on top" % index)
	var near := node.get_node("legs") as GeometryInstance3D
	var mid := node.get_node("mid") as GeometryInstance3D
	var far := node.get_node("far") as GeometryInstance3D
	_check(near.visibility_range_begin == 0.0 and near.visibility_range_end == LatticeBuilder.NEAR_END_M,
			"section %d: members up to %.0f m" % [index, LatticeBuilder.NEAR_END_M])
	_check(mid.visibility_range_begin == LatticeBuilder.NEAR_END_M and mid.visibility_range_end == LatticeBuilder.MID_END_M,
			"section %d: merged mesh %.0f .. %.0f m" % [index, LatticeBuilder.NEAR_END_M, LatticeBuilder.MID_END_M])
	_check(far.visibility_range_begin == LatticeBuilder.MID_END_M and far.visibility_range_end == 0.0,
			"section %d: lattice shader beyond %.0f m" % [index, LatticeBuilder.MID_END_M])
	var far_material := far.material_override as ShaderMaterial
	_check(absf(float(far_material.get_shader_parameter("panel_m")) * panels - float(section["height_m"])) < 1.0e-4,
			"section %d: far shader rows = the section's brace rows" % index)
	_check((node.get_node("face_0") as MultiMeshInstance3D).material_override is ShaderMaterial, "section %d: steel" % index)
	# Every tier node measures its range from the tower's middle (one swap for the whole tower); local transforms only
	# (the global ones are Earth-fixed, ~6.4e6 m: float32 noise).
	var middle := Vector3(0.0, 0.5 * total_m, 0.0)
	for child in node.get_children():
		var geometry := child as GeometryInstance3D
		var centre: Vector3 = (node as Node3D).transform * (geometry.transform * geometry.custom_aabb.get_center())
		_check(geometry.custom_aabb.has_volume() and centre.is_equal_approx(middle),
				"section %d %s: range centre = the tower's middle" % [index, child.name])


func _pivot(view: TowerView, arm: int) -> Node3D:
	return view.arm_pivot(arm)


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
