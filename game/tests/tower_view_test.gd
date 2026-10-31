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
	var floors := 0
	var silhouette_m := 0.0
	var plates_lo_m := INF
	var plates_hi_m := 0.0
	var plates := 0
	var deck_top_m := 0.0
	var plinth_top_m := 0.0
	var leg_size: float = models["profiles"]["leg"].get_aabb().size.x
	var brace_size: float = models["profiles"]["brace"].get_aabb().size.x
	_check(absf(leg_size - 2.2) < 0.01, "leg 2.2 m (%.3f)" % leg_size)
	_check(absf(brace_size - 0.5) < 0.01, "brace 0.5 m (%.3f)" % brace_size)
	for i in range(sections.size()):
		var section: Dictionary = sections[i]
		var node := lattice.get_node("section_%d" % i)
		var panels := maxi(1, roundi(float(section["height_m"]) / float(section["pitch_m"])))
		floors += LatticeBuilder.floor_count(models["detail"], section)
		var expected := panels * 3 + (1 if i == sections.size() - 1 else 0)  # 2 diagonals + 1 horizontal, top cap
		_check((node.get_node("legs") as MultiMeshInstance3D).multimesh.instance_count == 4, "section %d: 4 legs" % i)
		for face in range(4):
			var multimesh := (node.get_node("face_%d" % face) as MultiMeshInstance3D).multimesh
			_check(multimesh.instance_count == expected, "section %d face %d: %d braces" % [i, face, expected])
			instances += multimesh.instance_count
		var half := LatticeBuilder.leg_line_m(float(section["face_width_m"]), leg_size)
		var is_top := i == sections.size() - 1
		for member: Array in LatticeBuilder.face_members(section, half, is_top):
			top_m = maxf(top_m, node.position.y + maxf(member[0].y, member[1].y))
		var detail: Dictionary = models["detail"]
		var darks := LatticeBuilder.dark_solids(detail, section, node.position.y, half, is_top)
		var clads := LatticeBuilder.clad_solids(detail, section, node.position.y, half, leg_size, i == 0, is_top)
		for t in darks:
			silhouette_m = maxf(silhouette_m, node.position.y + _bounds(t).end.y)
		for t in clads:
			var b := _bounds(t)
			silhouette_m = maxf(silhouette_m, node.position.y + b.end.y)
			if b.get_center().z < -half - 0.1:  # a plate on the pad-side face
				plates += 1
				plates_lo_m = minf(plates_lo_m, node.position.y + b.position.y)
				plates_hi_m = maxf(plates_hi_m, node.position.y + b.end.y)
			elif i == 0 and b.size.y > 10.0 and absf(b.get_center().x) < 0.01:
				plinth_top_m = node.position.y + b.end.y
		if is_top:
			for t in darks:  # the roof deck plate: the widest flat box
				var b := _bounds(t)
				if b.size.x > 12.5 and b.size.z > 12.5:
					deck_top_m = node.position.y + b.end.y
			for t in LatticeBuilder.top_frame_members(detail, section, node.position.y, half):
				silhouette_m = maxf(silhouette_m, node.position.y + t.origin.y)
			var roof_node := node.get_node("roof") as MultiMeshInstance3D
			_check(roof_node.multimesh.instance_count == 4, "top: four frame members under the roof deck, no pyramid")
		_check_hand_over(node, models["detail"], section, i, sections.size(), LatticeBuilder.total_height_m(models["detail"]))
		_check(node.get_node_or_null("shadow_proxy") == null, "section %d: no shadow proxy (the Sun casts no shadows)" % i)
		_check((node.get_node("face_0") as MultiMeshInstance3D).cast_shadow
				== GeometryInstance3D.SHADOW_CASTING_SETTING_OFF, "section %d: members cast no shadow" % i)
	var detail_lattice: float = models["detail"]["tower"]["lattice_height_m"]
	_check(absf(top_m - 139.0) < 0.2 and absf(top_m - detail_lattice) < 0.01, "lattice panels end at %.2f m (139)" % top_m)
	_check(floors == 26, "26 floors: %d" % floors)
	_check(absf(top_m / floors - 5.3) < 0.1, "floor pitch %.3f m (5.3)" % (top_m / floors))
	_check(absf(deck_top_m - 144.0) < 0.01, "roof deck top at %.2f m (144)" % deck_top_m)
	_check(absf(plinth_top_m - 12.2) < 0.01, "plinth top at %.2f m (12.2)" % plinth_top_m)
	_check(plates > 0 and absf(plates_lo_m - 12.2) < 0.15 and absf(plates_hi_m - 79.0) < 0.15,
			"cladding %d plates on the pad-side face from %.2f to %.2f m (12.2 .. 79)" % [plates, plates_lo_m, plates_hi_m])
	_check(silhouette_m <= 148.0, "silhouette top %.2f m <= 148" % silhouette_m)
	print("  lattice: %d sections, %d brace instances" % [sections.size(), instances])

	var again := _view(MODEL_SITE)
	again.build(_geometry())
	_check(_count(again, "MultiMeshInstance3D") == _count(view, "MultiMeshInstance3D"), "second build: no duplicates")
	_check(again.get_child_count() == view.get_child_count(), "second build: same child count")

	# Arm pivots follow the physics state exactly like the boxes' pivots.
	for arm_index in range(2):
		_check(_pivot(view, arm_index).get_node_or_null("Chopstick") != null, "arm %d is a chopstick" % arm_index)
	_check(_pivot(boxes, 0).get_node_or_null("Chopstick") == null, "box path has no chopstick")
	var plus_arm := _pivot(view, 0).get_node("Chopstick") as Node3D
	var minus_arm := _pivot(view, 1).get_node("Chopstick") as Node3D
	_check(plus_arm.basis.determinant() > 0.0 and minus_arm.basis.determinant() < 0.0, "-across arm mirrored")
	var geometry := _geometry()
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	for state in STATES:
		view.update_state(state)
		boxes.update_state(state)
		var opening: float = state.get("opening_rad", geometry["arm_open_rad"])
		var offset: float = state.get("carriage_offset_m", 0.0)
		for arm_index in range(2):
			var a := _pivot(view, arm_index)
			var b := _pivot(boxes, arm_index)
			_check(a.transform.is_equal_approx(b.transform), "state %s arm %d: pivot = box path" % [state, arm_index])
			# TowerGeometry::arm_direction in site axes; the pivot's forward axis in render axes (+Z).
			var side := 1.0 if arm_index == 0 else -1.0
			var expected := Frames.to_render(-toward * cos(opening) + across * (side * sin(opening)))
			var angle := rad_to_deg(a.basis.z.normalized().angle_to(expected))
			_check(angle < 0.1, "state %s arm %d: %.3f deg from arm_direction" % [state, arm_index, angle])
		# The state dictionaries stand for FlightSim.tower_state(); the carriage's height is the static hinge height (physics
		# geometry) plus the state's offset: a live catch flight is not started here.
		var carriage := view.get_node("CarriageMount") as Node3D
		var height_m := Frames.to_render(Vector3(0.0, 0.0, 1.0)).dot(carriage.position)
		var want_m: float = float(geometry["hinge_plus_m"].z) + offset
		_check(absf(height_m - want_m) < 0.01, "state %s: carriage %.3f m = height + offset %.3f m" % [state, height_m, want_m])
		var cable := view.get_node("cable_0") as MeshInstance3D
		var cable_top_m: float = Frames.to_render(Vector3(0.0, 0.0, 1.0)).dot(cable.position) + 0.5 * cable.basis.y.length()
		_check(absf(cable_top_m - float(models["detail"]["tower.roof"]["block_base_m"])) < 0.01, "cable reaches the roof block")
	# The closed arm: with the pivot at the hinge the +across arm's inner face and its reach against the catch axis.
	var c: Dictionary = models["detail"]["chopstick"]
	var lateral: float = float(geometry["hinge_plus_m"].dot(across))
	var half_gap: float = lateral - 0.5 * float(geometry["arm_width_m"])
	_check(absf(lateral - float(c["inner_face_y_m"]) - half_gap) < 0.05, "inner face at the physics half gap %.2f m" % half_gap)
	var hinge_to_axis: float = float(geometry["hinge_plus_m"].dot(toward))
	_check(float(c["length_m"]) >= hinge_to_axis, "closed arm reaches the catch axis (%.1f m >= %.1f m)" % [c["length_m"], hinge_to_axis])
	# Bounds of the model in the pivot's frame: hinge at 0, deck plane at Y = 0, the pad under it.
	var deck := plus_arm.get_node("strip") as MeshInstance3D
	var deck_bounds: AABB = plus_arm.transform * deck.get_aabb()
	_check(absf(deck_bounds.end.y) < 0.01, "strip top at the arm top plane (%.3f m)" % deck_bounds.end.y)
	_check(absf(deck_bounds.position.z - float(c["bearing_start_x_m"])) < 0.5 and absf(deck_bounds.end.z - float(c["plate_end_x_m"])) < 0.5,
			"strip on the bearing piece (%.1f .. %.1f m)" % [deck_bounds.position.z, deck_bounds.end.z])
	var plate_bounds: AABB = plus_arm.transform * (plus_arm.get_node("bearing") as MeshInstance3D).get_aabb()
	_check(plate_bounds.size.x < 0.2, "bearing plate is flat and along the arm (%.3f m thick)" % plate_bounds.size.x)
	_check(plus_arm.get_node_or_null("deck") == null, "open frame: no deck slab")
	var truss_bounds: AABB = plus_arm.transform * (plus_arm.get_node("truss") as MeshInstance3D).get_aabb()
	_check(absf(truss_bounds.position.y + float(c["foot_depth_m"])) < 1.0, "root columns' foot %.1f m" % truss_bounds.position.y)
	var pad_bounds: AABB = plus_arm.transform * (plus_arm.get_node("pad") as MeshInstance3D).get_aabb()
	_check(pad_bounds.end.y < 0.0 and absf(pad_bounds.get_center().z - float(c["pad_x_m"])) < 0.01, "contact pad at the tip")
	_check_closed_pose(view, models["detail"], geometry)
	_check_actuators(view, geometry)
	_check_mount(view, models, geometry)
	var paint := ((plus_arm.get_node("truss") as MeshInstance3D).material_override as ShaderMaterial).get_shader_parameter("albedo") as Color
	_check(paint.v < 0.1, "chopstick: matte black paint (value %.3f)" % paint.v)
	# The QD arm stands at its deck height, static.
	var qd := view.get_node("QdArmMount") as Node3D
	var qd_deck_m := Frames.to_render(Vector3(0.0, 0.0, 1.0)).dot(qd.position)
	_check(absf(qd_deck_m - float(models["detail"]["qd_arm"]["deck_m"])) < 0.01, "QD arm deck at %.2f m" % qd_deck_m)


## Real poses at the closed pose (opening 0): the pads' centres, taken from the drawn meshes through the pivots, lie where
## the physics puts them: at the hinge, `pad_x` toward the pad, across by the gap. Both arms (mirror) on the same station,
## so the main beams are parallel; the physics gap between the inner pad faces is arm_closed_gap_m - 2 x proud.
## The bearing pieces are parallel: their plan is checked on the layout in pytest).
func _check_closed_pose(view: TowerView, detail: Dictionary, geometry: Dictionary) -> void:
	view.update_state({"opening_rad": 0.0, "carriage_offset_m": 0.0})
	var c: Dictionary = detail["chopstick"]
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	var up := Vector3(0.0, 0.0, 1.0)
	var centres: Array[Vector3] = []
	for arm_index in range(2):
		var side := 1.0 if arm_index == 0 else -1.0
		var pivot := _pivot(view, arm_index)
		var arm := pivot.get_node("Chopstick") as Node3D
		var bounds: AABB = pivot.transform * (arm.transform * (arm.get_node("pad") as MeshInstance3D).get_aabb())
		var y_pad: float = float(c["inner_face_y_m"]) + float(c["pad_proud_m"]) - 0.5 * float(c["pad_width_m"])
		var z_pad: float = -float(c["pad_top_depth_m"]) - 0.5 * float(c["pad_height_m"])
		var hinge: Vector3 = geometry["hinge_plus_m" if arm_index == 0 else "hinge_minus_m"]
		var want := Frames.to_render(hinge - toward * float(c["pad_x_m"]) - across * (side * y_pad) + up * z_pad)
		_check(bounds.get_center().distance_to(want) < 0.05, "closed pose arm %d: pad at %s, want %s" % [arm_index, bounds.get_center(), want])
		centres.append(bounds.get_center())
	var gap := (centres[0] - centres[1]).length()
	# Pad centres: 2 x (gap / 2 + width / 2 - y_pad) apart, from the physics numbers
	var y_c: float = float(c["inner_face_y_m"]) + float(c["pad_proud_m"]) - 0.5 * float(c["pad_width_m"])
	var lateral: float = float(geometry["hinge_plus_m"].dot(across))  # = gap / 2 + width / 2 of the physics hinge
	var want_gap := 2.0 * (lateral - y_c)
	_check(absf(gap - want_gap) < 0.05, "closed pose: pad centres %.3f m apart (want %.3f)" % [gap, want_gap])


## The mount model: measured plan, the physics deck height, ring, exits, the terrain mask on its footprint, the stack's
## clearance (the full surface sampling is in test_mount_layout.py; here the generated vertices).
func _check_mount(view: TowerView, models: Dictionary, geometry: Dictionary) -> void:
	var m: Dictionary = models["detail"]["mount"]
	var t: Dictionary = models["detail"]["trench"]
	_check(view.has_models and view.get_node_or_null("MountModel/Mount") != null, "mount model built")
	_check(view.get_node_or_null("TrenchModel/Trench") != null, "trench model built")
	var parts := view.get_node("MountModel/Mount")
	var plinth: AABB = (parts.get_node("plinth") as MeshInstance3D).mesh.get_aabb()  # glTF axes: X tower, Y across, Z up
	_check(absf(plinth.size.y - 44.6) < 0.1 and absf(plinth.size.x - 40.0) < 0.1, "base 44.6 x 40 m (%s)" % plinth.size)
	_check(absf(plinth.position.z + float(m["pit_depth_m"])) < 0.01, "pit floor %.1f m below the ground" % -plinth.position.z)
	var shell: AABB = (parts.get_node("shell") as MeshInstance3D).mesh.get_aabb()
	var ring_top := float(m["deck_height_m"])  # as measured; the physics mount height follows in the physics plan
	var deck_z := ring_top - float(m["deck_drop_m"])
	_check(absf(shell.end.z - deck_z) < 0.01, "deck at the measured height (%.3f, want %.3f)" % [shell.end.z, deck_z])
	var top_half := 0.5 * float(m["top_across_m"])
	_check(absf(shell.size.y - float(m["base_across_m"])) < 0.05, "slope foot across %.2f m" % shell.size.y)
	var hardware: AABB = (parts.get_node("hardware") as MeshInstance3D).mesh.get_aabb()
	_check(hardware.end.z > ring_top, "ring and clamps above the deck")
	# Stack clearance on the generated vertices: nothing of the mount inside the 4.5 m hull above the ring top, the ring
	# opening is the hull's radius at the ring top.
	var min_r_above := INF
	var min_r_ring := INF
	for node in parts.get_children():
		var arrays := ((node as MeshInstance3D).mesh as ArrayMesh).surface_get_arrays(0)
		for v: Vector3 in arrays[Mesh.ARRAY_VERTEX]:
			var r := Vector2(v.x, v.y).length()
			if v.z > ring_top + 1.0e-4:
				min_r_above = minf(min_r_above, r)
			elif absf(v.z - ring_top) < 1.0e-4:
				min_r_ring = minf(min_r_ring, r)
	_check(min_r_above >= 4.5 + 0.04 - 1.0e-4, "nothing of the mount inside the hull above the ring (%.3f m)" % min_r_above)
	_check(absf(min_r_ring - 0.5 * float(m["ring_opening_diameter_m"])) < 0.01, "ring opening %.3f m radius" % min_r_ring)
	# Exits: the trench's ramps run across the mount-tower line, 33 m from the axis; the hardstand stays out of the pit.
	var hardstand: AABB = (view.get_node("TrenchModel/Trench/hardstand") as MeshInstance3D).mesh.get_aabb()
	_check(hardstand.position.z > 0.0 and hardstand.size.z < 0.001, "hardstand a flat plate above the ground (%s)" % hardstand)
	var ramp: AABB = (view.get_node("TrenchModel/Trench/concrete") as MeshInstance3D).mesh.get_aabb()
	_check(absf(ramp.size.x - float(t["width_m"])) < 0.01, "ramps %.2f m wide" % ramp.size.x)
	var mid := 0.5 * (0.5 * float(m["base_across_m"]) + ramp.end.y)
	_check(absf(mid - float(t["centre_m"])) < 0.5, "ramp middle %.2f m from the axis" % mid)
	# The terrain mask lies on the drawn footprint: central rectangle = the plinth, ramp = the ramp's plan.
	var shape: Vector4 = ShaderGlobals.value(&"trench_shape")
	_check(absf(shape.x - 0.5 * plinth.size.x) < TowerView.MASK_INSET_M + 0.01, "mask along %.2f m, plinth %.2f" % [shape.x, 0.5 * plinth.size.x])
	_check(absf(shape.y - 0.5 * plinth.size.y) < TowerView.MASK_INSET_M + 0.01, "mask across %.2f m" % shape.y)
	_check(absf(shape.z - 0.5 * ramp.size.x) < TowerView.MASK_INSET_M + 0.01, "mask ramp half width %.2f m" % shape.z)
	_check(absf(shape.w - ramp.end.y) < TowerView.MASK_INSET_M + 0.01, "mask ramp end %.2f m, ramp %.2f" % [shape.w, ramp.end.y])
	_check(float(ShaderGlobals.value(&"trench_range_m")) == float(t["mask_range_m"]), "mask range set")
	var axis_t: Vector3 = ShaderGlobals.value(&"trench_axis_t")
	var yaw := deg_to_rad(float(m["plan_yaw_deg"]))
	var square_x: Vector3 = geometry["tower_direction"] * cos(yaw) - geometry["across"] * sin(yaw)
	_check(axis_t.is_equal_approx((view.basis * Frames.to_render(square_x)).normalized()), "mask axis along the measured square")
	_check(Basis(geometry["tower_direction"], geometry["across"], Vector3(0.0, 0.0, 1.0)).determinant() > 0.0, "site axes right-handed")
	_check(top_half > 0.0, "top half width %.2f" % top_half)
	var boxes_view := _view(OTHER_SITE)
	_check(not boxes_view.has_models, "a site without models keeps the old mount cylinder")


## The actuators run from the carriage to the arm's root column and follow the opening angle.
func _check_actuators(view: TowerView, geometry: Dictionary) -> void:
	var lengths: Array[float] = []
	for opening in [0.0, 0.7]:
		view.update_state({"opening_rad": opening, "carriage_offset_m": 0.0})
		for arm_index in range(2):
			var ends := view.actuator_ends(arm_index)
			var rod := view.get_node("actuator_%d_1" % arm_index) as MeshInstance3D
			var far_end := rod.position + rod.basis.y.normalized() * 0.5 * rod.basis.y.length()
			_check(far_end.distance_to(ends[1]) < 0.01, "actuator %d at %.1f rad: rod ends at the arm's lug" % [arm_index, opening])
			var barrel := view.get_node("actuator_%d_0" % arm_index) as MeshInstance3D
			var start := barrel.position - barrel.basis.y.normalized() * 0.5 * barrel.basis.y.length()
			_check(start.distance_to(ends[0]) < 0.01, "actuator %d at %.1f rad: barrel starts on the carriage" % [arm_index, opening])
		lengths.append(view.actuator_ends(0)[0].distance_to(view.actuator_ends(0)[1]))
	_check(absf(lengths[1] - lengths[0]) > 0.5, "actuator length follows the opening: %.2f -> %.2f m" % [lengths[0], lengths[1]])


## Tiers of one section: near MultiMeshes up to NEAR_END_M, the merged mesh to MID_END_M, the lattice shader beyond;
## decks / column bundle / roof cap / foot ring as dark solids.
func _check_hand_over(node: Node, detail: Dictionary, section: Dictionary, index: int, count: int,
		total_m: float) -> void:
	var panels := LatticeBuilder.panel_count(section)
	var decks := node.get_node("decks") as MultiMeshInstance3D
	var leg_size := 2.2
	var half := LatticeBuilder.leg_line_m(float(section["face_width_m"]), leg_size)
	var is_top := index == count - 1
	var base_m: float = (node as Node3D).position.y
	var expected := LatticeBuilder.dark_solids(detail, section, base_m, half, is_top).size()
	_check(decks.multimesh.instance_count == expected, "section %d: %d dark solids (decks, column, stairs, roof)" % [index, expected])
	var clad_count := LatticeBuilder.clad_solids(detail, section, base_m, half, leg_size, index == 0, is_top).size()
	_check((node.get_node_or_null("clad") != null) == (clad_count > 0), "section %d: clad node iff there are light solids" % index)
	_check((node.get_node_or_null("roof") != null) == is_top, "section %d: roof frame only on top" % index)
	_check((node.get_node_or_null("far_roof") != null) == is_top, "section %d: far roof boxes only on top" % index)
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


## Axis-aligned bounds of a unit box scaled by a transform without rotation (the solids' boxes; stair slabs are tilted).
func _bounds(t: Transform3D) -> AABB:
	var size := Vector3(t.basis.x.length(), t.basis.y.length(), t.basis.z.length())
	return AABB(t.origin - 0.5 * size, size)


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
		"hinge_plus_m": toward * 24.0 + across * 5.8 + up * 95.0,
		"hinge_minus_m": toward * 24.0 - across * 5.8 + up * 95.0,
	}


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
