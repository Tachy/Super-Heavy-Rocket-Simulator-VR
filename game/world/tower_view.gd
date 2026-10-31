class_name TowerView
extends Node3D
## Catch tower of a launch site (tower, launch mount, two arms on their carriage), built procedurally from
## the core's geometry (FlightSim.site_tower): what is drawn is what the physics collides with.
## Child of the Earth body, placed at the site origin in Earth-fixed coordinates. Local coordinates are the
## site frame (east, north, up) mapped to render axes like every body frame (Frames.to_render).

const MOUNT_SEGMENTS := 24
## [E] Mount materials (linear albedo from the photos: weathered concrete, dark tunnel interiors, grey hoods).
const PLINTH_CONCRETE := Color(0.50, 0.49, 0.46)
const INTERIOR_DARK := Color(0.14, 0.13, 0.12)
const DEFLECTOR_DARK := Color(0.09, 0.09, 0.10)
const MOUNT_STEEL := Color(0.30, 0.30, 0.31)
const STAIR_TAN := Color(0.46, 0.38, 0.27)
const TRENCH_WALL := Color(0.12, 0.12, 0.13)
const MASK_INSET_M := 0.05  ## [E] the terrain mask ends this far inside the walls, so no sliver of ground shows beside them
const MASK_VISIBLE_MARGIN_M := 150.0  ## the pit model is drawn this much beyond the mask's camera range
const ARM_STRIP := Color(0.45, 0.36, 0.24)  ## [E] tan strip on the arm deck, images.jpeg

var site_id := ""
## True when the Pad B models were built (the mount pyramid, the trench and the terrain mask), false for the boxes.
var has_models := false
## render_check: fixed arm opening (radians) instead of the flight state.
var preview_opening_rad := NAN
var _geometry: Dictionary = {}
var _arms: Array[Node3D] = []  ## pivots at the hinges: [+across, -across]
var _carriage: Node3D = null  ## models only: follows the carriage offset
var _cables: Array[MeshInstance3D] = []  ## hoist cables, sheave block to the roof block
var _cable_lateral_m: Array[float] = []
var _cable_top_m := 0.0  ## height of the cables' upper end above the site
var _cable_base_m := 0.0  ## height of the sheave block's top above the deck plane
var _actuators: Array[Array] = []  ## per arm: [barrel, rod] cylinders from the carriage to the arm's root column
var _actuator: Dictionary = {}  ## detail file [carriage] actuator_* keys
var _steel := LitAerial.material(Color(0.42, 0.43, 0.45), 0.5, 0.5)  # [E] albedo ~0.15 as TowerMaterial
var _concrete := LitAerial.material(Color(0.62, 0.61, 0.58), 0.9, 0.0)
var _lattice := ShaderMaterial.new()


## Rebuilding (site change, re-entry) frees what an earlier build made.
func build(geometry: Dictionary) -> void:
	for child in get_children():
		remove_child(child)
		child.queue_free()
	_arms.clear()
	_carriage = null
	_cables.clear()
	_cable_lateral_m.clear()
	_actuators.clear()
	var had_models := has_models
	has_models = false
	_geometry = geometry
	_lattice.shader = load("res://shaders/lattice.gdshader")
	position = Frames.to_render(geometry["origin_itrf_m"])
	basis = Frames.basis_to_render(Basis(geometry["east"], geometry["north"], geometry["up"]))
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	var up := Vector3(0.0, 0.0, 1.0)
	var models := StructureModels.load_for(site_id)  ## {} -> today's boxes
	if models.is_empty():
		add_child(_box(geometry["tower_size_m"], Basis(toward, across, up), geometry["tower_center_m"], _lattice))
	else:
		add_child(_pad_b_lattice(models, geometry, Basis(toward, across, up)))
	if models.is_empty():
		_build_mount(geometry)
		if had_models:
			ShaderGlobals.set_value(&"trench_range_m", 0.0)  # this view stopped masking the terrain
	else:
		_build_mount_model(models, geometry)
	for i in range(2):
		var pivot := Node3D.new()
		if models.is_empty():
			var length: float = geometry["arm_length_m"]
			var thickness: float = geometry["arm_thickness_m"]
			pivot.add_child(_box(Vector3(length, geometry["arm_width_m"], thickness), Basis(),
					Vector3(0.5 * length, 0.0, -0.5 * thickness), _steel))
		else:
			# The model is drawn for the +across arm (+Y toward the axis). The arm frame's Y = up x u points to -across at
			# the closed pose, so it stands as drawn only on +across; the -across arm mirrors it. Length: Pad 2's 27 m, the
			# physics arm stays arm_length_m (deviation table).
			var arm := StructureModels.model_node("Chopstick", models["chopstick"], _arm_materials(), TowerMaterial.black_paint())
			if i == 1:
				arm.basis = arm.basis * Basis.from_scale(Vector3(1.0, -1.0, 1.0))
			pivot.add_child(arm)
		add_child(pivot)
		_arms.append(pivot)
	if not models.is_empty():
		_build_carriage(models)
		_build_qd_arm(models, geometry)
	update_state({})


## The detail file's lattice at the physics tower box: base on the box's base, the face toward the pad on the box's
## pad-side face (the footprint centre moves by the difference of the face widths), faces rotated with the box.
func _pad_b_lattice(models: Dictionary, geometry: Dictionary, axes: Basis) -> Node3D:
	var size: Vector3 = geometry["tower_size_m"]
	var face_width: float = models["detail"]["tower.sections"][0]["face_width_m"]
	var lattice := LatticeBuilder.build(models["detail"], models["profiles"], TowerMaterial.lattice_set())
	var up := Vector3(0.0, 0.0, 1.0)
	var toward: Vector3 = axes.x
	var centre_site: Vector3 = geometry["tower_center_m"] - up * (0.5 * size.z) + toward * (0.5 * (face_width - size.x))
	lattice.basis = Frames.basis_to_render(axes)
	lattice.position = Frames.to_render(centre_site)
	return lattice


## Pivot node at the hinge of arm `index` (0 = +across, 1 = -across).
func arm_pivot(index: int) -> Node3D:
	return _arms[index]


## state: FlightSim.tower_state() of the flight at this site, or {} (arms open, carriage at rest).
func update_state(state: Dictionary) -> void:
	if _geometry.is_empty():
		return
	var opening: float = state.get("opening_rad", _geometry["arm_open_rad"])
	if not is_nan(preview_opening_rad):
		opening = preview_opening_rad
	var offset: float = state.get("carriage_offset_m", 0.0)
	var toward: Vector3 = _geometry["tower_direction"]
	var across: Vector3 = _geometry["across"]
	var up := Vector3(0.0, 0.0, 1.0)
	for i in range(_arms.size()):
		var side := 1.0 if i == 0 else -1.0
		var hinge: Vector3 = _geometry["hinge_plus_m" if i == 0 else "hinge_minus_m"]
		var u := -toward * cos(opening) + across * (side * sin(opening))  # mirrors TowerGeometry::arm_direction
		_arms[i].basis = Frames.basis_to_render(Basis(u, up.cross(u), up))
		_arms[i].position = Frames.to_render(hinge + up * offset)
	if _carriage != null:
		_place_carriage(offset)
		_place_actuators()


func _arm_materials() -> Dictionary:
	var light := TowerMaterial.clad()
	return {"pad": light, "hardware": light, "strip": LitAerial.material(ARM_STRIP, 0.8, 0.0)}


## The carriage across the tower face (its origin in the hinge line, centre plane, deck plane) and its hoist cables.
func _build_carriage(models: Dictionary) -> void:
	_carriage = Node3D.new()
	_carriage.name = "CarriageMount"
	var toward: Vector3 = _geometry["tower_direction"]
	var across: Vector3 = _geometry["across"]
	_carriage.basis = Frames.basis_to_render(Basis(-toward, -across, Vector3(0.0, 0.0, 1.0)))
	_carriage.add_child(StructureModels.model_node("Carriage", models["carriage"], {"hardware": TowerMaterial.clad()},
			TowerMaterial.black_paint()))
	add_child(_carriage)
	var k: Dictionary = models["detail"]["carriage"]
	var count := int(k["cable_count"])
	var size: float = k["cable_size_m"]
	var pitch: float = k["cable_spacing_m"]
	# The sheave block's top (the layout puts its centre at top + gap + diameter / 2); cables run to the roof block's underside.
	_cable_base_m = float(k["top_z_m"]) + float(k["drum_gap_m"]) + float(k["drum_diameter_m"])
	_cable_top_m = float(models["detail"]["tower.roof"]["block_base_m"])
	var mesh := CylinderMesh.new()
	mesh.top_radius = 0.5 * size
	mesh.bottom_radius = 0.5 * size
	mesh.height = 1.0
	mesh.radial_segments = 6
	mesh.rings = 1
	for c in range(count):
		var cable := MeshInstance3D.new()
		cable.name = "cable_%d" % c
		cable.mesh = mesh
		cable.material_override = TowerMaterial.dark()
		add_child(cable)
		_cables.append(cable)
		_cable_lateral_m.append((c - 0.5 * (count - 1)) * pitch)
	_build_actuators(k)


## The electromechanical actuators (NSF 2026-05: not hydraulic): per arm a barrel fixed on the carriage and a rod to the lug
## on the arm's inclined root column, both cylinders placed from the two end points each update, so the pair follows the
## opening angle by geometry.
func _build_actuators(k: Dictionary) -> void:
	_actuator = k
	for i in range(2):
		var pair: Array = []
		for part in range(2):
			var mesh := CylinderMesh.new()
			mesh.top_radius = 0.5 * float(k["actuator_barrel_m"] if part == 0 else k["actuator_rod_m"])
			mesh.bottom_radius = mesh.top_radius
			mesh.height = 1.0
			mesh.radial_segments = 16
			mesh.rings = 1
			var node := MeshInstance3D.new()
			node.name = "actuator_%d_%d" % [i, part]
			node.mesh = mesh
			node.material_override = TowerMaterial.clad() if part == 1 else TowerMaterial.dark()
			add_child(node)
			pair.append(node)
		_actuators.append(pair)


## Both end points in this node's (render) frame: the anchor on the carriage, the lug on the arm (model coordinates).
func actuator_ends(arm: int) -> Array[Vector3]:
	var side := 1.0 if arm == 0 else -1.0
	var k := _actuator
	var anchor_local := Vector3(k["actuator_anchor_x_m"], -side * (float(k["hinge_y_m"]) - float(k["actuator_anchor_in_m"])),
			-float(k["actuator_anchor_depth_m"]))
	# Model coordinates of the arm as drawn for +across (+Y toward the axis): the lug lies outboard, Y < 0 (the mirrored
	# -across arm's node flips it).
	var lug_local := Vector3(k["actuator_lug_x_m"], -float(k["actuator_lug_out_m"]), -float(k["actuator_lug_depth_m"]))
	var carriage_model := _carriage.get_node("Carriage") as Node3D
	var arm_model := _arms[arm].get_node("Chopstick") as Node3D
	var a: Vector3 = _carriage.transform * (carriage_model.transform * anchor_local)
	var b: Vector3 = _arms[arm].transform * (arm_model.transform * lug_local)
	return [a, b]


func _place_actuators() -> void:
	for i in range(_actuators.size()):
		var ends := actuator_ends(i)
		var a: Vector3 = ends[0]
		var b: Vector3 = ends[1]
		var axis := b - a
		var length := axis.length()
		var barrel := minf(float(_actuator["actuator_barrel_length_m"]), length)
		var dir := axis / length
		var tilt := Basis(Quaternion(Vector3(0.0, 1.0, 0.0), dir))
		for part in range(2):
			var node := _actuators[i][part] as MeshInstance3D
			var from_a := 0.0 if part == 0 else barrel
			var len_part := barrel if part == 0 else length - barrel
			node.position = a + dir * (from_a + 0.5 * len_part)
			node.basis = tilt * Basis.from_scale(Vector3(1.0, maxf(len_part, 0.01), 1.0))


## Carriage node and cables at the carriage offset; the hinge line's height comes from the hinges.
func _place_carriage(offset: float) -> void:
	var up := Vector3(0.0, 0.0, 1.0)
	var across: Vector3 = _geometry["across"]
	var plus: Vector3 = _geometry["hinge_plus_m"]
	var minus: Vector3 = _geometry["hinge_minus_m"]
	var centre: Vector3 = 0.5 * (plus + minus) + up * offset
	_carriage.position = Frames.to_render(centre)
	var low_m: float = centre.z + _cable_base_m
	var length := maxf(0.1, _cable_top_m - low_m)
	for i in range(_cables.size()):
		var at: Vector3 = centre + across * _cable_lateral_m[i]
		at.z = low_m + 0.5 * length
		_cables[i].position = Frames.to_render(at)
		# The cylinder's axis (local Y) is the column that basis_to_render sends to render Y: the site's up.
		_cables[i].basis = Frames.basis_to_render(Basis(across, up.cross(across), up)) * Basis.from_scale(Vector3(1.0, length, 1.0))


## The ship QD arm, static with its tip clear of the hull (no core state yet: deviation table). Its X runs to the pad from
## the physics box's pad-side face, on the centre plane, its tip deck at the measured height.
func _build_qd_arm(models: Dictionary, geometry: Dictionary) -> void:
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	var size: Vector3 = geometry["tower_size_m"]
	var centre: Vector3 = geometry["tower_center_m"]
	var face: Vector3 = toward * (centre.dot(toward) - 0.5 * size.x)
	var mount := Node3D.new()
	mount.name = "QdArmMount"
	mount.basis = Frames.basis_to_render(Basis(-toward, -across, Vector3(0.0, 0.0, 1.0)))
	mount.position = Frames.to_render(face + Vector3(0.0, 0.0, float(models["detail"]["qd_arm"]["deck_m"])))
	mount.add_child(StructureModels.model_node("QdArm", models["qd_arm"], {"hardware": TowerMaterial.clad()},
			TowerMaterial.black_paint()))
	add_child(mount)


## Box with `size_local` along the columns of `axes` (site frame), centred at `center_site`.
func _box(size_local: Vector3, axes: Basis, center_site: Vector3, material: Material) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(size_local.y, size_local.z, size_local.x)  # local (a, b, c) -> render (b, c, a)
	var node := MeshInstance3D.new()
	node.mesh = mesh
	node.material_override = material
	node.basis = Frames.basis_to_render(axes)
	node.position = Frames.to_render(center_site)
	return node


func _build_mount(geometry: Dictionary) -> void:
	var inner: float = geometry["mount_inner_radius_m"]
	var outer: float = geometry["mount_outer_radius_m"]
	var height: float = geometry["mount_height_m"]
	var radius := 0.5 * (inner + outer)
	var segment := TAU * radius / MOUNT_SEGMENTS
	for k in range(MOUNT_SEGMENTS):
		var a := TAU * k / MOUNT_SEGMENTS
		var radial := Vector3(cos(a), sin(a), 0.0)
		var tangent := Vector3(-sin(a), cos(a), 0.0)
		add_child(_box(Vector3(outer - inner, segment * 1.02, height), Basis(radial, tangent, Vector3(0.0, 0.0, 1.0)),
				radial * radius + Vector3(0.0, 0.0, 0.5 * height), _concrete))


## The Pad B mount (X toward the tower) and the trench, at the site origin; the terrain is not drawn inside their footprint.
func _build_mount_model(models: Dictionary, geometry: Dictionary) -> void:
	has_models = true
	var detail: Dictionary = models["detail"]
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	var frame := Frames.basis_to_render(Basis(toward, across, Vector3(0.0, 0.0, 1.0)))
	var shell := TowerMaterial.mount_shell(float(detail["mount"]["tile_m"]))
	var materials := {
		"shell": shell, "plinth": LitAerial.material(PLINTH_CONCRETE, 0.9, 0.0),
		"interior": LitAerial.material(INTERIOR_DARK, 0.9, 0.0), "deflector": LitAerial.material(DEFLECTOR_DARK, 0.6, 0.4),
		"hood": TowerMaterial.clad(), "hardware": LitAerial.material(MOUNT_STEEL, 0.5, 0.5),
		"stair": LitAerial.material(STAIR_TAN, 0.8, 0.0), "door": TowerMaterial.dark()}
	var mount := Node3D.new()
	mount.name = "MountModel"
	mount.basis = frame
	mount.add_child(StructureModels.model_node("Mount", models["mount"], materials, _concrete))
	add_child(mount)
	var trench := Node3D.new()
	trench.name = "TrenchModel"
	trench.basis = frame
	var parts := StructureModels.model_node("Trench", models["trench"], {"concrete": _concrete,
			"wall": LitAerial.material(TRENCH_WALL, 0.9, 0.0)}, _concrete)
	var range_m: float = float(detail["trench"]["mask_range_m"]) + MASK_VISIBLE_MARGIN_M
	for part in parts.get_children():
		(part as GeometryInstance3D).visibility_range_end = range_m
	trench.add_child(parts)
	add_child(trench)
	_set_trench_mask(detail, geometry)


## Terrain mask globals (terrain_earth.gdshader): the footprint of the mount and the two ramps, in the site's axes.
func _set_trench_mask(detail: Dictionary, geometry: Dictionary) -> void:
	var m: Dictionary = detail["mount"]
	var t: Dictionary = detail["trench"]
	var split := TerrainQuadtree.split_float32(position)
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	ShaderGlobals.set_value(&"trench_site_hi", split[0])
	ShaderGlobals.set_value(&"trench_site_lo", split[1])
	ShaderGlobals.set_value(&"trench_axis_t", (basis * Frames.to_render(toward)).normalized())
	ShaderGlobals.set_value(&"trench_axis_a", (basis * Frames.to_render(across)).normalized())
	var half_a: float = 0.5 * float(m["base_across_m"])
	ShaderGlobals.set_value(&"trench_shape", Vector4(0.5 * float(m["base_toward_m"]) - MASK_INSET_M,
			half_a - MASK_INSET_M, 0.5 * float(t["width_m"]) - MASK_INSET_M,
			half_a + float(t["length_m"]) - MASK_INSET_M))
	ShaderGlobals.set_value(&"trench_range_m", float(t["mask_range_m"]))
