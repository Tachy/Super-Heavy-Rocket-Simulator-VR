class_name TowerView
extends Node3D
## Catch tower of a launch site (tower, launch mount, two arms on their carriage), built procedurally from
## the core's geometry (FlightSim.site_tower): what is drawn is what the physics collides with.
## Child of the Earth body, placed at the site origin in Earth-fixed coordinates. Local coordinates are the
## site frame (east, north, up) mapped to render axes like every body frame (Frames.to_render).

const MOUNT_SEGMENTS := 24
const ARM_STRIP := Color(0.45, 0.36, 0.24)  ## [E] tan strip on the arm deck, images.jpeg

var site_id := ""
## render_check: fixed arm opening (radians) instead of the flight state.
var preview_opening_rad := NAN
var _geometry: Dictionary = {}
var _arms: Array[Node3D] = []  ## pivots at the hinges: [+across, -across]
var _carriage: Node3D = null  ## models only: follows the carriage offset
var _cables: Array[MeshInstance3D] = []  ## hoist cables, sheave block to the roof block
var _cable_lateral_m: Array[float] = []
var _cable_top_m := 0.0  ## height of the cables' upper end above the site
var _cable_base_m := 0.0  ## height of the sheave block's top above the deck plane
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
	_build_mount(geometry)
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
	# The sheave block's top, as in mechazilla_layout.carriage_drum_top_z; cables run to the roof block's underside.
	_cable_base_m = float(k["top_z_m"]) + 0.6 + float(k["drum_diameter_m"])
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
