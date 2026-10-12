class_name TowerView
extends Node3D
## Catch tower of a launch site (tower, launch mount, two arms on their carriage), built procedurally from
## the core's geometry (FlightSim.site_tower): what is drawn is what the physics collides with.
## Child of the Earth body, placed at the site origin in Earth-fixed coordinates. Local coordinates are the
## site frame (east, north, up) mapped to render axes like every body frame (Frames.to_render).

const MOUNT_SEGMENTS := 24

var site_id := ""
## render_check: fixed arm opening (radians) instead of the flight state.
var preview_opening_rad := NAN
var _geometry: Dictionary = {}
var _arms: Array[Node3D] = []  ## pivots at the hinges: [+across, -across]
var _steel := StandardMaterial3D.new()
var _concrete := StandardMaterial3D.new()
var _lattice := ShaderMaterial.new()


func build(geometry: Dictionary) -> void:
	_geometry = geometry
	_steel.albedo_color = Color(0.55, 0.56, 0.58)
	_steel.metallic = 0.6
	_steel.roughness = 0.45
	_concrete.albedo_color = Color(0.62, 0.61, 0.58)
	_concrete.roughness = 0.9
	_lattice.shader = load("res://shaders/lattice.gdshader")
	position = Frames.to_render(geometry["origin_itrf_m"])
	basis = Frames.basis_to_render(Basis(geometry["east"], geometry["north"], geometry["up"]))
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	var up := Vector3(0.0, 0.0, 1.0)
	add_child(_box(geometry["tower_size_m"], Basis(toward, across, up), geometry["tower_center_m"], _lattice))
	_build_mount(geometry)
	for i in range(2):
		var pivot := Node3D.new()
		var length: float = geometry["arm_length_m"]
		var thickness: float = geometry["arm_thickness_m"]
		pivot.add_child(_box(Vector3(length, geometry["arm_width_m"], thickness), Basis(),
				Vector3(0.5 * length, 0.0, -0.5 * thickness), _steel))
		add_child(pivot)
		_arms.append(pivot)
	update_state({})


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
