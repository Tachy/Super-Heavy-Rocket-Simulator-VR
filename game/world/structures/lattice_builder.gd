class_name LatticeBuilder
## Pad B tower lattice from the detail file (config/structures/starbase_olp2.toml): per section one MultiMesh of the four
## corner legs and one per face of the braces, instanced from the generated profile meshes (unit length along +Z).
## Local frame of the returned node = the frame of TowerView's tower box: origin at the centre of the footprint on the
## ground, +X across, +Y up, +Z away from the pad. Face 0 is the face at -Z (toward the pad); faces 1..3 follow by
## 90 degrees about Y.
## Members cast no shadow; a per-section proxy box with the lattice shader casts the lattice's shadow instead.

const FACES := 4
const LEGS := 4
## Braces per panel: two diagonals and the horizontal at the panel's lower edge (the topmost section adds one at its top).
const BRACES_PER_PANEL := 3


## Panels of a section: its height over the gusset pitch, at least one.
static func panel_count(section: Dictionary) -> int:
	return maxi(1, roundi(float(section["height_m"]) / float(section["pitch_m"])))


## Braces on one face of a section; `is_top` marks the topmost one.
static func braces_per_face(section: Dictionary, is_top: bool) -> int:
	return panel_count(section) * BRACES_PER_PANEL + (1 if is_top else 0)


static func total_height_m(detail: Dictionary) -> float:
	var height := 0.0
	for section: Dictionary in detail["tower.sections"]:
		height += float(section["height_m"])
	return height


## detail: TomlLite dictionary of the detail file; profiles: {name: Mesh}; material: for every member.
static func build(detail: Dictionary, profiles: Dictionary, material: Material) -> Node3D:
	var root := Node3D.new()
	root.name = "Lattice"
	var sections: Array = detail["tower.sections"]
	var base_m := 0.0
	for i in range(sections.size()):
		var section: Dictionary = sections[i]
		var node := _build_section(section, i == sections.size() - 1, profiles, material)
		node.name = "section_%d" % i
		node.position = Vector3(0.0, base_m, 0.0)
		root.add_child(node)
		base_m += float(section["height_m"])
	return root


static func _build_section(section: Dictionary, is_top: bool, profiles: Dictionary, material: Material) -> Node3D:
	var node := Node3D.new()
	var height: float = section["height_m"]
	var width: float = section["face_width_m"]
	var leg_mesh: Mesh = profiles[section["leg_profile"]]
	var brace_mesh: Mesh = profiles[section["brace_profile"]]
	var leg_size: float = float(leg_mesh.get_aabb().size.x)
	var half := 0.5 * width - 0.5 * leg_size  # leg and brace centre lines
	# Legs: profile +Z (length) -> up, x stays, y -> -z; origin at the section's lower end.
	var leg_transforms: Array[Transform3D] = []
	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			leg_transforms.append(Transform3D(Basis(Vector3.RIGHT, Vector3(0.0, 0.0, -1.0), Vector3(0.0, height, 0.0)),
					Vector3(sx * half, 0.0, sz * half)))
	node.add_child(_instances("legs", leg_mesh, leg_transforms, material))
	var members := face_members(section, half, is_top)
	for face in range(FACES):
		var transforms: Array[Transform3D] = []
		for member: Array in members:
			transforms.append(_member_transform(member[0], member[1], half))
		var instance := _instances("face_%d" % face, brace_mesh, transforms, material)
		instance.rotation.y = face * 0.5 * PI
		node.add_child(instance)
	node.add_child(_shadow_proxy(width, height, float(section["pitch_m"])))
	return node


## Members of one face as [from, to] pairs in the face plane (x along the face, y up from the section's lower edge).
## Public because the headless renderer keeps no MultiMesh transforms: tests read the layout here.
static func face_members(section: Dictionary, half: float, is_top: bool) -> Array:
	var members: Array = []
	var panels := panel_count(section)
	var panel_h := float(section["height_m"]) / panels
	for p in range(panels):
		var y0 := p * panel_h
		var y1 := y0 + panel_h
		match section["pattern"]:
			"k_brace":
				var mid := 0.5 * (y0 + y1)
				members.append([Vector2(-half, y0), Vector2(half, mid)])
				members.append([Vector2(-half, y1), Vector2(half, mid)])
			_:  # x_brace
				members.append([Vector2(-half, y0), Vector2(half, y1)])
				members.append([Vector2(half, y0), Vector2(-half, y1)])
		members.append([Vector2(-half, y0), Vector2(half, y0)])
	if is_top:
		members.append([Vector2(-half, section["height_m"]), Vector2(half, section["height_m"])])
	return members


## Profile +Z along from -> to (scaled to its length), on the face plane z = -half (outward normal -Z).
static func _member_transform(from: Vector2, to: Vector2, half: float) -> Transform3D:
	var along := Vector3(to.x - from.x, to.y - from.y, 0.0)
	var direction := along.normalized()
	var side := direction.cross(Vector3(0.0, 0.0, -1.0)).normalized()
	return Transform3D(Basis(side, direction.cross(side), along), Vector3(from.x, from.y, -half))


static func _instances(node_name: String, mesh: Mesh, transforms: Array[Transform3D],
		material: Material) -> MultiMeshInstance3D:
	var multimesh := MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D
	multimesh.mesh = mesh
	multimesh.instance_count = transforms.size()
	for i in range(transforms.size()):
		multimesh.set_instance_transform(i, transforms[i])
	var instance := MultiMeshInstance3D.new()
	instance.name = node_name
	instance.multimesh = multimesh
	instance.material_override = material
	instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	return instance


## Box of the section with the lattice shader: invisible to the camera, its alpha-cut pattern casts the shadow.
static func _shadow_proxy(width: float, height: float, pitch_m: float) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(width, height, width)
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/lattice.gdshader")
	material.set_shader_parameter("bay_m", pitch_m)
	var proxy := MeshInstance3D.new()
	proxy.name = "shadow_proxy"
	proxy.mesh = mesh
	proxy.material_override = material
	proxy.position = Vector3(0.0, 0.5 * height, 0.0)
	proxy.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_SHADOWS_ONLY
	return proxy
