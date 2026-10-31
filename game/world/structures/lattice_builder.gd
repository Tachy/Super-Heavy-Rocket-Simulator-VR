class_name LatticeBuilder
## Pad B tower lattice from the detail file (config/structures/starbase_olp2.toml): per section one MultiMesh of the four
## corner legs, one per face of the braces and one of the dark solids (floor decks, inner column bundle, roof cap, foot
## ring), instanced from the generated profile meshes (unit length along +Z).
## Local frame of the returned node = the frame of TowerView's tower box: origin at the centre of the footprint on the
## ground, +X across, +Y up, +Z away from the pad. Face 0 is the face at -Z (toward the pad); faces 1..3 follow by
## 90 degrees about Y.
## Distance hand-over per section (visibility ranges with dithered fades, anti-goal: a visible band):
##   near  (< NEAR_END_M)            the MultiMeshes (every member an instance),
##   mid   (NEAR_END_M .. MID_END_M) one merged mesh of the same geometry: 2 draw calls instead of ~10,
##   far   (> MID_END_M)             a box with the lattice shader tower_far.gdshader (analytic, filtered coverage of the
##                                   measured pitch: members thinner than a pixel become a density, never shimmer).
## Members cast no shadow; a per-section proxy box with the lattice shader casts the lattice's shadow instead.

const FACES := 4
const LEGS := 4
## Braces per panel: two diagonals and the horizontal at the panel's lower edge (the topmost section adds one at its top).
const BRACES_PER_PANEL := 3
## [E] Hand-over ranges. A brace (0.5 m) is 1 px wide at ~390 m on a 60 deg / 900 px desktop view (779 px focal length)
## and 1.7 px on a VR eye (~1300 px): individual members go at 380 m. The corner legs (2.2 m) stay >= 1.5 px to ~1100 m
## (desktop); the merged mesh ends at 1000 m, where the lattice shader (filtered) takes over: there the tiers' mean
## luminance agree within ~1 % (800 m: 0.0 %, 950 m: 0.9 %, 1050 m: 1.2 %).
const NEAR_END_M := 380.0
const MID_END_M := 1000.0
## Dither fade length each side of a range boundary (Godot: both neighbours fade over boundary +- this). 0 = hard swap:
## measured (tower_handover_check): with 60 m the two dithered neighbours were not complementary (the tower's rectangle
## 2.6 % brighter before the boundary and 3.6 % darker after it), while the tiers shown alone agree within 0.3 % (members /
## merged) and 0.1 % (merged / shader at 800-1000 m): a hard swap of matched tiers has no visible band.
const FADE_MARGIN_M := 0.0
## Visible share of a deck seen through the lattice at a grazing angle (far shader): thickness x this.
const FAR_DECK_VIEW_FACTOR := 1.6
const FAR_SHADER := preload("res://shaders/tower_far.gdshader")
const LATTICE_SHADER := preload("res://shaders/lattice.gdshader")


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


## Centre line of the legs and braces from the face width and the leg profile's width.
static func leg_line_m(face_width_m: float, leg_size_m: float) -> float:
	return 0.5 * face_width_m - 0.5 * leg_size_m


## detail: TomlLite dictionary of the detail file; profiles: {name: Mesh}; steel: material of the members; dark:
## material of the decks / column bundle.
static func build(detail: Dictionary, profiles: Dictionary, steel: Material, dark: Material) -> Node3D:
	var root := Node3D.new()
	root.name = "Lattice"
	var sections: Array = detail["tower.sections"]
	var base_m := 0.0
	for i in range(sections.size()):
		var node := _build_section(detail, i, profiles, steel, dark)
		node.name = "section_%d" % i
		node.position = Vector3(0.0, base_m, 0.0)
		root.add_child(node)
		base_m += float(sections[i]["height_m"])
	return root


static func _build_section(detail: Dictionary, index: int, profiles: Dictionary, steel: Material,
		dark: Material) -> Node3D:
	var sections: Array = detail["tower.sections"]
	var section: Dictionary = sections[index]
	var is_top := index == sections.size() - 1
	var node := Node3D.new()
	var height: float = section["height_m"]
	var width: float = section["face_width_m"]
	var leg_mesh: Mesh = profiles[section["leg_profile"]]
	var brace_mesh: Mesh = profiles[section["brace_profile"]]
	var leg_size: float = float(leg_mesh.get_aabb().size.x)
	var brace_size: float = float(brace_mesh.get_aabb().size.x)
	var half := leg_line_m(width, leg_size)
	var leg_transforms := _leg_transforms(height, half)
	var members := face_members(section, half, is_top)
	var face_transforms: Array[Transform3D] = []
	for member: Array in members:
		face_transforms.append(_member_transform(member[0], member[1], half))
	var solids := _solids(detail, section, half, index == 0, is_top)
	var roof := _roof_members(detail, section, half) if is_top else ([] as Array[Transform3D])
	var unit_box := BoxMesh.new()

	var near: Array[MultiMeshInstance3D] = [_instances("legs", leg_mesh, leg_transforms, steel)]
	for face in range(FACES):
		var instance := _instances("face_%d" % face, brace_mesh, face_transforms, steel)
		instance.rotation.y = face * 0.5 * PI
		near.append(instance)
	near.append(_instances("decks", unit_box, solids, dark))
	if not roof.is_empty():
		near.append(_instances("roof", brace_mesh, roof, steel))
	for instance in near:
		set_range(instance, 0.0, NEAR_END_M)
		node.add_child(instance)

	var steel_parts: Array = [[leg_mesh, leg_transforms]]
	for face in range(FACES):
		var turn := Transform3D(Basis(Vector3.UP, face * 0.5 * PI), Vector3.ZERO)
		var turned: Array[Transform3D] = []
		for t in face_transforms:
			turned.append(turn * t)
		steel_parts.append([brace_mesh, turned])
	if not roof.is_empty():
		steel_parts.append([brace_mesh, roof])
	var mid := MeshInstance3D.new()
	mid.name = "mid"
	mid.mesh = _merged([steel_parts, [[unit_box, solids]]], [steel, dark])
	mid.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	set_range(mid, NEAR_END_M, MID_END_M)
	node.add_child(mid)

	var far := MeshInstance3D.new()
	far.name = "far"
	var far_box := BoxMesh.new()
	far_box.size = Vector3(width, height, width)
	far.mesh = far_box
	far.material_override = _far_material(detail, section, leg_size, brace_size, half)
	far.position = Vector3(0.0, 0.5 * height, 0.0)
	far.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	set_range(far, MID_END_M, 0.0)
	node.add_child(far)

	node.add_child(_shadow_proxy(width, height, float(section["pitch_m"]), brace_size))
	return node


## Legs: profile +Z (length) -> up, x stays, y -> -z; origin at the section's lower end.
static func _leg_transforms(height: float, half: float) -> Array[Transform3D]:
	var transforms: Array[Transform3D] = []
	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			transforms.append(Transform3D(Basis(Vector3.RIGHT, Vector3(0.0, 0.0, -1.0), Vector3(0.0, height, 0.0)),
					Vector3(sx * half, 0.0, sz * half)))
	return transforms


## Members of one face as [from, to] pairs in the face plane (x along the face, y up from the section's lower edge).
## Public because the headless renderer keeps no MultiMesh transforms: tests read the layout here.
## Patterns: x_brace (two crossing diagonals), k_brace (two diagonals meeting at the middle of the far side),
## chevron (two diagonals meeting at the middle of the panel's upper edge, a Lambda); each plus the lower horizontal.
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
			"chevron":
				members.append([Vector2(-half, y0), Vector2(0.0, y1)])
				members.append([Vector2(half, y0), Vector2(0.0, y1)])
			_:  # x_brace
				members.append([Vector2(-half, y0), Vector2(half, y1)])
				members.append([Vector2(half, y0), Vector2(-half, y1)])
		members.append([Vector2(-half, y0), Vector2(half, y0)])
	if is_top:
		members.append([Vector2(-half, section["height_m"]), Vector2(half, section["height_m"])])
	return members


## Unit-cube transforms of the dark solids of a section: a floor deck at every brace row, the inner column bundle, on the
## first section the foot ring, on the top one the roof cap. (Unit BoxMesh scaled per instance.)
static func _solids(detail: Dictionary, section: Dictionary, half: float, is_first: bool,
		is_top: bool) -> Array[Transform3D]:
	var transforms: Array[Transform3D] = []
	var height: float = section["height_m"]
	var panels := panel_count(section)
	var panel_h := height / panels
	var deck: Dictionary = detail["tower.deck"]
	var thickness: float = deck["thickness_m"]
	var span := 2.0 * (half - float(deck["inset_m"]))
	for p in range(panels):
		transforms.append(_box(Vector3(span, thickness, span), Vector3(0.0, p * panel_h + 0.5 * thickness, 0.0)))
	var core: float = detail["tower.core"]["size_m"]
	transforms.append(_box(Vector3(core, height, core), Vector3(0.0, 0.5 * height, 0.0)))
	if is_top:
		transforms.append(_box(Vector3(span, thickness, span), Vector3(0.0, height - 0.5 * thickness, 0.0)))
		var roof: Dictionary = detail["tower.roof"]
		var cap := float(section["face_width_m"]) + 2.0 * float(roof["overhang_m"])
		var cap_thickness: float = roof["plate_thickness_m"]
		transforms.append(_box(Vector3(cap, cap_thickness, cap), Vector3(0.0, height + 0.5 * cap_thickness, 0.0)))
	if is_first:
		var foot: Dictionary = detail["tower.foot"]
		var ring: float = foot["ring_width_m"]
		var ring_thickness: float = foot["ring_thickness_m"]
		var width: float = section["face_width_m"]
		var centre := 0.5 * width + 0.5 * ring
		for side in [-1.0, 1.0]:
			transforms.append(_box(Vector3(width + 2.0 * ring, ring_thickness, ring),
					Vector3(0.0, 0.5 * ring_thickness, side * centre)))
			transforms.append(_box(Vector3(ring, ring_thickness, width),
					Vector3(side * centre, 0.5 * ring_thickness, 0.0)))
	return transforms


## The shallow pyramid frame on the roof cap: four rafters from the cap's corners to the flat top, and its top ring.
static func _roof_members(detail: Dictionary, section: Dictionary, half: float) -> Array[Transform3D]:
	var roof: Dictionary = detail["tower.roof"]
	var base_y := float(section["height_m"]) + float(roof["plate_thickness_m"])
	var top_y := base_y + float(roof["pyramid_height_m"])
	var top_half := half * float(roof["pyramid_top_fraction"])
	var transforms: Array[Transform3D] = []
	var corners: Array[Vector2] = [Vector2(-1.0, -1.0), Vector2(1.0, -1.0), Vector2(1.0, 1.0), Vector2(-1.0, 1.0)]
	for k in range(corners.size()):
		var c := corners[k]
		var n := corners[(k + 1) % corners.size()]
		transforms.append(_along(Vector3(c.x * half, base_y, c.y * half), Vector3(c.x * top_half, top_y, c.y * top_half)))
		transforms.append(_along(Vector3(c.x * top_half, top_y, c.y * top_half),
				Vector3(n.x * top_half, top_y, n.y * top_half)))
	return transforms


static func _box(size: Vector3, centre: Vector3) -> Transform3D:
	return Transform3D(Basis.from_scale(size), centre)


## Profile +Z along from -> to (scaled to its length); the cross-section's x axis stays horizontal.
static func _along(from: Vector3, to: Vector3) -> Transform3D:
	var along := to - from
	var direction := along.normalized()
	var reference := Vector3.UP if absf(direction.y) < 0.99 else Vector3.RIGHT
	var side := reference.cross(direction).normalized()
	return Transform3D(Basis(side, direction.cross(side), along), from)


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


## One mesh with a surface per material from `groups` ([[mesh, transforms], ...] each); the headless renderer returns no
## surface arrays, then the mesh stays empty (tests count the instances instead).
static func _merged(groups: Array, materials: Array) -> ArrayMesh:
	var result := ArrayMesh.new()
	if DisplayServer.get_name() == "headless":
		return result
	for g in range(groups.size()):
		var surface := SurfaceTool.new()
		surface.begin(Mesh.PRIMITIVE_TRIANGLES)
		for part: Array in groups[g]:
			for transform: Transform3D in part[1]:
				surface.append_from(part[0], 0, transform)
		surface.commit(result)
		result.surface_set_material(result.get_surface_count() - 1, materials[g])
	return result


## Visibility range [begin, end] (0: open), dither-fading each side by FADE_MARGIN_M when that is above 0.
static func set_range(instance: GeometryInstance3D, begin_m: float, end_m: float) -> void:
	instance.visibility_range_begin = begin_m
	instance.visibility_range_end = end_m
	instance.visibility_range_begin_margin = FADE_MARGIN_M if begin_m > 0.0 else 0.0
	instance.visibility_range_end_margin = FADE_MARGIN_M if end_m > 0.0 else 0.0
	instance.visibility_range_fade_mode = GeometryInstance3D.VISIBILITY_RANGE_FADE_SELF if FADE_MARGIN_M > 0.0 			else GeometryInstance3D.VISIBILITY_RANGE_FADE_DISABLED


static func _far_material(detail: Dictionary, section: Dictionary, leg_size: float, brace_size: float,
		half: float) -> ShaderMaterial:
	var material := ShaderMaterial.new()
	material.shader = FAR_SHADER
	material.set_shader_parameter("face_width_m", section["face_width_m"])
	material.set_shader_parameter("height_m", section["height_m"])
	material.set_shader_parameter("panel_m", float(section["height_m"]) / panel_count(section))
	material.set_shader_parameter("half_m", half)
	material.set_shader_parameter("leg_m", leg_size)
	material.set_shader_parameter("brace_m", brace_size)
	material.set_shader_parameter("deck_m", float(detail["tower.deck"]["thickness_m"]) * FAR_DECK_VIEW_FACTOR)
	material.set_shader_parameter("core_m", detail["tower.core"]["size_m"])
	material.set_shader_parameter("crossed", section["pattern"] == "x_brace")
	return material


## Box of the section with the lattice shader: invisible to the camera, its alpha-cut pattern casts the shadow.
static func _shadow_proxy(width: float, height: float, pitch_m: float, brace_size: float) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(width, height, width)
	var material := ShaderMaterial.new()
	material.shader = LATTICE_SHADER
	material.set_shader_parameter("bay_m", pitch_m)
	material.set_shader_parameter("member_fraction", 0.5 * brace_size / pitch_m)
	var proxy := MeshInstance3D.new()
	proxy.name = "shadow_proxy"
	proxy.mesh = mesh
	proxy.material_override = material
	proxy.position = Vector3(0.0, 0.5 * height, 0.0)
	proxy.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_SHADOWS_ONLY
	return proxy
