class_name LatticeBuilder
## Pad B tower body from the detail file (config/structures/starbase_olp2.toml): per section one MultiMesh of the four
## corner legs, one per face of the braces, one of the dark solids (floor decks, inner column bundle, stairs, cable trays,
## roof deck, louvres, railing) and one of the light-grey solids (plinth, cladding plates on the pad-side face, the roof
## block), instanced from the generated profile meshes (unit length along +Z) and unit boxes.
## Local frame of the returned node = the frame of TowerView's tower box: origin at the centre of the footprint on the
## ground, +X across, +Y up, +Z away from the pad. Face 0 is the face at -Z (toward the pad); faces 1..3 follow by
## 90 degrees about Y.
## Distance hand-over (visibility ranges, hard swap; all sections of the tower swap at the same distance, see
## _share_range_centre):
##   near  (< NEAR_END_M)            the MultiMeshes (every member an instance),
##   mid   (NEAR_END_M .. MID_END_M) one merged mesh of the same geometry: 4 draw calls per section (legs, braces, dark,
##                                   light solids) instead of ~10,
##   far   (> MID_END_M)             a box with the lattice shader tower_far.gdshader (analytic, filtered coverage of the
##                                   measured pitch: members thinner than a pixel become a density, never shimmer).
## Members cast no shadow: the game's Sun has no shadow maps (the ground shader is unshaded), so nothing would receive it.

const FACES := 4
const LEGS := 4
## Braces per panel: two diagonals and the horizontal at the panel's lower edge (the topmost section adds one at its top).
const BRACES_PER_PANEL := 3
## [E] Hand-over ranges. A brace (0.5 m) is 1.2 px wide at ~310 m on a 60 deg / 900 px desktop view (779 px focal length)
## and 2.4 px on a VR eye (1500 px): individual members go at 300 m. The merged mesh (same geometry, a few draw calls per
## section) runs to 5 km, where the tower is only 2 px (desktop) to 3.6 px (VR) wide and the lattice shader (filtered)
## takes over: the tiers' mean luminance agree within a few % at 4.5-6.5 km (tower_handover_check, both focal lengths).
## At 1 km, where the far tier began before, it showed visibly less truss than the merged mesh at VR focal lengths.
const NEAR_END_M := 300.0
const MID_END_M := 5000.0
## Dither fade length each side of a range boundary (Godot: both neighbours fade over boundary +- this). 0 = hard swap:
## measured (tower_handover_check): with 60 m the two dithered neighbours were not complementary (the tower's rectangle
## 2.6 % brighter before the boundary and 3.6 % darker after it), while the tiers shown alone agree within 0.3 % (members /
## merged) and 0.1 % (merged / shader at 800-1000 m): a hard swap of matched tiers has no visible band.
const FADE_MARGIN_M := 0.0
## Visible share of a deck seen through the lattice at a grazing angle (far shader): thickness x this.
const FAR_DECK_VIEW_FACTOR := 1.6
const FAR_SHADER := preload("res://shaders/tower_far.gdshader")
## Half extent (m) of the shared range box across the tower: the corner legs, the plinth and the roof cap sit inside it.
const RANGE_HALF_WIDTH_M := 12.0
## Margin (m) of the shared range box above the tower's top and below its base (roof block, railing).
const RANGE_MARGIN_M := 10.0
## Width (m) the legs' inner faces leave for the cladding plates to overlap the legs by (plates sit between the legs).
const CLAD_LEG_OVERLAP_M := 0.2
## Distance (m) of a stair flight's centre line inside the leg line, and of a cable tray inside the back face's braces.
const STAIR_INSET_M := 2.4
const TRAY_OFFSET_M := 0.5
const TRAY_X_M := 2.5
## Gap (m) of the top frame's horizontal under the roof deck, and the cap's inset of the railing.
const TOP_FRAME_GAP_M := 0.4
const RAILING_INSET_M := 0.1


## Panels of a section: its height over the gusset pitch, at least one.
static func panel_count(section: Dictionary) -> int:
	return maxi(1, roundi(float(section["height_m"]) / float(section["pitch_m"])))


## Floor levels of a section (decks): its height over the tower's floor pitch, at least one.
static func floor_count(detail: Dictionary, section: Dictionary) -> int:
	return maxi(1, roundi(float(section["height_m"]) / float(detail["tower.floors"]["pitch_m"])))


## Braces on one face of a section; `is_top` marks the topmost one.
static func braces_per_face(section: Dictionary, is_top: bool) -> int:
	return panel_count(section) * BRACES_PER_PANEL + (1 if is_top else 0)


## Height of the lattice (sum of the sections): the top of the last panel.
static func total_height_m(detail: Dictionary) -> float:
	var height := 0.0
	for section: Dictionary in detail["tower.sections"]:
		height += float(section["height_m"])
	return height


## Centre line of the legs and braces from the face width and the leg profile's width.
static func leg_line_m(face_width_m: float, leg_size_m: float) -> float:
	return 0.5 * face_width_m - 0.5 * leg_size_m


## Material set of build(): {"leg", "brace", "dark", "clad"}.
## detail: TomlLite dictionary of the detail file; profiles: {name: Mesh}.
static func build(detail: Dictionary, profiles: Dictionary, materials: Dictionary) -> Node3D:
	var root := Node3D.new()
	root.name = "Lattice"
	var sections: Array = detail["tower.sections"]
	var base_m := 0.0
	var total_m := total_height_m(detail)
	for i in range(sections.size()):
		var node := _build_section(detail, i, base_m, profiles, materials)
		node.name = "section_%d" % i
		node.position = Vector3(0.0, base_m, 0.0)
		_share_range_centre(node, total_m, base_m)
		root.add_child(node)
		base_m += float(sections[i]["height_m"])
	return root


static func _build_section(detail: Dictionary, index: int, base_m: float, profiles: Dictionary,
		materials: Dictionary) -> Node3D:
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
	var leg_transforms := _leg_transforms(leg_height_m(detail, section, base_m, is_top), half)
	var members := face_members(section, half, is_top)
	var face_transforms: Array[Transform3D] = []
	for member: Array in members:
		face_transforms.append(_member_transform(member[0], member[1], half))
	var dark_boxes := dark_solids(detail, section, base_m, half, is_top)
	var clad_boxes := clad_solids(detail, section, base_m, half, leg_size, index == 0, is_top)
	var frame := top_frame_members(detail, section, base_m, half) if is_top else ([] as Array[Transform3D])
	var unit_box := BoxMesh.new()

	var near: Array[MultiMeshInstance3D] = [_instances("legs", leg_mesh, leg_transforms, materials["leg"])]
	for face in range(FACES):
		var instance := _instances("face_%d" % face, brace_mesh, face_transforms, materials["brace"])
		instance.rotation.y = face * 0.5 * PI
		near.append(instance)
	near.append(_instances("decks", unit_box, dark_boxes, materials["dark"]))
	if not clad_boxes.is_empty():
		near.append(_instances("clad", unit_box, clad_boxes, materials["clad"]))
	if not frame.is_empty():
		near.append(_instances("roof", brace_mesh, frame, materials["brace"]))
	for instance in near:
		set_range(instance, 0.0, NEAR_END_M)
		node.add_child(instance)

	var brace_parts: Array = []
	for face in range(FACES):
		var turn := Transform3D(Basis(Vector3.UP, face * 0.5 * PI), Vector3.ZERO)
		var turned: Array[Transform3D] = []
		for t in face_transforms:
			turned.append(turn * t)
		brace_parts.append([brace_mesh, turned])
	if not frame.is_empty():
		brace_parts.append([brace_mesh, frame])
	var mid := MeshInstance3D.new()
	mid.name = "mid"
	mid.mesh = _merged([[[leg_mesh, leg_transforms]], brace_parts, [[unit_box, dark_boxes]], [[unit_box, clad_boxes]]],
			[materials["leg"], materials["brace"], materials["dark"], materials["clad"]])
	mid.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	set_range(mid, NEAR_END_M, MID_END_M)
	node.add_child(mid)

	var far := MeshInstance3D.new()
	far.name = "far"
	var far_box := BoxMesh.new()
	far_box.size = Vector3(width, height, width)
	far.mesh = far_box
	far.material_override = _far_material(detail, section, base_m, leg_size, brace_size, half)
	far.position = Vector3(0.0, 0.5 * height, 0.0)
	far.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	set_range(far, MID_END_M, 0.0)
	node.add_child(far)
	if is_top:
		node.add_child(_far_roof(detail, base_m, materials))
	return node


## Height of the section's legs: the top section's run up to the roof deck's underside.
static func leg_height_m(detail: Dictionary, section: Dictionary, base_m: float, is_top: bool) -> float:
	if not is_top:
		return float(section["height_m"])
	var roof: Dictionary = detail["tower.roof"]
	return float(roof["deck_top_m"]) - float(roof["plate_thickness_m"]) - base_m


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


## Unit-box transforms (unit BoxMesh scaled per instance) of the dark solids of a section: a floor deck at every floor
## level, the inner column bundle, stair flights and cable trays above the plinth; on the top section the roof deck, the
## louvre bands of the roof block and the railing. `base_m`: the section's lower edge above the ground.
static func dark_solids(detail: Dictionary, section: Dictionary, base_m: float, half: float,
		is_top: bool) -> Array[Transform3D]:
	var transforms: Array[Transform3D] = []
	var height: float = section["height_m"]
	var floors := floor_count(detail, section)
	var floor_h := height / floors
	var deck: Dictionary = detail["tower.deck"]
	var thickness: float = deck["thickness_m"]
	var span := 2.0 * (half - float(deck["inset_m"]))
	for p in range(floors):
		transforms.append(_box(Vector3(span, thickness, span), Vector3(0.0, p * floor_h + 0.5 * thickness, 0.0)))
	var core: float = detail["tower.core"]["size_m"]
	transforms.append(_box(Vector3(core, height, core), Vector3(0.0, 0.5 * height, 0.0)))
	transforms.append_array(_stairs(detail, base_m, height, floors, half))
	if is_top:
		transforms.append(_box(Vector3(span, thickness, span), Vector3(0.0, height - 0.5 * thickness, 0.0)))
		transforms.append_array(_roof_dark(detail, section, base_m))
	return transforms


## Light-grey solids: the plinth on the first section, the cladding plates (courses with seams, columns across) on the
## pad-side face where the section overlaps the cladding's height range, the louvred block on the top section.
static func clad_solids(detail: Dictionary, section: Dictionary, base_m: float, half: float, leg_size_m: float,
		is_first: bool, is_top: bool) -> Array[Transform3D]:
	var transforms: Array[Transform3D] = []
	var plinth: Dictionary = detail["tower.plinth"]
	if is_first:
		var h: float = plinth["height_m"]
		var w: float = plinth["width_m"]
		transforms.append(_box(Vector3(w, h, w), Vector3(0.0, 0.5 * h, 0.0)))
	var clad: Dictionary = detail["tower.cladding"]
	var lo := maxf(float(clad["bottom_m"]), base_m)
	var hi := minf(float(clad["top_m"]), base_m + float(section["height_m"]))
	var seam: float = clad["seam_m"]
	var columns: int = clad["columns"]
	var span := 2.0 * (half - 0.5 * leg_size_m) + CLAD_LEG_OVERLAP_M
	var plate_w := (span - (columns - 1) * seam) / columns
	var z := -half - float(clad["recess_m"])
	var edge := float(clad["bottom_m"])
	while edge < hi - 1.0e-6:
		var next := minf(edge + float(clad["course_m"]), float(clad["top_m"]))
		var y0 := maxf(edge, lo)
		var y1 := minf(next, hi)
		if y1 - y0 > seam:
			var plate_h := y1 - y0 - seam
			for c in range(columns):
				var x := -0.5 * span + (c + 0.5) * plate_w + c * seam
				transforms.append(_box(Vector3(plate_w, plate_h, clad["thickness_m"]),
						Vector3(x, 0.5 * (y0 + y1) - base_m, z)))
		edge = next
	if is_top:
		var roof: Dictionary = detail["tower.roof"]
		var b: float = roof["block_size_m"]
		var bh: float = roof["block_height_m"]
		transforms.append(_box(Vector3(b, bh, b), Vector3(0.0, float(roof["block_base_m"]) + 0.5 * bh - base_m, 0.0)))
	return transforms


## Roof deck plate, louvre bands on the four faces of the block, and the railing (two rails and posts), dark.
static func _roof_dark(detail: Dictionary, section: Dictionary, base_m: float) -> Array[Transform3D]:
	var roof: Dictionary = detail["tower.roof"]
	var transforms: Array[Transform3D] = []
	var plate: float = roof["plate_thickness_m"]
	var cap := float(section["face_width_m"]) + 2.0 * float(roof["overhang_m"])
	var deck_y := float(roof["deck_top_m"]) - base_m
	transforms.append(_box(Vector3(cap, plate, cap), Vector3(0.0, deck_y - 0.5 * plate, 0.0)))
	var b: float = roof["block_size_m"]
	var bh: float = roof["block_height_m"]
	var louvres: int = roof["louvre_count"]
	var band_h: float = roof["louvre_height_m"]
	var depth: float = roof["louvre_depth_m"]
	var block_y := float(roof["block_base_m"]) - base_m
	for k in range(louvres):
		var y := block_y + (k + 0.5) * bh / louvres
		for side in [-1.0, 1.0]:
			var offset: float = side * (0.5 * b + 0.5 * depth)
			transforms.append(_box(Vector3(b, band_h, depth), Vector3(0.0, y, offset)))
			transforms.append(_box(Vector3(depth, band_h, b), Vector3(offset, y, 0.0)))
	var m: float = roof["railing_member_m"]
	var rail_h: float = roof["railing_height_m"]
	var extent := 0.5 * cap - RAILING_INSET_M
	for side in [-1.0, 1.0]:
		for rail_y in [rail_h - 0.5 * m, 0.5 * rail_h]:
			transforms.append(_box(Vector3(2.0 * extent, m, m), Vector3(0.0, deck_y + rail_y, side * extent)))
			transforms.append(_box(Vector3(m, m, 2.0 * extent), Vector3(side * extent, deck_y + rail_y, 0.0)))
		var posts := maxi(1, roundi(2.0 * extent / float(roof["railing_post_pitch_m"])))
		for k in range(posts + 1):
			var along := -extent + 2.0 * extent * k / posts
			transforms.append(_box(Vector3(m, rail_h, m), Vector3(along, deck_y + 0.5 * rail_h, side * extent)))
			if k > 0 and k < posts:  # the corner posts are already on the other pair of sides
				transforms.append(_box(Vector3(m, rail_h, m), Vector3(side * extent, deck_y + 0.5 * rail_h, along)))
	return transforms


## Zig-zag stair flights on the +X side (one per floor above the plinth) and two cable trays up the +Z face.
static func _stairs(detail: Dictionary, base_m: float, height: float, floors: int, half: float) -> Array[Transform3D]:
	var stairs: Dictionary = detail["tower.stairs"]
	var transforms: Array[Transform3D] = []
	var pitch: float = detail["tower.floors"]["pitch_m"]
	var plinth_m: float = detail["tower.plinth"]["height_m"]
	var floor_h := height / floors
	var x := half - STAIR_INSET_M
	var run: float = stairs["run_m"]
	for j in range(floors):
		var y0 := base_m + j * floor_h
		if y0 < plinth_m - 1.0e-3:
			continue
		var global_floor := roundi(y0 / pitch)
		var z0 := -0.5 * run if global_floor % 2 == 0 else 0.5 * run
		transforms.append(_slab(Vector3(x, j * floor_h, z0), Vector3(x, (j + 1) * floor_h, -z0),
				stairs["width_m"], stairs["thickness_m"]))
	var tray_lo := maxf(plinth_m, base_m)
	var tray_hi := base_m + height
	if tray_hi > tray_lo:
		for side in [-1.0, 1.0]:
			transforms.append(_box(Vector3(stairs["tray_width_m"], tray_hi - tray_lo, stairs["tray_depth_m"]),
					Vector3(side * TRAY_X_M, 0.5 * (tray_lo + tray_hi) - base_m, half - TRAY_OFFSET_M)))
	return transforms


## The top section's frame under the roof deck: a horizontal member on each face (profile along the face).
static func top_frame_members(detail: Dictionary, section: Dictionary, base_m: float,
		half: float) -> Array[Transform3D]:
	var roof: Dictionary = detail["tower.roof"]
	var y := float(roof["deck_top_m"]) - float(roof["plate_thickness_m"]) - TOP_FRAME_GAP_M - base_m
	var transforms: Array[Transform3D] = []
	var corners: Array[Vector2] = [Vector2(-1.0, -1.0), Vector2(1.0, -1.0), Vector2(1.0, 1.0), Vector2(-1.0, 1.0)]
	for k in range(corners.size()):
		var c := corners[k]
		var n := corners[(k + 1) % corners.size()]
		transforms.append(_along(Vector3(c.x * half, y, c.y * half), Vector3(n.x * half, y, n.y * half)))
	return transforms


## Far tier of the roof: the block and the deck plate as two plain boxes (the far box ends at the lattice top).
static func _far_roof(detail: Dictionary, base_m: float, materials: Dictionary) -> MeshInstance3D:
	var roof: Dictionary = detail["tower.roof"]
	var section: Dictionary = detail["tower.sections"][detail["tower.sections"].size() - 1]
	var plate: float = roof["plate_thickness_m"]
	var cap := float(section["face_width_m"]) + 2.0 * float(roof["overhang_m"])
	var b: float = roof["block_size_m"]
	var bh: float = roof["block_height_m"]
	var unit_box := BoxMesh.new()
	var block_y := float(roof["block_base_m"]) + 0.5 * bh - base_m
	var block: Array[Transform3D] = [_box(Vector3(b, bh, b), Vector3(0.0, block_y, 0.0))]
	var deck: Array[Transform3D] = [_box(Vector3(cap, plate, cap),
			Vector3(0.0, float(roof["deck_top_m"]) - 0.5 * plate - base_m, 0.0))]
	var far := MeshInstance3D.new()
	far.name = "far_roof"
	far.mesh = _merged([[[unit_box, block]], [[unit_box, deck]]], [materials["clad"], materials["dark"]])
	far.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	set_range(far, MID_END_M, 0.0)
	return far


static func _box(size: Vector3, centre: Vector3) -> Transform3D:
	return Transform3D(Basis.from_scale(size), centre)


## A flat slab (unit box scaled to width x thickness x length) from one end centre line point to the other.
static func _slab(from: Vector3, to: Vector3, width: float, thickness: float) -> Transform3D:
	var along := to - from
	var side := Vector3.RIGHT
	var normal := along.cross(side).normalized()
	return Transform3D(Basis(side * width, normal * thickness, along), 0.5 * (from + to))


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
## surface arrays, then the mesh stays empty (tests count the instances instead). A group without parts adds no surface.
static func _merged(groups: Array, materials: Array) -> ArrayMesh:
	var result := ArrayMesh.new()
	if DisplayServer.get_name() == "headless":
		return result
	for g in range(groups.size()):
		var surface := SurfaceTool.new()
		surface.begin(Mesh.PRIMITIVE_TRIANGLES)
		var parts := 0
		for part: Array in groups[g]:
			for transform: Transform3D in part[1]:
				surface.append_from(part[0], 0, transform)
				parts += 1
		if parts == 0:
			continue
		surface.commit(result)
		result.surface_set_material(result.get_surface_count() - 1, materials[g])
	return result


## Godot measures a visibility range from the centre of the instance's bounding box, so tiers of sections at different heights
## (and the four faces' MultiMeshes, whose boxes are ~5 m off the middle) would swap at different distances: a tower half in
## one tier and half in the other. Every tier node of the tower gets the same box centre, the tower's middle, so the whole
## tower swaps at once. `node` is the section node (at `base_m` above the lattice origin); the box holds all members.
static func _share_range_centre(node: Node3D, total_m: float, base_m: float) -> void:
	var half := Vector3(RANGE_HALF_WIDTH_M, 0.5 * total_m + RANGE_MARGIN_M, RANGE_HALF_WIDTH_M)
	var centre := Vector3(0.0, 0.5 * total_m - base_m, 0.0)
	for child in node.get_children():
		var geometry := child as GeometryInstance3D
		if geometry != null:
			geometry.custom_aabb = AABB(centre - geometry.position - half, 2.0 * half)  # in the node's own frame


## Visibility range [begin, end] (0: open), dither-fading each side by FADE_MARGIN_M when that is above 0.
static func set_range(instance: GeometryInstance3D, begin_m: float, end_m: float) -> void:
	instance.visibility_range_begin = begin_m
	instance.visibility_range_end = end_m
	instance.visibility_range_begin_margin = FADE_MARGIN_M if begin_m > 0.0 else 0.0
	instance.visibility_range_end_margin = FADE_MARGIN_M if end_m > 0.0 else 0.0
	instance.visibility_range_fade_mode = (GeometryInstance3D.VISIBILITY_RANGE_FADE_SELF if FADE_MARGIN_M > 0.0
			else GeometryInstance3D.VISIBILITY_RANGE_FADE_DISABLED)


static func _far_material(detail: Dictionary, section: Dictionary, base_m: float, leg_size: float, brace_size: float,
		half: float) -> ShaderMaterial:
	var material := ShaderMaterial.new()
	var height: float = section["height_m"]
	var clad: Dictionary = detail["tower.cladding"]
	material.shader = FAR_SHADER
	material.set_shader_parameter("face_width_m", section["face_width_m"])
	material.set_shader_parameter("height_m", height)
	material.set_shader_parameter("panel_m", height / panel_count(section))
	material.set_shader_parameter("floor_m", height / floor_count(detail, section))
	material.set_shader_parameter("half_m", half)
	material.set_shader_parameter("leg_m", leg_size)
	material.set_shader_parameter("brace_m", brace_size)
	material.set_shader_parameter("deck_half_m", half - float(detail["tower.deck"]["inset_m"]))
	material.set_shader_parameter("deck_m", float(detail["tower.deck"]["thickness_m"]) * FAR_DECK_VIEW_FACTOR)
	material.set_shader_parameter("core_m", detail["tower.core"]["size_m"])
	material.set_shader_parameter("crossed", section["pattern"] == "x_brace")
	material.set_shader_parameter("plinth_m", clampf(float(detail["tower.plinth"]["height_m"]) - base_m, 0.0, height))
	material.set_shader_parameter("clad_lo_m", clampf(float(clad["bottom_m"]) - base_m, 0.0, height))
	material.set_shader_parameter("clad_hi_m", clampf(float(clad["top_m"]) - base_m, 0.0, height))
	material.set_shader_parameter("clad_half_m", half - 0.5 * leg_size)
	return material
