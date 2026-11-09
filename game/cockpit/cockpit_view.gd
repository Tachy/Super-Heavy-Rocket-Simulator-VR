class_name CockpitView
extends Node3D
## The Crew Dragon's cabin (docs/subsystems/cockpit.md) as a child of the capsule stage node: wall, floor, seats, console with
## screens / buttons / tablets, hatch, windows, the HOTAS props, the cabin lamps and the crew figures. Everything is in the
## capsule stage frame (the node's own transform is the identity); the stage node maps it to render axes. The cabin is part
## of the vehicle, so it exists in every view; the pilot camera (a later task) sits at `eye_node`.

const CABIN_SHADER := preload("res://shaders/cabin.gdshader")
const GLASS_SHADER := preload("res://shaders/cabin_glass.gdshader")
const DISPLAY_SHADER := preload("res://shaders/display_colour.gdshader")
const VEHICLE_ID := "crew_dragon"
const FIGURE_FILE := "crew_figure"
## Seats in the order the crew fills them: commander (the player body), pilot, then the outer seats.
const CREW_ORDER: Array[String] = ["s2", "s3", "s1", "s4"]
## [call] Look pass: luminous flux of one ceiling lamp panel and of the cabin fill (stands in for the light the white walls bounce).
const LAMP_LUMENS := 3000.0
const FILL_LUMENS := 7000.0
const LAMP_RANGE_M := 6.0
## [E] The cabin lamps' colour (sRGB-encoded Light3D colour): white LED panels, D65 (crew1_1 reads them neutral white).
const LAMP_COLOUR := Color.WHITE
## [call] Look pass 2: the light the white walls bounce around the closed cabin (no GI): a flat illuminance on every cabin
## surface as emission albedo x E / pi, scene-referred like the lamps (so the eye adaptation sees a lit cabin at any hour).
const FILL_LUX := 450.0
## [E] The white walls / floor (sRGB): also the surround the glossy cabin parts reflect (fix round 5: the closed cabin reflects its walls, not the sky).
const WALL_ALBEDO := Color(0.8, 0.8, 0.79)
## Lamp offset in front of its panel on the console (toward the crew, along the console normal).
const LAMP_DROP_M := 0.08
## [E] The ceiling ring light around the forward tunnel (crew1_1: a bright white annulus, r ~0.45..0.9 m at the ceiling):
## RING_LAMPS omnis on the mid radius of [cabin] ring_light_radii_m, RING_LUMENS in total (an area light's stand-in; its emissive mesh `ceiling_ring_light` is the generator's).
const RING_LAMPS := 8
const RING_RADIUS_FALLBACK_M := 0.675
const RING_LUMENS := 6000.0
const RING_DROP_M := 0.10
## [E] Screen placeholder: a dark Dragon-blue UI background, display-referred (no exposure, as every emitter).
const SCREEN_COLOUR := Color(0.025, 0.07, 0.17)
## [E] crew1_1 (round 4): the dark vent band across the console's silver back, in the console's up coordinate from the centre screen centre
## (the back face spans -0.235..+0.265: the band at 35-75 % of its height from the top) and to 0.58 m either side of the centre (slot 1.20 of the 1.32 m back).
## Render layer of the console parts (round 4): the two console lamps hang 8 cm in front of the console face and light it only at grazing
## angles (their shadow maps there are prone to acne). The concentric rings at the top of pilot_eye were NOT this: they were the aerial
## perspective's float32 noise over sub-metre paths (fix round 5, atm_to_point_offset's short-path form). The lamps face the crew, so they do not light the console (light_cull_mask without this layer); the ring and fill lights still do.
const CONSOLE_LAYER := 1 << 11
const CONSOLE_VENT_UP_M := Vector2(-0.11, 0.07)
const CONSOLE_VENT_HALF_WIDTH_M := 0.58
## [E] Task 5b, conv_42860a / discovery10: a red light strip along the bottom lip of each shelf module (the strip's height on the module face) and its
## luminance (an indicator strip, dim red: it reads saturated red, not white, in the 2020 photo); colour linear Rec.709 at luminance 1.
const SHELF_STRIP_HEIGHT_M := 0.010
const SHELF_STRIP_CD_M2 := 40.0
const SHELF_STRIP_COLOUR := Vector3(4.2, 0.08, 0.05)
## [E] conv_42860a: the T-handle housing ends in a red marking at its lower right (the crew's right) corner: share of the housing width and
## height and its colour (sRGB); the first render's full-height 22 % block read far larger than the photo's red tab.
const HANDLE_MARK_SHARE := 0.15
const HANDLE_MARK_HEIGHT_SHARE := 0.40
const HANDLE_MARK_ALBEDO := Color(0.55, 0.03, 0.03)
## [E] Key labels: printed light grey letters on the dark keys (conv_42860a), sRGB.
const LABEL_ALBEDO := Color(0.80, 0.80, 0.80)
## [E] Task 8: the torso leans (about the waist) when the pointer is farther than this share of the straight arm's reach, by at most
## MAX_LEAN_DEG (a seated, harnessed crew member's lean towards the console).
const LEAN_REACH_SHARE := 0.97
const MAX_LEAN_DEG := 22.0
const LEAN_PASSES := 4

var config: CockpitConfig
var eye_node := Node3D.new()
var _nodes := {}  # cabin node name -> MeshInstance3D
var _lights: Array[OmniLight3D] = []
var _light_cd: Array[float] = []
var _figures := {}  # seat id -> Node3D (figure root)
var _skeletons := {}  # seat id -> Skeleton3D
var _crew := 0
var _materials := {}
var _strip_materials: Array[ShaderMaterial] = []
var _cockpit_manifest := {}
var _label_atlas: ImageTexture = null
var _figure_bones := {}  # manifest crew_figure.bones (the arm chains' finger lengths)
var _arm_iks := {}  # side "l" / "r" -> ArmIk of the player's figure
var _arm_results := {}  # side -> the last solved pose (ArmIk.solve)
var _lean := Quaternion.IDENTITY  # the torso's lean towards the pointer (skeleton frame)


## Builds the cabin from the model set's `models` (VehicleModels.load_for("crew_dragon")) and the cockpit definition.
func build(models: Dictionary, cockpit: CockpitConfig) -> void:
	config = cockpit
	name = "cockpit"
	eye_node.name = "eye"
	eye_node.transform = config.eye
	add_child(eye_node)
	var cabin: Dictionary = models.get("cabin", {})
	_cockpit_manifest = (models.get("manifest", {}) as Dictionary).get("cockpit", {})
	for part: String in cabin:
		var instance := MeshInstance3D.new()
		instance.name = part
		instance.mesh = cabin[part]
		instance.material_override = _material_of(part)
		if part.begins_with("window_glass") or part.begins_with("light_") or part.begins_with("screen_") or part == "ceiling_ring_light" or part == "console_lamps":
			instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		if _on_console(part):
			instance.layers = CONSOLE_LAYER
		add_child(instance)
		_nodes[part] = instance
	_build_lights()
	_build_figures(models)
	set_hotas_props(config.hotas_props_enabled)
	set_crew(config.seats.size())


func _on_console(part: String) -> bool:
	return part.begins_with("console_") or part.begins_with("tablet_") or part.begins_with("button_") or part == "handle" 			or part.begins_with("light_")


## The mesh of a screen ("left", "centre", "right").
func screen_surface(id: String) -> MeshInstance3D:
	return _nodes.get("screen_" + id)


## The node of a button (config id; the centre handle is "handle").
func button_node(id: String) -> Node3D:
	return _nodes.get("handle" if id == "handle" else "button_" + id)


func cabin_node(node_name: String) -> MeshInstance3D:
	return _nodes.get(node_name)


## Crew of `count` (0..seats): figures in the seats in CREW_ORDER; the first is the player body (commander).
func set_crew(count: int) -> void:
	_crew = clampi(count, 0, CREW_ORDER.size())
	for i in CREW_ORDER.size():
		var figure: Node3D = _figures.get(CREW_ORDER[i])
		if figure != null:
			figure.visible = i < _crew


func crew_count() -> int:
	return _crew


func visible_figures() -> int:
	var n := 0
	for figure: Node3D in _figures.values():
		n += 1 if figure.visible else 0
	return n


## The figure's skeleton of a seat ("s1".."s4"; null without the model): the arm poses of the pointer (later task) set its bones.
func figure_skeleton(seat_id: String) -> Skeleton3D:
	return _skeletons.get(seat_id)


## The player's arm chain of a side ("l" crew's left, "r"), null without the figure.
func player_arm(side: String) -> ArmIk:
	if not _arm_iks.has(side):
		var skeleton: Skeleton3D = _skeletons.get(CREW_ORDER[0])
		_arm_iks[side] = ArmIk.for_skeleton(skeleton, side, _figure_bones) if skeleton != null else null
	return _arm_iks[side]


## The resting fingertip of a side in the cabin frame (the hand on its armrest / HOTAS grip; Zero without the figure).
func idle_tip(side: String) -> Vector3:
	var arm := player_arm(side)
	return _skeleton_to_cabin() * arm.rest_tip() if arm != null else Vector3.ZERO


## The last solved pose of a side (ArmIk.solve result with the tip in the SKELETON frame), {} before the first pose_arms.
func arm_result(side: String) -> Dictionary:
	return _arm_results.get(side, {})


## The fingertip of a side as posed, in the cabin frame (Zero before the first pose_arms).
func posed_tip(side: String) -> Vector3:
	var result: Dictionary = _arm_results.get(side, {})
	return _skeleton_to_cabin() * (result["tip"] as Vector3) if not result.is_empty() else Vector3.ZERO


## Poses the player's arms: `pose[side] = {tip: Vector3 (cabin frame), extend: float (0 relaxed hand .. 1 index straight)}`. The torso leans
## (a rotation about the waist, at most MAX_LEAN_DEG) when the target is beyond the straight arm's reach (the outer screen); beyond
## that the tip is clamped to the reach.
func pose_arms(pose: Dictionary) -> void:
	var skeleton: Skeleton3D = _skeletons.get(CREW_ORDER[0])
	if skeleton == null:
		return
	var to_skeleton := _skeleton_to_cabin().affine_inverse()
	var targets := {}
	var arms := {}
	for side: String in ["l", "r"]:
		arms[side] = player_arm(side)
		if arms[side] == null or not pose.has(side):
			return
		targets[side] = to_skeleton * (pose[side]["tip"] as Vector3)
	_apply_lean(skeleton, arms, targets)
	for side: String in ["l", "r"]:
		var arm: ArmIk = arms[side]
		var shoulder: Vector3 = _waist() + _lean * (arm.shoulder_rest - _waist())
		var result := arm.solve(shoulder, targets[side])
		arm.apply(skeleton, result, float(pose[side].get("extend", 0.0)))
		_arm_results[side] = result


func _waist() -> Vector3:
	var bone := (_skeletons[CREW_ORDER[0]] as Skeleton3D).find_bone("spine")
	return (_skeletons[CREW_ORDER[0]] as Skeleton3D).get_bone_global_rest(bone).origin


## The torso's lean: the rotation about the waist that brings the farther shoulder towards its target, limited and smooth in the target.
func _apply_lean(skeleton: Skeleton3D, arms: Dictionary, targets: Dictionary) -> void:
	var waist := _waist()
	var lean := Quaternion.IDENTITY
	var limit := deg_to_rad(MAX_LEAN_DEG)
	# the farther shoulder swings towards its target about the waist; a few passes since the swing is an arc, not a line
	var side := "l"
	var worst := -INF
	for s: String in ["l", "r"]:
		var e := (targets[s] as Vector3).distance_to((arms[s] as ArmIk).shoulder_rest) - LEAN_REACH_SHARE * (arms[s] as ArmIk).reach_m()
		if e > worst:
			worst = e
			side = s
	var arm: ArmIk = arms[side]
	for pass_index in LEAN_PASSES:
		var shoulder := waist + lean * (arm.shoulder_rest - waist)
		var excess := (targets[side] as Vector3).distance_to(shoulder) - LEAN_REACH_SHARE * arm.reach_m()
		if excess <= 0.0:
			break
		var v := shoulder - waist
		var moved := v + ((targets[side] as Vector3) - shoulder).normalized() * excess
		lean = ArmIk._arc(v.normalized(), moved.normalized()) * lean
		var angle := lean.get_angle()
		if angle > limit:
			lean = Quaternion.IDENTITY.slerp(lean, limit / angle)
	_lean = lean
	var bone := skeleton.find_bone("spine")
	var parent_basis := skeleton.get_bone_global_rest(skeleton.get_bone_parent(bone)).basis.orthonormalized()
	var rest_local := skeleton.get_bone_rest(bone)
	skeleton.set_bone_pose_rotation(bone, (parent_basis.inverse() * Basis(lean) * parent_basis * rest_local.basis.orthonormalized()).get_rotation_quaternion())


## The player skeleton's frame in the cabin frame.
func _skeleton_to_cabin() -> Transform3D:
	var skeleton: Skeleton3D = _skeletons.get(CREW_ORDER[0])
	var t := Transform3D.IDENTITY
	var node: Node = skeleton
	while node != null and node != self:
		if node is Node3D:
			t = (node as Node3D).transform * t
		node = node.get_parent()
	return t


## Hides (collapses) the head, helmet and visor of the player's body (the commander's figure): the camera sits inside that
## helmet. The pilot camera switches it on; the external views show the whole figure.
func set_player_head_hidden(hidden: bool) -> void:
	var skeleton: Skeleton3D = _skeletons.get(CREW_ORDER[0])
	if skeleton == null:
		return
	var bone := skeleton.find_bone("head")
	if bone >= 0:
		skeleton.set_bone_pose_scale(bone, Vector3.ONE * (0.001 if hidden else 1.0))


## The stick and throttle props on the commander's armrests (the deliberate deviation, switchable).
func set_hotas_props(enabled: bool) -> void:
	for part in ["hotas_stick", "hotas_throttle"]:
		var node: MeshInstance3D = _nodes.get(part)
		if node != null:
			node.visible = enabled


## Whether a point (render world) lies inside the cabin's inner wall of revolution ([cabin] profile_m, [x, r] about the
## capsule's x axis): the eye there sees the cabin lit by its lamps (World: Atmosphere.interior_light_colour).
func contains(point_global: Vector3) -> bool:
	if not is_inside_tree():
		return false
	var local := global_transform.affine_inverse() * point_global
	var profile: Array = config.cabin.get("profile_m", []) if config != null else []
	var radius := sqrt(local.y * local.y + local.z * local.z)
	for k in range(profile.size() - 1):
		var a: Array = profile[k]
		var b: Array = profile[k + 1]
		var x0 := float(a[0])
		var x1 := float(b[0])
		if x1 > x0 and local.x >= x0 and local.x <= x1:
			return radius < lerpf(float(a[1]), float(b[1]), (local.x - x0) / (x1 - x0))
	return false


## The cabin lamps' colour (linear Rec.709 at luminance 1): the omnis' light_color (white: D65) - the adopted white of
## the enclosed cabin (look fixes T5b fix round 1).
func light_colour() -> Vector3:
	var c := LAMP_COLOUR.srgb_to_linear()
	var v := Vector3(c.r, c.g, c.b)
	return v / v.dot(Vector3(0.2126, 0.7152, 0.0722))


func _process(_delta: float) -> void:
	var scene_per_cd: float = ShaderGlobals.value(&"atmosphere_scene_per_cd_m2")
	for i in _lights.size():
		_lights[i].light_energy = TowerLights.energy_of(_light_cd[i], scene_per_cd)
	var fill := FILL_LUX / PI * scene_per_cd
	var surround := fill * WALL_ALBEDO.srgb_to_linear().get_luminance()  # the walls' luminance under the same bounced light
	for material: Material in _materials.values():
		if material is ShaderMaterial and (material as ShaderMaterial).shader == CABIN_SHADER:
			(material as ShaderMaterial).set_shader_parameter("fill", fill)
			(material as ShaderMaterial).set_shader_parameter("fill_surround", surround)
	for material: ShaderMaterial in _strip_materials:
		material.set_shader_parameter("mark_emission", SHELF_STRIP_COLOUR * SHELF_STRIP_CD_M2 * scene_per_cd)


func _build_lights() -> void:
	for i in config.lights.size():
		var row: Dictionary = config.lights[i]
		var p: Vector3 = CockpitConfig._vec(row.get("position_m"))
		var lamp := _add_omni("light_%s" % row.get("id", i), p + _console_normal() * LAMP_DROP_M, LAMP_LUMENS)
		lamp.shadow_enabled = true  # two lamps: cheap shadows
		lamp.shadow_bias = 0.12  # the ceiling is lit at a grazing angle: the default bias showed radial shadow acne (pilot_eye, round 3)
		lamp.shadow_normal_bias = 3.0
		lamp.light_cull_mask &= ~CONSOLE_LAYER
	var ceiling_x := float(config.cabin.get("ceiling_x_m", 3.35))
	var radii: Array = config.cabin.get("ring_light_radii_m", [])
	var ring_r := 0.5 * (float(radii[0]) + float(radii[1])) if radii.size() == 2 else RING_RADIUS_FALLBACK_M
	for k in RING_LAMPS:
		var az := TAU * float(k) / RING_LAMPS
		_add_omni("ring_%d" % k, Vector3(ceiling_x - RING_DROP_M, ring_r * cos(az), ring_r * sin(az)), RING_LUMENS / RING_LAMPS)
	_add_omni("fill_ceiling", Vector3(ceiling_x - 0.25, 0.0, 0.0), FILL_LUMENS)  # 0.25 m under the ceiling plane (was 3.1 for the ceiling at 3.35)
	_add_omni("fill_seats", Vector3(2.0, 0.55, 0.0), FILL_LUMENS)


func _console_normal() -> Vector3:
	return CockpitConfig._vec(config.console.get("normal")).normalized()


func _add_omni(light_name: String, at: Vector3, lumens: float) -> OmniLight3D:
	var omni := OmniLight3D.new()
	omni.name = light_name
	omni.omni_attenuation = 2.0
	omni.omni_range = LAMP_RANGE_M
	omni.light_specular = 0.4
	omni.light_color = LAMP_COLOUR
	omni.shadow_enabled = false
	omni.position = at
	add_child(omni)
	_lights.append(omni)
	_light_cd.append(TowerLights.candela_omni(lumens))
	return omni


func _build_figures(models: Dictionary) -> void:
	var manifest: Dictionary = (models.get("manifest", {}) as Dictionary).get("crew_figure", {})
	var look: Dictionary = manifest.get("materials", {})
	_figure_bones = manifest.get("bones", {})
	var template := VehicleModels.scene_for(VEHICLE_ID, FIGURE_FILE)  # parsed once, duplicated per seat
	if template == null:
		return
	for seat: Dictionary in config.seats:
		var scene := template.duplicate()
		var seat_id := String(seat["id"])
		var angle := deg_to_rad(float(seat.get("back_angle_deg", 0.0)))
		var root := Node3D.new()
		root.name = "figure_" + seat_id
		root.transform = Transform3D(Basis(Vector3(cos(angle), -sin(angle), 0.0), Vector3(sin(angle), cos(angle), 0.0),
				Vector3(0.0, 0.0, 1.0)), seat["hip"])
		root.add_child(scene)
		add_child(root)
		_figures[seat_id] = root
		var skeletons := scene.find_children("*", "Skeleton3D", true, false)
		if not skeletons.is_empty():
			_skeletons[seat_id] = skeletons[0]
		for node in scene.find_children("*", "MeshInstance3D", true, false):
			var mesh_node := node as MeshInstance3D
			for s in mesh_node.mesh.get_surface_count():
				var source := mesh_node.mesh.surface_get_material(s)
				var key := source.resource_name if source != null else ""
				if look.has(key):
					var entry: Dictionary = look[key]
					var c: Array = entry["albedo"]
					var material := _cabin_material(Color(c[0], c[1], c[2]).linear_to_srgb(), float(entry["roughness"]), float(entry["metallic"]), 0.0)
					if key == String(manifest.get("visor_material", "")) and manifest.has("visor_tint"):
						material = _visor_material(material, manifest["visor_tint"])
					elif key == String(manifest.get("helmet_material", "")) and manifest.has("helmet_grooves"):
						material = _helmet_material(material, manifest["helmet_grooves"])
					mesh_node.set_surface_override_material(s, material)
	template.free()


func _material_of(part: String) -> Material:
	if part.begins_with("screen_"):
		return _display(SCREEN_COLOUR)
	if part.begins_with("window_glass"):
		var glass := ShaderMaterial.new()
		glass.shader = GLASS_SHADER
		return glass
	if part.begins_with("light_") or part == "ceiling_ring_light" or part == "console_lamps":
		return _display(Color(0.95, 0.95, 0.9))
	if part == "cabin_shell":
		return _cabin_material(WALL_ALBEDO, 0.85, 0.0, 0.5)
	if part == "floor" or part == "hatch_frame" or part.begins_with("window_"):
		return _cabin_material(WALL_ALBEDO, 0.8, 0.0, 0.0)
	## [E] Console (review item 3): the housing front is near black and matte (no specular sheen: it must not read beige under the lamps),
	## the shelf modules dark grey, the back and the posts silver with the dark vent band, the underside dark grey (crew1_1, round 4: not a white slab), the lockers dark.
	## The console colours are LINEAR albedos (photo measures), converted for the shader's sRGB uniform (round 5: they went in as sRGB, 0.07 became 0.006 linear).
	if part == "console_housing":
		return _cabin_material(Color(0.028, 0.028, 0.032).linear_to_srgb(), 0.9, 0.0, 0.0, 0.1)
	if part == "console_shelf":
		return _shelf_material()
	if part == "console_back":
		return _console_back_material()
	if part == "console_under":
		return _cabin_material(Color(0.10, 0.10, 0.11).linear_to_srgb(), 0.6, 0.2, 0.0, 0.3)
	if part == "console_lockers":
		return _cabin_material(Color(0.02, 0.02, 0.024).linear_to_srgb(), 0.85, 0.0, 0.0, 0.03)
	## [E] Seats (review item 4, crew1_1): white shells, black pads and black carbon leg-rest frames.
	if part.begins_with("seat_shell_"):
		return _cabin_material(Color(0.82, 0.82, 0.81), 0.5, 0.0, 0.0)  # sRGB 0.82 = linear 0.64: white paint, like the walls (WALL_ALBEDO, sRGB)
	if part.begins_with("seat_"):
		return _cabin_material(Color(0.035, 0.035, 0.037).linear_to_srgb(), 0.7, 0.0, 0.0, 0.2)  # Task 5b: LINEAR 0.035 (black pads / carbon), was the sRGB literal 0.03 = 0.0023 linear
	if part.begins_with("tablet_"):
		return _cabin_material(Color(0.03, 0.03, 0.036).linear_to_srgb(), 0.85, 0.0, 0.0, 0.03)  # glass front: dark, no metal sheen under the lamps
	if part.begins_with("hotas_"):
		return _cabin_material(Color(0.04, 0.04, 0.043).linear_to_srgb(), 0.5, 0.4, 0.0)  # Task 5b: LINEAR 0.04 (black plastic), was the sRGB literal 0.06 = 0.005 linear
	if part == "handle":
		return _handle_material()
	var key_id := part.trim_prefix("button_")
	var entry: Dictionary = (_cockpit_manifest.get("buttons", {}) as Dictionary).get(key_id, {})
	if entry.has("label_cell"):
		return _label_material(key_id, entry)
	return _key_base()  # buttons without a legible label: blank key faces


## [E] The keys: dark grey (conv_42860a: near-black keys on the dark grey modules). Task 5b: the earlier Color(0.09, 0.09, 0.1) literal went into
## the sRGB uniform unconverted (0.009 linear, darker than the module); now a linear 0.03 converted like the console colours.
func _key_base() -> ShaderMaterial:
	return _cabin_material(Color(0.03, 0.03, 0.033).linear_to_srgb(), 0.45, 0.3, 0.0)


## The shelf modules (discovery2 / conv_42860a: dark grey) with the red light strip along each module's bottom lip (mark, emission set each frame).
func _shelf_material() -> ShaderMaterial:
	if _materials.has("console_shelf"):
		return _materials["console_shelf"]
	var m := _cabin_material(Color(0.07, 0.07, 0.075).linear_to_srgb(), 0.8, 0.0, 0.0, 0.05).duplicate() as ShaderMaterial
	var centre: Dictionary = config.screens.get("centre", {})
	if not centre.is_empty():
		var n: Vector3 = (centre["normal"] as Vector3).normalized()
		var up: Vector3 = (centre["up"] as Vector3).normalized()
		var bottom := -0.5 * (centre["size"] as Vector2).y - float(config.console.get("shelf_top_gap_m", 0.0175)) - float(config.console.get("shelf_height_m", 0.10))
		_set_mark(m, centre["centre"], (-n).cross(up).normalized(), up, n, Vector2(-1.0, 1.0), Vector2(bottom - 0.01, bottom + SHELF_STRIP_HEIGHT_M), Color(0.35, 0.02, 0.02))
		_strip_materials.append(m)
	_materials["console_shelf"] = m
	return m


## The centre T-handle housing: the key colour with the red marking at its right end (conv_42860a).
func _handle_material() -> ShaderMaterial:
	var m := _key_base().duplicate() as ShaderMaterial
	var entry: Dictionary = (_cockpit_manifest.get("buttons", {}) as Dictionary).get("handle", {})
	if not entry.is_empty():
		var w := float((entry["size"] as Array)[0])
		var h := float((entry["size"] as Array)[1])
		_set_mark(m, CockpitConfig._vec(entry["centre"]), CockpitConfig._vec(entry["right"]), CockpitConfig._vec(entry["up"]), CockpitConfig._vec(entry["normal"]),
				Vector2(0.5 * w - HANDLE_MARK_SHARE * w, 0.5 * w + 0.01), Vector2(-0.5 * h - 0.01, -0.5 * h + HANDLE_MARK_HEIGHT_SHARE * h), HANDLE_MARK_ALBEDO)
	_materials["handle"] = m
	return m


func _set_mark(m: ShaderMaterial, origin: Vector3, right: Vector3, up: Vector3, normal: Vector3, lat: Vector2, up_range: Vector2, albedo: Color) -> void:
	m.set_shader_parameter("mark_on", 1.0)
	m.set_shader_parameter("mark_origin", origin)
	m.set_shader_parameter("mark_right", right)
	m.set_shader_parameter("mark_up", up)
	m.set_shader_parameter("mark_normal", normal)
	m.set_shader_parameter("mark_lat", lat)
	m.set_shader_parameter("mark_up_range", up_range)
	m.set_shader_parameter("mark_albedo", albedo)


## A labelled key: its own material with the atlas cell on the key face (manifest cockpit.buttons[id].label_cell; atlas cockpit.labels.file).
func _label_material(key_id: String, entry: Dictionary) -> ShaderMaterial:
	var atlas := _labels()
	if atlas == null:
		return _key_base()
	var m := _key_base().duplicate() as ShaderMaterial
	var cell: Array = entry["label_cell"]
	m.set_shader_parameter("label_on", 1.0)
	m.set_shader_parameter("label_atlas", atlas)
	m.set_shader_parameter("label_cell", Vector4(cell[0], cell[1], cell[2], cell[3]))
	m.set_shader_parameter("label_centre", CockpitConfig._vec(entry["centre"]))
	m.set_shader_parameter("label_right", CockpitConfig._vec(entry["right"]))
	m.set_shader_parameter("label_up", CockpitConfig._vec(entry["up"]))
	m.set_shader_parameter("label_normal", CockpitConfig._vec(entry["normal"]))
	m.set_shader_parameter("label_size", Vector2(float((entry["size"] as Array)[0]), float((entry["size"] as Array)[1])))
	m.set_shader_parameter("label_albedo", LABEL_ALBEDO)
	_materials["label_" + key_id] = m
	return m


func _labels() -> ImageTexture:
	if _label_atlas == null:
		var file := String((_cockpit_manifest.get("labels", {}) as Dictionary).get("file", ""))
		if file.is_empty():
			return null
		var image := Image.load_from_file(VehicleModels._repo_dir().path_join("data/models").path_join(VEHICLE_ID).path_join(file))
		if image == null or image.is_empty():
			return null
		image.generate_mipmaps()
		_label_atlas = ImageTexture.create_from_image(image)
	return _label_atlas


## The helmet shell with its panel lines (manifest crew_figure.helmet_grooves, crew_figure.toml [helmet] grooves; planes in the figure frame).
func _helmet_material(base: ShaderMaterial, grooves: Dictionary) -> ShaderMaterial:
	if not _materials.has("helmet_grooves"):
		var m := base.duplicate() as ShaderMaterial
		var planes: Array = grooves.get("planes", [])
		var plane_rows: Array[Vector4] = []
		var limit_rows: Array[Vector4] = []
		for row: Array in planes.slice(0, 2):
			plane_rows.append(Vector4(row[0], row[1], row[2], row[3]))
			limit_rows.append(Vector4(row[4], row[5], row[6], row[7]))
		while plane_rows.size() < 2:
			plane_rows.append(Vector4.ZERO)
			limit_rows.append(Vector4.ZERO)
		var a: Array = grooves.get("albedo", [0.08, 0.08, 0.085])
		m.set_shader_parameter("groove_count", mini(planes.size(), 2))
		m.set_shader_parameter("groove_plane", plane_rows)
		m.set_shader_parameter("groove_limit", limit_rows)
		m.set_shader_parameter("groove_centre", CockpitConfig._vec(grooves.get("centre")))
		m.set_shader_parameter("groove_radius", float(grooves.get("radius_m", 0.25)))
		m.set_shader_parameter("groove_width", float(grooves.get("width_m", 0.004)))
		m.set_shader_parameter("groove_albedo", Color(a[0], a[1], a[2]).linear_to_srgb())
		_materials["helmet_grooves"] = m
	return _materials["helmet_grooves"]


## The visor: copper tint on the upper part, dark clear glass below (manifest crew_figure.visor_tint, crew_figure.toml [helmet]).
func _visor_material(base: ShaderMaterial, tint: Dictionary) -> ShaderMaterial:
	var key := "visor_%s" % [tint]
	if not _materials.has(key):
		var m := base.duplicate() as ShaderMaterial
		var low: Array = tint["albedo_low"]
		m.set_shader_parameter("tint_split", float(tint["split"]))
		m.set_shader_parameter("albedo_low", Color(low[0], low[1], low[2]).linear_to_srgb())
		m.set_shader_parameter("roughness_low", float(tint["roughness_low"]))
		m.set_shader_parameter("metallic_low", float(tint["metallic_low"]))
		_materials[key] = m
	return _materials[key]


## The silver back with the dark vent band across it (shader band in the console frame; surface detail, not geometry).
func _console_back_material() -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = CABIN_SHADER
	m.set_shader_parameter("albedo", Color(0.30, 0.31, 0.33))  # [E] crew1_1: brushed silver, mid grey under the ring light
	m.set_shader_parameter("roughness", 0.5)
	m.set_shader_parameter("metallic", 0.25)
	m.set_shader_parameter("seam_depth", 0.0)
	_materials["console_back"] = m  # gets the fill term every frame like the other cabin materials
	var centre: Dictionary = config.screens.get("centre", {})
	if not centre.is_empty():
		var normal := _console_normal()
		m.set_shader_parameter("band_on", 1.0)
		m.set_shader_parameter("band_origin", centre["centre"])
		m.set_shader_parameter("band_up", CockpitConfig._vec(config.console.get("up")).normalized())
		m.set_shader_parameter("band_face_normal", -normal)
		m.set_shader_parameter("band_up_range", CONSOLE_VENT_UP_M)
		m.set_shader_parameter("band_half_width", CONSOLE_VENT_HALF_WIDTH_M)
		m.set_shader_parameter("band_albedo", Color(0.03, 0.03, 0.035))
	return m


func _display(colour: Color) -> ShaderMaterial:
	var key := "display_%s" % colour
	if not _materials.has(key):
		var m := ShaderMaterial.new()
		m.shader = DISPLAY_SHADER
		m.set_shader_parameter("colour", colour)
		_materials[key] = m
	return _materials[key]


func _cabin_material(albedo: Color, roughness: float, metallic: float, seams: float, specular := 0.4) -> ShaderMaterial:
	var key := "cabin_%s_%s_%s_%s_%s" % [albedo, roughness, metallic, seams, specular]
	if not _materials.has(key):
		var m := ShaderMaterial.new()
		m.shader = CABIN_SHADER
		m.set_shader_parameter("albedo", albedo)
		m.set_shader_parameter("roughness", roughness)
		m.set_shader_parameter("metallic", metallic)
		m.set_shader_parameter("seam_depth", seams)
		m.set_shader_parameter("specular", specular)
		_materials[key] = m
	return _materials[key]
