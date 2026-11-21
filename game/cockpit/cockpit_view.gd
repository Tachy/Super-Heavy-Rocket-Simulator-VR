class_name CockpitView
extends Node3D
## The Crew Dragon's cabin (docs/subsystems/cockpit.md) as a child of the capsule stage node: wall, floor, seats, console with
## screens / buttons / tablets, hatch, windows, the HOTAS props, the cabin lamps and the crew figures. Everything is in the
## capsule stage frame (the node's own transform is the identity); the stage node maps it to render axes. The cabin is part
## of the vehicle, so it exists in every view; the pilot camera (a later task) sits at `eye_node`.

const CABIN_SHADER := preload("res://shaders/cabin.gdshader")
const SUIT_SHADER := preload("res://shaders/crew_suit.gdshader")
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
## [E] The cove light all round the side wall under the ceiling corner (fidelity Task 9; was a ceiling annulus r 0.50-0.85): RING_LAMPS omnis
## RING_DROP_M below the middle of [cabin] cove_light_x_m and RING_IN_M in from the wall (cove_ring), RING_LUMENS in total (an area light's stand-in; its emissive band
## `ceiling_ring_light` is the generator's). tools/structures/photo_delight.cove_ring uses the same rule.
const RING_LAMPS := 8
const RING_LUMENS := 6000.0
const RING_DROP_M := 0.30
const RING_IN_M := 0.25
## Fidelity Task 9 room colours (linear albedo). [D] ratios to a white neighbour in the same photo x the white's albedo, [E] where shaded:
## the closed forward hatch (crew1_1: 0.032 of the lit wall; dark grey, partly in shade -> 0.04 [E]); the tunnel hand-holds' red
## (jsc2024e000258: R / G / B 0.43 / 0.11 / 0.12 of the white tunnel ring, albedo ~0.6 -> 0.26 / 0.07 / 0.07 [D]); the vent grilles (same photo:
## 0.056 of the ring -> 0.03); the window gaskets and the hatch seal (nasatour_0420 / crew1_1 / nasatour_0500: black -> 0.03 [E]).
const FORWARD_HATCH_ALBEDO := Color(0.04, 0.04, 0.045)
const HANDLE_RED := Color(0.26, 0.07, 0.07)
const VENT_ALBEDO := Color(0.03, 0.03, 0.03)
const SEAL_ALBEDO := Color(0.03, 0.03, 0.032)
## [E] Screen placeholder: a dark Dragon-blue UI background, display-referred (no exposure, as every emitter).
const SCREEN_COLOUR := Color(0.025, 0.07, 0.17)
## Render layer of the console parts (round 4): the two console lamps hang 8 cm in front of the console face and light it only at grazing
## angles (their shadow maps there are prone to acne). The concentric rings at the top of pilot_eye were NOT this: they were the aerial
## perspective's float32 noise over sub-metre paths (fix round 5, atm_to_point_offset's short-path form). The lamps face the crew, so they do not light the console (light_cull_mask without this layer); the ring and fill lights still do.
const CONSOLE_LAYER := 1 << 11
## [E] Fidelity Task 6: the console atlas' emitters (the red strip on the tray lip, the small display; ORM alpha = mask) emit albedo x mask x
## this luminance per unit albedo: the red strip (albedo 0.35 / 0.02 / 0.02, mask 1) ~40 cd/m2 as the Task 5b strip, the display (mask 0.25) ~10 cd/m2.
const CONSOLE_EMIT_CD_M2 := 500.0
## The console nodes on the atlas (cabin_layout.NODE_CHARTS; the manifest's cockpit.console_maps).
const ATLAS_NODES: Array[String] = ["console_housing", "console_back", "console_under", "console_shelf", "console_lockers"]
## [E] Fidelity Task 7: the wall recess around the console and its bevels (crew1_1 / KSC-20240927: light grey against the white panels).
const RECESS_ALBEDO := Color(0.6, 0.6, 0.6)
## [D] the grey starts 0.09 down the bevel (fidelity Task 8; T7 fix round 1 had 0.05, where the edge followed the wall's seam-cell staircase in
## close-up): crew1_1's white -> grey line lies 0.5-5.5 px inside the read panel edge (mean 2.0 / 3.7 px l / r; at 0.15 it sat 2-8 px inside).
const RECESS_TINT_FROM := 0.09
## [E] Fidelity Task 7: the iPads' lit placeholder page (tablet maps, ORM alpha = the display): emits albedo x this luminance per unit albedo,
## the white text ~280 cd/m2, the dark page ~15 (an iPad mini at a comfortable cabin brightness; max 500 cd/m2 [S]).
const TABLET_EMIT_CD_M2 := 400.0
## [D] discovery10 de-lit tray (fidelity Task 6, Ruling G): a blue LED in the middle of each r_led key face, 7 x 6 mm, its centre 2.5 mm above the
## key centre; [E] luminance (a lit indicator, reads saturated blue) and colour (linear Rec.709 at luminance 1).
const LED_KEYS: Array[String] = ["r_led_1", "r_led_2", "r_led_3"]
const LED_SIZE_M := Vector2(0.007, 0.006)
const LED_UP_M := 0.0025
const LED_CD_M2 := 150.0
const LED_COLOUR := Vector3(0.35, 0.95, 4.2)
const LED_ALBEDO := Color(0.05, 0.25, 0.9)
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
## [E] Task 8b: the elbow hangs down the seat back, a little back and out (skeleton frame, crew's left; mirrored for the right): the rest pole
## of the hands-on-thighs pose points the elbow sideways, which flares the arm at the HOTAS grip (KSC-20190731, demo1_1: elbows by the sides)
const ELBOW_POLE := Vector3(-0.8, -0.45, 0.4)
## [E] Task 8b: a hand closed round a HOTAS grip has its palm on the grip and the index tip this far ahead of it (along the chest normal)
const GRIP_TIP_AHEAD_M := 0.09
## HOTAS props (Task 9): the generator (tools/structures/blender/gen_dragon_cabin.py build_stick / build_throttle) bakes the props at the grip
## point in the capsule frame; seat back 35 deg from +X toward -Y (b up the back, n the chest normal, z capsule Z = the crew's left), the
## stick base box a distance below the grip along b. The view tilts the prop about that base.
const HOTAS_BACK_DEG := 35.0
const STICK_BASE_BELOW_M := 0.17  # [D] = the generator's stick base offset
const THROTTLE_BASE_BELOW_M := 0.15  # [D] = the generator's throttle base offset
const STICK_MAX_DEG := 15.0  # [E] T.16000M: about +-15 deg of tilt at full deflection
const STICK_TWIST_MAX_DEG := 10.0  # [E] the stick's twist (yaw axis)
const THROTTLE_TRAVEL_DEG := 50.0  # [E] TWCS lever travel from idle to full
const NUDGE_STEP_M := 0.01  # dev aid (--hotas-nudge): one key press moves a prop this far (Shift: NUDGE_FINE_M)
const NUDGE_FINE_M := 0.002
## [E] Task 9 VR fix B: the player's own head (helmet, visor) is hidden while the active camera is this close to the eye point: the helmet
## reaches 0.07-0.19 m from the eye, and a seated VR player leans or turns the head by up to ~0.2 m; the figure's head does not follow the
## headset, so a helmet drawn around the camera would stay put while the view turns (shell walls across the view).
const HEAD_HIDE_RADIUS_M := 0.30
## [E] The raised visor's opacity (coated polycarbonate seen against the white shell: 066389 shows the shell clearly through it, a faint tint).
const RAISED_VISOR_OPACITY := 0.15

var config: CockpitConfig
var eye_node := Node3D.new()
var _nodes := {}  # cabin node name -> MeshInstance3D
var _lights: Array[OmniLight3D] = []
var _light_cd: Array[float] = []
var _figures := {}  # seat id -> Node3D (figure root)
var _skeletons := {}  # seat id -> Skeleton3D
var _crew := 0
var _materials := {}
static var _tablet_textures := {}  # tablet map file -> ImageTexture (or null when missing), shared by every CockpitView
static var _tablet_warned := false
var _led_materials: Array[ShaderMaterial] = []
var _console_pending := false  ## console maps still being prepared in the background (polled in _process)
var _seat_pending := false  ## seat atlas maps still being prepared in the background (polled in _process)
var _seat_warned := false
var _console_warned := false
var _cockpit_manifest := {}
var _label_atlas: ImageTexture = null
var _figure_bones := {}  # manifest crew_figure.bones (the arm chains' finger lengths)
var _arm_iks := {}  # side "l" / "r" -> ArmIk of the player's figure
var _arm_results := {}  # side -> the last solved pose (ArmIk.solve)
var _lean := Quaternion.IDENTITY  # the torso's lean towards the pointer (skeleton frame)
var _deforms := {}  # seat id -> FigureDeform (joint folds: corrective shapes, wrinkle weights)
var _suit_textures := {}  # manifest crew_figure.suit_maps file -> ImageTexture
var _joint_table := {}  # manifest crew_figure.joints
var _hotas_axes := Vector3.ZERO  # last set_hotas_axes: pitch, sideways, twist (-1..1)
var _hotas_throttle := 0.0
var _hotas_nudge := {"stick": Vector3.ZERO, "throttle": Vector3.ZERO}  # dev aid: prop offsets in the capsule frame (m)
var _nudge_enabled := false
var _nudge_part := "stick"
var _helmet_nodes := {}  # seat id -> {"closed": MeshInstance3D, "open": MeshInstance3D} (manifest crew_figure.helmet_nodes)
var _head_hidden_request := false  # set_player_head_hidden
var _camera_at_eye := false  # the active camera within HEAD_HIDE_RADIUS_M of the eye point


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
		if part.begins_with("window_glass") or part.begins_with("light_") or part.begins_with("screen_") or part == "ceiling_ring_light" or part == "console_lamps" \
				or (part.begins_with("guard_") and not part.begins_with("guard_rim_")):
			instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		if part == "console_hit":  # the pointer's flat hit faces (fidelity Task 5): geometry only, never drawn
			instance.visible = false
		if _on_console(part):
			instance.layers = CONSOLE_LAYER
		add_child(instance)
		_nodes[part] = instance
	_build_lights()
	_build_figures(models)
	set_hotas_props(config.hotas_props_enabled)
	set_crew(config.seats.size())
	_nudge_enabled = "--hotas-nudge" in OS.get_cmdline_user_args() or "--hotas-nudge" in OS.get_cmdline_args()
	set_process_unhandled_key_input(_nudge_enabled)


func _on_console(part: String) -> bool:
	return part.begins_with("console_") or part.begins_with("tablet_") or part.begins_with("button_") or part == "handle" \
			or part.begins_with("light_") or part.begins_with("guard_")


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


## Whether the figure of a seat is shown (set_crew order: s2, s3, s1, s4).
func figure_visible(seat_id: String) -> bool:
	var figure: Node3D = _figures.get(seat_id)
	return figure != null and figure.visible


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


## The resting fingertip of a side in the cabin frame (Zero without the figure): with the HOTAS props shown the hand on its grip (the
## throttle for the crew's left hand, the stick for the right; [hotas] *_grip_m from the eye point, Task 9 calibrates them), otherwise
## the figure's rest pose (hands on the thighs).
func idle_tip(side: String) -> Vector3:
	if hotas_grip(side):
		var chest := _skeleton_to_cabin().basis.y.normalized() if _skeletons.has(CREW_ORDER[0]) else Vector3.ZERO  # the seat's chest normal
		var part := "hotas_throttle" if side == "l" else "hotas_stick"
		var rest := config.eye.origin + CockpitConfig._vec(config.hotas.get("throttle_grip_m" if side == "l" else "stick_grip_m")) + chest * GRIP_TIP_AHEAD_M
		return (_nodes[part] as MeshInstance3D).transform * rest  # the hand rides the deflected prop
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
		var result := arm.solve(shoulder, targets[side], _lean * Vector3(ELBOW_POLE.x, ELBOW_POLE.y, ELBOW_POLE.z * (1.0 if side == "l" else -1.0)))
		var extend := float(pose[side].get("extend", 0.0))
		arm.apply(skeleton, result, extend, (1.0 - extend) if hotas_grip(side) else 0.0)
		_arm_results[side] = result
	if _deforms.has(CREW_ORDER[0]):
		(_deforms[CREW_ORDER[0]] as FigureDeform).update()


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


## Hides (collapses) the head, helmet and visor of the player's body (the commander's figure) on request (shots); independent of it
## the head is hidden while the active camera is within HEAD_HIDE_RADIUS_M of the eye point (_process): the camera then sits in that
## helmet, and the figure's head does not follow the headset, so nothing of the own helmet may show in the view.
func set_player_head_hidden(hidden: bool) -> void:
	_head_hidden_request = hidden
	_apply_player_head()


## Whether the player's head is collapsed now (request or camera at the eye).
func player_head_hidden() -> bool:
	return _head_hidden_request or _camera_at_eye


func _apply_player_head() -> void:
	var skeleton: Skeleton3D = _skeletons.get(CREW_ORDER[0])
	if skeleton == null:
		return
	var bone := skeleton.find_bone("head")
	if bone >= 0:
		skeleton.set_bone_pose_scale(bone, Vector3.ONE * (0.001 if player_head_hidden() else 1.0))


## Task 9 VR fix B: whether a seat's figure wears the open helmet (visor raised; the player's figure) - false: the closed one.
func figure_helmet_open(seat_id: String) -> bool:
	var nodes: Dictionary = _helmet_nodes.get(seat_id, {})
	return nodes.has("open") and (nodes["open"] as Node3D).visible


func _update_player_head() -> void:
	var camera := get_viewport().get_camera_3d() if is_inside_tree() else null
	var at_eye := camera != null and config != null and to_local(camera.global_position).distance_to(config.eye.origin) < HEAD_HIDE_RADIUS_M
	if at_eye != _camera_at_eye:
		_camera_at_eye = at_eye
		_apply_player_head()


## Whether the idle hand of a side holds its HOTAS grip (the props shown and the grip point configured).
func hotas_grip(side: String) -> bool:
	var node: MeshInstance3D = _nodes.get("hotas_throttle" if side == "l" else "hotas_stick")
	return node != null and node.visible and config != null and config.hotas.has("throttle_grip_m" if side == "l" else "stick_grip_m")


## The stick and throttle props on the commander's armrests (the deliberate deviation, switchable).
func set_hotas_props(enabled: bool) -> void:
	for part in ["hotas_stick", "hotas_throttle"]:
		var node: MeshInstance3D = _nodes.get(part)
		if node != null:
			node.visible = enabled


## Stick and throttle follow the commander's real inputs (view only): `pitch` > 0 pushes the stick forward, `sideways` > 0 to the crew's right,
## `twist` > 0 twists it clockwise (seen from above). These are the PHYSICAL stick axes: with the T.16000M mapping (config/input_hotas.toml [stick])
## sideways is the game's yaw input and the twist its roll input (World passes them so). `throttle` 0..1 moves the lever from idle (back) to full (forward).
func set_hotas_axes(pitch: float, sideways: float, twist: float, throttle: float) -> void:
	_hotas_axes = Vector3(clampf(pitch, -1.0, 1.0), clampf(sideways, -1.0, 1.0), clampf(twist, -1.0, 1.0))
	_hotas_throttle = clampf(throttle, 0.0, 1.0)
	_update_hotas_transforms()


func _seat_axes() -> Dictionary:
	var a := deg_to_rad(HOTAS_BACK_DEG)
	return {"n": Vector3(sin(a), cos(a), 0.0), "b": Vector3(cos(a), -sin(a), 0.0), "z": Vector3.BACK}


func _update_hotas_transforms() -> void:
	if config == null or not config.hotas.has("stick_grip_m"):
		return
	var axes := _seat_axes()
	var n: Vector3 = axes["n"]
	var b: Vector3 = axes["b"]
	var z: Vector3 = axes["z"]
	var stick: MeshInstance3D = _nodes.get("hotas_stick")
	if stick != null:
		var grip := config.eye.origin + CockpitConfig._vec(config.hotas["stick_grip_m"])
		var r := Basis(z, deg_to_rad(STICK_MAX_DEG) * _hotas_axes.x) * Basis(n, deg_to_rad(STICK_MAX_DEG) * _hotas_axes.y) * Basis(b, -deg_to_rad(STICK_TWIST_MAX_DEG) * _hotas_axes.z)
		stick.transform = _about(r, grip - b * STICK_BASE_BELOW_M, _hotas_nudge["stick"])
	var lever: MeshInstance3D = _nodes.get("hotas_throttle")
	if lever != null and config.hotas.has("throttle_grip_m"):
		var grip := config.eye.origin + CockpitConfig._vec(config.hotas["throttle_grip_m"])
		var r := Basis(z, deg_to_rad(THROTTLE_TRAVEL_DEG) * (_hotas_throttle - 0.5))
		lever.transform = _about(r, grip - b * THROTTLE_BASE_BELOW_M, _hotas_nudge["throttle"])


static func _about(rotation: Basis, pivot: Vector3, offset: Vector3) -> Transform3D:
	return Transform3D(Basis.IDENTITY, offset) * Transform3D(rotation, pivot - rotation * pivot)


## The tilt (deg) of a prop from its rest pose: the angle its up-the-back axis has turned through.
## The prop's rotation in the seat frame (tests: direction of tilt and twist).
func hotas_prop_basis(part: String) -> Basis:
	var node: MeshInstance3D = _nodes.get(part)
	return node.transform.basis.orthonormalized() if node != null else Basis.IDENTITY


func seat_axes() -> Dictionary:
	return _seat_axes()


func hotas_tilt_deg(part: String) -> float:
	var node: MeshInstance3D = _nodes.get(part)
	if node == null:
		return 0.0
	var b: Vector3 = _seat_axes()["b"]
	return rad_to_deg(b.angle_to(node.transform.basis * b))


## Dev aid (`--hotas-nudge`): Alt+1 / Alt+2 pick stick / throttle, Alt+arrows move it (left/right: crew's left/right, up/down: forward/back),
## Alt+PageUp / PageDown up / down the seat back; Shift = fine. Prints the new [hotas] value for config/cockpits/crew_dragon.toml.
func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo or not key.alt_pressed or config == null:
		return
	var axes := _seat_axes()
	var step := NUDGE_FINE_M if key.shift_pressed else NUDGE_STEP_M
	var move := Vector3.ZERO
	match key.keycode:
		KEY_1:
			_nudge_part = "stick"
		KEY_2:
			_nudge_part = "throttle"
		KEY_LEFT:
			move = (axes["z"] as Vector3) * step
		KEY_RIGHT:
			move = -(axes["z"] as Vector3) * step
		KEY_UP:
			move = (axes["n"] as Vector3) * step
		KEY_DOWN:
			move = -(axes["n"] as Vector3) * step
		KEY_PAGEUP:
			move = (axes["b"] as Vector3) * step
		KEY_PAGEDOWN:
			move = -(axes["b"] as Vector3) * step
		_:
			return
	_hotas_nudge[_nudge_part] = (_hotas_nudge[_nudge_part] as Vector3) + move
	_update_hotas_transforms()
	var cfg_key := "stick_grip_m" if _nudge_part == "stick" else "throttle_grip_m"
	var v: Vector3 = CockpitConfig._vec(config.hotas[cfg_key]) + _hotas_nudge[_nudge_part]
	print("[hotas-nudge] %s = [%.3f, %.3f, %.3f]   (offset from the config %.0f / %.0f / %.0f mm)" % [cfg_key, v.x, v.y, v.z,
		(_hotas_nudge[_nudge_part] as Vector3).x * 1000.0, (_hotas_nudge[_nudge_part] as Vector3).y * 1000.0, (_hotas_nudge[_nudge_part] as Vector3).z * 1000.0])


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
	_update_player_head()
	if _console_pending and _materials.has("console_atlas"):
		_apply_console_maps(_materials["console_atlas"])
	if _seat_pending:
		_seat_pending = false
		for kind: String in ["shell", "pad", "rest"]:
			if _materials.has("seat_" + kind):
				_apply_seat_maps(_materials["seat_" + kind])
	var scene_per_cd: float = ShaderGlobals.value(&"atmosphere_scene_per_cd_m2")
	for i in _lights.size():
		_lights[i].light_energy = TowerLights.energy_of(_light_cd[i], scene_per_cd)
	var fill := FILL_LUX / PI * scene_per_cd
	var surround := fill * WALL_ALBEDO.srgb_to_linear().get_luminance()  # the walls' luminance under the same bounced light
	for material: Material in _materials.values():
		if material is ShaderMaterial and ((material as ShaderMaterial).shader == CABIN_SHADER or (material as ShaderMaterial).shader == SUIT_SHADER):
			(material as ShaderMaterial).set_shader_parameter("fill", fill)
			(material as ShaderMaterial).set_shader_parameter("fill_surround", surround)
	if _materials.has("console_atlas"):
		(_materials["console_atlas"] as ShaderMaterial).set_shader_parameter("emit_gain", CONSOLE_EMIT_CD_M2 * scene_per_cd)
	if _materials.has("tablet"):
		(_materials["tablet"] as ShaderMaterial).set_shader_parameter("emit_gain", TABLET_EMIT_CD_M2 * scene_per_cd)
	for material: ShaderMaterial in _led_materials:
		material.set_shader_parameter("mark_emission", LED_COLOUR * LED_CD_M2 * scene_per_cd)


func _build_lights() -> void:
	for i in config.lights.size():
		var row: Dictionary = config.lights[i]
		var p: Vector3 = CockpitConfig._vec(row.get("position_m"))
		var lamp := _add_omni("light_%s" % row.get("id", i), p + _console_normal() * LAMP_DROP_M, LAMP_LUMENS)
		lamp.shadow_enabled = true  # two lamps: cheap shadows
		lamp.shadow_bias = 0.12  # the ceiling is lit at a grazing angle: the default bias showed radial shadow acne (pilot_eye, round 3)
		lamp.shadow_normal_bias = 3.0
		lamp.light_cull_mask &= ~CONSOLE_LAYER
	var ceiling_x := float(config.cabin.get("ceiling_x_m", 3.48))
	var ring := cove_ring(config.cabin)
	for k in RING_LAMPS:
		var az := TAU * float(k) / RING_LAMPS
		_add_omni("ring_%d" % k, Vector3(ring.x, ring.y * cos(az), ring.y * sin(az)), RING_LUMENS / RING_LAMPS)
	_add_omni("fill_ceiling", Vector3(ceiling_x - 0.25, 0.0, 0.0), FILL_LUMENS)  # 0.25 m under the ceiling plane (was 3.1 for the ceiling at 3.35)
	_add_omni("fill_seats", Vector3(2.0, 0.55, 0.0), FILL_LUMENS)


## (x, r) of the ring omnis: RING_DROP_M below the middle of [cabin] cove_light_x_m and RING_DROP_M in from the side wall there (profile_m up to the
## ceiling corner): 0.10 under the ceiling plane and 0.10 from the wall they lit it in hot spots (Task 9 r1 shots), so the ring hangs
## 0.30 under it and 0.25 in. The middle of the span is clamped to the profile's side-wall run (the same rule as photo_delight.cove_ring, np.interp).
static func cove_ring(cabin: Dictionary) -> Vector2:
	var span: Array = cabin.get("cove_light_x_m", [3.42, 3.47])
	var x := 0.5 * (float(span[0]) + float(span[1]))
	var ceiling_x := float(cabin.get("ceiling_x_m", 3.48))
	var profile: Array = cabin.get("profile_m", [])
	var run: Array = []
	for i in range(1, profile.size()):
		if float(profile[i][0]) > ceiling_x + 1.0e-9:
			break
		run.append(profile[i])
	if run.is_empty():
		return Vector2(x - RING_DROP_M, 0.0)
	x = clampf(x, float(run[0][0]), float(run[run.size() - 1][0]))
	var r := float(run[run.size() - 1][1])
	for i in range(run.size() - 1):
		var a: Array = run[i]
		var b: Array = run[i + 1]
		if x <= float(b[0]) and float(b[0]) > float(a[0]):
			r = lerpf(float(a[1]), float(b[1]), (x - float(a[0])) / (float(b[0]) - float(a[0])))
			break
	return Vector2(x - RING_DROP_M, r - RING_IN_M)


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
	_joint_table = manifest.get("joints", {})
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
		# Task 9 VR fix B: the player's figure (the commander) wears the open helmet (visor raised), the others the closed one
		var helmets: Dictionary = manifest.get("helmet_nodes", {})
		var open := seat_id == CREW_ORDER[0]
		var found := {}
		for kind: String in helmets:
			var node := scene.find_child(String(helmets[kind]), true, false) as MeshInstance3D
			if node != null:
				node.visible = (kind == "open") == open
				found[kind] = node
		_helmet_nodes[seat_id] = found
		for node in scene.find_children("*", "MeshInstance3D", true, false):
			var mesh_node := node as MeshInstance3D
			for s in mesh_node.mesh.get_surface_count():
				var source := mesh_node.mesh.surface_get_material(s)
				var key := source.resource_name if source != null else ""
				if look.has(key):
					var entry: Dictionary = look[key]
					var c: Array = entry["albedo"]
					var material := _cabin_material(Color(c[0], c[1], c[2]).linear_to_srgb(), float(entry["roughness"]), float(entry["metallic"]), 0.0)
					if key == String(manifest.get("suit_material", "")) and manifest.has("suit_maps"):
						material = _suit_material(entry, manifest["suit_maps"])
					elif key == String(manifest.get("visor_material", "")) and mesh_node.name == String((manifest.get("helmet_nodes", {}) as Dictionary).get("open", "")):
						material = _raised_visor_material(entry)
					elif key == String(manifest.get("visor_material", "")) and manifest.has("visor_tint"):
						material = _visor_material(material, manifest["visor_tint"])
					elif key == String(manifest.get("helmet_material", "")) and manifest.has("helmet_grooves"):
						material = _helmet_material(material, manifest["helmet_grooves"])
					mesh_node.set_surface_override_material(s, material)
			if mesh_node.mesh.get_blend_shape_count() == 0:
				continue  # fix round 1: only the suit mesh carries the corrective shapes (the rigid parts are a separate mesh)
			var deform := FigureDeform.for_figure(_skeletons.get(seat_id), mesh_node, manifest.get("joints", {}), manifest.get("joint_order", []))
			if deform != null:
				_deforms[seat_id] = deform
				deform.update()
	template.free()


## The manifest's joint table (crew_figure.joints: bend angles, shapes, slots).
func _figure_joints() -> Dictionary:
	return _joint_table


## The figure's joint deformer of a seat (null without the model).
func figure_deform(seat_id: String) -> FigureDeform:
	return _deforms.get(seat_id)


## The suit (Task 8b): crew_suit.gdshader with the baked albedo and wrinkle maps (manifest crew_figure.suit_maps); falls back to the
## plain cabin material without the maps.
func _suit_material(entry: Dictionary, maps: Dictionary) -> ShaderMaterial:
	if _materials.has("crew_suit"):
		return _materials["crew_suit"]
	var albedo_map := _suit_texture(String(maps.get("albedo", "")))
	var wrinkle_map := _suit_texture(String(maps.get("wrinkle", "")))
	var m := ShaderMaterial.new()
	m.shader = SUIT_SHADER
	if albedo_map == null or wrinkle_map == null:
		var c: Array = entry["albedo"]
		return _cabin_material(Color(c[0], c[1], c[2]).linear_to_srgb(), float(entry["roughness"]), float(entry["metallic"]), 0.0)
	var atlas: Array = maps.get("atlas_m", [2.56, 1.6])
	m.set_shader_parameter("suit_albedo", albedo_map)
	m.set_shader_parameter("suit_wrinkle", wrinkle_map)
	m.set_shader_parameter("atlas_m", Vector2(float(atlas[0]), float(atlas[1])))
	m.set_shader_parameter("slope_range", float(maps.get("slope_range", 1.6)))
	m.set_shader_parameter("roughness", float(entry["roughness"]))
	m.set_shader_parameter("metallic", float(entry["metallic"]))
	_materials["crew_suit"] = m
	return m


func _suit_texture(file: String) -> ImageTexture:
	if file.is_empty():
		return null
	if not _suit_textures.has(file):
		var image := Image.load_from_file(VehicleModels.repo_dir().path_join("data/models").path_join(VEHICLE_ID).path_join(file))
		if image == null or image.is_empty():
			return null
		image.generate_mipmaps()
		_suit_textures[file] = ImageTexture.create_from_image(image)
	return _suit_textures[file]


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
		return _shell_material()
	if part == "cabin_recess":
		return _recess_material()
	## Fidelity Task 9: the window wells' gasket band and the hatch's seal frame are the faces with glTF UV.y 1 (the generator's strip_uv):
	## tinted dark, the rest white.
	if part == "hatch_frame" or part.begins_with("window_"):
		return _two_tone_material("trim_" + part.get_slice("_", 0), SEAL_ALBEDO)
	if part == "floor":
		return _cabin_material(WALL_ALBEDO, 0.8, 0.0, 0.0)
	if part == "forward_hatch":
		return _cabin_material(FORWARD_HATCH_ALBEDO.linear_to_srgb(), 0.6, 0.0, 0.0)
	if part == "tunnel_handles":
		return _cabin_material(HANDLE_RED.linear_to_srgb(), 0.45, 0.0, 0.0)
	if part == "ceiling_vents":
		return _cabin_material(VENT_ALBEDO.linear_to_srgb(), 0.9, 0.0, 0.0)
	## Fidelity Task 6: the console body, back with the posts, underside, tray and lockers share one material on the baked console atlas
	## (tools/structures/cabin_textures.py: colours, seams, screws, placards, vent, plates, the red lip strip; docs/subsystems/cockpit.md).
	if part in ATLAS_NODES:
		return _console_atlas_material()
	## [E] Fidelity Task 5: the screens' dark glass border (glossy black glass, jsc2022e068617), the clear key guards and their red rims
	## (rim colour from the manifest, linear), the hidden pointer hit faces.
	if part == "console_bezel":
		return _cabin_material(Color(0.012, 0.012, 0.014).linear_to_srgb(), 0.15, 0.0, 0.0, 0.5)
	if part == "console_hit":
		return _cabin_material(Color(0.5, 0.5, 0.5), 1.0, 0.0, 0.0)
	if part.begins_with("guard_rim_"):
		var g: Dictionary = (_cockpit_manifest.get("guards", {}) as Dictionary).get(part.trim_prefix("guard_rim_"), {})
		var c: Array = g.get("rim_colour", [0.55, 0.04, 0.03])
		return _cabin_material(Color(float(c[0]), float(c[1]), float(c[2])).linear_to_srgb(), 0.4, 0.0, 0.0, 0.4)
	if part.begins_with("guard_"):
		return _guard_material()
	## Seats (fidelity Task 8): the seat atlas (manifest cockpit.seat_maps: white shell with its grey rim, black cushion with seams and the
	## harness, carbon) on all three seat nodes; the cushion carries the fabric grain tile, the leg supports / armrests the carbon twill tile.
	if part.begins_with("seat_shell_"):
		return _seat_material("shell")
	if part.begins_with("seat_pad_"):
		return _seat_material("pad")
	if part.begins_with("seat_"):
		return _seat_material("rest")
	if part.begins_with("tablet_"):
		return _tablet_material()
	if part.begins_with("hotas_"):
		return _cabin_material(Color(0.04, 0.04, 0.043).linear_to_srgb(), 0.5, 0.4, 0.0)  # Task 5b: LINEAR 0.04 (black plastic), was the sRGB literal 0.06 = 0.005 linear
	if part == "handle":
		return _handle_material()
	var key_id := part.trim_prefix("button_")
	var entry: Dictionary = (_cockpit_manifest.get("buttons", {}) as Dictionary).get(key_id, {})
	if entry.has("label_cell"):
		return _label_material(key_id, entry)
	if key_id in LED_KEYS and not entry.is_empty():
		return _led_material(key_id, entry)
	return _key_base()  # buttons without a legible label: blank key faces


## [E] The keys: dark grey (conv_42860a: near-black keys on the dark grey modules). Task 5b: the earlier Color(0.09, 0.09, 0.1) literal went into
## the sRGB uniform unconverted (0.009 linear, darker than the module); now a linear 0.03 converted like the console colours.
func _key_base() -> ShaderMaterial:
	return _cabin_material(Color(0.03, 0.03, 0.033).linear_to_srgb(), 0.45, 0.3, 0.0)


## [E] The clear key guards (jsc2022e068617: a clear polycarbonate cover, the keys read through it): faint, glossy, double sided.
func _guard_material() -> StandardMaterial3D:
	if _materials.has("guard"):
		return _materials["guard"]
	var m := StandardMaterial3D.new()
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	m.albedo_color = Color(0.6, 0.63, 0.66, 0.04)
	m.roughness = 0.08
	m.metallic_specular = 0.5
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	_materials["guard"] = m
	return m


## [E] Console colours while the atlas is pending or missing (cabin_textures.DARK_PANEL 0.030 / 0.030 / 0.034 linear, panel roughness 0.85).
const CONSOLE_FLAT_ALBEDO := Color(0.030, 0.030, 0.034)
const CONSOLE_FLAT_ROUGHNESS := 0.85


## Background maps state: 0 = still waiting (cabin not built yet or maps pending), 1 = console and seat maps are in, -1 = a map is missing / failed
## (flat colours). For the look and timing scripts, which must not capture before the maps are in.
func maps_state() -> int:
	if not _materials.has("console_atlas") or _console_pending or _seat_pending:
		return 0
	var ok := float((_materials["console_atlas"] as ShaderMaterial).get_shader_parameter("maps_on")) == 1.0
	for kind: String in ["shell", "pad", "rest"]:
		if _materials.has("seat_" + kind):
			ok = ok and float((_materials["seat_" + kind] as ShaderMaterial).get_shader_parameter("maps_on")) == 1.0
	return 1 if ok else -1


## The console atlas material (fidelity Task 6): albedo / normal / ORM maps of the model set (manifest cockpit.console_maps); its emission gain
## is set every frame (_process). The cabin shader's fill and wall reflection apply as on every cabin part.
func _console_atlas_material() -> ShaderMaterial:
	if _materials.has("console_atlas"):
		return _materials["console_atlas"]
	var m := ShaderMaterial.new()
	m.shader = CABIN_SHADER
	m.set_shader_parameter("specular", 0.4)
	m.set_shader_parameter("albedo", CONSOLE_FLAT_ALBEDO.linear_to_srgb())  # flat colours until (or without) the maps; the shader default is white
	m.set_shader_parameter("roughness", CONSOLE_FLAT_ROUGHNESS)
	_apply_console_maps(m)
	_materials["console_atlas"] = m
	return m


## Sets the three maps on the atlas material as far as the background preparation (VehicleModels.prewarm_console_maps) has finished; never blocks.
## While one is pending maps_on stays 0 (flat colours) and _process polls until all are in.
func _apply_console_maps(m: ShaderMaterial) -> void:
	var maps: Dictionary = _cockpit_manifest.get("console_maps", {})
	var all_loaded := true
	_console_pending = false
	for role: String in ["albedo", "normal", "orm"]:
		var file := String(maps.get(role, ""))
		var texture := _console_texture(file, role)
		if texture != null:
			m.set_shader_parameter(role + "_map", texture)
			continue
		all_loaded = false
		if not file.is_empty() and VehicleModels.is_console_map_pending(file):
			_console_pending = true
		elif not _console_warned:
			_console_warned = true
			push_warning("CockpitView: console map '%s' missing (rebuild the crew_dragon model set); the console keeps its flat colours" % role)
	m.set_shader_parameter("maps_on", 1.0 if all_loaded else 0.0)


func _console_texture(file: String, role: String) -> ImageTexture:
	if file.is_empty():
		return null
	return VehicleModels.console_texture(VehicleModels.repo_dir().path_join("data/models").path_join(VEHICLE_ID).path_join(file), file, role)


## [E] Seat colours while the atlas is still being prepared (the atlas' own values, cabin_textures.py: shell 0.62, cushion 0.025, carbon 0.03 linear).
const SEAT_FLAT := {"shell": [Color(0.62, 0.62, 0.61), 0.45, 0.6], "pad": [Color(0.025, 0.025, 0.027), 0.5, 0.2], "rest": [Color(0.03, 0.03, 0.032), 0.28, 0.5]}
## [E] Detail tile per seat part (cabin_textures.bake_seat_detail): normal gain, roughness gain.
const SEAT_DETAIL := {"pad": ["fabric", 1.0, 1.0], "rest": ["carbon", 1.0, 1.0]}


## One seat material per part ("shell", "pad", "rest") on the seat atlas; maps come from the background preparation (VehicleModels.console_texture:
## compressed off the main thread, prewarmed at boot), polled in _process while pending (flat SEAT_FLAT colours meanwhile).
func _seat_material(kind: String) -> ShaderMaterial:
	var key := "seat_" + kind
	if _materials.has(key):
		return _materials[key]
	var flat: Array = SEAT_FLAT[kind]
	var m := ShaderMaterial.new()
	m.shader = CABIN_SHADER
	m.set_shader_parameter("albedo", (flat[0] as Color).linear_to_srgb())
	m.set_shader_parameter("roughness", float(flat[1]))
	m.set_shader_parameter("specular", float(flat[2]))
	if SEAT_DETAIL.has(kind):
		var d: Array = SEAT_DETAIL[kind]
		var maps: Dictionary = _cockpit_manifest.get("seat_maps", {})
		var tile := _tablet_texture(String(maps.get(String(d[0]), "")), false)  # small periodic tile, mipmapped once per process
		if tile != null:
			m.set_shader_parameter("detail_map", tile)
			m.set_shader_parameter("detail_tiles", float((_cockpit_manifest.get("seat_detail_tiles", {}) as Dictionary).get(String(d[0]), 1.0)))
			m.set_shader_parameter("detail_strength", float(d[1]))
			m.set_shader_parameter("detail_rough", float(d[2]))
			m.set_shader_parameter("detail_on", 1.0)
	_materials[key] = m
	_apply_seat_maps(m)
	return m


## Sets the seat atlas maps on a seat material as far as they are prepared; never blocks (_seat_pending: _process polls).
func _apply_seat_maps(m: ShaderMaterial) -> void:
	var maps: Dictionary = _cockpit_manifest.get("seat_maps", {})
	var all_loaded := true
	for role: String in ["albedo", "normal", "orm"]:
		var file := String(maps.get(role, ""))
		var texture := _console_texture(file, role)
		if texture != null:
			m.set_shader_parameter(role + "_map", texture)
			continue
		all_loaded = false
		if not file.is_empty() and VehicleModels.is_console_map_pending(file):
			_seat_pending = true
		elif not _seat_warned:
			_seat_warned = true
			push_warning("CockpitView: seat map '%s' missing (rebuild the crew_dragon model set); the seats keep their flat colours" % role)
	m.set_shader_parameter("maps_on", 1.0 if all_loaded else 0.0)


## The wall recess (fidelity Task 7): white at the panel edge, RECESS_ALBEDO from RECESS_TINT_FROM of the way down the bevel (the generator writes
## the recess fraction into UV.y; the shader's tint split follows it smoothly across the wall grid).
## The wall panels (fidelity Task 9): white, seams along meridians at [cabin] seam_phase_deg + k x seam_pitch_deg (crew1_1's ceiling ring seams).
func _shell_material() -> ShaderMaterial:
	if _materials.has("shell"):
		return _materials["shell"]
	var m := _cabin_material(WALL_ALBEDO, 0.85, 0.0, 0.5).duplicate() as ShaderMaterial
	m.set_shader_parameter("seam_polar_pitch_deg", float(config.cabin.get("seam_pitch_deg", 30.0)))
	m.set_shader_parameter("seam_polar_phase_deg", float(config.cabin.get("seam_phase_deg", 15.0)))
	_materials["shell"] = m
	return m


## White wall material whose glTF UV.y > 0.5 faces take `low` (linear albedo): window gaskets, the hatch seal (fidelity Task 9).
func _two_tone_material(key: String, low: Color) -> ShaderMaterial:
	if _materials.has(key):
		return _materials[key]
	var m := _cabin_material(WALL_ALBEDO, 0.8, 0.0, 0.0).duplicate() as ShaderMaterial
	m.set_shader_parameter("tint_split", 0.5)
	m.set_shader_parameter("albedo_low", low.linear_to_srgb())
	m.set_shader_parameter("roughness_low", 0.6)
	m.set_shader_parameter("metallic_low", 0.0)
	_materials[key] = m
	return m


func _recess_material() -> ShaderMaterial:
	if _materials.has("recess"):
		return _materials["recess"]
	var m := _shell_material().duplicate() as ShaderMaterial
	m.set_shader_parameter("tint_split", RECESS_TINT_FROM)
	m.set_shader_parameter("albedo_low", RECESS_ALBEDO)
	m.set_shader_parameter("roughness_low", 0.85)
	m.set_shader_parameter("metallic_low", 0.0)
	_materials["recess"] = m
	return m


## The iPads (fidelity Task 7): the cabin shader on the tablet maps (manifest cockpit.tablet_maps: white bezel, the lit placeholder page, the
## aluminium and arm swatches). Small maps (576 x 1024), loaded once per process (_tablet_textures, shared by every CockpitView); without
## them a dark flat material, cached per view like the lit one, with one warning per process.
func _tablet_material() -> ShaderMaterial:
	if _materials.has("tablet"):
		return _materials["tablet"]
	if _materials.has("tablet_fallback"):
		return _materials["tablet_fallback"]
	var maps: Dictionary = _cockpit_manifest.get("tablet_maps", {})
	var m := ShaderMaterial.new()
	m.shader = CABIN_SHADER
	m.set_shader_parameter("specular", 0.5)
	var loaded := 0
	for role: String in ["albedo", "normal", "orm"]:
		var texture := _tablet_texture(String(maps.get(role, "")), role == "normal")
		if texture == null:
			continue
		m.set_shader_parameter(role + "_map", texture)
		loaded += 1
	if loaded < 3:
		if not _tablet_warned:
			_tablet_warned = true
			push_warning("CockpitView: tablet maps missing (rebuild the crew_dragon model set); the tablets stay dark")
		var fallback := _cabin_material(Color(0.03, 0.03, 0.036).linear_to_srgb(), 0.85, 0.0, 0.0, 0.03)
		_materials["tablet_fallback"] = fallback
		return fallback
	m.set_shader_parameter("maps_on", 1.0)
	_materials["tablet"] = m
	return m


## One tablet map as a mipmapped texture, loaded once per process (null when the file is missing; the miss is cached too).
static func _tablet_texture(file: String, normal_map: bool) -> ImageTexture:
	if file.is_empty():
		return null
	if _tablet_textures.has(file):
		return _tablet_textures[file]
	var image := Image.load_from_file(VehicleModels.repo_dir().path_join("data/models").path_join(VEHICLE_ID).path_join(file))
	var texture: ImageTexture = null
	if image != null and not image.is_empty():
		image.generate_mipmaps(normal_map)
		texture = ImageTexture.create_from_image(image)
	_tablet_textures[file] = texture
	return texture


## An r_led key: the blank key face with its blue LED (a lit mark, emission set every frame).
func _led_material(key_id: String, entry: Dictionary) -> ShaderMaterial:
	var m := _key_base().duplicate() as ShaderMaterial
	_set_mark(m, CockpitConfig._vec(entry["centre"]), CockpitConfig._vec(entry["right"]), CockpitConfig._vec(entry["up"]), CockpitConfig._vec(entry["normal"]),
			Vector2(-0.5 * LED_SIZE_M.x, 0.5 * LED_SIZE_M.x), Vector2(LED_UP_M - 0.5 * LED_SIZE_M.y, LED_UP_M + 0.5 * LED_SIZE_M.y), LED_ALBEDO.linear_to_srgb())
	_materials["led_" + key_id] = m
	_led_materials.append(m)
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
		var image := Image.load_from_file(VehicleModels.repo_dir().path_join("data/models").path_join(VEHICLE_ID).path_join(file))
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


## Task 9 VR fix B: the player's raised visor is see-through coated glass over the white crown (jsc2024e066389, conv_2ks5k: the shell reads
## white through it), tinted by the visor's copper albedo; the transparent pass like the windows (no depth writes, no shadow).
func _raised_visor_material(entry: Dictionary) -> ShaderMaterial:
	if not _materials.has("visor_raised"):
		var m := ShaderMaterial.new()
		m.shader = GLASS_SHADER
		var c: Array = entry["albedo"]
		m.set_shader_parameter("tint", Vector3(float(c[0]), float(c[1]), float(c[2])))
		m.set_shader_parameter("opacity", RAISED_VISOR_OPACITY)
		_materials["visor_raised"] = m
	return _materials["visor_raised"]


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
