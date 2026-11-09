class_name ConsolePointer
extends Node
## The console pointer (docs/subsystems/cockpit.md, "Console pointer"): the mouse operates the cabin's screens and keys until hand
## tracking exists (user 2026-11-07). Desktop: the mouse ray is cast onto the console plane (ConsoleStrip.pick). VR: the physical
## mouse's motion moves a 2D pointer on the strip (`move_by`), which lies exactly on the 3D surfaces. The suited hand of the
## pointer's side (left screen and the keys under it: left hand, otherwise right hand) follows the target with HandMotion's
## critically damped spring, the index finger extended; a click taps. The other hand rests (grip pose). The arms are posed by
## CockpitView.pose_arms (ArmIk). Click dispatch: the camera button on the left screen -> ScreenCamera.next(); a key with
## function = "camera" -> select(camera_slot); every other key only taps (user decision 2026-11-09).

signal pressed(surface_id: String, uv: Vector2)

## [E] A desktop click this far (strip metres) from every surface hits nothing (the hand stays where it is).
const PICK_MARGIN_M := 0.03
## [E] VR: strip metres per mouse count (a full screen width of 0.38 m takes ~640 counts).
const VR_M_PER_COUNT := 0.0006
## [E] The pointer must be this far (strip metres) past the left/centre boundary to change the hand (no flapping).
const SIDE_HYSTERESIS_M := 0.015

var strip: ConsoleStrip
var config: CockpitConfig
var cabin: CockpitView
var screen_camera: ScreenCamera
var active := false  ## the cabin is the view (set by the caller each frame)
var vr_mode := false  ## mouse deltas move the pointer instead of the cursor ray
var strip_position := Vector2.ZERO  ## the pointer on the strip (VR; desktop: the last pick)
var hover := {}  ## ConsoleStrip.to_surface of the surface under the pointer, {} when none
var hands := {"l": HandMotion.new(), "r": HandMotion.new()}  ## cabin-frame fingertip positions (hover height included)
var fingers := {"l": HandMotion.new(), "r": HandMotion.new()}  ## index finger extension 0..1 (position.x)
var active_side := "r"
var driven_externally := false  ## a script (shots, tests) calls setup / update itself: World leaves the pointer alone
var last_action := ""  ## what the last click did ("camera_next", "camera:<id>", "none"); tests and logs
var _mouse_px := Vector2(-1.0, -1.0)
var _have_mouse := false
var _initialised := false


## The cabin of the controlled vehicle (null: none). `screen` is the left screen's camera feed.
func setup(cabin_config: CockpitConfig, cabin_view: CockpitView, screen: ScreenCamera) -> void:
	config = cabin_config
	cabin = cabin_view
	screen_camera = screen
	strip = ConsoleStrip.from_config(cabin_config) if cabin_config != null else null
	_initialised = false
	hover = {}


func _unhandled_input(event: InputEvent) -> void:
	if not active or strip == null:
		return
	if event is InputEventMouseMotion:
		var motion := event as InputEventMouseMotion
		_mouse_px = motion.position
		_have_mouse = true
		if vr_mode:
			move_by(motion.relative)
	elif event is InputEventMouseButton:
		var button := event as InputEventMouseButton
		if button.button_index == MOUSE_BUTTON_LEFT and button.pressed:
			_mouse_px = button.position
			_have_mouse = true
			if click():
				get_viewport().set_input_as_handled()


## VR: moves the pointer by a mouse delta (counts).
func move_by(delta_counts: Vector2) -> void:
	if strip == null:
		return
	strip_position = strip.clamp(strip_position + Vector2(delta_counts.x, -delta_counts.y) * VR_M_PER_COUNT)


## A pointer position on the strip (tests, desktop pick): the surface under it, {} when farther than the pick margin.
func point_at(p: Vector2, margin_m := PICK_MARGIN_M) -> Dictionary:
	strip_position = p
	var found := strip.to_surface(p) if strip != null else {}
	hover = found if not found.is_empty() and float(found["distance_m"]) <= margin_m else {}
	return hover


## A ray in the cabin frame (desktop pick).
func point_ray(ray_origin: Vector3, ray_dir: Vector3) -> Dictionary:
	if strip == null:
		return {}
	var hit := strip.pick(ray_origin, ray_dir)
	if not bool(hit["hit"]):
		hover = {}
		return hover
	return point_at(hit["strip"])


## A click: taps with the hand of the pointer's side and dispatches. false when nothing is under the pointer.
func click() -> bool:
	if strip == null:
		return false
	if not vr_mode:
		_pick_from_mouse()
	if hover.is_empty():
		return false
	(hands[active_side] as HandMotion).tap()
	pressed.emit(String(hover["surface_id"]), hover["uv"])
	last_action = activate(String(hover["surface_id"]), hover["uv"])
	return true


## What a press on a surface does: returns "camera_next", "camera:<id>" or "none". Keys with another function than "camera" do nothing yet.
func activate(surface_id: String, uv: Vector2) -> String:
	if surface_id == "left" and ScreenCamera.CAMERA_BUTTON_UV.has_point(uv):
		if screen_camera != null:
			screen_camera.next()
		return "camera_next"
	if surface_id.begins_with("button:"):
		var key := _button(surface_id.trim_prefix("button:"))
		if String(key.get("function", "none")) == "camera":
			var ids := CameraCatalog.all_ids(config)
			var slot := int(key.get("camera_slot", 0))
			if slot >= 1 and slot <= ids.size():
				if screen_camera != null:
					screen_camera.select(ids[slot - 1])
				return "camera:" + ids[slot - 1]
	return "none"


## One frame: the hands towards their targets, the arms posed.
func update(delta: float, camera: Camera3D) -> void:
	if cabin == null or strip == null:
		return
	if not _initialised:
		for side: String in hands:
			(hands[side] as HandMotion).reset(cabin.idle_tip(side))
			(fingers[side] as HandMotion).reset(Vector3.ZERO)
		_initialised = true
	if active and not vr_mode and camera != null and _have_mouse:
		_pick_from_mouse(camera)
	elif active and vr_mode:
		point_at(strip_position, INF)
	else:
		hover = {}
	if not hover.is_empty():
		active_side = _side_of(hover, active_side)
	var normal: Vector3 = strip.normal
	for side: String in hands:
		var hand := hands[side] as HandMotion
		var target := cabin.idle_tip(side)
		var extend := 0.0
		if not hover.is_empty() and side == active_side:
			target = (hover["point_m"] as Vector3) + normal * HandMotion.HOVER_M
			extend = 1.0
		hand.step(target, delta)
		(fingers[side] as HandMotion).step(Vector3(extend, 0.0, 0.0), delta)
	var pose := {}
	for side: String in hands:
		var hand := hands[side] as HandMotion
		pose[side] = {"tip": hand.position - normal * hand.tap_depth(), "extend": clampf((fingers[side] as HandMotion).position.x, 0.0, 1.0)}
	cabin.pose_arms(pose)


## The fingertip (cabin frame) of a side as last posed.
func tip_of(side: String) -> Vector3:
	var hand := hands[side] as HandMotion
	return hand.position - (strip.normal if strip != null else Vector3.ZERO) * hand.tap_depth()


func _pick_from_mouse(camera: Camera3D = null) -> void:
	var cam := camera if camera != null else get_viewport().get_camera_3d()
	if cam == null or cabin == null or not cabin.is_inside_tree():
		return
	var to_cabin := cabin.global_transform.affine_inverse()
	var origin := to_cabin * cam.project_ray_origin(_mouse_px)
	var direction := (to_cabin.basis * cam.project_ray_normal(_mouse_px)).normalized()
	point_ray(origin, direction)


## Left hand for the left screen and the keys under it (u left of the gap between the left and the centre screen), else right.
func _side_of(surface: Dictionary, current: String) -> String:
	var left: Dictionary = strip.surface("left")
	var centre: Dictionary = strip.surface("centre")
	if left.is_empty() or centre.is_empty():
		return "r"
	var boundary := 0.5 * ((left["rect"] as Rect2).end.x + (centre["rect"] as Rect2).position.x)
	var u: float = (surface["strip"] as Vector2).x
	if current == "l":
		return "l" if u < boundary + SIDE_HYSTERESIS_M else "r"
	return "l" if u < boundary - SIDE_HYSTERESIS_M else "r"


func _button(button_id: String) -> Dictionary:
	if config == null:
		return {}
	for b: Dictionary in config.buttons:
		if b.get("id", "") == button_id:
			return b
	return {}
