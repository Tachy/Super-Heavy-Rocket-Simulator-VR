class_name OrbitCamera
extends Camera3D
## Camera that is always at the render origin (floating origin). Its true position is kept in float64
## GCRF coordinates (`position_gcrf_m`); the world moves around it.
##
## Modes: orbit around a target (a body centre with inertial axes, north up; or a vehicle with the reference body's
## local vertical as up) or stand on the ground (local horizon view). Never rolled.
## Mouse: right drag = rotate (orbit: left/right about the local vertical, up/down pitch about the target) / look
## around (ground); wheel = zoom x1.15 per notch (orbit modes); the left button stays free.
## VR: this camera is not current; its pose is the XR base pose (XrManager.set_base), the headset moves on top.

enum Mode { ORBIT, GROUND }

## [E] Wheel zoom limits: a few metres from a vehicle out to ~10 lunar distances.
const MIN_DISTANCE_M := 5.0
const MAX_DISTANCE_M := 4.0e9
## [E] Mouse drag rate, rad per pixel.
const DRAG_RAD_PER_PX := 0.005
## [E] Ground mode elevation limit (rad).
const GROUND_ELEVATION_LIMIT := 1.5

var mode := Mode.ORBIT
var target_gcrf_m := Vector3.ZERO  ## orbit target: the viewed object's CG or a body centre, set by the world
var reference_center_gcrf_m := Vector3.ZERO  ## centre of the body whose local vertical through the target is "up"
## Name of that reference body ("earth", "moon"); empty: inertial axes, north up. A change between two bodies
## blends "up" over CameraMath.UP_BLEND_S.
var reference_body := ""
var distance_m := 2.0e7
var yaw := 0.6
var pitch := -0.35

## Ground mode: local frame of the observer, set by the world.
var site_gcrf_m := Vector3.ZERO
var site_up_gcrf := Vector3.UP
var site_north_gcrf := Vector3.FORWARD
var site_east_gcrf := Vector3.RIGHT
var azimuth := deg_to_rad(90.0)
var elevation := deg_to_rad(5.0)

var position_gcrf_m := Vector3.ZERO  ## true camera position, read by the world after update()

var _rotating := false
var _frame := Basis.IDENTITY  ## orbit frame of the last update (render axes), blended during a body change
var _frame_body := ""
var _north_render := Vector3.FORWARD
var _blend_from := Basis.IDENTITY
var _blend_left_s := 0.0
## The XrManager autoload, looked up at run time: test scripts compile this class before autoloads exist.
var _xr: Node


func _ready() -> void:
	near = 1.0
	far = 3.0e7
	fov = 50.0
	_xr = get_node_or_null("/root/XrManager")
	current = not _vr()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var button := event as InputEventMouseButton
		if button.button_index == MOUSE_BUTTON_RIGHT:
			_rotating = button.pressed
		elif button.pressed and mode == Mode.ORBIT:
			if button.button_index == MOUSE_BUTTON_WHEEL_UP:
				distance_m = CameraMath.zoom(distance_m, 1, MIN_DISTANCE_M, MAX_DISTANCE_M)
			elif button.button_index == MOUSE_BUTTON_WHEEL_DOWN:
				distance_m = CameraMath.zoom(distance_m, -1, MIN_DISTANCE_M, MAX_DISTANCE_M)
	elif event is InputEventMouseMotion and _rotating:
		var motion := (event as InputEventMouseMotion).relative * DRAG_RAD_PER_PX
		if mode == Mode.ORBIT:
			yaw -= motion.x
			pitch = clampf(pitch - motion.y, -CameraMath.PITCH_LIMIT, CameraMath.PITCH_LIMIT)
		else:
			azimuth = fposmod(azimuth + motion.x * fov / 60.0, TAU)
			elevation = clampf(elevation - motion.y * fov / 60.0, -GROUND_ELEVATION_LIMIT, GROUND_ELEVATION_LIMIT)


## Computes position_gcrf_m and the render orientation for the current mode; `delta_s` (real time) advances the
## "up" blend after a reference-body change.
func update_pose(delta_s: float = 0.0) -> void:
	var view: Basis
	if mode == Mode.ORBIT:
		_frame = _orbit_frame(delta_s)
		position_gcrf_m = target_gcrf_m + Frames.from_render(CameraMath.orbit_offset(_frame, yaw, pitch, distance_m))
		view = CameraMath.orbit_basis(_frame, yaw, pitch)
		near = clampf(distance_m * 1e-4, 1.0, 1000.0)
	else:
		position_gcrf_m = site_gcrf_m
		var horizontal := site_north_gcrf * cos(azimuth) + site_east_gcrf * sin(azimuth)
		var look := horizontal * cos(elevation) + site_up_gcrf * sin(elevation)
		view = Basis.looking_at(Frames.to_render(look).normalized(), Frames.to_render(site_up_gcrf))
		near = 0.2
	position = Vector3.ZERO
	basis = view
	if _vr():
		_xr.camera.near = near
		_xr.camera.far = far
		_xr.set_base(view)


func _vr() -> bool:
	return _xr != null and _xr.xr_active


## Orbit reference frame in render axes: identity (north up) or the reference body's horizon frame through the
## target (y = local up, -z = north), blended over UP_BLEND_S when the reference body changes.
func _orbit_frame(delta_s: float) -> Basis:
	if reference_body == "":
		_frame_body = ""
		_blend_left_s = 0.0
		return Basis.IDENTITY
	var live := CameraMath.horizon_frame(Frames.to_render(target_gcrf_m - reference_center_gcrf_m), _north_render)
	_north_render = -live.z
	if _frame_body != "" and _frame_body != reference_body:
		_blend_from = _frame
		_blend_left_s = CameraMath.UP_BLEND_S
	_frame_body = reference_body
	if _blend_left_s <= 0.0:
		return live
	_blend_left_s = maxf(_blend_left_s - delta_s, 0.0)
	return CameraMath.blend_frame(_blend_from, live, smoothstep(0.0, 1.0, 1.0 - _blend_left_s / CameraMath.UP_BLEND_S))
