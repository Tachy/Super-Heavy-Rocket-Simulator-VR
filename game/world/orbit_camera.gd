class_name OrbitCamera
extends Camera3D
## Camera that is always at the render origin (floating origin). Its true position is kept in float64
## GCRF coordinates (`position_gcrf_m`); the world moves around it.
##
## Modes: orbit around a body (inertial axes, north up) or stand on the ground (local horizon view).
## Mouse: drag = rotate / look around, wheel = zoom (orbit modes).

enum Mode { ORBIT, GROUND }

var mode := Mode.ORBIT
var focus_center_gcrf_m := Vector3.ZERO  ## orbit target (body centre), set by the world
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

var _dragging := false


func _ready() -> void:
	near = 1.0
	far = 3.0e7
	fov = 50.0
	current = true


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var button := event as InputEventMouseButton
		if button.button_index in [MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT]:
			_dragging = button.pressed
		elif button.pressed and mode == Mode.ORBIT:
			if button.button_index == MOUSE_BUTTON_WHEEL_UP:
				distance_m /= 1.15
			elif button.button_index == MOUSE_BUTTON_WHEEL_DOWN:
				distance_m *= 1.15
	elif event is InputEventMouseMotion and _dragging:
		var motion := (event as InputEventMouseMotion).relative * 0.005
		if mode == Mode.ORBIT:
			yaw -= motion.x
			pitch = clampf(pitch - motion.y, -1.55, 1.55)
		else:
			azimuth = fposmod(azimuth + motion.x * fov / 60.0, TAU)
			elevation = clampf(elevation - motion.y * fov / 60.0, -1.5, 1.5)


## Computes position_gcrf_m and the render orientation for the current mode.
func update_pose() -> void:
	if mode == Mode.ORBIT:
		var offset_render := Basis.from_euler(Vector3(pitch, yaw, 0.0)) * Vector3(0.0, 0.0, distance_m)
		position_gcrf_m = focus_center_gcrf_m + Frames.from_render(offset_render)
		basis = Basis.looking_at(-offset_render.normalized(), Vector3.UP)
		near = clampf(distance_m * 1e-4, 1.0, 1000.0)
	else:
		position_gcrf_m = site_gcrf_m
		var horizontal := site_north_gcrf * cos(azimuth) + site_east_gcrf * sin(azimuth)
		var look := horizontal * cos(elevation) + site_up_gcrf * sin(elevation)
		basis = Basis.looking_at(Frames.to_render(look).normalized(), Frames.to_render(site_up_gcrf))
		near = 0.2
	position = Vector3.ZERO
