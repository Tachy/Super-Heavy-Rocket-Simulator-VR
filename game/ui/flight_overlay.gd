class_name FlightOverlay
extends Control
## Screen-space flight director over the 3D view: the commanded thrust direction (magenta crosshair)
## and the velocity vector (yellow), drawn where those directions appear from the camera.

const TARGET := Color(1.0, 0.25, 0.9)
const PROGRADE := Color(1.0, 0.85, 0.2)

var world: World
var target_gcrf := Vector3.ZERO
var prograde_gcrf := Vector3.ZERO


func _ready() -> void:
	set_anchors_preset(Control.PRESET_FULL_RECT)
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _process(_delta: float) -> void:
	queue_redraw()


func _draw() -> void:
	if not world.flight.sim.is_active():
		return
	if prograde_gcrf != Vector3.ZERO:
		var p := _screen_point(prograde_gcrf)
		if p.x < INF:
			draw_arc(p, 10.0, 0.0, TAU, 20, PROGRADE, 2.0)
			for v in [Vector2(0, -1), Vector2(1, 0), Vector2(-1, 0)]:
				draw_line(p + v * 10.0, p + v * 18.0, PROGRADE, 2.0)
	if target_gcrf != Vector3.ZERO:
		var t := _screen_point(target_gcrf)
		if t.x < INF:
			draw_arc(t, 16.0, 0.0, TAU, 24, TARGET, 2.5)
			draw_line(t + Vector2(-28, 0), t + Vector2(-8, 0), TARGET, 2.5)
			draw_line(t + Vector2(8, 0), t + Vector2(28, 0), TARGET, 2.5)
			draw_line(t + Vector2(0, -28), t + Vector2(0, -8), TARGET, 2.5)
			draw_line(t + Vector2(0, 8), t + Vector2(0, 28), TARGET, 2.5)


## Where a direction (seen from the camera) appears on screen; INF if behind the camera.
func _screen_point(direction_gcrf: Vector3) -> Vector2:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return Vector2(INF, INF)
	var point := camera.global_position + Frames.to_render(direction_gcrf.normalized()) * 1000.0
	if camera.is_position_behind(point):
		return Vector2(INF, INF)
	return camera.unproject_position(point)
