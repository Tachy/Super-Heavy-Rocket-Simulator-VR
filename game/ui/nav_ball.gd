class_name NavBall
extends Control
## Attitude indicator: azimuthal-equidistant view of the sky around the vehicle's nose.
## Centre = nose (body +x), screen up = body -y, screen right = body -z (so stick forward = nose down,
## yaw right = nose right; see sim::FlightSession). Reference frame: local horizon at the vehicle.
## Markers: prograde (yellow) / retrograde, target thrust direction from guidance or burn (magenta).

const RADIUS := 105.0
const SKY := Color(0.20, 0.38, 0.62)
const GROUND := Color(0.42, 0.29, 0.17)
const PROGRADE := Color(1.0, 0.85, 0.2)
const TARGET := Color(1.0, 0.25, 0.9)

var attitude := Quaternion.IDENTITY   ## body -> GCRF
var up_gcrf := Vector3.UP             ## local vertical
var north_gcrf := Vector3.FORWARD     ## local north (horizontal)
var prograde_gcrf := Vector3.ZERO     ## velocity direction (surface-relative low, inertial in space)
var target_gcrf := Vector3.ZERO       ## commanded thrust direction, zero = none


func _ready() -> void:
	custom_minimum_size = Vector2(RADIUS * 2.0 + 20.0, RADIUS * 2.0 + 20.0)


func _process(_delta: float) -> void:
	queue_redraw()


func _draw() -> void:
	var center := size * 0.5
	draw_circle(center, RADIUS + 3.0, Color(0.05, 0.05, 0.07))
	# Sky / ground: shade by elevation, sampled on a polar grid.
	for ring in 12:
		for sector in 36:
			var r0 := RADIUS * ring / 12.0
			var r1 := RADIUS * (ring + 1) / 12.0
			var a0 := TAU * sector / 36.0
			var a1 := TAU * (sector + 1) / 36.0
			var mid := _unproject(center, Vector2.from_angle((a0 + a1) * 0.5) * (r0 + r1) * 0.5)
			var color := SKY if mid.dot(up_gcrf) >= 0.0 else GROUND
			draw_colored_polygon(PackedVector2Array([center + Vector2.from_angle(a0) * r0, center + Vector2.from_angle(a1) * r0,
					center + Vector2.from_angle(a1) * r1, center + Vector2.from_angle(a0) * r1]), color)
	# Horizon and pitch ladder.
	for elevation in [-60, -30, 0, 30, 60]:
		var points := PackedVector2Array()
		for azimuth in range(0, 361, 5):
			var p := _project(center, _direction(deg_to_rad(azimuth), deg_to_rad(elevation)))
			if p.x < INF:
				points.append(p)
		if points.size() > 1:
			draw_polyline(points, Color(1, 1, 1, 0.9 if elevation == 0 else 0.35), 2.0 if elevation == 0 else 1.0)
	for label in [["N", 0.0], ["O", 90.0], ["S", 180.0], ["W", 270.0]]:
		var p := _project(center, _direction(deg_to_rad(label[1]), 0.0))
		if p.x < INF:
			draw_string(get_theme_default_font(), p + Vector2(-5, -4), label[0], HORIZONTAL_ALIGNMENT_LEFT, -1, 13)
	# Markers.
	if prograde_gcrf != Vector3.ZERO:
		_marker(center, prograde_gcrf, PROGRADE, false)
		_marker(center, -prograde_gcrf, PROGRADE, true)
	if target_gcrf != Vector3.ZERO:
		_marker(center, target_gcrf, TARGET, false)
	# Fixed nose reference.
	draw_line(center + Vector2(-22, 0), center + Vector2(-8, 0), Color.ORANGE, 3.0)
	draw_line(center + Vector2(8, 0), center + Vector2(22, 0), Color.ORANGE, 3.0)
	draw_circle(center, 3.0, Color.ORANGE)
	draw_arc(center, RADIUS + 3.0, 0.0, TAU, 64, Color(0.6, 0.6, 0.65), 2.0)


func _direction(azimuth: float, elevation: float) -> Vector3:
	var east := north_gcrf.cross(up_gcrf).normalized()  # N x U = E
	var horizontal := north_gcrf * cos(azimuth) + east * sin(azimuth)
	return horizontal * cos(elevation) + up_gcrf * sin(elevation)


## GCRF direction -> point on the ball (azimuthal equidistant around the nose).
func _project(center: Vector2, direction_gcrf: Vector3) -> Vector2:
	var d := attitude.inverse() * direction_gcrf.normalized()
	var angle := acos(clampf(d.x, -1.0, 1.0))
	var screen := Vector2(-d.z, d.y)  # right = -z, down = +y
	if screen.length() < 1e-9:
		return center if d.x > 0.0 else Vector2(INF, INF)
	return center + screen.normalized() * RADIUS * angle / PI


## Ball point (offset from centre) -> GCRF direction (inverse of _project).
func _unproject(_center: Vector2, offset: Vector2) -> Vector3:
	var angle := offset.length() / RADIUS * PI
	var dir2 := offset.normalized() if offset.length() > 1e-9 else Vector2.ZERO
	var body := Vector3(cos(angle), dir2.y * sin(angle), -dir2.x * sin(angle))
	return attitude * body


func _marker(center: Vector2, direction_gcrf: Vector3, color: Color, retro: bool) -> void:
	var p := _project(center, direction_gcrf)
	if p.x == INF:
		return
	if retro:
		draw_arc(p, 7.0, 0.0, TAU, 16, color, 2.0)
		draw_line(p + Vector2(-5, -5), p + Vector2(5, 5), color, 2.0)
		draw_line(p + Vector2(-5, 5), p + Vector2(5, -5), color, 2.0)
	else:
		draw_arc(p, 8.0, 0.0, TAU, 16, color, 2.5)
		draw_circle(p, 2.5, color)
		for v in [Vector2(0, -1), Vector2(1, 0), Vector2(-1, 0)]:
			draw_line(p + v * 8.0, p + v * 14.0, color, 2.5)
