class_name TimelineArc
extends Control
## Mission timeline as a 120 deg arc whose apex is the control's position (its centre lies RADIUS_PX below; a control
## placed off screen would be culled): the time window maps linearly onto the arc (left = earlier), events are dots
## (filled = happened, hollow = predicted) with radial labels from TimelineWindow.visible_labels, and a marker for
## "now". Both ends fade out.

const RADIUS_PX := 230.0
const SPAN_RAD := TAU / 3.0  ## 120 deg
## Outer part of the arc length over which line, dots and labels fade out at each end (smoothstep). [E] user
## 2026-10-22: the arc ends fade as in the webcasts.
const ARC_FADE_FRACTION := 0.15
const SEGMENTS := 96
const DOT_RADIUS_PX := 4.5
const LABEL_SIZE := 14
const LABEL_GAP_PX := 12.0
## [E] Opacity of the arc after "now" (still to come) relative to the flown part.
const FUTURE_ALPHA := 0.45
const NOW_MARKER_PX := 9.0

var window := Vector2.ZERO  ## (start, end) TDB s of the displayed window
var now_tdb_s := 0.0
var events: Array = []      ## mission_timeline()["events"]


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_timeline(new_window: Vector2, new_now_tdb_s: float, new_events: Array) -> void:
	window = new_window
	now_tdb_s = new_now_tdb_s
	events = new_events
	queue_redraw()


## Opacity at `fraction` (0 = left end, 1 = right end) of the arc length.
static func fade_alpha(fraction: float) -> float:
	if fraction <= 0.0 or fraction >= 1.0:
		return 0.0
	return smoothstep(0.0, ARC_FADE_FRACTION, minf(fraction, 1.0 - fraction))


## Arc length in px (TimelineWindow.visible_labels thins the labels on it).
static func arc_px() -> float:
	return RADIUS_PX * SPAN_RAD


## Angle from the vertical (negative = left) of `fraction` along the arc.
static func angle_of(fraction: float) -> float:
	return (fraction - 0.5) * SPAN_RAD


static func point_of(fraction: float, radius := RADIUS_PX) -> Vector2:
	var a := angle_of(fraction)
	return Vector2(0.0, RADIUS_PX) + Vector2(sin(a), -cos(a)) * radius


func _fraction_of(tdb_s: float) -> float:
	return (tdb_s - window.x) / (window.y - window.x)


func _draw() -> void:
	if window.y <= window.x:
		return
	var now_f := _fraction_of(now_tdb_s)
	var points := PackedVector2Array()
	var colors := PackedColorArray()
	var halo := PackedColorArray()
	for i in SEGMENTS + 1:
		var f := float(i) / SEGMENTS
		var a := fade_alpha(f) * (1.0 if f <= now_f else FUTURE_ALPHA)
		points.append(point_of(f))
		colors.append(Color(WebcastStyle.WHITE, a))
		halo.append(Color(WebcastStyle.HALO, WebcastStyle.HALO.a * a))
	draw_polyline_colors(points, halo, WebcastStyle.HALO_PX, true)
	draw_polyline_colors(points, colors, WebcastStyle.STROKE_PX, true)

	for e: Dictionary in events:
		var f := _fraction_of(float(e["tdb_s"]))
		var a := fade_alpha(f)
		if a <= 0.0:
			continue
		if e.get("predicted", false):
			WebcastStyle.draw_arc_stroke(self, point_of(f), DOT_RADIUS_PX, 0.0, TAU, Color(WebcastStyle.WHITE, a))
		else:
			WebcastStyle.draw_dot(self, point_of(f), DOT_RADIUS_PX, Color(WebcastStyle.WHITE, a))
	var font := WebcastStyle.label_font()
	for e: Dictionary in TimelineWindow.visible_labels(events, window, arc_px()):
		var f := _fraction_of(float(e["tdb_s"]))
		_draw_label(str(e["label"]), f, Color(WebcastStyle.WHITE, fade_alpha(f)), font)
	_draw_now_marker(now_f)
	draw_set_transform_matrix(Transform2D.IDENTITY)


## Radial label outside the dot. Right of the top it reads outward, left of it inward (towards the dot), so that
## every label reads from left to right.
func _draw_label(text: String, f: float, color: Color, font: Font) -> void:
	if color.a <= 0.0:
		return
	var a := angle_of(f)
	var start := point_of(f, RADIUS_PX + LABEL_GAP_PX)
	var baseline := WebcastStyle.center_offset(font, LABEL_SIZE)
	if a >= 0.0:
		draw_set_transform(start, a - 0.5 * PI)
		WebcastStyle.draw_text(self, Vector2(0.0, baseline), text, font, LABEL_SIZE, color, WebcastStyle.Align.LEFT)
	else:
		draw_set_transform(start, a + 0.5 * PI)
		WebcastStyle.draw_text(self, Vector2(0.0, baseline), text, font, LABEL_SIZE, color, WebcastStyle.Align.RIGHT)
	draw_set_transform_matrix(Transform2D.IDENTITY)


## Small triangle inside the arc pointing at "now".
func _draw_now_marker(f: float) -> void:
	var alpha := fade_alpha(f)
	if alpha <= 0.0:
		return
	var a := angle_of(f)
	var outward := Vector2(sin(a), -cos(a))
	var side := Vector2(-outward.y, outward.x)
	var tip := point_of(f, RADIUS_PX - 5.0)
	var base := tip - outward * NOW_MARKER_PX
	var triangle := PackedVector2Array([tip, base + side * NOW_MARKER_PX * 0.6, base - side * NOW_MARKER_PX * 0.6])
	draw_colored_polygon(triangle, Color(WebcastStyle.WHITE, alpha))
