class_name TimelineArc
extends Control
## Mission timeline as a flat arc whose apex is the control's position (its centre lies RADIUS_PX below; a control
## placed off screen would be culled). "Now" is always the apex (user 2026-10-22): TimelineWindow.arc_position maps
## the time from now compressively onto the arc, so events enter on the right, pass the apex and leave on the left
## (the scale eases, so they glide). Events are dots
## (filled = happened, hollow = predicted) with labels from TimelineWindow.visible_labels, written along the arc and
## alternating below and above the line (a label that would cover the mission clock under the apex goes to the other
## side, or is left out when both collide); the flown part (left of the apex) is brighter. Both ends fade out.

## [E] Flat and wide (webcast reference image, user 2026-10-22): ~610 px chord, ~70 px rise.
const RADIUS_PX := 720.0
const HALF_ANGLE_RAD := 0.44
const ARC_FADE_FRACTION := TimelineWindow.ARC_FADE_FRACTION
const SEGMENTS := 96
const DOT_RADIUS_PX := 4.5
const LABEL_SIZE := 14
const LABEL_GAP_PX := 9.0       ## line to the near edge of a label
## [E] Opacity of the arc after "now" (still to come) relative to the flown part.
const FUTURE_ALPHA := 0.45
const NOW_TICK_PX := 8.0        ## half length of the "now" tick across the line
const NOW_TICK_WIDTH_PX := 3.0
## label_side results: thinned out, below or above the line.
const SIDE_NONE := 0
const SIDE_BELOW := 1
const SIDE_ABOVE := 2

var time_scale := Vector2.ZERO  ## TimelineWindow scale (knee_s, reach_s) shown; zero: nothing drawn
var now_tdb_s := 0.0
var events: Array = []      ## mission_timeline()["events"], sorted by time
## Area no label may cover, relative to the apex (the mission clock and name under it); empty: none.
var keep_out := Rect2()


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_timeline(new_scale: Vector2, new_now_tdb_s: float, new_events: Array) -> void:
	time_scale = new_scale
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
	return RADIUS_PX * 2.0 * HALF_ANGLE_RAD


## Fraction along the arc (0 = left end, 0.5 = the apex, 1 = right end) of an event `dt_s` from now.
static func fraction_of(dt_s: float, arc_scale: Vector2) -> float:
	return 0.5 * (TimelineWindow.arc_position(dt_s, arc_scale) + 1.0)


## Angle from the vertical (negative = left) of `fraction` along the arc.
static func angle_of(fraction: float) -> float:
	return (2.0 * fraction - 1.0) * HALF_ANGLE_RAD


## Point at `fraction` along the arc, relative to the apex; `radius` > RADIUS_PX lies above the line.
static func point_of(fraction: float, radius := RADIUS_PX) -> Vector2:
	var a := angle_of(fraction)
	return Vector2(0.0, RADIUS_PX) + Vector2(sin(a), -cos(a)) * radius


## Sides of `count` visible labels (true = below the line), alternating along the arc. The first label's side comes
## from its index in the whole event list, so the others keep their sides while events leave on the left.
static func label_sides(first_event_index: int, count: int) -> Array:
	var sides: Array = []
	for k in count:
		sides.append((first_event_index + k) % 2 == 0)
	return sides


func _draw() -> void:
	if time_scale.y <= 0.0:
		return
	var now_f := 0.5
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
		var f := fraction_of(float(e["tdb_s"]) - now_tdb_s, time_scale)
		var a := fade_alpha(f)
		if a <= 0.0:
			continue
		if e.get("predicted", false):
			WebcastStyle.draw_arc_stroke(self, point_of(f), DOT_RADIUS_PX, 0.0, TAU, Color(WebcastStyle.WHITE, a))
		else:
			WebcastStyle.draw_dot(self, point_of(f), DOT_RADIUS_PX, Color(WebcastStyle.WHITE, a))
	var shown := TimelineWindow.visible_labels(events, now_tdb_s, time_scale, arc_px())
	if not shown.is_empty():
		var sides := label_sides(events.find(shown[0]), shown.size())
		var font := WebcastStyle.label_font()
		for k in shown.size():
			var f := fraction_of(float(shown[k]["tdb_s"]) - now_tdb_s, time_scale)
			var text := str(shown[k]["label"])
			var side := label_side(text, f, sides[k], font, keep_out)
			if side != SIDE_NONE:
				_draw_label(text, f, side == SIDE_BELOW, Color(WebcastStyle.WHITE, fade_alpha(f)), font)
	_draw_now_tick(now_f)


## Screen-aligned bounding box (relative to the apex) of a label drawn by _draw_label.
static func label_rect(text: String, f: float, below: bool, font: Font) -> Rect2:
	var ascent := font.get_ascent(LABEL_SIZE)
	var descent := font.get_descent(LABEL_SIZE)
	var width := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, LABEL_SIZE).x
	var baseline := LABEL_GAP_PX + ascent if below else -LABEL_GAP_PX - descent
	var xform := Transform2D(angle_of(f), point_of(f))
	var local := Rect2(-0.5 * width, baseline - ascent, width, ascent + descent)
	return xform * local  # Transform2D * Rect2: the axis-aligned box of the rotated rectangle


## Side of a label: the preferred one unless its box meets `avoid` (the mission clock), then the other; neither:
## SIDE_NONE (thinned out).
static func label_side(text: String, f: float, prefer_below: bool, font: Font, avoid: Rect2) -> int:
	for below: bool in [prefer_below, not prefer_below]:
		if not avoid.has_area() or not label_rect(text, f, below, font).intersects(avoid):
			return SIDE_BELOW if below else SIDE_ABOVE
	return SIDE_NONE


## Label written along the arc (rotated to the local tangent, upright), centred on its dot, below or above the line.
func _draw_label(text: String, f: float, below: bool, color: Color, font: Font) -> void:
	if color.a <= 0.0:
		return
	var ascent := font.get_ascent(LABEL_SIZE)
	var descent := font.get_descent(LABEL_SIZE)
	draw_set_transform(point_of(f), angle_of(f))
	var baseline := LABEL_GAP_PX + ascent if below else -LABEL_GAP_PX - descent
	WebcastStyle.draw_text(self, Vector2(0.0, baseline), text, font, LABEL_SIZE, color)
	draw_set_transform_matrix(Transform2D.IDENTITY)


## A short bright tick across the line at "now" (the apex).
func _draw_now_tick(f: float) -> void:
	var alpha := fade_alpha(f)
	if alpha <= 0.0:
		return
	var a := angle_of(f)
	var outward := Vector2(sin(a), -cos(a))
	var p := point_of(f)
	draw_line(p - outward * NOW_TICK_PX, p + outward * NOW_TICK_PX, Color(WebcastStyle.HALO, WebcastStyle.HALO.a * alpha),
			NOW_TICK_WIDTH_PX + WebcastStyle.HALO_PX - WebcastStyle.STROKE_PX, true)
	draw_line(p - outward * NOW_TICK_PX, p + outward * NOW_TICK_PX, Color(WebcastStyle.WHITE, alpha), NOW_TICK_WIDTH_PX, true)
