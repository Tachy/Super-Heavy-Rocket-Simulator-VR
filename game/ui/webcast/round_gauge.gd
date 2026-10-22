class_name RoundGauge
extends Control
## One round webcast gauge drawn around the control's position: a thin ring with the value arc, the value and its
## unit inside, the label below. Unsigned gauges fill clockwise over a 270 deg track open at the bottom; signed
## gauges fill from the top to the right (positive) or left (negative).

const RADIUS_PX := 40.0
const TRACK_START_RAD := 0.75 * PI  ## bottom left (screen angles: clockwise from +x)
const TRACK_SPAN_RAD := 1.5 * PI
const SIGNED_SPAN_RAD := 0.75 * PI  ## each side of the top
const VALUE_SIZE := 24
const UNIT_SIZE := 12
const LABEL_SIZE := 13
const LABEL_GAP_PX := 22.0

var label := ""
var unit := ""
var text := ""          ## formatted value
var fraction := 0.0     ## 0..1 of the track (signed: -1..1)
var signed := false


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_value(new_text: String, new_fraction: float) -> void:
	if new_text == text and is_equal_approx(new_fraction, fraction):
		return
	text = new_text
	fraction = new_fraction
	queue_redraw()


func _draw() -> void:
	var c := Vector2.ZERO
	if signed:
		WebcastStyle.draw_arc_stroke(self, c, RADIUS_PX, -0.5 * PI - SIGNED_SPAN_RAD, -0.5 * PI + SIGNED_SPAN_RAD,
				WebcastStyle.DIM)
		var f := clampf(fraction, -1.0, 1.0)
		WebcastStyle.draw_arc_stroke(self, c, RADIUS_PX, -0.5 * PI, -0.5 * PI + f * SIGNED_SPAN_RAD)
	else:
		WebcastStyle.draw_arc_stroke(self, c, RADIUS_PX, TRACK_START_RAD, TRACK_START_RAD + TRACK_SPAN_RAD, WebcastStyle.DIM)
		WebcastStyle.draw_arc_stroke(self, c, RADIUS_PX, TRACK_START_RAD,
				TRACK_START_RAD + clampf(fraction, 0.0, 1.0) * TRACK_SPAN_RAD)
	var value_font := WebcastStyle.value_font()
	var label_font := WebcastStyle.label_font()
	WebcastStyle.draw_text(self, Vector2(0.0, WebcastStyle.center_offset(value_font, VALUE_SIZE) - 4.0), text,
			value_font, VALUE_SIZE)
	WebcastStyle.draw_text(self, Vector2(0.0, 22.0), unit, label_font, UNIT_SIZE, Color(1, 1, 1, 0.8))
	WebcastStyle.draw_text(self, Vector2(0.0, RADIUS_PX + LABEL_GAP_PX), label.to_upper(), label_font, LABEL_SIZE)


## German number format: "." groups thousands, "," is the decimal mark; no negative zero.
static func format_number(value: float, decimals: int) -> String:
	var scale := pow(10.0, decimals)
	var scaled := roundf(absf(value) * scale)
	var negative := value < 0.0 and scaled > 0.0
	var whole := int(scaled / scale) if decimals > 0 else int(scaled)
	var frac := int(scaled - float(whole) * scale) if decimals > 0 else 0
	var digits := str(whole)
	var grouped := ""
	for i in digits.length():
		if i > 0 and (digits.length() - i) % 3 == 0:
			grouped += "."
		grouped += digits[i]
	if decimals > 0:
		grouped += "," + str(frac).pad_zeros(decimals)
	return ("-" if negative else "") + grouped
