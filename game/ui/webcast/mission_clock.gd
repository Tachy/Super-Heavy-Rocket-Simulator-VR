class_name MissionClock
extends Control
## Mission clock under the timeline arc's apex, as in the webcast (user 2026-10-22): T- before T-0, T+ afterwards
## (hidden without a T-0), the mission's display name below it. Drawn around the control's position (the apex).

const CLOCK_SIZE := 38
const CLOCK_BASELINE_PX := 58.0  ## below the apex
const NAME_SIZE := 14
const NAME_BASELINE_PX := 82.0
const NAME_ALPHA := 0.85
const SECONDS_PER_DAY := 86400

var clock := ""         ## empty: no mission clock
var mission_name := ""


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_values(new_clock: String, new_name: String) -> void:
	if new_clock == clock and new_name == mission_name:
		return
	clock = new_clock
	mission_name = new_name
	queue_redraw()


func _draw() -> void:
	if clock != "":
		WebcastStyle.draw_text(self, Vector2(0.0, CLOCK_BASELINE_PX), clock, WebcastStyle.value_font(), CLOCK_SIZE)
	if mission_name != "":
		WebcastStyle.draw_text(self, Vector2(0.0, NAME_BASELINE_PX), mission_name.to_upper(), WebcastStyle.label_font(),
				NAME_SIZE, Color(1, 1, 1, NAME_ALPHA))


## Area covered by the clock and the mission name, relative to the control's position (empty Rect2 when neither is
## shown): the timeline arc keeps its labels out of it.
func occupied_rect() -> Rect2:
	return text_rect(clock, mission_name, WebcastStyle.value_font(), WebcastStyle.label_font())


static func text_rect(clock_text: String, name_text: String, clock_font: Font, name_font: Font) -> Rect2:
	var rect := Rect2()
	if clock_text != "":
		rect = _line_rect(clock_text, clock_font, CLOCK_SIZE, CLOCK_BASELINE_PX)
	if name_text != "":
		var line := _line_rect(name_text.to_upper(), name_font, NAME_SIZE, NAME_BASELINE_PX)
		rect = line if not rect.has_area() else rect.merge(line)
	return rect


## A centred line of text around its baseline.
static func _line_rect(text: String, font: Font, size: int, baseline_px: float) -> Rect2:
	var width := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, size).x
	var ascent := font.get_ascent(size)
	return Rect2(-0.5 * width, baseline_px - ascent, width, ascent + font.get_descent(size))


## "T- hh:mm:ss" before T-0 (counting down to zero), "T+ hh:mm:ss" from T-0 on; days as "T+ 3 d 04:12:09".
static func format_mission_clock(clock_s: float) -> String:
	var before := clock_s < 0.0
	var total := int(ceilf(-clock_s)) if before else int(floorf(clock_s))
	var days := total / SECONDS_PER_DAY
	var rest := total % SECONDS_PER_DAY
	var hms := "%02d:%02d:%02d" % [rest / 3600, (rest / 60) % 60, rest % 60]
	return "%s %s%s" % ["T-" if before else "T+", "%d d " % days if days > 0 else "", hms]
