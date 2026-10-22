class_name TopBar
extends Control
## Top centre of the webcast overlay, drawn around the control's position (the screen's top centre): the viewed
## object's name, the mission clock (T- before T-0, T+ afterwards; hidden without a T-0) and the UTC.

const NAME_SIZE := 15
const CLOCK_SIZE := 40
const UTC_SIZE := 16
const SECONDS_PER_DAY := 86400

var object_name := ""
var clock := ""  ## empty: no mission clock
var utc := ""


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_values(new_name: String, new_clock: String, new_utc: String) -> void:
	if new_name == object_name and new_clock == clock and new_utc == utc:
		return
	object_name = new_name
	clock = new_clock
	utc = new_utc
	queue_redraw()


func _draw() -> void:
	var label_font := WebcastStyle.label_font()
	var y := 28.0
	if object_name != "":
		WebcastStyle.draw_text(self, Vector2(0.0, y), object_name.to_upper(), label_font, NAME_SIZE, Color(1, 1, 1, 0.85))
	if clock != "":
		y += 42.0
		WebcastStyle.draw_text(self, Vector2(0.0, y), clock, WebcastStyle.value_font(), CLOCK_SIZE)
	y += 26.0
	WebcastStyle.draw_text(self, Vector2(0.0, y), utc, label_font, UTC_SIZE, Color(1, 1, 1, 0.85))


## "T- hh:mm:ss" before T-0 (counting down to zero), "T+ hh:mm:ss" from T-0 on; days as "T+ 3 d 04:12:09".
static func format_mission_clock(clock_s: float) -> String:
	var before := clock_s < 0.0
	var total := int(ceilf(-clock_s)) if before else int(floorf(clock_s))
	var days := total / SECONDS_PER_DAY
	var rest := total % SECONDS_PER_DAY
	var hms := "%02d:%02d:%02d" % [rest / 3600, (rest / 60) % 60, rest % 60]
	return "%s %s%s" % ["T-" if before else "T+", "%d d " % days if days > 0 else "", hms]


## "YYYY-MM-DD hh:mm:ss UTC" from the core's ISO string.
static func format_utc(utc_iso: String) -> String:
	return "%s UTC" % utc_iso.substr(0, 19).replace("T", " ")
