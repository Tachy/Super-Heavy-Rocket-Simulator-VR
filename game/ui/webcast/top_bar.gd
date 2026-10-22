class_name TopBar
extends Control
## Top centre of the webcast overlay, drawn around the control's position (the screen's top centre): the UTC.

const UTC_SIZE := 16
const UTC_BASELINE_PX := 30.0
const UTC_ALPHA := 0.85

var utc := ""


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_utc(new_utc: String) -> void:
	if new_utc == utc:
		return
	utc = new_utc
	queue_redraw()


func _draw() -> void:
	WebcastStyle.draw_text(self, Vector2(0.0, UTC_BASELINE_PX), utc, WebcastStyle.label_font(), UTC_SIZE,
			Color(1, 1, 1, UTC_ALPHA))


## "YYYY-MM-DD hh:mm:ss UTC" from the core's ISO string.
static func format_utc(utc_iso: String) -> String:
	return "%s UTC" % utc_iso.substr(0, 19).replace("T", " ")
