class_name TopBar
extends Control
## Top centre of the webcast overlay, drawn around the control's position (the screen's top centre): the UTC and,
## smaller under it, the local time at the camera (LocalTime; only near the Earth's surface, else empty).

const UTC_SIZE := 16
const UTC_BASELINE_PX := 30.0
const UTC_ALPHA := 0.85
const LOCAL_SIZE := 12
const LOCAL_BASELINE_PX := 48.0
const LOCAL_ALPHA := 0.75
const WEATHER_BASELINE_PX := 64.0

var utc := ""
var local_time := ""
var weather := ""


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_utc(new_utc: String) -> void:
	if new_utc == utc:
		return
	utc = new_utc
	queue_redraw()


## The local-time line under the UTC ("" hides it).
func show_local_time(line: String) -> void:
	if line == local_time:
		return
	local_time = line
	queue_redraw()


## The weather source line under the local time ("" hides it).
func show_weather(line: String) -> void:
	if line == weather:
		return
	weather = line
	queue_redraw()


func _draw() -> void:
	WebcastStyle.draw_text(self, Vector2(0.0, UTC_BASELINE_PX), utc, WebcastStyle.label_font(), UTC_SIZE,
			Color(1, 1, 1, UTC_ALPHA))
	if local_time != "":
		WebcastStyle.draw_text(self, Vector2(0.0, LOCAL_BASELINE_PX), local_time, WebcastStyle.label_font(),
				LOCAL_SIZE, Color(1, 1, 1, LOCAL_ALPHA))
	if weather != "":
		WebcastStyle.draw_text(self, Vector2(0.0, WEATHER_BASELINE_PX), weather, WebcastStyle.label_font(),
				LOCAL_SIZE, Color(1, 1, 1, LOCAL_ALPHA))


## "YYYY-MM-DD hh:mm:ss UTC" from the core's ISO string.
static func format_utc(utc_iso: String) -> String:
	return "%s UTC" % utc_iso.substr(0, 19).replace("T", " ")
