class_name FreeCameraWizard
extends VBoxContainer
## Setup wizard of the free camera ("Achsen einrichten"): asks for the six motions in turn, detects the axis the user moved
## furthest (device + index), records its direction, centre and both extremes (FreeCameraCalibration) and saves the mapping
## to user://. Plain controls in the time panel, so it works on the desktop and in the VR panel alike.

signal closed

## [role, title, instruction]: the asked ("positive") direction is the first one named.
const STEPS := [
	["roll", "Rollen", "Stick ganz nach rechts, dann ganz nach links, dann zurück in die Mitte."],
	["pitch", "Nicken", "Stick ganz nach hinten (Nase hoch), dann ganz nach vorne, dann zurück in die Mitte."],
	["yaw", "Gieren", "Stick nach rechts verdrehen (Twist), dann nach links, dann zurück in die Mitte."],
	["forward", "Vorwärts / Rückwärts (Wippe)", "Wippe ganz nach vorne (vorwärts), dann ganz zurück (rückwärts), dann in die Mitte."],
	["strafe", "Seitlich", "Mini-Stick ganz nach rechts, dann ganz nach links, dann zurück in die Mitte."],
	["vertical", "Hoch / Runter", "Mini-Stick ganz nach oben, dann ganz nach unten, dann zurück in die Mitte."],
]

enum Phase { PROMPT, MEASURING, RESULT, DONE }

var control: FreeCameraControl

var _calibrator := FreeCameraCalibration.new()
var _entries := {}  ## role -> entry, built up in this run (starts from the saved ones)
var _index := 0
var _phase := Phase.PROMPT
var _last_entry := {}

var _title := Label.new()
var _text := Label.new()
var _status := Label.new()
var _buttons := HBoxContainer.new()


func _ready() -> void:
	visible = false
	set_process(false)
	_title.add_theme_font_size_override("font_size", 18)
	_text.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_text.custom_minimum_size.x = 320
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_status.custom_minimum_size.x = 320
	add_child(HSeparator.new())
	add_child(_title)
	add_child(_text)
	add_child(_status)
	add_child(_buttons)


func is_open() -> bool:
	return visible


func open() -> void:
	_entries = control.calibration.duplicate(true)
	_index = 0
	visible = true
	_show_prompt()


func cancel() -> void:
	if visible:
		_close()


func _process(delta: float) -> void:
	if _phase != Phase.MEASURING:
		return
	_calibrator.feed(FreeCameraCalibration.snapshot(), delta)
	var best := _calibrator.best_key()
	_status.text = "Erkannt: %s (Ausschlag %.2f)" % [_describe(best), _calibrator.excursion_of(best)] if best != "" \
			else "Noch keine Bewegung erkannt ..."
	if _calibrator.settled():
		_finish_measuring()


func _show_prompt() -> void:
	_phase = Phase.PROMPT
	set_process(false)
	var step: Array = STEPS[_index]
	_title.text = "Schritt %d/%d: %s" % [_index + 1, STEPS.size(), step[1]]
	_text.text = "%s\nDrücke danach \"Messen\" (Hände in Ruhelage) und bewege den Regler." % step[2]
	_status.text = "Bereits gespeichert: %s" % _describe_entry(_entries[step[0]]) if _entries.has(step[0]) else ""
	_set_buttons([["Messen", _start_measuring], ["Überspringen", _next], ["Abbrechen", _close]])


func _start_measuring() -> void:
	_calibrator.begin(FreeCameraCalibration.snapshot(), _taken_keys())
	_phase = Phase.MEASURING
	set_process(true)
	_text.text = "%s\nDie Messung endet von selbst, sobald der Regler wieder in der Mitte ist." % STEPS[_index][2]
	_status.text = "Bewege den Regler ..."
	_set_buttons([["Fertig", _finish_measuring], ["Abbrechen", _close]])


## Keys ("device:axis") of the axes the earlier steps of this run took, so one control is not taken twice.
func _taken_keys() -> Array:
	var keys: Array = []
	for role: String in _entries:
		if role == STEPS[_index][0]:
			continue
		var entry: Dictionary = _entries[role]
		for device in Input.get_connected_joypads():
			if Input.get_joy_name(device) == str(entry["device_name"]):
				keys.append("%d:%d" % [device, int(entry["axis"])])
	return keys


func _finish_measuring() -> void:
	set_process(false)
	var result := _calibrator.result()
	if result.is_empty():
		_phase = Phase.RESULT
		_last_entry = {}
		_status.text = "Keine Bewegung erkannt."
		_set_buttons([["Wiederholen", _show_prompt], ["Überspringen", _next], ["Abbrechen", _close]])
		return
	var parts: PackedStringArray = str(result["key"]).split(":")
	var device := int(parts[0])
	_last_entry = {"device_name": Input.get_joy_name(device), "axis": int(parts[1]), "center": result["center"],
			"raw_pos": result["raw_pos"], "raw_neg": result["raw_neg"]}
	_entries[STEPS[_index][0]] = _last_entry
	_phase = Phase.RESULT
	_status.text = "Erkannt: %s" % _describe_entry(_last_entry)
	_set_buttons([["Weiter", _next], ["Wiederholen", _show_prompt], ["Abbrechen", _close]])


func _next() -> void:
	_index += 1
	if _index < STEPS.size():
		_show_prompt()
		return
	_phase = Phase.DONE
	var saved := control.set_calibration(_entries)
	_title.text = "Fertig"
	_text.text = "Alle Achsen eingerichtet." if saved else "Speichern fehlgeschlagen - die Zuordnung gilt nur bis zum Beenden."
	_status.text = ""
	_set_buttons([["Schließen", _close]])


func _close() -> void:
	set_process(false)
	visible = false
	closed.emit()


func _describe(key: String) -> String:
	if key == "":
		return "-"
	var parts: PackedStringArray = key.split(":")
	return "%s, Achse %s" % [Input.get_joy_name(int(parts[0])), parts[1]]


static func _describe_entry(entry: Dictionary) -> String:
	return "%s, Achse %d (Mitte %+.2f, Ausschlag %+.2f / %+.2f)" % [entry["device_name"], int(entry["axis"]),
			float(entry["center"]), float(entry["raw_pos"]), float(entry["raw_neg"])]


func _set_buttons(specs: Array) -> void:
	for child in _buttons.get_children():
		_buttons.remove_child(child)
		child.queue_free()
	for spec in specs:
		var button := Button.new()
		button.text = spec[0]
		button.pressed.connect(spec[1])
		_buttons.add_child(button)
