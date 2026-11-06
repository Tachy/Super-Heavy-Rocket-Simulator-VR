class_name FreeCameraWizard
extends VBoxContainer
## Setup wizard of the free camera ("Set up axes" in the time panel): asks for the six motions in turn, detects the axis
## the user moved furthest (device + index), records its direction, centre and both extremes (FreeCameraCalibration) and
## saves the mapping to user://. Plain controls in the time panel, so it works on the desktop and in the VR panel alike.
## Texts come from the text tables (wizard.*); the state is kept as data and drawn by _render, so a language switch
## re-renders the open step.

signal closed

## [role, title key, instruction key]: the asked ("positive") direction is the first one named.
const STEPS := [
	["roll", "wizard.step.roll.title", "wizard.step.roll.text"],
	["pitch", "wizard.step.pitch.title", "wizard.step.pitch.text"],
	["yaw", "wizard.step.yaw.title", "wizard.step.yaw.text"],
	["forward", "wizard.step.forward.title", "wizard.step.forward.text"],
	["strafe", "wizard.step.strafe.title", "wizard.step.strafe.text"],
	["vertical", "wizard.step.vertical.title", "wizard.step.vertical.text"],
]

enum Phase { PROMPT, MEASURING, RESULT, DONE }

var control: FreeCameraControl

var _calibrator := FreeCameraCalibration.new()
var _entries := {}  ## role -> entry, built up in this run (starts from the saved ones)
var _index := 0
var _phase := Phase.PROMPT
var _last_entry := {}
var _saved := true  ## the mapping was written to user:// (DONE)

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
	I18n.events.language_changed.connect(func(_language: String) -> void: _render())


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
	_status.text = _measuring_status()
	if _calibrator.settled():
		_finish_measuring()


func _measuring_status() -> String:
	var best := _calibrator.best_key()
	if best == "":
		return I18n.text("wizard.nothing_yet")
	return I18n.text("wizard.detected_live", {"axis": _describe(best), "excursion": _calibrator.excursion_of(best)})


func _show_prompt() -> void:
	_phase = Phase.PROMPT
	set_process(false)
	_render()


## Draws the current step from the state (phase, step index, entries).
func _render() -> void:
	if not visible:
		return
	var step: Array = STEPS[mini(_index, STEPS.size() - 1)]
	_title.text = I18n.text("wizard.step_title", {"n": _index + 1, "total": STEPS.size(), "title": I18n.text(step[1])})
	match _phase:
		Phase.PROMPT:
			_text.text = I18n.text(step[2]) + "\n" + I18n.text("wizard.prompt")
			_status.text = I18n.text("wizard.saved", {"entry": _describe_entry(_entries[step[0]])}) \
					if _entries.has(step[0]) else ""
			_set_buttons([["wizard.button.measure", _start_measuring], ["wizard.button.skip", _next],
					["wizard.button.cancel", _close]])
		Phase.MEASURING:
			_text.text = I18n.text(step[2]) + "\n" + I18n.text("wizard.measuring")
			_status.text = _measuring_status() if _calibrator.best_key() != "" else I18n.text("wizard.move_control")
			_set_buttons([["wizard.button.finish", _finish_measuring], ["wizard.button.cancel", _close]])
		Phase.RESULT:
			_text.text = I18n.text(step[2]) + "\n" + I18n.text("wizard.measuring")
			if _last_entry.is_empty():
				_status.text = I18n.text("wizard.not_detected")
				_set_buttons([["wizard.button.repeat", _show_prompt], ["wizard.button.skip", _next],
						["wizard.button.cancel", _close]])
			else:
				_status.text = I18n.text("wizard.detected", {"entry": _describe_entry(_last_entry)})
				_set_buttons([["wizard.button.next", _next], ["wizard.button.repeat", _show_prompt],
						["wizard.button.cancel", _close]])
		Phase.DONE:
			_title.text = I18n.text("wizard.done_title")
			_text.text = I18n.text("wizard.done_saved" if _saved else "wizard.done_failed")
			_status.text = ""
			_set_buttons([["wizard.button.close", _close]])


func _start_measuring() -> void:
	_calibrator.begin(FreeCameraCalibration.snapshot(), _taken_keys())
	_phase = Phase.MEASURING
	set_process(true)
	_render()


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
	_phase = Phase.RESULT
	if result.is_empty():
		_last_entry = {}
		_render()
		return
	var parts: PackedStringArray = str(result["key"]).split(":")
	var device := int(parts[0])
	_last_entry = {"device_name": Input.get_joy_name(device), "axis": int(parts[1]), "center": result["center"],
			"raw_pos": result["raw_pos"], "raw_neg": result["raw_neg"]}
	_entries[STEPS[_index][0]] = _last_entry
	_render()


func _next() -> void:
	_index += 1
	if _index < STEPS.size():
		_show_prompt()
		return
	_phase = Phase.DONE
	_saved = control.set_calibration(_entries)
	_render()


func _close() -> void:
	set_process(false)
	visible = false
	closed.emit()


func _describe(key: String) -> String:
	if key == "":
		return I18n.DASH
	var parts: PackedStringArray = key.split(":")
	return I18n.text("wizard.axis", {"device": Input.get_joy_name(int(parts[0])), "axis": parts[1]})


static func _describe_entry(entry: Dictionary) -> String:
	return I18n.text("wizard.entry", {"device": entry["device_name"], "axis": str(int(entry["axis"])),
			"center": _signed(float(entry["center"])), "pos": _signed(float(entry["raw_pos"])),
			"neg": _signed(float(entry["raw_neg"]))})


## A value with an explicit sign and two decimals (as %+.2f), in the active language's number format.
static func _signed(value: float) -> String:
	var text := I18n.number(value, 2)
	return text if text.begins_with("-") else "+" + text


## Rebuilds the button row from [text key, action] specs.
func _set_buttons(specs: Array) -> void:
	for child in _buttons.get_children():
		_buttons.remove_child(child)
		child.queue_free()
	for spec in specs:
		var button := Button.new()
		button.text = I18n.text(spec[0])
		button.pressed.connect(spec[1])
		_buttons.add_child(button)
