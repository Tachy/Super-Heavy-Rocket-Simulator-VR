class_name HotasButtons
extends RefCounted
## Joystick buttons by role, edge-detected (config/input_hotas.toml [buttons]): `stick_device` names the device by a
## substring of its name (as [stick] does), every other integer key is a role -> 0-based button index.
## update() once per frame; pressed(role) is true on the frame of the press only.

var _device_name := ""
var _buttons := {}  ## role -> button index
var _down := {}  ## role -> held at the last update
var _pressed := {}  ## role -> went down at the last update
var _reader := Callable()  ## test hook: reader.call(role) -> bool replaces the joystick read


func setup(config_dir: String) -> void:
	configure(TomlLite.load_file(config_dir.path_join("input_hotas.toml")).get("buttons", {}))


## From a [buttons] table.
func configure(table: Dictionary) -> void:
	_device_name = str(table.get("stick_device", ""))
	_buttons.clear()
	for key: String in table:
		if key != "stick_device" and (table[key] is int or table[key] is float):
			_buttons[key] = int(table[key])


## Replaces the joystick read (tests): `reader` takes the role and returns whether the button is held.
func set_reader(reader: Callable) -> void:
	_reader = reader


func update() -> void:
	for role: String in _buttons:
		var now := _held(role)
		_pressed[role] = now and not _down.get(role, false)
		_down[role] = now


## True on the frame (update) the button of `role` went down.
func pressed(role: String) -> bool:
	return _pressed.get(role, false)


func held(role: String) -> bool:
	return _down.get(role, false)


func button_index(role: String) -> int:
	return _buttons.get(role, -1)


func _held(role: String) -> bool:
	if _reader.is_valid():
		return bool(_reader.call(role))
	var device := FlightController._find_device(_device_name)
	return device >= 0 and Input.is_joy_button_pressed(device, _buttons[role])
