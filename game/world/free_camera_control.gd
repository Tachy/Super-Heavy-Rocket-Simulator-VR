class_name FreeCameraControl
extends RefCounted
## 6-DOF HOTAS control of the free camera: the stick rotates (roll, pitch, yaw about the camera's own axes), the throttle
## translates (rocker = forward / backward, mini-stick = strafe and up / down), each relative to the view direction.
## Mapping and curves: [free_camera] of config/input_hotas.toml (devices by the [stick] / [throttle] names).

## Fallbacks when the file lacks a key (same numbers as the shipped file).
const DEFAULTS := {
	"rotation_max_deg_s": 90.0, "rotation_expo": 0.3, "rotation_deadzone": 0.05,
	"speed_min_mps": 0.1, "speed_max_mps": 1.0e7, "speed_deadzone": 0.05,
	"roll_axis": 0, "pitch_axis": 1, "yaw_axis": 2, "forward_axis": 5, "strafe_axis": 0, "vertical_axis": 1,
}

var _free := {}  ## [free_camera] table
var _stick := {}
var _throttle := {}
var _last_raw := {}  ## role -> raw axis value of the last poll (the readout)
var _devices := {"stick": -1, "throttle": -1}
## The wizard's measured axes (FreeCameraCalibration): role -> {device_name, axis, center, raw_pos, raw_neg}. They replace the
## [free_camera] table's indices for the roles they cover; roles without an entry keep the file's defaults.
var calibration := {}
var _missing_devices := PackedStringArray()


func setup(config_dir: String, calibration_path := FreeCameraCalibration.SAVE_PATH) -> void:
	var mapping := TomlLite.load_file(config_dir.path_join("input_hotas.toml"))
	configure(mapping.get("free_camera", {}), mapping.get("stick", {}), mapping.get("throttle", {}))
	calibration = FreeCameraCalibration.load_file(calibration_path)


func has_calibration() -> bool:
	return not calibration.is_empty()


func set_calibration(entries: Dictionary, calibration_path := FreeCameraCalibration.SAVE_PATH) -> bool:
	calibration = entries
	return FreeCameraCalibration.save(calibration_path, entries)


func configure(free_camera: Dictionary, stick: Dictionary, throttle: Dictionary) -> void:
	_free = free_camera
	_stick = stick
	_throttle = throttle


## Speed of a throttle-type axis: 0 inside the dead zone, else v_min * (v_max / v_min)^((|d| - dz) / (1 - dz)), signed.
## `value` in [-1, 1] after inversion.
static func log_speed_mps(value: float, deadzone: float, v_min_mps: float, v_max_mps: float) -> float:
	var magnitude := absf(value)
	if magnitude <= deadzone:
		return 0.0
	var t := minf((magnitude - deadzone) / (1.0 - deadzone), 1.0)
	return signf(value) * v_min_mps * pow(v_max_mps / v_min_mps, t)


## Rotation rate of a stick axis: 0 inside the dead zone, else the dead-zone-rescaled deflection through the expo curve
## ((1 - expo) x + expo x^3) times the maximum rate, signed.
static func rotation_rate_rad_s(value: float, deadzone: float, expo: float, max_rate_rad_s: float) -> float:
	var magnitude := absf(value)
	if magnitude <= deadzone:
		return 0.0
	var x := minf((magnitude - deadzone) / (1.0 - deadzone), 1.0)
	return signf(value) * ((1.0 - expo) * x + expo * x * x * x) * max_rate_rad_s


## Rotates `basis` about its own axes by `rates` (x pitch, y yaw, z roll; rad/s, positive = right-handed about the local
## axis) over `delta_s`, orthonormalised.
static func rotate_local(basis: Basis, rates: Vector3, delta_s: float) -> Basis:
	var angle := rates * delta_s  # a rotation vector: exact for any combination of the three
	if angle.length() < 1e-12:
		return basis.orthonormalized()
	return (basis * Basis(angle.normalized(), angle.length())).orthonormalized()


## The current command: {rates: Vector3 (pitch, yaw, roll about the camera's axes, rad/s; stick forward = nose down,
## twist right = turn right, stick right = roll right), velocity: Vector3 (camera axes x right, y up, z back; m/s)}.
func poll() -> Dictionary:
	_missing_devices = PackedStringArray()
	_devices["stick"] = _find_device(_stick.get("device_name", ""))
	_devices["throttle"] = _find_device(_throttle.get("device_name", ""))
	var max_rate := deg_to_rad(_number("rotation_max_deg_s"))
	var rotation_deadzone := _number("rotation_deadzone")
	var expo := _number("rotation_expo")
	var roll := rotation_rate_rad_s(_axis("stick", "roll"), rotation_deadzone, expo, max_rate)
	var pitch := rotation_rate_rad_s(_axis("stick", "pitch"), rotation_deadzone, expo, max_rate)
	var yaw := rotation_rate_rad_s(_axis("stick", "yaw"), rotation_deadzone, expo, max_rate)
	var v_min := _number("speed_min_mps")
	var v_max := _number("speed_max_mps")
	var speed_deadzone := _number("speed_deadzone")
	var forward := log_speed_mps(_axis("throttle", "forward"), speed_deadzone, v_min, v_max)
	var strafe := log_speed_mps(_axis("throttle", "strafe"), speed_deadzone, v_min, v_max)
	var vertical := log_speed_mps(_axis("throttle", "vertical"), speed_deadzone, v_min, v_max)
	# Right-handed rotation about the camera axes: stick forward (value -1) is nose down = negative about +x; twist / roll
	# to the right are negative about +y / +z (the camera looks along -z, y is up).
	return {"rates": Vector3(pitch, -yaw, -roll), "velocity": Vector3(strafe, vertical, -forward)}


## German one-line readout of the raw axes (to check the mapping on the hardware), or a hint when a device is missing.
func readout_text() -> String:
	var parts := PackedStringArray()
	for role in ["roll", "pitch", "yaw"]:
		parts.append("%s %+.2f" % [role_label(role), float(_last_raw.get(role, 0.0))])
	for role in ["forward", "strafe", "vertical"]:
		parts.append("%s %+.2f" % [role_label(role), float(_last_raw.get(role, 0.0))])
	var missing := _missing_devices.duplicate()
	if _devices["stick"] < 0 and not calibration.has("roll"):
		missing.append("Joystick")
	if _devices["throttle"] < 0 and not calibration.has("forward"):
		missing.append("Schubhebel")
	var text := "Freie Kamera - Achsen: " + "  ".join(parts)
	if not missing.is_empty():
		text += "\nNicht gefunden: " + ", ".join(missing)
	return text


## Index of the first connected joypad whose name contains `name_part` (as FlightController's), or -1.
static func _find_device(name_part: String) -> int:
	if name_part.is_empty():
		return -1
	for device in Input.get_connected_joypads():
		var raw_name := str(Input.get_joy_info(device).get("raw_name", ""))
		if Input.get_joy_name(device).contains(name_part) or raw_name.contains(name_part):
			return device
	return -1


static func role_label(role: String) -> String:
	match role:
		"roll": return "Rollen"
		"pitch": return "Nicken"
		"yaw": return "Gieren"
		"forward": return "Vor/Zurück"
		"strafe": return "Seitlich"
		_: return "Hoch/Runter"


func _number(key: String) -> float:
	return float(_free.get(key, DEFAULTS[key]))


## The mapped axis' value in [-1, 1] after inversion (0 without the device); remembered for the readout.
func _axis(device_role: String, role: String) -> float:
	if calibration.has(role):
		var entry: Dictionary = calibration[role]
		var found := _find_device(str(entry.get("device_name", "")))
		var measured := 0.0
		if found >= 0:
			measured = FreeCameraCalibration.normalize(Input.get_joy_axis(found, int(entry["axis"])), entry)
		elif not _missing_devices.has(str(entry.get("device_name", ""))):
			_missing_devices.append(str(entry.get("device_name", "")))
		_last_raw[role] = measured
		return measured
	var device: int = _devices[device_role]
	var axis := int(_free.get(role + "_axis", DEFAULTS[role + "_axis"]))
	var value := 0.0
	if device >= 0 and axis >= 0:
		value = Input.get_joy_axis(device, axis)
		if _free.get(role + "_invert", false):
			value = -value
	_last_raw[role] = value
	return value
