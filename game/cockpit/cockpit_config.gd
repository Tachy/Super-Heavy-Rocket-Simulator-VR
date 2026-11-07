class_name CockpitConfig
extends RefCounted
## A vehicle's cabin definition (config/cockpits/<id>.toml) read through TomlLite. All positions are in the capsule
## stage body frame (docs/subsystems/cockpit.md). A vehicle names its cabin with `cockpit = "<id>"` in its [vehicle]
## table; a `+` stack id (falcon9_block5+crew_dragon) uses the first part that names one.

static var _cache := {}  # vehicle id -> CockpitConfig or null

var id := ""
var capsule_stage_id := ""
var eye := Transform3D.IDENTITY  ## eye point; -Z = gaze forward, Y = head up (Godot camera convention), body axes
var seats: Array[Dictionary] = []  ## raw [[seat]] entries plus "hip" (Vector3)
var screens := {}  ## id -> {centre: Vector3, normal: Vector3, up: Vector3, size: Vector2}
var buttons: Array[Dictionary] = []  ## raw [[button]] entries plus "centre" (Vector3) and "size" (Vector2)
var cameras: Array[Dictionary] = []  ## [[camera]] entries (none until the camera task adds them)
var hotas := {}  ## raw [hotas] table
var hotas_props_enabled := true
var cabin := {}  ## raw [cabin] table (wall, ceiling, tunnel, windows ...)
var console := {}  ## raw [console] table
var tablets: Array[Dictionary] = []  ## raw [[tablet]] entries plus "centre", "normal", "up", "size"
var lights: Array[Dictionary] = []  ## raw [[light]] entries
var raw := {}  ## the whole parsed file


## The cockpit of a vehicle (or `+` stack) id, null when none names one. Cached.
static func for_vehicle(vehicle_id: String) -> CockpitConfig:
	if _cache.has(vehicle_id):
		return _cache[vehicle_id]
	var result: CockpitConfig = null
	for part in vehicle_id.split("+"):
		var path: String = Paths.config_dir().path_join("vehicles").path_join(part + ".toml")
		if not FileAccess.file_exists(path):
			continue
		var cockpit_id: String = str((TomlLite.load_file(path).get("vehicle", {}) as Dictionary).get("cockpit", ""))
		if cockpit_id.is_empty():
			continue
		result = from_file(Paths.config_dir().path_join("cockpits").path_join(cockpit_id + ".toml"), cockpit_id)
		break
	_cache[vehicle_id] = result
	print("CockpitConfig: %s -> %s" % [vehicle_id, result.id if result != null else "none"])
	return result


static func from_file(path: String, cockpit_id: String) -> CockpitConfig:
	if not FileAccess.file_exists(path):
		push_error("CockpitConfig: missing %s" % path)
		return null
	return from_dict(TomlLite.load_file(path), cockpit_id)


static func from_dict(data: Dictionary, cockpit_id: String) -> CockpitConfig:
	var c := CockpitConfig.new()
	c.raw = data
	c.id = cockpit_id
	c.capsule_stage_id = str((data.get("cockpit", {}) as Dictionary).get("capsule_stage", ""))
	var eye_table: Dictionary = data.get("eye", {})
	if eye_table.has("position_m"):
		var forward := _vec(eye_table.get("forward")).normalized()
		var up := _vec(eye_table.get("up")).normalized()
		c.eye = Transform3D(Basis(forward.cross(up), up, -forward), _vec(eye_table["position_m"]))
	for seat: Dictionary in data.get("seat", []):
		var entry := seat.duplicate()
		entry["hip"] = _vec(seat.get("hip_m"))
		c.seats.append(entry)
	for screen: Dictionary in data.get("screen", []):
		c.screens[screen.get("id", "")] = _panel(screen)
	for button: Dictionary in data.get("button", []):
		var entry := button.duplicate()
		entry["centre"] = _vec(button.get("centre_m"))
		entry["size"] = _vec2(button.get("size_m"))
		c.buttons.append(entry)
	for camera: Dictionary in data.get("camera", []):
		c.cameras.append(camera)
	c.hotas = data.get("hotas", {})
	c.hotas_props_enabled = bool(c.hotas.get("props", true))
	c.cabin = data.get("cabin", {})
	c.console = data.get("console", {})
	for tablet: Dictionary in data.get("tablet", []):
		var entry := _panel(tablet)
		entry["id"] = tablet.get("id", "")
		c.tablets.append(entry)
	for light: Dictionary in data.get("light", []):
		c.lights.append(light)
	return c


## Problems of the definition, English developer messages; empty = valid.
func validate() -> PackedStringArray:
	var errors := PackedStringArray()
	if capsule_stage_id.is_empty():
		errors.append("[cockpit] capsule_stage missing")
	if eye.origin == Vector3.ZERO:
		errors.append("[eye] position_m missing")
	if seats.is_empty():
		errors.append("no [[seat]]")
	var commanders := 0
	var seat_ids := {}
	for seat in seats:
		if seat.get("role", "") == "commander":
			commanders += 1
		if seat_ids.has(seat.get("id", "")):
			errors.append("duplicate seat id '%s'" % seat.get("id", ""))
		seat_ids[seat.get("id", "")] = true
	if commanders != 1:
		errors.append("exactly one seat must have role \"commander\" (found %d)" % commanders)
	for screen_id: String in ["left", "centre", "right"]:
		if not screens.has(screen_id):
			errors.append("[[screen]] '%s' missing" % screen_id)
	for screen_id: String in screens:
		var screen: Dictionary = screens[screen_id]
		if absf((screen["normal"] as Vector3).length() - 1.0) > 1e-3:
			errors.append("screen '%s': normal is not a unit vector" % screen_id)
		if (screen["size"] as Vector2).x <= 0.0 or (screen["size"] as Vector2).y <= 0.0:
			errors.append("screen '%s': size_m must be positive" % screen_id)
	var button_ids := {}
	for button in buttons:
		var button_id: String = button.get("id", "")
		if button_ids.has(button_id):
			errors.append("duplicate button id '%s'" % button_id)
		button_ids[button_id] = true
		if button.has("panel") and not screens.has(button["panel"]):
			errors.append("button '%s': unknown panel '%s'" % [button_id, button["panel"]])
	return errors


## Problems with a crew of `count` in this cabin (1..seat count); empty = valid.
func validate_crew(count: int) -> PackedStringArray:
	var errors := PackedStringArray()
	if count < 1 or count > seats.size():
		errors.append("crew count %d outside 1..%d seats" % [count, seats.size()])
	return errors


static func _panel(table: Dictionary) -> Dictionary:
	return {
		"centre": _vec(table.get("centre_m")),
		"normal": _vec(table.get("normal")),
		"up": _vec(table.get("up")),
		"size": _vec2(table.get("size_m")),
	}


static func _vec(value: Variant) -> Vector3:
	if value is Array and (value as Array).size() == 3:
		return Vector3(value[0], value[1], value[2])
	return Vector3.ZERO


static func _vec2(value: Variant) -> Vector2:
	if value is Array and (value as Array).size() == 2:
		return Vector2(value[0], value[1])
	return Vector2.ZERO
