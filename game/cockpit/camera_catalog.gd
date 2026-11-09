class_name CameraCatalog
extends RefCounted
## The external cameras of the left screen (docs/subsystems/cockpit.md, "External cameras"): the [[camera]] entries of the
## cabin definition against the live bodies of the simulation. A camera is mounted on a stage (config id, optionally
## a vehicle id fragment); it is offered while a body carries that stage (and, with `requires`, while that capsule_state
## flag is true), and it rides that body. Poses in float64 from the snapshot, never from scene nodes.
## `sim`: a FlightSim (typed Object so that a test can pass a stub with the same getters).

const DEFAULT_ID := "chase"  ## the camera the screen falls back to
const DEFAULT_COCKPIT := "crew_dragon"


## The ids offered now, in config order.
static func available(sim: Object, config: CockpitConfig = null) -> Array[String]:
	var ids: Array[String] = []
	config = _config(config)
	if config == null or not sim.is_active():
		return ids
	for entry in config.cameras:
		if body_of(sim, entry) >= 0:
			ids.append(String(entry["id"]))
	return ids


## Every configured id in config order (offered or not).
static func all_ids(config: CockpitConfig = null) -> Array[String]:
	var ids: Array[String] = []
	config = _config(config)
	if config != null:
		for entry in config.cameras:
			ids.append(String(entry["id"]))
	return ids


## The config entry of `id`, {} when unknown.
static func entry_of(id: String, config: CockpitConfig = null) -> Dictionary:
	config = _config(config)
	if config != null:
		for entry in config.cameras:
			if String(entry["id"]) == id:
				return entry
	return {}


## The index of the body that carries the camera's stage (and meets its `requires`), -1 when there is none.
static func body_of(sim: Object, entry: Dictionary) -> int:
	if entry.is_empty() or not sim.is_active():
		return -1
	var vehicle_part := String(entry.get("vehicle", ""))
	for i in sim.body_count():
		for stage: Dictionary in sim.body_stages(i):
			if String(stage["id"]) != String(entry["stage"]):
				continue
			if not vehicle_part.is_empty() and not String(stage.get("vehicle_id", "")).contains(vehicle_part):
				continue
			var requires := String(entry.get("requires", ""))
			if not requires.is_empty() and not bool((sim.capsule_state(i) as Dictionary).get(requires, false)):
				continue
			return i
	return -1


## The camera's pose in GCRF (float64): origin = body position + attitude * ((stage base - CG) along the station axis +
## mount), axes (columns right, up, back) = attitude * (look, up) as a camera basis. Identity when the camera has no body.
static func mount_pose_gcrf(sim: Object, id: String, config: CockpitConfig = null) -> Transform3D:
	var entry := entry_of(id, config)
	var index := body_of(sim, entry)
	if index < 0:
		return Transform3D.IDENTITY
	var base_m := 0.0
	var vehicle_part := String(entry.get("vehicle", ""))
	for stage: Dictionary in sim.body_stages(index):
		if String(stage["id"]) == String(entry["stage"]) and (vehicle_part.is_empty() or String(stage.get("vehicle_id", "")).contains(vehicle_part)):
			base_m = float(stage["base_station_m"])
			break
	var attitude := Basis(sim.body_attitude_gcrf(index) as Quaternion)
	var mount := _vec(entry.get("mount_m"))
	var look := _vec(entry.get("look")).normalized()
	var up := _vec(entry.get("up")).normalized()
	var right := look.cross(up).normalized()
	var camera_basis := Basis(right, right.cross(look), -look)
	var origin := (sim.body_position_gcrf_m(index) as Vector3) \
			+ attitude * (Vector3(base_m - float(sim.body_cg_station_m(index)), 0.0, 0.0) + mount)
	return Transform3D(attitude * camera_basis, origin)


static func _config(config: CockpitConfig) -> CockpitConfig:
	return config if config != null else CockpitConfig.for_vehicle(DEFAULT_COCKPIT)


static func _vec(value: Variant) -> Vector3:
	if value is Array and (value as Array).size() == 3:
		return Vector3(value[0], value[1], value[2])
	return Vector3.ZERO
