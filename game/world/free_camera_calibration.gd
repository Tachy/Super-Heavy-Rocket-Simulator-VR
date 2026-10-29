class_name FreeCameraCalibration
extends RefCounted
## Axis detection and normalisation for the free camera's setup wizard. During one wizard step the user pushes one
## control fully in the asked direction, then to the opposite end, then back to the middle; the axis with the largest
## excursion from its rest value is the one meant. Snapshots are {"device:axis": raw value}.
##
## A stored axis entry: {device_name, axis, center, raw_pos, raw_neg}: raw value at rest and at the end of the asked ("positive")
## direction and of the opposite one. normalize() maps the raw value to -1..+1 with the axis' own centre and own extremes.

const SAVE_PATH := "user://free_camera_axes.json"
## [E] An axis counts as moved from this excursion on (raw units, full range 2); settled = back within the tolerance of its
## rest value for SETTLE_S.
const MIN_EXCURSION := 0.4
const SETTLE_TOLERANCE := 0.15
const SETTLE_S := 0.6
## [E] The far end counts as reached from this excursion on; else the axis is taken as symmetric about its centre.
const OPPOSITE_MIN_EXCURSION := 0.2
const AXES_PER_DEVICE := 10

var _rest := {}
var _min := {}
var _max := {}
var _first_side := {}  ## key -> +1 / -1: raw side of the first excursion (the asked direction)
var _last := {}
var _excluded: Array = []
var _settled_s := 0.0


## Starts a measurement; `snapshot` is the rest state. `excluded`: keys already taken by other motions.
func begin(snapshot: Dictionary, excluded: Array = []) -> void:
	_rest = snapshot.duplicate()
	_min = snapshot.duplicate()
	_max = snapshot.duplicate()
	_last = snapshot.duplicate()
	_first_side = {}
	_excluded = excluded
	_settled_s = 0.0


func feed(snapshot: Dictionary, delta_s: float) -> void:
	for key: String in snapshot:
		if not _rest.has(key):
			continue
		var value: float = snapshot[key]
		_min[key] = minf(_min[key], value)
		_max[key] = maxf(_max[key], value)
		if not _first_side.has(key) and absf(value - _rest[key]) > MIN_EXCURSION * 0.5:
			_first_side[key] = 1.0 if value > _rest[key] else -1.0
	_last = snapshot
	var best := best_key()
	if best != "" and absf(float(_last.get(best, 0.0)) - float(_rest[best])) <= SETTLE_TOLERANCE:
		_settled_s += delta_s
	else:
		_settled_s = 0.0


## The axis with the largest excursion (at least MIN_EXCURSION), or "".
func best_key() -> String:
	var best := ""
	var best_excursion := MIN_EXCURSION
	for key: String in _rest:
		if _excluded.has(key):
			continue
		var excursion := excursion_of(key)
		if excursion > best_excursion:
			best_excursion = excursion
			best = key
	return best


func excursion_of(key: String) -> float:
	return maxf(float(_max[key]) - float(_rest[key]), float(_rest[key]) - float(_min[key])) if _rest.has(key) else 0.0


## Moved and back at rest: the step can end by itself.
func settled() -> bool:
	return best_key() != "" and _settled_s >= SETTLE_S


## {key, center, raw_pos, raw_neg} of the best axis ({} when none moved). raw_pos is the extreme on the side the user moved
## first (the asked direction), raw_neg the opposite extreme (mirrored about the centre when that side was not used).
func result() -> Dictionary:
	var key := best_key()
	if key == "":
		return {}
	var center: float = _rest[key]
	var side: float = _first_side.get(key, 1.0)
	var raw_pos: float = _max[key] if side > 0.0 else _min[key]
	var raw_neg: float = _min[key] if side > 0.0 else _max[key]
	if absf(raw_neg - center) < OPPOSITE_MIN_EXCURSION:
		raw_neg = 2.0 * center - raw_pos
	return {"key": key, "center": center, "raw_pos": raw_pos, "raw_neg": raw_neg}


## Raw axis value -> -1..+1: 0 at the centre, +1 at raw_pos, -1 at raw_neg, each side scaled by its own extent.
static func normalize(raw: float, entry: Dictionary) -> float:
	var center: float = entry["center"]
	var positive: float = float(entry["raw_pos"]) - center
	var negative: float = float(entry["raw_neg"]) - center
	var d := raw - center
	if is_zero_approx(positive) or is_zero_approx(negative):
		return 0.0
	var value := d / positive if d * positive >= 0.0 else -d / negative
	return clampf(value, -1.0, 1.0)


## Every connected joypad's first AXES_PER_DEVICE axes: {"device:axis": value}.
static func snapshot() -> Dictionary:
	var values := {}
	for device in Input.get_connected_joypads():
		for axis in AXES_PER_DEVICE:
			values["%d:%d" % [device, axis]] = Input.get_joy_axis(device, axis)
	return values


static func save(path: String, entries: Dictionary) -> bool:
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file == null:
		return false
	file.store_string(JSON.stringify(entries, "\t"))
	return true


static func load_file(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(path))
	return parsed if parsed is Dictionary else {}
