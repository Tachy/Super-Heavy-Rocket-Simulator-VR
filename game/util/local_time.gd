class_name LocalTime
extends RefCounted
## Local civil time at a position on the Earth: the time zone from data/timezones/zone_grid.bin (timezone-boundary-
## builder with oceans, 0.02 deg cells, the zone of the cell centre: up to ~1.1 km off near a border) and its UTC offset
## and abbreviation from data/timezones/zones.json (IANA tzdata, every transition 1990-2060 exact to the second).
## Both files come from tools/data_pipeline/timezones.py; without them `is_loaded()` is false and every lookup is {}.

## The local time is shown while the camera is at most this high above the WGS84 ellipsoid: the Kármán line (100 km),
## the conventional edge of space. Above it (in orbit, at the Moon) a local time zone means nothing to the viewer.
const MAX_HEIGHT_M := 100000.0
const GRID_MAGIC := "STZG"
const GRID_VERSION := 1
const HEADER_BYTES := 40
const RUN_BYTES := 4  ## u16 first column + u16 zone index
const MINUS := "−"  ## U+2212, the typographic minus of the offset

static var _shared: LocalTime = null

var _columns := 0
var _rows := 0
var _west_deg := 0.0
var _north_deg := 0.0
var _step_deg := 0.0
var _row_start := PackedByteArray()  ## u32[rows + 1]: each row's first run
var _runs := PackedByteArray()
var _zone_ids := PackedStringArray()
var _transitions: Array = []  ## per zone: {"t": PackedInt64Array, "offset": PackedInt32Array, "abbr": PackedStringArray}
var _loaded := false


## The instance of data/timezones (loaded on first use).
static func shared() -> LocalTime:
	if _shared == null:
		_shared = LocalTime.load_from(Paths.data_dir().path_join("timezones"))
	return _shared


static func load_from(dir: String) -> LocalTime:
	var lt := LocalTime.new()
	lt._loaded = lt._load_grid(dir.path_join("zone_grid.bin")) and lt._load_zones(dir.path_join("zones.json"))
	return lt


func is_loaded() -> bool:
	return _loaded


## {zone_id, offset_s, abbreviation} at a geodetic position (degrees) and a UTC instant (Unix seconds), or {}
## without data.
func lookup(latitude_deg: float, longitude_deg: float, unix_s: int) -> Dictionary:
	if not _loaded:
		return {}
	var zone := zone_index(latitude_deg, longitude_deg)
	if zone >= _zone_ids.size():  # a grid from another build than the table
		return {}
	var table: Dictionary = _transitions[zone]
	var times: PackedInt64Array = table["t"]
	var i := maxi(times.bsearch(unix_s, false) - 1, 0)  # the last transition at or before unix_s
	return {"zone_id": _zone_ids[zone], "offset_s": table["offset"][i], "abbreviation": table["abbr"][i]}


func zone_index(latitude_deg: float, longitude_deg: float) -> int:
	var row := clampi(int(floor((_north_deg - latitude_deg) / _step_deg)), 0, _rows - 1)
	var column := posmod(int(floor((longitude_deg - _west_deg) / _step_deg)), _columns)
	var lo := _row_start.decode_u32(row * 4)
	var hi := _row_start.decode_u32(row * 4 + 4) - 1
	while lo < hi:  # the last run whose first column <= column
		var mid := (lo + hi + 1) >> 1
		if _runs.decode_u16(mid * RUN_BYTES) <= column:
			lo = mid
		else:
			hi = mid - 1
	return _runs.decode_u16(lo * RUN_BYTES + 2)


## The local-time line for a camera at `position_gcrf_m` (Earth-centred GCRF) at `tdb_s`, or "" when the camera is
## higher than MAX_HEIGHT_M above the Earth or there is no data.
func camera_line(core: StarsimCore, position_gcrf_m: Vector3, tdb_s: float) -> String:
	if not _loaded:
		return ""
	var geodetic := core.gcrf_to_geodetic(position_gcrf_m, tdb_s)
	if geodetic.z > MAX_HEIGHT_M:
		return ""
	var unix_s := LocalTime.unix_from_iso(core.tdb_s_to_utc(tdb_s))
	return format_line(lookup(geodetic.x, geodetic.y, unix_s), unix_s)


## Unix seconds from the core's UTC ISO string (fraction dropped; a leap second 23:59:60 reads as the next 00:00:00).
static func unix_from_iso(utc_iso: String) -> int:
	var text := utc_iso.substr(0, 19)
	var leap := text.ends_with(":60")
	if leap:
		text = text.substr(0, 17) + "59"
	return Time.get_unix_time_from_datetime_string(text) + (1 if leap else 0)


## "Ortszeit 07:00:00 CDT (UTC−5)", without an abbreviation "Ortszeit 17:30:00 (UTC+5:30)"; "" for {}. When the local
## date differs from the UTC date (evenings west of Greenwich, mornings east of it) the local day and month lead the
## clock: "Ortszeit 20.10. 19:28:00 CDT (UTC−5)" under "2024-10-21 00:28:00 UTC".
static func format_line(zone: Dictionary, unix_s: int) -> String:
	if zone.is_empty():
		return ""
	var offset_s: int = zone["offset_s"]
	var clock := Time.get_time_string_from_unix_time(unix_s + offset_s)
	var local_date := Time.get_date_dict_from_unix_time(unix_s + offset_s)
	if Time.get_date_string_from_unix_time(unix_s + offset_s) != Time.get_date_string_from_unix_time(unix_s):
		clock = "%02d.%02d. %s" % [local_date["day"], local_date["month"], clock]
	var abbreviation: String = zone["abbreviation"]
	var utc := "(%s)" % format_offset(offset_s)
	return "Ortszeit %s %s" % [clock, utc if abbreviation == "" else "%s %s" % [abbreviation, utc]]


## "UTC−5", "UTC+5:30", "UTC+0".
static func format_offset(offset_s: int) -> String:
	var sign := MINUS if offset_s < 0 else "+"
	var minutes := absi(offset_s) / 60
	var text := "UTC%s%d" % [sign, minutes / 60]
	if minutes % 60 != 0:
		text += ":%02d" % (minutes % 60)
	return text


func _load_grid(path: String) -> bool:
	if not FileAccess.file_exists(path):
		return false
	var data := FileAccess.get_file_as_bytes(path)
	if data.size() < HEADER_BYTES or data.slice(0, 4).get_string_from_ascii() != GRID_MAGIC \
			or data.decode_u32(4) != GRID_VERSION:
		push_warning("LocalTime: %s is not a version-%d zone grid" % [path, GRID_VERSION])
		return false
	_columns = data.decode_u32(8)
	_rows = data.decode_u32(12)
	_west_deg = data.decode_double(16)
	_north_deg = data.decode_double(24)
	_step_deg = data.decode_double(32)
	var runs_at := HEADER_BYTES + 4 * (_rows + 1)
	_row_start = data.slice(HEADER_BYTES, runs_at)
	_runs = data.slice(runs_at)
	return _rows > 0 and _columns > 0 and _step_deg > 0.0 \
			and _runs.size() == RUN_BYTES * _row_start.decode_u32(4 * _rows)


func _load_zones(path: String) -> bool:
	if not FileAccess.file_exists(path):
		return false
	var table: Variant = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not table is Dictionary or not table.has("zones"):
		push_warning("LocalTime: cannot read %s" % path)
		return false
	for zone: Dictionary in table["zones"]:
		var times := PackedInt64Array()
		var offsets := PackedInt32Array()
		var abbreviations := PackedStringArray()
		for entry: Array in zone["transitions"]:
			times.append(int(entry[0]))
			offsets.append(int(entry[1]))
			abbreviations.append(String(entry[2]))
		_zone_ids.append(zone["id"])
		_transitions.append({"t": times, "offset": offsets, "abbr": abbreviations})
	return not _zone_ids.is_empty()
