# Headless test of LocalTime (time zone grid + tzdata transitions) and the camera's local-time line.
# Needs data/timezones (python tools/data_pipeline/fetch.py timezones) and the SPICE kernels for the camera cases.
# Run: godot --headless --path game --script res://tests/local_time_test.gd
extends SceneTree

const STARBASE := Vector2(25.9972, -97.1561)
const CAPE := Vector2(28.6082, -80.6041)
const CHICAGO := Vector2(41.8781, -87.6298)
const NEW_DELHI := Vector2(28.6139, 77.2090)
const MID_PACIFIC := Vector2(10.4, -150.6)
const HOUR_S := 3600

var _failures := 0


func _initialize() -> void:
	_test_formatting()
	_test_missing_data()
	var lt := LocalTime.load_from(Paths.data_dir().path_join("timezones"))
	_check(lt.is_loaded(), "data/timezones loaded")
	if lt.is_loaded():
		_test_zones(lt)
		_test_camera(lt)
	print("LOCAL TIME TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _test_formatting() -> void:
	_check(LocalTime.format_offset(-5 * HOUR_S) == "UTC−5", "offset -5 h")
	_check(LocalTime.format_offset(19800) == "UTC+5:30", "offset +5:30")
	_check(LocalTime.format_offset(-12600) == "UTC−3:30", "offset -3:30")
	_check(LocalTime.format_offset(0) == "UTC+0", "offset 0")
	_check(LocalTime.unix_from_iso("2024-07-01T12:00:00.000") == 1719835200, "ISO -> Unix")
	_check(LocalTime.unix_from_iso("2016-12-31T23:59:60.500") == 1483228800, "leap second -> next midnight")
	_check(LocalTime.format_line({}, 0) == "", "no zone: no line")


func _test_missing_data() -> void:
	var empty := LocalTime.load_from("res://does_not_exist")
	_check(not empty.is_loaded(), "missing data: not loaded")
	_check(empty.lookup(STARBASE.x, STARBASE.y, 0).is_empty(), "missing data: empty lookup")
	_check(empty.camera_line(null, Vector3.ZERO, 0.0) == "", "missing data: no line")


func _test_zones(lt: LocalTime) -> void:
	var summer := LocalTime.unix_from_iso("2024-07-01T12:00:00")
	var starbase := lt.lookup(STARBASE.x, STARBASE.y, summer)
	_check(starbase.get("zone_id") == "America/Chicago", "Starbase zone (%s)" % starbase.get("zone_id"))
	var line := LocalTime.format_line(starbase, summer)
	_check(line == "Ortszeit 07:00:00 CDT (UTC−5)", "Starbase summer: %s" % line)
	var winter := LocalTime.unix_from_iso("2024-01-15T12:00:00")
	line = LocalTime.format_line(lt.lookup(STARBASE.x, STARBASE.y, winter), winter)
	_check(line == "Ortszeit 06:00:00 CST (UTC−6)", "Starbase winter: %s" % line)

	var cape := lt.lookup(CAPE.x, CAPE.y, summer)
	_check(cape.get("zone_id") == "America/New_York", "Cape zone (%s)" % cape.get("zone_id"))
	_check(LocalTime.format_line(cape, summer) == "Ortszeit 08:00:00 EDT (UTC−4)", "Cape summer EDT")
	_check(LocalTime.format_line(lt.lookup(CAPE.x, CAPE.y, winter), winter) == "Ortszeit 07:00:00 EST (UTC−5)",
			"Cape winter EST")

	# DST start in Chicago: 2024-03-10 02:00 CST = 08:00 UTC jumps to 03:00 CDT
	var change := LocalTime.unix_from_iso("2024-03-10T08:00:00")
	var before := lt.lookup(CHICAGO.x, CHICAGO.y, change - 1)
	var after := lt.lookup(CHICAGO.x, CHICAGO.y, change)
	_check(before.get("abbreviation") == "CST" and before.get("offset_s") == -6 * HOUR_S, "1 s before: CST")
	_check(after.get("abbreviation") == "CDT" and after.get("offset_s") == -5 * HOUR_S, "at the change: CDT")
	_check(LocalTime.format_line(before, change - 1) == "Ortszeit 01:59:59 CST (UTC−6)", "local 01:59:59 CST")
	_check(LocalTime.format_line(after, change) == "Ortszeit 03:00:00 CDT (UTC−5)", "local 03:00:00 CDT")

	var india := lt.lookup(NEW_DELHI.x, NEW_DELHI.y, summer)
	_check(india.get("zone_id") == "Asia/Kolkata", "India zone (%s)" % india.get("zone_id"))
	line = LocalTime.format_line(india, summer)
	_check(line == "Ortszeit 17:30:00 IST (UTC+5:30)", "India: %s" % line)

	var ocean := lt.lookup(MID_PACIFIC.x, MID_PACIFIC.y, summer)
	_check(ocean.get("zone_id") == "Etc/GMT+10", "mid-Pacific nautical zone (%s)" % ocean.get("zone_id"))
	line = LocalTime.format_line(ocean, summer)
	_check(line == "Ortszeit 02:00:00 (UTC−10)", "mid-Pacific: %s" % line)

	# Local date differs from the UTC date: shown as day and month before the clock
	var evening := LocalTime.unix_from_iso("2024-10-21T00:28:00")
	line = LocalTime.format_line(lt.lookup(STARBASE.x, STARBASE.y, evening), evening)
	_check(line == "Ortszeit 20.10. 19:28:00 CDT (UTC−5)", "Starbase evening, previous local day: %s" % line)
	var late := LocalTime.unix_from_iso("2024-07-01T20:00:00")
	line = LocalTime.format_line(lt.lookup(NEW_DELHI.x, NEW_DELHI.y, late), late)
	_check(line == "Ortszeit 02.07. 01:30:00 IST (UTC+5:30)", "India after local midnight, next local day: %s" % line)


func _test_camera(lt: LocalTime) -> void:
	var core := StarsimCore.new()
	if not core.open(Paths.data_dir().path_join("spice")):
		_check(false, "SPICE kernels for the camera cases")
		return
	var tdb := core.utc_to_tdb_s("2024-07-01T12:00:00")
	var ground := core.geodetic_to_gcrf_m(STARBASE.x, STARBASE.y, 2.0, tdb)
	var geodetic := core.gcrf_to_geodetic(ground, tdb)
	_check(absf(geodetic.x - STARBASE.x) < 1e-9 and absf(geodetic.y - STARBASE.y) < 1e-9
			and absf(geodetic.z - 2.0) < 1e-6, "gcrf_to_geodetic inverts geodetic_to_gcrf_m (%s)" % geodetic)
	var line := lt.camera_line(core, ground, tdb)
	_check(line == "Ortszeit 07:00:00 CDT (UTC−5)", "camera at Starbase: %s" % line)
	var below_karman := core.geodetic_to_gcrf_m(STARBASE.x, STARBASE.y, 99000.0, tdb)
	_check(lt.camera_line(core, below_karman, tdb) != "", "camera at 99 km: shown")
	var high := core.geodetic_to_gcrf_m(STARBASE.x, STARBASE.y, 150000.0, tdb)
	_check(lt.camera_line(core, high, tdb) == "", "camera at 150 km: hidden")
	var moon := core.body_position_gcrf_m("moon", tdb)
	var on_moon := moon + (Vector3.ZERO - moon).normalized() * 1737400.0
	_check(lt.camera_line(core, on_moon, tdb) == "", "camera on the Moon: hidden")


func _check(condition: bool, label: String) -> void:
	if condition:
		print("  ok   ", label)
	else:
		_failures += 1
		print("  FAIL ", label)
