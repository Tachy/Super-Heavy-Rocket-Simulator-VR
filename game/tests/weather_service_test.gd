# Headless test of WeatherService (Task 7): requests, builds, fallbacks, warp policy, switch, cache cap - with a fake
# fetcher (files from memory) and a fake builder (the core's slab snapshot), no network.
# Needs the SPICE kernels and the shipped data/weather/analog.
# Run: godot --headless --path game --script res://tests/weather_service_test.gd
extends SceneTree

const ARCHIVE_UTC := "2024-04-08T18:42:00"
const LATER_UTC := "2024-04-09T03:10:00"
const ZATTRS := '{"attributes": {"valid_time_stop": "2026-06-30", "valid_time_stop_era5t": "2026-10-19"}}'
const MAX_FRAME_S := 0.25  ## update() must return within this (it never waits for a download or build)

var _failures := 0
var _core := StarsimCore.new()
var _serial := 0
var _dirs: Array = []  ## the test directories, removed at the end


## The fetcher double: records requests, answers them on pump() (after request() returned, as the real one does).
class FakeFetcher:
	extends RefCounted
	signal finished(id: int, ok: bool, http_code: int, body: PackedByteArray)
	var requests: Array = []
	var fail_files := false  ## every file download answers 404
	var offline := false  ## every request, probes included, fails
	var truncate_files := false  ## every file download "succeeds" with one byte
	var _next := 1
	var _queue: Array = []

	func request(url: String, first_byte: int, last_byte: int, dest: String) -> int:
		var id := _next
		_next += 1
		requests.append({"url": url, "first": first_byte, "last": last_byte, "dest": dest})
		var answer := {"id": id, "ok": true, "code": 200, "body": PackedByteArray()}
		if offline:
			answer["ok"] = false
			answer["code"] = 0
		elif url.ends_with("/.zattrs"):
			answer["body"] = ZATTRS.to_utf8_buffer()
		elif url.contains("noaa-gfs"):
			answer["ok"] = false
			answer["code"] = 404
		elif fail_files:
			answer["ok"] = false
			answer["code"] = 404
		else:
			DirAccess.make_dir_recursive_absolute(dest.get_base_dir())
			var f := FileAccess.open(dest, FileAccess.WRITE)
			f.store_buffer(PackedByteArray([1]) if truncate_files else PackedByteArray([1, 2, 3, 4]))
			f.close()
		_queue.append(answer)
		return id

	func pump() -> void:
		var batch := _queue
		_queue = []
		for a: Dictionary in batch:
			finished.emit(a["id"], a["ok"], a["code"], a["body"])

	func file_requests() -> Array:
		return requests.filter(func(r: Dictionary) -> bool: return String(r["dest"]) != "")


func _initialize() -> void:
	var kernel_dir := Paths.data_dir().path_join("spice")
	_check(_core.open(kernel_dir), "SPICE kernels open")
	_test_settings()
	_test_archive_and_warp()
	_test_fallbacks()
	_test_disabled()
	_test_cache_cap()
	_test_bad_load_recovers()
	_test_no_data()
	_test_probe_retry()
	_test_core_cache_eviction()
	for dir: String in _dirs:
		_remove_dir(dir)
	DirAccess.remove_absolute(_settings_path())
	print("WEATHER SERVICE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


## The settings file of this test: never the user's real user://settings.cfg.
func _settings_path() -> String:
	return OS.get_user_data_dir().path_join("weather_test_%d_settings.cfg" % OS.get_process_id())


func _test_settings() -> void:
	var path := _settings_path()
	WeatherService.save_enabled_setting(false, path)
	_check(not WeatherService.load_enabled_setting(path), "setting persists: off")
	WeatherService.save_enabled_setting(true, path)
	_check(WeatherService.load_enabled_setting(path), "setting persists: on")


func _remove_dir(dir: String) -> void:
	for name: String in DirAccess.get_files_at(dir):
		DirAccess.remove_absolute(dir.path_join(name))
	for name: String in DirAccess.get_directories_at(dir):
		_remove_dir(dir.path_join(name))
	DirAccess.remove_absolute(dir)


func _make(fetcher: FakeFetcher, loader := Callable(), analog_dir := "") -> WeatherService:
	_serial += 1
	var dir := OS.get_user_data_dir().path_join("weather_test_%d_%d" % [OS.get_process_id(), _serial])
	DirAccess.make_dir_recursive_absolute(dir)
	_dirs.append(dir)
	var service := WeatherService.new()
	var builder := func(snapshot: Dictionary, raw_dir: String, out_file: String) -> Dictionary:
		var names := PackedStringArray()
		for req: Dictionary in _core.weather_requests(snapshot, ""):
			names.append(req["file_name"])
		for n in names:
			var f := FileAccess.open(raw_dir.path_join(n), FileAccess.READ)
			if f == null or f.get_length() < 2:
				return {"ok": false, "error": "raw file missing or truncated: " + n}
		var ok := _core.weather_write_slab_snapshot(out_file, 1000.0, 3000.0, 1.0e-4, 0.8)
		return {"ok": ok, "error": ""}
	service.setup(_core, Paths.config_dir(), dir.path_join("cache"),
			analog_dir if analog_dir != "" else Paths.data_dir().path_join("weather/analog"), fetcher, builder)
	if loader.is_valid():
		service._loader = loader
	return service


## Runs update() frames (with the fetcher answering) until cond is true; fails the check on timeout or a slow frame.
func _run_until(service: WeatherService, fetcher: FakeFetcher, utc: String, warp: float, cond: Callable,
		label: String, timeout_s := 60.0) -> bool:
	var epoch := _core.utc_to_tdb_s(utc)
	var start := Time.get_ticks_msec()
	var slowest := 0.0
	while (Time.get_ticks_msec() - start) / 1000.0 < timeout_s:
		var t0 := Time.get_ticks_usec()
		service.update(epoch, warp)
		slowest = maxf(slowest, (Time.get_ticks_usec() - t0) / 1.0e6)
		fetcher.pump()
		if cond.call():
			_check(slowest < MAX_FRAME_S, "%s: update() never waited (slowest %.0f ms)" % [label, slowest * 1000.0])
			return true
		OS.delay_msec(5)
	_check(false, "%s: timed out, status '%s'" % [label, service.status_text()])
	return false


func _test_archive_and_warp() -> void:
	var fetcher := FakeFetcher.new()
	var service := _make(fetcher)
	var changes: Array = []
	service.snapshots_changed.connect(func(b: Dictionary, a: Dictionary) -> void: changes.append([b, a]))
	var epoch := _core.utc_to_tdb_s(ARCHIVE_UTC)
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service.status_text().begins_with("Wetter: typisch"),
			"analog first")
	var bracket := _core.weather_bracket(_core.weather_utc_hour(epoch), 1, ZATTRS, -1, WeatherService.ANALOG_YEAR)
	_check(bracket["before"]["kind"] == "era5", "the archive epoch brackets ERA5 snapshots")
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service.status_text().begins_with("Wetter: ERA5"),
			"archive epoch")
	_check(service.status_text() == "Wetter: ERA5, 08.04.2024 18:00 UTC", "ERA5 line: " + service.status_text())
	_check(changes.size() >= 2 and not changes.back()[0].is_empty(), "snapshots_changed emitted with loaded snapshots")
	var kept_texels := 0
	for file: String in service._loaded:
		kept_texels += 1 if service._loaded[file].has("cloud_texels") else 0
	_check(kept_texels == 0, "no layer attached: the texels are dropped after the announcement (%d dictionaries keep them)" % kept_texels)
	var keys := [String(bracket["before"]["cache_key"]), String(bracket["after"]["cache_key"])]
	var static_count := 0
	var timed := 0
	for r: Dictionary in fetcher.file_requests():
		var name := String(r["dest"]).get_file()
		if name == "era5_geopotential_at_surface.blosc":
			static_count += 1
		elif name.begins_with(keys[0]) or name.begins_with(keys[1]):
			timed += 1
	_check(static_count == 1, "the static geopotential file is fetched once (%d)" % static_count)
	_check(timed == 12, "6 timed files for each of the two bracketing hours (%d)" % timed)
	_check(service.blend() > 0.6 and service.blend() < 0.8, "blend of 18:42 is 0.7 (%.3f)" % service.blend())
	for key: String in keys:
		_check(FileAccess.file_exists(service._cache_dir.path_join(key + ".wthr")), "cache file of " + key)
	var leftovers := 0
	for name: String in DirAccess.get_files_at(service._raw_dir()):
		if name.begins_with(keys[0]) or name.begins_with(keys[1]):
			leftovers += 1
	_check(leftovers == 0, "raw files of the built hours deleted after the builds (%d left)" % leftovers)

	# Warp: no new requests, "Stand".
	service._seconds_per_download = 20.0
	var count := fetcher.requests.size()
	_run_until(service, fetcher, LATER_UTC, 1.0e7, func() -> bool: return service.status_text().begins_with("Wetter: Stand"),
			"warp hold")
	for i in range(30):
		service.update(_core.utc_to_tdb_s(LATER_UTC), 1.0e7)
		fetcher.pump()
	_check(fetcher.requests.size() == count, "warp 1e7: no new requests")
	_check(service.status_text() == "Wetter: Stand 18:00 UTC", "hold line: " + service.status_text())
	# Warp back to 1: the current bracket (09.04. 03/04 UTC) is requested and blended in.
	_run_until(service, fetcher, LATER_UTC, 1.0, func() -> bool: return service.status_text() == "Wetter: ERA5, 09.04.2024 03:00 UTC",
			"warp back")
	_check(fetcher.requests.size() > count, "warp back to 1: the current bracket is requested")
	_check(service.blend() > 0.0 and service.blend() < 0.3, "blended in at 03:10 (%.3f)" % service.blend())
	service.shutdown()


func _test_fallbacks() -> void:
	var fetcher := FakeFetcher.new()
	fetcher.fail_files = true
	var service := _make(fetcher)
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service.status_text().begins_with("Wetter: typisch"),
			"404")
	for i in range(20):
		service.update(_core.utc_to_tdb_s(ARCHIVE_UTC), 1.0)
		fetcher.pump()
		OS.delay_msec(5)
	_check(service.status_text() == "Wetter: typisch (Analogtag 15.04.2025)", "404: analog line: " + service.status_text())
	_check(_count_state(service, "failed") >= 1, "the failed snapshot is recorded")
	service.shutdown()

	var truncated := FakeFetcher.new()
	truncated.truncate_files = true
	var service2 := _make(truncated)
	_run_until(service2, truncated, ARCHIVE_UTC, 1.0, func() -> bool: return _count_state(service2, "failed") >= 1,
			"truncated")
	# The analog's load runs on a worker and may still be running when the job fails (Task 8: the load also builds the
	# GPU form): wait for its line instead of reading it in the same frame.
	_run_until(service2, truncated, ARCHIVE_UTC, 1.0,
			func() -> bool: return service2.status_text().begins_with("Wetter: typisch"), "truncated: analog line", 10.0)
	for key: String in service2._jobs:
		_check(not FileAccess.file_exists(service2._cache_dir.path_join(key + ".wthr")), "no cache file of a damaged build")
	service2.shutdown()


func _count_state(service: WeatherService, state: String) -> int:
	var n := 0
	for key: String in service._jobs:
		if service._jobs[key]["state"] == state:
			n += 1
	return n


func _test_disabled() -> void:
	var fetcher := FakeFetcher.new()
	var service := _make(fetcher)
	var cleared: Array = []
	service.snapshots_changed.connect(func(b: Dictionary, a: Dictionary) -> void: cleared.append(b.is_empty() and a.is_empty()))
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service.status_text().begins_with("Wetter: typisch"),
			"before switch off")
	var count := fetcher.requests.size()
	service.enabled = false
	service.update(_core.utc_to_tdb_s(ARCHIVE_UTC), 1.0)
	_check(service.status_text() == "Wetter: aus", "off line")
	_check(cleared.size() > 0 and cleared.back(), "textures cleared on switch off")
	for i in range(10):
		service.update(_core.utc_to_tdb_s(ARCHIVE_UTC), 1.0)
		fetcher.pump()
	_check(fetcher.requests.size() == count, "off: no requests")
	service.shutdown()


func _test_cache_cap() -> void:
	var fetcher := FakeFetcher.new()
	var service := _make(fetcher)
	DirAccess.make_dir_recursive_absolute(service._cache_dir)
	var names := ["a.wthr", "b.wthr", "c.wthr"]
	for n: String in names:
		var f := FileAccess.open(service._cache_dir.path_join(n), FileAccess.WRITE)
		f.store_buffer(PackedByteArray([0]).duplicate())
		f.store_buffer(PackedByteArray([0, 0, 0]))
		for i in range(1000):
			f.store_8(0)
		f.close()
		OS.delay_msec(1100)  # file times have 1 s resolution
	service.cache_cap_bytes = 2500
	service.enforce_cache_cap()
	_check(not FileAccess.file_exists(service._cache_dir.path_join("a.wthr")), "LRU: the oldest file is deleted")
	_check(FileAccess.file_exists(service._cache_dir.path_join("b.wthr"))
			and FileAccess.file_exists(service._cache_dir.path_join("c.wthr")), "LRU: the newer files stay")
	service.shutdown()


## A snapshot file that fails to load once is deleted and rebuilt after the retry, and the rebuilt file is kept.
func _test_bad_load_recovers() -> void:
	var fetcher := FakeFetcher.new()
	var seen := {}
	var failures := [0]
	var loader := func(file: String) -> Dictionary:
		if file.contains("weather_test_") and not seen.has(file):
			seen[file] = true
			failures[0] += 1
			return {"ok": false}
		return _core.weather_load_snapshot(file)
	var service := _make(fetcher, loader)
	service.retry_after_s = 0.3
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service.status_text().begins_with("Wetter: ERA5"),
			"bad load: recovered to ERA5")
	_check(failures[0] >= 1, "bad load: a load was made to fail (%d)" % failures[0])
	for i in range(60):  # the old bug deleted the rebuilt good file again and again
		service.update(_core.utc_to_tdb_s(ARCHIVE_UTC), 1.0)
		fetcher.pump()
		OS.delay_msec(10)
	_check(service.status_text().begins_with("Wetter: ERA5"), "bad load: stays on ERA5: " + service.status_text())
	_check(service._bad_files.is_empty(), "bad load: no stale bad-file entry")
	var kept := 0
	for name: String in DirAccess.get_files_at(service._cache_dir):
		if name.ends_with(".wthr"):
			kept += 1
	_check(kept >= 2, "bad load: the rebuilt cache files are kept (%d)" % kept)
	service.shutdown()


## Nothing shipped and nothing fetchable: the line ends in "aus (keine Daten)".
func _test_no_data() -> void:
	var fetcher := FakeFetcher.new()
	fetcher.fail_files = true
	var empty := OS.get_user_data_dir().path_join("weather_test_%d_empty_analog" % OS.get_process_id())
	DirAccess.make_dir_recursive_absolute(empty)
	_dirs.append(empty)
	var service := _make(fetcher, Callable(), empty)
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0,
			func() -> bool: return service.status_text() == "Wetter: aus (keine Daten)", "no data")
	service.shutdown()


## A session started offline gets ERA5 once the line is back (the probes are repeated).
func _test_probe_retry() -> void:
	var fetcher := FakeFetcher.new()
	fetcher.offline = true
	var service := _make(fetcher)
	service.retry_after_s = 0.3
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service.settled(), "offline settled")
	_check(service.status_text().begins_with("Wetter: typisch"), "offline: analog line: " + service.status_text())
	fetcher.offline = false
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service.status_text().begins_with("Wetter: ERA5"),
			"back online: ERA5")
	# Switched off during a probe, back on later: probing resumes.
	var service2 := _make(fetcher)
	service2.retry_after_s = 0.3
	fetcher.offline = true
	service2.update(_core.utc_to_tdb_s(ARCHIVE_UTC), 1.0)
	service2.enabled = false
	for i in range(5):
		service2.update(_core.utc_to_tdb_s(ARCHIVE_UTC), 1.0)
		fetcher.pump()
	fetcher.offline = false
	service2.enabled = true
	_run_until(service2, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service2.status_text().begins_with("Wetter: ERA5"),
			"off during a probe, then on: ERA5")
	service.shutdown()
	service2.shutdown()


## Final review I2: the core's active pair is cache-only. A file that fell out of the core's load cache is not read on the
## main thread: the push waits (the active pair stays), a worker loads it again, then the push goes through.
func _test_core_cache_eviction() -> void:
	var fetcher := FakeFetcher.new()
	var service := _make(fetcher)
	_run_until(service, fetcher, ARCHIVE_UTC, 1.0, func() -> bool: return service.status_text().begins_with("Wetter: ERA5"),
			"eviction: ERA5 active")
	var files: Array = service._active["files"]
	var spares: Array = []
	for i in range(3):  # three other files push the active pair out of the core's 3-entry cache
		var spare := service._cache_dir.path_join("spare_%d.wthr" % i)
		_core.weather_write_slab_snapshot(spare, 1000.0, 2000.0, 1e-4, 0.5)
		_core.weather_load_snapshot(spare)
		spares.append(spare)
	var cached: PackedStringArray = _core.weather_cached_files()
	_check(not (files[0] in cached) and not (files[1] in cached), "eviction: the active pair is out of the core's cache")
	var refused: Dictionary = _core.weather_set_active(files[0], files[1], 0.5)
	_check(not refused.get("ok", true), "eviction: weather_set_active refuses instead of reading")
	var pushed_before: float = service._pushed_blend
	# 18:55: the blend moves past BLEND_PUSH_STEP; the push must neither read nor stall, and succeeds after the reload.
	var epoch := _core.utc_to_tdb_s("2024-04-08T18:55:00")
	service.update(epoch, 1.0)
	_check(service._pushed_blend == pushed_before, "eviction: the first push after the eviction did not read on the main thread")
	var start := Time.get_ticks_msec()
	var slowest := 0.0
	while service._pushed_blend == pushed_before and Time.get_ticks_msec() - start < 20000:
		var t0 := Time.get_ticks_usec()
		service.update(epoch, 1.0)
		slowest = maxf(slowest, (Time.get_ticks_usec() - t0) / 1.0e6)
		OS.delay_msec(5)
	_check(service._pushed_blend != pushed_before and absf(service._pushed_blend - service.blend()) < 1e-9,
			"eviction: a worker reload brings the push through (blend %.3f)" % service._pushed_blend)
	_check(slowest < MAX_FRAME_S, "eviction: update() never waited (slowest %.0f ms)" % [slowest * 1000.0])
	# Weather off: results of loads still running are dropped, nothing refills the loaded dictionaries.
	service.enabled = false
	for i in range(30):
		service.update(epoch, 1.0)
		OS.delay_msec(5)
	_check(service._loaded.is_empty(), "off: no loaded snapshot dictionaries kept")
	service.shutdown()


func _check(condition: bool, message: String) -> void:
	if condition:
		print("ok   ", message)
	else:
		print("FAIL ", message)
		_failures += 1
