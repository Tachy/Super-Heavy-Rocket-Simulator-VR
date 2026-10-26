class_name WeatherService
extends RefCounted
## The weather of the game's Earth (M8 weather and clouds, Task 7): which ERA5 / GFS / analog snapshots bracket the
## world clock, their downloads, the build and the cache on disk, the load into memory and the warp policy. No physics
## and no decoding here: the sources, requests, bracket, build and load all come from the core (StarsimCore.weather_*).
##
## Nothing blocks the frame: downloads are HTTPRequest nodes (WeatherFetcher), builds and loads run on the
## WorkerThreadPool, `update()` only starts and polls them. The flight never waits for weather: until the wanted
## snapshots are on disk (or when they cannot be fetched) the last pair stays, or the analog shipped in
## data/weather/analog (1 degree, one snapshot per month and 6 hours of the reference year) is used at once; online the
## full-resolution ERA5 hour of the same calendar day replaces it. A damaged file (failed build or load) is deleted and
## its snapshot falls back to the analog like a failed download.

## The active pair changed (loaded snapshot dictionaries of StarsimCore.weather_load_snapshot, texels included; both
## `{}` when the weather was switched off). The blend between them is `blend()`.
signal snapshots_changed(before: Dictionary, after: Dictionary)

## Hours between 1900-01-01 and 1970-01-01 (25567 days) [S: calendar]: wall clock -> the core's hour scale.
const HOURS_1900_TO_1970 := 613608.0
## [S: tools/data_pipeline weather_analog: config/weather index] the reference year of the analog days.
const ANALOG_YEAR := 2025
const ERA5_STORE_URL := "https://storage.googleapis.com/gcp-public-data-arco-era5/ar/full_37-1h-0p25deg-chunk-1.zarr-v3"
## The raw file shared by every ERA5 hour (not time dependent): kept when a build deletes its raw files.
const STATIC_RAW_FILES := ["era5_geopotential_at_surface.blosc"]
## [call: user] Size cap of the snapshot cache; the oldest built files go first.
const CACHE_CAP_BYTES := 4 * 1024 * 1024 * 1024
## [call] A failed snapshot is retried after this long (s); a missing GFS cycle or a dropped connection may come back.
const RETRY_AFTER_S := 60.0
## [call] Initial guess of the wall time one snapshot download + build takes (s), until the first is measured.
const INITIAL_SECONDS_PER_DOWNLOAD := 20.0
## [call] Smoothing of the measured download time (exponential average weight of the newest).
const DOWNLOAD_TIME_WEIGHT := 0.5
## [call] Concurrent HTTP requests of the running job.
const MAX_PARALLEL_REQUESTS := 4
## [call] GFS cycles probed on start (newest first) for the newest one on the server.
const GFS_PROBE_COUNT := 4
## [call] The blend is pushed to the core's CPU light when it moved this much.
const BLEND_PUSH_STEP := 0.01
## STARSIM_DEBUG_WEATHER=1 prints the size and time of every built snapshot.
const DEBUG_ENV := "STARSIM_DEBUG_WEATHER"
const SETTINGS_FILE := "user://settings.cfg"

## "Wetter" switch. Off: no requests, no snapshots, "Wetter: aus".
var enabled := true:
	set(value):
		if value == enabled:
			return
		enabled = value
		_enabled_changed = true
## True when a CloudLayer is the only writer of the core's active pair (it follows the visible cross-fade, so the CPU
## light never leads the GPU clouds): the service then neither sets the pair nor its blend. The world sets it; without a
## layer (headless tests) the service sets the pair itself at the switch and strips the texels after emitting.
var light_driven_by_layer := false
## The size cap of the snapshot cache (tests lower it).
var cache_cap_bytes: int = CACHE_CAP_BYTES
## Seconds until a failed snapshot or probe is tried again (tests shorten it).
var retry_after_s: float = RETRY_AFTER_S

var _core: StarsimCore
var _config_dir := ""
var _cache_dir := ""
var _analog_dir := ""
var _fetcher: Object
## Builder(snapshot, raw_dir, out_file) -> {ok, error}; the core's decoder unless a test injects one.
var _builder: Callable
## Loader(file) -> {ok, ...}; the core's snapshot loader unless a test injects one (runs on a worker thread).
var _loader: Callable
var _wall_hour_fn: Callable  ## () -> float, hours since 1900 of the wall clock (tests fix it)

var _debug := OS.get_environment(DEBUG_ENV) != ""
var _enabled_changed := false
var _analog_index: Array = []
var _seconds_per_download := INITIAL_SECONDS_PER_DOWNLOAD

# Source probes (retried every retry_after_s while unanswered: a session started offline gets ERA5 / GFS once online).
var _era5_zattrs := ""
var _era5_probing := false
var _era5_probe_at_s := -INF  ## when the last ERA5 probe ended
var _era5_tried := false  ## the first ERA5 probe has ended (answered or failed)
var _gfs_cycle_hour := -1
var _gfs_probing := false
var _gfs_probe_at_s := -INF
var _gfs_tried := false
var _gfs_candidates: PackedInt64Array = PackedInt64Array()

# Requests in flight: fetcher id -> Callable(ok, code, body).
var _pending := {}

# Jobs: key -> {snapshot, state ("queued" | "running" | "done" | "failed"), failed_at_s}.
var _jobs := {}
var _queue: Array = []  ## keys, wanted ones in front
var _current := {}  ## the running job's state (see _start_job)

# Loaded / active snapshots.
var _loaded := {}  ## file -> loaded dictionary (texels included)
var _loading := {}  ## file -> {task, result}
var _bad_files := {}  ## file -> true: failed to load, not tried again (cleared when the file is deleted for a rebuild)

var _active := {}  ## {files: [before, after], metas: [snapshot dict, snapshot dict], kind: String}
var _target := {}  ## the pair that should become active once loaded (same layout)
var _blend := 0.0
var _target_blend := 0.0
var _pushed_blend := -1.0
var _held := false
var _last_bracket_hour := -1.0
var _wanted_keys: Array = []


## The persisted "Wetter" switch (user://settings.cfg [weather] enabled, default on).
static func load_enabled_setting(path := SETTINGS_FILE) -> bool:
	var config := ConfigFile.new()
	if config.load(path) != OK:
		return true
	return bool(config.get_value("weather", "enabled", true))


static func save_enabled_setting(value: bool, path := SETTINGS_FILE) -> void:
	var config := ConfigFile.new()
	config.load(path)  # keeps other settings; a missing file starts empty
	config.set_value("weather", "enabled", value)
	config.save(path)


## fetcher: a WeatherFetcher (added to the tree by the owner) or a test double with `request(url, first, last, dest)`
## and the signal `finished(id, ok, code, body)`. builder: optional Callable(snapshot, raw_dir, out_file) -> Dictionary.
func setup(core: StarsimCore, config_dir: String, cache_dir: String, analog_dir: String, fetcher: Object,
		builder: Callable = Callable()) -> void:
	_core = core
	_config_dir = config_dir
	_cache_dir = cache_dir
	_analog_dir = analog_dir
	_fetcher = fetcher
	_builder = builder if builder.is_valid() else Callable(self, "_core_build")
	_loader = Callable(_core, "weather_load_snapshot")
	_wall_hour_fn = func() -> float: return Time.get_unix_time_from_system() / 3600.0 + HOURS_1900_TO_1970
	_fetcher.finished.connect(_on_fetch_finished)
	DirAccess.make_dir_recursive_absolute(_raw_dir())
	_analog_index = _read_analog_index()


## Checks only: a fixed overlay line while a pinned snapshot is shown (CloudLayer.show_now); "" = the service's line.
var pinned_label := ""


func blend() -> float:
	return _blend


## The German source line of the overlay.
func status_text() -> String:
	if pinned_label != "":
		return pinned_label
	if not enabled:
		return "Wetter: aus"
	if _active.is_empty():
		return "Wetter: aus (keine Daten)" if _no_data() else "Wetter: wird geladen ..."
	if _held:
		var meta: Dictionary = _active["metas"][0]
		return "Wetter: Stand %02d:00 UTC" % posmod(int(meta["valid_hour"]), 24)
	var before: Dictionary = _active["metas"][0]
	match String(_active["kind"]):
		"era5":
			return "Wetter: ERA5, %s %02d:00 UTC" % [_date_text(int(before["valid_hour"]), true),
					posmod(int(before["valid_hour"]), 24)]
		"gfs":
			var cycle := int(before["cycle_hour"])
			var day := _date_text(cycle, false)
			return "Wetter: GFS-Vorhersage vom %s %02d UTC, +%d h" % [day, posmod(cycle, 24), int(before["lead_h"])]
		_:
			return "Wetter: typisch (Analogtag %s)" % _date_text(int(before.get("analog_hour", before["valid_hour"])), true)


## The overlay line of one loaded snapshot (StarsimCore.weather_load_snapshot): checks that pin a file (render_check).
static func snapshot_label(snapshot: Dictionary) -> String:
	match String(snapshot.get("kind", "")):
		"era5":
			var hour := int(snapshot["valid_hour"])
			return "Wetter: ERA5, %s %02d:00 UTC" % [_date_text(hour, true), posmod(hour, 24)]
		"gfs":
			var cycle := int(snapshot["cycle_hour"])
			return "Wetter: GFS-Vorhersage vom %s %02d UTC, +%d h" % [_date_text(cycle, false), posmod(cycle, 24),
					int(snapshot["lead_h"])]
		"analog":
			return "Wetter: typisch (Analogtag %s)" % _date_text(int(snapshot.get("analog_hour",
					snapshot["valid_hour"])), true)
	return "Wetter: Testschicht"


## Nothing is active and nothing can come: no usable analog file and no wanted snapshot still downloading or building.
func _no_data() -> bool:
	if not _active.is_empty() or not _target.is_empty() or not _loading.is_empty() or _last_bracket_hour < 0.0:
		return false
	var entry := _analog_entry(_last_bracket_hour)
	if not entry.is_empty() and not _bad_files.has(_analog_dir.path_join(String(entry["file"]))):
		return false
	for key: String in _wanted_keys:
		if _jobs.get(key, {}).get("state", "") in ["queued", "running"]:
			return false
	return true


## CloudLayer.core_cache_missed: a resident snapshot fell out of the core's load cache; load it again on a worker
## (the cache-only weather_set_active never reads on the main thread).
func reload_into_core_cache(file: String) -> void:
	if enabled:
		_start_load(file)


## True while a download, build or load is running (tests, UI).
func busy() -> bool:
	return not _current.is_empty() or not _queue.is_empty() or not _loading.is_empty() or not _pending.is_empty()


## True when the wanted snapshots are in (or failed), loaded and active and the source probes are answered: the source
## line is final for this instant (render checks wait for it; prefetches of later hours do not count).
func settled() -> bool:
	if not enabled:
		return true
	if not (_era5_tried and _gfs_tried) or not _loading.is_empty() or not _target.is_empty() or _active.is_empty():
		return false
	for key: String in _wanted_keys:
		if _jobs.get(key, {}).get("state", "") in ["queued", "running"]:
			return false
	return true


## Per frame, main thread: never waits. epoch_tdb_s: the world clock; warp_factor: its time warp (negative allowed).
func update(epoch_tdb_s: float, warp_factor: float) -> void:
	if _core == null:
		return
	_poll_loads()
	if _enabled_changed:
		_enabled_changed = false
		_apply_enabled()
	if not enabled:
		return
	_update_probes()
	_poll_build()
	_retry_failed()
	_evaluate(_core.weather_utc_hour(epoch_tdb_s), warp_factor)
	if not _held:
		if not _current.is_empty() and not _current.has("build_task") and int(_current["inflight"]) == 0 \
				and not _current["todo"].is_empty():
			_issue_requests()
		_pump_queue()
	_switch_if_ready()
	_push_blend()


## Waits for running worker tasks (the owner is going away).
func shutdown() -> void:
	for file: String in _loading:
		WorkerThreadPool.wait_for_task_completion(_loading[file]["task"])
	_loading.clear()
	if _current.has("build_task"):
		WorkerThreadPool.wait_for_task_completion(_current["build_task"])
		_current = {}


# ---------------------------------------------------------------------------------------------------------------------
# Probes: the ERA5 era ends (.zattrs) and the newest GFS cycle on the server.

func _update_probes() -> void:
	var now := Time.get_ticks_msec() / 1000.0
	if _era5_zattrs == "" and not _era5_probing and now - _era5_probe_at_s >= retry_after_s:
		_era5_probing = true
		_send(ERA5_STORE_URL + "/.zattrs", -1, -1, "", func(ok: bool, _code: int, body: PackedByteArray) -> void:
			_era5_probing = false
			_era5_tried = true
			_era5_probe_at_s = Time.get_ticks_msec() / 1000.0
			if ok:
				_era5_zattrs = body.get_string_from_utf8())
	if _gfs_cycle_hour < 0 and not _gfs_probing and now - _gfs_probe_at_s >= retry_after_s:
		_gfs_probing = true
		_gfs_candidates = _core.weather_gfs_cycle_candidates(float(_wall_hour_fn.call()), GFS_PROBE_COUNT)
		_probe_gfs(0)


func _end_gfs_probe() -> void:
	_gfs_probing = false
	_gfs_tried = true
	_gfs_probe_at_s = Time.get_ticks_msec() / 1000.0


func _probe_gfs(index: int) -> void:
	if index >= _gfs_candidates.size() or not enabled:
		_end_gfs_probe()  # no cycle found (or the weather was switched off): tried again later
		return
	var cycle: int = _gfs_candidates[index]
	var url := _core.weather_gfs_index_url({"kind": "gfs", "valid_hour": cycle, "cycle_hour": cycle, "lead_h": 0,
			"analog_hour": 0, "cache_key": ""})
	if url == "":
		_probe_gfs(index + 1)
		return
	_send(url, -1, -1, "", func(ok: bool, _code: int, _body: PackedByteArray) -> void:
		if ok:
			_gfs_cycle_hour = cycle
			_end_gfs_probe()
		else:
			_probe_gfs(index + 1))


# ---------------------------------------------------------------------------------------------------------------------
# Which snapshots are wanted.

func _evaluate(utc_hour: float, warp_factor: float) -> void:
	_last_bracket_hour = utc_hour
	var step := int(_core.weather_snapshot_step(absf(warp_factor), _seconds_per_download))
	_held = step == 0
	if _held:
		_ensure_baseline(utc_hour)
		return
	var bracket: Dictionary = _core.weather_bracket(utc_hour, step, _era5_zattrs, _gfs_cycle_hour, ANALOG_YEAR)
	if not bracket.get("ok", false):
		_wanted_keys = []
		_ensure_baseline(utc_hour)
		return
	var before: Dictionary = bracket["before"]
	var after: Dictionary = bracket["after"]
	var keys: Array = [String(before["cache_key"]), String(after["cache_key"])]
	var files: Array = [_cache_file(before), _cache_file(after)]
	var any_failed := false
	var all_ready := true
	for i in range(2):
		if FileAccess.file_exists(files[i]):
			continue
		all_ready = false
		var state: String = _jobs.get(keys[i], {}).get("state", "")
		if state == "failed":
			any_failed = true
		elif state != "running":
			_enqueue([before, after][i], true)
	if keys != _wanted_keys:
		_wanted_keys = keys
		var keep := keys.duplicate()
		keep.append_array(_enqueue_prefetch(utc_hour, step))
		_prune_queue(keep)  # a time jump leaves no stale downloads behind
	if all_ready:
		if _active.get("files", []) == files:
			_blend = float(bracket["blend"])  # the same pair: only the blend moves with the clock
			_target = {}
		else:
			_target = {"files": files, "metas": [before, after], "kind": String(before["kind"])}
			_target_blend = float(bracket["blend"])
	elif any_failed or _active.is_empty():
		_ensure_baseline(utc_hour)


## The shipped analog pair as the target while nothing else is active or the wanted snapshots failed.
func _ensure_baseline(utc_hour: float) -> void:
	var failed := false
	for key: String in _wanted_keys:
		if _jobs.get(key, {}).get("state", "") == "failed":
			failed = true
	if not _active.is_empty() and not failed:
		return
	var entry := _analog_entry(utc_hour)
	if entry.is_empty():
		return
	var file := _analog_dir.path_join(String(entry["file"]))
	var meta := {"kind": "analog", "valid_hour": int(String(entry["key"]).substr(5)), "cycle_hour": 0, "lead_h": 0,
			"analog_hour": int(String(entry["key"]).substr(5)), "cache_key": String(entry["key"])}
	if _active.get("files", []) == [file, file]:
		_target = {}
		return
	_target = {"files": [file, file], "metas": [meta, meta], "kind": "analog"}
	_target_blend = 0.0


## The shipped analog file for the UTC hour (hours since 1900), "" if there is none (checks: render_check
## --weather=analog).
func analog_file(utc_hour: float) -> String:
	var entry := _analog_entry(utc_hour)
	return "" if entry.is_empty() else _analog_dir.path_join(String(entry["file"]))


## The shipped snapshot of the month and nearest 6-hour slot of the UTC hour.
func _analog_entry(utc_hour: float) -> Dictionary:
	var unix := int((utc_hour - HOURS_1900_TO_1970) * 3600.0)
	var dt := Time.get_datetime_dict_from_unix_time(unix)
	var slot: int = ((int(dt["hour"]) + 3) / 6) % 4 * 6
	for entry: Dictionary in _analog_index:
		if int(entry["month"]) == int(dt["month"]) and int(entry["hour_utc"]) == slot:
			return entry
	return {}


func _read_analog_index() -> Array:
	var f := FileAccess.open(_analog_dir.path_join("index.json"), FileAccess.READ)
	if f == null:
		return []
	var parsed: Variant = JSON.parse_string(f.get_as_text())
	f.close()
	return parsed.get("entries", []) if parsed is Dictionary else []


## Queues the next bracket for a background download; returns its keys.
func _enqueue_prefetch(utc_hour: float, step: int) -> Array:
	var next: Dictionary = _core.weather_bracket(utc_hour + float(step), step, _era5_zattrs, _gfs_cycle_hour, ANALOG_YEAR)
	var keys: Array = []
	if not next.get("ok", false):
		return keys
	for name: String in ["before", "after"]:
		var snapshot: Dictionary = next[name]
		var key := String(snapshot["cache_key"])
		keys.append(key)
		if not FileAccess.file_exists(_cache_file(snapshot)) and not _jobs.has(key):
			_enqueue(snapshot, false)
	return keys


func _prune_queue(keep: Array) -> void:
	for key: String in _queue.duplicate():
		if not (key in keep):
			_queue.erase(key)
			_jobs.erase(key)


func _enqueue(snapshot: Dictionary, urgent: bool) -> void:
	var key := String(snapshot["cache_key"])
	if _jobs.has(key) and _jobs[key]["state"] in ["queued", "running"]:
		if urgent and key in _queue:  # a wanted snapshot jumps the prefetches
			_queue.erase(key)
			_queue.push_front(key)
		return
	_jobs[key] = {"snapshot": snapshot, "state": "queued", "failed_at_s": 0.0}
	if urgent:
		_queue.push_front(key)
	else:
		_queue.append(key)


func _retry_failed() -> void:
	var now := Time.get_ticks_msec() / 1000.0
	for key: String in _jobs:
		var job: Dictionary = _jobs[key]
		if job["state"] == "failed" and now - float(job["failed_at_s"]) >= retry_after_s:
			_jobs.erase(key)
			return


# ---------------------------------------------------------------------------------------------------------------------
# One job at a time: requests -> build -> cache file.

func _pump_queue() -> void:
	if not _current.is_empty() or _queue.is_empty():
		return
	var key: String = _queue.pop_front()
	var job: Dictionary = _jobs[key]
	job["state"] = "running"
	_current = {"key": key, "snapshot": job["snapshot"], "todo": [], "inflight": 0, "failed": false, "started_s":
			Time.get_ticks_msec() / 1000.0, "raw_files": []}
	var snapshot: Dictionary = job["snapshot"]
	if String(snapshot["kind"]) == "gfs":  # the index first: the records' byte ranges
		_send(_core.weather_gfs_index_url(snapshot), -1, -1, "", func(ok: bool, _code: int, body: PackedByteArray) -> void:
			if not ok:
				_job_failed()
			else:
				_plan_requests(body.get_string_from_utf8()))
	else:
		_plan_requests("")


func _plan_requests(idx_text: String) -> void:
	var requests: Array = _core.weather_requests(_current["snapshot"], idx_text)
	if requests.is_empty():
		_job_failed()
		return
	_current["todo"] = requests
	_issue_requests()


func _issue_requests() -> void:
	# Warp hold / weather off: the running job issues no further requests (it resumes in update()).
	while enabled and not _held and not _current.is_empty() and not _current["failed"] and not _current["todo"].is_empty() \
			and _current["inflight"] < MAX_PARALLEL_REQUESTS:
		var req: Dictionary = _current["todo"].pop_front()
		var dest := _raw_dir().path_join(String(req["file_name"]))
		if not (String(req["file_name"]) in STATIC_RAW_FILES):
			_current["raw_files"].append(dest)
		if FileAccess.file_exists(dest):  # the static file, or a leftover of an interrupted run
			continue
		_current["inflight"] += 1
		var job_key: String = _current["key"]
		_send(String(req["url"]), int(req["first_byte"]), int(req["last_byte"]), dest,
				func(ok: bool, _code: int, _body: PackedByteArray) -> void:
					if _current.is_empty() or _current["key"] != job_key:
						return
					_current["inflight"] -= 1
					if not ok:
						_current["failed"] = true
					_issue_requests())
	_request_done()


func _request_done() -> void:
	if _current.is_empty() or _current.has("build_task"):
		return
	if _current["failed"]:
		if _current["inflight"] == 0:
			_job_failed()
		return
	if _current["todo"].is_empty() and _current["inflight"] == 0:
		_start_build()


func _start_build() -> void:
	var snapshot: Dictionary = _current["snapshot"]
	var out_file := _cache_file(snapshot)
	DirAccess.make_dir_recursive_absolute(out_file.get_base_dir())
	if _debug:
		_current["raw_bytes"] = _file_sizes(_current["raw_files"])
	var slot := {"result": {}}
	_current["build_slot"] = slot
	_current["build_task"] = WorkerThreadPool.add_task(func() -> void:
		slot["result"] = _builder.call(snapshot, _raw_dir(), out_file), false, "weather build")


func _poll_build() -> void:
	if _current.is_empty() or not _current.has("build_task"):
		return
	if not WorkerThreadPool.is_task_completed(_current["build_task"]):
		return
	WorkerThreadPool.wait_for_task_completion(_current["build_task"])
	var result: Dictionary = _current["build_slot"]["result"]
	var key: String = _current["key"]
	var out_file := _cache_file(_current["snapshot"])
	var raw_files: Array = _current["raw_files"]
	var started_s: float = _current["started_s"]
	var raw_bytes: int = _current.get("raw_bytes", 0)
	_current = {}
	for path: String in raw_files:  # raw files go after a build, damaged or not
		DirAccess.remove_absolute(path)
	if result.get("ok", false) and FileAccess.file_exists(out_file):
		_jobs[key]["state"] = "done"
		var took := Time.get_ticks_msec() / 1000.0 - started_s
		_seconds_per_download = lerpf(_seconds_per_download, took, DOWNLOAD_TIME_WEIGHT)
		if _debug:
			print("weather: %s downloaded %.1f MB, built %.1f MB, %.1f s" % [key, raw_bytes / 1048576.0,
					_file_sizes([out_file]) / 1048576.0, took])
		enforce_cache_cap()
	else:
		if FileAccess.file_exists(out_file):
			DirAccess.remove_absolute(out_file)
		_mark_failed(key)


func _file_sizes(paths: Array) -> int:
	var total := 0
	for path: String in paths:
		var f := FileAccess.open(path, FileAccess.READ)
		if f != null:
			total += f.get_length()
			f.close()
	return total


func _job_failed() -> void:
	if _current.is_empty():
		return
	var key: String = _current["key"]
	for path: String in _current["raw_files"]:
		DirAccess.remove_absolute(path)
		DirAccess.remove_absolute(path + ".part")
	_current = {}
	_mark_failed(key)


func _mark_failed(key: String) -> void:
	if _jobs.has(key):
		_jobs[key]["state"] = "failed"
		_jobs[key]["failed_at_s"] = Time.get_ticks_msec() / 1000.0


func _core_build(snapshot: Dictionary, raw_dir: String, out_file: String) -> Dictionary:
	return _core.weather_build_snapshot(snapshot, raw_dir, out_file, _config_dir)


# ---------------------------------------------------------------------------------------------------------------------
# Cache on disk.

func _raw_dir() -> String:
	return _cache_dir.path_join("raw")


func _cache_file(snapshot: Dictionary) -> String:
	var key := String(snapshot["cache_key"]).validate_filename()
	return _cache_dir.path_join(key + ".wthr")


## Deletes the oldest built files while the cache is over its cap (files of the active / target pairs stay). The
## order is the build time (the file's modification time): a file in use is not re-touched, so this is "oldest built".
func enforce_cache_cap() -> void:
	var entries: Array = []
	var total := 0
	for name: String in DirAccess.get_files_at(_cache_dir):
		if not name.ends_with(".wthr"):
			continue
		var path := _cache_dir.path_join(name)
		var f := FileAccess.open(path, FileAccess.READ)
		if f == null:
			continue
		var size := f.get_length()
		f.close()
		entries.append({"path": path, "size": size, "time": FileAccess.get_modified_time(path)})
		total += size
	if total <= cache_cap_bytes:
		return
	entries.sort_custom(func(a: Dictionary, b: Dictionary) -> bool: return a["time"] < b["time"])
	var keep: Array = []
	keep.append_array(_active.get("files", []))
	keep.append_array(_target.get("files", []))
	for key: String in _wanted_keys:
		keep.append(_cache_dir.path_join(key.validate_filename() + ".wthr"))
	for entry: Dictionary in entries:
		if total <= cache_cap_bytes:
			break
		if entry["path"] in keep:
			continue
		if DirAccess.remove_absolute(entry["path"]) == OK:
			total -= int(entry["size"])
			for key: String in _jobs.keys():
				if _cache_dir.path_join(key.validate_filename() + ".wthr") == entry["path"]:
					_jobs.erase(key)
			_loaded.erase(entry["path"])


# ---------------------------------------------------------------------------------------------------------------------
# Loading and switching the active pair (loads on a worker; set_active only reads the core's cache).

func _switch_if_ready() -> void:
	if _target.is_empty():
		return
	var files: Array = _target["files"]
	var ready := true
	var cached: PackedStringArray = _core.weather_cached_files()
	for file: String in files:
		if _bad_files.has(file):
			_on_bad_target()
			return
		# Loaded with its texels (a consumer strips them once uploaded) and still in the core's cache.
		if not _loaded.get(file, {}).has("cloud_texels") or not (file in cached):
			ready = false
			_start_load(file)
	if not ready:
		return
	_active = _target
	_target = {}
	_blend = _target_blend
	_pushed_blend = -1.0
	var keep := {}
	for file: String in files:
		keep[file] = true
	for file: String in _loaded.keys():
		if not keep.has(file):
			_loaded.erase(file)
	if not light_driven_by_layer:
		_core.weather_set_active(files[0], files[1], _blend)
		_pushed_blend = _blend
	snapshots_changed.emit(_loaded[files[0]], _loaded[files[1]])
	if not light_driven_by_layer:  # nobody uploads them: do not keep ~230 MB of texels per snapshot
		for file: String in files:
			for key: String in ["cloud_texels", "cloud_ice_texels", "cloud_column_texels"]:
				_loaded[file].erase(key)


func _on_bad_target() -> void:
	# A file that cannot be read is deleted (it may be damaged) and its job marked failed; the analog replaces it.
	for i in range(2):
		var file: String = _target["files"][i]
		if _bad_files.has(file):
			var key := String(_target["metas"][i]["cache_key"])
			if file.begins_with(_cache_dir):
				DirAccess.remove_absolute(file)
				_bad_files.erase(file)  # the rebuilt file under this name gets a fresh try
				_loaded.erase(file)
				_mark_failed(key)
	_target = {}
	_ensure_baseline(_last_bracket_hour)


func _start_load(file: String) -> void:
	if _loading.has(file) or _bad_files.has(file):
		return
	var slot := {"result": {}}
	var task := WorkerThreadPool.add_task(func() -> void:
		slot["result"] = _loader.call(file), false, "weather load")
	_loading[file] = {"task": task, "slot": slot}


func _poll_loads() -> void:
	for file: String in _loading.keys():
		var entry: Dictionary = _loading[file]
		if not WorkerThreadPool.is_task_completed(entry["task"]):
			continue
		WorkerThreadPool.wait_for_task_completion(entry["task"])
		_loading.erase(file)
		var result: Dictionary = entry["slot"]["result"]
		if not result.get("ok", false):
			_bad_files[file] = true
			continue
		# Only the target's snapshots are kept (their texels go to the layer); a reload into the core's cache, a result
		# that arrives after the weather was switched off or the target moved on is dropped (the core cache has it).
		if enabled and file in _target.get("files", []):
			_loaded[file] = result


func _push_blend() -> void:
	if light_driven_by_layer or _active.is_empty() or absf(_blend - _pushed_blend) < BLEND_PUSH_STEP:
		return
	var files: Array = _active["files"]
	if _core.weather_set_active(files[0], files[1], _blend).get("ok", false):
		_pushed_blend = _blend
	else:  # fell out of the core's cache: load it again on a worker, retry next frame (never read on the main thread)
		for file: String in files:
			_start_load(file)


func _apply_enabled() -> void:
	if enabled:
		return
	_active = {}
	_target = {}
	_loaded.clear()
	_pushed_blend = -1.0
	if not light_driven_by_layer:
		_core.weather_set_active("", "", 0.0)
	snapshots_changed.emit({}, {})


# ---------------------------------------------------------------------------------------------------------------------
# HTTP plumbing and formatting.

func _send(url: String, first_byte: int, last_byte: int, dest: String, done: Callable) -> void:
	var id: int = _fetcher.request(url, first_byte, last_byte, dest)
	_pending[id] = done


func _on_fetch_finished(id: int, ok: bool, code: int, body: PackedByteArray) -> void:
	var done: Callable = _pending.get(id, Callable())
	_pending.erase(id)
	if done.is_valid():
		done.call(ok, code, body)


## "dd.mm.yyyy" (with_year) or "dd.mm." of an hour since 1900.
static func _date_text(hour_1900: int, with_year: bool) -> String:
	var dt := Time.get_datetime_dict_from_unix_time(int((float(hour_1900) - HOURS_1900_TO_1970) * 3600.0))
	if with_year:
		return "%02d.%02d.%04d" % [dt["day"], dt["month"], dt["year"]]
	return "%02d.%02d." % [dt["day"], dt["month"]]
