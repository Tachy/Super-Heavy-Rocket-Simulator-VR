class_name LunarSiteService
extends Node
## Downloads, caches and hands over the terrain tile (and the albedo tile) of a lunar landing site (lunar sites plan, Task
## 11). The core plans the byte ranges (`StarsimCore.lunar_site_tile_requests`) and assembles the downloaded bytes into
## the tile files (`assemble_lunar_site_tile`, atomic, worker-callable); this node only moves bytes, through a fetcher
## (WeatherFetcher or a double with `request()` / `finished`, optionally `progress`, `cancel()`, `retry_after_s`).
##
## `prepare(lat, lon)`: tiles already in data/terrain/sites are used without a request; otherwise the missing ranges are
## downloaded with `parallel` requests at a time, every range into its own file `<key>.parts/<n>.range` (a complete range
## survives an interruption and is not fetched again; the fetcher's own `.part` file is renamed only for a complete answer),
## then the tile is assembled on a worker thread (the frame never waits) and `site_ready` is emitted. A new `prepare` (or
## `cancel`) discards the running one: its answers are ignored (generation counter).
##
## Rulings: only a missing / failed HEIGHT tile fails the site. The albedo tile is look only (R6): a refused or failed albedo
## download is dropped, the site then uses the global albedo (`site_albedo_toml()` is ""). HTTP 429 (LROC CloudFront
## blocks ~1500 small requests for an hour) is never retried: the service fails with the Retry-After; the height ranges'
## other failures (PDS 403 = too many connections, a stall, a 5xx) are retried MAX_ATTEMPTS times per range, a 403 halves
## the parallel requests.
##
## The Node signal `ready` of the brief is `site_ready` here: Node already declares `ready`.

signal progress(fraction: float)  ## bytes received (finished ranges and the running ones) / bytes to fetch, 0 .. 1
signal site_ready(lat_deg: float, lon_deg: float)
signal failed(report: Dictionary)

const DEFAULT_PARALLEL := 8  ## [call] the PDS answers 403 with too many connections (the Python pipeline uses 16)
const MAX_ATTEMPTS := 3  ## [call] per height range
const SITES_SUBDIR := "terrain/sites"
const REPORT_TILE_MISSING := "data.lunar_site_tile_missing"
const REPORT_DOWNLOAD_FAILED := "data.lunar_site_download_failed"
const REPORT_RATE_LIMITED := "data.lunar_site_rate_limited"

var sites_dir := ""  ## where the tiles and the range files live (tests: their own directory)
var parallel := DEFAULT_PARALLEL

var _core: Object
var _fetcher: Object
var _generation := 0
var _active := false
var _lat_deg := 0.0
var _lon_deg := 0.0
var _jobs: Array[Dictionary] = []  ## {kind, request, index, url, first, last, dest, bytes, attempts, done}
var _queue: Array[int] = []  ## job indices waiting for a request slot
var _in_flight := {}  ## fetcher id -> {job: int, generation: int}
var _live_bytes := {}  ## fetcher id -> bytes so far
var _bytes_done := 0
var _bytes_total := 0
var _parallel_now := DEFAULT_PARALLEL
var _requests: Dictionary = {}  ## kind -> request
var _albedo_dropped := false
var _assemble_task := -1
var _assemble_result := {}  ## written by the worker, read once the task completed
var _albedo_toml := ""


## core: the StarsimCore (or a double with lunar_site_tile_requests / assemble_lunar_site_tile / lunar_site_tiles_present);
## fetcher: request(url, first, last, dest) -> id and the `finished(id, ok, http_code, body)` signal.
func setup(core: Object, fetcher: Object, parallel_requests := DEFAULT_PARALLEL) -> void:
	_core = core
	_fetcher = fetcher
	parallel = parallel_requests
	if sites_dir == "":
		sites_dir = Paths.data_dir().path_join(SITES_SUBDIR)
	fetcher.finished.connect(_on_finished)
	if fetcher.has_signal("progress"):
		fetcher.progress.connect(_on_progress)
	set_process(false)


## A coordinate on the tile cache's grid: hundredths of a degree (the core's tile key and site id have this granularity but
## the tile is centred on the exact coordinate, so one key must always come from one coordinate).
static func snap_deg(deg: float) -> float:
	return roundf(deg * 100.0) / 100.0


## Makes the tiles of the site at (lat_deg, lon_deg) - snapped to hundredths of a degree (snap_deg) - available;
## `site_ready(lat, lon)` carries the snapped coordinate to hand to World.switch_lunar_site; `failed` otherwise. Cancels a
## running prepare.
func prepare(lat_deg: float, lon_deg: float) -> void:
	lat_deg = snap_deg(lat_deg)
	lon_deg = snap_deg(lon_deg)
	_stop()
	_generation += 1
	_active = true
	_lat_deg = lat_deg
	_lon_deg = lon_deg
	_albedo_toml = ""
	_albedo_dropped = false
	_parallel_now = maxi(parallel, 1)
	var generation := _generation
	_requests = {}
	for request: Dictionary in _core.lunar_site_tile_requests(lat_deg, lon_deg):
		_requests[String(request.get("kind", ""))] = request
	var height: Dictionary = _requests.get("height", {})
	if height.is_empty():
		_fail(_download_report(-1))
		return
	if not bool(height.get("ok", false)):
		_fail(height.get("report", {}))
		return
	_plan_jobs()
	if _jobs.is_empty():  # every wanted tile is cached: no request at all
		_finish.call_deferred(generation)
		return
	set_process(true)
	progress.emit(_fraction())
	_pump()


## Gives the running prepare up (no signal): requests in flight are cancelled, finished ranges stay for a retry.
func cancel() -> void:
	_stop()


## Is a prepare running?
func busy() -> bool:
	return _active


## The albedo tile's .toml of the prepared site, "" when it has none (refused, failed, not wanted): the global albedo.
func site_albedo_toml() -> String:
	return _albedo_toml


## The albedo tile's .toml of the site if it is on disk, else "" (without a prepare; the world's switch reads it).
static func albedo_toml_path(core: Object, sites_directory: String, lat_deg: float, lon_deg: float) -> String:
	for request: Dictionary in core.lunar_site_tile_requests(lat_deg, lon_deg):
		if String(request.get("kind", "")) == "albedo" and bool(request.get("ok", false)):
			var path := sites_directory.path_join(String(request["cache_key"]) + ".toml")
			return path if FileAccess.file_exists(path) else ""
	return ""


# ---- planning ----------------------------------------------------------------------------------------------------------

func _plan_jobs() -> void:
	_jobs.clear()
	_queue.clear()
	_bytes_done = 0
	_bytes_total = 0
	for kind: String in ["height", "albedo"]:
		var request: Dictionary = _requests.get(kind, {})
		if request.is_empty() or not bool(request.get("ok", false)) or bool(request.get("present", false)):
			continue
		var dir := sites_dir.path_join(String(request["cache_key"]) + ".parts")
		var ranges: Array = request["ranges"]
		for n in ranges.size():
			var r: Dictionary = ranges[n]
			var size := int(r["last"]) - int(r["first"]) + 1
			var job := {"kind": kind, "request": request, "index": n, "url": String(r["url"]), "first": int(r["first"]),
					"last": int(r["last"]), "dest": dir.path_join("%d.range" % n), "bytes": size, "attempts": 0,
					"done": false}
			_bytes_total += size
			if _file_size(job["dest"]) == size:  # a complete range of an earlier, interrupted run
				job["done"] = true
				_bytes_done += size
			else:
				_queue.append(_jobs.size())
			_jobs.append(job)
	if _jobs.is_empty():
		return
	var all_done := true
	for job: Dictionary in _jobs:
		all_done = all_done and bool(job["done"])
	if all_done:
		_queue.clear()


static func _file_size(path: String) -> int:
	if not FileAccess.file_exists(path):
		return -1
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return -1
	var size := f.get_length()
	f.close()
	return size


# ---- downloading -------------------------------------------------------------------------------------------------------

func _pump() -> void:
	while _active and _in_flight.size() < _parallel_now and not _queue.is_empty():
		var index: int = _queue.pop_front()
		var job: Dictionary = _jobs[index]
		if bool(job["done"]) or (job["kind"] == "albedo" and _albedo_dropped):
			continue
		job["attempts"] = int(job["attempts"]) + 1
		var id: int = _fetcher.request(job["url"], job["first"], job["last"], job["dest"])
		_in_flight[id] = {"job": index, "generation": _generation}
		_live_bytes[id] = 0
	if _active and _in_flight.is_empty() and _queue.is_empty():
		_start_assembly()


func _on_progress(id: int, bytes: int) -> void:
	if not _in_flight.has(id):
		return
	_live_bytes[id] = bytes
	progress.emit(_fraction())


func _fraction() -> float:
	if _bytes_total <= 0:
		return 1.0
	var live := 0
	for id: int in _live_bytes:
		live += int(_live_bytes[id])
	return clampf(float(_bytes_done + live) / float(_bytes_total), 0.0, 1.0)


func _on_finished(id: int, ok: bool, http_code: int, _body: PackedByteArray) -> void:
	var entry: Dictionary = _in_flight.get(id, {})
	var retry_after := 0.0
	if _fetcher.get("retry_after_s") is Dictionary:  # always erase: a stale id's entry must not pile up
		var table: Dictionary = _fetcher.retry_after_s
		retry_after = float(table.get(id, 0.0))
		table.erase(id)
	if entry.is_empty() or int(entry["generation"]) != _generation or not _active:
		return  # an answer of a cancelled / replaced prepare
	_in_flight.erase(id)
	_live_bytes.erase(id)
	var index: int = entry["job"]
	var job: Dictionary = _jobs[index]
	if ok and _file_size(job["dest"]) != int(job["bytes"]):
		ok = false  # a short or long answer is never a range of the tile
	if ok:
		job["done"] = true
		_bytes_done += int(job["bytes"])
		progress.emit(_fraction())
		_pump()
		return
	if FileAccess.file_exists(job["dest"]):
		DirAccess.remove_absolute(job["dest"])
	if job["kind"] == "albedo":
		_drop_albedo()
		_pump()
		return
	if http_code == 429:  # never hammer: the limit lasts (CloudFront: an hour)
		_fail(_rate_limit_report(retry_after))
		return
	if int(job["attempts"]) >= MAX_ATTEMPTS:
		_fail(_download_report(http_code))
		return
	if http_code == 403:
		_parallel_now = maxi(_parallel_now / 2, 1)
	_queue.push_back(index)
	_pump()


## The albedo is look only: its remaining ranges are given up (in flight ones cancelled), the bytes leave the total.
func _drop_albedo() -> void:
	_albedo_dropped = true
	for id: int in _in_flight.keys():
		if _jobs[int(_in_flight[id]["job"])]["kind"] == "albedo":
			_cancel_request(id)
			_in_flight.erase(id)
			_live_bytes.erase(id)
	var total := 0
	var done := 0
	for job: Dictionary in _jobs:
		if job["kind"] == "albedo":
			continue
		total += int(job["bytes"])
		if bool(job["done"]):
			done += int(job["bytes"])
	_bytes_total = total
	_bytes_done = done
	progress.emit(_fraction())


func _cancel_request(id: int) -> void:
	if _fetcher.has_method("cancel"):
		_fetcher.cancel(id)


func _stop() -> void:
	_generation += 1
	_active = false
	for id: int in _in_flight.keys():
		_cancel_request(id)
	_in_flight.clear()
	_live_bytes.clear()
	_queue.clear()
	set_process(false)


# ---- assembling --------------------------------------------------------------------------------------------------------

func _start_assembly() -> void:
	progress.emit(1.0)
	var wanted: Array = []
	for kind: String in ["height", "albedo"]:
		var request: Dictionary = _requests.get(kind, {})
		if request.is_empty() or not bool(request.get("ok", false)) or bool(request.get("present", false)):
			continue
		if kind == "albedo" and _albedo_dropped:
			continue
		wanted.append(request)
	_assemble_result = {}
	_assemble_task = WorkerThreadPool.add_task(_assemble_worker.bind(_generation, wanted.duplicate(true), sites_dir))


## Worker thread: reads each tile's range files, hands the bodies to the core, removes the range files of an assembled
## tile. The result is published as one assignment (read by _process after the task completed).
func _assemble_worker(generation: int, wanted: Array, directory: String) -> void:
	var results := {}
	for request: Dictionary in wanted:
		var ranges: Array = request["ranges"]
		var bodies: Array[PackedByteArray] = []
		var dir := directory.path_join(String(request["cache_key"]) + ".parts")
		for n in ranges.size():
			bodies.append(FileAccess.get_file_as_bytes(dir.path_join("%d.range" % n)))
		var answer: Dictionary = _core.assemble_lunar_site_tile(request, bodies)
		results[String(request["kind"])] = answer
		# A tile that assembled needs its ranges no more; a refused one would refuse again from the same bytes.
		for n in ranges.size():
			DirAccess.remove_absolute(dir.path_join("%d.range" % n))
		DirAccess.remove_absolute(dir)
	_assemble_result = {"generation": generation, "results": results}


func _process(_delta: float) -> void:
	if _assemble_task < 0 or not WorkerThreadPool.is_task_completed(_assemble_task):
		return
	WorkerThreadPool.wait_for_task_completion(_assemble_task)
	_assemble_task = -1
	var outcome := _assemble_result
	_assemble_result = {}
	if int(outcome.get("generation", -1)) != _generation or not _active:
		return  # a prepare replaced this one meanwhile: its tile files are valid but nobody waits for them
	var results: Dictionary = outcome.get("results", {})
	var height: Dictionary = results.get("height", {"ok": true})  # absent: the tile was already there
	if not bool(height.get("ok", false)):
		_fail(height.get("report", _download_report(-1)))
		return
	_finish(_generation)


func _finish(generation: int) -> void:
	if generation != _generation or not _active:
		return
	_albedo_toml = ""
	var albedo: Dictionary = _requests.get("albedo", {})
	if not albedo.is_empty() and bool(albedo.get("ok", false)):
		var path := sites_dir.path_join(String(albedo["cache_key"]) + ".toml")
		if FileAccess.file_exists(path):
			_albedo_toml = path
	_active = false
	set_process(false)
	site_ready.emit(_lat_deg, _lon_deg)


func _fail(report: Dictionary) -> void:
	_stop()
	failed.emit(report)


func _exit_tree() -> void:
	if _assemble_task >= 0:
		WorkerThreadPool.wait_for_task_completion(_assemble_task)
		_assemble_task = -1


# ---- reports (codes of the game's own: the core knows nothing of the network) -------------------------------------------

## No answer at all (http_code 0: offline) or no plan: the tile is missing (the code of the core's own refusal); an HTTP
## answer that is not a success: the download failed with that status.
func _download_report(http_code: int) -> Dictionary:
	var params := {"latitude": deg_to_rad(_lat_deg), "longitude": deg_to_rad(_lon_deg)}
	var kinds := {"latitude": "angle", "longitude": "angle"}
	if http_code > 0:
		params["http_code"] = float(http_code)
		kinds["http_code"] = "number"
	return {"code": REPORT_DOWNLOAD_FAILED if http_code > 0 else REPORT_TILE_MISSING, "params": params, "kinds": kinds,
			"causes": []}


func _rate_limit_report(retry_after_s: float) -> Dictionary:
	return {"code": REPORT_RATE_LIMITED, "params": {"retry_after": retry_after_s}, "kinds": {"retry_after": "time"},
			"causes": []}


# ---- the albedo tile for the shaders -----------------------------------------------------------------------------------

## Reads an albedo tile (TerrainGridFile, int16, kind "albedo", relative albedo = value x dn_scale_m, R3) into a float
## image and the shader mapping (moon_site_albedo.gdshaderinc). Pure data: callable on a worker thread.
## Returns {ok, image (FORMAT_RF), bounds: Vector4 (lat min, lat max, lon min, lon max; rad), grid: Vector4}.
static func read_albedo(toml_path: String) -> Dictionary:
	var out := {"ok": false}
	if toml_path == "" or not FileAccess.file_exists(toml_path):
		return out
	var grid: Dictionary = TomlLite.load_file(toml_path).get("grid", {})
	if String(grid.get("kind", "")) != "albedo":
		return out
	var width := int(grid.get("samples", 0))
	var height := int(grid.get("lines", 0))
	var bytes := FileAccess.get_file_as_bytes(toml_path.get_base_dir().path_join(String(grid.get("heights", ""))))
	if width < 2 or height < 2 or bytes.size() != 2 * width * height:
		return out
	var dn_scale := float(grid.get("dn_scale_m", 0.0))
	var values := PackedFloat32Array()
	values.resize(width * height)
	for i in values.size():
		values[i] = bytes.decode_s16(2 * i) * dn_scale
	out["image"] = Image.create_from_data(width, height, false, Image.FORMAT_RF, values.to_byte_array())
	var radius_m := float(grid.get("sphere_radius_m", 1737400.0))
	var projection := String(grid.get("projection", ""))
	if projection == "simple_cylindrical":
		var ppd := float(grid.get("pixels_per_degree", 0.0))
		var zero_line := float(grid.get("zero_line", 0.0))
		var zero_sample := float(grid.get("zero_sample", 0.0))
		var lon_min := (-zero_sample - 0.5) / ppd
		out["bounds"] = Vector4(deg_to_rad((zero_line - (height - 1.0) - 0.5) / ppd), deg_to_rad((zero_line + 0.5) / ppd),
				deg_to_rad(lon_min), deg_to_rad(lon_min + float(width) / ppd))
		out["grid"] = Vector4(zero_line, zero_sample, ppd, 1.0)
	elif projection == "polar_stereographic_north" or projection == "polar_stereographic_south":
		var north := projection == "polar_stereographic_north"
		var scale_m := float(grid.get("scale_m", 0.0))
		var pole_line := float(grid.get("pole_line", 0.0)) - float(grid.get("first_line", 0.0))
		var pole_sample := float(grid.get("pole_sample", 0.0)) - float(grid.get("first_sample", 0.0))
		# The tile's colatitude band: the pole's distance to the nearest / farthest point of the pixel rectangle.
		var nearest := Vector2(clampf(pole_line, -0.5, height - 0.5), clampf(pole_sample, -0.5, width - 0.5)) \
				- Vector2(pole_line, pole_sample)
		var far_line := maxf(absf(pole_line + 0.5), absf(pole_line - height + 0.5))
		var far_sample := maxf(absf(pole_sample + 0.5), absf(pole_sample - width + 0.5))
		var colat_min := 2.0 * atan(nearest.length() * scale_m / (2.0 * radius_m))
		var colat_max := 2.0 * atan(Vector2(far_line, far_sample).length() * scale_m / (2.0 * radius_m))
		var pole := 0.5 * PI
		if north:
			out["bounds"] = Vector4(pole - colat_max, pole - colat_min, -PI, PI)
		else:
			out["bounds"] = Vector4(-pole + colat_min, -pole + colat_max, -PI, PI)
		out["grid"] = Vector4(pole_line, pole_sample, scale_m, 2.0 if north else 3.0)
	else:
		return out
	out["ok"] = true
	return out


## Sets (albedo from read_albedo) or clears (empty) the site albedo uniforms of a terrain / rock material. Main thread.
static func apply_albedo(material: ShaderMaterial, albedo: Dictionary, blend_m := 5000.0) -> void:
	material.set_shader_parameter("site_albedo_blend_m", blend_m)
	if albedo.is_empty() or not bool(albedo.get("ok", false)):
		material.set_shader_parameter("site_albedo_grid", Vector4.ZERO)
		material.set_shader_parameter("site_albedo_bounds", Vector4.ZERO)
		return
	material.set_shader_parameter("site_albedo_texture", ImageTexture.create_from_image(albedo["image"]))
	material.set_shader_parameter("site_albedo_bounds", albedo["bounds"])
	material.set_shader_parameter("site_albedo_grid", albedo["grid"])
