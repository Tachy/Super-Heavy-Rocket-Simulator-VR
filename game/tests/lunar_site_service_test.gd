# Headless test of LunarSiteService (lunar sites plan Task 11): cache hit without a request, progress + assembly + ready,
# interrupted download and its resumed retry, a site switch mid-download, offline, the albedo rulings (non-fatal), the
# 429 rule, the 503 backoff, a switch during assembly, exact coordinates and the albedo tile reader - with a fake fetcher (files of the requested size) and a fake core, no network.
# Run: godot --headless --path game --script res://tests/lunar_site_service_test.gd
extends SceneTree

const RANGE_BYTES := 100
const HEIGHT_RANGES := 6
const ALBEDO_RANGES := 2

var _failures := 0
var _dirs: Array[String] = []
var _serial := 0


## The fetcher double: a request writes a file of the requested size at once and is answered on pump().
class FakeFetcher:
	extends RefCounted
	signal finished(id: int, ok: bool, http_code: int, body: PackedByteArray)
	signal progress(id: int, bytes: int)
	var requests: Array = []
	var cancelled: Array = []
	var retry_after_s := {}
	var fail_after := -1  ## requests after this many answer code `fail_code`
	var fail_code := 0
	var retry_after_value := 3600.0  ## the Retry-After of a 429 / 503 answer
	var fail_albedo_code := -1  ## >= 0: every albedo request answers this code
	var offline := false
	var _next := 1
	var _queue: Array = []

	func request(url: String, first_byte: int, last_byte: int, dest: String) -> int:
		var id := _next
		_next += 1
		requests.append({"url": url, "first": first_byte, "last": last_byte, "dest": dest, "id": id})
		var answer := {"id": id, "ok": true, "code": 206}
		if offline:
			answer["ok"] = false
			answer["code"] = 0
		elif fail_albedo_code >= 0 and url.contains("albedo"):
			answer["ok"] = false
			answer["code"] = fail_albedo_code
		elif fail_after >= 0 and requests.size() > fail_after:
			answer["ok"] = false
			answer["code"] = fail_code
			if fail_code == 429 or fail_code == 503:
				retry_after_s[id] = retry_after_value
		else:
			DirAccess.make_dir_recursive_absolute(dest.get_base_dir())
			var f := FileAccess.open(dest, FileAccess.WRITE)
			var bytes := PackedByteArray()
			bytes.resize(last_byte - first_byte + 1)
			f.store_buffer(bytes)
			f.close()
		_queue.append(answer)
		return id

	func cancel(id: int) -> void:
		cancelled.append(id)
		_queue = _queue.filter(func(a: Dictionary) -> bool: return a["id"] != id)

	func pump() -> void:
		var batch := _queue
		_queue = []
		for a: Dictionary in batch:
			if a["ok"]:
				progress.emit(a["id"], RANGE_BYTES / 2)
			finished.emit(a["id"], a["ok"], a["code"], PackedByteArray())

	func files_requested(kind: String) -> int:
		return requests.filter(func(r: Dictionary) -> bool: return String(r["url"]).contains(kind)).size()


## The core double: plans HEIGHT_RANGES height ranges (+ ALBEDO_RANGES unless `no_albedo`) per site, assembles by writing
## the tile's .toml; a site's key follows its latitude. `present` follows the files on disk.
class FakeCore:
	extends RefCounted
	var sites_dir := ""
	var no_albedo := false
	var assembled: Array = []
	var assemble_ok := true
	var albedo_assemble_ok := true
	var gate := Semaphore.new()  ## assembling the tile of `block_key` waits for a post
	var block_key := ""
	var entered := 0  ## assemblies that began (worker thread)

	func key(kind: String, lat: float, lon: float) -> String:
		return "%s_%d_%d" % [kind, int(lat * 100.0), int(lon * 100.0)]

	var requested: Array = []

	func lunar_site_tile_requests(lat: float, lon: float) -> Array:
		requested.append([lat, lon])
		var out: Array = []
		for kind: String in ["height", "albedo"]:
			var count := HEIGHT_RANGES if kind == "height" else ALBEDO_RANGES
			var request := {"kind": kind, "lat_deg": lat, "lon_deg": lon, "cache_key": key(kind, lat, lon), "ranges": [],
					"ok": true, "report": {}, "present": FileAccess.file_exists(sites_dir.path_join(key(kind, lat, lon) + ".toml"))}
			if kind == "albedo" and no_albedo:
				request["ok"] = false
				request["report"] = {"code": "lunar.site_no_product", "params": {}, "kinds": {}, "causes": []}
			else:
				for n in count:
					request["ranges"].append({"url": "https://example.test/%s.img" % kind, "first": n * RANGE_BYTES,
							"last": (n + 1) * RANGE_BYTES - 1})
			out.append(request)
		return out

	func assemble_lunar_site_tile(request: Dictionary, bodies: Array) -> Dictionary:
		entered += 1
		if block_key != "" and request["cache_key"] == block_key:
			gate.wait()
		for body: PackedByteArray in bodies:
			if body.size() != RANGE_BYTES:
				return {"ok": false, "report": {"code": "command.internal_error", "params": {}, "kinds": {}, "causes": []}}
		var ok := assemble_ok if request["kind"] == "height" else albedo_assemble_ok
		if not ok:
			return {"ok": false, "report": {"code": "lunar.site_no_product", "params": {}, "kinds": {}, "causes": []}}
		assembled.append(request["cache_key"])
		var f := FileAccess.open(sites_dir.path_join(String(request["cache_key"]) + ".toml"), FileAccess.WRITE)
		f.store_string("[grid]\n")
		f.close()
		return {"ok": true, "report": {}}


class Probe:
	extends RefCounted
	var ready: Array = []
	var failed: Array = []
	var fractions: Array = []

	func on_ready(lat: float, lon: float) -> void:
		ready.append([lat, lon])

	func on_failed(report: Dictionary) -> void:
		failed.append(report)

	func on_progress(fraction: float) -> void:
		fractions.append(fraction)


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	await _test_cached()
	await _test_exact_coordinates()
	await _test_download()
	await _test_interrupted()
	await _test_switch()
	await _test_offline()
	await _test_albedo_non_fatal()
	await _test_rate_limit()
	await _test_backoff_503()
	await _test_switch_during_assembly()
	await _test_assemble_failure()
	_test_read_albedo()
	for dir in _dirs:
		_remove_tree(dir)
	print("LUNAR SITE SERVICE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


## A fresh test directory + the objects; the service is a child of the root.
func _make(parallel := 8) -> Dictionary:
	_serial += 1
	var dir := OS.get_user_data_dir().path_join("lunar_site_test_%d_%d" % [OS.get_process_id(), _serial])
	DirAccess.make_dir_recursive_absolute(dir)
	_dirs.append(dir)
	var core := FakeCore.new()
	core.sites_dir = dir
	var fetcher := FakeFetcher.new()
	var service := LunarSiteService.new()
	service.sites_dir = dir
	service.backoff_base_s = 0.02
	root.add_child(service)
	service.setup(core, fetcher, parallel)
	var probe := Probe.new()
	service.site_ready.connect(probe.on_ready)
	service.failed.connect(probe.on_failed)
	service.progress.connect(probe.on_progress)
	return {"core": core, "fetcher": fetcher, "service": service, "probe": probe, "dir": dir}


## Pumps the fetcher and frames until `done` holds (at most `frames`).
func _until(env: Dictionary, done: Callable, frames := 400) -> void:
	for i in frames:
		(env["fetcher"] as FakeFetcher).pump()
		await process_frame
		if done.call():
			return


func _touch(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	f.store_string("[grid]\n")
	f.close()


func _test_cached() -> void:
	var env := _make()
	var core: FakeCore = env["core"]
	var fetcher: FakeFetcher = env["fetcher"]
	var probe: Probe = env["probe"]
	_touch(env["dir"].path_join(core.key("height", 10.0, 20.0) + ".toml"))
	_touch(env["dir"].path_join(core.key("albedo", 10.0, 20.0) + ".toml"))
	env["service"].prepare(10.0, 20.0)
	await _until(env, func() -> bool: return not probe.ready.is_empty(), 10)
	_check(probe.ready.size() == 1 and fetcher.requests.is_empty(), "cached site prepares without a request")
	_check((env["service"] as LunarSiteService).site_albedo_toml().ends_with(core.key("albedo", 10.0, 20.0) + ".toml"),
			"cached site names its albedo tile")
	env["service"].queue_free()


func _test_exact_coordinates() -> void:
	# R19: the user's coordinates are never rounded; the core decides what its cache key accepts.
	var env := _make()
	var probe: Probe = env["probe"]
	var fetcher: FakeFetcher = env["fetcher"]
	env["service"].prepare(10.004, 20.006)
	await _until(env, func() -> bool: return not probe.ready.is_empty() or not probe.failed.is_empty())
	_check(probe.ready.size() == 1 and probe.ready[0][0] == 10.004 and probe.ready[0][1] == 20.006,
			"site_ready carries the exact coordinates (%s)" % str(probe.ready))
	var core: FakeCore = env["core"]
	_check(core.requested.size() > 0 and core.requested.all(func(c: Array) -> bool: return c[0] == 10.004 and c[1] == 20.006),
			"the core is asked for the exact coordinates (%s)" % str(core.requested))
	_check(fetcher.requests.size() > 0, "downloaded")
	env["service"].queue_free()


func _test_download() -> void:
	var env := _make(3)
	var core: FakeCore = env["core"]
	var fetcher: FakeFetcher = env["fetcher"]
	var probe: Probe = env["probe"]
	env["service"].prepare(10.0, 20.0)
	_check(fetcher.requests.size() == 3, "at most `parallel` requests at a time (%d)" % fetcher.requests.size())
	await _until(env, func() -> bool: return not probe.ready.is_empty() or not probe.failed.is_empty())
	_check(probe.ready.size() == 1 and probe.failed.is_empty(), "download assembles and emits ready")
	_check(fetcher.requests.size() == HEIGHT_RANGES + ALBEDO_RANGES, "every range requested once (%d)" % fetcher.requests.size())
	_check(core.assembled.has(core.key("height", 10.0, 20.0)) and core.assembled.has(core.key("albedo", 10.0, 20.0)),
			"height and albedo tile assembled")
	var ranged: bool = fetcher.requests.all(func(r: Dictionary) -> bool: return int(r["first"]) >= 0 and r["dest"] != "")
	_check(ranged, "ranged requests into files")
	var monotone := true
	for i in range(1, probe.fractions.size()):
		monotone = monotone and probe.fractions[i] >= probe.fractions[i - 1] - 1.0e-9
	_check(monotone and probe.fractions.size() > 3 and is_equal_approx(probe.fractions.back(), 1.0),
			"progress rises to 1 (%d events)" % probe.fractions.size())
	_check(not DirAccess.dir_exists_absolute(env["dir"].path_join(core.key("height", 10.0, 20.0) + ".parts")),
			"range files removed after assembly")
	_check(not (env["service"] as LunarSiteService).site_albedo_toml().is_empty(), "albedo tile named")
	env["service"].queue_free()


func _test_interrupted() -> void:
	var env := _make(2)
	var core: FakeCore = env["core"]
	var fetcher: FakeFetcher = env["fetcher"]
	var probe: Probe = env["probe"]
	fetcher.fail_after = 3  # the 4th request and later fail with 500
	fetcher.fail_code = 500
	env["service"].prepare(11.0, 22.0)
	await _until(env, func() -> bool: return not probe.failed.is_empty() or not probe.ready.is_empty())
	_check(probe.failed.size() == 1 and probe.ready.is_empty(), "interrupted download fails")
	_check(probe.failed.size() == 1 and probe.failed[0]["code"] == "data.lunar_site_download_failed",
			"with the download code (%s)" % str(probe.failed[0].get("code") if probe.failed.size() > 0 else ""))
	_check(core.assembled.is_empty() and not FileAccess.file_exists(env["dir"].path_join(core.key("height", 11.0, 22.0) + ".toml")),
			"no tile left behind")
	var first_run := fetcher.requests.size()
	fetcher.fail_after = -1
	fetcher.requests.clear()
	probe.failed.clear()
	env["service"].prepare(11.0, 22.0)
	await _until(env, func() -> bool: return not probe.ready.is_empty() or not probe.failed.is_empty())
	_check(probe.ready.size() == 1, "the retry completes")
	_check(fetcher.requests.size() < HEIGHT_RANGES + ALBEDO_RANGES and first_run > 0,
			"finished ranges are not fetched again (%d of %d)" % [fetcher.requests.size(), HEIGHT_RANGES + ALBEDO_RANGES])
	env["service"].queue_free()


func _test_switch() -> void:
	var env := _make(2)
	var core: FakeCore = env["core"]
	var fetcher: FakeFetcher = env["fetcher"]
	var probe: Probe = env["probe"]
	env["service"].prepare(30.0, 40.0)
	fetcher.pump()
	await process_frame
	env["service"].prepare(-5.0, 60.0)  # the first site's answers still arrive
	await _until(env, func() -> bool: return not probe.ready.is_empty() or not probe.failed.is_empty())
	await _until(env, func() -> bool: return false, 20)  # late answers of the first site
	_check(probe.ready.size() == 1 and is_equal_approx(probe.ready[0][0], -5.0), "only the second site is ready")
	_check(not core.assembled.has(core.key("height", 30.0, 40.0)), "the first site's tile was never assembled")
	_check(fetcher.cancelled.size() > 0, "the first site's requests were cancelled")
	env["service"].queue_free()


func _test_offline() -> void:
	var env := _make()
	var fetcher: FakeFetcher = env["fetcher"]
	var probe: Probe = env["probe"]
	fetcher.offline = true
	env["service"].prepare(10.0, 20.0)
	await _until(env, func() -> bool: return not probe.failed.is_empty() or not probe.ready.is_empty())
	_check(probe.failed.size() == 1 and probe.failed[0]["code"] == "data.lunar_site_tile_missing",
			"offline missing tile fails with data.lunar_site_tile_missing")
	_check(probe.failed.size() == 1 and I18n.report_text(probe.failed[0]) != ""
			and not I18n.report_text(probe.failed[0]).begins_with("report."), "and the text exists")
	env["service"].queue_free()


func _test_albedo_non_fatal() -> void:
	# A refused albedo (no product): height only, ready.
	var env := _make()
	var core: FakeCore = env["core"]
	var probe: Probe = env["probe"]
	core.no_albedo = true
	env["service"].prepare(10.0, 20.0)
	await _until(env, func() -> bool: return not probe.ready.is_empty() or not probe.failed.is_empty())
	_check(probe.ready.size() == 1 and (env["service"] as LunarSiteService).site_albedo_toml() == "",
			"albedo refusal is non-fatal (global albedo)")
	env["service"].queue_free()
	# An albedo download failure: dropped, still ready.
	env = _make()
	probe = env["probe"]
	(env["fetcher"] as FakeFetcher).fail_albedo_code = 429
	env["service"].prepare(10.0, 20.0)
	await _until(env, func() -> bool: return not probe.ready.is_empty() or not probe.failed.is_empty())
	_check(probe.ready.size() == 1 and probe.failed.is_empty() and (env["service"] as LunarSiteService).site_albedo_toml() == "",
			"albedo download failure is non-fatal")
	var albedo_requests: int = (env["fetcher"] as FakeFetcher).files_requested("albedo")
	_check(albedo_requests <= 8, "no retry storm on the albedo (%d requests)" % albedo_requests)
	env["service"].queue_free()
	# An albedo assembly refusal: dropped too.
	env = _make()
	probe = env["probe"]
	(env["core"] as FakeCore).albedo_assemble_ok = false
	env["service"].prepare(10.0, 20.0)
	await _until(env, func() -> bool: return not probe.ready.is_empty() or not probe.failed.is_empty())
	_check(probe.ready.size() == 1, "albedo assembly refusal does not fail the site")
	env["service"].queue_free()


func _test_rate_limit() -> void:
	var env := _make(2)
	var fetcher: FakeFetcher = env["fetcher"]
	var probe: Probe = env["probe"]
	fetcher.fail_after = 1
	fetcher.fail_code = 429
	env["service"].prepare(10.0, 20.0)
	await _until(env, func() -> bool: return not probe.failed.is_empty() or not probe.ready.is_empty())
	_check(probe.failed.size() == 1 and probe.failed[0]["code"] == "data.lunar_site_rate_limited"
			and is_equal_approx(float(probe.failed[0]["params"]["retry_after"]), 3600.0), "429 fails with the Retry-After")
	var count := fetcher.requests.size()
	await _until(env, func() -> bool: return false, 20)
	_check(fetcher.requests.size() == count, "no request after the 429 (never hammer)")
	env["service"].queue_free()


func _test_backoff_503() -> void:
	var env := _make(1)
	var fetcher: FakeFetcher = env["fetcher"]
	var probe: Probe = env["probe"]
	fetcher.fail_after = 0
	fetcher.fail_code = 503
	fetcher.retry_after_value = 0.3
	var started := Time.get_ticks_msec()
	env["service"].prepare(10.0, 20.0)
	await _until(env, func() -> bool: return not probe.failed.is_empty() or not probe.ready.is_empty())
	var elapsed_s := (Time.get_ticks_msec() - started) / 1000.0
	_check(probe.failed.size() == 1 and probe.failed[0]["code"] == "data.lunar_site_download_failed"
			and is_equal_approx(float(probe.failed[0]["params"]["http_code"]), 503.0), "503 ends in a download report")
	_check(fetcher.requests.size() <= HEIGHT_RANGES * LunarSiteService.MAX_ATTEMPTS, "bounded requests, no storm (%d)" % fetcher.requests.size())
	_check(elapsed_s >= 0.55, "the Retry-After spaces the retries (%.2f s for 2 waits of 0.3 s)" % elapsed_s)
	env["service"].queue_free()


## Site A is assembling (its worker blocked) when site B is prepared and completes; A's late result must not disturb B.
func _test_switch_during_assembly() -> void:
	var env := _make(8)
	var core: FakeCore = env["core"]
	var fetcher: FakeFetcher = env["fetcher"]
	var probe: Probe = env["probe"]
	var service: LunarSiteService = env["service"]
	core.block_key = core.key("height", 30.0, 40.0)
	service.prepare(30.0, 40.0)
	await _until(env, func() -> bool: return core.entered > 0, 200)
	_check(core.entered == 1 and service.busy(), "site A is assembling")
	service.prepare(-5.0, 60.0)
	await _until(env, func() -> bool: return not probe.ready.is_empty() or not probe.failed.is_empty(), 200)
	_check(probe.ready.size() == 1 and is_equal_approx(probe.ready[0][0], -5.0) and probe.failed.is_empty(),
			"switch during assembly completes the second site (%s)" % str(probe.ready))
	core.gate.post()  # A's worker finishes late
	await _until(env, func() -> bool: return false, 30)
	_check(probe.ready.size() == 1 and probe.failed.is_empty() and not service.busy(), "A's late result is discarded")
	_check(FileAccess.file_exists(env["dir"].path_join(core.key("height", 30.0, 40.0) + ".toml")), "A's tile file is complete")
	var leftovers := DirAccess.get_directories_at(env["dir"]).size()
	_check(leftovers == 0, "no work directory left (%d)" % leftovers)
	service.queue_free()


func _test_assemble_failure() -> void:
	var env := _make()
	var probe: Probe = env["probe"]
	(env["core"] as FakeCore).assemble_ok = false
	env["service"].prepare(10.0, 20.0)
	await _until(env, func() -> bool: return not probe.failed.is_empty() or not probe.ready.is_empty())
	_check(probe.failed.size() == 1 and probe.failed[0]["code"] == "lunar.site_no_product",
			"a height assembly refusal fails the site with the core's report")
	env["service"].queue_free()


## read_albedo on a synthetic tile of each projection: values x dn_scale, bounds from the grid keys.
func _test_read_albedo() -> void:
	_serial += 1
	var dir := OS.get_user_data_dir().path_join("lunar_site_test_%d_%d" % [OS.get_process_id(), _serial])
	DirAccess.make_dir_recursive_absolute(dir)
	_dirs.append(dir)
	var raw := PackedByteArray()
	raw.resize(2 * 4 * 3)
	for i in 12:
		raw.encode_s16(2 * i, 16384 + i * 100)
	var f := FileAccess.open(dir.path_join("a.i16"), FileAccess.WRITE)
	f.store_buffer(raw)
	f.close()
	var cyl := FileAccess.open(dir.path_join("cyl.toml"), FileAccess.WRITE)
	cyl.store_string('[grid]\nkind = "albedo"\nprojection = "simple_cylindrical"\nsphere_radius_m = 1737400.0\n'
			+ 'dn_scale_m = 6.103515625e-05\nheights = "a.i16"\nlines = 3\nsamples = 4\npixels_per_degree = 10.0\n'
			+ "zero_line = 100.5\nzero_sample = -200.5\n")
	cyl.close()
	var a := LunarSiteService.read_albedo(dir.path_join("cyl.toml"))
	_check(bool(a["ok"]), "synthetic cylindrical albedo reads")
	if bool(a["ok"]):
		var image: Image = a["image"]
		_check(image.get_width() == 4 and image.get_height() == 3, "image size")
		_check(is_equal_approx(image.get_pixel(1, 0).r, 1.0 + 100.0 / 16384.0), "value x dn_scale")
		var b: Vector4 = a["bounds"]
		# line 0 is latitude 10.05 (zero_line 100.5 / 10 ppd): the extent reaches half a pixel beyond the first / last centre.
		_check(is_equal_approx(b.y, deg_to_rad(10.1)) and is_equal_approx(b.x, deg_to_rad(9.8)),
				"cylindrical latitude bounds (%s)" % str(b))
		_check(is_equal_approx(b.z, deg_to_rad(20.0)) and is_equal_approx(b.w, deg_to_rad(20.4)),
				"cylindrical longitude bounds (%s)" % str(b))
		_check(a["grid"] == Vector4(100.5, -200.5, 10.0, 1.0), "cylindrical grid uniform")
	var polar := FileAccess.open(dir.path_join("polar.toml"), FileAccess.WRITE)
	polar.store_string('[grid]\nkind = "albedo"\nprojection = "polar_stereographic_south"\nsphere_radius_m = 1737400.0\n'
			+ 'dn_scale_m = 6.103515625e-05\nheights = "a.i16"\nlines = 3\nsamples = 4\nscale_m = 100.0\n'
			+ "pole_line = 51.0\npole_sample = 52.0\nfirst_line = 50\nfirst_sample = 50\n")
	polar.close()
	a = LunarSiteService.read_albedo(dir.path_join("polar.toml"))
	_check(bool(a["ok"]) and a["grid"] == Vector4(1.0, 2.0, 100.0, 3.0), "polar grid uniform (pole inside the tile)")
	if bool(a["ok"]):
		var b: Vector4 = a["bounds"]
		_check(is_equal_approx(b.x, -0.5 * PI) and b.y > -0.5 * PI and b.y < -0.5 * PI + 0.001,
				"polar bounds reach the pole, tile smaller than 0.001 rad (%s)" % str(b))
	var height := FileAccess.open(dir.path_join("height.toml"), FileAccess.WRITE)
	height.store_string('[grid]\nkind = "height"\nheights = "a.i16"\nlines = 3\nsamples = 4\n')
	height.close()
	_check(not bool(LunarSiteService.read_albedo(dir.path_join("height.toml"))["ok"]), "a height tile is refused as albedo")
	_check(not bool(LunarSiteService.read_albedo(dir.path_join("missing.toml"))["ok"]), "a missing file reads as not ok")


func _remove_tree(dir: String) -> void:
	for sub in DirAccess.get_directories_at(dir):
		_remove_tree(dir.path_join(sub))
	for name: String in DirAccess.get_files_at(dir):
		DirAccess.remove_absolute(dir.path_join(name))
	DirAccess.remove_absolute(dir)


func _check(condition: bool, label: String) -> void:
	if condition:
		print("  ok   ", label)
	else:
		print("  FAIL ", label)
		_failures += 1
