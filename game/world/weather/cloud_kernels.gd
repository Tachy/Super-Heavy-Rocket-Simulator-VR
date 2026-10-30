class_name CloudKernels
extends RefCounted
## The cloud compute kernels, built off the main and the render thread (the fast start, 2026-10-30): a set of kernels
## (name, preprocessor defines, include files, with globals - CloudShaderPort.build's arguments) is ported and compiled
## on WorkerThreadPool workers, one task per kernel (in parallel), while the loading screen keeps animating: glslang to
## SPIR-V, then the RenderingDevice's driver binary (shader_compile_binary_from_spirv: on D3D12 the SPIR-V -> DXIL
## translation, as slow as glslang). The binary is kept on disk under CACHE_DIR, named by the SHA-256 of the final GLSL
## source (the defines are in its header), the Godot version and build hash, the rendering driver and the device: a
## later start reads it and skips both steps. The owner (CloudEffect, CloudBaked, CloudShadowEffect) polls done() and
## then creates its shaders from it on the render thread (shader_create_from_bytecode); when a cached binary is rejected
## there (a truncated or corrupt file), the owner calls rebuild_rejected() and polls again. Without a RenderingDevice
## (headless) nothing is built and done() is true at once.
## The build's state lives in a Job the workers' tasks hold a strong reference to (a lambda's captured RefCounted), so
## a task never works on a freed object (quitting during a build: the scene tree goes before the engine waits for the
## workers); a running group is waited for when this object is freed (_notification).

## The compiled kernels; environment STARSIM_CLOUD_SHADER_CACHE=0 neither reads nor writes them (measurements).
const CACHE_DIR := "user://shader_cache/clouds"
## Cache hygiene (once per process, on the first worker, _prune_once): a binary not used for CACHE_MAX_AGE_S is
## removed (a cache hit older than CACHE_REFRESH_AGE_S is written anew, so the file time is the last use) - every
## shader edit leaves the old binaries behind and tracking which keys are live is not worth it: a wrongly dropped one
## costs one recompile on the workers. A temporary file older than TEMP_MAX_AGE_S is a killed process's (a live one's
## exists for milliseconds).
const CACHE_MAX_AGE_S := 30 * 86400
const CACHE_REFRESH_AGE_S := 86400
const TEMP_MAX_AGE_S := 3600
static var cache_enabled := OS.get_environment("STARSIM_CLOUD_SHADER_CACHE") != "0"
static var _pruned := false
static var _prune_mutex := Mutex.new()

var label := ""
var _job: Job
var _group := -1
var _group_mutex := Mutex.new()
var _start_us := 0
var _done_us := -1
var _rebuilt := false  ## a rejected cached binary's rebuild has run (once per set)


## The build's state, shared with the workers' tasks (they keep it alive).
class Job:
	extends RefCounted
	var label := ""
	var specs: Array = []  ## [name, defines, files, with_globals] per kernel
	var binary: Array = []  ## the driver binary per kernel (empty: failed)
	var ports: Array = []  ## CloudShaderPort per kernel
	var missed: Array[bool] = []  ## per kernel: compiled (not read from the cache)
	var read_cache: Array[bool] = []  ## per kernel: the cache may be read (off for a rejected binary's rebuild)
	var rejected: Array[bool] = []  ## per kernel: its cached binary was rejected by the RenderingDevice
	var mutex := Mutex.new()  ## guards the results
	var rd: RenderingDevice  ## the compilers' entry point (both touch no device state; Godot's ShaderRD uses them on workers)
	var cache_suffix := ""  ## the version, driver and device in the cache key
	var built_us := 0  ## when the last worker finished (the build's own time, not the first poll's)
	## The include files' text (path -> code; read-only), loaded on the creating thread: a worker's load() can wait for
	## the main thread (ResourceLoader hands a resource's changed-connections to the main thread's queue and waits) -
	## the cause of the deadlocks of a main- / render-thread wait for the workers (switchover, 2026-10-30).
	var codes := {}

	func _init(label_text: String, kernel_specs: Array, device: RenderingDevice) -> void:
		label = label_text
		specs = kernel_specs
		rd = device
		if rd != null:
			for spec: Array in specs:
				CloudShaderPort.include_codes(spec[2], codes)
		codes.make_read_only()
		for list: Array in [binary, ports, missed, read_cache, rejected]:
			list.resize(specs.size())
		read_cache.fill(true)
		rejected.fill(false)
		if rd != null:
			var version := Engine.get_version_info()
			cache_suffix = "\n%s\n%s\n%s\n%s" % [version["string"], version["hash"],
					RenderingServer.get_current_rendering_driver_name(), rd.get_device_name()]

	## Worker: ports kernel i, reads its binary from the cache or compiles it (and stores it).
	func build(i: int) -> void:
		CloudKernels._prune_once()
		var spec: Array = specs[i]
		var port := CloudShaderPort.build(spec[1], spec[2], spec[3], codes)
		var path := CloudKernels.CACHE_DIR.path_join((port.source + cache_suffix).sha256_text() + ".bin")
		mutex.lock()
		var may_read := read_cache[i]
		mutex.unlock()
		var binary_i := PackedByteArray()
		if CloudKernels.cache_enabled and FileAccess.file_exists(path):
			if may_read:
				binary_i = FileAccess.get_file_as_bytes(path)
				if not binary_i.is_empty() \
						and Time.get_unix_time_from_system() - FileAccess.get_modified_time(path) > CACHE_REFRESH_AGE_S:
					CloudKernels._write_cached(path, binary_i)  # (its file time: the last use, for the pruning)
			else:
				DirAccess.remove_absolute(path)  # the rejected file (rewritten below when the compile succeeds)
		var missed_i := binary_i.is_empty()
		if missed_i:
			var src := RDShaderSource.new()
			src.language = RenderingDevice.SHADER_LANGUAGE_GLSL
			src.source_compute = port.source
			var code := rd.shader_compile_spirv_from_source(src, false)
			if code.compile_error_compute != "":
				push_error("CloudKernels %s %s: %s" % [label, spec[0], code.compile_error_compute])
			else:
				binary_i = rd.shader_compile_binary_from_spirv(code, spec[0])
				if CloudKernels.cache_enabled and not binary_i.is_empty():
					CloudKernels._write_cached(path, binary_i)
		mutex.lock()
		binary[i] = binary_i
		ports[i] = port
		missed[i] = missed_i
		built_us = maxi(built_us, Time.get_ticks_usec())
		mutex.unlock()


## Starts building `specs` ([name, defines (PackedStringArray), files (PackedStringArray), with_globals] each).
func _init(specs: Array, label_text := "") -> void:
	label = label_text
	_start_us = Time.get_ticks_usec()
	var rd := RenderingServer.get_rendering_device()
	_job = Job.new(label, specs, rd)
	if specs.is_empty() or rd == null:
		_done_us = _start_us
		return
	_group = _start(_job, PackedInt32Array(range(specs.size())))


## The workers' group over the kernels `indices` of `job`. Static, so the task's lambda captures the job only (a
## strong reference) and not this object.
static func _start(job: Job, indices: PackedInt32Array) -> int:
	return WorkerThreadPool.add_group_task(func(k: int) -> void: job.build(indices[k]), indices.size(), -1, false,
			"cloud kernels " + job.label)


## True once every kernel is built (or failed); the first call that sees it releases the worker group. Poll it: the
## frames (the loading screen, the game) must not stall on a build.
func done() -> bool:
	_group_mutex.lock()
	var finished := _group < 0
	if not finished and WorkerThreadPool.is_group_task_completed(_group):
		WorkerThreadPool.wait_for_group_task_completion(_group)
		_group = -1
		_done_us = Time.get_ticks_usec()
		finished = true
		_job.mutex.lock()
		var built_us := _job.built_us
		_job.mutex.unlock()
		print("CloudKernels %s: %d kernels built in %.2f s (%d compiled, %d from the cache)" % [label,
				_job.specs.size(), (built_us - _start_us) / 1.0e6, misses(), _job.specs.size() - misses()])
	_group_mutex.unlock()
	return finished


func _notification(what: int) -> void:
	# The owner went (quitting, a check's end) with the group never released: wait for it here - the one blocking wait.
	# A group still running at the engine's exit was never released and hung the exit (cold build, quit at frame 5:
	# no CPU, never ends; waited here it exits after the running kernels, ~1 s). Safe since the workers load nothing
	# (Job.codes): the earlier deadlocks of a blocking wait were most likely a worker's load() waiting for the main
	# thread (with it gone the workers progressed through an 8 s main-thread sleep, 2026-10-30).
	if what == NOTIFICATION_PREDELETE and _group >= 0:
		WorkerThreadPool.wait_for_group_task_completion(_group)


## The kernels compiled (not read from the cache).
func misses() -> int:
	_job.mutex.lock()
	var n := _job.missed.count(true)
	_job.mutex.unlock()
	return n


func names() -> PackedStringArray:
	var result := PackedStringArray()
	for spec: Array in _job.specs:
		result.append(spec[0])
	return result


## The kernel's port (its globals block and samplers, CloudShaderPort) once done; null for an unknown name.
func port(name: String) -> CloudShaderPort:
	var i := _index(name)
	return _job.ports[i] if i >= 0 else null


## Creates the kernel's shader on `rd` (render thread); an invalid RID when it failed (a binary read from the cache
## and rejected is marked for rebuild_rejected).
func create_shader(rd: RenderingDevice, name: String) -> RID:
	var i := _index(name)
	if i < 0:
		return RID()
	_job.mutex.lock()
	var binary: PackedByteArray = _job.binary[i]
	var cached := not _job.missed[i]
	_job.mutex.unlock()
	if binary.is_empty():
		return RID()
	var shader := rd.shader_create_from_bytecode(binary)
	if not shader.is_valid():
		if cached:
			_job.mutex.lock()
			_job.rejected[i] = true
			_job.mutex.unlock()
		push_error("CloudKernels %s: shader %s not created%s" % [label, name, " (cached binary)" if cached else ""])
	return shader


## Render thread, after a failed create_shader: when a cached binary was rejected (a truncated or corrupt file, a
## container the engine no longer reads), every kernel of this set read from the cache is built again - compiled, its
## file replaced - once per set. True when that rebuild runs: free the shaders already created, poll done() and create
## them anew. False: the failure is final.
func rebuild_rejected() -> bool:
	_job.mutex.lock()
	var any_rejected := _job.rejected.has(true)
	var indices := PackedInt32Array()
	if any_rejected and not _rebuilt:
		for i in range(_job.specs.size()):
			if not _job.missed[i]:
				indices.append(i)
				_job.read_cache[i] = false
				_job.rejected[i] = false
	_job.mutex.unlock()
	if not any_rejected or _rebuilt or indices.is_empty():
		return false
	_rebuilt = true
	print("CloudKernels %s: a cached binary was rejected - rebuilding %d kernels without the cache" % [label,
			indices.size()])
	_group_mutex.lock()
	_start_us = Time.get_ticks_usec()
	_done_us = -1
	_group = _start(_job, indices)
	_group_mutex.unlock()
	return true


func _index(name: String) -> int:
	for i in range(_job.specs.size()):
		if _job.specs[i][0] == name:
			return i
	push_error("CloudKernels %s: no kernel %s" % [label, name])
	return -1


## Written to a unique temporary name and renamed: parallel processes (checks) never read a half-written file; a write
## error (disk full, I/O) or a short file never becomes the cache entry.
static func _write_cached(path: String, bytes: PackedByteArray) -> void:
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	var temp := "%s.%d.%d.tmp" % [path, OS.get_process_id(), Time.get_ticks_usec()]
	var file := FileAccess.open(temp, FileAccess.WRITE)
	if file == null:
		return
	var ok := file.store_buffer(bytes) and file.get_error() == OK
	file.close()
	var check := FileAccess.open(temp, FileAccess.READ)
	ok = ok and check != null and check.get_length() == bytes.size()
	check = null  # (closed before the rename)
	if not ok or DirAccess.rename_absolute(temp, path) != OK:
		DirAccess.remove_absolute(temp)


## Cache hygiene, once per process (the first worker; see CACHE_MAX_AGE_S): stale binaries and orphaned temporary files.
static func _prune_once() -> void:
	_prune_mutex.lock()
	var first := not _pruned
	_pruned = true
	_prune_mutex.unlock()
	if not first or not cache_enabled:
		return
	var dir := DirAccess.open(CACHE_DIR)
	if dir == null:
		return
	var now := Time.get_unix_time_from_system()
	for file in dir.get_files():
		var limit := TEMP_MAX_AGE_S if file.ends_with(".tmp") else CACHE_MAX_AGE_S if file.ends_with(".bin") else -1
		var path := CACHE_DIR.path_join(file)
		if limit > 0 and now - FileAccess.get_modified_time(path) > limit:
			DirAccess.remove_absolute(path)
