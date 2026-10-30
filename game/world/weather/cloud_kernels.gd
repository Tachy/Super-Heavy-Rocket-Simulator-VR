class_name CloudKernels
extends RefCounted
## The cloud compute kernels, built off the main and the render thread (the fast start, 2026-10-30): a set of kernels
## (name, preprocessor defines, include files, with globals - CloudShaderPort.build's arguments) is ported and compiled
## on WorkerThreadPool workers, one task per kernel (in parallel), while the loading screen keeps animating: glslang to
## SPIR-V, then the RenderingDevice's driver binary (shader_compile_binary_from_spirv: on D3D12 the SPIR-V -> DXIL
## translation, as slow as glslang). The binary is kept on disk under CACHE_DIR, named by the SHA-256 of the final GLSL
## source (the defines are in its header), the Godot version, the rendering driver and the device: a later start reads
## it and skips both steps. The owner (CloudEffect, CloudBaked, CloudShadowEffect) polls done() and then creates its
## shaders from it on the render thread (shader_create_from_bytecode). Without a RenderingDevice (headless) nothing is
## built and done() is true at once.

## The compiled kernels; environment STARSIM_CLOUD_SHADER_CACHE=0 neither reads nor writes them (measurements).
const CACHE_DIR := "user://shader_cache/clouds"
static var cache_enabled := OS.get_environment("STARSIM_CLOUD_SHADER_CACHE") != "0"

var label := ""
var _specs: Array = []  ## [name, defines, files, with_globals] per kernel
var _binary: Array = []  ## the driver binary per kernel (empty: failed)
var _ports: Array = []  ## CloudShaderPort per kernel
var _missed: Array[bool] = []  ## per kernel: compiled (not read from the cache)
var _results_mutex := Mutex.new()
var _rd: RenderingDevice  ## the compilers' entry point (both touch no device state; Godot's ShaderRD uses them on workers)
var _cache_suffix := ""  ## the version, driver and device in the cache key
var _group := -1
var _group_mutex := Mutex.new()
var _start_us := 0
var _done_us := -1
var _built_us := 0  ## when the last worker finished (the build's own time, not the first poll's)


## Starts building `specs` ([name, defines (PackedStringArray), files (PackedStringArray), with_globals] each).
func _init(specs: Array, label_text := "") -> void:
	label = label_text
	_specs = specs
	_binary.resize(specs.size())
	_ports.resize(specs.size())
	_missed.resize(specs.size())
	_start_us = Time.get_ticks_usec()
	_rd = RenderingServer.get_rendering_device()
	if specs.is_empty() or _rd == null:
		_done_us = _start_us
		return
	_cache_suffix = "\n%s\n%s\n%s" % [Engine.get_version_info()["string"],
			RenderingServer.get_current_rendering_driver_name(), _rd.get_device_name()]
	_group = WorkerThreadPool.add_group_task(_build, specs.size(), -1, false, "cloud kernels " + label)


## True once every kernel is built (or failed); the first call that sees it releases the worker group. Poll it - never
## block on the workers from the main or the render thread (that deadlocked them, 2026-10-30).
func done() -> bool:
	_group_mutex.lock()
	var finished := _group < 0
	if not finished and WorkerThreadPool.is_group_task_completed(_group):
		WorkerThreadPool.wait_for_group_task_completion(_group)
		_group = -1
		_done_us = Time.get_ticks_usec()
		finished = true
		print("CloudKernels %s: %d kernels built in %.2f s (%d compiled, %d from the cache)" % [label, _specs.size(),
				(_built_us - _start_us) / 1.0e6, misses(), _specs.size() - misses()])
	_group_mutex.unlock()
	return finished


func _notification(what: int) -> void:
	# A finished build never polled (the owner went): its group released. A running one is left (blocking on the
	# workers from the main or the render thread deadlocked them).
	if what == NOTIFICATION_PREDELETE and _group >= 0 and WorkerThreadPool.is_group_task_completed(_group):
		WorkerThreadPool.wait_for_group_task_completion(_group)


## The kernels compiled (not read from the cache).
func misses() -> int:
	_results_mutex.lock()
	var n := _missed.count(true)
	_results_mutex.unlock()
	return n


func names() -> PackedStringArray:
	var result := PackedStringArray()
	for spec: Array in _specs:
		result.append(spec[0])
	return result


## The kernel's port (its globals block and samplers, CloudShaderPort) once done.
func port(name: String) -> CloudShaderPort:
	return _ports[_index(name)]


## Creates the kernel's shader on `rd` (render thread); an invalid RID when it failed.
func create_shader(rd: RenderingDevice, name: String) -> RID:
	var binary: PackedByteArray = _binary[_index(name)]
	if binary.is_empty():
		return RID()
	var shader := rd.shader_create_from_bytecode(binary)
	if not shader.is_valid():
		push_error("CloudKernels %s: shader %s not created" % [label, name])
	return shader


func _index(name: String) -> int:
	for i in range(_specs.size()):
		if _specs[i][0] == name:
			return i
	push_error("CloudKernels %s: no kernel %s" % [label, name])
	return -1


## Worker: ports kernel i, reads its binary from the cache or compiles it (and stores it).
func _build(i: int) -> void:
	var spec: Array = _specs[i]
	var port := CloudShaderPort.build(spec[1], spec[2], spec[3])
	var path := CACHE_DIR.path_join((port.source + _cache_suffix).sha256_text() + ".bin")
	var binary := FileAccess.get_file_as_bytes(path) if cache_enabled and FileAccess.file_exists(path) \
			else PackedByteArray()
	var missed := binary.is_empty()
	if missed:
		var src := RDShaderSource.new()
		src.language = RenderingDevice.SHADER_LANGUAGE_GLSL
		src.source_compute = port.source
		var code := _rd.shader_compile_spirv_from_source(src, false)
		if code.compile_error_compute != "":
			push_error("CloudKernels %s %s: %s" % [label, spec[0], code.compile_error_compute])
		else:
			binary = _rd.shader_compile_binary_from_spirv(code, spec[0])
			if cache_enabled and not binary.is_empty():
				_write_cached(path, binary)
	_results_mutex.lock()
	_binary[i] = binary
	_ports[i] = port
	_missed[i] = missed
	_built_us = maxi(_built_us, Time.get_ticks_usec())
	_results_mutex.unlock()


## Written to a unique temporary name and renamed: parallel processes (checks) never read a half-written file.
static func _write_cached(path: String, bytes: PackedByteArray) -> void:
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	var temp := "%s.%d.%d.tmp" % [path, OS.get_process_id(), Time.get_ticks_usec()]
	var file := FileAccess.open(temp, FileAccess.WRITE)
	if file == null:
		return
	file.store_buffer(bytes)
	file.close()
	if DirAccess.rename_absolute(temp, path) != OK:
		DirAccess.remove_absolute(temp)
