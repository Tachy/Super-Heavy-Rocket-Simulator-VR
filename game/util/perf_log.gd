class_name PerfLog
extends Node
## Opt-in frame-time log of a real session (desktop or VR): `-- --perf-log` (e.g. `Start-VR.cmd --perf-log`).
## One CSV row per wall-clock second in data/perf/perf_<date>_<time>.csv: frame interval percentiles, the root
## viewport's measured CPU / GPU render time (in VR the headset's stereo frame), the camera's altitude and the cloud
## layer's state, so stalls can be matched to the flight phase (e.g. flying through a cloud deck). Frames whose
## interval exceeds SPIKE_MS also get their own row (kind "spike"). script_max_ms: the main thread's _process work
## (from SceneTree.process_frame to this node, which runs last), the CPU side a GPU-light frame can still miss on;
## main_max_ms: process_frame to RenderingServer.frame_pre_draw (also the deferred calls and the UI's _draw); then
## one column per FrameProfiler section (its per-frame total, max over the second). A spike row carries the previous
## frame's main / section times: the long interval ends the frame that did that work. The cloud view cache: the tiles
## its last plan filled, the cube's fraction at the full level, the last measured fill interval (GPU ms).

const ARG := "--perf-log"
const PROJECTION_NEAR_M := 0.2  ## the site view's near plane (OrbitCamera)
const PROJECTION_FAR_M := 3.0e7  ## OrbitCamera.far
const PROJECTION_WAIT_FRAMES := 180  ## [E] 2 s at 90 Hz: the session runs and the head is tracked
const PROJECTION_RETRY_FRAMES := 30  ## [E]
const PROJECTION_MAX_ATTEMPTS := 40  ## [E] about 13 s of retries
const SPIKE_MS := 22.0  ## [E] two 90 Hz frames: a frame the headset certainly had to reproject

var _projection_interface: XRInterface  ## set while the headset projections are still to be recorded
var _projection_path := ""
var _projection_frames := 0
var _projection_attempts := 0
var _projection_last_reason := ""
var _world: Node
var _file: FileAccess
var _vp: RID
var _start_msec := 0
var _second_start_msec := 0
var _intervals: PackedFloat64Array = []
var _gpu_sum := 0.0
var _gpu_max := 0.0
var _cpu_max := 0.0
var _process_max := 0.0
var _frame_start_usec := 0
var _main_max := 0.0
var _last_main_ms := 0.0
var _section_max := {}  ## section -> max ms this second
var _last_sections := {}  ## the previous frame's section totals (usec)


static func requested() -> bool:
	return OS.get_cmdline_user_args().has(ARG)


func _init(world: Node) -> void:
	_world = world
	name = "PerfLog"
	process_mode = Node.PROCESS_MODE_ALWAYS
	process_priority = 1000  # after the world's update: the row sees this frame's state


func _ready() -> void:
	var dir := Paths.data_dir().path_join("perf")
	DirAccess.make_dir_recursive_absolute(dir)
	var stamp := Time.get_datetime_string_from_system().replace(":", "").replace("-", "").replace("T", "_")
	var path := dir.path_join("perf_%s.csv" % stamp)
	_file = FileAccess.open(path, FileAccess.WRITE)
	if _file == null:
		push_warning("[PerfLog] cannot write %s" % path)
		set_process(false)
		return
	_file.store_line("kind,t_s,frames,frame_p50_ms,frame_p99_ms,frame_max_ms,gpu_mean_ms,gpu_max_ms,cpu_render_max_ms,"
			+ "script_max_ms,altitude_m,cloud_far,cloud_uploading,cloud_last_upload_ms,draw_calls,primitives,vram_mb,xr,"
			+ "main_max_ms,cloud_image_creations,cloud_output_swaps,cloud_low_w,cloud_low_h,"
			+ "cache_tiles_filled,cache_full_fraction,cache_fill_ms,"
			+ ",".join(FrameProfiler.SECTIONS.map(func(s: StringName) -> String: return "sec_%s_max_ms" % s)))
	_vp = get_viewport().get_viewport_rid()
	RenderingServer.viewport_set_measure_render_time(_vp, true)
	get_tree().process_frame.connect(func() -> void: _frame_start_usec = Time.get_ticks_usec())
	RenderingServer.frame_pre_draw.connect(_on_frame_pre_draw)
	FrameProfiler.enabled = true
	_start_msec = Time.get_ticks_msec()
	_second_start_msec = _start_msec
	print("[PerfLog] writing ", path)
	var xr_interface := XRServer.primary_interface
	if xr_interface != null and xr_interface.is_initialized():
		# the per-eye size the runtime asks for: the GPU time scales with it (the desktop proxy is 1440p)
		print("[PerfLog] XR render target per eye: ", xr_interface.get_render_target_size(),
				", display refresh: ", xr_interface.get("display_refresh_rate"))
		_projection_path = dir.path_join("xr_projections_%s.json" % stamp)
		_projection_interface = xr_interface


## Tries to record once the head is tracked; called every frame until it succeeded (or gave up).
func _try_write_projections() -> void:
	_projection_frames += 1
	if _projection_frames < PROJECTION_WAIT_FRAMES or _projection_frames % PROJECTION_RETRY_FRAMES != 0:
		return
	var head := XRServer.get_hmd_transform()
	var reason := ""
	var record := {}
	if not XrProjectionRecord.head_pose_is_tracked(head):
		reason = "head pose not tracked yet"
	else:
		record = XrProjectionRecord.record(_projection_interface, PROJECTION_NEAR_M, PROJECTION_FAR_M, head)
		reason = XrProjectionRecord.validate(record)
	if reason != "":
		if reason != _projection_last_reason:
			print("[PerfLog] XR projections not recorded (retrying): ", reason)
			_projection_last_reason = reason
		_projection_attempts += 1
		if _projection_attempts >= PROJECTION_MAX_ATTEMPTS:
			push_warning("[PerfLog] XR projections: giving up (%s)" % reason)
			_projection_interface = null
		return
	var file := FileAccess.open(_projection_path, FileAccess.WRITE)
	if file == null:
		push_warning("[PerfLog] cannot write %s" % _projection_path)
	else:
		file.store_string(JSON.stringify(record, "	"))
		file.close()
		print("[PerfLog] XR projections written to ", _projection_path)
	_projection_interface = null


func _process(delta: float) -> void:
	if _projection_interface != null:
		_try_write_projections()
	var interval_ms := delta * 1000.0
	var gpu := RenderingServer.viewport_get_measured_render_time_gpu(_vp)
	var cpu := RenderingServer.viewport_get_measured_render_time_cpu(_vp)
	var process_ms := (Time.get_ticks_usec() - _frame_start_usec) / 1000.0 if _frame_start_usec > 0 else 0.0
	_intervals.append(interval_ms)
	_gpu_sum += gpu
	_gpu_max = maxf(_gpu_max, gpu)
	_cpu_max = maxf(_cpu_max, cpu)
	_process_max = maxf(_process_max, process_ms)
	var now := Time.get_ticks_msec()
	if interval_ms > SPIKE_MS:
		var spike_sections := {}
		for section: StringName in _last_sections:
			spike_sections[section] = int(_last_sections[section]) / 1000.0
		_write_row("spike", now, PackedFloat64Array([interval_ms]), gpu, gpu, cpu, process_ms, _last_main_ms, spike_sections)
	if now - _second_start_msec >= 1000:
		_write_row("second", now, _intervals, _gpu_sum / _intervals.size(), _gpu_max, _cpu_max, _process_max, _main_max,
				_section_max)
		_intervals = []
		_gpu_sum = 0.0
		_gpu_max = 0.0
		_cpu_max = 0.0
		_process_max = 0.0
		_main_max = 0.0
		_section_max = {}
		_second_start_msec = now
		_file.flush()


## Main thread, after the frame's process and deferred calls (the UI's _draw included), before the draw.
func _on_frame_pre_draw() -> void:
	if _frame_start_usec > 0:
		_last_main_ms = (Time.get_ticks_usec() - _frame_start_usec) / 1000.0
		_main_max = maxf(_main_max, _last_main_ms)
	_last_sections = FrameProfiler.take_frame()
	for section: StringName in _last_sections:
		_section_max[section] = maxf(float(_section_max.get(section, 0.0)), int(_last_sections[section]) / 1000.0)


func _exit_tree() -> void:
	FrameProfiler.enabled = false
	if _file != null:
		_file.close()


func _write_row(kind: String, now_msec: int, intervals: PackedFloat64Array, gpu_mean: float, gpu_max: float,
		cpu_max: float, process_max: float, main_max: float, sections: Dictionary) -> void:
	var sorted := intervals.duplicate()
	sorted.sort()
	var n := sorted.size()
	var clouds: CloudLayer = _world.clouds
	var cloud_state := clouds.state()
	var cache := clouds.view_cache_stats()  # the view cache's last plan and fill (cloud view cache Task 4)
	var xr := get_node_or_null("/root/XrManager")
	_file.store_line("%s,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%d,%d,%.2f,%d,%d,%.0f,%d" % [
			kind, (now_msec - _start_msec) / 1000.0, n, sorted[n / 2], sorted[mini(n - 1, int(n * 0.99))], sorted[n - 1],
			gpu_mean, gpu_max, cpu_max, process_max, _world.atmosphere.camera_altitude_m, int(clouds.far_enabled),
			int(cloud_state["uploading"]), clouds.last_upload_ms,
			Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME),
			Performance.get_monitor(Performance.RENDER_TOTAL_PRIMITIVES_IN_FRAME),
			Performance.get_monitor(Performance.RENDER_VIDEO_MEM_USED) / 1048576.0,
			int(xr != null and xr.xr_active)]
			+ ",%.2f" % main_max
			+ ",%d,%d,%d,%d" % [clouds.effect.image_creations, clouds.effect.output_generation,
					clouds.effect.centre_low.x, clouds.effect.centre_low.y]
			+ ",%d,%.4f,%.3f" % [cache["tiles_filled"], cache["full_fraction"], cache["fill_ms"]]
			+ "".join(FrameProfiler.SECTIONS.map(func(s: StringName) -> String: return ",%.2f" % float(sections.get(s, 0.0)))))
