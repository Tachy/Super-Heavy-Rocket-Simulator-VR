class_name PerfLog
extends Node
## Opt-in frame-time log of a real session (desktop or VR): `-- --perf-log` (e.g. `Start-VR.cmd --perf-log`).
## One CSV row per wall-clock second in data/perf/perf_<date>_<time>.csv: frame interval percentiles, the root
## viewport's measured CPU / GPU render time (in VR the headset's stereo frame), the camera's altitude and the cloud
## layer's state, so stalls can be matched to the flight phase (e.g. flying through a cloud deck). Frames whose
## interval exceeds SPIKE_MS also get their own row (kind "spike").

const ARG := "--perf-log"
const SPIKE_MS := 22.0  ## [E] two 90 Hz frames: a frame the headset certainly had to reproject

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
			+ "process_max_ms,altitude_m,cloud_far,cloud_uploading,cloud_last_upload_ms,draw_calls,primitives,vram_mb,xr")
	_vp = get_viewport().get_viewport_rid()
	RenderingServer.viewport_set_measure_render_time(_vp, true)
	_start_msec = Time.get_ticks_msec()
	_second_start_msec = _start_msec
	print("[PerfLog] writing ", path)


func _process(delta: float) -> void:
	var interval_ms := delta * 1000.0
	var gpu := RenderingServer.viewport_get_measured_render_time_gpu(_vp)
	var cpu := RenderingServer.viewport_get_measured_render_time_cpu(_vp)
	var process_ms := Performance.get_monitor(Performance.TIME_PROCESS) * 1000.0
	_intervals.append(interval_ms)
	_gpu_sum += gpu
	_gpu_max = maxf(_gpu_max, gpu)
	_cpu_max = maxf(_cpu_max, cpu)
	_process_max = maxf(_process_max, process_ms)
	var now := Time.get_ticks_msec()
	if interval_ms > SPIKE_MS:
		_write_row("spike", now, PackedFloat64Array([interval_ms]), gpu, gpu, cpu, process_ms)
	if now - _second_start_msec >= 1000:
		_write_row("second", now, _intervals, _gpu_sum / _intervals.size(), _gpu_max, _cpu_max, _process_max)
		_intervals = []
		_gpu_sum = 0.0
		_gpu_max = 0.0
		_cpu_max = 0.0
		_process_max = 0.0
		_second_start_msec = now
		_file.flush()


func _exit_tree() -> void:
	if _file != null:
		_file.close()


func _write_row(kind: String, now_msec: int, intervals: PackedFloat64Array, gpu_mean: float, gpu_max: float,
		cpu_max: float, process_max: float) -> void:
	var sorted := intervals.duplicate()
	sorted.sort()
	var n := sorted.size()
	var clouds: CloudLayer = _world.clouds
	var cloud_state := clouds.state()
	var xr := get_node_or_null("/root/XrManager")
	_file.store_line("%s,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%d,%d,%.2f,%d,%d,%.0f,%d" % [
			kind, (now_msec - _start_msec) / 1000.0, n, sorted[n / 2], sorted[mini(n - 1, int(n * 0.99))], sorted[n - 1],
			gpu_mean, gpu_max, cpu_max, process_max, _world.atmosphere.camera_altitude_m, int(clouds.far_enabled),
			int(cloud_state["uploading"]), clouds.last_upload_ms,
			Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME),
			Performance.get_monitor(Performance.RENDER_TOTAL_PRIMITIVES_IN_FRAME),
			Performance.get_monitor(Performance.RENDER_VIDEO_MEM_USED) / 1048576.0,
			int(xr != null and xr.xr_active)])
