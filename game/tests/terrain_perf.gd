# Terrain renderer performance run (needs a GPU and a desktop window, no --headless):
# godot --path game --script res://tests/terrain_perf.gd [-- --utc=2024-04-10T18:00:00 --warp=1 --descent_s=60
#       --pan_s=20 --cape_s=20 --shots=<dir> --spikes --weather=<abs path .wthr> --cloud_quality=desktop|vr
#       --cloud_scattering=table|closed --render_scale=<f>]
# --render_scale=<f>: the root viewport renders at f x the window per axis (bilinear, Viewport.scaling_3d_scale): the
# VR proxy is 1.65 (2560 x 1440 x 1.65^2 = 20 MP, the two 3530 x 2834 eyes of the user's headset at Pimax FSR 1.5;
# docs/subsystems/vr-performance.md).
# Phases: "descent" - a ground camera above OLP-1 descends from 400 km to 2 m in descent_s (eye height logarithmic in
# time, looking 20 deg down); "pan_low" - 360 deg pan at 2 m eye height on the plateau PAN_LOW_OFFSET_M from the mount
# (free view, not blocked by the mount); "pan_high" - 360 deg pan PAN_HIGH_EYE_M above the pad, looking 10 deg down (the
# whole site and the coast); "cape" - Tab-style jump to LC-39A (one frame) at 2 m, held cape_s. The clock runs (warp 1:
# the Earth turns under the floating origin, every node moves each frame, as in play). Vsync off, 2560 x 1440.
# Logs per second: frame time p50 / p99 / max, the main-thread split of the slowest frame (process / the rest: render,
# present), the terrain's update time, GPU time, terrain nodes drawn / loading, uploads, video memory; then per phase
# and overall summaries with the desktop proxy gate [E]: p99 <= 8 ms (11 ms with clouds, --weather) and max <= 16 ms
# outside the jump frame.
# --shots=<dir>: a PNG at the start and half-way through each pan (those frames are left out of the statistics).
# --spikes[=ms]: every frame above ms (default GATE_MAX_MS) listed at the end (process / rest split, the
# RenderingServer draw inside the rest, terrain profile, draw-time pipeline compilations).
# All lines are printed at the end of the run: console output during it stalls the main thread for up to ~20 ms (the
# output pipe of the console wrapper), which was the cause of the "process" spikes this run used to report.
extends SceneTree

const WINDOW_SIZE := Vector2i(2560, 1440)
const START_EYE_HEIGHT_M := 4.0e5
const END_EYE_HEIGHT_M := 2.0
const DESCENT_ELEVATION_DEG := -20.0
## Low pan: on the OLP-1 plateau this far east of the mount (m): open view over the site, the flats and the coast.
const PAN_LOW_OFFSET_M := Vector2(180.0, 0.0)  ## (east, north)
const PAN_HIGH_EYE_M := 50.0
const PAN_HIGH_ELEVATION_DEG := -10.0
## Frames before the run starts (window, roots, first loads settle).
const WARMUP_FRAMES := 30
## [E] desktop proxy for 90 Hz VR (11.1 ms per frame for two eyes): a single 1440p view has to stay well inside it.
## Without clouds p99 <= 8 ms; with clouds on (--weather=<file.wthr>, VR preset) the clouds' budget of +3 ms brings it
## to p99 <= 11 ms (the 90 Hz frame); max <= 16 ms in both.
const GATE_P99_MS := 8.0
const GATE_P99_CLOUDS_MS := 11.0
const GATE_MAX_MS := 16.0
const PAD_SITE := "starbase_olp1"
const CAPE_SITE := "ksc_lc39a"
## --spikes: the viewport's measured CPU / GPU render times of this many frames after a spike (they arrive late).
const SPIKE_FOLLOWUP_FRAMES := 4
const PHASES := ["descent", "pan_low", "pan_high", "cape"]
const METRES_PER_DEG := 111319.49

var _args := {}
var _main: Node
var _world: World
var _probe: _Probe
var _frames := 0
var _t0_us := 0
var _last_us := 0
var _frame_start_us := 0
var _phase := "warmup"
var _phase_start_s := 0.0
var _durations := {}
var _skip_frame := -1  ## a frame left out of the statistics (jump, screenshot)
var _jump_ms := 0.0
var _phase_ms := {}  ## phase -> Array of frame times (ms)
var _phase_max := {}  ## phase -> {drawn, loading, vram_mb}
var _second := {}
var _second_index := 0
var _spikes: PackedStringArray = []
var _pipelines_seen := -1  ## pipeline compilations at draw time (ubershader, the main thread waits) so far
var _gpu_pass_us := {}  ## --gpu_passes: timestamp name -> GPU ns this second
var _gpu_pass_frames := 0
var _log_lines: PackedStringArray = []  ## per-second lines, printed at the end (see the header)
var _spike_ms := 0.0  ## --spikes threshold (ms); 0: off
var _pre_draw_us := 0  ## RenderingServer.frame_pre_draw / frame_post_draw of the last frame
var _post_draw_us := 0
var _spike_followup := 0  ## frames still to report after the last spike
var _pad: Dictionary
var _cape: Dictionary
var _shots_dir := ""
var _shots_taken := {}


class _Probe extends Node:
	## Runs after every other node's _process: frame start -> end_us is the process part, end_us -> next frame the rest.
	var end_us := 0

	func _process(_delta: float) -> void:
		end_us = Time.get_ticks_usec()


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_durations = {"descent": float(_args.get("descent_s", "60")), "pan_low": float(_args.get("pan_s", "20")),
			"pan_high": float(_args.get("pan_s", "20")), "cape": float(_args.get("cape_s", "20"))}
	_shots_dir = _args.get("shots", "")
	if _args.has("spikes"):
		_spike_ms = float(_args["spikes"]) if not String(_args["spikes"]).is_empty() else GATE_MAX_MS
	for phase: String in PHASES:
		_phase_ms[phase] = []
		_phase_max[phase] = {"drawn": 0, "loading": 0, "vram_mb": 0.0}
	_reset_second()
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	Engine.max_fps = 0
	DisplayServer.window_set_size(WINDOW_SIZE)
	if _args.has("render_scale"):
		root.scaling_3d_mode = Viewport.SCALING_3D_MODE_BILINEAR
		root.scaling_3d_scale = float(_args["render_scale"])
	RenderingServer.viewport_set_measure_render_time(root.get_viewport_rid(), true)
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_probe = _Probe.new()
	_probe.process_priority = 1 << 30
	root.add_child(_probe)
	RenderingServer.frame_pre_draw.connect(func() -> void: _pre_draw_us = Time.get_ticks_usec())
	RenderingServer.frame_post_draw.connect(func() -> void: _post_draw_us = Time.get_ticks_usec())


func _process(_delta: float) -> bool:
	var now_us := Time.get_ticks_usec()
	var process_ms := (_probe.end_us - _frame_start_us) * 1.0e-3  # the previous frame's
	var rest_ms := (now_us - _probe.end_us) * 1.0e-3
	_frame_start_us = now_us
	if _frames == 0:
		_setup()
	_frames += 1
	if _frames < WARMUP_FRAMES:
		return false
	if _frames == WARMUP_FRAMES:
		_t0_us = now_us
		_last_us = now_us
		_enter("descent", 0.0)
		return false
	var frame_ms := (now_us - _last_us) * 1.0e-3
	_last_us = now_us
	var t := (now_us - _t0_us) * 1.0e-6
	_record(frame_ms, process_ms, rest_ms)
	if _args.has("gpu_passes"):
		_record_gpu_passes()
	if t >= _second_index + 1.0:
		if _args.has("gpu_passes"):
			_log_gpu_passes(t)
		_log_second()
	return _drive(t)


## --gpu_passes: the GPU time between the RenderingDevice's captured timestamps of the previous frame (ns), summed per
## timestamp name over each second and logged as the mean ms per frame (the top entries). The engine stamps only the
## viewport (vp_begin...); CloudEffect.capture_timestamps adds the cloud passes' (an interval is named by its start).
func _record_gpu_passes() -> void:
	var rd := RenderingServer.get_rendering_device()
	var count := rd.get_captured_timestamps_count()
	for i in range(1, count):
		var name := rd.get_captured_timestamp_name(i - 1)
		var us := float(rd.get_captured_timestamp_gpu_time(i) - rd.get_captured_timestamp_gpu_time(i - 1))
		_gpu_pass_us[name] = float(_gpu_pass_us.get(name, 0.0)) + us
	_gpu_pass_frames += 1


func _log_gpu_passes(t: float) -> void:
	var names := _gpu_pass_us.keys()
	names.sort_custom(func(a: String, b: String) -> bool: return _gpu_pass_us[a] > _gpu_pass_us[b])
	var parts := PackedStringArray()
	for name: String in names.slice(0, 14):
		parts.append("%s %.2f" % [name, _gpu_pass_us[name] / 1.0e6 / maxi(_gpu_pass_frames, 1)])
	_log_lines.append("terrain_perf: gpu passes t=%3d s %s  %s" % [int(t), _phase, ", ".join(parts)])
	_gpu_pass_us.clear()
	_gpu_pass_frames = 0


## --cloud_tune=steps=48,max_steps=96,fine=0.03,light_near=4,light_far=2,divisor=3: overrides single preset values of
## the clouds
## (the shader globals the preset sets) for a tuning run; the game's preset is CloudLayer.PRESETS.
func _tune_clouds(spec: String) -> void:
	var light: Vector2i = ShaderGlobals.value(&"cloud_light_samples")
	for item in spec.split(","):
		var kv := item.split("=")
		match kv[0]:
			"steps": ShaderGlobals.set_value(&"cloud_steps", int(kv[1]))
			"max_steps": ShaderGlobals.set_value(&"cloud_max_steps", int(kv[1]))
			"fine": ShaderGlobals.set_value(&"cloud_fine_step_fraction", float(kv[1]))
			"light_near":
				light.x = int(kv[1])
				ShaderGlobals.set_value(&"cloud_light_march", Vector4(20.0, CloudLayer.light_march_ratio(20.0, 6000.0,
						light.x), 6000.0, 0.0))
			"light_far": light.y = int(kv[1])
			"divisor":
				var preset: Dictionary = CloudLayer.PRESETS[_world.clouds.quality]
				_world.clouds.effect.set_quality(int(kv[1]), float(preset["temporal_weight"]),
						float(preset["clip_gamma"]))
	ShaderGlobals.set_value(&"cloud_light_samples", light)


func _setup() -> void:
	_world = _main.get_node("World")
	_world.clock.warp = float(_args.get("warp", "1"))
	_world.clock.set_time(_world.core.utc_to_tdb_s(_args.get("utc", "2024-04-10T18:00:00")))
	# Weather: off (no downloads in a performance run) unless --weather=<file.wthr> pins that snapshot for the clouds;
	# --cloud_quality=desktop|vr picks the clouds' preset (the gate: vr); --cloud_scattering=table|closed the
	# multiple-scattering model (CloudLayer.ScatteringModel; default the table).
	_world.weather.enabled = false
	if _args.has("cloud_quality"):
		_world.clouds.quality = CloudLayer.Quality.VR if _args["cloud_quality"] == "vr" else CloudLayer.Quality.DESKTOP
	if _args.has("cloud_scattering"):
		_world.clouds.scattering_model = CloudLayer.ScatteringModel.CLOSED_FORM \
				if _args["cloud_scattering"] == "closed" else CloudLayer.ScatteringModel.TABLE
	CloudEffect.capture_timestamps = _args.has("gpu_passes")
	if _args.has("cloud_tune"):  # tuning runs: steps=, max_steps=, fine=, light_near=, light_far=, divisor= (comma list)
		_tune_clouds(String(_args["cloud_tune"]))
	if String(_args.get("weather", "")).ends_with(".wthr"):
		var snapshot: Dictionary = _world.core.weather_load_snapshot(_args["weather"])
		if snapshot.get("ok", false):
			_world.clouds.wait_until_ready()
			_world.clouds.show_now(snapshot, snapshot, 0.0)
		else:
			push_error("terrain_perf: " + str(snapshot.get("error", "")))
	for site: Dictionary in _world.sites:
		if site["id"] == PAD_SITE:
			_pad = site.duplicate()
		elif site["id"] == CAPE_SITE:
			_cape = site.duplicate()
	_pad["eye_height_m"] = START_EYE_HEIGHT_M
	_world.set_ground_site(_pad)
	_world.camera.elevation = deg_to_rad(DESCENT_ELEVATION_DEG)
	_world.camera.azimuth = 0.0
	print("terrain_perf: window %s, render scale %.2f (internal %s), renderer %s, GPU %s, warp %s" % [
			DisplayServer.window_get_size(), root.scaling_3d_scale,
			Vector2i(Vector2(DisplayServer.window_get_size()) * root.scaling_3d_scale),
			RenderingServer.get_current_rendering_driver_name(), RenderingServer.get_video_adapter_name(),
			_world.clock.warp])


## Camera script; true when the run is over.
func _drive(t: float) -> bool:
	var local := t - _phase_start_s
	if local >= _durations[_phase]:
		var next := PHASES.find(_phase) + 1
		if next >= PHASES.size():
			_log_second()
			_summary()
			return true
		_enter(PHASES[next], t)
		return false
	var fraction: float = local / _durations[_phase]
	match _phase:
		"descent":
			_pad["eye_height_m"] = START_EYE_HEIGHT_M * pow(END_EYE_HEIGHT_M / START_EYE_HEIGHT_M, fraction)
		"pan_low", "pan_high":
			_world.camera.azimuth = TAU * fraction
			if not _shots_dir.is_empty():
				for mark: float in [0.0, 0.5]:
					var shot := "%s_%d" % [_phase, int(mark * 360.0)]
					if fraction >= mark and not _shots_taken.has(shot):
						_shots_taken[shot] = true
						_screenshot(shot)
	return false


## Starts a phase at run time t.
func _enter(phase: String, t: float) -> void:
	_phase = phase
	_phase_start_s = t
	match phase:
		"pan_low":
			var ground := _ground_site("pan_low", PAN_LOW_OFFSET_M)
			ground["eye_height_m"] = END_EYE_HEIGHT_M
			_world.set_ground_site(ground)
			_world.camera.elevation = 0.0
			_skip_frame = _frames + 1
		"pan_high":
			_pad["eye_height_m"] = PAN_HIGH_EYE_M
			_world.set_ground_site(_pad)
			_world.camera.elevation = deg_to_rad(PAN_HIGH_ELEVATION_DEG)
			_skip_frame = _frames + 1
		"cape":
			_cape["eye_height_m"] = END_EYE_HEIGHT_M
			_world.set_ground_site(_cape)
			_world.camera.azimuth = deg_to_rad(90.0)
			_world.camera.elevation = 0.0
			_skip_frame = _frames + 1  # the next frame draws the new site first


## A ground site offset (east, north; m) from OLP-1, at the core's surface height there.
func _ground_site(id: String, offset_m: Vector2) -> Dictionary:
	var lat: float = _pad["latitude_deg"] + offset_m.y / METRES_PER_DEG
	var lon: float = _pad["longitude_deg"] + offset_m.x / (METRES_PER_DEG * cos(deg_to_rad(lat)))
	var samples: Dictionary = _world.core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var height: float = samples["height_m"][0] if bool(samples["present"]) else float(_pad["height_m"])
	return {"id": id, "name": id, "latitude_deg": lat, "longitude_deg": lon, "height_m": height}


func _screenshot(name: String) -> void:
	var path := _shots_dir.path_join("terrain_perf_%s.png" % name)
	var error := root.get_viewport().get_texture().get_image().save_png(path)
	print("terrain_perf: screenshot %s (error %d)" % [path, error])
	_skip_frame = _frames + 1


func _record(frame_ms: float, process_ms: float, rest_ms: float) -> void:
	var stats: Dictionary = _world.earth_terrain.stats
	var vp := root.get_viewport_rid()
	_second["uploads"] += int(stats["uploads_last_frame"])
	_second["terrain_ms"] = maxf(_second["terrain_ms"], float(stats.get("update_ms", 0.0)))
	_second["gpu_ms"] = maxf(_second["gpu_ms"], RenderingServer.viewport_get_measured_render_time_gpu(vp))
	var limits: Dictionary = _phase_max[_phase]
	limits["drawn"] = maxi(limits["drawn"], int(stats["nodes_drawn"]))
	limits["loading"] = maxi(limits["loading"], int(stats["nodes_loading"]))
	if _frames == _skip_frame:
		if _phase == "cape" and _jump_ms == 0.0:
			_jump_ms = frame_ms
		return
	_phase_ms[_phase].append(frame_ms)  # Arrays: appended in place (a Packed array would be a copy)
	_second["ms"].append(frame_ms)
	if frame_ms > _second["worst_ms"]:
		_second["worst_ms"] = frame_ms
		_second["worst_split"] = Vector2(process_ms, rest_ms)
	var pipelines := int(Performance.get_monitor(Performance.PIPELINE_COMPILATIONS_DRAW))
	var new_pipelines := pipelines - _pipelines_seen if _pipelines_seen >= 0 else 0
	_pipelines_seen = pipelines
	_second["pipelines"] += new_pipelines
	if _spike_followup > 0:  # the measured viewport times arrive a few frames late
		_spikes[_spikes.size() - 1] += "; viewport cpu %.2f gpu %.2f" % [
				RenderingServer.viewport_get_measured_render_time_cpu(vp),
				RenderingServer.viewport_get_measured_render_time_gpu(vp)]
		_spike_followup -= 1
	if _spike_ms > 0.0 and frame_ms > _spike_ms:
		_spikes.append(("terrain_perf: spike %s t=%.3f s %.2f ms: process %.2f rest %.2f (draw %.2f), terrain %.2f ms, "
				+ "uploads %d, nodes loading %d, draw-time pipeline compilations %d; next frames:") % [_phase,
				(_last_us - _t0_us) * 1.0e-6, frame_ms, process_ms, rest_ms, (_post_draw_us - _pre_draw_us) * 1.0e-3,
				stats.get("update_ms", 0.0), stats["uploads_last_frame"], stats.get("nodes_loading", 0), new_pipelines])
		_spike_followup = SPIKE_FOLLOWUP_FRAMES


func _reset_second() -> void:
	_second = {"ms": [], "uploads": 0, "terrain_ms": 0.0, "gpu_ms": 0.0, "worst_ms": 0.0,
			"worst_split": Vector2.ZERO, "pipelines": 0}


func _log_second() -> void:
	var ms: Array = _second["ms"]
	if ms.is_empty():
		_second_index += 1
		_reset_second()
		return
	var stats: Dictionary = _world.earth_terrain.stats
	var vram_mb := Performance.get_monitor(Performance.RENDER_VIDEO_MEM_USED) / 1048576.0
	var limits: Dictionary = _phase_max[_phase]
	limits["vram_mb"] = maxf(limits["vram_mb"], vram_mb)
	var q := _quantiles(ms)
	var split: Vector2 = _second["worst_split"]
	var eye_m := float(_world.ground_site.get("eye_height_m", 0.0))
	_log_lines.append("terrain_perf: t=%3d s %-8s eye %9.1f m  frame p50 %5.2f p99 %5.2f max %6.2f ms "
			% [_second_index, _phase, eye_m, q[0], q[1], q[2]]
			+ "(process %5.2f rest %5.2f)  " % [split.x, split.y]
			+ ("terrain max %5.2f ms  gpu max %5.2f ms  drawn %4d loading %4d uploads %3d  vram %5.0f MB  frames %d  "
			+ "pipelines %d")
			% [_second["terrain_ms"], _second["gpu_ms"], stats["nodes_drawn"], stats["nodes_loading"],
			_second["uploads"], vram_mb, ms.size(), _second["pipelines"]])
	_second_index += 1
	_reset_second()


func _summary() -> void:
	for line in _log_lines:
		print(line)
	for line in _spikes:
		print(line)
	var all := []
	var max_drawn := 0
	var max_vram := 0.0
	for phase: String in PHASES:
		var ms: Array = _phase_ms[phase]
		all.append_array(ms)
		var limits: Dictionary = _phase_max[phase]
		max_drawn = maxi(max_drawn, limits["drawn"])
		max_vram = maxf(max_vram, limits["vram_mb"])
		if ms.is_empty():
			continue
		var q := _quantiles(ms)
		print("terrain_perf: PHASE %-8s frames %5d  p50 %5.2f p99 %5.2f max %6.2f ms  max drawn %4d loading %4d  vram %5.0f MB"
				% [phase, ms.size(), q[0], q[1], q[2], limits["drawn"], limits["loading"], limits["vram_mb"]])
	var q := _quantiles(all)
	var gate_p99 := GATE_P99_CLOUDS_MS if String(_args.get("weather", "")).ends_with(".wthr") else GATE_P99_MS
	var ok := q[1] <= gate_p99 and q[2] <= GATE_MAX_MS
	print("terrain_perf: SUMMARY frames %d  p50 %.2f p99 %.2f max %.2f ms (jump frame %.2f ms)  max drawn %d  "
			% [all.size(), q[0], q[1], q[2], _jump_ms, max_drawn]
			+ "max vram %.0f MB  gate p99 <= %.0f max <= %.0f ms: %s" % [max_vram, gate_p99, GATE_MAX_MS,
			"PASS" if ok else "FAIL"])


## [p50, p99, max] of frame times (ms).
static func _quantiles(values: Array) -> Array[float]:
	var sorted := values.duplicate()
	sorted.sort()
	var n := sorted.size()
	return [sorted[int(0.5 * (n - 1))], sorted[int(0.99 * (n - 1))], sorted[n - 1]]
