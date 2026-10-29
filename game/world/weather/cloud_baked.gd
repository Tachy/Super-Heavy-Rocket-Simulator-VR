class_name CloudBaked
extends RefCounted
## The baked cloud volume prototype ("Weg 2", user 2026-10-29): the clouds' shape and light precomputed once in a
## world-fixed clipmap around an anchor (the camera when the bake starts; static clouds, independent of the view
## direction) and each frame a cheap march through it (shaders/cloud_baked.gdshaderinc). The bake runs off the march in
## slices (a few dispatches per frame) into the second of two volume sets; a finished set is cross-faded in over
## FADE_FRAMES frames (both sets marched, their renders mixed: no pop, never a hole). A re-bake starts when the camera
## leaves REBAKE_MOVE of the anchor, the Sun turns by REBAKE_SUN_RAD or the weather changes. Owned by CloudEffect
## (render thread); CloudEffect marches it in its march's place with MarchMethod.BAKED (until the first set is baked:
## today's march).

## Columns per side and level 0's horizontal cell; levels (each doubles the cell); the vertical cell of level 0 and the
## largest (each level doubles it up to that). [E] prototype figures: level 0 spans +-12.8 km at 100 m, level 5
## +-410 km at 3.2 km.
static var columns := 256
static var cell0_m := 100.0 if OS.get_environment("STARSIM_BAKED_CELL0") == "" \
		else float(OS.get_environment("STARSIM_BAKED_CELL0"))
static var levels := 6
static var dz0_m := 50.0
static var dz_max_m := 400.0
## The march's per-sample detail (one fetch of the detail's level 1 near the edges in level 0; CLOUD_BAKED_DETAIL).
## Environment STARSIM_BAKED_DETAIL=0 switches it off (measurements).
static var detail := OS.get_environment("STARSIM_BAKED_DETAIL") != "0"
## Checks: > 0 forces a re-bake (and its cross-fade) this many frames after the last one is shown (environment
## STARSIM_BAKED_REBAKE_FRAMES; the swap's cost and look in a measurement).
static var rebake_frames := int(OS.get_environment("STARSIM_BAKED_REBAKE_FRAMES"))
## Diagnostics (measurements only): STARSIM_BAKED_DEFINES=<A,B,..> adds shader defines to every kernel,
## STARSIM_BAKED_MEMORY_MB overrides the memory cap, STARSIM_BAKED_CELL0 level 0's horizontal cell (m).
static var extra_defines := OS.get_environment("STARSIM_BAKED_DEFINES").split(",", false)
static var memory_cap_bytes := MEMORY_CAP_BYTES if OS.get_environment("STARSIM_BAKED_MEMORY_MB") == "" \
		else float(OS.get_environment("STARSIM_BAKED_MEMORY_MB")) * 1024.0 * 1024.0
## [E] The volumes' memory cap (both sets; the brief's ~512 MB): the vertical cells double until a set fits half.
const MEMORY_CAP_BYTES := 512.0 * 1024.0 * 1024.0
## [E] Bake budget (measured 2026-10-29: <= ~2 ms GPU per dispatch): voxels per frame per pass kind (each dispatch a slice range of one level); render_check scales it.
static var bake_scale := 1.0
const VOXELS_PER_FRAME := {"density": 0.75e6, "occupancy": 8.0e6, "light": 0.4e6}
## [E] Re-bake triggers: the camera's distance from the anchor (a quarter of level 0's half-size), the Sun's turn, the
## weather's blend.
const REBAKE_MOVE_FRACTION := 0.25
const REBAKE_SUN_RAD := 0.5 * PI / 180.0
const REBAKE_BLEND := 0.02
## Frames of the cross-fade of a new set.
const FADE_FRAMES := 8
const BLOCK := 4
const GRID_BINDING := 53
const BINDINGS := [[50, 51, 52], [54, 55, 56]]
## The band's floor and the cloud shell's top (clouds.gdshaderinc CLOUD_TOP_M); the band's margin above the field's top.
const BAND_BOTTOM_M := 0.0
const CLOUD_TOP_M := 20000.0
const BAND_TOP_MARGIN_M := 200.0
const SET_FLOATS := 14 * 4
const GRID_BLOCK_SIZE := 2 * SET_FLOATS * 4 + 16
const FILES := ["res://shaders/cloud_baked.gdshaderinc"]
const STAMP := "cloud_baked_bake"

## Render thread -> main thread: a set is shown (the march reads it); the last bake's figures.
var ready := false
var last_bake_report := ""
var bakes_done := 0
var _rd: RenderingDevice
var _shaders := {}
var _pipelines := {}
var _march_buffer := RID()
var _bake_buffer := RID()
var _band_buffer := RID()  ## the band kernel's lowest / highest cloudy field layer
var _sets: Array[Dictionary] = [{}, {}]  ## {plan, a, b, o}
var _shown := -1  ## the set the march reads (-1: none yet)
var _baking := -1  ## the set being baked (-1: none)
var _fade := 0  ## frames of the cross-fade still to run (the baked set fading in)
var _phases: Array[Dictionary] = []
var _bake_frames := 0
var _bake_start_us := 0
var _bake_gpu_ms := 0.0
var _bake_dispatches := 0
var _shown_frames := 0  ## frames since the shown set was swapped in
var _read_frame := -1  ## the frame whose captured timestamps were read last


## Compiles the kernels (the march's files + cloud_baked.gdshaderinc; set 0 is the march's). false on an error.
func setup(rd: RenderingDevice, stereo: bool) -> bool:
	_rd = rd
	var files := PackedStringArray(CloudEffect.MARCH_FILES + FILES)
	var march := PackedStringArray(["CLOUD_MARCH", "CLOUD_BAKED_MARCH"])
	march.append_array(CloudEffect.diagnostic_defines())
	if detail:
		march.append("CLOUD_BAKED_DETAIL")
	if OS.get_environment("STARSIM_BAKED_DEBUG") == "alpha":
		march.append("CLOUD_BAKED_DEBUG_ALPHA")
	elif OS.get_environment("STARSIM_BAKED_DEBUG") != "":
		march.append("CLOUD_BAKED_DEBUG")
		if OS.get_environment("STARSIM_BAKED_DEBUG_H") != "":
			ShaderGlobals.set_value(&"cloud_debug_height_m", -float(OS.get_environment("STARSIM_BAKED_DEBUG_H")))
	if stereo:
		march.append("CLOUD_CENTRE")
	var fade := march.duplicate()
	fade.append("CLOUD_BAKED_FADE")
	# The density bake knows the march's per-sample detail (its threshold state counts it).
	var density := PackedStringArray(["CLOUD_BAKED_DENSITY"])
	if detail:
		density.append("CLOUD_BAKED_DETAIL")
	var kernels := {"march": march, "march_fade": fade,
		"density": density, "occupancy": PackedStringArray(["CLOUD_BAKED_OCCUPANCY"]),
		"light": PackedStringArray(["CLOUD_BAKED_LIGHT"]), "band": PackedStringArray(["CLOUD_BAKED_BAND"])}
	for name: String in kernels:
		var defines: PackedStringArray = kernels[name]
		defines.append_array(extra_defines)
		var start_us := Time.get_ticks_usec()
		var src := RDShaderSource.new()
		src.language = RenderingDevice.SHADER_LANGUAGE_GLSL
		src.source_compute = CloudShaderPort.build(defines, files).source
		var spirv := rd.shader_compile_spirv_from_source(src)
		if spirv.compile_error_compute != "":
			push_error("CloudBaked %s: %s" % [name, spirv.compile_error_compute])
			free_all()
			return false
		_shaders[name] = rd.shader_create_from_spirv(spirv)
		if not _shaders[name].is_valid():
			push_error("CloudBaked %s: shader not created" % name)
			free_all()
			return false
		_pipelines[name] = rd.compute_pipeline_create(_shaders[name])
		print("CloudBaked: kernel %s compiled in %.1f s" % [name, (Time.get_ticks_usec() - start_us) / 1.0e6])
	_march_buffer = rd.uniform_buffer_create(GRID_BLOCK_SIZE)
	_bake_buffer = rd.uniform_buffer_create(GRID_BLOCK_SIZE)
	_band_buffer = rd.storage_buffer_create(8)
	return true


## The march kernel's shader / pipeline of this frame (the cross-fade's while a new set fades in).
func march_kernel() -> String:
	return "march_fade" if _fade > 0 else "march"


func shader(kernel: String) -> RID:
	return _shaders[kernel]


func pipeline(kernel: String) -> RID:
	return _pipelines[kernel]


## True once a set is shown and no bake or cross-fade runs (render_check measures the settled state).
func settled() -> bool:
	return _shown >= 0 and _baking < 0 and _fade == 0


## Render thread, per frame before the march: re-bake triggers, the next bake dispatches, the march's block. Returns the
## march's extra set 1 uniforms, empty while no set is baked yet.
func frame(set0: Array[RDUniform], samplers: Dictionary, camera: Vector3) -> Array[RDUniform]:
	_read_bake_time()
	var sun: Vector3 = ShaderGlobals.value(&"atmosphere_sun_direction")
	var weather := _weather_key()
	_shown_frames += 1
	var forced := rebake_frames > 0 and _shown_frames > rebake_frames
	if _baking < 0 and (_shown < 0 or forced or _needs_rebake(_sets[_shown]["plan"], camera, sun, weather)):
		_start_bake(1 - _shown if _shown >= 0 else 0, camera, sun, weather)
	if _baking >= 0:
		_bake_step(set0, samplers, camera)
	if _shown < 0:
		ready = false
		return []
	ready = true
	var nearest: RID = samplers["false|false|false"]
	var linear: RID = samplers["true|false|false"]
	var order: Array[int] = [_shown]
	var weight := 0.0
	if _fade > 0:
		order.append(1 - _shown)
		weight = 1.0 - float(_fade) / float(FADE_FRAMES + 1)
	var floats := PackedFloat32Array()
	for i in range(2):
		floats.append_array(_set_floats(_sets[order[mini(i, order.size() - 1)]]["plan"], camera, weight if i == 0
				else 0.0))
	var bytes := floats.to_byte_array()
	bytes.append_array(PackedInt32Array([0, 0, 0, _model()]).to_byte_array())
	_rd.buffer_update(_march_buffer, 0, GRID_BLOCK_SIZE, bytes)
	var uniforms: Array[RDUniform] = [CloudEffect._ubo(_march_buffer, GRID_BINDING)]
	for i in range(order.size()):
		var s: Dictionary = _sets[order[i]]
		uniforms.append_array([CloudEffect._tex(BINDINGS[i][0], linear, s["a"]),
			CloudEffect._tex(BINDINGS[i][1], linear, s["b"]), CloudEffect._tex(BINDINGS[i][2], nearest, s["o"])])
	if _fade > 0:
		_fade -= 1
		if _fade == 0:
			_finish_fade()
	return uniforms


## Frees the kernels and volumes.
func free_all() -> void:
	if _rd == null:
		return
	for s in _sets:
		for key in ["a", "b", "o"]:
			if s.has(key) and s[key].is_valid():
				_rd.free_rid(s[key])
		s.clear()
	for rid: RID in _shaders.values():
		_rd.free_rid(rid)
	_shaders.clear()
	_pipelines.clear()
	for rid in [_march_buffer, _bake_buffer, _band_buffer]:
		if rid.is_valid():
			_rd.free_rid(rid)
	_march_buffer = RID()
	_bake_buffer = RID()
	_band_buffer = RID()


static func _model() -> int:
	return int(ShaderGlobals.value(&"cloud_scattering_model"))


## What a re-bake follows of the weather: the snapshot pair's blend, sizes, top and the scattering model.
static func _weather_key() -> Dictionary:
	return {"blend": float(ShaderGlobals.value(&"cloud_blend")),
		"before": ShaderGlobals.value(&"cloud_field_before_size"), "after": ShaderGlobals.value(&"cloud_field_after_size"),
		"top": float(ShaderGlobals.value(&"cloud_air_shadow_top_m")), "model": _model()}


func _needs_rebake(plan: Dictionary, camera: Vector3, sun: Vector3, weather: Dictionary) -> bool:
	if camera.distance_to(plan["anchor"]) > REBAKE_MOVE_FRACTION * 0.5 * columns * cell0_m:
		return true
	if sun.normalized().angle_to(plan["sun"]) > REBAKE_SUN_RAD:
		return true
	var w: Dictionary = plan["weather"]
	return absf(float(w["blend"]) - float(weather["blend"])) > REBAKE_BLEND or w["before"] != weather["before"] \
			or w["after"] != weather["after"] or absf(float(w["top"]) - float(weather["top"])) > 1.0 \
			or w["model"] != weather["model"]


## The layout of a set anchored at the camera: its axes; the band and levels follow from the band kernel (_levels).
func _plan(camera: Vector3, sun: Vector3, weather: Dictionary) -> Dictionary:
	var stretch := Vector3(1.0, CloudLayer.POLAR_STRETCH, 1.0)
	var up0 := (camera * stretch).normalized()
	var axis_x := Vector3(0.0, 1.0, 0.0).cross(up0)
	if axis_x.length() < 1.0e-3:
		axis_x = Vector3(1.0, 0.0, 0.0).cross(up0)
	axis_x = axis_x.normalized()
	var axis_y := up0.cross(axis_x)
	var plan := {"anchor": camera, "sun": sun.normalized(), "weather": weather, "up0": up0, "axis_x": axis_x,
		"axis_y": axis_y}
	_levels(plan, BAND_BOTTOM_M, CLOUD_TOP_M)
	return plan


## The band [bottom, top] (m) and the levels' vertical cells and slices into plan: the vertical cells double from
## dz0_m until both sets fit MEMORY_CAP_BYTES (and the texture's depth the D3D12 cap).
func _levels(plan: Dictionary, bottom: float, top: float) -> void:
	var dz0 := dz0_m
	while true:
		var dz_max := maxf(dz_max_m, dz0)
		var height := maxf(ceilf((top - bottom) / dz_max), 1.0) * dz_max
		var dz: Array[float] = []
		var slab: Array[int] = []
		var oslab: Array[int] = []
		var onz: Array[int] = []
		var depth := 0
		var odepth := 0
		for k in range(levels):
			var d := minf(dz0 * pow(2.0, k), dz_max)
			var nz := roundi(height / d)
			dz.append(d)
			slab.append(depth)
			depth += nz + 2
			var nb := (nz + BLOCK - 1) / BLOCK
			oslab.append(odepth)
			onz.append(nb)
			odepth += nb
		var bytes := float(columns) * columns * depth * 8.0 + float(columns / BLOCK) * (columns / BLOCK) * odepth
		plan.merge({"band": Vector2(bottom, bottom + height), "dz": dz, "slab": slab, "oslab": oslab, "onz": onz,
			"depth": depth, "odepth": odepth, "bytes": bytes}, true)
		if 2.0 * bytes <= memory_cap_bytes and depth <= 2048:
			return
		dz0 *= 2.0


## A set's floats of the CloudBakedSet block (cloud_baked.gdshaderinc) for the camera; fade: misc.y.
func _set_floats(plan: Dictionary, camera: Vector3, fade: float) -> PackedFloat32Array:
	var stretch := Vector3(1.0, CloudLayer.POLAR_STRETCH, 1.0)
	var up0: Vector3 = plan["up0"]
	var axis_x: Vector3 = plan["axis_x"]
	var axis_y: Vector3 = plan["axis_y"]
	var band: Vector2 = plan["band"]
	var cam_m := camera * stretch  # float64: the camera's grid coordinates
	var anchor: Vector3 = plan["anchor"] - camera
	var f := PackedFloat32Array([up0.x, up0.y, up0.z, columns, axis_x.x, axis_x.y, axis_x.z, cell0_m,
			axis_y.x, axis_y.y, axis_y.z, levels, band.x, band.y, cam_m.dot(axis_x), cam_m.dot(axis_y),
			anchor.x, anchor.y, anchor.z, plan["depth"]])
	for key in ["dz", "slab", "oslab", "onz"]:
		var values: Array = plan[key]
		for i in range(8):
			f.append(float(values[i]) if i < values.size() else 0.0)
	f.append_array([plan["odepth"], fade, 0.0, 0.0])
	return f


func _volume(size: Vector3i, format: int) -> RID:
	var tf := RDTextureFormat.new()
	tf.texture_type = RenderingDevice.TEXTURE_TYPE_3D
	tf.width = size.x
	tf.height = size.y
	tf.depth = size.z
	tf.format = format
	tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT
	return _rd.texture_create(tf, RDTextureView.new())


func _start_bake(index: int, camera: Vector3, sun: Vector3, weather: Dictionary) -> void:
	_sets[index]["plan"] = _plan(camera, sun, weather)
	_baking = index
	_phases = [{"kind": "band", "level": 0, "total": 2, "next": 0, "step": 1}]
	_bake_frames = 0
	_bake_dispatches = 0
	_bake_gpu_ms = 0.0
	_bake_start_us = Time.get_ticks_usec()


## After the band kernel: the band from its layers, the levels, the set's volumes (re-created when their size
## changes) and the bake's phases.
func _start_volumes(lowest: int, highest: int) -> void:
	var s: Dictionary = _sets[_baking]
	var plan: Dictionary = s["plan"]
	var bottom := BAND_BOTTOM_M
	var top := BAND_BOTTOM_M + 1000.0  # no cloud: a thin empty band
	if highest >= 0:
		# The field's profile reaches zero at the centres of the layers around the cloudy ones (core
		# cloud_render_top_m); the base is cut within the band (the LCL).
		bottom = maxf(BAND_BOTTOM_M, _layer_centre_m(lowest - 1) if lowest > 0 else BAND_BOTTOM_M)
		top = minf(_layer_centre_m(highest + 1) + BAND_TOP_MARGIN_M if highest < 31 else CLOUD_TOP_M, CLOUD_TOP_M)
	_levels(plan, bottom, top)
	if not s.has("a") or s["size"] != Vector2i(plan["depth"], plan["odepth"]):
		for key in ["a", "b", "o"]:
			if s.has(key) and s[key].is_valid():
				_rd.free_rid(s[key])
		s["a"] = _volume(Vector3i(columns, columns, plan["depth"]), RenderingDevice.DATA_FORMAT_R16G16_SFLOAT)
		s["b"] = _volume(Vector3i(columns, columns, plan["depth"]), RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM)
		s["o"] = _volume(Vector3i(columns / BLOCK, columns / BLOCK, plan["odepth"]),
				RenderingDevice.DATA_FORMAT_R8_UNORM)
		s["size"] = Vector2i(plan["depth"], plan["odepth"])
		if not (s["a"].is_valid() and s["b"].is_valid() and s["o"].is_valid()):
			push_error("CloudBaked: a volume was not created")
	var area := float(columns) * columns
	for kind in ["density", "occupancy", "light"]:
		for k in range(levels):
			var dz: float = plan["dz"][k]
			var nz := roundi((plan["band"].y - plan["band"].x) / dz)
			var slices: int = plan["onz"][k] if kind == "occupancy" else nz
			var per_slice := area / (BLOCK * BLOCK) if kind == "occupancy" else area
			var step := maxi(1, int(VOXELS_PER_FRAME[kind] * bake_scale / per_slice))
			_phases.append({"kind": kind, "level": k, "total": slices, "next": 0, "step": step})
	print("CloudBaked: bake of set %d: field layers %d..%d, band %.0f..%.0f m, dz %s, %d slices, %.0f MB per set, "
			% [_baking, lowest, highest, plan["band"].x, plan["band"].y, plan["dz"], plan["depth"],
			plan["bytes"] / 1048576.0] + "anchor %s" % plan["anchor"])


## The field's layer centre (clouds.gdshaderinc cloud_layer_centre_m).
static func _layer_centre_m(layer: int) -> float:
	return (layer + 0.5) * 250.0 if layer < 16 else 4000.0 + (layer - 16 + 0.5) * 1000.0


## This frame's bake dispatches: one per pass kind's budget (each its own compute list), stamped for the GPU time.
func _bake_step(set0: Array[RDUniform], samplers: Dictionary, camera: Vector3) -> void:
	var s: Dictionary = _sets[_baking]
	var plan: Dictionary = s["plan"]
	var nearest: RID = samplers["false|false|false"]
	var linear: RID = samplers["true|false|false"]
	_bake_frames += 1
	if _phases.is_empty():
		return
	var phase: Dictionary = _phases[0]
	var kind: String = phase["kind"]
	if kind == "band" and phase["next"] == 1:
		# The band kernel ran last frame: its layers (a small read-back), then the volumes' phases.
		var layers := _rd.buffer_get_data(_band_buffer).to_int32_array()
		_phases.pop_front()
		_start_volumes(layers[0], layers[1])
		phase = _phases[0]
		kind = phase["kind"]
	var first: int = phase["next"]
	var slices: int = mini(phase["step"], phase["total"] - first)
	var floats := _set_floats(plan, camera, 0.0)
	floats.append_array(_set_floats(plan, camera, 0.0))
	var bytes := floats.to_byte_array()
	bytes.append_array(PackedInt32Array([phase["level"], first, slices, _model()]).to_byte_array())
	_rd.buffer_update(_bake_buffer, 0, GRID_BLOCK_SIZE, bytes)
	var set1: Array[RDUniform] = [CloudEffect._ubo(_bake_buffer)]
	var groups := Vector3i((columns - 1) / 8 + 1, (columns - 1) / 8 + 1, slices)
	if kind == "band":
		_rd.buffer_update(_band_buffer, 0, 8, PackedInt32Array([32, -1]).to_byte_array())
		set1.append(CloudEffect._storage(1, _band_buffer))
		groups = Vector3i(8, 8, 1)
	elif kind == "density":
		set1.append(CloudEffect._image(1, s["a"]))
	elif kind == "occupancy":
		set1.append_array([CloudEffect._image(1, s["o"]), CloudEffect._tex(2, nearest, s["a"])])
		groups = Vector3i((columns / BLOCK - 1) / 8 + 1, (columns / BLOCK - 1) / 8 + 1, slices)
	else:
		set1.append_array([CloudEffect._image(1, s["b"]), CloudEffect._tex(2, linear, s["a"]),
				CloudEffect._tex(3, nearest, s["o"])])
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp(STAMP)
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _pipelines[kind])
	_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders[kind], 0, set0), 0)
	_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders[kind], 1, set1), 1)
	_rd.compute_list_dispatch(list, groups.x, groups.y, groups.z)
	_rd.compute_list_end()
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp("cloud_pass")
	_bake_dispatches += 1
	phase["next"] = first + slices
	if phase["next"] >= phase["total"]:
		_phases.pop_front()
	if _phases.is_empty():
		_finish_bake()


func _finish_bake() -> void:
	var ms := (Time.get_ticks_usec() - _bake_start_us) / 1000.0
	last_bake_report = "set %d baked: %d frames, %d dispatches, %.0f ms wall" % [_baking, _bake_frames,
			_bake_dispatches, ms]
	bakes_done += 1
	print("CloudBaked: " + last_bake_report)
	if _shown < 0:
		_shown = _baking
		_baking = -1
		_shown_frames = 0
	else:
		_fade = FADE_FRAMES


func _finish_fade() -> void:
	_shown = _baking
	_baking = -1
	_shown_frames = 0


## The previous frame's bake GPU time (its stamped intervals), summed over the bake; printed once the bake is done.
func _read_bake_time() -> void:
	if not CloudEffect.capture_timestamps:
		return
	var frame := _rd.get_captured_timestamps_frame()
	if frame == _read_frame:
		return
	_read_frame = frame
	for i in range(1, _rd.get_captured_timestamps_count()):
		if _rd.get_captured_timestamp_name(i - 1) == STAMP:
			var ms := float(_rd.get_captured_timestamp_gpu_time(i) - _rd.get_captured_timestamp_gpu_time(i - 1)) / 1.0e6
			_bake_gpu_ms += ms
			print("CloudBaked: bake dispatch GPU %.2f ms (total %.1f ms)" % [ms, _bake_gpu_ms])
