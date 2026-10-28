class_name CloudWorldVolume
extends RefCounted
## The cloud world volume's GPU side (world-volume plan Task 5; spec B; shaders/cloud_volume.gdshaderinc,
## cloud_volume_bake.gdshaderinc): per level (CloudWorldVolumeLayout) the toroidal density (RGBA16F, 1/km: rho and
## sigma_c of slot 0's snapshot in RG, of slot 1's in BA), the light (RGBA16F, Task 6), the max-extinction mip (R16F,
## 1/km) and the Chebyshev distance field (R8, blocks) at 1 / MIP_DIV resolution; texture index 8 is the transient spare
## level of a re-frame (created with its first job, freed when it is swapped in).
## Threads (the light cache's pattern): CloudLayer (main thread) plans CloudWorldVolumeSchedule's batch and submit()s
## it; CloudEffect (render thread) take()s it and run()s it before the march - the bake jobs (CLOUD_VOLUME_BAKE), then
## per level touched the max mip over the baked blocks and the whole level's distance field (CLOUD_VOLUME_OCCUPANCY), then
## (Task 6) the light - and only then adopts the batch's lookup state, so nothing is shown before its bake ran. A batch
## not yet taken holds the schedule back. The upkeep's GPU time (timestamp UPKEEP_STAMP) comes back a few frames later
## with the batch's tap count (take_feedback: CloudWorldVolumeSchedule.feedback).
## Channel pairs follow the layer's slots one to one (channel_of_slot): a slot's snapshot is replaced only without
## weight, so its pair is rebaked while it shows nothing (the schedule's slot rebake; CloudLayer holds the weight off a
## slot until the adopted state says its pair holds it).

const FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_light_cache.gdshaderinc", "res://shaders/cloud_far.gdshaderinc",
		"res://shaders/cloud_light.gdshaderinc", "res://shaders/cloud_pass.gdshaderinc",
		"res://shaders/cloud_volume.gdshaderinc", "res://shaders/cloud_volume_bake.gdshaderinc"]
const DENSITY_FORMAT := RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT
const LIGHT_FORMAT := RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT
const MAX_FORMAT := RenderingDevice.DATA_FORMAT_R16_SFLOAT
const DISTANCE_FORMAT := RenderingDevice.DATA_FORMAT_R8_UNORM
## The levels' textures and the spare's.
const TEXTURES := CloudWorldVolumeLayout.LEVELS + 1
const SPARE := CloudWorldVolumeLayout.LEVELS
## Stored extinction unit (cloud_volume.gdshaderinc CLOUD_VOLUME_PER_KM).
const PER_KM := 1000.0
## [E] The distance field's cap (blocks of MIP_DIV cells): sphere tracing skips up to 8 blocks a step (1.6 km at level
## 0, 205 km at level 7); 17 fetches per texel and pass keep a whole level's three passes ~0.05 ms.
const DISTANCE_CAP_BLOCKS := 8
## Compute groups (cloud_volume_bake.gdshaderinc).
const GROUP := 4
const PROBE_GROUP := 64
## The lookup's set 1 bindings (cloud_volume.gdshaderinc CLOUD_VOLUME_BLOCK_BINDING and the texture groups after it).
const BLOCK_BINDING := 40
const FRAME_VEC4S := 9
const VOLUME_BLOCK_SIZE := (2 * FRAME_VEC4S + TEXTURES + 1) * 16
const JOB_BLOCK_SIZE := 3 * 16
const OCCUPANCY_BLOCK_SIZE := 2 * 16
const PROBE_BLOCK_SIZE := 16
## Level flags (cloud_volume.gdshaderinc).
const FLAG_ACTIVE := 1
const FLAG_READY := 2
const FLAG_VERTICAL_FADE := 4
const FLAG_CLAMPED_BOTTOM := 8
const FLAG_FRAME_SHIFT := 4
## The upkeep's GPU interval (always stamped: the schedule's rate calibration reads it) and the stamp after it.
const UPKEEP_STAMP := "cloud_volume_upkeep"
const UPKEEP_END_STAMP := "cloud_pass"
## [E] Measured frames the main thread may leave unread (the oldest are dropped) and runs awaiting their timestamps.
const FEEDBACK_MAX := 64
const PENDING_MAX := 16

## Render thread: the last run's work (diagnostics; CloudLayer.world_volume_stats).
var last_bake_cells := 0
var last_bake_taps := 0
var last_light_cells := 0
var last_upkeep_ms := 0.0

var _rd: RenderingDevice
var _failed := false
var _shaders := {}  ## kernel name -> shader RID (the shader frees its pipeline)
var _pipelines := {}
var _levels: Array[Dictionary] = []  ## per texture index: {density, light, max, distance} RIDs ({} = none)
var _scratch: Array[RID] = []  ## the distance passes' two R8 volumes (shared by the levels)
var _volume_buffer := RID()
var _job_buffers: Array[RID] = []
var _occupancy_buffers: Array[RID] = []
var _probe_buffer := RID()
var _linear := RID()
var _nearest := RID()
var _layouts := {}  ## frame origin (Vector3) -> CloudWorldVolumeLayout set to it
var _last_set0: Array[RDUniform] = []
## Main thread <-> render thread (under _mutex): the batch handed over, the adopted lookup, the measured upkeeps.
var _mutex := Mutex.new()
var _batch := {}
var _lookup := {}
var _feedback: Array[Vector3] = []  ## (upkeep ms, bake taps, light cells)
var _pending := {}  ## Engine frame of a run -> Vector2(bake taps, light cells), until its timestamps come back


## Bytes of the levels' textures (the spare's transient 1/8 more not counted): density + light per cell, the mips.
static func texture_bytes() -> int:
	var L := CloudWorldVolumeLayout
	var mip := L.mip_size()
	return L.LEVELS * (L.CELLS_H * L.CELLS_H * L.CELLS_V * L.BYTES_PER_CELL + mip.x * mip.y * mip.z * 3)


## The density channel pair holding slot `slot`'s snapshot (pair 0: RG, pair 1: BA).
static func channel_of_slot(slot: int) -> int:
	return slot


# ---------------------------------------------------------------- main thread

## Hands over the schedule's batch (CloudWorldVolumeSchedule.plan).
func submit(batch: Dictionary) -> void:
	_mutex.lock()
	_batch = batch
	_mutex.unlock()


## True while the last batch was not taken by the render thread (the schedule waits).
func waiting() -> bool:
	_mutex.lock()
	var w := not _batch.is_empty()
	_mutex.unlock()
	return w


## The lookup state of the last batch run ({} before the first).
func adopted() -> Dictionary:
	_mutex.lock()
	var l := _lookup
	_mutex.unlock()
	return l


## True while the render thread runs the volume's batches: one was adopted since the last forget() and the last one
## handed over was taken (a stopped cloud pass leaves it waiting: then nothing holds the cross-fade). Frame-based, not
## wall-clock: a long frame must not open the cross-fade's gate.
func running() -> bool:
	_mutex.lock()
	var r := not _lookup.is_empty() and _batch.is_empty()
	_mutex.unlock()
	return r


## Whether the adopted state's channel pair of `slot` holds the snapshot `id` (baked: shown with weight).
func slot_holds(slot: int, id: String) -> bool:
	var l := adopted()
	if l.is_empty():
		return false
	var ids: Array = l["slot_ids"]
	return ids[channel_of_slot(slot)] == id and bool(l["slot_ready"][channel_of_slot(slot)])


## The upkeeps measured since the last call: (GPU ms, bake taps, light cells) per frame.
func take_feedback() -> Array[Vector3]:
	_mutex.lock()
	var f := _feedback
	_feedback = []
	_mutex.unlock()
	return f


## Forgets the adopted state (the schedule restarted: nothing of the old volume is shown).
func forget() -> void:
	_mutex.lock()
	_lookup = {}
	_batch = {}
	_mutex.unlock()


# ---------------------------------------------------------------- render thread

## The batch handed over ({} = none).
func take() -> Dictionary:
	_mutex.lock()
	var b := _batch
	_batch = {}
	_mutex.unlock()
	return b


func density_texture(k: int) -> RID:
	return _texture(k, "density")


func light_texture(k: int) -> RID:
	return _texture(k, "light")


func max_extinction_texture(k: int) -> RID:
	return _texture(k, "max")


func distance_texture(k: int) -> RID:
	return _texture(k, "distance")


func _texture(k: int, key: String) -> RID:
	return _levels[k].get(key, RID()) if k < _levels.size() else RID()


## Creates the kernels and the levels' textures (cleared: an unbaked cell reads 0). false on an error (printed once).
func ensure(rd: RenderingDevice) -> bool:
	if not _shaders.is_empty():
		return true
	if _failed or rd == null:
		return false
	_rd = rd
	var kernels := {"bake": ["CLOUD_VOLUME_BAKE", "CLOUD_VOLUME_BLOCK_BINDING 0"],
		"max": ["CLOUD_VOLUME_OCCUPANCY", "CLOUD_VOLUME_BLOCK_BINDING 0"],
		"distance": ["CLOUD_VOLUME_OCCUPANCY", "CLOUD_VOLUME_DISTANCE", "CLOUD_VOLUME_BLOCK_BINDING 0"],
		"probe": ["CLOUD_VOLUME_PROBE", "CLOUD_VOLUME_LOOKUP"]}
	for name: String in kernels:
		var src := RDShaderSource.new()
		src.language = RenderingDevice.SHADER_LANGUAGE_GLSL
		src.source_compute = CloudShaderPort.build(PackedStringArray(kernels[name]), PackedStringArray(FILES)).source
		var spirv := rd.shader_compile_spirv_from_source(src)
		if spirv.compile_error_compute != "":
			push_error("CloudWorldVolume %s: %s" % [name, spirv.compile_error_compute])
			_failed = true
			return false
		var shader := rd.shader_create_from_spirv(spirv)
		if not shader.is_valid():
			push_error("CloudWorldVolume %s: shader not created" % name)
			_failed = true
			return false
		_shaders[name] = shader
		_pipelines[name] = rd.compute_pipeline_create(shader)
	_volume_buffer = rd.uniform_buffer_create(VOLUME_BLOCK_SIZE)
	_probe_buffer = rd.uniform_buffer_create(PROBE_BLOCK_SIZE)
	for linear in [true, false]:
		var state := RDSamplerState.new()
		var filter := RenderingDevice.SAMPLER_FILTER_LINEAR if linear else RenderingDevice.SAMPLER_FILTER_NEAREST
		state.min_filter = filter
		state.mag_filter = filter
		state.repeat_u = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
		state.repeat_v = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
		state.repeat_w = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
		if linear:
			_linear = rd.sampler_create(state)
		else:
			_nearest = rd.sampler_create(state)
	_levels.resize(TEXTURES)
	for i in range(TEXTURES):
		_levels[i] = {}
	for k in range(CloudWorldVolumeLayout.LEVELS):
		if not _create_level(k):
			_failed = true
			return false
	var mip := CloudWorldVolumeLayout.mip_size()
	for i in range(2):
		_scratch.append(_volume(mip, DISTANCE_FORMAT))
	return true


func _create_level(index: int) -> bool:
	var L := CloudWorldVolumeLayout
	var size := Vector3i(L.CELLS_H, L.CELLS_H, L.CELLS_V)
	var level := {"density": _volume(size, DENSITY_FORMAT), "light": _volume(size, LIGHT_FORMAT),
		"max": _volume(L.mip_size(), MAX_FORMAT), "distance": _volume(L.mip_size(), DISTANCE_FORMAT)}
	for rid: RID in level.values():
		if not rid.is_valid():
			push_error("CloudWorldVolume: a texture of level %d was not created" % index)
			return false
	_levels[index] = level
	return true


func _volume(size: Vector3i, format: int) -> RID:
	var tf := RDTextureFormat.new()
	tf.texture_type = RenderingDevice.TEXTURE_TYPE_3D
	tf.width = size.x
	tf.height = size.y
	tf.depth = size.z
	tf.format = format
	# Copy-to: cleared once; copy-from: the checks read them back.
	tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT \
			| RenderingDevice.TEXTURE_USAGE_CAN_COPY_TO_BIT | RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
	var t := _rd.texture_create(tf, RDTextureView.new())
	if t.is_valid():
		_rd.texture_clear(t, Color(0, 0, 0, 0), 0, 1, 0, 1)
	return t


## Runs one batch (render thread, before the march): its bake jobs, the occupancy of the levels they touched, then
## adopts its lookup. Light and provisional jobs wait for Task 6. rd null (headless checks): adopts only.
## set0: the march's set 0 of this frame (the snapshots' fields, the noise, the tables); camera_earth_m: the camera of
## the same pass (CloudEffect's, float64 Earth-local render axes), so the CloudVolume block and set 0 agree.
func run(rd: RenderingDevice, batch: Dictionary, set0: Array[RDUniform], camera_earth_m: Vector3) -> void:
	if batch.is_empty():
		return
	if rd != null and ensure(rd):
		_read_timestamps()
		_last_set0 = set0
		rd.capture_timestamp(UPKEEP_STAMP)
		_upkeep(batch, set0, camera_earth_m)
		rd.capture_timestamp(UPKEEP_END_STAMP)
		_pending[Engine.get_frames_drawn()] = Vector2(last_bake_taps, last_light_cells)
		while _pending.size() > PENDING_MAX:
			_pending.erase(_pending.keys().min())
	_mutex.lock()
	_lookup = batch["lookup"]
	_mutex.unlock()


func _upkeep(batch: Dictionary, set0: Array[RDUniform], camera: Vector3) -> void:
	var jobs: Array = batch.get("bake", [])
	var lookup: Dictionary = batch["lookup"]
	var frames := _frames_of(lookup)
	_rd.buffer_update(_volume_buffer, 0, VOLUME_BLOCK_SIZE, _volume_bytes(lookup, frames, camera))
	last_bake_cells = 0
	last_bake_taps = 0
	last_light_cells = 0
	if jobs.is_empty():
		_finish_reframe(batch)
		return
	var same := _slots_identical(lookup)
	var touched := {}  ## texture index -> Array of {from, to} (cells)
	var bytes: Array[PackedByteArray] = []
	for job: Dictionary in jobs:
		var index: int = SPARE if bool(job["spare"]) else int(job["level"])
		if _levels[index].is_empty() and not _create_level(index):
			return
		var origin: Vector3 = lookup["build_spare"]["frame_origin_earth_m"] if bool(job["spare"]) \
				else lookup["levels"][int(job["level"])]["frame_origin_earth_m"]
		var from: Vector3i = job["from"]
		var to: Vector3i = job["to"]
		bytes.append(_job_bytes(int(job["level"]), from, to - from, int(job["snapshot_channel"]), frames.find(origin),
				same))
		if not touched.has(index):
			touched[index] = []
		touched[index].append({"from": from, "to": to, "level": int(job["level"])})
		var cells := CloudWorldVolumeLayout.box_cells(job)
		last_bake_cells += cells
		last_bake_taps += cells * CloudWorldVolumeLayout.cell_taps(int(job["level"]))
	while _job_buffers.size() < bytes.size():
		_job_buffers.append(_rd.uniform_buffer_create(JOB_BLOCK_SIZE))
	for i in range(bytes.size()):
		_rd.buffer_update(_job_buffers[i], 0, JOB_BLOCK_SIZE, bytes[i])
	# The occupancy's blocks: per touched texture the bake boxes grown by the trilinear support, in blocks.
	var occupancy: Array[Dictionary] = []  ## {index, kernel, first, size, pass}
	for index: int in touched:
		for box: Dictionary in _block_boxes(touched[index]):
			occupancy.append({"index": index, "kernel": "max", "first": box["from"], "size": box["to"] - box["from"],
					"pass": 0})
		for pass_index in range(1, 4):
			occupancy.append({"index": index, "kernel": "distance", "first": Vector3i.ZERO,
					"size": CloudWorldVolumeLayout.mip_size(), "pass": pass_index})
	while _occupancy_buffers.size() < occupancy.size():
		_occupancy_buffers.append(_rd.uniform_buffer_create(OCCUPANCY_BLOCK_SIZE))
	for i in range(occupancy.size()):
		var o: Dictionary = occupancy[i]
		var first: Vector3i = o["first"]
		var size: Vector3i = o["size"]
		_rd.buffer_update(_occupancy_buffers[i], 0, OCCUPANCY_BLOCK_SIZE, PackedInt32Array([first.x, first.y, first.z,
				o["pass"], size.x, size.y, size.z, DISTANCE_CAP_BLOCKS]).to_byte_array())
	# The bake (its own compute list: the occupancy then samples the density).
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _pipelines["bake"])
	_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders["bake"], 0, set0), 0)
	for i in range(jobs.size()):
		var job: Dictionary = jobs[i]
		var index: int = SPARE if bool(job["spare"]) else int(job["level"])
		var size: Vector3i = (job["to"] as Vector3i) - (job["from"] as Vector3i)
		_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders["bake"], 1, [
				_ubo(0, _volume_buffer), _ubo(1, _job_buffers[i]), _image(2, _levels[index]["density"])]), 1)
		_rd.compute_list_dispatch(list, size.x, size.y, size.z)  # one group per cell
	_rd.compute_list_end()
	# The max mip, then the three distance passes, level by level (the scratch volumes are shared).
	list = _rd.compute_list_begin()
	for i in range(occupancy.size()):
		var o: Dictionary = occupancy[i]
		var level: Dictionary = _levels[int(o["index"])]
		var pass_index: int = o["pass"]
		var input: RID = [level["density"], level["max"], _scratch[0], _scratch[1]][pass_index]
		var output: RID = [level["max"], _scratch[0], _scratch[1], level["distance"]][pass_index]
		var kernel: String = o["kernel"]
		_rd.compute_list_bind_compute_pipeline(list, _pipelines[kernel])
		_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders[kernel], 0, set0), 0)
		_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders[kernel], 1, [
				_ubo(0, _volume_buffer), _ubo(1, _occupancy_buffers[i]), _tex(2, _nearest, input),
				_image(3, output)]), 1)
		var size: Vector3i = o["size"]
		_rd.compute_list_dispatch(list, (size.x - 1) / GROUP + 1, (size.y - 1) / GROUP + 1, (size.z - 1) / GROUP + 1)
		_rd.compute_list_add_barrier(list)
	_rd.compute_list_end()
	_finish_reframe(batch)


## The spare that finished its cross-fade is now the level (CloudWorldVolumeSchedule._swap_spare): its textures take
## the level's place; the level's old ones are freed (the RenderingDevice frees them once no frame in flight uses them).
func _finish_reframe(batch: Dictionary) -> void:
	var rf: Dictionary = batch.get("reframe", {})
	var k := int(rf.get("level", -1))
	if k < 0 or bool(rf.get("spare", true)) or _levels[SPARE].is_empty():
		return
	var old: Dictionary = _levels[k]
	_levels[k] = _levels[SPARE]
	_levels[SPARE] = {}
	for rid: RID in old.values():
		_rd.free_rid(rid)


## The max mip's boxes (global blocks, half-open) whose texels read cells of the baked boxes (a block's support is its
## cells and the one-cell ring); a box covering a level's whole mip along an axis is cut to it.
static func _block_boxes(boxes: Array) -> Array[Dictionary]:
	var L := CloudWorldVolumeLayout
	var mip := L.mip_size()
	var out: Array[Dictionary] = []
	for box: Dictionary in boxes:
		var from: Vector3i = box["from"]
		var to: Vector3i = box["to"]
		var b0 := Vector3i(ceili((from.x - L.MIP_DIV) / float(L.MIP_DIV)), ceili((from.y - L.MIP_DIV) / float(L.MIP_DIV)),
				ceili((from.z - L.MIP_DIV) / float(L.MIP_DIV)))
		var b1 := Vector3i(floori(to.x / float(L.MIP_DIV)), floori(to.y / float(L.MIP_DIV)),
				floori(to.z / float(L.MIP_DIV))) + Vector3i.ONE
		for axis in 3:
			if b1[axis] - b0[axis] > mip[axis]:
				b1[axis] = b0[axis] + mip[axis]
		out.append({"from": b0, "to": b1})
	return out


func _slots_identical(lookup: Dictionary) -> bool:
	var ids: Array = lookup.get("slot_ids", [0, 1])
	return ids[0] == ids[1]


## The distinct frame origins of the levels (the finest active level's first) and of a spare being built (<= 2).
func _frames_of(lookup: Dictionary) -> Array[Vector3]:
	var frames: Array[Vector3] = []
	for lv: Dictionary in lookup["levels"]:
		if bool(lv["active"]) and not frames.has(lv["frame_origin_earth_m"]):
			frames.append(lv["frame_origin_earth_m"])
	for key in ["build_spare", "spare"]:
		if lookup.has(key) and not frames.has(lookup[key]["frame_origin_earth_m"]):
			frames.append(lookup[key]["frame_origin_earth_m"])
	if frames.is_empty():
		frames.append((lookup["levels"][0] as Dictionary)["frame_origin_earth_m"])
	return frames


## The CloudVolume block (std140) for a lookup state and the camera (float64 Earth-local render axes): per frame the
## camera terms differenced in float64 (frame_floats), per level its window origin and flags, the spare.
func globals_bytes(lookup: Dictionary, camera_earth_m: Vector3) -> PackedByteArray:
	return _volume_bytes(lookup, _frames_of(lookup), camera_earth_m)


func _volume_bytes(lookup: Dictionary, frames: Array[Vector3], camera: Vector3) -> PackedByteArray:
	var floats := PackedFloat32Array()
	for f in range(2):
		floats.append_array(frame_floats(_layout_for(frames[mini(f, frames.size() - 1)]), camera))
	var bytes := floats.to_byte_array()
	var ints := PackedInt32Array()
	for lv: Dictionary in lookup["levels"]:
		var origin: Vector3i = lv["origin"]
		var flags := (FLAG_ACTIVE if bool(lv["active"]) else 0) | (FLAG_READY if bool(lv.get("shown", lv["ready"])) else 0) \
				| (FLAG_VERTICAL_FADE if bool(lv["vertical_fade"]) else 0) \
				| (FLAG_CLAMPED_BOTTOM if bool(lv["clamped_bottom"]) else 0)
		flags |= maxi(frames.find(lv["frame_origin_earth_m"]), 0) << FLAG_FRAME_SHIFT
		ints.append_array([origin.x, origin.y, origin.z, flags])
	var spare: Dictionary = lookup.get("spare", lookup.get("build_spare", {}))
	var spare_origin: Vector3i = spare.get("origin", Vector3i.ZERO)
	var spare_flags := 0
	if not spare.is_empty():
		var k: int = spare["level"]
		spare_flags = FLAG_ACTIVE | (FLAG_READY if lookup.has("spare") else 0) \
				| (FLAG_VERTICAL_FADE if k < CloudWorldVolumeLayout.WINDOWED_LEVELS else 0) \
				| (FLAG_CLAMPED_BOTTOM if spare_origin.z == 0 else 0) \
				| (maxi(frames.find(spare["frame_origin_earth_m"]), 0) << FLAG_FRAME_SHIFT)
	ints.append_array([spare_origin.x, spare_origin.y, spare_origin.z, spare_flags])
	bytes.append_array(ints.to_byte_array())
	bytes.append_array(PackedFloat32Array([float(spare.get("level", -1)), float(spare.get("fade", 0.0)), 0.0, 0.0])
			.to_byte_array())
	return bytes


func _layout_for(origin: Vector3) -> CloudWorldVolumeLayout:
	if not _layouts.has(origin):
		if _layouts.size() > 4:
			_layouts.clear()
		var layout := CloudWorldVolumeLayout.new()
		layout.set_frame(origin)
		_layouts[origin] = layout
	return _layouts[origin]


## One frame's CloudVolumeFrame (cloud_volume.gdshaderinc) for the camera, computed in float64: the camera's normal
## split into its float32 part n_h and the rest, the reference grid point R (a, b) t / sin(t) of n_h (the inverse map's
## own value for it), the unit normal there minus the camera's.
static func frame_floats(layout: CloudWorldVolumeLayout, camera: Vector3) -> PackedFloat32Array:
	var geo := layout.geodetic_of(camera)
	var n := Vector3(geo.x, geo.y, geo.z)
	var h := geo.w
	var n_h := _f32(n)
	var n_lo := n - n_h
	var axes := layout.axes()
	var a := n_h.dot(axes[1])
	var b := n_h.dot(axes[2])
	var z := n_h.dot(axes[0])
	var r := sqrt(a * a + b * b)
	var t := atan2(r, z)
	var radius := layout.sphere_radius_m
	var x_over_sin := t / sin(t) if t > 1e-12 else 1.0
	var grid := Vector3(radius * a * x_over_sin, radius * b * x_over_sin, h)
	var grid_hi := _f32(grid)
	var unit := layout.up_of_grid(grid.x, grid.y) - n
	var big_n := CloudLayer.WGS84_A_M / sqrt(1.0 - CloudWorldVolumeLayout.E2 * n.y * n.y)
	return PackedFloat32Array([grid_hi.x, grid_hi.y, grid_hi.z, radius,
			grid.x - grid_hi.x, grid.y - grid_hi.y, grid.z - grid_hi.z, 0.0,
			n_h.x, n_h.y, n_h.z, big_n,
			n_lo.x, n_lo.y, n_lo.z, n_h.length_squared() - 1.0,
			axes[0].x, axes[0].y, axes[0].z, big_n + h,
			axes[1].x, axes[1].y, axes[1].z, 0.0,
			axes[2].x, axes[2].y, axes[2].z, 0.0,
			a, b, z, r,
			unit.x, unit.y, unit.z, 0.0])


## The float32 value of each component (as float64).
static func _f32(v: Vector3) -> Vector3:
	var f := PackedFloat32Array([v.x, v.y, v.z])
	return Vector3(f[0], f[1], f[2])


## A bake job's CloudVolumeJob block.
static func _job_bytes(level: int, first: Vector3i, size: Vector3i, channel: int, frame: int, same: bool) \
		-> PackedByteArray:
	return PackedInt32Array([first.x, first.y, first.z, level, size.x, size.y, size.z, channel,
			maxi(frame, 0), 1 if same else 0, 0, 0]).to_byte_array()


## The upkeep intervals of earlier frames whose timestamps came back, with the work they ran (take_feedback).
func _read_timestamps() -> void:
	var frame := _rd.get_captured_timestamps_frame()
	if not _pending.has(frame):
		return
	var work: Vector2 = _pending[frame]
	_pending.erase(frame)
	var count := _rd.get_captured_timestamps_count()
	for i in range(count - 1):
		if _rd.get_captured_timestamp_name(i) == UPKEEP_STAMP:
			last_upkeep_ms = float(_rd.get_captured_timestamp_gpu_time(i + 1)
					- _rd.get_captured_timestamp_gpu_time(i)) / 1.0e6
			_mutex.lock()
			_feedback.append(Vector3(last_upkeep_ms, work.x, work.y))
			if _feedback.size() > FEEDBACK_MAX:
				_feedback = _feedback.slice(_feedback.size() - FEEDBACK_MAX)
			_mutex.unlock()
			return


## Diagnostics (cloud_check.gd section 16; main thread, after a frame ran a batch): mode 0 cloud_volume_grid_in of
## camera offsets (points: xyz the offset, w the frame index), mode 1 texture `index`'s raw density [1/m] at grid points
## (level k), mode 2 the occupancy lookups at grid points of level k (cloud_volume_free_m [m],
## cloud_volume_max_extinction [1/m], the level's weight), mode 3 the density both snapshots give at grid points of
## level k as cloud_volume_density blends it toward level k + 1 [1/m]; the lookup state and the camera as given.
## Returns 4 floats per point.
func probe(rd: RenderingDevice, mode: int, points: PackedFloat32Array, lookup: Dictionary, camera: Vector3,
		index := 0, k := 0) -> PackedFloat32Array:
	if not ensure(rd) or _last_set0.is_empty():
		return PackedFloat32Array()
	var count := points.size() / 4
	rd.buffer_update(_volume_buffer, 0, VOLUME_BLOCK_SIZE, globals_bytes(lookup, camera))
	rd.buffer_update(_probe_buffer, 0, PROBE_BLOCK_SIZE, PackedInt32Array([count, mode, index, k]).to_byte_array())
	var input := rd.storage_buffer_create(points.size() * 4, points.to_byte_array())
	var output := rd.storage_buffer_create(points.size() * 4)
	var set1: Array[RDUniform] = [_ubo(BLOCK_BINDING, _volume_buffer), _ubo(1, _probe_buffer), _storage(2, input),
			_storage(3, output)]
	var groups := ["density", "light", "max", "distance"]
	for g in range(groups.size()):
		for i in range(TEXTURES):
			var level: Dictionary = _levels[i] if not _levels[i].is_empty() else _levels[0]
			set1.append(_tex(BLOCK_BINDING + 1 + 9 * g + i, _linear if g < 2 else _nearest, level[groups[g]]))
	var list := rd.compute_list_begin()
	rd.compute_list_bind_compute_pipeline(list, _pipelines["probe"])
	rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders["probe"], 0, _last_set0), 0)
	rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders["probe"], 1, set1), 1)
	rd.compute_list_dispatch(list, (count - 1) / PROBE_GROUP + 1, 1, 1)
	rd.compute_list_end()
	var result := rd.buffer_get_data(output).to_float32_array()
	rd.free_rid(input)
	rd.free_rid(output)
	return result


## Frees the kernels and textures (render thread, the effect's teardown).
func free_resources(rd: RenderingDevice) -> void:
	if rd == null:
		return
	for level: Dictionary in _levels:
		for rid: RID in level.values():
			rd.free_rid(rid)
	_levels.clear()
	var rids: Array = _scratch + _job_buffers + _occupancy_buffers + [_volume_buffer, _probe_buffer, _linear, _nearest]
	for rid: RID in rids:
		if rid.is_valid():
			rd.free_rid(rid)
	for shader: RID in _shaders.values():
		rd.free_rid(shader)
	_shaders.clear()
	_scratch.clear()
	_job_buffers.clear()
	_occupancy_buffers.clear()


static func _ubo(binding: int, buffer: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	u.binding = binding
	u.add_id(buffer)
	return u


static func _image(binding: int, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	u.binding = binding
	u.add_id(texture)
	return u


static func _tex(binding: int, sampler: RID, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
	u.binding = binding
	u.add_id(sampler)
	u.add_id(texture)
	return u


static func _storage(binding: int, buffer: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_STORAGE_BUFFER
	u.binding = binding
	u.add_id(buffer)
	return u
