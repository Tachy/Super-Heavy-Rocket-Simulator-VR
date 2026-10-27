class_name CloudViewCache
extends RefCounted
## The world-fixed cloud view cache's GPU side (cloud view cache Task 3): per resolution level a cube texture of the
## march's colour + the background's transmittance T (RGBA16F) and one of the alpha-weighted cloud distance (R32F),
## faces and texels as CloudViewCacheLayout (level texels per face = face_texels / 4, / 2, / 1), and their fill: the
## march's own per-ray shading (cloud_pass.gdshaderinc cloud_shade_ray) for the texels of a list of tiles, from an
## anchor (shaders/cloud_view_cache.gdshaderinc CLOUD_CACHE_FILL). Render thread only (CloudEffect owns it).
## Memory at the VR face size (3456): the full level 6 x 3456^2 x 12 B = 0.86 GB, all three 1.13 GB (cap 2 GB, spec).

const COLOR_FORMAT := RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT
const DISTANCE_FORMAT := RenderingDevice.DATA_FORMAT_R32_SFLOAT
const BLOCK := 8  ## the fill's work group side (texels)
const VIEW_BLOCK_SIZE := 4 * 64 + 5 * 16  ## CloudView (cloud_pass.gdshaderinc), as CloudEffect.VIEW_BLOCK_SIZE
const PUSH_SIZE := 16  ## CloudCacheFill: face_texels, tiles_per_side, first, pad

var layout: CloudViewCacheLayout

var _textures: Array[RID] = []  ## per level: colour + T
var _distances: Array[RID] = []  ## per level: the cloud distance
var _view_buffers: Array[RID] = []  ## per level: the fill's CloudView block (anchor offset, texel angle)
var _tiles := RID()  ## the tile list (uint per entry, sorted by level)
var _kernels := {}  ## "fill" / "fill_prefilter" -> {shader, pipeline} (CloudEffect's)
var _prefilter_needed: Callable  ## texel angle [rad] -> whether the shape prefilter's kernel is needed


## kernels: {"fill": {shader, pipeline}, "fill_prefilter": {...}}; prefilter_needed(pixel_angle) as CloudEffect's.
func _init(rd: RenderingDevice, layout_: CloudViewCacheLayout, kernels: Dictionary, prefilter_needed: Callable) -> void:
	layout = layout_
	_kernels = kernels
	_prefilter_needed = prefilter_needed
	for level in range(CloudViewCacheLayout.LEVELS):
		var texels := layout.face_texels >> (CloudViewCacheLayout.LEVELS - 1 - level)
		_textures.append(_cube(rd, texels, COLOR_FORMAT))
		_distances.append(_cube(rd, texels, DISTANCE_FORMAT))
		_view_buffers.append(rd.uniform_buffer_create(VIEW_BLOCK_SIZE))
	assert((layout.face_texels / layout.tiles_per_side) % BLOCK == 0, "tile size must be a multiple of the 8 x 8 work group")
	# A {tile, level} list may hold one tile at several levels.
	_tiles = rd.storage_buffer_create(4 * layout.tile_count() * CloudViewCacheLayout.LEVELS)


## The colour + T cube (6 layers) of a level.
func texture(level: int) -> RID:
	return _textures[level]


## The cloud distance cube of a level.
func distance_texture(level: int) -> RID:
	return _distances[level]


## Fills the texels of tiles ({tile, level} each) by the march's shading from the anchor - its Earth-local offset from
## the camera (float64 difference on the CPU), the frame's set 0 uniforms (the march's globals: CloudEffect). Before the
## frame's march, in its own compute list (no buffer updates inside a list).
func fill(rd: RenderingDevice, tiles: Array[Dictionary], anchor_offset_earth: Vector3,
		set0: Array[RDUniform]) -> void:
	var by_level: Array[PackedInt32Array] = []
	for level in range(CloudViewCacheLayout.LEVELS):
		by_level.append(PackedInt32Array())
	for t in tiles:
		by_level[int(t["level"])].append(int(t["tile"]))
	var entries := PackedInt32Array()
	for level in range(CloudViewCacheLayout.LEVELS):
		entries.append_array(by_level[level])
	if entries.is_empty():
		return
	var capacity := layout.tile_count() * CloudViewCacheLayout.LEVELS
	if entries.size() > capacity:
		push_error("CloudViewCache.fill: tile list of %d entries exceeds the buffer (%d); the rest is dropped"
				% [entries.size(), capacity])
		assert(false, "cloud view cache tile list overflow")
		entries.resize(capacity)
		# Keep by_level consistent with the truncated list.
		var left := capacity
		for level in range(CloudViewCacheLayout.LEVELS):
			if by_level[level].size() > left:
				by_level[level].resize(left)
			left -= by_level[level].size()
	rd.buffer_update(_tiles, 0, 4 * entries.size(), entries.to_byte_array())
	var angles := PackedFloat32Array()
	for level in range(CloudViewCacheLayout.LEVELS):
		# The texel's angle at a face centre (the tangent mapping's coarsest): the detail's band limit, as the march's
		# pixel angle.
		var texel_angle := 2.0 / float(layout.face_texels >> (CloudViewCacheLayout.LEVELS - 1 - level))
		angles.append(texel_angle)
		if not by_level[level].is_empty():
			rd.buffer_update(_view_buffers[level], 0, VIEW_BLOCK_SIZE,
					_view_block(anchor_offset_earth, texel_angle).to_byte_array())
	var list := rd.compute_list_begin()
	var first := 0
	for level in range(CloudViewCacheLayout.LEVELS):
		var count := by_level[level].size()
		if count == 0:
			continue
		var kernel: Dictionary = _kernels["fill_prefilter" if _prefilter_needed.call(angles[level]) else "fill"]
		var texels := layout.face_texels >> (CloudViewCacheLayout.LEVELS - 1 - level)
		var blocks := texels / layout.tiles_per_side / BLOCK
		rd.compute_list_bind_compute_pipeline(list, kernel["pipeline"])
		rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(kernel["shader"], 0, set0), 0)
		var set1: Array[RDUniform] = [CloudEffect._ubo(_view_buffers[level]), CloudEffect._image(1, _textures[level]),
				CloudEffect._image(2, _distances[level]), CloudEffect._storage(3, _tiles)]
		rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(kernel["shader"], 1, set1), 1)
		var push := PackedInt32Array([texels, layout.tiles_per_side, first, 0]).to_byte_array()
		rd.compute_list_set_push_constant(list, push, PUSH_SIZE)
		rd.compute_list_dispatch(list, blocks * blocks, count, 1)
		first += count
	rd.compute_list_end()


## Frees the GPU resources (Object.free is not this class's to override).
func free_resources(rd: RenderingDevice) -> void:
	for rid: RID in _textures + _distances + _view_buffers + [_tiles]:
		if rid.is_valid():
			rd.free_rid(rid)
	_textures.clear()
	_distances.clear()
	_view_buffers.clear()
	_tiles = RID()


## The fill's CloudView block: only what the march reads besides the frame's matrices - params (jitter 0, weight 1, -,
## the texel angle) and eye_earth (the anchor's offset; frame 0: the jittered march's sequences unmoved).
static func _view_block(anchor_offset_earth: Vector3, texel_angle: float) -> PackedFloat32Array:
	var floats := PackedFloat32Array()
	floats.resize(VIEW_BLOCK_SIZE / 4)
	var params := (4 * 64 + 2 * 16) / 4  # after the four matrices, sizes and camera_delta
	floats[params + 1] = 1.0
	floats[params + 3] = texel_angle
	floats[params + 4] = anchor_offset_earth.x
	floats[params + 5] = anchor_offset_earth.y
	floats[params + 6] = anchor_offset_earth.z
	return floats


static func _cube(rd: RenderingDevice, texels: int, format: int) -> RID:
	var tf := RDTextureFormat.new()
	tf.texture_type = RenderingDevice.TEXTURE_TYPE_CUBE
	tf.width = texels
	tf.height = texels
	tf.depth = 1
	tf.array_layers = 6
	tf.format = format
	# Copy-from: the checks read the cache back (cloud_check section 15).
	tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT \
			| RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
	return rd.texture_create(tf, RDTextureView.new())
