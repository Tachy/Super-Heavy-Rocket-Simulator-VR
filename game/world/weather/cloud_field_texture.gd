class_name CloudFieldTexture
extends RefCounted
## One volume of a weather snapshot on the GPU (M8 weather and clouds, Task 8): the field (RGBA8) or the ice share (R8),
## columns x rows x 32 layers, no mipmaps. Uploaded in chunks of LAYERS_PER_STEP layers, one chunk per step() (per
## frame), so a 133 MB 0.25-deg field never blocks a frame (one ImageTexture3D creation took 49 ms on the main thread):
## the chunk goes into a small staging volume (texture_update) and is copied into its slices of the target on the GPU.
## Without a RenderingDevice (headless tests) the whole volume is created at once as an ImageTexture3D.

## [E] Layers per frame: one 4.2 MB layer of a 0.25-deg RGBA8 field (32 + 32 frames per snapshot with its ice share;
## render_check's "upload" line: the longest step 5.2 ms with one layer, 6.6 ms with two).
const LAYERS_PER_STEP := 1
const LAYERS := 32

var texture: Texture3D  ## valid once done
var done := false
var bytes := 0
var max_step_ms := 0.0  ## the longest step (main / render thread)

var _rd: RenderingDevice
var _columns := 0
var _rows := 0
var _texel_bytes := 4
var _data := PackedByteArray()
var _next_layer := 0
var _target := RID()
var _staging := RID()
var _format := 0


## texels: layer-major, texel_bytes per texel (4: RGBA8, 1: R8).
static func create(texels: PackedByteArray, columns: int, rows: int, texel_bytes: int) -> CloudFieldTexture:
	var t := CloudFieldTexture.new()
	t._columns = columns
	t._rows = rows
	t._texel_bytes = texel_bytes
	t._data = texels
	t.bytes = columns * rows * LAYERS * texel_bytes
	t._rd = RenderingServer.get_rendering_device()
	if t._rd == null:
		t._create_at_once()
	return t


## Uploads the next chunk (or everything: all = true, for checks).
func step(all := false) -> void:
	if done:
		return
	RenderingServer.call_on_render_thread(_step_on_render_thread.bind(all))


func _step_on_render_thread(all: bool) -> void:
	if done:
		return
	var start := Time.get_ticks_usec()
	_format = RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM if _texel_bytes == 4 else RenderingDevice.DATA_FORMAT_R8_UNORM
	if not _target.is_valid():
		var tf := RDTextureFormat.new()
		tf.texture_type = RenderingDevice.TEXTURE_TYPE_3D
		tf.width = _columns
		tf.height = _rows
		tf.depth = LAYERS
		tf.format = _format
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_CAN_COPY_TO_BIT
		_target = _rd.texture_create(tf, RDTextureView.new())
		tf.depth = LAYERS_PER_STEP
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT | RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
		_staging = _rd.texture_create(tf, RDTextureView.new())
	var layer_bytes := _columns * _rows * _texel_bytes
	while _next_layer < LAYERS:
		var count := mini(LAYERS_PER_STEP, LAYERS - _next_layer)
		var chunk := _data.slice(_next_layer * layer_bytes, (_next_layer + count) * layer_bytes)
		if count < LAYERS_PER_STEP:
			chunk.resize(LAYERS_PER_STEP * layer_bytes)
		_rd.texture_update(_staging, 0, chunk)
		_rd.texture_copy(_staging, _target, Vector3.ZERO, Vector3(0, 0, _next_layer), Vector3(_columns, _rows, count),
				0, 0, 0, 0)
		_next_layer += count
		if not all:
			break
	max_step_ms = maxf(max_step_ms, (Time.get_ticks_usec() - start) / 1000.0)
	if _next_layer >= LAYERS:
		_rd.free_rid(_staging)
		_staging = RID()
		_data = PackedByteArray()
		var rd_texture := Texture3DRD.new()
		rd_texture.texture_rd_rid = _target
		texture = rd_texture
		done = true


func _create_at_once() -> void:
	var start := Time.get_ticks_usec()
	var format := Image.FORMAT_RGBA8 if _texel_bytes == 4 else Image.FORMAT_R8
	var layer_bytes := _columns * _rows * _texel_bytes
	var images: Array[Image] = []
	for layer in range(LAYERS):
		var data := _data.slice(layer * layer_bytes, (layer + 1) * layer_bytes)
		if data.size() < layer_bytes:
			data.resize(layer_bytes)
		images.append(Image.create_from_data(_columns, _rows, false, format, data))
	var image_texture := ImageTexture3D.new()
	image_texture.create(format, _columns, _rows, LAYERS, false, images)
	texture = image_texture
	_data = PackedByteArray()
	max_step_ms = (Time.get_ticks_usec() - start) / 1000.0
	done = true


## Frees the GPU volume (the slot no longer shows it) or an unfinished upload.
func release() -> void:
	if texture is Texture3DRD:
		(texture as Texture3DRD).texture_rd_rid = RID()
	texture = null
	done = true  # no further steps
	if _rd == null:
		return
	var rids: Array[RID] = [_target, _staging]
	_target = RID()
	_staging = RID()
	var rd := _rd
	RenderingServer.call_on_render_thread(func() -> void:
		for rid in rids:
			if rid.is_valid():
				rd.free_rid(rid))
