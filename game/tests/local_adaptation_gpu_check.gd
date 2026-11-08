# CPU-vs-GPU check of the local adaptation (LocalAdaptation vs the generated shaders of LocalAdaptationEffect, look fixes
# T10 fix round 1): needs a GPU (no --headless); uses a local RenderingDevice, so it can run next to a render in another
# process. Images: antialiased discs (1e4 cd/m^2 over a 1e-3 cd/m^2 sky, headset numbers: 31.5 px/deg, block 8), the
# input rounded to fp16 for both sides. The factor c is read back as output / input on a grey image (cone part = identity)
# and compared with LocalAdaptation.factors. Second part: the two eyes of a stereo pair see the disc at disparities of
# 0 .. 8 px (a near object; the Moon and the Earth have none); both eyes use the same block size and pooling footprint (one
# block size per frame, from view 0), so only the grid phase differs: the gain difference between the eyes is printed and
# bounded.
# godot --path game --script res://tests/local_adaptation_gpu_check.gd
extends SceneTree

const TOA_LUX := 127000.0
const NIGHT_GAIN := 7557.0 * 0.368
const PPD := 31.5
## Largest |c_gpu / c_cpu - 1| over pixels with c < 0.99 and input above 1e-4 scene units (fp16 output, exp2 / log2 of
## the GPU's math).
const TOLERANCE := 0.03
## Largest ratio between the two eyes' displayed peak for a 24 px disc at any disparity (measured 1.12; the CPU analysis
## bounds one eye's phase variation by 1.22).
const EYE_BOUND := 1.2

var _rd: RenderingDevice
var _failures := 0
var _shaders := []
var _pipelines := []


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("local_adaptation_gpu_check: %-52s %s %s" % [label, detail, "ok" if ok else "FAIL"])


func _initialize() -> void:
	_rd = RenderingServer.create_local_rendering_device()
	if _rd == null:
		print("local_adaptation_gpu_check: no rendering device (run without --headless)")
		quit(2)
		return
	for code in [LocalAdaptationEffect.down_shader_source(), LocalAdaptationEffect.pool_shader_source(),
			LocalAdaptationEffect.apply_shader_source()]:
		var source := RDShaderSource.new()
		source.language = RenderingDevice.SHADER_LANGUAGE_GLSL
		source.source_compute = code
		var spirv := _rd.shader_compile_spirv_from_source(source)
		if spirv.compile_error_compute != "":
			print("local_adaptation_gpu_check: shader compile error: ", spirv.compile_error_compute)
			quit(1)
			return
		var shader := _rd.shader_create_from_spirv(spirv)
		_shaders.append(shader)
		_pipelines.append(_rd.compute_pipeline_create(shader))
	var base := Atmosphere.DAY_EXPOSURE * PI / TOA_LUX
	var reference := TOA_LUX / PI
	var spc := base * NIGHT_GAIN
	var w := 96
	var h := 96
	var block := LocalAdaptation.block_size(PPD)
	var parameters := {"scene_per_cd_m2": spc, "display_gain": NIGHT_GAIN, "reference_cd_m2": reference, "block": block,
			"sigma_blocks": LocalAdaptation.POOL_SIGMA_DEG * PPD / float(block)}
	for diameter in [10.0, 16.0, 24.0, 32.0]:
		for shift in [0.0, 2.5, 5.0]:
			var image := _disc(w, h, diameter, 40.0 + shift, 40.0 + 0.6 * shift, spc)
			var rounded := _round_half(image)
			var cpu := LocalAdaptation.factors(rounded, w, h, parameters)
			var gpu := _gpu_factors(rounded, w, h, parameters)
			var worst := 0.0
			var compared := 0
			for i in range(w * h):
				if cpu[i] < 0.99 and rounded[i] > 1.0e-4:
					worst = maxf(worst, absf(gpu[i] / cpu[i] - 1.0))
					compared += 1
			_check("disc %.0f px, shift %.1f: GPU = CPU" % [diameter, shift], worst <= TOLERANCE and compared > 20,
					"worst %.4f over %d px" % [worst, compared])
	# Stereo: the same 24 px disc, the left eye at x, the right at x - disparity.
	var worst_ratio := 1.0
	for disparity in range(0, 9):
		var left := _gpu_factors(_round_half(_disc(w, h, 24.0, 44.0, 44.0, spc)), w, h, parameters)
		var right_image := _round_half(_disc(w, h, 24.0, 44.0 - disparity, 44.0, spc))
		var right := _gpu_factors(right_image, w, h, parameters)
		var left_image := _round_half(_disc(w, h, 24.0, 44.0, 44.0, spc))
		var peak_left := 0.0
		var peak_right := 0.0
		for i in range(w * h):
			peak_left = maxf(peak_left, left_image[i] * left[i])
			peak_right = maxf(peak_right, right_image[i] * right[i])
		var ratio := maxf(peak_left, peak_right) / minf(peak_left, peak_right)
		worst_ratio = maxf(worst_ratio, ratio)
		print("local_adaptation_gpu_check: stereo disparity %d px: peak displayed L %.3f R %.3f (x%.3f)" % [disparity,
				peak_left, peak_right, ratio])
	_check("stereo: left / right gain difference bounded", worst_ratio <= EYE_BOUND, "worst x%.3f" % worst_ratio)
	quit(1 if _failures > 0 else 0)


func _disc(w: int, h: int, diameter: float, cx: float, cy: float, spc: float) -> PackedFloat64Array:
	var image := PackedFloat64Array()
	image.resize(w * h)
	for y in range(h):
		for x in range(w):
			var cover := 0.0
			for sy in range(4):
				for sx in range(4):
					var px := x + (sx + 0.5) / 4.0 - cx
					var py := y + (sy + 0.5) / 4.0 - cy
					if px * px + py * py <= diameter * diameter / 4.0:
						cover += 1.0 / 16.0
			image[y * w + x] = (1.0e4 * cover + 1.0e-3 * (1.0 - cover)) * spc
	return image


func _round_half(image: PackedFloat64Array) -> PackedFloat64Array:
	var bytes := PackedByteArray()
	bytes.resize(2)
	var out := PackedFloat64Array()
	out.resize(image.size())
	for i in range(image.size()):
		bytes.encode_half(0, image[i])
		out[i] = bytes.decode_half(0)
	return out


func _texture(w: int, h: int, format: int, data := PackedByteArray()) -> RID:
	var f := RDTextureFormat.new()
	f.width = w
	f.height = h
	f.format = format
	f.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT \
			| RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
	return _rd.texture_create(f, RDTextureView.new(), [data] if data.size() > 0 else [])


func _uniform(binding: int, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	u.binding = binding
	u.add_id(texture)
	return u


func _dispatch(index: int, uniforms: Array, push: PackedFloat32Array, groups: Vector2i) -> void:
	var set_rid := _rd.uniform_set_create(uniforms, _shaders[index], 0)
	var bytes := push.to_byte_array()
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _pipelines[index])
	_rd.compute_list_bind_uniform_set(list, set_rid, 0)
	_rd.compute_list_set_push_constant(list, bytes, bytes.size())
	_rd.compute_list_dispatch(list, groups.x, groups.y, 1)
	_rd.compute_list_end()


## The GPU's factor c per pixel for a grey image: DOWN, POOL, APPLY with the effect's push constants, output / input.
func _gpu_factors(image: PackedFloat64Array, w: int, h: int, parameters: Dictionary) -> PackedFloat64Array:
	var block: int = parameters["block"]
	var spc: float = parameters["scene_per_cd_m2"]
	var size := Vector2i(w, h)
	var low := Vector2i((w - 1) / block + 1, (h - 1) / block + 1)
	var sigma_blocks: float = parameters["sigma_blocks"]
	var radius := int(ceil(LocalAdaptation.KERNEL_RADIUS_SIGMAS * sigma_blocks))
	var floor_log := LocalAdaptation.log2_floor(spc)
	var k_eff := LocalAdaptation.effective_per_local(spc, parameters["display_gain"], parameters["reference_cd_m2"])
	var bytes := PackedByteArray()
	bytes.resize(w * h * 8)
	for i in range(w * h):
		for k in range(3):
			bytes.encode_half(i * 8 + k * 2, image[i])
		bytes.encode_half(i * 8 + 6, 1.0)
	var color := _texture(w, h, RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT, bytes)
	var none := _texture(1, 1, RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	var blocks := _texture(low.x, low.y, RenderingDevice.DATA_FORMAT_R32_SFLOAT)
	var map := _texture(low.x, low.y, RenderingDevice.DATA_FORMAT_R32G32_SFLOAT)
	var groups := Vector2i((low.x - 1) / 8 + 1, (low.y - 1) / 8 + 1)
	_dispatch(0, [_uniform(0, color), _uniform(1, none), _uniform(2, blocks)],
			LocalAdaptationEffect.down_push(size, block, false, low, floor_log), groups)
	_dispatch(1, [_uniform(0, blocks), _uniform(1, map)], LocalAdaptationEffect.pool_push(low, radius, sigma_blocks,
			floor_log, spc, parameters["display_gain"], parameters["reference_cd_m2"], k_eff), groups)
	_dispatch(2, [_uniform(0, color), _uniform(1, map)], LocalAdaptationEffect.apply_push(size, block, low, floor_log,
			Basis.IDENTITY), Vector2i((w - 1) / 8 + 1, (h - 1) / 8 + 1))
	_rd.submit()
	_rd.sync()
	var out_bytes := _rd.texture_get_data(color, 0)
	var factors := PackedFloat64Array()
	factors.resize(w * h)
	for i in range(w * h):
		var value := out_bytes.decode_half(i * 8)
		factors[i] = value / image[i] if image[i] > 0.0 else 1.0
	for rid in [color, none, blocks, map]:
		_rd.free_rid(rid)
	return factors
