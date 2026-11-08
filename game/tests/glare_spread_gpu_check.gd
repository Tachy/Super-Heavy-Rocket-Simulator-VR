# CPU-vs-GPU check of the glare spread (GlareSpread vs the shaders of GlareSpreadEffect, look fixes T13): needs a GPU (no
# --headless); uses a local RenderingDevice, so it can run next to a render in another process.
# - a grey frame (192 x 128 px, 20 px/deg, a perspective projection, so the dOmega ratio is not 1 off the axis) with a bright
#   disc, a display-white UI rectangle and the Sun's disc: the level-0 veil A_0 of the GPU pyramid (fp16 storage) against the
#   CPU mirror (GlareSpread.spread with the same excess and ratio images): <= TOLERANCE of the peak everywhere, and 0 where
#   nothing spreads (the UI rectangle alone);
# - the Sun's disc masked out of the sources (no veil from a frame whose only source is the masked Sun);
# - the Sun's analytic veil in the APPLY step against GlareSpread.sun_veil_cd_m2 at 5 pixels.
# godot --path game --script res://tests/glare_spread_gpu_check.gd
extends SceneTree

const W := 192
const H := 128
## Largest |gpu - cpu| over the peak of A_0 (fp16 storage of every level; the exp of the blur's weights).
const TOLERANCE := 0.02

var _rd: RenderingDevice
var _effect: GlareSpreadEffect
var _failures := 0
var _ppd := 20.0
var PPD := 20.0  ## px/deg of the case: 20 (base factor 1), 47 (base factor 3: the tent over 3 px)


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("glare_spread_gpu_check: %-52s %s %s" % [label, detail, "ok" if ok else "FAIL"])


func _initialize() -> void:
	_rd = RenderingServer.create_local_rendering_device()
	if _rd == null:
		print("glare_spread_gpu_check: no rendering device (run without --headless)")
		quit(2)
		return
	_effect = GlareSpreadEffect.new(false)
	if not _effect.create_pipelines(_rd, true):
		quit(1)
		return
	for ppd in [6.6, 20.0, 47.0]:
		PPD = ppd
		_case()
	print("glare_spread_gpu_check: %s" % ("FAIL" if _failures > 0 else "PASS"))
	quit(1 if _failures > 0 else 0)


func _case() -> void:
	var projection_y := 2.0 * PPD * 180.0 / (PI * H)
	var projection_x := projection_y * H / W
	var projection := Projection(Vector4(projection_x, 0, 0, 0), Vector4(0, projection_y, 0, 0), Vector4(0.0, 0.0, -1.0, -1.0),
			Vector4(0, 0, -0.1, 0))
	var ray := GlareSpreadEffect.ray_parameters(projection)
	_ppd = GlareSpread.pixels_per_degree(H, projection_y)  # as the effect derives it from the projection
	# The frame: a disc of 1000 around (60, 64), a UI rectangle at 1.2 (display white), the Sun's disc at (140, 40).
	var image := PackedFloat32Array()
	image.resize(W * H)
	for y in range(H):
		for x in range(W):
			var v := 0.002
			if pow(x - 60.0, 2.0) + pow(y - 64.0, 2.0) <= 49.0:
				v = 1000.0
			if x >= 90 and x < 120 and y >= 90 and y < 110:
				v = 1.2
			if pow(x - 140.0, 2.0) + pow(y - 40.0, 2.0) <= 25.0:
				v = 60000.0
			image[y * W + x] = v
	for i in range(image.size()):  # the input rounded to fp16 for both sides
		var b := PackedByteArray()
		b.resize(2)
		b.encode_half(0, image[i])
		image[i] = b.decode_half(0)
	var sun_world := Vector3(ndc_x(140.5) * ray[0] + ray[1], ndc_y(40.5) * ray[2] + ray[3], -1.0).normalized()
	var mask_cos := cos(deg_to_rad(0.5 * 10.0 / PPD) * 1.5)  # the disc of radius 5 px with margin
	var color := _color_texture(image)
	_effect_run(color, ray, sun_world, mask_cos)

	# CPU mirror: excess with the Sun masked, the ratio of the ray.
	var excess := PackedFloat32Array()
	var ratio := PackedFloat32Array()
	excess.resize(W * H)
	ratio.resize(W * H)
	for y in range(H):
		for x in range(W):
			var u := ndc_x(x + 0.5) * ray[0] + ray[1]
			var v := ndc_y(y + 0.5) * ray[2] + ray[3]
			var r := Vector3(u, v, -1.0)
			ratio[y * W + x] = pow(r.length_squared(), -1.5)
			excess[y * W + x] = 0.0 if r.normalized().dot(sun_world) > mask_cos else GlareSpread.excess_luminance(image[y * W + x])
	var cpu := GlareSpread.spread(excess, ratio, W, H, _ppd)
	var view: Dictionary = _effect._views[0]
	var data := _rd.texture_get_data(view["a0"], 0)
	var size: Vector2i = cpu["size"]
	var peak := 0.0
	var worst := 0.0
	for i in range(size.x * size.y):
		peak = maxf(peak, float(cpu["a0"][i]))
	for i in range(size.x * size.y):
		worst = maxf(worst, absf(data.decode_half(i * 8) - float(cpu["a0"][i])) / peak)
	_check("A_0 against the CPU mirror, %.0f px/deg" % PPD, worst <= TOLERANCE and peak > 0.0, "peak %s scene units, worst %.4f of the peak (tolerance %.2f)" % [peak, worst, TOLERANCE])
	_check("factor / size, %.0f px/deg" % PPD, view["factor"] == cpu["factor"] and view["sizes"][0] == size, "f0 %d, %s" % [view["factor"], size])

	# A frame whose only source is the masked Sun, and one with display-white UI alone: nothing spreads.
	var sun_only := PackedFloat32Array()
	sun_only.resize(W * H)
	for i in range(sun_only.size()):
		sun_only[i] = 0.002
	for y in range(35, 46):
		for x in range(135, 146):
			sun_only[y * W + x] = 60000.0
	var other := _color_texture(sun_only)
	_effect_run(other, ray, sun_world, mask_cos)
	var data_sun := _rd.texture_get_data(_effect._views[0]["a0"], 0)
	var sun_peak := 0.0
	for i in range(size.x * size.y):
		sun_peak = maxf(sun_peak, data_sun.decode_half(i * 8))
	_check("Sun disc alone: no image veil, %.0f px/deg" % PPD, sun_peak == 0.0, "max %s" % sun_peak)
	_rd.free_rid(other)

	# The Sun's analytic veil in APPLY (image part off), against GlareSpread.sun_veil_cd_m2 in scene units.
	var scene_lux := 20.0  # normal illuminance x visibility x scene per cd/m^2 (any number)
	var black := PackedFloat32Array()
	black.resize(W * H)
	var blank := _color_texture(black)
	var applied := _effect.view_textures(0, Vector2i(W, H), GlareSpread.base_factor(_ppd), GlareSpread.plan(_ppd)["weights"].size())
	_effect.apply_veil(applied, blank, Vector2i(W, H), ray, sun_world, scene_lux, Vector3(1.0, 1.0, 1.0), false)
	_rd.submit()
	_rd.sync()
	_effect.free_local_sets()
	var out := _rd.texture_get_data(blank, 0)
	var worst_sun := 0.0
	for p: Vector2i in [Vector2i(140, 40), Vector2i(150, 40), Vector2i(100, 40), Vector2i(20, 100), Vector2i(180, 120)]:
		var u := ndc_x(p.x + 0.5) * ray[0] + ray[1]
		var v := ndc_y(p.y + 0.5) * ray[2] + ray[3]
		var theta := rad_to_deg(Vector3(u, v, -1.0).normalized().angle_to(sun_world))
		var expected := GlareSpread.sun_veil_cd_m2(scene_lux, theta, 1.0)
		var got := out.decode_half((p.y * W + p.x) * 8)
		worst_sun = maxf(worst_sun, absf(got / expected - 1.0))
		print("glare_spread_gpu_check: Sun veil at (%d, %d), %.2f deg: gpu %s, cpu %s" % [p.x, p.y, theta, got, expected])
	_check("Sun analytic veil, %.0f px/deg" % PPD, worst_sun < 0.01, "worst %.4f" % worst_sun)
	# Off-axis Sun (above and to the side) with the projection as the render data may give it (y flipped): the analytic
	# veil peaks at the Sun's projected pixel, not at its vertical mirror.
	for flipped in [false, true]:
		var p := Projection(Vector4(projection_x, 0, 0, 0), Vector4(0, projection_y * (-1.0 if flipped else 1.0), 0, 0),
				Vector4(0.0, 0.0, -1.0, -1.0), Vector4(0, 0, -0.1, 0))
		var r := GlareSpreadEffect.ray_parameters(p)
		var sun_dir := Vector3(0.4 / projection_x, 0.5 / projection_y, -1.0).normalized()  # NDC (0.4, 0.5): right of and above the axis
		var applied2 := _effect.view_textures(0, Vector2i(W, H), GlareSpread.base_factor(_ppd), GlareSpread.plan(_ppd)["weights"].size())
		var fresh := _color_texture(black)
		_effect.apply_veil(applied2, fresh, Vector2i(W, H), r, sun_dir, scene_lux, Vector3.ONE, false)
		_rd.submit()
		_rd.sync()
		_effect.free_local_sets()
		var o := _rd.texture_get_data(fresh, 0)
		_rd.free_rid(fresh)
		var best := 0.0
		for i in range(W * H):
			best = maxf(best, o.decode_half(i * 8 + 2))
		var sum := Vector2.ZERO  # the centroid of the plateau (the veil is held inside the disc's radius)
		var n := 0.0
		for y in range(H):
			for x in range(W):
				if o.decode_half((y * W + x) * 8 + 2) >= 0.995 * best:
					sum += Vector2(x, y)
					n += 1.0
		var at := Vector2i(sum / n)
		var want := Vector2i(int(W * 0.7), int(H * 0.25))
		_check("off-axis Sun peak at its pixel, y %s, %.0f px/deg" % ["flipped" if flipped else "plain", PPD],
				absi(at.x - want.x) <= 2 and absi(at.y - want.y) <= 2, "peak %s, expected %s" % [at, want])
	_rd.free_rid(blank)
	_rd.free_rid(color)


func ndc_x(px: float) -> float:
	return 2.0 * px / W - 1.0


func ndc_y(py: float) -> float:
	return 1.0 - 2.0 * py / H


func _effect_run(color: RID, ray: PackedFloat32Array, sun_world: Vector3, mask_cos: float) -> void:
	_effect.build_veil(0, color, Vector2i(W, H), _ppd, ray, PackedFloat32Array([sun_world.x, sun_world.y, sun_world.z, mask_cos]))
	_rd.submit()
	_rd.sync()
	_effect.free_local_sets()


func _color_texture(image: PackedFloat32Array) -> RID:
	var format := RDTextureFormat.new()
	format.format = RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT
	format.width = W
	format.height = H
	format.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT \
			| RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
	var bytes := PackedByteArray()
	bytes.resize(W * H * 8)
	for i in range(image.size()):
		for k in range(3):
			bytes.encode_half(i * 8 + k * 2, image[i])
		bytes.encode_half(i * 8 + 6, 1.0)
	return _rd.texture_create(format, RDTextureView.new(), [bytes])
