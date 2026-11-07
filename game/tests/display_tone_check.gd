# GPU check of the display tone step (DisplayToneEffect, look fixes T5): the shader against its CPU mirror DisplayTone.
# Unshaded patches of known scene values (grey, the noon sky, the ocean, a saturated blue, an orange, a red, a highlight
# at scene 6, one above AgX's white) under Godot's AgX (glow, FXAA, debanding off; an HDR 2D viewport, so the read-back
# is display-linear before the sRGB encoding) with the step on at Hunt factor 1 and 1.64: each patch's centre against
# DisplayTone.agx(DisplayTone.apply(x)) within 3e-3 (half-float buffer, float32 shader). With the step off (the global
# vision_hunt then 1) the patches are AgX itself. A last row of display-referred emitters (vision.gdshaderinc
# vision_display_referred, unshaded, opaque; no eye post step in this scene) is shown as AgX alone showed it, within
# EMITTER_TOLERANCE. A glow pass (glow on at threshold 4, intensity 0) checks the glow bound (all patches incl. the noon
# aureole (1.75, 2.15, 2.35)) against DisplayTone.apply with the same bound.
# Needs a GPU: godot --path game --script res://tests/display_tone_check.gd
extends SceneTree

const VIEW_SIZE := 256
const SETTLE_FRAMES := 4
const TOLERANCE := 3.0e-3
const PATCHES := [Vector3(0.18, 0.18, 0.18), Vector3(0.0869, 0.1426, 0.284), Vector3(0.012, 0.035, 0.09),
	Vector3(0.02, 0.05, 0.6), Vector3(0.9, 0.35, 0.03), Vector3(0.5, 0.02, 0.02), Vector3(3.0, 6.0, 9.0),
	Vector3(40.0, 30.0, 20.0), Vector3(0.002, 0.003, 0.004), Vector3(1.75, 2.15, 2.35)]
const COLUMNS := 3
const ROWS := 5
const EMITTERS := [Vector3(1.0, 0.45, 0.1), Vector3(0.2, 0.4, 1.0), Vector3(0.9, 0.7, 0.4)]
## [call] The emitters' inverse is exact on the CPU within 0.7 % (display_tone_test); half floats and float32 here.
const EMITTER_TOLERANCE := 0.01

var _failures := 0
var _viewport := SubViewport.new()
var _environment := Environment.new()
var _effect := DisplayToneEffect.new()
var _materials: Array[ShaderMaterial] = []


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	if RenderingServer.get_rendering_device() == null:
		print("display_tone_check: no RenderingDevice (headless?) - needs a GPU")
		quit(1)
		return
	_setup()
	var p := DisplayTone.curve_parameters(_environment.tonemap_agx_contrast, _environment.tonemap_agx_white)
	for hunt: float in [1.0, 1.64]:
		_effect.update(_environment, hunt)
		var image := await _frame()
		for i in range(PATCHES.size()):
			var x: Vector3 = PATCHES[i]
			var expected := DisplayTone.agx(DisplayTone.apply(x, hunt, p), p)
			_compare("step, S %.2f, patch %s" % [hunt, x], _read(image, i), expected)
		for k in range(EMITTERS.size()):
			var e: Vector3 = EMITTERS[k]
			_compare("emitter, S %.2f, %s = AgX" % [hunt, e], _read(image, PATCHES.size() + k), DisplayTone.agx(e, p),
					EMITTER_TOLERANCE)
	# The glow bound (fix round 1): glow on at world.gd's threshold 4 but at intensity 0 (additive), so the read-back
	# still shows the step alone; the shader's bound against DisplayTone.apply with the same bound.
	_environment.glow_enabled = true
	_environment.glow_hdr_threshold = 4.0
	_environment.glow_intensity = 0.0
	_environment.glow_blend_mode = Environment.GLOW_BLEND_MODE_ADDITIVE
	var glow := DisplayTone.glow_bound(_environment)
	_effect.update(_environment, 1.64)
	var bounded := await _frame()
	for i in range(PATCHES.size()):
		var x: Vector3 = PATCHES[i]
		_compare("glow bound 4, S 1.64, patch %s" % x, _read(bounded, i),
				DisplayTone.agx(DisplayTone.apply(x, 1.64, _p_for_check(), glow), _p_for_check()))
	_environment.glow_enabled = false
	DisplayTone.enabled = false
	_effect.update(_environment, 1.64)
	var plain := await _frame()
	for i in range(PATCHES.size()):
		var x: Vector3 = PATCHES[i]
		_compare("step off: AgX, patch %s" % x, _read(plain, i), DisplayTone.agx(x, p))
	_check("step off: vision_hunt 1", float(ShaderGlobals.value(&"vision_hunt")) == 1.0,
			"vision_hunt %s" % ShaderGlobals.value(&"vision_hunt"))
	DisplayTone.enabled = true
	print("display_tone_check: %s (%d failures)" % ["PASS" if _failures == 0 else "FAIL", _failures])
	quit(1 if _failures > 0 else 0)


func _p_for_check() -> PackedFloat64Array:
	return DisplayTone.curve_parameters(_environment.tonemap_agx_contrast, _environment.tonemap_agx_white)


func _setup() -> void:
	_viewport.size = Vector2i(VIEW_SIZE * COLUMNS, VIEW_SIZE * ROWS)
	_viewport.use_hdr_2d = true
	_viewport.own_world_3d = true
	_viewport.msaa_3d = Viewport.MSAA_DISABLED
	_viewport.screen_space_aa = Viewport.SCREEN_SPACE_AA_DISABLED
	_viewport.use_debanding = false
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(_viewport)
	_environment.background_mode = Environment.BG_COLOR
	_environment.background_color = Color(0, 0, 0)
	_environment.tonemap_mode = Environment.TONE_MAPPER_AGX
	_environment.tonemap_exposure = 1.0
	_environment.glow_enabled = false
	_environment.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
	_environment.reflected_light_source = Environment.REFLECTION_SOURCE_DISABLED
	var world_environment := WorldEnvironment.new()
	world_environment.environment = _environment
	var compositor := Compositor.new()
	compositor.compositor_effects = [_effect]
	world_environment.compositor = compositor
	_viewport.add_child(world_environment)
	var camera := Camera3D.new()
	camera.projection = Camera3D.PROJECTION_ORTHOGONAL
	camera.size = float(ROWS)  # keep height: ROWS tiles high, COLUMNS wide
	camera.near = 0.1
	camera.far = 10.0
	camera.current = true
	_viewport.add_child(camera)
	var shader := Shader.new()
	shader.code = """shader_type spatial;
render_mode unshaded, cull_disabled, fog_disabled, shadows_disabled;
uniform vec3 colour;
void fragment() { ALBEDO = colour; }
"""
	for i in range(PATCHES.size()):
		var quad := MeshInstance3D.new()
		var mesh := QuadMesh.new()
		mesh.size = Vector2(1.0, 1.0)
		quad.mesh = mesh
		var material := ShaderMaterial.new()
		material.shader = shader
		material.set_shader_parameter("colour", PATCHES[i])
		quad.material_override = material
		quad.position = _tile_position(i)
		_viewport.add_child(quad)
	var emitter_shader := Shader.new()
	emitter_shader.code = """shader_type spatial;
render_mode unshaded, cull_disabled, fog_disabled, shadows_disabled;
#include "res://shaders/vision.gdshaderinc"
uniform vec3 colour;
void fragment() { ALBEDO = vision_display_referred(colour); }
"""
	for k in range(EMITTERS.size()):
		var quad := MeshInstance3D.new()
		var mesh := QuadMesh.new()
		mesh.size = Vector2(1.0, 1.0)
		quad.mesh = mesh
		var material := ShaderMaterial.new()
		material.shader = emitter_shader
		material.set_shader_parameter("colour", EMITTERS[k])
		quad.material_override = material
		quad.position = _tile_position(PATCHES.size() + k)
		_viewport.add_child(quad)


func _tile_position(i: int) -> Vector3:
	return Vector3(float(i % COLUMNS) - 1.0, 0.5 * float(ROWS - 1) - float(i / COLUMNS), -2.0)


func _frame() -> Image:
	for i in range(SETTLE_FRAMES):
		await RenderingServer.frame_post_draw
	return _viewport.get_texture().get_image()


func _read(image: Image, i: int) -> Vector3:
	var c := image.get_pixel((i % COLUMNS) * VIEW_SIZE + VIEW_SIZE / 2, (i / COLUMNS) * VIEW_SIZE + VIEW_SIZE / 2)
	return Vector3(c.r, c.g, c.b)


func _compare(label: String, got: Vector3, expected: Vector3, tolerance := TOLERANCE) -> void:
	var error := (got - expected).abs()
	var worst := maxf(error.x, maxf(error.y, error.z))
	_check(label, worst < tolerance, "GPU %s, CPU %s, error %.5f" % [got, expected, worst])


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("display_tone_check: %-52s %s %s" % [label, detail, "ok" if ok else "FAIL"])
