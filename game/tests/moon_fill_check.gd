# GPU check of the Moon fill light's per-wall Sun visibility and the drawn rock's view-elevation clamp (moon procedural
# surface plan Task 13; review m2 of the Task 12 re-review 2: both rested on one-off renders). Through the production
# includes (moon_fill.gdshaderinc, moon_rock_light.gdshaderinc) in game/tests/moon_fill_patch.gdshader, the detail
# statistics from the core (MoonDetailShading.configure; needs data/ with the lunar terrain):
# - a horizontal floor that sees no Sun (walls of WALL_ELEVATION_DEG in every azimuth, the Sun at SUN_ELEVATION_DEG below
#   them) keeps a fill from the sunlit walls: > 0, and below FILL_MAX_SHARE of the same floor's direct reflectance if it
#   were lit (Task 12 fix 2 measured 0.23 % on a rendered floor); before fix 2 the walls were limited by the receiver's
#   own visibility and the fill was 0;
# - moon_rock_lit with the eye 11 deg below the rock's local horizontal equals the value at the 1 deg floor and is a
#   finite share in [0, 1] (before fix 2: Smith's G2 / G1 crossed a zero denominator - blown-out and black rocks).
# Run: godot --path game --script res://tests/moon_fill_check.gd
extends SceneTree

const SIZE := 16
const FRAMES := 6
const WALL_ELEVATION_DEG := 12.0
const SUN_ELEVATION_DEG := 7.0
const FILL_MAX_SHARE := 0.05  ## [E] a generous bound: the walls' view factor sin^2 12 deg is 4.3 %

var _viewport: SubViewport
var _material: ShaderMaterial
var _mode := 0
var _frames := 0
var _values: Array[float] = []
var _failures := 0


func _initialize() -> void:
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	var core := StarsimCore.new()
	var loaded: Dictionary = core.load_terrain(config_dir)
	if not bool(loaded.get("moon", false)):
		print("moon_fill_check: data/ lunar terrain absent: SKIPPED")
		quit(0)
		return
	core.set_moon_surface_detail(true)
	Atmosphere.configure_moon_brdf(core, config_dir, "hapke")
	_material = ShaderMaterial.new()
	_material.shader = load("res://tests/moon_fill_patch.gdshader")
	MoonDetailShading.configure(core, _material, true)
	# The horizon map: every sector's elevation WALL_ELEVATION_DEG (4 layers x RGBA = 16 sectors).
	var layers: Array[Image] = []
	for c in 4:
		var image := Image.create(1, 1, false, Image.FORMAT_RGBAF)
		var h := deg_to_rad(WALL_ELEVATION_DEG)
		image.set_pixel(0, 0, Color(h, h, h, h))
		layers.append(image)
	var horizon := Texture2DArray.new()
	horizon.create_from_images(layers)
	_material.set_shader_parameter("horizon_map", horizon)
	_material.set_shader_parameter("horizon_enabled", true)
	_material.set_shader_parameter("horizon_max_rad", deg_to_rad(WALL_ELEVATION_DEG) + 0.01)
	var e := deg_to_rad(SUN_ELEVATION_DEG)
	_material.set_shader_parameter("sun_local", Vector3(0.0, sin(e), -cos(e)))  # the Sun due north
	ShaderGlobals.set_value(&"moon_near_shadow", Vector4(0.0, 0.0, 0.0, 1.0))  # map off, fill on
	_viewport = SubViewport.new()
	_viewport.size = Vector2i(SIZE, SIZE)
	_viewport.use_hdr_2d = true
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_viewport.own_world_3d = true
	root.add_child(_viewport)
	var quad := MeshInstance3D.new()
	var mesh := QuadMesh.new()
	mesh.size = Vector2(4.0, 4.0)
	quad.mesh = mesh
	quad.material_override = _material
	_viewport.add_child(quad)
	var camera := Camera3D.new()
	camera.position = Vector3(0.0, 0.0, 2.0)
	_viewport.add_child(camera)
	_material.set_shader_parameter("mode", 0)


func _process(_delta: float) -> bool:
	if _material == null:
		return true
	_frames += 1
	if _frames < FRAMES:
		return false
	var image := _viewport.get_texture().get_image()
	_values.append(image.get_pixel(SIZE / 2, SIZE / 2).r)
	_mode += 1
	_frames = 0
	if _mode < 4:
		_material.set_shader_parameter("mode", _mode)
		return false
	var fill := _values[0]
	var lit := _values[1]
	_check(fill > 0.0 and lit > 0.0 and fill < FILL_MAX_SHARE * lit,
			("shadowed floor (walls %.0f deg, Sun %.0f deg): fill %.6f = %.3f %% of the lit floor's %.4f (bound > 0, < %.0f %%)"
			% [WALL_ELEVATION_DEG, SUN_ELEVATION_DEG, fill, 100.0 * fill / maxf(lit, 1e-12), lit, 100.0 * FILL_MAX_SHARE]))
	var below := _values[2]
	var floor_value := _values[3]
	_check(is_finite(below) and below >= 0.0 and below <= 1.0 and absf(below - floor_value) <= 1e-4,
			("rock lit share with the eye 11 deg below the local horizontal %.5f, at the 1 deg floor %.5f (equal, finite, in [0, 1])"
			% [below, floor_value]))
	print("moon_fill_check: %s" % ("PASS" if _failures == 0 else "FAIL (%d)" % _failures))
	quit(0 if _failures == 0 else 1)
	return true


func _check(ok: bool, message: String) -> void:
	print("moon_fill_check:   %s %s" % ["ok  " if ok else "FAIL", message])
	if not ok:
		_failures += 1
