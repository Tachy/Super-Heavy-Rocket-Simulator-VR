# GPU checks of the near-field Sun shadows on the Moon (moon procedural surface plan Task 12; MoonNearShadow,
# moon_near_shadow.gdshaderinc). A flat ground drawn through the production terrain_moon.gdshader (detail statistics
# off, no horizon map: the Task 9 path) in its own world, lit by a DirectionalLight3D configured by MoonNearShadow:
# - shadow length: a 2 m tall caster at a Sun elevation of 7 deg casts a shadow 2 m / tan 7 deg = 16.29 m long, the
#   half-shadow point (50 % between the umbra and the lit ground, the solar disc's penumbra centred on it) within
#   +-1 m (the brief's gate), read along the Sun's azimuth in a top-down orthographic image;
# - umbra: the ground in the caster's shadow keeps only the earthshine (no Sun);
# - earthshine only (the Sun 5 deg below the horizon, the Earth 7 deg up): the near shadows on and off give the same
#   pixels;
# - off: with the near shadows off the image equals the former unshaded shader's (--old_shader=<path>: the shader's
#   text before Task 12, e.g. from `git show dbb5c598:game/shaders/terrain_moon.gdshader`), pixel for pixel.
# Run: godot --path game --script res://tests/moon_shadow_check.gd [-- --old_shader=<abs path>] [--out=<dir>]
extends SceneTree

const R := HeightSource.MOON_RADIUS_M
const SIZE := 1024  ## viewport edge (px)
const VIEW_M := 40.0  ## orthographic view edge (m)
const CASTER_HEIGHT_M := 2.0  ## the brief's test rock
const CASTER_WIDTH_M := 0.5
const SUN_ELEVATION_DEG := 7.0
const LENGTH_TOLERANCE_M := 1.0  ## the brief's gate
const FRAMES := 8  ## frames after each change before the readback

var _viewport: SubViewport
var _camera: Camera3D
var _light: DirectionalLight3D
var _ground: MeshInstance3D
var _caster: MeshInstance3D
var _material: ShaderMaterial
var _near := MoonNearShadow.new()
var _steps: Array = []
var _step := -1
var _wait := 0
var _images := {}
var _failures := 0
var _out_dir := ""


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--out="):
			_out_dir = arg.get_slice("=", 1)
	_viewport = SubViewport.new()
	_viewport.size = Vector2i(SIZE, SIZE)
	_viewport.own_world_3d = true
	_viewport.use_hdr_2d = true
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color(0, 0, 0)
	environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	var world_environment := WorldEnvironment.new()
	world_environment.environment = environment
	_viewport.add_child(world_environment)
	_camera = Camera3D.new()
	_camera.projection = Camera3D.PROJECTION_ORTHOGONAL
	_camera.size = VIEW_M
	_camera.far = 200.0
	_viewport.add_child(_camera)
	_camera.look_at_from_position(Vector3(-VIEW_M * 0.4, 50.0, 0.0), Vector3(-VIEW_M * 0.4, 0.0, 0.0), Vector3(0, 0, -1))
	_light = DirectionalLight3D.new()
	_viewport.add_child(_light)
	# The ground: a flat 400 m square through the production terrain shader; the Moon-local frame is the world's,
	# offset to the surface (vertex_offset: the node centre's local position).
	var plane := PlaneMesh.new()
	plane.size = Vector2(400.0, 400.0)
	plane.subdivide_width = 100
	plane.subdivide_depth = 100
	_ground = MeshInstance3D.new()
	_ground.mesh = plane
	_material = ShaderMaterial.new()
	_material.shader = load("res://shaders/terrain_moon.gdshader")
	var white := Image.create(1, 1, false, Image.FORMAT_RGBF)
	white.set_pixel(0, 0, Color(0.5, 0.5, 0.5))
	_material.set_shader_parameter("albedo_texture", ImageTexture.create_from_image(white))
	_material.set_shader_parameter("sun_radius_m", World.SUN_RADIUS_M)
	_material.set_shader_parameter("earth_radius_m", World.EARTH_MEAN_RADIUS_M)
	_ground.material_override = _material
	_ground.set_instance_shader_parameter(&"vertex_offset", Vector3(0.0, R, 0.0))
	_viewport.add_child(_ground)
	var box := BoxMesh.new()
	box.size = Vector3(CASTER_WIDTH_M, CASTER_HEIGHT_M, CASTER_WIDTH_M)
	_caster = MeshInstance3D.new()
	_caster.mesh = box
	_caster.position = Vector3(0.0, 0.5 * CASTER_HEIGHT_M, 0.0)
	_viewport.add_child(_caster)
	root.add_child(_viewport)
	ShaderGlobals.set_value(&"moon_regolith_scale", 1.0)
	ShaderGlobals.set_value(&"moon_sun_scene", 1.0)
	ShaderGlobals.set_value(&"moon_brdf", 0)
	ShaderGlobals.set_value(&"atmosphere_enabled", false)
	ShaderGlobals.set_value(&"moon_terrain_camera_hi", Vector3(0.0, R, 0.0) + _camera.position)
	ShaderGlobals.set_value(&"moon_terrain_camera_lo", Vector3.ZERO)
	_steps = [
		{"name": "shadow", "sun": SUN_ELEVATION_DEG, "earth": -10.0, "near": true, "caster": true},
		{"name": "earthshine_on", "sun": -5.0, "earth": 7.0, "near": true, "caster": true},
		{"name": "earthshine_off", "sun": -5.0, "earth": 7.0, "near": false, "caster": true},
		{"name": "off_new", "sun": SUN_ELEVATION_DEG, "earth": 7.0, "near": false, "caster": false},
	]
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--old_shader="):
			var text := FileAccess.get_file_as_string(arg.get_slice("=", 1))
			if text.is_empty():
				_fail("cannot read %s" % arg.get_slice("=", 1))
			else:
				var old := Shader.new()
				old.code = text
				_steps.append({"name": "off_old", "sun": SUN_ELEVATION_DEG, "earth": 7.0, "near": false, "caster": false,
						"shader": old})
	_next()


func _next() -> void:
	_step += 1
	if _step >= _steps.size():
		_finish()
		return
	var step: Dictionary = _steps[_step]
	var sun := _direction(float(step["sun"]), 0.0)
	var earth := _direction(float(step["earth"]), 120.0)
	ShaderGlobals.set_value(&"moon_sun_position_local", Vector3(0.0, R, 0.0) + sun * Atmosphere.ASTRONOMICAL_UNIT_M)
	ShaderGlobals.set_value(&"moon_earth_position_local", Vector3(0.0, R, 0.0) + earth * 3.844e8)  # [S] mean distance
	ShaderGlobals.set_value(&"moon_earthshine", 0.05 if float(step["earth"]) > 0.0 else 0.0)
	_light.basis = Basis.looking_at(-sun, Vector3.UP if absf(sun.y) < 0.99 else Vector3.RIGHT)
	_light.visible = float(step["sun"]) > -90.0
	MoonNearShadow.enabled = bool(step["near"])
	_near.update(_light, true, 2.0, Atmosphere.ASTRONOMICAL_UNIT_M, _camera.position)
	_caster.visible = bool(step["caster"])
	_ground.material_override = _material
	if step.has("shader"):
		var old := ShaderMaterial.new()
		old.shader = step["shader"]
		for p in ["albedo_texture", "sun_radius_m", "earth_radius_m"]:
			old.set_shader_parameter(p, _material.get_shader_parameter(p))
		_ground.material_override = old
	_wait = FRAMES


## Unit direction at elevation / azimuth (deg) in the ground's frame (+Y up, azimuth 0 along +X).
func _direction(elevation_deg: float, azimuth_deg: float) -> Vector3:
	var e := deg_to_rad(elevation_deg)
	var a := deg_to_rad(azimuth_deg)
	return Vector3(cos(e) * cos(a), sin(e), cos(e) * sin(a))


func _process(_delta: float) -> bool:
	if _step >= _steps.size():
		return true
	if _wait > 0:
		_wait -= 1
		return false
	var step: Dictionary = _steps[_step]
	var image := _viewport.get_texture().get_image()
	_images[step["name"]] = image
	if not _out_dir.is_empty():
		image.save_png(_out_dir.path_join("shadow_check_%s.png" % step["name"]))
	_next()
	return _step >= _steps.size()


func _finish() -> void:
	_check_length(_images["shadow"])
	_check_same("earthshine only: near shadows on and off", _images["earthshine_on"], _images["earthshine_off"])
	if _images.has("off_old"):
		_check_same("near shadows off: the shader before Task 12", _images["off_new"], _images["off_old"])
	else:
		print("moon_shadow_check:   info  no --old_shader: the off-identity check skipped")
	print("moon_shadow_check: %s" % ("PASS" if _failures == 0 else "FAIL (%d failures)" % _failures))
	quit(0 if _failures == 0 else 1)


## The shadow's tip along the Sun's azimuth (image x: world -x, the caster at world x = 0, image column of world x).
func _check_length(image: Image) -> void:
	var px_per_m := SIZE / VIEW_M
	var centre_x := -VIEW_M * 0.4
	var column := func(world_x: float) -> int: return int(round((world_x - centre_x) * px_per_m + SIZE * 0.5))
	var row := SIZE / 2
	var level := func(x: int) -> float:
		var total := 0.0
		for dy in range(-2, 3):
			total += image.get_pixel(x, row + dy).r
		return total / 5.0
	var lit: float = level.call(column.call(-19.0))  # beyond the tip
	var umbra: float = level.call(column.call(-8.0))  # mid-shadow
	var half := 0.5 * (lit + umbra)
	var tip_m := NAN
	for x in range(column.call(-8.0), column.call(-19.5), -1):
		if level.call(x) >= half:
			tip_m = -((x - SIZE * 0.5) / px_per_m + centre_x)
			break
	var length_m := tip_m - 0.5 * CASTER_WIDTH_M  # from the caster's shadow-side face
	var expected := CASTER_HEIGHT_M / tan(deg_to_rad(SUN_ELEVATION_DEG))
	_check(absf(length_m - expected) <= LENGTH_TOLERANCE_M,
			"2 m caster at Sun %.0f deg: shadow %.2f m long (half-shadow point), expected %.2f m +- %.1f m (umbra %.5f, lit %.5f)"
			% [SUN_ELEVATION_DEG, length_m, expected, LENGTH_TOLERANCE_M, umbra, lit])
	_check(umbra < 0.02 * lit, "the umbra keeps no Sun: %.5f of the lit ground's %.5f" % [umbra, lit])
	# The penumbra: 10 % - 90 % width against the solar disc's (h x (cot(e - r) - cot(e + r)), r the disc's radius).
	var x10 := NAN
	var x90 := NAN
	for x in range(column.call(-8.0), column.call(-19.5), -1):
		var v: float = level.call(x)
		if is_nan(x10) and v >= umbra + 0.1 * (lit - umbra):
			x10 = x
		if is_nan(x90) and v >= umbra + 0.9 * (lit - umbra):
			x90 = x
	var r := deg_to_rad(0.5 * MoonNearShadow.solar_disc_diameter_deg(Atmosphere.ASTRONOMICAL_UNIT_M))
	var e := deg_to_rad(SUN_ELEVATION_DEG)
	var disc := CASTER_HEIGHT_M * (1.0 / tan(e - r) - 1.0 / tan(e + r))
	print("moon_shadow_check:   info  penumbra 10-90 %% %.2f m, the solar disc's full width %.2f m" % [
			absf(x10 - x90) / px_per_m, disc])


func _check_same(what: String, a: Image, b: Image) -> void:
	var worst := 0.0
	for y in range(0, SIZE, 2):
		for x in range(0, SIZE, 2):
			var ca := a.get_pixel(x, y)
			var cb := b.get_pixel(x, y)
			worst = maxf(worst, maxf(absf(ca.r - cb.r), maxf(absf(ca.g - cb.g), absf(ca.b - cb.b))))
	_check(worst == 0.0, "%s: largest pixel difference %.6f" % [what, worst])


func _check(ok: bool, message: String) -> void:
	print("moon_shadow_check:   %s %s" % ["ok  " if ok else "FAIL", message])
	if not ok:
		_failures += 1


func _fail(message: String) -> void:
	_check(false, message)
