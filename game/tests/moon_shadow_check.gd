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
#   text before Task 12, e.g. from `git show dbb5c598:game/shaders/terrain_moon.gdshader`), pixel for pixel;
# - a lander on the ground (Task 12 fix 1, review I1): the HLS model (data/models/starship_hls, VehicleModels; legs
#   deployed, feet on the ground) placed statically on the flat ground at a Sun elevation of 20 deg shadows the ground:
#   the shadow's half-shadow point on the centre line lies between the geometric tip (the model's vertices projected
#   along the Sun, + 1 m) and the tip minus the solar disc's penumbra at the model's top (a pointed nose's shadow narrows
#   into the penumbra), and the ground halfway along the shadow keeps no Sun;
# - the fidelity setting's persistence (user settings file round trip, a test file).
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
const HLS_VEHICLE := "starship_hls"
const HLS_SUN_ELEVATION_DEG := 20.0
const HLS_VIEW_M := 220.0  ## orthographic view edge for the lander's shadow (m)
const HLS_CAMERA_Y_M := 200.0
## The eye height handed to MoonNearShadow for the lander step: its range (150 + 4 h m) holds the orthographic camera's
## view depth (HLS_CAMERA_Y_M) inside the unfaded part.
const HLS_NEAR_HEIGHT_M := 100.0
const TIP_MARGIN_M := 1.0  ## beyond the geometric tip (the rock gate's tolerance)

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
var _hls: Node3D  ## the lander model (null: no fresh models)
var _hls_tip_m := NAN  ## the geometric shadow tip along the Sun's azimuth (m from the axis)
var _hls_height_m := 0.0


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
	plane.size = Vector2(400.0, 400.0)  # also holds the lander's shadow (view x -184 .. 36 m)
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
	if not OS.get_cmdline_user_args().has("--no_lander"):  # last: it moves the camera and stays in the scene
		_steps.append({"name": "hls", "sun": HLS_SUN_ELEVATION_DEG, "earth": -10.0, "near": true, "caster": false,
				"hls": true})
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
	var lander := bool(step.get("hls", false))
	if lander:
		# Loaded only now: its materials' pipelines compiling in the background changed the identity steps' images by
		# 2 codes (the engine's fallback pipelines) when it was there from the start.
		_hls = _build_hls()
		if _hls == null:
			_next()
			return
		_viewport.add_child(_hls)
		_camera.size = HLS_VIEW_M
		_camera.far = 2.0 * HLS_CAMERA_Y_M
		var centre_x := -0.5 * _hls_tip_m
		_camera.look_at_from_position(Vector3(centre_x, HLS_CAMERA_Y_M, 0.0), Vector3(centre_x, 0.0, 0.0), Vector3(0, 0, -1))
	ShaderGlobals.set_value(&"moon_terrain_camera_hi", Vector3(0.0, R, 0.0) + _camera.position)
	_near.update(_light, true, HLS_NEAR_HEIGHT_M if lander else 2.0, Atmosphere.ASTRONOMICAL_UNIT_M, _camera.position)
	_caster.visible = bool(step["caster"])
	_ground.material_override = _material
	if step.has("shader"):
		var old := ShaderMaterial.new()
		old.shader = step["shader"]
		for p in ["albedo_texture", "sun_radius_m", "earth_radius_m"]:
			old.set_shader_parameter(p, _material.get_shader_parameter(p))
		_ground.material_override = old
	_wait = FRAMES * (4 if lander else 1)


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
	if _images.has("hls"):
		_check_hls(_images["hls"])
	elif OS.get_cmdline_user_args().has("--no_lander"):
		print("moon_shadow_check:   info  --no_lander: the lander check skipped")
	else:
		_check(false, "lander shadow: no %s model (%s; run python tools/structures/build_models.py --vehicle %s)" % [
				HLS_VEHICLE, VehicleModels.fallback_reason(HLS_VEHICLE), HLS_VEHICLE])
	_check_setting()
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


## The HLS model (hull, legs, pods; no engines - under the hull, they cast no shadow beyond its own) standing on the
## ground at the origin: body X (the station axis) up, the feet's lowest point at y = 0. Null without fresh models.
func _build_hls() -> Node3D:
	var models := VehicleModels.load_for(HLS_VEHICLE)
	if models.is_empty():
		return null
	var meshes: Array[Mesh] = []
	for part: Dictionary in [(models["hulls"] as Dictionary).values()[0], models.get("hls_parts", {}),
			models.get("dorsal", {})]:
		for mesh: Mesh in part.values():
			meshes.append(mesh)
	# Body (x, y, z) -> (y, z, x)... here x up: the cyclic map body X -> +Y, Y -> +Z, Z -> +X (a rotation).
	var body_up := Basis(Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(1, 0, 0))
	var node := Node3D.new()
	var lowest := INF
	var points: Array[Vector3] = []
	for mesh in meshes:
		var instance := MeshInstance3D.new()
		instance.mesh = mesh
		instance.transform = Transform3D(body_up, Vector3.ZERO)
		node.add_child(instance)
		for surface in mesh.get_surface_count():
			var vertices: PackedVector3Array = mesh.surface_get_arrays(surface)[Mesh.ARRAY_VERTEX]
			for v in vertices:
				var w := body_up * v
				points.append(w)
				lowest = minf(lowest, w.y)
	node.position = Vector3(0.0, -lowest, 0.0)
	# The geometric shadow tip along the Sun's azimuth (0: the shadow runs along -X): each vertex projected along the Sun.
	var sun := _direction(HLS_SUN_ELEVATION_DEG, 0.0)
	_hls_tip_m = 0.0
	for w in points:
		var height := w.y - lowest
		_hls_height_m = maxf(_hls_height_m, height)
		var ground := Vector3(w.x, 0.0, w.z) - Vector3(sun.x, 0.0, sun.z) * (height / sun.y)
		if absf(ground.z) < 0.5:  # on the centre line (the image row the check reads)
			_hls_tip_m = maxf(_hls_tip_m, -ground.x)
	print("moon_shadow_check:   info  %s model: %d meshes, %.1f m tall, geometric shadow tip %.1f m at Sun %.0f deg" % [
			HLS_VEHICLE, meshes.size(), _hls_height_m, _hls_tip_m, HLS_SUN_ELEVATION_DEG])
	return node


## The lander's shadow along the centre line (image row of world z = 0; columns of world x).
func _check_hls(image: Image) -> void:
	var px_per_m := SIZE / HLS_VIEW_M
	var centre_x := -0.5 * _hls_tip_m
	var column := func(world_x: float) -> int: return int(round((world_x - centre_x) * px_per_m + SIZE * 0.5))
	var row := SIZE / 2
	var level := func(x: int) -> float:
		var total := 0.0
		for dy in range(-2, 3):
			total += image.get_pixel(x, row + dy).r
		return total / 5.0
	var lit: float = level.call(column.call(-_hls_tip_m - 0.1 * HLS_VIEW_M))  # beyond the tip
	var umbra: float = level.call(column.call(-0.5 * _hls_tip_m))  # halfway along the shadow
	var half := 0.5 * (lit + umbra)
	var tip_m := NAN
	for x in range(column.call(-0.5 * _hls_tip_m), column.call(-_hls_tip_m - 0.1 * HLS_VIEW_M), -1):
		if level.call(x) >= half:
			tip_m = -((x - SIZE * 0.5) / px_per_m + centre_x)
			break
	var r := deg_to_rad(0.5 * MoonNearShadow.solar_disc_diameter_deg(Atmosphere.ASTRONOMICAL_UNIT_M))
	var e := deg_to_rad(HLS_SUN_ELEVATION_DEG)
	var penumbra := _hls_height_m * (1.0 / tan(e - r) - 1.0 / tan(e + r))
	_check(tip_m <= _hls_tip_m + TIP_MARGIN_M and tip_m >= _hls_tip_m - penumbra,
			"lander (%s, %.1f m tall) on the ground at Sun %.0f deg: shadow tip %.1f m (half-shadow point), geometric tip %.1f m, penumbra at the top %.1f m: within [%.1f, %.1f] m"
			% [HLS_VEHICLE, _hls_height_m, HLS_SUN_ELEVATION_DEG, tip_m, _hls_tip_m, penumbra, _hls_tip_m - penumbra,
			_hls_tip_m + TIP_MARGIN_M])
	_check(umbra < 0.02 * lit, "the lander's shadow keeps no Sun halfway: %.5f of the lit ground's %.5f" % [umbra, lit])


## The fidelity setting's round trip through a settings file of this check (never the user's).
func _check_setting() -> void:
	var path := OS.get_user_data_dir().path_join("moon_shadow_check_settings_%d.cfg" % OS.get_process_id())
	MoonNearShadow.save_enabled_setting(false, path)
	var off := MoonNearShadow.load_enabled_setting(path)
	MoonNearShadow.save_enabled_setting(true, path)
	var on := MoonNearShadow.load_enabled_setting(path)
	DirAccess.remove_absolute(path)
	var missing := MoonNearShadow.load_enabled_setting(path)
	_check(not off and on and missing, "setting moon_near_shadows persists (off %s, on %s, no file: %s)" % [off, on, missing])
