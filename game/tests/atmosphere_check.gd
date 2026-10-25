# GPU check of the atmosphere shader include (M8 atmosphere, Task 5): renders the sky (shaders/sky.gdshader, nothing
# else in the scene) into an HDR SubViewport for sample views, reads the float pixel on the view direction and compares
# it with the core's CPU evaluation (StarsimCore.atmosphere_sky_luminance x Atmosphere.luminance_scale) within 2 % per
# channel. Linear tone mapper, exposure 1, glow off: the read value is the scene value. Exit code 1 on failure.
# Task 6 adds the Earth's surfaces (terrain_earth.gdshader, earth.gdshader incl. scaled space: lit by the transmitted
# Sun and the sky light, seen through the atmosphere; 3 %). STARSIM_VERBOSE=1 prints the surface samples' parts.
# Task 7 adds the stars (hidden at noon, unchanged above the atmosphere, visible in twilight), the Sun disc's reddening
# and the daytime Moon (Moon x T + sky, 3 %); Task 8 a star near the threshold (Sun -6 deg), vehicles and structures
# (a white lit_aerial cube at the pad and 40 km away, 5 %) and the Sun light's colour at a low Sun; the eye adaptation
# (section 10, otherwise off); Task 9 the eclipse sky over Dallas at totality (section 11).
# Needs a GPU (no --headless): godot --path game --script res://tests/atmosphere_check.gd
extends SceneTree

const TOLERANCE := 0.02
## Channels below this fraction of the sample's brightest channel are compared absolutely (at that level).
const RELATIVE_FLOOR := 1e-3
const VIEW_SIZE := 65  ## odd: the centre pixel's ray is the camera axis
const SETTLE_FRAMES := 3
## OLP-1 (config/launch_sites.toml, height [D] ellipsoidal).
const PAD_LAT_DEG := 25.9967
const PAD_LON_DEG := -97.1543
const PAD_HEIGHT_M := -21.11
const MOON_DISTANCE_M := 3.844e8
const AU_M := 149597870700.0
const EPOCH_UTC := "2024-04-08T18:42:00"
## Surface samples (Task 6): a point near Cape Canaveral, the terrain pixel this far west of the camera.
const CAPE_LAT_DEG := 28.45
const CAPE_LON_DEG := -80.6
const CAPE_POINT_DISTANCE_M := 20000.0
const SURFACE_TOLERANCE := 0.03
## The surfaces' albedo: every texel of the bound day texture is this sRGB-8 colour, so the texel the shader reads is
## known exactly (decoded sRGB -> linear on the GPU as Color.srgb_to_linear on the CPU). The real day texture would
## make "the same texel" depend on mip level and anisotropic filtering at the grazing terrain view.
const SURFACE_ALBEDO_8 := Color8(150, 120, 90)
const WGS84_A_M := 6378137.0
const WGS84_FLATTENING := 1.0 / 298.257223563

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()
var _viewport := SubViewport.new()
var _camera := Camera3D.new()
var _earth_body := CelestialBody.new()  ## the Earth's ellipsoid mesh with earth.gdshader (scaled space far out)
var _earth_material := ShaderMaterial.new()
var _terrain_parent := Node3D.new()  ## Earth-fixed frame of the terrain quad (unscaled)
var _terrain_quad := MeshInstance3D.new()  ## one camera-facing quad with terrain_earth.gdshader
var _terrain_material := ShaderMaterial.new()
var _earth_to_gcrf := Basis.IDENTITY
var _environment := Environment.new()  ## the check viewport's environment (ambient switched on for the cube samples)
var _failures := 0
var _samples := 0


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	if not _core.open(Paths.data_dir().path_join("spice")):
		_fail_and_quit("SPICE kernels missing")
		return
	_earth_to_gcrf = _core.earth_fixed_to_gcrf(_core.utc_to_tdb_s(EPOCH_UTC))  # a real, non-trivial rotation
	if OS.get_cmdline_user_args().has("--f32"):
		_atmosphere.table_format = Image.FORMAT_RGBAF  # compare with the full-float tables
	_atmosphere.eye_adaptation_enabled = false  # the fixed day exposure, except in section 10
	print("atmosphere_check: table format %s" % ("RGBAF" if _atmosphere.table_format == Image.FORMAT_RGBAF else "RGBAH"))
	_atmosphere.start_loading(_core, Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	if not _atmosphere.wait_until_loaded():
		_fail_and_quit("atmosphere tables not loaded: " + _atmosphere.status_text)
		return
	_setup_scene()

	# 1. The pad looking up, Sun 60 deg (and two other directions).
	var pad := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 2.0)
	var sun_60 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 180.0, 60.0)
	await _sample("pad, zenith, Sun 60 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 90.0), sun_60)
	await _sample("pad, az 90 el 30, Sun 60 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, 30.0), sun_60)
	await _sample("pad, az 0 el 5, Sun 60 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 5.0), sun_60)

	# 2. The pad at sunset looking at the Sun (just above it and a little higher).
	var sun_set := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, 0.5)
	await _sample("sunset, toward the Sun el 1", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, 1.0), sun_set)
	await _sample("sunset, toward the Sun el 5", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 272.0, 5.0), sun_set)
	await _sample("sunset, away from the Sun el 10", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, 10.0), sun_set)

	# 3. 400 km above the ellipsoid looking at the limb (tangent heights 10 / 30 / 60 km), Sun 30 deg at the camera.
	var orbit := _geodetic(10.0, -60.0, 400000.0)
	var sun_orbit := _local_direction(10.0, -60.0, 90.0, 30.0)
	for tangent_km in [10.0, 30.0, 60.0]:
		await _sample("400 km, limb at %.0f km" % tangent_km, orbit, _limb_direction(orbit, Vector3(0, 0, 1),
				tangent_km * 1000.0), sun_orbit)

	# 4. Moon distance looking at the Earth's limb (Review Focus 2): tangent heights 20 / 50 km on the lit side.
	var far_dir := Vector3(cos(0.3), sin(0.3), 0.0)
	var far := far_dir * MOON_DISTANCE_M
	var side := Vector3(-sin(0.3), cos(0.3), 0.0)  # in the equator plane, perpendicular to the camera direction
	var sun_far := (side * 0.8 + far_dir * 0.6).normalized()  # lights the limb on the `side` edge
	for tangent_km in [20.0, 50.0]:
		await _sample("Moon distance, limb at %.0f km" % tangent_km, far, _limb_direction(far, side,
				tangent_km * 1000.0), sun_far)

	# 5. Pad camera at -21 m ellipsoidal (Review Focus 1): 129 m above the -150 m model ground; grazing rays are the
	# most sensitive to the camera height. The control renders the grazing ray with the altitude uniform forced to the
	# model ground: it must differ, proving the shader takes the camera height from atmosphere_camera_altitude_m (the
	# float64-derived uniform) and not from its float32 position.
	var low := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, PAD_HEIGHT_M)
	var sun_30 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 120.0, 30.0)
	for elevation in [0.3, 1.0, 3.0, 20.0]:
		await _sample("pad at -21 m, az 90 el %.1f" % elevation, low,
				_local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, elevation), sun_30)
	print("atmosphere_check: camera altitude above the model ground at -21 m: %.3f m" % _atmosphere.camera_altitude_m)
	await _altitude_control(low, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, 0.3), sun_30)

	# 6. Ground rays (sky.gdshader ground_radiance: atm_to_point + atm_sun_and_sky, the stretched-normal tilt): from the
	# pad a few km out, from 400 km (atm_to_point entering from space) and from the Moon distance.
	await _ground_sample("pad, ground 2 deg down (~4 km)", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, -2.0),
			sun_60)
	await _ground_sample("400 km, ground 40 deg down", orbit, _local_direction(10.0, -60.0, 0.0, -40.0), sun_orbit)
	await _ground_sample("Moon distance, Earth's disc", far, (-far_dir + side * 0.01).normalized(), sun_far)

	# 7. Earth surfaces (Task 6): a terrain pixel (terrain_earth.gdshader) 20 km from a camera 2 m above the ground at
	# the Cape, and Earth-disc pixels of the ellipsoid mesh (earth.gdshader) from 400 km and from the Moon distance (the
	# scaled-space mesh): albedo x (sun + sky lux) / pi x T + to-point luminance, 3 %.
	var cape := _geodetic(CAPE_LAT_DEG, CAPE_LON_DEG, 2.0)
	var cape_point := _geodetic(CAPE_LAT_DEG, CAPE_LON_DEG - CAPE_POINT_DISTANCE_M / (111319.49 * cos(deg_to_rad(
			CAPE_LAT_DEG))), 0.0)
	var sun_cape := _local_direction(CAPE_LAT_DEG, CAPE_LON_DEG, 120.0, 40.0)
	await _surface_sample("Cape, terrain 20 km", cape, cape_point, _geodetic_up(cape_point), sun_cape, true)
	var disc_400 := _ellipsoid_hit(orbit, _local_direction(10.0, -60.0, 0.0, -40.0))
	await _surface_sample("400 km, Earth mesh 40 deg down", orbit, disc_400, _geodetic_up(disc_400), sun_orbit, false)
	var disc_far := _ellipsoid_hit(far, (-far_dir + side * 0.01).normalized())
	await _surface_sample("Moon distance, Earth mesh (scaled)", far, disc_far, _geodetic_up(disc_far), sun_far, false)

	# 8. Moon, Sun disc and stars through the atmosphere (Task 7).
	var star_view := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 60.0)
	await _star_sample("pad noon, magnitude-0 star", pad, star_view, sun_60, 0.0, StarCase.HIDDEN)
	var high := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 120000.0)
	await _star_sample("120 km noon, magnitude-0 star", high, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 90.0),
			sun_60, 0.0, StarCase.UNCHANGED)
	var sun_night := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -18.0)
	await _star_sample("pad, Sun -18 deg, magnitude-1 star", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, 45.0),
			sun_night, 1.0, StarCase.VISIBLE)
	await _sun_sample("pad, Sun disc at 1 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, 1.0))
	await _moon_sample("pad noon, Moon el 20 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 20.0),
			_local_direction(PAD_LAT_DEG, PAD_LON_DEG, 180.0, 50.0))
	# Near the threshold (Task 6+7 review, Minor 2): Sun -6 deg, the zenith's limiting magnitude ~3.6 - a magnitude-2
	# star must show, a magnitude-5 star must not (the pixel is the sky).
	var sun_m6 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -6.0)
	var zenith := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 90.0)
	await _star_sample("pad, Sun -6 deg, magnitude-2 star", pad, zenith, sun_m6, 2.0, StarCase.VISIBLE)
	await _star_sample("pad, Sun -6 deg, magnitude-5 star", pad, zenith, sun_m6, 5.0, StarCase.HIDDEN)

	# 9. Vehicles and structures (Task 8): a white cube with lit_aerial.gdshader under the Sun light of
	# Atmosphere.sun_light_at and the sky's ambient. At the pad (top face, Sun 60 deg) it renders the CPU light on that
	# face: (sun + sky lux) / pi x luminance scale, 5 %. 40 km east of the pad, seen from it, the same cube's face is
	# the near render of that face x T + the in-scatter between (CPU), 5 %. The Sun light's colour at 2 deg: red / blue > 3.
	await _cube_near_sample("pad noon, white cube top face", sun_60)
	await _cube_far_sample("pad, white cube 40 km east", _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, 30.0))
	_sun_light_colour_sample("pad, Sun light colour at 2 deg", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0,
			2.0))

	# 10. Eye adaptation (controller ruling 2026-10-25; every other sample runs with it off: the fixed day exposure).
	# Adapted to each view: the noon zenith (reference), the zenith at Sun -3 deg (visible: > ADAPTED_TWILIGHT_MIN and
	# darker than the noon zenith), at Sun -12 deg (dark: < ADAPTED_NIGHT_MAX, a magnitude-1 star visible); 400 km by
	# day: gain 1 (the day look in orbit).
	var noon_zenith := await _adapted_sample("adapted, pad noon zenith", pad, zenith, sun_60)
	var twilight := await _adapted_sample("adapted, pad zenith, Sun -3 deg", pad, zenith,
			_local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -3.0))
	var twilight_y := twilight.dot(LUMINANCE_WEIGHTS)
	var noon_y := noon_zenith.dot(LUMINANCE_WEIGHTS)
	_check("adapted Sun -3 deg zenith visible, darker than noon", twilight_y > ADAPTED_TWILIGHT_MIN and twilight_y < noon_y,
			"luminance %s (needs > %s), noon %s, ratio %s" % [_num(twilight_y), _num(ADAPTED_TWILIGHT_MIN), _num(noon_y),
			_num(twilight_y / noon_y)])
	var sun_m12 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -12.0)
	var night := await _adapted_sample("adapted, pad zenith, Sun -12 deg", pad, zenith, sun_m12, false)
	_check("adapted Sun -12 deg zenith dark", night.dot(LUMINANCE_WEIGHTS) < ADAPTED_NIGHT_MAX,
			"luminance %s (needs < %s)" % [_num(night.dot(LUMINANCE_WEIGHTS)), _num(ADAPTED_NIGHT_MAX)])
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	await _star_sample("adapted, Sun -12 deg, magnitude-1 star", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0,
			45.0), sun_m12, 1.0, StarCase.VISIBLE)
	_atmosphere.eye_adaptation_enabled = false
	await _adapted_sample("adapted, 400 km by day, limb at 10 km", orbit, _limb_direction(orbit, Vector3(0, 0, 1),
			10000.0), sun_orbit)
	_check("adapted 400 km by day: the day exposure", is_equal_approx(_last_gain, 1.0), "gain %.4f" % _last_gain)

	# 11. Solar eclipse in the sky (Task 9): 2024-04-08 18:42 UTC from Dallas (totality) - the zenith with the segments
	# < 1 % of the same sky without them, the horizon away from the umbra > 10 x the zenith; two hours later (Dallas out
	# of the eclipse, the penumbra still on the Earth) segments on and off agree within 1 %.
	await _eclipse_samples()

	print("atmosphere_check: %d samples, %d failures" % [_samples, _failures])
	print("atmosphere_check: %s" % ("PASS" if _failures == 0 else "FAIL"))
	quit(1 if _failures > 0 else 0)


func _setup_scene() -> void:
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_viewport.use_hdr_2d = true
	_viewport.own_world_3d = true
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(_viewport)
	var sky_material := ShaderMaterial.new()
	sky_material.shader = load("res://shaders/sky.gdshader")
	var sky := Sky.new()
	sky.sky_material = sky_material
	sky.process_mode = Sky.PROCESS_MODE_REALTIME
	var environment := _environment
	environment.background_mode = Environment.BG_SKY
	environment.sky = sky
	environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	environment.tonemap_exposure = 1.0
	environment.glow_enabled = false
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
	environment.reflected_light_source = Environment.REFLECTION_SOURCE_DISABLED
	var world_environment := WorldEnvironment.new()
	world_environment.environment = environment
	_viewport.add_child(world_environment)
	_camera.fov = 10.0
	_camera.near = 0.5
	_camera.far = 4.0e7  # the scaled-space Earth sits at CelestialBody.SCALED_SPACE_DISTANCE_M
	_camera.current = true
	_viewport.add_child(_camera)
	_setup_surfaces()


## The Earth's surfaces of Task 6, hidden until a surface sample shows one: the ellipsoid mesh (CelestialBody with
## earth.gdshader) and a terrain quad (terrain_earth.gdshader), both with the constant albedo texture and black night
## lights, materials set up as world.gd does.
func _setup_surfaces() -> void:
	var albedo_texture := _constant_texture(SURFACE_ALBEDO_8)
	var black := _constant_texture(Color(0, 0, 0))
	_earth_material.shader = load("res://shaders/earth.gdshader")
	_earth_material.set_shader_parameter("day_texture", albedo_texture)
	_earth_material.set_shader_parameter("night_texture", black)
	_earth_material.set_shader_parameter("moon_radius_m", 1737400.0)
	_earth_material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	_earth_body.setup(WGS84_A_M, WGS84_FLATTENING, _earth_material)
	_earth_body.visible = false
	_viewport.add_child(_earth_body)
	_terrain_material.shader = load("res://shaders/terrain_earth.gdshader")
	_terrain_material.set_shader_parameter("day_texture", albedo_texture)
	_terrain_material.set_shader_parameter("day_texture_bound", true)
	_terrain_material.set_shader_parameter("night_texture", black)
	_terrain_material.set_shader_parameter("moon_radius_m", 1737400.0)
	_terrain_material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	_terrain_quad.material_override = _terrain_material
	_terrain_parent.add_child(_terrain_quad)
	_terrain_parent.visible = false
	_viewport.add_child(_terrain_parent)


static func _constant_texture(color: Color) -> ImageTexture:
	var image := Image.create(4, 4, true, Image.FORMAT_RGBA8)
	image.fill(color)
	image.generate_mipmaps()
	return ImageTexture.create_from_image(image)


## A surface pixel: the terrain quad (terrain = true) centred on point_itrf_m and facing the camera, its NORMAL the
## surface normal normal_itrf (the lighting's normal; the quad's geometry only puts a fragment on the camera axis), or
## the ellipsoid mesh (the point is the ray's WGS84 hit). Sun far, the Moon behind the Earth (no eclipse).
## CPU: albedo x (sun + sky lux on the normal) / pi x T + to-point luminance, all x luminance_scale.
func _surface_sample(label: String, camera_itrf_m: Vector3, point_itrf_m: Vector3, normal_itrf: Vector3,
		sun_itrf: Vector3, terrain: bool) -> void:
	var view_itrf := (point_itrf_m - camera_itrf_m).normalized()
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var sun_local := Frames.to_render(sun_itrf * AU_M)
	var moon_local := -Frames.to_render(sun_itrf) * MOON_DISTANCE_M
	RenderingServer.global_shader_parameter_set(&"earth_sun_position_local", sun_local)
	RenderingServer.global_shader_parameter_set(&"earth_moon_position_local", moon_local)
	_earth_material.set_shader_parameter("sun_position_local", sun_local)
	_earth_material.set_shader_parameter("moon_position_local", moon_local)
	_earth_body.visible = not terrain
	_terrain_parent.visible = terrain
	if terrain:
		var center := Frames.to_render(point_itrf_m)
		var distance := (point_itrf_m - camera_itrf_m).length()
		_terrain_parent.transform = Transform3D(Frames.basis_to_render(_earth_to_gcrf), Frames.to_render(-camera_gcrf))
		_terrain_quad.mesh = _facing_quad(Frames.to_render(view_itrf), Frames.to_render(normal_itrf), distance * 0.01)
		_terrain_quad.position = center
		var split := TerrainQuadtree.split_float32(center)
		_terrain_quad.set_instance_shader_parameter(&"vertex_offset", split[0])
		_terrain_quad.set_instance_shader_parameter(&"vertex_offset_lo", split[1])
		var camera_split := TerrainQuadtree.split_float32(Frames.to_render(camera_itrf_m))
		RenderingServer.global_shader_parameter_set(&"earth_terrain_camera_hi", camera_split[0])
		RenderingServer.global_shader_parameter_set(&"earth_terrain_camera_lo", camera_split[1])
	else:
		_earth_body.fixed_to_gcrf = _earth_to_gcrf
		_earth_body.center_gcrf_m = Vector3.ZERO
		_earth_body.place(camera_gcrf)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	_earth_body.visible = false
	_terrain_parent.visible = false
	var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera_itrf_m, point_itrf_m, sun_itrf)
	var lux: Dictionary = _core.atmosphere_illuminance(point_itrf_m, normal_itrf, sun_itrf)
	var linear := SURFACE_ALBEDO_8.srgb_to_linear()
	var albedo := Vector3(linear.r, linear.g, linear.b)
	var transmittance: Vector3 = to_point["transmittance"]
	var light: Vector3 = (lux["sun_lux"] + lux["sky_lux"]) / PI
	var expected: Vector3 = (albedo * light * transmittance + to_point["luminance"]) * _atmosphere.luminance_scale
	if OS.get_environment("STARSIM_VERBOSE") == "1":
		print("atmosphere_check:   surface %s x T %s + in-scatter %s" % [_vec(albedo * light * _atmosphere.luminance_scale),
				_vec(transmittance), _vec(to_point["luminance"] * _atmosphere.luminance_scale)])
		# Sensitivity to the drawn surface's height: the ellipsoid mesh's facets lie up to ~0.5 km below the ellipsoid.
		var lower := point_itrf_m - normal_itrf * 300.0
		var lux_low: Dictionary = _core.atmosphere_illuminance(lower, normal_itrf, sun_itrf)
		var to_low: Dictionary = _core.atmosphere_sky_luminance_to_point(camera_itrf_m, lower, sun_itrf)
		var t_low: Vector3 = to_low["transmittance"]
		print("atmosphere_check:   the same 300 m lower: %s" % _vec((albedo * (lux_low["sun_lux"] + lux_low["sky_lux"]) / PI
				* t_low + to_low["luminance"]) * _atmosphere.luminance_scale))
	_compare(label + " [%.0f km]" % ((point_itrf_m - camera_itrf_m).length() / 1000.0), expected, got,
			SURFACE_TOLERANCE)


## Task 7 star cases: HIDDEN (noon: the pixel is the sky, 2 %), UNCHANGED (above the atmosphere: the look before the
## atmosphere plus the sky, 5 %), VISIBLE (twilight: the pixel's luminance > STAR_VISIBLE_FACTOR x the sky's).
enum StarCase { HIDDEN, UNCHANGED, VISIBLE }
const STAR_UNCHANGED_TOLERANCE := 0.05
const STAR_VISIBLE_FACTOR := 2.0
const STAR_COLOR_INDEX := 0.6  ## B-V of the test star (near white)
const SUN_RADIUS_M := 695700000.0
const SUN_RENDER_DISTANCE_M := 2.0e7  ## World.SUN_RENDER_DISTANCE_M
const SUN_MIN_RED_OVER_BLUE := 3.0
const MOON_TOLERANCE := 0.03
## [S: ITU-R BT.709] luminance weights (as star_visibility.gdshaderinc).
const LUMINANCE_WEIGHTS := Vector3(0.2126, 0.7152, 0.0722)


## One star of `magnitude` on the camera axis (StarField.build, shaders/stars.gdshader), the sky behind it compared per
## StarCase. The look before the atmosphere: the vertex colour (stored as RGBA8, truncated) x clamp(E / E_ref, 0.04, 1).
func _star_sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, magnitude: float,
		star_case: StarCase) -> void:
	var stars := StarField.new()
	stars.build(PackedVector3Array([_earth_to_gcrf * view_itrf]), PackedFloat32Array([magnitude]),
			PackedFloat32Array([STAR_COLOR_INDEX]))
	_viewport.add_child(stars)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	stars.queue_free()
	var cpu: Dictionary = _core.atmosphere_sky_luminance(camera_itrf_m, view_itrf, sun_itrf)
	var sky: Vector3 = cpu["luminance"] * _atmosphere.luminance_scale
	match star_case:
		StarCase.HIDDEN:
			_compare(label + " = sky", sky, got)
		StarCase.UNCHANGED:
			var c := StarField._color_from_bv(STAR_COLOR_INDEX)
			var color8 := Vector3(floorf(c.r * 255.0), floorf(c.g * 255.0), floorf(c.b * 255.0)) / 255.0
			var relative := minf(StarField.illuminance_lux(magnitude)
					/ StarField.illuminance_lux(StarField.REFERENCE_MAGNITUDE), 1.0)
			_compare(label + " = before + sky", color8 * clampf(relative, 0.04, 1.0) + sky, got, STAR_UNCHANGED_TOLERANCE)
		StarCase.VISIBLE:
			var ratio := got.dot(LUMINANCE_WEIGHTS) / sky.dot(LUMINANCE_WEIGHTS)
			_samples += 1
			var ok := ratio > STAR_VISIBLE_FACTOR
			if not ok:
				_failures += 1
			print("atmosphere_check: %-38s sky %s got %s luminance ratio %s (needs > %.0f) %s" % [label, _vec(sky),
					_vec(got), _num(ratio), STAR_VISIBLE_FACTOR, "ok" if ok else "FAIL"])


## The Sun disc (shaders/sun.gdshader, placed as World._place_sun) on the camera axis with the Sun along sun_itrf:
## reddened near the horizon by the transmittance alone, red / blue > SUN_MIN_RED_OVER_BLUE.
func _sun_sample(label: String, camera_itrf_m: Vector3, sun_itrf: Vector3) -> void:
	var disc := MeshInstance3D.new()
	var sphere := SphereMesh.new()
	sphere.radius = 1.0
	sphere.height = 2.0
	disc.mesh = sphere
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/sun.gdshader")
	disc.material_override = material
	disc.position = Frames.to_render(_earth_to_gcrf * sun_itrf) * SUN_RENDER_DISTANCE_M
	disc.scale = Vector3.ONE * SUN_RADIUS_M * SUN_RENDER_DISTANCE_M / AU_M
	_viewport.add_child(disc)
	var got := await _render(camera_itrf_m, sun_itrf, sun_itrf)
	disc.queue_free()
	var cpu: Dictionary = _core.atmosphere_sky_luminance(camera_itrf_m, sun_itrf, sun_itrf)
	var red_over_blue := got.x / got.z if got.z > 0.0 else INF
	var ok := red_over_blue > SUN_MIN_RED_OVER_BLUE
	_samples += 1
	if not ok:
		_failures += 1
	print("atmosphere_check: %-38s got %s (CPU transmittance %s) red / blue %s (needs > %.0f) %s" % [label, _vec(got),
			_vec(cpu["transmittance"]), _num(red_over_blue), SUN_MIN_RED_OVER_BLUE, "ok" if ok else "FAIL"])


## The Moon (CelestialBody with shaders/moon.gdshader, scaled space) on the camera axis: rendered once without the
## atmosphere (atmosphere_enabled false: the Moon as before) and once with it; the second must be the first x the
## view ray's transmittance + the sky luminance in front of it (CPU), MOON_TOLERANCE.
func _moon_sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var moon_gcrf := camera_gcrf + _earth_to_gcrf * view_itrf * MOON_DISTANCE_M
	var sun_gcrf := _earth_to_gcrf * sun_itrf * AU_M
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/moon.gdshader")
	material.set_shader_parameter("albedo_texture", _constant_texture(SURFACE_ALBEDO_8))
	material.set_shader_parameter("earth_radius_m", 6371000.0)
	material.set_shader_parameter("sun_position_local", Frames.to_render(sun_gcrf - moon_gcrf))
	material.set_shader_parameter("earth_position_local", Frames.to_render(-moon_gcrf))
	material.set_shader_parameter("camera_position_local", Frames.to_render(camera_gcrf - moon_gcrf))
	var moon := CelestialBody.new()
	moon.setup(1737400.0, 0.0, material)
	moon.fixed_to_gcrf = Basis.IDENTITY  # Moon-local = GCRF in render axes (the uniforms above)
	moon.center_gcrf_m = moon_gcrf
	moon.place(camera_gcrf)
	_viewport.add_child(moon)
	RenderingServer.global_shader_parameter_set(&"atmosphere_enabled", false)
	var vacuum := await _render(camera_itrf_m, view_itrf, sun_itrf)
	RenderingServer.global_shader_parameter_set(&"atmosphere_enabled", true)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	moon.queue_free()
	var cpu: Dictionary = _core.atmosphere_sky_luminance(camera_itrf_m, view_itrf, sun_itrf)
	var transmittance: Vector3 = cpu["transmittance"]
	var expected: Vector3 = vacuum * transmittance + cpu["luminance"] * _atmosphere.luminance_scale
	print("atmosphere_check:   Moon without the atmosphere %s, transmittance %s" % [_vec(vacuum), _vec(transmittance)])
	_compare(label + " = Moon x T + sky", expected, got, MOON_TOLERANCE)


const DALLAS_LAT_DEG := 32.78
const DALLAS_LON_DEG := -96.80
const ECLIPSE_ZENITH_MAX_FRACTION := 0.01
const ECLIPSE_HORIZON_MIN_RATIO := 10.0
const ECLIPSE_OFF_TOLERANCE := 0.01
var _moon_gcrf := Vector3.ZERO  ## eclipse samples: the Moon's apparent position (GCRF, from the Earth's centre)
var _sun_gcrf := Vector3.ZERO  ## and the Sun's
var _force_segments := -1  ## >= 0: atmosphere_eclipse_segments forced after the update


## Section 11: the eclipse sky over Dallas (see _run).
func _eclipse_samples() -> void:
	var camera := _geodetic(DALLAS_LAT_DEG, DALLAS_LON_DEG, 2.0)
	var zenith := _local_direction(DALLAS_LAT_DEG, DALLAS_LON_DEG, 0.0, 90.0)
	var horizon := _local_direction(DALLAS_LAT_DEG, DALLAS_LON_DEG, 315.0, 3.0)  # NW: across the umbra's path
	var saved_rotation := _earth_to_gcrf
	var t := _core.utc_to_tdb_s(EPOCH_UTC)
	var sun_itrf := _set_true_sun_and_moon(t)
	print("atmosphere_check:   Dallas %s: eclipse fraction %.4f" % [EPOCH_UTC,
			_core.solar_eclipse_fraction(DALLAS_LAT_DEG, DALLAS_LON_DEG, 2.0, t)])
	_force_segments = 0
	var zenith_off := await _render(camera, zenith, sun_itrf)
	_force_segments = -1
	var zenith_on := await _render(camera, zenith, sun_itrf)
	_check("eclipse flag on at totality", _atmosphere.eclipse_segments > 0, "segments %d" % _atmosphere.eclipse_segments)
	var horizon_on := await _render(camera, horizon, sun_itrf)
	var y_off := zenith_off.dot(LUMINANCE_WEIGHTS)
	var y_on := zenith_on.dot(LUMINANCE_WEIGHTS)
	var y_horizon := horizon_on.dot(LUMINANCE_WEIGHTS)
	_check("totality, Dallas zenith", y_on < ECLIPSE_ZENITH_MAX_FRACTION * y_off,
			"segments %s, off %s, fraction %s (needs < %s)" % [_vec(zenith_on), _vec(zenith_off), _num(y_on / y_off),
			_num(ECLIPSE_ZENITH_MAX_FRACTION)])
	_check("totality, Dallas horizon NW el 3", y_horizon > ECLIPSE_HORIZON_MIN_RATIO * y_on,
			"horizon %s, %s x the zenith (needs > %.0f)" % [_vec(horizon_on), _num(y_horizon / y_on),
			ECLIPSE_HORIZON_MIN_RATIO])
	# Two hours later: Dallas outside the eclipse.
	var later := t + 7200.0
	_earth_to_gcrf = _core.earth_fixed_to_gcrf(later)
	sun_itrf = _set_true_sun_and_moon(later)
	print("atmosphere_check:   Dallas +2 h: eclipse fraction %.4f" % _core.solar_eclipse_fraction(DALLAS_LAT_DEG,
			DALLAS_LON_DEG, 2.0, later))
	for view in [["zenith", zenith], ["horizon NW el 3", horizon]]:
		_force_segments = 0
		var off := await _render(camera, view[1], sun_itrf)
		_force_segments = Atmosphere.ECLIPSE_SEGMENTS
		var on := await _render(camera, view[1], sun_itrf)
		_compare("+2 h, Dallas %s, segments on = off (flag %d)" % [view[0], _atmosphere.eclipse_segments], off, on,
				ECLIPSE_OFF_TOLERANCE)
	_force_segments = -1
	_moon_gcrf = Vector3.ZERO
	_earth_to_gcrf = saved_rotation


## The true apparent Sun and Moon at t (from the Earth's centre) for _render; returns the Sun's ITRF direction.
func _set_true_sun_and_moon(t: float) -> Vector3:
	_sun_gcrf = _core.apparent_body_position_gcrf_m("sun", t, Vector3.ZERO, Vector3.ZERO)
	_moon_gcrf = _core.apparent_body_position_gcrf_m("moon", t, Vector3.ZERO, Vector3.ZERO)
	return (_earth_to_gcrf.transposed() * _sun_gcrf).normalized()


## Eye adaptation: the adapted zenith at Sun -3 deg must exceed this scene luminance - ~7 % of the noon zenith at the
## day exposure (0.13), well above black through the AgX tone mapper - and the adapted night zenith (Sun -12 deg) stay
## below the second, ~1 % of the twilight minimum.
const ADAPTED_TWILIGHT_MIN := 0.01
const ADAPTED_NIGHT_MAX := 1.0e-4
var _last_gain := 1.0


## A sky sample with the eye adaptation on, adapted to the view (snap): the CPU sky x the adapted luminance scale (2 %;
## compare false: not compared - a night sky of ~1e-7 lies below the half-float HDR viewport's resolution); returns the
## rendered pixel.
func _adapted_sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3,
		compare := true) -> Vector3:
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	_last_gain = _atmosphere.exposure_gain
	var cpu: Dictionary = _core.atmosphere_sky_luminance(camera_itrf_m, view_itrf, sun_itrf)
	if compare:
		_compare(label + " [gain %.2f]" % _last_gain, cpu["luminance"] * _atmosphere.luminance_scale, got)
	else:
		print("atmosphere_check: %-38s CPU %s got %s (gain %.2f)" % [label, _vec(cpu["luminance"]
				* _atmosphere.luminance_scale), _vec(got), _last_gain])
	_atmosphere.eye_adaptation_enabled = false
	return got


## Counts and prints one pass / fail condition.
func _check(label: String, ok: bool, detail: String) -> void:
	_samples += 1
	if not ok:
		_failures += 1
	print("atmosphere_check: %-38s %s %s" % [label, detail, "ok" if ok else "FAIL"])


const CUBE_TOLERANCE := 0.05
const SUN_LIGHT_MIN_RED_OVER_BLUE := 3.0
const CUBE_FAR_DISTANCE_M := 40000.0
const CUBE_FAR_HEIGHT_M := 600.0  ## above the ellipsoid: the cube stands clear of the pad's horizon (drop ~125 m)
const CUBE_FAR_SIZE_M := 600.0  ## ~6 px at the check's 10 deg / 65 px
const CUBE_NEAR_DISTANCE_M := 20.0


## A white cube (lit_aerial.gdshader: albedo 1, roughness 1, no specular) of edge size_m centred at center_itrf_m, its
## local +Y along up_itrf and +Z along toward_itrf (horizontal), lit by the Sun light of Atmosphere.sun_light_at at
## light_point_itrf_m and the sky's ambient (radiance cubemap of the sky shader); rendered from camera_itrf_m along
## view_itrf (centre pixel).
func _render_cube(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, center_itrf_m: Vector3,
		up_itrf: Vector3, toward_itrf: Vector3, size_m: float, light_point_itrf_m: Vector3) -> Vector3:
	var cube := MeshInstance3D.new()
	var box := BoxMesh.new()
	box.size = Vector3.ONE * size_m
	cube.mesh = box
	var material := LitAerial.material(Color.WHITE, 1.0, 0.0)
	material.set_shader_parameter("specular", 0.0)
	cube.material_override = material
	var y := Frames.to_render(_earth_to_gcrf * up_itrf).normalized()
	var z := Frames.to_render(_earth_to_gcrf * toward_itrf)
	z = (z - y * z.dot(y)).normalized()
	cube.transform = Transform3D(Basis(y.cross(z), y, z), Frames.to_render(_earth_to_gcrf * (center_itrf_m - camera_itrf_m)))
	_viewport.add_child(cube)
	var light := DirectionalLight3D.new()
	var lit: Array = _atmosphere.sun_light_at(light_point_itrf_m, sun_itrf, 1.0)
	light.light_color = lit[0]
	light.light_energy = lit[1]
	var sun_render := Frames.to_render(_earth_to_gcrf * sun_itrf).normalized()
	light.basis = Basis.looking_at(-sun_render, Vector3.UP if absf(sun_render.y) < 0.99 else Vector3.RIGHT)
	_viewport.add_child(light)
	_environment.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	_environment.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
	cube.queue_free()
	light.queue_free()
	return got


## The cube at the pad seen from 12 m above it, straight down onto its top face; Sun 60 deg.
func _cube_near_sample(label: String, sun_itrf: Vector3) -> void:
	var up := _geodetic_up(_geodetic(PAD_LAT_DEG, PAD_LON_DEG, 0.0))
	var camera := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 14.0)
	var center := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 1.0)
	var top := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 2.0)
	var got := await _render_cube(camera, -up, sun_itrf, center, up, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 0.0),
			2.0, top)
	var lux: Dictionary = _core.atmosphere_illuminance(top, up, sun_itrf)
	var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera, top, sun_itrf)
	var light: Vector3 = (lux["sun_lux"] + lux["sky_lux"]) / PI
	var expected: Vector3 = (light * to_point["transmittance"] + to_point["luminance"]) * _atmosphere.luminance_scale
	print("atmosphere_check:   cube light: sun %s + sky %s (scene)" % [_vec(lux["sun_lux"] / PI * _atmosphere.luminance_scale),
			_vec(lux["sky_lux"] / PI * _atmosphere.luminance_scale)])
	_compare(label, expected, got, CUBE_TOLERANCE)


## The cube 40 km east of the pad, 600 m up, one face toward the pad camera (2 m); the Sun behind the camera
## (sun_itrf). Reference: the same face of the same cube 20 m in front of the camera, lit by the same light (at the far
## cube) and ambient; expected = (near - S_near) / T_near x T_far + S_far with the CPU's T and in-scatter.
func _cube_far_sample(label: String, sun_itrf: Vector3) -> void:
	var camera := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 2.0)
	var lon_far := PAD_LON_DEG + rad_to_deg(CUBE_FAR_DISTANCE_M / (WGS84_A_M * cos(deg_to_rad(PAD_LAT_DEG))))
	var center := _geodetic(PAD_LAT_DEG, lon_far, CUBE_FAR_HEIGHT_M)
	var up := _geodetic_up(center)
	var toward := camera - center
	toward = (toward - up * toward.dot(up)).normalized()  # horizontal at the cube, toward the camera
	var face := center + toward * (CUBE_FAR_SIZE_M * 0.5)
	var view := (face - camera).normalized()
	var far := await _render_cube(camera, view, sun_itrf, center, up, toward, CUBE_FAR_SIZE_M, face)
	var near_face := camera + view * CUBE_NEAR_DISTANCE_M
	var near := await _render_cube(camera, view, sun_itrf, near_face - toward * 0.5, up, toward, 1.0, face)
	var to_near: Dictionary = _core.atmosphere_sky_luminance_to_point(camera, near_face, sun_itrf)
	var to_far: Dictionary = _core.atmosphere_sky_luminance_to_point(camera, face, sun_itrf)
	var t_near: Vector3 = to_near["transmittance"]
	var t_far: Vector3 = to_far["transmittance"]
	var lit: Vector3 = (near - to_near["luminance"] * _atmosphere.luminance_scale) / t_near
	var expected: Vector3 = lit * t_far + to_far["luminance"] * _atmosphere.luminance_scale
	print("atmosphere_check:   cube face lit %s, T %s, in-scatter %s" % [_vec(lit), _vec(t_far),
			_vec(to_far["luminance"] * _atmosphere.luminance_scale)])
	_compare(label, expected, far, CUBE_TOLERANCE)


## The Sun light's linear colour (Atmosphere.sun_light_at) at a low Sun: reddened by the transmittance.
func _sun_light_colour_sample(label: String, point_itrf_m: Vector3, sun_itrf: Vector3) -> void:
	var lit: Array = _atmosphere.sun_light_at(point_itrf_m, sun_itrf, 1.0)
	var linear: Color = (lit[0] as Color).srgb_to_linear()
	var red_over_blue := linear.r / linear.b if linear.b > 0.0 else INF
	var ok := red_over_blue > SUN_LIGHT_MIN_RED_OVER_BLUE
	_samples += 1
	if not ok:
		_failures += 1
	print("atmosphere_check: %-38s linear (%.4f, %.4f, %.4f) energy %s red / blue %s (needs > %.0f) %s" % [label,
			linear.r, linear.g, linear.b, _num(lit[1]), _num(red_over_blue), SUN_LIGHT_MIN_RED_OVER_BLUE,
			"ok" if ok else "FAIL"])


## A square of half size `half_m` perpendicular to `view` (centred on the origin), every vertex with the normal `normal`.
static func _facing_quad(view: Vector3, normal: Vector3, half_m: float) -> ArrayMesh:
	var u := view.cross(Vector3.UP if absf(view.y) < 0.9 else Vector3.RIGHT).normalized() * half_m
	var w := view.cross(u).normalized() * half_m
	var vertices := PackedVector3Array([-u - w, u - w, u + w, -u - w, u + w, -u + w])
	var normals := PackedVector3Array()
	for i in range(vertices.size()):
		normals.append(normal)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


## The ray's first hit on the WGS84 ellipsoid (float64, in the polar-stretched space where it is a sphere of radius a).
static func _ellipsoid_hit(camera_itrf_m: Vector3, view_itrf: Vector3) -> Vector3:
	var stretch := Atmosphere.POLAR_STRETCH
	var p := Vector3(camera_itrf_m.x, camera_itrf_m.y, camera_itrf_m.z * stretch)
	var v := Vector3(view_itrf.x, view_itrf.y, view_itrf.z * stretch)
	var v_length := v.length()
	v /= v_length
	var along := p.dot(v)
	var b2 := p.length_squared() - along * along
	var t := -along - sqrt(WGS84_A_M * WGS84_A_M - b2)
	var hit := p + v * t
	return Vector3(hit.x, hit.y, hit.z / stretch)


## Geodetic up of a point on (or near) the WGS84 ellipsoid: the gradient (x / a^2, y / a^2, z / b^2).
static func _geodetic_up(point_itrf_m: Vector3) -> Vector3:
	var stretch := Atmosphere.POLAR_STRETCH
	return Vector3(point_itrf_m.x, point_itrf_m.y, point_itrf_m.z * stretch * stretch).normalized()


## Renders the sky for the camera (ITRF m) along view_itrf with the Sun along sun_itrf and compares the centre pixel.
func _sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var cpu: Dictionary = _core.atmosphere_sky_luminance(camera_itrf_m, view_itrf, sun_itrf)
	var expected: Vector3 = cpu["luminance"] * _atmosphere.luminance_scale
	_compare(label, expected, got)


## Counts and prints one sample: every channel within TOLERANCE (relative; absolute below the floor).
func _compare(label: String, expected: Vector3, got: Vector3, tolerance := TOLERANCE) -> void:
	_samples += 1
	var peak := maxf(expected.x, maxf(expected.y, expected.z))
	var ok := peak > 0.0
	var ratios := PackedStringArray()
	for k in range(3):
		var e := expected[k]
		var g := got[k]
		var ratio := g / e if e > 0.0 else INF
		ratios.append("%.4f" % ratio)
		var floor_value := peak * RELATIVE_FLOOR
		if e >= floor_value:
			ok = ok and absf(ratio - 1.0) <= tolerance
		else:
			ok = ok and absf(g - e) <= tolerance * floor_value
	if not ok:
		_failures += 1
	print("atmosphere_check: %-38s expected %s got %s ratio (%s) %s" % [label, _vec(expected), _vec(got),
			", ".join(ratios), "ok" if ok else "FAIL"])


## A ground ray: the sky shader shows the model ground (albedo GROUND_ALBEDO without a day texture) lit by the Sun and
## the sky, seen through the atmosphere. CPU: the model-ground hit in the stretched space (float64), then
## albedo x (sun + sky lux) / pi x T + in-scattered luminance, all x luminance_scale.
const GROUND_ALBEDO := 0.1  ## sky.gdshader's ground_albedo default
func _ground_sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var stretch := Atmosphere.POLAR_STRETCH
	var p := Vector3(camera_itrf_m.x, camera_itrf_m.y, camera_itrf_m.z * stretch)
	var v := Vector3(view_itrf.x, view_itrf.y, view_itrf.z * stretch).normalized()
	var radius := float(_atmosphere.parameters["bottom_radius_m"])
	var along := p.dot(v)
	var t := -along - sqrt(along * along - (p.length_squared() - radius * radius))
	var hit := p + v * t
	var point_itrf := Vector3(hit.x, hit.y, hit.z / stretch)
	var normal_itrf := Vector3(hit.x, hit.y, hit.z * stretch).normalized()
	var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera_itrf_m, point_itrf, sun_itrf)
	var lux: Dictionary = _core.atmosphere_illuminance(point_itrf, normal_itrf, sun_itrf)
	var transmittance: Vector3 = to_point["transmittance"]
	var light: Vector3 = (lux["sun_lux"] + lux["sky_lux"]) * (GROUND_ALBEDO / PI)
	var expected: Vector3 = (light * transmittance + to_point["luminance"]) * _atmosphere.luminance_scale
	_compare(label + " [ground, %.0f km]" % (t / 1000.0), expected, got)


## Review Focus 1 control: the grazing sample with the camera altitude uniform overridden to 0 (the model ground),
## the float32 position unchanged. The shader must follow the uniform: the forced render matches the CPU for a camera
## on the model ground (2 %), and it moved away from the true render by more than CONTROL_MIN_CHANGE (the true sample
## agrees with the CPU to ~0.1 %, so a change of that size is the uniform's doing, not noise).
const CONTROL_MIN_CHANGE := 0.005
func _altitude_control(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var true_value := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var true_altitude := _atmosphere.camera_altitude_m
	var forced := await _render(camera_itrf_m, view_itrf, sun_itrf, 0.0)
	var ground_camera := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, PAD_HEIGHT_M - true_altitude)
	var cpu: Dictionary = _core.atmosphere_sky_luminance(ground_camera, view_itrf, sun_itrf)
	var expected: Vector3 = cpu["luminance"] * _atmosphere.luminance_scale
	var change := absf(forced.y / true_value.y - 1.0) if true_value.y > 0.0 else 0.0
	var match_ratio := forced.y / expected.y if expected.y > 0.0 else INF
	var ok := change > CONTROL_MIN_CHANGE and absf(match_ratio - 1.0) <= TOLERANCE
	_samples += 1
	if not ok:
		_failures += 1
	print(("atmosphere_check: altitude control (uniform forced %.1f -> 0 m): green %s -> %s (change %.2f %%, needs > "
			+ "%.1f %%), CPU on the model ground %s, ratio %.4f %s") % [true_altitude, _num(true_value.y), _num(forced.y),
			change * 100.0, CONTROL_MIN_CHANGE * 100.0, _num(expected.y), match_ratio, "ok" if ok else "FAIL"])


func _render(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, altitude_override := NAN) -> Vector3:
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var sun_gcrf: Vector3 = _earth_to_gcrf * sun_itrf * AU_M
	if _moon_gcrf != Vector3.ZERO:
		sun_gcrf = _sun_gcrf  # the eclipse samples: the true Sun (its direction is sun_itrf) and the Moon
	_atmosphere.update(camera_gcrf, Vector3.ZERO, _earth_to_gcrf, sun_gcrf, 0.0, 1.0, _moon_gcrf)
	if _force_segments >= 0:
		RenderingServer.global_shader_parameter_set(&"atmosphere_eclipse_segments", _force_segments)
	if not is_nan(altitude_override):
		RenderingServer.global_shader_parameter_set(&"atmosphere_camera_altitude_m", altitude_override)
	var view_render := Frames.to_render(_earth_to_gcrf * view_itrf).normalized()
	var up := Vector3.UP if absf(view_render.y) < 0.9 else Vector3.RIGHT
	_camera.transform = Transform3D(Basis.looking_at(view_render, up), Vector3.ZERO)
	for i in range(SETTLE_FRAMES):
		await RenderingServer.frame_post_draw
	var image := _viewport.get_texture().get_image()
	var c := image.get_pixel(VIEW_SIZE / 2, VIEW_SIZE / 2)
	return Vector3(c.r, c.g, c.b)


static func _geodetic(lat_deg: float, lon_deg: float, height_m: float) -> Vector3:
	return HeightSource.geodetic_to_fixed(lat_deg, lon_deg, height_m)


## ITRF unit vector at azimuth (from north, east positive) and elevation above the geodetic horizon.
static func _local_direction(lat_deg: float, lon_deg: float, azimuth_deg: float, elevation_deg: float) -> Vector3:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	var up := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var east := Vector3(-sin(lon), cos(lon), 0.0)
	var north := Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	var az := deg_to_rad(azimuth_deg)
	var el := deg_to_rad(elevation_deg)
	return (up * sin(el) + (north * cos(az) + east * sin(az)) * cos(el)).normalized()


## The view direction from `camera` grazing the sphere of radius a + tangent_height on the `toward` side (a sphere
## of the equatorial radius: the CPU and the GPU see the same ray, its exact tangent height does not matter).
static func _limb_direction(camera: Vector3, toward: Vector3, tangent_height_m: float) -> Vector3:
	var c := camera.normalized()
	var e := (toward - c * toward.dot(c)).normalized()
	var d := camera.length()
	var radius := 6378137.0 + tangent_height_m
	var tangent := c * (radius * radius / d) + e * (radius * sqrt(1.0 - (radius / d) * (radius / d)))
	return (tangent - camera).normalized()


func _fail_and_quit(message: String) -> void:
	print("atmosphere_check: FAIL - " + message)
	quit(1)


## Five significant digits in scientific form (GDScript's % has no %g).
static func _num(x: float) -> String:
	if x == 0.0 or not is_finite(x):
		return str(x)
	var exponent := floori(log(absf(x)) / log(10.0))
	return "%.4fe%d" % [x / pow(10.0, exponent), exponent]


static func _vec(v: Vector3) -> String:
	return "(%s, %s, %s)" % [_num(v.x), _num(v.y), _num(v.z)]
