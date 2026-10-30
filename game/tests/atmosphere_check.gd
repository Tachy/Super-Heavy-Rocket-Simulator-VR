# GPU check of the atmosphere shader include (M8 atmosphere, Task 5): renders the sky (shaders/sky.gdshader, nothing
# else in the scene) into an HDR SubViewport for sample views, reads the float pixel on the view direction and compares
# it with the core's CPU evaluation (StarsimCore.atmosphere_sky_luminance x Atmosphere.luminance_scale) within 2 % per
# channel. Linear tone mapper, exposure 1, glow off: the read value is the scene value. Exit code 1 on failure.
# Task 6 adds the Earth's surfaces (terrain_earth.gdshader, earth.gdshader incl. scaled space: lit by the transmitted
# Sun and the sky light, seen through the atmosphere; 3 %). STARSIM_VERBOSE=1 prints the surface samples' parts.
# Task 7 adds the stars (hidden at noon, unchanged above the atmosphere, visible in twilight), the Sun disc's reddening
# and the daytime Moon (Moon x T + sky, 3 %); Task 8 a star near the threshold (Sun -6 deg), vehicles and structures
# (a white lit_aerial cube at the pad and 40 km away, 5 %) and the Sun light's colour at a low Sun; the eye adaptation
# (section 10, otherwise off); Task 9 the eclipse sky over Dallas at totality (section 11); the final review (I1)
# stars beside the sunlit Earth from 400 km by day, hidden for the adapted eye, and the adapted totality star; section
# 12 the eye adaptation away from the Earth (lunar day / night, deep space); section 13 night vision (twilight, a
# moonless and a full-Moon night, the earthshine on a crescent, day frames unchanged). The run asserts its sample count
# (EXPECTED_SAMPLES): a GDScript error inside an awaited sample ends that coroutine silently and would otherwise pass.
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
## Every sample of a complete run (_check / _compare calls and the inline checks): a run with fewer ended a sample early.
const EXPECTED_SAMPLES := 88
var _last_image: Image  ## the last _render's frame


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
	_atmosphere.umbra_light_synchronous = true  # the eclipse samples render one frame after the update
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
	# Adapted to each view: the noon zenith (reference), the zenith at Sun -3 deg (visible: > ADAPTED_TWILIGHT_MIN,
	# darker and less colourful than noon on screen: the brightness/colourfulness term), at Sun -8 deg, at Sun -12 deg
	# (dark: < ADAPTED_NIGHT_MAX, a magnitude-1 star visible) and the order noon > -3 > -8 > -12 of the displayed zenith;
	# 400 km by day: gain 1 (the day look in orbit).
	var noon_zenith := await _adapted_sample("adapted, pad noon zenith", pad, zenith, sun_60)
	var noon_scene := _last_scene
	var twilight := await _adapted_sample("adapted, pad zenith, Sun -3 deg", pad, zenith,
			_local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -3.0))
	var twilight_scene := _last_scene
	var twilight_y := twilight.dot(LUMINANCE_WEIGHTS)
	var noon_y := noon_zenith.dot(LUMINANCE_WEIGHTS)
	var twilight_factors := "brightness %.4f, chroma %.4f" % [_atmosphere.brightness, _atmosphere.chroma]
	# The suprathreshold term (user decision 2026-10-25, Stevens and Hunt effects): the dusk zenith darker and less
	# colourful than the noon zenith on screen. (Before it, Ferwerda's Weber-range gain with half the brightest sky mapped
	# the -3 deg zenith 1.92 x brighter than noon; this check then only asked "below the brightest sky".)
	_check("adapted Sun -3 deg zenith visible, below noon", twilight_y > ADAPTED_TWILIGHT_MIN and twilight_y < noon_y,
			"luminance %s (needs > %s and < the noon zenith %s), ratio %s; %s" % [_num(twilight_y),
			_num(ADAPTED_TWILIGHT_MIN), _num(noon_y), _num(twilight_y / noon_y), twilight_factors])
	# Less colourful: the displayed chroma relative to the scene's (the sky's own colour, from the CPU) below noon's
	# (1: the day look). Not the absolute chroma: the -3 deg zenith's own colour is ~1.6 x more saturated than the
	# noon zenith's (the deep twilight blue), more than the Hunt factor takes away (printed).
	var twilight_relative := _chroma_of(twilight) / _chroma_of(twilight_scene)
	var noon_relative := _chroma_of(noon_zenith) / _chroma_of(noon_scene)
	_check("adapted Sun -3 deg zenith less colourful than noon", twilight_relative < noon_relative
			and noon_relative > 0.99, "displayed / scene chroma %s (needs < noon %s, ~1); chroma |c - Y| / Y on screen %s, scene %s, noon on screen %s; %s" % [
			_num(twilight_relative), _num(noon_relative), _num(_chroma_of(twilight)), _num(_chroma_of(twilight_scene)),
			_num(_chroma_of(noon_zenith)), twilight_factors])
	var dusk := await _adapted_sample("adapted, pad zenith, Sun -8 deg", pad, zenith,
			_local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -8.0))
	var dusk_y := dusk.dot(LUMINANCE_WEIGHTS)
	var sun_m12 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -12.0)
	var night := await _adapted_sample("adapted, pad zenith, Sun -12 deg", pad, zenith, sun_m12, false)
	var night_rods := _atmosphere.rod_scale
	var night_y := night.dot(LUMINANCE_WEIGHTS)
	_check("adapted Sun -12 deg zenith dim, rod vision", night_y < ADAPTED_NIGHT_MAX * twilight_y
			and night_rods > 1.0, "luminance %s (needs < %s x the Sun -3 deg zenith), rod scale %s (needs > 1)" % [
			_num(night_y), _num(ADAPTED_NIGHT_MAX), _num(night_rods)])
	_check("adapted zenith order noon > -3 > -8 > -12", noon_y > twilight_y and twilight_y > dusk_y
			and dusk_y > night_y, "on screen %s > %s > %s > %s" % [_num(noon_y), _num(twilight_y), _num(dusk_y),
			_num(night_y)])
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	await _star_sample("adapted, Sun -12 deg, magnitude-1 star", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0,
			45.0), sun_m12, 1.0, StarCase.VISIBLE)
	_atmosphere.eye_adaptation_enabled = false
	await _adapted_sample("adapted, 400 km by day, limb at 10 km", orbit, _limb_direction(orbit, Vector3(0, 0, 1),
			10000.0), sun_orbit)
	_check("adapted 400 km by day: the day exposure", is_equal_approx(_last_gain, 1.0), "gain %.4f" % _last_gain)
	# Stars beside the sunlit Earth from 400 km by day (final review I1): toward deep space just above the limb (the ray
	# passes 200 km above the ground, outside the atmosphere: no sky behind the star), the eye adapted to the Sun and
	# the day-side Earth hides faint stars; with the fixed exposure (no adaptation) the same star shows (control).
	var space_view := _limb_direction(orbit, Vector3(0, 0, 1), 200000.0)
	await _space_star_sample("adapted 400 km day, magnitude-4 star", orbit, space_view, sun_orbit, 4.0, true, false)
	await _space_star_sample("adapted 400 km day, magnitude-0 star", orbit, space_view, sun_orbit, 0.0, true, false)
	await _space_star_sample("fixed 400 km day, magnitude-4 star", orbit, space_view, sun_orbit, 4.0, false, true)
	# ~60 s after orbital sunset at 400 km (the Sun 24 deg below the camera's horizontal, the shadow entry at ~20 deg):
	# the lit limb toward the Sun in view (review I1). The game's path: the Moon passed (world.gd always passes it; a
	# missing Moon once hid that the limb glow never ran in the game - night-vision review I1), 40 deg from the Sun
	# (a waning crescent: its light negligible).
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	var sun_after_sunset := _local_direction(10.0, -60.0, 90.0, -24.0)
	var moon_side := _local_direction(10.0, -60.0, 0.0, 0.0)
	_moon_center_gcrf = _earth_to_gcrf * ((sun_after_sunset * cos(deg_to_rad(40.0)) + moon_side
			* sin(deg_to_rad(40.0))).normalized() * MOON_DISTANCE_M)
	var limb_east := _limb_direction(orbit, _local_direction(10.0, -60.0, 90.0, 0.0), 10000.0)
	var limb_pixel := await _render(orbit, limb_east, sun_after_sunset)
	_moon_center_gcrf = Vector3.ZERO
	var limb_peak := maxf(limb_pixel.x, maxf(limb_pixel.y, limb_pixel.z))
	var glow := _atmosphere.last_adaptation_glow_cd_m2
	_check("adapted 400 km, 60 s after sunset", limb_peak < ADAPTED_LIMB_MAX and glow > 0.0
			and is_equal_approx(_atmosphere.last_target_cd_m2, glow),
			"sunward limb at 10 km %s (peak needs < %.1f, the glow threshold); limb glow %s cd/m^2 (needs > 0 and chosen; the Moon passed), gain %.2f, down %s, Earth in view %s, Moon in view %s cd/m^2" % [
			_vec(limb_pixel), ADAPTED_LIMB_MAX, _num(glow), _atmosphere.exposure_gain,
			_num(_atmosphere.last_adaptation_down_cd_m2), _num(_atmosphere.last_adaptation_up_cd_m2),
			_num(_atmosphere.last_adaptation_moon_cd_m2)])
	_atmosphere.eye_adaptation_enabled = false

	# 11. Solar eclipse in the sky (Task 9, fix round 1): 2024-04-08 18:42 UTC from Dallas (totality) - the zenith within
	# 0.3 .. 10 cd/m^2 (with the umbra's second-order light), the horizon away from the umbra > 3 x the zenith, a
	# magnitude-1 star visible and a magnitude-4 star hidden; the adapted eye 400 km up in the umbra; two hours later
	# (Dallas out of the eclipse, the penumbra still on the Earth) segments on and off agree within 1 %.
	await _eclipse_samples()

	# 12. Eye adaptation away from the Earth (Moon-eye fix 2026-10-25): an eye on the Moon at lunar day (gain 1: the
	# full Sun at the eye), at lunar night with the Earth nearly full (adapted to the Earth in view and the earthshine-lit
	# ground: gain > 1, at most GAIN_MAX; the Earth term against the Lambert disc at lunar distance), and in deep space
	# halfway to the Moon, sunlit (gain 1); the earthshine (full Earth in lux, the Moon shader's dark side at two phases)
	# and the vehicles' Sun light on lunar terrain below the mean sphere at lunar day.
	await _away_from_earth_samples()
	_moon_terrain_sun_samples()

	# 13. Night vision (user decision 2026-10-25): day frames unchanged (the adapted day = the fixed exposure x the
	# gain, no rods, no moonlight), the twilight zenith at Sun -8 deg visible and blue, a moonless night dark with a
	# magnitude-6 star visible, the full-Moon ground visible with rod vision dominating (its cone image the albedo's
	# colour), a magnitude-2 star shown and a magnitude-5 star hidden under the full Moon, and the earthshine on a
	# 2-day crescent above the twilight sky beside it.
	await _night_vision_samples()

	if _samples != EXPECTED_SAMPLES:
		_failures += 1
		print("atmosphere_check: FAIL - %d samples ran, %d expected (a sample ended early: a script error?)" % [_samples,
				EXPECTED_SAMPLES])
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
	# Night vision's rod step (as world.gd): active only with the eye adaptation on and rods in view.
	var compositor := Compositor.new()
	compositor.compositor_effects = [_atmosphere.vision_effect]
	world_environment.compositor = compositor
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
	_earth_material.set_shader_parameter("moon_radius_m", 1737400.0)
	_earth_material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	_earth_body.setup(WGS84_A_M, WGS84_FLATTENING, _earth_material)
	_earth_body.visible = false
	_viewport.add_child(_earth_body)
	_terrain_material.shader = load("res://shaders/terrain_earth.gdshader")
	_terrain_material.set_shader_parameter("day_texture", albedo_texture)
	_terrain_material.set_shader_parameter("day_texture_bound", true)
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
	var got := await _render_surface(camera_itrf_m, point_itrf_m, normal_itrf, sun_itrf, terrain)
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


## The surface pixel of _surface_sample (rendered; the terrain quad or the ellipsoid mesh).
func _render_surface(camera_itrf_m: Vector3, point_itrf_m: Vector3, normal_itrf: Vector3, sun_itrf: Vector3,
		terrain: bool) -> Vector3:
	var view_itrf := (point_itrf_m - camera_itrf_m).normalized()
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var sun_local := Frames.to_render(sun_itrf * AU_M)
	var moon_local := -Frames.to_render(sun_itrf) * MOON_DISTANCE_M
	ShaderGlobals.set_value(&"earth_sun_position_local", sun_local)
	ShaderGlobals.set_value(&"earth_moon_position_local", moon_local)
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
		ShaderGlobals.set_value(&"earth_terrain_camera_hi", camera_split[0])
		ShaderGlobals.set_value(&"earth_terrain_camera_lo", camera_split[1])
	else:
		_earth_body.fixed_to_gcrf = _earth_to_gcrf
		_earth_body.center_gcrf_m = Vector3.ZERO
		_earth_body.place(camera_gcrf)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	_earth_body.visible = false
	_terrain_parent.visible = false
	return got


## A star where the sky behind it is (near) black: the render with the star against the same render without it, with
## the eye adaptation on (adapted, snapped to the view) or off. Visible: the star adds more than SPACE_STAR_MIN_ADDED
## (its undimmed look is >= 0.04 x its colour); hidden: it adds less than SPACE_STAR_MAX_ADDED (absolute, scene
## units - the 2 % relative comparison has no scale on a black sky).
const SPACE_STAR_MIN_ADDED := 0.01
const SPACE_STAR_MAX_ADDED := 1.0e-4
func _space_star_sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3,
		magnitude: float, adapted: bool, visible: bool) -> void:
	_atmosphere.eye_adaptation_enabled = adapted
	_atmosphere.snap_adaptation()
	var sky := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var stars := StarField.new()
	stars.build(PackedVector3Array([_earth_to_gcrf * view_itrf]), PackedFloat32Array([magnitude]),
			PackedFloat32Array([STAR_COLOR_INDEX]))
	_viewport.add_child(stars)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	stars.queue_free()
	var adaptation := _atmosphere.adaptation_cd_m2
	_atmosphere.eye_adaptation_enabled = false
	var added := (got - sky).dot(LUMINANCE_WEIGHTS)
	var ok := added > SPACE_STAR_MIN_ADDED if visible else absf(added) < SPACE_STAR_MAX_ADDED
	_check(label + (" visible" if visible else " hidden"), ok, "sky %s got %s added %s (needs %s %s), adapted to %s cd/m^2"
			% [_vec(sky), _vec(got), _num(added), ">" if visible else "|.| <",
			_num(SPACE_STAR_MIN_ADDED if visible else SPACE_STAR_MAX_ADDED), _num(adaptation)])


## Task 7 star cases: HIDDEN (noon: the pixel is the sky, 2 %), UNCHANGED (above the atmosphere: the look before the
## atmosphere plus the sky, 5 %), VISIBLE (twilight: the pixel's luminance > STAR_VISIBLE_FACTOR x the sky's).
enum StarCase { HIDDEN, UNCHANGED, VISIBLE }
const STAR_UNCHANGED_TOLERANCE := 0.05
const STAR_VISIBLE_FACTOR := 2.0
const STAR_COLOR_INDEX := 0.6  ## B-V of the test star (near white)
const SUN_RENDER_DISTANCE_M := 2.0e7  ## World.SUN_RENDER_DISTANCE_M
const SUN_MIN_RED_OVER_BLUE := 3.0
const MOON_TOLERANCE := 0.03
## [S: ITU-R BT.709] luminance weights (as star_visibility.gdshaderinc).
const LUMINANCE_WEIGHTS := Vector3(0.2126, 0.7152, 0.0722)


## One star of `magnitude` on the camera axis (StarField.build, shaders/stars.gdshader), the sky behind it compared per
## StarCase. The look before the atmosphere: the vertex colour (stored as RGBA8, truncated) x clamp(E / E_ref, 0.04, 1).
func _star_sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, magnitude: float,
		star_case: StarCase) -> void:
	# VISIBLE: against the rendered sky (with the eye adaptation on it carries night vision's rods, the CPU sky not).
	var rendered_sky := Vector3.ZERO
	if star_case == StarCase.VISIBLE:
		rendered_sky = await _render(camera_itrf_m, view_itrf, sun_itrf)
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
			sky = rendered_sky
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
	disc.material_override = World.sun_disc_material()  # the game's (its physical value)
	disc.position = Frames.to_render(_earth_to_gcrf * sun_itrf) * SUN_RENDER_DISTANCE_M
	disc.scale = Vector3.ONE * Atmosphere.SUN_RADIUS_M * SUN_RENDER_DISTANCE_M / AU_M
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
	ShaderGlobals.set_value(&"atmosphere_enabled", false)
	var vacuum := await _render(camera_itrf_m, view_itrf, sun_itrf)
	ShaderGlobals.set_value(&"atmosphere_enabled", true)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	moon.queue_free()
	var cpu: Dictionary = _core.atmosphere_sky_luminance(camera_itrf_m, view_itrf, sun_itrf)
	var transmittance: Vector3 = cpu["transmittance"]
	var expected: Vector3 = vacuum * transmittance + cpu["luminance"] * _atmosphere.luminance_scale
	print("atmosphere_check:   Moon without the atmosphere %s, transmittance %s" % [_vec(vacuum), _vec(transmittance)])
	_compare(label + " = Moon x T + sky", expected, got, MOON_TOLERANCE)


const DALLAS_LAT_DEG := 32.78
const DALLAS_LON_DEG := -96.80
## Totality zenith: measured ~1-2 cd/m^2 (sky brightness ~11-12 mag/arcsec^2; Task 8+9 review); the band allows the
## second-order model's spread.
const ECLIPSE_ZENITH_MIN_CD_M2 := 0.3
const ECLIPSE_ZENITH_MAX_CD_M2 := 10.0
## Eye adaptation just past orbital sunset: the sunward limb stays below Godot's glow threshold (world.gd: 4.0) - not
## blown out.
const ADAPTED_LIMB_MAX := 4.0
## The eye 400 km up in the umbra adapts to the sunlit, penumbral Earth around it: above this (cd/m^2; Task 10: 2390;
## the eye's own shadow would leave it near 0). Was "gain < GAIN_MAX = 25" while the gain had that display cap.
const ADAPTED_UMBRA_MIN_CD_M2 := 100.0
var _tdb := NAN  ## the instant for the Moon's shadow on other points (eclipse samples); NAN: none
const ECLIPSE_HORIZON_MIN_RATIO := 3.0
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
	_tdb = t
	var sun_itrf := _set_true_sun_and_moon(t)
	print("atmosphere_check:   Dallas %s: eclipse fraction %.4f" % [EPOCH_UTC,
			_core.solar_eclipse_fraction(DALLAS_LAT_DEG, DALLAS_LON_DEG, 2.0, t)])
	var zenith_on := await _render(camera, zenith, sun_itrf)
	_check("eclipse flag on at totality", _atmosphere.eclipse_segments > 0, "segments %d" % _atmosphere.eclipse_segments)
	print("atmosphere_check:   umbra fluence (cd/m^2 sr at 1 AU) at 0 / 4 / 12 km: %s %s %s" % [
			_vec(_atmosphere.umbra_fluence[0]), _vec(_atmosphere.umbra_fluence[1]), _vec(_atmosphere.umbra_fluence[2])])
	var horizon_on := await _render(camera, horizon, sun_itrf)
	var zenith_cd := zenith_on.dot(LUMINANCE_WEIGHTS) / _atmosphere.scene_per_cd_m2
	var horizon_cd := horizon_on.dot(LUMINANCE_WEIGHTS) / _atmosphere.scene_per_cd_m2
	_check("totality, Dallas zenith luminance", zenith_cd >= ECLIPSE_ZENITH_MIN_CD_M2 and zenith_cd <= ECLIPSE_ZENITH_MAX_CD_M2,
			"%s cd/m^2 (needs %s .. %s)" % [_num(zenith_cd), _num(ECLIPSE_ZENITH_MIN_CD_M2), _num(ECLIPSE_ZENITH_MAX_CD_M2)])
	_check("totality, Dallas horizon NW el 3", horizon_cd > ECLIPSE_HORIZON_MIN_RATIO * zenith_cd,
			"%s cd/m^2, %s x the zenith (needs > %.0f)" % [_num(horizon_cd), _num(horizon_cd / zenith_cd),
			ECLIPSE_HORIZON_MIN_RATIO])
	await _eclipse_star_sample("totality, magnitude-1 star", camera, zenith, sun_itrf, 1.0, true)
	await _eclipse_star_sample("totality, magnitude-4 star", camera, zenith, sun_itrf, 4.0, false)
	# The same with the eye adapted to the totality (final review I1: the adaptation floor must not hide them).
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	await _eclipse_star_sample("adapted totality, magnitude-1 star", camera, zenith, sun_itrf, 1.0, true)
	var started_us := Time.get_ticks_usec()
	_atmosphere.adaptation_luminance(camera, sun_itrf, 0.0, 1.0)
	var evaluation_ms := (Time.get_ticks_usec() - started_us) / 1000.0
	# The eye in totality adapts by the twilight rule (night-vision review I2): half the brightest sky, the ring around
	# the umbra (brighter than the zenith by the horizon check's ratio).
	var ring_glow := _atmosphere.last_adaptation_glow_cd_m2
	_check("adapted totality: the brightest sky", is_equal_approx(_atmosphere.last_target_cd_m2, ring_glow)
			and ring_glow > Atmosphere.GLOW_ADAPTATION_SHARE * ECLIPSE_HORIZON_MIN_RATIO * zenith_cd,
			"gain %.2f, adaptation luminance %s cd/m^2 = glow %s (needs chosen and > %.1f x the zenith %s; down %s, Earth in view %s; one evaluation in the eclipse %.2f ms)" % [
			_atmosphere.exposure_gain, _num(_atmosphere.adaptation_cd_m2), _num(ring_glow),
			Atmosphere.GLOW_ADAPTATION_SHARE * ECLIPSE_HORIZON_MIN_RATIO, _num(zenith_cd),
			_num(_atmosphere.last_adaptation_down_cd_m2), _num(_atmosphere.last_adaptation_up_cd_m2), evaluation_ms])
	await _eclipsed_moon_sample(camera, t, sun_itrf)
	_atmosphere.eye_adaptation_enabled = false
	# The eye adaptation 400 km up in the umbra (on the shadow axis), the sunlit Earth around the umbra in view: it
	# adapts to that Earth (review I1), not to the eye's own shadow.
	var umbra_400 := _umbra_axis_point(t, WGS84_A_M + 400000.0)
	var umbra_ground := _umbra_axis_point(t, WGS84_A_M)
	var east := Vector3(-umbra_ground.y, umbra_ground.x, 0.0).normalized()
	for km in [200.0, 400.0, 1000.0, 2000.0]:
		var p: Vector3 = (umbra_ground + east * km * 1000.0).normalized() * WGS84_A_M
		print("atmosphere_check:   eclipse fraction %.0f km east of the umbra centre: %.3f" % [km,
				_core.solar_eclipse_fraction_gcrf(_earth_to_gcrf * p, t)])
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	await _render(umbra_400, -umbra_400.normalized(), sun_itrf)
	_check("adapted 400 km in the umbra", _atmosphere.adaptation_cd_m2 > ADAPTED_UMBRA_MIN_CD_M2,
			"gain %.2f, adapted to %s cd/m^2 (needs > %s: the penumbral Earth in view, not the eye's shadow; down %s, Earth in view %s cd/m^2, eclipse fraction %.3f)" % [
			_atmosphere.exposure_gain, _num(_atmosphere.adaptation_cd_m2), _num(ADAPTED_UMBRA_MIN_CD_M2),
			_num(_atmosphere.last_adaptation_down_cd_m2),
			_num(_atmosphere.last_adaptation_up_cd_m2), _core.solar_eclipse_fraction_gcrf(_earth_to_gcrf * umbra_400, t)])
	_atmosphere.eye_adaptation_enabled = false
	# Two hours later: Dallas outside the eclipse.
	var later := t + 7200.0
	_tdb = later
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
	# A twilight eye on the eclipse day (night-vision review I2): the Sun 3 deg below its horizon on the side of the
	# Earth away from the Moon's shadow while the penumbra is still on the Earth - it keeps the twilight glow.
	var moon_itrf := (_earth_to_gcrf.transposed() * _moon_gcrf).normalized()
	var away := -(moon_itrf - sun_itrf * moon_itrf.dot(sun_itrf)).normalized()
	var twilight_up := (sun_itrf * cos(deg_to_rad(93.0)) + away * sin(deg_to_rad(93.0))).normalized()
	var twilight_eye := _geodetic(rad_to_deg(asin(twilight_up.z)), rad_to_deg(atan2(twilight_up.y, twilight_up.x)), 2.0)
	var fraction := _core.solar_eclipse_fraction_gcrf(_earth_to_gcrf * twilight_eye, later)
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	await _render(twilight_eye, twilight_up, sun_itrf)
	_check("+2 h, twilight eye off the shadow: glow", _atmosphere.eclipse_segments > 0 and fraction == 0.0
			and _atmosphere.last_adaptation_glow_cd_m2 > 0.0
			and is_equal_approx(_atmosphere.last_target_cd_m2, _atmosphere.last_adaptation_glow_cd_m2),
			"penumbra on the Earth %s, eclipse fraction at the eye %.3f, glow %s cd/m^2 (needs > 0 and chosen), gain %.1f" % [
			_atmosphere.eclipse_segments > 0, fraction, _num(_atmosphere.last_adaptation_glow_cd_m2),
			_atmosphere.exposure_gain])
	_atmosphere.eye_adaptation_enabled = false
	_moon_gcrf = Vector3.ZERO
	_tdb = NAN
	_earth_to_gcrf = saved_rotation


## The eclipsed Moon at totality (brightness-term task 2, 2026-10-25): the eye adapted, MSAA 2x as the game, the Moon a
## moon.gdshader body (grey MOON_MEAN_TEXTURE_ALBEDO) at its true apparent place filling about half the view, the true
## Sun behind it. No pixel may exceed ECLIPSE_RIM_MAX x the brighter of the disc centre and the sky corner: the Sun
## lit the rim through normals facing away from the eye (white pixels around the disc). The disc centre (earthshine)
## and the sky are printed in cd/m^2.
const ECLIPSE_RIM_MAX := 2.0
const ECLIPSE_MOON_FOV_DEG := 1.0
func _eclipsed_moon_sample(camera_itrf_m: Vector3, t: float, sun_itrf: Vector3) -> void:
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var moon_gcrf := _moon_gcrf  # apparent from the Earth's centre, as World draws it
	var albedo := Color(Atmosphere.MOON_MEAN_TEXTURE_ALBEDO, Atmosphere.MOON_MEAN_TEXTURE_ALBEDO,
			Atmosphere.MOON_MEAN_TEXTURE_ALBEDO).linear_to_srgb()
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/moon.gdshader")
	material.set_shader_parameter("albedo_texture", _constant_texture(albedo))
	material.set_shader_parameter("earth_radius_m", 6371000.0)
	material.set_shader_parameter("sun_position_local", Frames.to_render(_sun_gcrf - moon_gcrf))
	material.set_shader_parameter("earth_position_local", Frames.to_render(-moon_gcrf))
	material.set_shader_parameter("camera_position_local", Frames.to_render(camera_gcrf - moon_gcrf))
	var moon := CelestialBody.new()
	moon.setup(Atmosphere.MOON_RADIUS_M, 0.0, material)
	moon.fixed_to_gcrf = Basis.IDENTITY
	moon.center_gcrf_m = moon_gcrf
	moon.place(camera_gcrf)
	_viewport.add_child(moon)
	_moon_center_gcrf = _core.body_position_gcrf_m("moon", t)  # the earthshine (moon_earthshine)
	var view := (_earth_to_gcrf.transposed() * (moon_gcrf - camera_gcrf)).normalized()
	_camera.fov = ECLIPSE_MOON_FOV_DEG
	_viewport.msaa_3d = Viewport.MSAA_2X
	_atmosphere.snap_adaptation()
	var centre := await _render(camera_itrf_m, view, sun_itrf)
	var image := _last_image
	_viewport.msaa_3d = Viewport.MSAA_DISABLED
	_camera.fov = 10.0
	moon.queue_free()
	_moon_center_gcrf = Vector3.ZERO
	var corner_c := image.get_pixel(0, 0)
	var corner := Vector3(corner_c.r, corner_c.g, corner_c.b)
	var peak := 0.0
	for y in range(VIEW_SIZE):
		for x in range(VIEW_SIZE):
			var p := image.get_pixel(x, y)
			peak = maxf(peak, Vector3(p.r, p.g, p.b).dot(LUMINANCE_WEIGHTS))
	var reference := maxf(centre.dot(LUMINANCE_WEIGHTS), corner.dot(LUMINANCE_WEIGHTS))
	var per_cd := _atmosphere.scene_per_cd_m2
	_check("adapted totality: eclipsed Moon, no lit rim", peak <= ECLIPSE_RIM_MAX * reference,
			"brightest pixel %s (needs <= %.1f x %s); disc centre %s = %s cd/m^2 on screen, sky beside it %s cd/m^2, contrast %.3f; earthshine %s lux; %s" % [
			_num(peak), ECLIPSE_RIM_MAX, _num(reference), _vec(centre), _num(centre.dot(LUMINANCE_WEIGHTS) / per_cd),
			_num(corner.dot(LUMINANCE_WEIGHTS) / per_cd), centre.dot(LUMINANCE_WEIGHTS) / maxf(corner.dot(
			LUMINANCE_WEIGHTS), 1.0e-30) - 1.0, _num(_atmosphere.last_earthshine_lux), _vision_parts()])


## A star of `magnitude` on the camera axis during the eclipse: compared with the same render without it (the CPU sky
## knows no eclipse). Visible: luminance ratio > STAR_VISIBLE_FACTOR; hidden: the pixel is the sky (2 %).
func _eclipse_star_sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3,
		magnitude: float, visible: bool) -> void:
	var sky := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var stars := StarField.new()
	stars.build(PackedVector3Array([_earth_to_gcrf * view_itrf]), PackedFloat32Array([magnitude]),
			PackedFloat32Array([STAR_COLOR_INDEX]))
	_viewport.add_child(stars)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	stars.queue_free()
	if visible:
		var ratio := got.dot(LUMINANCE_WEIGHTS) / sky.dot(LUMINANCE_WEIGHTS)
		_check(label + " visible", ratio > STAR_VISIBLE_FACTOR, "luminance ratio %s (needs > %.0f)" % [_num(ratio),
				STAR_VISIBLE_FACTOR])
	else:
		_compare(label + " hidden = sky", sky, got)


## The point on the Moon's shadow axis (from the Sun through the Moon) at distance radius_m from the Earth's centre,
## ITRF at t (the umbra's centre at that height).
func _umbra_axis_point(t: float, radius_m: float) -> Vector3:
	var rotation := _core.earth_fixed_to_gcrf(t).transposed()
	var sun := rotation * _core.apparent_body_position_gcrf_m("sun", t, Vector3.ZERO, Vector3.ZERO)
	var moon := rotation * _core.apparent_body_position_gcrf_m("moon", t, Vector3.ZERO, Vector3.ZERO)
	var axis := (moon - sun).normalized()
	var b := moon.dot(axis)
	return moon + axis * (-b - sqrt(b * b - (moon.length_squared() - radius_m * radius_m)))


## The true apparent Sun and Moon at t (from the Earth's centre) for _render; returns the Sun's ITRF direction.
func _set_true_sun_and_moon(t: float) -> Vector3:
	_sun_gcrf = _core.apparent_body_position_gcrf_m("sun", t, Vector3.ZERO, Vector3.ZERO)
	_moon_gcrf = _core.apparent_body_position_gcrf_m("moon", t, Vector3.ZERO, Vector3.ZERO)
	return (_earth_to_gcrf.transposed() * _sun_gcrf).normalized()


## Section 12: the Moon's geometric centre (GCRF) for the eye adaptation away from the Earth; ZERO: none.
var _moon_center_gcrf := Vector3.ZERO
## A gain "1" away from the Earth: within this.
const AWAY_GAIN_TOLERANCE := 0.05
## The Earth in view at lunar distance against the Lambert disc (its cap quadrature, the atmosphere): within this.
const AWAY_EARTH_TOLERANCE := 0.25
## Lunar night: the cone gain lies above the former display cap 25 (night vision 2026-10-25), so the earthshine-lit
## ground shows (adapted to the Earth in view, ~6 cd/m^2: ~2900).
const AWAY_NIGHT_MIN_GAIN := 25.0


## Section 12 (see _run). Synthetic geometry in the Earth-fixed frame: the Moon at MOON_DISTANCE_M, the eye 2 m above
## its mean sphere at the sub-Earth point; the Sun at 1 AU.
func _away_from_earth_samples() -> void:
	var moon_dir := Vector3(cos(0.3), sin(0.3), 0.0)
	var side := Vector3(-sin(0.3), cos(0.3), 0.0)
	var moon := moon_dir * MOON_DISTANCE_M
	var eye := moon - moon_dir * (Atmosphere.MOON_RADIUS_M + 2.0)
	_moon_center_gcrf = _earth_to_gcrf * moon
	_atmosphere.eye_adaptation_enabled = true
	# Lunar day: the Sun 60 deg up at the eye (30 deg from the Earth-Moon line on the Earth's side: no eclipse).
	var sun_day := (-moon_dir * cos(deg_to_rad(30.0)) + side * sin(deg_to_rad(30.0))).normalized()
	_atmosphere.snap_adaptation()
	await _render(eye, -moon_dir, sun_day)
	_check("adapted, lunar day (Sun 60 deg)", absf(_atmosphere.exposure_gain - 1.0) <= AWAY_GAIN_TOLERANCE,
			"gain %.4f (needs 1 +- %.2f); %s" % [_atmosphere.exposure_gain, AWAY_GAIN_TOLERANCE, _adaptation_parts()])
	# Lunar night, the Earth nearly full: the Sun 80 deg below the eye's horizon (10 deg from the Moon's direction seen
	# from the Earth: no solar eclipse on the Earth, no lunar eclipse).
	var phase := deg_to_rad(10.0)
	var sun_night := (moon_dir * cos(phase) + side * sin(phase)).normalized()
	_atmosphere.snap_adaptation()
	await _render(eye, -moon_dir, sun_night)
	var gain := _atmosphere.exposure_gain
	var reference := _atmosphere.reference_luminance(1.0)
	var cone_gain := EyeAdaptation.cone_gain(_atmosphere.adaptation_cd_m2, reference)
	_check("adapted, lunar night (Earth nearly full)", gain > AWAY_NIGHT_MIN_GAIN and is_equal_approx(gain, cone_gain)
			and _atmosphere.last_adaptation_moon_cd_m2 > 0.0,
			"gain %.2f (needs > %.0f and the cone gain %s of the adaptation; Weber gain %s), Moon in view > 0; %s" % [gain,
			AWAY_NIGHT_MIN_GAIN, _num(cone_gain), _num(reference / _atmosphere.adaptation_cd_m2), _adaptation_parts()])
	# The Earth term at lunar distance: a Lambert sphere's disc-mean plate equivalent at phase angle a is
	# (2/3) x (sin a + (pi - a) cos a) / pi of the full-Sun plate, x the view factor (R / r)^2.
	var radius := float(_atmosphere.parameters["bottom_radius_m"])
	var r := Atmosphere._stretch(eye).length()
	var lambert := 2.0 / 3.0 * (sin(phase) + (PI - phase) * cos(phase)) / PI * reference * (radius / r) * (radius / r)
	var ratio := _atmosphere.last_adaptation_up_cd_m2 / lambert
	_check("adapted, lunar night: Earth in view", absf(ratio - 1.0) <= AWAY_EARTH_TOLERANCE,
			"%s cd/m^2, Lambert disc %s, ratio %.3f (needs 1 +- %.2f); view factor %s" % [
			_num(_atmosphere.last_adaptation_up_cd_m2), _num(lambert), ratio, AWAY_EARTH_TOLERANCE,
			_num((radius / r) * (radius / r))])
	# Deep space halfway to the Moon, the Sun perpendicular to the Earth-Moon line.
	_atmosphere.snap_adaptation()
	await _render(moon * 0.5, -moon_dir, side)
	_check("adapted, deep space halfway to the Moon", absf(_atmosphere.exposure_gain - 1.0) <= AWAY_GAIN_TOLERANCE,
			"gain %.4f (needs 1 +- %.2f); %s" % [_atmosphere.exposure_gain, AWAY_GAIN_TOLERANCE, _adaptation_parts()])
	_atmosphere.eye_adaptation_enabled = false
	# Earthshine (fix 2026-10-25): the full Earth's light at the Moon in lux, and the Moon shader's dark side at two
	# phases of the Earth against albedo x moon_earthshine.
	var toa_lux := float(_atmosphere.parameters["toa_solar_illuminance_lux"])
	var full_lux := Atmosphere.earthshine_relative(moon, moon_dir * AU_M) * toa_lux
	_check("earthshine at full Earth", full_lux >= EARTHSHINE_FULL_MIN_LUX and full_lux <= EARTHSHINE_FULL_MAX_LUX,
			"%.2f lux (needs %.0f .. %.0f)" % [full_lux, EARTHSHINE_FULL_MIN_LUX, EARTHSHINE_FULL_MAX_LUX])
	await _earthshine_sample("earthshine on the Moon, full Earth", moon_dir, side, 0.0)
	await _earthshine_sample("earthshine on the Moon, Earth phase 60", moon_dir, side, 60.0)
	# The vehicles' Sun light (world._update_sun_light) on lunar terrain below the mean sphere at lunar day: the core's
	# mean-sphere fraction hides the Sun there, Atmosphere.moon_visible_fraction must not.
	var low := moon - moon_dir * (Atmosphere.MOON_RADIUS_M - LOW_SITE_DEPTH_M)
	var sun_m := sun_day * AU_M
	var mean_visible := 1.0 - Atmosphere.occulted_fraction(sun_m - low, Atmosphere.SUN_RADIUS_M, moon - low,
			Atmosphere.MOON_RADIUS_M)
	var visible := Atmosphere.moon_visible_fraction(low, sun_m, moon, mean_visible)
	var lit: float = _atmosphere.sun_light_at(low, sun_day, visible)[1]
	var open_sky: float = _atmosphere.sun_light_at(low, sun_day, 1.0)[1]
	_check("vehicle Sun light, lunar day, low site", mean_visible < 0.5 and lit > 0.0 and is_equal_approx(lit, open_sky),
			"energy %s of the unobstructed %s (mean-sphere visible fraction %.3f, %.0f m below the mean radius)" % [
			_num(lit), _num(open_sky), mean_visible, LOW_SITE_DEPTH_M])
	_moon_center_gcrf = Vector3.ZERO


## The vehicles' direct Sun on the lunar ground from the terrain's horizon (MoonSunHorizon): a point 2 m above the
## ground at the IM-2 site, the Sun TERRAIN_SUN_MARGIN_DEG below / above / at the horizon in a sector's azimuth, and
## halfway between two sectors (the horizon interpolated like terrain_moon.gdshader). Moon-fixed frame, the Sun at 1 AU.
## Heights, pre-check, eclipse, orbit: tests/moon_sun_test.gd (headless).
func _moon_terrain_sun_samples() -> void:
	var present: Dictionary = _core.load_terrain(Paths.config_dir())
	if not present.get("moon", false):
		_check("terrain Sun, IM-2 site", true, "skipped: no lunar terrain data")
		return
	var lat := deg_to_rad(IM2_LAT_DEG)
	var lon := deg_to_rad(IM2_LON_DEG)
	var up := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var east := Vector3(-sin(lon), cos(lon), 0.0)
	var north := up.cross(east)
	var ground: Dictionary = _core.surface_samples("moon", PackedFloat64Array([IM2_LAT_DEG]),
			PackedFloat64Array([IM2_LON_DEG]))
	var height_m := float(ground["height_m"][0]) + 2.0
	var point := up * (Atmosphere.MOON_RADIUS_M + height_m)
	var started := Time.get_ticks_usec()
	var horizon_0: float = _core.horizon_elevation_rad("moon", IM2_LAT_DEG, IM2_LON_DEG, height_m, 0.0)
	var horizon_ms := (Time.get_ticks_usec() - started) / 1000.0
	var horizon_1: float = _core.horizon_elevation_rad("moon", IM2_LAT_DEG, IM2_LON_DEG, height_m, TAU / 16.0)
	var margin := deg_to_rad(TERRAIN_SUN_MARGIN_DEG)
	# Sector 0 (north): the fraction against the horizon there.
	var below := _terrain_sun_fraction(point, up, north, horizon_0 - margin)
	var above := _terrain_sun_fraction(point, up, north, horizon_0 + margin)
	var at_horizon := _terrain_sun_fraction(point, up, north, horizon_0)
	_check("terrain Sun, IM-2 site", below == 0.0 and above == 1.0 and absf(at_horizon - 0.5) <= 0.05,
			"horizon %.3f deg north: %.0f deg below %.3f, above %.3f, at the horizon %.3f (%.2f ms per march)" % [
			rad_to_deg(horizon_0), TERRAIN_SUN_MARGIN_DEG, below, above, at_horizon, horizon_ms])
	# Halfway between sector 0 and 1: the mean of the two horizons.
	var between := cos(TAU / 32.0) * north + sin(TAU / 32.0) * east
	var halfway := _terrain_sun_fraction(point, up, between, 0.5 * (horizon_0 + horizon_1))
	_check("terrain Sun, between two sectors", absf(halfway - 0.5) <= 0.05, "horizons %.3f / %.3f deg, fraction %.3f" % [
			rad_to_deg(horizon_0), rad_to_deg(horizon_1), halfway])


## The terrain Sun fraction at `point` for the Sun at 1 AU in the horizontal direction `toward` raised to
## elevation_rad above the local horizontal.
func _terrain_sun_fraction(point: Vector3, up: Vector3, toward: Vector3, elevation_rad: float) -> float:
	var direction := cos(elevation_rad) * toward + sin(elevation_rad) * up
	return MoonSunHorizon.sun_visible_fraction(_core, point, point + direction * AU_M)


## IM-2 site (smoke_test.gd's horizon checks), the Sun's margin about the terrain horizon, the orbit's height.
const IM2_LAT_DEG := -84.79
const IM2_LON_DEG := 29.19
const TERRAIN_SUN_MARGIN_DEG := 1.0
const ORBIT_HEIGHT_M := 100000.0


## Earthshine: the full Earth's light at the Moon (Atmosphere.earthshine_relative x the TOA illuminance) within this
## band [call]: the Lambert sphere of the Bond albedo gives 7.2 lux at 384 400 km; measured estimates run ~7-15 lux
## ("about 10 lux"); the band catches a unit or phase error (x pi, x 1/pi, a lost (R/d)^2), not the albedo choice.
const EARTHSHINE_FULL_MIN_LUX := 5.0
const EARTHSHINE_FULL_MAX_LUX := 15.0
## The earthshine-lit pixel against albedo x moon_earthshine (half-float viewport near 1e-5: subnormal steps ~0.3 %).
const EARTHSHINE_TOLERANCE := 0.03
## The terrain at 0 N 45 E lies 1551 m below the Moon's mean radius (render_check's lunar ground view).
const LOW_SITE_DEPTH_M := 1551.0


## Section 12: the Moon (moon.gdshader, the SURFACE_ALBEDO_8 texture) seen from 50 000 km on the Earth-Moon line, its
## sub-Earth point on the camera axis; the Sun phase_deg from the Moon's direction seen from the Earth (the Earth's phase
## angle at the Moon; 0: full Earth) on the far side, so the pixel is unlit by the Sun. Fixed exposure (gain 1), outside
## the atmosphere (T 1, no sky): the pixel must be albedo x moon_earthshine (x cos 1).
func _earthshine_sample(label: String, moon_dir: Vector3, side: Vector3, phase_deg: float) -> void:
	var phase := deg_to_rad(phase_deg)
	var sun_itrf := (moon_dir * cos(phase) + side * sin(phase)).normalized()
	var moon_itrf := moon_dir * MOON_DISTANCE_M
	var camera_itrf := moon_itrf - moon_dir * EARTHSHINE_VIEW_DISTANCE_M
	var camera_gcrf := _earth_to_gcrf * camera_itrf
	var moon_gcrf := _earth_to_gcrf * moon_itrf
	var sun_gcrf := _earth_to_gcrf * sun_itrf * AU_M
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/moon.gdshader")
	material.set_shader_parameter("albedo_texture", _constant_texture(SURFACE_ALBEDO_8))
	material.set_shader_parameter("sun_position_local", Frames.to_render(sun_gcrf - moon_gcrf))
	material.set_shader_parameter("earth_position_local", Frames.to_render(-moon_gcrf))
	material.set_shader_parameter("camera_position_local", Frames.to_render(camera_gcrf - moon_gcrf))
	var body := CelestialBody.new()
	body.setup(Atmosphere.MOON_RADIUS_M, 0.0, material)
	body.fixed_to_gcrf = Basis.IDENTITY  # Moon-local = GCRF in render axes (the uniforms above)
	body.center_gcrf_m = moon_gcrf
	body.place(camera_gcrf)
	_viewport.add_child(body)
	_moon_center_gcrf = moon_gcrf  # _render's update sets moon_earthshine from it
	var got := await _render(camera_itrf, moon_dir, sun_itrf)
	body.queue_free()
	var scene := _atmosphere.earthshine_scene  # what the update wrote into moon_earthshine
	var albedo := Vector3(SURFACE_ALBEDO_8.srgb_to_linear().r, SURFACE_ALBEDO_8.srgb_to_linear().g,
			SURFACE_ALBEDO_8.srgb_to_linear().b)
	var expected_relative := (2.0 / 3.0 * 0.306 * pow(6371000.0 / MOON_DISTANCE_M, 2.0)
			* (sin(phase) + (PI - phase) * cos(phase)) / PI)
	_check(label + " (scene value)", absf(scene / (Atmosphere.DAY_EXPOSURE * expected_relative) - 1.0) < 1.0e-6,
			"moon_earthshine %s, expected %s (%.2f lux)" % [_num(scene), _num(Atmosphere.DAY_EXPOSURE
			* expected_relative), _atmosphere.last_earthshine_lux])
	# The regolith reflectance at the sub-Earth point seen from the Earth's side: mu0 = mu = 1, phase 0 (the opposition
	# surge), x the regolith scale (night-vision fix round I3: the earthshine through the sunlight's BRDF).
	var regolith := Atmosphere.regolith_scale(float(_atmosphere.parameters["toa_solar_illuminance_lux"])) \
			* Atmosphere.regolith_reflectance(Vector3.UP, Vector3.UP, Vector3.UP)
	_compare(label + " [regolith %.4f]" % regolith, albedo * scene * regolith, got, EARTHSHINE_TOLERANCE)


const EARTHSHINE_VIEW_DISTANCE_M := 5.0e7


## The eye adaptation's parts at the last update, for the section-12 lines.
func _adaptation_parts() -> String:
	return "adapted to %s cd/m^2: Sun on a plate %s (visible %.3f), Earth in view %s, Moon in view %s, earthshine %s lux" % [
			_num(_atmosphere.adaptation_cd_m2), _num(_atmosphere.last_adaptation_down_cd_m2),
			_atmosphere.last_adaptation_visible_fraction, _num(_atmosphere.last_adaptation_up_cd_m2),
			_num(_atmosphere.last_adaptation_moon_cd_m2), _num(_atmosphere.last_earthshine_lux)]


## Eye adaptation: the adapted zenith at Sun -3 deg must exceed this scene luminance - ~7 % of the noon zenith at the
## day exposure (0.13), well above black through the AgX tone mapper - and the adapted zenith at Sun -12 deg (night
## vision, 2026-10-25: rod vision, no longer black) stay below this fraction of the Sun -3 deg zenith (dimmer).
const ADAPTED_TWILIGHT_MIN := 0.01
const ADAPTED_NIGHT_MAX := 0.5
var _last_gain := 1.0
var _last_scene := Vector3.ZERO  ## the last adapted sample's CPU sky as a scene value, before the post step


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
	# The CPU sky through the display: the luminance scale (cone gain x brightness) and the post step (chroma, rods).
	_last_scene = cpu["luminance"] * _atmosphere.luminance_scale
	var expected := _post(_last_scene)
	if compare:
		_compare(label + " [gain %.2f, B %.3f, S %.3f]" % [_last_gain, _atmosphere.brightness, _atmosphere.chroma],
				expected, got)
	else:
		print("atmosphere_check: %-38s CPU %s got %s (gain %.2f, B %.3f, S %.3f)" % [label, _vec(expected), _vec(got),
				_last_gain, _atmosphere.brightness, _atmosphere.chroma])
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
## floor_share: channels below this share of the peak are compared absolutely, at tolerance x that level (1: every
## channel against tolerance x the peak).
func _compare(label: String, expected: Vector3, got: Vector3, tolerance := TOLERANCE, floor_share := RELATIVE_FLOOR) -> void:
	_samples += 1
	var peak := maxf(expected.x, maxf(expected.y, expected.z))
	var ok := peak > 0.0
	var ratios := PackedStringArray()
	for k in range(3):
		var e := expected[k]
		var g := got[k]
		var ratio := g / e if e > 0.0 else INF
		ratios.append("%.4f" % ratio)
		var floor_value := peak * floor_share
		if e >= floor_value:
			ok = ok and absf(ratio - 1.0) <= tolerance
		else:
			ok = ok and absf(g - e) <= tolerance * floor_value
	if not ok:
		_failures += 1
	print("atmosphere_check: %-38s expected %s got %s ratio (%s) %s" % [label, _vec(expected), _vec(got),
			", ".join(ratios), "ok" if ok else "FAIL"])


## A ground ray: the sky shader shows the model ground (the tables' ground_albedo without a day texture) lit by the Sun and
## the sky, seen through the atmosphere. CPU: the model-ground hit in the stretched space (float64), then
## albedo x (sun + sky lux) / pi x T + in-scattered luminance, all x luminance_scale.
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
	var light: Vector3 = (lux["sun_lux"] + lux["sky_lux"]) * (float(_atmosphere.parameters["ground_albedo"]) / PI)
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
	var visible := 1.0 if is_nan(_tdb) else 1.0 - _core.solar_eclipse_fraction_gcrf(camera_gcrf, _tdb)
	if _moon_center_gcrf != Vector3.ZERO:  # section 12: the Moon's disc in front of the Sun (world.gd: the core's)
		visible = 1.0 - Atmosphere.occulted_fraction(sun_gcrf - camera_gcrf, Atmosphere.SUN_RADIUS_M,
				_moon_center_gcrf - camera_gcrf, Atmosphere.MOON_RADIUS_M)
	_atmosphere.update(camera_gcrf, Vector3.ZERO, _earth_to_gcrf, sun_gcrf, 0.0, visible, _moon_gcrf, _tdb,
			_moon_center_gcrf)
	if _force_segments >= 0:
		ShaderGlobals.set_value(&"atmosphere_eclipse_segments", _force_segments)
	if not is_nan(altitude_override):
		ShaderGlobals.set_value(&"atmosphere_camera_altitude_m", altitude_override)
	var view_render := Frames.to_render(_earth_to_gcrf * view_itrf).normalized()
	var up := Vector3.UP if absf(view_render.y) < 0.9 else Vector3.RIGHT
	_camera.transform = Transform3D(Basis.looking_at(view_render, up), Vector3.ZERO)
	for i in range(SETTLE_FRAMES):
		await RenderingServer.frame_post_draw
	var image := _viewport.get_texture().get_image()
	_last_image = image
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


## Section 13 (night vision, user decision 2026-10-25; see _run). Bounds [call], scene units (the HDR viewport before
## the tone mapper):
## - twilight at Sun -8 deg: the adapted zenith's blue channel above NV_TWILIGHT_MIN (~sRGB 17 in blue through a
##   linear tone mapper; the zenith is a deep blue: the eye adapts to the brighter glow over the western horizon) and
##   bluish (blue > NV_BLUISH x red). Was 0.01 before the brightness term (2026-10-25: B = 0.368 below the Weber range;
##   the zenith's blue went from 0.018 to 0.0065);
## - a moonless night (Sun -30 deg, no Moon): a starlit ground (_moonless_ground_sample) as the model renders it and
##   below the display threshold;
## - the full-Moon ground: above NV_MOONLIT_MIN (AgX: ~sRGB 8; the Cape's 2 km view renders ~sRGB 20-60), its cone
##   image (the post step undone) the CPU's moonlit surface (albedo x the Moon's direct and sky light x its relative
##   illuminance, through the view's transmittance; SURFACE_TOLERANCE) and the rods more than NV_ROD_SHARE of its
##   luminance (desaturated);
## - the earthshine on a 2-day crescent: the dark side's luminance above the sky beside the limb by more than the cones'
##   Weber fraction 10^-1.255 = 5.6 % (Ferwerda et al. 1996 Eq. 4: visible);
## - day frames: the adapted noon equals the fixed exposure x the gain (NV_DAY_TOLERANCE: half-float rounding), no rods,
##   no moonlight.
const NV_TWILIGHT_MIN := 0.004
const NV_BLUISH := 1.2
const NV_MOONLIT_MIN := 0.003
const NV_ROD_SHARE := 0.5
const NV_EARTHSHINE_CONTRAST := 0.056
const NV_DAY_TOLERANCE := 0.002
const NV_CRESCENT_UTC := "2026-11-12T00:15:00"
const NV_CRESCENT_FOV_DEG := 1.0
## The full-Moon geometry: the Moon this high in the south of the Cape, the Sun this far from opposite (phase angle).
const NV_MOON_ELEVATION_DEG := 70.0
const NV_MOON_PHASE_DEG := 10.0


func _night_vision_samples() -> void:
	var pad := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 2.0)
	var zenith := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 0.0, 90.0)
	var sun_60 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 180.0, 60.0)
	# Day frames unchanged: the adapted noon zenith and the 400 km limb = the fixed exposure x the cone gain.
	var orbit := _geodetic(10.0, -60.0, 400000.0)
	for day: Array in [["noon zenith", pad, zenith, sun_60], ["400 km limb", orbit, _limb_direction(orbit,
			Vector3(0, 0, 1), 10000.0), _local_direction(10.0, -60.0, 90.0, 30.0)]]:
		_atmosphere.eye_adaptation_enabled = false
		var fixed := await _render(day[1], day[2], day[3])
		_atmosphere.eye_adaptation_enabled = true
		_atmosphere.snap_adaptation()
		var adapted := await _render(day[1], day[2], day[3])
		var gain := _atmosphere.exposure_gain
		_compare("night vision: day %s = fixed x gain %.3f" % [day[0], gain], fixed * gain, adapted, NV_DAY_TOLERANCE)
		_check("night vision: day %s, no rods, no moonlight, B = S = 1" % day[0], _atmosphere.rod_scale == 0.0
				and _atmosphere.brightness == 1.0 and _atmosphere.chroma == 1.0 and not _atmosphere.moonlight_active,
				"rod scale %s, moonlight %s, brightness %s, chroma %s" % [_num(_atmosphere.rod_scale),
				_atmosphere.moonlight_active, _num(_atmosphere.brightness), _num(_atmosphere.chroma)])
		# The day look unchanged (night-vision review I7): the gain is the former Weber gain L_ref / L_a (at least 1)
		# of the unpaced target.
		var weber := maxf(_atmosphere.reference_luminance(1.0) / _atmosphere.last_target_cd_m2, 1.0)
		_check("night vision: day %s gain = Weber gain" % day[0], absf(gain / weber - 1.0) < 1.0e-9,
				"gain %.9f, L_ref / L_a %.9f (L_a %s cd/m^2)" % [gain, weber, _num(_atmosphere.last_target_cd_m2)])
	await _display_referred_samples(pad, zenith, sun_60)
	# Twilight at Sun -8 deg: the zenith visible and bluish.
	var sun_m8 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -8.0)
	_atmosphere.snap_adaptation()
	var twilight := await _render(pad, zenith, sun_m8)
	var twilight_y := twilight.dot(LUMINANCE_WEIGHTS)
	_check("night vision: Sun -8 deg zenith blue", twilight.z > NV_TWILIGHT_MIN and twilight.z > NV_BLUISH * twilight.x,
			"%s luminance %s, blue %s (needs > %s), blue / red %.2f (needs > %.1f); %s" % [_vec(twilight),
			_num(twilight_y), _num(twilight.z), _num(NV_TWILIGHT_MIN), twilight.z / maxf(twilight.x, 1.0e-30), NV_BLUISH,
			_vision_parts()])
	# A moonless night: Sun -30 deg, no Moon, the horizon in the middle of the frame.
	var sun_m30 := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -30.0)
	_atmosphere.snap_adaptation()
	await _moonless_ground_sample(pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 90.0, 0.0), sun_m30)
	await _night_star("night vision: moonless, magnitude-6 star", pad, _local_direction(PAD_LAT_DEG, PAD_LON_DEG,
			90.0, 45.0), sun_m30, 6.0, true)
	# The full-Moon night at the Cape: the ground from 2 km, the stars beside the Moon.
	var cape := _geodetic(CAPE_LAT_DEG, CAPE_LON_DEG, 2000.0)
	var up := _geodetic_up(cape)
	var north := _local_direction(CAPE_LAT_DEG, CAPE_LON_DEG, 0.0, 0.0)
	var east := _local_direction(CAPE_LAT_DEG, CAPE_LON_DEG, 90.0, 0.0)
	var elevation := deg_to_rad(NV_MOON_ELEVATION_DEG)
	var moon_dir := (up * sin(elevation) - north * cos(elevation)).normalized()
	var phase := deg_to_rad(NV_MOON_PHASE_DEG)
	var sun_full := (-moon_dir * cos(phase) + east * sin(phase)).normalized()
	_moon_center_gcrf = _earth_to_gcrf * (moon_dir * MOON_DISTANCE_M)
	var point := _geodetic(CAPE_LAT_DEG, CAPE_LON_DEG - 5000.0 / (111319.49 * cos(deg_to_rad(CAPE_LAT_DEG))), 0.0)
	_atmosphere.snap_adaptation()
	var ground := await _render_surface(cape, point, _geodetic_up(point), sun_full, true)
	var cones := _undo_post(ground)  # the post step undone: the cones' image (its chroma before the Hunt factor)
	var linear := SURFACE_ALBEDO_8.srgb_to_linear()
	var albedo := Vector3(linear.r, linear.g, linear.b)
	var normal := _geodetic_up(point)
	var moon_lux: Dictionary = _core.atmosphere_illuminance(point, normal, moon_dir)
	var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(cape, point, sun_full)
	var moon_light: Vector3 = (moon_lux["sun_lux"] + moon_lux["sky_lux"]) / PI * _atmosphere.moonlight_relative_value
	var view_t: Vector3 = to_point["transmittance"]
	var expected_cones: Vector3 = (albedo * moon_light * view_t + to_point["luminance"]) * _atmosphere.luminance_scale
	_compare("night vision: full-Moon ground, cone image", expected_cones, cones, SURFACE_TOLERANCE)
	var ground_y := ground.dot(LUMINANCE_WEIGHTS)
	var rod_share := 1.0 - cones.dot(LUMINANCE_WEIGHTS) / maxf(ground_y, 1.0e-30)
	_check("night vision: full-Moon ground", ground_y > NV_MOONLIT_MIN and rod_share > NV_ROD_SHARE,
			"%s luminance %s (needs > %s), rod share %.2f (needs > %.1f); moonlight %s of the Sun; %s" % [
			_vec(ground), _num(ground_y), _num(NV_MOONLIT_MIN), rod_share, NV_ROD_SHARE,
			_num(_atmosphere.moonlight_relative_value), _vision_parts()])
	var away := (up * cos(deg_to_rad(30.0)) + north * sin(deg_to_rad(30.0))).normalized()  # ~60 deg from the Moon
	await _night_star("night vision: full Moon, magnitude-2 star", cape, away, sun_full, 2.0, true)
	await _night_star("night vision: full Moon, magnitude-5 star", cape, away, sun_full, 5.0, false)
	_moon_center_gcrf = Vector3.ZERO
	# The earthshine on a 2-day crescent from Starbase at Sun -8 deg (true geometry).
	await _crescent_sample()
	_atmosphere.eye_adaptation_enabled = false


## Night-vision fix round I7: a moonless night's ground. No airglow or starlight is modelled (the sky is exactly 0),
## so the sample is a scene-referred quad at a starlit ground's luminance: the moonless clear sky's illuminance
## NV_STARLIT_LUX [S: P. Schlyter, "Radiometry and photometry in astronomy", the illuminance table: moonless clear
## night sky (starlight + airglow) 0.002 lux] on a grey ground of NV_STARLIT_ALBEDO [E] (Lambert). The pixel must be the
## model's (the cone image x the gain plus the rods, SURFACE_TOLERANCE), and - its scotopic luminance below the rods'
## absolute threshold (Ferwerda Eq. 5) - below the display observer's threshold DAY_EXPOSURE x 10^-1.255 (dark).
const NV_STARLIT_LUX := 0.002
const NV_STARLIT_ALBEDO := 0.2
## The display-referred quads (review I7): their pixel at night against the day's, the rod scale at least
## NV_DISPLAY_MIN_ROD_SCALE (the moonless night), the blended sample against its formula; every channel within
## NV_DISPLAY_TOLERANCE of the colour's peak. The limit is the rgba16f colour buffer: the inverse writes channels of
## ~0.5 whose scotopic sum cancels to ~1e-3 (w . c' = w . c / (1 + s w . tint)), and the post step multiplies that sum's
## half-float rounding (~2^-11 x 0.5 per channel) by s x tint ~ 140 x 2 - up to ~1-2 % of the peak at s = 138 (measured
## 0.1 % opaque, 1.4 % blended).
const NV_DISPLAY_TOLERANCE := 0.02
const NV_DISPLAY_MIN_ROD_SCALE := 100.0
const NV_DISPLAY_COLOUR := Color(1.0, 0.5, 0.1, 1.0)  ## an orange plume-like marker (sRGB)
const NV_DISPLAY_BACKGROUND_CD_M2 := 0.002  ## the blended sample's scene-referred background
const NV_DISPLAY_ALPHA := 0.5
const FLAT_LUMINANCE_SHADER := """shader_type spatial;
render_mode unshaded, cull_disabled, shadows_disabled, fog_disabled;
global uniform float atmosphere_scene_per_cd_m2;
uniform vec3 luminance_cd_m2;
void fragment() {
	ALBEDO = luminance_cd_m2 * atmosphere_scene_per_cd_m2;
}
"""


## A quad filling the view (fov 10 deg) at distance_m on the camera axis with the material.
func _view_quad(view_itrf: Vector3, distance_m: float, material: Material) -> MeshInstance3D:
	var view_render := Frames.to_render(_earth_to_gcrf * view_itrf).normalized()
	var up := Vector3.UP if absf(view_render.y) < 0.9 else Vector3.RIGHT
	var quad := MeshInstance3D.new()
	var mesh := QuadMesh.new()
	mesh.size = Vector2(distance_m, distance_m)
	quad.mesh = mesh
	quad.material_override = material
	quad.transform = Transform3D(Basis.looking_at(view_render, up), view_render * distance_m)
	_viewport.add_child(quad)
	return quad


## A scene-referred quad of this true luminance (cd/m^2, grey) through the eye model like the terrain.
func _luminance_quad(view_itrf: Vector3, distance_m: float, luminance_cd_m2: float) -> MeshInstance3D:
	var shader := Shader.new()
	shader.code = FLAT_LUMINANCE_SHADER
	var material := ShaderMaterial.new()
	material.shader = shader
	material.set_shader_parameter("luminance_cd_m2", Vector3.ONE * luminance_cd_m2)
	return _view_quad(view_itrf, distance_m, material)


## The pixel the eye model's post step makes of a scene value (VisionEffect: the cones' chroma, then the rods).
func _post(scene: Vector3) -> Vector3:
	return EyeAdaptation.post_step(scene, _atmosphere.rod_scale, _atmosphere.chroma)


## The scene value the post step turned into this pixel (its linear map inverted).
func _undo_post(pixel: Vector3) -> Vector3:
	var map := Basis(_post(Vector3(1, 0, 0)), _post(Vector3(0, 1, 0)), _post(Vector3(0, 0, 1)))
	return map.inverse() * pixel


## A pixel's chroma: its distance from the grey of its luminance, relative to that luminance.
static func _chroma_of(c: Vector3) -> float:
	var y := c.dot(LUMINANCE_WEIGHTS)
	return (c - Vector3.ONE * y).length() / maxf(y, 1.0e-30)


## Section 13: the moonless night's ground (see NV_STARLIT_LUX); the eye adaptation on and snapped by the caller.
func _moonless_ground_sample(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var luminance := NV_STARLIT_LUX * NV_STARLIT_ALBEDO / PI
	var quad := _luminance_quad(view_itrf, 10.0, luminance)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	quad.queue_free()
	var expected := _post(Vector3.ONE * luminance * _atmosphere.scene_per_cd_m2)
	_compare("night vision: moonless ground = model", expected, got, SURFACE_TOLERANCE)
	var scotopic := luminance * EyeAdaptation.SCOTOPIC_WEIGHTS.dot(Vector3.ONE)
	var rod_threshold := pow(10.0, EyeAdaptation.rod_log_threshold(_atmosphere.scotopic_cd_m2))
	var display_threshold := Atmosphere.DAY_EXPOSURE * pow(10.0, -EyeAdaptation.CONE_WEBER_OFFSET)
	var got_y := got.dot(LUMINANCE_WEIGHTS)
	_check("night vision: moonless night dark", scotopic < rod_threshold and got_y > 0.0 and got_y < display_threshold,
			"starlit ground %s cd/m^2 (scotopic %s below the rods' threshold %s): pixel %s luminance %s (needs 0 .. %s, the display threshold); %s" % [
			_num(luminance), _num(scotopic), _num(rod_threshold), _vec(got), _num(got_y), _num(display_threshold),
			_vision_parts()])


## Section 13 (review I7): display-referred emitters (display_colour.gdshader, as orbit lines, labels, plume, stars use
## vision.gdshaderinc) look the same by day and at night: an opaque quad's pixel at the moonless night (rod scale >=
## NV_DISPLAY_MIN_ROD_SCALE) equals its day pixel, and a blended one over a scene-referred background is
## alpha x the colour + (1 - alpha) x the background with its rods.
func _display_referred_samples(pad: Vector3, view_itrf: Vector3, sun_day: Vector3) -> void:
	var sun_night := _local_direction(PAD_LAT_DEG, PAD_LON_DEG, 270.0, -30.0)
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/display_colour.gdshader")
	material.set_shader_parameter("colour", NV_DISPLAY_COLOUR)
	var quad := _view_quad(view_itrf, 10.0, material)
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	var day := await _render(pad, view_itrf, sun_day)
	_atmosphere.snap_adaptation()
	var night := await _render(pad, view_itrf, sun_night)
	var night_rods := _atmosphere.rod_scale
	_compare("night vision: display colour night = day [rods %.0f]" % night_rods, day, night, NV_DISPLAY_TOLERANCE, 1.0)
	_check("night vision: display sample at full rods", night_rods >= NV_DISPLAY_MIN_ROD_SCALE,
			"rod scale %s (needs >= %.0f); day pixel %s, the colour %s" % [_num(night_rods), NV_DISPLAY_MIN_ROD_SCALE,
			_vec(day), _vec(_linear(NV_DISPLAY_COLOUR))])
	var blended := NV_DISPLAY_COLOUR
	blended.a = NV_DISPLAY_ALPHA
	material.set_shader_parameter("colour", blended)
	var background := _luminance_quad(view_itrf, 20.0, NV_DISPLAY_BACKGROUND_CD_M2)
	var got := await _render(pad, view_itrf, sun_night)
	quad.queue_free()
	background.queue_free()
	var behind := _post(Vector3.ONE * NV_DISPLAY_BACKGROUND_CD_M2 * _atmosphere.scene_per_cd_m2)
	var expected := _linear(NV_DISPLAY_COLOUR) * NV_DISPLAY_ALPHA + behind * (1.0 - NV_DISPLAY_ALPHA)
	_compare("night vision: display colour blended over rods", expected, got, NV_DISPLAY_TOLERANCE, 1.0)


static func _linear(colour: Color) -> Vector3:
	var c := colour.srgb_to_linear()
	return Vector3(c.r, c.g, c.b)


## The eye adaptation's night-vision parts at the last update (section 13 lines).
func _vision_parts() -> String:
	return "gain %s, adapted to %s cd/m^2 (glow %s), scotopic %s, rod scale %s" % [_num(_atmosphere.exposure_gain),
			_num(_atmosphere.adaptation_cd_m2), _num(_atmosphere.last_adaptation_glow_cd_m2),
			_num(_atmosphere.scotopic_cd_m2), _num(_atmosphere.rod_scale)]


## A star of `magnitude` on the camera axis for the adapted eye (snapped), against the same render without it: visible
## (luminance ratio > STAR_VISIBLE_FACTOR) or hidden (= the sky, 2 %).
func _night_star(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, magnitude: float,
		visible: bool) -> void:
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	var sky := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var stars := StarField.new()
	stars.build(PackedVector3Array([_earth_to_gcrf * view_itrf]), PackedFloat32Array([magnitude]),
			PackedFloat32Array([STAR_COLOR_INDEX]))
	_viewport.add_child(stars)
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	stars.queue_free()
	if visible:
		var ratio := got.dot(LUMINANCE_WEIGHTS) / maxf(sky.dot(LUMINANCE_WEIGHTS), 1.0e-30)
		_check(label + " visible", ratio > STAR_VISIBLE_FACTOR, "sky %s got %s ratio %s (needs > %.0f), adapted to %s cd/m^2"
				% [_vec(sky), _vec(got), _num(ratio), STAR_VISIBLE_FACTOR, _num(_atmosphere.adaptation_cd_m2)])
	else:
		_compare(label + " hidden = sky", sky, got)


## Section 13: the 2-day crescent of 2026-11-12 00:15 UTC from Starbase (Sun -8 deg), the Moon a moon.gdshader body at
## its true apparent place (grey albedo MOON_MEAN_TEXTURE_ALBEDO), the eye adapted: the pixel half a lunar radius into
## the dark side against the sky two radii beyond the dark limb, both on the line away from the lit limb.
func _crescent_sample() -> void:
	var saved_rotation := _earth_to_gcrf
	var t := _core.utc_to_tdb_s(NV_CRESCENT_UTC)
	_earth_to_gcrf = _core.earth_fixed_to_gcrf(t)
	var sun_itrf := _set_true_sun_and_moon(t)  # _sun_gcrf, _moon_gcrf: the apparent Sun and Moon
	var camera := _geodetic(PAD_LAT_DEG, PAD_LON_DEG, 2.0)
	var camera_gcrf := _earth_to_gcrf * camera
	var moon_geometric := _core.body_position_gcrf_m("moon", t)
	var moon_gcrf := _moon_gcrf  # apparent from the Earth's centre (light time, aberration), as World draws it
	_moon_center_gcrf = moon_geometric
	var albedo := Color(Atmosphere.MOON_MEAN_TEXTURE_ALBEDO, Atmosphere.MOON_MEAN_TEXTURE_ALBEDO,
			Atmosphere.MOON_MEAN_TEXTURE_ALBEDO).linear_to_srgb()
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/moon.gdshader")
	material.set_shader_parameter("albedo_texture", _constant_texture(albedo))
	material.set_shader_parameter("earth_radius_m", 6371000.0)
	material.set_shader_parameter("sun_position_local", Frames.to_render(_sun_gcrf - moon_gcrf))
	material.set_shader_parameter("earth_position_local", Frames.to_render(-moon_gcrf))
	material.set_shader_parameter("camera_position_local", Frames.to_render(camera_gcrf - moon_gcrf))
	var moon := CelestialBody.new()
	moon.setup(Atmosphere.MOON_RADIUS_M, 0.0, material)
	moon.fixed_to_gcrf = Basis.IDENTITY
	moon.center_gcrf_m = moon_gcrf
	moon.place(camera_gcrf)
	_viewport.add_child(moon)
	var to_moon := moon_gcrf - camera_gcrf
	var distance := to_moon.length()
	var axis := to_moon / distance
	var to_sun := (_sun_gcrf - moon_gcrf).normalized()
	var dark := -(to_sun - axis * to_sun.dot(axis)).normalized()  # across the disc, away from the lit limb
	var radius_angle := Atmosphere.MOON_RADIUS_M / distance
	var gcrf_to_itrf := _earth_to_gcrf.transposed()
	var dark_view := gcrf_to_itrf * (axis + dark * 0.5 * radius_angle).normalized()
	var sky_view := gcrf_to_itrf * (axis + dark * 3.0 * radius_angle).normalized()
	_camera.fov = NV_CRESCENT_FOV_DEG
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.snap_adaptation()
	var dark_side := await _render(camera, dark_view, sun_itrf)
	var sky := await _render(camera, sky_view, sun_itrf)
	_camera.fov = 10.0
	moon.queue_free()
	var contrast := dark_side.dot(LUMINANCE_WEIGHTS) / maxf(sky.dot(LUMINANCE_WEIGHTS), 1.0e-30) - 1.0
	_check("night vision: earthshine on the crescent", contrast > NV_EARTHSHINE_CONTRAST,
			"dark side %s, sky beside it %s, contrast %.3f (needs > %.3f); earthshine %s lux; %s" % [_vec(dark_side),
			_vec(sky), contrast, NV_EARTHSHINE_CONTRAST, _num(_atmosphere.last_earthshine_lux), _vision_parts()])
	_moon_center_gcrf = Vector3.ZERO
	_moon_gcrf = Vector3.ZERO
	_earth_to_gcrf = saved_rotation
