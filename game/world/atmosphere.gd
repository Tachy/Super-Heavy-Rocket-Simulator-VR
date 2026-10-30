class_name Atmosphere
extends RefCounted
## The Earth's precomputed atmosphere on the GPU (M8 atmosphere): loads the core's tables (StarsimCore.load_atmosphere:
## the cache in data/atmosphere, or a first-run precomputation) on a WorkerThreadPool task, uploads them as float
## textures, sets the global shader uniforms of shaders/atmosphere.gdshaderinc and updates the per-frame ones.
## No physics here: the tables and every evaluation come from the core; this only converts frames and units.
##
## Scene units (one exposure, spec "One exposure for everything"): scene value = luminance (cd/m^2) x DAY_EXPOSURE x
## pi / E_v,TOA x 1 / d_AU^2 - a white Lambertian surface under the top-of-atmosphere Sun at 1 AU renders as
## DAY_EXPOSURE, as the Earth shaders' day lighting does. The eye adaptation (EyeAdaptation, fidelity flag
## eye_adaptation_enabled) multiplies all of it by the cone gain (1 by day, up to ~1.2e4 at night) and adds the rods'
## image in a post step (VisionEffect, night vision).

## Scene value of a white Lambertian surface under the full TOA Sun, the one exposure of the game [call: M8 atmosphere
## spec, "stays 1.5"]. The `day_exposure` uniforms of shaders/earth.gdshader and terrain_earth.gdshader are set from
## it (world.gd); they have no default of their own.
const DAY_EXPOSURE := 1.5
## Astronomical unit (m) [S: IAU 2012 Resolution B2]; the tables are for the Sun at 1 AU.
const ASTRONOMICAL_UNIT_M := 149597870700.0
## a / b of WGS84 (1 / (1 - f), f = 1 / 298.257223563) [S: NIMA TR8350.2]: the model space's polar stretch, as the core.
const POLAR_STRETCH := 1.0 / (1.0 - 1.0 / 298.257223563)
const CACHE_FILE := "earth_atmosphere.bin"
## [S: ITU-R BT.709] luminance weights of the linear RGB channels (as star_visibility.gdshaderinc).
const LUMINANCE_WEIGHTS := Vector3(0.2126, 0.7152, 0.0722)
## [call] In-scattering segments of the eclipse sky while the Moon's penumbra touches the atmosphere (spec goal 8).
const ECLIPSE_SEGMENTS := 8
## [S: IAU 2015 Resolution B3] nominal solar radius; [S: core constants.hpp moon_mean_radius_m] the Moon's mean radius.
const SUN_RADIUS_M := 695700000.0
const MOON_RADIUS_M := 1737400.0
## [S: NIMA TR8350.2] WGS84 semi-major axis.
const WGS84_A_M := 6378137.0
## [call] Heights (m, above the ellipsoid) of the umbra's light (_update_umbra_light) and its update interval (s).
const UMBRA_HEIGHTS_M := [0.0, 4000.0, 12000.0]
const UMBRA_UPDATE_S := 1.0
## [call] _eclipsed_sky_fluence's elevation samples (deg: the zenith, then rings of four azimuths) and the edges of the
## bands they stand for; dense near the horizon, where the totality sky brightens steeply (a 0-15 deg band sampled at
## 6 deg alone gave 3x less fluence than at 4 deg). Solid angle of a band: 2 pi (sin e2 - sin e1), split over its ring.
const UMBRA_ELEVATIONS_DEG := [90.0, 30.0, 11.0, 6.0, 3.0, 1.0]
const UMBRA_BAND_EDGES_DEG := [50.0, 15.0, 8.0, 4.5, 2.0, 0.0]
## [call] Segments per direction (converged: 8 vs 16 within 9 %).
const UMBRA_SEGMENTS := 8
## [call] The eye adaptation's limb samples graze the Earth this high (m) above the model ground.
const LIMB_SAMPLE_HEIGHT_M := 10000.0
## The eye adaptation away from the Earth (adaptation_luminance, _moon_in_view_luminance):
## [S: World.EARTH_MEAN_RADIUS_M, the Moon shaders' earth_radius_m] the Earth's mean radius of the Earth's shadow.
const EARTH_MEAN_RADIUS_M := 6371000.0
## [D] Mean albedo-texture value of the lunar surface: data/textures/moon_color.png (CGI Moon Kit, LROC WAC colour
## mosaic, tools/data_pipeline/sources.toml "moon_color") sRGB-decoded, BT.709 luminance, area-weighted over the sphere
## (0.2997; the near side 0.255) - the albedo the Moon shaders multiply.
const MOON_MEAN_TEXTURE_ALBEDO := 0.30
## The regolith's reflectance (shaders/moon_regolith.gdshaderinc, regolith_reflectance here; night-vision fix round I3,
## I4): albedo x regolith_scale x E/pi x Lommel-Seeliger x the K&S phase factor, for the Sun and the earthshine alike.
## [D: tools/validation/moon_disc_calibration.py] the albedo texture's near-side mean weighted by projected area
## (cos^2 lat cos lon, sRGB-decoded, BT.709): the full-phase disc seen from the Earth (the maria pull it below the
## sphere's 0.30).
const MOON_NEAR_SIDE_DISC_ALBEDO := 0.2245
## [S: K&S 1991 sec. 2, as Schaefer's skycalc applies it] the opposition surge: the Moon's illuminance x (1.35 - 0.05 a)
## for phase angles a < 7 deg (1 at 7 deg). The shaders' REGOLITH_* constants: keep in sync.
const MOON_OPPOSITION_PEAK := 1.35
const MOON_OPPOSITION_PER_DEG := 0.05
const MOON_OPPOSITION_END_DEG := 7.0
## [call] The regolith's phase law is held beyond this angle (K&S's quartic turns up past ~150 deg).
const REGOLITH_PHASE_MAX_DEG := 150.0
## [call] The regolith's least cosine to the eye (moon_regolith.gdshaderinc): a normal turned further away (the faceted
## silhouette) is taken back onto the limb before the light's cosine.
const REGOLITH_MU_MIN := 1.0e-3
## [D: pi x the tables' 4.24e4 cd/m^2 white plate] TOA illuminance (lux) for the regolith scale before the tables load.
const NOMINAL_TOA_LUX := 1.3327e5
## [S: NASA Earth Fact Sheet, Bond albedo 0.306] the Earth's light on the Moon (earthshine, earthshine_relative).
const EARTH_BOND_ALBEDO := 0.306
## [S: Lambert-sphere phase integral q = 3/2, A_Bond = p q; e.g. Seager, Exoplanet Atmospheres (2010) ch. 3] the
## geometric albedo of a Lambertian sphere over its Bond albedo.
const LAMBERT_GEOMETRIC_PER_BOND := 2.0 / 3.0
## Moonlight (night vision, 2026-10-25): the Moon's illuminance outside the atmosphere at the mean distance by
## Krisciunas & Schaefer 1991, "A model of the brightness of moonlight", PASP 103, 1033 [S]: Eq. 9 the Moon's V
## magnitude m = -12.73 + 0.026 |a| + 4e-9 a^4 (phase angle a in degrees), Eq. 8 the illuminance
## I* = 10^(-0.4 (m + 16.57)) foot-candles (full Moon 0.313 lux) [S: 1 fc = 10.7639 lux]; scaled by
## (MOON_MEAN_DISTANCE_M / d)^2 [D]. Their accuracy: 8-23 %.
const MOON_MAGNITUDE_FULL := -12.73
const MOON_MAGNITUDE_PER_DEG := 0.026
const MOON_MAGNITUDE_PER_DEG4 := 4.0e-9
const MOON_FOOT_CANDLE_OFFSET := 16.57
const LUX_PER_FOOT_CANDLE := 10.763910
## [S: Krisciunas & Schaefer 1991, the mean Earth-Moon distance of their model] (m).
const MOON_MEAN_DISTANCE_M := 384400000.0
## [call] Moonlight is evaluated (shaders, the vehicles' second light, the adaptation's Moon terms) only while the
## Moon's white plate (relative illuminance x the reference luminance) is at least this share of the luminance the eye
## is adapted to: below it the moonlight is invisible. By day (full Moon ~0.1 cd/m^2 against >= 1e3) and at new Moon
## (an eclipse) it is off - day frames stay exactly as before, at no cost.
const MOONLIGHT_MIN_SHARE := 1.0e-3
## [call] The twilight glow (_twilight_glow_luminance): elevations sampled along the Sun's azimuth (deg; the glow is
## brightest low over the horizon) and the least angle from the Sun (deg) - outside the Sun's aureole, so the term grows
## from below the white plate at sunset instead of jumping.
const TWILIGHT_GLOW_ELEVATIONS_DEG := [1.0, 3.0, 6.0, 10.0, 20.0, 40.0]
const TWILIGHT_GLOW_MIN_SUN_DEG := 8.0
## [call] In a solar eclipse at the eye (night-vision fix round I2) the brightest sky is the ring around the umbra: the
## glow then samples these azimuths (evenly from the Sun's) at these elevations, the eclipsed sky (_eclipsed_in_scatter).
const ECLIPSE_GLOW_AZIMUTHS := 8
const ECLIPSE_GLOW_ELEVATIONS_DEG := [1.0, 3.0, 6.0, 12.0]
## [S: Ferwerda et al. 1996 sec. 4.4] the world adaptation luminance is half the highest visible luminance: the glow
## terms (twilight, totality, the orbital limb) enter at this share of the brightest sky (review M5).
const GLOW_ADAPTATION_SHARE := 0.5
## [call] The orbital-night limb glow (_limb_glow_luminance): tangent heights sampled toward the Sun (m).
const LIMB_GLOW_HEIGHTS_M := [2000.0, 5000.0, 10000.0, 20000.0, 40000.0]
## [call] An eye on the Moon stands at least this high above its ground sphere (render ground views: 2 m).
const MOON_EYE_MIN_HEIGHT_M := 2.0
## [E] The terrain's horizon decides the Sun within this distance of the Moon's centre (its sphere of influence, 66 000 km):
## no height switch (relief shadows at every height, MoonSunHorizon's exact pre-check skips the march where nothing can
## reach the Sun); beyond it - Earth's surroundings, where the atmosphere / the Earth's eclipse own the Sun - the mean
## sphere stays (its step there: relief 10.8 km / 66 000 km = 0.009 deg, under 2 % of the solar disc).
const MOON_TERRAIN_SUN_MAX_DISTANCE_M := 66.0e6
## [call] The lunar cap's quadrature: rings uniform in sin^2 of the nadir angle (cosine weights), directions per ring.
const MOON_VIEW_RINGS := 4
const MOON_VIEW_AZIMUTHS := 8

## True once the textures are uploaded and atmosphere_enabled is set.
var ready := false
## A table load is running on a worker thread.
var loading := false
## German status line for the UI while the tables load or compute ("" when idle); the error text after a failure.
var status_text := ""
## AtmosphereParameters of the loaded tables (StarsimCore.load_atmosphere "parameters").
var parameters := {}
## GPU format of the tables, chosen before start_loading. Half float [call, Task 5 fix round 1]: half the VRAM (42 vs 85
## MB) and fetch bandwidth; relative rounding ~5e-4, the GPU check stays within its 2 % (numbers in the Task 5
## report); RGBAF for comparisons (atmosphere_check.gd --f32). The transmittance table is always RGBAF [call, Task 6]:
## its grazing values from the ground (T to the top ~1e-5 and below) fall into the half-float subnormal range, and the
## aerial perspective divides two of them (a terrain pixel 20 km from the Cape camera was 4 % off in blue with RGBAH,
## 0.05 % with RGBAF); +8 MB.
var table_format := Image.FORMAT_RGBAH
## Per-frame values (readable by tests): table luminance (cd/m^2 for the Sun at 1 AU) -> scene value (includes the
## 1 / d_AU^2 of the true Sun distance: do not apply it to a true luminance), camera height above the model ground (stretched
## space), the Sun's Earth-local direction and the camera's Earth-local position.
var luminance_scale := 0.0
var camera_altitude_m := 0.0
var sun_direction_local := Vector3.ZERO
var camera_local_m := Vector3.ZERO
## Fidelity flag: eye adaptation (EyeAdaptation) - on in the game; off: the fixed day exposure (atmosphere_check.gd).
var eye_adaptation_enabled := true
## The cone gain of the eye adaptation this frame (1 without it). Times `brightness` it is in luminance_scale and
## scene_per_cd_m2, and the shaders read that product as atmosphere_exposure_gain for what is not lit through the tables
## (Moon, Sun disc).
var exposure_gain := 1.0
## A true luminance (cd/m^2, at the Sun's actual distance) -> scene value, incl. the exposure gain.
var scene_per_cd_m2 := 0.0
## The luminance the eye is adapted to this frame (cd/m^2, EyeAdaptation.adaptation_cd_m2; 0 with the adaptation off):
## the global atmosphere_adaptation_cd_m2, the star threshold's background floor.
var adaptation_cd_m2 := 0.0
## Eclipse sky segments this frame (0: no penumbra on the Earth).
var eclipse_segments := 0
var _eye := EyeAdaptation.new()
## The eye adaptation's two parts at the last update (cd/m^2; adaptation_luminance), for tests and the report.
var last_adaptation_down_cd_m2 := 0.0
var last_adaptation_up_cd_m2 := 0.0
## Away from the Earth: the Moon in view (cd/m^2), the earthshine on a plate at the Moon facing the Earth (lux) and the
## Sun's visible fraction the down part used (0 .. 1).
var last_adaptation_moon_cd_m2 := 0.0
var last_earthshine_lux := 0.0
## The earthshine this frame as the Moon shaders' scene value (global moon_earthshine): a white Lambertian at the Moon
## facing the Earth, DAY_EXPOSURE x earthshine_relative, without the exposure gain (the shaders apply
## atmosphere_exposure_gain to the whole Moon); 0 without a Moon.
var earthshine_scene := 0.0
var last_adaptation_visible_fraction := 1.0
## The Sun's visible fraction at the camera from the lunar terrain's horizon (MoonSunHorizon; world.gd sets it per
## frame, NAN: no terrain or beyond MOON_TERRAIN_SUN_MAX_DISTANCE_M): it replaces the Moon sphere's occultation.
var eye_terrain_sun_fraction := NAN
## The twilight glow at the last update (cd/m^2; 0 with the Sun up or outside the atmosphere).
var last_adaptation_glow_cd_m2 := 0.0
## Night vision at the last update: the scotopic adaptation luminance (scotopic cd/m^2, paced), the rods' scale of the
## post step (EyeAdaptation.rod_scale), the moonlight relative to the TOA Sun at its distance (moonlight_relative; 0
## without a Moon) and whether the moonlight terms were active (MOONLIGHT_MIN_SHARE).
var scotopic_cd_m2 := 0.0
var rod_scale := 0.0
## The suprathreshold factors at the last update (EyeAdaptation.brightness / chroma; 1 by day or with the adaptation
## off): the brightness is in luminance_scale, scene_per_cd_m2 and atmosphere_exposure_gain (not in exposure_gain), the
## chroma in the post step.
var brightness := 1.0
var chroma := 1.0
var moonlight_relative_value := 0.0
var moonlight_active := false
## The unpaced targets of the last adaptation_luminance (photopic and scotopic cd/m^2).
var last_target_cd_m2 := 0.0
var last_target_scotopic_cd_m2 := 0.0
## Night vision's rod step on the HDR scene (attached to the WorldEnvironment's Compositor: world.gd, the GPU check).
var vision_effect := VisionEffect.new()
var _moon_direction_fixed := Vector3.ZERO
var _cloud_cache := {}  ## cloud_light_at per frame: [position, direction, probe, frame] -> Vector3
## The direct Sun through the clouds as drawn (World: CloudLayer.direct_sun_transmittance(key, position_itrf_m) -> T or
## NAN): replaces the grid mean's direct beam of the Sun light and the eye's plate where it has a value. Unset (checks):
## the grid mean.
var direct_sun: Callable
## The umbra's light (_update_umbra_light) at the last evaluation: the eclipsed sky's fluence (scalar irradiance, cd/m^2 x
## sr per channel at 1 AU) at UMBRA_HEIGHTS_M (tests).
var umbra_fluence: Array[Vector3] = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var _umbra_tdb_s := -INF
## Tests: compute the umbra's light on the calling thread (one render later it is in place).
var umbra_light_synchronous := false
var _umbra_task_id := -1
var _umbra_result: Array[Vector3] = []
var _tdb_s := NAN
var _fixed_to_gcrf := Basis.IDENTITY
var _earth_center_gcrf_m := Vector3.ZERO

var _core: StarsimCore
var _task_id := -1
var _result := {}  ## written by the worker task, read after it completed
var _textures: Array[Texture] = []  ## keeps the uploaded textures alive


## Starts loading the tables in the background (config_dir/atmosphere/earth_optics.toml, cache in cache_dir).
func start_loading(core: StarsimCore, config_dir: String, cache_dir: String) -> void:
	if loading or ready:
		return
	_core = core
	ShaderGlobals.set_value(&"atmosphere_enabled", false)
	var cached := FileAccess.file_exists(cache_dir.path_join(CACHE_FILE))
	status_text = ("Atmosphäre wird geladen ..." if cached
			else "Atmosphäre wird berechnet (einmalig, etwa eine Minute) ...")
	loading = true
	_task_id = WorkerThreadPool.add_task(_load_task.bind(config_dir, cache_dir), false, "atmosphere tables")


## Main thread: uploads the tables once the worker finished. Returns true when the atmosphere is ready.
func poll() -> bool:
	if loading and WorkerThreadPool.is_task_completed(_task_id):
		WorkerThreadPool.wait_for_task_completion(_task_id)
		loading = false
		_upload(_result)
		_result = {}
	return ready


## Blocks until a running load finished and uploads it (tests).
func wait_until_loaded() -> bool:
	if loading:
		WorkerThreadPool.wait_for_task_completion(_task_id)
		loading = false
		_upload(_result)
		_result = {}
	return ready


## Waits for the worker tasks still running (the table load, the umbra light) before the owner goes away (review M2:
## they write into this object and use the core). Quitting during a first-run precomputation blocks until it ends.
func shutdown() -> void:
	if loading:
		WorkerThreadPool.wait_for_task_completion(_task_id)
		loading = false
		_result = {}
	if _umbra_task_id >= 0:
		WorkerThreadPool.wait_for_task_completion(_umbra_task_id)
		_umbra_task_id = -1
	_set_vision(0.0)  # review M4


## Per frame: the camera, the Earth's centre and orientation (fixed -> GCRF) and the apparent Sun position (GCRF, as
## seen from the Earth's centre), all float64. The camera is differenced against the Earth's centre here, in float64.
## Eye adaptation (eye_adaptation_enabled): adaptation_dt_s is the time the viewer had to adapt since the last frame
## (world.gd: the longer of the frame time and the simulated time step), sun_visible_fraction the part of the solar disc
## the Moon leaves at the camera.
## moon_apparent_gcrf_m: the Moon's apparent position (GCRF, as seen from the Earth's centre) for the eclipse sky;
## Vector3.ZERO: no eclipse. tdb_s: the instant (the Moon's shadow on points other than the camera: the eye adaptation's
## view of the Earth, the umbra's light); NAN: no shadow there. moon_gcrf_m: the Moon's geometric centre (GCRF) for the eye
## adaptation away from the Earth (the lunar ground in view); Vector3.ZERO: none.
func update(camera_gcrf_m: Vector3, earth_center_gcrf_m: Vector3, earth_fixed_to_gcrf: Basis,
		sun_apparent_gcrf_m: Vector3, adaptation_dt_s := 0.0, sun_visible_fraction := 1.0,
		moon_apparent_gcrf_m := Vector3.ZERO, tdb_s := NAN, moon_gcrf_m := Vector3.ZERO) -> void:
	# The Moon shaders' lights need no tables (before them: the vacuum scene units, gain 1).
	var earthshine_relative_value := 0.0
	if moon_gcrf_m != Vector3.ZERO:
		earthshine_relative_value = earthshine_relative(moon_gcrf_m - earth_center_gcrf_m,
				sun_apparent_gcrf_m - earth_center_gcrf_m)
	earthshine_scene = DAY_EXPOSURE * earthshine_relative_value
	var sun_au := (sun_apparent_gcrf_m - earth_center_gcrf_m).length() / ASTRONOMICAL_UNIT_M
	ShaderGlobals.set_value(&"moon_earthshine", earthshine_scene)
	ShaderGlobals.set_value(&"moon_sun_scene", DAY_EXPOSURE / (sun_au * sun_au))
	ShaderGlobals.set_value(&"moon_regolith_scale",
			regolith_scale(float(parameters.get("toa_solar_illuminance_lux", NOMINAL_TOA_LUX))))
	if not poll():
		_set_vision(0.0)  # no rods left from an earlier state (review M4)
		return
	last_earthshine_lux = earthshine_relative_value * float(parameters["toa_solar_illuminance_lux"])
	_fixed_to_gcrf = earth_fixed_to_gcrf
	_earth_center_gcrf_m = earth_center_gcrf_m
	var gcrf_to_fixed := earth_fixed_to_gcrf.transposed()
	var camera_fixed := gcrf_to_fixed * (camera_gcrf_m - earth_center_gcrf_m)
	camera_local_m = Frames.to_render(camera_fixed)
	var split := TerrainQuadtree.split_float32(camera_local_m)
	var stretched := Vector3(camera_fixed.x, camera_fixed.y, camera_fixed.z * POLAR_STRETCH)
	camera_altitude_m = stretched.length() - float(parameters["bottom_radius_m"])
	var sun_from_earth := sun_apparent_gcrf_m - earth_center_gcrf_m
	var d_au := sun_from_earth.length() / ASTRONOMICAL_UNIT_M
	sun_direction_local = Frames.to_render(gcrf_to_fixed * sun_from_earth).normalized()
	var sun_fixed := (gcrf_to_fixed * sun_from_earth).normalized()
	# The Moon's shadow on other points than the camera only while its penumbra touches the atmosphere (review M1:
	# otherwise every _visible_at is 1 and its CSPICE calls are skipped).
	var penumbra := false
	if moon_apparent_gcrf_m != Vector3.ZERO:
		penumbra = penumbra_touches(sun_from_earth, moon_apparent_gcrf_m - earth_center_gcrf_m,
				float(parameters["top_radius_m"]))
	_tdb_s = tdb_s if penumbra else NAN
	# The Moon as a light (night vision): its direction from the Earth's centre (apparent if given, else geometric) and
	# its illuminance; active only while the eye is adapted to a dim scene.
	var moon_light_from_earth := moon_apparent_gcrf_m if moon_apparent_gcrf_m != Vector3.ZERO else moon_gcrf_m
	moonlight_relative_value = 0.0
	_moon_direction_fixed = Vector3.ZERO
	if moon_light_from_earth != Vector3.ZERO:
		moon_light_from_earth -= earth_center_gcrf_m
		moonlight_relative_value = moonlight_relative(moon_light_from_earth, sun_from_earth,
				float(parameters["toa_solar_illuminance_lux"]))
		_moon_direction_fixed = (gcrf_to_fixed * moon_light_from_earth).normalized()
	exposure_gain = 1.0
	adaptation_cd_m2 = 0.0
	scotopic_cd_m2 = 0.0
	rod_scale = 0.0
	brightness = 1.0
	chroma = 1.0
	moonlight_active = false
	if eye_adaptation_enabled:
		var moon_fixed := gcrf_to_fixed * (moon_gcrf_m - earth_center_gcrf_m) if moon_gcrf_m != Vector3.ZERO else Vector3.ZERO
		var luminance := adaptation_luminance(camera_fixed, sun_fixed, sun_visible_fraction, d_au, moon_fixed)
		_eye.advance(luminance, last_target_scotopic_cd_m2, reference_luminance(d_au), adaptation_dt_s)
		exposure_gain = _eye.gain
		adaptation_cd_m2 = _eye.adaptation_cd_m2
		scotopic_cd_m2 = _eye.scotopic_cd_m2
		rod_scale = _eye.rod_scale
		brightness = _eye.brightness
		chroma = _eye.chroma
		moonlight_active = moonlight_relative_value * reference_luminance(d_au) >= MOONLIGHT_MIN_SHARE * adaptation_cd_m2
	ShaderGlobals.set_value(&"atmosphere_adaptation_cd_m2", adaptation_cd_m2)
	ShaderGlobals.set_value(&"atmosphere_moon_light",
			moonlight_relative_value if moonlight_active else 0.0)
	if _moon_direction_fixed != Vector3.ZERO:
		ShaderGlobals.set_value(&"atmosphere_moon_direction", Frames.to_render(_moon_direction_fixed))
	_set_vision(rod_scale, chroma)
	# The display's pre-exposure: the cone gain x the brightness factor (Stevens effect; the rods, relative to the
	# pixel, follow it). The star threshold reads the sky's true luminance back through scene_per_cd_m2: unchanged.
	var display_gain := exposure_gain * brightness
	var base := DAY_EXPOSURE * PI / float(parameters["toa_solar_illuminance_lux"])
	luminance_scale = base * display_gain / (d_au * d_au)
	scene_per_cd_m2 = base * display_gain
	ShaderGlobals.set_value(&"atmosphere_scene_per_cd_m2", scene_per_cd_m2)
	ShaderGlobals.set_value(&"atmosphere_exposure_gain", display_gain)
	eclipse_segments = 0
	if moon_apparent_gcrf_m != Vector3.ZERO:
		var moon_from_earth := moon_apparent_gcrf_m - earth_center_gcrf_m
		if penumbra:
			eclipse_segments = ECLIPSE_SEGMENTS
			_update_umbra_light(gcrf_to_fixed * sun_from_earth, gcrf_to_fixed * moon_from_earth, sun_fixed)
		ShaderGlobals.set_value(&"atmosphere_moon_position",
				Frames.to_render(gcrf_to_fixed * moon_from_earth))
	ShaderGlobals.set_value(&"atmosphere_sun_distance_m", sun_from_earth.length())
	ShaderGlobals.set_value(&"atmosphere_eclipse_segments", eclipse_segments)
	if eclipse_segments == 0 and _umbra_task_id >= 0:
		# The eclipse ended with a task in flight: its fluence belongs to another epoch, never uploaded (review M5).
		WorkerThreadPool.wait_for_task_completion(_umbra_task_id)
		_umbra_task_id = -1
	if eclipse_segments == 0 and _umbra_tdb_s != -INF:
		_umbra_tdb_s = -INF
		for k in range(3):
			umbra_fluence[k] = Vector3.ZERO
			ShaderGlobals.set_value(StringName("atmosphere_umbra_fluence_%d" % k), Vector3.ZERO)
	# Render world axes are GCRF in render axes; Earth-local = P R^T P^T (the transpose of the body's render basis).
	var world_to_earth := Frames.basis_to_render(earth_fixed_to_gcrf).transposed()
	ShaderGlobals.set_value(&"atmosphere_camera_hi", split[0])
	ShaderGlobals.set_value(&"atmosphere_camera_lo", split[1])
	ShaderGlobals.set_value(&"atmosphere_camera_altitude_m", camera_altitude_m)
	ShaderGlobals.set_value(&"atmosphere_world_to_earth", world_to_earth)
	ShaderGlobals.set_value(&"atmosphere_sun_direction", sun_direction_local)
	ShaderGlobals.set_value(&"atmosphere_luminance_scale", luminance_scale)


## Eye adaptation's luminance at an Earth-fixed eye (cd/m^2), the larger of two white-plate equivalents (both view-
## direction independent, so identical for both VR eyes and steady under head motion):
## - down: a white Lambertian at the eye lit by the direct Sun at normal incidence (TOA x the transmittance to the Sun)
##   and the sky's horizontal irradiance, (E_sun,n + E_sky) / pi x visible_fraction (the sky's part as well: the CPU
##   irradiance knows no eclipse, inside the Moon's shadow the sky overhead is shadowed with the eye);
## - up [Task 8+9 review I1]: the Earth in view. The visible ground along 13 directions below the eye (the nadir,
##   weight 0.25; six at 0.6 x the Earth disc's angular radius, 0.5; six grazing the limb LIMB_SAMPLE_HEIGHT_M up, or
##   2 deg above the horizontal on the ground, 0.25 - a cosine-weighted cap), each the plate equivalent of what it
##   shows: the ground's illuminance / pi x the view's transmittance plus the in-scattered light on the way
##   (StarsimCore.atmosphere_sky_luminance_to_point), both x the visible fraction at the ground point, or the limb's sky
##   luminance (x the visible fraction at the ray's closest approach); times the view factor of the Earth's cap
##   (R / r)^2. The ground's albedo is not applied: the plate equivalent of a lit ground is the light on it, as for the
##   down part. On the ground this stays below the down part (the light on the ground and the horizon sky do not exceed
##   the Sun + sky at the eye); in orbit an eye in the Moon's umbra or just past orbital sunset adapts to the sunlit
##   Earth or limb it looks at.
## Away from the Earth (the eye above the top of the atmosphere; inside it nothing changes) [Moon-eye fix 2026-10-25]:
## - down: the transmittance is 1 there (or the grazing ray's), the sky irradiance the top boundary's (~0); the visible
##   fraction also takes the Earth's disc in front of the Sun (an eye in the Earth's shadow far out: the transmittance
##   alone is the Sun's centre ray, a sharp edge) [call: where both act, within the Sun's radius of the limb, the
##   penumbra is counted twice - a little too dark there]. For an eye within MOON_EYE_MIN_HEIGHT_M of the lunar mean
##   sphere or below it, the Moon's part is recomputed on its ground sphere (_moon_ground_radius: an eye below the mean
##   sphere stands on the terrain; the passed fraction uses the mean sphere, which would hide the Sun from it) unless
##   eye_terrain_sun_fraction is finite (the lunar terrain's horizon, world.gd), which replaces the Moon's part.
## - up: the Earth in view x the part of its disc the Moon leaves (a lunar eye with the Earth below its horizon);
## - Moon in view (_moon_in_view_luminance, with moon_itrf_m): the sunlit and earthshine-lit regolith x its view factor.
## The result is the largest of the three (as down / up: plates facing the Sun, the Earth, the Moon).
## BT.709 luminance of the RGB luminance channels; the tables' 1 AU values / d_AU^2. moon_itrf_m: the Moon's geometric
## centre, Earth-fixed (Vector3.ZERO: none).
func adaptation_luminance(eye_itrf_m: Vector3, sun_direction_itrf: Vector3, visible_fraction: float, d_au: float,
		moon_itrf_m := Vector3.ZERO) -> float:
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	var transmittance := _core.atmosphere_sun_transmittance(eye_itrf_m, sun_direction_itrf)
	var up := _geodetic_up(eye_itrf_m)
	var sky: Vector3 = _core.atmosphere_illuminance(eye_itrf_m, up, sun_direction_itrf)["sky_lux"]
	var outside := _stretch(eye_itrf_m).length() > float(parameters["top_radius_m"])
	# The Moon's parts above the atmosphere (its occultations, the Moon in view): physical anywhere, negligible far out.
	var lunar := outside and moon_itrf_m != Vector3.ZERO
	# Near the Moon: its disc larger than the Earth's (review I1: an eye in LEO is not near it and keeps the limb glow).
	var near_moon := lunar and MOON_RADIUS_M * eye_itrf_m.length() > EARTH_MEAN_RADIUS_M * (moon_itrf_m
			- eye_itrf_m).length()
	var sun_itrf_m := sun_direction_itrf * (d_au * ASTRONOMICAL_UNIT_M)
	var visible := visible_fraction
	var moon_ground_m := 0.0
	if lunar:
		moon_ground_m = _moon_ground_radius((moon_itrf_m - eye_itrf_m).length())
		visible = sun_visible_near_moon(eye_itrf_m, sun_itrf_m, moon_itrf_m, visible_fraction, eye_terrain_sun_fraction)
	# (the terrain's fraction comes with the Earth's occultation already)
	if outside and not (lunar and is_finite(eye_terrain_sun_fraction)):
		visible *= 1.0 - occulted_fraction(sun_itrf_m - eye_itrf_m, SUN_RADIUS_M, -eye_itrf_m, EARTH_MEAN_RADIUS_M)
	last_adaptation_visible_fraction = visible
	# Clouds (Task 9): the plate's direct Sun through them, the Sun's light diffused through the column (isotropic from
	# above: mu0 x its normal-incidence irradiance on the horizontal) and the sky's diffuse transmittance
	# (StarsimCore.weather_cloud_light: 1, 0, 1 with weather off - the same numbers as before).
	var clouds := cloud_light_at(eye_itrf_m, sun_direction_itrf, "eye")
	var sun_plate: Vector3 = solar * sun_factor * transmittance
	var down_rgb := (sun_plate * (clouds.x + clouds.y * maxf(sun_direction_itrf.dot(up), 0.0)) + sky * clouds.z) \
			* visible / (PI * d_au * d_au)
	var earth_rgb := _earth_in_view_luminance(eye_itrf_m, sun_direction_itrf) / (d_au * d_au)
	var moon_rgb := Vector3.ZERO
	if lunar:
		earth_rgb *= 1.0 - occulted_fraction(-eye_itrf_m, EARTH_MEAN_RADIUS_M, moon_itrf_m - eye_itrf_m, moon_ground_m)
		# Earthshine on a plate at the Moon facing the Earth: the Lambert-sphere Earth of earthshine_relative, the same
		# value the Moon shaders get (moon_earthshine).
		last_earthshine_lux = earthshine_relative(moon_itrf_m, sun_itrf_m) * float(parameters["toa_solar_illuminance_lux"])
		# The regolith is grey: the solar colour (sunlight and earthshine) at the term's luminance.
		var solar_rgb := solar * sun_factor
		moon_rgb = solar_rgb / solar_rgb.dot(LUMINANCE_WEIGHTS) * _moon_in_view_luminance(eye_itrf_m, moon_itrf_m,
				sun_itrf_m, moon_ground_m, last_earthshine_lux, d_au) * (1.0 - occulted_fraction(moon_itrf_m - eye_itrf_m,
				moon_ground_m, -eye_itrf_m, EARTH_MEAN_RADIUS_M))
	# Moonlight (night vision), only while it can matter (the Moon's white plate at least MOONLIGHT_MIN_SHARE of the
	# Sun's terms): inside the atmosphere the Moon's plate at the eye (direct + the moonlit sky),
	# above it (orbit night side; an eye nearer the Earth than the Moon) the moonlit Earth in view.
	var dim := moonlight_relative_value * reference_luminance(d_au) >= MOONLIGHT_MIN_SHARE * maxf(
			down_rgb.dot(LUMINANCE_WEIGHTS), earth_rgb.dot(LUMINANCE_WEIGHTS))
	var nearer_earth := moon_itrf_m == Vector3.ZERO or (moon_itrf_m - eye_itrf_m).length() > eye_itrf_m.length()
	if dim and _moon_direction_fixed != Vector3.ZERO and moonlight_relative_value > 0.0 and nearer_earth:
		var moon_scale := moonlight_relative_value / (d_au * d_au)
		if outside:
			earth_rgb += _earth_in_view_luminance(eye_itrf_m, _moon_direction_fixed) * moon_scale
		else:
			var moon_t := _core.atmosphere_sun_transmittance(eye_itrf_m, _moon_direction_fixed)
			var moon_sky: Vector3 = _core.atmosphere_illuminance(eye_itrf_m, up, _moon_direction_fixed)["sky_lux"]
			var moon_clouds := cloud_light_at(eye_itrf_m, _moon_direction_fixed)
			var moon_plate: Vector3 = solar * sun_factor * moon_t
			down_rgb += (moon_plate * (moon_clouds.x + moon_clouds.y * maxf(_moon_direction_fixed.dot(up), 0.0))
					+ moon_sky * moon_clouds.z) * (moon_scale / PI)
	# The glow (night vision), GLOW_ADAPTATION_SHARE of the brightest sky: inside the atmosphere with the Sun below the
	# eye's horizon (twilight) or the eye in a solar eclipse (the ring around the umbra; review I2: the eye's own
	# eclipse, not the global penumbra flag - a twilight eye far from the shadow keeps its glow); above it with the Sun
	# behind the Earth (orbital night) and the eye not near the Moon, the brightest limb toward the Sun.
	var glow_rgb := Vector3.ZERO
	var eclipsed := visible < 1.0 and not is_nan(_tdb_s)
	if not outside and (sun_direction_itrf.dot(up) < 0.0 or eclipsed):
		glow_rgb = _twilight_glow_luminance(eye_itrf_m, up, sun_direction_itrf, eclipsed) * (GLOW_ADAPTATION_SHARE
				/ (d_au * d_au))
	elif outside and transmittance == Vector3.ZERO and not near_moon:
		glow_rgb = _limb_glow_luminance(eye_itrf_m, sun_direction_itrf) * (GLOW_ADAPTATION_SHARE / (d_au * d_au))
	last_adaptation_down_cd_m2 = down_rgb.dot(LUMINANCE_WEIGHTS)
	last_adaptation_up_cd_m2 = earth_rgb.dot(LUMINANCE_WEIGHTS)
	last_adaptation_moon_cd_m2 = moon_rgb.dot(LUMINANCE_WEIGHTS)
	last_adaptation_glow_cd_m2 = glow_rgb.dot(LUMINANCE_WEIGHTS)
	# The largest term (as before) sets the photopic adaptation luminance; its colour the scotopic one.
	var chosen := down_rgb
	if last_adaptation_up_cd_m2 > chosen.dot(LUMINANCE_WEIGHTS):
		chosen = earth_rgb
	if last_adaptation_moon_cd_m2 > chosen.dot(LUMINANCE_WEIGHTS):
		chosen = moon_rgb
	if last_adaptation_glow_cd_m2 > chosen.dot(LUMINANCE_WEIGHTS):
		chosen = glow_rgb
	last_target_cd_m2 = chosen.dot(LUMINANCE_WEIGHTS)
	last_target_scotopic_cd_m2 = maxf(chosen.dot(EyeAdaptation.SCOTOPIC_WEIGHTS), 0.0)
	return last_target_cd_m2


## The glow of the eye adaptation (RGB luminance, cd/m^2 at 1 AU): the brightest sky luminance, leaving out directions
## within TWILIGHT_GLOW_MIN_SUN_DEG of the Sun, wherever the head points (view-independent, VR-stable); the caller takes
## GLOW_ADAPTATION_SHARE of it (Ferwerda et al. 1996 sec. 4.4: half the highest visible luminance). By day the white
## plate is the larger anyway (the day look unchanged).
## - Twilight: along the Sun's azimuth at TWILIGHT_GLOW_ELEVATIONS_DEG (the glow over the Sun's horizon is the
##   brightest sky). Accepted limit (review M6): azimuth-only, a brighter sky elsewhere (the Moon's aureole) is not
##   looked for.
## - eclipsed (the eye in a solar eclipse, review I2): the ring around the umbra surrounds the observer, so
##   ECLIPSE_GLOW_AZIMUTHS azimuths at ECLIPSE_GLOW_ELEVATIONS_DEG, each the eclipsed sky (_eclipsed_in_scatter, as the
##   Earth in view) - the same "brightest sky" rule as in twilight.
func _twilight_glow_luminance(eye_itrf_m: Vector3, up: Vector3, sun_direction_itrf: Vector3,
		eclipsed := false) -> Vector3:
	var horizontal := sun_direction_itrf - up * sun_direction_itrf.dot(up)
	if horizontal.length_squared() < 1.0e-12:
		if not eclipsed:
			return Vector3.ZERO
		horizontal = up.cross(Vector3(0, 0, 1) if absf(up.z) < 0.9 else Vector3(1, 0, 0))
	horizontal = horizontal.normalized()
	var side := up.cross(horizontal)
	var directions: Array[Vector3] = []
	if eclipsed:
		for k in range(ECLIPSE_GLOW_AZIMUTHS):
			var azimuth := TAU * k / ECLIPSE_GLOW_AZIMUTHS
			var h := horizontal * cos(azimuth) + side * sin(azimuth)
			for elevation_deg: float in ECLIPSE_GLOW_ELEVATIONS_DEG:
				var e := deg_to_rad(elevation_deg)
				directions.append((h * cos(e) + up * sin(e)).normalized())
	else:
		for elevation_deg: float in TWILIGHT_GLOW_ELEVATIONS_DEG:
			var e := deg_to_rad(elevation_deg)
			directions.append((horizontal * cos(e) + up * sin(e)).normalized())
	var brightest := Vector3.ZERO
	for v in directions:
		if rad_to_deg(v.angle_to(sun_direction_itrf)) < TWILIGHT_GLOW_MIN_SUN_DEG:
			continue
		var sky: Vector3
		if eclipsed:
			sky = _eclipsed_in_scatter(eye_itrf_m, v, _top_exit_distance(eye_itrf_m, v), false, sun_direction_itrf, [])
		else:
			sky = _core.atmosphere_sky_luminance(eye_itrf_m, v, sun_direction_itrf)["luminance"]
		if sky.dot(LUMINANCE_WEIGHTS) > brightest.dot(LUMINANCE_WEIGHTS):
			brightest = sky
	return brightest


## The orbital-night analogue of _twilight_glow_luminance (RGB, cd/m^2 at 1 AU): an eye above the atmosphere with the
## Sun behind the Earth sees the twilight limb toward the Sun; the brightest sky along the limb in the Sun's direction
## at the tangent heights LIMB_GLOW_HEIGHTS_M (above the WGS84 semi-major axis) [call: as the ground's glow, the
## brightest part of the view wherever the head points].
func _limb_glow_luminance(eye_itrf_m: Vector3, sun_direction_itrf: Vector3) -> Vector3:
	var r := eye_itrf_m.length()
	var c := eye_itrf_m / r
	var toward := sun_direction_itrf - c * sun_direction_itrf.dot(c)
	if toward.length_squared() < 1.0e-12:
		return Vector3.ZERO
	toward = toward.normalized()
	var brightest := Vector3.ZERO
	for height: float in LIMB_GLOW_HEIGHTS_M:
		var radius := WGS84_A_M + height
		if radius >= r:
			continue
		var tangent := c * (radius * radius / r) + toward * (radius * sqrt(1.0 - (radius / r) * (radius / r)))
		var v := (tangent - eye_itrf_m).normalized()
		var sky: Vector3 = _core.atmosphere_sky_luminance(eye_itrf_m, v, sun_direction_itrf)["luminance"]
		if sky.dot(LUMINANCE_WEIGHTS) > brightest.dot(LUMINANCE_WEIGHTS):
			brightest = sky
	return brightest


## The Moon's illuminance outside the atmosphere over the TOA Sun's at 1 AU [D]: Krisciunas & Schaefer 1991 (Eq. 8, 9)
## at the phase angle a (Sun - Moon - Earth's centre) with the opposition surge (moon_illuminance_lux), x
## (MOON_MEAN_DISTANCE_M / d)^2, / the tables' TOA illuminance at 1 AU, x the solar disc the Earth leaves at the Moon's
## centre (a lunar eclipse; the umbra's red glow left out). Its users (shaders, the adaptation) multiply it by the 1 AU
## tables / d_AU^2, so the Moon follows the Sun's distance like the real one (review M1). Full Moon at the mean
## distance: 0.313 lux (x 1.35 at a = 0), ~2.4e-6 of the Sun. Vectors from the Earth's centre, m, one frame.
static func moonlight_relative(moon_from_earth_m: Vector3, sun_from_earth_m: Vector3, toa_lux_1au: float) -> float:
	var to_sun := sun_from_earth_m - moon_from_earth_m
	var phase_deg := rad_to_deg(to_sun.angle_to(-moon_from_earth_m))
	var distance := MOON_MEAN_DISTANCE_M / moon_from_earth_m.length()
	var lit := 1.0 - occulted_fraction(to_sun, SUN_RADIUS_M, -moon_from_earth_m, EARTH_MEAN_RADIUS_M)
	return moon_illuminance_lux(phase_deg) * distance * distance / toa_lux_1au * lit


## K&S 1991 Eq. 8, 9 with the opposition surge: the Moon's illuminance (lux) at the mean distance, phase angle in deg.
static func moon_illuminance_lux(phase_deg: float) -> float:
	var magnitude := MOON_MAGNITUDE_FULL + MOON_MAGNITUDE_PER_DEG * phase_deg + MOON_MAGNITUDE_PER_DEG4 * pow(phase_deg, 4.0)
	return pow(10.0, -0.4 * (magnitude + MOON_FOOT_CANDLE_OFFSET)) * LUX_PER_FOOT_CANDLE * moon_opposition(phase_deg)


## The opposition surge factor at the phase angle (deg): MOON_OPPOSITION_PEAK - MOON_OPPOSITION_PER_DEG a below
## MOON_OPPOSITION_END_DEG, else 1.
static func moon_opposition(phase_deg: float) -> float:
	if phase_deg < MOON_OPPOSITION_END_DEG:
		return MOON_OPPOSITION_PEAK - MOON_OPPOSITION_PER_DEG * phase_deg
	return 1.0


## The regolith scale R (the shaders' moon_regolith_scale) [D]: the full-phase disc of the albedo texture's near side
## (MOON_NEAR_SIDE_DISC_ALBEDO; Lommel-Seeliger 1 at mu0 = mu, the phase factor without the surge 1) gives K&S's
## illuminance at the mean distance: R = E_KS(0) / (E_toa / pi x the disc's solid angle x the albedo) - ~0.513, the
## former shaders' 0.8 / 1.5 = 0.533 within 4 %: their excess was the missing phase law (quarter Moon x 4), not the scale.
static func regolith_scale(toa_lux_1au: float) -> float:
	var disc_sr := PI * pow(MOON_RADIUS_M / MOON_MEAN_DISTANCE_M, 2.0)
	var full_lux := moon_illuminance_lux(0.0) / moon_opposition(0.0)
	return full_lux / (toa_lux_1au / PI * disc_sr * MOON_NEAR_SIDE_DISC_ALBEDO)


## The regolith's phase factor f(g) (shaders/moon_regolith.gdshaderinc: keep in sync) from the local phase angle's
## cosine: the opposition surge x K&S's relative phase law / the Lommel-Seeliger sphere's disc-integrated phase function.
static func regolith_phase_factor(cos_g: float) -> float:
	var g_deg := minf(rad_to_deg(acos(clampf(cos_g, -1.0, 1.0))), REGOLITH_PHASE_MAX_DEG)
	var g := deg_to_rad(g_deg)
	var phi := 1.0 if g_deg < 0.01 else 1.0 - sin(0.5 * g) * tan(0.5 * g) * log(1.0 / tan(0.25 * g))
	var relative := pow(10.0, -0.4 * (MOON_MAGNITUDE_PER_DEG * g_deg + MOON_MAGNITUDE_PER_DEG4 * pow(g_deg, 4.0)))
	return moon_opposition(g_deg) * relative / maxf(phi, 1.0e-6)


## Lommel-Seeliger x f(g) for unit vectors (the shaders' regolith_reflectance): 0 with the light below the horizon.
## A normal facing away from the eye is taken onto the limb first (the shader's silhouette fix: REGOLITH_MU_MIN).
static func regolith_reflectance(n: Vector3, to_light: Vector3, to_eye: Vector3) -> float:
	var mu := n.dot(to_eye)
	if mu < REGOLITH_MU_MIN:
		n = (n + to_eye * (REGOLITH_MU_MIN - mu)).normalized()
		mu = REGOLITH_MU_MIN
	var mu0 := n.dot(to_light)
	if mu0 <= 0.0:
		return 0.0
	return 2.0 * mu0 / (mu0 + mu) * regolith_phase_factor(to_light.dot(to_eye))


## The Moon's light for Godot's lit materials (world.gd's second DirectionalLight3D) at an Earth-fixed point:
## [colour, energy] as sun_light_at, with the Moon's direction and its relative illuminance; [WHITE, 0] while the
## moonlight is inactive (daylight adaptation, no Moon, the tables not loaded).
func moon_light_at(position_itrf_m: Vector3) -> Array:
	if not ready or not moonlight_active or _moon_direction_fixed == Vector3.ZERO:
		return [Color.WHITE, 0.0]
	# No probe key: S is baked toward the Sun only, the Moon takes the grid mean along its own direction.
	return sun_light_at(position_itrf_m, _moon_direction_fixed, moonlight_relative_value, "")


## Night vision's state into the shaders and the post step: the rods' scale and the cones' chroma factor
## (VisionEffect: c' = C c + s t (w . c), C c = Y + chroma (c - Y)) and the inverse for the display-referred emitters
## (vision.gdshaderinc, Sherman-Morrison: c = q - u (w . q), q = C^-1 c', u = s C^-1 t / (1 + s w . C^-1 t)).
func _set_vision(scale: float, chroma_factor := 1.0) -> void:
	var tint := EyeAdaptation.ROD_TINT
	var weights := EyeAdaptation.SCOTOPIC_WEIGHTS
	var inverse := Vector3.ZERO
	if scale > 0.0:
		var tint_q := EyeAdaptation.chroma_inverse(tint, chroma_factor)
		inverse = tint_q * (scale / (1.0 + scale * weights.dot(tint_q)))
	ShaderGlobals.set_value(&"vision_rod_weights", weights)
	ShaderGlobals.set_value(&"vision_inverse_tint", inverse)
	ShaderGlobals.set_value(&"vision_chroma", chroma_factor)
	vision_effect.set_rods(scale, chroma_factor)


## [call] The lunar ground sphere of the eye adaptation (m about the Moon's centre) for an eye this far from it: the
## mean sphere, or MOON_EYE_MIN_HEIGHT_M below the eye when the eye stands below it (terrain under the mean radius).
static func _moon_ground_radius(eye_distance_m: float) -> float:
	return minf(MOON_RADIUS_M, eye_distance_m - MOON_EYE_MIN_HEIGHT_M)


## Whether the lunar terrain's horizon (MoonSunHorizon) decides the Sun at a point: within
## MOON_TERRAIN_SUN_MAX_DISTANCE_M of the Moon's centre (any height; terrain above the mean sphere hides the Sun from
## its own ground, which the mean-sphere fraction cannot know).
static func moon_terrain_sun_applies(point_m: Vector3, moon_m: Vector3) -> bool:
	return (moon_m - point_m).length() < MOON_TERRAIN_SUN_MAX_DISTANCE_M


## The Sun's visible fraction at a point near the Moon, the one combination of the vehicles' light (world.gd) and the eye
## adaptation: terrain_fraction (MoonSunHorizon.sun_visible_fraction; NAN: none) x the Earth's disc in front of the Sun
## (the terrain knows only the Moon), else moon_visible_fraction. Vectors Earth-centred, any one frame, m.
static func sun_visible_near_moon(point_m: Vector3, sun_m: Vector3, moon_m: Vector3, mean_sphere_visible: float,
		terrain_fraction: float) -> float:
	if not is_finite(terrain_fraction):
		return moon_visible_fraction(point_m, sun_m, moon_m, mean_sphere_visible)
	return terrain_fraction * (1.0 - occulted_fraction(sun_m - point_m, SUN_RADIUS_M, -point_m, EARTH_MEAN_RADIUS_M))


## The Sun's visible fraction past the Moon at a point (vectors in any one frame and origin, m) for the eye adaptation
## and the vehicles' Sun light (world.gd): mean_sphere_visible is the core's (StarsimCore.solar_eclipse_fraction_gcrf,
## the Moon's mean sphere); a point within MOON_EYE_MIN_HEIGHT_M of that sphere or below it - on lunar terrain under the
## mean radius, where the mean sphere would hide the Sun - gets it on its own ground sphere (_moon_ground_radius).
## Near the Moon the callers use sun_visible_near_moon (the terrain's horizon replaces this).
static func moon_visible_fraction(point_m: Vector3, sun_m: Vector3, moon_m: Vector3, mean_sphere_visible: float) -> float:
	var ground_m := _moon_ground_radius((moon_m - point_m).length())
	if ground_m >= MOON_RADIUS_M:
		return mean_sphere_visible
	return 1.0 - occulted_fraction(sun_m - point_m, SUN_RADIUS_M, moon_m - point_m, ground_m)


## Earthshine at the Moon (its centre; the near side's ground is 0.5 % closer, left out): the illuminance on a plate
## facing the Earth over the top-of-atmosphere solar illuminance at 1 AU [D]. The Earth a Lambertian sphere of
## EARTH_BOND_ALBEDO (geometric albedo LAMBERT_GEOMETRIC_PER_BOND x A) at the phase angle a (Sun - Earth - Moon):
## p (R / d)^2 (sin a + (pi - a) cos a) / pi / d_AU^2 - pi x A x the Lambert disc's plate luminance, the quantity the eye
## adaptation's Earth in view approximates (its 13-direction cap reads 0.8 x at full and ~0.4 x at half Earth from the
## Moon, hence the closed form here). Full Earth at 384 400 km and 1 AU: 5.6e-5 (7.2 lux; measured estimates of the full
## earthshine run ~7-15 lux with the Earth's real, non-Lambertian phase curve). Clouds, oceans' glint, city lights: not
## modelled. Vectors from the Earth's centre, m, one frame.
static func earthshine_relative(moon_from_earth_m: Vector3, sun_from_earth_m: Vector3) -> float:
	var d_au := sun_from_earth_m.length() / ASTRONOMICAL_UNIT_M
	var a := moon_from_earth_m.angle_to(sun_from_earth_m)
	var phase := (sin(a) + (PI - a) * cos(a)) / PI
	var view := EARTH_MEAN_RADIUS_M / moon_from_earth_m.length()
	return LAMBERT_GEOMETRIC_PER_BOND * EARTH_BOND_ALBEDO * view * view * phase / (d_au * d_au)


## The Moon in view (cd/m^2 at the true Sun distance): the regolith's luminance as the Moon shaders draw it, averaged
## cosine-weighted over the lunar cap below the eye (MOON_VIEW_RINGS rings uniform in sin^2 of the nadir angle, each
## MOON_VIEW_AZIMUTHS directions), times the cap's view factor (R / r)^2 - the analogue of _earth_in_view_luminance.
## Both lights through the one regolith reflectance (regolith_reflectance, moon_regolith.gdshaderinc; review I3):
## albedo x regolith_scale x E / pi x LS x f(g), E the Sun's (x the solar disc the Earth leaves at the ground point: a
## lunar eclipse; the reddened umbra glow of the shaders is left out) and the earthshine's (earthshine_relative).
## albedo: MOON_MEAN_TEXTURE_ALBEDO [call: the map's mean, not the texel under the eye]. Cast shadows: none (as the
## shaders). Vectors Earth-fixed, m.
func _moon_in_view_luminance(eye_itrf_m: Vector3, moon_itrf_m: Vector3, sun_itrf_m: Vector3, radius: float,
		earthshine_lux: float, d_au: float) -> float:
	var eye := eye_itrf_m - moon_itrf_m  # from the Moon's centre
	var r := eye.length()
	var nadir := -eye / r
	var east := nadir.cross(Vector3(0, 0, 1) if absf(nadir.z) < 0.9 else Vector3(1, 0, 0)).normalized()
	var north := nadir.cross(east)
	var sin_cap := radius / r
	var regolith := MOON_MEAN_TEXTURE_ALBEDO * regolith_scale(float(parameters["toa_solar_illuminance_lux"]))
	var sun_scale := regolith * reference_luminance(d_au)
	var earth_scale := regolith * earthshine_lux / PI
	var total := 0.0
	for ring in range(MOON_VIEW_RINGS):
		var sin_theta := sin_cap * sqrt((ring + 0.5) / MOON_VIEW_RINGS)
		var cos_theta := sqrt(maxf(1.0 - sin_theta * sin_theta, 0.0))
		for k in range(MOON_VIEW_AZIMUTHS):
			var azimuth := TAU * (k + 0.5 * (ring % 2)) / MOON_VIEW_AZIMUTHS
			var v := nadir * cos_theta + (east * cos(azimuth) + north * sin(azimuth)) * sin_theta
			var along := eye.dot(v)
			var hit := eye + v * (-along - sqrt(maxf(along * along - (r * r - radius * radius), 0.0)))
			var n := hit.normalized()
			var ground_itrf := moon_itrf_m + hit
			var sunlit := regolith_reflectance(n, (sun_itrf_m - ground_itrf).normalized(), -v)
			var luminance := 0.0
			if sunlit > 0.0:
				luminance += sun_scale * sunlit * (1.0 - occulted_fraction(sun_itrf_m - ground_itrf, SUN_RADIUS_M,
						-ground_itrf, EARTH_MEAN_RADIUS_M))
			luminance += earth_scale * regolith_reflectance(n, -ground_itrf.normalized(), -v)
			total += luminance
	return total / (MOON_VIEW_RINGS * MOON_VIEW_AZIMUTHS) * sin_cap * sin_cap


## Fraction of a light source's disc hidden by an occluding sphere, seen from the observer (vectors from it, m). Mirror
## of core/src/environment/eclipse.cpp (Montenbruck & Gill 3.85-3.90) and shaders/eclipse.gdshaderinc: keep in sync.
static func occulted_fraction(to_source: Vector3, source_radius: float, to_occluder: Vector3,
		occluder_radius: float) -> float:
	var source_distance := to_source.length()
	var occluder_distance := to_occluder.length()
	if occluder_distance <= occluder_radius:
		return 1.0
	if occluder_distance > source_distance:
		return 0.0
	var a := asin(minf(1.0, source_radius / source_distance))
	var b := asin(occluder_radius / occluder_distance)
	var c := atan2(to_source.cross(to_occluder).length(), to_source.dot(to_occluder))
	if c >= a + b:
		return 0.0
	if c <= b - a:
		return 1.0
	if c <= a - b:
		return (b * b) / (a * a)
	var x := (c * c + a * a - b * b) / (2.0 * c)
	var y := sqrt(maxf(0.0, a * a - x * x))
	var area := a * a * acos(clampf(x / a, -1.0, 1.0)) + b * b * acos(clampf((c - x) / b, -1.0, 1.0)) - c * y
	return clampf(area / (PI * a * a), 0.0, 1.0)


## The "up" part of adaptation_luminance (RGB luminance channels, cd/m^2 at 1 AU); sun_direction_itrf: the light
## (the Sun, or the Moon for the moonlit Earth: then x its relative illuminance by the caller).
func _earth_in_view_luminance(eye_itrf_m: Vector3, sun_direction_itrf: Vector3) -> Vector3:
	var radius := float(parameters["bottom_radius_m"])
	var eye := _stretch(eye_itrf_m)  # the model space: the ground a sphere of radius `radius`
	var r := eye.length()
	var nadir := -eye / r
	var east := nadir.cross(Vector3(0, 0, 1) if absf(nadir.z) < 0.9 else Vector3(1, 0, 0)).normalized()
	var north := nadir.cross(east)
	var disc := asin(clampf(radius / r, 0.0, 1.0))  # the Earth disc's angular radius about the nadir
	var limb_radius := radius + LIMB_SAMPLE_HEIGHT_M
	var limb := asin(limb_radius / r) if r > limb_radius else PI * 0.5 + deg_to_rad(2.0)
	var total := _view_luminance(eye_itrf_m, eye, nadir, radius, sun_direction_itrf) * 0.25
	for k in range(6):
		var azimuth := TAU * k / 6.0
		var side := east * cos(azimuth) + north * sin(azimuth)
		for ring: Array in [[0.6 * disc, 0.5 / 6.0], [limb, 0.25 / 6.0]]:
			var angle: float = ring[0]
			var v := (nadir * cos(angle) + side * sin(angle)).normalized()
			total += _view_luminance(eye_itrf_m, eye, v, radius, sun_direction_itrf) * float(ring[1])
	return total * (radius / r) * (radius / r)


## One view direction of _earth_in_view_luminance (model-space unit vector v from the model-space eye), RGB.
func _view_luminance(eye_itrf_m: Vector3, eye: Vector3, v: Vector3, radius: float, sun_itrf: Vector3) -> Vector3:
	var along := eye.dot(v)
	var discriminant := along * along - (eye.length_squared() - radius * radius)
	if along < 0.0 and discriminant >= 0.0:
		var hit := _unstretch(eye + v * (-along - sqrt(discriminant)))
		var lux: Dictionary = _core.atmosphere_illuminance(hit, _geodetic_up(hit), sun_itrf)
		var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(eye_itrf_m, hit, sun_itrf)
		var t: Vector3 = to_point["transmittance"]
		var lit: Vector3 = lux["sun_lux"] + lux["sky_lux"]
		var seen: Vector3 = lit / PI * t + to_point["luminance"]
		return seen * _visible_at(hit)
	var view := _unstretch(v).normalized()
	if not is_nan(_tdb_s):
		# The Moon's penumbra on the atmosphere: the sky along the ray segmented and weighted by the Sun's visible
		# fraction (as _eclipsed_sky_fluence). One factor at the ray's closest approach - behind a ground eye looking
		# up - let the uneclipsed horizon sky beyond the umbra set the eye in totality at ~56 cd/m^2 (the real
		# totality sky: ~1-16), which hid the magnitude-1 stars once the stars read the adaptation (final review I1).
		var exit := _top_exit_distance(eye_itrf_m, view)
		if exit > 0.0:
			return _eclipsed_in_scatter(eye_itrf_m, view, exit, false, sun_itrf, [])
	var sky: Vector3 = _core.atmosphere_sky_luminance(eye_itrf_m, view, sun_itrf)["luminance"]
	return sky * _visible_at(_unstretch(eye - v * along))  # the ray's closest approach


## Distance (m) along the Earth-fixed unit ray from the point to where it leaves the top of the atmosphere (the
## sphere of _eclipsed_sky_fluence: the model's top height above the WGS84 semi-major axis); 0 if it never crosses it.
func _top_exit_distance(point_itrf_m: Vector3, v: Vector3) -> float:
	var top := float(parameters["top_radius_m"]) - float(parameters["bottom_radius_m"]) + WGS84_A_M
	var along := point_itrf_m.dot(v)
	var discriminant := along * along - (point_itrf_m.length_squared() - top * top)
	return maxf(-along + sqrt(discriminant), 0.0) if discriminant >= 0.0 else 0.0


## The in-scattered sky along the ray from the point (Earth-fixed unit v) up to d_end (cd/m^2 per channel at 1 AU) with
## the Moon's shadow: UMBRA_SEGMENTS segments at d_end (i/n)^2, each segment's in-scatter (differences of
## StarsimCore.atmosphere_sky_luminance_to_point; the last one to the sky's end unless the ray ends on the ground)
## weighted by the Sun's visible fraction at its midpoint. instant: see _visible_at.
func _eclipsed_in_scatter(point_itrf_m: Vector3, v: Vector3, d_end: float, ends_on_ground: bool, sun_itrf: Vector3,
		instant: Array) -> Vector3:
	var previous := Vector3.ZERO
	var eclipsed := Vector3.ZERO
	for i in range(1, UMBRA_SEGMENTS + 1):
		var f := float(i) / UMBRA_SEGMENTS
		var cumulative: Vector3
		if i < UMBRA_SEGMENTS or ends_on_ground:
			cumulative = _core.atmosphere_sky_luminance_to_point(point_itrf_m, point_itrf_m + v * (d_end * f * f),
					sun_itrf)["luminance"]
		else:
			cumulative = _core.atmosphere_sky_luminance(point_itrf_m, v, sun_itrf)["luminance"]
		var f_mid := (float(i) - 0.5) / UMBRA_SEGMENTS
		eclipsed += (cumulative - previous) * _visible_at(point_itrf_m + v * (d_end * f_mid * f_mid), instant)
		previous = cumulative
	return eclipsed


## The Sun's unocculted fraction (Moon) at an Earth-fixed point at this update's instant (1 without one); instant:
## [tdb_s, fixed_to_gcrf, earth_center_gcrf_m] captured for a worker, [] = this update's.
func _visible_at(point_itrf_m: Vector3, instant := []) -> float:
	var t: float = _tdb_s if instant.is_empty() else instant[0]
	if is_nan(t):
		return 1.0
	var rotation: Basis = _fixed_to_gcrf if instant.is_empty() else instant[1]
	var center: Vector3 = _earth_center_gcrf_m if instant.is_empty() else instant[2]
	return 1.0 - _core.solar_eclipse_fraction_gcrf(rotation * point_itrf_m + center, t)


## The light in the umbra air (atmosphere_umbra_fluence_*, the second-order source of the eclipse sky,
## atmosphere.gdshaderinc): at the ground point of the umbra axis (or the surface point nearest to it) and
## UMBRA_HEIGHTS_M above, the fluence (scalar irradiance) of the eclipsed sky - its luminance along the zenith and four
## azimuths on each ring of UMBRA_ELEVATIONS_DEG (UMBRA_SEGMENTS segments at d (i/n)^2, each weighted by the visible fraction at its midpoint; the
## segments' in-scatter as differences of StarsimCore.atmosphere_sky_luminance_to_point), times the solid angles of the
## elevation bands (UMBRA_BAND_EDGES_DEG); above the ground also the lower hemisphere (the air below and the lit ground
## beyond the umbra). The low ring dominates: in totality the light reaches the umbra
## nearly horizontally from the sunlit air ~100 km away. Once per simulated UMBRA_UPDATE_S (the umbra moves ~1 km/s).
func _update_umbra_light(sun_fixed_m: Vector3, moon_fixed_m: Vector3, sun_direction_itrf: Vector3) -> void:
	if _umbra_task_id >= 0:
		if not WorkerThreadPool.is_task_completed(_umbra_task_id):
			return
		WorkerThreadPool.wait_for_task_completion(_umbra_task_id)
		_umbra_task_id = -1
		_upload_umbra_light()
	if is_nan(_tdb_s) or absf(_tdb_s - _umbra_tdb_s) < UMBRA_UPDATE_S:
		return
	_umbra_tdb_s = _tdb_s
	var axis := (moon_fixed_m - sun_fixed_m).normalized()
	var b := moon_fixed_m.dot(axis)
	var c := moon_fixed_m.length_squared() - WGS84_A_M * WGS84_A_M
	var ground: Vector3
	if b * b - c >= 0.0:
		ground = moon_fixed_m + axis * (-b - sqrt(b * b - c))
	else:  # the umbra misses the Earth: the surface point nearest to the axis
		ground = (moon_fixed_m - axis * b).normalized() * WGS84_A_M
	# ~10 ms of core calls: on a worker (the core's lookups are const, CSPICE is serialised by the core), uploaded on a
	# later frame; synchronous for tests that render one frame later (umbra_light_synchronous).
	var instant := [_tdb_s, _fixed_to_gcrf, _earth_center_gcrf_m]
	var job := _compute_umbra_light.bind(ground, sun_direction_itrf, instant)
	if umbra_light_synchronous:
		job.call()
		_upload_umbra_light()
	else:
		_umbra_task_id = WorkerThreadPool.add_task(job, false, "umbra light")


## Worker part of _update_umbra_light: the fluences at UMBRA_HEIGHTS_M into _umbra_result.
func _compute_umbra_light(ground: Vector3, sun_direction_itrf: Vector3, instant: Array) -> void:
	var up := _geodetic_up(ground)
	var result: Array[Vector3] = []
	for k in range(3):
		var height: float = UMBRA_HEIGHTS_M[k]
		result.append(_eclipsed_sky_fluence(ground + up * height, up, sun_direction_itrf, ground.length(), instant))
	_umbra_result = result


## Main thread: the computed fluences into the shader globals.
func _upload_umbra_light() -> void:
	var heights := Vector3.ZERO
	var model_offset := WGS84_A_M - float(parameters["bottom_radius_m"])  # ellipsoid above the model ground
	for k in range(3):
		umbra_fluence[k] = _umbra_result[k]
		heights[k] = float(UMBRA_HEIGHTS_M[k]) + model_offset
		ShaderGlobals.set_value(StringName("atmosphere_umbra_fluence_%d" % k), _umbra_result[k])
	ShaderGlobals.set_value(&"atmosphere_umbra_heights_m", heights)


## The eclipsed sky's fluence at an Earth-fixed point (cd/m^2 x sr per channel at 1 AU; see _update_umbra_light);
## ground_radius_m: the Earth's radius under the point. Points above the ground also take the lower hemisphere (the
## mirrored rings): the air down to the ground and the ground itself (the tables' ground_albedo, lit by the Sun and the
## sky, x its visible fraction).
func _eclipsed_sky_fluence(point_itrf_m: Vector3, up: Vector3, sun_itrf: Vector3, ground_radius_m: float,
		instant: Array) -> Vector3:
	var east := up.cross(Vector3(0, 0, 1) if absf(up.z) < 0.9 else Vector3(1, 0, 0)).normalized()
	var north := up.cross(east)
	var top := float(parameters["top_radius_m"]) - float(parameters["bottom_radius_m"]) + WGS84_A_M
	var fluence := Vector3.ZERO
	var signs := [1.0, -1.0] if point_itrf_m.length() - ground_radius_m > 10.0 else [1.0]
	for hemisphere: float in signs:
		for k in range(1 + 4 * (UMBRA_ELEVATIONS_DEG.size() - 1)):
			var ring := 0 if k == 0 else 1 + (k - 1) / 4
			var elevation: float = UMBRA_ELEVATIONS_DEG[ring] * hemisphere
			var upper: float = 90.0 if ring == 0 else UMBRA_BAND_EDGES_DEG[ring - 1]
			var solid_angle := TAU * (sin(deg_to_rad(upper)) - sin(deg_to_rad(UMBRA_BAND_EDGES_DEG[ring])))
			if ring > 0:
				solid_angle /= 4.0
			var azimuth := (k % 4) * PI * 0.5
			var v := up * hemisphere
			if k > 0:
				v = (up * sin(deg_to_rad(elevation)) + (north * cos(azimuth) + east * sin(azimuth))
						* cos(deg_to_rad(elevation))).normalized()
			var along := point_itrf_m.dot(v)
			var d_end := -along + sqrt(along * along - (point_itrf_m.length_squared() - top * top))
			var ground_discriminant := along * along - (point_itrf_m.length_squared() - ground_radius_m * ground_radius_m)
			var hits_ground := along < 0.0 and ground_discriminant >= 0.0
			if hits_ground:
				d_end = -along - sqrt(ground_discriminant)
			var eclipsed := _eclipsed_in_scatter(point_itrf_m, v, d_end, hits_ground, sun_itrf, instant)
			if hits_ground:
				var hit := point_itrf_m + v * d_end
				var lux: Dictionary = _core.atmosphere_illuminance(hit, _geodetic_up(hit), sun_itrf)
				var t: Vector3 = _core.atmosphere_sky_luminance_to_point(point_itrf_m, hit, sun_itrf)["transmittance"]
				var lit: Vector3 = lux["sun_lux"] + lux["sky_lux"]
				eclipsed += lit * (float(parameters["ground_albedo"]) / PI) * t * _visible_at(hit, instant)
			fluence += eclipsed * solid_angle
	return fluence


static func _geodetic_up(point_itrf_m: Vector3) -> Vector3:
	return Vector3(point_itrf_m.x, point_itrf_m.y, point_itrf_m.z * POLAR_STRETCH * POLAR_STRETCH).normalized()


static func _stretch(v: Vector3) -> Vector3:
	return Vector3(v.x, v.y, v.z * POLAR_STRETCH)


static func _unstretch(v: Vector3) -> Vector3:
	return Vector3(v.x, v.y, v.z / POLAR_STRETCH)


## Eye adaptation's reference luminance (cd/m^2): the white Lambertian under the top-of-atmosphere Sun at d_AU, the state
## DAY_EXPOSURE is defined for (same channels and weights as adaptation_luminance).
func reference_luminance(d_au: float) -> float:
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	return (solar * sun_factor).dot(LUMINANCE_WEIGHTS) / (PI * d_au * d_au)


## True when the Moon's penumbral cone reaches the sphere of radius_m about the Earth's centre (vectors from it, m): the
## cone's half angle asin((R_sun + R_moon) / |Sun - Moon|), its apex on the Sun's side of the Moon.
static func penumbra_touches(sun_m: Vector3, moon_m: Vector3, radius_m: float) -> bool:
	var axis := moon_m - sun_m
	var sun_moon_m := axis.length()
	axis /= sun_moon_m
	var along := -moon_m.dot(axis)  # the Earth's centre beyond the Moon along the shadow axis
	if along <= 0.0:
		return false
	var off_axis := (-moon_m - axis * along).length()
	var sin_half := (SUN_RADIUS_M + MOON_RADIUS_M) / sun_moon_m
	var apex_m := MOON_RADIUS_M / sin_half  # apex distance on the Sun's side of the Moon
	var cone_radius := (apex_m + along) * tan(asin(sin_half))
	# The Earth's sphere touches the cone where its distance from the axis, measured across the cone wall, is below
	# its radius: off_axis - cone_radius < radius / cos(half angle).
	return off_axis - cone_radius < radius_m / cos(asin(sin_half))


## The next update takes the eye as adapted to its view (a new view, a test's fixed instant).
func snap_adaptation() -> void:
	_eye.snap()


## The Sun light of Godot's lit materials (vehicles, structures: world.gd's DirectionalLight3D) at an Earth-fixed point:
## [colour, energy]. Per channel it is the scene value atm_sun_at gives the Earth's surfaces (one exposure): the TOA
## solar illuminance of the tables (its own colour in the RGB luminance space) x the transmittance to the Sun
## (StarsimCore.atmosphere_sun_transmittance: red and dim at sunset, zero below the horizon or in the Earth's shadow) x
## luminance_scale / pi x visible_fraction (the unocculted part of the solar disc, Moon) - a white Lambertian at normal
## incidence renders it; in luminance that is DAY_EXPOSURE / d_AU^2 x T under the TOA Sun. Godot multiplies a
## directional light's energy by pi and its Lambert BRDF divides by pi, so energy = the brightest channel's value and
## colour = the channels over it, sRGB-encoded (Godot linearises light colours). Before the tables are loaded: the
## vacuum Sun (white, DAY_EXPOSURE). probe_key: the drawn shapes' S probe (Sun direction only; "" for other lights).
func sun_light_at(position_itrf_m: Vector3, sun_direction_itrf: Vector3, visible_fraction: float,
		probe_key := "sun_light") -> Array:
	var scene := Vector3.ONE * DAY_EXPOSURE
	if ready:
		var transmittance := _core.atmosphere_sun_transmittance(position_itrf_m, sun_direction_itrf)
		var solar: Vector3 = parameters["solar_illuminance_rgb"]
		var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
		scene = solar * sun_factor * transmittance * (luminance_scale / PI)
		# The direct beam through the clouds (the drawn shapes' S, else the grid mean along the ray; 1 with weather off).
		scene *= cloud_light_at(position_itrf_m, sun_direction_itrf, probe_key).x
	scene *= visible_fraction
	var peak := maxf(scene.x, maxf(scene.y, scene.z))
	var color := Color.WHITE
	if peak > 0.0:
		color = Color(scene.x / peak, scene.y / peak, scene.z / peak).linear_to_srgb()
	return [color, peak]


## The clouds' light at an Earth-fixed point for the light from light_direction_itrf (StarsimCore.weather_cloud_light,
## the active weather pair's grid mean, Task 9): x the direct beam's transmittance, y the light's diffuse part through
## the column per unit of its horizontal irradiance, z the sky's diffuse transmittance; (1, 0, 1) without weather.
## sun_probe (the Sun's direct beam only): x from the drawn clouds' shapes instead (direct_sun under that key, 2026-10-30)
## where they have a value. Cached per frame (the same point and direction several times: Sun light, ambient,
## adaptation).
func cloud_light_at(position_itrf_m: Vector3, light_direction_itrf: Vector3, sun_probe := "") -> Vector3:
	var key := [position_itrf_m, light_direction_itrf, sun_probe, Engine.get_process_frames()]
	if _cloud_cache.has(key):
		return _cloud_cache[key]
	if _cloud_cache.size() > 8:
		_cloud_cache.clear()
	# Only while the clouds are drawn (cloud_enabled: noise ready, weather shown, within the scaled-space limit): the
	# vehicles and the eye are dimmed exactly when the GPU's clouds are there (Task 9 fix 1; one frame behind at most).
	var shown: bool = ShaderGlobals.value(&"cloud_enabled")
	var light: Dictionary = _core.weather_cloud_light(position_itrf_m, light_direction_itrf) if _core != null and shown else {}
	var result := Vector3(float(light.get("sun_transmittance", 1.0)), float(light.get("sun_diffuse", 0.0)),
			float(light.get("sky_factor", 1.0)))
	if shown and sun_probe != "" and direct_sun.is_valid():
		var shaped: float = direct_sun.call(sun_probe, position_itrf_m)
		if not is_nan(shaped):
			result.x = shaped
	_cloud_cache[key] = result
	return result


## The factor on Godot's sky ambient (Environment.ambient_light_energy) at an Earth-fixed point under the clouds: the
## diffuse light there over the clear sky's, (T_d E_sky + T_sd E_sun,h) / E_sky - above 1 under a thin bright deck,
## exactly 1 with weather off (T_d = 1, T_sd = 0). Reflections keep the clear sky (a documented limit). Task 9.
func cloud_ambient_factor(position_itrf_m: Vector3, sun_direction_itrf: Vector3) -> float:
	var clouds := cloud_light_at(position_itrf_m, sun_direction_itrf)
	if clouds.y <= 0.0 or not ready:
		return clouds.z
	var up := _geodetic_up(position_itrf_m)
	var mu0 := sun_direction_itrf.dot(up)
	var lux: Dictionary = _core.atmosphere_illuminance(position_itrf_m, up, sun_direction_itrf)
	var sky := (lux["sky_lux"] as Vector3).dot(LUMINANCE_WEIGHTS)
	if mu0 <= 0.0 or sky <= 0.0:
		return clouds.z
	var sun_horizontal := (lux["sun_lux"] as Vector3).dot(LUMINANCE_WEIGHTS)
	return clouds.z + clouds.y * sun_horizontal / sky


func _load_task(config_dir: String, cache_dir: String) -> void:
	var result: Dictionary = _core.load_atmosphere(config_dir, cache_dir)
	if result.get("ok", false):
		# Images are built here (the large copies), textures on the main thread (_upload).
		var t_size: Vector2i = result["transmittance_size"]
		var s_size: Vector3i = result["scattering_size"]
		var i_size: Vector2i = result["irradiance_size"]
		result["transmittance_image"] = _image_2d(result["transmittance"], t_size, Image.FORMAT_RGBAF)
		result["irradiance_image"] = _image_2d(result["irradiance"], i_size)
		result["scattering_images"] = _images_3d(result["scattering"], s_size)
		result["single_mie_images"] = _images_3d(result["single_mie"], s_size)
		result["single_rayleigh_images"] = _images_3d(result["single_rayleigh"], s_size)
		for key in ["transmittance", "irradiance", "scattering", "single_mie", "single_rayleigh"]:
			result.erase(key)
	_result = result


func _image_2d(data: PackedFloat32Array, size: Vector2i, format := table_format) -> Image:
	var image := Image.create_from_data(size.x, size.y, false, Image.FORMAT_RGBAF, data.to_byte_array())
	if format != Image.FORMAT_RGBAF:
		image.convert(format)
	return image


## One image per depth slice (z = r), x fastest within a slice (the core's layout).
func _images_3d(data: PackedFloat32Array, size: Vector3i) -> Array[Image]:
	var bytes := data.to_byte_array()
	var slice_bytes := size.x * size.y * 4 * 4
	var images: Array[Image] = []
	for z in range(size.z):
		var image := Image.create_from_data(size.x, size.y, false, Image.FORMAT_RGBAF,
				bytes.slice(z * slice_bytes, (z + 1) * slice_bytes))
		if table_format != Image.FORMAT_RGBAF:
			image.convert(table_format)
		images.append(image)
	return images


func _upload(result: Dictionary) -> void:
	if not result.get("ok", false):
		status_text = str(result.get("error", "Atmosphäre konnte nicht geladen werden"))
		push_error("Atmosphere: " + status_text)
		return
	parameters = result["parameters"]
	var s_size: Vector3i = result["scattering_size"]
	var transmittance := ImageTexture.create_from_image(result["transmittance_image"])
	var irradiance := ImageTexture.create_from_image(result["irradiance_image"])
	var scattering := ImageTexture3D.new()
	scattering.create(table_format, s_size.x, s_size.y, s_size.z, false, result["scattering_images"])
	var single_mie := ImageTexture3D.new()
	single_mie.create(table_format, s_size.x, s_size.y, s_size.z, false, result["single_mie_images"])
	# The single Rayleigh part (static clouds Task 9b): the cloud pass's cloud shadows on the air.
	var single_rayleigh := ImageTexture3D.new()
	single_rayleigh.create(table_format, s_size.x, s_size.y, s_size.z, false, result["single_rayleigh_images"])
	_textures = [transmittance, irradiance, scattering, single_mie, single_rayleigh]
	var t_size: Vector2i = result["transmittance_size"]
	var i_size: Vector2i = result["irradiance_size"]
	var cells: Vector4i = result["scattering_cells"]  # (nu, mu_s, mu, r)
	ShaderGlobals.set_value(&"atmosphere_transmittance", transmittance)
	ShaderGlobals.set_value(&"atmosphere_irradiance", irradiance)
	ShaderGlobals.set_value(&"atmosphere_scattering", scattering)
	ShaderGlobals.set_value(&"atmosphere_single_mie", single_mie)
	ShaderGlobals.set_value(&"atmosphere_single_rayleigh", single_rayleigh)
	ShaderGlobals.set_value(&"atmosphere_bottom_radius_m", float(parameters["bottom_radius_m"]))
	ShaderGlobals.set_value(&"atmosphere_top_radius_m", float(parameters["top_radius_m"]))
	ShaderGlobals.set_value(&"atmosphere_mu_s_min", float(parameters["mu_s_min"]))
	ShaderGlobals.set_value(&"atmosphere_mie_phase_g", float(parameters["mie_phase_g"]))
	ShaderGlobals.set_value(&"atmosphere_sun_angular_radius_rad", float(parameters["sun_angular_radius_rad"]))
	ShaderGlobals.set_value(&"atmosphere_sky_radiance_to_luminance", parameters["sky_radiance_to_luminance"])
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	ShaderGlobals.set_value(&"atmosphere_sun_illuminance_lux", solar * sun_factor)
	ShaderGlobals.set_value(&"atmosphere_ground_albedo", float(parameters["ground_albedo"]))
	# A true luminance (cd/m^2, at the Sun's actual distance) -> scene value: luminance_scale without the 1 / d_AU^2
	# (that factor converts the tables' 1 AU values); the star threshold reads the sky's true luminance back with it.
	# Without the exposure gain until the first update sets both.
	scene_per_cd_m2 = DAY_EXPOSURE * PI / float(parameters["toa_solar_illuminance_lux"])
	ShaderGlobals.set_value(&"atmosphere_scene_per_cd_m2", scene_per_cd_m2)
	ShaderGlobals.set_value(&"atmosphere_scattering_size", Vector4(cells.x, cells.y, cells.z, cells.w))
	ShaderGlobals.set_value(&"atmosphere_table_sizes", Vector4(t_size.x, t_size.y, i_size.x, i_size.y))
	ShaderGlobals.set_value(&"atmosphere_enabled", true)
	ready = true
	status_text = ""
