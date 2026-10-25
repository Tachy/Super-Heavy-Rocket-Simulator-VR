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
## eye_adaptation_enabled) multiplies all of it by one exposure gain (1 by day, up to EyeAdaptation.GAIN_MAX in twilight).

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
## [E] The ground's albedo seen from the umbra air: config/atmosphere/earth_optics.toml ground_albedo (the tables').
const UMBRA_GROUND_ALBEDO := 0.1
## [call] The eye adaptation's limb samples graze the Earth this high (m) above the model ground.
const LIMB_SAMPLE_HEIGHT_M := 10000.0

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
## The exposure gain of the eye adaptation this frame (1 without it), already in luminance_scale and scene_per_cd_m2;
## the shaders read it as atmosphere_exposure_gain for what is not lit through the tables (Moon, Sun disc).
var exposure_gain := 1.0
## A true luminance (cd/m^2, at the Sun's actual distance) -> scene value, incl. the exposure gain.
var scene_per_cd_m2 := 0.0
## Eclipse sky segments this frame (0: no penumbra on the Earth).
var eclipse_segments := 0
var _eye := EyeAdaptation.new()
## The eye adaptation's two parts at the last update (cd/m^2; adaptation_luminance), for tests and the report.
var last_adaptation_down_cd_m2 := 0.0
var last_adaptation_up_cd_m2 := 0.0
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
	RenderingServer.global_shader_parameter_set(&"atmosphere_enabled", false)
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


## Per frame: the camera, the Earth's centre and orientation (fixed -> GCRF) and the apparent Sun position (GCRF, as
## seen from the Earth's centre), all float64. The camera is differenced against the Earth's centre here, in float64.
## Eye adaptation (eye_adaptation_enabled): adaptation_dt_s is the time the viewer had to adapt since the last frame
## (world.gd: the longer of the frame time and the simulated time step), sun_visible_fraction the part of the solar disc
## the Moon leaves at the camera.
## moon_apparent_gcrf_m: the Moon's apparent position (GCRF, as seen from the Earth's centre) for the eclipse sky;
## Vector3.ZERO: no eclipse. tdb_s: the instant (the Moon's shadow on points other than the camera: the eye adaptation's
## view of the Earth, the umbra's light); NAN: no shadow there.
func update(camera_gcrf_m: Vector3, earth_center_gcrf_m: Vector3, earth_fixed_to_gcrf: Basis,
		sun_apparent_gcrf_m: Vector3, adaptation_dt_s := 0.0, sun_visible_fraction := 1.0,
		moon_apparent_gcrf_m := Vector3.ZERO, tdb_s := NAN) -> void:
	if not poll():
		return
	_tdb_s = tdb_s
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
	exposure_gain = 1.0
	if eye_adaptation_enabled:
		var target := EyeAdaptation.target_gain(adaptation_luminance(camera_fixed, sun_fixed, sun_visible_fraction, d_au),
				reference_luminance(d_au))
		exposure_gain = _eye.advance(target, adaptation_dt_s)
	var base := DAY_EXPOSURE * PI / float(parameters["toa_solar_illuminance_lux"])
	luminance_scale = base * exposure_gain / (d_au * d_au)
	scene_per_cd_m2 = base * exposure_gain
	RenderingServer.global_shader_parameter_set(&"atmosphere_scene_per_cd_m2", scene_per_cd_m2)
	RenderingServer.global_shader_parameter_set(&"atmosphere_exposure_gain", exposure_gain)
	eclipse_segments = 0
	if moon_apparent_gcrf_m != Vector3.ZERO:
		var moon_from_earth := moon_apparent_gcrf_m - earth_center_gcrf_m
		if penumbra_touches(sun_from_earth, moon_from_earth, float(parameters["top_radius_m"])):
			eclipse_segments = ECLIPSE_SEGMENTS
			_update_umbra_light(gcrf_to_fixed * sun_from_earth, gcrf_to_fixed * moon_from_earth, sun_fixed)
		RenderingServer.global_shader_parameter_set(&"atmosphere_moon_position",
				Frames.to_render(gcrf_to_fixed * moon_from_earth))
	RenderingServer.global_shader_parameter_set(&"atmosphere_sun_distance_m", sun_from_earth.length())
	RenderingServer.global_shader_parameter_set(&"atmosphere_eclipse_segments", eclipse_segments)
	if eclipse_segments == 0 and _umbra_tdb_s != -INF:
		_umbra_tdb_s = -INF
		for k in range(3):
			umbra_fluence[k] = Vector3.ZERO
			RenderingServer.global_shader_parameter_set(StringName("atmosphere_umbra_fluence_%d" % k), Vector3.ZERO)
	# Render world axes are GCRF in render axes; Earth-local = P R^T P^T (the transpose of the body's render basis).
	var world_to_earth := Frames.basis_to_render(earth_fixed_to_gcrf).transposed()
	RenderingServer.global_shader_parameter_set(&"atmosphere_camera_hi", split[0])
	RenderingServer.global_shader_parameter_set(&"atmosphere_camera_lo", split[1])
	RenderingServer.global_shader_parameter_set(&"atmosphere_camera_altitude_m", camera_altitude_m)
	RenderingServer.global_shader_parameter_set(&"atmosphere_world_to_earth", world_to_earth)
	RenderingServer.global_shader_parameter_set(&"atmosphere_sun_direction", sun_direction_local)
	RenderingServer.global_shader_parameter_set(&"atmosphere_luminance_scale", luminance_scale)


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
## BT.709 luminance of the RGB luminance channels; the tables' 1 AU values / d_AU^2.
func adaptation_luminance(eye_itrf_m: Vector3, sun_direction_itrf: Vector3, visible_fraction: float, d_au: float) -> float:
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	var transmittance := _core.atmosphere_sun_transmittance(eye_itrf_m, sun_direction_itrf)
	var up := _geodetic_up(eye_itrf_m)
	var sky: Vector3 = _core.atmosphere_illuminance(eye_itrf_m, up, sun_direction_itrf)["sky_lux"]
	var down := ((solar * sun_factor * transmittance + sky) * visible_fraction).dot(LUMINANCE_WEIGHTS) / PI
	last_adaptation_down_cd_m2 = down / (d_au * d_au)
	last_adaptation_up_cd_m2 = _earth_in_view_luminance(eye_itrf_m, sun_direction_itrf) / (d_au * d_au)
	return maxf(last_adaptation_down_cd_m2, last_adaptation_up_cd_m2)


## The "up" part of adaptation_luminance (cd/m^2 at 1 AU).
func _earth_in_view_luminance(eye_itrf_m: Vector3, sun_direction_itrf: Vector3) -> float:
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


## One view direction of _earth_in_view_luminance (model-space unit vector v from the model-space eye).
func _view_luminance(eye_itrf_m: Vector3, eye: Vector3, v: Vector3, radius: float, sun_itrf: Vector3) -> float:
	var along := eye.dot(v)
	var discriminant := along * along - (eye.length_squared() - radius * radius)
	if along < 0.0 and discriminant >= 0.0:
		var hit := _unstretch(eye + v * (-along - sqrt(discriminant)))
		var lux: Dictionary = _core.atmosphere_illuminance(hit, _geodetic_up(hit), sun_itrf)
		var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(eye_itrf_m, hit, sun_itrf)
		var t: Vector3 = to_point["transmittance"]
		var lit: Vector3 = lux["sun_lux"] + lux["sky_lux"]
		var seen: Vector3 = lit / PI * t + to_point["luminance"]
		return seen.dot(LUMINANCE_WEIGHTS) * _visible_at(hit)
	var view := _unstretch(v).normalized()
	var sky: Vector3 = _core.atmosphere_sky_luminance(eye_itrf_m, view, sun_itrf)["luminance"]
	return sky.dot(LUMINANCE_WEIGHTS) * _visible_at(_unstretch(eye - v * along))  # the ray's closest approach


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
		RenderingServer.global_shader_parameter_set(StringName("atmosphere_umbra_fluence_%d" % k), _umbra_result[k])
	RenderingServer.global_shader_parameter_set(&"atmosphere_umbra_heights_m", heights)


## The eclipsed sky's fluence at an Earth-fixed point (cd/m^2 x sr per channel at 1 AU; see _update_umbra_light);
## ground_radius_m: the Earth's radius under the point. Points above the ground also take the lower hemisphere (the
## mirrored rings): the air down to the ground and the ground itself (UMBRA_GROUND_ALBEDO, lit by the Sun and the sky,
## x its visible fraction).
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
			var previous := Vector3.ZERO
			var eclipsed := Vector3.ZERO
			for i in range(1, UMBRA_SEGMENTS + 1):
				var f := float(i) / UMBRA_SEGMENTS
				var cumulative: Vector3
				if i < UMBRA_SEGMENTS or hits_ground:
					cumulative = _core.atmosphere_sky_luminance_to_point(point_itrf_m, point_itrf_m + v * (d_end * f * f),
							sun_itrf)["luminance"]
				else:
					cumulative = _core.atmosphere_sky_luminance(point_itrf_m, v, sun_itrf)["luminance"]
				var f_mid := (float(i) - 0.5) / UMBRA_SEGMENTS
				eclipsed += (cumulative - previous) * _visible_at(point_itrf_m + v * (d_end * f_mid * f_mid), instant)
				previous = cumulative
			if hits_ground:
				var hit := point_itrf_m + v * d_end
				var lux: Dictionary = _core.atmosphere_illuminance(hit, _geodetic_up(hit), sun_itrf)
				var t: Vector3 = _core.atmosphere_sky_luminance_to_point(point_itrf_m, hit, sun_itrf)["transmittance"]
				var lit: Vector3 = lux["sun_lux"] + lux["sky_lux"]
				eclipsed += lit * (UMBRA_GROUND_ALBEDO / PI) * t * _visible_at(hit, instant)
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
## vacuum Sun (white, DAY_EXPOSURE).
func sun_light_at(position_itrf_m: Vector3, sun_direction_itrf: Vector3, visible_fraction: float) -> Array:
	var scene := Vector3.ONE * DAY_EXPOSURE
	if ready:
		var transmittance := _core.atmosphere_sun_transmittance(position_itrf_m, sun_direction_itrf)
		var solar: Vector3 = parameters["solar_illuminance_rgb"]
		var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
		scene = solar * sun_factor * transmittance * (luminance_scale / PI)
	scene *= visible_fraction
	var peak := maxf(scene.x, maxf(scene.y, scene.z))
	var color := Color.WHITE
	if peak > 0.0:
		color = Color(scene.x / peak, scene.y / peak, scene.z / peak).linear_to_srgb()
	return [color, peak]


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
		for key in ["transmittance", "irradiance", "scattering", "single_mie"]:
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
	_textures = [transmittance, irradiance, scattering, single_mie]
	var t_size: Vector2i = result["transmittance_size"]
	var i_size: Vector2i = result["irradiance_size"]
	var cells: Vector4i = result["scattering_cells"]  # (nu, mu_s, mu, r)
	var rs := RenderingServer
	rs.global_shader_parameter_set(&"atmosphere_transmittance", transmittance)
	rs.global_shader_parameter_set(&"atmosphere_irradiance", irradiance)
	rs.global_shader_parameter_set(&"atmosphere_scattering", scattering)
	rs.global_shader_parameter_set(&"atmosphere_single_mie", single_mie)
	rs.global_shader_parameter_set(&"atmosphere_bottom_radius_m", float(parameters["bottom_radius_m"]))
	rs.global_shader_parameter_set(&"atmosphere_top_radius_m", float(parameters["top_radius_m"]))
	rs.global_shader_parameter_set(&"atmosphere_mu_s_min", float(parameters["mu_s_min"]))
	rs.global_shader_parameter_set(&"atmosphere_mie_phase_g", float(parameters["mie_phase_g"]))
	rs.global_shader_parameter_set(&"atmosphere_sun_angular_radius_rad", float(parameters["sun_angular_radius_rad"]))
	rs.global_shader_parameter_set(&"atmosphere_sky_radiance_to_luminance", parameters["sky_radiance_to_luminance"])
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	rs.global_shader_parameter_set(&"atmosphere_sun_illuminance_lux", solar * sun_factor)
	# A true luminance (cd/m^2, at the Sun's actual distance) -> scene value: luminance_scale without the 1 / d_AU^2
	# (that factor converts the tables' 1 AU values); the star threshold reads the sky's true luminance back with it.
	# Without the exposure gain until the first update sets both.
	scene_per_cd_m2 = DAY_EXPOSURE * PI / float(parameters["toa_solar_illuminance_lux"])
	rs.global_shader_parameter_set(&"atmosphere_scene_per_cd_m2", scene_per_cd_m2)
	rs.global_shader_parameter_set(&"atmosphere_scattering_size", Vector4(cells.x, cells.y, cells.z, cells.w))
	rs.global_shader_parameter_set(&"atmosphere_table_sizes", Vector4(t_size.x, t_size.y, i_size.x, i_size.y))
	rs.global_shader_parameter_set(&"atmosphere_enabled", true)
	ready = true
	status_text = ""
