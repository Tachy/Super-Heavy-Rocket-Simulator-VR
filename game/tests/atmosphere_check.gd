# GPU check of the atmosphere shader include (M8 atmosphere, Task 5): renders the sky (shaders/sky.gdshader, nothing
# else in the scene) into an HDR SubViewport for sample views, reads the float pixel on the view direction and compares
# it with the core's CPU evaluation (StarsimCore.atmosphere_sky_luminance x Atmosphere.luminance_scale) within 2 % per
# channel. Linear tone mapper, exposure 1, glow off: the read value is the scene value. Exit code 1 on failure.
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

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()
var _viewport := SubViewport.new()
var _camera := Camera3D.new()
var _earth_to_gcrf := Basis.IDENTITY
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
	var environment := Environment.new()
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
	_camera.current = true
	_viewport.add_child(_camera)


## Renders the sky for the camera (ITRF m) along view_itrf with the Sun along sun_itrf and compares the centre pixel.
func _sample(label: String, camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3) -> void:
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf)
	var cpu: Dictionary = _core.atmosphere_sky_luminance(camera_itrf_m, view_itrf, sun_itrf)
	var expected: Vector3 = cpu["luminance"] * _atmosphere.luminance_scale
	_compare(label, expected, got)


## Counts and prints one sample: every channel within TOLERANCE (relative; absolute below the floor).
func _compare(label: String, expected: Vector3, got: Vector3) -> void:
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
			ok = ok and absf(ratio - 1.0) <= TOLERANCE
		else:
			ok = ok and absf(g - e) <= TOLERANCE * floor_value
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
	_atmosphere.update(camera_gcrf, Vector3.ZERO, _earth_to_gcrf, _earth_to_gcrf * sun_itrf * AU_M)
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
