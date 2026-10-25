# GPU check of the volumetric clouds (M8 weather and clouds, Task 8): renders the cloud layer (shaders/cloud_layer.gdshader
# through CloudLayer) over the sky into an HDR SubViewport and compares the centre pixel with the core's CPU reference.
# Synthetic snapshots from StarsimCore.weather_write_slab_snapshot (uniform slabs, 1 deg grid, ground at 0 m):
#   1. background transmittance (debug output 1, grid-mean field) seen from below, inside and above a slab of vertical tau ~1 vs the
#      core's Beer-Lambert along the same ray (weather_cloud_light's sun_transmittance with the ray as "Sun"), 3 %;
#   2. a camera inside the slab renders a finite, non-black image (Review Focus 5);
#   3. climbing out of the slab in 50 m steps (looking up, the shapes on) changes the centre 5 x 5 pixels by <= 5 % of
#      the climb's brightest value per step (no pop);
#   4. the Sun's scattered radiance (debug output 2) of a slab of tau ~10 seen from above vs the core's closed-form
#      slab_reflected_radiance (the shader's TMS source integrated by the raymarch), 5 %;
#   5. the GPU's filtered coverage (debug output 3) at the slab's centre height over a wide view = the cloud fraction
#      (0.1 / 0.5 / 0.9) within 0.02 (Task 4 deferred: the RGBA8 filter weights against the float64 coverage table).
# Detail noise off for the numbers (its amplitude 0); the desktop quality preset. Exit code 1 on failure.
# Needs a GPU (no --headless): godot --path game --script res://tests/cloud_check.gd
extends SceneTree

const VIEW_SIZE := 33  ## odd: the centre pixel's ray is the camera axis
const COVERAGE_VIEW_SIZE := 161
const SETTLE_FRAMES := 3
const EPOCH_UTC := "2024-04-08T18:42:00"
const AU_M := 149597870700.0
const LAT_DEG := 30.0
const LON_DEG := 45.0
const SLAB_BOTTOM_M := 1000.0
const SLAB_TOP_M := 2000.0
## Liquid water of vertical optical depth 1 over 1 km: beta = 3 LWC / (2 rho_w r_l) = 150 LWC [1/m].
const THIN_LIQUID_KG_M3 := 1.0e-3 / 150.0
const THICK_LIQUID_KG_M3 := 1.0e-2 / 150.0
const TRANSMITTANCE_TOLERANCE := 0.03
const TRANSMITTANCE_FLOOR := 0.02
const RADIANCE_TOLERANCE := 0.05
const CLIMB_STEP_LIMIT := 0.05
const COVERAGE_TOLERANCE := 0.02
## Transmittance and radiance samples: the mean of the 5 x 5 pixels around the centre (see _render).
const BLOCK_RADIUS := 2
const EXPECTED_SAMPLES := 19

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()
var _clouds := CloudLayer.new()
var _viewport := SubViewport.new()
var _camera := Camera3D.new()
var _earth_to_gcrf := Basis.IDENTITY
var _failures := 0
var _samples := 0
var _files: Array[String] = []


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	if not _core.open(Paths.data_dir().path_join("spice")):
		_fail_and_quit("SPICE kernels missing")
		return
	_earth_to_gcrf = _core.earth_fixed_to_gcrf(_core.utc_to_tdb_s(EPOCH_UTC))
	_atmosphere.eye_adaptation_enabled = false
	_atmosphere.start_loading(_core, Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	if not _atmosphere.wait_until_loaded():
		_fail_and_quit("atmosphere tables not loaded")
		return
	_setup_scene()
	_clouds.setup(_core)
	if not _clouds.wait_until_ready():
		_fail_and_quit("cloud noise not ready")
		return
	_clouds.quality = CloudLayer.Quality.DESKTOP
	RenderingServer.global_shader_parameter_set(&"cloud_detail_amplitude", 0.0)

	var thin := _slab("thin", SLAB_BOTTOM_M, SLAB_TOP_M, THIN_LIQUID_KG_M3, 1.0)
	_show(thin)
	var sun := _local_direction(45.0, 135.0)
	var up := _local_direction(90.0, 0.0)
	# 1. Background transmittance below, inside, above.
	await _transmittance_sample("below, zenith", 10.0, up, sun, thin)
	await _transmittance_sample("below, elevation 30 deg", 10.0, _local_direction(30.0, 200.0), sun, thin)
	await _transmittance_sample("inside, zenith", 1500.0, up, sun, thin)
	await _transmittance_sample("inside, horizontal", 1500.0, _local_direction(0.0, 70.0), sun, thin)
	await _transmittance_sample("above, nadir", 3000.0, _local_direction(-90.0, 0.0), sun, thin)
	await _transmittance_sample("above, elevation -20 deg", 3000.0, _local_direction(-20.0, 10.0), sun, thin)
	# 2. Inside the slab: finite, not black.
	for view in [up, _local_direction(0.0, 70.0), _local_direction(-60.0, 300.0)]:
		var c := await _render(_point(1500.0), view, sun, 0)
		var lum := _luminance(c)
		_check("inside the slab: finite, non-black", is_finite(lum) and lum > 1e-4, "luminance %.5f" % lum)
	# 3. Climbing out of the slab in 50 m steps, looking up (a slanted view would cross 50 m / sin(e) more cloud per step).
	var view_climb := up
	# A step is measured against the climb's brightest value (the lit cloud around the camera): near the top the pixel
	# falls toward the much darker clear zenith sky, where a step relative to the pixel itself grows without a pop
	# (measured: uniform steps of ~0.01 down to the sky's 0.04).
	var values: Array[float] = []
	var h := 700.0
	var trace := ""
	while h <= 2300.0:
		var lum := _luminance(await _render(_point(h), view_climb, sun, 0, BLOCK_RADIUS))
		trace += " %.0f:%.4f" % [h, lum]
		values.append(lum)
		h += 50.0
	if OS.get_environment("STARSIM_VERBOSE") != "":
		print("cloud_check: climb (height: luminance)" + trace)
	var brightest: float = values.max()
	var worst := 0.0
	var worst_at := 0.0
	for i in range(1, values.size()):
		var step := absf(values[i] - values[i - 1])
		if step / brightest > worst:
			worst = step / brightest
			worst_at = 700.0 + 50.0 * i
	_check("climbing out in 50 m steps", worst <= CLIMB_STEP_LIMIT,
			"largest step %.2f %% of the brightest (at %.0f m)" % [worst * 100.0, worst_at])
	# 4. Scattered Sun radiance from above a thick slab vs the closed form.
	var thick := _slab("thick", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, 1.0)
	_show(thick)
	var tau := -log(float(_core.weather_cloud_light(_point(10.0), up)["sun_transmittance"]))
	print("cloud_check: thick slab vertical tau %.4f" % tau)
	for c in [[0.0, 0.0, 0.0], [30.0, 0.0, 0.0], [0.0, 60.0, 180.0], [45.0, 60.0, 0.0], [45.0, 60.0, 180.0],
			[30.0, 30.0, 90.0]]:
		await _radiance_sample(c[0], c[1], c[2], tau)
	# 5. Coverage of the GPU's filtered shape noise.
	_viewport.size = Vector2i(COVERAGE_VIEW_SIZE, COVERAGE_VIEW_SIZE)
	for fraction in [0.1, 0.5, 0.9]:
		var file := _slab("coverage_%d" % int(fraction * 100), SLAB_BOTTOM_M, SLAB_TOP_M, THIN_LIQUID_KG_M3, fraction)
		_show(file)
		await _coverage_sample(fraction)
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)

	for f in _files:
		DirAccess.remove_absolute(f)
	_clouds.shutdown()
	_check("sample count", _samples == EXPECTED_SAMPLES, "%d samples" % _samples)
	print("cloud_check: %s (%d samples, %d failures)" % ["PASS" if _failures == 0 else "FAIL", _samples, _failures])
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
	_camera.near = 0.5
	_camera.far = 4.0e7
	_camera.current = true
	_viewport.add_child(_camera)
	_viewport.add_child(_clouds)


## Writes a slab snapshot, returns its loaded dictionary (the file is removed at the end).
func _slab(label: String, bottom_m: float, top_m: float, liquid: float, fraction: float) -> Dictionary:
	var file := OS.get_temp_dir().path_join("starsim_cloud_check_%s_%d.wthr" % [label, OS.get_process_id()])
	_files.append(file)
	if not _core.weather_write_slab_snapshot(file, bottom_m, top_m, liquid, fraction):
		_fail_and_quit("slab snapshot not written")
	var loaded: Dictionary = _core.weather_load_snapshot(file)
	loaded["file"] = file
	return loaded


func _show(snapshot: Dictionary) -> void:
	_clouds.show_now(snapshot, snapshot, 0.0)
	_core.weather_set_active(snapshot["file"], snapshot["file"], 0.0)


func _transmittance_sample(label: String, height_m: float, view_itrf: Vector3, sun_itrf: Vector3,
		_snapshot: Dictionary) -> void:
	var camera := _point(height_m)
	var got := (await _render(camera, view_itrf, sun_itrf, 1, BLOCK_RADIUS)).x
	var expected := float(_core.weather_cloud_light(camera, view_itrf)["sun_transmittance"])
	# Relative, with a floor of 0.02 (an opaque view: a few 1e-4 of light left where the step cap ends the march).
	var error := absf(got - expected) / maxf(expected, TRANSMITTANCE_FLOOR)
	_check("transmittance " + label, error <= TRANSMITTANCE_TOLERANCE,
			"GPU %.5f CPU %.5f (%.2f %%)" % [got, expected, error * 100.0])


## View zenith angle, Sun zenith angle, relative azimuth (0: the camera looks toward the Sun's side - the forward side).
func _radiance_sample(view_zenith_deg: float, sun_zenith_deg: float, azimuth_deg: float, tau: float) -> void:
	var view := _local_direction(view_zenith_deg - 90.0, 0.0)  # looking down, toward north
	var sun := _local_direction(90.0 - sun_zenith_deg, azimuth_deg)  # azimuth 0: Sun in the north, ahead
	var got := (await _render(_point(5000.0), view, sun, 2, BLOCK_RADIUS)).x
	var expected := float(_core.cloud_slab_radiance(tau, cos(deg_to_rad(sun_zenith_deg)),
			cos(deg_to_rad(view_zenith_deg)), deg_to_rad(azimuth_deg)))
	var error := absf(got - expected) / maxf(expected, 1e-6)
	_check("Sun radiance view %.0f / Sun %.0f / azimuth %.0f deg" % [view_zenith_deg, sun_zenith_deg, azimuth_deg],
			error <= RADIANCE_TOLERANCE, "GPU %.5f CPU %.5f /sr (%.2f %%)" % [got, expected, error * 100.0])


func _coverage_sample(fraction: float) -> void:
	RenderingServer.global_shader_parameter_set(&"cloud_debug_height_m", 0.5 * (SLAB_BOTTOM_M + SLAB_TOP_M))
	_camera.fov = 60.0
	await _render(_point(300000.0), _local_direction(-90.0, 0.0), _local_direction(45.0, 0.0), 3)
	_camera.fov = 10.0
	var image := _viewport.get_texture().get_image()
	var covered := 0
	for y in range(image.get_height()):
		for x in range(image.get_width()):
			covered += 1 if image.get_pixel(x, y).r > 0.5 else 0
	var got := float(covered) / float(image.get_width() * image.get_height())
	_check("coverage %.1f" % fraction, absf(got - fraction) <= COVERAGE_TOLERANCE, "covered %.4f" % got)


func _render(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, mode: int, block_radius := 0) -> Vector3:
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var sun_gcrf: Vector3 = _earth_to_gcrf * sun_itrf * AU_M
	_atmosphere.update(camera_gcrf, Vector3.ZERO, _earth_to_gcrf, sun_gcrf, 0.0, 1.0)
	_clouds.update(camera_itrf_m, 0.0, 0.0)
	RenderingServer.global_shader_parameter_set(&"cloud_debug_output", mode)
	var view_render := Frames.to_render(_earth_to_gcrf * view_itrf).normalized()
	var up := Vector3.UP if absf(view_render.y) < 0.9 else Vector3.RIGHT
	_camera.transform = Transform3D(Basis.looking_at(view_render, up), Vector3.ZERO)
	for i in range(SETTLE_FRAMES):
		await RenderingServer.frame_post_draw
	var image := _viewport.get_texture().get_image()
	# The mean over a (2 r + 1)^2 block around the centre: the jitter differs per pixel, so the block mean is the
	# estimator's expectation (rays within +-0.6 deg of the axis at r = 2).
	var sum := Vector3.ZERO
	for dy in range(-block_radius, block_radius + 1):
		for dx in range(-block_radius, block_radius + 1):
			var c := image.get_pixel(image.get_width() / 2 + dx, image.get_height() / 2 + dy)
			sum += Vector3(c.r, c.g, c.b)
	return sum / float((2 * block_radius + 1) * (2 * block_radius + 1))


func _point(height_m: float) -> Vector3:
	return HeightSource.geodetic_to_fixed(LAT_DEG, LON_DEG, height_m)


## ITRF unit vector at elevation above the geodetic horizon and azimuth (from north, east positive) at the site.
static func _local_direction(elevation_deg: float, azimuth_deg: float) -> Vector3:
	var lat := deg_to_rad(LAT_DEG)
	var lon := deg_to_rad(LON_DEG)
	var up := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var east := Vector3(-sin(lon), cos(lon), 0.0)
	var north := Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	var az := deg_to_rad(azimuth_deg)
	var el := deg_to_rad(elevation_deg)
	return (up * sin(el) + (north * cos(az) + east * sin(az)) * cos(el)).normalized()


static func _luminance(c: Vector3) -> float:
	return 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z


func _check(label: String, ok: bool, detail: String) -> void:
	_samples += 1
	if not ok:
		_failures += 1
	print("  %s  %s: %s" % ["ok  " if ok else "FAIL", label, detail])


func _fail_and_quit(message: String) -> void:
	print("cloud_check: FAIL - " + message)
	quit(1)
