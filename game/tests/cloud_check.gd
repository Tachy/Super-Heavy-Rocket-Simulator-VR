# GPU check of the volumetric clouds (M8 weather and clouds, Task 8): renders the cloud pass (CloudEffect, composited by
# CloudLayer's quad) over the sky into an HDR SubViewport and compares pixels with the core's CPU reference.
# Synthetic snapshots from StarsimCore.weather_write_slab_snapshot (uniform slabs, 1 deg grid, ground at 0 m):
#   1. background transmittance (debug output 1, grid-mean field) seen from below, inside and above a slab of vertical
#      tau ~1 vs the core's Beer-Lambert along the same ray (weather_cloud_light's sun_transmittance with the ray as
#      "Sun"), 3 %; orientation: a wide view toward the horizon below the slab shows it in the top row at that row's
#      elevation (3 %) and not in the bottom row (the pass's rays match the screen);
#   2. a camera inside the slab renders a finite, non-black image (Review Focus 5); the transmittance to an occluder
#      2 km ahead inside the slab is exp(-beta 2 km) within 3 % (the scene depth -> distance convention); a near
#      "mountain" edge through a half-resolution block: no cloud over the mountain, the sky's slab within 5 %;
#   3. climbing out of the slab in 50 m steps (looking up, the shapes on, temporal accumulation off) changes the centre
#      5 x 5 pixels by <= 5 % of the climb's brightest value per step (no pop);
#   4. the Sun's scattered radiance (debug output 2) of a slab of tau ~10 seen from above vs the core's CPU reference,
#      for both multiple-scattering models: the closed form vs slab_reflected_radiance with a converged light march (24
#      samples, mean of 8 frames) 5 % - the port of the 3D source; the Monte Carlo table (Task 8b) vs
#      slab_reflected_radiance_mc 3 %, including a low-Sun forward view; with the preset's march at the worst geometry
#      15 % (the stochastic exp(-tau_s) is brighter on average);
#   5. the GPU's filtered coverage (debug output 3, full resolution) at the slab's centre height over a wide view = the
#      cloud fraction (0.1 / 0.5 / 0.9) within 0.02 (Task 4 deferred: the RGBA8 filter weights against the float64
#      coverage table);
#   6. light on the ground (Task 9; terrain_earth.gdshader on a quad 300 m from a camera 2 m up, under a thin slab of
#      tau ~1, a deck of tau ~3 and a broken layer of cf 0.4, Sun 45 / 20 deg up): with the near shadow map of the grid mean, the shader's cloud
#      factors (debug output 5: direct, the Sun's diffuse share, the sky's) vs weather_cloud_light within 3 %, the
#      rendered ground's luminance ratio weather on / off = (T_s E_dir + T_sd E_sun,h + T_d E_sky) / (E_dir + E_sky)
#      of the CPU within 5 % (the broken layer as independent columns, cloud_optics.hpp); with the game's map (the
#      shapes) under the slabs of cf 1 the ground's total light vs the vehicle's CPU light (grid mean) at the same spot
#      within 10 % (broken: reported - the spot is in a gap or under a cumulus, the CPU's the cell's mean); with weather off every pixel of land and sea bit-identical to the
#      shader without its CLOUD_LIGHT blocks.
#   7. (fix round 1) through cloud_light_probe.gdshader (the surfaces' cloud-light functions at receivers on annuli
#      around the camera, no terrain geometry), the means over eight independent sites (shape-noise realisations):
#      the hand-overs near map -> far map (9-11 km: annuli at 8 / 10 / 12 km) and far map -> the slanted column
#      (120-140 km: 110 / 130 / 150 km), a cf 0.3 layer and a deck, Sun 45 / 8 deg: steps <= 0.05 + 3 standard errors
#      (0.03 for the smooth far tiers); over the disc inside the near tier the shapes' area mean of the direct Sun
#      vs the CPU's cell (cf 0.15 / 0.3 / 0.4, Sun 45 / 8 deg; the beam-cover overlap model, constants.hpp
#      cloud_overlap_*, is fitted to these shadows: STARSIM_SLANT_TABLE=1 prints the fit table), the near map vs the
#      far map over the same receivers, the column terms vs the CPU, the vehicle's total light vs the ground's mean.
#   8. (static clouds Task 1) the light cache (CloudLightCache, one cascade 8 x 8 km over the shell) against the
#      per-sample light march with the cache's converged sample count (CloudLightCache.SAMPLES: 96 near + 32 far,
#      jittered, mean of 8 frames), the same view jitter in both (CloudEffect.pinned_frame): the Sun's scattered
#      radiance at every pixel of a 64 x 64 view (30 deg fov, 75 deg down, asserted inside the cascade) from 5 km:
#      a slab of tau ~10 (debug output 2, the grid mean) 5 %, a
#      broken layer of cf 0.4 (debug output 7, the shapes) 10 % - Sun 45 / 20 deg up; a view whose samples all lie
#      outside the cascade equals the march exactly (bit for bit); the fill's GPU time (information).
# The other sections run with the light cache on (the game's path). Detail levels off for the numbers
# (cloud_detail_levels 0); the desktop quality preset. Exit code 1 on failure.
# Needs a GPU (no --headless): godot --path game --script res://tests/cloud_check.gd
extends SceneTree

const VIEW_SIZE := 33  ## odd: the centre pixel's ray is the camera axis
const COVERAGE_VIEW_SIZE := 161
const SETTLE_FRAMES := 4
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
const TABLE_RADIANCE_TOLERANCE := 0.03
const PRESET_RADIANCE_TOLERANCE := 0.15
const CONVERGED_LIGHT_SAMPLES := 24
const RADIANCE_FRAMES := 8
const CLIMB_STEP_LIMIT := 0.05
const COVERAGE_TOLERANCE := 0.02
## Transmittance and radiance samples: the mean of the 5 x 5 pixels around the centre (see _render).
const BLOCK_RADIUS := 2
const EXPECTED_SAMPLES := 101
## Occluders: one inside the slab at a known distance (depth -> distance), one near "mountain" over half the view.
const OCCLUDER_DISTANCE_M := 2000.0
const OCCLUDER_SIZE_M := 1000.0
const EDGE_DISTANCE_M := 50.0
const EDGE_SIZE_M := 100.0
## Light on the ground (section 6).
const GROUND_DISTANCE_M := 300.0
const GROUND_ALBEDO_8 := Color8(150, 120, 90)
const MOON_DISTANCE_M := 3.844e8
const DECK_LIQUID_KG_M3 := 3.0e-3 / 150.0
## A broken layer (independent columns: clear on 1 - cf, the in-cloud depth on cf).
const BROKEN_FRACTION := 0.4
const DIRECT_FLOOR := 0.01
## The vehicle's CPU light is the grid mean, the ground's shadow the visible shapes (cf 1 slabs still vary along a ray
## by the shape noise's low octaves): their total light on a horizontal plate (measured: within 6 %).
const VEHICLE_TOLERANCE := 0.10
const FACTOR_TOLERANCE := 0.03
const GROUND_RATIO_TOLERANCE := 0.05
## Section 7 (hand-overs and the shapes' area means, receivers on annuli through cloud_light_probe.gdshader).
const PROBE_WIDTH := 256
## Sites ~55 km apart (degrees north, east of the check site): independent realisations of the shape noise.
const SITES := [Vector2(0.0, 0.0), Vector2(0.0, 0.55), Vector2(0.0, -0.55), Vector2(0.5, 0.0), Vector2(-0.5, 0.0),
		Vector2(0.5, 0.55), Vector2(-0.5, -0.55), Vector2(0.5, -0.55)]
const DISC_RADIUS_M := 8000.0  ## inside the near tier (it is the only one below 9 km)
## The near map holds one realisation of the shape noise: an area mean over N independent cloud cells carries
## sigma ~ sqrt(cf (1 - cf) / N) in the covered fraction (the coverage-preserving noise fixes the mean over a tile, not
## over a disc), and the shapes' internal density variation (Jensen: mean of exp(-tau) above exp(-mean tau)) moves the
## cloudy part's transmittance up. Tolerances are set from the measured values with that margin.
const DISC_TOLERANCE := 0.08
## A step across the hand-over: this much (the tiers' systematic agreement: the fitted overlap is good to 0.02) plus
## three standard errors of the two annuli's means over the sites (the shape noise's sampling error).
const HANDOVER_TOLERANCE := 0.05
const FAR_HANDOVER_TOLERANCE := 0.03  ## the far map and the slanted column are both smooth (the grid mean)
## Section 8 (the light cache against the light march).
const CACHE_VIEW_SIZE := 64
## The compared view's farthest cloud sample may use at most this fraction of the cascade's half extent (margin for the
## anchor's cell snapping).
const CACHE_VIEW_MARGIN := 0.9
const CACHE_SLAB_TOLERANCE := 0.05
const CACHE_BROKEN_TOLERANCE := 0.10  ## the voxel size (62.5 m across, 320 m up) against the shapes
## A pixel's error is relative to max(its reference, this fraction of the view's mean reference): a cloud's fringe or a
## gap's last light carries no relative accuracy of its own.
const CACHE_ERROR_FLOOR := 0.05

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()
var _clouds := CloudLayer.new()
var _viewport := SubViewport.new()
var _camera := Camera3D.new()
var _earth_to_gcrf := Basis.IDENTITY
var _failures := 0
var _samples := 0
var _files: Array[String] = []
var _terrain_parent := Node3D.new()  ## Earth-fixed frame of the terrain quad (section 6)
var _terrain_quad := MeshInstance3D.new()
var _terrain_material := ShaderMaterial.new()
var _probe_viewport := SubViewport.new()  ## section 7: the receivers' probe
var _probe_material := ShaderMaterial.new()
var _water_material := ShaderMaterial.new()  ## the same with an all-water mask (the sea model)


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
	_setup_probe()
	_clouds.setup(_core)
	if not _clouds.wait_until_ready():
		_fail_and_quit("cloud noise not ready")
		return
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_detail_off()
	_check("transfer table loaded", _clouds.transfer_ready, "config/" + CloudLayer.TRANSFER_TABLE)

	if OS.get_environment("STARSIM_SLANT_TABLE") != "":
		await _slant_table()
		quit(0)
		return
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
	# Orientation: below the slab looking at the horizon, the top rows see it (30 deg up), the bottom rows do not.
	_camera.fov = 60.0
	await _render(_point(10.0), _local_direction(0.0, 70.0), sun, 1)
	_camera.fov = 10.0
	var frame := _viewport.get_texture().get_image()
	var top_row := frame.get_pixel(VIEW_SIZE / 2, 0).r
	var bottom_row := frame.get_pixel(VIEW_SIZE / 2, VIEW_SIZE - 1).r
	# The top row shows the ray of the first half-resolution row (its centre 1 px below the edge).
	var top_elevation := rad_to_deg(atan((1.0 - 2.0 / VIEW_SIZE) * tan(deg_to_rad(30.0))))
	var top_expected := float(_core.weather_cloud_light(_point(10.0),
			_local_direction(top_elevation, 70.0))["sun_transmittance"])
	_check("orientation (the slab in the top rows at their elevation, not in the bottom rows)",
			absf(top_row - top_expected) <= TRANSMITTANCE_TOLERANCE * top_expected and bottom_row > 0.99,
			"top %.4f (CPU %.4f at %.1f deg) bottom %.3f" % [top_row, top_expected, top_elevation, bottom_row])
	# 2. Inside the slab: finite, not black.
	for view in [up, _local_direction(0.0, 70.0), _local_direction(-60.0, 300.0)]:
		var c := await _render(_point(1500.0), view, sun, 0)
		var lum := _luminance(c)
		_check("inside the slab: finite, non-black", is_finite(lum) and lum > 1e-4, "luminance %.5f" % lum)
	# 2b. Depth -> distance: an occluder 2 km ahead of a camera inside the slab; the transmittance to it is the slab's
	# in-cloud extinction over 2 km (beta from the CPU's vertical depth of the 1 km slab), 3 %.
	var slab_tau := -log(float(_core.weather_cloud_light(_point(10.0), up)["sun_transmittance"]))
	var beta := slab_tau / (SLAB_TOP_M - SLAB_BOTTOM_M)
	var occluder := _occluder(Vector3(0.0, 0.0, -OCCLUDER_DISTANCE_M), Vector2(OCCLUDER_SIZE_M, OCCLUDER_SIZE_M))
	var t_occluded := (await _render(_point(1500.0), _local_direction(0.0, 70.0), sun, 1, BLOCK_RADIUS)).x
	occluder.queue_free()
	var t_expected := exp(-beta * OCCLUDER_DISTANCE_M)
	_check("transmittance to an occluder at %.0f m inside the slab" % OCCLUDER_DISTANCE_M,
			absf(t_occluded - t_expected) <= TRANSMITTANCE_TOLERANCE * t_expected,
			"GPU %.5f expected %.5f (beta %.6f /m)" % [t_occluded, t_expected, beta])
	await _mountain_edge_sample(sun)
	# 3. Climbing out of the slab in 50 m steps, looking up (a slanted view would cross 50 m / sin(e) more cloud per step).
	# The temporal accumulation off (weight 1): the check is on the field, not on the history's lag (with it the steps
	# measured up to 4.9 % at the 5 % limit).
	_clouds.effect.set_quality(int(CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]["divisor"]), 1.0)
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
	_clouds.quality = CloudLayer.Quality.DESKTOP  # the preset's temporal weight again
	_detail_off()
	# 4. Scattered Sun radiance from above a thick slab vs the closed form.
	var thick := _slab("thick", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, 1.0)
	_show(thick)
	var tau := -log(float(_core.weather_cloud_light(_point(10.0), up)["sun_transmittance"]))
	print("cloud_check: thick slab vertical tau %.4f" % tau)
	_clouds.scattering_model = CloudLayer.ScatteringModel.CLOSED_FORM
	for c in [[0.0, 0.0, 0.0], [30.0, 0.0, 0.0], [0.0, 60.0, 180.0], [45.0, 60.0, 0.0], [45.0, 60.0, 180.0],
			[30.0, 30.0, 90.0]]:
		await _radiance_sample(c[0], c[1], c[2], tau, true)
	# The preset's own light march at the worst geometry: its stochastic estimate exp(-tau_s) is brighter on average
	# than exp(-E[tau_s]) (Jensen; the march is unbiased in tau_s), wider tolerance.
	await _radiance_sample(45.0, 60.0, 0.0, tau, false)
	# The Monte Carlo table, incl. a low Sun (10 deg up) seen on the forward side (the backlit glow).
	_clouds.scattering_model = CloudLayer.ScatteringModel.TABLE
	for c in [[0.0, 0.0, 0.0], [45.0, 60.0, 0.0], [45.0, 60.0, 180.0], [30.0, 30.0, 90.0], [75.0, 80.0, 0.0],
			[60.0, 80.0, 180.0]]:
		await _radiance_sample(c[0], c[1], c[2], tau, true)
	await _radiance_sample(75.0, 80.0, 0.0, tau, false)
	# 5. Coverage of the GPU's filtered shape noise.
	_viewport.size = Vector2i(COVERAGE_VIEW_SIZE, COVERAGE_VIEW_SIZE)
	for fraction in [0.1, 0.5, 0.9]:
		var file := _slab("coverage_%d" % int(fraction * 100), SLAB_BOTTOM_M, SLAB_TOP_M, THIN_LIQUID_KG_M3, fraction)
		_show(file)
		await _coverage_sample(fraction)
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	# 6. Light on the ground under the clouds.
	await _ground_light_samples()
	# 7. The shadow maps' hand-overs and the shapes on broken cloud.
	await _handover_samples()
	# 8. The light cache against the per-sample light march.
	await _light_cache_samples()

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
	var compositor := Compositor.new()
	compositor.compositor_effects = [_clouds.shadow_effect, _clouds.effect]
	world_environment.compositor = compositor
	_viewport.add_child(world_environment)
	_camera.fov = 10.0
	_camera.near = 0.5
	_camera.far = 4.0e7
	_camera.current = true
	_viewport.add_child(_camera)
	_viewport.add_child(_clouds)
	var albedo := Image.create(4, 4, true, Image.FORMAT_RGBA8)
	albedo.fill(GROUND_ALBEDO_8)
	albedo.generate_mipmaps()
	var black := Image.create(4, 4, true, Image.FORMAT_RGBA8)
	black.fill(Color(0, 0, 0))
	black.generate_mipmaps()
	var white := Image.create(4, 4, false, Image.FORMAT_R8)
	white.fill(Color(1, 1, 1))
	for material: ShaderMaterial in [_terrain_material, _water_material]:
		material.shader = load("res://shaders/terrain_earth.gdshader")
		material.set_shader_parameter("day_texture", ImageTexture.create_from_image(albedo))
		material.set_shader_parameter("day_texture_bound", true)
		material.set_shader_parameter("night_texture", ImageTexture.create_from_image(black))
		material.set_shader_parameter("moon_radius_m", 1737400.0)
		material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	_water_material.set_shader_parameter("water_mask", ImageTexture.create_from_image(white))
	_terrain_quad.material_override = _terrain_material
	_terrain_parent.add_child(_terrain_quad)
	_terrain_parent.visible = false
	_viewport.add_child(_terrain_parent)


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
## converged: the light march with CONVERGED_LIGHT_SAMPLES near samples (the port against the closed form); else the
## quality preset's (tolerance PRESET_RADIANCE_TOLERANCE). The mean of RADIANCE_FRAMES frames (each its own jitter).
func _radiance_sample(view_zenith_deg: float, sun_zenith_deg: float, azimuth_deg: float, tau: float,
		converged: bool) -> void:
	var view := _local_direction(view_zenith_deg - 90.0, 0.0)  # looking down, toward north
	var sun := _local_direction(90.0 - sun_zenith_deg, azimuth_deg)  # azimuth 0: Sun in the north, ahead
	if converged:
		var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
		var first: float = preset["light_first_m"]
		var near: float = preset["light_near_m"]
		ShaderGlobals.set_value(&"cloud_light_march", Vector4(first,
				CloudLayer.light_march_ratio(first, near, CONVERGED_LIGHT_SAMPLES), near, 0.0))
		ShaderGlobals.set_value(&"cloud_light_samples", Vector2i(CONVERGED_LIGHT_SAMPLES, int(preset["light_far"])))
	var got := 0.0
	for i in range(RADIANCE_FRAMES):
		got += (await _render(_point(5000.0), view, sun, 2, BLOCK_RADIUS)).x / RADIANCE_FRAMES
	_clouds.quality = CloudLayer.Quality.DESKTOP  # the preset's march again
	_detail_off()
	var table := _clouds.scattering_model == CloudLayer.ScatteringModel.TABLE
	var args := [tau, cos(deg_to_rad(sun_zenith_deg)), cos(deg_to_rad(view_zenith_deg)), deg_to_rad(azimuth_deg)]
	var expected := float(_core.callv(&"cloud_slab_radiance_mc" if table else &"cloud_slab_radiance", args))
	var error := absf(got - expected) / maxf(expected, 1e-6)
	var tolerance := (TABLE_RADIANCE_TOLERANCE if table else RADIANCE_TOLERANCE) if converged \
			else PRESET_RADIANCE_TOLERANCE
	_check("Sun radiance (%s) view %.0f / Sun %.0f / azimuth %.0f deg (%s march)" % ["table" if table else "closed form",
			view_zenith_deg, sun_zenith_deg, azimuth_deg, "converged" if converged else "preset"],
			error <= tolerance, "GPU %.5f CPU %.5f /sr (%.2f %%)" % [got, expected, error * 100.0])


## Detail levels off: the CPU references have the shapes of state 0 at most (cloud_noise.hpp).
func _detail_off() -> void:
	ShaderGlobals.set_value(&"cloud_detail_levels", 0)


## An unshaded box in front of the camera: centre in the camera's frame (x right, y up, -z ahead), size across x / y.
func _occluder(centre_camera: Vector3, size: Vector2) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(size.x, size.y, 1.0)
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = Color(0.2, 0.2, 0.2)
	var box := MeshInstance3D.new()
	box.mesh = mesh
	box.material_override = material
	box.position = centre_camera  # a child of the camera
	_camera.add_child(box)
	return box


## A mountain edge at the half-resolution pass: below the slab, looking 30 deg up, a near occluder (50 m) covers the
## columns left of the edge, which runs through the middle of a 2 x 2 block (between columns 16 and 17 of 33); the
## sky right of it sees the slab. The full-resolution columns next to the edge: the occluder's keep T = 1 (no cloud
## bleeds over the mountain), the sky's the slab's T of their rays (the CPU's) within 5 %.
func _mountain_edge_sample(sun: Vector3) -> void:
	var view := _local_direction(30.0, 200.0)
	var pixel_m := 2.0 * EDGE_DISTANCE_M * tan(deg_to_rad(_camera.fov * 0.5)) / VIEW_SIZE
	var edge_x := 0.5 * pixel_m  # between the centre column 16 and column 17
	var occluder := _occluder(Vector3(edge_x - 0.5 * EDGE_SIZE_M, 0.0, -EDGE_DISTANCE_M),
			Vector2(EDGE_SIZE_M, EDGE_SIZE_M))
	await _render(_point(10.0), view, sun, 1)
	occluder.queue_free()
	var image := _viewport.get_texture().get_image()
	var mountain := 0.0
	var sky := 0.0
	var rows := 0
	for y in range(VIEW_SIZE / 2 - BLOCK_RADIUS, VIEW_SIZE / 2 + BLOCK_RADIUS + 1):
		mountain += image.get_pixel(16, y).r
		sky += image.get_pixel(17, y).r
		rows += 1
	mountain /= rows
	sky /= rows
	# Column 17's ray: 1 pixel right of the axis (the camera's x toward the screen's right).
	var basis := _camera.transform.basis
	var tan_x := (17.5 - VIEW_SIZE * 0.5) / (VIEW_SIZE * 0.5) * tan(deg_to_rad(_camera.fov * 0.5))
	var ray_render := (basis * Vector3(tan_x, 0.0, -1.0)).normalized()
	var ray_itrf: Vector3 = _earth_to_gcrf.inverse() * Frames.from_render(ray_render)
	var expected := float(_core.weather_cloud_light(_point(10.0), ray_itrf.normalized())["sun_transmittance"])
	var sky_error := absf(sky - expected) / maxf(expected, TRANSMITTANCE_FLOOR)
	_check("mountain edge upsampling (occluder side clear, sky side the slab)", mountain >= 0.97 and sky_error <= 0.05,
			"occluder column %.4f, sky column %.4f (CPU %.4f, %.2f %%)" % [mountain, sky, expected, sky_error * 100.0])


func _coverage_sample(fraction: float) -> void:
	ShaderGlobals.set_value(&"cloud_debug_height_m", 0.5 * (SLAB_BOTTOM_M + SLAB_TOP_M))
	_camera.fov = 60.0
	# Full resolution: the check is on the field's filtered coverage, not on the pass's upsampling (half-resolution
	# samples lose holes narrower than two pixels at 0.9).
	_clouds.effect.set_quality(1, 1.0)
	await _render(_point(300000.0), _local_direction(-90.0, 0.0), _local_direction(45.0, 0.0), 3)
	_camera.fov = 10.0
	var image := _viewport.get_texture().get_image()
	_clouds.quality = _clouds.quality  # the preset's resolution again
	_detail_off()
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
	ShaderGlobals.set_value(&"cloud_debug_output", mode)
	var view_render := Frames.to_render(_earth_to_gcrf * view_itrf).normalized()
	# The screen's up toward the local vertical (rows = elevations), any other for a zenith or nadir view.
	var up := Frames.to_render(_earth_to_gcrf * camera_itrf_m.normalized()).normalized()
	if absf(view_render.dot(up)) > 0.9:
		up = Vector3.UP if absf(view_render.y) < 0.9 else Vector3.RIGHT
	_camera.transform = Transform3D(Basis.looking_at(view_render, up), Vector3.ZERO)
	for i in range(SETTLE_FRAMES):
		await RenderingServer.frame_post_draw
		_clouds.update(camera_itrf_m, 0.0, 0.0)  # as the world does every frame (a re-created result is shown)
	var image := _viewport.get_texture().get_image()
	# The mean over a (2 r + 1)^2 block around the centre: the jitter differs per pixel, so the block mean is the
	# estimator's expectation (rays within +-0.6 deg of the axis at r = 2).
	var sum := Vector3.ZERO
	for dy in range(-block_radius, block_radius + 1):
		for dx in range(-block_radius, block_radius + 1):
			var c := image.get_pixel(image.get_width() / 2 + dx, image.get_height() / 2 + dy)
			sum += Vector3(c.r, c.g, c.b)
	return sum / float((2 * block_radius + 1) * (2 * block_radius + 1))


## Section 8: the light cache against the per-sample light march with the cache's own converged sample count (the
## cache's fill: CloudLightCache.SAMPLES at the stratified midpoints; the march: the same counts, jittered per frame and
## averaged over RADIANCE_FRAMES frames), the view's jitter pinned to the same frames in both.
func _light_cache_samples() -> void:
	var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
	var first: float = preset["light_first_m"]
	var near: float = preset["light_near_m"]
	var samples := CloudLightCache.SAMPLES
	_viewport.size = Vector2i(CACHE_VIEW_SIZE, CACHE_VIEW_SIZE)
	# A steep, narrow view: every cloud sample of every pixel must lie inside the cascade (asserted below), else the
	# pixels outside would be marched in both renders and compare equal.
	const VIEW_ELEVATION_DEG := -75.0
	const VIEW_AZIMUTH_DEG := 10.0
	_camera.fov = 30.0
	var reach_m := _view_reach_m(VIEW_ELEVATION_DEG, VIEW_AZIMUTH_DEG, _camera.fov, 5000.0 - SLAB_BOTTOM_M)
	var half_extent_m := CloudLayer.LIGHT_CACHE_EXTENT_M.x * 0.5
	_check("light cache view: every cloud sample inside the cascade", reach_m <= CACHE_VIEW_MARGIN * half_extent_m,
			"farthest sample (the slab's base at the corner rays) %.0f m of the camera's vertical, cascade half extent %.0f m"
			% [reach_m, half_extent_m])
	var slab := _slab("cache_slab", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, 1.0)
	var broken := _slab("cache_broken", SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, BROKEN_FRACTION)
	var view := _local_direction(VIEW_ELEVATION_DEG, VIEW_AZIMUTH_DEG)
	for c in [[slab, 2, CACHE_SLAB_TOLERANCE, "slab tau ~10"], [broken, 7, CACHE_BROKEN_TOLERANCE, "broken cf 0.4"]]:
		_show(c[0])
		for sun_elevation in [45.0, 20.0]:
			var sun := _local_direction(sun_elevation, 135.0)
			var images: Array[PackedFloat32Array] = []
			for cached in [false, true]:
				_clouds.light_cache_enabled = cached
				ShaderGlobals.set_value(&"cloud_light_march", Vector4(first,
						CloudLayer.light_march_ratio(first, near, samples.x), near, 0.0))
				ShaderGlobals.set_value(&"cloud_light_samples", samples)
				images.append(await _mean_image(_point(5000.0), view, sun, c[1]))
			_clouds.light_cache_enabled = true
			_clouds.quality = CloudLayer.Quality.DESKTOP
			_detail_off()
			var mean := 0.0
			for v in images[0]:
				mean += v / images[0].size()
			var worst := 0.0
			var worst_at := 0
			var mean_error := 0.0
			var bias := 0.0
			for i in range(images[0].size()):
				var error := absf(images[1][i] - images[0][i]) / maxf(images[0][i], CACHE_ERROR_FLOOR * mean)
				mean_error += error / images[0].size()
				bias += (images[1][i] - images[0][i]) / (mean * images[0].size())
				if error > worst:
					worst = error
					worst_at = i
			_check("light cache vs light march, %s, Sun %.0f deg" % [c[3], sun_elevation],
					mean > 0.0 and worst <= c[2],
					("worst pixel %.2f %% at (%d, %d) (cache %.5f march %.5f), mean error %.2f %%, bias %.2f %% of the"
					+ " view mean %.5f /sr") % [worst * 100.0, worst_at % CACHE_VIEW_SIZE, worst_at / CACHE_VIEW_SIZE,
					images[1][worst_at], images[0][worst_at], mean_error * 100.0, bias * 100.0, mean])
	# The fill's GPU time (information; the budget is Task 2's progressive fill): a forced refill with the timestamps.
	CloudEffect.capture_timestamps = true
	var camera := _point(5000.0)
	_atmosphere.update(_earth_to_gcrf * camera, Vector3.ZERO, _earth_to_gcrf,
			_earth_to_gcrf * _local_direction(30.0, 135.0) * AU_M, 0.0, 1.0)
	var fill_ms := 0.0
	for i in range(SETTLE_FRAMES):
		_clouds.update(camera, 0.0, 0.0)
		await RenderingServer.frame_post_draw
		fill_ms = maxf(fill_ms, _light_fill_ms())
	CloudEffect.capture_timestamps = false
	print("  info  light cache fill: %.2f ms GPU for %d voxels" % [fill_ms, _clouds.light_cache.voxel_count()])
	# Outside the cascade: below the slab toward the northern horizon (azimuth 0: the cascade's north axis), every
	# cloud sample is > 5.6 km north of the anchor (the slab's base at <= 10 deg up); the cache must change no bit.
	_show(slab)
	_camera.fov = 10.0
	var outside: Array[PackedByteArray] = []
	for cached in [false, true]:
		_clouds.light_cache_enabled = cached
		CloudEffect.pinned_frame = 0
		await _render(_point(10.0), _local_direction(5.0, 0.0), _local_direction(45.0, 135.0), 2)
		CloudEffect.pinned_frame = -1
		outside.append(_viewport.get_texture().get_image().get_data())
	_clouds.light_cache_enabled = true
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_detail_off()
	var lit := _viewport.get_texture().get_image().get_pixel(CACHE_VIEW_SIZE / 2, CACHE_VIEW_SIZE / 2).r
	_check("light cache: a view outside the cascade equals the march", outside[0] == outside[1] and lit > 0.0,
			"%s (centre %.5f /sr)" % ["identical" if outside[0] == outside[1] else "DIFFERENT", lit])
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)


## Horizontal distance from the camera's vertical at which the corner rays of a square view (flat local frame, view
## elevation / azimuth in degrees, full fov in degrees) reach `drop_m` below the camera; INF if a corner ray stays level.
static func _view_reach_m(elevation_deg: float, azimuth_deg: float, fov_deg: float, drop_m: float) -> float:
	var el := deg_to_rad(elevation_deg)
	var az := deg_to_rad(azimuth_deg)
	var forward := Vector3(cos(el) * sin(az), cos(el) * cos(az), sin(el))  # east, north, up
	var up := Vector3(-sin(el) * sin(az), -sin(el) * cos(az), cos(el))
	var right := Vector3(cos(az), -sin(az), 0.0)
	var t := tan(deg_to_rad(fov_deg * 0.5))
	var reach := 0.0
	for sx in [-1.0, 1.0]:
		for sy in [-1.0, 1.0]:
			var ray: Vector3 = forward + t * (sy * up + sx * right)
			if ray.z >= 0.0:
				return INF
			reach = maxf(reach, Vector2(ray.x, ray.y).length() / -ray.z * drop_m)
	return reach


## The GPU time of the light cache's fill in the frame whose timestamps the RenderingDevice holds (0: no fill).
func _light_fill_ms() -> float:
	var rd := RenderingServer.get_rendering_device()
	var worst := 0.0
	for i in range(1, rd.get_captured_timestamps_count()):
		if rd.get_captured_timestamp_name(i - 1) == "cloud_light_fill":
			var ns := rd.get_captured_timestamp_gpu_time(i) - rd.get_captured_timestamp_gpu_time(i - 1)
			worst = maxf(worst, float(ns))
	return worst * 1.0e-6


## The red channel of the view, the mean over RADIANCE_FRAMES frames with the march's jitter pinned to frames 0, 1, ...
func _mean_image(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, mode: int) -> PackedFloat32Array:
	var mean := PackedFloat32Array()
	mean.resize(CACHE_VIEW_SIZE * CACHE_VIEW_SIZE)
	for frame in range(RADIANCE_FRAMES):
		CloudEffect.pinned_frame = frame
		await _render(camera_itrf_m, view_itrf, sun_itrf, mode)
		var image := _viewport.get_texture().get_image()
		for y in range(CACHE_VIEW_SIZE):
			for x in range(CACHE_VIEW_SIZE):
				mean[y * CACHE_VIEW_SIZE + x] += image.get_pixel(x, y).r / RADIANCE_FRAMES
	CloudEffect.pinned_frame = -1
	return mean


## Section 6: the ground 300 m north of a camera 2 m up, under a thin slab (tau ~1) and a deck (tau ~3), Sun 45 and
## 20 deg up; then the identity of the weather-off look (land and sea).
func _ground_light_samples() -> void:
	var camera := _point(2.0)
	var point := HeightSource.geodetic_to_fixed(LAT_DEG + rad_to_deg(GROUND_DISTANCE_M / 6.371e6), LON_DEG, 0.0)
	var up := _local_direction(90.0, 0.0)
	var thin := _slab("ground_thin", SLAB_BOTTOM_M, SLAB_TOP_M, THIN_LIQUID_KG_M3, 1.0)
	var deck := _slab("ground_deck", SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, 1.0)
	var broken := _slab("ground_broken", SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, BROKEN_FRACTION)
	for slab: Dictionary in [thin, deck, broken]:
		for elevation in [45.0, 20.0]:
			var sun := _local_direction(elevation, 135.0)
			var label := "%s, Sun %.0f deg" % ["tau 1" if slab == thin else ("deck tau 3" if slab == deck
					else "broken cf %.1f" % BROKEN_FRACTION), elevation]
			_clouds.show_now({}, {}, 0.0)
			_core.weather_set_active("", "", 0.0)
			var off := await _render_ground(camera, point, up, sun, 0, _terrain_material)
			_show(slab)
			var light: Dictionary = _core.weather_cloud_light(point, sun)
			var t_s := float(light["sun_transmittance"])
			var t_sd := float(light["sun_diffuse"])
			var t_d := float(light["sky_factor"])
			var lux: Dictionary = _core.atmosphere_illuminance(point, up, sun)
			var e_sun: Vector3 = lux["sun_lux"]
			var e_sky: Vector3 = lux["sky_lux"]
			var sun_share := _luminance(e_sun) / _luminance(e_sun + e_sky)
			# a. The maps' machinery: the near map of the grid mean against the CPU's Beer-Lambert, the column terms, and
			# the shading's composition (the ratio on / off of the surface part: the in-scattered light taken out).
			_clouds.shadow_effect.grid_mean_only = true
			var on := await _render_ground(camera, point, up, sun, 0, _terrain_material)
			var factors := await _render_ground(camera, point, up, sun, 5, _terrain_material)
			_check("ground (grid-mean map): direct Sun " + label,
					absf(factors.x - t_s) <= maxf(FACTOR_TOLERANCE * t_s, DIRECT_FLOOR * 0.1),
					"GPU %.4f CPU %.4f" % [factors.x, t_s])
			_check("ground: the Sun's diffuse share " + label, absf(factors.y - t_sd) <= FACTOR_TOLERANCE * t_sd,
					"GPU %.4f CPU %.4f" % [factors.y, t_sd])
			_check("ground: the sky's diffuse transmittance " + label, absf(factors.z - t_d) <= FACTOR_TOLERANCE * t_d,
					"GPU %.4f CPU %.4f" % [factors.z, t_d])
			var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera, point, sun)
			var in_scatter: Vector3 = to_point["luminance"] * _atmosphere.luminance_scale
			var expected := _luminance(e_sun * (t_s + t_sd) + e_sky * t_d) / _luminance(e_sun + e_sky)
			var got := _luminance(on - in_scatter) / _luminance(off - in_scatter)
			_check("ground luminance ratio weather on / off " + label,
					absf(got - expected) <= GROUND_RATIO_TOLERANCE * expected,
					"rendered %.4f CPU %.4f (T_s %.4f T_sd %.4f T_d %.4f, Sun share %.3f)" % [got, expected, t_s, t_sd,
					t_d, sun_share])
			# b. The game's near map (the visible clouds' shapes) against the vehicle's CPU light (the grid mean) at the
			# same spot: their total light on a horizontal plate (direct + the Sun's diffuse + sky).
			_clouds.shadow_effect.grid_mean_only = false
			var shaped := await _render_ground(camera, point, up, sun, 5, _terrain_material)
			var ground_total := (shaped.x + shaped.y) * sun_share + shaped.z * (1.0 - sun_share)
			var vehicle_total := (t_s + t_sd) * sun_share + t_d * (1.0 - sun_share)
			var detail := "vehicle %.4f ground %.4f (direct CPU %.4f, shapes %.4f)" % [vehicle_total, ground_total, t_s,
					shaped.x]
			if slab == broken:  # the spot lies in a gap or under a cumulus; the CPU's is the cell's mean (reported)
				print("  info  vehicle light vs the ground's (shapes) %s: %s" % [label, detail])
			else:
				_check("vehicle light (CPU, grid mean) vs the ground's (shapes) " + label,
						absf(vehicle_total - ground_total) <= VEHICLE_TOLERANCE * ground_total, detail)
	# Final review I1 / I3: a cross-fade with an empty slot (deck -> no weather). The GPU's light factors (grid-mean map) equal
	# the CPU light of the pair CloudLayer pushed (I1), and the ground light never dips below the end states (I3: the empty
	# slot's column texels are zero; an R8 texture would read as cover 1 and darken the middle of the fade).
	var fade_sun := _local_direction(45.0, 135.0)
	var fade_up := _local_direction(90.0, 0.0)
	_clouds.shadow_effect.grid_mean_only = true
	var fade_lux: Dictionary = _core.atmosphere_illuminance(point, fade_up, fade_sun)
	var fade_sun_share := _luminance(fade_lux["sun_lux"]) / _luminance(fade_lux["sun_lux"] + fade_lux["sky_lux"])
	var totals: Array = []
	var fade_ok := true
	var fade_detail := ""
	for w1 in [0.0, 0.25, 0.5, 0.75, 0.95]:  # 1.0 is weather off (no cloud pass: the bit-identity check below)
		_clouds.show_now(deck, {}, w1)
		var gpu := await _render_ground(camera, point, fade_up, fade_sun, 5, _terrain_material)
		var cpu: Dictionary = _core.weather_cloud_light(point, fade_sun)
		var cpu_factors := Vector3(float(cpu["sun_transmittance"]), float(cpu["sun_diffuse"]), float(cpu["sky_factor"]))
		for axis in range(3):
			fade_ok = fade_ok and absf(gpu[axis] - cpu_factors[axis]) <= maxf(FACTOR_TOLERANCE * cpu_factors[axis], DIRECT_FLOOR * 0.1)
		fade_detail += " w1 %.2f GPU %s CPU %s;" % [w1, gpu, cpu_factors]
		totals.append((gpu.x + gpu.y) * fade_sun_share + gpu.z * (1.0 - fade_sun_share))  # light on a horizontal plate
	_clouds.shadow_effect.grid_mean_only = false
	_check("cross-fade deck -> empty: GPU light factors equal the CPU light of the pushed pair", fade_ok, fade_detail)
	var monotonic: bool = totals[0] < totals[4]
	for i in range(1, totals.size()):
		monotonic = monotonic and totals[i] >= totals[i - 1] - 1e-3
	_check("ground light is monotonic through a deck -> empty cross-fade", monotonic, "light on a plate at w1 0 .25 .5 .75 .95: %s" % [totals])
	# Weather off: the shader with and without its CLOUD_LIGHT blocks, every pixel bit-identical (land and sea).
	_clouds.show_now({}, {}, 0.0)
	_core.weather_set_active("", "", 0.0)
	var sun := _local_direction(30.0, 135.0)
	var stripped := Shader.new()
	stripped.code = _strip_cloud_light((load("res://shaders/terrain_earth.gdshader") as Shader).code)
	for material: ShaderMaterial in [_terrain_material, _water_material]:
		await _render_ground(camera, point, up, sun, 0, material)
		var with_blocks := _viewport.get_texture().get_image().get_data()
		var without := material.duplicate() as ShaderMaterial
		without.shader = stripped
		await _render_ground(camera, point, up, sun, 0, without)
		var reference := _viewport.get_texture().get_image().get_data()
		var differing := 0
		for i in range(reference.size()):
			differing += 1 if reference[i] != with_blocks[i] else 0
		_check("weather off: %s bit-identical to the shader without its cloud blocks" % ["land" if material
				== _terrain_material else "sea"], differing == 0 and reference.size() > 0,
				"%d of %d bytes differ" % [differing, reference.size()])


## The shader source without its CLOUD_LIGHT_BEGIN / _END blocks (what it was before Task 9).
static func _strip_cloud_light(code: String) -> String:
	var out := PackedStringArray()
	var inside := false
	for line in code.split("\n"):
		if line.strip_edges() == "// CLOUD_LIGHT_BEGIN":
			inside = true
		elif line.strip_edges() == "// CLOUD_LIGHT_END":
			inside = false
		elif not inside:
			out.append(line)
	return "\n".join(out)


## The terrain quad at point (normal = the lighting's normal), seen from camera (the quad fills the view); mode: the
## cloud debug output. Returns the centre 5 x 5 mean.
func _render_ground(camera_itrf_m: Vector3, point_itrf_m: Vector3, normal_itrf: Vector3, sun_itrf: Vector3, mode: int,
		material: ShaderMaterial) -> Vector3:
	var sun_local := Frames.to_render(sun_itrf * AU_M)
	ShaderGlobals.set_value(&"earth_sun_position_local", sun_local)
	ShaderGlobals.set_value(&"earth_moon_position_local", -Frames.to_render(sun_itrf) * MOON_DISTANCE_M)
	var view_itrf := (point_itrf_m - camera_itrf_m).normalized()
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var center := Frames.to_render(point_itrf_m)
	var distance := (point_itrf_m - camera_itrf_m).length()
	_terrain_quad.material_override = material
	_terrain_parent.transform = Transform3D(Frames.basis_to_render(_earth_to_gcrf), Frames.to_render(-camera_gcrf))
	_terrain_quad.mesh = _facing_quad(Frames.to_render(view_itrf), Frames.to_render(normal_itrf), distance * 0.2)
	_terrain_quad.position = center
	var split := TerrainQuadtree.split_float32(center)
	_terrain_quad.set_instance_shader_parameter(&"vertex_offset", split[0])
	_terrain_quad.set_instance_shader_parameter(&"vertex_offset_lo", split[1])
	var camera_split := TerrainQuadtree.split_float32(Frames.to_render(camera_itrf_m))
	ShaderGlobals.set_value(&"earth_terrain_camera_hi", camera_split[0])
	ShaderGlobals.set_value(&"earth_terrain_camera_lo", camera_split[1])
	_terrain_parent.visible = true
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf, mode, BLOCK_RADIUS)
	_terrain_parent.visible = false
	return got


## A quad facing the view (atmosphere_check.gd), its normals the lighting's normal.
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


## Section 7 (Task 9 fix round 1): the shadow maps' hand-overs and the shipped shapes on broken cloud, through
## cloud_light_probe.gdshader (the surfaces' own cloud-light functions at receivers around a camera 2 m up). Receivers
## on annuli around the camera (area-uniform), the Sun at azimuth 135 deg. One site holds one realisation of the shape
## noise (its cells are kilometres to tens of km across: a disc of 8 km sees a few of them), so every number is the mean
## over SITES (~55 km apart: independent realisations), which brings the sampling error down by sqrt(8).
func _handover_samples() -> void:
	_clouds.shadow_effect.grid_mean_only = false
	# a. Hand-overs: the direct Sun's mean over 1 km annuli at 8 / 10 / 12 km (near map -> far map, 9-11 km) and over
	# 4 km annuli at 110 / 130 / 150 km (far map -> the slanted column, 120-140 km), a broken layer and a deck.
	for case: Array in [["cf 0.3 broken", 0.3, 2.0 * DECK_LIQUID_KG_M3], ["deck cf 1", 1.0, DECK_LIQUID_KG_M3]]:
		_show(_slab("handover_%d" % int(case[1] * 10.0), SLAB_BOTTOM_M, SLAB_TOP_M, case[2], case[1]))
		for elevation in [45.0, 8.0]:
			var label := "%s, Sun %.0f deg" % [case[0], elevation]
			var near_bands: Array = await _sites_mean(elevation, [[7000.0, 9000.0, 32, 720], [9000.0, 11000.0, 32, 720],
					[11000.0, 13000.0, 32, 720]], SITES.size())
			var far_bands: Array = await _sites_mean(elevation, [[108000.0, 112000.0, 12, 360],
					[128000.0, 132000.0, 12, 360], [148000.0, 152000.0, 12, 360]], 1)
			var step_a: float = absf(near_bands[1].direct - near_bands[0].direct)
			var step_b: float = absf(near_bands[2].direct - near_bands[1].direct)
			var allowed_a: float = HANDOVER_TOLERANCE + 3.0 * sqrt(pow(near_bands[0].direct_se, 2.0) + pow(near_bands[1].direct_se, 2.0))
			var allowed_b: float = HANDOVER_TOLERANCE + 3.0 * sqrt(pow(near_bands[1].direct_se, 2.0) + pow(near_bands[2].direct_se, 2.0))
			_check("hand-over 9-11 km, no ring, " + label, step_a <= allowed_a and step_b <= allowed_b,
					"direct at 8 / 10 / 12 km %.4f %.4f %.4f (steps %.4f %.4f, allowed %.4f %.4f; near map %.4f %.4f %.4f, far map %.4f %.4f %.4f)" % [
					near_bands[0].direct, near_bands[1].direct, near_bands[2].direct, step_a, step_b, allowed_a, allowed_b,
					near_bands[0].near, near_bands[1].near, near_bands[2].near, near_bands[0].far,
					near_bands[1].far, near_bands[2].far])
			var far_a: float = absf(far_bands[1].direct - far_bands[0].direct)
			var far_b: float = absf(far_bands[2].direct - far_bands[1].direct)
			var column_gap: float = absf(far_bands[0].far - far_bands[0].column)
			_check("hand-over 120-140 km, no ring, " + label,
					maxf(maxf(far_a, far_b), column_gap) <= FAR_HANDOVER_TOLERANCE,
					"direct at 110 / 130 / 150 km %.4f %.4f %.4f (steps %.4f %.4f); far map %.4f vs the slanted column %.4f at 110 km" % [
					far_bands[0].direct, far_bands[1].direct, far_bands[2].direct, far_a, far_b, far_bands[0].far,
					far_bands[0].column])
	# b. The shipped near map (the shapes) on broken cloud over the disc inside the near tier (r < 8 km): the area mean
	# of the ground factors against the CPU's cell factors, the vehicle's light against the ground's.
	for fraction in [0.15, 0.3, BROKEN_FRACTION]:
		_show(_slab("disc_%d" % int(fraction * 100.0), SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, fraction))
		for elevation in [45.0, 8.0]:
			var label := "cf %.2f, Sun %.0f deg" % [fraction, elevation]
			var disc: Dictionary = (await _sites_mean(elevation, [[200.0, DISC_RADIUS_M, 40, 720]], SITES.size()))[0]
			var camera := _point(2.0)
			var sun := _local_direction(elevation, 135.0)
			var cpu: Dictionary = _core.weather_cloud_light(camera, sun)
			var t_s := float(cpu["sun_transmittance"])
			var t_sd := float(cpu["sun_diffuse"])
			var t_d := float(cpu["sky_factor"])
			_check("shapes: area mean of the direct Sun vs the CPU's cell " + label,
					absf(disc.direct - t_s) <= DISC_TOLERANCE * maxf(t_s, 0.1) and disc.direct <= 1.0,
					"ground %.4f (near map %.4f, far map %.4f) CPU %.4f" % [disc.direct, disc.near, disc.far, t_s])
			_check("shapes: the near map's mean vs the far map's over the same receivers " + label,
					absf(disc.near - disc.far) <= DISC_TOLERANCE * maxf(disc.far, 0.1),
					"near %.4f far %.4f" % [disc.near, disc.far])
			_check("ground: the column terms vs the CPU " + label,
					absf(disc.diffuse - t_sd) <= FACTOR_TOLERANCE * maxf(t_sd, 0.01)
					and absf(disc.sky - t_d) <= FACTOR_TOLERANCE * t_d,
					"Sun's diffuse %.4f (CPU %.4f) sky %.4f (CPU %.4f)" % [disc.diffuse, t_sd, disc.sky, t_d])
			var lux: Dictionary = _core.atmosphere_illuminance(camera, _local_direction(90.0, 0.0), sun)
			var e_sun: Vector3 = lux["sun_lux"]
			var e_sky: Vector3 = lux["sky_lux"]
			var sun_share := _luminance(e_sun) / _luminance(e_sun + e_sky)
			var ground_total: float = (disc.direct + disc.diffuse) * sun_share + disc.sky * (1.0 - sun_share)
			var vehicle_total := (t_s + t_sd) * sun_share + t_d * (1.0 - sun_share)
			_check("vehicle light (CPU) vs the ground's mean (shapes) " + label,
					absf(vehicle_total - ground_total) <= VEHICLE_TOLERANCE * ground_total,
					"vehicle %.4f ground %.4f (Sun share %.3f)" % [vehicle_total, ground_total, sun_share])


## The annulus means (see _annulus_means) averaged over the first site_count SITES; ranges: [[r0 m, r1 m, rings,
## azimuths], ...]. Returns one Dictionary per range.
func _sites_mean(elevation_deg: float, ranges: Array, site_count: int) -> Array:
	var sums: Array = []
	var squares: Array[float] = []
	for r in ranges:
		sums.append({})
		squares.append(0.0)
	for s in range(site_count):
		var lat: float = LAT_DEG + SITES[s].x
		var lon: float = LON_DEG + SITES[s].y
		var camera := HeightSource.geodetic_to_fixed(lat, lon, 2.0)
		var sun := _local_direction(elevation_deg, 135.0, lat, lon)
		await _render(camera, sun, sun, 0)
		for i in range(ranges.size()):
			var m: Dictionary = await _annulus_means(camera, lat, lon, ranges[i][0], ranges[i][1], ranges[i][2],
					ranges[i][3])
			for key in m:
				sums[i][key] = float(sums[i].get(key, 0.0)) + float(m[key]) / float(site_count)
			squares[i] += float(m["direct"]) * float(m["direct"]) / float(site_count)
	for i in range(ranges.size()):
		# The standard error of the mean over the sites (the sites' spread is the shape noise's sampling error).
		var variance := maxf(squares[i] - float(sums[i]["direct"]) * float(sums[i]["direct"]), 0.0)
		sums[i]["direct_se"] = sqrt(variance / maxf(float(site_count - 1), 1.0)) if site_count > 1 else 0.0
	return sums


func _slant_table() -> void:
	_clouds.shadow_effect.grid_mean_only = false
	for fraction in [0.15, 0.3, 0.4, 0.6]:
		_show(_slab("table_%d" % int(fraction * 100.0), SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, fraction))
		for elevation in [75.0, 60.0, 45.0, 30.0, 20.0, 12.0, 8.0, 5.0]:
			var disc: Dictionary = (await _sites_mean(elevation, [[200.0, DISC_RADIUS_M, 40, 720]], SITES.size()))[0]
			print("slant table: cf %.2f elevation %4.1f  T %.4f  C_eff %.4f  n %.3f" % [fraction, elevation, disc.direct,
					1.0 - disc.direct, log(1.0 - (1.0 - disc.direct)) / log(1.0 - fraction)])


func _setup_probe() -> void:
	_probe_viewport.use_hdr_2d = true
	_probe_viewport.own_world_3d = true
	_probe_viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	root.add_child(_probe_viewport)
	var camera := Camera3D.new()
	camera.current = true
	_probe_viewport.add_child(camera)
	var shader := load("res://tests/cloud_light_probe.gdshader") as Shader
	_probe_material.shader = shader
	var quad := MeshInstance3D.new()
	var mesh := QuadMesh.new()
	mesh.size = Vector2(2.0, 2.0)
	quad.mesh = mesh
	quad.material_override = _probe_material
	quad.position = Vector3(0.0, 0.0, -1.0)
	quad.extra_cull_margin = 16384.0
	_probe_viewport.add_child(quad)


## The Earth-fixed point at arc distance s (m) from the camera's site toward the azimuth (from north, east positive),
## height 0 (spherical destination on the mean radius: the distance is nominal, to 0.3 %).
static func _destination(site_lat_deg: float, site_lon_deg: float, distance_m: float, azimuth_rad: float) -> Vector3:
	var lat := deg_to_rad(site_lat_deg)
	var delta := distance_m / 6.371e6
	var lat2 := asin(sin(lat) * cos(delta) + cos(lat) * sin(delta) * cos(azimuth_rad))
	var lon2 := deg_to_rad(site_lon_deg) + atan2(sin(azimuth_rad) * sin(delta) * cos(lat),
			cos(delta) - sin(lat) * sin(lat2))
	return HeightSource.geodetic_to_fixed(rad_to_deg(lat2), rad_to_deg(lon2), 0.0)


## The area mean of the surfaces' cloud light over the annulus r0..r1 around the camera (rings x azimuths, weight r):
## {direct (the distance rule), near, far (the maps alone), column (slanted to the Sun), diffuse, sky}.
func _annulus_means(camera_itrf: Vector3, site_lat_deg: float, site_lon_deg: float, r0: float, r1: float, rings: int,
		azimuths: int) -> Dictionary:
	var offsets := PackedVector3Array()
	var weights := PackedFloat64Array()
	for i in range(rings):
		var r := r0 + (float(i) + 0.5) / float(rings) * (r1 - r0)
		for k in range(azimuths):
			var azimuth := TAU * (float(k) + 0.5 + 0.37 * float(i)) / float(azimuths)  # staggered rings
			offsets.append(Frames.to_render(_destination(site_lat_deg, site_lon_deg, r, azimuth) - camera_itrf))
			weights.append(r)
	var first := await _probe(offsets, 0)
	var second := await _probe(offsets, 1)
	var sum := PackedFloat64Array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	var total := 0.0
	for i in range(offsets.size()):
		var w := weights[i]
		total += w
		sum[0] += w * first[i].x
		sum[1] += w * first[i].y
		sum[2] += w * first[i].z
		sum[3] += w * second[i].x
		sum[4] += w * second[i].y
		sum[5] += w * second[i].z
	return {"direct": sum[0] / total, "near": sum[1] / total, "far": sum[2] / total, "column": sum[3] / total,
			"diffuse": sum[4] / total, "sky": sum[5] / total}


## Renders the probe shader over the receivers (camera-relative offsets, Earth-local render axes): one pixel each.
func _probe(offsets: PackedVector3Array, channel: int) -> PackedVector3Array:
	var width := PROBE_WIDTH
	var height := (offsets.size() - 1) / width + 1
	var floats := PackedFloat32Array()
	floats.resize(width * height * 4)
	for i in range(offsets.size()):
		floats[4 * i] = offsets[i].x
		floats[4 * i + 1] = offsets[i].y
		floats[4 * i + 2] = offsets[i].z
		floats[4 * i + 3] = 1.0
	var image := Image.create_from_data(width, height, false, Image.FORMAT_RGBAF, floats.to_byte_array())
	_probe_material.set_shader_parameter(&"receivers", ImageTexture.create_from_image(image))
	_probe_material.set_shader_parameter(&"channel", channel)
	_probe_viewport.size = Vector2i(width, height)
	_probe_viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
	await RenderingServer.frame_post_draw
	await RenderingServer.frame_post_draw
	var result := _probe_viewport.get_texture().get_image()
	var out := PackedVector3Array()
	for i in range(offsets.size()):
		var c := result.get_pixel(i % width, i / width)
		out.append(Vector3(c.r, c.g, c.b))
	return out


func _point(height_m: float) -> Vector3:
	return HeightSource.geodetic_to_fixed(LAT_DEG, LON_DEG, height_m)


## ITRF unit vector at elevation above the geodetic horizon and azimuth (from north, east positive) at the site.
static func _local_direction(elevation_deg: float, azimuth_deg: float, lat_deg := LAT_DEG,
		lon_deg := LON_DEG) -> Vector3:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
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
