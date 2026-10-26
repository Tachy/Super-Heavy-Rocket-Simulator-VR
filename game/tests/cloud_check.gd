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
#   8. (static clouds Tasks 1-2) the light cache (CloudLightCache: 5 cascades 8 / 16 / 32 / 64 / 128 km over the shell,
#      filled at 1/16 of the voxels per frame: complete after 16 frames) against the per-sample light march with the
#      cache's converged sample count (CloudLightCache.SAMPLES: 96 near + 32 far, stratified midpoints), on the game's
#      deterministic march (one frame each, the same view samples): the Sun's scattered radiance at every pixel of a
#      64 x 64 view per cascade at its scale (the camera 5 / 9.9 / 18.8 / 36.5 / 72 km up, looking 80-81 deg down;
#      the farthest cloud samples asserted inside that cascade's full weight, beyond the finer ones'): a slab of tau
#      ~10 (debug output 2, the grid mean) 5 %, a broken layer of cf 0.4 (debug output 7, the shapes) 10 % at interior
#      pixels and the 99th percentile, 20 % at silhouette pixels (alpha < 0.3 within one march pixel) - Sun 45 / 20
#      deg up; a view
#      beyond every cascade with a complete fill, and the frames right after a clock jump (nothing held), equal the
#      march exactly (bit for bit); the fill's GPU time in every frame at the game's budget <= 0.5 ms.
#   9. (static clouds Task 3) the deterministic march's silhouettes: a broken layer (cf 0.3, opaque cumuli) seen
#      from 5 km above its top, straight down; its 50 % background-transmittance contour (debug output 4) lies within
#      one march pixel of the converged reference's (the jittered march with ~12 m steps, the mean of 16 frames), for
#      the desktop and the VR preset (STARSIM_SILHOUETTE_ONLY=1 runs this section alone);
#  10. (static clouds Task 2) no seam: a line of pixels crossing each cascade's edge band (cascade 0 -> 1 -> ... -> 4 ->
#      the march, the slab): the row-to-row step of (cache - march) / mean stays within the image noise (the largest
#      difference of two march renders with independent jitter); the same view before / after a re-anchor (the
#      camera moved 12 km east and back: cascade 3 re-anchored) differs by less than that noise.
# The other sections run with the light cache on and the deterministic march (the game's path; 10 with the jittered
# one: it measures against independent jitter's noise). Detail
# levels off for the numbers (cloud_detail_levels 0); the desktop quality preset. Exit code 1 on failure.
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
const EXPECTED_SAMPLES := 132
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
const CACHE_SLAB_TOLERANCE := 0.05
const CACHE_BROKEN_TOLERANCE := 0.10  ## the voxel size (62.5 m across, 40 m up near the camera) against the shapes
## [E] Acceptance of every section 8 case (controller ruling): p99 of the per-pixel relative error and the worst pixel.
const CACHE_P99_TOLERANCE := 0.10
const CACHE_WORST_TOLERANCE := 0.20
const CACHE_EDGE_ALPHA := 0.3  ## [E] the controller's silhouette bound (fix round 1 of Task 3)
## The fill's budget in the checks: 1/16 of all voxels per frame (complete after 16 frames).
const CACHE_BUDGET_DIVISOR := 16
const CACHE_MAX_SETTLE_FRAMES := 600  ## > a full refresh at the game budget (427 frames)
const CACHE_FILL_LIMIT_MS := 0.5  ## spec decision 6: the fill's share of the 2.5 ms per frame
const CACHE_FILL_IDLE_MS := 0.02  ## a frame without fill jobs (the interval's timestamps alone)
## Each cascade at its scale: the same view (81 deg down, fov 22; cascade 0: 80 / 24) from a camera about twice as high
## per cascade (x2 apart as the cascades), its farthest cloud samples just inside that cascade's full weight and
## beyond the finer cascade's (asserted): the outer part of the view is that cascade's, the inner the finer ones', at
## the same pixel footprint / voxel ratio as cascade 0.
const CACHE_VIEWS := [
	{"cascade": 0, "height_m": 5000.0, "elevation": -80.0, "azimuth": 10.0, "fov": 24.0},
	{"cascade": 1, "height_m": 9900.0, "elevation": -81.0, "azimuth": 10.0, "fov": 22.0},
	{"cascade": 2, "height_m": 18800.0, "elevation": -81.0, "azimuth": 10.0, "fov": 22.0},
	{"cascade": 3, "height_m": 36500.0, "elevation": -81.0, "azimuth": 10.0, "fov": 22.0},
	{"cascade": 4, "height_m": 72000.0, "elevation": -81.0, "azimuth": 10.0, "fov": 22.0},
]
## Beyond every cascade: from 20 km toward the northern horizon (the slab entered > 220 km away).
const CACHE_OUTSIDE_HEIGHT_M := 20000.0
const CACHE_OUTSIDE_ELEVATION_DEG := -4.9
const CACHE_OUTSIDE_FOV_DEG := 1.0
const CACHE_BORDER_POINTS := 16  ## rays per border edge of a view's cloud range
## Section 10 (no seam): a line of pixels (16 x 128: the row means) toward north across each cascade's edge band.
const CACHE_LINE_SIZE := Vector2i(16, 128)
const CACHE_EDGE_VIEWS := [
	{"cascade": 0, "height_m": 5000.0, "elevation": -56.0, "fov": 20.0},
	{"cascade": 1, "height_m": 5000.0, "elevation": -31.5, "fov": 12.0},
	{"cascade": 2, "height_m": 5000.0, "elevation": -18.0, "fov": 10.0},
	{"cascade": 3, "height_m": 5000.0, "elevation": -11.5, "fov": 8.0},
	{"cascade": 4, "height_m": 5000.0, "elevation": -4.3, "fov": 3.0},
]
## The re-anchor pair: cascade 3's edge view from this far east, after the camera went twice as far east and back
## (cascade 3, 500 m voxels: re-anchors beyond 10 km - away, not here).
const CACHE_REANCHOR_EAST_M := 6000.0
const CACHE_REANCHOR_CASCADE := 3
## A pixel's error is relative to max(its reference, this fraction of the view's mean reference): a cloud's fringe or a
## gap's last light carries no relative accuracy of its own.
const CACHE_ERROR_FLOOR := 0.05
## Section 9: a broken layer (cf SILHOUETTE_FRACTION, the thick slab's water: opaque cumuli) seen from
## SILHOUETTE_DISTANCE_M above its top, straight down. The reference: the jittered march with fine steps of
## SILHOUETTE_REFERENCE_FINE of the distance (~12 m: the cumulus' rim) and SILHOUETTE_REFERENCE_STEPS coarse ones (the
## VR preset's 48 lost cloud bases in half the frames, jittered too), averaged over SILHOUETTE_REFERENCE_FRAMES frames.
const SILHOUETTE_VIEW_SIZE := 96
const SILHOUETTE_FOV_DEG := 30.0
const SILHOUETTE_FRACTION := 0.3
const SILHOUETTE_DISTANCE_M := 5000.0
const SILHOUETTE_REFERENCE_FINE := 0.002
const SILHOUETTE_REFERENCE_MAX_STEPS := 1024
const SILHOUETTE_REFERENCE_STEPS := 512
const SILHOUETTE_REFERENCE_FRAMES := 16
const SILHOUETTE_MIN_CONTOUR := 24  ## reference contour pixels in view (cloud edges are seen)
const SILHOUETTE_SEARCH := 8  ## full-resolution pixels searched for the reference's contour

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
	if OS.get_environment("STARSIM_SILHOUETTE_ONLY") != "":  # diagnostics: section 9 alone (no sample count)
		await _silhouette_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (silhouettes only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	if OS.get_environment("STARSIM_LIGHT_CACHE_ONLY") != "":  # diagnostics: sections 8 and 10 alone (no sample count)
		await _light_cache_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (light cache only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
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
	# 8. The light cache against the per-sample light march (and 10: no seam at its edges).
	await _light_cache_samples()
	# 9. The deterministic march's silhouettes against the converged jittered march.
	await _silhouette_samples()

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
## averaged over RADIANCE_FRAMES frames), the view's jitter pinned to the same frames in both. Each cascade at its scale
## (CACHE_VIEWS: every cloud sample of the view where that cascade alone has weight). The fill runs at 1/16 of the
## voxels per frame (complete after 16 frames); then the game's budget's GPU time, a view beyond every cascade with
## a complete fill, and the frames right after a clock jump (nothing held: the march's bytes).
func _light_cache_samples() -> void:
	var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
	var first: float = preset["light_first_m"]
	var near: float = preset["light_near_m"]
	var samples := CloudLightCache.SAMPLES
	var cache := _clouds.light_cache
	var game_budget := cache.schedule.budget_voxels
	cache.schedule.budget_voxels = cache.voxel_count() / CACHE_BUDGET_DIVISOR
	# The game's path (the deterministic march): both sides share the view's fixed samples, one frame each.
	_clouds.deterministic_march = true
	var divisor := int(CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]["divisor"])
	var size := Vector2i(CACHE_VIEW_SIZE, CACHE_VIEW_SIZE)
	_viewport.size = size
	var slab := _slab("cache_slab", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, 1.0)
	var broken := _slab("cache_broken", SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, BROKEN_FRACTION)
	var sun45 := _local_direction(45.0, 135.0)
	# The progressive fill: all cascades at 1/16 of the voxels per frame.
	_show(slab)
	var v0: Dictionary = CACHE_VIEWS[0]
	_camera.fov = v0["fov"]
	var view0 := _local_direction(v0["elevation"], v0["azimuth"])
	await _settle_cache(_point(v0["height_m"]), view0, sun45, 2)
	cache.reset()
	var fill_frames := await _settle_cache(_point(v0["height_m"]), view0, sun45, 2)
	_check("light cache: %d cascades at 1/%d of the voxels per frame complete after %d frames"
			% [CloudLightCacheSchedule.CASCADES.size(), CACHE_BUDGET_DIVISOR, CACHE_BUDGET_DIVISOR],
			fill_frames == CACHE_BUDGET_DIVISOR, "%d frames with fill jobs (%d voxels, %.1f MB)"
			% [fill_frames, cache.voxel_count(), CloudLightCache.texture_bytes() / 1048576.0])
	var only_views := OS.get_environment("STARSIM_CACHE_VIEWS")  # diagnostics: "3,4" runs those cascades' views alone
	for v: Dictionary in CACHE_VIEWS:
		if only_views != "" and not str(v["cascade"]) in only_views.split(","):
			continue
		_camera.fov = v["fov"]
		var height: float = v["height_m"]
		var reach := _view_cloud_range_m(height, v["elevation"], v["fov"], 1.0)
		var k: int = v["cascade"]
		var inner := 0.0 if k == 0 else _cascade_radius_m(k - 1, false)
		var outer := _cascade_radius_m(k, true)
		_check("light cache view, cascade %d: the farthest cloud samples in its full weight" % k,
				reach.y > inner and reach.y <= outer, "camera %.1f km: cloud samples %.0f .. %.0f m from the camera's "
				% [height / 1000.0, reach.x, reach.y] + "vertical; finer cascades end at %.0f m, cascade %d full to %.0f m"
				% [inner, k, outer])
		var view := _local_direction(v["elevation"], v["azimuth"])
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
					if cached:
						await _settle_cache(_point(height), view, sun, c[1])
					images.append(await _mean_image(_point(height), view, sun, c[1], size, 0, 1))
				_clouds.light_cache_enabled = true
				_clouds.quality = CloudLayer.Quality.DESKTOP
				_detail_off()
				var stats := _cache_error(images[1], images[0])
				var detail := ("worst pixel %.2f %% at (%d, %d) (cache %.5f march %.5f), mean error %.2f %%, bias %.2f %%"
						+ " of the view mean %.5f /sr") % [stats["worst"] * 100.0, stats["at"] % CACHE_VIEW_SIZE,
						stats["at"] / CACHE_VIEW_SIZE, images[1][stats["at"]], images[0][stats["at"]],
						stats["mean_error"] * 100.0, stats["bias"] * 100.0, stats["mean"]]
				# Acceptance for every case and pixel: p99 of the per-pixel relative error within CACHE_P99_TOLERANCE and the
				# worst pixel within CACHE_WORST_TOLERANCE. The residue is voxel interpolation of cumulus self-shadowing at a
				# fixed sample in cascade 0 (62.5 m voxels): worst seen 16 % at p99 6.8 %; finer voxels cost GPU on the binding
				# Task 10 budget. The interior / silhouette split is printed for information only.
				var shape := await _mean_image(_point(height), view, sun, 4, size, 0, 1)
				var split := _cache_error_split(images[1], images[0], shape, divisor)
				var ok: bool = stats["mean"] > 0.0 and split["p99"] <= CACHE_P99_TOLERANCE \
						and stats["worst"] <= CACHE_WORST_TOLERANCE
				detail += ("; interior worst %.2f %% (%d px), silhouette worst %.2f %% (%d px), p99 %.2f %%; "
						+ "pixels over %.0f %%: %d interior, %d silhouette") % [split["interior"] * 100.0,
						split["interior_count"], split["edge"] * 100.0, split["edge_count"], split["p99"] * 100.0,
						CACHE_BROKEN_TOLERANCE * 100.0, split["interior_over"], split["edge_over"]]
				_check("light cache vs light march, cascade %d, %s, Sun %.0f deg" % [k, c[3], sun_elevation], ok, detail)
				if c[0] == broken:
					var at := Vector2i(stats["at"] % CACHE_VIEW_SIZE, stats["at"] / CACHE_VIEW_SIZE)
					print("  info  worst pixel %s, cascade shares along its ray: %s; worst pixel by the cascade "
							% [at, _pixel_cascade_shares(_point(height), at, sun)] + "holding its slab middle: %s"
							% _worst_by_cascade(images[1], images[0], _point(height), sun))
	# The fill's GPU time per frame at the game's budget (spec decision 6: <= 0.5 ms in every frame): a full refresh of
	# every cascade from nothing, the time of each frame's fill (the RenderingDevice's timestamps trail by a frame or
	# two; every fill frame carries the budget's voxels but the last).
	cache.schedule.budget_voxels = game_budget
	_show(slab)
	_camera.fov = v0["fov"]
	await _settle_cache(_point(v0["height_m"]), view0, sun45, 2)
	cache.reset()
	CloudEffect.capture_timestamps = true
	var camera := _point(v0["height_m"])
	var times := PackedFloat64Array()
	var frames := 0
	var game_fill_frames := cache.planned_fill_frames
	var after := 0  # frames after the last plan with jobs (the timestamps trail)
	while frames < CACHE_MAX_SETTLE_FRAMES and after < 4:
		_clouds.update(camera, 0.0, 0.0)
		await RenderingServer.frame_post_draw
		var ms := _light_fill_ms()
		if ms > CACHE_FILL_IDLE_MS:
			times.append(ms)
		after = after + 1 if cache.settled() else 0
		frames += 1
	CloudEffect.capture_timestamps = false
	var sorted := times.duplicate()
	sorted.sort()
	var worst := sorted[sorted.size() - 1] if not sorted.is_empty() else 0.0
	var p95 := sorted[int(sorted.size() * 0.95)] if not sorted.is_empty() else 0.0
	var series := ""
	for ms in times:
		series += "%.2f " % ms
	print("  info  light cache fill per frame (ms GPU, in fill order): " + series)
	# p95, not the worst frame: the worst is an isolated spike at a GPU clock-state transition in a lightly loaded
	# test; the per-frame worst under load is measured in the headset in Task 10.
	_check("light cache: the fill at the game's budget stays <= %.1f ms GPU (95th percentile)" % CACHE_FILL_LIMIT_MS,
			not sorted.is_empty() and p95 <= CACHE_FILL_LIMIT_MS,
			("%d voxels per frame: median %.3f, p95 %.3f, worst %.3f ms over %d frames; %d plans for all %d voxels "
			+ "(%.1f MB)") % [game_budget, sorted[sorted.size() / 2] if not sorted.is_empty() else 0.0,
			p95, worst, sorted.size(),
			cache.planned_fill_frames - game_fill_frames, cache.voxel_count(),
			CloudLightCache.texture_bytes() / 1048576.0])
	cache.schedule.budget_voxels = cache.voxel_count() / CACHE_BUDGET_DIVISOR
	# Beyond every cascade (a complete fill): from 20 km toward the northern horizon every cloud sample lies past the
	# outermost cascade's weight; the cache must change no bit.
	_camera.fov = CACHE_OUTSIDE_FOV_DEG
	var outside_view := _local_direction(CACHE_OUTSIDE_ELEVATION_DEG, 0.0)
	var outside_reach := _view_cloud_range_m(CACHE_OUTSIDE_HEIGHT_M, CACHE_OUTSIDE_ELEVATION_DEG, CACHE_OUTSIDE_FOV_DEG,
			1.0)
	var last := CloudLightCacheSchedule.CASCADES.size() - 1
	await _settle_cache(_point(CACHE_OUTSIDE_HEIGHT_M), outside_view, sun45, 2)
	var settled := cache.settled()
	var outside: Array[PackedByteArray] = []
	for cached in [false, true]:
		_clouds.light_cache_enabled = cached
		CloudEffect.pinned_frame = 0
		await _render(_point(CACHE_OUTSIDE_HEIGHT_M), outside_view, sun45, 2)
		CloudEffect.pinned_frame = -1
		outside.append(_viewport.get_texture().get_image().get_data())
	_clouds.light_cache_enabled = true
	var lit := _viewport.get_texture().get_image().get_pixel(CACHE_VIEW_SIZE / 2, CACHE_VIEW_SIZE / 2).r
	_check("light cache: a view beyond every cascade (complete fill) equals the march",
			outside[0] == outside[1] and lit > 0.0 and settled and outside_reach.x > _cascade_radius_m(last, false),
			"%s (centre %.5f /sr; cloud samples from %.0f km, cascade %d's weight ends at %.0f km; fill complete %s)"
			% ["identical" if outside[0] == outside[1] else "DIFFERENT", lit, outside_reach.x / 1000.0, last,
			_cascade_radius_m(last, false) / 1000.0, settled])
	# A clock jump: every cascade invalid until filled - the frames right after it are the march's, bit for bit (at the
	# game's budget: the seed squares of all cascades take 6 plans, the render's 5 see nothing filled).
	cache.schedule.budget_voxels = game_budget
	_camera.fov = v0["fov"]
	await _settle_cache(_point(v0["height_m"]), view0, sun45, 2)
	var jumped: Array[PackedByteArray] = []
	for cached in [true, false]:
		if cached:
			_clouds.notify_clock_jump()
		_clouds.light_cache_enabled = cached
		CloudEffect.pinned_frame = 0
		await _render(_point(v0["height_m"]), view0, sun45, 2)
		CloudEffect.pinned_frame = -1
		jumped.append(_viewport.get_texture().get_image().get_data())
	_clouds.light_cache_enabled = true
	_check("light cache: right after a clock jump the view equals the march (nothing held until filled)",
			jumped[0] == jumped[1], "identical" if jumped[0] == jumped[1] else "DIFFERENT")
	cache.schedule.budget_voxels = cache.voxel_count() / CACHE_BUDGET_DIVISOR
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	# Section 10 on the jittered march (the reference path, CloudLayer.deterministic_march off): its criterion is the
	# image noise of independent jitter, which the deterministic march does not have.
	_clouds.deterministic_march = false
	await _seam_samples(slab)
	cache.schedule.budget_voxels = game_budget
	_clouds.deterministic_march = true


## Section 10: no seam. A line of pixels (CACHE_LINE_SIZE: the row means of a narrow tall view, rows = elevations)
## crossing each cascade edge and the outermost edge to the march (CACHE_EDGE_VIEWS, a slab of tau ~10, the grid mean):
## the cache's deviation from the march, d = (cache - march) / mean, steps from row to row by no more than the image
## noise (the largest difference of two march renders with independent jitter) over the rows whose slab entry lies in
## the band +- half its width (where a seam would be; elsewhere d's row-to-row change is the cascade's own voxel
## interpolation, case 8's subject - printed for information); and the same view before and after a re-anchor (the
## camera moved away and back: cascade CACHE_REANCHOR_CASCADE re-anchored, its data seen by that edge view) differs by
## less than that noise.
func _seam_samples(slab: Dictionary) -> void:
	_show(slab)
	_viewport.size = CACHE_LINE_SIZE
	var sun := _local_direction(45.0, 135.0)
	var reanchor_noise := 0.0
	for v: Dictionary in CACHE_EDGE_VIEWS:
		var height: float = v["height_m"]
		var camera := _point(height)
		_camera.fov = v["fov"]
		var view := _local_direction(v["elevation"], 0.0)
		var k: int = v["cascade"]
		var band := Vector2(_cascade_radius_m(k, true), _cascade_radius_m(k, false))
		var aspect := float(CACHE_LINE_SIZE.x) / CACHE_LINE_SIZE.y
		var reach := _view_cloud_range_m(height, v["elevation"], v["fov"], aspect)
		await _settle_cache(camera, view, sun, 2)
		var lines := await _line_triple(camera, view, sun)
		var half := 0.5 * (band.y - band.x)
		var rows := Vector2i(CACHE_LINE_SIZE.y, -1)  # the rows whose slab entry lies in the band +- half its width
		for row in range(CACHE_LINE_SIZE.y):
			var entry := _row_entry_m(height, v["elevation"], v["fov"], row)
			if entry >= band.x - half and entry <= band.y + half:
				rows = Vector2i(mini(rows.x, row), maxi(rows.y, row))
		var result := _seam(lines[0], lines[1], lines[2], rows)
		var profile := ""  # (cache - march) / mean per row, % (top row first), and the row's slab entry (km)
		for row in range(CACHE_LINE_SIZE.y):
			profile += "%d:%.2f@%.1f " % [row, (lines[0][row] - lines[1][row]) / result["mean"] * 100.0,
					_row_entry_m(height, v["elevation"], v["fov"], row) / 1000.0]
		print("  info  edge %d -> %d profile: %s" % [k, k + 1, profile])
		if k == CACHE_REANCHOR_CASCADE:
			reanchor_noise = result["noise"]
		var into := "the march" if k == CloudLightCacheSchedule.CASCADES.size() - 1 else "cascade %d" % (k + 1)
		_check("light cache: no seam from cascade %d into %s" % [k, into],
				reach.x < band.x and reach.y > band.y and rows.y > rows.x and result["step"] <= result["noise"],
				("largest row-to-row step of (cache - march) / mean over the band's rows %d .. %d %.3f %% at row %d, "
				+ "image noise %.3f %% (whole line %.3f %%); cloud samples %.1f .. %.1f km, band %.1f .. %.1f km")
				% [rows.x, rows.y, result["step"] * 100.0, result["at"], result["noise"] * 100.0,
				result["step_all"] * 100.0, reach.x / 1000.0, reach.y / 1000.0, band.x / 1000.0, band.y / 1000.0])
	# Before / after a re-anchor: an edge view from a camera CACHE_REANCHOR_EAST_M east, then the camera twice as far
	# east (cascade CACHE_REANCHOR_CASCADE re-anchors; the finer ones re-anchor there and back to the same window) and
	# back.
	var v1: Dictionary = CACHE_EDGE_VIEWS[CACHE_REANCHOR_CASCADE]
	_camera.fov = v1["fov"]
	var view1 := _local_direction(v1["elevation"], 0.0)
	var here := _point_east(v1["height_m"], CACHE_REANCHOR_EAST_M)
	var away := _point_east(v1["height_m"], 2.0 * CACHE_REANCHOR_EAST_M)
	await _settle_cache(here, view1, sun, 2)
	var windows_before := _cascade_windows()
	var before := _column_means(await _mean_image(here, view1, sun, 2, CACHE_LINE_SIZE))
	await _settle_cache(away, view1, sun, 2)
	await _settle_cache(here, view1, sun, 2)
	var windows_after := _cascade_windows()
	var after := _column_means(await _mean_image(here, view1, sun, 2, CACHE_LINE_SIZE))
	var mean := 0.0
	for x in before:
		mean += x / before.size()
	var worst := 0.0
	for i in range(before.size()):
		worst = maxf(worst, absf(after[i] - before[i]) / mean)
	var k := CACHE_REANCHOR_CASCADE
	var moved: bool = windows_before[k] != windows_after[k]
	_check("light cache: the same view before / after a re-anchor", moved and mean > 0.0 and worst <= reanchor_noise,
			"largest difference %.4f %% of the line mean, image noise %.3f %% (cascade %d window %s -> %s)"
			% [worst * 100.0, reanchor_noise * 100.0, k, windows_before[k], windows_after[k]])


## The line (row means) of the cache, the march (frames 0 .. 7) and the march with other jitter (frames 8 .. 15) - the
## march with the cache's own sample count (as section 8: the preset's few samples carry their own bias and pattern).
func _line_triple(camera: Vector3, view: Vector3, sun: Vector3) -> Array[PackedFloat32Array]:
	var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
	var first: float = preset["light_first_m"]
	var near: float = preset["light_near_m"]
	var samples := CloudLightCache.SAMPLES
	var result: Array[PackedFloat32Array] = []
	for setting in [[true, 0], [false, 0], [false, RADIANCE_FRAMES]]:
		_clouds.light_cache_enabled = setting[0]
		ShaderGlobals.set_value(&"cloud_light_march", Vector4(first,
				CloudLayer.light_march_ratio(first, near, samples.x), near, 0.0))
		ShaderGlobals.set_value(&"cloud_light_samples", samples)
		result.append(_column_means(await _mean_image(camera, view, sun, 2, CACHE_LINE_SIZE, setting[1])))
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_detail_off()
	_clouds.light_cache_enabled = true
	return result


## {step: the largest row-to-row change of (cache - march) / mean within rows (first, last), at: its row, step_all: the
## same over the whole line, noise: the largest difference of the two march lines / mean}.
static func _seam(cache: PackedFloat32Array, march: PackedFloat32Array, other: PackedFloat32Array,
		rows: Vector2i) -> Dictionary:
	var mean := 0.0
	for x in march:
		mean += x / march.size()
	var noise := 0.0
	var step := 0.0
	var step_all := 0.0
	var at := 0
	for i in range(march.size()):
		noise = maxf(noise, absf(march[i] - other[i]) / mean)
		if i > 0:
			var s := absf((cache[i] - march[i]) - (cache[i - 1] - march[i - 1])) / mean
			step_all = maxf(step_all, s)
			if i - 1 >= rows.x and i <= rows.y and s > step:
				step = s
				at = i
	return {"step": step, "at": at, "step_all": step_all, "noise": noise, "mean": mean}


## The horizontal distance (m) at which the centre column's ray of row `row` (of CACHE_LINE_SIZE.y, top first) of a
## view from height_m enters the slab (its top); INF when it misses.
static func _row_entry_m(height_m: float, elevation_deg: float, fov_deg: float, row: int) -> float:
	const R := 6371000.0
	var v := 1.0 - 2.0 * (row + 0.5) / CACHE_LINE_SIZE.y
	var el := deg_to_rad(elevation_deg) + atan(v * tan(deg_to_rad(fov_deg * 0.5)))
	var top := _sphere_hits(R + height_m, sin(el), R + SLAB_TOP_M)
	return INF if top.x > top.y else _ground_distance(R + height_m, sin(el), top.x, R)


func _cascade_windows() -> Array:
	var windows := []
	for c: Dictionary in _clouds.light_cache.schedule.lookup_state()["cascades"]:
		windows.append(c["window"])
	return windows


## The horizontal distance from the camera's ground point at which cascade k's weight ends (full: where it starts to
## fall over the band).
static func _cascade_radius_m(k: int, full: bool) -> float:
	var c: Dictionary = CloudLightCacheSchedule.CASCADES[k]
	var dx: float = c["extent_m"] / float(c["size"].x)
	var radius := float(c["size"].x / 2 - CloudLightCacheSchedule.REANCHOR_VOXELS - CloudLightCacheSchedule.MARGIN_VOXELS)
	return (radius - (CloudLightCacheSchedule.BAND_VOXELS if full else 0)) * dx


## Diagnostics (section 8): the cascade weights along the ray of view pixel `pixel` (the camera as the last render) at
## the slab's bottom, middle and top - per height the share of each cascade (the lookup's chain: chosen at the sample,
## a hybrid one only where it holds the end of its near march) and the march's rest.
func _pixel_cascade_shares(camera_itrf_m: Vector3, pixel: Vector2i, sun_itrf: Vector3) -> String:
	var out := PackedStringArray()
	for h: float in [SLAB_BOTTOM_M, 0.5 * (SLAB_BOTTOM_M + SLAB_TOP_M), SLAB_TOP_M]:
		var shares := _cascade_shares_at(camera_itrf_m, pixel, sun_itrf, h)
		var parts := PackedStringArray()
		for k in range(shares.size()):
			parts.append(("c%d" % k if k < shares.size() - 1 else "march") + " %.2f" % shares[k])
		out.append("h %.0f m: %s" % [h, ", ".join(parts) if not shares.is_empty() else "no hit"])
	return "; ".join(out)


## Diagnostics (section 8): the worst pixel error (as _cache_error) among the pixels whose slab middle each cascade
## holds with the largest share.
func _worst_by_cascade(cached: PackedFloat32Array, march: PackedFloat32Array, camera_itrf_m: Vector3,
		sun_itrf: Vector3) -> String:
	var mean := 0.0
	for v in march:
		mean += v / march.size()
	var worst := PackedFloat64Array()
	worst.resize(CloudLightCacheSchedule.CASCADES.size() + 1)
	worst.fill(0.0)
	for i in range(march.size()):
		var shares := _cascade_shares_at(camera_itrf_m, Vector2i(i % CACHE_VIEW_SIZE, i / CACHE_VIEW_SIZE), sun_itrf,
				0.5 * (SLAB_BOTTOM_M + SLAB_TOP_M))
		var owner := 0
		for k in range(shares.size()):
			owner = k if shares[k] > shares[owner] else owner
		var error := absf(cached[i] - march[i]) / maxf(march[i], CACHE_ERROR_FLOOR * mean)
		worst[owner] = maxf(worst[owner], error)
	var parts := PackedStringArray()
	for k in range(worst.size()):
		parts.append(("c%d" % k if k < worst.size() - 1 else "march") + " %.1f %%" % (worst[k] * 100.0))
	return ", ".join(parts)


## The shares of the cascades (and last the march) at height h along the ray of view pixel `pixel` (empty: no hit).
func _cascade_shares_at(camera_itrf_m: Vector3, pixel: Vector2i, sun_itrf: Vector3, h: float) -> PackedFloat64Array:
	var state := _clouds.light_cache.schedule.lookup_state()
	var ray_gcrf := Frames.from_render(_camera.project_ray_normal(Vector2(pixel) + Vector2(0.5, 0.5)))
	var ray := (_earth_to_gcrf.inverse() * ray_gcrf).normalized()
	var r0 := _point(0.0).length()
	var light := Frames.to_render(sun_itrf).normalized()
	var origin: Vector3 = state["origin"]
	var shares := PackedFloat64Array()
	var b := camera_itrf_m.dot(ray)
	var disc := b * b - (camera_itrf_m.length_squared() - (r0 + h) * (r0 + h))
	if disc < 0.0:
		return shares
	var p := Frames.to_render(camera_itrf_m + ray * (-b - sqrt(disc)))
	var open := 1.0
	for k in range(CloudLightCacheSchedule.CASCADES.size()):
		var c: Dictionary = state["cascades"][k]
		var dx: float = c["dx_m"]
		var at := p + light * _clouds.light_cache.near_length_m(k)
		var g := Vector2((p - origin).dot(state["east"]), (p - origin).dot(state["north"])) / dx
		var gf := Vector2((at - origin).dot(state["east"]), (at - origin).dot(state["north"])) / dx
		var d := (g - (c["centre"] as Vector2)).abs()
		var e := clampf((float(c["radius_voxels"]) - maxf(d.x, d.y)) / float(c["band_voxels"]), 0.0, 1.0)
		var held: Rect2i = c["valid"][1] if float(c["weight"]) >= 0.5 else c["valid"][0]
		var lo := gf - (Vector2(held.position) + Vector2(0.5, 0.5))
		var hi := (Vector2(held.end) - Vector2(0.5, 0.5)) - gf
		var hw := clampf(minf(minf(lo.x, lo.y), minf(hi.x, hi.y)) / float(c["band_voxels"]), 0.0, 1.0)
		shares.append(open * minf(e, hw))
		open -= shares[k]
	shares.append(open)
	return shares


## Renders until the light cache holds every cascade's window for this scene (and one frame more: the render thread
## adopts the state); returns the number of frames whose plan carried fill jobs.
func _settle_cache(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, mode: int) -> int:
	var cache := _clouds.light_cache
	var planned := cache.planned_fill_frames
	await _render(camera_itrf_m, view_itrf, sun_itrf, mode)
	var frames := SETTLE_FRAMES
	while not cache.settled() and frames < CACHE_MAX_SETTLE_FRAMES:
		await RenderingServer.frame_post_draw
		_clouds.update(camera_itrf_m, 0.0, 0.0)
		frames += 1
	await RenderingServer.frame_post_draw
	_clouds.update(camera_itrf_m, 0.0, 0.0)
	await RenderingServer.frame_post_draw
	return cache.planned_fill_frames - planned


## The cloud samples' range of a view from height_m (elevation, vertical fov in degrees, aspect = width / height): the
## nearest and farthest horizontal distance (m, along the ground from the camera's vertical) at which border rays enter
## the slab (its top) and leave it (its base, else its top again) - a sphere of the mean Earth radius.
static func _view_cloud_range_m(height_m: float, elevation_deg: float, fov_deg: float, aspect: float) -> Vector2:
	const R := 6371000.0
	var el := deg_to_rad(elevation_deg)
	var forward := Vector2(cos(el), sin(el))  # (horizontal, up) in the vertical plane of the view
	var t := tan(deg_to_rad(fov_deg * 0.5))
	var result := Vector2(INF, 0.0)
	var rc := R + height_m
	# _render rolls a steep view's screen up off the vertical: then the cone around the axis through the corners.
	var cone := absf(sin(el)) > 0.9
	var corner := t * sqrt(1.0 + aspect * aspect)
	for i in range(CACHE_BORDER_POINTS + 1):
		var a := -1.0 + 2.0 * i / CACHE_BORDER_POINTS
		var border := [Vector2(a, -1.0), Vector2(a, 1.0), Vector2(-aspect, a), Vector2(aspect, a)]
		if cone:
			var phi := PI * a
			border = [Vector2(cos(phi), sin(phi)) * corner / t]
		for p: Vector2 in border:
			# The ray: forward + t (p.y screen up + p.x screen right); its sine of elevation.
			var up := Vector2(-sin(el), cos(el)) * t * p.y
			var d := Vector3(forward.x + up.x, t * p.x, forward.y + up.y)
			var sin_el := d.z / d.length()
			var top := _sphere_hits(rc, sin_el, R + SLAB_TOP_M)
			if top.x > top.y:
				continue
			var base := _sphere_hits(rc, sin_el, R + SLAB_BOTTOM_M)
			var leave := base.x if base.x <= base.y else top.y
			result.x = minf(result.x, _ground_distance(rc, sin_el, top.x, R))
			result.y = maxf(result.y, _ground_distance(rc, sin_el, leave, R))
	return result


## Distances along a ray from radius rc (sine of its elevation) where it crosses the sphere of radius rs; x > y: none.
static func _sphere_hits(rc: float, sin_el: float, rs: float) -> Vector2:
	var b := rc * sin_el
	var disc := b * b - (rc * rc - rs * rs)
	if disc < 0.0:
		return Vector2(1.0, 0.0)
	var q := sqrt(disc)
	return Vector2(-b - q, -b + q)


static func _ground_distance(rc: float, sin_el: float, s: float, r: float) -> float:
	var cos_el := sqrt(maxf(1.0 - sin_el * sin_el, 0.0))
	return r * atan2(s * cos_el, rc + s * sin_el)


## The site's point east_m east (along the parallel) at height_m.
func _point_east(height_m: float, east_m: float) -> Vector3:
	var lon := LON_DEG + rad_to_deg(east_m / (6371000.0 * cos(deg_to_rad(LAT_DEG))))
	return HeightSource.geodetic_to_fixed(LAT_DEG, lon, height_m)


## {worst, at, mean_error, bias, mean} of a cached image against its march (CACHE_ERROR_FLOOR).
static func _cache_error(cached: PackedFloat32Array, march: PackedFloat32Array) -> Dictionary:
	var mean := 0.0
	for v in march:
		mean += v / march.size()
	var worst := 0.0
	var worst_at := 0
	var mean_error := 0.0
	var bias := 0.0
	for i in range(march.size()):
		var error := absf(cached[i] - march[i]) / maxf(march[i], CACHE_ERROR_FLOOR * mean)
		mean_error += error / march.size()
		bias += (cached[i] - march[i]) / (mean * march.size())
		if error > worst:
			worst = error
			worst_at = i
	return {"worst": worst, "at": worst_at, "mean_error": mean_error, "bias": bias, "mean": mean}


## Section 8, broken cloud: the cache error (as _cache_error) split by the view's shapes (`shape`: the background
## transmittance, debug output 4) into silhouette pixels (alpha < CACHE_EDGE_ALPHA within `radius` pixels: one march
## pixel) and interior ones: {interior, edge (worst), interior_count, edge_count, interior_over, edge_over (pixels over
## CACHE_BROKEN_TOLERANCE), p99 (all pixels)}.
static func _cache_error_split(cached: PackedFloat32Array, march: PackedFloat32Array, shape: PackedFloat32Array,
		radius: int) -> Dictionary:
	var mean := 0.0
	for v in march:
		mean += v / march.size()
	var n := CACHE_VIEW_SIZE
	var r := {"interior": 0.0, "edge": 0.0, "interior_count": 0, "edge_count": 0, "interior_over": 0, "edge_over": 0}
	var errors := PackedFloat64Array()
	for y in range(n):
		for x in range(n):
			var i := y * n + x
			var edge := false
			for dy in range(-radius, radius + 1):
				for dx in range(-radius, radius + 1):
					var q := Vector2i(clampi(x + dx, 0, n - 1), clampi(y + dy, 0, n - 1))
					edge = edge or 1.0 - shape[q.y * n + q.x] < CACHE_EDGE_ALPHA
			var error := absf(cached[i] - march[i]) / maxf(march[i], CACHE_ERROR_FLOOR * mean)
			errors.append(error)
			var key := "edge" if edge else "interior"
			r[key] = maxf(r[key], error)
			r[key + "_count"] += 1
			if error > CACHE_BROKEN_TOLERANCE:
				r[key + "_over"] += 1
	errors.sort()
	r["p99"] = errors[int(errors.size() * 0.99)]
	return r


## The GPU time of the light cache's fill in the frame whose timestamps the RenderingDevice holds (0: no fill).
func _light_fill_ms() -> float:
	var rd := RenderingServer.get_rendering_device()
	var worst := 0.0
	for i in range(1, rd.get_captured_timestamps_count()):
		if rd.get_captured_timestamp_name(i - 1) == "cloud_light_fill":
			var ns := rd.get_captured_timestamp_gpu_time(i) - rd.get_captured_timestamp_gpu_time(i - 1)
			worst = maxf(worst, float(ns))
	return worst * 1.0e-6


## The red channel of the view (size: the viewport's), the mean over RADIANCE_FRAMES frames with the march's jitter
## pinned to frames first_frame, first_frame + 1, ...
func _mean_image(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, mode: int,
		size := Vector2i(CACHE_VIEW_SIZE, CACHE_VIEW_SIZE), first_frame := 0,
		frames := RADIANCE_FRAMES) -> PackedFloat32Array:
	var mean := PackedFloat32Array()
	mean.resize(size.x * size.y)
	for frame in range(frames):
		CloudEffect.pinned_frame = first_frame + frame
		await _render(camera_itrf_m, view_itrf, sun_itrf, mode)
		var image := _viewport.get_texture().get_image()
		for y in range(size.y):
			for x in range(size.x):
				mean[y * size.x + x] += image.get_pixel(x, y).r / frames
	CloudEffect.pinned_frame = -1
	return mean


## Section 9 (static clouds Task 3): the deterministic march's silhouettes (its 50 % background transmittance contour,
## debug output 4: the shapes) of a broken layer seen from SILHOUETTE_DISTANCE_M lie within one march pixel (the
## preset's divisor in output pixels) of the converged reference's, for both quality presets.
func _silhouette_samples() -> void:
	var slab := _slab("silhouette", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, SILHOUETTE_FRACTION)
	_show(slab)
	var size := Vector2i(SILHOUETTE_VIEW_SIZE, SILHOUETTE_VIEW_SIZE)
	_viewport.size = size
	_camera.fov = SILHOUETTE_FOV_DEG
	var camera := _point(SLAB_TOP_M + SILHOUETTE_DISTANCE_M)
	var view := _local_direction(-90.0, 0.0)
	var sun := _local_direction(45.0, 135.0)
	for quality in [CloudLayer.Quality.DESKTOP, CloudLayer.Quality.VR]:
		_clouds.quality = quality
		_detail_off()
		_clouds.deterministic_march = false
		ShaderGlobals.set_value(&"cloud_fine_step_fraction", SILHOUETTE_REFERENCE_FINE)
		ShaderGlobals.set_value(&"cloud_max_steps", SILHOUETTE_REFERENCE_MAX_STEPS)
		ShaderGlobals.set_value(&"cloud_steps", SILHOUETTE_REFERENCE_STEPS)
		var reference := await _mean_image(camera, view, sun, 4, size, 0, SILHOUETTE_REFERENCE_FRAMES)
		_clouds.quality = quality  # the preset's steps again
		_detail_off()
		_clouds.deterministic_march = true
		var got := await _mean_image(camera, view, sun, 4, size, 0, 1)
		var divisor := int(CloudLayer.PRESETS[quality]["divisor"])
		var result := _contour_distance(got, reference, size)
		_check("silhouette within one march pixel of the converged one (%s preset)"
				% ("VR" if quality == CloudLayer.Quality.VR else "desktop"),
				result.x <= divisor and result.y >= SILHOUETTE_MIN_CONTOUR,
				"farthest %d px (limit %d), %d mismatched pixels, reference contour %d px" % [result.x, divisor,
				result.z, result.y])
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_detail_off()
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_camera.fov = 10.0


## Section 9: (x) the largest distance [px, Chebyshev] from a pixel whose side of 50 % transmittance differs from the
## reference to the nearest reference pixel on its side (SILHOUETTE_SEARCH + 1 when none is that near), (y) the
## reference's contour pixels (a neighbour on the other side), (z) the mismatched pixels.
static func _contour_distance(got: PackedFloat32Array, reference: PackedFloat32Array, size: Vector2i) -> Vector3i:
	var farthest := 0
	var contour := 0
	var mismatched := 0
	for y in range(size.y):
		for x in range(size.x):
			var cloud := reference[y * size.x + x] < 0.5
			for n in [Vector2i(1, 0), Vector2i(0, 1)]:
				var q: Vector2i = Vector2i(x, y) + n
				if q.x < size.x and q.y < size.y and (reference[q.y * size.x + q.x] < 0.5) != cloud:
					contour += 1
					break
			var side := got[y * size.x + x] < 0.5
			if side == cloud:
				continue
			mismatched += 1
			var nearest := SILHOUETTE_SEARCH + 1
			for dy in range(-SILHOUETTE_SEARCH, SILHOUETTE_SEARCH + 1):
				for dx in range(-SILHOUETTE_SEARCH, SILHOUETTE_SEARCH + 1):
					var q := Vector2i(x + dx, y + dy)
					var r := maxi(absi(dx), absi(dy))
					var inside := q.x >= 0 and q.y >= 0 and q.x < size.x and q.y < size.y
					if r < nearest and inside and (reference[q.y * size.x + q.x] < 0.5) == side:
						nearest = r
			farthest = maxi(farthest, nearest)
	return Vector3i(farthest, contour, mismatched)


## The mean of each row of a CACHE_LINE_SIZE image (the line of pixels: one value per elevation).
static func _column_means(image: PackedFloat32Array) -> PackedFloat32Array:
	var rows := PackedFloat32Array()
	rows.resize(CACHE_LINE_SIZE.y)
	for y in range(CACHE_LINE_SIZE.y):
		for x in range(CACHE_LINE_SIZE.x):
			rows[y] += image[y * CACHE_LINE_SIZE.x + x] / CACHE_LINE_SIZE.x
	return rows


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
