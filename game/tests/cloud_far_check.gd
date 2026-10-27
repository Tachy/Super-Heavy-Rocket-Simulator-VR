# GPU check of the far-field clouds (M8 weather and clouds, Task 10; the near / far hand-over in the cloud pass, static
# clouds Task 8): the scaled-space Earth (earth_clouds.gdshader, cloud_far.gdshaderinc) beyond
# CelestialBody.SCALED_SPACE_DISTANCE_M against the cloud pass inside it, and the cloud pass's far tier against its
# march, through the game's own scene (main.tscn: World, terrain quadtree, CloudLayer); HDR, linear tone map, no glow,
# fixed exposure, UI hidden.
#   1. the far field's slab radiance (cloud_far_light_radiance with the Monte Carlo table, tests/cloud_far_probe
#      .gdshader) vs the core's slab_reflected_radiance_mc at 72 geometries (tau 0.5-30, Sun 0-80 deg, views 0-75 deg,
#      forward / side / back), 3 %;
#   2. weather off: the scaled-space Earth drawn with earth.gdshader, bit-identical to the pre-Task-10 shader (one
#      file: earth_surface.gdshaderinc without its EARTH_CLOUDS parts);
#   3. the scaled-space hand-over (the no-pop rule): decks of tau ~10 and ~2 (cf 1) seen from just inside and just
#      outside 1.5e7 m: with the volume on the grid mean (cloud debug output 6: the far field's physics without the
#      sub-grid shapes; the debug outputs march the whole segment) the 5 x 5 block luminance at seven Earth points (Sun
#      20-75 deg up) agrees within 5 %; the game's cloud pass - every cloud there lies beyond the near / far hand-over,
#      its far tier draws them with the far field's column model at these footprints - within 5 %, a broken layer (cf
#      0.4) as 21 x 21 block means within 5 %; points where the ground itself steps across the limit by more than the
#      tolerance (weather off: the terrain's sea glint at the sub-camera point, which the scaled-space Earth lacks) are
#      listed, not asserted; the volume pass draws only inside (cloud_enabled), the far field only outside
#      (earth_clouds.gdshader); the CPU light factors are (1, 0, 1) outside;
#   4. from the Moon's distance (3.844e8 m) with the ERA5 snapshot of 2024-04-08 18:00 UTC: five cells the snapshot
#      has cloudy are brighter with the weather on than off (ratio >= 1.3), five clear cells unchanged (2 %);
#   5. the near / far hand-over in the cloud pass (static clouds Task 8; CloudLayer.PRESETS handover_m), on the cloud
#      pass's own image (CloudEffect.output): from 420 km every pixel is the far tier's (the image with the preset's
#      band equals the far tier alone, globals cloud_handover_near_m / _far_m = 0); a broken layer (cf 0.4) and a deck
#      (tau 10) seen from 9 km, rendered by the march alone (the band beyond every scene), the far tier alone and the
#      game: the far tier's mean radiance at 30-50 km within 10 % of the march's (the M8 ruling), the game's image in
#      every 2 km distance bin (14-60 km, the ray's crossing of the slab's middle) the linear cross-fade of the two
#      within 3 % (the weight at the slab's top), and the band's own step between bins - the weight's largest step
#      times the two's difference over the band - within 3 %; a camera at 20 km moved by 100 m: interior cover flips
#      (alpha changing by > 0.5 where a 7 x 7 window was all clear or all cloud) within a static pair's + 0.1 % of the
#      covered pixels.
# Needs a GPU (no --headless): godot --path game --script res://tests/cloud_far_check.gd [-- --save=<dir>] (PNGs of
# the renders) [--only=scaled|handover] (sections 1-4 or 5 alone). Exit code 1 on failure.
extends SceneTree

const EPOCH_UTC := "2024-04-08T14:00:00"  ## no eclipse; the Sun over the Atlantic
const HANDOVER_M := CelestialBody.SCALED_SPACE_DISTANCE_M
const INSIDE_M := HANDOVER_M * 0.999
const OUTSIDE_M := HANDOVER_M * 1.001
const MOON_DISTANCE_M := 3.844e8
const SUB_CAMERA := Vector2(0.0, -45.0)  ## (lat, lon) deg under the hand-over camera
const MOON_SUB_CAMERA := Vector2(10.0, -40.0)
const ERA5_FILE := "weather/cache/era5_1089330.wthr"  ## under Paths.data_dir()
const SLAB_BOTTOM_M := 1000.0
const SLAB_TOP_M := 2000.0
## Liquid water of vertical optical depth tau over 1 km: beta = 3 LWC / (2 rho_w r_l) = 150 LWC [1/m].
const DECK_LIQUID_KG_M3 := 1.0e-2 / 150.0
const THIN_LIQUID_KG_M3 := 2.0e-3 / 150.0
const BROKEN_LIQUID_KG_M3 := 3.0e-3 / 150.0
const BROKEN_FRACTION := 0.4
const SETTLE_FRAMES := 8
## After the settle the cloud history and the pass's frame counter reset and exactly this many frames are drawn (like
## render_check --deterministic): the temporal mean of every sample is the same number of frames, not a timing-dependent one.
const DETERMINISTIC_FRAMES := 64
const MAX_FRAMES := 6000
const PROBE_TOLERANCE := 0.03
const PROBE_FLOOR := 1.0e-3  ## /sr: below it the absolute difference counts
const HANDOVER_TOLERANCE := 0.05
const CLOUDY_RATIO_MIN := 1.3
const CLEAR_TOLERANCE := 0.02
const EARTH_CLOUDS_SHADER := "res://shaders/earth_clouds.gdshader"
const EARTH_SURFACE_INCLUDE := "res://shaders/earth_surface.gdshaderinc"
const WGS84_A_M := 6378137.0
const WGS84_E2 := 6.69437999014e-3
## 5. The near / far hand-over in the cloud pass (static clouds Task 8).
const ORBIT_HEIGHT_M := 420000.0
const BAND_SITE := SUB_CAMERA  ## the Atlantic (sea level), the Sun ~60 deg up
const BAND_EYE_HEIGHT_M := 9000.0
const BAND_AZIMUTH_DEG := 0.0
const BAND_ELEVATION_DEG := -11.0  ## the view's centre meets the slab ~38 km away
const BAND_SLAB_MID_M := 0.5 * (SLAB_BOTTOM_M + SLAB_TOP_M)
const BAND_ALL_MARCH_M := 1.0e9  ## a band beyond every scene: the march alone
const BAND_COMPARE_M := Vector2(30000.0, 50000.0)  ## the brief's 30-50 km
const BAND_BINS_M := Vector2(14000.0, 60000.0)
const BIN_M := 2000.0
const BAND_PIXEL_STRIDE := 2  ## every second pixel in x and y (GDScript's speed; ~360 000 samples)
## The M8 ruling's near-far difference (decision 5 of the static clouds spec) and the band's smoothness (brief).
const NEAR_FAR_TOLERANCE := 0.10
const BIN_STEP_TOLERANCE := 0.03
const PAIR_EYE_HEIGHT_M := 20000.0  ## the 20 km view (t3_render.sh 20km)
const PAIR_AZIMUTH_DEG := 45.0
const PAIR_ELEVATION_DEG := -15.0
const PAIR_MOVE_M := 100.0
## A cover flip: alpha changes by more than FLIP_ALPHA_STEP where a's window of (2 FLIP_WINDOW + 1)^2 pixels is all
## below FLIP_UNIFORM_ALPHA.x or all above .y (the camera's 100 m shift the clouds by < FLIP_WINDOW pixels at >= 30 km).
const FLIP_ALPHA_STEP := 0.5
const FLIP_WINDOW := 3
const FLIP_UNIFORM_ALPHA := Vector2(0.1, 0.9)
## [E] Allowed interior flips of the moving pair beyond the static pair's, as a fraction of the covered pixels.
const FLIP_ALLOWANCE := 1.0e-3

var _main: Node
var _world: World
var _failures := 0
var _samples := 0
var _save_dir := ""
var _only := ""  ## --only=scaled | handover: sections 1-4 or 5 alone
var _files: Array[String] = []
var _probe_viewport := SubViewport.new()
var _probe_material := ShaderMaterial.new()


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--save="):
			_save_dir = arg.trim_prefix("--save=")
		elif arg.begins_with("--only="):
			_only = arg.trim_prefix("--only=")
	_run.call_deferred()


func _run() -> void:
	root.use_hdr_2d = true
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_world = _main.get_node("World")
	_world.weather.enabled = false
	for i in range(MAX_FRAMES):
		await process_frame
		if _world.atmosphere.ready and _world.clouds.noise_ready and _world.clouds.transfer_ready:
			break
	if not (_world.atmosphere.ready and _world.clouds.noise_ready and _world.clouds.transfer_ready):
		_fail_and_quit("atmosphere / cloud noise / transfer table not ready")
		return
	_hide_ui(_main)
	_linear_environment()
	_world.atmosphere.eye_adaptation_enabled = false
	_world.clock.warp = 0.0
	_world.clock.set_time(_world.core.utc_to_tdb_s(EPOCH_UTC))
	_world.set_focus(World.FOCUS_EARTH)
	_world.camera.fov = 60.0

	if _only == "" or _only == "scaled":
		# 1. The far field's slab radiance vs the core.
		await _probe_samples()
		# 2. Weather off: bit-identical to the shader without its cloud blocks (scaled space, outside the limit).
		await _identity_sample()
		# 3. The scaled-space hand-over.
		# The volume on the grid mean (cloud debug output 6: the same field, light, sky, air and composite without the
		# sub-grid shapes; the debug outputs march the whole segment) is what the far field reproduces: 5 %. The game's
		# cloud pass draws every cloud there with its far tier (static clouds Task 8: all lie beyond the near / far
		# hand-over), the far field's model with the pattern faded out at these footprints: 5 %.
		for deck in [["deck tau 10", DECK_LIQUID_KG_M3], ["deck tau 2", THIN_LIQUID_KG_M3]]:
			var snapshot := _slab(deck[0], deck[1], 1.0)
			_show(snapshot)
			ShaderGlobals.set_value(&"cloud_debug_output", 6)
			await _handover_sample(deck[0] + ", volume on the grid mean", 2, HANDOVER_TOLERANCE)
			ShaderGlobals.set_value(&"cloud_debug_output", 0)
			await _handover_sample(deck[0] + ", the cloud pass's far tier", 2, HANDOVER_TOLERANCE, snapshot)
		var broken := _slab("broken", BROKEN_LIQUID_KG_M3, BROKEN_FRACTION)
		_show(broken)
		await _handover_sample("broken cf 0.4, the cloud pass's far tier", 10, HANDOVER_TOLERANCE, broken)
		# 4. From the Moon.
		await _moon_samples()
	if _only == "" or _only == "handover":
		# 5. The near / far hand-over in the cloud pass.
		await _handover_band_samples()

	for f in _files:
		DirAccess.remove_absolute(f)
	print("cloud_far_check: %s (%d samples, %d failures)" % ["PASS" if _failures == 0 else "FAIL", _samples, _failures])
	quit(1 if _failures > 0 else 0)


## 1. tests/cloud_far_probe.gdshader evaluates cloud_far_light_radiance per pixel of a row of geometries.
func _probe_samples() -> void:
	var cases: Array = []
	for tau in [0.5, 2.0, 8.0, 30.0]:
		for mu0 in [1.0, 0.5, 0.17]:
			for view in [[1.0, 0.0], [0.7, 0.0], [0.7, PI], [0.26, 0.0], [0.26, PI], [0.5, 0.5 * PI]]:
				cases.append([tau, mu0, view[0], view[1]])
	var params := Image.create(cases.size(), 1, false, Image.FORMAT_RGBAF)
	for i in range(cases.size()):
		params.set_pixel(i, 0, Color(cases[i][0], cases[i][1], cases[i][2], cases[i][3]))
	_probe_viewport.size = Vector2i(cases.size(), 1)
	_probe_viewport.use_hdr_2d = true
	_probe_viewport.own_world_3d = true
	_probe_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(_probe_viewport)
	var camera := Camera3D.new()
	camera.current = true
	_probe_viewport.add_child(camera)
	_probe_material.shader = load("res://tests/cloud_far_probe.gdshader") as Shader
	_probe_material.set_shader_parameter("params", ImageTexture.create_from_image(params))
	var quad := MeshInstance3D.new()
	var mesh := QuadMesh.new()
	mesh.size = Vector2(2.0, 2.0)
	quad.mesh = mesh
	quad.material_override = _probe_material
	quad.position = Vector3(0.0, 0.0, -1.0)
	quad.extra_cull_margin = 16384.0
	_probe_viewport.add_child(quad)
	for i in range(4):
		await process_frame
	var image := _probe_viewport.get_texture().get_image()
	_probe_viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	var worst := 0.0
	var worst_case := ""
	var bad := 0
	for i in range(cases.size()):
		var c: Array = cases[i]
		var gpu := image.get_pixel(i, 0).r
		var cpu := _world.core.cloud_slab_radiance_mc(c[0], c[1], c[2], c[3])
		var error := absf(gpu - cpu) / maxf(cpu, PROBE_FLOOR)
		if error > PROBE_TOLERANCE:
			bad += 1
		if error > worst or is_nan(gpu):
			worst = error if not is_nan(gpu) else INF
			worst_case = "tau %.1f mu0 %.2f mu %.2f phi %.2f: GPU %.5f CPU %.5f" % [c[0], c[1], c[2], c[3], gpu, cpu]
	_check("far-field slab radiance vs slab_reflected_radiance_mc (%d geometries)" % cases.size(), bad == 0,
			"%d outside %.0f %%, worst %.2f %% (%s)" % [bad, PROBE_TOLERANCE * 100.0, worst * 100.0, worst_case])


## 2. Weather off, outside the limit: the world's Earth (earth.gdshader) against the pre-Task-10 shader as one file.
func _identity_sample() -> void:
	var material := _world.earth.material_override as ShaderMaterial
	await _place(SUB_CAMERA, OUTSIDE_M)
	var shader_path := _earth_shader_path()
	var drawn := _frame().get_data()
	var original := material.shader
	var reference := Shader.new()
	reference.code = _without_clouds(original.code)
	material.shader = reference
	await _place(SUB_CAMERA, OUTSIDE_M)
	var expected := _frame().get_data()
	material.shader = original
	var differing := 0
	for i in range(mini(drawn.size(), expected.size())):
		differing += 1 if drawn[i] != expected[i] else 0
	_check("weather off: the scaled-space Earth (%s) bit-identical to the shader without clouds" % shader_path.get_file(),
			shader_path != EARTH_CLOUDS_SHADER and differing == 0 and drawn.size() > 0
			and drawn.size() == expected.size(), "%d of %d bytes differ" % [differing, drawn.size()])


func _earth_shader_path() -> String:
	return (_world.earth.material_override as ShaderMaterial).shader.resource_path


## 3. The same Earth points just inside (terrain + cloud pass) and just outside (earth.gdshader + far field).
## The scaled-space hand-over at seven Earth points (header 3). snapshot (the one shown) given: points where the ground
## itself steps across the limit by more than the tolerance (rendered without weather: the terrain inside, the
## scaled-space Earth outside - the sea's sun glint is the terrain's alone, seen through a broken layer's gaps) cannot
## show the clouds' step; they are listed, not asserted.
func _handover_sample(label: String, radius: int, tolerance: float, snapshot := {}) -> void:
	var points: Array[Vector3] = []
	points.append(_itrf(SUB_CAMERA.x, SUB_CAMERA.y))
	for i in range(3):
		points.append(_destination(SUB_CAMERA, 25.0, 120.0 * i))
		points.append(_destination(SUB_CAMERA, 45.0, 60.0 + 120.0 * i))
	var ground := {}
	if not snapshot.is_empty():
		_world.clouds.show_now({}, {}, 0.0)
		for distance: float in [INSIDE_M, OUTSIDE_M]:
			await _place(SUB_CAMERA, distance)
			var image := _frame()
			var lums: Array[float] = []
			for p in points:
				lums.append(_block_luminance(image, _pixel_of(p), radius))
			ground[distance < HANDOVER_M] = lums
		_show(snapshot)
	var values := {}
	for distance: float in [INSIDE_M, OUTSIDE_M]:
		await _place(SUB_CAMERA, distance)
		var inside: bool = distance < HANDOVER_M
		var volume: bool = ShaderGlobals.value(&"cloud_enabled")
		var far: bool = _world.clouds.far_enabled and _earth_shader_path() == EARTH_CLOUDS_SHADER
		_check("%s, %s: the volume pass %s, the far field %s" % [label, "inside" if inside else "outside",
				"on" if inside else "off", "off" if inside else "on"], volume == inside and far == not inside,
				"cloud_enabled %s, CloudLayer.far_enabled %s, Earth shader %s" % [volume,
				_world.clouds.far_enabled, _earth_shader_path().get_file()])
		if not inside:
			var camera_fixed := _world.earth.fixed_to_gcrf.transposed() * _world.camera.position_gcrf_m
			var light := _world.atmosphere.cloud_light_at(camera_fixed, camera_fixed.normalized())
			_check("%s, outside: the CPU light factors (1, 0, 1)" % label, light == Vector3(1.0, 0.0, 1.0),
					"%s" % light)
		var image := _frame()
		_save(image, "t10check_%s_%s" % [label.replace(",", "").replace(" ", "_"), "inside" if inside else "outside"])
		var lums: Array[float] = []
		for p in points:
			lums.append(_block_luminance(image, _pixel_of(p), radius))
		values[inside] = lums
	var worst := 0.0
	var trace := ""
	var excluded := ""
	var asserted := 0
	for i in range(points.size()):
		var a: float = values[true][i]
		var b: float = values[false][i]
		if not snapshot.is_empty():
			var ground_step: float = absf(ground[false][i] - ground[true][i]) / maxf(ground[true][i], 1.0e-6)
			if ground_step > tolerance:
				excluded += " point %d (ground %.4f/%.4f, clouds %.4f/%.4f)" % [i, ground[true][i], ground[false][i], a,
						b]
				continue
		asserted += 1
		worst = maxf(worst, absf(b - a) / maxf(a, 1.0e-6))
		trace += " %.4f/%.4f" % [a, b]
	_check("%s: %d x %d block luminance inside / outside at %d points within %.0f %%" % [label, 2 * radius + 1,
			2 * radius + 1, asserted, tolerance * 100.0], worst <= tolerance and asserted >= points.size() - 1,
			"worst %.2f %%:%s%s" % [worst * 100.0, trace, "" if excluded == "" else
			"; the ground itself steps at" + excluded])


## 4. The Moon's distance: cloudy and clear cells of the ERA5 snapshot, weather off then on.
func _moon_samples() -> void:
	var file := Paths.data_dir().path_join(ERA5_FILE)
	if not FileAccess.file_exists(file):
		_check("ERA5 snapshot for the Moon view", false, "%s missing" % file)
		return
	var snapshot: Dictionary = _world.core.weather_load_snapshot(file)
	_world.core.weather_set_active(file, file, 0.0)
	var cells := _pick_cells()
	_check("cells picked (5 cloudy, 5 clear)", cells[0].size() == 5 and cells[1].size() == 5,
			"%d cloudy, %d clear" % [cells[0].size(), cells[1].size()])
	_world.camera.fov = 2.6
	_world.clouds.show_now({}, {}, 0.0)
	await _place(MOON_SUB_CAMERA, MOON_DISTANCE_M)
	var off := _frame()
	_save(off, "t10check_moon_off")
	_world.clouds.show_now(snapshot, snapshot, 0.0)
	await _place(MOON_SUB_CAMERA, MOON_DISTANCE_M)
	var on := _frame()
	_save(on, "t10check_moon_on")
	for k in range(2):
		for p: Vector3 in cells[k]:
			var px := _pixel_of(p)
			var ratio := _block_luminance(on, px, 1) / maxf(_block_luminance(off, px, 1), 1.0e-6)
			var ll := _lat_lon(p)
			if k == 0:
				_check("from the Moon: cloudy cell (%.1f, %.1f) brighter" % [ll.x, ll.y], ratio >= CLOUDY_RATIO_MIN,
						"on / off %.3f" % ratio)
			else:
				_check("from the Moon: clear cell (%.1f, %.1f) unchanged" % [ll.x, ll.y],
						absf(ratio - 1.0) <= CLEAR_TOLERANCE, "on / off %.4f" % ratio)
	_world.camera.fov = 60.0


## 5. The near / far hand-over in the cloud pass (static clouds Task 8; header).
func _handover_band_samples() -> void:
	var preset: Vector2 = CloudLayer.PRESETS[_world.clouds.quality]["handover_m"]
	# From low orbit every pixel is the far tier's: the image with the preset's band equals the far tier alone.
	_show(_slab("broken", BROKEN_LIQUID_KG_M3, BROKEN_FRACTION))
	await _place(SUB_CAMERA, WGS84_A_M + ORBIT_HEIGHT_M)  # the camera's up blends after the move: settled first
	_set_handover(Vector2.ZERO)
	await _place(SUB_CAMERA, WGS84_A_M + ORBIT_HEIGHT_M)
	var far_only := _cloud_image()
	_set_handover(preset)
	await _place(SUB_CAMERA, WGS84_A_M + ORBIT_HEIGHT_M)
	var game := _cloud_image()
	_save(far_only, "t8check_orbit_far")
	_save(game, "t8check_orbit_game")
	var covered := _covered_pixels(game)
	var difference := _image_difference(game, far_only)
	_check("from %.0f km every pixel is the far tier's (the image equals the far tier alone; clouds drawn)" % (
			ORBIT_HEIGHT_M / 1000.0), difference.x == 0 and covered > 0,
			"%d covered pixels, %d differ (largest %.5f)" % [covered, int(difference.x), difference.y])
	# Across the band: the broken layer and the deck seen from 9 km, the march alone / the far tier alone / the game.
	for layer in [["broken cf 0.4", BROKEN_LIQUID_KG_M3, BROKEN_FRACTION], ["deck tau 10", DECK_LIQUID_KG_M3, 1.0]]:
		_show(_slab(layer[0], layer[1], layer[2]))
		var images := []
		for band: Vector2 in [Vector2(BAND_ALL_MARCH_M, BAND_ALL_MARCH_M), Vector2.ZERO, preset]:
			_set_handover(band)
			if not await _ground_view(BAND_SITE, BAND_EYE_HEIGHT_M, BAND_AZIMUTH_DEG, BAND_ELEVATION_DEG):
				_check("%s: the band view settled" % layer[0], false, "")
				return
			images.append(_cloud_image())
		_save(images[0], "t8check_%s_near" % layer[0].replace(" ", "_"))
		_save(images[1], "t8check_%s_far" % layer[0].replace(" ", "_"))
		_save(images[2], "t8check_%s_game" % layer[0].replace(" ", "_"))
		_band_checks(layer[0], images[0], images[1], images[2], preset)
	# A moving camera: no cover flips beyond a static pair's.
	_show(_slab("broken", BROKEN_LIQUID_KG_M3, BROKEN_FRACTION))
	_set_handover(preset)
	var shots := []
	for moved_m: float in [0.0, 0.0, PAIR_MOVE_M]:
		var site := BAND_SITE + Vector2(rad_to_deg(moved_m / WGS84_A_M), 0.0)
		if not await _ground_view(site, PAIR_EYE_HEIGHT_M, PAIR_AZIMUTH_DEG, PAIR_ELEVATION_DEG):
			_check("the moving pair settled", false, "")
			return
		shots.append(_cloud_image())
	var static_flips := _interior_flips(shots[0], shots[1])
	var moving_flips := _interior_flips(shots[0], shots[2])
	_check("moving camera (%.0f m at %.0f km): interior cover flips within the static pair's + %.2f %% of the cloud" % [
			PAIR_MOVE_M, PAIR_EYE_HEIGHT_M / 1000.0, FLIP_ALLOWANCE * 100.0],
			moving_flips <= static_flips + FLIP_ALLOWANCE * _covered_pixels(shots[0]),
			"moving %d, static %d, covered %d" % [moving_flips, static_flips, _covered_pixels(shots[0])])
	_set_handover(preset)


## The band's checks (header 5) on the march alone (near), the far tier alone (far) and the game's image, binned by the
## distance at which each pixel's ray crosses the slab's middle (BAND_SLAB_MID_M); the hand-over weight of a pixel at
## the distance of the slab's top (where its cloud is seen from above).
func _band_checks(label: String, near: Image, far: Image, game: Image, band: Vector2) -> void:
	var bins := int((BAND_BINS_M.y - BAND_BINS_M.x) / BIN_M)
	var sums := [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array(),
			PackedFloat64Array()]  # near, far, game, the linear cross-fade of near and far, the weight
	var counts := PackedFloat64Array()
	for s in sums:
		s.resize(bins)
	counts.resize(bins)
	var compare := Vector3.ZERO  # near, far, pixels over BAND_COMPARE_M
	var in_band := Vector2.ZERO  # near, far over the band
	var w := near.get_width()
	var h := near.get_height()
	var site_up := Frames.to_render(_world.earth.fixed_to_gcrf * _itrf(BAND_SITE.x, BAND_SITE.y).normalized())
	var r_ground := _itrf(BAND_SITE.x, BAND_SITE.y).length()
	var r_c := r_ground + BAND_EYE_HEIGHT_M  # the eye above the ellipsoid (the site's surface is the sea: ~0 m)
	var r_mid := r_ground + BAND_SLAB_MID_M
	var r_top := r_ground + SLAB_TOP_M
	for y in range(0, h, BAND_PIXEL_STRIDE):
		for x in range(0, w, BAND_PIXEL_STRIDE):
			var ray := _world.camera.project_ray_normal(Vector2(x + 0.5, y + 0.5))
			var b := r_c * ray.dot(site_up)
			var disc_mid := b * b - (r_c * r_c - r_mid * r_mid)
			var disc_top := b * b - (r_c * r_c - r_top * r_top)
			if b >= 0.0 or disc_mid < 0.0 or disc_top < 0.0:
				continue
			var d := -b - sqrt(disc_mid)
			var weight := _handover_weight(-b - sqrt(disc_top), band)
			var lums := [_luminance(near.get_pixel(x, y)), _luminance(far.get_pixel(x, y)),
					_luminance(game.get_pixel(x, y))]
			if d >= BAND_COMPARE_M.x and d < BAND_COMPARE_M.y:
				compare += Vector3(lums[0], lums[1], 1.0)
			if d >= band.x and d < band.y:
				in_band += Vector2(lums[0], lums[1])
			var i := int(floor((d - BAND_BINS_M.x) / BIN_M))
			if i < 0 or i >= bins:
				continue
			for k in range(3):
				sums[k][i] += lums[k]
			sums[3][i] += (1.0 - weight) * lums[0] + weight * lums[1]
			sums[4][i] += weight
			counts[i] += 1.0
	var ratio := compare.y / maxf(compare.x, 1.0e-12)
	_check("%s at %.0f-%.0f km: far tier vs march mean radiance within %.0f %%" % [label, BAND_COMPARE_M.x / 1000.0,
			BAND_COMPARE_M.y / 1000.0, NEAR_FAR_TOLERANCE * 100.0], absf(ratio - 1.0) <= NEAR_FAR_TOLERANCE,
			"far / near %.4f over %d pixels" % [ratio, int(compare.z)])
	# The game's image is the linear cross-fade of the two in every bin (no dip, no ring from the composite) ...
	var worst := 0.0
	var worst_at := 0.0
	var largest_weight_step := 0.0
	var table := ""
	for i in range(bins):
		if counts[i] <= 0.0:
			continue
		var m := Vector4(sums[0][i], sums[1][i], sums[2][i], sums[3][i]) / counts[i]
		table += " %.0f:%.4f/%.4f/%.4f/%.4f" % [(BAND_BINS_M.x + (i + 0.5) * BIN_M) / 1000.0, m.x, m.y, m.z, m.w]
		var residual := absf(m.z / maxf(m.w, 1.0e-12) - 1.0)
		if residual > worst:
			worst = residual
			worst_at = BAND_BINS_M.x + (i + 0.5) * BIN_M
		if i + 1 < bins and counts[i + 1] > 0.0:
			largest_weight_step = maxf(largest_weight_step,
					absf(sums[4][i + 1] / counts[i + 1] - sums[4][i] / counts[i]))
	_check("%s: the game's image is the cross-fade of the march and the far tier within %.0f %% per %.0f km bin" % [
			label, BIN_STEP_TOLERANCE * 100.0, BIN_M / 1000.0], worst <= BIN_STEP_TOLERANCE,
			"worst %.2f %% at %.0f km; bins km:near/far/game/cross-fade%s" % [worst * 100.0, worst_at / 1000.0, table])
	# ... whose own step between neighbouring bins is the weight's step times the two models' difference over the band.
	var band_ratio := in_band.y / maxf(in_band.x, 1.0e-12)
	var step := largest_weight_step * absf(band_ratio - 1.0)
	_check("%s: the hand-over's step between %.0f km bins (band %.0f-%.0f km) within %.0f %%" % [label,
			BIN_M / 1000.0, band.x / 1000.0, band.y / 1000.0, BIN_STEP_TOLERANCE * 100.0], step <= BIN_STEP_TOLERANCE,
			"%.2f %% (weight step %.3f x far / near %.4f over the band)" % [step * 100.0, largest_weight_step,
			band_ratio])


## The far tier's weight at distance d (cloud_pass.gdshaderinc cloud_handover_weight).
static func _handover_weight(d: float, band: Vector2) -> float:
	if band.y <= band.x:
		return 1.0 if d >= band.y else 0.0
	return smoothstep(band.x, band.y, d)


## Pixels whose alpha flips by more than FLIP_ALPHA_STEP between a and b although a's window of FLIP_WINDOW pixels
## around them is all clear or all cloud in a (no edge within the motion's shift: aliasing, not motion).
static func _interior_flips(a: Image, b: Image) -> int:
	var flips := 0
	var w := a.get_width()
	var h := a.get_height()
	for y in range(FLIP_WINDOW, h - FLIP_WINDOW):
		for x in range(FLIP_WINDOW, w - FLIP_WINDOW):
			if absf(a.get_pixel(x, y).a - b.get_pixel(x, y).a) <= FLIP_ALPHA_STEP:
				continue
			var clear := true
			var cloud := true
			for v in range(y - FLIP_WINDOW, y + FLIP_WINDOW + 1):
				for u in range(x - FLIP_WINDOW, x + FLIP_WINDOW + 1):
					var alpha := a.get_pixel(u, v).a
					clear = clear and alpha < FLIP_UNIFORM_ALPHA.x
					cloud = cloud and alpha > FLIP_UNIFORM_ALPHA.y
			if clear or cloud:
				flips += 1
	return flips


## (x) the pixels of a and b that differ in any channel, (y) the largest difference.
static func _image_difference(a: Image, b: Image) -> Vector2:
	var result := Vector2.ZERO
	for y in range(a.get_height()):
		for x in range(a.get_width()):
			var ca := a.get_pixel(x, y)
			var cb := b.get_pixel(x, y)
			var d := maxf(maxf(absf(ca.r - cb.r), absf(ca.g - cb.g)), maxf(absf(ca.b - cb.b), absf(ca.a - cb.a)))
			if d > 0.0:
				result.x += 1.0
				result.y = maxf(result.y, d)
	return result


static func _block_alpha(image: Image, centre: Vector2, radius: int) -> float:
	var sum := 0.0
	var n := 0
	for y in range(int(round(centre.y)) - radius, int(round(centre.y)) + radius + 1):
		for x in range(int(round(centre.x)) - radius, int(round(centre.x)) + radius + 1):
			if x >= 0 and y >= 0 and x < image.get_width() and y < image.get_height():
				sum += image.get_pixel(x, y).a
				n += 1
	return sum / maxf(n, 1)


static func _covered_pixels(image: Image) -> int:
	var n := 0
	for y in range(image.get_height()):
		for x in range(image.get_width()):
			n += 1 if image.get_pixel(x, y).a > FLIP_UNIFORM_ALPHA.x else 0
	return n


static func _luminance(c: Color) -> float:
	return 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b


func _set_handover(band: Vector2) -> void:
	ShaderGlobals.set_value(&"cloud_handover_near_m", band.x)
	ShaderGlobals.set_value(&"cloud_handover_far_m", band.y)


## The cloud pass's own image (premultiplied colour + alpha, the view's resolution).
func _cloud_image() -> Image:
	return _world.clouds.effect.output.get_layer_data(0)


## A ground view at site (lat, lon) with the eye eye_height_m above the core's surface; waits until the terrain, the
## clouds and the light cache have settled (false after MAX_FRAMES).
func _ground_view(site: Vector2, eye_height_m: float, azimuth_deg: float, elevation_deg: float) -> bool:
	var samples: Dictionary = _world.core.surface_samples("earth", PackedFloat64Array([site.x]),
			PackedFloat64Array([site.y]))
	_world.set_ground_site({"id": "far_check", "name": "far_check", "body": "earth", "latitude_deg": site.x,
			"longitude_deg": site.y, "height_m": samples["height_m"][0] if bool(samples["present"]) else 0.0,
			"eye_height_m": eye_height_m})
	_world.camera.azimuth = deg_to_rad(azimuth_deg)
	_world.camera.elevation = deg_to_rad(elevation_deg)
	var settled := 0
	for i in range(MAX_FRAMES):
		await process_frame
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		var clouds: Dictionary = _world.clouds.state()
		var ready: bool = loading == 0 and not _world.atmosphere.loading and not clouds["uploading"] \
				and _world.clouds.light_cache.settled()
		settled = settled + 1 if ready else 0
		if settled >= SETTLE_FRAMES and i >= 2 * SETTLE_FRAMES:
			return true
	return false


## Five cloudy cells (vertical transmittance < 0.05 there and < 0.2 at +-0.5 deg) and five clear ones (no cloud within
## +-1 deg), Sun > 25 deg up, within 50 deg of the sub-camera point, at least 6 deg apart.
func _pick_cells() -> Array:
	var cloudy: Array[Vector3] = []
	var clear: Array[Vector3] = []
	var sun_itrf := (_world.earth.fixed_to_gcrf.transposed() * _world.sun_apparent_from_earth_m).normalized()
	var centre := _itrf(MOON_SUB_CAMERA.x, MOON_SUB_CAMERA.y).normalized()
	var lat := -50.0
	while lat <= 50.0:
		var lon := -100.0
		while lon <= 20.0:
			var p := _itrf(lat, lon)
			var up := p.normalized()
			if up.dot(centre) > cos(deg_to_rad(50.0)) and up.dot(sun_itrf) > sin(deg_to_rad(25.0)):
				var t := _vertical_transmittance(lat, lon)
				if t < 0.05 and cloudy.size() < 5 and _far_from(cloudy, p):
					var solid := true
					for d in [Vector2(0.5, 0.0), Vector2(-0.5, 0.0), Vector2(0.0, 0.5), Vector2(0.0, -0.5)]:
						solid = solid and _vertical_transmittance(lat + d.x, lon + d.y) < 0.2
					if solid:
						cloudy.append(p)
				elif t >= 0.9999 and clear.size() < 5 and _far_from(clear, p):
					var empty := true
					for d in [Vector2(1.0, 0.0), Vector2(-1.0, 0.0), Vector2(0.0, 1.0), Vector2(0.0, -1.0),
							Vector2(1.0, 1.0), Vector2(-1.0, -1.0), Vector2(1.0, -1.0), Vector2(-1.0, 1.0)]:
						empty = empty and _vertical_transmittance(lat + d.x, lon + d.y) >= 0.9999
					if empty:
						clear.append(p)
			lon += 1.0
		lat += 1.0
	return [cloudy, clear]


func _vertical_transmittance(lat: float, lon: float) -> float:
	var p := _itrf(lat, lon)
	return float(_world.core.weather_cloud_light(p, p.normalized())["sun_transmittance"])


static func _far_from(points: Array[Vector3], p: Vector3) -> bool:
	for q in points:
		if q.normalized().dot(p.normalized()) > cos(deg_to_rad(6.0)):
			return false
	return true


## The orbit camera over (lat, lon) at distance_m from the Earth's centre; waits until the terrain and the clouds have
## settled.
func _place(sub_camera: Vector2, distance_m: float) -> void:
	var up_gcrf := _world.earth.fixed_to_gcrf * _itrf(sub_camera.x, sub_camera.y).normalized()
	var u := Frames.to_render(up_gcrf).normalized()
	_world.camera.pitch = -asin(u.y)
	_world.camera.yaw = atan2(u.x, u.z)
	_world.camera.distance_m = distance_m
	_world.clouds.effect.reset_history()
	var idle := 0
	for i in range(MAX_FRAMES):
		await process_frame
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		var uploading: bool = _world.clouds.state()["uploading"]
		idle = idle + 1 if loading == 0 and not uploading else 0
		if idle >= SETTLE_FRAMES and i >= 2 * SETTLE_FRAMES:
			break
	var effect: CloudEffect = _world.clouds.effect
	effect.reset_history()
	RenderingServer.call_on_render_thread(func() -> void: effect._frame = 0)
	for i in range(DETERMINISTIC_FRAMES):
		await process_frame


func _frame() -> Image:
	return root.get_texture().get_image()


func _save(image: Image, name: String) -> void:
	if _save_dir == "":
		return
	var copy := image.duplicate() as Image
	copy.convert(Image.FORMAT_RGBA8)
	copy.save_png(_save_dir.path_join(name + ".png"))


func _pixel_of(itrf: Vector3) -> Vector2:
	var p := _world.earth.fixed_to_gcrf * itrf - _world.camera.position_gcrf_m
	var dir := Frames.to_render(p).normalized()
	return _world.camera.unproject_position(_world.camera.global_position + dir * 1000.0)


static func _block_luminance(image: Image, centre: Vector2, radius: int) -> float:
	var sum := 0.0
	var n := 0
	var x0 := int(round(centre.x))
	var y0 := int(round(centre.y))
	for y in range(y0 - radius, y0 + radius + 1):
		for x in range(x0 - radius, x0 + radius + 1):
			if x >= 0 and y >= 0 and x < image.get_width() and y < image.get_height():
				var c := image.get_pixel(x, y)
				sum += 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b
				n += 1
	return sum / maxf(n, 1)


## Earth-fixed (ITRF) point at geodetic (lat, lon) deg on the ellipsoid.
static func _itrf(lat_deg: float, lon_deg: float) -> Vector3:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	var n := WGS84_A_M / sqrt(1.0 - WGS84_E2 * sin(lat) * sin(lat))
	return Vector3(n * cos(lat) * cos(lon), n * cos(lat) * sin(lon), n * (1.0 - WGS84_E2) * sin(lat))


static func _lat_lon(p: Vector3) -> Vector2:
	return Vector2(rad_to_deg(atan2(p.z, Vector2(p.x, p.y).length() * (1.0 - WGS84_E2))), rad_to_deg(atan2(p.y, p.x)))


## The point at great-circle angle angle_deg from (lat, lon) toward the azimuth (from north, east positive).
static func _destination(from: Vector2, angle_deg: float, azimuth_deg: float) -> Vector3:
	var lat := deg_to_rad(from.x)
	var delta := deg_to_rad(angle_deg)
	var azimuth := deg_to_rad(azimuth_deg)
	var lat2 := asin(sin(lat) * cos(delta) + cos(lat) * sin(delta) * cos(azimuth))
	var lon2 := deg_to_rad(from.y) + atan2(sin(azimuth) * sin(delta) * cos(lat), cos(delta) - sin(lat) * sin(lat2))
	return _itrf(rad_to_deg(lat2), rad_to_deg(lon2))


func _slab(label: String, liquid: float, fraction: float) -> Dictionary:
	var file := OS.get_temp_dir().path_join("starsim_cloud_far_check_%s_%d.wthr" % [label.replace(" ", "_"),
			OS.get_process_id()])
	_files.append(file)
	if not _world.core.weather_write_slab_snapshot(file, SLAB_BOTTOM_M, SLAB_TOP_M, liquid, fraction):
		_fail_and_quit("slab snapshot not written")
	var loaded: Dictionary = _world.core.weather_load_snapshot(file)
	loaded["file"] = file
	return loaded


func _show(snapshot: Dictionary) -> void:
	_world.clouds.show_now(snapshot, snapshot, 0.0)
	_world.core.weather_set_active(snapshot["file"], snapshot["file"], 0.0)


## Linear tone map, exposure 1, no glow: pixel values are the scene values.
func _linear_environment() -> void:
	for child in _world.get_children():
		if child is WorldEnvironment:
			var environment: Environment = (child as WorldEnvironment).environment
			environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
			environment.tonemap_exposure = 1.0
			environment.glow_enabled = false


## Hides every 2D layer (the HUD, labels): the frame holds only the 3D scene.
func _hide_ui(node: Node) -> void:
	for child in node.get_children():
		if child is CanvasLayer:
			(child as CanvasLayer).visible = false
		elif child is Control:
			(child as Control).visible = false
		else:
			_hide_ui(child)


## The Earth shader as one file without clouds (the pre-Task-10 earth.gdshader): the include of
## earth_surface.gdshaderinc replaced by its text with the #ifdef EARTH_CLOUDS ... #endif parts removed.
static func _without_clouds(code: String) -> String:
	var surface := FileAccess.get_file_as_string(EARTH_SURFACE_INCLUDE)
	var out := PackedStringArray()
	var inside := false
	for line in surface.split("\n"):
		if line.strip_edges() == "#ifdef EARTH_CLOUDS":
			inside = true
		elif inside and line.strip_edges() == "#endif":
			inside = false
		elif not inside:
			out.append(line)
	return code.replace('#include "%s"' % EARTH_SURFACE_INCLUDE, "\n".join(out))


func _check(label: String, ok: bool, detail: String) -> void:
	_samples += 1
	if not ok:
		_failures += 1
	print("  %s  %s: %s" % ["ok  " if ok else "FAIL", label, detail])


func _fail_and_quit(message: String) -> void:
	print("cloud_far_check: FAIL - " + message)
	quit(1)
