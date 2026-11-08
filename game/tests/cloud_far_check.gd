# GPU check of the far-field clouds (M8 weather and clouds, Task 10): the scaled-space Earth (earth_clouds.gdshader,
# cloud_far.gdshaderinc) beyond CelestialBody.SCALED_SPACE_DISTANCE_M against the cloud pass inside it, through the
# game's own scene (main.tscn: World, terrain quadtree, CloudLayer); HDR, linear tone map, no glow,
# fixed exposure, UI hidden.
#   1. the far field's slab radiance (cloud_far_light_radiance with the Monte Carlo table, tests/cloud_far_probe
#      .gdshader) vs the core's slab_reflected_radiance_mc at 72 geometries (tau 0.5-30, Sun 0-80 deg, views 0-75 deg,
#      forward / side / back), 3 %; 1b. the air above a bright top (cloud_far_bright_top_transmittance) against its
#      two-stream form, its limits and the cloud's blue / red increment; 1c. the cover pattern (cloud_far_meso_cover)
#      keeps the cell's cover as its mean over 4096 random points (look fixes T7b, T7 review I2);
#   2. weather off: the scaled-space Earth drawn with earth.gdshader, bit-identical to the pre-Task-10 shader (one
#      file: earth_surface.gdshaderinc without its EARTH_CLOUDS parts);
#   3. the scaled-space hand-over (the no-pop rule): decks of tau ~10 and ~2 (cf 1) seen from just inside and just
#      outside 1.5e7 m: the game's cloud pass (the baked volume: every cloud there in its flat levels) against the far
#      field, the 5 x 5 block luminance at seven Earth points (Sun 20-75 deg up) within 5 %, a broken layer (cf 0.4) as
#      21 x 21 block means within 5 %; points where the ground itself steps across the limit by more than the
#      tolerance are listed, not asserted - and, weather off, the cloudless ground itself is asserted within the same
#      tolerance at all seven points (look fixes T3: the scaled-space sea has the terrain's sea model, glint included);
#      the volume pass draws only inside (cloud_enabled), the far field only outside (earth_clouds.gdshader); the
#      CPU light factors are (1, 0, 1) outside; the ground is measured in the runs that get a snapshot, with the
#      weather cleared first; 3b. the same with the shipped analog snapshot of the month (1 deg, 0.25 deg fine
#      column: the far field and the flat levels' even cells read the fine column; look fixes T7b);
#   4. from the Moon's distance (3.844e8 m) with an ERA5 snapshot (cached 2024-04-08 18:00 UTC, else the
#      service fetches the epoch's pair, 14:00 UTC, see _era5_file): five cells the snapshot
#      has cloudy are brighter with the weather on than off (ratio >= 1.3), five clear cells unchanged (2 %);
#   5. the flat levels across a re-bake (look fixes T8): the broken layer (cf 0.4) straight down from 400 m below and
#      above the first shape-tile edge over 63 km (the wrapped shape origin steps by a whole tile there), each with a
#      new flat geometry (a detour to 200 km first): 25 ground points' 21 x 21 block luminance the same within 1.5 % (the
#      flat bake's turned / tile-breaking readings followed the camera's tile: the pattern jumped at the geometry's
#      re-bakes on a climb, 27 % here);
#   5b. the climb through 40-60 km (look fixes T7c / fix 2, user 2026-11-08: no visible change there from any view
#      angle; since fix 2 the volume is drawn at every height, the flat levels only beyond it): the broken layer at 0 /
#      30 / 45 / 60 deg off nadir, at a low Sun, the analog over the Gulf and over the Atlantic's cumulus; 25 ground
#      points' block luminance about the aim point over the climb 39.5 -> 60.5 km: straight down each within
#      TAKEOVER_TOLERANCE and their mean within TAKEOVER_MEAN_TOLERANCE (before T7c the flat levels placed the clouds
#      by their own 2-D pattern: regions moved by ~12 codes over 45-65 km), off nadir their mean within
#      TAKEOVER_GLOBAL_PER_KM per km;
# Needs a GPU (no --headless): godot --path game --script res://tests/cloud_far_check.gd [-- --save=<dir>] (PNGs of
# the renders) [--no_3d_light] (CloudLayer.far_3d_light off) [--only_rebake] (sections 5 and 5b alone) [--only_takeover] (5b alone) [--only_probe] (section 1 alone). Exit code 1 on failure.
extends SceneTree

const EPOCH_UTC := "2024-04-08T14:00:00"  ## no eclipse; the Sun over the Atlantic
const HANDOVER_M := CelestialBody.SCALED_SPACE_DISTANCE_M
const INSIDE_M := HANDOVER_M * 0.999
const OUTSIDE_M := HANDOVER_M * 1.001
const MOON_DISTANCE_M := 3.844e8
const SUB_CAMERA := Vector2(0.0, -45.0)  ## (lat, lon) deg under the hand-over camera
const MOON_SUB_CAMERA := Vector2(10.0, -40.0)
const ERA5_FILE := "weather/cache/era5_1089330.wthr"  ## under Paths.data_dir(); fetched when missing (_era5_file)
const ERA5_WAIT_S := 600  ## [E] the download and build of one ERA5 pair, generous
const SLAB_BOTTOM_M := 1000.0
const SLAB_TOP_M := 2000.0
## Liquid water of vertical optical depth tau over 1 km: beta = 3 LWC / (2 rho_w r_l) = 150 LWC [1/m].
const DECK_LIQUID_KG_M3 := 1.0e-2 / 150.0
const THIN_LIQUID_KG_M3 := 2.0e-3 / 150.0
const BROKEN_LIQUID_KG_M3 := 3.0e-3 / 150.0
const BROKEN_FRACTION := 0.4
const SETTLE_FRAMES := 8
const MAX_FRAMES := 6000
const PROBE_TOLERANCE := 0.03
const PROBE_FLOOR := 1.0e-3  ## /sr: below it the absolute difference counts
const PROBE_ROW := 1024  ## the probe image's width (cases per row)
## Section 1b (T7b): the GPU bright-top factor against its two-stream form (the probe's half-float target rounds by
## up to 4.9e-4 relative; 2026-11-08 worst 0.09 %);
## the cloud's blue / red increment band [E] about review I2's +8-10 % estimate (the form gives +10 / +11 % at A 0.6 /
## 0.75); the tables' ground albedo (cloud_far.gdshaderinc CLOUD_FAR_TABLE_GROUND_ALBEDO).
const BRIGHT_TOP_TOLERANCE := 1.5e-3
const BRIGHT_TOP_BR_RANGE := Vector2(1.07, 1.13)
const TABLE_GROUND_ALBEDO := 0.1
## Section 1c (T7b): random points per cover and footprint, the mean's tolerance [E] (sampling SD <= 0.35 / sqrt(4096)
## = 0.006 plus the field's correlation; T7c: the exact rank - C 0.1 -> 0.100 / 0.100 at 8 / 32 km, C 0.9 -> 0.899,
## worst 0.005; T7b's median / sigma normalisation gave 0.079 / 0.921) and the least spread of a working pattern at
## C 0.5 (measured 0.35).
const MESO_POINTS := 4096
const MESO_MEAN_TOLERANCE := 0.015
const MESO_SPREAD_MIN := 0.15
## Section 1d (T7b): points on the sphere; the mean's tolerance [E] (sampling 0.75 / sqrt(16384) = 0.6 % plus the
## noise field's correlation over a few km) and the fractional SD's (the Wilson-Hilferty quantile and the clamp).
const SPREAD_POINTS := 16384
const SPREAD_MEAN_TOLERANCE := 0.03
const SPREAD_FSD_TOLERANCE := 0.06
const SPREAD_FSD := 1.0  ## the cumulus spread's (cloud_far.gdshaderinc CLOUD_SPREAD_NU = 1 / FSD^2; look fixes T7c)
## Section 1e (T7c): the blocks probed, columns per case and the tolerances [E]: water clouds (cumulus, stratiform:
## sampling sd <= 0.5 / sqrt(2048) = 0.011 per case plus the fit's residual; 2026-11-08 worst 0.013) and cirrus (its
## 15.6 m vertical texels put a 1-3 km block at fractions of 0.002-0.03 per height, where the threshold table's 8-bit
## fraction codes and their linear interpolation are coarse: 2026-11-08 worst 0.087, raw shapes up to 0.6 over).
const PROJECTION_TYPES := [0, 1, 2]
const PROJECTION_DEPTHS := [250.0, 500.0, 1000.0, 1500.0, 2000.0, 3000.0]
const PROJECTION_FRACTIONS := [0.02, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8]
const PROJECTION_POINTS := 2048
const PROJECTION_TOLERANCE := Vector2(0.03, 0.1)  ## (water clouds, cirrus)
## cirrus below this fraction is listed, not asserted (its per-height fraction under one code: 1 - 3 km at f 0.02 keep
## 0.05-0.16)
const PROJECTION_ICE_FRACTION_MIN := 0.1
const HANDOVER_TOLERANCE := 0.05
const CLOUDY_RATIO_MIN := 1.3
## Section 5 (look fixes T8): the first height (above the former 40-60 km takeover; since T7c fix 2 the volume is
## drawn there too), how far up the next shape tile edge is searched and how far below / above it the two cameras are, the
## ground points' grid step (deg; 5 x 5 about SUB_CAMERA), the block radius (px), the detour height that forces a new
## flat geometry and the tolerance [E] (the mean relative block step; 2026-11-07: 1.42 % with the T8 fix, 27.4 % before;
## 2026-11-08 0.77 % with T7c's volume column - tightened from 3 %).
const REBAKE_HEIGHT_M := 63000.0
const REBAKE_SEARCH_M := 60000.0
const REBAKE_EDGE_M := 400.0
const REBAKE_DETOUR_M := 200000.0
const REBAKE_GRID_DEG := 0.12
const REBAKE_BLOCK_RADIUS := 10
const REBAKE_TOLERANCE := 0.015
const CLEAR_TOLERANCE := 0.02
## Section 5b (T7c): the climb's heights (the former takeover's band, cloud_baked_flat.gdshaderinc
## CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M), the ground grid step (deg; 5 x 5 about the aim point), the block's ground size
## across the view (m: a region of ~6 km, the fly-up sheets' 4 x 4 regions are 7-12 km) and the climb's tolerances
## straight down [E]: per point (linear luminance; 7 % ~ 3 sRGB codes of ~130, the user's mark; 2026-11-08 worst 5.2 %,
## mean 1.1 %) and the mean.
const TAKEOVER_HEIGHTS_M := Vector2(39500.0, 60500.0)
const TAKEOVER_GRID_DEG := 0.04
const TAKEOVER_BLOCK_M := 3000.0
const TAKEOVER_TOLERANCE := 0.07
const TAKEOVER_MEAN_TOLERANCE := 0.02
## (fix 2, review I1) the off-nadir angles of the broken layer's views (deg; looking north, the ray aimed at the
## layer's TAKEOVER_LAYER_M over the aim point), its low-Sun epoch (Sun ~12 deg up at SUB_CAMERA), the analog
## snapshot's sites (name, (lat, lon), off-nadir deg, UTC: a morning Sun over the Gulf, an evening one over the
## Atlantic's cumulus).
const TAKEOVER_ANGLES_DEG := [0.0, 30.0, 45.0, 60.0]
const TAKEOVER_LAYER_M := 1500.0
const TAKEOVER_LOW_SUN_UTC := "2024-04-08T20:15:00"
const TAKEOVER_ANALOG_SITES := [["Gulf 26 N 97 W", Vector2(26.0, -97.157), 45.0, "2024-04-08T14:00:00"],
		["cumulus 15 N 45 W, low Sun", Vector2(15.0, -45.0), 60.0, "2024-04-08T20:15:00"]]
## (fix 2) off nadir the 25 points' mean luminance change per km of climb within TAKEOVER_GLOBAL_PER_KM (the fly-up's
## global target, T7c: <= 1.5 % per km) [E]; their single points are reported (the view of the 3-D shapes changes with
## the distance - the other points' rays turn, the footprint's level changes - as much below the band: 25.8 -> 39.5 km
## worst points 14-50 %, 2026-11-08).
const TAKEOVER_GLOBAL_PER_KM := 0.015
## Section 3b (T7b): the analog hour of the epoch's month (2024-04-08 14 UTC -> April, 12 UTC).
const ANALOG_MONTH := 4
const ANALOG_HOUR := 12
const EARTH_CLOUDS_SHADER := "res://shaders/earth_clouds.gdshader"
const EARTH_SURFACE_INCLUDE := "res://shaders/earth_surface.gdshaderinc"
const WGS84_A_M := 6378137.0
const WGS84_E2 := 6.69437999014e-3

var _main: Node
var _world: World
var _failures := 0
var _samples := 0
var _save_dir := ""
var _no_3d_light := false  ## --no_3d_light: CloudLayer.far_3d_light off (the plane-parallel slab alone)
var _only_rebake := false  ## --only_rebake: section 5 alone
var _only_takeover := false  ## --only_takeover: section 5b alone
var _only_probe := false  ## --only_probe: section 1 (1, 1b, 1c) alone
var _files: Array[String] = []


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--save="):
			_save_dir = arg.trim_prefix("--save=")
		elif arg == "--no_3d_light":
			_no_3d_light = true
		elif arg == "--only_rebake":
			_only_rebake = true
		elif arg == "--only_probe":
			_only_probe = true
		elif arg == "--only_takeover":
			_only_takeover = true
	_run.call_deferred()


func _run() -> void:
	root.use_hdr_2d = true
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_world = _main.get_node("World")
	_world.weather.enabled = false
	for i in range(6000):  # the cloud kernels built (CloudKernels, workers) before anything is rendered
		await process_frame
		if i > 2 and _world.clouds.kernels_status() == "":
			break
	_world.clouds.far_3d_light = not _no_3d_light
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
	var broken := _slab("broken", BROKEN_LIQUID_KG_M3, BROKEN_FRACTION)
	if _only_rebake or _only_takeover:
		_show(broken)
		if not _only_takeover:
			await _rebake_tile_sample()
		await _takeover_sample()
		_finish()
		return

	# 1. The far field's slab radiance vs the core.
	await _probe_samples()
	await _bright_top_samples()
	await _meso_samples()
	await _spread_samples()
	await _projection_samples()
	if _only_probe:
		_finish()
		return
	# 2. Weather off: bit-identical to the shader without its cloud blocks (scaled space, outside the limit).
	await _identity_sample()
	# 3. The scaled-space hand-over: the game's cloud pass (the baked volume's flat levels, which follow the far
	# field's cover and light, baked steps 3-6) against the far field, 5 %.
	for deck in [["deck tau 10", DECK_LIQUID_KG_M3], ["deck tau 2", THIN_LIQUID_KG_M3]]:
		var snapshot := _slab(deck[0], deck[1], 1.0)
		_show(snapshot)
		await _handover_sample(deck[0] + ", the cloud pass", 2, HANDOVER_TOLERANCE, snapshot)
	_show(broken)
	await _handover_sample("broken cf 0.4, the cloud pass", 10, HANDOVER_TOLERANCE, broken)
	# 3b. The shipped analog set (1 deg, its 0.25 deg fine column; look fixes T7b, review I2): the far field and the
	# flat levels' even cells read the fine column, the placement the coarse layers.
	var analog := _analog_snapshot()
	if analog.is_empty():
		print("  SKIP  analog hand-over: no analog file for month %d %02d UTC under data/weather/analog" % [
				ANALOG_MONTH, ANALOG_HOUR])
	else:
		var rows := int(analog.get("rows", 0))
		var fine_rows := int(analog.get("fine_rows", 0))
		_check("analog snapshot %s carries its fine column" % String(analog["file"]).get_file(),
				fine_rows > rows and (fine_rows - 1) % (rows - 1) == 0
				and int(analog.get("fine_columns", 0)) == int(analog.get("columns", 0)) * ((fine_rows - 1) / (rows - 1)),
				"coarse %d x %d, fine %d x %d" % [rows, int(analog.get("columns", 0)), fine_rows,
				int(analog.get("fine_columns", 0))])
		_show(analog)
		await _handover_sample("analog 1 deg + fine column, the cloud pass", 10, HANDOVER_TOLERANCE, analog)
	# 4. From the Moon.
	await _moon_samples()
	# 5. The flat levels across a re-bake whose camera is in another shape tile (look fixes T8).
	_show(broken)
	await _rebake_tile_sample()
	await _takeover_sample()
	_finish()


func _finish() -> void:
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
	var values := await _probe(0, cases)
	var worst := 0.0
	var worst_case := ""
	var bad := 0
	for i in range(cases.size()):
		var c: Array = cases[i]
		var gpu := values[i]
		var cpu := _world.core.cloud_slab_radiance_mc(c[0], c[1], c[2], c[3])
		var error := absf(gpu - cpu) / maxf(cpu, PROBE_FLOOR)
		if error > PROBE_TOLERANCE:
			bad += 1
		if error > worst or is_nan(gpu):
			worst = error if not is_nan(gpu) else INF
			worst_case = "tau %.1f mu0 %.2f mu %.2f phi %.2f: GPU %.5f CPU %.5f" % [c[0], c[1], c[2], c[3], gpu, cpu]
	_check("far-field slab radiance vs slab_reflected_radiance_mc (%d geometries)" % cases.size(), bad == 0,
			"%d outside %.0f %%, worst %.2f %% (%s)" % [bad, PROBE_TOLERANCE * 100.0, worst * 100.0, worst_case])


## The probe shader's mode over the cases (four values each, one pixel per case); the red channel per case.
func _probe(mode: int, cases: Array) -> PackedFloat32Array:
	var colors := await _probe_colors(mode, cases)
	var out := PackedFloat32Array()
	out.resize(cases.size())
	for i in range(cases.size()):
		out[i] = colors[i].r
	return out


## The probe shader's mode over the cases: the colour per case.
func _probe_colors(mode: int, cases: Array) -> PackedColorArray:
	var width := mini(cases.size(), PROBE_ROW)
	var height := ceili(float(cases.size()) / width)
	var params := Image.create(width, height, false, Image.FORMAT_RGBAF)
	for i in range(cases.size()):
		var c: Array = cases[i]
		params.set_pixel(i % width, i / width, Color(c[0], c[1], c[2], c[3]))
	# a new viewport per call, sized before it enters the tree (a resized one read back its first frame's image)
	var viewport := SubViewport.new()
	viewport.size = Vector2i(width, height)
	viewport.use_hdr_2d = true
	viewport.own_world_3d = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(viewport)
	var camera := Camera3D.new()
	camera.current = true
	viewport.add_child(camera)
	var material := ShaderMaterial.new()
	material.shader = load("res://tests/cloud_far_probe.gdshader") as Shader
	material.set_shader_parameter("mode", mode)
	material.set_shader_parameter("params", ImageTexture.create_from_image(params))
	var quad := MeshInstance3D.new()
	var mesh := QuadMesh.new()
	mesh.size = Vector2(2.0, 2.0)
	quad.mesh = mesh
	quad.material_override = material
	quad.position = Vector3(0.0, 0.0, -1.0)
	quad.extra_cull_margin = 16384.0
	viewport.add_child(quad)
	for i in range(4):
		await process_frame
	var image := viewport.get_texture().get_image()
	viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	viewport.queue_free()
	var out := PackedColorArray()
	out.resize(cases.size())
	for i in range(cases.size()):
		out[i] = image.get_pixel(i % width, i / width)
	return out


## The two-stream factor of cloud_far_bright_top_transmittance (one channel), the reference for 1b.
static func _bright_top_reference(t: float, mu: float, cover: float, tau_cloud: float, g: float) -> float:
	var tc := clampf(t, 1.0e-6, 1.0)
	var tau := -log(tc) * mu
	var s := tau / (4.0 / 3.0 + tau)
	var x := 0.75 * (1.0 - g) * tau_cloud
	var albedo := lerpf(TABLE_GROUND_ALBEDO, x / (1.0 + x), clampf(cover, 0.0, 1.0))
	var down := (1.0 - TABLE_GROUND_ALBEDO * s) / (1.0 - albedo * s)
	var total := ((2.0 / 3.0 + mu) + (2.0 / 3.0 - mu) * tc) / (4.0 / 3.0 + tau)
	return maxf(total, t) * down


## 1b. The air above a bright top (look fixes T7 step 3, T7b review I2): the GPU function against its two-stream form
## over a grid, its limits (1 without air above; the Beer term alone at the tables' ground albedo, between T and 1;
## never below T) and the cloud's blue / red increment of review I2's estimate (+8-10 % for A 0.6-0.75 under the
## Rayleigh depth above a 2 km top, 0.15 blue / 0.04 red, nadir: BRIGHT_TOP_BR_RANGE).
func _bright_top_samples() -> void:
	var g := float(ShaderGlobals.value(&"cloud_asymmetry_g"))
	var cases: Array = []
	for t in [1.0, 0.99, 0.9, 0.5, 0.1, 0.01]:
		for mu in [0.02, 0.05, 0.3, 0.7, 1.0]:
			for cover in [0.0, 0.5, 1.0]:
				for tau_cloud in [0.0, 2.0, 30.0]:
					cases.append([t, mu, cover, tau_cloud])
	# blue and red at A 0.6 and 0.75 (x = A / (1 - A) = 0.75 (1 - g) tau)
	var grid := cases.size()
	var albedos := [0.6, 0.75]
	for albedo: float in albedos:
		var tau_a := albedo / (1.0 - albedo) / (0.75 * (1.0 - g))
		cases.append([exp(-0.15), 1.0, 1.0, tau_a])
		cases.append([exp(-0.04), 1.0, 1.0, tau_a])
	var values := await _probe(1, cases)
	var worst := 0.0
	var worst_case := ""
	var limits_ok := true
	var limit_case := ""
	for i in range(grid):
		var c: Array = cases[i]
		var gpu := values[i]
		var reference := _bright_top_reference(c[0], c[1], c[2], c[3], g)
		var error := absf(gpu - reference) / maxf(reference, 1.0e-6)
		if error > worst or is_nan(gpu):
			worst = error if not is_nan(gpu) else INF
			worst_case = "T %.2f mu %.2f cover %.1f tau %.0f: GPU %.6f ref %.6f" % [c[0], c[1], c[2], c[3], gpu, reference]
		var ok := gpu >= float(c[0]) * (1.0 - BRIGHT_TOP_TOLERANCE)  # never below the Beer term
		if c[0] == 1.0:
			ok = ok and absf(gpu - 1.0) <= BRIGHT_TOP_TOLERANCE  # no air above: 1
		if c[2] == 0.0 or c[3] == 0.0:
			ok = ok and gpu <= 1.0 + BRIGHT_TOP_TOLERANCE  # the tables' ground: transmission only, at most 1
		if not ok and limit_case == "":
			limit_case = "T %.2f mu %.2f cover %.1f tau %.0f: %.6f" % [c[0], c[1], c[2], c[3], gpu]
		limits_ok = limits_ok and ok
	_check("bright top vs its two-stream form (%d cases)" % grid, worst <= BRIGHT_TOP_TOLERANCE,
			"worst %.4f %% (%s)" % [worst * 100.0, worst_case])
	_check("bright top limits: 1 without air, <= 1 over the tables' ground, >= the Beer term", limits_ok,
			"first violation: %s" % (limit_case if limit_case != "" else "none"))
	var parts: Array[String] = []
	var in_range := true
	for k in range(albedos.size()):
		var i := grid + 2 * k
		var ratio := (values[i] / exp(-0.15)) / (values[i + 1] / exp(-0.04))
		in_range = in_range and ratio >= BRIGHT_TOP_BR_RANGE.x and ratio <= BRIGHT_TOP_BR_RANGE.y
		parts.append("A %.2f: B/R x%.4f" % [albedos[k], ratio])
	_check("bright top: the cloud's blue / red increment (review I2 estimate)", in_range,
			"%s (range %.2f-%.2f)" % [", ".join(parts), BRIGHT_TOP_BR_RANGE.x, BRIGHT_TOP_BR_RANGE.y])


## 1d. The cloud-to-cloud spread (look fixes T7b, cloud_spread_factor): over SPREAD_POINTS random points on the sphere
## the factor's mean is 1 within SPREAD_MEAN_TOLERANCE (the cell's water kept) and its fractional standard deviation
## SPREAD_FSD within SPREAD_FSD_TOLERANCE; its 10 / 50 / 90 % points printed (the exponential, nu 1: 0.105 / 0.69 / 2.30).
func _spread_samples() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = 20261109
	var cases: Array = []
	for i in range(SPREAD_POINTS):
		cases.append([0.0, rng.randf_range(-PI, PI), rng.randf_range(-1.0, 1.0), 0.0])
	var values := await _probe(3, cases)
	var sum := 0.0
	var sum2 := 0.0
	for v in values:
		sum += v
		sum2 += v * v
	var mean := sum / SPREAD_POINTS
	var fsd := sqrt(maxf(sum2 / SPREAD_POINTS - mean * mean, 0.0)) / maxf(mean, 1.0e-6)
	var sorted := Array(values)
	sorted.sort()
	_check("cloud-to-cloud spread: mean 1 (+-%.2f), fsd %.2f (+-%.2f)" % [SPREAD_MEAN_TOLERANCE, SPREAD_FSD,
			SPREAD_FSD_TOLERANCE],
			absf(mean - 1.0) <= SPREAD_MEAN_TOLERANCE and absf(fsd - SPREAD_FSD) <= SPREAD_FSD_TOLERANCE,
			"mean %.4f, fsd %.3f, p10 / p50 / p90 %.3f / %.3f / %.3f" % [mean, fsd, sorted[SPREAD_POINTS / 10],
			sorted[SPREAD_POINTS / 2], sorted[SPREAD_POINTS * 9 / 10]])


## 1c. The cover pattern's mean (look fixes T7 B, T7b review I2): cloud_far_meso_cover over MESO_POINTS random points on
## the sphere keeps the cell's cover as its mean (f = Phi(a + k z), a = Phi^-1(C) sqrt(1 + k^2): E[f] = C for a Gaussian
## z; the noise's quantile-normalised z is near Gaussian), stays in [0, 1], and draws a pattern (its spread at C 0.5
## > MESO_SPREAD_MIN, else the sub-grid is off); at the footprints of one and four pattern texels (8 / 32 km: levels
## 0 and 2, contrast full).
func _meso_samples() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = 20261108
	var points: Array[Vector2] = []
	for i in range(MESO_POINTS):
		points.append(Vector2(rng.randf_range(-PI, PI), rng.randf_range(-1.0, 1.0)))
	var covers := [0.1, 0.3, 0.5, 0.7, 0.9]
	var footprints := [8000.0, 32000.0]
	var cases: Array = []
	for footprint: float in footprints:
		for cover: float in covers:
			for q in points:
				cases.append([cover, q.x, q.y, footprint])
	var values := await _probe(2, cases)
	var worst := 0.0
	var parts: Array[String] = []
	var bounded := true
	var spread_ok := true
	for fi in range(footprints.size()):
		for ci in range(covers.size()):
			var base := (fi * covers.size() + ci) * MESO_POINTS
			var sum := 0.0
			var sum2 := 0.0
			for j in range(MESO_POINTS):
				var v := values[base + j]
				bounded = bounded and v >= 0.0 and v <= 1.0 and not is_nan(v)
				sum += v
				sum2 += v * v
			var mean := sum / MESO_POINTS
			var spread := sqrt(maxf(sum2 / MESO_POINTS - mean * mean, 0.0))
			worst = maxf(worst, absf(mean - float(covers[ci])))
			if covers[ci] == 0.5:
				spread_ok = spread_ok and spread > MESO_SPREAD_MIN
			parts.append("%.0f km C %.1f: %.4f (sd %.3f)" % [float(footprints[fi]) / 1000.0, covers[ci], mean, spread])
	_check("meso cover pattern keeps the cell's cover as its mean (%d points)" % MESO_POINTS,
			worst <= MESO_MEAN_TOLERANCE and bounded and spread_ok,
			"worst |mean - C| %.4f (tolerance %.3f), in [0, 1] %s, pattern %s; %s" % [worst, MESO_MEAN_TOLERANCE, bounded,
			spread_ok, "; ".join(parts)])


## 1e. The volume's depth-aware cover (look fixes T7c, cloud_volume_fraction): the shapes seen straight down through a
## block of uniform fraction f and depth D cover more than f (the 3-D noise's heights are displaced against each other);
## with the volume's fraction the block's nadir cover is f (the data's total cover: maximum overlap inside a block)
## within PROJECTION_TOLERANCE (water clouds, cirrus), f 0.02-0.8 and D 0.25-3 km (PROJECTION_POINTS columns each).
func _projection_samples() -> void:
	var cases: Array = []
	for type in PROJECTION_TYPES:
		for depth: float in PROJECTION_DEPTHS:
			for f: float in PROJECTION_FRACTIONS:
				for j in range(PROJECTION_POINTS):
					cases.append([f, depth, float(type), float(j + 1)])
	var colors := await _probe_colors(4, cases)
	var worst := 0.0
	var worst_ice := 0.0
	var lines: Array[String] = []
	var k := 0
	for type in PROJECTION_TYPES:
		for depth: float in PROJECTION_DEPTHS:
			var parts: Array[String] = []
			for f: float in PROJECTION_FRACTIONS:
				var raw := 0.0
				var vol := 0.0
				var opacity := 0.0
				for j in range(PROJECTION_POINTS):
					raw += colors[k].r
					vol += colors[k].g
					opacity += colors[k].b
					k += 1
				raw /= PROJECTION_POINTS
				vol /= PROJECTION_POINTS
				opacity /= PROJECTION_POINTS
				if type == 2 and f >= PROJECTION_ICE_FRACTION_MIN:
					worst_ice = maxf(worst_ice, absf(vol - f))
				elif type != 2:
					worst = maxf(worst, absf(vol - f))
				parts.append("%.2f: %.3f -> %.3f (%.3f)" % [f, raw, vol, opacity])
			lines.append("type %d D %.0f m: %s" % [type, depth, ", ".join(parts)])
	print("projected cover (f: shapes -> volume):\n  " + "\n  ".join(lines))
	_check("the volume's nadir cover of a uniform block is its fraction (%d columns per case)" % PROJECTION_POINTS,
			worst <= PROJECTION_TOLERANCE.x and worst_ice <= PROJECTION_TOLERANCE.y,
			"worst |cover - f| water clouds %.4f (tolerance %.3f), cirrus %.4f (%.3f)" % [worst, PROJECTION_TOLERANCE.x,
			worst_ice, PROJECTION_TOLERANCE.y])


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
	if not snapshot.is_empty():
		# Look fixes T3: the scaled-space Earth draws the sea with the terrain's sea model (body, Fresnel sky, glint),
		# so the cloudless ground itself no longer steps across the limit, the sub-camera glint included.
		var ground_worst := 0.0
		var ground_trace := ""
		for i in range(points.size()):
			ground_worst = maxf(ground_worst, absf(ground[false][i] - ground[true][i]) / maxf(ground[true][i], 1.0e-6))
			ground_trace += " %.4f/%.4f" % [ground[true][i], ground[false][i]]
		_check("%s, weather off: the ground's %d x %d block luminance inside / outside at %d points within %.0f %%" % [
				label, 2 * radius + 1, 2 * radius + 1, points.size(), tolerance * 100.0], ground_worst <= tolerance,
				"worst %.2f %%:%s" % [ground_worst * 100.0, ground_trace])
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
	var file: String = await _era5_file()
	if file == "":
		print("  SKIP  Moon view: no ERA5 snapshot for %s (not cached, download failed or > %d s)" % [EPOCH_UTC,
				ERA5_WAIT_S])
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


## The ERA5 snapshot of the Moon view: the cached ERA5_FILE, else the one the weather service downloads and builds for
## the clock's epoch (EPOCH_UTC, the "before" file of its ERA5 pair; online, ~1-3 min). "" when there is none within
## ERA5_WAIT_S. The service is switched off again (a few frames, so that it clears the core's pair) before the check
## sets its own.
## The shipped analog snapshot of ANALOG_MONTH / ANALOG_HOUR (data/weather/analog/index.json), {} when absent.
func _analog_snapshot() -> Dictionary:
	var dir := Paths.data_dir().path_join("weather/analog")
	var index: Variant = JSON.parse_string(FileAccess.get_file_as_string(dir.path_join("index.json")))
	if not (index is Dictionary):
		return {}
	for entry: Dictionary in (index as Dictionary).get("entries", []):
		if int(entry["month"]) == ANALOG_MONTH and int(entry["hour_utc"]) == ANALOG_HOUR:
			var file := dir.path_join(String(entry["file"]))
			if not FileAccess.file_exists(file):
				return {}
			var loaded: Dictionary = _world.core.weather_load_snapshot(file)
			if not loaded.get("ok", false):
				return {}
			loaded["file"] = file
			return loaded
	return {}


func _era5_file() -> String:
	var cached := Paths.data_dir().path_join(ERA5_FILE)
	if FileAccess.file_exists(cached):
		return cached
	print("  ERA5 snapshot %s missing: fetching the epoch's ERA5 pair through the weather service" % cached)
	_world.weather.enabled = true
	var start_ms := Time.get_ticks_msec()
	while Time.get_ticks_msec() - start_ms < ERA5_WAIT_S * 1000:
		await process_frame
		if _world.weather.settled() and _world.weather.active_kind() != "analog" and _world.weather.active_kind() != "":
			break
	var kind: String = _world.weather.active_kind()
	var files: Array = _world.weather.active_files()
	_world.weather.enabled = false
	for i in range(SETTLE_FRAMES):
		await process_frame
	if kind != "era5" or files.is_empty():
		print("  ERA5 fetch: active source '%s' after %.0f s" % [kind, (Time.get_ticks_msec() - start_ms) / 1000.0])
		return ""
	print("  ERA5 fetch: %s after %.0f s" % [String(files[0]).get_file(), (Time.get_ticks_msec() - start_ms) / 1000.0])
	return files[0]


static func _luminance(c: Color) -> float:
	return 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b


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


## 5. The flat levels across a re-bake (look fixes T8): the broken layer seen straight down from just below and above
## the first shape-tile edge over REBAKE_HEIGHT_M (CloudLayer._shape_origin's wrap steps by a whole tile there), each
## with a new flat geometry about the camera; the ground points' block luminance the same within REBAKE_TOLERANCE
## (the mean over the points, relative; the worst point reported). Both heights above the takeover (60 km: the flat
## levels draw every cloud).
func _rebake_tile_sample() -> void:
	# the first tile edge above REBAKE_HEIGHT_M; the heights REBAKE_EDGE_M below and above it (nearly the same view)
	var edge := REBAKE_HEIGHT_M
	var tile0 := _shape_tile(edge)
	while _shape_tile(edge) == tile0 and edge < REBAKE_HEIGHT_M + REBAKE_SEARCH_M:
		edge += 10.0
	var h1 := edge - REBAKE_EDGE_M
	var h2 := edge + REBAKE_EDGE_M
	var points: Array[Vector3] = []
	for i in range(-2, 3):
		for j in range(-2, 3):
			points.append(_itrf(SUB_CAMERA.x + REBAKE_GRID_DEG * i, SUB_CAMERA.y + REBAKE_GRID_DEG * j))
	var lums := {}
	var tiles := {}
	for h: float in [h1, h2]:
		# a new flat geometry about this camera (a re-bake keeps the shown one while it holds the height range): from
		# REBAKE_DETOUR_M, outside that range, back down
		await _place(SUB_CAMERA, WGS84_A_M + REBAKE_DETOUR_M)
		await _place(SUB_CAMERA, WGS84_A_M + h)
		var camera_fixed := _world.earth.fixed_to_gcrf.transposed() * _world.camera.position_gcrf_m
		tiles[h] = _shape_tile_of(camera_fixed)
		var image := _frame()
		_save(image, "t8check_rebake_%.0f" % h)
		var values: Array[float] = []
		for p in points:
			values.append(_block_luminance(image, _pixel_of(p), REBAKE_BLOCK_RADIUS))
		lums[h] = values
	_check("re-bake: the heights %.1f / %.1f km put the camera in different shape tiles" % [h1 * 1e-3, h2 * 1e-3],
			tiles[h1] != tiles[h2], "tiles %s / %s" % [tiles[h1], tiles[h2]])
	var sum_diff := 0.0
	var sum := 0.0
	var worst := 0.0
	for i in range(points.size()):
		var a: float = lums[h1][i]
		var b: float = lums[h2][i]
		sum_diff += absf(b - a)
		sum += a
		worst = maxf(worst, absf(b - a) / maxf(a, 1.0e-6))
	var mean := sum_diff / maxf(sum, 1.0e-6)
	_check("re-bake across a shape tile, broken cf 0.4: the %d ground points' %d x %d block luminance within %.1f %%" % [
			points.size(), 2 * REBAKE_BLOCK_RADIUS + 1, 2 * REBAKE_BLOCK_RADIUS + 1, REBAKE_TOLERANCE * 100.0],
			mean <= REBAKE_TOLERANCE, "mean %.2f %%, worst point %.2f %%" % [mean * 100.0, worst * 100.0])


## 5b. The climb through the former 40-60 km takeover (look fixes T7c; fix 2,
## review I1: the oblique views, two more sites, a low Sun). A case: label, snapshot, the aim point (lat, lon), its
## off-nadir angle (deg), the UTC.
func _takeover_sample() -> void:
	var broken := _slab("broken", BROKEN_LIQUID_KG_M3, BROKEN_FRACTION)
	var analog := _analog_snapshot()
	var cases: Array = []
	for angle: float in TAKEOVER_ANGLES_DEG:
		cases.append(["broken cf 0.4", broken, SUB_CAMERA, angle, EPOCH_UTC])
	cases.append(["broken cf 0.4, low Sun", broken, SUB_CAMERA, 45.0, TAKEOVER_LOW_SUN_UTC])
	if analog.is_empty():
		print("  SKIP  takeover on the analog snapshot: no analog file for month %d %02d UTC" % [ANALOG_MONTH,
				ANALOG_HOUR])
	else:
		for site: Array in TAKEOVER_ANALOG_SITES:
			cases.append(["analog " + site[0], analog, site[1], site[2], site[3]])
	for c: Array in cases:
		_show(c[1])
		_world.clock.set_time(_world.core.utc_to_tdb_s(c[4]))
		await _takeover_case(c[0], c[2], c[3])
	_world.clock.set_time(_world.core.utc_to_tdb_s(EPOCH_UTC))
	_world.set_focus(World.FOCUS_EARTH)
	_world.camera.fov = 60.0
	_show(broken)


## One 5b case: the aim point p (lat, lon) seen off_nadir_deg off the nadir (looking north), the camera on the same ray
## to p at TAKEOVER_HEIGHTS_M.x and .y; 25 ground points about p (TAKEOVER_GRID_DEG), block luminance (blocks of
## TAKEOVER_BLOCK_M across the view), the in-cloud texture (the high-pass SD in a block of 3 x TAKEOVER_BLOCK_M about p,
## reported). Straight down the points within TAKEOVER_TOLERANCE and their mean within TAKEOVER_MEAN_TOLERANCE; off nadir
## their mean within TAKEOVER_GLOBAL_PER_KM per km.
func _takeover_case(label: String, p: Vector2, off_nadir_deg: float) -> void:
	var points: Array[Vector3] = []
	for i in range(-2, 3):
		for j in range(-2, 3):
			points.append(_itrf(p.x + TAKEOVER_GRID_DEG * i, p.y + TAKEOVER_GRID_DEG * j))
	var lums := {}
	var texture := {}
	var theta := deg_to_rad(off_nadir_deg)
	for h: float in [TAKEOVER_HEIGHTS_M.x, TAKEOVER_HEIGHTS_M.y]:
		# the camera south of p at h; its ray theta off its nadir meets the layer's height over p (central angle gamma)
		var r_c := WGS84_A_M + h
		var r_p := WGS84_A_M + TAKEOVER_LAYER_M
		var gamma := asin(clampf(r_c * sin(theta) / r_p, -1.0, 1.0)) - theta
		var distance := sqrt(r_c * r_c + r_p * r_p - 2.0 * r_c * r_p * cos(gamma))
		_world.set_ground_site({"id": "custom", "name": "custom", "body": "earth",
				"latitude_deg": p.x - rad_to_deg(gamma), "longitude_deg": p.y, "height_m": 0.0, "eye_height_m": h})
		_world.camera.fov = 60.0
		_world.camera.azimuth = 0.0
		_world.camera.elevation = -(PI * 0.5 - theta - gamma)
		await _settle()
		var image := _frame()
		_save(image, "t7c_takeover_%s_%.0f_%.0f" % [label.replace(" ", "_").replace(",", ""), off_nadir_deg, h])
		var pixels_per_m := image.get_height() / (2.0 * distance * tan(deg_to_rad(_world.camera.fov) * 0.5))
		var radius := maxi(int(round(TAKEOVER_BLOCK_M * pixels_per_m)), 2)
		var values: Array[float] = []
		for q in points:
			values.append(_block_luminance(image, _pixel_of(q), radius))
		lums[h] = values
		texture[h] = _highpass_sd(image, _pixel_of(points[12]), 3 * radius)
	var h0 := TAKEOVER_HEIGHTS_M.x
	var h1 := TAKEOVER_HEIGHTS_M.y
	var climb := _point_stats(lums[h0], lums[h1])
	var per_km := climb.y / ((h1 - h0) * 1e-3)
	var detail := "worst point %.2f %%, mean %.2f %% (%.3f %% per km), texture (high-pass SD) %.4f / %.4f" % [
			climb.x * 100.0, climb.y * 100.0, per_km * 100.0, texture[h0], texture[h1]]
	if off_nadir_deg == 0.0:
		_check(("climb %.1f -> %.1f km, %s, straight down: the %d ground points' block luminance within %.0f %%, their "
				+ "mean within %.0f %%") % [h0 * 1e-3, h1 * 1e-3, label, points.size(), TAKEOVER_TOLERANCE * 100.0,
				TAKEOVER_MEAN_TOLERANCE * 100.0], climb.x <= TAKEOVER_TOLERANCE and climb.y <= TAKEOVER_MEAN_TOLERANCE,
				detail)
	else:
		_check("climb %.1f -> %.1f km, %s, %.0f deg off nadir: the %d ground points' mean within %.1f %% per km" % [
				h0 * 1e-3, h1 * 1e-3, label, off_nadir_deg, points.size(), TAKEOVER_GLOBAL_PER_KM * 100.0],
				per_km <= TAKEOVER_GLOBAL_PER_KM, detail)


## The worst relative difference of b against a and the relative difference of their sums.
static func _point_stats(a: Array, b: Array) -> Vector2:
	var sum_a := 0.0
	var sum_b := 0.0
	var worst := 0.0
	for i in range(a.size()):
		sum_a += a[i]
		sum_b += b[i]
		worst = maxf(worst, absf(float(b[i]) - float(a[i])) / maxf(float(a[i]), 1.0e-6))
	return Vector2(worst, absf(sum_b - sum_a) / maxf(sum_a, 1.0e-6))


## Waits until the terrain and the clouds have settled (as _place does).
func _settle() -> void:
	var idle := 0
	for i in range(MAX_FRAMES):
		await process_frame
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		var uploading: bool = _world.clouds.state()["uploading"]
		var baked := _world.clouds.effect.baked
		var baking: bool = ShaderGlobals.value(&"cloud_enabled") and (baked == null or not baked.settled())
		idle = idle + 1 if loading == 0 and not uploading and not baking else 0
		if idle >= SETTLE_FRAMES and i >= 2 * SETTLE_FRAMES:
			break


## The SD of the luminance minus its 9 x 9 box mean over the (2 r + 1)^2 block about centre (the in-cloud texture).
static func _highpass_sd(image: Image, centre: Vector2, radius: int) -> float:
	var w := image.get_width()
	var h := image.get_height()
	var x0 := clampi(int(round(centre.x)) - radius, 4, w - 5)
	var x1 := clampi(int(round(centre.x)) + radius, 4, w - 5)
	var y0 := clampi(int(round(centre.y)) - radius, 4, h - 5)
	var y1 := clampi(int(round(centre.y)) + radius, 4, h - 5)
	var bw := x1 - x0 + 9
	var bh := y1 - y0 + 9
	# summed-area table of the luminance (the box means in O(1) per pixel)
	var sat := PackedFloat64Array()
	sat.resize((bw + 1) * (bh + 1))
	for y in range(bh):
		var row := 0.0
		for x in range(bw):
			var c := image.get_pixel(x0 - 4 + x, y0 - 4 + y)
			row += 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b
			sat[(y + 1) * (bw + 1) + x + 1] = sat[y * (bw + 1) + x + 1] + row
	var sum := 0.0
	var sum2 := 0.0
	var n := 0
	for y in range(4, bh - 4):
		for x in range(4, bw - 4):
			var box := (sat[(y + 5) * (bw + 1) + x + 5] - sat[(y - 4) * (bw + 1) + x + 5]
					- sat[(y + 5) * (bw + 1) + x - 4] + sat[(y - 4) * (bw + 1) + x - 4]) / 81.0
			var c := image.get_pixel(x0 - 4 + x, y0 - 4 + y)
			var d := 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b - box
			sum += d
			sum2 += d * d
			n += 1
	var m := sum / maxf(n, 1)
	return sqrt(maxf(sum2 / maxf(n, 1) - m * m, 0.0))


## The cumulus shape tile (the unwrapped origin's integer part, CloudLayer._shape_origin) of a camera straight above
## SUB_CAMERA at this height.
func _shape_tile(height_m: float) -> Vector3:
	return _shape_tile_of(_itrf(SUB_CAMERA.x, SUB_CAMERA.y).normalized() * (WGS84_A_M + height_m))


static func _shape_tile_of(camera_fixed: Vector3) -> Vector3:
	var render := Frames.to_render(camera_fixed)
	var stretched := Vector3(camera_fixed.x, camera_fixed.y, camera_fixed.z * CloudLayer.POLAR_STRETCH)
	var height := stretched.length() - CloudLayer.WGS84_A_M
	var k := 1.0 / CloudLayer.SHAPE_TILE_V_M - 1.0 / CloudLayer.SHAPE_TILE_H_M
	return (render / CloudLayer.SHAPE_TILE_H_M + render.normalized() * (height * k)).floor()


## The orbit camera over (lat, lon) at distance_m from the Earth's centre; waits until the terrain and the clouds have
## settled.
func _place(sub_camera: Vector2, distance_m: float) -> void:
	var up_gcrf := _world.earth.fixed_to_gcrf * _itrf(sub_camera.x, sub_camera.y).normalized()
	var u := Frames.to_render(up_gcrf).normalized()
	_world.camera.pitch = -asin(u.y)
	_world.camera.yaw = atan2(u.x, u.z)
	_world.camera.distance_m = distance_m
	var idle := 0
	for i in range(MAX_FRAMES):
		await process_frame
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		var uploading: bool = _world.clouds.state()["uploading"]
		var baked := _world.clouds.effect.baked
		var baking: bool = ShaderGlobals.value(&"cloud_enabled") and (baked == null or not baked.settled())
		idle = idle + 1 if loading == 0 and not uploading and not baking else 0
		if idle >= SETTLE_FRAMES and i >= 2 * SETTLE_FRAMES:
			break


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
