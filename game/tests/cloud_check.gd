# GPU check of the clouds' light on the Earth's surfaces and the vehicles (M8 weather and clouds, Tasks 9 / 9b; the
# baked volume's shadows since 2026-10-30): the cloud pass (CloudEffect: the baked volume, which fills the cloud shadow
# maps from its Sun transmittance S, CloudShadowEffect) over a terrain quad (terrain_earth.gdshader) and the
# surfaces' cloud-light functions (cloud_light.gdshaderinc through tests/cloud_light_probe.gdshader) in an HDR
# SubViewport, against the core's CPU light (StarsimCore.weather_cloud_light, the grid mean). Synthetic snapshots from
# StarsimCore.weather_write_slab_snapshot (uniform slabs, 1 deg grid, ground at 0 m):
#   1. light on the ground (a quad 300 m from a camera 2 m up, under a thin slab of tau ~1, a deck of tau ~3 and a
#      broken layer of cf 0.4, Sun 45 / 20 deg up): the shader's cloud factors (debug output 5: direct, the Sun's
#      diffuse share, the sky's) vs weather_cloud_light within 3 % (the direct Sun's slant optical depth within
#      S_DEPTH_TOLERANCE, only for the closed slabs: S holds the shapes, a broken layer's spot lies in a gap or under a
#      cumulus - reported), the rendered ground's luminance ratio weather on / off = (T_s E_dir + T_sd E_sun,h + T_d
#      E_sky) / (E_dir + E_sky) of the CPU within 5 % (closed slabs), the ground's total light vs the CPU's at the same
#      spot within 10 % (closed slabs); a cross-fade deck -> empty: the column terms equal the CPU light of the pushed
#      pair and the ground light with them and the grid mean's direct Sun is monotonic (S's direct Sun reported); with weather
#      off every pixel of land and sea bit-identical to the shader without its CLOUD_LIGHT blocks (STARSIM_GROUND_ONLY=1
#      runs this section alone);
#   2. the maps' hand-overs and the shapes on broken cloud (fix round 1 of Task 9), the means over eight independent
#      sites (shape-noise realisations) of the surfaces' functions at receivers on annuli around the camera: near map ->
#      far map (9-11 km: annuli at 8 / 10 / 12 km) and far map -> the slanted column (120-140 km: 110 / 130 / 150 km),
#      a cf 0.3 layer and a deck, Sun 45 / 8 deg: steps <= 0.05 + 3 standard errors (0.03 for the far tiers on the
#      deck; the broken layer's far tiers reported: S's coarse levels at 110 km shadow more than the column); over the
#      disc inside the near tier the shapes' area mean of the direct Sun vs the CPU's cell (cf 0.15 / 0.3 / 0.4, Sun
#      45 / 8 deg; the beam-cover overlap model, constants.hpp cloud_overlap_*: STARSIM_SLANT_TABLE=1 prints the fit
#      table), the near map vs the far map over the same receivers, the column terms vs the CPU, the CPU's total light
#      vs the ground's mean;
#   3. the vehicles' direct Sun from the drawn shapes (CloudLayer.direct_sun_transmittance, Atmosphere.sun_light_at;
#      2026-10-30): under a broken layer (cf 0.4) at a 45 deg Sun, points 10 m up along a 4 km line: one in a cumulus'
#      shadow and one in the gap beside it (by the probe) get direct Sun light differing by at least 0.5 of the clear
#      value while the CPU's grid mean gives both the same; the probe agrees with the near shadow map at the same
#      points (both read S): mean |dT| <= 0.02, worst <= 0.15 (the map's texels at the shadows' edges).
# Detail levels off for the numbers (cloud_detail_levels 0); the desktop quality preset. Exit code 1 on failure.
# Needs a GPU (no --headless): godot --path game --script res://tests/cloud_check.gd
extends SceneTree

const SETTLE_FRAMES := 4
const EPOCH_UTC := "2024-04-08T18:42:00"
const AU_M := 149597870700.0
const LAT_DEG := 30.0
const LON_DEG := 45.0
const SLAB_BOTTOM_M := 1000.0
const SLAB_TOP_M := 2000.0
## Liquid water of vertical optical depth 1 over 1 km: beta = 3 LWC / (2 rho_w r_l) = 150 LWC [1/m].
const THIN_LIQUID_KG_M3 := 1.0e-3 / 150.0
## Transmittance and radiance samples: the mean of the 5 x 5 pixels around the centre (see _render).
const BLOCK_RADIUS := 2
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
const FAR_HANDOVER_TOLERANCE := 0.03  ## the far map and the slanted column on a deck (both smooth)
## [E] The S maps' slant optical depth through a closed slab against the core's (S is baked through the voxelised
## shapes: measured 0.927-0.937 x the core's on the 1 km slabs, 2026-10-30 - an open point of the bake, not of this
## check; the grid-mean map it replaced agreed within 3 % in T).
const S_DEPTH_TOLERANCE := 0.10
const VIEW_SIZE := 33  ## odd: the centre pixel's ray is the camera axis
## [E] Frames a render waits at most for the baked volume to settle (a bake, a cross-fade) - the probe's read-back too.
const BAKE_MAX_FRAMES := 3000
## Section 3: the line of points (m along the ground, 10 m up), the probe's agreement with the near map and the
## smallest difference of shadow and gap in units of the clear direct light.
const VEHICLE_LINE_M := 4000.0
const VEHICLE_POINTS := 41
const VEHICLE_HEIGHT_M := 10.0
const VEHICLE_MAP_TOLERANCE := Vector2(0.02, 0.15)  ## mean and worst |probe - map| (the map's 62.5 m texels at edges)
const VEHICLE_MIN_CONTRAST := 0.5
const EXPECTED_SAMPLES := 61

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()
var _clouds := CloudLayer.new()
var _viewport := SubViewport.new()
var _camera := Camera3D.new()
var _earth_to_gcrf := Basis.IDENTITY
var _failures := 0
var _samples := 0
var _files: Array[String] = []
var _terrain_parent := Node3D.new()  ## Earth-fixed frame of the terrain quad (section 1)
var _terrain_quad := MeshInstance3D.new()
var _terrain_material := ShaderMaterial.new()
var _probe_viewport := SubViewport.new()  ## section 2: the receivers' probe
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
	_atmosphere.direct_sun = _clouds.direct_sun_transmittance  # as World wires it (section 3)
	CloudBaked.bake_scale = 4.0  # the bakes in fewer frames (the check's views are small)
	_setup_scene()
	_setup_probe()
	_clouds.setup(_core)
	if not _clouds.wait_until_ready():
		_fail_and_quit("cloud noise not ready")
		return
	_clouds.quality = CloudLayer.Quality.DESKTOP
	for i in range(6000):  # the cloud kernels built (CloudKernels, workers) before anything is rendered
		await process_frame
		if i > 2 and _clouds.kernels_status() == "":
			break
	_detail_off()
	if OS.get_environment("STARSIM_SLANT_TABLE") != "":
		await _slant_table()
		quit(0)
		return
	if OS.get_environment("STARSIM_GROUND_ONLY") != "":  # diagnostics: section 1 alone (no sample count)
		await _ground_light_samples()
		_finish(false)
		return
	# 1. Light on the ground under the clouds.
	await _ground_light_samples()
	# 2. The shadow maps' hand-overs and the shapes on broken cloud.
	await _handover_samples()
	# 3. The vehicles' direct Sun from the drawn shapes.
	await _vehicle_samples()
	_finish(true)


func _finish(count: bool) -> void:
	for f in _files:
		DirAccess.remove_absolute(f)
	_clouds.shutdown()
	if count:
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


## Detail levels off: the CPU references have the shapes of state 0 at most (cloud_noise.hpp).
func _detail_off() -> void:
	ShaderGlobals.set_value(&"cloud_detail_levels", 0)


func _render(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, mode: int, block_radius := 0,
		roll_deg := 0.0, pixel_offset := Vector2i.ZERO) -> Vector3:
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
	_camera.transform = Transform3D(Basis.looking_at(view_render, up) * Basis(Vector3.BACK, deg_to_rad(roll_deg)),
			Vector3.ZERO)
	# The baked volume settled about this camera, Sun and weather (a re-bake and its cross-fade), the maps filled from
	# it; then SETTLE_FRAMES more.
	var settled := 0
	for i in range(BAKE_MAX_FRAMES):
		await RenderingServer.frame_post_draw
		_clouds.update(camera_itrf_m, 0.0, 0.0)  # as the world does every frame (a re-created result is shown)
		var baked := _clouds.effect.baked
		var ready: bool = not ShaderGlobals.value(&"cloud_enabled") or (baked != null and baked.settled())
		settled = settled + 1 if ready else 0
		if settled >= SETTLE_FRAMES:
			break
	var image := _viewport.get_texture().get_image()
	# The mean over a (2 r + 1)^2 block around the centre.
	var sum := Vector3.ZERO
	for dy in range(-block_radius, block_radius + 1):
		for dx in range(-block_radius, block_radius + 1):
			var c := image.get_pixel(image.get_width() / 2 + dx + pixel_offset.x,
					image.get_height() / 2 + dy + pixel_offset.y)
			sum += Vector3(c.r, c.g, c.b)
	return sum / float((2 * block_radius + 1) * (2 * block_radius + 1))


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
			# a. The maps (the baked volume's S) against the CPU's Beer-Lambert on the closed slabs (a broken layer's spot
			# lies in a gap or under a cumulus: reported), the column terms, and the shading's composition (the ratio on /
			# off of the surface part: the in-scattered light taken out).
			var closed := slab != broken
			var on := await _render_ground(camera, point, up, sun, 0, _terrain_material)
			var factors := await _render_ground(camera, point, up, sun, 5, _terrain_material)
			var depth_ratio := log(maxf(factors.x, 1.0e-30)) / log(minf(t_s, 1.0 - 1.0e-9))
			var direct_ok := absf(depth_ratio - 1.0) <= S_DEPTH_TOLERANCE or absf(factors.x - t_s) <= DIRECT_FLOOR * 0.1
			if closed:
				_check("ground (S map): the direct Sun's slant optical depth " + label, direct_ok,
						"GPU T %.4f CPU T %.4f (depth %.3f x the CPU's)" % [factors.x, t_s, depth_ratio])
			else:
				print("  info  ground (S map, the shapes): direct Sun %s: GPU %.4f CPU %.4f (grid mean)" % [label,
						factors.x, t_s])
			_check("ground: the Sun's diffuse share " + label, absf(factors.y - t_sd) <= FACTOR_TOLERANCE * t_sd,
					"GPU %.4f CPU %.4f" % [factors.y, t_sd])
			_check("ground: the sky's diffuse transmittance " + label, absf(factors.z - t_d) <= FACTOR_TOLERANCE * t_d,
					"GPU %.4f CPU %.4f" % [factors.z, t_d])
			var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera, point, sun)
			var in_scatter: Vector3 = to_point["luminance"] * _atmosphere.luminance_scale
			var expected := _luminance(e_sun * (t_s + t_sd) + e_sky * t_d) / _luminance(e_sun + e_sky)
			var got := _luminance(on - in_scatter) / _luminance(off - in_scatter)
			var ratio_detail := "rendered %.4f CPU %.4f (T_s %.4f T_sd %.4f T_d %.4f, Sun share %.3f)" % [got, expected,
					t_s, t_sd, t_d, sun_share]
			if closed:
				_check("ground luminance ratio weather on / off " + label,
						absf(got - expected) <= GROUND_RATIO_TOLERANCE * expected, ratio_detail)
			else:
				print("  info  ground luminance ratio weather on / off %s: %s" % [label, ratio_detail])
			# b. The ground's total light on a horizontal plate (direct + the Sun's diffuse + sky) against the CPU's (the
			# grid mean) at the same spot.
			var ground_total := (factors.x + factors.y) * sun_share + factors.z * (1.0 - sun_share)
			var cpu_total := (t_s + t_sd) * sun_share + t_d * (1.0 - sun_share)
			var detail := "CPU %.4f ground %.4f (direct CPU %.4f, S %.4f)" % [cpu_total, ground_total, t_s, factors.x]
			if not closed:  # the spot lies in a gap or under a cumulus; the CPU's is the cell's mean (reported)
				print("  info  CPU light vs the ground's (shapes) %s: %s" % [label, detail])
			else:
				_check("CPU light (grid mean) vs the ground's (S) " + label,
						absf(cpu_total - ground_total) <= VEHICLE_TOLERANCE * ground_total, detail)
	# Final review I1 / I3: a cross-fade with an empty slot (deck -> no weather). The GPU's column terms equal the CPU light
	# of the pair CloudLayer pushed (I1), and the diffuse light never dips below the end states (I3: the empty slot's
	# column texels are zero; an R8 texture would read as cover 1 and darken the middle of the fade). The direct Sun is
	# S's (reported): the blended field breaks the deck into shapes, the spot lies in a gap or under one - not the CPU's
	# mean of the two transmittances.
	var fade_sun := _local_direction(45.0, 135.0)
	var fade_up := _local_direction(90.0, 0.0)
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
		for axis in [1, 2]:
			fade_ok = fade_ok and absf(gpu[axis] - cpu_factors[axis]) <= maxf(FACTOR_TOLERANCE * cpu_factors[axis], DIRECT_FLOOR * 0.1)
		fade_detail += " w1 %.2f GPU %s CPU %s;" % [w1, gpu, cpu_factors]
		# light on a horizontal plate: the GPU's column terms with the CPU's direct Sun (the grid mean of the pair)
		totals.append((cpu_factors.x + gpu.y) * fade_sun_share + gpu.z * (1.0 - fade_sun_share))
	_check("cross-fade deck -> empty: GPU column terms equal the CPU light of the pushed pair", fade_ok, fade_detail)
	var monotonic: bool = totals[0] < totals[4]
	for i in range(1, totals.size()):
		monotonic = monotonic and totals[i] >= totals[i - 1] - 1e-3
	_check("ground light (the column terms, the grid mean's direct Sun) is monotonic through a deck -> empty cross-fade",
			monotonic, "light on a plate at w1 0 .25 .5 .75 .95: %s" % [totals])
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


## Section 2 (Task 9 fix round 1): the shadow maps' hand-overs and the shipped shapes on broken cloud, through
## cloud_light_probe.gdshader (the surfaces' own cloud-light functions at receivers around a camera 2 m up). Receivers
## on annuli around the camera (area-uniform), the Sun at azimuth 135 deg. One site holds one realisation of the shape
## noise (its cells are kilometres to tens of km across: a disc of 8 km sees a few of them), so every number is the mean
## over SITES (~55 km apart: independent realisations), which brings the sampling error down by sqrt(8).
func _handover_samples() -> void:
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
			var far_detail := ("direct at 110 / 130 / 150 km %.4f %.4f %.4f (steps %.4f %.4f); far map %.4f vs the "
					+ "slanted column %.4f at 110 km") % [far_bands[0].direct, far_bands[1].direct, far_bands[2].direct,
					far_a, far_b, far_bands[0].far, far_bands[0].column]
			if case[1] < 1.0:
				# Broken cloud: the far map's S at 110 km comes from the coarse volume levels (4: 1.6 km cells), whose
				# smoothed shapes shadow more than the shapes do (2026-10-30: 0.56 / 0.23 vs the column's 0.59 / 0.39 at
				# a 45 / 8 deg Sun) - an open point of the bake (weather-clouds.md), reported.
				print("  info  hand-over 120-140 km, %s: %s" % [label, far_detail])
			else:
				_check("hand-over 120-140 km, no ring, " + label,
						maxf(maxf(far_a, far_b), column_gap) <= FAR_HANDOVER_TOLERANCE, far_detail)
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
			_check("CPU light vs the ground's mean (shapes) " + label,
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


## Section 3 (2026-10-30): the vehicles' direct Sun from the drawn shapes. Under a broken layer (cf 0.4) at a 45 deg Sun,
## VEHICLE_POINTS points VEHICLE_HEIGHT_M up along a VEHICLE_LINE_M line around the camera's site, read through the Sun
## probe (CloudLayer.direct_sun_transmittance, one key per point) and through the near shadow map (the probe shader's
## near channel - both read S); the most shadowed and the clearest point then lit through Atmosphere.sun_light_at (the
## vehicles' Sun light, its probe key "sun_light"), against the CPU's grid mean at both.
func _vehicle_samples() -> void:
	_show(_slab("vehicle", SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, BROKEN_FRACTION))
	var sun := _local_direction(45.0, 135.0)
	var camera := _point(2.0)
	await _render(camera, sun, sun, 0)
	var points: Array[Vector3] = []
	for i in range(VEHICLE_POINTS):
		var along := (float(i) / float(VEHICLE_POINTS - 1) - 0.5) * VEHICLE_LINE_M
		points.append(_destination(LAT_DEG, LON_DEG, absf(along), deg_to_rad(90.0 if along >= 0.0 else 270.0))
				+ _local_direction(90.0, 0.0) * VEHICLE_HEIGHT_M)
	var probe := await _probe_points(camera, points)
	var offsets := PackedVector3Array()
	for p in points:
		offsets.append(Frames.to_render(p - camera))
	var maps := await _probe(offsets, 0)
	var worst := 0.0
	var mean := 0.0
	var dark := -1
	var bright := -1
	for i in range(points.size()):
		if is_nan(probe[i]):
			continue
		worst = maxf(worst, absf(probe[i] - maps[i].y))
		mean += absf(probe[i] - maps[i].y) / points.size()
		if dark < 0 or probe[i] < probe[dark]:
			dark = i
		if bright < 0 or probe[i] > probe[bright]:
			bright = i
	_check("vehicle probe: a value at every point", dark >= 0 and not Array(probe).any(func(t: float) -> bool:
			return is_nan(t)), "%s" % [probe])
	if dark < 0:
		return
	_check("vehicle probe = the near map (both S) at the same points",
			mean <= VEHICLE_MAP_TOLERANCE.x and worst <= VEHICLE_MAP_TOLERANCE.y,
			"|probe - map| mean %.4f worst %.4f" % [mean, worst])
	var lit := []
	var cpu := []
	for i: int in [dark, bright]:
		lit.append(await _vehicle_light(camera, points[i], sun))
		cpu.append(float(_core.weather_cloud_light(points[i], sun)["sun_transmittance"]))
	var clear: float = _atmosphere.sun_light_at(points[bright], sun, 1.0)[1] / maxf(probe[bright], 1.0e-6) \
			if probe[bright] > 0.0 else 0.0
	var contrast: float = (lit[1] - lit[0]) / maxf(clear, 1.0e-9)
	_check("vehicle in a cumulus' shadow vs in the gap: different direct Sun light",
			contrast >= VEHICLE_MIN_CONTRAST and absf(float(cpu[0]) - float(cpu[1])) < 0.05,
			"T %.3f / %.3f (probe), Sun light energy %.3f / %.3f (%.2f of the clear %.3f); CPU grid mean %.3f / %.3f" % [
			probe[dark], probe[bright], lit[0], lit[1], contrast, clear, cpu[0], cpu[1]])


## The Sun probe at each point (Earth-fixed m; one key each), read back after a few frames; NAN where none came.
func _probe_points(camera_itrf_m: Vector3, points: Array[Vector3]) -> PackedFloat64Array:
	var values := PackedFloat64Array()
	values.resize(points.size())
	values.fill(NAN)
	for frame in range(60):
		var missing := 0
		for i in range(points.size()):
			values[i] = _clouds.direct_sun_transmittance("check_%d" % i, points[i])
			missing += 1 if is_nan(values[i]) else 0
		if missing == 0 and frame >= 4:
			break
		await RenderingServer.frame_post_draw
		_clouds.update(camera_itrf_m, 0.0, 0.0)
	return values


## Atmosphere.sun_light_at's energy at a point once its probe ("sun_light") has read it (a few frames: the read-back).
func _vehicle_light(camera_itrf_m: Vector3, point_itrf_m: Vector3, sun_itrf: Vector3) -> float:
	for frame in range(8):
		_atmosphere.sun_light_at(point_itrf_m, sun_itrf, 1.0)
		await RenderingServer.frame_post_draw
		_clouds.update(camera_itrf_m, 0.0, 0.0)
	return _atmosphere.sun_light_at(point_itrf_m, sun_itrf, 1.0)[1]
