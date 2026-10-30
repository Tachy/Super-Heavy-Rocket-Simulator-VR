# Headless smoke test: the GDExtension loads in the double-precision engine and delivers float64 data.
# Run: godot --headless --path game --script res://tests/smoke_test.gd
extends SceneTree

const KERNEL_DIR := "../data/spice"
# Horizons (DE441) geocentric Moon at JD 2451545.0 TDB, km (tests/validation/fixtures).
const MOON_J2000_REF_M := Vector3(-291608.3841877129, -266716.8338540655, -76102.48730658794) * 1000.0

var _failures := 0


func _initialize() -> void:
	_check(OS.has_feature("double"), "engine is a double-precision build")

	var core := StarsimCore.new()
	var kernel_dir := ProjectSettings.globalize_path("res://").path_join(KERNEL_DIR).simplify_path()
	_check(core.open(kernel_dir), "SPICE kernels loaded from %s" % kernel_dir)

	var j2000 := core.utc_to_tdb_s("2000-01-01T11:58:55.816")
	_check(absf(j2000) < 2e-3, "UTC -> TDB at J2000 (%.6f s)" % j2000)

	var moon := core.body_position_gcrf_m("moon", 0.0)
	var moon_error := moon.distance_to(MOON_J2000_REF_M)
	_check(moon_error < 3.5, "Moon at J2000 vs Horizons: %.3f m" % moon_error)

	# float64 end to end: a 1 mm height step at Starbase must be visible in GCRF coordinates.
	var tdb := core.utc_to_tdb_s("2026-10-12T12:00:00")
	var pad := core.geodetic_to_gcrf_m(25.99686, -97.15394, 10.0, tdb)
	var pad_up := core.geodetic_to_gcrf_m(25.99686, -97.15394, 10.001, tdb)
	var step := pad.distance_to(pad_up)
	_check(absf(step - 0.001) < 1e-7, "1 mm step at Starbase resolved as %.9f m" % step)
	print("Starbase GCRF at 2026-10-12T12:00:00 UTC: ", pad)
	print("Moon distance now: %.3f km" % (core.body_position_gcrf_m("moon", tdb).length() / 1000.0))

	# Catch tower geometry comes from the core's config (one source of truth for physics and drawing).
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	var tower_sim := FlightSim.new()
	tower_sim.setup(config_dir)
	var tower: Dictionary = tower_sim.site_tower("starbase_olp1")
	_check(tower.get("present", false), "Starbase has a catch tower")

	# Terrain through the shared cache (M8 Plan B): StarsimCore samples what FlightSim's physics uses.
	var terrain_present: Dictionary = core.load_terrain(config_dir)
	_check(terrain_present.has("earth") and terrain_present.has("moon"), "load_terrain returns {earth, moon}")
	# The free camera's height over sea level: 1000 m above the ellipsoid at 0 N 80 E (the geoid's low, N ~ -100 m).
	var above_low := core.sea_level_height_m(core.geodetic_to_gcrf_m(0.0, 80.0, 1000.0, 0.0), 0.0)
	_check(above_low > 1050.0 and above_low < 1150.0, "sea_level_height_m applies the geoid (%.1f m)" % above_low)
	_check(TimePanel.camera_height_line(2500.0, false) == "Kamerahöhe: 2500 m über NN"
			and TimePanel.camera_height_line(1.2345e5, true) == "Kamerahöhe: 123.5 km über mittlerem Mondradius"
			and TimePanel.camera_height_line(NAN, false) == "", "TimePanel.camera_height_line formats the height")
	var starbase := _site(config_dir, "starbase_olp1")
	var lat_arr := PackedFloat64Array([float(starbase.get("latitude_deg", 0.0))])
	var lon_arr := PackedFloat64Array([float(starbase.get("longitude_deg", 0.0))])
	var earth_samples: Dictionary = core.surface_samples("earth", lat_arr, lon_arr)
	if terrain_present.get("earth", false):
		var phys: Dictionary = tower_sim.earth_surface(lat_arr[0], lon_arr[0])
		_check(earth_samples.get("present", false) and (earth_samples["height_m"] as PackedFloat64Array).size() == 1
				and absf((earth_samples["height_m"] as PackedFloat64Array)[0] - float(phys.get("height_m", INF))) < 1e-9
				and (earth_samples["water"] as PackedByteArray)[0] == (1 if phys.get("water", false) else 0),
				"surface_samples at the Starbase pad equals FlightSim.earth_surface")
		_check(is_finite(core.surface_resolution_m("earth", lat_arr[0], lon_arr[0]))
				and core.surface_resolution_m("earth", lat_arr[0], lon_arr[0]) <= 1.5,
				"surface_resolution_m: the 1 m local grid at the pad")
	else:
		_check(not earth_samples.get("present", true) and (earth_samples["height_m"] as PackedFloat64Array).is_empty()
				and is_inf(core.surface_resolution_m("earth", 0.0, 0.0)), "no Earth terrain: surface_samples not present")
	if not terrain_present.get("moon", false):
		_check(not core.surface_samples("moon", lat_arr, lon_arr).get("present", true), "no Moon terrain: not present")
	# Terrain horizon bindings: an n x n lattice with 16 azimuths, a single march; none without the terrain.
	var h_lat := PackedFloat64Array()
	var h_lon := PackedFloat64Array()
	for i in 17:
		for j in 17:
			h_lat.append(-84.79 + (i - 8) * 1e-4)
			h_lon.append(29.19 + (j - 8) * 1e-4)
	var h_heights := PackedFloat64Array()
	h_heights.resize(17 * 17)
	if terrain_present.get("moon", false):
		var horizons: PackedFloat32Array = core.horizon_elevations("moon", h_lat, h_lon, h_heights, 16, 3.0)
		_check(horizons.size() == 17 * 17 * 16 and is_finite(horizons[0]),
				"horizon_elevations: 17 x 17 x 16 values (%d)" % horizons.size())
		_check(is_finite(core.horizon_elevation_rad("moon", -84.79, 29.19, 3000.0, 1.0)),
				"horizon_elevation_rad finite at the IM-2 site")
	_check(core.horizon_elevations("earth", h_lat, h_lon, h_heights, 16, 3.0).is_empty()
			and is_nan(core.horizon_elevation_rad("earth", 0.0, 0.0, 0.0, 0.0)), "no horizon for Earth")
	_check(absf(float(tower.get("mount_height_m", 0.0)) - 20.0) < 1e-9, "tower geometry from the core config")
	_check(_check_terrain_streaming(core, starbase, bool(terrain_present.get("earth", false))) == true,
			"terrain streaming checks ran to the end")
	_check(_check_atmosphere(core) == true, "atmosphere checks ran to the end")
	_check(_check_weather(core) == true, "weather checks ran to the end")

	# A script error aborts a check function (it then returns null) without failing a check: count it as a failure.
	_check(_check_falcon9(config_dir, tdb) == true, "Falcon 9 checks ran to the end")
	_check(_check_dragon(config_dir, tdb) == true, "Crew Dragon checks ran to the end")
	_check(_check_thread(config_dir, tdb) == true, "simulation thread checks ran to the end")
	_check(_check_lunar(config_dir, tdb) == true, "lunar mission checks ran to the end")
	_check(_check_iss(config_dir, tdb) == true, "ISS mission checks ran to the end")
	_check(_check_refuelling(config_dir, tdb) == true, "refuelling checks ran to the end")

	print("SMOKE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1


## Waits for the simulation thread; true when command `id` finished ok (the former bool result).
func _ok(sim: FlightSim, id: int) -> bool:
	return id > 0 and sim.wait_idle(120.0) and bool(sim.result(id).get("ok", false))


## Advances to `tdb_s` and waits until the simulation thread has reached it.
## The [[site]] entry of config/launch_sites.toml with this id (empty if missing).
func _site(config_dir: String, id: String) -> Dictionary:
	for site: Dictionary in TomlLite.load_file(config_dir.path_join("launch_sites.toml")).get("site", []):
		if site.get("id", "") == id:
			return site
	return {}


func _advance(sim: FlightSim, tdb_s: float) -> void:
	sim.advance_to(tdb_s)
	sim.wait_idle(60.0)


# Terrain renderer (M8 Plan B): the quadtree builds nodes on WorkerThreadPool threads through the core's surface
# samples (after load_terrain) and refines under a camera 2 m above OLP-1.
func _check_terrain_streaming(core: StarsimCore, starbase: Dictionary, with_data: bool) -> bool:
	var heights := HeightSource.new()
	heights.core = core
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain_earth.gdshader")
	var tree := TerrainQuadtree.new()
	root.add_child(tree)
	tree.setup(heights, null, 6378137.0, material)
	var camera := HeightSource.geodetic_to_fixed(starbase["latitude_deg"], starbase["longitude_deg"],
			float(starbase["height_m"]) + 2.0)
	var t0 := Time.get_ticks_msec()
	var frames := 0
	var level := 0
	while frames < 20000 and Time.get_ticks_msec() - t0 < 120000:
		tree.update_for_camera(camera)
		frames += 1
		if tree.stats["nodes_loading"] == 0 and tree.stats["uploads_last_frame"] == 0:
			break
		OS.delay_msec(1)
	for key: Vector4i in tree.drawn_keys():
		if key == CubeSphere.key_at(CubeSphere.geodetic_direction(starbase["latitude_deg"], starbase["longitude_deg"]),
				key.y):
			level = key.y
	_check(tree.stats["nodes_loading"] == 0 and level >= (17 if with_data else 8),
			"terrain streamed on worker threads: leaf level %d at OLP-1, %d leaves, %d frames, %d ms"
			% [level, tree.drawn_keys().size(), frames, Time.get_ticks_msec() - t0])
	tree.free()
	return true


# Atmosphere (M8): the precomputed tables and the CPU lookups. Uses the game's cache folder (data/atmosphere); run
# build/core/atmosphere_tables.exe once beforehand (a cache miss would precompute for ~1 min).
func _check_atmosphere(core: StarsimCore) -> bool:
	var empty_sky: Dictionary = core.atmosphere_sky_luminance(Vector3(6378147.0, 0, 0), Vector3(1, 0, 0), Vector3(0.5, 0.866, 0))
	_check(empty_sky["luminance"] == Vector3.ZERO and core.atmosphere_sun_transmittance(Vector3(6378147.0, 0, 0),
			Vector3(1, 0, 0)) == Vector3.ZERO, "atmosphere lookups before load return zeros")
	var t0 := Time.get_ticks_msec()
	var loaded: Dictionary = core.load_atmosphere(Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	_check(loaded.get("ok", false), "load_atmosphere ok (%d ms) %s" % [Time.get_ticks_msec() - t0, loaded.get("error", "")])
	if not loaded.get("ok", false):
		return true
	var size: Vector3i = loaded["scattering_size"]
	var tsize: Vector2i = loaded["transmittance_size"]
	_check((loaded["scattering"] as PackedFloat32Array).size() == 4 * size.x * size.y * size.z
			and (loaded["single_mie"] as PackedFloat32Array).size() == 4 * size.x * size.y * size.z
			and (loaded["single_rayleigh"] as PackedFloat32Array).size() == 4 * size.x * size.y * size.z
			and (loaded["transmittance"] as PackedFloat32Array).size() == 4 * tsize.x * tsize.y,
			"atmosphere table sizes %s, transmittance %s" % [size, tsize])
	var params: Dictionary = loaded["parameters"]
	_check(absf(float(params["bottom_radius_m"]) - (6378137.0 - 150.0)) < 1.0, "atmosphere parameters: bottom radius")
	# Pad on the equator (ITRF x axis), looking at the zenith with the Sun 60 deg high.
	var pad := Vector3(6378137.0 + 10.0, 0, 0)
	var sun := Vector3(sin(deg_to_rad(60.0)), cos(deg_to_rad(60.0)), 0)
	var sky: Dictionary = core.atmosphere_sky_luminance(pad, Vector3(1, 0, 0), sun)
	var green: float = (sky["luminance"] as Vector3).y
	_check(green > 2000.0 and green < 10000.0, "zenith sky luminance, Sun 60 deg up: %.0f cd/m^2 (green)" % green)
	var lux: Dictionary = core.atmosphere_illuminance(pad, Vector3(1, 0, 0), sun)
	_check((lux["sun_lux"] as Vector3).y > 1e4 and (lux["sky_lux"] as Vector3).y > 1e3,
			"illuminance: sun %s, sky %s" % [lux["sun_lux"], lux["sky_lux"]])
	_check((core.atmosphere_sun_transmittance(pad, sun) as Vector3).y > 0.3, "sun transmittance at 60 deg elevation")
	return true


# Weather (M8): source selection, snapshot files, the CPU cloud light and the noise textures.
func _check_weather(core: StarsimCore) -> bool:
	var zattrs := '{"valid_time_stop": "2025-12-31", "valid_time_stop_era5t": "2026-10-20"}'
	var hour: float = core.weather_utc_hour(core.utc_to_tdb_s("2024-04-08T18:30:00"))
	var bracket: Dictionary = core.weather_bracket(hour, 1, zattrs, -1, 2025)
	_check(bracket.get("ok", false) and bracket["before"]["kind"] == "era5" and bracket["after"]["kind"] == "era5"
			and absf(float(bracket["blend"]) - 0.5) < 1e-6, "weather_bracket 2024-04-08 18:30: era5, blend 0.5")
	_check(int(bracket["after"]["valid_hour"]) - int(bracket["before"]["valid_hour"]) == 1
			and String(bracket["before"]["cache_key"]).begins_with("era5_"), "weather_bracket: hourly ids with cache keys")
	var offline: Dictionary = core.weather_bracket(hour + 24.0 * 365 * 5, 1, "", -1, 2025)
	_check(offline.get("ok", false) and offline["before"]["kind"] == "analog", "weather_bracket offline: analog")
	var requests: Array = core.weather_requests(bracket["before"], "")
	_check(requests.size() == 10 and String(requests[0]["url"]).begins_with("https://"), "weather_requests: ten ERA5 files")
	_check(core.weather_gfs_cycle_candidates(hour + 24.0 * 365 * 5, 3).size() == 3, "weather_gfs_cycle_candidates")
	_check(core.weather_gfs_cycle_candidates(hour, -1).is_empty(), "weather_gfs_cycle_candidates: negative count returns empty")
	_check(core.weather_snapshot_step(1.0, 10.0) == 1 and core.weather_snapshot_step(1e6, 100.0) == 0, "weather_snapshot_step")
	_check(core.weather_cloud_light(Vector3(6378137.0, 0, 0), Vector3(1, 0, 0)) == {"sun_transmittance": 1.0, "sky_factor": 1.0,
			"sun_diffuse": 0.0},
			"weather_cloud_light without weather: 1, 1, 0")
	var bad: Dictionary = core.weather_load_snapshot("no_such_dir/none.wthr")
	_check(not bad.get("ok", true) and not String(bad.get("error", "")).is_empty(), "weather_load_snapshot of a missing file fails")
	var file := OS.get_temp_dir().path_join("starsim_smoke_slab_%d.wthr" % OS.get_process_id())
	_check(core.weather_write_slab_snapshot(file, 2000.0, 3000.0, 3e-4, 1.0), "weather_write_slab_snapshot")
	var loaded: Dictionary = core.weather_load_snapshot(file)
	_check(loaded.get("ok", false) and (loaded["cloud_texels"] as PackedByteArray).size()
			== int(loaded["rows"]) * int(loaded["columns"]) * int(loaded["layers"]) * 4
			and (loaded["layer_bottoms_m"] as PackedFloat64Array).size() == int(loaded["layers"]) + 1,
			"weather_load_snapshot: cloud_texels %d x %d x %d" % [loaded.get("rows", 0), loaded.get("columns", 0), loaded.get("layers", 0)])
	_check(core.weather_set_active(file, file, 0.5).get("ok", false), "weather_set_active")
	var pos := Vector3(6378137.0, 0, 0)
	var under: Dictionary = core.weather_cloud_light(pos, Vector3(1, 0, 0))
	_check(float(under["sun_transmittance"]) < 0.5 and float(under["sky_factor"]) < 1.0,
			"weather_cloud_light under a slab: sun %.3f, sky %.3f" % [under["sun_transmittance"], under["sky_factor"]])
	var above: Dictionary = core.weather_cloud_light(Vector3(6378137.0 + 5000.0, 0, 0), Vector3(1, 0, 0))
	_check(absf(float(above["sun_transmittance"]) - 1.0) < 1e-9 and absf(float(above["sky_factor"]) - 1.0) < 1e-9,
			"weather_cloud_light above the slab: 1, 1")
	_check(loaded.get("file", "") == file, "weather_load_snapshot: names the file")
	# The cell bake reference (world volume Task 4): a rewritten file is baked anew, not from the kept render field.
	var bake_file := OS.get_temp_dir().path_join("starsim_smoke_bake_%d.wthr" % OS.get_process_id())
	var centre := Vector3(0.0, 0.0, 6378137.0 + 2500.0)  # render axes: 0 N 0 E, in the 2-3 km slab
	var anchor := PackedFloat64Array([0.0, 0.0, 6378137.0])
	core.weather_write_slab_snapshot(bake_file, 2000.0, 3000.0, 3e-4, 1.0)
	var in_slab: Vector2 = core.weather_cloud_cell_bake(bake_file, centre, Vector2(200.0, 164.0625), anchor)
	core.weather_write_slab_snapshot(bake_file, 2000.0, 3000.0, 0.0, 0.0)
	var rewritten: Vector2 = core.weather_cloud_cell_bake(bake_file, centre, Vector2(200.0, 164.0625), anchor)
	_check(in_slab.x > 0.0 and in_slab.x <= in_slab.y and rewritten == Vector2.ZERO,
			"weather_cloud_cell_bake: slab %s, the same file rewritten clear %s" % [in_slab, rewritten])
	DirAccess.remove_absolute(bake_file)
	# Final review I1 / I2: the pair is cache-only, a missing side blends toward the empty snapshot, hits become the newest.
	var clear_file := OS.get_temp_dir().path_join("starsim_smoke_clear_%d.wthr" % OS.get_process_id())
	_check(core.weather_write_slab_snapshot(clear_file, 2000.0, 3000.0, 0.0, 0.0), "weather_write_slab_snapshot: empty field")
	_check(core.weather_load_snapshot(clear_file).get("ok", false), "weather_load_snapshot: empty field")
	core.weather_set_active(file, "", 0.5)
	var half_a: Dictionary = core.weather_cloud_light(pos, Vector3(1, 0, 0))
	core.weather_set_active("", file, 0.5)
	var half_b: Dictionary = core.weather_cloud_light(pos, Vector3(1, 0, 0))
	core.weather_set_active(clear_file, file, 0.5)
	var half_c: Dictionary = core.weather_cloud_light(pos, Vector3(1, 0, 0))
	core.weather_set_active(file, file, 1.0)
	var full: Dictionary = core.weather_cloud_light(pos, Vector3(1, 0, 0))
	_check(half_a == half_b and half_a == half_c, "a missing side is the empty snapshot: %s" % half_a)
	_check(float(half_a["sun_transmittance"]) > float(full["sun_transmittance"])
			and float(half_a["sun_transmittance"]) < 1.0, "half weight: between clear and the slab (%.3f)" % half_a["sun_transmittance"])
	core.weather_set_active(file, file, 0.5)
	var before_miss: Dictionary = core.weather_cloud_light(pos, Vector3(1, 0, 0))
	var miss: Dictionary = core.weather_set_active(file, "no_such_dir/never_loaded.wthr", 0.1)
	_check(not miss.get("ok", true) and "nicht geladen" in String(miss.get("error", "")), "weather_set_active of an unloaded file fails without reading")
	_check(core.weather_cloud_light(pos, Vector3(1, 0, 0)) == before_miss, "a failed weather_set_active keeps the active pair")
	var spares: Array = []
	for i in range(3):
		var spare := OS.get_temp_dir().path_join("starsim_smoke_spare%d_%d.wthr" % [i, OS.get_process_id()])
		core.weather_write_slab_snapshot(spare, 1000.0, 2000.0, 1e-4, 0.5)
		spares.append(spare)
	core.weather_load_snapshot(spares[0])
	core.weather_load_snapshot(spares[1])  # cache now: [.., spare0, spare1]
	core.weather_set_active(spares[0], spares[0], 0.0)  # a hit becomes the newest
	core.weather_load_snapshot(spares[2])
	var cached: PackedStringArray = core.weather_cached_files()
	# Loaded: clear, spare0, spare1 (file went first); the hit moved spare0 behind spare1, spare2 evicted clear.
	_check(Array(cached) == [spares[1], spares[0], spares[2]],
			"the load cache keeps hits newest and evicts the least recently used file: %s" % [cached])
	core.weather_set_active("", "", 0.0)
	DirAccess.remove_absolute(file)
	for path: String in spares + [clear_file]:
		DirAccess.remove_absolute(path)
	var noise: Dictionary = core.cloud_noise()
	var n: int = int(noise["shape_size"])
	_check(n == 128 and (noise["shape"] as PackedByteArray).size() == 4 * n * n * n
			and (noise["detail"] as PackedByteArray).size() == 4 * int(noise["detail_size"]) ** 3
			and (noise["coverage_threshold"] as PackedFloat32Array).size() == 256 * int(noise["detail_states"])
			and (noise["edge_mean_factor"] as PackedFloat32Array).size() == 256 * int(noise["detail_states"])
			and (noise["level_amplitude"] as PackedFloat32Array).size() == int(noise["detail_states"]) - 1,
			"cloud_noise sizes")
	# The shape prefilter (Task 10 fix round 1): the mips and tables, and the constants the shaders mirror
	# (clouds.gdshaderinc CLOUD_SHAPE_PREFILTER_*: 2 levels, 4 rows per level).
	var mips: Array = noise["shape_mips"]
	var rows: int = int(noise["prefilter_rows"])
	_check(mips.size() == 7 and (mips[0] as PackedByteArray).size() == 4 * (n / 2) ** 3
			and (mips[6] as PackedByteArray).size() == 4 and rows == 9 and int(noise["prefilter_levels"]) == 2
			and int(noise["prefilter_rows_per_level"]) == 4
			and (noise["prefilter_threshold"] as PackedFloat32Array).size() == 256 * rows
			and (noise["prefilter_edge_mean"] as PackedFloat32Array).size() == 256 * rows,
			"cloud_noise shape prefilter sizes and constants")
	# The shader's mirrored copies of the same constants (text of clouds.gdshaderinc: a typo there is invisible to the
	# core tests).
	_check(_shader_constant("CLOUD_SHAPE_PREFILTER_LEVELS") == float(noise["prefilter_levels"])
			and _shader_constant("CLOUD_SHAPE_PREFILTER_ROWS_PER_LEVEL") == float(noise["prefilter_rows_per_level"])
			and _shader_constant("CLOUD_SHAPE_PREFILTER_ROWS") == float(rows),
			"clouds.gdshaderinc prefilter constants equal the core's")
	# The cloud types (static clouds Task 5 fix round 1): the shader's band constants and CloudLayer's tiles.
	var types: Dictionary = noise["type_constants"]
	var types_ok := types.size() == 7 and (noise["level_lift"] as PackedFloat32Array).size() == 3
	for constant: String in types:
		types_ok = types_ok and is_equal_approx(_shader_constant(constant), float(types[constant]))
	_check(types_ok and float(types["CLOUD_ICE_TILE_V_M"]) == CloudLayer.ICE_TILE_V_M
			and float(types["CLOUD_STRATIFORM_STRETCH"]) == CloudLayer.STRATIFORM_STRETCH,
			"clouds.gdshaderinc / CloudLayer cloud type constants equal the core's")
	var field: Dictionary = noise["field_constants"]
	var field_ok := field.size() == 5
	for constant: String in field:
		field_ok = field_ok and is_equal_approx(_shader_constant(constant), float(field[constant]))
	_check(field_ok, "clouds.gdshaderinc cloud field constants (LCL ramp, base transition, overlap) equal the core's")
	var optics: Dictionary = core.cloud_optics_parameters()
	_check(absf(float(optics.get("asymmetry_g", 0.0)) - 0.8793) < 1e-3 and float(optics["extinction_step_per_m"]) > 0.0,
			"cloud_optics_parameters (g %.4f)" % float(optics.get("asymmetry_g", 0.0)))
	_check(core.cloud_slab_radiance(10.0, 1.0, 1.0, 0.0) > 0.0 and core.cloud_slab_radiance(10.0, -0.1, 1.0, 0.0) == 0.0,
			"cloud_slab_radiance")
	return true


# Simulation thread (l6): commands by id, results, idle wait, re-setup while busy.
func _check_thread(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	var id: int = sim.start_mission("starship_leo", tdb)
	_check(id > 0, "start_mission returns a command id")
	_check(sim.wait_idle(60.0), "the simulation thread becomes idle")
	var r: Dictionary = sim.result(id)
	_check(r.get("done", false) and r.get("ok", false), "start_mission finished ok")
	var reached := sim.advance_to(tdb + 2.0)
	_check(sim.wait_idle(30.0), "the simulation thread reaches the target")
	# 400 tick additions at ~8.5e8 s TDB round to a few microseconds
	var gap := (tdb + 2.0) - sim.advance_to(tdb + 2.0)
	_check(absf(gap) < 1e-4, "advance reaches the target (gap %s s)" % str(gap))
	_check(reached <= tdb + 2.0, "advance_to returns the snapshot's epoch, never ahead")
	var bad: int = sim.start_mission("no_such_mission", tdb)
	sim.wait_idle(10.0)
	_check(not sim.result(bad).get("ok", true) and str(sim.result(bad).get("message", "")) != "",
			"unknown mission: refused with a message")
	# External view: mission timeline, body serials, tank levels, countdown start.
	_advance(sim, tdb + 200.0)
	var timeline: Dictionary = sim.mission_timeline()
	var kinds: Array = []
	for e in timeline.get("events", []):
		kinds.append(e["kind"])
	_check(kinds.has("liftoff"), "mission timeline contains the liftoff (%s)" % str(kinds))
	_check(sim.body_serial(0) >= 1, "body serial is stable and positive")
	_check((sim.body_engines(0)[0] as Dictionary).has("stage"), "engines carry their stage index")
	_check(float((sim.body_stages(0)[0] as Dictionary).get("oxidizer_capacity_kg", 0.0)) > 0.0, "stages carry tank capacities")
	_check(sim.mission_display_name() == "Testflug in den Erdorbit", "mission display name (%s)" % sim.mission_display_name())
	_check(sim.body_display_name(0) != "" and not sim.body_display_name(0).contains("Starship"),
			"body display name without trademark (%s)" % sim.body_display_name(0))
	_check(float(sim.telemetry(0).get("propellant_capacity_kg", 0.0)) >= float(sim.telemetry(0).get("propellant_kg", 1.0)),
			"telemetry carries the propellant load's capacity")
	_check((sim.body_engines(0)[0] as Dictionary).has("auxiliary"), "engines carry the auxiliary flag")
	# a fresh sim: the old one still has its advance target
	var countdown_sim := FlightSim.new()
	countdown_sim.setup(config_dir)
	_check(_ok(countdown_sim, countdown_sim.launch_from_site("starship_block3", "starbase_olp1", 0.0, tdb, 30.0)), "countdown launch starts")
	var clock = countdown_sim.mission_timeline().get("clock_s")
	_check(clock != null and absf(float(clock) + 30.0) < 1e-3, "countdown launch: T-30 clock at the start (%s)" % str(clock))
	for i in 10:  # past T-0 without the autopilot: nobody releases the clamps
		_advance(countdown_sim, tdb + 45.0)
	clock = countdown_sim.mission_timeline().get("clock_s")
	_check(clock != null and float(clock) == 0.0, "no liftoff past T-0: the clock holds at T-0 (%s)" % str(clock))
	# launch_from_pad starts a fresh mission log (the LEO mission above had lifted off)
	_check(_ok(sim, sim.launch_from_pad("starship_block3", 25.99686, -97.15394, 10.0, 90.0, tdb + 300.0)),
			"launch_from_pad on a flown session")
	var fresh_kinds: Array = []
	for e in sim.mission_timeline().get("events", []):
		fresh_kinds.append(e["kind"])
	_check(not fresh_kinds.has("liftoff"), "launch_from_pad: a fresh mission log (%s)" % str(fresh_kinds))
	sim.setup(config_dir)  # re-setup joins the old thread
	_check(not sim.is_active(), "re-setup starts empty")
	var empty: Dictionary = sim.mission_timeline()
	_check(empty.has("t0_tdb_s") and empty["t0_tdb_s"] == null and empty["clock_s"] == null
			and empty.get("events") is Array and (empty["events"] as Array).is_empty(),
			"no flight: the mission log has the full shape, empty (%s)" % str(empty))
	return true


# Falcon 9 missions (c5 game interface): mission start, landing zone with and without a deck, legs and grid fins.
func _check_falcon9(config_dir: String, tdb: float) -> bool:
	var f9 := FlightSim.new()
	f9.setup(config_dir)
	# the first launch planned in this process (no memo yet): the planning flag is published while start_mission plans,
	# although the session holds no flight yet, and cleared afterwards
	var rtls_id := f9.start_mission("falcon9_rtls", tdb)
	var planning_seen := false
	var wait_start := Time.get_ticks_msec()
	while not planning_seen and Time.get_ticks_msec() - wait_start < 5000 and not f9.result(rtls_id).get("done", false):
		planning_seen = f9.autopilot_status().get("planning", false)
		OS.delay_msec(5)
	_check(planning_seen, "start_mission of a pad launch publishes the planning flag while it plans")
	_check(_ok(f9, rtls_id), "Falcon 9 RTLS mission starts")
	_check(not f9.autopilot_status().get("planning", true), "the planning flag is cleared after start_mission")
	var zone: Dictionary = f9.landing_zone()
	_check(zone.get("present", false) and not zone.get("has_deck", true), "RTLS: landing zone without a deck")
	_check(absf(float(zone.get("pad_radius_m", 0.0)) - 42.7) < 1e-9, "RTLS: LZ-40 pad radius from the config")
	_check(f9.mission_site_id() == "ccsfs_slc40", "RTLS: launch site of the mission")
	_check(_ok(f9, f9.start_mission("falcon9_asds", tdb)), "Falcon 9 drone-ship mission starts on the same FlightSim")
	zone = f9.landing_zone()
	# The ship floats on the EGM2008 geoid at its station (M8 Plan A Task 5; was a fixed -35 m): the Earth terrain's
	# sea there when loaded, else the coarse geoid (-20 .. -50 m off the Carolinas).
	var ship_sea: Dictionary = f9.earth_surface(float(zone.get("latitude_deg", 0.0)), float(zone.get("longitude_deg", 0.0)))
	var sea_m := float(zone.get("sea_height_m", 0.0))
	var sea_ok := absf(sea_m - float(ship_sea.get("height_m", 0.0))) < 1e-6 if ship_sea.get("present", false) \
			else (sea_m < -20.0 and sea_m > -50.0)
	_check(zone.get("has_deck", false) and absf(float(zone.get("deck_length_m", 0.0)) - 91.0) < 1e-9 and sea_ok,
			"ASDS: the drone ship's deck from the config, on the geoid")
	if ship_sea.get("present", false):  # the Earth terrain (data/ fetched): the ship's sea is water, OLP-1 its plateau
		var olp_site := _site(config_dir, "starbase_olp1")  # the plateau height and position come from the config
		var olp := f9.earth_surface(float(olp_site.get("latitude_deg", 0.0)), float(olp_site.get("longitude_deg", 0.0)))
		_check(not olp_site.is_empty() and ship_sea.get("water", false) and not olp.get("water", true)
				and absf(float(olp.get("height_m", INF)) - float(olp_site.get("height_m", NAN))) < 1e-6,
				"Earth terrain: sea at the ship, pad plateau at OLP-1")
	else:
		print("  (Erdgelände fehlt: earth_surface not checked)")
	# Sea state and deck motion (Task 6): falcon9_asds is calm (no sea state, level deck); the hull from drone_ships.toml
	_check(not zone.has("sea_state") and float(zone.get("deck_heave_m", NAN)) == 0.0
			and float(zone.get("deck_roll_deg", NAN)) == 0.0 and float(zone.get("deck_pitch_deg", NAN)) == 0.0
			and absf(float(zone.get("hull_length_m", 0.0)) - 91.4) < 1e-9
			and absf(float(zone.get("hull_beam_m", 0.0)) - 30.5) < 1e-9
			and absf(float(zone.get("hull_draft_m", 0.0)) - 3.0) < 1e-9,
			"ASDS calm: no sea state, a level deck, the hull from drone_ships.toml")
	_check(_ok(f9, f9.start_mission("falcon9_asds_sea", tdb)), "Falcon 9 drone-ship mission in a sea starts")
	var sea_zone: Dictionary = f9.landing_zone()
	var sea: Dictionary = sea_zone.get("sea_state", {})
	_check(absf(float(sea.get("hs_m", 0.0)) - 2.5) < 1e-9 and absf(float(sea.get("tp_s", 0.0)) - 8.0) < 1e-9
			and absf(float(sea.get("heading_deg", 0.0)) - 60.0) < 1e-9, "ASDS sea: the mission's sea state Hs 2.5 m, Tp 8 s, 60 deg")
	_check(absf(float(sea_zone.get("heading_deg", 0.0)) - 60.0) < 1e-6, "ASDS sea: the deck turned along the waves (60 deg)")
	var readout := TimePanel.sea_state_line(sea_zone)
	_check(readout.begins_with("Seegang Hs 2.5 m, Tp 8 s - Deck: Hub ")
			and TimePanel.sea_state_line(zone) == "Seegang ruhig - Deck in Ruhe" and TimePanel.sea_state_line({}) == "",
			"time panel readout: '%s'" % readout)
	_check(TimePanel.frame_rate_line(45.0) == "Bildrate: 45 fps (22.2 ms)" and TimePanel.frame_rate_line(0.0) == "",
			"time panel frame rate line: '%s'" % TimePanel.frame_rate_line(45.0))
	var pose_seen := false  # the deck moves: some pose within 10 s differs from the level deck, all within the limits
	var pose_sane := true
	for k in range(5):
		_advance(f9, tdb + 2.0 * (k + 1))
		var z: Dictionary = f9.landing_zone()
		var heave := float(z.get("deck_heave_m", NAN))
		var roll := float(z.get("deck_roll_deg", NAN))
		var pitch := float(z.get("deck_pitch_deg", NAN))
		pose_sane = pose_sane and absf(heave) < 3.0 and absf(roll) < 10.0 and absf(pitch) < 10.0
		pose_seen = pose_seen or absf(heave) > 1e-3 or absf(pitch) > 1e-3
		var bow: Vector3 = z.get("deck_bow", Vector3.ZERO)
		var normal: Vector3 = z.get("deck_normal", Vector3.ZERO)
		pose_sane = pose_sane and absf(bow.length() - 1.0) < 1e-9 and absf(bow.dot(normal)) < 1e-9 \
				and absf(rad_to_deg(asin(clampf(bow.z, -1.0, 1.0))) - pitch) < 1e-6
	_check(pose_seen and pose_sane, "ASDS sea: the deck heaves and pitches (pose finite, axes orthonormal, bow up = pitch)")
	var legs: Dictionary = f9.body_legs(0)
	_check(legs.get("present", false) and (legs.get("azimuths_deg", []) as Array).size() == 4
			and float(legs.get("deployment", 1.0)) == 0.0 and legs.get("touchdown", "x") == "", "Falcon 9 stack: four stowed legs")
	var fins: Array = f9.body_grid_fins(0)
	_check(fins.size() == 4 and absf(float(fins[0].get("area_m2", 0.0)) - 1.8) < 1e-9, "Falcon 9 stack: four grid fins")
	_check((f9.return_state(0) as Dictionary).has("legs"), "return state carries the legs")
	_check(not _ok(f9, f9.start_mission("no_such_mission", tdb)), "unknown mission is refused")
	_check(_ok(f9, f9.launch_from_site("falcon9_block5", "ccsfs_slc40", 0.0, tdb)), "Falcon 9 launches without a mission")
	_advance(f9, tdb + 5.0)
	_check(f9.autopilot_status().get("state", "") == "aus", "a plain launch after a mission starts with the autopilot off")
	_check(not f9.landing_zone().get("present", true), "a plain launch has no landing zone")
	_check(f9.mission_site_id() == "ccsfs_slc40", "plain launch: its site")
	_check(f9.body_legs(7).is_empty() and f9.body_grid_fins(7).is_empty(), "invalid body index: empty results")
	# ascent Task 5: a free launch with a target is planned on the pad; a site without a tower refuses Super Heavy
	var refused := FlightSim.new()
	refused.setup(config_dir)
	var target := {"periapsis_km": 200.0, "apoapsis_km": 200.0, "inclination_deg": 26.5, "northerly": true}
	var refusal_id := refused.launch_from_site("starship_block3", "ccsfs_slc40", 0.0, tdb, 10.0, target)
	refused.wait_idle(60.0)
	var refusal: Dictionary = refused.result(refusal_id)
	_check(refusal.get("done", false) and not refusal.get("ok", true)
			and str(refusal.get("message", "")) == "Kein Fangturm am Startplatz.",
			"Starship at SLC-40: the launch is refused with the German reason (%s)" % str(refusal))
	_check(refused.is_active() and refused.autopilot_status().get("state", "") == "aus"
			and not refused.autopilot_status().get("planning", true), "refused launch: on the pad, autopilot off, not planning")
	var ship := FlightSim.new()
	ship.setup(config_dir)
	_check(_ok(ship, ship.launch_from_site("starship_block3", "starbase_olp1", 0.0, tdb)), "Starship launches")
	_check(not ship.landing_zone().get("present", true), "Starship: no landing zone")
	_check(not ship.body_legs(0).get("present", true), "Starship: no legs")
	_check(ship.body_grid_fins(0).size() == 3, "Super Heavy: three grid fins")
	_check(_ok(ship, ship.start_mission("starship_leo", tdb)) and ship.mission_site_id() == "starbase_olp1",
			"Starship LEO mission starts at Starbase")
	_check(not _ok(ship, ship.abort_landing(0)), "abort refused for a body that is not returning")
	_check(ship.is_active() and ship.tower_state().get("phase", "") != "Abbruch",
			"abort before staging changes nothing (tower not aborted)")
	# a refuelling flight that starts on its phasing orbit does not plan an ascent: the flag never shows
	var orbit := FlightSim.new()
	orbit.setup(config_dir)
	var orbit_id := orbit.start_mission("tanker_flight_orbit", tdb)
	var orbit_planning := false
	wait_start = Time.get_ticks_msec()
	while Time.get_ticks_msec() - wait_start < 120000 and not orbit.result(orbit_id).get("done", false):
		orbit_planning = orbit_planning or orbit.autopilot_status().get("planning", false)
		OS.delay_msec(5)
	_check(not orbit_planning, "start_mission from a phasing orbit never publishes the planning flag")
	return true


# Crew Dragon (d5 game interface): both missions via start_mission, thrusters, parachutes and the capsule state.
func _check_dragon(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	_check(sim.body_rcs(0).is_empty() and sim.body_parachutes(0).is_empty() and sim.capsule_state(0).is_empty()
			and not sim.splashdown_zone().get("present", true), "before a flight: empty thruster/parachute/capsule state")
	_check(_ok(sim, sim.start_mission("falcon9_crew_dragon", tdb)), "Falcon 9 with Crew Dragon starts")
	_check(absf(sim.mission_start_tdb_s() - tdb) < 1e-9, "an ascent starts at the requested time")
	var rcs: Array = sim.body_rcs(0)
	_check(rcs.size() == 8 + 8 + 16, "stack: 8 + 8 Falcon 9 thrusters and 16 Dracos")
	var dracos_blocked := true
	for t in rcs:
		if t.get("stage", 0) == 3 and not t.get("blocked", false):
			dracos_blocked = false
	_check(dracos_blocked, "stacked Dracos are blocked")
	_check((rcs[0] as Dictionary).has_all(["id", "position_m", "direction", "firing", "duty", "thrust_n", "starved"]),
			"thruster fields")
	_check(not sim.capsule_state(0).get("present", true), "a capsule stacked on its launcher is not a capsule body")
	_check(sim.body_parachutes(0).size() == 2, "the stacked capsule carries its two parachute sets")
	_check(not _ok(sim, sim.start_mission("ship_return_starbase", tdb)), "a ship return is not started by start_mission")

	var ret := FlightSim.new()
	ret.setup(config_dir)
	_check(_ok(ret, ret.start_mission("crew_dragon_return", tdb)), "Crew Dragon return mission starts")
	_check(absf(ret.mission_start_tdb_s() - tdb) > 1.0, "the return starts at its own epoch, not the requested one")
	var capsule: Dictionary = ret.capsule_state(ret.controlled_index())
	_check(capsule.get("present", false) and capsule.get("trunk_attached", false) and capsule.get("nose_cone_open", false),
			"capsule on its trunk with the nose cone open")
	_check(capsule.get("return_step", "") == "Deorbit-Brennvorgang", "the deorbit burn is planned")
	_check(capsule.get("parachute_phase", "x") == "", "no parachute sequencer while the trunk is attached")
	_check(ret.autopilot_status().get("return", {}).get("feasible", false), "the return plan is feasible")
	_check(ret.autopilot_status().get("capsule_step", "") == "Deorbit-Brennvorgang", "autopilot status: capsule step")
	var zone: Dictionary = ret.splashdown_zone()
	_check(zone.get("present", false) and absf(float(zone.get("radius_m", 0.0)) - 2000.0) < 1e-9, "splashdown zone radius 2 km")
	var chutes: Array = ret.body_parachutes(ret.controlled_index())
	_check(chutes.size() == 2 and chutes[0].get("state", "") == "verstaut" and float(chutes[0].get("inflation", 1.0)) == 0.0,
			"parachutes stowed")
	_check(ret.body_rcs(ret.controlled_index()).size() == 16, "Crew Dragon alone: 16 Dracos")
	_advance(ret, ret.mission_start_tdb_s() + 10.0)
	_check(ret.is_active(), "the return mission advances")
	_check(ret.body_rcs(9).is_empty() and ret.body_parachutes(9).is_empty() and ret.capsule_state(9).is_empty(),
			"invalid body index: empty results")
	_check(not _ok(ret, ret.plan_capsule_return()), "a second return plan is refused while one runs")
	_check(not _ok(ret, ret.set_splashdown_zone("no_such_zone")), "unknown splashdown zone is refused")
	return true


# Lunar missions (l6): both kinds start, lunar telemetry, terrain query, refusal, re-plan command.
func _check_lunar(config_dir: String, tdb: float) -> bool:
	var started := Time.get_ticks_msec()
	var ret := FlightSim.new()
	ret.setup(config_dir)
	var id: int = ret.start_mission("ship_lunar_return_llo", tdb)
	_check(ret.wait_idle(120.0), "lunar return planned within 120 s")
	_check(ret.result(id).get("ok", false), "lunar return mission starts: %s" % ret.result(id).get("message", ""))
	var t: Dictionary = ret.telemetry(ret.controlled_index())
	_check(t.get("reference_body", "") == "moon", "return ship is Moon-referenced")
	_check(absf(float(t.get("altitude_terrain_m", -1.0)) - float(t["altitude_m"])) < 1e-6,
			"no terrain: terrain altitude = shape altitude")
	_check(absf(float(t.get("periapsis_altitude_km", 0.0)) - 100.0) < 15.0, "LLO periselene ~100 km (osculating)")
	var lunar: Dictionary = ret.lunar_state()
	_check(lunar.get("present", false) and lunar["return"].get("feasible", false), "return plan in lunar_state")
	_check(float(lunar["return"].get("tei_delta_v_mps", 0.0)) > 700.0, "TEI delta-v plausible (> 700 m/s)")
	_check(not ret.moon_terrain(-84.8, 0.0).get("present", true), "no terrain loaded for the return mission")
	var rp: int = ret.replan_lunar_return()
	_check(ret.wait_idle(120.0) and ret.result(rp).get("done", false), "re-plan command finishes")
	print("  lunar return start: %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))

	started = Time.get_ticks_msec()
	var land := FlightSim.new()
	land.setup(config_dir)
	var lid: int = land.start_mission("hls_lunar_llo", tdb)
	_check(land.wait_idle(120.0) and land.result(lid).get("ok", false), "lunar landing mission starts")
	var site: Dictionary = land.lunar_site()
	_check(site.get("present", false), "landing site present")
	var ground: Dictionary = land.moon_terrain(site["latitude_deg"], site["longitude_deg"])
	_check(ground.get("present", false) and ground.get("in_tile", false), "terrain query at the site (tile)")
	_check(absf(float(ground["height_m"]) - float(site["height_m"])) < 1.0, "terrain height = site height")
	_check(land.telemetry(land.controlled_index()).get("reference_body", "") == "earth", "HLS starts in LEO")
	print("  lunar landing start: %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))

	started = Time.get_ticks_msec()
	var nrho := FlightSim.new()
	nrho.setup(config_dir)
	var nid: int = nrho.start_mission("hls_lunar_nrho", tdb)
	nrho.wait_idle(180.0)
	_check(not nrho.result(nid).get("ok", true) and "nicht durchführbar" in str(nrho.result(nid).get("message", "")),
			"NRHO landing at 120 t is refused honestly")
	print("  NRHO landing start (refused): %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))
	return true


# ISS missions (i6): the three kinds start, ISS / rendezvous / docking telemetry, station geometry, commands.
func _check_iss(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	var started := Time.get_ticks_msec()
	var id: int = sim.start_mission("crew_dragon_iss", tdb)
	_check(sim.wait_idle(120.0), "ISS rendezvous planned within 120 s")
	_check(sim.result(id).get("ok", false), "crew_dragon_iss starts: %s" % sim.result(id).get("message", ""))
	print("  ISS rendezvous start: %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))
	var iss: Dictionary = sim.iss_state()
	_check(iss.get("present", false) and iss["ports"].size() == 2, "iss_state with both ports")
	var plan: Dictionary = sim.rendezvous_plan()
	_check(plan.get("present", false) and plan.get("feasible", false), "rendezvous_plan feasible")
	_check(plan.get("burns", []).size() >= 3, "rendezvous_plan has its burns")
	_check(sim.iss_mission().get("leg", "") == "Phasenflug", "iss_mission in the phasing leg")
	_check(sim.iss_mission().get("separation_tdb_s", 0.0) > 0.0, "the start is a launch window's separation")
	var geo: Dictionary = sim.station_geometry()
	_check(geo.get("present", false) and geo["boxes"].size() + geo["capsules"].size() >= 9,
			"station_geometry primitives")
	_check(geo["ports"][0].get("mechanism", "") == "idss" and iss["ports"][0].get("mechanism", "") == "idss",
			"ISS ports export their mechanism")
	var hold: int = sim.proximity_hold()
	_check(sim.wait_idle(10.0) and not sim.result(hold).get("ok", true), "hold refused outside the approach")
	var und: int = sim.undock()
	_check(sim.wait_idle(10.0) and not sim.result(und).get("ok", true), "undock refused while not docked")
	_check(not sim.docking_state().get("docked", false), "not docked during phasing")
	_check(sim.iss_mission().get("leg", "") == "Phasenflug", "refused commands leave the sequence running")

	var pad := FlightSim.new()
	pad.setup(config_dir)
	var pid: int = pad.start_mission("falcon9_crew_dragon_iss", tdb)
	_check(pad.wait_idle(120.0) and pad.result(pid).get("ok", false), "falcon9_crew_dragon_iss starts")
	var liftoff: float = pad.iss_mission().get("liftoff_tdb_s", 0.0)
	_check(liftoff > 0.0, "launch window in iss_mission")
	_check(absf(pad.mission_start_tdb_s() - liftoff) < 1e-6, "the mission starts at the window")
	_check(pad.mission_site_id() == "ccsfs_slc40", "pad mission: its launch site")

	var dep := FlightSim.new()
	dep.setup(config_dir)
	var did: int = dep.start_mission("crew_dragon_iss_departure", tdb)
	_check(dep.wait_idle(120.0) and dep.result(did).get("ok", false), "crew_dragon_iss_departure starts")
	_check(dep.docking_state().get("docked", false), "departure mission starts docked")
	_check(dep.docking_state().get("hooks_closed", false), "hooks closed while docked")
	_check(dep.iss_mission().get("undock_tdb_s", 0.0) > dep.mission_start_tdb_s(), "undocking ahead")
	var ab: int = dep.proximity_abort()
	_check(dep.wait_idle(10.0) and not dep.result(ab).get("ok", true), "abort refused while docked")
	return true


# Refuelling r6: the campaign command, transfer and depot getters.
func _check_refuelling(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	_check(not sim.transfer_state().get("present", true), "no transfer without a docked pair")
	_check(not sim.depot_state().get("present", true), "no depot before a refuelling mission")
	_check(not sim.campaign_flight_state().get("present", true), "no flown campaign before one is started")
	var id: int = sim.start_mission("tanker_flight_orbit", tdb)
	_check(sim.wait_idle(120.0) and sim.result(id).get("ok", false), "tanker_flight_orbit starts")
	var depot: Dictionary = sim.depot_state()
	_check(depot.get("present", false) and depot.get("main_propellant_kg", 0.0) > 0.0, "depot_state with its fill")
	_check(depot.get("capacity_kg", 0.0) > depot.get("main_propellant_kg", 0.0), "depot_state with its tank volume")
	var depot_geo: Dictionary = sim.station_geometry()
	_check(depot_geo.get("present", false) and depot_geo["ports"][0].get("mechanism", "") == "probe_drogue"
			and depot_geo["ports"][0].get("elements_m", []).size() == 4, "depot port with its four drogues")
	# Crew escape: uncrewed by default; a crew-escape config makes the flight crewed (four seats, nothing armed).
	_check(not sim.crew_escape_state().get("present", true), "no crew escape on an uncrewed flight")
	var crewed: int = sim.set_crew_escape("starship_crew")
	_check(sim.wait_idle(10.0) and sim.result(crewed).get("ok", false), "set_crew_escape starship_crew")
	var crew: Dictionary = sim.crew_escape_state()
	_check(crew.get("present", false) and not crew.get("armed", true) and crew.get("members", []).size() == 4,
			"crew_escape_state with four seats, not armed")
	var no_crew: int = sim.set_crew_escape("no_such_crew")
	_check(sim.wait_idle(10.0) and not sim.result(no_crew).get("ok", true), "an unknown crew escape is refused")
	var uncrewed: int = sim.set_crew_escape("")
	_check(sim.wait_idle(10.0) and sim.result(uncrewed).get("ok", false)
			and not sim.crew_escape_state().get("present", true), "set_crew_escape('') makes the flight uncrewed")
	var bad: int = sim.refuelling_campaign({"height_km": "x"})
	_check(sim.wait_idle(10.0) and not sim.result(bad).get("ok", true), "a wrongly typed lever is refused")
	var unknown: int = sim.refuelling_campaign({"no_such_lever": 1.0})
	_check(sim.wait_idle(10.0) and not sim.result(unknown).get("ok", true), "an unknown lever is refused")
	var zero: int = sim.refuelling_campaign({"insulation": 0.0})
	_check(sim.wait_idle(10.0) and not sim.result(zero).get("ok", true), "a lever out of range is refused")
	var started := Time.get_ticks_msec()
	var cid: int = sim.refuelling_campaign({"height_km": 200.0})
	_check(sim.wait_idle(600.0) and sim.result(cid).get("ok", false), "refuelling_campaign finishes")
	var data: Dictionary = sim.result(cid).get("data", {})
	_check(data.get("tanker_flights", 0) >= 4 and data.get("tankers", []).size() == data.get("tanker_flights", -1),
			"campaign result with its tankers")
	_check(data.get("losses", {}).has("boil_off_t"), "campaign result with its losses")
	print("  refuelling campaign: %d tankers, %.1f d (%.1f s)" % [data.get("tanker_flights", 0),
			data.get("duration_d", 0.0), (Time.get_ticks_msec() - started) / 1000.0])
	return true


## The value of `const float <name> = <number>;` in game/shaders/clouds.gdshaderinc (NAN when absent).
func _shader_constant(constant_name: String) -> float:
	var text := FileAccess.get_file_as_string("res://shaders/clouds.gdshaderinc")
	var regex := RegEx.create_from_string("const float " + constant_name + "\\s*=\\s*([-0-9.eE+]+)\\s*;")
	var found := regex.search(text)
	return float(found.get_string(1)) if found != null else NAN
