class_name World
extends Node3D
## Earth-Moon-Sun world at the true (TDB) instant of the simulation clock.
##
## Every frame: query the float64 core (ephemeris, orientations), compute the camera's true GCRF
## position, then place everything relative to it (floating origin, scaled space for far bodies).

const EARTH_SEMI_MAJOR_AXIS_M := 6378137.0
const EARTH_FLATTENING := 1.0 / 298.257223563
const EARTH_MEAN_RADIUS_M := 6371000.0
const MOON_RADIUS_M := 1737400.0
const SUN_RADIUS_M := Atmosphere.SUN_RADIUS_M
const GROUND_EYE_HEIGHT_M := 2.0
## [E] Lag of the sim snapshot behind the instant it was asked for in the previous frame tolerated before the clock is
## held back (the sim thread answers last frame's request; a genuinely lagging sim falls further behind).
const CLOCK_HOLD_SLACK_S := 0.05
## [E] The pad disc's top sits this far above the site height: the terrain's plateau lies exactly at the site height,
## and coplanar surfaces z-fight. Far below anything a vehicle's contact could show.
const PAD_TOP_LIFT_M := 0.03
## [E] A site's viewpoint looks this far above its pad: the pad sits a little below the centre, the tower above it.
const VIEWPOINT_AIM_RAISE_DEG := 3.0
## The solar disk is drawn behind all scaled-space bodies (1.5e7 m) but in front of the stars (2.5e7 m).
const SUN_RENDER_DISTANCE_M := 2.0e7
## The linear limb darkening of shaders/sun.gdshader (u): the disk's mean radiance is (1 - u / 3) of its centre's.
const SUN_LIMB_DARKENING := 0.6

## Focus targets selectable in the UI.
const FOCUS_EARTH := "earth"
const FOCUS_MOON := "moon"
const FOCUS_EARTH_MOON := "earth_moon"
const FOCUS_VEHICLE := "vehicle"
const FOCUS_FREE := "free_camera"  ## detached camera flown with the HOTAS (FreeCameraControl)

var core := StarsimCore.new()
var clock := SimClock.new()
var camera := OrbitCamera.new()
var earth := CelestialBody.new()
var earth_terrain := TerrainQuadtree.new()  ## streamed Earth surface (relief + imagery) while not in scaled space
var water_waves := WaterWaves.new()  ## the sea's wave set (water.gdshaderinc), phases at the camera
var towers: Array[TowerView] = []  ## catch towers of the launch sites
var drone_ship: DroneShipView = null  ## the drone ship of the flight's landing zone (while it has a deck)
var moon := CelestialBody.new()
var moon_terrain := TerrainQuadtree.new()  ## streamed lunar surface (LDEM_16 + site tiles) while not in scaled space
var sun_disk := MeshInstance3D.new()
var sun_light := DirectionalLight3D.new()
## The Moon's light on Godot's lit materials at night (Atmosphere.moon_light_at; energy 0 by day; no shadows).
var moon_light := DirectionalLight3D.new()
var stars := StarField.new()
var moon_orbit := OrbitLine.new()
var flight := FlightController.new()
var view_target := ViewTarget.new()  ## the simulated body the external view shows (Tab / Shift+Tab)
var earth_label := MapLabel.new()
var moon_label := MapLabel.new()
var sites: Array = []  ## launch sites from config/launch_sites.toml
var focus := FOCUS_EARTH
var free_control := FreeCameraControl.new()  ## HOTAS mapping of the free camera
var free_pose := FreeCameraPose.new()  ## its pose (body-fixed near a body)
var free_wizard_requested := false  ## set on entering the free camera while no axis mapping exists (a joypad is connected)
var ground_site: Dictionary = {}
var spice_ok := false  ## SPICE kernels opened in _ready
var terrain_loaded: Dictionary = {}  ## core.load_terrain's result: {earth, moon}
var atmosphere := Atmosphere.new()  ## the precomputed atmosphere's tables and global shader uniforms (sky, ambient)
var refraction := Refraction.new()  ## astronomical refraction of Sun, Moon and stars for the camera
var weather := WeatherService.new()  ## ERA5 / GFS / analog snapshots for the clouds (M8 weather, Task 7)
var weather_fetcher := WeatherFetcher.new()  ## the service's HTTP requests (a node: HTTPRequest children)
var clouds := CloudLayer.new()  ## the volumetric clouds near the camera from the weather snapshots (M8 weather, Task 8)

## Per-frame state, readable by UI and tests.
var moon_gcrf_m := Vector3.ZERO
## The Sun's visible fraction at the camera from the lunar terrain (this frame; NAN: not applicable).
var _camera_terrain_sun_fraction := NAN
var _earth_heights: HeightSource  ## the terrains' height sources (SurfaceClearance)
var _moon_heights: HeightSource
var sun_apparent_from_earth_m := Vector3.ZERO
var moon_apparent_from_earth_m := Vector3.ZERO

var _earth_material := ShaderMaterial.new()
var _earth_shader: Shader = load("res://shaders/earth.gdshader")
var _earth_clouds_shader: Shader = load("res://shaders/earth_clouds.gdshader")  ## with the far-field clouds (Task 10)
var _moon_material := ShaderMaterial.new()
var _sky_material := ShaderMaterial.new()
var _radiance_refresh := SkyRadianceRefresh.new()
var _radiance_cloud_factor := 1.0
## [E] The Earth's terrain horizon is marched anew past these: 50 m at a horizon some km away is well under 1 % of the Earth's
## disc (radius 0.95 deg, 1 % ~ 0.02 deg), as is 0.02 deg of the Earth itself.
const EARTH_HORIZON_EYE_STEP_M := 50.0
const EARTH_HORIZON_EARTH_STEP_RAD := 3.5e-4
var _earth_horizon_eye_fixed_m := Vector3.INF
var _earth_horizon_earth_dir := Vector3.ZERO
var _earth_horizon_fraction := NAN
var _earth_horizon_ground_height_m := 0.0  # the terrain under the eye above the mean sphere
var _radiance_serial := 0


## Emitted when `_ready` has run through (or stopped at the missing SPICE kernels). `_ready` only suspends (between its
## loading-screen steps) in the VR start, where Main waits for this signal; everywhere else it runs synchronously.
signal boot_finished

var booted := false  ## the whole setup of `_ready` is done (the per-frame update waits for it)


func _ready() -> void:
	var data_dir := Paths.data_dir()
	await LoadingScreen.step("SPICE-Kernel (Ephemeriden, Erdorientierung) laden")
	if not core.open(data_dir.path_join("spice")):
		push_error("World: SPICE kernels missing - run tools/data_pipeline/fetch.py")
		boot_finished.emit()
		return
	spice_ok = true
	await LoadingScreen.step("Startplätze und Fahrzeugdaten laden")
	sites = TomlLite.load_file(Paths.config_dir().path_join("launch_sites.toml")).get("site", [])
	flight.setup(Paths.config_dir())
	free_control.setup(Paths.config_dir())
	add_child(flight)
	add_child(view_target)

	add_child(clock)
	add_child(camera)
	await LoadingScreen.step("Himmel und Atmosphäre vorbereiten")
	_setup_environment()
	atmosphere.start_loading(core, Paths.config_dir(), data_dir.path_join("atmosphere"))
	await LoadingScreen.step("Wetterdienst und Wolken einrichten")
	add_child(weather_fetcher)
	weather.enabled = WeatherService.load_enabled_setting()
	weather.setup(core, Paths.config_dir(), data_dir.path_join("weather/cache"), data_dir.path_join("weather/analog"),
			weather_fetcher)
	add_child(clouds)
	clouds.setup(core)
	var xr := get_node_or_null("/root/XrManager")
	clouds.quality = CloudLayer.Quality.VR if xr != null and xr.vr_requested() else CloudLayer.Quality.DESKTOP
	clouds.pass_enabled = not OS.get_cmdline_user_args().has("--no_cloud_pass")
	_xr = xr
	weather.light_driven_by_layer = true  # the layer pushes the CPU light's pair with its cross-fade
	weather.snapshots_changed.connect(clouds.set_target)
	clouds.core_cache_missed.connect(weather.reload_into_core_cache)
	atmosphere.direct_sun = clouds.direct_sun_transmittance  # the vehicles' and the eye's direct Sun: the drawn shapes
	clock.time_jumped.connect(func(_tdb_s: float) -> void: _sim_target_tdb_s = NAN)  # no lag after a jump
	await _setup_bodies(data_dir)
	await LoadingScreen.step("Wolken-Shader vorwärmen, Sternkatalog laden")
	_warm_earth_clouds_shader()
	stars.load_catalogue(data_dir.path_join("stars/stars.csv"))
	add_child(stars)
	add_child(moon_orbit)
	moon_orbit.visible = false
	_setup_label(earth_label, "Erde")
	_setup_label(moon_label, "Mond")
	clock.set_time(core.utc_to_tdb_s(Time.get_datetime_string_from_system(true)))
	if PerfLog.requested():
		add_child(PerfLog.new(self))
	booted = true
	boot_finished.emit()


var _delta := 0.0
var _xr: Node  ## the XrManager autoload (null in tests without it)
var _warmup_mesh: MeshInstance3D  ## draws earth_clouds.gdshader once at load (first switch: no pipeline compile mid-flight)
var _warmup_frames := 0
var _mission_press_tdb_s := 0.0  ## clock instant of the pending mission start (the clock holds there meanwhile)
var _camera_serial := 0  ## serial the camera distance was last fitted to
var _last_world_tdb_s := NAN  ## the instant of the previous update_world (eye adaptation's time step)
var _sim_target_tdb_s := NAN  ## the instant the flight sim was asked for last frame (clock_held_back)
var _environment: Environment  ## the WorldEnvironment's (ambient energy under the clouds, Task 9)


## The atmosphere's worker tasks write into it and use the core: finished before both go away (review M2).
func _exit_tree() -> void:
	atmosphere.shutdown()
	weather.shutdown()
	clouds.shutdown()


func _process(delta: float) -> void:
	if not booted or not core.is_open():
		return
	_delta = delta
	update_world()
	if _warmup_mesh != null:
		_warmup_frames -= 1
		if _warmup_frames <= 0:
			_warmup_mesh.queue_free()
			_warmup_mesh = null


## Whether the world clock waits for the flight sim: its snapshot (`reached_tdb_s`) is behind the instant the sim was
## asked for in the previous frame by more than CLOCK_HOLD_SLACK_S. The snapshot always trails the clock by about one
## frame step (warp x delta); comparing with this frame's delta instead held the clock back whenever a frame was shorter
## than the one before (jitter, VR reprojection) - the flight ran at 50-75 % speed at high time warp. NAN: no previous
## request (first frame, after a clock jump) - no hold.
static func clock_held_back(previous_target_tdb_s: float, reached_tdb_s: float) -> bool:
	return not is_nan(previous_target_tdb_s) and previous_target_tdb_s - reached_tdb_s > CLOCK_HOLD_SLACK_S


## Recomputes the whole scene for clock.tdb_s. Public so tests can render a fixed instant.
func update_world() -> void:
	# The flight simulation may lag behind the clock at high time warp; the clock then waits for it.
	var t := clock.tdb_s
	FrameProfiler.begin(&"flight_advance")
	if flight.mission_pending():
		# "Startvorbereitung": the world clock stands at the press instant while the mission start runs; its outcome
		# (mission_started, inside this advance) sets the clock to the mission's epoch - no hold-back in this frame.
		clock.tdb_s = _mission_press_tdb_s
		flight.advance(clock.tdb_s, _delta)
		t = clock.tdb_s
	elif flight.start_pending() and not flight.sim.is_active():
		flight.advance(clock.tdb_s, _delta)  # only collects the start's outcome
	elif flight.sim.is_active():
		flight.sim.set_realtime_planning(clock.warp <= 1.0)  # landing re-plans never stall real-time play
		var previous_target := _sim_target_tdb_s
		var reached := flight.advance(clock.tdb_s, _delta)
		_sim_target_tdb_s = clock.tdb_s
		if flight.take_clock_resync():  # a refused mission start left its vehicle at its own epoch
			clock.set_time(reached)
			_sim_target_tdb_s = NAN
		elif clock_held_back(previous_target, reached):
			clock.tdb_s = reached
			_sim_target_tdb_s = reached
		# The scene is drawn at the instant of the simulation snapshot (the sim thread trails the clock by about a
		# frame, unevenly): Earth, pad and tower at the clock time but vehicles at the snapshot made the vehicles jump
		# by up to several metres against the pad every frame (Earth rotation ~420 m/s).
		t = reached
	FrameProfiler.end(&"flight_advance")
	FrameProfiler.begin(&"weather")
	weather.update(t, clock.warp)  # non-blocking: downloads, builds and loads run in the background
	if _xr != null:
		_xr.apply_frame_rate(weather.enabled)  # VR: 45 Hz with weather, 90 Hz without (user 2026-10-27)
	FrameProfiler.end(&"weather")
	FrameProfiler.begin(&"towers_zone")
	for view in towers:
		var at_site: bool = flight.sim.is_active() and view.site_id == flight.launch_site_id
		view.update_state(flight.sim.tower_state() if at_site else {})
	var zone: Dictionary = flight.sim.landing_zone() if flight.sim.is_active() else {}
	_update_drone_ship(zone)
	water_waves.set_sea_state(zone.get("sea_state", {}))  # the calm set without a sea state
	FrameProfiler.end(&"towers_zone")
	FrameProfiler.begin(&"ephemeris")
	earth.fixed_to_gcrf = core.earth_fixed_to_gcrf(t)
	moon.fixed_to_gcrf = core.moon_fixed_to_gcrf(t)
	moon_gcrf_m = core.body_position_gcrf_m("moon", t)
	moon.center_gcrf_m = moon_gcrf_m
	FrameProfiler.end(&"ephemeris")

	FrameProfiler.begin(&"camera")
	_update_camera(t)
	FrameProfiler.end(&"camera")
	var cam := camera.position_gcrf_m

	# Apparent directions: as seen from the Earth's centre for Earth shading, from the camera for
	# what the camera sees directly (solar disk, light direction).
	FrameProfiler.begin(&"ephemeris")
	sun_apparent_from_earth_m = core.apparent_body_position_gcrf_m("sun", t, Vector3.ZERO, Vector3.ZERO)
	moon_apparent_from_earth_m = core.apparent_body_position_gcrf_m("moon", t, Vector3.ZERO, Vector3.ZERO)
	var sun_from_camera := core.apparent_body_position_gcrf_m("sun", t, cam, Vector3.ZERO)
	FrameProfiler.end(&"ephemeris")
	# Eye adaptation: the viewer adapts over the longer of the frame time and the simulated step (paused: real time;
	# time warp: a time-lapse whose exposure follows the scene; a clock jump: adapted at once).
	FrameProfiler.begin(&"atmosphere")
	var adaptation_dt := maxf(_delta, absf(t - _last_world_tdb_s)) if not is_nan(_last_world_tdb_s) else 0.0
	_last_world_tdb_s = t
	_camera_terrain_sun_fraction = _moon_terrain_sun_fraction(cam)
	atmosphere.eye_terrain_sun_fraction = _camera_terrain_sun_fraction
	var camera_sun_fraction := 1.0 - core.solar_eclipse_fraction_gcrf(cam, t)
	atmosphere.update(cam, Vector3.ZERO, earth.fixed_to_gcrf, sun_apparent_from_earth_m, adaptation_dt,
			camera_sun_fraction, moon_apparent_from_earth_m, t, moon_gcrf_m)
	_update_sky_radiance(cam, camera_sun_fraction)
	FrameProfiler.end(&"atmosphere")

	FrameProfiler.begin(&"camera")
	earth.place(cam)
	FrameProfiler.end(&"camera")
	var camera_fixed := earth.fixed_to_gcrf.transposed() * cam
	FrameProfiler.begin(&"clouds")
	clouds.update(camera_fixed, weather.blend(), _delta)
	# The scaled-space Earth with the far-field clouds exactly while the cloud pass is off and weather is shown (two
	# shaders: weather off keeps the earth.gdshader of before bit for bit, Task 10).
	var earth_shader := _earth_clouds_shader if clouds.far_enabled else _earth_shader
	if _earth_material.shader != earth_shader:
		_earth_material.shader = earth_shader
	FrameProfiler.end(&"clouds")
	FrameProfiler.begin(&"refraction")
	refraction.update(core, camera_fixed, earth.fixed_to_gcrf)
	FrameProfiler.end(&"refraction")
	FrameProfiler.begin(&"earth_terrain")
	earth_terrain.update_for_camera(camera_fixed, _delta)
	FrameProfiler.end(&"earth_terrain")
	FrameProfiler.begin(&"water")
	water_waves.update(camera_fixed, t)
	FrameProfiler.end(&"water")
	earth.body_mesh_visible = not earth_terrain.active
	FrameProfiler.begin(&"moon")
	moon.place(cam)
	moon_terrain.update_for_camera(moon.fixed_to_gcrf.transposed() * (cam - moon_gcrf_m), _delta)
	FrameProfiler.end(&"moon")
	moon.body_mesh_visible = not moon_terrain.active
	FrameProfiler.begin(&"shader_globals")
	var sun_local := earth.to_local_frame(sun_apparent_from_earth_m)
	var moon_local := earth.to_local_frame(moon_apparent_from_earth_m)
	_earth_material.set_shader_parameter("sun_position_local", sun_local)
	_earth_material.set_shader_parameter("moon_position_local", moon_local)
	# Global shader uniforms (project.godot [shader_globals]): one write for every terrain node material.
	ShaderGlobals.set_value("earth_sun_position_local", sun_local)
	ShaderGlobals.set_value("earth_moon_position_local", moon_local)
	ShaderGlobals.set_value("water_wave_vectors", water_waves.vectors)
	ShaderGlobals.set_value("water_wave_phases", water_waves.phases)
	var moon_sun_local := moon.to_local_frame(sun_apparent_from_earth_m - moon_gcrf_m)
	var moon_earth_local := moon.to_local_frame(-moon_gcrf_m)
	_moon_material.set_shader_parameter("sun_position_local", moon_sun_local)
	_moon_material.set_shader_parameter("earth_position_local", moon_earth_local)
	ShaderGlobals.set_value("moon_sun_position_local", moon_sun_local)
	ShaderGlobals.set_value("moon_earth_position_local", moon_earth_local)
	_moon_material.set_shader_parameter("camera_position_local", moon.to_local_frame(cam - moon_gcrf_m))
	FrameProfiler.end(&"shader_globals")

	FrameProfiler.begin(&"sun")
	_place_sun(sun_from_camera)
	_update_sun_light(t, cam)
	FrameProfiler.end(&"sun")
	FrameProfiler.begin(&"flight_views")
	flight.place_views(self, cam)
	FrameProfiler.end(&"flight_views")
	FrameProfiler.begin(&"moon_orbit")
	moon_orbit.update_line(core, t, cam)
	earth_label.position = earth.position
	moon_label.position = moon.position
	FrameProfiler.end(&"moon_orbit")


## Puts a vehicle on a launch site at the current instant and focuses the camera on it; the ascent autopilot counts
## down `countdown_s` and lifts off at T-0. `vehicle_name`: display name for the messages ("" shows the file id).
func launch_vehicle(vehicle_id: String, site: Dictionary, countdown_s := FlightController.LAUNCH_COUNTDOWN_S,
		vehicle_name := "") -> bool:
	if not flight.launch(vehicle_id, site, clock.tdb_s, 90.0, countdown_s, vehicle_name):
		return false
	clock.warp = 1.0
	set_focus(FOCUS_VEHICLE)
	return true


## Starts a mission of config/missions at the current instant (FlightController.start_mission). On success the clock
## goes to the mission's start epoch (a return mission starts at its own, possibly earlier one) at warp 1 and the camera
## follows the vehicle; a refused start that placed a vehicle anyway puts the clock at that vehicle's epoch
## (take_clock_resync). `on_pad`: the mission starts from a pad. Outcome: flight.mission_started.
func start_mission(mission_id: String, on_pad := false) -> bool:
	if not flight.mission_started.is_connected(_on_mission_started):
		flight.mission_started.connect(_on_mission_started)
	_mission_press_tdb_s = clock.tdb_s
	return flight.start_mission(mission_id, clock.tdb_s, on_pad)


func _on_mission_started(ok: bool, _message: String) -> void:
	if not ok:
		return
	clock.set_time(flight.sim.mission_start_tdb_s())
	clock.warp = 1.0
	set_focus(FOCUS_VEHICLE)


## A new view (ground site, focus, Tab target): the viewer is taken as adapted to it.
func _view_jumped() -> void:
	atmosphere.snap_adaptation()


## Ground view from an arbitrary site (dictionary like the entries of config/launch_sites.toml; optional eye_height_m
## above height_m, default GROUND_EYE_HEIGHT_M; optional body "moon": selenographic coordinates, height_m above the
## mean sphere).
func set_ground_site(site: Dictionary) -> void:
	_view_jumped()
	focus = site.get("id", "custom")
	ground_site = site
	camera.mode = OrbitCamera.Mode.GROUND
	moon_orbit.visible = false
	earth_label.visible = false
	moon_label.visible = false


func set_focus(new_focus: String) -> void:
	_view_jumped()
	var was_free := focus == FOCUS_FREE
	focus = new_focus
	ground_site = {}
	if new_focus == FOCUS_FREE:
		_enter_free_camera(was_free)
		return
	camera.mode = OrbitCamera.Mode.ORBIT
	moon_orbit.visible = focus == FOCUS_EARTH_MOON
	earth_label.visible = moon_orbit.visible
	moon_label.visible = moon_orbit.visible
	match focus:
		FOCUS_EARTH:
			camera.distance_m = 2.2e7
		FOCUS_MOON:
			camera.distance_m = 7.0e6
		FOCUS_EARTH_MOON:
			camera.distance_m = 1.1e9
			camera.pitch = -1.2
		FOCUS_VEHICLE:
			view_target.follow()  # a new flight: follow the controlled body
			_camera_serial = 0
			camera.distance_m = 260.0
			camera.pitch = -0.15
			camera.yaw = 0.8
			flight.engage_autopilot()  # the external view flies on the autopilot (user 2026-10-22)
		_:
			for site in sites:
				if site["id"] == focus:
					ground_site = site
					camera.mode = OrbitCamera.Mode.GROUND
					if site.has("viewpoint"):
						ground_site = _viewpoint_site(site)
						_aim_ground_camera_at(site)


## The ground view of a site with a [site.viewpoint]: the eye eye_height_m above the core's surface at the viewpoint
## (the Earth terrain; without it the site's flat height). Keeps the site's id: the focus stays that site.
func _viewpoint_site(site: Dictionary) -> Dictionary:
	var viewpoint: Dictionary = site["viewpoint"]
	var lat: float = viewpoint["latitude_deg"]
	var lon: float = viewpoint["longitude_deg"]
	var samples: Dictionary = core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var surface_m: float = samples["height_m"][0] if bool(samples["present"]) else float(site["height_m"])
	return {"id": site["id"], "name": viewpoint["name"], "latitude_deg": lat, "longitude_deg": lon, "height_m": surface_m,
			"eye_height_m": float(viewpoint["eye_height_m"])}


## Turns the ground camera from ground_site towards the site's pad at its height, raised by VIEWPOINT_AIM_RAISE_DEG.
func _aim_ground_camera_at(site: Dictionary) -> void:
	var t := clock.tdb_s
	var eye_m: float = ground_site["height_m"] + float(ground_site["eye_height_m"])
	var eye := core.geodetic_to_gcrf_m(ground_site["latitude_deg"], ground_site["longitude_deg"], eye_m, t)
	var pad := core.geodetic_to_gcrf_m(site["latitude_deg"], site["longitude_deg"], site["height_m"], t)
	var lat := deg_to_rad(ground_site["latitude_deg"])
	var lon := deg_to_rad(ground_site["longitude_deg"])
	var rotation := core.earth_fixed_to_gcrf(t)
	var up := rotation * Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var east := rotation * Vector3(-sin(lon), cos(lon), 0.0)
	var north := rotation * Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	var d := (pad - eye).normalized()
	camera.azimuth = fposmod(atan2(d.dot(east), d.dot(north)), TAU)
	camera.elevation = asin(clampf(d.dot(up), -1.0, 1.0)) + deg_to_rad(VIEWPOINT_AIM_RAISE_DEG)


## The free camera starts where the view is now (position and orientation); selecting it again keeps its pose.
func _enter_free_camera(already_free: bool) -> void:
	moon_orbit.visible = false
	earth_label.visible = false
	moon_label.visible = false
	if already_free:
		return
	free_wizard_requested = not free_control.has_calibration() and not Input.get_connected_joypads().is_empty()
	free_pose.reset_to(camera.position_gcrf_m, camera.basis, earth.fixed_to_gcrf, moon.fixed_to_gcrf, moon_gcrf_m)
	camera.mode = OrbitCamera.Mode.FREE
	camera.free_basis = camera.basis


## One frame of the free camera: the HOTAS command (real-time delta, independent of the time warp) moves its pose.
func _update_free_camera() -> void:
	var command := free_control.poll(_free_surface_distance_m())
	free_pose.turn(camera.free_look_rad)  # mouse look (desktop)
	camera.free_look_rad = Vector3.ZERO
	var pose := free_pose.step(command["rates"], command["velocity"], _delta, earth.fixed_to_gcrf, moon.fixed_to_gcrf,
			moon_gcrf_m)
	camera.position_gcrf_m = pose["position_gcrf_m"]
	camera.free_basis = pose["basis"]


## The free camera's distance to the nearest surface of Earth (WGS84 ellipsoid, no terrain) or the Moon (sphere): its
## translation speed scales with it (FreeCameraControl.poll). The Sun is only a disc (no body to fly to): not counted.
func _free_surface_distance_m() -> float:
	var p := camera.position_gcrf_m
	var r := p.length()
	var sin_lat := p.z / maxf(r, 1.0)  # geocentric; GCRF z ~ Earth's axis (precession is far below this use)
	var b := Atmosphere.WGS84_A_M / Atmosphere.POLAR_STRETCH
	var a := Atmosphere.WGS84_A_M
	var surface := a * b / sqrt(b * b * (1.0 - sin_lat * sin_lat) + a * a * sin_lat * sin_lat)
	var earth := r - surface
	var moon := (p - moon_gcrf_m).length() - MOON_RADIUS_M
	return minf(earth, moon)


func _update_camera(t: float) -> void:
	if camera.mode == OrbitCamera.Mode.FREE:
		_update_free_camera()
	elif camera.mode == OrbitCamera.Mode.GROUND:
		var lat := deg_to_rad(ground_site["latitude_deg"])
		var lon := deg_to_rad(ground_site["longitude_deg"])
		var eye_m: float = ground_site["height_m"] + float(ground_site.get("eye_height_m", GROUND_EYE_HEIGHT_M))
		var on_moon: bool = ground_site.get("body", "earth") == "moon"
		var rotation := moon.fixed_to_gcrf if on_moon else earth.fixed_to_gcrf
		if on_moon:  # selenographic (MOON_ME) on the mean sphere
			camera.site_gcrf_m = moon_gcrf_m + rotation * (Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
					* (MOON_RADIUS_M + eye_m))
		else:
			camera.site_gcrf_m = core.geodetic_to_gcrf_m(ground_site["latitude_deg"], ground_site["longitude_deg"], eye_m, t)
		camera.site_up_gcrf = rotation * Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
		camera.site_east_gcrf = rotation * Vector3(-sin(lon), cos(lon), 0.0)
		camera.site_north_gcrf = rotation * Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	elif focus == FOCUS_VEHICLE and flight.sim.is_active():
		_update_vehicle_camera()
	else:
		camera.target_gcrf_m = moon_gcrf_m if focus == FOCUS_MOON else Vector3.ZERO
		camera.reference_body = ""
	camera.update_pose(_delta)
	_keep_camera_above_surface()


## No camera closer than SurfaceClearance.MIN_CLEARANCE_M to the Earth's or the Moon's terrain (user 2026-10-29): a
## pose below it is moved out along the local vertical; the free camera's own pose follows (no drift into the ground).
func _keep_camera_above_surface() -> void:
	var p := camera.position_gcrf_m
	var on_moon := (p - moon_gcrf_m).length() - MOON_RADIUS_M < p.length() - EARTH_SEMI_MAJOR_AXIS_M
	var rotation := moon.fixed_to_gcrf if on_moon else earth.fixed_to_gcrf
	var center := moon_gcrf_m if on_moon else Vector3.ZERO
	var fixed := rotation.transposed() * (p - center)
	var kept := SurfaceClearance.clamp_fixed(fixed, _moon_heights if on_moon else _earth_heights)
	if kept == fixed:
		return
	camera.position_gcrf_m = center + rotation * kept
	if camera.mode == OrbitCamera.Mode.FREE:
		free_pose.reset_to(camera.position_gcrf_m, camera.free_basis, earth.fixed_to_gcrf, moon.fixed_to_gcrf, moon_gcrf_m)


## Orbit camera on the viewed object's CG, with the local vertical of the body it is flying around.
func _update_vehicle_camera() -> void:
	var index := view_target.resolve(flight.sim)
	if index < 0:
		return
	if view_target.serial != _camera_serial:  # a switch: yaw/pitch stay, the distance fits the new object
		if _camera_serial != 0:
			camera.distance_m = ViewTarget.clamp_distance_m(camera.distance_m, view_target.object_length_m(flight.sim, index))
		_camera_serial = view_target.serial
	camera.target_gcrf_m = view_target.cg_gcrf_m(flight.sim, index)
	camera.reference_body = str(flight.sim.telemetry(index).get("reference_body", "earth"))
	camera.reference_center_gcrf_m = moon_gcrf_m if camera.reference_body == "moon" else Vector3.ZERO


## Tab / Shift+Tab switch the viewed object. Handled in _input (before GUI focus traversal), so it also works while a
## control such as the UTC field of the time panel holds the focus; only when the vehicle view is active.
func _input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo or key.keycode != KEY_TAB:
		return
	if focus != FOCUS_VEHICLE or not flight.sim.is_active():
		return
	view_target.cycle(flight.sim, -1 if key.shift_pressed else 1)
	_view_jumped()  # another body is another place
	get_viewport().set_input_as_handled()


## The Sun disc's central scene value: its physical radiance on the scene scale of a white Lambertian surface in full
## Sun (Atmosphere.DAY_EXPOSURE = E / pi): the disc's mean L = E / solid angle, so DAY_EXPOSURE x pi / (pi (R / AU)^2) =
## 1.5 x 46 200 = 6.9e4, its centre that over the mean limb factor (1 - u / 3): 8.67e4. The Sun seen through cloud then
## stays above the lit ground until the direct beam falls to the ground's radiance (static clouds Task 9); sun.gdshader
## clamps it to the half-float range (above T_atm x the exposure gain ~0.69, colour kept).
static func sun_disc_scene_value() -> float:
	var ratio := Atmosphere.ASTRONOMICAL_UNIT_M / SUN_RADIUS_M
	return Atmosphere.DAY_EXPOSURE * ratio * ratio / (1.0 - SUN_LIMB_DARKENING / 3.0)


## The Sun disc's material (shaders/sun.gdshader) at its physical centre value and limb darkening - the game's and the
## checks' one source.
static func sun_disc_material() -> ShaderMaterial:
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/sun.gdshader")
	material.set_shader_parameter("intensity", sun_disc_scene_value())
	material.set_shader_parameter("limb_darkening", SUN_LIMB_DARKENING)
	return material


func _place_sun(sun_from_camera_m: Vector3) -> void:
	var distance := sun_from_camera_m.length()
	var direction := Frames.to_render(sun_from_camera_m / distance)
	var render_distance := SUN_RENDER_DISTANCE_M
	sun_disk.position = direction * render_distance
	sun_disk.scale = Vector3.ONE * SUN_RADIUS_M * render_distance / distance
	sun_light.basis = Basis.looking_at(-direction, Vector3.UP if absf(direction.y) < 0.99 else Vector3.RIGHT)


## The Sun's visible fraction at a point from the lunar terrain's horizon (MoonSunHorizon) near the Moon's ground
## (Atmosphere.moon_terrain_sun_applies: within the Moon's sphere of influence, at any height); NAN elsewhere or without
## lunar terrain data (the mean sphere's occultation stays).
func _moon_terrain_sun_fraction(point_gcrf_m: Vector3) -> float:
	if not Atmosphere.moon_terrain_sun_applies(point_gcrf_m, moon_gcrf_m):
		return NAN
	var gcrf_to_moon_fixed := moon.fixed_to_gcrf.transposed()
	return MoonSunHorizon.sun_visible_fraction(core, gcrf_to_moon_fixed * (point_gcrf_m - moon_gcrf_m),
			gcrf_to_moon_fixed * (sun_apparent_from_earth_m - moon_gcrf_m))


## The DirectionalLight's colour and energy: the Sun transmitted through the atmosphere to the followed body (the
## camera when no vehicle is followed), with the part of the solar disc the Moon leaves (Atmosphere.sun_light_at; near
## the Moon on the lunar terrain's horizon, _moon_terrain_sun_fraction; Atmosphere.moon_visible_fraction otherwise).
func _update_sun_light(t: float, camera_gcrf_m: Vector3) -> void:
	var lit_gcrf := camera_gcrf_m
	if focus == FOCUS_VEHICLE and flight.sim.is_active():
		var index := view_target.resolve(flight.sim)
		if index >= 0:
			lit_gcrf = flight.sim.body_position_gcrf_m(index)
	var gcrf_to_fixed := earth.fixed_to_gcrf.transposed()
	# The camera's fraction is already computed this frame (eye adaptation); a followed vehicle gets its own march.
	var terrain_visible := _camera_terrain_sun_fraction
	if lit_gcrf != camera_gcrf_m:
		terrain_visible = _moon_terrain_sun_fraction(lit_gcrf)
	# The core's fraction uses the Moon's mean sphere (the fallback without terrain data).
	var visible := Atmosphere.sun_visible_near_moon(lit_gcrf, sun_apparent_from_earth_m, moon_gcrf_m,
			1.0 - core.solar_eclipse_fraction_gcrf(lit_gcrf, t), terrain_visible)
	var light := atmosphere.sun_light_at(gcrf_to_fixed * lit_gcrf, (gcrf_to_fixed * sun_apparent_from_earth_m).normalized(),
			visible)
	sun_light.light_color = light[0]
	sun_light.light_energy = light[1]
	# The sky's ambient under the clouds (Task 9): the diffuse light at the lit body over the clear sky's (1 without
	# weather).
	_radiance_cloud_factor = atmosphere.cloud_ambient_factor(gcrf_to_fixed * lit_gcrf,
			(gcrf_to_fixed * sun_apparent_from_earth_m).normalized())
	var moon_lit: Array = atmosphere.moon_light_at(gcrf_to_fixed * lit_gcrf)
	# Near the Moon its light is the ground in the sky's cube: no second light from the Moon's centre through the ground.
	# (The one switch of both, near_moon: from there on the Moon's disc outgrows the Earth's. Far out it is ~2 cube texels and
	# the light still the Earth-distance value: a stated limit, atmosphere-scattering.md.)
	moon_light.visible = moon_lit[1] > 0.0 and not Atmosphere.near_moon(lit_gcrf.length(), (moon_gcrf_m - lit_gcrf).length())
	if moon_light.visible:
		var to_moon := Frames.to_render((moon_gcrf_m - lit_gcrf).normalized())
		moon_light.basis = Basis.looking_at(-to_moon, Vector3.UP if absf(to_moon.y) < 0.99 else Vector3.RIGHT)
		moon_light.light_color = moon_lit[0]
		moon_light.light_energy = moon_lit[1]


## Draws the radiance cubemap anew when the sky has really changed (SkyRadianceRefresh): the write to the sky material is
## what makes Godot do it, over a few frames (INCREMENTAL). Near the Moon the cube also shows the lunar ground, whose place
## and lights are set with that write, for the cube alone (sky.gdshader).
func _update_sky_radiance(camera_gcrf_m: Vector3, sun_fraction: float) -> void:
	var eye_from_moon := camera_gcrf_m - moon_gcrf_m
	var moon_distance_m := eye_from_moon.length()
	var near_moon := Atmosphere.near_moon(camera_gcrf_m.length(), moon_distance_m)
	var moon_radius_m := Atmosphere.moon_ground_radius(moon_distance_m)
	var moon_zenith := Frames.to_render(eye_from_moon / moon_distance_m)
	var to_earth_m := -camera_gcrf_m
	var to_moon_m := moon_gcrf_m - camera_gcrf_m
	# Near the Moon: what the eye sees of the Earth decides its flux in the cube (the terrain's horizon where the core has
	# it), and the eye's shadows stand for the ground in view, at any distance the Earth's shadow on the ground.
	var earth_visible := 1.0
	var ground_sun := 1.0
	var ground_earth := 1.0
	if near_moon:
		earth_visible = _moon_earth_visible(camera_gcrf_m, moon_radius_m)
		var earth_on_sun := 1.0 - Atmosphere.occulted_fraction(sun_apparent_from_earth_m - camera_gcrf_m, Atmosphere.SUN_RADIUS_M,
				to_earth_m, Atmosphere.EARTH_MEAN_RADIUS_M)
		var moon_on_sun := _camera_terrain_sun_fraction if is_finite(_camera_terrain_sun_fraction) \
				else Atmosphere.moon_visible_fraction(camera_gcrf_m, sun_apparent_from_earth_m, moon_gcrf_m, sun_fraction)
		var height_m := moon_distance_m - moon_radius_m - _earth_horizon_ground_height_m
		ground_sun = earth_on_sun * RadianceFarEarth.ground_fraction(moon_on_sun, height_m)
		ground_earth = RadianceFarEarth.ground_fraction(earth_visible, height_m)
	var due := _radiance_refresh.due(atmosphere.ready, Frames.to_render(camera_gcrf_m.normalized()),
			camera_gcrf_m.length() - Atmosphere.EARTH_MEAN_RADIUS_M, moon_zenith,
			moon_distance_m - moon_radius_m if near_moon else -1.0, atmosphere.sun_direction_local,
			atmosphere.luminance_scale, atmosphere.moonlight_relative_value if atmosphere.moonlight_active else 0.0,
			[sun_fraction, earth_visible, ground_sun, ground_earth, _radiance_cloud_factor])
	if not due:
		return
	_radiance_serial += 1
	_sky_material.set_shader_parameter("radiance_serial", float(_radiance_serial))
	var lift := RadianceFarEarth.lift_rad(to_earth_m, to_moon_m, moon_radius_m) if near_moon else 0.0
	_sky_material.set_shader_parameter("radiance_earth_flux", earth_visible)
	_sky_material.set_shader_parameter("radiance_earth_flat", near_moon)
	_sky_material.set_shader_parameter("radiance_earth_lifted", lift > 0.0)
	_sky_material.set_shader_parameter("radiance_earth_true", Frames.to_render(to_earth_m.normalized()))
	_sky_material.set_shader_parameter("radiance_earth_drawn",
			Frames.to_render(RadianceFarEarth.lifted_direction(to_earth_m, to_moon_m, lift)))
	_sky_material.set_shader_parameter("radiance_moon_sun_visible", ground_sun)
	_sky_material.set_shader_parameter("radiance_moon_earth_visible", ground_earth)
	_sky_material.set_shader_parameter("radiance_cloud_factor", _radiance_cloud_factor)
	_sky_material.set_shader_parameter("radiance_moon_radius_m", moon_radius_m if near_moon else 0.0)
	_sky_material.set_shader_parameter("radiance_moon_distance_m", moon_distance_m)
	_sky_material.set_shader_parameter("radiance_moon_zenith", moon_zenith)
	_sky_material.set_shader_parameter("radiance_moon_to_sun",
			Frames.to_render((sun_apparent_from_earth_m - moon_gcrf_m).normalized()))
	_sky_material.set_shader_parameter("radiance_moon_to_earth", Frames.to_render((-moon_gcrf_m).normalized()))


## The part of the Earth's disc an eye near the Moon sees: the terrain's horizon where the core has it (marched anew only
## when the eye or the Earth moved noticeably: the Earth creeps across a lunar sky), else the cube's ground sphere. Also
## notes the terrain's height under the eye (the eye's height over the ground, for RadianceFarEarth.ground_fraction).
func _moon_earth_visible(camera_gcrf_m: Vector3, moon_radius_m: float) -> float:
	var to_moon_m := moon_gcrf_m - camera_gcrf_m
	var mean := 1.0 - Atmosphere.occulted_fraction(-camera_gcrf_m, Atmosphere.EARTH_MEAN_RADIUS_M, to_moon_m, moon_radius_m)
	if not Atmosphere.moon_terrain_sun_applies(camera_gcrf_m, moon_gcrf_m):
		_earth_horizon_ground_height_m = 0.0
		return mean
	var gcrf_to_moon_fixed := moon.fixed_to_gcrf.transposed()
	var eye_fixed := gcrf_to_moon_fixed * -to_moon_m
	var earth_fixed := gcrf_to_moon_fixed * -moon_gcrf_m
	if eye_fixed.distance_to(_earth_horizon_eye_fixed_m) > EARTH_HORIZON_EYE_STEP_M \
			or earth_fixed.normalized().distance_to(_earth_horizon_earth_dir) > EARTH_HORIZON_EARTH_STEP_RAD:
		_earth_horizon_eye_fixed_m = eye_fixed
		_earth_horizon_earth_dir = earth_fixed.normalized()
		_earth_horizon_fraction = MoonSunHorizon.disc_visible_fraction(core, eye_fixed, earth_fixed,
				Atmosphere.EARTH_MEAN_RADIUS_M)
		var ground: Dictionary = core.surface_samples("moon", PackedFloat64Array([rad_to_deg(asin(eye_fixed.normalized().z))]),
				PackedFloat64Array([rad_to_deg(atan2(eye_fixed.y, eye_fixed.x))]))
		_earth_horizon_ground_height_m = float(ground["height_m"][0]) if bool(ground["present"]) else 0.0
	return _earth_horizon_fraction if is_finite(_earth_horizon_fraction) else mean


## Draws earth_clouds.gdshader on a sub-pixel quad in front of the camera for a few frames at load, so the scaled-space
## Earth's first switch to the far-field clouds does not compile its pipeline in the middle of a frame (up to 5 ms).
func _warm_earth_clouds_shader() -> void:
	var material := _earth_material.duplicate() as ShaderMaterial
	material.shader = _earth_clouds_shader
	var quad := QuadMesh.new()
	quad.size = Vector2(1.0e-3, 1.0e-3)
	_warmup_mesh = MeshInstance3D.new()
	_warmup_mesh.mesh = quad
	_warmup_mesh.material_override = material
	_warmup_mesh.position = Vector3(0.0, 0.0, -2.0)
	camera.add_child(_warmup_mesh)
	_warmup_frames = 4


func _setup_bodies(data_dir: String) -> void:
	await LoadingScreen.step("Erdtextur laden")
	_earth_material.shader = _earth_shader
	_earth_material.set_shader_parameter("day_texture", _load_texture(data_dir.path_join("textures/earth_day.jpg")))
	_earth_material.set_shader_parameter("moon_radius_m", MOON_RADIUS_M)
	_earth_material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	_sky_material.set_shader_parameter("day_texture", _earth_material.get_shader_parameter("day_texture"))
	_sky_material.set_shader_parameter("day_texture_bound", _earth_material.get_shader_parameter("day_texture") != null)
	earth.name = "Earth"
	earth.setup(EARTH_SEMI_MAJOR_AXIS_M, EARTH_FLATTENING, _earth_material)
	add_child(earth)
	await LoadingScreen.step("Erdgelände und Satellitenbilder einrichten")
	_setup_earth_terrain(data_dir)
	await LoadingScreen.step("Startplätze und Fangtürme aufbauen")
	for site: Dictionary in sites:
		earth.add_child(_make_pad(site))
	for site: Dictionary in sites:
		var geometry: Dictionary = flight.sim.site_tower(site["id"])
		if geometry.get("present", false):
			var view := TowerView.new()
			view.site_id = site["id"]
			earth.add_child(view)
			view.build(geometry)
			towers.append(view)

	await LoadingScreen.step("Mond und Mondgelände aufbauen")
	_moon_material.shader = load("res://shaders/moon.gdshader")
	_moon_material.set_shader_parameter("albedo_texture", _load_texture(data_dir.path_join("textures/moon_color.png")))
	_moon_material.set_shader_parameter("earth_radius_m", EARTH_MEAN_RADIUS_M)
	moon.name = "Moon"
	moon.setup(MOON_RADIUS_M, 0.0, _moon_material)
	add_child(moon)
	_setup_moon_terrain()

	var sphere := SphereMesh.new()
	sphere.radius = 1.0
	sphere.height = 2.0
	sun_disk.mesh = sphere
	sun_disk.material_override = sun_disc_material()
	sun_disk.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	sun_disk.name = "Sun"
	add_child(sun_disk)
	sun_light.light_energy = 1.0
	add_child(sun_light)
	moon_light.shadow_enabled = false
	moon_light.visible = false
	add_child(moon_light)


## The Earth's terrain quadtree: heights from the core's Earth terrain (the physics' surface; without data/ the flat
## rule), imagery from data/imagery, drawn with terrain_earth.gdshader.
func _setup_earth_terrain(data_dir: String) -> void:
	terrain_loaded = core.load_terrain(Paths.config_dir())  # once, on the main thread, before any worker build
	var heights := HeightSource.new()
	_earth_heights = heights
	heights.body = "earth"
	heights.core = core
	heights.sites = sites
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain_earth.gdshader")
	material.set_shader_parameter("day_texture", _earth_material.get_shader_parameter("day_texture"))
	material.set_shader_parameter("day_texture_bound", _earth_material.get_shader_parameter("day_texture") != null)
	material.set_shader_parameter("moon_radius_m", MOON_RADIUS_M)
	material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	ShaderGlobals.set_value("water_wave_steepness", WaterWaves.STEEPNESS)  # constant: set once
	earth_terrain.name = "EarthTerrain"
	earth_terrain.camera_global = "earth_terrain_camera"  # project.godot [shader_globals], read by terrain_earth.gdshader
	earth.add_child(earth_terrain)
	earth_terrain.setup(heights, ImagerySource.new(data_dir.path_join("imagery/earth/index.toml")),
			EARTH_SEMI_MAJOR_AXIS_M, material)


## The Moon's terrain quadtree: heights from the core's lunar terrain (LDEM_16 everywhere, the 20 m site tiles; the
## cached instance the lunar missions fly on; without data/ the mean sphere) in MOON_ME, the frame of LOLA and of
## moon.fixed_to_gcrf; no imagery: terrain_moon.gdshader takes the albedo from moon_color through the global UV.
## After _setup_earth_terrain (which loads the core's terrain).
func _setup_moon_terrain() -> void:
	var heights := HeightSource.new()
	_moon_heights = heights
	heights.body = "moon"
	heights.core = core
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain_moon.gdshader")
	material.set_shader_parameter("albedo_texture", _moon_material.get_shader_parameter("albedo_texture"))
	material.set_shader_parameter("earth_radius_m", EARTH_MEAN_RADIUS_M)
	material.set_shader_parameter("sun_radius_m", SUN_RADIUS_M)
	moon_terrain.name = "MoonTerrain"
	moon_terrain.camera_global = "moon_terrain_camera"  # project.godot [shader_globals], read by terrain_moon.gdshader
	moon.add_child(moon_terrain)
	moon_terrain.setup(heights, null, MOON_RADIUS_M, material)


## Creates, re-poses or removes the drone ship for the flight's landing zone (FlightSim.landing_zone; {}: no flight).
func _update_drone_ship(zone: Dictionary) -> void:
	if not zone.get("has_deck", false):
		if drone_ship != null:
			drone_ship.queue_free()
			drone_ship = null
		return
	if drone_ship != null and not drone_ship.matches(zone):  # another mission's ship (or a re-placed one)
		drone_ship.queue_free()
		drone_ship = null
	if drone_ship == null:
		drone_ship = DroneShipView.new()
		drone_ship.name = "DroneShip"
		earth.add_child(drone_ship)
		drone_ship.build(zone)
	else:
		drone_ship.update_pose(zone)


## Concrete pad disc at a launch site (Earth-fixed, top surface at the site height).
func _make_pad(site: Dictionary) -> MeshInstance3D:
	var disc := CylinderMesh.new()
	disc.top_radius = 35.0
	disc.bottom_radius = 37.0
	disc.height = 2.0
	var concrete := LitAerial.material(Color(0.62, 0.61, 0.58), 0.9, 0.0)
	var pad := MeshInstance3D.new()
	pad.name = "Pad_" + str(site["id"])
	pad.mesh = disc
	pad.material_override = concrete
	var lat := deg_to_rad(site["latitude_deg"])
	var lon := deg_to_rad(site["longitude_deg"])
	var up := Frames.to_render(Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)))
	var east := Frames.to_render(Vector3(-sin(lon), cos(lon), 0.0))
	var center := HeightSource.geodetic_to_fixed(site["latitude_deg"], site["longitude_deg"],
			site["height_m"] - 1.0 + PAD_TOP_LIFT_M)
	pad.transform = Transform3D(Basis(east, up, east.cross(up)), Frames.to_render(center))
	return pad


## Screen-space label for the map view, where bodies are only a few pixels large (MapLabel).
func _setup_label(label: MapLabel, text: String) -> void:
	label.setup(text, Color(0.75, 0.85, 1.0))
	label.visible = false
	add_child(label)


## The sky shader (shaders/sky.gdshader) draws the background from the precomputed atmosphere; its radiance cubemap
## (sky plus the lit ground below the horizon) is the ambient light and the reflections of the standard materials: sky
## light, ground bounce and earthshine from the same model. Black until the tables are loaded (atmosphere_enabled).
func _setup_environment() -> void:
	_sky_material.shader = load("res://shaders/sky.gdshader")
	_sky_material.set_shader_parameter("radiance_moon_albedo", Atmosphere.MOON_MEAN_TEXTURE_ALBEDO)
	_sky_material.set_shader_parameter("radiance_earth_min_radius_rad", RadianceFarEarth.MIN_RADIUS_RAD)
	var sky := Sky.new()
	sky.sky_material = _sky_material
	# [call, Task 5 fix round 1: performance] The radiance cubemap (ambient + reflections) is refreshed over a few
	# frames at 128 px: the sky changes slowly at flight rates, a few frames of lag in the ambient are invisible, and
	# REALTIME would force 256 px every frame. The background pass is unaffected (drawn per pixel each frame). The sky
	# reads global uniforms only, so Godot redraws the cube only after a material write: _update_sky_radiance.
	sky.process_mode = Sky.PROCESS_MODE_INCREMENTAL
	sky.radiance_size = Sky.RADIANCE_SIZE_128
	var environment := Environment.new()
	_environment = environment
	environment.background_mode = Environment.BG_SKY
	environment.sky = sky
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	environment.ambient_light_sky_contribution = 1.0
	environment.ambient_light_energy = 1.0
	environment.reflected_light_source = Environment.REFLECTION_SOURCE_SKY
	environment.tonemap_mode = Environment.TONE_MAPPER_AGX
	environment.glow_enabled = true
	environment.glow_hdr_threshold = 4.0
	environment.glow_intensity = 0.6
	var world_environment := WorldEnvironment.new()
	world_environment.environment = environment
	# The cloud pass (CloudEffect, before the transparent pass) and night vision's rod step (EyeAdaptation,
	# VisionEffect: after the transparent pass, before glow and AgX).
	var compositor := Compositor.new()
	compositor.compositor_effects = [clouds.shadow_effect, clouds.effect, atmosphere.vision_effect]
	world_environment.compositor = compositor
	add_child(world_environment)


static func _load_texture(path: String) -> Texture2D:
	var image := Image.load_from_file(path)
	if image == null:
		push_error("World: missing texture %s - run tools/data_pipeline/prepare.py" % path)
		return null
	image.generate_mipmaps()
	return ImageTexture.create_from_image(image)
