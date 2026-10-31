# The sky's radiance cubemap (Godot's ambient light and reflections) follows the Sun and the place. GPU, windowed.
# Three modes, exit code 1 on a failed expectation:
# - Phases (default): one world, tables loaded first, then a chain of clock / site jumps without any other help. A white
#   Lambertian sphere with every direct light off, so its pixels are the cube alone; linear display, no glow, no rod step,
#   so a pixel / scene_per_cd_m2 is a luminance. Each lit phase against its own reference (a stale cube misses by orders of
#   magnitude): pad by day (the core's sky irradiance), moonlit night (the Moon's sky), orbit (an estimate of the ground in
#   view), the Moon by day (the regolith's analytic radiance), under the Earth (earthshine), the Earth at the lunar horizon
#   (its true visible fraction), an eye in terrain shadow at IM-2, overcast, back at the pad (the same pixels as before).
#   godot --path game --script res://tests/sky_radiance_check.gd [-- --only=<phase>] [--shots=<dir>]
# - --timelapse: a sunset at warp 120 through twilight: no flicker (largest frame step), the cube live (against the sky's
#   irradiance all the way; a frozen cube fails). --steady: the far Earth moving through the cube's texels, the earthshine
#   steady.
# - --flight --utc=<day or night> --expect=lit|dark: a real flight session (VehicleRig, starship_leo on the pad, clock
#   paused, every hull white): the centre pixel of the hull's shade side (camera on the side away from the Sun). With
#   --live the clock runs through the ascent: the cube must refresh and follow the sky at the eye.
extends SceneTree

const FRAMES_PER_VIEW := 50  # [E] two refilter passes of the incremental cube (8 to 16 frames each) and margin
const RADIUS_M := 1.0
const PROBE_LAYER := 1 << 19
const DIST_M := 12.0
const EYE_HEIGHT_M := 20.0  # clear of terrain with the view 30 deg down
const LOOK_DEG := 30.0
const DARK_LEVEL := 3.0 / 255.0  # sRGB code value: what "~0" means on the display
const LIT_LEVEL := 8.0 / 255.0
# [E] The plate against sky_lux / pi at the pad (measured 0.906): the roughest mip is a cosine convolution sampled with a
# finite GGX sample set. A stale cube misses by orders of magnitude.
const IRRADIANCE_TOLERANCE := 0.15
# [E] The small sources (the Earth's disc, the Moon's sky at 8 bit): sampling and quantisation of a dark pixel.
const SMALL_SOURCE_TOLERANCE := 0.30
# [E] The orbit's estimate is a nadir point with one albedo over the whole cap: it holds within this factor.
const ORBIT_ESTIMATE_FACTOR := 2.0
# [E] Back at the pad, the same cube: one code value of rounding and one of the refilter's sample pattern.
const SAME_CODE_VALUES := 2.0 / 255.0
const STARBASE := {"lat": 25.9966694, "lon": -97.153728}
const IM2 := {"lat": -84.7906, "lon": 29.1957}
const EARTH_REFERENCE_ELEVATION_DEG := 10.0
const LIMB_UTC := "2026-11-12T06:00:00"  # three days after new Moon: a nearly full Earth, the limb site in the night
const OVERCAST_SLAB := [1000.0, 2000.0, 1.0e-4, 1.0]  # bottom m, top m, liquid kg/m^3, fraction: a thin bright deck
const OVERCAST_TOLERANCE := 0.15
const TIMELAPSE_WARP := 120.0
const TIMELAPSE_FRAMES := 900  # ~7.5 deg of the Sun: from +4 deg into twilight
const TIMELAPSE_READ_LEVEL := 16.0 / 255.0  # below this a code value is too coarse for a ratio
# [E] Largest frame-to-frame step in sRGB code values that reads as a smooth change, not as a flicker.
const TIMELAPSE_MAX_STEP := 6.0 / 255.0
const TIMELAPSE_GAIN_STEP_LOG := 0.1  # [E] a frame's change of the gain that is a jump of its target, not the adaptation
const TIMELAPSE_SETTLE_FRAMES := 40  # [E] the refilter's 8 to 16 frames, twice, and margin
# [E] The ratio (cube's plate / sky_lux) drifts legitimately as the ground and the colour of the twilight change; a frozen
# cube leaves this band within seconds (the sky falls by a decade per 4 deg of the Sun).
const LIVE_RATIO_BAND := 1.6
const LIVE_FLIGHT_FRAMES := 900
const LIVE_WARP := 4.0  # ~1 min of the ascent, to ~20 km
const LIVE_MIN_REFRESHES := 5
const LIVE_MIN_CHANGE_LOG := 0.2  # [E] the sky overhead falls far more than 20 % over that climb
const STEADY_STEPS := 10
const STEADY_STEP_H := 1  # the Earth moves ~0.55 deg per hour in the cube's axes: 5.5 deg, over 3 texels
const STEADY_MAX_RANGE := 0.15  # [E] (max - min) / mean of the earthshine, reached over hours: no visible pumping
const SCAN_STEP_H := 6
const SCAN_STEPS := 120

# label, utc, site (lat, lon, body, eye height), adaptation, what is expected of (up, side, down).
var _phases := [
	{"label": "pad_day", "utc": "2026-10-12T17:30:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": false,
			"check": "irradiance"},
	{"label": "pad_night_no_moon", "utc": "2026-11-09T06:00:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": true,
			"check": "dark"},
	{"label": "pad_night_full_moon", "utc": "2026-11-24T06:00:00", "lat": STARBASE.lat, "lon": STARBASE.lon,
			"adapt": true, "check": "moon_sky"},
	{"label": "orbit_day", "utc": "2026-10-12T17:30:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": false,
			"height": 400000.0, "check": "orbit"},
	{"label": "moon_day", "utc": "2026-11-24T06:00:00", "lat": 0.0, "lon": 0.0, "body": "moon", "adapt": true,
			"check": "regolith"},
	{"label": "moon_night_under_earth", "utc": "2026-11-09T06:00:00", "lat": 0.0, "lon": 0.0, "body": "moon",
			"adapt": true, "check": "earthshine"},
	{"label": "moon_night_earth_below_horizon", "utc": "2026-11-17T12:00:00", "lat": 0.0, "lon": -100.0, "body": "moon",
			"adapt": true, "check": "dark"},
	{"label": "earth_elevation_ref", "utc": LIMB_UTC, "limb_deg": EARTH_REFERENCE_ELEVATION_DEG, "body": "moon",
			"adapt": true, "check": "far_earth"},
	{"label": "earth_elevation_+1", "utc": LIMB_UTC, "limb_deg": 1.0, "body": "moon", "adapt": true, "check": "far_earth"},
	{"label": "earth_elevation_-1", "utc": LIMB_UTC, "limb_deg": -1.0, "body": "moon", "adapt": true, "check": "far_earth"},
	{"label": "im2_sunlit", "shadow": false, "lat": IM2.lat, "lon": IM2.lon, "body": "moon", "adapt": true,
			"check": "regolith"},
	{"label": "im2_terrain_shadow", "shadow": true, "lat": IM2.lat, "lon": IM2.lon, "body": "moon", "adapt": true,
			"check": "regolith"},
	{"label": "pad_day_again", "utc": "2026-10-12T17:30:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": false,
			"check": "same_as_first"},
	{"label": "pad_day_overcast", "utc": "2026-10-12T17:30:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": false,
			"overcast": true, "check": "overcast"},
]

var _args := {}
var _main: Node
var _world: World
var _rig: VehicleRig
var _frames := 0
var _flight_frame := 0
var _started := false
var _phase := -1
var _wait := 0
var _looking_up := false
var _up := Color.BLACK
var _first := {}
var _earth_reference := 0.0
var _sphere: MeshInstance3D
var _failures := 0
var _timelapse_frame := -1
var _timelapse_last := -1.0
var _timelapse_max_step := 0.0
var _timelapse_gain := 0.0
var _timelapse_gain_steps := 0
var _timelapse_settle := 0
var _live_ratio_min := INF
var _live_ratio_max := 0.0
var _serial_at_start := 0
var _live_first := 0.0
var _steady_values: Array[float] = []


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	if _args.has("only"):
		_phases = _phases.filter(func(p: Dictionary) -> bool: return p["label"] == _args["only"])
	if _args.has("timelapse") or _args.has("steady"):
		_phases.clear()
	if _args.has("steady"):
		for k in range(STEADY_STEPS):
			_phases.append({"label": "steady_%d" % k, "utc": "2026-11-09T%02d:00:00" % (6 + k * STEADY_STEP_H), "lat": 0.0,
					"lon": 0.0, "body": "moon", "adapt": true, "check": "steady"})
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.weather.enabled = false
	_frames += 1
	if _frames < 20 or _world.atmosphere.loading:  # the order of a real session: the tables first, then the view
		return false
	if not _started:
		_started = true
		TimePanel.show_frame_rate = false
		if _args.has("flight"):
			_start_flight()
		else:
			_add_sphere()
			_next_phase()
		return false
	if _args.has("flight"):
		return _flight_step()
	if _timelapse_frame >= 0:
		return _timelapse_step()
	_world.clock.warp = 0.0
	_face_earth()
	_place_sphere()
	_wait += 1
	if _wait < FRAMES_PER_VIEW:
		return false
	var image := root.get_viewport().get_texture().get_image()
	var render_up := _render_up()
	var to_cam := (_world.camera.global_position - _sphere.global_position).normalized()
	if not _looking_up:
		_up = _read(image, render_up)
		_set_view(LOOK_DEG)
		_looking_up = true
		_wait = FRAMES_PER_VIEW - 5  # the same cube: only the view changes
		return false
	var down := _read(image, -render_up)
	if _args.has("shots"):
		DirAccess.make_dir_recursive_absolute(String(_args["shots"]))
		image.save_png(String(_args["shots"]).path_join(String(_phases[_phase]["label"]) + ".png"))
	var side_n := (to_cam - render_up * to_cam.dot(render_up)).normalized()
	_finish_phase(_up, _read(image, side_n), down)
	return _next_phase()


func _render_up() -> Vector3:
	var up: Vector3 = _world.camera.site_up_gcrf
	return Frames.to_render(up).normalized()


func _add_sphere() -> void:
	var mesh := SphereMesh.new()
	mesh.radius = RADIUS_M
	mesh.height = 2.0 * RADIUS_M
	mesh.radial_segments = 64
	mesh.rings = 32
	var material := LitAerial.material(Color.WHITE, 1.0, 0.0)
	material.set_shader_parameter("specular", 0.0)
	_sphere = MeshInstance3D.new()
	_sphere.mesh = mesh
	_sphere.material_override = material
	_sphere.top_level = true
	root.add_child(_sphere)
	# The sphere shows the cube alone: on a layer the direct lights do not reach.
	_sphere.layers = PROBE_LAYER
	_world.sun_light.light_cull_mask &= ~PROBE_LAYER
	_world.moon_light.light_cull_mask &= ~PROBE_LAYER


func _place_sphere() -> void:
	var cam := root.get_viewport().get_camera_3d()
	if cam != null:
		_sphere.global_position = cam.global_position - cam.global_transform.basis.z * DIST_M


func _set_view(elevation_deg: float) -> void:
	_world.camera.azimuth = deg_to_rad(315.0)
	_world.camera.elevation = deg_to_rad(elevation_deg)
	_world.camera.fov = 40.0


## The Earth-horizon phases read the side facing the Earth: the camera's back to the Earth.
func _face_earth() -> void:
	if not _phases[_phase].has("limb_deg"):
		return
	var camera := _world.camera
	var to_earth: Vector3 = -camera.position_gcrf_m.normalized()
	_world.camera.azimuth = atan2(-to_earth.dot(camera.site_east_gcrf), -to_earth.dot(camera.site_north_gcrf))


func _next_phase() -> bool:
	_phase += 1
	_wait = 0
	_looking_up = false
	if _phase >= _phases.size():
		if _args.has("timelapse"):
			_start_timelapse()
			return false
		if _args.has("steady"):
			_finish_steady()
		print("sky_radiance_check: %s (%d failures)" % ["PASS" if _failures == 0 else "FAIL", _failures])
		quit(1 if _failures > 0 else 0)
		return true
	var p: Dictionary = _phases[_phase]
	_go_to(p)
	return false


## A site on the Moon with the Earth at elevation_deg above the mean horizon and the Sun well below: on the far side of the
## sub-Earth point from the Sun.
func _limb_site(utc: String, elevation_deg: float) -> Dictionary:
	var core: StarsimCore = _world.core
	var t: float = core.utc_to_tdb_s(utc)
	var to_fixed: Basis = core.moon_fixed_to_gcrf(t).transposed()
	var moon: Vector3 = core.body_position_gcrf_m("moon", t)
	var earth := (to_fixed * -moon).normalized()
	var sun: Vector3 = (to_fixed * (core.apparent_body_position_gcrf_m("sun", t, Vector3.ZERO, Vector3.ZERO) - moon)).normalized()
	var along := (sun - earth * sun.dot(earth)).normalized()
	var theta := deg_to_rad(90.0 - elevation_deg)
	var site := earth * cos(theta) - along * sin(theta)
	return {"lat": rad_to_deg(asin(site.z)), "lon": rad_to_deg(atan2(site.y, site.x))}


## The first instant from the mission's month on with the Sun above the mean horizon at the site, and hidden by the terrain
## (shadow) or fully above it (not shadow).
func _im2_utc(shadow: bool) -> String:
	var core: StarsimCore = _world.core
	var surface: Dictionary = core.surface_samples("moon", PackedFloat64Array([IM2.lat]), PackedFloat64Array([IM2.lon]))
	var lat := deg_to_rad(IM2.lat)
	var lon := deg_to_rad(IM2.lon)
	var radius: float = Atmosphere.MOON_RADIUS_M + float(surface["height_m"][0]) + EYE_HEIGHT_M
	var point := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)) * radius
	var unix := Time.get_unix_time_from_datetime_string("2026-11-01T00:00:00")
	for k in range(SCAN_STEPS):
		var utc := Time.get_datetime_string_from_unix_time(int(unix) + k * SCAN_STEP_H * 3600)
		var t: float = core.utc_to_tdb_s(utc)
		var moon: Vector3 = core.body_position_gcrf_m("moon", t)
		var sun: Vector3 = core.moon_fixed_to_gcrf(t).transposed() \
				* (core.apparent_body_position_gcrf_m("sun", t, Vector3.ZERO, Vector3.ZERO) - moon)
		var fraction := MoonSunHorizon.sun_visible_fraction(core, point, sun)
		if sun.normalized().dot(point.normalized()) > 0.0 and fraction == (0.0 if shadow else 1.0):
			return utc
	push_error("sky_radiance_check: no instant found at IM-2")
	return "2026-11-01T00:00:00"


func _go_to(p: Dictionary) -> void:
	if not p.has("utc"):
		p["utc"] = _im2_utc(p["shadow"])
	if p.has("limb_deg"):
		p.merge(_limb_site(p["utc"], p["limb_deg"]))
	var environment: Environment = _world._environment
	# Scene values on the display: no tone curve, no glow, no rod step; the luminance comes back through scene_per_cd_m2.
	environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	environment.glow_enabled = false
	_world.atmosphere.vision_effect.enabled = false
	_world.atmosphere.eye_adaptation_enabled = p["adapt"]
	_world.clock.warp = 0.0
	_world.clock.set_time(_world.core.utc_to_tdb_s(p["utc"]))
	_world.set_focus(World.FOCUS_EARTH)
	var body: String = p.get("body", "earth")
	var surface: Dictionary = _world.core.surface_samples(body, PackedFloat64Array([p["lat"]]), PackedFloat64Array([p["lon"]]))
	_world.set_ground_site({"id": "custom", "name": "custom", "body": body, "latitude_deg": p["lat"],
			"longitude_deg": p["lon"], "height_m": surface["height_m"][0] if bool(surface["present"]) else 0.0,
			"eye_height_m": p.get("height", EYE_HEIGHT_M)})
	_set_view(-LOOK_DEG)
	_world.atmosphere.snap_adaptation()
	if p.get("overcast", false):
		_set_overcast()


func _set_overcast() -> void:
	var file := OS.get_temp_dir().path_join("starsim_sky_radiance_slab_%d.wthr" % OS.get_process_id())
	var slab: Array = OVERCAST_SLAB
	_world.core.weather_write_slab_snapshot(file, slab[0], slab[1], slab[2], slab[3])
	var snapshot: Dictionary = _world.core.weather_load_snapshot(file)
	_world.clouds.wait_until_ready()
	_world.clouds.show_now(snapshot, snapshot, 0.0)
	_world.core.weather_set_active(file, file, 0.0)


func _read(image: Image, n: Vector3) -> Color:
	var cam := root.get_viewport().get_camera_3d()
	var px := cam.unproject_position(_sphere.global_position + n.normalized() * RADIUS_M * 0.999)
	var sum := Color(0, 0, 0, 0)
	var count := 0
	for dx in range(-2, 3):
		for dy in range(-2, 3):
			var x := int(px.x) + dx
			var y := int(px.y) + dy
			if x >= 0 and y >= 0 and x < image.get_width() and y < image.get_height():
				sum += image.get_pixel(x, y)
				count += 1
	return sum / float(maxi(count, 1))


func _fmt(c: Color) -> String:
	return "(%3d %3d %3d)" % [roundi(c.r * 255.0), roundi(c.g * 255.0), roundi(c.b * 255.0)]


func _expect(ok: bool, what: String) -> void:
	print("sky_radiance_check:   %s %s" % ["ok  " if ok else "FAIL", what])
	if not ok:
		_failures += 1


## A display pixel as a luminance (cd/m^2): linear tone curve, so the pixel is scene_per_cd_m2 x the luminance.
func _cd(c: Color) -> float:
	return c.srgb_to_linear().g / _world.atmosphere.scene_per_cd_m2


func _expect_ratio(what: String, measured_cd: float, expected_cd: float, tolerance: float) -> void:
	var ratio := measured_cd / maxf(expected_cd, 1.0e-30)
	_expect(absf(ratio - 1.0) <= tolerance, "%s: measured %s cd/m^2, expected %s: ratio %.3f (+- %.2f)" % [what,
			String.num_scientific(measured_cd), String.num_scientific(expected_cd), ratio, tolerance])


func _sky_cd() -> float:
	var gcrf_to_fixed: Basis = _world.earth.fixed_to_gcrf.transposed()
	var eye_itrf: Vector3 = gcrf_to_fixed * _world.camera.position_gcrf_m
	var up_itrf: Vector3 = eye_itrf.normalized()
	var sun_itrf: Vector3 = (gcrf_to_fixed * _world.sun_apparent_from_earth_m).normalized()
	return _world.core.atmosphere_illuminance(eye_itrf, up_itrf, sun_itrf)["sky_lux"].y / PI


## The regolith's radiance in the lower half of the cube, the analytic sum of the Moon shaders' Sun and earthshine terms over
## the cosine-weighted cap, each x the part of its disc the eye sees (the cube's stated stand-in for the terrain's shadows).
func _regolith_cd(sun_visible: float, earth_visible: float) -> float:
	var a := _world.atmosphere
	var eye: Vector3 = _world.camera.position_gcrf_m
	var moon: Vector3 = _world.moon_gcrf_m
	var sun: Vector3 = _world.sun_apparent_from_earth_m
	var radius := Atmosphere.moon_ground_radius((moon - eye).length())
	var d_au := sun.length() / Atmosphere.ASTRONOMICAL_UNIT_M
	var sun_part: float = a._moon_in_view_luminance(eye, moon, sun, radius, 0.0, d_au)
	var all_part: float = a._moon_in_view_luminance(eye, moon, sun, radius, a.last_earthshine_lux, d_au)
	return sun_part * sun_visible + (all_part - sun_part) * earth_visible


## The Moon's sky at the pad: the sky's irradiance with the Moon as its source, x the Moon's relative illuminance / pi.
func _moon_sky_cd() -> float:
	var a := _world.atmosphere
	var gcrf_to_fixed: Basis = _world.earth.fixed_to_gcrf.transposed()
	var eye_itrf: Vector3 = gcrf_to_fixed * _world.camera.position_gcrf_m
	var to_moon: Vector3 = (gcrf_to_fixed * _world.moon_gcrf_m).normalized()
	return _world.core.atmosphere_illuminance(eye_itrf, eye_itrf.normalized(), to_moon)["sky_lux"].y / PI \
			* a.moonlight_relative_value


## The Earth in view from orbit: the ground under the eye, lit by the Sun and the sky at the ground's albedo, through the air
## with the in-scattered light on the way, x the view factor of the Earth's cap (R / r)^2.
func _orbit_cd() -> float:
	var a := _world.atmosphere
	var gcrf_to_fixed: Basis = _world.earth.fixed_to_gcrf.transposed()
	var eye_itrf: Vector3 = gcrf_to_fixed * _world.camera.position_gcrf_m
	var r := eye_itrf.length()
	var ground_itrf: Vector3 = eye_itrf / r * Atmosphere.EARTH_MEAN_RADIUS_M
	var sun_itrf: Vector3 = (gcrf_to_fixed * _world.sun_apparent_from_earth_m).normalized()
	var lux: Dictionary = _world.core.atmosphere_illuminance(ground_itrf, ground_itrf.normalized(), sun_itrf)
	var path: Dictionary = _world.core.atmosphere_sky_luminance_to_point(eye_itrf, ground_itrf, sun_itrf)
	var albedo := float(a.parameters["ground_albedo"]) if a.parameters.has("ground_albedo") else 0.1
	var view := pow(Atmosphere.EARTH_MEAN_RADIUS_M / r, 2.0)
	return (albedo * (lux["sun_lux"].y + lux["sky_lux"].y) / PI * path["transmittance"].y + path["luminance"].y) * view


## The Earth's true visible fraction at the eye: the terrain's horizon where the core has it, else the ground sphere.
func _earth_true_fraction() -> float:
	var eye: Vector3 = _world.camera.position_gcrf_m
	var moon: Vector3 = _world.moon_gcrf_m
	var to_fixed: Basis = _world.moon.fixed_to_gcrf.transposed()
	var terrain := MoonSunHorizon.disc_visible_fraction(_world.core, to_fixed * (eye - moon), to_fixed * -moon,
			Atmosphere.EARTH_MEAN_RADIUS_M)
	if is_finite(terrain):
		return terrain
	return 1.0 - Atmosphere.occulted_fraction(-eye, Atmosphere.EARTH_MEAN_RADIUS_M, moon - eye,
			Atmosphere.moon_ground_radius((moon - eye).length()))


func _finish_phase(up: Color, side: Color, down: Color) -> void:
	var p: Dictionary = _phases[_phase]
	print("sky_radiance_check: %s %s  up %s  side %s  down %s  gain %.1f  scene_per_cd %.6f  moon_light %.8f" % [p["label"], p["utc"],
			_fmt(up), _fmt(side), _fmt(down), _world.atmosphere.exposure_gain, _world.atmosphere.scene_per_cd_m2,
			_world.atmosphere.moonlight_relative_value if _world.atmosphere.moonlight_active else 0.0])
	var eye_moon: Vector3 = _world.camera.position_gcrf_m - _world.moon_gcrf_m
	print("sky_radiance_check:   Moon zenith . Sun %+.3f, . Earth %+.3f, height above the mean sphere %.0f m, cube radius %.0f m" % [
			eye_moon.normalized().dot((_world.sun_apparent_from_earth_m - _world.moon_gcrf_m).normalized()),
			eye_moon.normalized().dot((-_world.moon_gcrf_m).normalized()), eye_moon.length() - World.MOON_RADIUS_M,
			_world._sky_material.get_shader_parameter("radiance_moon_radius_m")])
	if p.get("body", "") == "moon":
		_expect(not _world.moon_light.visible, "no light from the Moon's centre through the ground")
	match p["check"]:
		"dark":
			_expect(maxf(maxf(up.g, side.g), down.g) <= DARK_LEVEL, "no light, the shade ~0")
		"irradiance":
			_expect(side.g >= LIT_LEVEL and down.g >= LIT_LEVEL, "shade side and ground side lit")
			_expect_ratio("up plate vs sky_lux / pi", _cd(up), _sky_cd(), IRRADIANCE_TOLERANCE)
			_first = {"up": up, "side": side, "down": down}
		"same_as_first":
			var same := absf(up.g - _first.up.g) <= SAME_CODE_VALUES and absf(side.g - _first.side.g) <= SAME_CODE_VALUES \
					and absf(down.g - _first.down.g) <= SAME_CODE_VALUES
			_expect(same, "the same pixels as the first pad_day: %s %s %s" % [_fmt(_first.up), _fmt(_first.side), _fmt(_first.down)])
		"moon_sky":
			_expect_ratio("up plate vs the Moon's sky", _cd(up), _moon_sky_cd(), SMALL_SOURCE_TOLERANCE)
		"orbit":
			var ratio := _cd(down) / _orbit_cd()
			_expect(ratio >= 1.0 / ORBIT_ESTIMATE_FACTOR and ratio <= ORBIT_ESTIMATE_FACTOR,
					"down plate %.0f cd/m^2 vs the estimate %.0f: ratio %.3f (within a factor %.1f)" % [_cd(down), _orbit_cd(), ratio,
					ORBIT_ESTIMATE_FACTOR])
		"regolith":
			var sun_visible := 1.0
			print("sky_radiance_check:   ground Sun %.3f Earth %.3f, eye's terrain Sun %.3f, Earth true %.3f" % [
					_world._sky_material.get_shader_parameter("radiance_moon_sun_visible"),
					_world._sky_material.get_shader_parameter("radiance_moon_earth_visible"),
					_world._camera_terrain_sun_fraction, _earth_true_fraction()])
			if p.get("shadow", false):
				# No Sun on the ground: at most the earthshine's plate (the disc low in the sky leaks into the lower lobe).
				var limit: float = _world.atmosphere.last_earthshine_lux / PI
				_expect(_cd(down) <= limit, "an eye in terrain shadow: no sunlit ground: down plate %.3f cd/m^2 <= %.3f" % [_cd(down), limit])
			else:
				_expect_ratio("down plate vs the regolith", _cd(down), _regolith_cd(sun_visible, _earth_true_fraction()),
						SMALL_SOURCE_TOLERANCE)
		"earthshine":
			_expect_ratio("up plate vs last_earthshine_lux / pi x cos(Earth's zenith angle)", _cd(up),
					_world.atmosphere.last_earthshine_lux / PI * eye_moon.normalized().dot((-_world.moon_gcrf_m).normalized()),
					SMALL_SOURCE_TOLERANCE)
		"far_earth":
			var fraction := _earth_true_fraction()
			var value := _cd(side) / maxf(_world.atmosphere.last_earthshine_lux, 1.0e-30)
			print("sky_radiance_check:   Earth's true visible fraction %.3f, side plate / earthshine %.5f" % [fraction, value])
			if p["limb_deg"] == EARTH_REFERENCE_ELEVATION_DEG:
				_earth_reference = value
			if fraction <= 0.0:
				_expect(_cd(side) <= _earth_cd_floor(), "the Earth hidden: no earthshine in the cube")
			elif _earth_reference > 0.0:
				_expect_ratio("side plate vs the true fraction x the reference", value, _earth_reference * fraction,
						SMALL_SOURCE_TOLERANCE)
		"overcast":
			var gcrf_to_fixed: Basis = _world.earth.fixed_to_gcrf.transposed()
			var factor: float = _world.atmosphere.cloud_ambient_factor(gcrf_to_fixed * _world.camera.position_gcrf_m,
					(gcrf_to_fixed * _world.sun_apparent_from_earth_m).normalized())
			_expect(absf(factor - 1.0) > 2.0 * OVERCAST_TOLERANCE, "the deck changes the diffuse light: factor %.3f" % factor)
			_expect_ratio("up plate vs the clear sky x the cloud factor", _cd(up), _cd(_first.up) * factor, OVERCAST_TOLERANCE)
		"steady":
			_steady_values.append(_cd(up) / _world.atmosphere.last_earthshine_lux)


## What "no earthshine" reads: DARK_LEVEL as a luminance at this gain.
func _earth_cd_floor() -> float:
	return Color(0, DARK_LEVEL, 0).srgb_to_linear().g / _world.atmosphere.scene_per_cd_m2


func _finish_steady() -> void:
	var lo: float = _steady_values.min()
	var hi: float = _steady_values.max()
	var mean := 0.0
	for v in _steady_values:
		mean += v / _steady_values.size()
	print("sky_radiance_check: far Earth, up plate / earthshine over %d h: %s" % [STEADY_STEPS * STEADY_STEP_H, str(_steady_values)])
	_expect((hi - lo) / mean <= STEADY_MAX_RANGE, "far Earth steady: range %.3f of the mean (limit %.2f)" % [(hi - lo) / mean,
			STEADY_MAX_RANGE])


## A sunset as a time-lapse: the camera's site at Sun ~+4 deg, warp TIMELAPSE_WARP, on into the twilight.
func _start_timelapse() -> void:
	var p := {"label": "timelapse", "utc": "2026-10-12T23:40:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": true,
			"check": "none"}
	_phases.append(p)
	_go_to(p)
	_set_view(0.0)
	_world.clock.warp = TIMELAPSE_WARP
	_timelapse_frame = 0
	_serial_at_start = _world._radiance_serial


## The cube against the sky at the eye, while the pixel still has resolution (the live check of both running modes).
func _track_live(pixel: Color) -> void:
	if pixel.g < TIMELAPSE_READ_LEVEL:
		return
	var ratio := _cd(pixel) / _sky_cd()
	_live_ratio_min = minf(_live_ratio_min, ratio)
	_live_ratio_max = maxf(_live_ratio_max, ratio)


func _finish_live() -> void:
	_expect(_world._radiance_serial - _serial_at_start >= LIVE_MIN_REFRESHES, "the cube refreshed %d times while the clock ran"
			% (_world._radiance_serial - _serial_at_start))
	_expect(_live_ratio_max > 0.0 and _live_ratio_max / _live_ratio_min <= LIVE_RATIO_BAND,
			"the plate follows the sky: plate / (sky_lux / pi) %.3f .. %.3f (band x %.2f)" % [_live_ratio_min, _live_ratio_max,
			LIVE_RATIO_BAND])


func _timelapse_step() -> bool:
	_place_sphere()
	_timelapse_frame += 1
	var image := root.get_viewport().get_texture().get_image()
	var cam_to := (_world.camera.global_position - _sphere.global_position).normalized()
	var pixel := _read(image, cam_to)
	var value := pixel.g
	# The eye adaptation's glow term sets in with a step at the instant of sunset (eye-adaptation.md, open points): the pixel
	# follows it with the cube's refresh. Not the cube's flicker: that step is counted, its settling left out.
	var gain: float = _world.atmosphere.exposure_gain
	if _timelapse_gain > 0.0 and absf(log(gain / _timelapse_gain)) > TIMELAPSE_GAIN_STEP_LOG:
		_timelapse_gain_steps += 1
		_timelapse_settle = TIMELAPSE_SETTLE_FRAMES
	_timelapse_gain = gain
	_timelapse_settle -= 1
	if _timelapse_frame > 20 and _timelapse_settle < 0:  # the snap of the jump is over
		if _timelapse_last >= 0.0:
			_timelapse_max_step = maxf(_timelapse_max_step, absf(value - _timelapse_last))
		_track_live(pixel)
	if _timelapse_frame > 20:
		_timelapse_last = value
	if _timelapse_frame % 60 == 0:
		print("sky_radiance_check: timelapse frame %d value %.3f gain %.1f plate / sky %.3f serial %d" % [_timelapse_frame, value,
				_world.atmosphere.exposure_gain, _cd(pixel) / _sky_cd(), _world._radiance_serial])
	if _timelapse_frame < TIMELAPSE_FRAMES:
		return false
	_expect(_timelapse_max_step <= TIMELAPSE_MAX_STEP, "largest frame step %.1f code values" % (_timelapse_max_step * 255.0))
	_expect(_timelapse_gain_steps <= 1, "%d step(s) of the eye's gain (the known one at sunset)" % _timelapse_gain_steps)
	_finish_live()
	quit(1 if _failures > 0 else 0)
	return true


func _start_flight() -> void:
	_rig = VehicleRig.new(_world, _args)
	var sim := _rig.start()
	_rig.fly_to_target(sim)
	_rig.hide_ui(_main)
	_world.atmosphere.snap_adaptation()
	_flight_frame = 0


func _whiten(node: Node) -> void:
	var mesh := node as MeshInstance3D
	if mesh != null and mesh.material_override is ShaderMaterial:
		var path: String = (mesh.material_override as ShaderMaterial).shader.resource_path
		if path.contains("vehicle_hull"):
			var white := LitAerial.material(Color.WHITE, 1.0, 0.0)
			white.set_shader_parameter("specular", 0.0)
			mesh.material_override = white
	for child in node.get_children():
		_whiten(child)


func _flight_step() -> bool:
	var live := _args.has("live")
	_flight_frame += 1
	if live and _flight_frame == 1:
		_add_sphere()
	if not live or _flight_frame < FRAMES_PER_VIEW:
		_world.clock.warp = 0.0
	if _flight_frame == 2:
		_rig.place(_world.flight.sim, 0, float(_args.get("station", "30")), Vector3(0, 40, 0), 30.0, 180.0)
	if _flight_frame == 3:
		_whiten(_world)  # the views exist from the first frame of the session on
		if live:
			_world._environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
			_world._environment.glow_enabled = false
	if _flight_frame < FRAMES_PER_VIEW:
		return false
	if live:
		return _live_flight_step()
	var image := root.get_viewport().get_texture().get_image()
	var centre := image.get_pixel(image.get_width() / 2, image.get_height() / 2)
	var expect_lit: bool = _args.get("expect", "lit") == "lit"
	print("sky_radiance_check: flight %s hull shade side %s gain %.1f" % [_args.get("utc", ""), _fmt(centre),
			_world.atmosphere.exposure_gain])
	_expect(centre.g >= LIT_LEVEL if expect_lit else centre.g <= DARK_LEVEL, "expected " + _args.get("expect", "lit"))
	var out: String = _args.get("out", "")
	if out != "":
		DirAccess.make_dir_recursive_absolute(out.get_base_dir())
		image.save_png(out)
	quit(1 if _failures > 0 else 0)
	return true


## The clock runs through the real ascent: height and Sun cross the thresholds, the cube must follow. A probe sphere in front
## of the camera (direct lights off) shows the cube alone: the sky falls with the height, a frozen cube stays as it was.
func _live_flight_step() -> bool:
	_place_sphere()
	if _flight_frame == FRAMES_PER_VIEW:
		_world.clock.warp = LIVE_WARP
		_serial_at_start = _world._radiance_serial
	var image := root.get_viewport().get_texture().get_image()
	var to_cam := (_world.camera.global_position - _sphere.global_position).normalized()
	var pixel := _read(image, to_cam)
	if _flight_frame == FRAMES_PER_VIEW:
		_live_first = _cd(pixel)
	if (_flight_frame - FRAMES_PER_VIEW) % 120 == 0:
		print("sky_radiance_check: live frame %d probe %s %.0f cd/m^2 altitude %.0f m serial %d" % [_flight_frame, _fmt(pixel), _cd(pixel),
				_world.atmosphere.camera_altitude_m, _world._radiance_serial])
	if _flight_frame < FRAMES_PER_VIEW + LIVE_FLIGHT_FRAMES:
		return false
	_expect(_world._radiance_serial - _serial_at_start >= LIVE_MIN_REFRESHES, "the cube refreshed %d times during the ascent"
			% (_world._radiance_serial - _serial_at_start))
	_expect(absf(log(_cd(pixel) / _live_first)) >= LIVE_MIN_CHANGE_LOG, "the probe changed with the climb: %.0f -> %.0f cd/m^2"
			% [_live_first, _cd(pixel)])
	quit(1 if _failures > 0 else 0)
	return true
