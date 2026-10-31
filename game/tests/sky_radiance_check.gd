# The sky's radiance cubemap (Godot's ambient light and reflections) follows the Sun and the place. GPU, windowed.
# Two modes, exit code 1 on a failed expectation:
# - Phases (default): one world, tables loaded first, then a chain of clock / site jumps without any other help. A white
#   Lambertian sphere with every direct light off, so its pixels are the cube alone: pad by day (against the core's sky
#   irradiance), moonless and moonlit night, orbit, the Moon by day, under the Earth, behind the limb, back to the pad.
#   godot --path game --script res://tests/sky_radiance_check.gd [-- --only=<phase>] [--shots=<dir>] [--timelapse]
#   --timelapse: a sunset at warp 120, the largest frame-to-frame step of the ambient (flicker, pumping).
# - --flight --utc=<day or night> --expect=lit|dark: a real flight session (VehicleRig, starship_leo on the pad, clock
#   paused, every hull white): the centre pixel of the hull's shade side (camera on the side away from the Sun).
extends SceneTree

const FRAMES_PER_VIEW := 40  # [E] two refilter passes of the incremental cube (~8 frames each) and margin
const RADIUS_M := 1.0
const PROBE_LAYER := 1 << 19
const DIST_M := 12.0
const EYE_HEIGHT_M := 20.0  # clear of terrain with the view 30 deg down
const LOOK_DEG := 30.0
const DARK_LEVEL := 3.0 / 255.0  # sRGB code value: what "~0" means on the display
const LIT_LEVEL := 8.0 / 255.0
# [E] Godot's ambient is the cube's roughest GGX mip, not a cosine convolution; measured against sky_lux / pi at the pad.
const IRRADIANCE_TOLERANCE := 0.30
const STARBASE := {"lat": 25.9966694, "lon": -97.153728}
const TIMELAPSE_WARP := 120.0
const TIMELAPSE_FRAMES := 400
# [E] Largest frame-to-frame step in sRGB code values that reads as a smooth change, not as a flicker.
const TIMELAPSE_MAX_STEP := 6.0 / 255.0

# label, utc, site (lat, lon, body, eye height), adaptation, what is expected of (up, side, down).
var _phases := [
	{"label": "pad_day", "utc": "2026-10-12T17:30:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": false,
			"check": "irradiance"},
	{"label": "pad_night_no_moon", "utc": "2026-11-09T06:00:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": true,
			"check": "dark"},
	{"label": "pad_night_full_moon", "utc": "2026-11-24T06:00:00", "lat": STARBASE.lat, "lon": STARBASE.lon,
			"adapt": true, "check": "lit_up"},
	{"label": "orbit_day", "utc": "2026-10-12T17:30:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": false,
			"height": 400000.0, "check": "lit_down"},
	{"label": "moon_day", "utc": "2026-11-24T06:00:00", "lat": 0.0, "lon": 0.0, "body": "moon", "adapt": true,
			"check": "lit_down"},
	{"label": "moon_night_under_earth", "utc": "2026-11-09T06:00:00", "lat": 0.0, "lon": 0.0, "body": "moon",
			"adapt": true, "check": "lit_up"},
	{"label": "moon_night_earth_below_horizon", "utc": "2026-11-17T12:00:00", "lat": 0.0, "lon": -100.0, "body": "moon",
			"adapt": true, "check": "dark"},
	{"label": "pad_day_again", "utc": "2026-10-12T17:30:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": false,
			"check": "irradiance"},
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
var _sphere: MeshInstance3D
var _failures := 0
var _timelapse_frame := -1
var _timelapse_last := -1.0
var _timelapse_max_step := 0.0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	if _args.has("only"):
		_phases = _phases.filter(func(p: Dictionary) -> bool: return p["label"] == _args["only"])
	if _args.has("timelapse"):
		_phases.clear()
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


func _next_phase() -> bool:
	_phase += 1
	_wait = 0
	_looking_up = false
	if _phase >= _phases.size():
		if _args.has("timelapse"):
			_start_timelapse()
			return false
		print("sky_radiance_check: %s (%d failures)" % ["PASS" if _failures == 0 else "FAIL", _failures])
		quit(1 if _failures > 0 else 0)
		return true
	var p: Dictionary = _phases[_phase]
	_go_to(p)
	return false


func _go_to(p: Dictionary) -> void:
	var environment: Environment = _world._environment
	# The irradiance comparison needs scene values on the display: no tone curve, no glow, gain 1.
	var linear: bool = p["check"] == "irradiance"
	environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR if linear else Environment.TONE_MAPPER_AGX
	environment.glow_enabled = not linear
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


func _finish_phase(up: Color, side: Color, down: Color) -> void:
	var p: Dictionary = _phases[_phase]
	print("sky_radiance_check: %s  up %s  side %s  down %s  gain %.1f  scene_per_cd %.6f  moon_light %.8f" % [p["label"],
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
		"lit_up":
			_expect(up.g >= LIT_LEVEL, "the sky side lit")
		"lit_down":
			_expect(down.g >= LIT_LEVEL, "the ground side lit")
		"irradiance":
			_expect(side.g >= LIT_LEVEL and down.g >= LIT_LEVEL, "shade side and ground side lit")
			var gcrf_to_fixed: Basis = _world.earth.fixed_to_gcrf.transposed()
			var eye_itrf: Vector3 = gcrf_to_fixed * _world.camera.position_gcrf_m
			var up_itrf: Vector3 = gcrf_to_fixed * _world.camera.site_up_gcrf
			var sun_itrf: Vector3 = (gcrf_to_fixed * _world.sun_apparent_from_earth_m).normalized()
			var sky_lux: Vector3 = _world.core.atmosphere_illuminance(eye_itrf, up_itrf, sun_itrf)["sky_lux"]
			var expected := sky_lux.y / PI * _world.atmosphere.scene_per_cd_m2
			var measured := up.srgb_to_linear().g
			var ratio := measured / expected
			_expect(absf(ratio - 1.0) <= IRRADIANCE_TOLERANCE, "up plate %.4f, sky_lux %.0f / pi = %.4f: ratio %.3f"
					% [measured, sky_lux.y, expected, ratio])


## A sunset as a time-lapse: the camera's site at Sun ~+4 deg, warp TIMELAPSE_WARP.
func _start_timelapse() -> void:
	var p := {"label": "timelapse", "utc": "2026-10-12T23:40:00", "lat": STARBASE.lat, "lon": STARBASE.lon, "adapt": true,
			"check": "none"}
	_phases.append(p)
	_go_to(p)
	_set_view(0.0)
	_world.clock.warp = TIMELAPSE_WARP
	_timelapse_frame = 0


func _timelapse_step() -> bool:
	_place_sphere()
	_timelapse_frame += 1
	var image := root.get_viewport().get_texture().get_image()
	var cam_to := (_world.camera.global_position - _sphere.global_position).normalized()
	var value := _read(image, cam_to).g
	if _timelapse_frame > 20:  # the snap of the jump is over
		if _timelapse_last >= 0.0:
			_timelapse_max_step = maxf(_timelapse_max_step, absf(value - _timelapse_last))
		_timelapse_last = value
	if _timelapse_frame % 40 == 0:
		print("sky_radiance_check: timelapse frame %d value %.3f gain %.1f" % [_timelapse_frame, value,
				_world.atmosphere.exposure_gain])
	if _timelapse_frame < TIMELAPSE_FRAMES:
		return false
	_expect(_timelapse_max_step <= TIMELAPSE_MAX_STEP, "largest frame step %.1f code values" % (_timelapse_max_step * 255.0))
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
	_world.clock.warp = 0.0
	_flight_frame += 1
	if _flight_frame == 2:
		_rig.place(_world.flight.sim, 0, float(_args.get("station", "30")), Vector3(0, 40, 0), 30.0, 180.0)
	if _flight_frame == 3:
		_whiten(_world)  # the views exist from the first frame of the session on
	if _flight_frame < FRAMES_PER_VIEW:
		return false
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
