extends SceneTree
## Radiance budget of vertical faces and the ground at the Pad B site (moon procedural surface plan, Task 13b): the direct Sun,
## sky and ground-bounce illuminance on a horizontal plane and on vertical faces (toward / away from the Sun), from the core's
## atmosphere (sun_and_sky_illuminance, the model the renderer's light follows), and the radiance of materials of given albedo
## under it: L = albedo x E / pi. The ground bounce is listed with the albedo the radiance cube draws (the global day
## texture, bilinear at the site, before U-4; the imagery profile now: GroundAlbedoNear) against two references: the hardstand
## and the imagery's exact view-factor integral for each face (imagery_face_albedo.gd).
##   godot --headless --path game --script res://tests/radiance_budget.gd [-- --utc=2025-06-15T18:30:00 --face_height=40]
## No GPU. Prints one table per UTC; exit code 0 (a probe, not a gate).

const ImageryFaceAlbedo := preload("res://tests/imagery_face_albedo.gd")
const SITE_LAT_DEG := 25.99677
const SITE_LON_DEG := -97.15804
## [E] Materials as the game draws them (linear albedo): tower legs, braces, cladding (tower_material.gd); [D] the
## hardstand: mount_textures.py TERRAIN_BRIGHT, the imagery mean inside the outline, linear luminance 0.195
## (launch-structures.md; final review m4: not the rejected 95th-percentile 0.26).
const MATERIALS := {"leg": 0.16, "brace": 0.06, "clad": 0.13, "hardstand": 0.195}

var _args := {}
var _main: Node
var _world: World
var _frames := 0
var _utcs: Array = []
var _index := -1
var _wait := 0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_utcs = String(_args.get("utc", "2025-06-15T18:30:00,2025-06-15T14:45:00")).split(",")
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _srgb_to_linear(c: float) -> float:
	return c / 12.92 if c <= 0.04045 else pow((c + 0.055) / 1.055, 2.4)


## The day texture's linear luminance at the site (what the radiance cube's ground uses), bilinear.
func _cube_ground_albedo() -> Vector3:
	var path := ProjectSettings.globalize_path("res://").path_join("../data/textures/earth_day.jpg").simplify_path()
	var image := Image.load_from_file(path)
	if image == null or image.is_empty():
		return Vector3(-1, -1, -1)
	var u := (SITE_LON_DEG / 360.0 + 0.5) * image.get_width() - 0.5
	var v := (0.5 - SITE_LAT_DEG / 180.0) * image.get_height() - 0.5
	var x0 := int(floor(u))
	var y0 := int(floor(v))
	var fx := u - x0
	var fy := v - y0
	var sum := Vector3.ZERO
	for dy in range(2):
		for dx in range(2):
			var c := image.get_pixel(x0 + dx, y0 + dy)
			var w := (fx if dx == 1 else 1.0 - fx) * (fy if dy == 1 else 1.0 - fy)
			sum += Vector3(_srgb_to_linear(c.r), _srgb_to_linear(c.g), _srgb_to_linear(c.b)) * w
	return sum


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.weather.enabled = false
	_frames += 1
	if _frames < 20 or _world.atmosphere.loading:
		return false
	if _index < 0 or _wait >= 6:
		_index += 1
		_wait = 0
		if _index >= _utcs.size():
			quit(0)
			return true
		_world.clock.set_time(_world.core.utc_to_tdb_s(_utcs[_index]))
		var surface: Dictionary = _world.core.surface_samples("earth", PackedFloat64Array([SITE_LAT_DEG]),
				PackedFloat64Array([SITE_LON_DEG]))
		_world.set_ground_site({"id": "custom", "name": "custom", "body": "earth", "latitude_deg": SITE_LAT_DEG,
				"longitude_deg": SITE_LON_DEG, "height_m": surface["height_m"][0] if bool(surface["present"]) else 0.0,
				"eye_height_m": 2.0})
		return false
	_wait += 1
	if _wait < 6:
		return false
	_report(_utcs[_index])
	return false


func _report(utc: String) -> void:
	var core: StarsimCore = _world.core
	var eye_itrf: Vector3 = _world.earth.fixed_to_gcrf.transposed() * _world.camera.position_gcrf_m
	var up := eye_itrf.normalized()
	var to_sun: Vector3 = (_world.earth.fixed_to_gcrf.transposed() * _world.sun_apparent_from_earth_m).normalized()
	var sin_e := to_sun.dot(up)
	var east := up.cross(Vector3(0, 0, 1)).normalized()
	var north := east.cross(up).normalized()
	var horizontal_lux: Dictionary = core.atmosphere_illuminance(eye_itrf, up, to_sun)
	# Direct beam: the lux on the horizontal plane / sin(e).
	var beam: Vector3 = (horizontal_lux["sun_lux"] as Vector3) / maxf(sin_e, 1e-6)
	var sky_h: Vector3 = horizontal_lux["sky_lux"]
	var cube_albedo := _cube_ground_albedo()
	var sun_azimuth := atan2(to_sun.dot(east), to_sun.dot(north))
	# The ground the cube draws now: the imagery's sector x ring profile at the eye (U-4, GroundAlbedoNear), all albedos as
	# linear luminance, weighted for the sun-facing and the shaded vertical face (their azimuths) at --face_height
	# (default 40 m: a tower face; below ~ 15 m the probe point's imagery is the mount's top, not ground).
	var face_height := float(_args.get("face_height", "40"))
	var profile := GroundAlbedoNear.ring_albedo(ImagerySource.new(Paths.data_dir().path_join("imagery/earth/index.toml")),
			SITE_LAT_DEG, SITE_LON_DEG)
	var global_rgb := cube_albedo
	var now_sun := ImageryFaceAlbedo.luma(GroundAlbedoNear.face_albedo(profile, face_height, sun_azimuth, global_rgb))
	var now_shade := ImageryFaceAlbedo.luma(GroundAlbedoNear.face_albedo(profile, face_height, sun_azimuth + PI, global_rgb))
	# References for the ground each face sees, independent of the cube's profile: [sun-facing, shaded] albedo. The hardstand
	# (Task 13b; [D] MATERIALS) and the imagery's exact view-factor integral for each face at face_height (imagery_face_albedo.gd:
	# full-resolution imagery, polar grid, no rings or sectors; luminance).
	var exact: Array = ImageryFaceAlbedo.new(ImagerySource.new(Paths.data_dir().path_join("imagery/earth/index.toml")),
			SITE_LAT_DEG, SITE_LON_DEG).face_albedo(face_height, [sun_azimuth, sun_azimuth + PI], ImageryFaceAlbedo.luma(cube_albedo))
	var references := {"hardstand": [MATERIALS["hardstand"], MATERIALS["hardstand"]], "imagery, exact per face": exact}
	print("radiance_budget: %s Sun %.2f deg, beam %.0f lux (G), sky on a horizontal plane %.0f lux (G), ratio sky/global %.3f" % [
			utc, rad_to_deg(asin(sin_e)), beam.y, sky_h.y, sky_h.y / (beam.y * sin_e + sky_h.y)])
	print("  radiance cube's ground albedo before U-4 (global day texture at the site, linear RGB): (%.3f %.3f %.3f)" % [
			cube_albedo.x, cube_albedo.y, cube_albedo.z])
	print("  cube's ground now (profile, luminance, faces at %.0f m): sun-facing face %.3f, shaded face %.3f" % [face_height, now_sun,
			now_shade])
	var global_h := beam.y * sin_e + sky_h.y
	var face_direct := beam.y * cos(asin(sin_e))
	var face_sky := 0.5 * sky_h.y  # the model's (1 + n.up) / 2 tilt, n.up = 0
	var cube_g := ImageryFaceAlbedo.luma(cube_albedo)
	for ref_label: String in references:
		var ref: Array = references[ref_label]
		var truth_sun: float = face_direct + face_sky + 0.5 * float(ref[0]) * global_h
		var truth_shade: float = face_sky + 0.5 * float(ref[1]) * global_h
		print("  against the %s (albedo sun-facing %.3f / shaded %.3f, analytic budget: sun side %.0f lux, shade side %.0f lux):"
				% [ref_label, ref[0], ref[1], truth_sun, truth_shade])
		for row in [["cube ground before U-4", cube_g, cube_g], ["cube ground now", now_sun, now_shade]]:
			var sun_side: float = face_direct + face_sky + 0.5 * float(row[1]) * global_h
			var shade_side: float = face_sky + 0.5 * float(row[2]) * global_h
			print("    %-22s: E_vertical sun side %.0f lux (%+.1f %%), shade side %.0f lux (%+.1f %%)" % [row[0], sun_side,
					100.0 * (sun_side / truth_sun - 1.0), shade_side, 100.0 * (shade_side / truth_shade - 1.0)])
	for m in ["leg", "clad", "brace"]:
		var l_sun: float = MATERIALS[m] * (face_direct + face_sky + 0.5 * now_sun * global_h) / PI
		var l_shade: float = MATERIALS[m] * (face_sky + 0.5 * now_shade * global_h) / PI
		var l_ground: float = MATERIALS["hardstand"] * global_h / PI
		print("    %-5s albedo %.2f (cube ground now): sun side %6.0f cd/m2 (%.3f of the hardstand %6.0f), shade side %6.0f (%.3f)" % [
				m, MATERIALS[m], l_sun, l_sun / l_ground, l_ground, l_shade, l_shade / l_ground])
	print("  sun azimuth from north %.1f deg" % rad_to_deg(sun_azimuth))
