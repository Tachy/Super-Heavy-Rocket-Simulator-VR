# Diagnostic (night investigation 1): reads the lit and the shade side of test spheres of several materials in front of
# the camera of the real world (ground view at the pad), to find where the black shade side of hulls comes from.
# GPU, windowed: godot --path game --script res://tests/shade_probe.gd -- --eye_height=2 --elevation=52 [--free]
#   [--azimuth=315] [--utc=2026-10-12T14:10:00] [--lat=25.9966694 --lon=-97.153728] [--out=<png>] [--ambient=<x>]
#   [--dist=12] [--no_ground_half]: nothing here changes the lighting, --ambient only multiplies the energy to compare.
extends SceneTree

const FRAMES_TO_SETTLE := 60
const RADIUS_M := 1.0

var _args := {}
var _main: Node
var _world: World
var _frames := 0
var _spheres: Array = []  # {node, name, offset}
var _free_done := false
var _started := false
var _late_frame := 0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	var begin := _frames == 0 and not _args.has("late")
	if _args.has("late") and not _started and _frames > 20 and not _world.atmosphere.loading:
		begin = true  # the view is set only after the atmosphere tables are loaded (the order of a real session)
		_late_frame = _frames
	if begin:
		_started = true
		TimePanel.show_frame_rate = false
		if not _args.has("keep_time"):
			_world.clock.set_time(_world.core.utc_to_tdb_s(_args.get("utc", "2026-10-12T14:10:00")))
		_world.atmosphere.eye_adaptation_enabled = _args.has("adapt")
		_world.set_focus(World.FOCUS_EARTH)
		var site := {"id": "custom", "name": "custom", "body": "earth", "latitude_deg": float(_args.get("lat", "25.9966694")),
				"longitude_deg": float(_args.get("lon", "-97.153728")), "height_m": 0.0,
				"eye_height_m": float(_args.get("eye_height", "2"))}
		_world.set_ground_site(site)
		var cam := _world.camera
		cam.azimuth = deg_to_rad(float(_args.get("azimuth", "315")))
		cam.elevation = deg_to_rad(float(_args.get("elevation", "52")))
		cam.fov = float(_args.get("fov", "75"))
		_world.atmosphere.snap_adaptation()
		_add_spheres()
	_frames += 1
	if _args.has("late") and not _started:
		return false
	if _started and _frames == _late_frame + 3 and _args.has("poke"):
		_world._sky_material.set_shader_parameter("poke", 1.0)  # one write to the sky material (marks it updated)
	if _frames == 2 and _args.has("sky_realtime"):
		(_world._environment.sky as Sky).process_mode = Sky.PROCESS_MODE_REALTIME
	if _frames == 30 and _args.has("free"):
		_world.set_focus(World.FOCUS_FREE)
	if _frames == 31:
		_world.atmosphere.snap_adaptation()
	_place_spheres()
	if _frames < FRAMES_TO_SETTLE + _late_frame or _world.atmosphere.loading:
		return false
	_report()
	return true


func _add_spheres() -> void:
	var defs := [
		["white_diffuse", LitAerial.material(Color.WHITE, 1.0, 0.0)],
		["vehicle_steel", LitAerial.material(Color(0.74, 0.75, 0.77), 0.38, 0.65)],
		["tower_steel", LitAerial.material(Color(0.55, 0.56, 0.58), 0.45, 0.6)],
		["mirror_m1_r0.05", LitAerial.material(Color.WHITE, 0.05, 1.0)],
		["black_dielectric_r0.05", LitAerial.material(Color.BLACK, 0.05, 0.0)],
		["steel_m0", LitAerial.material(Color(0.74, 0.75, 0.77), 0.38, 0.0)],
		["tower_steel_tex", TowerMaterial.steel()],
		["lattice_steel", _lattice()],
	]
	(defs[0][1] as ShaderMaterial).set_shader_parameter("specular", 0.0)
	var count := defs.size()
	for i in range(count):
		var sphere := MeshInstance3D.new()
		if _args.has("cyl"):
			var cylinder := CylinderMesh.new()
			cylinder.top_radius = RADIUS_M
			cylinder.bottom_radius = RADIUS_M
			cylinder.height = 30.0
			cylinder.radial_segments = 64
			sphere.mesh = cylinder
		else:
			var mesh := SphereMesh.new()
			mesh.radius = RADIUS_M
			mesh.height = 2.0 * RADIUS_M
			mesh.radial_segments = 64
			mesh.rings = 32
			sphere.mesh = mesh
		sphere.material_override = defs[i][1]
		sphere.top_level = true
		sphere.set_meta("probe_name", defs[i][0])
		root.add_child(sphere)
		_spheres.append({"node": sphere, "name": defs[i][0], "slot": float(i) - 0.5 * float(count - 1)})


func _lattice() -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = load("res://shaders/lattice.gdshader")
	m.set_shader_parameter("member_fraction", 1.0)  # solid: every pixel a member
	return m


func _place_spheres() -> void:
	var cam := root.get_viewport().get_camera_3d()
	if cam == null:
		return
	var dist := float(_args.get("dist", "12"))
	for s in _spheres:
		var forward := -cam.global_transform.basis.z
		var right := cam.global_transform.basis.x
		var node := s["node"] as Node3D
		node.global_position = cam.global_position + forward * dist + right * (s["slot"] * 2.6)
		if _args.has("cyl"):
			var y := _render_up()
			var x := y.cross(forward).normalized()
			node.global_transform.basis = Basis(x, y, x.cross(y))


func _render_up() -> Vector3:
	var up: Vector3 = _world.camera.site_up_gcrf if _world.camera.mode == OrbitCamera.Mode.GROUND 			else _world.camera.position_gcrf_m.normalized()
	return Frames.to_render(up).normalized()


func _patch(image: Image, p: Vector2) -> Color:
	var sum := Color(0, 0, 0, 0)
	var n := 0
	for dx in range(-2, 3):
		for dy in range(-2, 3):
			var x := int(p.x) + dx
			var y := int(p.y) + dy
			if x >= 0 and y >= 0 and x < image.get_width() and y < image.get_height():
				sum += image.get_pixel(x, y)
				n += 1
	return sum / float(maxi(n, 1))


func _fmt(c: Color) -> String:
	return "(%3d %3d %3d)" % [roundi(c.r * 255.0), roundi(c.g * 255.0), roundi(c.b * 255.0)]


func _report() -> void:
	var cam := root.get_viewport().get_camera_3d()
	var image := root.get_viewport().get_texture().get_image()
	var to_sun: Vector3 = _world.sun_light.global_transform.basis.z
	var render_up := _render_up()
	print("shade_probe: eye_height %s elevation %s %s | sun light energy %.3f colour %s | ambient energy %.3f" % [
			_args.get("eye_height", "2"), _args.get("elevation", "52"), "FREE" if _args.has("free") else "ground",
			_world.sun_light.light_energy, _world.sun_light.light_color,
			_world.get_node("WorldEnvironment").environment.ambient_light_energy if _world.has_node("WorldEnvironment") else -1.0])
	print("shade_probe: camera pitch up dot sun %.3f, local up (render) %s, sun elevation %.1f deg" % [
			(-cam.global_transform.basis.z).dot(render_up), render_up,
			rad_to_deg(asin(clampf(to_sun.dot(render_up), -1.0, 1.0)))])
	var sky_p := Vector2(image.get_width() * 0.9, image.get_height() * 0.1)
	print("shade_probe: exposure gain %.3f, adaptation %s cd/m2, brightness %.3f" % [_world.atmosphere.exposure_gain,
			_world.atmosphere.adaptation_cd_m2, _world.atmosphere.brightness])
	print("shade_probe: sky pixel (corner) %s" % _fmt(_patch(image, sky_p)))
	for s in _spheres:
		var node := s["node"] as Node3D
		var c := node.global_position
		var to_cam := (cam.global_position - c).normalized()
		var lines := PackedStringArray()
		var horizontal_away := (-to_sun - render_up * (-to_sun).dot(render_up)).normalized()
		var to_cam_h := (to_cam - render_up * to_cam.dot(render_up)).normalized()
		var normals := {
			"wall_to_cam": to_cam_h,
			"lit": (to_sun + to_cam).normalized(),
			"shade": (-to_sun + to_cam).normalized(),
			"shade_horiz": (horizontal_away + to_cam * 0.9).normalized(),
			"side_up": (render_up + to_cam * 0.9).normalized(),
			"side_down": (-render_up + to_cam * 0.9).normalized(),
		}
		for key in normals:
			var n: Vector3 = normals[key]
			if n.dot(to_cam) < 0.2:
				continue
			var point := c + n * RADIUS_M
			var px := cam.unproject_position(point)
			lines.append("%s n.up %+.2f n.sun %+.2f %s" % [key, n.dot(render_up), n.dot(to_sun), _fmt(_patch(image, px))])
		print("shade_probe: %-24s %s" % [s["name"], " | ".join(lines)])
	var out: String = _args.get("out", "")
	if out != "":
		DirAccess.make_dir_recursive_absolute(out.get_base_dir())
		image.save_png(out)
		print("shade_probe: saved ", out)
