# Close-ups of the detailed vehicle models from a REAL flight session (look pass of the Starship prototype plan, task 3):
# starts a mission, flies it to a mission time / timeline event, then renders a list of shots with the orbit camera
# placed relative to a body (VehicleRig). Needs a GPU (no --headless). Images go to screenshots/vehicles/<label>.png.
# godot --path game --script res://tests/vehicle_look_shots.gd -- --mission=starship_leo --t_plus=148 --hide_ui
#   "--shots=label,body,station_m,dx,dy,dz,fov[,sun<deg>];..."  camera at (station + offset in the body frame), aimed at the
#     station; `sun<deg>`: from the Sun's side, turned about the vehicle axis by <deg>, at the offset's distance;
#     body = index in the snapshot (printed at the start)
#   --scan=<from>,<to>,<step>  (s after T-0) prints, per step and body, the largest flap / grid fin deflection and exits
#   --at_event=<kind>,<offset s>  instead of --t_plus (e.g. entry_interface,120)
#   --ship_return=<s after T-0>  flies the ascent that long, then plans the ship return; t_plus / scan / events then count
#     from that instant
#   --stereo=<json>  renders through the multiview path with the stand-in headset (StereoMockXr, as render_check.gd
#     --stereo) and saves <label>_eye0.png / <label>_eye1.png per shot (the fov field is then the headset's)
#   --utc=2024-04-08T18:42:00  --out_dir=<dir>
extends SceneTree

const FRAMES_TO_SETTLE := 10
const TERRAIN_MAX_FRAMES := 3000

var _args := {}
var _main: Node
var _world: World
var _rig: VehicleRig
var _frames := 0
var _stage := 0  # 0 setup, 1 shots
var _shots: Array = []
var _shot_index := 0
var _shot_wait := 0
var _stereo: StereoMockXr
var _stereo_frames := 0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	if _args.has("stereo"):
		_stereo = StereoMockXr.new()
		if not _stereo.load(String(_args["stereo"])) or not _stereo.initialize():
			print("vehicle_look_shots: stereo harness %s unusable" % _args["stereo"])
			quit(1)
			return
		XRServer.add_interface(_stereo)
		XRServer.primary_interface = _stereo
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _finalize() -> void:
	if _stereo != null:  # the XR server must not keep the stand-in while the engine shuts down
		XRServer.primary_interface = null
		_stereo.uninitialize()
		XRServer.remove_interface(_stereo)


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	_frames += 1
	if _stage == 0:
		if _frames < FRAMES_TO_SETTLE or _world.atmosphere.loading:  # the atmosphere tables load on a worker
			return false
		return _setup()
	return _shoot()


func _setup() -> bool:
	TimePanel.show_frame_rate = false
	_rig = VehicleRig.new(_world, _args)
	var sim := _rig.start()
	if _args.has("scan"):
		_scan(sim)
		quit(0)
		return true
	_rig.fly_to_target(sim)
	_rig.print_bodies(sim)
	if _args.has("hide_ui"):
		_rig.hide_ui(_main)
	if _stereo != null:  # the VR panel (menu, overlay) is not part of the views
		for child in _main.get_node("/root/XrManager").origin.get_children():
			if child is VrOverlayPanel:
				child.visible = false
	for spec in String(_args.get("shots", "")).split(";", false):
		_shots.append(spec.split(",", false))
	_world.atmosphere.snap_adaptation()
	_stage = 1
	_frames = 0
	return false


## --scan=from,to,step: the largest |flap| / |fin| deflection per body at each mission time.
func _scan(sim: FlightSim) -> void:
	var spec := String(_args["scan"]).split(",")
	var zero := _rig.zero_tdb_s(sim)
	var t := float(spec[0])
	while t <= float(spec[1]) and sim.is_active():
		_rig.advance(sim, zero + t)
		var line := "t+%5.0f s:" % t
		for i in sim.body_count():
			var flap := 0.0
			for f: Dictionary in sim.body_flaps(i):
				flap = maxf(flap, absf(f["deflection_deg"]))
			var fin := 0.0
			for f: Dictionary in sim.body_grid_fins(i):
				fin = maxf(fin, absf(f["deflection_deg"]))
			line += "  [%d] alt %.1f km flap %.0f fin %.0f" % [i, sim.telemetry(i).get("altitude_m", 0.0) / 1000.0, flap, fin]
		print(line)
		t += float(spec[2])


## --override=white|mirror|std: replaces the shader materials of every mesh under node (hull, tower, lattice, steel) by a
## white LitAerial diffuse / a metallic mirror / a StandardMaterial3D (grey, metallic 0.65, roughness 0.38).
func _override_materials(node: Node, kind: String) -> void:
	var mesh := node as MeshInstance3D
	if mesh != null and mesh.material_override is ShaderMaterial:
		var path: String = (mesh.material_override as ShaderMaterial).shader.resource_path
		if path.contains("vehicle_hull") or path.contains("lit_aerial") or path.contains("lattice") 				or path.contains("tower_steel"):
			match kind:
				"white":
					mesh.material_override = LitAerial.material(Color.WHITE, 1.0, 0.0)
				"mirror":
					mesh.material_override = LitAerial.material(Color.WHITE, 0.05, 1.0)
				_:
					var m := StandardMaterial3D.new()
					m.albedo_color = Color(0.74, 0.75, 0.77)
					m.metallic = 0.65
					m.roughness = 0.38
					mesh.material_override = m
	for child in node.get_children():
		_override_materials(child, kind)


func _shoot() -> bool:
	if _shot_index >= _shots.size():
		quit(0)
		return true
	var sim := _world.flight.sim
	var shot: PackedStringArray = _shots[_shot_index]
	if _shot_wait == 0:
		if _args.has("override"):  # night investigation 1: every steel material of the world replaced (diagnostic)
			_override_materials(_world, String(_args["override"]))
		var sun := float(shot[7].trim_prefix("sun")) if shot.size() > 7 and shot[7].begins_with("sun") else NAN
		_rig.place(sim, int(shot[1]), float(shot[2]), Vector3(float(shot[3]), float(shot[4]), float(shot[5])),
				float(shot[6]), sun)
	_shot_wait += 1
	if _shot_wait == 2:
		var env: Environment = _world._environment
		print("vehicle_look_shots: ambient energy %s, sky mode %s, reflected %s, ambient src %s, sun energy %.3f, focus %s" % [
				env.ambient_light_energy, env.sky.process_mode, env.reflected_light_source, env.ambient_light_source,
				_world.sun_light.light_energy, _world.focus])
		if _args.has("nudge"):  # the clock moved by that many seconds once (the Sun moves, the sky uniforms change)
			_world.clock.set_time(_world.clock.tdb_s + float(_args["nudge"]))
		match String(_args.get("poke", "")):  # which change makes Godot re-render the radiance cubemap
			"sky_param":
				_world._sky_material.set_shader_parameter("poke", 1.0)
			"light_dir":
				_world.sun_light.rotate_y(0.01)
			"light_energy":
				_world.sun_light.light_energy *= 0.9
		if _args.has("sky_realtime"):  # night investigation 1: the radiance cubemap re-rendered every frame
			env.sky.process_mode = Sky.PROCESS_MODE_REALTIME
	if _args.has("free") and _shot_wait == 3:  # night investigation 1: the same pose through the free camera
		_world.set_focus(World.FOCUS_FREE)
	var loading: int = _world.earth_terrain.stats["nodes_loading"]
	if _shot_wait < int(_args.get("settle", FRAMES_TO_SETTLE)) or (loading > 0 and _frames < TERRAIN_MAX_FRAMES):  # the streamed terrain
		return false
	var out_dir := String(_args.get("out_dir", ProjectSettings.globalize_path("res://").path_join("../screenshots/vehicles")))
	if not out_dir.is_absolute_path():  # a relative path once created a stray directory on the drive root
		push_error("vehicle_look_shots: --out_dir must be an absolute path, got %s" % out_dir)
		quit(1)
		return true
	DirAccess.make_dir_recursive_absolute(out_dir)
	var path := out_dir.path_join(shot[0] + ".png")
	if _stereo != null:
		if _stereo.captured.is_empty() and _stereo_frames < 120 and _stereo.capture_error == "":
			_stereo.capture_requested = true
			_stereo_frames += 1
			return false
		for view in _stereo.captured.size():
			var eye_path := path.get_basename() + "_eye%d.png" % view
			print("vehicle_look_shots: saved %s (error %d)" % [eye_path.simplify_path(), _stereo.captured[view].save_png(eye_path)])
		print("vehicle_look_shots: stereo ", _stereo.capture_info, _stereo.capture_error)
		_stereo.captured.clear()
		_stereo_frames = 0
	else:
		var error := root.get_viewport().get_texture().get_image().save_png(path)
		print("vehicle_look_shots: saved %s (error %d)" % [path.simplify_path(), error])
	_shot_index += 1
	_shot_wait = 0
	return false
