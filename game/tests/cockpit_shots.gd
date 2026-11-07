# Look shots of the Crew Dragon cabin from a REAL session (needs a GPU, no --headless; models generated):
#   pilot_eye         the commander's eye point (CockpitConfig.eye), 100 deg horizontal
#   gopro_discovery2  the Discovery photographers' camera (between the s2 / s3 heads, 0.15 m behind the eye points, the eye's
#                     direction), same field of view; + gopro_vs_discovery2 (side by side) and gopro_overlay_discovery2 (50 %)
#   pilot_vs_discovery1  the eye shot beside discovery1
#   hatch_crew1_1     from the hatch toward the seat row, beside crew1_1 (side by side)
# godot --path game --script res://tests/cockpit_shots.gd -- --mission=falcon9_crew_dragon --t_plus=-60 --out_dir=<abs>/screenshots/m7-cockpit/look
#   --fov=100 (horizontal, deg)  --crew=<n> (default the mission's)  --refs=<abs dir of the reference photos>
extends SceneTree

const FRAMES_TO_SETTLE := 10
const SHOT_SETTLE_FRAMES := 40
const NEAR_M := 0.04
const GOPRO_BACK_M := 0.15  # task-3 review: about 0.15 m behind the eye points

var _args := {}
var _main: Node
var _world: World
var _rig: VehicleRig
var _frames := 0
var _stage := 0
var _shots: Array[Dictionary] = []
var _index := 0
var _wait := 0
var _cockpit: CockpitView
var _out_dir := ""
var _refs_dir := ""


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	root.size = Vector2i(1600, 900)
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	_frames += 1
	if _stage == 0:
		if _frames < FRAMES_TO_SETTLE or _world.atmosphere.loading:
			return false
		return _setup()
	return _shoot()


func _setup() -> bool:
	TimePanel.show_frame_rate = false
	_out_dir = String(_args.get("out_dir", ""))
	if not _out_dir.is_absolute_path():
		push_error("cockpit_shots: --out_dir must be an absolute path")
		quit(1)
		return true
	DirAccess.make_dir_recursive_absolute(_out_dir)
	_refs_dir = String(_args.get("refs", ProjectSettings.globalize_path("res://").path_join("../screenshots/m7-cockpit-refs/mielke").simplify_path()))
	_args["mission"] = _args.get("mission", "falcon9_crew_dragon")
	_rig = VehicleRig.new(_world, _args)
	var sim := _rig.start()
	_rig.fly_to_target(sim)
	_rig.print_bodies(sim)
	_rig.hide_ui(_main)
	_rig.place(sim, 0, 62.0, Vector3(0.0, 15.0, 0.0), 40.0)  # the orbit camera near the capsule first: the views are placed only near the camera
	RenderingServer.frame_pre_draw.connect(func() -> void: _world.camera.near = NEAR_M)
	_stage = 1
	_frames = 0
	return false


func _find_cockpit() -> CockpitView:
	for view in _world.flight.views:
		if view.cockpit != null:
			return view.cockpit
	return null


func _shoot() -> bool:
	if _frames < 20:
		return false
	if _world.focus != World.FOCUS_FREE:
		_world.set_focus(World.FOCUS_FREE)
		return false
	if _cockpit == null:
		_cockpit = _find_cockpit()
		if _cockpit == null:
			if _frames > 600:
				push_error("cockpit_shots: no cockpit in the session")
				quit(1)
				return true
			return false
		var crew := int(_args.get("crew", MissionCatalogue.crew_count(String(_args["mission"]))))
		_cockpit.set_crew(crew)
		_define_shots()
	if _index >= _shots.size():
		_compose()
		quit(0)
		return true
	var shot: Dictionary = _shots[_index]
	_cockpit.set_player_head_hidden(shot.get("local").origin.is_equal_approx(_cockpit.config.eye.origin) or bool(shot.get("hide_head", false)))
	for label in _world.find_children("*", "Label3D", true, false):
		(label as Label3D).visible = false  # the stale-structure-models notice is no part of the cabin
	var fov_deg := float(shot.get("fov", _args.get("fov", 100.0)))
	_world.camera.keep_aspect = Camera3D.KEEP_WIDTH
	_world.camera.fov = fov_deg
	# the pose in the capsule frame -> render axes of this frame; converge: the view is placed relative to the camera
	var stage := _cockpit.get_parent() as Node3D
	var local: Transform3D = shot["local"]
	var target := stage.global_transform * local
	var position_gcrf := _world.camera.position_gcrf_m + Frames.from_render(target.origin)
	_world.free_pose.reset_to(position_gcrf, target.basis, _world.earth.fixed_to_gcrf, _world.moon.fixed_to_gcrf, _world.moon_gcrf_m)
	_world.camera.position_gcrf_m = position_gcrf
	_world.camera.free_basis = target.basis
	_wait += 1
	if _wait == 3:
		_world.atmosphere.snap_adaptation()
	if _wait < SHOT_SETTLE_FRAMES:
		return false
	var path := _out_dir.path_join(String(shot["name"]) + ".png")
	print("cockpit_shots: saved %s (error %d), camera error %.4f m" % [path, root.get_viewport().get_texture().get_image().save_png(path),
			target.origin.length()])
	_index += 1
	_wait = 0
	return false


func _define_shots() -> void:
	var eye := _cockpit.config.eye
	var behind := eye.basis * Vector3(0.0, 0.0, GOPRO_BACK_M)
	var between := Vector3(eye.origin.x, eye.origin.y, 0.0) + behind  # centre line between the s2 and s3 eye points
	_shots = [
		{"name": "pilot_eye", "local": eye},
		{"name": "gopro_discovery2", "local": Transform3D(eye.basis, between)},
		{"name": "hatch_crew1_1", "local": Transform3D(Basis.looking_at(Vector3(-0.15, -1.0, 0.0).normalized(), Vector3.RIGHT), Vector3(2.45, 1.2, 0.0)), "fov": 85.0},
	]


func _compose() -> void:
	_side_by_side("gopro_discovery2", "discovery2", "gopro_vs_discovery2", false)
	_side_by_side("gopro_discovery2", "discovery2", "gopro_overlay_discovery2", true)
	_side_by_side("pilot_eye", "discovery1", "pilot_vs_discovery1", false)
	_side_by_side("hatch_crew1_1", "crew1_1", "hatch_vs_crew1_1", false)


## Our shot and the photo (reference photos stay outside the repo) scaled to our height, next to each other or blended 50 %.
func _side_by_side(ours: String, photo: String, name: String, overlay: bool) -> void:
	var a := Image.load_from_file(_out_dir.path_join(ours + ".png"))
	var b := Image.load_from_file(_refs_dir.path_join(photo + ".jpg"))
	if a == null or b == null:
		print("cockpit_shots: %s skipped (image missing)" % name)
		return
	a.convert(Image.FORMAT_RGB8)
	b.convert(Image.FORMAT_RGB8)
	b.resize(a.get_width() if overlay else roundi(float(b.get_width()) * a.get_height() / b.get_height()), a.get_height(), Image.INTERPOLATE_LANCZOS)
	var out: Image
	if overlay:
		out = Image.create(a.get_width(), a.get_height(), false, Image.FORMAT_RGB8)
		for y in a.get_height():
			for x in a.get_width():
				out.set_pixel(x, y, a.get_pixel(x, y).lerp(b.get_pixel(x, y), 0.5))
	else:
		out = Image.create(a.get_width() + b.get_width(), a.get_height(), false, Image.FORMAT_RGB8)
		out.blit_rect(a, Rect2i(Vector2i.ZERO, a.get_size()), Vector2i.ZERO)
		out.blit_rect(b, Rect2i(Vector2i.ZERO, b.get_size()), Vector2i(a.get_width(), 0))
	print("cockpit_shots: saved %s (error %d)" % [name, out.save_png(_out_dir.path_join(name + ".png"))])
