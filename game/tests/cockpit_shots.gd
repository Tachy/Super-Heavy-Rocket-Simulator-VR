# Look shots of the Crew Dragon cabin from a REAL session (needs a GPU, no --headless; models generated):
#   pilot_eye              the commander's eye point (CockpitConfig.eye), turned toward the screen row, 105 deg horizontal (no photo pair)
#   gopro_discovery2       the Discovery photographers' GoPro (pose fitted on discovery2's screen corners, docs/subsystems/cockpit.md),
#                          rendered rectilinear 140 deg and remapped to the GoPro's equidistant fisheye (113 x 64 deg, 1600 x 900);
#                          + gopro_vs_discovery2 / gopro_overlay_discovery2 and, as discovery1 has the same framing, gopro_vs_discovery1
#   hatch_crew1_1          from the open hatch toward the seat row (3:2, fitted: 96 deg horizontal, 8 deg up), hatch frame and exterior hull
#                          hidden; + hatch_vs_crew1_1
#   helmet_34 / helmet_side / helmet_profile / helmet_closeup  the pilot's (s3) helmet, isolated; + helmet_vs_detail2 / _detail1 / _demo1_2,
#                          helmet_side_vs_jsc066389, helmet_side_vs_spacex_anzug (pure side view, Task 5b)
# godot --path game --script res://tests/cockpit_shots.gd -- --mission=falcon9_crew_dragon --t_plus=-60 --out_dir=<abs>/screenshots/m7-cockpit/look
#   --fov=100 (pilot_eye, horizontal deg)  --crew=<n> (default the mission's)  --refs=<abs dir of the reference photos>
#   --field_key=<0.2..1> (the eye adaptation's field background for every shot; default the game's Y_b 0.2, look fixes T13;
#   not used with --anchored_adaptation); --cabin_keys: the hand keys before T13 (the shot's key or CABIN_FIELD_KEY)
#   --fixed_exposure (eye adaptation off; default: on as in the game, snapped to each shot)
#   --screen: instead of the cabin shots, one shot per external camera offered on the left screen (Task 7): screen_<id> = the pilot's
#   view of the screen (field of view --fov, default 40), screen_feed_<id> = the SubViewport's image itself
#   --hand: instead of the cabin shots, the console pointer sequences (Task 8): hand_eye_NN (frames of the hand following the pointer to the
#   left screen, the camera button, the centre screen, a key and the right screen, with taps; from the pilot's eye) and hand_hatch_<place> (the same
#   places from the hatch camera, the whole arm); the pointer is driven by this script (World.console_pointer.driven_externally)
#   --body: instead of the cabin shots, the crew body and suit (Task 8b): body_side (the pilot s3 in profile from its right, like
#   KSC-20190731-PH_SPX01_0002), body_34 (the pilot three-quarter from the front), body_arm_raised (the commander s2 reaching the centre
#   screen), body_elbow_bent / body_elbow_straight (the commander's right elbow at the HOTAS grip and reaching); + *_vs_<photo> sheets
#   --hotas: instead of the cabin shots, the commander with the HOTAS props (Task 9): hotas_eye_down / hotas_eye_forward (pilot eye), hotas_hatch /
#   hotas_hatch_deflected (hatch camera, stick and lever deflected)
#   --shoulder: the commander's shoulders at six arm poses (Task 9 VR fix B; shoulder_<pose>_eye/out/top_<side>)
#   --visor: the commander's open helmet (helmet_open_*) and the pilot's own view turned up / left / right / down (pilot_eye_*), Task 9 VR fix B
#   --fidelity: instead of the cabin shots, one shot fid_<id> per row of config/cockpits/photo_cameras.toml (the photo's fitted camera and framing) with
#   fid_<id>_vs (side by side) and fid_<id>_overlay (50 % blend + our edges in red); --photo=<ids> limits the rows. The left screen's feed is shown in every shot.
#   --console_closeup: stereo pairs of the console body and its ceiling posts (fidelity Task 4): closeup_eye_l/_r, closeup_back_l/_r + *_stereo sheets
#   --tone=agx (Godot's AgX alone, no display tone step), --hunt=day (the Hunt factor against L_day, as before look fixes T5)
extends SceneTree

const FRAMES_TO_SETTLE := 10
const SHOT_SETTLE_FRAMES := 40
const NEAR_M := 0.04
## GoPro and hatch cameras: config/cockpits/photo_cameras.toml (rows discovery2 and crew1_1, fits in docs/subsystems/cockpit.md "Shot cameras"); the output is 1600 px wide.
const GOPRO_OUT_SIZE := Vector2i(1600, 900)
const GOPRO_RENDER_FOV := 140.0  # rectilinear, covers the fisheye frame's corners (65 deg off axis)
## [call] Field key of the white cabin (Atmosphere.field_key; 0.18 = the default grey world): the wall reads ~0.85-0.9 sRGB as in the photos.
const CABIN_FIELD_KEY := 1.2
const RENDER_SIZE := Vector2i(2560, 1440)
## Edge overlay of the fidelity sheets: our shot's luminance gradient (|dx| + |dy| over 2 px, 0..1) above this is drawn red.
const EDGE_THRESHOLD := 0.12
## pilot_eye: looks from the eye toward the middle of the screen row as the commander sees it (0.15 m to the commander's side of the centre screen centre).
const PILOT_AIM_LATERAL_M := 0.15
const PILOT_FOV := 105.0
## [call] The eye shot is dominated by the dark console: metered at the white-cabin key it clips every white surface around it; a lower key keeps them white.
const PILOT_FIELD_KEY := 0.5

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
var _hatch_vfov := 0.0  # crew1_1 row: vertical field of the hatch shots
var _pointer: ConsolePointer


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	root.size = RENDER_SIZE
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
	_world.atmosphere.eye_adaptation_enabled = not _args.has("fixed_exposure")
	_world.atmosphere.field_key = float(_args.get("field_key", CABIN_FIELD_KEY if _args.has("cabin_keys") else EyeAdaptation.FIELD_MID_GREY))
	DisplayTone.enabled = _args.get("tone", "chroma") != "agx"  # --tone=agx: Godot's AgX alone (look fixes T5)
	EyeAdaptation.hunt_display_reference = _args.get("hunt", "display") != "day"  # --hunt=day: S <= 1 as before T5
	EyeAdaptation.adopted_white_follows_scene = _args.get("white_balance", "on") != "toa"  # --white_balance=toa: the fixed TOA-Sun white
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
		_set_exterior_visible(true)
		if _args.has("body"):
			_compose_body()
		elif _args.has("fidelity"):
			_compose_fidelity()
		elif _args.has("console_closeup"):
			_compose_console_closeup()
		elif not _args.has("only") and not _args.has("screen") and not _args.has("hand") and not _args.has("hotas") and not _args.has("shoulder") and not _args.has("visor"):
			_compose()
		quit(0)
		return true
	var shot: Dictionary = _shots[_index]
	if shot.has("pointer_ops") and _wait == 0:
		_run_pointer_ops(shot["pointer_ops"])
	if shot.has("hotas_axes"):
		var axes: Array = shot["hotas_axes"]
		_cockpit.set_hotas_axes(float(axes[0]), float(axes[1]), float(axes[2]), float(axes[3]))
	if shot.has("screen_id"):
		_world.screen_camera.render_always = true  # the free camera is no cockpit view: the feed renders anyway
		_world.screen_camera.select(String(shot["screen_id"]))
	_cockpit.set_player_head_hidden(shot.get("local").origin.is_equal_approx(_cockpit.config.eye.origin) or bool(shot.get("hide_head", false)))
	var isolate := bool(shot.get("isolate", false))  # close-up of one figure: the other figures and the leg rests are hidden
	var keep_figure := "figure_" + String(shot.get("isolate_seat", "s3"))
	for figure in _cockpit.find_children("figure_*", "Node3D", false, false):
		(figure as Node3D).visible = not isolate or figure.name == keep_figure
	for part: String in ["seat_rest_", "seat_shell_s1", "seat_shell_s2", "seat_shell_s4", "seat_pad_s1", "seat_pad_s2", "seat_pad_s4", "console_", "screen_", "tablet_", "button_", "handle"]:
		for node in _cockpit.find_children(part + "*" if part.ends_with("_") else part, "MeshInstance3D", false, false):
			(node as Node3D).visible = not isolate and node.name != "console_hit"  # fidelity Task 6: the hidden hit faces covered every key face in the shots
	if _args.has("hide"):  # debugging: hide cabin parts by name (comma separated)
		for hidden_name: String in String(_args["hide"]).split(","):
			var hidden_node := _cockpit.cabin_node(hidden_name)
			if hidden_node != null:
				hidden_node.visible = false
	for label in _world.find_children("*", "Label3D", true, false):
		(label as Label3D).visible = false  # the stale-structure-models notice is no part of the cabin
	var fov_deg := float(shot.get("fov", _args.get("fov", 100.0)))
	_world.camera.keep_aspect = Camera3D.KEEP_HEIGHT if shot.get("keep_height", false) else Camera3D.KEEP_WIDTH
	_world.camera.fov = fov_deg
	_set_exterior_visible(not (shot.get("hide_exterior", _args.has("hide_exterior"))))
	if shot.has("aim_bone") and _wait >= 1:  # Task 8b: aim at a bone of the posed commander (after the pointer ops have posed it)
		var skeleton := _cockpit.figure_skeleton("s2")
		var bone_at: Vector3 = _cockpit._skeleton_to_cabin() * skeleton.get_bone_global_pose(skeleton.find_bone(String(shot["aim_bone"]))).origin
		var cam_at: Vector3 = (shot["eye_at"] as Vector3) if shot.has("eye_at") else bone_at + (shot["aim_offset"] as Vector3)
		shot["local"] = Transform3D(Basis.looking_at(bone_at - cam_at, Vector3(1.0, 0.0, 0.0)), cam_at)
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
		# An explicit --field_key holds for every shot (comparisons); else the shot's own key, else the cabin key.
		var key := float(shot.get("field_key", CABIN_FIELD_KEY)) if _args.has("cabin_keys") else EyeAdaptation.FIELD_MID_GREY
		_world.atmosphere.field_key = float(_args["field_key"]) if _args.has("field_key") else key
		_world.atmosphere.snap_adaptation()
	# the left screen's feed: the free camera is no cockpit view, so the feed would stay black; render it on each of the two frames before the capture
	_world.screen_camera.render_always = true
	if _wait >= int(shot.get("settle", SHOT_SETTLE_FRAMES)) - 2:
		_world.screen_camera.viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
	if _wait < int(shot.get("settle", SHOT_SETTLE_FRAMES)):
		return false
	var path := _out_dir.path_join(String(shot["name"]) + ".png")
	var image := root.get_viewport().get_texture().get_image()
	image.convert(Image.FORMAT_RGB8)
	if shot.has("fisheye"):
		image = _fisheye(image, shot["fisheye_f_px"], shot["fisheye_photo_size"], shot["fisheye_out_size"])
	elif shot.has("crop_aspect"):
		var width := roundi(image.get_height() * float(shot["crop_aspect"]))
		image = image.get_region(Rect2i((image.get_width() - width) / 2, 0, width, image.get_height()))
	print("cockpit_shots: saved %s (error %d), %s" % [path, image.save_png(path), _brightness(image)])
	if not shot.has("fisheye") and not shot.has("crop_aspect"):
		print("cockpit_shots: screen_left_rect %s %s" % [shot["name"], _screen_rect_px("left")])
	if shot.has("screen_id"):
		var feed := _world.screen_camera.viewport.get_texture().get_image()
		feed.convert(Image.FORMAT_RGB8)
		print("cockpit_shots: saved feed %s (error %d), %s, %d renders" % [shot["screen_id"], feed.save_png(_out_dir.path_join("screen_feed_%s.png" % shot["screen_id"])), _brightness(feed), _world.screen_camera.render_count])
	var a := _world.atmosphere
	print("cockpit_shots: %s adaptation rule %s: gain %.3f, L_a %s cd/m^2, field %s, anchor %s, field key %s, from the field %s" % [
			shot["name"], "analytic" if not a.scene_referred_adaptation else a.field_rule_name(),
			a.exposure_gain, a.adaptation_cd_m2, a.last_field_cd_m2, a.last_anchor_cd_m2, a.field_key, a.adaptation_from_field])
	print("cockpit_shots: %s adaptation surround: frame (visibility statistic, whole-weight cut) %s cd/m^2, model in frame %s, visibility %.3f, out-of-frame share %.3f; field illuminant %s, D %.4f, interior %s" % [
			shot["name"], a.adaptation_surround.last_frame_field_cd_m2, a.adaptation_surround.last_model_in_cd_m2,
			a.adaptation_surround.last_visibility, a.adaptation_surround.last_out_share, a.field_illuminant_rgb, a.white_degree,
			a.interior_light_colour])
	_index += 1
	_wait = 0
	return false


func _define_shots() -> void:
	var eye := _cockpit.config.eye
	# the hatch (crew1_1) and GoPro (discovery2) cameras come from config/cockpits/photo_cameras.toml (single source of the fitted poses)
	var hatch := _row_pose(_photo_row("crew1_1"))
	_hatch_vfov = _row_vfov_deg(_photo_row("crew1_1"))
	var centre_screen: Dictionary = _cockpit.config.screens["centre"]
	var aim: Vector3 = centre_screen["centre"] + Vector3(0.0, 0.0, PILOT_AIM_LATERAL_M)
	var pilot := Transform3D(Basis.looking_at(aim - eye.origin, eye.basis.y), eye.origin)
	# helmet close-up of the pilot (s3) from the front, 30 deg toward the cabin centre, at the distance of eva_suit_detail2's framing; up = up the seat back
	var s3: Dictionary = {}
	for seat: Dictionary in _cockpit.config.seats:
		if seat["id"] == "s3":
			s3 = seat
	var back := deg_to_rad(float(s3["back_angle_deg"]))
	var up_back := Vector3(cos(back), -sin(back), 0.0)
	var chest := Vector3(sin(back), cos(back), 0.0)
	var helmet_c: Vector3 = s3["hip"] + up_back * 0.80 + chest * 0.17
	var helmet_cam := helmet_c + chest * 0.75 + Vector3(0.0, 0.0, 0.30) - up_back * 0.02
	var helmet_views: Array = []
	# like-for-like with eva_suit_detail2 (three-quarter), detail1 / demo1_2 / jsc2024e066389 (more from the side); helmet_profile: the pure side view
	# from the pilot's right (facing right in the image) like spacex-anzug.jpg (Task 5b)
	for view: Array in [["helmet_34", 38.0, 0.75, 48.0], ["helmet_side", 62.0, 0.75, 48.0], ["helmet_profile", -90.0, 1.1, 40.0]]:
		var az := deg_to_rad(float(view[1]))
		var cam := helmet_c + (chest * cos(az) + Vector3(0.0, 0.0, 1.0) * sin(az)) * float(view[2])
		helmet_views.append({"name": view[0], "local": Transform3D(Basis.looking_at(helmet_c - cam, up_back), cam), "fov": float(view[3]), "hide_exterior": false, "isolate": true})
	if _args.has("screen"):
		var left: Dictionary = _cockpit.config.screens["left"]
		var aim_left: Vector3 = left["centre"]
		var eye_left := Transform3D(Basis.looking_at(aim_left - eye.origin, eye.basis.y), eye.origin)
		_shots.clear()
		for id in CameraCatalog.available(_world.flight.sim):
			_shots.append({"name": "screen_" + id, "local": eye_left, "fov": float(_args.get("fov", 40.0)), "screen_id": id})
		return
	if _args.has("fidelity"):
		_define_fidelity_shots()
		return
	if _args.has("console_closeup"):
		_define_console_closeup_shots(eye)
		return
	if _args.has("hand"):
		_define_hand_shots(eye, pilot, hatch)
		return
	if _args.has("body"):
		_define_body_shots(s3)
		return
	if _args.has("hotas"):
		_define_hotas_shots(eye, hatch)
		return
	if _args.has("shoulder"):
		_define_shoulder_shots()
		return
	if _args.has("visor"):  # Task 9 VR fix B: the commander's open helmet from outside, the pilot's own view in several head turns
		var s2: Dictionary = {}
		for seat: Dictionary in _cockpit.config.seats:
			if seat["id"] == "s2":
				s2 = seat
		var back2 := deg_to_rad(float(s2["back_angle_deg"]))
		var up2 := Vector3(cos(back2), -sin(back2), 0.0)
		var chest2 := Vector3(sin(back2), cos(back2), 0.0)
		var c2: Vector3 = s2["hip"] + up2 * 0.80 + chest2 * 0.17
		_shots.clear()
		for view: Array in [["helmet_open_34", -38.0, 0.75, 48.0], ["helmet_open_front", 0.0, 0.8, 45.0], ["helmet_open_profile", -90.0, 1.1, 40.0], ["helmet_open_rear34", -140.0, 0.9, 45.0]]:
			var az := deg_to_rad(float(view[1]))
			var cam := c2 + (chest2 * cos(az) + Vector3(0.0, 0.0, 1.0) * sin(az)) * float(view[2]) + up2 * 0.1
			_shots.append({"name": view[0], "local": Transform3D(Basis.looking_at(c2 - cam, up2), cam), "fov": float(view[3]), "hide_exterior": false,
					"isolate": true, "isolate_seat": "s2", "hide_head": false})
		_shots.append({"name": "helmet_open_hatch", "local": hatch, "fov": _hatch_vfov, "keep_height": true, "crop_aspect": 1.5,
				"hide_exterior": false, "hide_head": false})
		for turn: Array in [["pilot_eye", 0.0, 0.0], ["pilot_eye_up", 0.0, 35.0], ["pilot_eye_left", 70.0, 0.0], ["pilot_eye_right", -70.0, 0.0], ["pilot_eye_down", 0.0, -45.0]]:
			var b := pilot.basis.rotated(eye.basis.y.normalized(), deg_to_rad(float(turn[1])))
			b = b.rotated(b.x.normalized(), deg_to_rad(float(turn[2])))
			_shots.append({"name": turn[0], "local": Transform3D(b, eye.origin), "fov": PILOT_FOV, "field_key": PILOT_FIELD_KEY})
		return
	var rest: Array = [
		{"name": "helmet_closeup", "local": Transform3D(Basis.looking_at(helmet_c - helmet_cam, up_back), helmet_cam), "fov": 48.0, "hide_exterior": false, "isolate": true},
		{"name": "pilot_eye", "local": pilot, "fov": PILOT_FOV, "field_key": PILOT_FIELD_KEY},
		_fisheye_shot("gopro_discovery2", _photo_row("discovery2")),
		# [E] crew1_1 (docs/subsystems/cockpit.md "Fix round 4"): the fitted camera, 3:2
		{"name": "hatch_crew1_1", "local": hatch, "fov": _hatch_vfov, "keep_height": true,
				"crop_aspect": 1.5, "hide_exterior": false, "hide_head": false},
	]
	_shots.assign(helmet_views + rest)
	if _args.has("only"):  # debugging: render only these shots (comma separated), no comparisons
		var keep := String(_args["only"]).split(",")
		_shots.assign(_shots.filter(func(s: Dictionary) -> bool: return keep.has(String(s["name"]))))
	if _args.has("no_lamp_shadows"):  # debugging: the console lamps without shadows
		for light in _cockpit.find_children("light_*", "OmniLight3D", false, false):
			(light as OmniLight3D).shadow_enabled = false
	if _args.has("no_shadows"):  # debugging: every light without shadows (cabin lamps and the Sun)
		for light in _world.find_children("*", "Light3D", true, false):
			(light as Light3D).shadow_enabled = false
	if _args.has("no_sky_light"):  # debugging: no ambient / reflected light from the sky (what reaches a closed cabin only through AO)
		var env: Environment = _world.get("_environment")
		env.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
		env.reflected_light_source = Environment.REFLECTION_SOURCE_DISABLED
	if _args.has("agx_contrast"):  # debugging: AgX contrast (game default 1.25 of Godot): how dark the black parts read under the high-key cabin
		(_world.get("_environment") as Environment).tonemap_agx_contrast = float(_args["agx_contrast"])
	if _args.has("lights_off"):  # debugging: hide lights by name pattern (comma separated, e.g. ring_*,fill_*; "sun" = the Sun)
		for pattern: String in String(_args["lights_off"]).split(","):
			if pattern == "sun":
				_world.sun_light.visible = false
			for light in _cockpit.find_children(pattern, "Light3D", false, false):
				(light as Light3D).visible = false


## Task 8: the pointer sequences. Ops per shot: ["goto", strip place id or "idle", frames], ["click"], ["wait", frames]; each frame is a 1/30 s step.
func _define_hand_shots(eye: Transform3D, pilot: Transform3D, hatch: Transform3D) -> void:
	_pointer = ConsolePointer.new()
	_world.add_child(_pointer)
	_pointer.setup(_cockpit.config, _cockpit, _world.screen_camera)
	_pointer.active = false  # idle until the first "goto"
	_pointer.vr_mode = true
	_world.console_pointer.driven_externally = true
	_world.screen_camera.render_always = true  # the free camera is no cockpit view: the feed renders anyway
	var places := [["left", "left"], ["camera_button", "camera_button"], ["centre", "centre"], ["key_cam3", "button:cam3"], ["right", "right"], ["key_far_right", "button:r_upper_5"]]
	_shots.clear()
	_shots.append({"name": "hand_eye_00_idle", "local": pilot, "fov": PILOT_FOV, "field_key": PILOT_FIELD_KEY, "pointer_ops": [["wait", 20]], "settle": 8})
	var n := 1
	for place: Array in places:
		var ops_move: Array = [["goto", place[1], 0]]
		# the sequence towards the place: a frame every 3 steps (0.1 s), then the tap with frames every 2 steps (0.067 s)
		for k in 8:
			_shots.append({"name": "hand_eye_%02d_%s_move" % [n, place[0]], "local": pilot, "fov": PILOT_FOV, "field_key": PILOT_FIELD_KEY,
					"pointer_ops": (ops_move if k == 0 else []) + [["wait", 3]], "settle": 4})
			n += 1
		_shots.append({"name": "hand_eye_%02d_%s_click" % [n, place[0]], "local": pilot, "fov": PILOT_FOV, "field_key": PILOT_FIELD_KEY, "pointer_ops": [["click"], ["wait", 2]], "settle": 4})
		n += 1
		for k in 5:
			_shots.append({"name": "hand_eye_%02d_%s_tap" % [n, place[0]], "local": pilot, "fov": PILOT_FOV, "field_key": PILOT_FIELD_KEY, "pointer_ops": [["wait", 2]], "settle": 4})
			n += 1
		_shots.append({"name": "hand_hatch_%s" % place[0], "local": hatch, "fov": _hatch_vfov, "keep_height": true, "crop_aspect": 1.5, "hide_exterior": false,
				"hide_head": false, "pointer_ops": [["wait", 20]], "settle": 8})


## Task 9: the commander with the HOTAS props: from the pilot's eye (looking down at the hands), from the hatch, and the stick / lever deflected
## (hotas_axes = [pitch, roll, yaw, throttle]); the hands ride the props (pointer "goto idle").
func _define_hotas_shots(eye: Transform3D, hatch: Transform3D) -> void:
	_pointer = ConsolePointer.new()
	_world.add_child(_pointer)
	_pointer.setup(_cockpit.config, _cockpit, _world.screen_camera)
	_pointer.active = false
	_pointer.vr_mode = true
	_world.console_pointer.driven_externally = true
	var grip: Vector3 = eye.origin + CockpitConfig._vec(_cockpit.config.hotas["stick_grip_m"])
	var down_aim := Transform3D(Basis.looking_at(grip - eye.origin, eye.basis.y), eye.origin)
	var idle := [["goto", "idle", 0], ["wait", 40]]
	_shots.clear()
	_shots.append({"name": "hotas_eye_down", "local": down_aim, "fov": 100.0, "hide_exterior": true, "pointer_ops": idle, "hotas_axes": [0.0, 0.0, 0.0, 0.5], "settle": 50})
	_shots.append({"name": "hotas_eye_forward", "local": Transform3D(Basis.looking_at(_cockpit.config.screens["centre"]["centre"] - eye.origin, eye.basis.y), eye.origin),
			"fov": 100.0, "hide_exterior": true, "pointer_ops": idle, "hotas_axes": [0.0, 0.0, 0.0, 0.5], "settle": 50})
	_shots.append({"name": "hotas_hatch", "local": hatch, "fov": _hatch_vfov, "keep_height": true, "crop_aspect": 1.5, "hide_exterior": false, "hide_head": false,
			"pointer_ops": idle, "hotas_axes": [0.0, 0.0, 0.0, 0.5], "settle": 50})
	_shots.append({"name": "hotas_hatch_deflected", "local": hatch, "fov": _hatch_vfov, "keep_height": true, "crop_aspect": 1.5, "hide_exterior": false, "hide_head": false,
			"pointer_ops": idle, "hotas_axes": [0.8, -0.6, 0.0, 1.0], "settle": 50})


## Task 8b: the crew body and suit. The pilot (s3) in profile and three-quarter; the commander (s2) reaching the centre screen and its right
## elbow bent (idle at the HOTAS grip) and straight (reaching), each figure isolated.
func _define_body_shots(s3: Dictionary) -> void:
	_pointer = ConsolePointer.new()
	_world.add_child(_pointer)
	_pointer.setup(_cockpit.config, _cockpit, _world.screen_camera)
	_pointer.active = false
	_pointer.vr_mode = true
	_world.console_pointer.driven_externally = true
	var s2: Dictionary = {}
	for seat: Dictionary in _cockpit.config.seats:
		if seat["id"] == "s2":
			s2 = seat
	_shots.clear()
	for entry: Array in [[s3, "body_side", -90.0, 1.25, 62.0, "s3"], [s3, "body_34", 35.0, 1.45, 60.0, "s3"]]:
		var seat: Dictionary = entry[0]
		var back := deg_to_rad(float(seat["back_angle_deg"]))
		var up_back := Vector3(cos(back), -sin(back), 0.0)
		var chest := Vector3(sin(back), cos(back), 0.0)
		var mid: Vector3 = seat["hip"] + up_back * 0.30 + chest * 0.28
		var az := deg_to_rad(float(entry[2]))
		var cam := mid + (chest * cos(az) + Vector3(0.0, 0.0, 1.0) * sin(az)) * float(entry[3]) + up_back * 0.15
		var up := Vector3(1.0, 0.0, 0.0)
		_shots.append({"name": entry[1], "local": Transform3D(Basis.looking_at(mid - cam, up), cam), "fov": float(entry[4]), "keep_height": true, "crop_aspect": 1.2, "hide_exterior": true,
				"isolate": true, "isolate_seat": entry[5], "hide_head": false, "pointer_ops": [["goto", "idle", 0], ["wait", 30]], "settle": 12})
	# the commander: camera to its right (-Z) and ahead, looking at the right arm
	var back2 := deg_to_rad(float(s2["back_angle_deg"]))
	var up2 := Vector3(cos(back2), -sin(back2), 0.0)
	var chest2 := Vector3(sin(back2), cos(back2), 0.0)
	var arm_mid: Vector3 = s2["hip"] + up2 * 0.45 + chest2 * 0.30 + Vector3(0.0, 0.0, -0.22)
	var arm_cam := arm_mid + chest2 * 1.05 + Vector3(0.0, 0.0, -0.75) + up2 * 0.25
	var arm_view := Transform3D(Basis.looking_at(arm_mid - arm_cam, Vector3(1.0, 0.0, 0.0)), arm_cam)
	var elbow_mid: Vector3 = s2["hip"] + up2 * 0.36 + chest2 * 0.22 + Vector3(0.0, 0.0, -0.27)
	var elbow_cam := elbow_mid + chest2 * 0.35 + Vector3(0.0, 0.0, -0.45) + up2 * 0.10
	var elbow_view := Transform3D(Basis.looking_at(elbow_mid - elbow_cam, Vector3(1.0, 0.0, 0.0)), elbow_cam)
	var common := {"hide_exterior": true, "isolate": true, "isolate_seat": "s2", "hide_head": false, "settle": 12, "keep_height": true, "crop_aspect": 1.2}
	var side_offset := chest2 * 0.55 + Vector3(0.0, 0.0, -0.55) + up2 * 0.15  # ahead of and to the right of the commander
	_shots.append(common.merged({"name": "body_elbow_bent", "local": elbow_view, "fov": 40.0, "aim_bone": "forearm_r", "aim_offset": side_offset * 1.3,
			"pointer_ops": [["goto", "idle", 0], ["wait", 40]]}))
	_shots.append(common.merged({"name": "body_arm_bent", "local": arm_view, "fov": 50.0, "aim_bone": "upper_arm_r", "aim_offset": side_offset * 1.8,
			"pointer_ops": [["wait", 2]]}))
	_shots.append(common.merged({"name": "body_arm_raised", "local": arm_view, "fov": 50.0, "aim_bone": "upper_arm_r", "aim_offset": side_offset * 1.8,
			"pointer_ops": [["goto", "centre", 0], ["wait", 40]]}))
	_shots.append(common.merged({"name": "body_elbow_straight", "local": elbow_view, "fov": 40.0, "aim_bone": "forearm_r", "aim_offset": side_offset * 1.3,
			"pointer_ops": [["wait", 10]]}))


## Task 9 VR fix B: the commander's shoulders at several arm poses (idle on the HOTAS, pointing left / centre / right, the far key r_upper_5,
## a backward swing of the right hand beside the hip): from the pilot's eye at each shoulder (with the own head as the game shows it) and
## from outside (front-right / front-left of the commander), the figure isolated.
func _define_shoulder_shots() -> void:
	_pointer = ConsolePointer.new()
	_world.add_child(_pointer)
	_pointer.setup(_cockpit.config, _cockpit, _world.screen_camera)
	_pointer.active = false
	_pointer.vr_mode = true
	_world.console_pointer.driven_externally = true
	var s2: Dictionary = {}
	for seat: Dictionary in _cockpit.config.seats:
		if seat["id"] == "s2":
			s2 = seat
	var back2 := deg_to_rad(float(s2["back_angle_deg"]))
	var up2 := Vector3(cos(back2), -sin(back2), 0.0)
	var chest2 := Vector3(sin(back2), cos(back2), 0.0)
	var poses := [["idle", [["goto", "idle", 0], ["wait", 40]]], ["left", [["goto", "left", 0], ["wait", 40]]], ["centre", [["goto", "centre", 0], ["wait", 40]]],
			["right", [["goto", "right", 0], ["wait", 40]]], ["r_upper_5", [["goto", "button:r_upper_5", 0], ["wait", 40]]], ["back", [["pose_back"]]]]
	_shots.clear()
	for pose: Array in poses:
		var ops: Array = pose[1]
		var again: Array = ops if pose[0] == "back" else []
		for side: String in ["r", "l"]:
			_shots.append({"name": "shoulder_%s_eye_%s" % [pose[0], side], "local": Transform3D(Basis.IDENTITY, _cockpit.config.eye.origin + up2 * 0.001),
					"fov": 90.0, "eye_at": _cockpit.config.eye.origin + up2 * 0.001, "aim_bone": "upper_arm_" + side, "hide_exterior": true,
					"pointer_ops": ops if side == "r" else again, "settle": 12})
			var lateral := Vector3(0.0, 0.0, -0.45 if side == "r" else 0.45)
			_shots.append({"name": "shoulder_%s_out_%s" % [pose[0], side], "local": Transform3D.IDENTITY, "fov": 45.0, "aim_bone": "upper_arm_" + side,
					"aim_offset": chest2 * 0.45 + lateral + up2 * 0.30, "hide_exterior": true, "isolate": true, "isolate_seat": "s2", "hide_head": false,
					"pointer_ops": again, "settle": 12, "keep_height": true, "crop_aspect": 1.2})
			_shots.append({"name": "shoulder_%s_top_%s" % [pose[0], side], "local": Transform3D.IDENTITY, "fov": 45.0, "aim_bone": "upper_arm_" + side,
					"aim_offset": up2 * 0.55 + chest2 * 0.15 + lateral * 0.5, "hide_exterior": true, "isolate": true, "isolate_seat": "s2", "hide_head": true,
					"pointer_ops": again, "settle": 12, "keep_height": true, "crop_aspect": 1.2})


func _photo_row(id: String) -> Dictionary:
	var rows: Array = TomlLite.load_file(Paths.config_dir().path_join("cockpits/photo_cameras.toml")).get("photo", [])
	for row: Dictionary in rows:
		if String(row["id"]) == id:
			return row
	push_error("cockpit_shots: no row '%s' in config/cockpits/photo_cameras.toml" % id)
	quit(1)
	return {"id": id, "position_m": [0.0, 0.0, 0.0], "forward": [0.0, 0.0, -1.0], "up": [0.0, 1.0, 0.0], "size_px": [1.0, 1.0], "fov_h_deg": 90.0}


## Camera transform of a photo row (capsule frame): looks along forward, image up = up, image right = forward x up.
func _row_pose(row: Dictionary) -> Transform3D:
	var position := Vector3(row["position_m"][0], row["position_m"][1], row["position_m"][2])
	var forward := Vector3(row["forward"][0], row["forward"][1], row["forward"][2]).normalized()
	var up := Vector3(row["up"][0], row["up"][1], row["up"][2]).normalized()
	var right := forward.cross(up).normalized()
	return Transform3D(Basis(right, right.cross(forward).normalized(), -forward), position)


## Vertical field (deg) of a pinhole row from its horizontal field and the photo's aspect.
func _row_vfov_deg(row: Dictionary) -> float:
	var fov_h := deg_to_rad(float(row["fov_h_deg"]))
	return rad_to_deg(2.0 * atan(tan(fov_h * 0.5) * float(row["size_px"][1]) / float(row["size_px"][0])))


## --fidelity: one shot fid_<id> per row of config/cockpits/photo_cameras.toml, rendered from the row's camera at the photo's framing
## (pinhole: the photo's aspect, keep height; fisheye_equidistant: rectilinear 140 deg remapped to the fisheye at width 1600).
func _fidelity_rows() -> Array:
	var rows: Array = TomlLite.load_file(Paths.config_dir().path_join("cockpits/photo_cameras.toml")).get("photo", [])
	if _args.has("photo"):
		var keep := String(_args["photo"]).split(",")
		rows = rows.filter(func(row: Dictionary) -> bool: return keep.has(String(row["id"])))
	return rows


## Shot of a fisheye_equidistant row: rectilinear GOPRO_RENDER_FOV remapped (r = f theta) to the photo's framing at the output width 1600.
## A row whose corner lies beyond the render field cannot be filled: it prints a warning and the edges render black.
func _fisheye_shot(shot_name: String, row: Dictionary) -> Dictionary:
	var size := Vector2(float(row["size_px"][0]), float(row["size_px"][1]))
	var fov_h := deg_to_rad(float(row["fov_h_deg"]))
	var f_px := size.x * 0.5 / (fov_h * 0.5)
	var corner_deg := rad_to_deg(size.length() * 0.5 / f_px)
	if corner_deg > GOPRO_RENDER_FOV * 0.5:
		push_warning("cockpit_shots: photo '%s' fisheye corner %.1f deg off axis exceeds the %.0f deg render field: the edges render black"
				% [row["id"], corner_deg, GOPRO_RENDER_FOV * 0.5])
	return {"name": shot_name, "local": _row_pose(row), "fov": GOPRO_RENDER_FOV, "fisheye": true, "fisheye_f_px": f_px, "fisheye_photo_size": size,
			"fisheye_out_size": Vector2i(GOPRO_OUT_SIZE.x, roundi(GOPRO_OUT_SIZE.x * size.y / size.x))}


func _define_fidelity_shots() -> void:
	_shots.clear()
	for row: Dictionary in _fidelity_rows():
		var size := Vector2(float(row["size_px"][0]), float(row["size_px"][1]))
		var fov_h := deg_to_rad(float(row["fov_h_deg"]))
		var shot := {"name": "fid_" + String(row["id"]), "local": _row_pose(row), "hide_exterior": false, "hide_head": false}
		if String(row["model"]) == "fisheye_equidistant":
			shot.merge(_fisheye_shot(shot["name"], row))
		else:
			var fov_v := 2.0 * atan(tan(fov_h * 0.5) * size.y / size.x)
			shot.merge({"fov": rad_to_deg(fov_v), "keep_height": true, "crop_aspect": size.x / size.y})
		_shots.append(shot)


## --console_closeup (fidelity Task 4): stereo pairs (IPD STEREO_IPD_M) of the console body and its ceiling posts: closeup_eye_<l|r> from the
## commander's eye looking up past the screens at the posts, closeup_back_<l|r> from 1.1 m behind the console's back (hatch side, below it);
## composed side by side (left | right eye) into closeup_eye_stereo / closeup_back_stereo.
const STEREO_IPD_M := 0.064
const CLOSEUP_SHOTS := ["closeup_eye", "closeup_back", "closeup_tray", "closeup_tray_r"]
const TRAY_CLOSEUP_M := 0.6  ## fidelity Task 5: the tray close-ups stand this far from the module, on the line to the commander's eye


func _define_console_closeup_shots(eye: Transform3D) -> void:
	_shots.clear()
	var centre_screen: Dictionary = _cockpit.config.screens["centre"]
	var up: Vector3 = centre_screen["up"]
	var normal: Vector3 = centre_screen["normal"]
	var top_mid: Vector3 = centre_screen["centre"] + up * 0.35 - normal * 0.1
	var back_cam: Vector3 = top_mid - normal * 1.1 - up * 0.35
	var right: Vector3 = (-normal).cross(up).normalized()
	var views := [["closeup_eye", eye.origin, top_mid + up * 0.25, 80.0, eye.basis.y], ["closeup_back", back_cam, top_mid, 70.0, Vector3(1.0, 0.0, 0.0)]]
	for tray: Array in [["closeup_tray", -0.53], ["closeup_tray_r", 0.53]]:  # Task 5: the outer modules (guarded blocks) at VR distance
		var target: Vector3 = centre_screen["centre"] + right * float(tray[1]) - up * 0.2 + normal * 0.03
		views.append([tray[0], target + (eye.origin - target).normalized() * TRAY_CLOSEUP_M, target, 40.0, eye.basis.y])
	for view: Array in views:
		var origin: Vector3 = view[1]
		var basis := Basis.looking_at(Vector3(view[2]) - origin, Vector3(view[4]))
		for side: Array in [["l", -0.5], ["r", 0.5]]:
			var pos: Vector3 = origin + basis.x * (STEREO_IPD_M * float(side[1]))
			_shots.append({"name": "%s_%s" % [view[0], side[0]], "local": Transform3D(basis, pos), "fov": float(view[3]), "hide_exterior": false, "hide_head": true})


func _compose_console_closeup() -> void:
	for shot_name: String in CLOSEUP_SHOTS:
		_side_by_side(shot_name + "_l", _out_dir.path_join(shot_name + "_r.png"), shot_name + "_stereo", false)


func _compose_fidelity() -> void:
	var repo := ProjectSettings.globalize_path("res://").path_join("..").simplify_path()
	for row: Dictionary in _fidelity_rows():
		var id := String(row["id"])
		var photo := repo.path_join(String(row["file"]))
		_side_by_side("fid_" + id, photo, "fid_%s_vs" % id, false)
		_side_by_side("fid_" + id, photo, "fid_%s_overlay" % id, true, true)


func _compose_body() -> void:
	_side_by_side("body_side", "../nasa/KSC-20190731-PH_SPX01_0002", "body_side_vs_ksc0002", false)
	_side_by_side("body_34", "../eva_suit_newatlas", "body_34_vs_newatlas", false)
	_side_by_side("body_34", "crew1_1", "body_34_vs_crew1_1", false)
	_side_by_side("body_arm_raised", "../nasa/jsc2024e066391", "body_arm_raised_vs_jsc066391", false)
	_side_by_side("body_elbow_bent", "demo1_1", "body_elbow_bent_vs_demo1_1", false)
	_side_by_side("body_elbow_straight", "demo1_1", "body_elbow_straight_vs_demo1_1", false)


func _run_pointer_ops(ops: Array) -> void:
	for op: Array in ops:
		match String(op[0]):
			"goto":
				_pointer.active = String(op[1]) != "idle"
				if _pointer.active:
					var surface := _pointer.strip.surface(String(op[1]))
					if String(op[1]) == "camera_button":
						var left: Dictionary = _pointer.strip.surface("left")
						var rect: Rect2 = left["rect"]
						var uv := ScreenCamera.CAMERA_BUTTON_UV.get_center()
						_pointer.strip_position = Vector2(rect.position.x + uv.x * rect.size.x, rect.end.y - uv.y * rect.size.y)
					else:
						_pointer.strip_position = (surface["rect"] as Rect2).get_center()
			"click":
				_pointer.click()
			"wait":
				pass
			"pose_back":  # the right hand swung back beside the hip (elbow behind the torso), the left on its idle grip
				var skeleton := _cockpit.figure_skeleton("s2")
				var shoulder: Vector3 = _cockpit._skeleton_to_cabin() * skeleton.get_bone_global_rest(skeleton.find_bone("upper_arm_r")).origin
				var up_back := _cockpit._skeleton_to_cabin().basis.x.normalized()
				var chest := _cockpit._skeleton_to_cabin().basis.y.normalized()
				_cockpit.pose_arms({"l": {"tip": _cockpit.idle_tip("l")}, "r": {"tip": shoulder - up_back * 0.45 - chest * 0.02 + Vector3(0.0, 0.0, -0.14)}})
				return
	var frames := 1
	for op: Array in ops:
		if String(op[0]) == "wait":
			frames = int(op[1])
	for i in frames:
		_pointer.update(1.0 / 30.0, null)


## The screen's mesh box projected into the current frame: "x y w h" in px of the window (for tests that measure the screen region).
func _screen_rect_px(id: String) -> String:
	var mesh := _cockpit.screen_surface(id)
	var cam := root.get_viewport().get_camera_3d()
	if mesh == null or cam == null:
		return "0 0 0 0"
	var box := mesh.get_aabb()
	var lo := Vector2(1.0e9, 1.0e9)
	var hi := Vector2(-1.0e9, -1.0e9)
	for i in 8:
		var corner := box.position + box.size * Vector3(i & 1, (i >> 1) & 1, (i >> 2) & 1)
		var world_point := mesh.global_transform * corner
		if cam.is_position_behind(world_point):
			continue
		var p := cam.unproject_position(world_point)
		lo = lo.min(p)
		hi = hi.max(p)
	if hi.x < lo.x:
		return "0 0 0 0"
	return "%d %d %d %d" % [lo.x, lo.y, hi.x - lo.x, hi.y - lo.y]


## The hatch frame and the exterior hull (every stage node child but the cockpit) are hidden for the shot from outside the inner wall.
func _set_exterior_visible(on: bool) -> void:
	var stage := _cockpit.get_parent() as Node3D
	for child in stage.get_children():
		if child != _cockpit and child is Node3D:
			(child as Node3D).visible = on
	var hatch := _cockpit.cabin_node("hatch_frame")
	if hatch != null:
		hatch.visible = on


## Rectilinear render (GOPRO_RENDER_FOV, keep width) -> the GoPro's equidistant fisheye (r = f theta), bilinear.
func _fisheye(src: Image, f_photo_px: float, photo_size: Vector2, out_size: Vector2i) -> Image:
	var out := Image.create(out_size.x, out_size.y, false, Image.FORMAT_RGB8)
	var f := f_photo_px * out_size.x / photo_size.x
	var px_per_tan := src.get_width() * 0.5 / tan(deg_to_rad(GOPRO_RENDER_FOV) * 0.5)
	var sw := src.get_width()
	var sh := src.get_height()
	var centre := Vector2(out_size) * 0.5
	for y in out_size.y:
		for x in out_size.x:
			var d := Vector2(x + 0.5, y + 0.5) - centre
			var r := d.length()
			var theta := r / f
			if theta >= 1.5:
				continue
			var k := tan(theta) / maxf(r, 1.0e-6)
			var u := sw * 0.5 + d.x * k * px_per_tan - 0.5
			var v := sh * 0.5 + d.y * k * px_per_tan - 0.5
			if u < 0.0 or v < 0.0 or u > sw - 1.0 or v > sh - 1.0:
				continue
			var x0 := int(u)
			var y0 := int(v)
			var x1 := mini(x0 + 1, sw - 1)
			var y1 := mini(y0 + 1, sh - 1)
			var top := src.get_pixel(x0, y0).lerp(src.get_pixel(x1, y0), u - x0)
			var bottom := src.get_pixel(x0, y1).lerp(src.get_pixel(x1, y1), u - x0)
			out.set_pixel(x, y, top.lerp(bottom, v - y0))
	return out


## Brightness reported with every shot: 90th percentile and mean of the grey over the frame (sRGB, 0..1); the white wall is ~ p90.
func _brightness(image: Image) -> String:
	var greys: Array[float] = []
	for y in range(0, image.get_height(), 8):
		for x in range(0, image.get_width(), 8):
			var c := image.get_pixel(x, y)
			greys.append((c.r + c.g + c.b) / 3.0)
	greys.sort()
	var mean := 0.0
	for g in greys:
		mean += g
	return "p90 grey %.2f, mean %.2f" % [greys[int(greys.size() * 0.9)], mean / greys.size()]


func _compose() -> void:
	_side_by_side("gopro_discovery2", "discovery2", "gopro_vs_discovery2", false)
	_side_by_side("gopro_discovery2", "discovery2", "gopro_overlay_discovery2", true)
	_side_by_side("gopro_discovery2", "discovery1", "gopro_vs_discovery1", false)  # same framing as discovery2
	_side_by_side("hatch_crew1_1", "crew1_1", "hatch_vs_crew1_1", false)
	_side_by_side("helmet_34", "../eva_suit_detail2", "helmet_vs_detail2", false)
	_side_by_side("helmet_side", "../eva_suit_detail1", "helmet_vs_detail1", false)
	_side_by_side("helmet_side", "demo1_2", "helmet_vs_demo1_2", false)
	_side_by_side("helmet_side", "../nasa/jsc2024e066389", "helmet_side_vs_jsc066389", false)
	_side_by_side("helmet_profile", "../user/anzug/spacex-anzug", "helmet_side_vs_spacex_anzug", false)


## Our shot and the photo (reference photos stay outside the repo) scaled to our height, next to each other or blended 50 %.
## `photo` is a name below the refs dir (no extension) or, when absolute, the photo file itself; `edges`: our shot's edges (Sobel) in red on the blend.
func _side_by_side(ours: String, photo: String, name: String, overlay: bool, edges := false) -> void:
	var a := Image.load_from_file(_out_dir.path_join(ours + ".png"))
	var b := Image.load_from_file(photo if photo.is_absolute_path() else _refs_dir.path_join(photo + ".jpg"))
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
		if edges:
			for y in range(1, a.get_height() - 1):
				for x in range(1, a.get_width() - 1):
					var gx := a.get_pixel(x + 1, y).get_luminance() - a.get_pixel(x - 1, y).get_luminance()
					var gy := a.get_pixel(x, y + 1).get_luminance() - a.get_pixel(x, y - 1).get_luminance()
					if absf(gx) + absf(gy) > EDGE_THRESHOLD:
						out.set_pixel(x, y, Color(1.0, 0.1, 0.1))
	else:
		out = Image.create(a.get_width() + b.get_width(), a.get_height(), false, Image.FORMAT_RGB8)
		out.blit_rect(a, Rect2i(Vector2i.ZERO, a.get_size()), Vector2i.ZERO)
		out.blit_rect(b, Rect2i(Vector2i.ZERO, b.get_size()), Vector2i(a.get_width(), 0))
	print("cockpit_shots: saved %s (error %d)" % [name, out.save_png(_out_dir.path_join(name + ".png"))])
