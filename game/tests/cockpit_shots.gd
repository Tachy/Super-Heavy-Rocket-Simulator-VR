# Look shots of the Crew Dragon cabin from a REAL session (needs a GPU, no --headless; models generated):
#   pilot_eye              the commander's eye point (CockpitConfig.eye), turned toward the screen row, 105 deg horizontal (no photo pair)
#   gopro_discovery2       the Discovery photographers' GoPro (pose fitted on discovery2's screen corners, docs/subsystems/cockpit.md),
#                          rendered rectilinear 140 deg and remapped to the GoPro's equidistant fisheye (113 x 64 deg, 1600 x 900);
#                          + gopro_vs_discovery2 / gopro_overlay_discovery2 and, as discovery1 has the same framing, gopro_vs_discovery1
#   hatch_crew1_1          from the open hatch toward the seat row (3:2, fitted: 96 deg horizontal, 8 deg up), hatch frame and exterior hull
#                          hidden; + hatch_vs_crew1_1
# godot --path game --script res://tests/cockpit_shots.gd -- --mission=falcon9_crew_dragon --t_plus=-60 --out_dir=<abs>/screenshots/m7-cockpit/look
#   --fov=100 (pilot_eye, horizontal deg)  --crew=<n> (default the mission's)  --refs=<abs dir of the reference photos>
#   --field_key=<0.18..1> (the eye adaptation's field grey for every shot, default the shot's key or CABIN_FIELD_KEY; 0.18 = the game default;
#   not used with the game's --anchored_adaptation)
#   --fixed_exposure (eye adaptation off; default: on as in the game, snapped to each shot)
extends SceneTree

const FRAMES_TO_SETTLE := 10
const SHOT_SETTLE_FRAMES := 40
const NEAR_M := 0.04
## GoPro fit (docs/subsystems/cockpit.md "Shot cameras"): position, forward and right (capsule frame), equidistant fisheye focal
## length in px / rad on the 4128 x 2322 photo (the output scales it by 1600 / 4128).
## Round 4: the fit is relative to the screens (0.61 m along their normal, 0.23 m along their up from the centre screen centre), so it moved
## rigidly with the console block (rotated 10 deg about Z, centre screen centre (2.671, 0.050) -> (2.512, -0.043)).
const GOPRO_POS := Vector3(2.2437, -0.6322, -0.0251)
const GOPRO_FORWARD := Vector3(0.32649, 0.94528, 0.00729)
const GOPRO_RIGHT := Vector3(-0.02409, 0.01602, -0.99958)
const GOPRO_F_PX := 2085.47
const GOPRO_PHOTO_SIZE := Vector2(4128.0, 2322.0)
const GOPRO_OUT_SIZE := Vector2i(1600, 900)
const GOPRO_RENDER_FOV := 140.0  # rectilinear, covers the fisheye frame's corners (65 deg off axis)
## [call] Field key of the white cabin (Atmosphere.field_key; 0.18 = the default grey world): the wall reads ~0.85-0.9 sRGB as in the photos.
const CABIN_FIELD_KEY := 1.2
const RENDER_SIZE := Vector2i(2560, 1440)
## crew1_1 camera, round 4 joint fit (docs/subsystems/cockpit.md "Fix round 4"): position, pitch above level, vertical fov of the 1200 x 800 photo
## (f 536 px: 96.4 x 73.4 deg); residual 1-8 px on windows, helmets, console back and ceiling.
const HATCH_POS := Vector3(1.839, 1.454, 0.013)
const HATCH_PITCH_DEG := 8.2
const HATCH_VFOV := 73.4
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
	_world.atmosphere.field_key = float(_args.get("field_key", CABIN_FIELD_KEY))
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
		if not _args.has("only"):
			_compose()
		quit(0)
		return true
	var shot: Dictionary = _shots[_index]
	_cockpit.set_player_head_hidden(shot.get("local").origin.is_equal_approx(_cockpit.config.eye.origin) or bool(shot.get("hide_head", false)))
	var isolate := bool(shot.get("isolate", false))  # close-up of one figure: the other figures and the leg rests are hidden
	for figure in _cockpit.find_children("figure_*", "Node3D", false, false):
		(figure as Node3D).visible = not isolate or figure.name == "figure_s3"
	for part: String in ["seat_rest_", "seat_shell_s1", "seat_shell_s2", "seat_shell_s4", "seat_pad_s1", "seat_pad_s2", "seat_pad_s4", "console_", "screen_", "tablet_", "button_", "handle"]:
		for node in _cockpit.find_children(part + "*" if part.ends_with("_") else part, "MeshInstance3D", false, false):
			(node as Node3D).visible = not isolate
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
		var key := float(shot.get("field_key", CABIN_FIELD_KEY))
		_world.atmosphere.field_key = float(_args["field_key"]) if _args.has("field_key") else key
		_world.atmosphere.snap_adaptation()
	if _wait < SHOT_SETTLE_FRAMES:
		return false
	var path := _out_dir.path_join(String(shot["name"]) + ".png")
	var image := root.get_viewport().get_texture().get_image()
	image.convert(Image.FORMAT_RGB8)
	if shot.has("fisheye"):
		image = _fisheye(image)
	elif shot.has("crop_aspect"):
		var width := roundi(image.get_height() * float(shot["crop_aspect"]))
		image = image.get_region(Rect2i((image.get_width() - width) / 2, 0, width, image.get_height()))
	print("cockpit_shots: saved %s (error %d), %s" % [path, image.save_png(path), _brightness(image)])
	var a := _world.atmosphere
	print("cockpit_shots: %s adaptation rule %s: gain %.3f, L_a %s cd/m^2, field %s, anchor %s, field key %s, from the field %s" % [
			shot["name"], "analytic" if not a.scene_referred_adaptation else a.field_rule_name(),
			a.exposure_gain, a.adaptation_cd_m2, a.last_field_cd_m2, a.last_anchor_cd_m2, a.field_key, a.adaptation_from_field])
	print("cockpit_shots: %s adaptation surround: frame alone %s cd/m^2, model in frame %s, visibility %.3f, out-of-frame share %.3f" % [
			shot["name"], a.adaptation_surround.last_frame_field_cd_m2, a.adaptation_surround.last_model_in_cd_m2,
			a.adaptation_surround.last_visibility, a.adaptation_surround.last_out_share])
	_index += 1
	_wait = 0
	return false


func _define_shots() -> void:
	var eye := _cockpit.config.eye
	var gopro_up := (-GOPRO_FORWARD).cross(GOPRO_RIGHT).normalized()
	var gopro_basis := Basis(GOPRO_RIGHT, gopro_up, -GOPRO_FORWARD)
	# hatch: looks along -Y (toward the seat row), image up = +X (toward the forward tunnel), image right = +Z
	var pitch := deg_to_rad(HATCH_PITCH_DEG)
	var hatch_basis := Basis(Vector3(0.0, 0.0, 1.0), Vector3(cos(pitch), sin(pitch), 0.0), Vector3(-sin(pitch), cos(pitch), 0.0))
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
	for view: Array in [["helmet_34", 38.0, 0.75, 48.0], ["helmet_side", 62.0, 0.75, 48.0]]:  # like-for-like with eva_suit_detail2 (three-quarter), detail1 / demo1_2 (more from the side)
		var az := deg_to_rad(float(view[1]))
		var cam := helmet_c + (chest * cos(az) + Vector3(0.0, 0.0, 1.0) * sin(az)) * float(view[2])
		helmet_views.append({"name": view[0], "local": Transform3D(Basis.looking_at(helmet_c - cam, up_back), cam), "fov": float(view[3]), "hide_exterior": false, "isolate": true})
	var rest: Array = [
		{"name": "helmet_closeup", "local": Transform3D(Basis.looking_at(helmet_c - helmet_cam, up_back), helmet_cam), "fov": 48.0, "hide_exterior": false, "isolate": true},
		{"name": "pilot_eye", "local": pilot, "fov": PILOT_FOV, "field_key": PILOT_FIELD_KEY},
		{"name": "gopro_discovery2", "local": Transform3D(gopro_basis, GOPRO_POS), "fov": GOPRO_RENDER_FOV, "fisheye": true},
		# [E] crew1_1 (docs/subsystems/cockpit.md "Fix round 4"): the fitted camera, 3:2
		{"name": "hatch_crew1_1", "local": Transform3D(hatch_basis, HATCH_POS), "fov": HATCH_VFOV, "keep_height": true,
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
func _fisheye(src: Image) -> Image:
	var out := Image.create(GOPRO_OUT_SIZE.x, GOPRO_OUT_SIZE.y, false, Image.FORMAT_RGB8)
	var f := GOPRO_F_PX * GOPRO_OUT_SIZE.x / GOPRO_PHOTO_SIZE.x
	var px_per_tan := src.get_width() * 0.5 / tan(deg_to_rad(GOPRO_RENDER_FOV) * 0.5)
	var sw := src.get_width()
	var sh := src.get_height()
	var centre := Vector2(GOPRO_OUT_SIZE) * 0.5
	for y in GOPRO_OUT_SIZE.y:
		for x in GOPRO_OUT_SIZE.x:
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
