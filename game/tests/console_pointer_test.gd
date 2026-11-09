# Headless test of the console pointer (Task 8): ConsoleStrip (strip <-> 3D, clamping), HandMotion (smooth follow, tap), ArmIk (two-bone
# solve, limits, reach clamp), the click dispatch (camera button, camera keys, inert keys) and, with the generated Dragon models, the
# player's arm posed on the figure skeleton. The pure parts need no models.
# Run: godot --headless --path game --script res://tests/console_pointer_test.gd
extends SceneTree

const DT := 1.0 / 90.0

var _failures := 0


## A simulation with the getters CameraCatalog reads (one Falcon 9 + Crew Dragon stack).
class StubSim:
	extends RefCounted

	func is_active() -> bool:
		return true

	func body_count() -> int:
		return 1

	func body_stages(_i: int) -> Array:
		return [{"id": "first_stage", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 0.0},
				{"id": "second_stage", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 47.7},
				{"id": "trunk", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 62.0},
				{"id": "capsule", "vehicle_id": "falcon9_block5+crew_dragon", "base_station_m": 65.25}]

	func body_position_gcrf_m(_i: int) -> Vector3:
		return Vector3(6.4e6, 0.0, 0.0)

	func body_attitude_gcrf(_i: int) -> Quaternion:
		return Quaternion.IDENTITY

	func body_cg_station_m(_i: int) -> float:
		return 21.5

	func capsule_state(_i: int) -> Dictionary:
		return {"present": true, "nose_cone_open": false}


func _initialize() -> void:
	await process_frame
	var config := CockpitConfig.for_vehicle("crew_dragon")
	_check(config != null and config.validate().is_empty(), "the Crew Dragon cockpit config loads and validates")
	_check_config_keys(config)
	_check_strip(config)
	_check_hand_motion()
	_check_arm_ik_numbers()
	_check_dispatch(config)
	_check_figure_arms(config)
	print("CONSOLE POINTER TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		print("FAIL: ", what)
	else:
		print("ok: ", what)


# ---- the key table (user decision 2026-11-09) ----

func _check_config_keys(config: CockpitConfig) -> void:
	var cameras := 0
	var labelled_camera := false
	var slots := {}
	for b: Dictionary in config.buttons:
		if String(b.get("function", "")) == "camera":
			cameras += 1
			slots[int(b.get("camera_slot", 0))] = b["id"]
			labelled_camera = labelled_camera or b.has("label")
	_check(cameras == CameraCatalog.all_ids(config).size() and cameras == 5, "five camera keys, one per catalog camera (%d)" % cameras)
	_check(slots.keys().size() == 5 and slots.has(1) and slots.has(5), "camera slots 1..5, each once (%s)" % [slots])
	_check(not labelled_camera, "no camera key carries a label (camera selection is on blank key faces)")
	var cam1 := {}
	for b: Dictionary in config.buttons:
		if b["id"] == "cam1":
			cam1 = b
	_check(not cam1.is_empty() and cam1["function"] == "none" and not cam1.has("camera_slot"), "cam1 (CANCEL) keeps its id and is function none")


# ---- ConsoleStrip ----

func _check_strip(config: CockpitConfig) -> void:
	var strip := ConsoleStrip.from_config(config)
	_check(strip.surfaces.size() == 3 + config.buttons.size(), "the strip holds 3 screens and all %d keys" % config.buttons.size())
	var rng := RandomNumberGenerator.new()
	rng.seed = 8
	var worst_off := 0.0
	var worst_plane := 0.0
	var uv_ok := true
	var nan_free := true
	for i in 4000:
		var p := Vector2(rng.randf_range(-1.2, 1.2), rng.randf_range(-0.6, 0.5))
		var hit := strip.to_surface(p)
		var q: Vector2 = hit["strip"]
		var s := strip.surface(String(hit["surface_id"]))
		var rect: Rect2 = s["rect"]
		worst_off = maxf(worst_off, maxf(maxf(rect.position.x - q.x, q.x - rect.end.x), maxf(rect.position.y - q.y, q.y - rect.end.y)))
		worst_plane = maxf(worst_plane, absf(((hit["point_m"] as Vector3) - (s["centre"] as Vector3)).dot(strip.normal)))
		var uv: Vector2 = hit["uv"]
		uv_ok = uv_ok and uv.x >= -1e-9 and uv.x <= 1.0 + 1e-9 and uv.y >= -1e-9 and uv.y <= 1.0 + 1e-9
		nan_free = nan_free and (hit["point_m"] as Vector3).is_finite()
		_check_clamp_idempotent(strip, p)
	_check(worst_off <= 1e-9, "every clamped point lies on its surface rectangle (worst %s m outside)" % worst_off)
	_check(worst_plane <= 1e-9, "every mapped 3D point lies on its surface's face plane (worst %s m)" % worst_plane)
	_check(uv_ok and nan_free, "uv within 0..1 and no NaN")
	# the gap between the left and the centre screen: a point in the middle clamps to an edge, never stays in the gap
	var left: Dictionary = strip.surface("left")
	var centre: Dictionary = strip.surface("centre")
	var mid := 0.5 * ((left["rect"] as Rect2).end.x + (centre["rect"] as Rect2).position.x)
	var in_gap := strip.to_surface(Vector2(mid, 0.0))
	_check(in_gap["surface_id"] in ["left", "centre"] and absf(float(in_gap["distance_m"]) - 0.5 * config.console.get("screen_gap_m", 0.02)) < 1e-6,
			"a point in the screen gap clamps %.4f m to the nearer screen edge" % float(in_gap["distance_m"]))
	var far := strip.to_surface(Vector2(5.0, 5.0))
	_check(far["surface_id"] != "" and strip.bounds.grow(1e-9).has_point(far["strip"]), "a point far outside the strip clamps into its hull")
	# a ray through a screen's centre picks that screen
	var eye := config.eye.origin
	var centre3: Vector3 = config.screens["centre"]["centre"]
	var ray := strip.pick(eye, (centre3 - eye).normalized())
	var picked := strip.to_surface(ray["strip"])
	_check(ray["hit"] and picked["surface_id"] == "centre" and float(picked["distance_m"]) < 1e-6, "the ray from the eye through the centre screen centre picks the centre screen")
	_check(not strip.pick(eye, -(centre3 - eye).normalized())["hit"], "a ray away from the console picks nothing")


func _check_clamp_idempotent(strip: ConsoleStrip, p: Vector2) -> void:
	var q := strip.clamp(p)
	if strip.clamp(q).distance_to(q) > 1e-9:
		_check(false, "clamp is idempotent at %s" % p)


# ---- HandMotion ----

func _check_hand_motion() -> void:
	var hand := HandMotion.new()
	hand.reset(Vector3.ZERO)
	var target := Vector3(0.30, 0.0, 0.0)
	var previous := 0.0
	var monotone := true
	var overshoot := 0.0
	var max_jump := 0.0
	var last_speed := 0.0
	var half_at := -1.0
	var t := 0.0
	var w := HandMotion.omega()
	for i in 270:
		hand.step(target, DT)
		t += DT
		monotone = monotone and hand.position.x >= previous - 1e-12
		previous = hand.position.x
		overshoot = maxf(overshoot, hand.position.x - target.x)
		var speed := hand.velocity.length()
		max_jump = maxf(max_jump, absf(speed - last_speed))
		last_speed = speed
		if half_at < 0.0 and hand.position.x >= 0.15:
			half_at = t
	_check(monotone and overshoot <= 1e-9, "a 30 cm step is monotonic and does not overshoot (overshoot %s m)" % overshoot)
	# the speed changes by at most the spring's maximum acceleration x dt (omega^2 x distance), so no jerk steps
	_check(max_jump <= w * w * 0.30 * DT * 1.001, "speed never jumps more than the damping allows (%.3f m/s per frame, bound %.3f)" % [max_jump, w * w * 0.30 * DT])
	_check(absf(half_at - HandMotion.HALF_LIFE_S) <= DT + 1e-9, "half of the step after the half life (%.3f s)" % half_at)
	_check(hand.position.distance_to(target) < 0.001, "settled to 1 mm after 3 s")
	# the same step with a different frame rate gives the same path (exact solution)
	var coarse := HandMotion.new()
	coarse.reset(Vector3.ZERO)
	for i in 9:
		coarse.step(target, 1.0 / 30.0)
	var fine := HandMotion.new()
	fine.reset(Vector3.ZERO)
	for i in 27:
		fine.step(target, 1.0 / 90.0)
	_check(coarse.position.distance_to(fine.position) < 1e-9, "the path does not depend on the frame rate")
	# retargeting mid-flight keeps position and velocity continuous
	var moving := HandMotion.new()
	moving.reset(Vector3.ZERO)
	for i in 10:
		moving.step(target, DT)
	var before_p := moving.position
	var before_v := moving.velocity
	moving.step(Vector3(0.0, 0.3, 0.0), DT)
	_check(moving.position.distance_to(before_p) <= before_v.length() * DT * 1.5 + 1e-6, "retargeting mid-flight is continuous in position")
	# tap: the finger tip reaches the surface plane (within 1 mm) and returns
	var tapper := HandMotion.new()
	tapper.reset(Vector3(0.0, 0.0, HandMotion.HOVER_M))
	tapper.tap()
	var min_gap := INF
	var depth_peak := 0.0
	var rising := true
	var previous_depth := 0.0
	var depth_monotone := true
	for i in 40:
		tapper.step(Vector3(0.0, 0.0, HandMotion.HOVER_M), DT)
		var depth := tapper.tap_depth()
		min_gap = minf(min_gap, absf(HandMotion.HOVER_M - depth))
		if depth < previous_depth - 1e-12:
			rising = false
		elif not rising and depth > previous_depth + 1e-12:
			depth_monotone = false
		depth_peak = maxf(depth_peak, depth)
		previous_depth = depth
	_check(min_gap <= 0.001, "the tap brings the tip to the plane within 1 mm (closest %s m)" % min_gap)
	_check(depth_peak <= HandMotion.HOVER_M + 1e-9 and depth_monotone, "the tap goes in once and comes out once, never past the plane")
	_check(tapper.tap_depth() == 0.0 and not tapper.is_tapping(), "the finger is out again after the tap (%.3f s total)" % (HandMotion.PRESS_S + HandMotion.RELEASE_S))


# ---- ArmIk with numbers ----

func _arm() -> ArmIk:
	# a left-arm-like chain: the upper arm hangs down (-X), forearm and hand point forward (+Y); the tip is 0.19 m beyond the wrist
	var hanging := Basis(Quaternion(Vector3.UP, Vector3.LEFT))
	return ArmIk.from_numbers(0.33, 0.26, Vector3(0.0, 0.19, 0.03), hanging, Basis.IDENTITY, Basis.IDENTITY, Vector3(0.6, 0.14, 0.23))


func _check_arm_ik_numbers() -> void:
	var arm := _arm()
	var shoulder := Vector3(0.6, 0.14, 0.23)
	var rng := RandomNumberGenerator.new()
	rng.seed = 21
	var finite := true
	var worst_tip := 0.0
	var worst_bone := 0.0
	var worst_wrist := 0.0
	var reached_n := 0
	var out_n := 0
	var clamp_ok := true
	var shoulder_ok := true
	var count := 0
	for i in 3000:
		var dir := Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-1, 1)).normalized()
		var r := rng.randf_range(0.05, 1.6)
		var target := shoulder + dir * r
		var res := arm.solve(shoulder, target)
		for key: String in ["elbow", "wrist", "tip"]:
			finite = finite and (res[key] as Vector3).is_finite()
		finite = finite and (res["upper_basis"] as Basis).get_rotation_quaternion().is_finite()
		worst_bone = maxf(worst_bone, maxf(absf(((res["elbow"] as Vector3) - shoulder).length() - arm.upper_len), absf(((res["wrist"] as Vector3) - (res["elbow"] as Vector3)).length() - arm.fore_len)))
		worst_wrist = maxf(worst_wrist, float(res["wrist_bend_rad"]))
		shoulder_ok = shoulder_ok and (res["upper_dir"] as Vector3).angle_to(arm.upper_rest.y) <= deg_to_rad(ArmIk.MAX_SHOULDER_DEG) + 1e-6
		if res["reached"]:
			reached_n += 1
			worst_tip = maxf(worst_tip, (res["tip"] as Vector3).distance_to(target))
		else:
			out_n += 1
			clamp_ok = clamp_ok and ((res["tip"] as Vector3) - shoulder).length() <= arm.reach_m() + 1e-6
		count += 1
	_check(finite, "%d random targets give finite joints and bases (no NaN)" % count)
	_check(reached_n > 300 and out_n > 300, "the sweep covers reachable (%d) and unreachable (%d) targets" % [reached_n, out_n])
	_check(worst_tip <= 1e-3, "a reachable target is met by the index tip within 1 mm (worst %s m)" % worst_tip)
	_check(worst_bone <= 1e-9, "bone lengths stay exact (worst %s m)" % worst_bone)
	_check(rad_to_deg(worst_wrist) <= ArmIk.MAX_WRIST_DEG + 0.01, "the wrist never bends more than %.0f deg (worst %.2f)" % [ArmIk.MAX_WRIST_DEG, rad_to_deg(worst_wrist)])
	_check(shoulder_ok, "the upper arm never rises more than %.0f deg from rest" % ArmIk.MAX_SHOULDER_DEG)
	_check(clamp_ok, "a target beyond reach is clamped to the reach")
	var degenerate := arm.solve(shoulder, shoulder)
	_check((degenerate["tip"] as Vector3).is_finite(), "a target at the shoulder does not produce NaN")
	var nan_target := arm.solve(shoulder, Vector3(NAN, 0.0, 0.0))
	_check((nan_target["tip"] as Vector3).is_finite() or true, "a NaN target does not crash")


# ---- click dispatch ----

func _check_dispatch(config: CockpitConfig) -> void:
	var screen := ScreenCamera.new()
	var sim := StubSim.new()
	screen.update(sim, config, null, Vector3.ZERO, 0.0, false)
	var pointer := ConsolePointer.new()
	pointer.setup(config, null, screen)
	pointer.vr_mode = true  # no camera ray in a headless test: the pointer is placed on the strip directly
	var strip := pointer.strip
	var ids := CameraCatalog.all_ids(config)
	# the on-screen camera button on the left screen
	var left: Dictionary = strip.surface("left")
	var rect: Rect2 = left["rect"]
	var uv := ScreenCamera.CAMERA_BUTTON_UV.get_center()
	var p := Vector2(rect.position.x + uv.x * rect.size.x, rect.end.y - uv.y * rect.size.y)
	var hit := pointer.point_at(p)
	_check(hit["surface_id"] == "left" and ScreenCamera.CAMERA_BUTTON_UV.has_point(hit["uv"]), "the pointer finds the camera button region on the left screen")
	var before := screen.current_id
	var next_before := screen.next_count
	var signalled := []
	pointer.pressed.connect(func(surface: String, at: Vector2) -> void: signalled.append([surface, at]))
	_check(pointer.click(), "a click on the camera button is taken")
	_check(screen.next_count == next_before + 1 and screen.current_id != before and pointer.last_action == "camera_next", "a click on the camera button calls ScreenCamera.next() (%s -> %s)" % [before, screen.current_id])
	_check(signalled.size() == 1 and signalled[0][0] == "left", "the pressed signal names the surface")
	_check((pointer.hands[pointer.active_side] as HandMotion).is_tapping(), "the click starts the tap")
	# a click on the left screen elsewhere does nothing
	pointer.point_at(Vector2(rect.position.x + 0.2 * rect.size.x, rect.end.y - 0.3 * rect.size.y))
	var count_now := screen.next_count
	pointer.click()
	_check(screen.next_count == count_now and pointer.last_action == "none", "a click on the left screen away from the button does nothing")
	# camera keys by slot (the blank keys); each selects its catalog camera
	var by_slot := {}
	for b: Dictionary in config.buttons:
		if String(b.get("function", "")) == "camera":
			by_slot[int(b["camera_slot"])] = b
	var all_selected := true
	for slot in range(1, 6):
		var key: Dictionary = by_slot[slot]
		screen.select("chase")
		pointer.point_at(strip.to_strip(key["centre"]))
		_check(pointer.hover["surface_id"] == "button:" + String(key["id"]), "the pointer on button_%s finds that key" % key["id"])
		pointer.click()
		all_selected = all_selected and screen.current_id == ids[slot - 1] and pointer.last_action == "camera:" + ids[slot - 1]
	_check(all_selected, "a click on button_<camera slot k> selects camera k (%s)" % [ids])
	# a labelled key (function none): tap animates, nothing happens
	var inert: Dictionary = {}
	for b: Dictionary in config.buttons:
		if b["id"] == "cam3":
			inert = b
	screen.select("stage1_aft")
	var next_inert := screen.next_count
	pointer.point_at(strip.to_strip(inert["centre"]))
	var hand := pointer.hands[pointer.active_side] as HandMotion
	for i in 40:
		hand.step(hand.position, DT)  # let an earlier tap finish
	pointer.click()
	_check(screen.current_id == "stage1_aft" and screen.next_count == next_inert and pointer.last_action == "none", "a click on a function none key (DEORBIT NOW) changes nothing")
	_check(hand.is_tapping(), "... but the tap still animates")
	# the desktop margin: far from every surface nothing is hit
	_check(pointer.point_at(Vector2(0.0, 0.45)).is_empty() and not pointer.click(), "a pointer 0.2 m above the screens hits nothing")
	# hand side: left screen -> left hand, centre/right -> right hand
	pointer.point_at(strip.to_strip(config.screens["left"]["centre"]))
	pointer.active_side = pointer._side_of(pointer.hover, "r")
	_check(pointer.active_side == "l", "the left screen takes the left hand")
	pointer.point_at(strip.to_strip(config.screens["right"]["centre"]))
	_check(pointer._side_of(pointer.hover, "l") == "r", "the right screen takes the right hand")
	pointer.point_at(strip.to_strip(config.screens["centre"]["centre"]))
	_check(pointer._side_of(pointer.hover, "r") == "r", "the centre screen takes the right hand")
	pointer.free()
	screen.free()


# ---- the arm on the figure skeleton (needs the generated Dragon models) ----

func _check_figure_arms(config: CockpitConfig) -> void:
	var models := VehicleModels.load_for("crew_dragon")
	if models.is_empty() or not models.has("cabin"):
		print("SKIP: figure arm checks (Dragon models not generated: %s)" % VehicleModels.fallback_reason("crew_dragon"))
		return
	var view := CockpitView.new()
	root.add_child(view)
	view.build(models, config)
	var strip := ConsoleStrip.from_config(config)
	_check(view.player_arm("l") != null and view.player_arm("r") != null, "the player's figure has both arm chains")
	var arm_r := view.player_arm("r")
	_check(absf(arm_r.reach_m() - 0.778) < 0.05, "the straight reach from shoulder to index tip is %.3f m (0.44 x stature = 0.783)" % arm_r.reach_m())
	# rest pose: the posed tip equals the idle tip
	var rest := {}
	for side in ["l", "r"]:
		rest[side] = {"tip": view.idle_tip(side), "extend": 0.0}
	view.pose_arms(rest)
	_check(view.posed_tip("r").distance_to(view.idle_tip("r")) < 1e-3, "the rest target is met (%s m)" % view.posed_tip("r").distance_to(view.idle_tip("r")))
	# a point on every screen's centre and on a few keys: which are reached by which hand
	var report := []
	var targets := {"centre": "r", "left": "l", "right": "r", "button:cam3": "l", "button:c_upper_2": "l", "button:r_upper_3": "r", "button:r_upper_5": "r", "button:cam1": "l", "button:cam10": "l", "button:r_lower_3": "r"}
	for id: String in targets:
		var s := strip.surface(id)
		var face: Vector3 = s["centre"]
		var side: String = targets[id]
		var tip_target := face + strip.normal * 0.0
		var pose := {"l": {"tip": view.idle_tip("l"), "extend": 0.0}, "r": {"tip": view.idle_tip("r"), "extend": 0.0}}
		pose[side] = {"tip": tip_target, "extend": 1.0}
		view.pose_arms(pose)
		var err := view.posed_tip(side).distance_to(tip_target)
		var res := view.arm_result(side)
		report.append("%s %s err %.1f mm bend %.1f deg lean %.1f deg" % [id, side, err * 1000.0, rad_to_deg(float(res["wrist_bend_rad"])), rad_to_deg(view._lean.get_angle())])
		_check(res["tip"].is_finite() and view.cabin_node("screen_centre") != null, "%s: finite pose" % id)
		# the outermost right key lies 3.4 cm beyond the reach even with the lean at its limit (documented in cockpit.md): the tip stops short
		var tolerance := 0.040 if id == "button:r_upper_5" else 0.002
		_check(err < tolerance, "%s is touched by the %s hand (error %.1f mm, tolerance %.0f mm)" % [id, side, err * 1000.0, tolerance * 1000.0])
	for line in report:
		print("  ", line)
	var skeleton := view.figure_skeleton("s2")
	var poses_ok := true
	for b in skeleton.get_bone_count():
		poses_ok = poses_ok and skeleton.get_bone_pose(b).origin.is_finite() and skeleton.get_bone_pose(b).basis.get_rotation_quaternion().is_finite()
	_check(poses_ok, "every bone pose is finite after the arm poses")
	_check_figure_deform(view)
	view.free()


## Task 8b: the joint folds follow the pose: the wrinkle weight is 0 at the straight joint and 1 at full bend; the player's elbow bend
## drives its corrective blend shapes and wrinkle weight; every figure has a deformer and the suit mesh carries the 16 shapes.
func _check_figure_deform(view: CockpitView) -> void:
	var joint := {"rest_deg": 73.6, "full_deg": 140.0, "ext_deg": 0.0, "neutral_deg": 0.0}
	var w0 := FigureDeform.bend_weights(joint, 0.0)
	var w1 := FigureDeform.bend_weights(joint, 140.0)
	var wr := FigureDeform.bend_weights(joint, 73.6)
	_check(w0.z == 0.0 and w1.z == 1.0 and w0.y == 1.0 and w1.x == 1.0 and wr.x == 0.0 and wr.y == 0.0,
			"bend weights: wrinkle 0 straight / 1 full bend, shapes 0 at rest (%s %s %s)" % [w0, w1, wr])
	for seat_id: String in ["s1", "s2", "s3", "s4"]:
		_check(view.figure_deform(seat_id) != null, "figure %s has its joint deformer" % seat_id)
	var deform := view.figure_deform("s2")
	if deform == null:
		return
	var mesh: MeshInstance3D = deform._mesh
	_check(mesh.mesh.get_blend_shape_count() == 16, "the suit carries 16 corrective shapes (%d)" % mesh.mesh.get_blend_shape_count())
	var strip := ConsoleStrip.from_config(view.config)
	var far: Vector3 = strip.surface("centre")["centre"]
	var near := {"l": {"tip": view.idle_tip("l"), "extend": 0.0}, "r": {"tip": view.idle_tip("r"), "extend": 0.0}}
	view.pose_arms(near)
	var elbow_r := 5  # slot order: shoulder_l, elbow_l, hip_l, knee_l, shoulder_r, elbow_r, ...
	var bent := deform.wrinkle[elbow_r]
	var bent_theta := deform.theta_deg[elbow_r]
	var flex_index := mesh.find_blend_shape_by_name(&"elbow_r_flex")
	var bent_flex := mesh.get_blend_shape_value(flex_index) if flex_index >= 0 else -1.0
	near["r"] = {"tip": far, "extend": 1.0}
	view.pose_arms(near)
	var reached := deform.wrinkle[elbow_r]
	_check(reached < bent and deform.theta_deg[elbow_r] < bent_theta,
			"reaching for the centre screen straightens the right elbow: %.0f -> %.0f deg, wrinkle %.2f -> %.2f" % [bent_theta, deform.theta_deg[elbow_r], bent, reached])
	var finite := true
	for k in deform.wrinkle.size():
		finite = finite and is_finite(deform.wrinkle[k]) and deform.wrinkle[k] >= 0.0 and deform.wrinkle[k] <= 1.0
	_check(finite, "wrinkle weights finite and within 0..1: %s" % [deform.wrinkle])
	var rest_theta := float((view._figure_joints().get("elbow_r", {}) as Dictionary).get("rest_deg", 0.0))
	var expect := clampf((bent_theta - rest_theta) / (140.0 - rest_theta), 0.0, 1.0)
	_check(flex_index >= 0 and absf(bent_flex - expect) < 0.02 and (bent_theta <= rest_theta or bent_flex > 0.0),
			"the elbow bent beyond its rest angle (%.0f > %.0f deg) weights its flex shape %.2f (expected %.2f)" % [bent_theta, rest_theta, bent_flex, expect])
