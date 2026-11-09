# Headless test of the Crew Dragon cabin in the game (CockpitView) in REAL sessions: one cabin per vehicle view at T+0, after
# staging (T+170 s) and after the trunk jettison of crew_dragon_return; the eye node at the capsule stage transform * config
# eye; set_crew; the exterior hull casting double-sided shadows. Needs the generated models (python tools/structures/
# build_models.py --vehicle crew_dragon) and the built GDExtension.
# Run: godot --headless --path game --script res://tests/cockpit_view_test.gd
extends SceneTree

const EYE_TOLERANCE_M := 0.001  # Task 5 brief
const SIM_TICK_S := 0.005

var _failures := 0
var _core: StarsimCore
var _config_dir := ""


func _initialize() -> void:
	await process_frame  # the root is in the tree from the first frame on (global transforms)
	var config := CockpitConfig.for_vehicle("crew_dragon")
	_check(config != null and config.capsule_stage_id == "capsule", "cockpit config of crew_dragon")
	var models := VehicleModels.load_for("crew_dragon")
	if models.is_empty() or not models.has("cabin"):
		_check(false, "crew_dragon models with the cabin generated (%s)" % VehicleModels.fallback_reason("crew_dragon"))
		quit(1)
		return
	_core = StarsimCore.new()
	_core.open(ProjectSettings.globalize_path("res://").path_join("../data/spice").simplify_path())
	_config_dir = ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	_check_figures(config)
	_check_launch()
	_check_return()
	print("COCKPIT VIEW TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		print("FAIL: ", what)
	else:
		print("ok: ", what)


func _session(mission: String) -> FlightSim:
	var sim := FlightSim.new()
	sim.setup(_config_dir)
	_core.load_terrain(_config_dir)
	var id := sim.start_mission(mission, _core.utc_to_tdb_s("2024-04-08T18:42:00"), false)
	sim.wait_idle(600.0)
	print("  ", mission, " start: ", sim.result(id))
	return sim


func _advance(sim: FlightSim, target: float) -> void:
	for i in 1000:
		sim.advance_to(target)
		sim.wait_idle(5.0)
		if sim.advance_to(target) >= target - 0.01 or not sim.is_active():
			return


func _zero(sim: FlightSim) -> float:
	var t0 = sim.mission_timeline().get("t0_tdb_s")
	return float(t0) if t0 != null else sim.mission_start_tdb_s()


## The views of the sim's bodies, updated once (floating origin at the first body).
func _views(sim: FlightSim) -> Array[VehicleView]:
	var views: Array[VehicleView] = []
	for i in sim.body_count():
		var view := VehicleView.new()
		view.body_index = i
		root.add_child(view)
		view.update_view(sim, sim.body_position_gcrf_m(0))
		views.append(view)
	return views


func _cockpits(views: Array[VehicleView]) -> Array[CockpitView]:
	var found: Array[CockpitView] = []
	for view in views:
		for node in view.find_children("cockpit", "Node3D", true, false):
			if node is CockpitView:
				found.append(node)
	return found


func _check_launch() -> void:
	var sim := _session("falcon9_crew_dragon")
	var zero := _zero(sim)
	_advance(sim, zero + SIM_TICK_S)
	var views := _views(sim)
	var cockpits := _cockpits(views)
	_check(cockpits.size() == 1, "T+0: exactly one CockpitView (%d)" % cockpits.size())
	if cockpits.size() == 1:
		_check_eye(cockpits[0])
		_check_hull_shadows(cockpits[0])
		_check_crew(cockpits[0])
	for view in views:
		view.free()
	_advance(sim, zero + 170.0)
	views = _views(sim)
	cockpits = _cockpits(views)
	_check(cockpits.size() == 1, "after staging (T+170 s, %d bodies): still exactly one CockpitView (%d)" % [sim.body_count(), cockpits.size()])
	for view in views:
		view.free()


func _check_return() -> void:
	var sim := _session("crew_dragon_return")
	var kinds := []
	for e: Dictionary in sim.mission_timeline().get("events", []):
		kinds.append(String(e["kind"]))
	print("  crew_dragon_return events: ", kinds)
	var target := 0.0
	for e: Dictionary in sim.mission_timeline().get("events", []):
		if e["kind"] == "entry_interface":
			target = float(e["tdb_s"]) + 10.0  # the trunk goes after the deorbit burn, before the entry interface
	_check(target > 0.0, "crew_dragon_return has an entry_interface event")
	_advance(sim, target)
	var views := _views(sim)
	var cockpits := _cockpits(views)
	var stage_ids: Array = []
	var together := false
	for i in sim.body_count():
		var ids: Array = sim.body_stages(i).map(func(st: Dictionary) -> String: return st["id"])
		stage_ids.append(ids)
		together = together or (ids.has("trunk") and ids.has("capsule"))
	_check(not together and sim.body_count() == 2, "the trunk is a body of its own (bodies %s)" % [stage_ids])
	_check(cockpits.size() == 1, "after the trunk jettison (stages %s): exactly one CockpitView (%d)" % [stage_ids, cockpits.size()])
	for view in views:
		view.free()


## The eye node at (capsule stage transform * config eye) within 1 mm.
func _check_eye(cockpit: CockpitView) -> void:
	var stage := cockpit.get_parent() as Node3D
	var expected := stage.global_transform * cockpit.config.eye.origin
	var error := cockpit.eye_node.global_position.distance_to(expected)
	_check(error < EYE_TOLERANCE_M, "eye node at capsule stage * config eye (%.6f m)" % error)
	_check(stage.name == "stage_capsule", "the cabin is a child of the capsule stage node (%s)" % stage.name)
	_check(cockpit.screen_surface("centre") != null and cockpit.screen_surface("left") != null and cockpit.screen_surface("right") != null, "three screen surfaces")
	_check(cockpit.button_node("handle") != null and cockpit.button_node("cam1") != null, "button nodes (handle, cam1)")
	_check(cockpit.screen_surface("centre").material_override.shader.resource_path.ends_with("display_colour.gdshader"), "screens use the display-referred colour shader")
	var ring := cockpit.cabin_node("ceiling_ring_light")
	_check(ring != null and ring.material_override.shader.resource_path.ends_with("display_colour.gdshader") and ring.cast_shadow == GeometryInstance3D.SHADOW_CASTING_SETTING_OFF,
			"ceiling ring light: an emissive (display-referred) mesh")
	var shell := cockpit.cabin_node("seat_shell_s2")
	var pad := cockpit.cabin_node("seat_pad_s2")
	_check(shell != null and pad != null and shell.material_override != pad.material_override, "seats: shell and pad are separate nodes with separate materials")
	var shell_albedo: Color = (shell.material_override as ShaderMaterial).get_shader_parameter("albedo")
	var pad_albedo: Color = (pad.material_override as ShaderMaterial).get_shader_parameter("albedo")
	# Task 5b: the uniform is sRGB-encoded; the pads are now a LINEAR 0.035 converted (sRGB 0.2), so compare linear albedos
	_check(shell_albedo.srgb_to_linear().get_luminance() > 0.6 and pad_albedo.srgb_to_linear().get_luminance() < 0.1, "seats: white shells, black pads")
	# Task 5b: labelled keys carry the label atlas cell (conv_42860a), keys without a legible label a blank face; red strips under the shelf modules; the
	# handle housing's red marking
	var labelled := (cockpit.button_node("cam3") as MeshInstance3D).material_override as ShaderMaterial
	var blank := (cockpit.button_node("r_led_1") as MeshInstance3D).material_override as ShaderMaterial
	_check(float(labelled.get_shader_parameter("label_on")) > 0.0 and labelled.get_shader_parameter("label_atlas") != null and float(blank.get_shader_parameter("label_on")) == 0.0,
			"keys: DEORBIT NOW (cam3) has its label, r_led_1 a blank face")
	var shelf := (cockpit.cabin_node("console_shelf").material_override as ShaderMaterial)
	var handle := ((cockpit.button_node("handle") as MeshInstance3D).material_override as ShaderMaterial)
	_check(float(shelf.get_shader_parameter("mark_on")) > 0.0 and float(handle.get_shader_parameter("mark_on")) > 0.0, "shelf red strips and the handle's red marking")
	_check(cockpit.cabin_node("console_shelf") != null and cockpit.cabin_node("console_back") != null and cockpit.cabin_node("console_lockers") != null, "console: shelf modules, back and lockers are nodes of their own")


## The capsule's exterior hull parts cast shadows double-sided (the sunlight enters only through the windows).
func _check_hull_shadows(cockpit: CockpitView) -> void:
	var stage := cockpit.get_parent()
	var hulls := 0
	var bad := 0
	for child in stage.get_children():
		if child is MeshInstance3D:
			hulls += 1
			if (child as MeshInstance3D).cast_shadow != GeometryInstance3D.SHADOW_CASTING_SETTING_DOUBLE_SIDED:
				bad += 1
	_check(hulls > 0 and bad == 0, "capsule hull parts cast shadows double-sided (%d parts, %d not)" % [hulls, bad])


func _check_crew(cockpit: CockpitView) -> void:
	cockpit.set_crew(2)
	_check(cockpit.visible_figures() == 2, "set_crew(2): the player body and one figure (%d visible)" % cockpit.visible_figures())
	cockpit.set_crew(4)
	_check(cockpit.visible_figures() == 4, "set_crew(4): four figures")
	cockpit.set_crew(0)
	_check(cockpit.visible_figures() == 0, "set_crew(0): none")
	_check(cockpit.figure_skeleton("s2") != null and cockpit.figure_skeleton("s2").get_bone_count() == 31, "commander figure keeps its 31-bone skeleton")
	cockpit.set_hotas_props(false)
	_check(not cockpit.cabin_node("hotas_stick").visible, "HOTAS props switch off")
	cockpit.set_hotas_props(true)


## The figures placed on the seats: the commander's hip joint stays near the s2 seat's hip point.
func _check_figures(config: CockpitConfig) -> void:
	var cockpit := CockpitView.new()
	root.add_child(cockpit)
	cockpit.build(VehicleModels.load_for("crew_dragon"), config)
	var skeleton := cockpit.figure_skeleton("s2")
	_check(skeleton != null, "figure skeleton of s2 loaded through the scene path")
	if skeleton != null:
		var hips := skeleton.get_bone_global_pose(skeleton.find_bone("hips")).origin
		var world := (skeleton.global_transform * Transform3D(Basis(), hips)).origin
		var hip: Vector3 = config.seats[1]["hip"]
		_check(world.distance_to(hip) < 0.2, "s2 figure's hips %.3f m from the seat hip point" % world.distance_to(hip))
		# orientation (the seat basis is duplicated from figure_layout.seat_basis): the head sits near the eye point
		var head_bone := skeleton.find_bone("head")
		var head := (skeleton.global_transform * Transform3D(Basis(), skeleton.get_bone_global_pose(head_bone).origin)).origin
		# round 4: the config eye IS the figure's eye, so the player's body sits under the camera. The eye's offset from the head bone's head comes
		# from the manifest (crew_figure eye_m and bones.head.head, figure frame: x up the back, y the chest normal); Task 5b: was the literal
		# 0.20 / 0.06 of the round-4 helmet (half_below 0.17 + EYE_UP_M 0.03)
		var figure_manifest: Dictionary = (VehicleModels.load_for("crew_dragon").get("manifest", {}) as Dictionary).get("crew_figure", {})
		var eye_m: Array = figure_manifest.get("eye_m", [0.0, 0.0, 0.0])
		var head_m: Array = figure_manifest.get("bones", {}).get("head", {}).get("head", [0.0, 0.0, 0.0])
		var back := deg_to_rad(float(config.seats[1]["back_angle_deg"]))
		var figure_eye := head + Vector3(cos(back), -sin(back), 0.0) * (float(eye_m[0]) - float(head_m[0])) 				+ Vector3(sin(back), cos(back), 0.0) * (float(eye_m[1]) - float(head_m[1]))
		_check(figure_eye.distance_to(config.eye.origin) < 0.02, "s2 figure's eye %.3f m from the eye point" % figure_eye.distance_to(config.eye.origin))
	cockpit.free()
