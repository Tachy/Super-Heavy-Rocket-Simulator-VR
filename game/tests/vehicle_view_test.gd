# Headless test of VehicleView with the Starship / Super Heavy detail models, driven by synthetic snapshots built from
# config/vehicles/starship_block3.toml (needs data/models/starship_block3 from tools/structures/build_models.py
# --vehicle starship_block3; without them only the cylinder cases run). The core's flap / grid-fin definitions
# (core/src/aero/flap.cpp, grid_fin.cpp) are re-implemented below, independent of the manifest signs the view uses.
# (The headless renderer keeps no MultiMesh transforms: instance counts are read, poses come from view.engine_pose.)
# Run: godot --headless --path game --script res://tests/vehicle_view_test.gd
extends SceneTree

const VEHICLE := "starship_block3"
const FLAP_STATES_DEG: Array[float] = [0.0, 35.0, 90.0]
const FIN_STATES_DEG: Array[float] = [0.0, -20.0, 30.0]
const EPS := 1e-4

var _failures := 0
var _views: Array[VehicleView] = []


## The FlightSim getters update_view reads, for one body.
class _StubSim:
	var stages: Array = []
	var capsule := {}

	func body_stages(_i: int) -> Array: return stages
	func body_position_gcrf_m(_i: int) -> Vector3: return Vector3.ZERO
	func body_engines(_i: int) -> Array: return []
	func body_flaps(_i: int) -> Array: return []
	func body_grid_fins(_i: int) -> Array: return []
	func body_legs(_i: int) -> Dictionary: return {}
	func telemetry(_i: int) -> Dictionary: return {}
	func capsule_state(_i: int) -> Dictionary: return capsule
	func body_attitude_gcrf(_i: int) -> Quaternion: return Quaternion.IDENTITY
	func body_cg_station_m(_i: int) -> float: return 0.0


func _initialize() -> void:
	var stages := _config_stages()
	var snapshot := _snapshot(stages)
	var old := _view(snapshot, VEHICLE + "x")  # not the prototype vehicle
	_check(_count(old, "MultiMeshInstance3D") == 0 and old.get_child(0).find_children("stage_*", "Node3D", false,
			false).is_empty() and _cylinders(old) == 3, "other vehicle: cylinders only (booster 1, ship barrel + nose)")
	_check(VehicleModels.fallback_reason(VEHICLE + "x") == "none" and VehicleModels.notice(VEHICLE + "x") == "",
			"a vehicle without models by design: reason none, no notice")
	_check(ModelNotice.text_for("stale").contains("veraltet") and ModelNotice.text_for("missing").contains("fehlen")
			and ModelNotice.text_for("none") == "" and ModelNotice.label("") == null, "notice texts: stale / missing / none")
	var no_snapshot_id := _view(snapshot, "")
	_check(_cylinders(no_snapshot_id) == 3, "a snapshot without vehicle_id keeps the cylinders")
	_check_other_model_vehicles(snapshot)
	_check(_check_falcon9() == true, "check_falcon9 ran to its end (an aborted check is a failure)")
	_check(_check_dragon() == true, "check_dragon ran to its end (an aborted check is a failure)")
	_check(_check_iss() == true, "check_iss ran to its end (an aborted check is a failure)")
	var models := VehicleModels.load_for(VEHICLE)
	if models.is_empty():
		print("  (no models: model cases skipped; run python tools/structures/build_models.py --vehicle %s)" % VEHICLE)
	else:
		_check_models(stages, snapshot, models)
		VehicleModels._cache[VEHICLE] = {}  # as without generated models
		var fallback := _view(snapshot, VEHICLE)
		_check(_cylinders(fallback) == 3 and _count(fallback, "MultiMeshInstance3D") == 0,
				"without models the vehicle keeps its cylinders")
		VehicleModels._reasons[VEHICLE] = "stale"
		var stale := _view(snapshot, VEHICLE)
		_check(stale.find_children("ModelNotice", "Label3D", true, false).size() == 1
				and stale.find_children("ModelNotice", "Label3D", true, false)[0].text.contains("veraltet"),
				"stale models: the view carries the German notice")
		_check(fallback.find_children("ModelNotice", "Label3D", true, false).is_empty(), "a fallback without a stale reason: no notice")
		var lit: Array = snapshot["engines"].duplicate(true)
		for engine: Dictionary in lit:
			engine.merge({"firing": true, "spool": 1.0, "throttle": 1.0}, true)
		fallback.sync(snapshot["stages"], lit, [], [], 0.0)
		_check(_plume_top(fallback, 0).is_equal_approx(Frames.to_render(lit[0]["position_m"] - Vector3(3.0, 0.0, 0.0))),
				"cylinders: plume starts 3 m below the gimbal point (former value)")
	for view in _views:  # free what the cases built, so the engine's exit report stays free of leaked objects
		view.free()
	print("VEHICLE VIEW TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


## Falcon 9 first stage: the leg rig against the contact physics' points and the stowed envelope, no flip at mid stroke, the view's
## nodes on a stack and alone. Rig and meshes from the generated models; without them the case fails: the models are a build step
## of this vehicle (python tools/structures/build_models.py --vehicle falcon9_block5).
func _check_falcon9() -> bool:
	VehicleModels.clear_cache()
	var models := VehicleModels.load_for("falcon9_block5")
	if models.is_empty():
		_check(false, "falcon9_block5: models generated (%s)" % VehicleModels.fallback_reason("falcon9_block5"))
		return false
	_check(VehicleModels.notice("falcon9_block5") == "", "falcon9_block5: no missing-model notice")
	var path := ProjectSettings.globalize_path("res://").path_join("../config/vehicles/falcon9_block5.toml")
	var first: Dictionary = TomlLite.parse("[[stage]]" + FileAccess.get_file_as_string(path).split("[[stage]]", false)[1])
	var legs: Dictionary = first["stage"][0]["landing_legs"]
	var hull_radius: float = first["stage"][0]["diameter_m"] * 0.5
	var rig: Dictionary = models["manifest"]["leg"]
	var foot_local := Vector3(float(rig["length_m"]), 0.0, 0.0)
	var envelope := hull_radius + float(rig["stowed_thickness_m"])
	var vertices: PackedVector3Array = (models["leg"]["leg_main"] as Mesh).surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
	for azimuth: float in legs["azimuths_deg"]:
		# deployment 0: inside the hull envelope + the stowed thickness
		var stowed: Transform3D = VehicleModels.leg_poses(rig, azimuth, 0.0)["main"]
		var worst := 0.0
		for vertex in vertices:
			var p := stowed * vertex
			worst = maxf(worst, Vector2(p.y, p.z).length())
		_check(worst <= envelope + EPS, "leg az %.0f stowed: r %.3f <= %.3f" % [azimuth, worst, envelope])
		# deployment 1: the foot on the contact physics' point
		var e_r := _radial(azimuth)
		var target := Vector3(legs["foot_station_m"], 0.0, 0.0) + e_r * float(legs["footprint_radius_m"])
		var foot: Vector3 = VehicleModels.leg_poses(rig, azimuth, 1.0)["main"] * foot_local
		_check(foot.distance_to(target) < 0.05, "leg az %.0f deployed: foot %.3f m from the physics' point" % [azimuth, foot.distance_to(target)])
		# in between: no flip
		var from: Vector3 = stowed * foot_local
		var stroke := from.distance_to(foot)
		var last := from
		var largest := 0.0
		for k in range(1, 21):
			var now: Vector3 = VehicleModels.leg_poses(rig, azimuth, k * 0.05)["main"] * foot_local
			largest = maxf(largest, now.distance_to(last))
			last = now
		_check(largest <= 0.25 * stroke, "leg az %.0f: largest step %.2f m of the %.2f m stroke" % [azimuth, largest, stroke])
		# the pusher joins its anchor and its point on the leg at every deployment
		for d: float in [0.0, 0.5, 1.0]:
			var poses := VehicleModels.leg_poses(rig, azimuth, d)
			var pusher: Transform3D = poses["pusher"]
			var on_leg: Vector3 = poses["main"] * Vector3(float(rig["pusher_leg_u_m"]), 0.0, 0.0)
			_check((pusher * Vector3.RIGHT).distance_to(on_leg) < EPS, "leg az %.0f at %.1f: pusher reaches the leg" % [azimuth, d])
	# The view: stack (fins folded, legs posed from the snapshot, second stage still a cylinder) and the stage alone.
	var stage_1 := {"id": "first_stage", "base_station_m": 0.0, "length_m": first["stage"][0]["length_m"], "diameter_m": 3.66,
			"nose_length_m": 0.0, "vehicle_id": "falcon9_block5"}
	var stage_2 := {"id": "second_stage", "base_station_m": stage_1["length_m"], "length_m": 13.8, "diameter_m": 3.66, "nose_length_m": 0.0,
			"vehicle_id": "falcon9_block5"}
	var merlin := {"stage": 0, "position_m": Vector3.ZERO, "direction": Vector3.RIGHT, "firing": true, "throttle": 1.0, "spool": 1.0, "vacuum": false, "auxiliary": false}
	var mvac := {"stage": 1, "position_m": Vector3(stage_1["length_m"], 0.0, 0.0), "direction": Vector3.RIGHT, "firing": true, "throttle": 1.0, "spool": 1.0, "vacuum": true, "auxiliary": false}
	var fin := {"station_m": first["stage"][0]["grid_fins"]["station_m"], "azimuth_deg": 45.0, "radius_m": 2.6, "area_m2": 1.8, "deflection_deg": 0.0}
	var snapshot_legs := {"present": true, "deployment": 0.0, "azimuths_deg": legs["azimuths_deg"]}
	var view := VehicleView.new()
	_views.append(view)
	root.add_child(view)
	view.sync([stage_1, stage_2], [merlin, mvac], [], [fin], 0.0, snapshot_legs)
	var node := view.get_child(0).get_node_or_null("stage_first_stage") as Node3D
	_check(node != null and node.get_node_or_null("interstage") != null and node.get_node_or_null("octaweb") != null, "first stage: hull, interstage, octaweb nodes")
	_check(_cylinders(view) == 0 and view.find_children("ModelNotice", "Label3D", true, false).is_empty(), "both stages modelled, no cylinder, no notice")
	var upper := view.get_child(0).get_node_or_null("stage_second_stage") as Node3D
	_check(upper != null and upper.get_node_or_null("hull") != null, "second stage: hull node")
	_check(upper != null and upper.get_node_or_null("engines_rvac") != null, "second stage: the MVac instance batch")
	_check(upper != null and upper.transform.origin.is_equal_approx(Frames.to_render(Vector3(47.7, 0.0, 0.0))), "second stage base 47.7 m above the first stage's station 0")
	var mvac_entry: Dictionary = models["manifest"]["engines"]["merlin_vacuum"]
	var exit_station := 47.7 + float(mvac_entry["axial_offset_m_by_stage"].get("second_stage", 0.0)) + float(mvac_entry["exit_plane_x_m"])
	var interstage_base := float(models["manifest"]["look"]["interstage_base_m"])
	_check(exit_station > interstage_base and exit_station < 47.7, "MVac exit plane (station %.2f) inside the interstage (%.1f .. 47.7)" % [exit_station, interstage_base])
	_check(_plume_top(view, 1).is_equal_approx(Frames.to_render(Vector3(exit_station, 0.0, 0.0))), "MVac plume starts at the drawn exit plane")
	var mvac_bounds := _bounds_x(models["hulls"]["second_stage"]["hull"])
	_check(absf(mvac_bounds.y - 13.8) < 0.01, "second stage hull top at its config length, 13.8 m")
	var fin_node := view.get_child(0).find_child("grid_fin_0", true, false) as Node3D
	_check(fin_node != null and (fin_node.transform.basis * Vector3.UP).is_equal_approx(Vector3.LEFT), "on the stack the fin lies folded, tip toward the base")
	_check(_plume_top(view, 0).is_equal_approx(Frames.to_render(Vector3.ZERO)), "Merlin plume starts at the exit plane, station 0")
	_check(is_equal_approx(view.engine_pose(0).origin.x, float(models["manifest"]["engines"]["merlin1d"]["axial_offset_m_by_stage"]["first_stage"])), "Merlin drawn one bell length above the exit plane")
	snapshot_legs["deployment"] = 1.0
	view.sync([stage_1], [merlin], [], [fin], 0.0, snapshot_legs)
	node = view.get_child(0).get_node_or_null("stage_first_stage") as Node3D
	var leg := node.get_node_or_null("leg_main_0") as Node3D
	_check(leg != null and leg.transform.is_equal_approx(VehicleModels.leg_poses(rig, float(legs["azimuths_deg"][0]), 1.0)["main"]), "alone, deployment 1: leg posed by the rig")
	fin_node = view.get_child(0).find_child("grid_fin_0", true, false) as Node3D
	_check(fin_node != null and (fin_node.transform.basis * Vector3.UP).is_equal_approx(_radial(45.0)), "alone: the fin stands out, hinge radial")
	# Separated parts keep their own models: "<vehicle>/<stage>" ids, each stage's base is station 0 again.
	var first_alone := stage_1.duplicate()
	first_alone["vehicle_id"] = "falcon9_block5/first_stage"
	var second_alone := stage_2.duplicate()
	second_alone["vehicle_id"] = "falcon9_block5/second_stage"
	second_alone["base_station_m"] = 0.0
	var mvac_alone := mvac.duplicate()
	mvac_alone["stage"] = 0
	mvac_alone["position_m"] = Vector3.ZERO
	var view_1 := VehicleView.new()
	_views.append(view_1)
	root.add_child(view_1)
	view_1.sync([first_alone], [merlin], [], [fin], 0.0, snapshot_legs)
	var node_1 := view_1.get_child(0).get_node_or_null("stage_first_stage") as Node3D
	_check(node_1 != null and node_1.get_node_or_null("octaweb") != null and _cylinders(view_1) == 0, "first stage alone: its model")
	var view_2 := VehicleView.new()
	_views.append(view_2)
	root.add_child(view_2)
	view_2.sync([second_alone], [mvac_alone], [], [], 0.0, {})
	var node_2 := view_2.get_child(0).get_node_or_null("stage_second_stage") as Node3D
	_check(node_2 != null and node_2.get_node_or_null("hull") != null and node_2.get_node_or_null("engines_rvac") != null and _cylinders(view_2) == 0,
			"second stage alone: hull and MVac, no cylinder, no notice (%d labels)" % view_2.find_children("ModelNotice", "Label3D", true, false).size())
	VehicleModels.clear_cache()
	return true


## Crew Dragon (needs the generated set: python tools/structures/build_models.py --vehicle crew_dragon): the nose cone's hinge and
## angle, its easing between the two binary states, the trunk, the capsule alone, and the Dragon on a Falcon 9 second stage.
func _check_dragon() -> bool:
	VehicleModels.clear_cache()
	var models := VehicleModels.load_for("crew_dragon")
	if models.is_empty():
		_check(false, "crew_dragon: models generated (%s)" % VehicleModels.fallback_reason("crew_dragon"))
		return false
	_check(VehicleModels.notice("crew_dragon") == "", "crew_dragon: no missing-model notice")
	var root_dir := ProjectSettings.globalize_path("res://").path_join("..")
	var chunks := FileAccess.get_file_as_string(root_dir.path_join("config/vehicles/crew_dragon.toml")).split("[[stage]]", false)
	var trunk_cfg: Dictionary = TomlLite.parse("[[stage]]" + chunks[1])["stage"][0]
	var capsule_cfg: Dictionary = TomlLite.parse("[[stage]]" + chunks[2])["stage"][0]
	var detail: Dictionary = TomlLite.parse(FileAccess.get_file_as_string(root_dir.path_join("config/vehicles/detail/crew_dragon.toml")))
	var nose: Dictionary = models["manifest"]["nose"]
	var angle_deg := float(detail["capsule"]["nose_open_angle_deg"])
	_check(is_equal_approx(float(nose["open_angle_deg"]), angle_deg) and is_equal_approx(float(nose["open_time_s"]), float(detail["capsule"]["nose_open_time_s"])),
			"nose: manifest angle %.0f deg and time come from the detail file" % angle_deg)
	# The nose never leaves its hinge: the model's origin is the hinge point, and the model has a vertex on it (the base rim's hinge edge).
	var vertices: PackedVector3Array = (models["nose"]["nose_cone"] as Mesh).surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
	var nearest := INF
	for v in vertices:
		nearest = minf(nearest, v.length())
	_check(nearest < 0.01, "nose: a vertex of the cone sits on the hinge point (%.4f m)" % nearest)
	var azimuth := deg_to_rad(float(nose["hinge_azimuth_deg"]))
	var hinge := Vector3(float(nose["hinge_station_m"]), 0.0, 0.0) + Vector3(0.0, cos(azimuth), sin(azimuth)) * float(nose["hinge_radius_m"])
	_check(is_equal_approx(hinge.y * hinge.y + hinge.z * hinge.z, pow(float(detail["capsule"]["nose_hinge_r_m"]), 2.0)),
			"nose: the hinge lies at the measured radius (detail file nose_hinge_r_m)")
	var worst := 0.0
	var model_vertex := vertices[0]
	for v in vertices:
		if v.length() < model_vertex.length():
			model_vertex = v
	for k in range(0, 11):
		worst = maxf(worst, (VehicleModels.nose_transform(nose, k * 0.1) * model_vertex).distance_to(hinge))
	_check(worst < 0.01, "nose: the hinge vertex stays at the hinge point at every opening (largest move %.4f m)" % worst)
	_check(VehicleModels.nose_transform(nose, 0.0).basis.is_equal_approx(Basis.IDENTITY), "nose: closed is the identity pose")
	var open_basis := VehicleModels.nose_transform(nose, 1.0).basis
	_check(absf(rad_to_deg(open_basis.get_rotation_quaternion().get_angle()) - angle_deg) < 0.01, "nose: open turns by the detail file's angle")
	_check((open_basis * Vector3.RIGHT).dot(Vector3(0.0, cos(azimuth), sin(azimuth))) > 0.0, "nose: opening swings the tip outward at the hinge side")
	# The view: trunk + capsule with the snapshot's binary nose state eased by the view.
	var trunk := {"id": "trunk", "base_station_m": 0.0, "length_m": trunk_cfg["length_m"], "diameter_m": trunk_cfg["diameter_m"], "nose_length_m": 0.0, "vehicle_id": "crew_dragon"}
	var capsule := {"id": "capsule", "base_station_m": trunk_cfg["length_m"], "length_m": capsule_cfg["length_m"], "diameter_m": 4.0, "nose_length_m": 4.0, "vehicle_id": "crew_dragon"}
	var view := VehicleView.new()
	_views.append(view)
	root.add_child(view)
	view.sync([trunk, capsule], [], [], [], 0.0, {}, {"present": true, "nose_cone": true, "nose_cone_open": false})
	var capsule_node := view.get_child(0).get_node_or_null("stage_capsule") as Node3D
	var trunk_node := view.get_child(0).get_node_or_null("stage_trunk") as Node3D
	var capsule_parts := ["capsule", "heat_shield", "deck", "superdraco_pod", "draco_pod", "nose_cone"]
	var present := capsule_node != null and capsule_parts.all(func(n: String) -> bool: return capsule_node.get_node_or_null(n) != null)
	_check(present, "capsule: capsule, heat_shield, deck, superdraco_pod, draco_pod, nose_cone nodes")
	_check(trunk_node != null and ["trunk", "trunk_array", "trunk_radiator"].all(func(n: String) -> bool: return trunk_node.get_node_or_null(n) != null),
			"trunk: trunk, trunk_array, trunk_radiator nodes")
	_check(_cylinders(view) == 0 and view.find_children("ModelNotice", "Label3D", true, false).is_empty(), "both stages modelled, no cylinder, no notice")
	var nose_node := capsule_node.get_node("nose_cone") as Node3D
	_check(nose_node.transform.is_equal_approx(VehicleModels.nose_transform(nose, 0.0)) and is_zero_approx(view.nose_fraction()), "nose closed with nose_cone_open false")
	view.sync([trunk, capsule], [], [], [], 0.0, {}, {"present": true, "nose_cone": true, "nose_cone_open": true})
	_check(is_zero_approx(view.nose_fraction()), "nose: the state flips, the view has not moved yet")
	var open_time := float(nose["open_time_s"])
	view.advance_nose(open_time * 0.5)
	_check(is_equal_approx(view.nose_fraction(), 0.5) and nose_node.transform.is_equal_approx(VehicleModels.nose_transform(nose, 0.5)), "nose: half the opening time, half open")
	view.advance_nose(open_time)
	_check(is_equal_approx(view.nose_fraction(), 1.0) and nose_node.transform.is_equal_approx(VehicleModels.nose_transform(nose, 1.0)), "nose: open after the opening time, no overshoot")
	view.sync([trunk, capsule], [], [], [], 0.0, {}, {"present": true, "nose_cone": true, "nose_cone_open": false})
	view.advance_nose(open_time * 2.0)
	_check(is_zero_approx(view.nose_fraction()) and nose_node.transform.is_equal_approx(VehicleModels.nose_transform(nose, 0.0)), "nose: closes again")
	var born_open := VehicleView.new()
	_views.append(born_open)
	root.add_child(born_open)
	born_open.sync([trunk, capsule], [], [], [], 0.0, {}, {"present": true, "nose_cone": true, "nose_cone_open": true})
	_check(is_equal_approx(born_open.nose_fraction(), 1.0), "nose: a Dragon first seen with the nose open shows it open")
	# The same through update_view (first build and the sets loaded once per stage list).
	var stub := _StubSim.new()
	stub.stages = [trunk, capsule]
	stub.capsule = {"present": true, "nose_cone": true, "nose_cone_open": true}
	var via_update := VehicleView.new()
	_views.append(via_update)
	root.add_child(via_update)
	via_update.update_view(stub, Vector3.ZERO)
	_check(is_equal_approx(via_update.nose_fraction(), 1.0), "update_view: a Dragon first built with the nose open shows it open at once")
	var sets_loaded := VehicleModels.load_calls
	via_update.update_view(stub, Vector3.ZERO)
	via_update.update_view(stub, Vector3.ZERO)
	_check(VehicleModels.load_calls == sets_loaded, "update_view: the model sets are not reloaded while the stage list is unchanged")
	# After trunk separation: the capsule alone shows no trunk, its base is station 0, the nose still hinges.
	var alone := capsule.duplicate()
	alone["vehicle_id"] = "crew_dragon/capsule"
	alone["base_station_m"] = 0.0
	var view_alone := VehicleView.new()
	_views.append(view_alone)
	root.add_child(view_alone)
	view_alone.sync([alone], [], [], [], 0.0, {}, {"present": true, "nose_cone": true, "nose_cone_open": false})
	_check(view_alone.get_child(0).get_node_or_null("stage_trunk") == null and view_alone.find_children("trunk*", "MeshInstance3D", true, false).is_empty(),
			"capsule alone: no trunk")
	_check(view_alone.get_child(0).get_node_or_null("stage_capsule/nose_cone") != null and _cylinders(view_alone) == 0, "capsule alone: its model with the nose")
	var trunk_alone := trunk.duplicate()
	trunk_alone["vehicle_id"] = "crew_dragon/trunk"
	var view_trunk := VehicleView.new()
	_views.append(view_trunk)
	root.add_child(view_trunk)
	view_trunk.sync([trunk_alone], [], [], [], 0.0, {})
	_check(view_trunk.get_child(0).get_node_or_null("stage_trunk/trunk") != null and view_trunk.get_child(0).get_node_or_null("stage_capsule") == null
			and _cylinders(view_trunk) == 0, "trunk alone: its model, no capsule")
	# On the Falcon 9 second stage ("falcon9_block5+crew_dragon"): the Dragon stands at its config stations, nose closed (no capsule state).
	var f9_chunks := FileAccess.get_file_as_string(root_dir.path_join("config/vehicles/falcon9_block5.toml")).split("[[stage]]", false)
	var first_len := float(TomlLite.parse("[[stage]]" + f9_chunks[1])["stage"][0]["length_m"])
	var second_len := float(TomlLite.parse("[[stage]]" + f9_chunks[2])["stage"][0]["length_m"])
	var stack_id := "falcon9_block5+crew_dragon"
	var trunk_base := first_len + second_len
	var capsule_base := trunk_base + float(trunk["length_m"])
	var stack: Array = [
		{"id": "first_stage", "base_station_m": 0.0, "length_m": first_len, "diameter_m": 3.66, "nose_length_m": 0.0, "vehicle_id": stack_id},
		{"id": "second_stage", "base_station_m": first_len, "length_m": second_len, "diameter_m": 3.66, "nose_length_m": 0.0, "vehicle_id": stack_id},
		{"id": "trunk", "base_station_m": trunk_base, "length_m": trunk["length_m"], "diameter_m": trunk["diameter_m"], "nose_length_m": 0.0, "vehicle_id": stack_id},
		{"id": "capsule", "base_station_m": capsule_base, "length_m": capsule["length_m"], "diameter_m": 4.0, "nose_length_m": 4.0, "vehicle_id": stack_id}]
	var view_stack := VehicleView.new()
	_views.append(view_stack)
	root.add_child(view_stack)
	view_stack.sync(stack, [], [], [], 0.0, {}, {"present": false})
	var s_trunk := view_stack.get_child(0).get_node_or_null("stage_trunk") as Node3D
	var s_capsule := view_stack.get_child(0).get_node_or_null("stage_capsule") as Node3D
	_check(s_trunk != null and s_trunk.transform.origin.is_equal_approx(Frames.to_render(Vector3(trunk_base, 0.0, 0.0))),
			"stack: the trunk stands on the second stage's top, station %.1f m" % trunk_base)
	_check(s_capsule != null and s_capsule.transform.origin.is_equal_approx(Frames.to_render(Vector3(capsule_base, 0.0, 0.0))),
			"stack: the capsule stands on the trunk, station %.1f m" % capsule_base)
	_check(view_stack.get_child(0).get_node_or_null("stage_second_stage") != null and view_stack.get_child(0).get_node_or_null("stage_first_stage") != null
			and _cylinders(view_stack) == 0 and view_stack.find_children("ModelNotice", "Label3D", true, false).is_empty(),
			"stack: all four stages modelled, no cylinder, no notice")
	_check(is_zero_approx(view_stack.nose_fraction()) and s_capsule != null and s_capsule.get_node("nose_cone").transform.is_equal_approx(VehicleModels.nose_transform(nose, 0.0)),
			"stack: the nose is closed on the launcher")
	VehicleModels.clear_cache()
	return true


## The ISS model: nodes, joints at the stated pose, and the docking rings against Dragon's at both configured ports (geometry read from the meshes, ports from the configs).
func _check_iss() -> bool:
	VehicleModels.clear_cache()
	var models := VehicleModels.load_for("iss")
	if models.is_empty():
		_check(false, "iss: models generated (%s)" % VehicleModels.fallback_reason("iss"))
		return false
	var dragon := VehicleModels.load_for("crew_dragon")
	if dragon.is_empty():
		_check(false, "iss: the crew_dragon models are needed for the ring check")
		return false
	var root_dir := ProjectSettings.globalize_path("res://").path_join("..")
	var ports := _config_ports(root_dir.path_join("config/vehicles/iss.toml"))
	_check(ports.size() == 2, "iss: two ports in the config")
	var length_m := float(TomlLite.parse(FileAccess.get_file_as_string(root_dir.path_join("config/vehicles/iss.toml")))["stage"][0]["length_m"])
	var station := {"id": "station", "base_station_m": 0.0, "length_m": length_m, "diameter_m": 4.2, "nose_length_m": 0.0, "vehicle_id": "iss"}
	var view := VehicleView.new()
	_views.append(view)
	root.add_child(view)
	view.sync([station], [], [], [], 0.0)
	var stage := view.get_child(0).get_node_or_null("stage_station") as Node3D
	_check(stage != null and _cylinders(view) == 0, "iss: the station draws its model, no cylinder")
	if stage == null:
		return false
	for part: String in ["pressurised", "truss", "pma_2", "ida_2", "pma_3", "ida_3"]:
		_check(stage.get_node_or_null(part) != null, "iss: node %s" % part)
	# Joints: separate nodes at the manifest's pose, the parts below them.
	var joints: Array = models["manifest"]["joints"]
	_check(joints.size() == 2 + 8 + 2, "iss: 2 alpha, 8 beta, 2 radiator joints (%d)" % joints.size())
	for joint: Dictionary in joints:
		var node := stage.find_child(String(joint["name"]), true, false) as Node3D
		_check(node != null, "iss: joint node %s" % joint["name"])
		if node == null:
			continue
		var axis := _vec3(joint["axis"])
		var parent_origin := Vector3.ZERO if String(joint["parent"]) == "" else _vec3(_joint_named(joints, joint["parent"])["origin_m"])
		var expected := Transform3D(Basis(axis, deg_to_rad(float(joint["pose_deg"]))), _vec3(joint["origin_m"]) - parent_origin)
		_check(node.transform.is_equal_approx(expected), "iss: %s at the stated pose %.0f deg" % [joint["name"], joint["pose_deg"]])
		for part: String in joint["meshes"]:
			_check(node.get_node_or_null(part) != null, "iss: %s under %s" % [part, joint["name"]])
	_check(stage.find_child("array_7", true, false) != null and stage.find_child("radiator_1", true, false) != null, "iss: array_<i>, radiator_<i>")
	# The wingspan at the stated pose: P6 / S6 blankets broadside give the published 109 m [S] (+-1 m).
	var reach := 0.0
	for i in 8:
		var array_node := stage.find_child("array_%d" % i, true, false) as MeshInstance3D
		for s in (array_node.mesh as Mesh).get_surface_count():
			for v: Vector3 in array_node.mesh.surface_get_arrays(s)[Mesh.ARRAY_VERTEX]:
				reach = maxf(reach, absf((_chain(array_node, stage) * v).y))
	_check(absf(2.0 * reach - 109.0) < 1.0, "iss: array span at the pose %.2f m (109 m published)" % (2.0 * reach))
	for pair: Array in [[0, "ida_2", "2"], [1, "ida_3", "3"]]:
		_check(_check_ring_mate(ports[pair[0]], String(pair[1]), models, dragon) == true, "%s: ring check ran to its end" % pair[1])
	VehicleModels.clear_cache()
	return true


## The docking ports of a vehicle config, read from its text (TomlLite has no nested arrays of tables): position_m, axis, up of every [[stage.docking_port]].
func _config_ports(path: String) -> Array:
	var ports := []
	for block in FileAccess.get_file_as_string(path).replace("\r\n", "\n").split("[[stage.docking_port]]").slice(1):
		var port := {}
		for line in block.split("\n"):
			if line.begins_with("[["):
				break
			var eq := line.find("=")
			var key := line.substr(0, eq).strip_edges() if eq > 0 else ""
			if key in ["position_m", "axis", "up"]:
				var list := line.substr(eq + 1, line.find("]") - eq).strip_edges().trim_prefix("[").trim_suffix("]").split(",")
				port[key] = [float(list[0]), float(list[1]), float(list[2])]
		ports.append(port)
	return ports


## The transform of `node` relative to its ancestor `stop` (the test's views are not in the tree yet: no global transforms).
func _chain(node: Node3D, stop: Node3D) -> Transform3D:
	var out := Transform3D.IDENTITY
	var at := node
	while at != stop:
		out = at.transform * out
		at = at.get_parent() as Node3D
	return out


func _joint_named(joints: Array, joint_name: String) -> Dictionary:
	for joint: Dictionary in joints:
		if joint["name"] == joint_name:
			return joint
	return {}


## Dragon's port on this station port (config data): plane within 0.01 m, axis within 0.1 deg, clocking as the config's up: the passive petals measured from the mesh at the IDD's angles
## about `up`, Dragon's petals between them (60 deg), Dragon's receptacles on the passive guide pins.
func _check_ring_mate(port: Dictionary, ida: String, models: Dictionary, dragon: Dictionary) -> bool:
	var parts: Dictionary = models["hulls"]["station"]
	var position := _vec3(port["position_m"])
	var x_s := _vec3(port["axis"]).normalized()
	var up_s := _vec3(port["up"]).normalized()
	var y_s := up_s.cross(x_s)
	_check(absf(x_s.dot(up_s)) < 1e-6, "%s: config up is normal to the axis" % ida)
	var idss: Dictionary = dragon["manifest"]["idss"]
	# Dragon's mechanism meshes: body axes with the origin on its port (X out, up -Z: asserted against the Dragon's config).
	var capsule_port: Dictionary = _config_ports(ProjectSettings.globalize_path("res://").path_join("../config/vehicles/crew_dragon.toml"))[0]
	_check(_vec3(capsule_port["axis"]).is_equal_approx(Vector3.RIGHT) and _vec3(capsule_port["up"]).is_equal_approx(Vector3(0.0, 0.0, -1.0)), "%s: Dragon's port frame as the mechanism is drawn" % ida)
	# Mated: axes opposed, ups equal (vehicle/docking_port). Dragon frame -> station: X -> -x_s, Z -> -up_s, Y -> y_s.
	var to_station := Transform3D(Basis(-x_s, y_s, -up_s), position)
	# Plane of Dragon's flange top (mating plane) and the IDA's face.
	var dragon_plane := _plane_points(parts_points(dragon["mechanism"]["flange"], to_station), -x_s, position)  # Dragon's face looks along -x_s
	var ida_plane := _plane_points(parts_points(parts[ida + "_face"], Transform3D.IDENTITY), x_s, position)
	_check(absf(-float(dragon_plane["axial"]) - float(ida_plane["axial"])) < 0.01, "%s: ring planes coincide (Dragon %.4f, IDA %.4f m from the port)" % [ida, -float(dragon_plane["axial"]), ida_plane["axial"]])
	_check(absf(ida_plane["axial"]) < 0.01, "%s: the IDA face stands on the configured port (%.4f m)" % [ida, ida_plane["axial"]])
	_check(dragon_plane["tilt_deg"] < 0.1 and ida_plane["tilt_deg"] < 0.1, "%s: axes within 0.1 deg (Dragon %.4f, IDA %.4f)" % [ida, dragon_plane["tilt_deg"], ida_plane["tilt_deg"]])
	_check(dragon_plane["centre_off"] < 0.01 and ida_plane["centre_off"] < 0.01, "%s: centres on the axis (Dragon %.4f, IDA %.4f m)" % [ida, dragon_plane["centre_off"], ida_plane["centre_off"]])
	# Passive petals: three clusters, each at the IDD's angle about the config's up (phi = 90 - theta, y_s = up x axis).
	var petals := parts_points(parts[ida + "_petals"], Transform3D.IDENTITY)
	var expected_phi: Array[float] = []
	for theta: float in idss["petal_theta_deg"]:
		expected_phi.append(fposmod(90.0 - theta, 360.0))
	for phi: float in expected_phi:
		var worst := INF
		var sum := Vector2.ZERO
		var in_cluster := 0
		for p in petals:
			var q := p - position
			var a := rad_to_deg(atan2(q.dot(up_s), q.dot(y_s)))
			if absf(angle_difference(deg_to_rad(a), deg_to_rad(phi))) < deg_to_rad(40.0):
				sum += Vector2(q.dot(y_s), q.dot(up_s))
				in_cluster += 1
		worst = rad_to_deg(absf(angle_difference(sum.angle(), deg_to_rad(phi)))) if in_cluster > 0 else INF
		_check(in_cluster > 0 and worst < 0.1, "%s: passive petal at phi %.0f deg about the config's up (%d vertices, off by %.3f deg)" % [ida, phi, in_cluster, worst])
	# Dragon's petals in the station, read from its mechanism: the petal mesh is drawn on azimuth 0 (+Y) and DockingMechanismView turns it
	# about the port axis to the IDD's theta; its centroid direction through that turn, in the station, stands between the passive ones (they mesh).
	var petal_points := parts_points(dragon["mechanism"]["petal"], Transform3D.IDENTITY)
	_check(petal_points.size() > 0, "%s: Dragon's petal mesh has vertices" % ida)
	var centroid := Vector3.ZERO
	for p in petal_points:
		centroid += p
	centroid /= maxf(1.0, float(petal_points.size()))
	var port_up := _vec3(idss["port_up"])
	var port_right := port_up.cross(_vec3(idss["port_axis"]))
	for theta: float in idss["petal_theta_deg"]:
		var radial := port_up * cos(deg_to_rad(theta)) + port_right * sin(deg_to_rad(theta))
		var turned := Basis(Vector3.RIGHT, atan2(radial.z, radial.y)) * Vector3(0.0, centroid.y, centroid.z)
		var world := to_station.basis * turned
		var phi := rad_to_deg(atan2(world.dot(up_s), world.dot(y_s)))
		var nearest := INF
		for other: float in expected_phi:
			nearest = minf(nearest, rad_to_deg(absf(angle_difference(deg_to_rad(phi), deg_to_rad(other)))))
		_check(absf(nearest - 60.0) < 1.0, "%s: Dragon's petal (mesh, theta %.0f) stands between the passive petals (%.3f deg from the nearest)" % [ida, theta, nearest])
	# The passive guide pins on Dragon's receptacles (mesh centroids of the pins against the IDD's receptacle theta on Dragon).
	var pins := parts_points(parts[ida + "_pins"], Transform3D.IDENTITY)
	var r_pin := 0.5 * float(idss["pin_circle_diameter_m"])
	for theta: float in idss["receptacle_theta_deg"]:
		var a_rad := deg_to_rad(90.0 - theta)
		var receptacle := to_station * (Vector3(0.0, -cos(a_rad), -sin(a_rad)) * r_pin)
		var sum := Vector3.ZERO
		var count := 0
		for p in pins:
			if p.distance_to(receptacle) < 0.15:
				sum += p
				count += 1
		var off: float = INF if count == 0 else ((sum / count) - x_s * (sum / count - position).dot(x_s)).distance_to(receptacle - x_s * (receptacle - position).dot(x_s))
		_check(off < 0.002, "%s: passive guide pin on Dragon's receptacle (theta %.1f, %.4f m)" % [ida, theta, off])
	return true


## All vertices of a mesh through a transform.
func parts_points(mesh: Mesh, frame: Transform3D) -> PackedVector3Array:
	var out := PackedVector3Array()
	for s in mesh.get_surface_count():
		for v: Vector3 in mesh.surface_get_arrays(s)[Mesh.ARRAY_VERTEX]:
			out.append(frame * v)
	return out


## The mesh's flat top along `axis` (the outermost vertices within 1 mm): its axial place from `origin`, tilt of its normal to the axis, and how far the ring's centre lies off the axis.
func _plane_points(points: PackedVector3Array, axis: Vector3, origin: Vector3) -> Dictionary:
	var top := -INF
	for p in points:
		var q := p - origin
		if absf(q.length()) > 0.5:  # the annulus only
			top = maxf(top, q.dot(axis))
	var ring := PackedVector3Array()
	var e1 := axis.cross(Vector3(0.123, 0.456, 0.789)).normalized()
	var e2 := axis.cross(e1)
	var box := Rect2()
	for p in points:
		var q := p - origin
		if q.length() > 0.5 and q.dot(axis) > top - 0.001:
			ring.append(q)
			var flat := Vector2(q.dot(e1), q.dot(e2))
			box = Rect2(flat, Vector2.ZERO) if ring.size() == 1 else box.expand(flat)
	# Normal from the widest triple of the ring (the points are on a circle).
	var a := ring[0]
	var b := ring[ring.size() / 3]
	var c := ring[2 * ring.size() / 3]
	var n := (b - a).cross(c - a).normalized()
	if n.dot(axis) < 0.0:
		n = -n
	return {"axial": top, "tilt_deg": rad_to_deg(acos(clampf(n.dot(axis), -1.0, 1.0))), "centre_off": box.get_center().length()}


func _vec3(values: Array) -> Vector3:
	return Vector3(float(values[0]), float(values[1]), float(values[2]))


## Largest and smallest station (x) of a mesh's vertices: Vector2(min, max).
func _bounds_x(mesh: Mesh) -> Vector2:
	var lo := INF
	var hi := -INF
	for s in mesh.get_surface_count():
		for v: Vector3 in mesh.surface_get_arrays(s)[Mesh.ARRAY_VERTEX]:
			lo = minf(lo, v.x)
			hi = maxf(hi, v.x)
	return Vector2(lo, hi)


## The model vehicles other than the Starship (Falcon 9, Crew Dragon, ISS): with no manifest, or a model set older than its
## sources, load_for returns {} with exactly one log line and the view shows its cylinders. Runs on a scratch repo folder
## (VehicleModels.root_override) so the real data/ is not touched.
func _check_other_model_vehicles(snapshot: Dictionary) -> void:
	for id in ["falcon9_block5", "crew_dragon", "iss"]:
		_check(VehicleModels.MODEL_VEHICLE_IDS.has(id), "%s is a model vehicle" % id)
	_check(VehicleModels.MODEL_VEHICLE_IDS.has(VEHICLE) and not VehicleModels.MODEL_VEHICLE_IDS.has("starship_tanker"),
			"the Starship is a model vehicle, the tanker is not")
	var scratch := OS.get_user_data_dir().path_join("vehicle_view_test_%d" % OS.get_process_id())
	for id in ["falcon9_block5", "crew_dragon"]:
		VehicleModels.clear_cache()
		VehicleModels.root_override = scratch
		DirAccess.make_dir_recursive_absolute(scratch.path_join("data/models"))
		DirAccess.make_dir_recursive_absolute(scratch.path_join("config/vehicles/detail"))
		var lines := VehicleModels.log_count
		_check(VehicleModels.load_for(id).is_empty() and VehicleModels.load_for(id).is_empty(), "%s: no manifest: {}" % id)
		var view := _view(snapshot, id)
		_check(VehicleModels.log_count - lines == 1, "%s: no manifest: exactly one log line (%d)" % [id, VehicleModels.log_count - lines])
		_check(_cylinders(view) == 3 and _count(view, "MultiMeshInstance3D") == 0, "%s: no manifest: cylinders" % id)
		_check(VehicleModels.fallback_reason(id) == "missing", "%s: no manifest: reason missing" % id)
		# A model set older than its detail file: the manifest first, the source one clock second later.
		VehicleModels.clear_cache()
		VehicleModels.root_override = scratch
		DirAccess.make_dir_recursive_absolute(scratch.path_join("data/models").path_join(id))
		var manifest := FileAccess.open(scratch.path_join("data/models").path_join(id).path_join("manifest.json"), FileAccess.WRITE)
		manifest.store_string("{}")
		manifest.close()
		OS.delay_msec(1100)  # get_modified_time has a resolution of 1 s
		var detail := FileAccess.open(scratch.path_join("config/vehicles/detail/%s.toml" % id), FileAccess.WRITE)
		detail.store_string("# newer than the models\n")
		detail.close()
		lines = VehicleModels.log_count
		_check(VehicleModels.load_for(id).is_empty() and VehicleModels.load_for(id).is_empty(), "%s: stale models: {}" % id)
		var stale := _view(snapshot, id)
		_check(VehicleModels.log_count - lines == 1, "%s: stale models: exactly one log line (%d)" % [id, VehicleModels.log_count - lines])
		_check(_cylinders(stale) == 3 and _count(stale, "MultiMeshInstance3D") == 0, "%s: stale models: cylinders" % id)
		_check(VehicleModels.fallback_reason(id) == "stale", "%s: stale models: reason stale" % id)
		_check(stale.find_children("ModelNotice", "Label3D", true, false).size() == 1, "%s: stale models: German notice" % id)
		# Fresh manifest but no model files defined for the vehicle yet (stub): cylinders, one line, no crash.
		VehicleModels.clear_cache()
		VehicleModels.root_override = scratch
		manifest = FileAccess.open(scratch.path_join("data/models").path_join(id).path_join("manifest.json"), FileAccess.WRITE)
		manifest.store_string("{}")
		manifest.close()
		lines = VehicleModels.log_count
		_check(VehicleModels.load_for(id).is_empty() and VehicleModels.log_count - lines == 1,
				"%s: fresh manifest, no generated files yet: {} with one log line" % id)
	VehicleModels.clear_cache()  # also resets root_override
	OS.move_to_trash(scratch)


func _check_models(stages: Array, snapshot: Dictionary, models: Dictionary) -> void:
	var ship_material: ShaderMaterial = models["materials"]["ship"]
	_check(is_equal_approx(ship_material.get_shader_parameter("belly_azimuth_rad"),
			deg_to_rad(_piece(stages, "ship")["stage"][0]["aero"]["belly_azimuth_deg"])),
			"the hull shader's belly azimuth is the vehicle config's (aero.belly_azimuth_deg)")
	var view := _view(snapshot, VEHICLE)
	var root: Node3D = view.get_child(0)
	_check(_cylinders(view) == 0, "models replace the cylinders")
	for stage: Dictionary in stages:
		var node := root.get_node_or_null("stage_" + String(stage["id"])) as Node3D
		_check(node != null and node.get_node_or_null("hull") != null, "%s: own stage node with a hull" % stage["id"])
		if node == null:
			continue
		_check(node.position.is_equal_approx(Vector3(0.0, 0.0, stage["base_station_m"])),
				"%s: stage node at its base station along render +Z" % stage["id"])
		_check(node.basis.is_equal_approx(VehicleModels.body_to_render()), "%s: body -> render permutation" % stage["id"])
		var multimeshes := node.find_children("engines_*", "MultiMeshInstance3D", false, false)
		var instances := 0
		for multimesh: MultiMeshInstance3D in multimeshes:
			instances += multimesh.multimesh.instance_count
		var expected := 0
		for engine: Dictionary in snapshot["engines"]:
			expected += 1 if stages[engine["stage"]] == stage else 0
		_check(instances == expected, "%s: %d engine instances (config)" % [stage["id"], expected])
	var booster_engines := 0
	for engine: Dictionary in snapshot["engines"]:
		booster_engines += 1 if engine["stage"] == 0 else 0
	_check(booster_engines == 33 and snapshot["engines"].size() - booster_engines == 6, "config: 33 + 6 engines")
	# Engine poses = the config's ring formula (gimbal point, thrust +X), relative to the stage's base.
	for i in snapshot["engines"].size():
		var engine: Dictionary = snapshot["engines"][i]
		var base: float = stages[engine["stage"]]["base_station_m"]
		var pose: Transform3D = view.engine_pose(i)
		var drawn := VehicleModels.axial_offset_m(models, stages[engine["stage"]]["id"], engine["vacuum"])
		_check(pose.origin.is_equal_approx(engine["position_m"] + Vector3(drawn - base, 0.0, 0.0))
				and (pose.basis * Vector3.RIGHT).is_equal_approx(engine["direction"]), "engine %d: ring position, thrust axis" % i)
	_check_gimbal(view, snapshot)
	_check_flaps(stages, snapshot, view, root)
	_check_fins(stages, snapshot, view, root)
	_check_plumes(snapshot, view, models)
	# Separation: the booster's node and everything on it go, the ship keeps its own transform (base station 0 now).
	var ship_snapshot := _separated(snapshot, stages)
	view.sync(ship_snapshot["stages"], ship_snapshot["engines"], ship_snapshot["flaps"], ship_snapshot["grid_fins"], 0.0)
	root = view.get_child(0)
	_check(root.get_node_or_null("stage_booster") == null, "after separation: no booster node")
	var ship_node := root.get_node_or_null("stage_ship") as Node3D
	_check(ship_node != null and ship_node.position.is_equal_approx(Vector3.ZERO),
			"after separation: ship node at base 0")
	_check(root.find_children("grid_fin_*", "Node3D", true, false).is_empty(), "after separation: no grid fins")
	_check(root.find_children("flap_*", "Node3D", true, false).size() == 4, "after separation: the ship's 4 flaps")
	# Missing snapshot pieces: no flaps / fins / vehicle_id must not break the view.
	view.sync(snapshot["stages"], snapshot["engines"], [], [], 0.0)
	view.sync(snapshot["stages"], snapshot["engines"].slice(0, 5), snapshot["flaps"], snapshot["grid_fins"], 0.0)
	_check(true, "short or empty engine / flap / fin lists do not break the view")


func _check_gimbal(view: VehicleView, snapshot: Dictionary) -> void:
	var tilted: Array = snapshot["engines"].duplicate(true)
	var direction := Vector3(0.995, 0.0, 0.1).normalized()
	tilted[0]["direction"] = direction
	view.sync(snapshot["stages"], tilted, snapshot["flaps"], snapshot["grid_fins"], 0.0)
	_check((view.engine_pose(0).basis * Vector3.RIGHT).is_equal_approx(direction),
			"a gimbaled engine follows the snapshot")
	view.sync(snapshot["stages"], snapshot["engines"], snapshot["flaps"], snapshot["grid_fins"], 0.0)


func _check_flaps(stages: Array, snapshot: Dictionary, view: VehicleView, root: Node3D) -> void:
	# Rest pose at liftoff: the config's initial state is d = 90, the panel in the meridian plane (no tilt off the axis).
	view.sync(snapshot["stages"], snapshot["engines"], snapshot["flaps"], snapshot["grid_fins"], 0.0)
	for i in snapshot["flaps"].size():
		var rest: Dictionary = snapshot["flaps"][i]
		_check(float(rest["deflection_deg"]) == 90.0, "flap %d rests at d = 90 at liftoff" % i)
		var rest_node := root.find_child("flap_%d" % i, true, false) as Node3D
		var panel_normal := rest_node.transform.basis * Vector3.UP  # model Y: the panel's thickness direction
		_check(absf(panel_normal.normalized().x) < 1e-6, "flap %d at liftoff: panel normal perpendicular to the body axis" % i)
	var belly_deg: float = _piece(stages, "ship")["stage"][0]["aero"]["belly_azimuth_deg"]
	var belly := _radial(belly_deg)
	var base: float = stages[1]["base_station_m"]
	var models := VehicleModels.load_for(VEHICLE)
	for state in FLAP_STATES_DEG:
		var flaps: Array = snapshot["flaps"].duplicate(true)
		for flap: Dictionary in flaps:
			flap["deflection_deg"] = state
		view.sync(snapshot["stages"], snapshot["engines"], flaps, snapshot["grid_fins"], 0.0)
		for i in flaps.size():
			var flap: Dictionary = flaps[i]
			var node := root.find_child("flap_%d" % i, true, false) as Node3D
			_check(node != null, "flap %d exists" % i)
			if node == null:
				continue
			var e_r := _radial(flap["hinge_azimuth_deg"])
			var entry := VehicleModels.flap_entry(models, "ship", flap, base)
			var lean := deg_to_rad(float(entry["drawn_hinge_lean_deg"]))
			var kind: String = entry["kind"]
			# the forward hinge leans 19.03 deg (the 1:100 model's flap face, detail file), the aft hinge not at all
			_check(absf(rad_to_deg(lean) - 19.03) < 1e-6 if kind == "fwd" else lean == 0.0,
					"flap %d (%s): hinge leans 19.03 deg (forward) / not (aft)" % [i, kind])
			# The drawn hinge leans towards the axis going forward in the meridian plane; Y' is its normal there.
			var hinge_axis := Vector3.RIGHT * cos(lean) - e_r * sin(lean)
			var normal := Vector3.RIGHT * sin(lean) + e_r * cos(lean)
			# core/src/aero/flap.cpp flap_span_direction: stow = X x e_r away from the belly; d = 0 stow, 90 broadside
			# (in the meridian plane, perpendicular to the hinge).
			var stow := Vector3.RIGHT.cross(e_r)
			stow = -stow if stow.dot(belly) > 0.0 else stow
			var span := stow * cos(deg_to_rad(state)) + normal * sin(deg_to_rad(state))
			var label := "flap %d at %.0f deg" % [i, state]
			_check((node.transform.basis * Vector3.BACK).is_equal_approx(span),
					label + ": panel along the core's span direction, perpendicular to the leaned hinge")
			_check((node.transform.basis * Vector3.RIGHT).is_equal_approx(hinge_axis),
					label + ": hinge axis = the leaned basis X' (does not move under the deflection)")
			_check(is_zero_approx(span.dot(hinge_axis)), label + ": span perpendicular to the hinge")
			var hinge := Vector3(float(entry["drawn_hinge_station_aft_m"]), 0.0, 0.0) + e_r * float(entry["drawn_hinge_radius_aft_m"])
			_check(node.transform.origin.is_equal_approx(hinge), label + ": hinge at the drawn aft end station and radius")
			if kind == "fwd" and state == 90.0:  # d = 90 is the STL pose: outer corner (u 0.529, v 3.154) at station 43.81, r 7.43
				var corner := node.transform * Vector3(0.529, 0.0, 3.154)
				_check(absf(corner.x - 43.810) < 0.02 and absf(corner.dot(e_r) - 7.430) < 0.02 and absf(corner.dot(stow)) < 1e-6,
						label + ": outer corner in the meridian plane at the STL station and radius")
			# The tile-covered +Y face looks at the flow side: along Y' at rest (hull tangent), towards the belly at 90 deg.
			var outer := node.transform.basis * Vector3.UP
			if state == 0.0:
				_check(outer.is_equal_approx(normal), label + ": rest pose, outer face along the hinge normal")
			elif state == 90.0:
				_check(outer.dot(belly) > 0.9, label + ": broadside, outer face to the belly side")
			# The panel may not be mirrored differently from the hinge side: lengths stay.
			_check(absf(absf(node.transform.basis.determinant()) - 1.0) < EPS, label + ": rigid (|det| = 1)")


func _check_fins(stages: Array, snapshot: Dictionary, view: VehicleView, root: Node3D) -> void:
	var hull_radius: float = stages[0]["diameter_m"] * 0.5
	for state in FIN_STATES_DEG:
		var fins: Array = snapshot["grid_fins"].duplicate(true)
		for fin: Dictionary in fins:
			fin["deflection_deg"] = state
		view.sync(snapshot["stages"], snapshot["engines"], snapshot["flaps"], fins, 0.0)
		_check(fins.size() == 3, "config: 3 grid fins")
		for i in fins.size():
			var fin: Dictionary = fins[i]
			var node := root.find_child("grid_fin_%d" % i, true, false) as Node3D
			if node == null:
				_check(false, "grid fin %d exists" % i)
				continue
			var azimuth := deg_to_rad(fin["azimuth_deg"])
			var tangent := Vector3(0.0, -sin(azimuth), cos(azimuth))
			# core/src/aero/grid_fin.cpp: normal = tangential cos d + X sin d (rest: the tangent).
			var normal := tangent * cos(deg_to_rad(state)) + Vector3.RIGHT * sin(deg_to_rad(state))
			var label := "grid fin %d at %.0f deg" % [i, state]
			_check((node.transform.basis * Vector3.BACK).is_equal_approx(normal), label + ": lattice normal as the core")
			_check((node.transform.basis * Vector3.UP).is_equal_approx(_radial(fin["azimuth_deg"])),
					label + ": hinge axis radial")
			_check(node.transform.origin.is_equal_approx(Vector3(fin["station_m"], cos(azimuth) * hull_radius,
					sin(azimuth) * hull_radius)), label + ": pivot on the hull at the fin's station")


func _check_plumes(snapshot: Dictionary, view: VehicleView, models: Dictionary) -> void:
	var engines: Array = snapshot["engines"].duplicate(true)
	for engine: Dictionary in engines:
		engine["firing"] = true
		engine["spool"] = 1.0
		engine["throttle"] = 1.0
	view.sync(snapshot["stages"], engines, [], [], 0.0)
	var depths := {false: 2.1, true: 3.6}  # manifest exit_plane_x_m (Raptor, RVac)
	var stages: Array = snapshot["stages"]
	var manifest_engines: Dictionary = models["manifest"]["engines"]
	for i in engines.size():
		var engine: Dictionary = engines[i]
		var stage: Dictionary = stages[engine["stage"]]
		var drawn := VehicleModels.axial_offset_m(models, stage["id"], engine["vacuum"])
		var exit_m: Vector3 = engine["position_m"] + Vector3(drawn, 0.0, 0.0) - engine["direction"] * depths[engine["vacuum"]]
		if not _plume_top(view, i).is_equal_approx(Frames.to_render(exit_m)):
			_check(false, "engine %d: plume starts at the drawn exit plane" % i)
			return
		if stage["id"] == "ship":  # the plume starts at the stated station above the ship's aft end
			var eid: String = "raptor_vacuum3" if engine["vacuum"] else "raptor3"
			if not is_equal_approx(exit_m.x - stage["base_station_m"], float(manifest_engines[eid]["ship_exit_station_m"])):
				_check(false, "ship engine %d: exit plane at the stated station" % i)
				return
		elif drawn != 0.0:
			_check(false, "booster engine %d: drawn at its physics station" % i)
			return
	_check(true, "all %d plumes start at the drawn exit planes (ship: at the stated recess)" % engines.size())
	# A gimbaled ship engine: the plume leaves the tilted bell's rim.
	var last := engines.size() - 1
	var tilt := Vector3(0.99, 0.0, 0.14).normalized()
	engines[last]["direction"] = tilt
	view.sync(snapshot["stages"], engines, [], [], 0.0)
	var pose := view.engine_pose(last)
	var base_m := Vector3(stages[engines[last]["stage"]]["base_station_m"], 0.0, 0.0)
	_check(_plume_top(view, last).is_equal_approx(Frames.to_render(base_m + pose.origin
			- tilt * depths[engines[last]["vacuum"]])), "gimbaled: the plume stays on the drawn bell")
	_check(is_equal_approx(VehicleModels.exit_depth_m(models, false), 2.1) and is_equal_approx(
			VehicleModels.exit_depth_m(models, true), 3.6), "exit depths come from the manifest")


# ------------------------------------------------------------------------------------------------ snapshot from config

## [{id, name, base_station_m, length_m, diameter_m, nose_length_m, vehicle_id, config piece}] bottom to top.
func _config_stages() -> Array:
	var stages: Array = []
	var base := 0.0
	for stage_piece in _pieces():
		var stage: Dictionary = stage_piece["stage"][0]
		stages.append({"id": stage["id"], "base_station_m": base, "length_m": stage["length_m"],
				"diameter_m": stage["diameter_m"], "nose_length_m": stage["aero"].get("nose_length_m", 0.0),
				"vehicle_id": VEHICLE, "name": stage["name"]})
		base += stage["length_m"]
	return stages


func _pieces() -> Array:
	var path := ProjectSettings.globalize_path("res://").path_join("../config/vehicles/starship_block3.toml")
	var pieces: Array = []
	for chunk in FileAccess.get_file_as_string(path).split("[[stage]]", false).slice(1):
		pieces.append(TomlLite.parse("[[stage]]" + chunk))
	return pieces


func _piece(stages: Array, id: String) -> Dictionary:
	return _pieces()[stages.find_custom(func(s: Dictionary) -> bool: return s["id"] == id)]


## Engines from the config's groups (ring formula of core/src/vehicle/vehicle.cpp), flaps and grid fins at rest.
func _snapshot(stages: Array) -> Dictionary:
	var engines: Array = []
	var flaps: Array = []
	var fins: Array = []
	var pieces := _pieces()
	for i in stages.size():
		var base: float = stages[i]["base_station_m"]
		for group: Dictionary in pieces[i].get("stage.engine_group", []):
			var count := int(group["count"])
			for k in count:
				var angle := deg_to_rad(group["angles_deg"][k]) if group.has("angles_deg") else TAU * k / count
				var ring: float = group["ring_radius_m"]
				engines.append({"stage": i, "position_m": Vector3(base + group.get("station_m", 0.0),
						ring * cos(angle), ring * sin(angle)), "direction": Vector3.RIGHT, "firing": false, "throttle": 0.0,
						"spool": 0.0, "vacuum": group["engine"] == "raptor_vacuum3", "auxiliary": false})
		for flap: Dictionary in pieces[i].get("stage.flap", []):
			flaps.append({"hinge_azimuth_deg": flap["hinge_azimuth_deg"], "station_fwd_m": base + flap["station_fwd_m"],
					"station_aft_m": base + flap["station_aft_m"], "span_m": flap["span_m"],
					"deflection_deg": flap["initial_deflection_deg"]})
		var grid: Dictionary = pieces[i]["stage"][0].get("grid_fins", {})
		for azimuth: float in grid.get("azimuths_deg", []):
			fins.append({"station_m": base + grid["station_m"], "azimuth_deg": azimuth, "radius_m": grid["radius_m"],
					"area_m2": grid["area_m2"], "deflection_deg": 0.0})
	return {"stages": stages, "engines": engines, "flaps": flaps, "grid_fins": fins}


## The ship alone: its base is station 0 now and everything on it is shifted (Vehicle::separate).
func _separated(snapshot: Dictionary, stages: Array) -> Dictionary:
	var shift: float = stages[1]["base_station_m"]
	var ship: Dictionary = stages[1].duplicate()
	ship["base_station_m"] = 0.0
	ship["vehicle_id"] = VEHICLE + "/ship"
	var engines: Array = []
	for engine: Dictionary in snapshot["engines"]:
		if engine["stage"] == 1:
			var copy: Dictionary = engine.duplicate()
			copy["stage"] = 0
			copy["position_m"] = engine["position_m"] - Vector3(shift, 0.0, 0.0)
			engines.append(copy)
	var flaps: Array = []
	for flap: Dictionary in snapshot["flaps"]:
		var copy: Dictionary = flap.duplicate()
		copy["station_fwd_m"] -= shift
		copy["station_aft_m"] -= shift
		flaps.append(copy)
	return {"stages": [ship], "engines": engines, "flaps": flaps, "grid_fins": []}


func _view(snapshot: Dictionary, vehicle_id: String) -> VehicleView:
	var stages: Array = []
	for stage: Dictionary in snapshot["stages"]:
		var copy := stage.duplicate()
		if vehicle_id == "":
			copy.erase("vehicle_id")
		else:
			copy["vehicle_id"] = vehicle_id
		stages.append(copy)
	var view := VehicleView.new()
	_views.append(view)
	root.add_child(view)
	view.sync(stages, snapshot["engines"], snapshot["flaps"], snapshot["grid_fins"], 0.0)
	return view


# ------------------------------------------------------------------------------------------------ helpers

func _radial(azimuth_deg: float) -> Vector3:
	return Vector3(0.0, cos(deg_to_rad(azimuth_deg)), sin(deg_to_rad(azimuth_deg)))


## Start of plume `i` (the nozzle-exit end) in the view's render frame.
func _plume_top(view: VehicleView, index: int) -> Vector3:
	var plume: MeshInstance3D = view._plumes[index]
	var cone := plume.mesh as CylinderMesh
	return plume.transform * Vector3(0.0, cone.height * 0.5, 0.0)


func _cylinders(view: VehicleView) -> int:
	var n := 0
	for child in view.get_child(0).get_children():
		n += 1 if child is MeshInstance3D and (child as MeshInstance3D).mesh is CylinderMesh and child != null \
				and not view._plumes.has(child) else 0
	return n


func _count(node: Node, class_name_: String) -> int:
	return node.find_children("*", class_name_, true, false).size()


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
