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
	_check_falcon9()
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
func _check_falcon9() -> void:
	VehicleModels.clear_cache()
	var models := VehicleModels.load_for("falcon9_block5")
	if models.is_empty():
		_check(false, "falcon9_block5: models generated (%s)" % VehicleModels.fallback_reason("falcon9_block5"))
		return
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
	_check(_cylinders(view) == 1 and view.find_children("ModelNotice", "Label3D", true, false).is_empty(), "first stage modelled, no notice; the second stage keeps its cylinder")
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
	VehicleModels.clear_cache()


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
