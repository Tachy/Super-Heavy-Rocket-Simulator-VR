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
	var no_snapshot_id := _view(snapshot, "")
	_check(_cylinders(no_snapshot_id) == 3, "a snapshot without vehicle_id keeps the cylinders")
	var models := VehicleModels.load_for(VEHICLE)
	if models.is_empty():
		print("  (no models: model cases skipped; run python tools/structures/build_models.py --vehicle %s)" % VEHICLE)
	else:
		_check_models(stages, snapshot, models)
		VehicleModels._cache[VEHICLE] = {}  # as without generated models
		var fallback := _view(snapshot, VEHICLE)
		_check(_cylinders(fallback) == 3 and _count(fallback, "MultiMeshInstance3D") == 0,
				"without models the vehicle keeps its cylinders")
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
		_check(pose.origin.is_equal_approx(engine["position_m"] - Vector3(base, 0.0, 0.0))
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
	var belly_deg: float = _piece(stages, "ship")["stage"][0]["aero"]["belly_azimuth_deg"]
	var belly := _radial(belly_deg)
	var hull_radius: float = stages[1]["diameter_m"] * 0.5
	var base: float = stages[1]["base_station_m"]
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
			# core/src/aero/flap.cpp flap_span_direction: stow = X x e_r away from the belly; d = 0 stow, 90 radial.
			var stow := Vector3.RIGHT.cross(e_r)
			stow = -stow if stow.dot(belly) > 0.0 else stow
			var span := stow * cos(deg_to_rad(state)) + e_r * sin(deg_to_rad(state))
			var label := "flap %d at %.0f deg" % [i, state]
			_check((node.transform.basis * Vector3.BACK).is_equal_approx(span),
					label + ": panel along the core's span direction")
			_check((node.transform.basis * Vector3.RIGHT).is_equal_approx(Vector3.RIGHT),
					label + ": hinge along the station axis")
			var hinge := Vector3(flap["station_aft_m"] - base, e_r.y * hull_radius, e_r.z * hull_radius)
			_check(node.transform.origin.is_equal_approx(hinge), label + ": hinge on the hull at the aft station")
			# The tile-covered +Y face looks at the flow side: radial at rest, towards the belly at 90 deg.
			var outer := node.transform.basis * Vector3.UP
			if state == 0.0:
				_check(outer.is_equal_approx(e_r), label + ": rest pose, outer face radial")
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
	for i in engines.size():
		var engine: Dictionary = engines[i]
		var exit_m: Vector3 = engine["position_m"] - engine["direction"] * depths[engine["vacuum"]]
		if not _plume_top(view, i).is_equal_approx(Frames.to_render(exit_m)):
			_check(false, "engine %d: plume starts at the exit plane" % i)
			return
	_check(true, "all %d plumes start at the exit planes (Raptor 2.1 m, RVac 3.6 m)" % engines.size())
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
					"station_aft_m": base + flap["station_aft_m"], "span_m": flap["span_m"], "deflection_deg": 0.0})
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
