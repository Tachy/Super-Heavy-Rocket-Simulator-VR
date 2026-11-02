class_name VehicleView
extends Node3D
## Visual of one flying body (stack, booster or ship) built from its stage geometry, with engine plumes.
## Placed every frame relative to the floating origin; mesh coordinates are body coordinates
## (x = vehicle axis) mapped to render axes and relative to the centre of gravity.
## A vehicle with detail models (VehicleModels: hull, engines, flaps, grid fins) shows them, posed from the snapshot
## every frame (no own clock); every other vehicle, and this one without generated models, keeps the cylinders.

const PLUME_SEA_LEVEL_LENGTH_M := 35.0
const PLUME_VACUUM_LENGTH_M := 160.0
const HIDE_BEYOND_M := 2.0e6  # vehicles are sub-pixel beyond this distance

var body_index := 0
var _stage_root := Node3D.new()
var _plumes: Array[MeshInstance3D] = []
## Stainless steel (no reflection probes yet), in the atmosphere's light and haze.
var _steel := LitAerial.material(Color(0.74, 0.75, 0.77), 0.38, 0.65)
## Additive glow, seen through the atmosphere's transmittance (shaders/plume.gdshader; its former unshaded
## StandardMaterial3D drew the albedo only - the emission settings it carried had no effect).
var _plume_material := ShaderMaterial.new()
## The ship's engine bay: black plate and sooted wall (frames f12 138.9 s, f13 2262 s) [E].
var _engine_bay_material := LitAerial.material(Color(0.035, 0.035, 0.04), 0.55, 0.3)
var _built_signature := ""
## Detail models of this vehicle ({} = cylinders) and the nodes posed from the snapshot, in snapshot order. `_models` is the
## set of the lowest stage's vehicle (or the first loaded one); a stack with a payload ("falcon9_block5+crew_dragon") also loads
## the payload's set, and every stage draws from the set that holds its hull (`_stage_models`, {} = cylinders).
var _models := {}
var _model_sets: Array[Dictionary] = []
var _stage_models: Array[Dictionary] = []
var _stage_bases_m: Array[float] = []
var _stage_ids: Array[String] = []
var _engine_slots: Array[Dictionary] = []  # per engine: {"multimesh", "slot", "stage"}
## Drawn deflection of every flap for comparison shots (`vehicle_look_shots.gd --flaps`); NAN = the snapshot's. Drawing only.
static var look_flap_deg := NAN
## The same for the legs' deployment (`vehicle_look_shots.gd --legs`).
static var look_leg_deployment := NAN
## The same for the Crew Dragon's nose cone (`--nose`, 0 closed .. 1 open, shown at once).
static var look_nose_fraction := NAN

var _flap_nodes: Array[Dictionary] = []  # per flap: {"node", "entry"} (manifest entry); the node is null without a model
var _fin_nodes: Array[Dictionary] = []  # per grid fin: {"node", "stage"}
var _engine_poses: Array[Transform3D] = []  # last pose per engine, stage-local body coordinates (headless tests)
var _leg_nodes: Array[Dictionary] = []  # per leg: {"azimuth_deg", "main", "pusher"}
## The Crew Dragon's nose cone: {"node", "nose"} (manifest nose block) or {}; the fraction (0 closed .. 1 open) eases towards the
## snapshot's binary nose_cone_open at 1 / open_time_s per second: a view easing, no state of the simulation.
var _nose := {}
var _nose_fraction := 0.0
var _nose_target := 0.0
var _nose_settled := false  # the first snapshot after a build shows the state as it is (no opening on appearing)


func _init() -> void:  # not _ready: sync() may run before the view enters the tree
	add_child(_stage_root)
	_plume_material.shader = preload("res://shaders/plume.gdshader")
	_plume_material.set_shader_parameter("color", Color(1.0, 0.55, 0.25, 0.55))


## The hull shader's material of a model part (bare steel, ship hull with tiles, flap face with tiles).
func _hull_material(kind: String, models: Dictionary = _models) -> Material:
	return models["materials"][kind]


## "ship" / "booster" for a stage's hull node (they carry the steel map), "steel" for every other part.
func _hull_kind(part: String, stage: Dictionary, models: Dictionary = _models) -> String:
	var part_materials: Dictionary = models["manifest"].get("look", {}).get("part_materials", {})
	var own: Dictionary = models["manifest"].get("look", {}).get("stage_part_materials", {}).get(stage["id"], {})
	if own.has(part):  # a part drawn differently on this stage (the Falcon 9's white second stage hull)
		return own[part]
	if part_materials.has(part):
		return part_materials[part]
	if part == "fin_fairings":
		return "dark"
	if part == "hinge_covers":
		return "ship"  # tiled: they lie inside the coverage
	if part != "hull":
		return "steel"
	return "ship" if stage["id"] == "ship" else "booster"


## Rebuilds geometry when the stage list changed (e.g. after separation), then places the body.
func update_view(sim: FlightSim, camera_gcrf_m: Vector3) -> void:
	var stages: Array = sim.body_stages(body_index)
	var modelled := not _load_sets(stages).is_empty()
	var offset := Frames.to_render(sim.body_position_gcrf_m(body_index) - camera_gcrf_m)
	visible = offset.length() < HIDE_BEYOND_M
	if not visible and _signature(stages) == _built_signature:
		return  # nothing to draw: no posing, no plumes (a new stage list is still built, so the next view is right)
	sync(stages, sim.body_engines(body_index), sim.body_flaps(body_index) if modelled else [],
			sim.body_grid_fins(body_index) if modelled else [], sim.telemetry(body_index).get("altitude_m", 0.0),
			sim.body_legs(body_index) if modelled else {}, sim.capsule_state(body_index) if modelled and not _nose.is_empty() else {})
	if not visible:
		return
	var attitude := Basis(sim.body_attitude_gcrf(body_index))
	transform = Transform3D(Frames.basis_to_render(attitude), offset)
	var cg := sim.body_cg_station_m(body_index)
	_stage_root.position = Frames.to_render(Vector3(-cg, 0.0, 0.0))


## The snapshot -> nodes step of update_view (no sim, no placement): rebuilds on a new stage list, then poses the
## engines, flaps, grid fins and legs of the models and the plumes. `flaps`, `grid_fins`, `legs`: read only when models are shown.
func sync(stages: Array, engines: Array, flaps: Array, grid_fins: Array, altitude_m: float, legs := {}, capsule := {}) -> void:
	var signature := _signature(stages)
	if signature != _built_signature:
		_load_sets(stages)
		_build(stages, engines, flaps, grid_fins, legs)
		_built_signature = signature
	if not _models.is_empty():
		_pose_models(stages, engines, flaps, grid_fins)
		_pose_legs(legs)
		_set_nose_target(capsule)
	_update_plumes(engines, altitude_m)


## Loads the model sets of every vehicle named in the stage list's id ("a+b/c": a launcher and its payload) and sets
## `_model_sets`, `_models` (the set holding the lowest stage's hull, else the first loaded one) and `_stage_models`.
## Returns the loaded sets.
func _load_sets(stages: Array) -> Array[Dictionary]:
	_model_sets.clear()
	_models = {}
	for id in _vehicle_ids(stages):
		var ms := VehicleModels.load_for(id)
		if not ms.is_empty():
			_model_sets.append(ms)
	for ms in _model_sets:
		if not stages.is_empty() and (ms["hulls"] as Dictionary).has(stages[0]["id"]):
			_models = ms
	if _models.is_empty() and not _model_sets.is_empty():
		_models = _model_sets[0]
	_stage_models.clear()
	for stage: Dictionary in stages:
		var own := {}
		for ms in _model_sets:
			if (ms["hulls"] as Dictionary).has(stage["id"]):
				own = ms
		_stage_models.append(own)
	return _model_sets


## Closed -> open and back: eased towards the snapshot's nose_cone_open (capsule_state; {} or not present = closed, as on a stack).
func _set_nose_target(capsule: Dictionary) -> void:
	if _nose.is_empty():
		return
	_nose_target = 1.0 if capsule.get("present", false) and capsule.get("nose_cone_open", false) else 0.0
	if not is_nan(look_nose_fraction):
		_nose_target = look_nose_fraction
		_nose_fraction = look_nose_fraction
	if not _nose_settled:
		_nose_fraction = _nose_target
		_nose_settled = true
	_apply_nose()


## Moves the nose fraction towards its target by `delta` seconds of the opening time (called by _process; tests call it directly).
func advance_nose(delta: float) -> void:
	if _nose.is_empty():
		return
	_nose_fraction = move_toward(_nose_fraction, _nose_target, delta / maxf(float(_nose["nose"]["open_time_s"]), 1e-3))
	_apply_nose()


func nose_fraction() -> float:
	return _nose_fraction


func _apply_nose() -> void:
	(_nose["node"] as Node3D).transform = VehicleModels.nose_transform(_nose["nose"], _nose_fraction)


func _process(delta: float) -> void:
	advance_nose(delta)


## Last pose of engine `index` (stage-local body coordinates: gimbal point, thrust axis +X).
func engine_pose(index: int) -> Transform3D:
	return _engine_poses[index]


func _signature(stages: Array) -> String:
	return _vehicle_id(stages) + ":" + ",".join(stages.map(func(s: Dictionary) -> String: return s["id"]))


## The vehicle a snapshot belongs to ("" = unknown, an older snapshot: cylinders); a separated part reports
## "<vehicle>/<stage>".
static func _vehicle_id(stages: Array) -> String:
	return String(stages[0].get("vehicle_id", "")).get_slice("/", 0) if not stages.is_empty() else ""


## The vehicles whose models a stage list may use: the parts of a stack's id ("falcon9_block5+crew_dragon" -> both), in order.
static func _vehicle_ids(stages: Array) -> Array[String]:
	var ids: Array[String] = []
	for part in _vehicle_id(stages).split("+", false):
		ids.append(part)
	return ids


func _build(stages: Array, engines: Array, flaps: Array, grid_fins: Array, legs: Dictionary) -> void:
	for child in _stage_root.get_children():
		child.free()  # immediately (not queue_free): the rebuilt nodes reuse the names, and nothing here is in a callback
	_plumes.clear()
	_stage_bases_m.clear()
	_stage_ids.clear()
	_engine_slots.clear()
	_engine_poses.clear()
	_engine_poses.resize(engines.size())
	_flap_nodes.clear()
	_fin_nodes.clear()
	_leg_nodes.clear()
	_nose = {}
	_nose_settled = false
	var axis_rotation := Basis(Vector3.RIGHT, PI / 2.0)  # CylinderMesh +Y -> render +Z (= body +x)
	var staged: Array[bool] = []
	for stage: Dictionary in stages:
		_stage_bases_m.append(stage["base_station_m"])
		_stage_ids.append(String(stage["id"]))
		var hull: Dictionary = _stage_hull(_stage_ids.size() - 1)
		staged.append(not hull.is_empty())
		if hull.is_empty():
			_add_cylinders(stage, axis_rotation)
	if _models.is_empty():
		var notice := ModelNotice.label(_notice_text(stages))  ## stale / missing models: say so on screen
		if notice != null:
			_stage_root.add_child(notice)
		return
	var stage_nodes: Array[Node3D] = []
	for i in stages.size():
		stage_nodes.append(_add_stage_node(stages[i], _stage_hull(i), _stage_models[i]) if staged[i] else null)
	_build_engines(stages, engines, stage_nodes)
	_build_flaps(stages, flaps, stage_nodes)
	_build_fins(stages, grid_fins, stage_nodes)
	if not stage_nodes.is_empty():
		_build_legs(legs, stage_nodes[0], _stage_models[0])
	_build_nose(stages, stage_nodes)


## The hull meshes of stage `index` from its own model set ({} = no model: cylinders).
func _stage_hull(index: int) -> Dictionary:
	if index < 0 or index >= _stage_models.size() or index >= _stage_ids.size():
		return {}
	return _stage_models[index].get("hulls", {}).get(_stage_ids[index], {})


## German notice for the first vehicle of the stage list whose models are stale or missing ("" = none).
static func _notice_text(stages: Array) -> String:
	for id in _vehicle_ids(stages):
		var text := VehicleModels.notice(id)
		if text != "":
			return text
	return ""


## The Crew Dragon's nose cone on its capsule stage, hinged at the manifest's point; posed by _apply_nose.
func _build_nose(stages: Array, stage_nodes: Array[Node3D]) -> void:
	for i in stages.size():
		var ms: Dictionary = _stage_models[i]
		if stage_nodes[i] == null or not ms.has("nose") or String(ms["manifest"]["nose"]["stage_id"]) != String(stages[i]["id"]):
			continue
		var node := Node3D.new()
		node.name = "nose_cone"
		for part: String in ms["nose"]:
			var instance := MeshInstance3D.new()
			instance.name = part
			instance.mesh = ms["nose"][part]
			instance.material_override = _hull_material(_hull_kind(part, stages[i], ms), ms)
			node.add_child(instance)
		stage_nodes[i].add_child(node)
		_nose = {"node": node, "nose": ms["manifest"]["nose"]}
		_apply_nose()
		return


## One node pair per snapshot leg azimuth on the lowest stage (the snapshot's legs are that stage's).
func _build_legs(legs: Dictionary, stage_node: Node3D, models: Dictionary) -> void:
	if stage_node == null or not legs.get("present", false) or not models.has("leg"):
		return
	var paint: Material = models["materials"]["paint_leg"]
	for i in (legs["azimuths_deg"] as Array).size():
		var parts := {}
		for part: String in ["leg_main", "leg_pusher"]:
			var instance := MeshInstance3D.new()
			instance.name = "%s_%d" % [part, i]
			instance.mesh = models["leg"][part]
			instance.material_override = paint
			stage_node.add_child(instance)
			parts[part] = instance
		_leg_nodes.append({"azimuth_deg": float(legs["azimuths_deg"][i]), "main": parts["leg_main"], "pusher": parts["leg_pusher"]})


func _pose_legs(legs: Dictionary) -> void:
	if _leg_nodes.is_empty():
		return
	var deployment := float(legs.get("deployment", 0.0)) if is_nan(look_leg_deployment) else look_leg_deployment
	for leg: Dictionary in _leg_nodes:
		var poses := VehicleModels.leg_poses(_models["manifest"]["leg"], leg["azimuth_deg"], deployment)
		(leg["main"] as Node3D).transform = poses["main"]
		(leg["pusher"] as Node3D).transform = poses["pusher"]


func _add_cylinders(stage: Dictionary, axis_rotation: Basis) -> void:
	var radius: float = stage["diameter_m"] * 0.5
	var nose: float = stage["nose_length_m"]
	var barrel: float = stage["length_m"] - nose
	_add_cylinder(radius, radius, barrel, stage["base_station_m"] + barrel * 0.5, axis_rotation)
	if nose > 0.0:
		_add_cylinder(radius * 0.08, radius, nose, stage["base_station_m"] + barrel + nose * 0.5, axis_rotation)


func _add_cylinder(top_radius: float, bottom_radius: float, height: float, center_station: float,
		rotation: Basis) -> void:
	var mesh := CylinderMesh.new()
	mesh.top_radius = top_radius
	mesh.bottom_radius = bottom_radius
	mesh.height = height
	mesh.radial_segments = 48
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	instance.material_override = _steel
	instance.transform = Transform3D(rotation, Frames.to_render(Vector3(center_station, 0.0, 0.0)))
	_stage_root.add_child(instance)


## A stage's own node (its transform survives a separation: the stage keeps its base): origin at the stage's station 0,
## the body -> render permutation as basis; hull, engines, flaps and fins are its children in body coordinates.
func _add_stage_node(stage: Dictionary, hull: Dictionary, models: Dictionary) -> Node3D:
	var node := Node3D.new()
	node.name = "stage_" + String(stage["id"])
	var base := Frames.to_render(Vector3(stage["base_station_m"], 0.0, 0.0))
	node.transform = Transform3D(VehicleModels.body_to_render(), base)
	_stage_root.add_child(node)
	for part: String in hull:
		var instance := MeshInstance3D.new()
		instance.name = part
		instance.mesh = hull[part]
		instance.material_override = _engine_bay_material if part == "engine_bay" \
				else _hull_material(_hull_kind(part, stage, models), models)
		node.add_child(instance)
	return node


## One MultiMesh per stage and engine kind, one instance per snapshot engine (placed in _pose_models).
func _build_engines(stages: Array, engines: Array, stage_nodes: Array[Node3D]) -> void:
	var batches := {}  # "stage:vacuum" -> {"node", "vacuum", "count"}
	for engine: Dictionary in engines:
		var stage: int = engine["stage"]
		var vacuum: bool = engine["vacuum"]
		if stage < 0 or stage >= stage_nodes.size() or stage_nodes[stage] == null \
				or not (_stage_models[stage].get("engine", {}) as Dictionary).has(vacuum):
			_engine_slots.append({})
			continue
		var key := "%d:%s" % [stage, vacuum]
		if not batches.has(key):
			batches[key] = {"stage": stage, "vacuum": vacuum, "count": 0, "multimesh": MultiMesh.new()}
		var batch: Dictionary = batches[key]
		_engine_slots.append({"multimesh": batch["multimesh"], "slot": batch["count"], "stage": stage})
		batch["count"] += 1
	for batch: Dictionary in batches.values():
		var multimesh: MultiMesh = batch["multimesh"]
		multimesh.transform_format = MultiMesh.TRANSFORM_3D
		multimesh.mesh = _stage_models[batch["stage"]]["engine"][batch["vacuum"]]
		multimesh.instance_count = batch["count"]
		var instance := MultiMeshInstance3D.new()
		instance.name = "engines_" + ("rvac" if batch["vacuum"] else "raptor")
		instance.multimesh = multimesh
		instance.material_override = VehicleModels.engine_material(batch["vacuum"], _stage_models[batch["stage"]])
		stage_nodes[batch["stage"]].add_child(instance)


func _build_flaps(stages: Array, flaps: Array, stage_nodes: Array[Node3D]) -> void:
	for i in flaps.size():
		var flap: Dictionary = flaps[i]
		var stage := _stage_at(flap["station_aft_m"])
		var entry := {} if stage < 0 or stage_nodes[stage] == null else \
				VehicleModels.flap_entry(_stage_models[stage], stages[stage]["id"], flap, _stage_bases_m[stage])
		if entry.is_empty():
			_flap_nodes.append({"node": null})
			continue
		var node := Node3D.new()
		node.name = "flap_%d" % i
		var parts: Dictionary = _stage_models[stage]["flap"][entry["kind"]]
		for part: String in parts:
			var instance := MeshInstance3D.new()
			instance.name = part
			instance.mesh = parts[part]
			instance.material_override = _hull_material("flap" if part == "panel" else "steel", _stage_models[stage])
			node.add_child(instance)
		stage_nodes[stage].add_child(node)
		_flap_nodes.append({"node": node, "entry": entry})


func _build_fins(stages: Array, grid_fins: Array, stage_nodes: Array[Node3D]) -> void:
	for i in grid_fins.size():
		var stage := _stage_at(grid_fins[i]["station_m"])
		if stage < 0 or stage_nodes[stage] == null:
			_fin_nodes.append({"node": null})
			continue
		var node := Node3D.new()
		node.name = "grid_fin_%d" % i
		var fin_models: Dictionary = _stage_models[stage]
		var catch_azimuths: Array = fin_models["manifest"].get("look", {}).get("catch_fin_azimuths_deg", [])
		for part: String in fin_models["grid_fin"]:
			if part == "bearing" and not _is_catch_fin(float(grid_fins[i]["azimuth_deg"]), catch_azimuths):
				continue
			var instance := MeshInstance3D.new()
			instance.name = part
			instance.mesh = fin_models["grid_fin"][part]
			var material_key := "fin_frame"
			if part == "lattice" or part == "plate" or part == "frame":  # the frame wall is black like the lattice
				material_key = "fin_cells"
			elif part == "band" or part == "bearing":
				material_key = "fin_band"
			instance.material_override = _hull_material(material_key, fin_models)
			if part == "lattice":
				instance.visibility_range_end = VehicleModels.FIN_LATTICE_RANGE_M
			elif part == "plate":
				instance.visibility_range_begin = VehicleModels.FIN_LATTICE_RANGE_M
			node.add_child(instance)
		stage_nodes[stage].add_child(node)
		_fin_nodes.append({"node": node, "stage": stage})


## True when `azimuth_deg` equals one of the manifest's catch fin azimuths (the fins with the flat bar under the root).
static func _is_catch_fin(azimuth_deg: float, catch_azimuths: Array) -> bool:
	for entry in catch_azimuths:
		if absf(fposmod(azimuth_deg - float(entry) + 180.0, 360.0) - 180.0) < 1.0:
			return true
	return false


## Index of the stage whose base is the last one at or below `station_m` (-1 without stages).
func _stage_at(station_m: float) -> int:
	var found := -1
	for i in _stage_bases_m.size():
		if _stage_bases_m[i] <= station_m + 1e-6:
			found = i
	return found


## Engines at their gimbal pose, flaps and grid fins at the snapshot deflections. A snapshot that lacks an entry the
## model was built for (counts differ) leaves that node as it was.
func _pose_models(stages: Array, engines: Array, flaps: Array, grid_fins: Array) -> void:
	for i in mini(engines.size(), _engine_slots.size()):
		var slot: Dictionary = _engine_slots[i]
		if slot.is_empty():
			continue
		var engine: Dictionary = engines[i]
		var pose := VehicleModels.engine_transform(engine["position_m"], engine["direction"],
				_stage_bases_m[slot["stage"]], _engine_axial_offset_m(engine))
		(slot["multimesh"] as MultiMesh).set_instance_transform(slot["slot"], pose)
		_engine_poses[i] = pose
	for i in mini(flaps.size(), _flap_nodes.size()):
		var entry: Dictionary = _flap_nodes[i]
		if entry["node"] != null:
			var flap: Dictionary = flaps[i]
			if not is_nan(look_flap_deg):
				flap = flap.duplicate()
				flap["deflection_deg"] = look_flap_deg
			(entry["node"] as Node3D).transform = VehicleModels.flap_transform(flap, entry["entry"])
	for i in mini(grid_fins.size(), _fin_nodes.size()):
		var entry: Dictionary = _fin_nodes[i]
		if entry["node"] != null:
			var stage: int = entry["stage"]
			var radius: float = stages[stage]["diameter_m"] * 0.5
			var standoff: Variant = _stage_models[stage]["manifest"].get("look", {}).get("fin_stow_standoff_m")
			if standoff != null and stage < stages.size() - 1:  # a vehicle whose fins fold: folded while a stage stands on this one
				(entry["node"] as Node3D).transform = VehicleModels.fin_stowed_transform(grid_fins[i], _stage_bases_m[stage], radius, float(standoff))
			else:
				(entry["node"] as Node3D).transform = VehicleModels.fin_transform(grid_fins[i], _stage_bases_m[stage], radius)


## The drawn engine's offset from the physics gimbal point (VehicleModels.axial_offset_m), from its stage's id.
func _engine_axial_offset_m(engine: Dictionary) -> float:
	var stage: int = engine["stage"]
	if stage < 0 or stage >= _stage_ids.size():
		return 0.0
	return VehicleModels.axial_offset_m(_stage_models[stage] if not _stage_models[stage].is_empty() else _models, _stage_ids[stage], engine["vacuum"])


func _update_plumes(engines: Array, altitude_m: float) -> void:
	while _plumes.size() < engines.size():
		var plume := MeshInstance3D.new()
		var cone := CylinderMesh.new()
		cone.radial_segments = 16
		cone.rings = 1
		plume.mesh = cone
		plume.material_override = _plume_material
		plume.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		_stage_root.add_child(plume)
		_plumes.append(plume)
	# Plumes widen and lengthen as ambient pressure drops (under-expanded exhaust).
	var expansion := clampf(altitude_m / 60000.0, 0.0, 1.0)
	for i in engines.size():
		var engine: Dictionary = engines[i]
		var plume := _plumes[i]
		plume.visible = engine["firing"]
		if not plume.visible:
			continue
		var strength: float = engine["spool"] * (0.4 + 0.6 * engine["throttle"])
		var base_length := PLUME_VACUUM_LENGTH_M if engine["vacuum"] else PLUME_SEA_LEVEL_LENGTH_M
		var length := base_length * (0.6 + 1.4 * expansion) * strength
		var cone := plume.mesh as CylinderMesh
		cone.height = length
		cone.top_radius = 0.6 if not engine["vacuum"] else 1.1
		if engine["vacuum"] and not _models.is_empty() and _models["manifest"].get("look", {}).get("plume_start_at_exit_diameter", false):
			var rim: Dictionary = _models["manifest"]["engines"].get(VehicleModels.engine_id(_models, true), {})
			if rim.has("exit_diameter_m"):
				cone.top_radius = 0.5 * float(rim["exit_diameter_m"])  # the plume leaves the rim of the bell
		cone.bottom_radius = (1.0 + 6.0 * expansion) * (1.6 if engine["vacuum"] else 1.0)
		# Exhaust leaves opposite to the thrust direction, starting at the DRAWN nozzle exit: the model's exit plane below
		# the drawn gimbal point (manifest; Raptor 2.1 m, RVac 3.6 m), ~3 m for the cylinders.
		var direction: Vector3 = engine["direction"]
		var exhaust := Frames.to_render(-direction).normalized()
		var depth := VehicleModels.exit_depth_m(_models, engine["vacuum"])
		var nozzle_exit: Vector3 = engine["position_m"] + Vector3(_engine_axial_offset_m(engine), 0.0, 0.0) \
				- direction * depth
		var center := Frames.to_render(nozzle_exit) + exhaust * length * 0.5
		plume.transform = Transform3D(_basis_from_y(-exhaust), center)


## Basis whose +Y axis points along `y` (CylinderMesh: top at +Y = nozzle side).
static func _basis_from_y(y: Vector3) -> Basis:
	var x := y.cross(Vector3.FORWARD if absf(y.z) < 0.9 else Vector3.RIGHT).normalized()
	return Basis(x, y, x.cross(y))
