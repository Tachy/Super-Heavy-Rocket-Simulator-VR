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
## Detail models of this vehicle ({} = cylinders) and the nodes posed from the snapshot, in snapshot order.
var _models := {}
var _stage_bases_m: Array[float] = []
var _stage_ids: Array[String] = []
var _engine_slots: Array[Dictionary] = []  # per engine: {"multimesh", "slot", "stage"}
var _flap_nodes: Array[Dictionary] = []  # per flap: {"node", "stage", "sign"}; the node is null without a model
var _fin_nodes: Array[Dictionary] = []  # per grid fin: {"node", "stage"}
var _engine_poses: Array[Transform3D] = []  # last pose per engine, stage-local body coordinates (headless tests)


func _init() -> void:  # not _ready: sync() may run before the view enters the tree
	add_child(_stage_root)
	_plume_material.shader = preload("res://shaders/plume.gdshader")
	_plume_material.set_shader_parameter("color", Color(1.0, 0.55, 0.25, 0.55))


## The hull shader's material of a model part (bare steel, ship hull with tiles, flap face with tiles).
func _hull_material(kind: String) -> Material:
	return _models["materials"][kind]


## "ship" / "booster" for a stage's hull node (they carry the steel map), "steel" for every other part.
func _hull_kind(part: String, stage: Dictionary) -> String:
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
	var modelled := not VehicleModels.load_for(_vehicle_id(stages)).is_empty()
	var offset := Frames.to_render(sim.body_position_gcrf_m(body_index) - camera_gcrf_m)
	visible = offset.length() < HIDE_BEYOND_M
	if not visible and _signature(stages) == _built_signature:
		return  # nothing to draw: no posing, no plumes (a new stage list is still built, so the next view is right)
	sync(stages, sim.body_engines(body_index), sim.body_flaps(body_index) if modelled else [],
			sim.body_grid_fins(body_index) if modelled else [], sim.telemetry(body_index).get("altitude_m", 0.0))
	if not visible:
		return
	var attitude := Basis(sim.body_attitude_gcrf(body_index))
	transform = Transform3D(Frames.basis_to_render(attitude), offset)
	var cg := sim.body_cg_station_m(body_index)
	_stage_root.position = Frames.to_render(Vector3(-cg, 0.0, 0.0))


## The snapshot -> nodes step of update_view (no sim, no placement): rebuilds on a new stage list, then poses the
## engines, flaps and grid fins of the models and the plumes. `flaps`, `grid_fins`: read only when models are shown.
func sync(stages: Array, engines: Array, flaps: Array, grid_fins: Array, altitude_m: float) -> void:
	var signature := _signature(stages)
	if signature != _built_signature:
		_models = VehicleModels.load_for(_vehicle_id(stages))
		_build(stages, engines, flaps, grid_fins)
		_built_signature = signature
	if not _models.is_empty():
		_pose_models(stages, engines, flaps, grid_fins)
	_update_plumes(engines, altitude_m)


## Last pose of engine `index` (stage-local body coordinates: gimbal point, thrust axis +X).
func engine_pose(index: int) -> Transform3D:
	return _engine_poses[index]


func _signature(stages: Array) -> String:
	return _vehicle_id(stages) + ":" + ",".join(stages.map(func(s: Dictionary) -> String: return s["id"]))


## The vehicle a snapshot belongs to ("" = unknown, an older snapshot: cylinders); a separated part reports
## "<vehicle>/<stage>".
static func _vehicle_id(stages: Array) -> String:
	return String(stages[0].get("vehicle_id", "")).get_slice("/", 0) if not stages.is_empty() else ""


func _build(stages: Array, engines: Array, flaps: Array, grid_fins: Array) -> void:
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
	var axis_rotation := Basis(Vector3.RIGHT, PI / 2.0)  # CylinderMesh +Y -> render +Z (= body +x)
	var staged: Array[bool] = []
	for stage: Dictionary in stages:
		_stage_bases_m.append(stage["base_station_m"])
		_stage_ids.append(String(stage["id"]))
		var hull: Dictionary = _models.get("hulls", {}).get(stage["id"], {})
		staged.append(not hull.is_empty())
		if hull.is_empty():
			_add_cylinders(stage, axis_rotation)
	if _models.is_empty():
		return
	var stage_nodes: Array[Node3D] = []
	for i in stages.size():
		stage_nodes.append(_add_stage_node(stages[i], _models["hulls"].get(stages[i]["id"], {})) if staged[i] else null)
	_build_engines(stages, engines, stage_nodes)
	_build_flaps(stages, flaps, stage_nodes)
	_build_fins(stages, grid_fins, stage_nodes)


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
func _add_stage_node(stage: Dictionary, hull: Dictionary) -> Node3D:
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
				else _hull_material(_hull_kind(part, stage))
		node.add_child(instance)
	return node


## One MultiMesh per stage and engine kind, one instance per snapshot engine (placed in _pose_models).
func _build_engines(stages: Array, engines: Array, stage_nodes: Array[Node3D]) -> void:
	var batches := {}  # "stage:vacuum" -> {"node", "vacuum", "count"}
	for engine: Dictionary in engines:
		var stage: int = engine["stage"]
		if stage < 0 or stage >= stage_nodes.size() or stage_nodes[stage] == null:
			_engine_slots.append({})
			continue
		var vacuum: bool = engine["vacuum"]
		var key := "%d:%s" % [stage, vacuum]
		if not batches.has(key):
			batches[key] = {"stage": stage, "vacuum": vacuum, "count": 0, "multimesh": MultiMesh.new()}
		var batch: Dictionary = batches[key]
		_engine_slots.append({"multimesh": batch["multimesh"], "slot": batch["count"], "stage": stage})
		batch["count"] += 1
	for batch: Dictionary in batches.values():
		var multimesh: MultiMesh = batch["multimesh"]
		multimesh.transform_format = MultiMesh.TRANSFORM_3D
		multimesh.mesh = _models["engine"][batch["vacuum"]]
		multimesh.instance_count = batch["count"]
		var instance := MultiMeshInstance3D.new()
		instance.name = "engines_" + ("rvac" if batch["vacuum"] else "raptor")
		instance.multimesh = multimesh
		instance.material_override = _steel
		stage_nodes[batch["stage"]].add_child(instance)


func _build_flaps(stages: Array, flaps: Array, stage_nodes: Array[Node3D]) -> void:
	for i in flaps.size():
		var flap: Dictionary = flaps[i]
		var stage := _stage_at(flap["station_aft_m"])
		var entry := {} if stage < 0 or stage_nodes[stage] == null else \
				VehicleModels.flap_entry(_models, stages[stage]["id"], flap, _stage_bases_m[stage])
		if entry.is_empty():
			_flap_nodes.append({"node": null})
			continue
		var node := Node3D.new()
		node.name = "flap_%d" % i
		var parts: Dictionary = _models["flap"][entry["kind"]]
		for part: String in parts:
			var instance := MeshInstance3D.new()
			instance.name = part
			instance.mesh = parts[part]
			instance.material_override = _hull_material("flap" if part == "panel" else "steel")
			node.add_child(instance)
		stage_nodes[stage].add_child(node)
		_flap_nodes.append({"node": node, "stage": stage, "sign": int(entry["rotation_sign_about_plus_x"])})


func _build_fins(stages: Array, grid_fins: Array, stage_nodes: Array[Node3D]) -> void:
	for i in grid_fins.size():
		var stage := _stage_at(grid_fins[i]["station_m"])
		if stage < 0 or stage_nodes[stage] == null:
			_fin_nodes.append({"node": null})
			continue
		var node := Node3D.new()
		node.name = "grid_fin_%d" % i
		for part: String in _models["grid_fin"]:
			var instance := MeshInstance3D.new()
			instance.name = part
			instance.mesh = _models["grid_fin"][part]
			instance.material_override = _hull_material("fin_cells" if part == "lattice" or part == "plate" else "fin_frame")
			if part == "lattice":
				instance.visibility_range_end = VehicleModels.FIN_LATTICE_RANGE_M
			elif part == "plate":
				instance.visibility_range_begin = VehicleModels.FIN_LATTICE_RANGE_M
			node.add_child(instance)
		stage_nodes[stage].add_child(node)
		_fin_nodes.append({"node": node, "stage": stage})


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
			var stage: int = entry["stage"]
			(entry["node"] as Node3D).transform = VehicleModels.flap_transform(flaps[i], _stage_bases_m[stage],
					stages[stage]["diameter_m"] * 0.5, entry["sign"])
	for i in mini(grid_fins.size(), _fin_nodes.size()):
		var entry: Dictionary = _fin_nodes[i]
		if entry["node"] != null:
			var stage: int = entry["stage"]
			(entry["node"] as Node3D).transform = VehicleModels.fin_transform(grid_fins[i], _stage_bases_m[stage],
					stages[stage]["diameter_m"] * 0.5)


## The drawn engine's offset from the physics gimbal point (VehicleModels.axial_offset_m), from its stage's id.
func _engine_axial_offset_m(engine: Dictionary) -> float:
	var stage: int = engine["stage"]
	if stage < 0 or stage >= _stage_ids.size():
		return 0.0
	return VehicleModels.axial_offset_m(_models, _stage_ids[stage], engine["vacuum"])


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
