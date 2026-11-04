## The ship's dorsal transfer hardware (tanker: four docking probes, two propellant connectors; depot: drogues, Task 4), drawn in the ship stage's body frame.
## `set_state` is the only entry for the probe visuals. Parts: dorsal_hardware.gltf (frames and the statement that the shapes are invented [E]:
## tools/structures/blender/gen_dorsal_hardware.py, config/vehicles/detail/starship_tanker.toml [dorsal]). Nodes: `probe_<i>` / `drogue_<i>` (origin = the config
## element = the extended tip), `connector_<name>` (origin = the coupling face), `dock_fairing` (fixed). No physics here.
class_name ProbeDrogueView
extends Node3D

## Snapshot docking phases (docking_state, German strings of contact::to_string / gnc::to_string) in which the probes are out: the final approach,
## contact and every phase until the hooks open again. Anything else stows them; "Halt", "Rückzug", "Abbruch" keep the state they find.
const EXTENDED_PROXIMITY := ["Endanflug", "Kontakt"]
const EXTENDED_DOCKING := ["Kontakt", "Weichfang", "Einzug", "Haken schließen", "angedockt", "Haken öffnen"]
const KEEP_PROXIMITY := ["Halt", "Rückzug", "Abbruch"]

var _axis := Vector3.BACK
var _stroke_m := 0.0
var _elements: Array[Vector3] = []
var _probes: Array[MeshInstance3D] = []
var _connectors: Dictionary = {}
var _extension := 0.0
var _captured := false


## `models`: the model set of the ship (VehicleModels.load_for); `material_of(part)`: the part's material.
func build(models: Dictionary, material_of: Callable) -> void:
	var manifest: Dictionary = models["manifest"]
	var parts: Dictionary = models["dorsal"]
	_axis = _vec(manifest["dorsal_axis"]).normalized()
	_stroke_m = float(manifest["probe_stroke_m"])
	var kind := String(manifest["dorsal_mechanism"])
	var names: Array = manifest["dorsal_nodes"]
	var elements: Array = manifest["dorsal_elements_m"]
	for i in names.size():
		var element := _vec(elements[i])
		_elements.append(element)
		var instance := _instance(String(names[i]), parts, material_of, self)
		instance.position = element
		if kind == "probe":
			_probes.append(instance)
	if parts.has("dock_fairing"):
		_instance("dock_fairing", parts, material_of, self)
	var connectors: Dictionary = manifest["dorsal_connectors"]
	for node_name: String in connectors:
		var instance := _instance(node_name, parts, material_of, self)
		instance.position = _vec(connectors[node_name]["position_m"])
		_connectors[node_name] = instance
	set_state(0.0, false)


## extension: 0 stowed .. 1 out (the drawing's `probe_stroke_m`, [E]); captured: the probes hold a drogue (stored, the V3 latch is not drawn). Drogues do not move.
func set_state(extension: float, captured: bool) -> void:
	_extension = clampf(extension, 0.0, 1.0)
	_captured = captured
	for i in _probes.size():
		_probes[i].position = _elements[i] - _axis * _stroke_m * (1.0 - _extension)


func extension() -> float:
	return _extension


func is_captured() -> bool:
	return _captured


func probe_count() -> int:
	return _probes.size()


## The probe's tip (its highest point along the port axis, on the axis line) in this view's frame, from the mesh itself.
func probe_tip_m(i: int) -> Vector3:
	return _probes[i].position + _axis * _axial_max(_probes[i])


## The coupling face centre of a connector node in this view's frame, from the mesh: the top of the part, on its axis.
func connector_face_m(node_name: String) -> Vector3:
	var instance: MeshInstance3D = _connectors[node_name]
	return instance.position + _axis * _axial_max(instance)


func connector_names() -> Array:
	return _connectors.keys()


## Highest point of the whole probe along the axis, local to the node (its tip), in this view's frame: the stowed envelope's test value.
func probe_top_axial_m(i: int) -> float:
	return _probes[i].position.dot(_axis) + _axial_max(_probes[i])


func _axial_max(instance: MeshInstance3D) -> float:
	var box := instance.mesh.get_aabb()
	var best := -INF
	for corner in 8:
		best = maxf(best, box.get_endpoint(corner).dot(_axis))
	return best


## The extension the snapshot's docking_state asks for: 1 in the final approach, contact and until the hooks open again (probes out), 0 otherwise;
## Halt / Rückzug / Abbruch keep `previous`. Mapping only.
static func extension_for(docking: Dictionary, previous: float) -> float:
	if not docking.get("present", false):
		return 0.0
	if docking.get("docked", false) or docking.get("latched", false) or EXTENDED_DOCKING.has(String(docking.get("docking_phase", ""))):
		return 1.0
	var proximity := String(docking.get("proximity_phase", ""))
	if EXTENDED_PROXIMITY.has(proximity):
		return 1.0
	return previous if KEEP_PROXIMITY.has(proximity) else 0.0


static func captured_for(docking: Dictionary) -> bool:
	return docking.get("present", false) and bool(docking.get("latched", false))


func _instance(part: String, parts: Dictionary, material_of: Callable, parent: Node3D) -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	instance.name = part
	instance.mesh = parts[part]
	instance.material_override = material_of.call(part)
	parent.add_child(instance)
	return instance


static func _vec(values: Array) -> Vector3:
	return Vector3(float(values[0]), float(values[1]), float(values[2]))
