## The ship's dorsal transfer hardware (tanker: four docking probes, two propellant connectors; depot: four drogues), drawn in the ship stage's body frame.
## `set_state` is the only entry for the probe visuals. Parts: dorsal_hardware.gltf (frames and the statement that the shapes are invented [E]:
## tools/structures/blender/gen_dorsal_hardware.py, config/vehicles/detail/starship_tanker.toml [dorsal]). Nodes: `probe_<i>` (origin = the probe TIP: extended = root + cone depth + extension, seated = root + cone depth; the config element is the probe ROOT on the interface plane) / `drogue_<i>` (origin = the centre of the mouth plane = the config element), `connector_<name>` (origin = the coupling face), `dock_fairing` (fixed). No physics here.
class_name ProbeDrogueView
extends Node3D

## Snapshot docking phases (docking_state, German strings of contact::to_string / gnc::to_string) in which the probes are out: the final approach,
## contact and every phase until the hooks open again. Anything else stows them; "Halt", "Rückzug", "Abbruch" keep the state they find.
const EXTENDED_PROXIMITY := ["Endanflug", "Kontakt"]
const EXTENDED_DOCKING := ["Kontakt", "Weichfang", "Einzug", "Haken schließen", "angedockt", "Haken öffnen"]
const KEEP_PROXIMITY := ["Halt", "Rückzug", "Abbruch"]

var _axis := Vector3.BACK
var _stroke_m := 0.0  # travel from stowed to extended (manifest probe_stroke_m, derived from the core's cone depth + extension)
var _stowed_m := 0.0  # tip offset along the axis from the ROOT (the config element): stowed
var _seated_extension := 1.0  # the extension 0..1 at which a head sits in its seat (the docked state)
var _elements: Array[Vector3] = []
var _probes: Array[MeshInstance3D] = []
var _drogues: Array[MeshInstance3D] = []
var _cone := {}  # manifest "dorsal_drogue": mouth radius, depth, throat radius of the funnel (drogue ships)
var _connectors: Dictionary = {}
var _extension := 0.0
var _captured := false


## `models`: the model set of the ship (VehicleModels.load_for); `material_of(part)`: the part's material.
func build(models: Dictionary, material_of: Callable) -> void:
	var manifest: Dictionary = models["manifest"]
	var parts: Dictionary = models["dorsal"]
	_axis = vec(manifest["dorsal_axis"]).normalized()
	_stroke_m = float(manifest["probe_stroke_m"])
	_stowed_m = float(manifest["probe_tip_offsets_m"]["stowed"])
	_seated_extension = float(manifest["probe_seated_extension"])
	var kind := String(manifest["dorsal_mechanism"])
	var names: Array = manifest["dorsal_nodes"]
	var elements: Array = manifest["dorsal_elements_m"]
	for i in names.size():
		var element := vec(elements[i])
		_elements.append(element)
		var instance := _instance(String(names[i]), parts, material_of, self)
		instance.position = element
		if kind == "probe":
			_probes.append(instance)
		else:
			_drogues.append(instance)  # the node origin is the centre of the mouth plane = the config element
	_cone = manifest.get("dorsal_drogue", {})
	if parts.has("dock_fairing"):
		_instance("dock_fairing", parts, material_of, self)
	var connectors: Dictionary = manifest["dorsal_connectors"]
	for node_name: String in connectors:
		var instance := _instance(node_name, parts, material_of, self)
		instance.position = vec(connectors[node_name]["position_m"])
		_connectors[node_name] = instance
	set_state(0.0, false)


## extension: 0 stowed .. 1 extended (travel = the manifest's probe_stroke_m, derived from the core's cone depth and extension); captured: the probes hold a drogue (stored, the V3 latch is not drawn). Drogues do not move.
func set_state(extension: float, captured: bool) -> void:
	_extension = clampf(extension, 0.0, 1.0)
	_captured = captured
	for i in _probes.size():
		_probes[i].position = _elements[i] + _axis * (_stowed_m + _stroke_m * _extension)  # the element is the ROOT, the node origin the tip


## The extension at which a head sits in its drogue's seat (the docked pose, the core's extension retracted to 0): between stowed (0) and extended (1).
func seated_extension() -> float:
	return _seated_extension


func extension() -> float:
	return _extension


func is_captured() -> bool:
	return _captured


func probe_count() -> int:
	return _probes.size()


func drogue_count() -> int:
	return _drogues.size()


## The centre of drogue i's mouth plane in this view's frame, from the mesh itself: the rim, the highest point along the port axis, on the axis line.
func drogue_mouth_m(i: int) -> Vector3:
	return _drogues[i].position + _axis * _axial_max(_drogues[i])


## Is `point_m` (this view's frame) inside the funnel of drogue i: between the mouth plane and the floor along the axis, within the cone's radius there?
## `tolerance_m` widens every bound (the closed cone, mouth plane included).
func inside_drogue_cone(i: int, point_m: Vector3, tolerance_m: float) -> bool:
	var rel := point_m - drogue_mouth_m(i)
	var below := -rel.dot(_axis)  # metres below the mouth plane
	var depth := float(_cone["drogue_depth_m"])
	if below < -tolerance_m or below > depth + tolerance_m:
		return false
	var lateral := (rel + _axis * below).length()
	var mouth := float(_cone["drogue_mouth_radius_m"])
	var throat := float(_cone["drogue_throat_radius_m"])
	var radius := mouth - (mouth - throat) * clampf(below, 0.0, depth) / depth
	return lateral <= radius + tolerance_m


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


## The extension the snapshot's docking_state asks for: 1 (extended: core extension_m beyond the seated length) in the final approach, contact and
## soft capture, halfway to `seated` during the draw-in ("Einzug"), `seated` (heads in their seats) from the hooks closing until they open again, docked or
## latched; 0 (stowed) otherwise; Halt / Rückzug / Abbruch keep `previous`. Mapping only.
static func extension_for(docking: Dictionary, previous: float, seated: float) -> float:
	if not docking.get("present", false):
		return 0.0
	var phase := String(docking.get("docking_phase", ""))
	if phase == "Kontakt" or phase == "Weichfang":
		return 1.0
	if phase == "Einzug":
		return 0.5 * (1.0 + seated)
	if docking.get("docked", false) or docking.get("latched", false) or EXTENDED_DOCKING.has(phase):
		return seated
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


static func vec(values: Array) -> Vector3:
	return Vector3(float(values[0]), float(values[1]), float(values[2]))
