# The tanker's dorsal probes and connectors and the depot's drogues (ProbeDrogueView) against the vehicle config, the snapshot -> extension mapping, and the
# docked pair (tanker on the depot in the sim's mated pose).
# Needs the generated models (python tools/structures/build_models.py --vehicle starship_tanker, then --vehicle starship_depot).
# godot --headless --path game --script res://tests/probe_drogue_test.gd
extends SceneTree

const TOLERANCE_M := 0.01  # Task 3 brief
# Elements, connector positions and the stroke are read from the manifest (dorsal_elements_m, dorsal_connectors, probe_stroke_m), which the build copies from
# config/vehicles/starship_tanker.toml; python test_dorsal_hardware.py compares the manifest with the config.

var _failures := 0
var _view: ProbeDrogueView
var _elements: Array[Vector3] = []
var _connectors: Dictionary = {}
var _stroke_m := 0.0


func _initialize() -> void:
	VehicleModels.clear_cache()
	var models := VehicleModels.load_for("starship_tanker")
	if models.is_empty() or (models.get("dorsal", {}) as Dictionary).is_empty():
		_check(false, "starship_tanker: dorsal models generated (%s)" % VehicleModels.fallback_reason("starship_tanker"))
		quit(1)
		return
	_view = ProbeDrogueView.new()
	root.add_child(_view)
	_view.build(models, func(_part: String) -> Material: return null)
	var manifest: Dictionary = models["manifest"]
	for e: Array in manifest["dorsal_elements_m"]:
		_elements.append(Vector3(e[0], e[1], e[2]))
	for node_name: String in manifest["dorsal_connectors"]:
		var p: Array = manifest["dorsal_connectors"][node_name]["position_m"]
		_connectors[node_name] = Vector3(p[0], p[1], p[2])
	_stroke_m = float(manifest["probe_stroke_m"])
	_check(_check_manifest(manifest) == true, "check_manifest ran to its end (an aborted check is a failure)")
	_check(_check_stowed(manifest) == true, "check_stowed ran to its end (an aborted check is a failure)")
	_check(_check_extended() == true, "check_extended ran to its end (an aborted check is a failure)")
	_check(_check_connectors() == true, "check_connectors ran to its end (an aborted check is a failure)")
	_check(_check_phase_mapping() == true, "check_phase_mapping ran to its end (an aborted check is a failure)")
	_check(_check_depot_drogues() == true, "check_depot_drogues ran to its end (an aborted check is a failure)")
	_check(_check_docked_pair() == true, "check_docked_pair ran to its end (an aborted check is a failure)")
	_check(_check_hls_pair() == true, "check_hls_pair ran to its end (an aborted check is a failure)")
	_view.free()
	print("PROBE DROGUE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		print("FAIL: ", what)
	else:
		print("ok: ", what)


func _check_manifest(manifest: Dictionary) -> bool:
	_check(_view.probe_count() == 4 and _elements.size() == 4, "four probes, four manifest elements")
	_check(_connectors.size() == 2 and _view.connector_names().size() == 2, "two connectors, two manifest positions")
	_check(_stroke_m > 0.0 and absf(float(manifest["probe_stroke_m"]) - _stroke_m) == 0.0, "stroke %.2f m from the manifest" % _stroke_m)
	return true


## Stowed: every probe, highest point along the axis, below the hull radius + fairing height, and lower than extended by the stroke.
func _check_stowed(manifest: Dictionary) -> bool:
	_view.set_state(0.0, false)
	var limit := float(manifest["dorsal_hull_radius_m"]) + float(manifest["dorsal_fairing_height_m"])
	var stroke := float(manifest["probe_stroke_m"])
	for i in 4:
		var top := _view.probe_top_axial_m(i)
		_check(top <= limit, "probe %d stowed top %.3f m <= hull radius + fairing height %.3f m" % [i, top, limit])
		_check(absf(_elements[i].z - top - stroke) < TOLERANCE_M, "probe %d stowed %.3f m below the element (stroke %.2f m)" % [i, _elements[i].z - top, stroke])
	return true


func _check_extended() -> bool:
	_view.set_state(1.0, true)
	for i in 4:
		var tip := _view.probe_tip_m(i)
		_check(tip.distance_to(_elements[i]) < TOLERANCE_M, "probe %d extended tip %s on the element (%.4f m)" % [i, tip, tip.distance_to(_elements[i])])
	_check(_view.is_captured() and is_equal_approx(_view.extension(), 1.0), "state stored: extension 1, captured")
	_view.set_state(0.5, false)
	var half := _view.probe_tip_m(0)
	_check(absf(_elements[0].z - half.z - 0.5 * _stroke_m) < TOLERANCE_M, "half extension is half the stroke below the element")
	return true


func _check_connectors() -> bool:
	for node_name: String in _connectors:
		var face := _view.connector_face_m(node_name)
		_check(face.distance_to(_connectors[node_name]) < TOLERANCE_M, "%s face %s on the manifest position (%.4f m)" % [node_name, face, face.distance_to(_connectors[node_name])])
	return true


func _check_phase_mapping() -> bool:
	var none := {"present": false}
	_check(is_zero_approx(ProbeDrogueView.extension_for(none, 1.0)), "no docking state: stowed")
	_check(is_equal_approx(ProbeDrogueView.extension_for({"present": true, "proximity_phase": "Endanflug"}, 0.0), 1.0), "final approach: out")
	_check(is_zero_approx(ProbeDrogueView.extension_for({"present": true, "proximity_phase": "Anflug"}, 1.0)), "approach: stowed")
	_check(is_equal_approx(ProbeDrogueView.extension_for({"present": true, "proximity_phase": "Halt"}, 1.0), 1.0), "hold keeps what it finds")
	_check(is_zero_approx(ProbeDrogueView.extension_for({"present": true, "proximity_phase": "Halt"}, 0.0)), "hold keeps stowed")
	_check(is_equal_approx(ProbeDrogueView.extension_for({"present": true, "docking_phase": "Weichfang", "latched": true}, 0.0), 1.0), "soft capture: out")
	_check(ProbeDrogueView.captured_for({"present": true, "latched": true}) and not ProbeDrogueView.captured_for({"present": true, "latched": false}), "captured follows latched")
	_check(is_equal_approx(ProbeDrogueView.extension_for({"present": true, "docked": true}, 0.0), 1.0), "docked: out")
	return true


## The depot's four drogues: mouth plane centres on the manifest elements (= the depot config's), nothing moves.
func _check_depot_drogues() -> bool:
	var models := VehicleModels.load_for("starship_depot")
	if models.is_empty() or (models.get("dorsal", {}) as Dictionary).is_empty():
		_check(false, "starship_depot: dorsal models generated (%s)" % VehicleModels.fallback_reason("starship_depot"))
		return false
	var view := _dorsal_view(models)
	var elements: Array = models["manifest"]["dorsal_elements_m"]
	_check(view.drogue_count() == 4 and view.probe_count() == 0, "depot: four drogues, no probes")
	for i in view.drogue_count():
		var element := ProbeDrogueView._vec(elements[i])
		var mouth := view.drogue_mouth_m(i)
		_check(mouth.distance_to(element) < TOLERANCE_M, "drogue %d mouth %s on the config element (%.4f m)" % [i, mouth, mouth.distance_to(element)])
	view.set_state(1.0, true)
	_check(view.drogue_mouth_m(0).distance_to(ProbeDrogueView._vec(elements[0])) < TOLERANCE_M, "drogues do not move with the state")
	_check(view.connector_names().size() == 2, "depot: two connectors")
	view.free()
	return true


## The sim's docked pose, mirrored from core/src/vehicle/port_mating.cpp `mated_stage_transform` (used for the mated compound): R = P_target *
## diag(-1, -1, 1) * P_chaser^T with P = [axis, up x axis, up] as columns, t = target.position - R * chaser.position. Maps a point of the chaser's
## (tanker's) stage frame into the target's (depot's). Port data from the manifests, which python test_dorsal_hardware.py / test_depot_models.py pin to the configs.
static func mated_transform(chaser_port: Dictionary, target_port: Dictionary) -> Transform3D:
	var rotation := _port_axes(target_port) * Basis.from_scale(Vector3(-1.0, -1.0, 1.0)) * _port_axes(chaser_port).transposed()
	var chaser_position := ProbeDrogueView._vec(chaser_port["position_m"])
	return Transform3D(rotation, ProbeDrogueView._vec(target_port["position_m"]) - rotation * chaser_position)


static func _port_axes(port: Dictionary) -> Basis:
	var axis := ProbeDrogueView._vec(port["axis"])
	var up := ProbeDrogueView._vec(port["up"])
	return Basis(axis, up.cross(axis), up)  # columns


## Tanker docked on the depot: each extended probe tip inside its drogue's mouth cone, and no hull vertex of one ship inside the other's hull radius.
func _check_docked_pair() -> bool:
	var tanker := VehicleModels.load_for("starship_tanker")
	var depot := VehicleModels.load_for("starship_depot")
	if tanker.is_empty() or depot.is_empty() or (depot.get("dorsal", {}) as Dictionary).is_empty():
		_check(false, "docked pair: tanker and depot models generated")
		return false
	var tanker_view := _dorsal_view(tanker)
	var depot_view := _dorsal_view(depot)
	tanker_view.set_state(1.0, true)
	var pose := mated_transform(tanker["manifest"]["dorsal_port"], depot["manifest"]["dorsal_port"])
	var used := {}
	for i in tanker_view.probe_count():
		var tip := pose * tanker_view.probe_tip_m(i)
		var best := -1
		var best_d := INF
		for j in depot_view.drogue_count():
			var d := tip.distance_to(depot_view.drogue_mouth_m(j))
			if d < best_d:
				best = j
				best_d = d
		_check(best >= 0 and depot_view.inside_drogue_cone(best, tip, TOLERANCE_M), "probe %d tip %s inside the mouth cone of drogue %d (%.4f m from its mouth centre)" % [i, tip, best, best_d])
		_check(best_d < TOLERANCE_M and not used.has(best), "probe %d tip on the mouth centre of its own drogue" % i)
		used[best] = true
	var hull_radius_depot := float(depot["manifest"]["dorsal_hull_radius_m"])
	var hull_radius_tanker := float(tanker["manifest"]["dorsal_hull_radius_m"])
	var to_tanker := pose.affine_inverse()
	var tanker_hull := _vertices(tanker["hulls"][VehicleModels.ship_stage_id(tanker)]["hull"])
	var depot_hull := _vertices(depot["hulls"][VehicleModels.ship_stage_id(depot)]["hull"])
	var tanker_in_depot := _min_axis_distance(tanker_hull, pose)
	var depot_in_tanker := _min_axis_distance(depot_hull, to_tanker)
	_check(tanker_in_depot > hull_radius_depot, "tanker hull (%d vertices) nearest to the depot's axis %.3f m > hull radius %.3f m" % [tanker_hull.size(), tanker_in_depot, hull_radius_depot])
	_check(depot_in_tanker > hull_radius_tanker, "depot hull (%d vertices) nearest to the tanker's axis %.3f m > hull radius %.3f m" % [depot_hull.size(), depot_in_tanker, hull_radius_tanker])
	_check(_min_axis_distance(_dorsal_vertices(tanker), pose) > hull_radius_depot, "tanker dorsal hardware outside the depot's hull radius")
	_check(_min_axis_distance(_dorsal_vertices(depot), to_tanker) > hull_radius_tanker, "depot dorsal hardware outside the tanker's hull radius")
	var face := pose * tanker_view.connector_face_m("connector_lox")
	_check(face.distance_to(depot_view.connector_face_m("connector_lox")) < TOLERANCE_M, "LOX connector faces meet (%.4f m)" % face.distance_to(depot_view.connector_face_m("connector_lox")))
	tanker_view.free()
	depot_view.free()
	return true


## The HLS pair (tanker or depot on the HLS): the HLS gets its dorsal hardware in Task 6. Pending, not a pass.
func _check_hls_pair() -> bool:
	var hls := VehicleModels.load_for("starship_hls")
	if hls.is_empty() or (hls.get("dorsal", {}) as Dictionary).is_empty():
		print("pending (Task 6): HLS docked pair - starship_hls has no dorsal hardware yet")
		return true
	_check(false, "HLS has dorsal hardware: extend _check_docked_pair to the HLS pair (Task 6)")
	return true


func _dorsal_view(models: Dictionary) -> ProbeDrogueView:
	var view := ProbeDrogueView.new()
	root.add_child(view)
	view.build(models, func(_part: String) -> Material: return null)
	return view


func _vertices(parts_or_mesh: Variant) -> PackedVector3Array:
	var out := PackedVector3Array()
	var meshes: Array = []
	if parts_or_mesh is Mesh:
		meshes.append(parts_or_mesh)
	else:
		meshes.append_array((parts_or_mesh as Dictionary).values())
	for mesh: Mesh in meshes:
		for surface in mesh.get_surface_count():
			out.append_array(mesh.surface_get_arrays(surface)[Mesh.ARRAY_VERTEX])
	return out


## Every vertex of the ship's dorsal hardware in the ship's frame (node origin + mesh vertices: the nodes sit at their elements / faces).
func _dorsal_vertices(models: Dictionary) -> PackedVector3Array:
	var manifest: Dictionary = models["manifest"]
	var parts: Dictionary = models["dorsal"]
	var out := PackedVector3Array()
	var names: Array = manifest["dorsal_nodes"]
	for i in names.size():
		var origin := ProbeDrogueView._vec(manifest["dorsal_elements_m"][i])
		for v in _vertices(parts[names[i]]):
			out.append(v + origin)
	for node_name: String in manifest["dorsal_connectors"]:
		var origin := ProbeDrogueView._vec(manifest["dorsal_connectors"][node_name]["position_m"])
		for v in _vertices(parts[node_name]):
			out.append(v + origin)
	if parts.has("dock_fairing"):
		out.append_array(_vertices(parts["dock_fairing"]))  # fixed, in the ship's frame
	return out


## Smallest distance of the points (mapped by `through`) from the target ship's station axis (x): the hull radius test (the barrel is a cylinder about it).
func _min_axis_distance(points: PackedVector3Array, through: Transform3D) -> float:
	var best := INF
	for p in points:
		var q := through * p
		best = minf(best, Vector2(q.y, q.z).length())
	return best
