# The tanker's dorsal probes and connectors (ProbeDrogueView) against the vehicle config, and the snapshot -> extension mapping.
# Needs the generated models (python tools/structures/build_models.py --vehicle starship_tanker).
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
