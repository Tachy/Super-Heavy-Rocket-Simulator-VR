# The tanker's dorsal probes and connectors (ProbeDrogueView) against the vehicle config, and the snapshot -> extension mapping.
# Needs the generated models (python tools/structures/build_models.py --vehicle starship_tanker).
# godot --headless --path game --script res://tests/probe_drogue_test.gd
extends SceneTree

const TOLERANCE_M := 0.01  # Task 3 brief
# config/vehicles/starship_tanker.toml [[stage.docking_port]] elements_m / [[stage.transfer_connector]] position_m (python test_dorsal_hardware.py compares the manifest with the config)
const ELEMENTS_M := [Vector3(12.0, -1.5, 5.0), Vector3(12.0, 1.5, 5.0), Vector3(28.0, -1.5, 5.0), Vector3(28.0, 1.5, 5.0)]
const CONNECTORS_M := {"connector_lox": Vector3(8.0, 0.0, 5.0), "connector_ch4": Vector3(24.0, 0.0, 5.0)}

var _failures := 0
var _view: ProbeDrogueView


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
	_check(_view.probe_count() == 4, "four probes")
	for i in 4:
		var e: Array = manifest["dorsal_elements_m"][i]
		_check(Vector3(e[0], e[1], e[2]).is_equal_approx(ELEMENTS_M[i]), "manifest element %d is the config element" % i)
	return true


## Stowed: every probe, highest point along the axis, below the hull radius + fairing height, and lower than extended by the stroke.
func _check_stowed(manifest: Dictionary) -> bool:
	_view.set_state(0.0, false)
	var limit := float(manifest["dorsal_hull_radius_m"]) + float(manifest["dorsal_fairing_height_m"])
	var stroke := float(manifest["probe_stroke_m"])
	for i in 4:
		var top := _view.probe_top_axial_m(i)
		_check(top <= limit, "probe %d stowed top %.3f m <= hull radius + fairing height %.3f m" % [i, top, limit])
		_check(absf((ELEMENTS_M[i] as Vector3).z - top - stroke) < TOLERANCE_M, "probe %d stowed %.3f m below the element (stroke %.2f m)" % [i, (ELEMENTS_M[i] as Vector3).z - top, stroke])
	return true


func _check_extended() -> bool:
	_view.set_state(1.0, true)
	for i in 4:
		var tip := _view.probe_tip_m(i)
		_check(tip.distance_to(ELEMENTS_M[i]) < TOLERANCE_M, "probe %d extended tip %s on the element (%.4f m)" % [i, tip, tip.distance_to(ELEMENTS_M[i])])
	_check(_view.is_captured() and is_equal_approx(_view.extension(), 1.0), "state stored: extension 1, captured")
	_view.set_state(0.5, false)
	var half := _view.probe_tip_m(0)
	_check(absf((ELEMENTS_M[0] as Vector3).z - half.z - 0.5 * 0.30) < TOLERANCE_M, "half extension is half the stroke below the element")
	return true


func _check_connectors() -> bool:
	for node_name: String in CONNECTORS_M:
		var face := _view.connector_face_m(node_name)
		_check(face.distance_to(CONNECTORS_M[node_name]) < TOLERANCE_M, "%s face %s on the config position (%.4f m)" % [node_name, face, face.distance_to(CONNECTORS_M[node_name])])
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
