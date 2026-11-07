# Headless test of the cockpit config loader and the mission crew count.
# Run: godot --headless --path game --script res://tests/cockpit_config_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	var cockpit := CockpitConfig.for_vehicle("crew_dragon")
	_check(cockpit != null, "crew_dragon has a cockpit")
	_check(CockpitConfig.for_vehicle("falcon9_block5+crew_dragon") != null, "stack id finds the cockpit")
	_check(CockpitConfig.for_vehicle("starship_block3") == null, "starship has none")
	if cockpit != null:
		_check(cockpit.id == "crew_dragon" and cockpit.capsule_stage_id == "capsule", "ids")
		_check(cockpit.seats.size() == 4, "four seats")
		var commanders := cockpit.seats.filter(func(s: Dictionary) -> bool: return s["role"] == "commander")
		_check(commanders.size() == 1 and commanders[0]["id"] == "s2", "commander is s2")
		for screen_id in ["left", "centre", "right"]:
			_check(cockpit.screens.has(screen_id), "screen %s" % screen_id)
		_check(cockpit.buttons.size() > 10 and cockpit.cameras.is_empty(), "buttons present, no cameras yet")
		_check(cockpit.tablets.size() == 2 and cockpit.lights.size() == 2 and cockpit.hotas_props_enabled, "extras")
		_check(cockpit.cabin.has("profile_m") and ((cockpit.cabin["profile_m"] as Array)[0] as Array).size() == 2, "nested profile")
		_check(absf((-cockpit.eye.basis.z).dot(Vector3(0.37956, 0.92517, 0.0)) - 1.0) < 1e-4, "eye looks along forward")  # round 4 [eye] forward
		var errors := cockpit.validate()
		_check(errors.is_empty(), "validate empty: %s" % ", ".join(errors))
		_check(cockpit.validate_crew(4).is_empty() and cockpit.validate_crew(1).is_empty(), "crew 1 and 4 valid")
		_check(not cockpit.validate_crew(5).is_empty() and not cockpit.validate_crew(0).is_empty(), "crew 5 and 0 invalid")
		var broken := CockpitConfig.from_dict({"seat": [{"id": "a", "role": "pilot"}]}, "x")
		_check(not broken.validate().is_empty(), "crafted broken table fails validate")
	_check(MissionCatalogue.crew_count("crew_dragon_return") == 4, "crew_dragon_return crew 4")
	_check(MissionCatalogue.crew_count("falcon9_crew_dragon") == 4, "falcon9_crew_dragon crew 4")
	_check(MissionCatalogue.crew_count("starship_leo") == 0, "starship_leo crew 0")
	print("COCKPIT CONFIG TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
