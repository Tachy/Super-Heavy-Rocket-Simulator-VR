# Headless test of the mission catalogue against the real config/.
# Run: godot --headless --path game --script res://tests/mission_catalogue_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	var catalogue := MissionCatalogue.load_from(Paths.config_dir())
	var place_ids := _ids(catalogue.places())
	for id in ["starbase_olp1", "ccsfs_slc40", "earth_orbit", "iss_docked", "lunar_orbit"]:
		_check(place_ids.has(id), "place %s listed" % id)
	_check(not place_ids.has("ccsfs_lz40"), "landing zone is not a place")
	_check(place_ids.find("starbase_olp1") < place_ids.find("earth_orbit"), "pads before start places")
	for place in catalogue.places():
		if place["id"] == "ccsfs_slc40":
			_check(not (place["site"] as Dictionary).is_empty(), "pad carries its site entry")
		if place["id"] == "earth_orbit":
			_check((place["site"] as Dictionary).is_empty(), "start place has no site")

	_check(_ids(catalogue.vehicles_at("ccsfs_slc40")) == ["falcon9_block5"], "SLC-40 offers only the Falcon 9")
	var orbit_vehicles := _ids(catalogue.vehicles_at("earth_orbit"))
	_check(orbit_vehicles.has("crew_dragon") and orbit_vehicles.has("starship_hls"), "orbit offers Dragon and HLS")
	_check(not orbit_vehicles.has("falcon9_block5"), "orbit does not offer the Falcon 9")
	for place_id in place_ids:
		for vehicle_id in _ids(catalogue.vehicles_at(place_id)):
			_check(not ["ejection_seat", "parachutist", "iss"].has(vehicle_id), "%s not at %s" % [vehicle_id, place_id])
	_check(catalogue.vehicles_at("ccsfs_slc40")[0]["name"] == "Medium Launcher", "vehicle display name")

	var slc := catalogue.missions_for("ccsfs_slc40", "falcon9_block5")
	var slc_ids := _ids(slc)
	_check(slc_ids[0] == MissionCatalogue.FREE_FLIGHT, "free flight first on a pad")
	for id in ["falcon9_rtls", "crs35_ascent", "falcon9_crew_dragon_iss"]:
		_check(slc_ids.has(id), "SLC-40 lists %s" % id)
	_check(slc[slc_ids.find("falcon9_rtls")]["name"] == "Medium Launcher: Landung am Startgelände", "display_name of RTLS")
	var lunar := catalogue.missions_for("lunar_orbit", "starship_block3")
	_check(lunar.size() == 4, "lunar orbit, Starship: 4 entries (%d)" % lunar.size())
	_check(not _ids(lunar).has(MissionCatalogue.FREE_FLIGHT), "no free flight in flight places")
	for place_id in place_ids:
		for vehicle_id in _ids(catalogue.vehicles_at(place_id)):
			var listed := _ids(catalogue.missions_for(place_id, vehicle_id))
			_check(not listed.has("ship_return_starbase") and not listed.has("leo_ascent"), "unlisted missions stay out")

	_check(MissionCatalogue.index_of([{"id": "a"}, {"id": "b"}], "b") == 1, "index_of found")
	_check(MissionCatalogue.index_of([], "x") == -1, "index_of absent")
	print("MISSION CATALOGUE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _ids(list: Array) -> Array:
	var ids: Array = []
	for entry in list:
		ids.append(entry["id"])
	return ids


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
