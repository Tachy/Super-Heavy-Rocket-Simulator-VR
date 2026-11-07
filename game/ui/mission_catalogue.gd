class_name MissionCatalogue
extends RefCounted
## What the mission selection panel offers: start places (launch pads, then flight start places), the vehicles at a
## place and the missions for a place + vehicle. Read from config/ only (launch_sites.toml `vehicles`,
## start_places.toml, missions/*.toml `[mission] start_place / start_vehicle`); the names are texts of the language
## tables (vehicle.<id>.display_name, mission.<id>.display_name, site.<id>.name, place.<id>.name) resolved when asked.

const FREE_FLIGHT := ""

var _places: Array[Dictionary] = []  # {id, name_key, name, site}
var _vehicle_ids := []  # vehicle file ids
var _pad_vehicles := {}  # pad id -> vehicle ids
var _missions: Array[Dictionary] = []  # {id, name_key, place, vehicle}
var _kinds := {}  # mission id -> [mission] kind ("" = ascent), of every mission file
var _stay_hours := {}  # mission id -> [surface] stay_hours of a lunar landing


static func load_from(config_dir: String) -> MissionCatalogue:
	var catalogue := MissionCatalogue.new()
	catalogue._load(config_dir)
	return catalogue


static func index_of(list: Array, id: String) -> int:
	for i in list.size():
		if list[i]["id"] == id:
			return i
	return -1


func places() -> Array[Dictionary]:
	for place in _places:
		place["name"] = I18n.text(place["name_key"])
	return _places


func vehicles_at(place_id: String) -> Array[Dictionary]:
	var ids: Array = []
	for vehicle_id in _pad_vehicles.get(place_id, []):
		ids.append(vehicle_id)
	for mission in _missions:
		if mission["place"] == place_id and not ids.has(mission["vehicle"]):
			ids.append(mission["vehicle"])
	ids.sort()
	var result: Array[Dictionary] = []
	for vehicle_id in ids:
		var known: bool = _vehicle_ids.has(vehicle_id)
		result.append({"id": vehicle_id, "name_key": "vehicle.%s.display_name" % vehicle_id if known else "",
				"name": I18n.text("vehicle.%s.display_name" % vehicle_id) if known else vehicle_id})
	return result


func missions_for(place_id: String, vehicle_id: String) -> Array[Dictionary]:
	var result: Array[Dictionary] = []
	if (_pad_vehicles.get(place_id, []) as Array).has(vehicle_id):
		result.append({"id": FREE_FLIGHT, "name": I18n.text("ui.catalogue.free_flight")})
	var matching: Array[Dictionary] = []
	for mission in _missions:
		if mission["place"] == place_id and mission["vehicle"] == vehicle_id:
			matching.append({"id": mission["id"], "name": I18n.text(mission["name_key"])})
	matching.sort_custom(func(a: Dictionary, b: Dictionary) -> bool: return a["name"] < b["name"])
	result.append_array(matching)
	return result


## The mission lands on the Moon at a free site (config/missions [mission] kind = "lunar_landing"): the panel offers the
## site / stay / window picker for it (LunarPicker).
func is_lunar_landing(mission_id: String) -> bool:
	return _kinds.get(mission_id, "") == "lunar_landing"


## The mission's default stay on the surface in hours ([surface] stay_hours), 0 when it has none.
func stay_hours(mission_id: String) -> float:
	return float(_stay_hours.get(mission_id, 0.0))


func _load(config_dir: String) -> void:
	for file_name in _sorted_files(config_dir.path_join("vehicles")):
		_vehicle_ids.append(file_name.get_basename())
	for file_name in _sorted_files(config_dir.path_join("missions")):
		var file: Dictionary = TomlLite.load_file(config_dir.path_join("missions").path_join(file_name))
		var mission: Dictionary = file.get("mission", {})
		_kinds[file_name.get_basename()] = mission.get("kind", "")
		if file.has("surface"):
			_stay_hours[file_name.get_basename()] = float((file["surface"] as Dictionary).get("stay_hours", 0.0))
		if not mission.has("start_place") or not mission.has("start_vehicle"):
			continue  # not offered by the panel
		_missions.append({
			"id": file_name.get_basename(),
			"name_key": "mission.%s.display_name" % file_name.get_basename(),
			"place": mission["start_place"],
			"vehicle": mission["start_vehicle"],
		})
	var pads: Array[Dictionary] = []
	for site: Dictionary in TomlLite.load_file(config_dir.path_join("launch_sites.toml")).get("site", []):
		if site.has("kind"):
			continue  # landing zones are no start places
		_pad_vehicles[site["id"]] = site.get("vehicles", [])
		pads.append({"id": site["id"], "name_key": "site.%s.name" % site["id"], "name": "", "site": site})
	var flight_places: Array[Dictionary] = []
	for place: Dictionary in TomlLite.load_file(config_dir.path_join("start_places.toml")).get("place", []):
		flight_places.append({"id": place["id"], "name_key": "place.%s.name" % place["id"], "name": "", "site": {}})
	for place in pads + flight_places:
		if not vehicles_at(place["id"]).is_empty():
			_places.append(place)


static func _sorted_files(dir_path: String) -> PackedStringArray:
	var files := PackedStringArray()
	for file_name in DirAccess.get_files_at(dir_path):
		if file_name.ends_with(".toml"):
			files.append(file_name)
	files.sort()
	return files
