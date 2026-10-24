class_name MissionCatalogue
extends RefCounted
## What the mission selection panel offers: start places (launch pads, then flight start places), the vehicles at a
## place and the missions for a place + vehicle. Read from config/ only (launch_sites.toml `vehicles`,
## start_places.toml, missions/*.toml `[mission] start_place / start_vehicle`, vehicles/*.toml display names).

const FREE_FLIGHT := ""
const FREE_FLIGHT_NAME := "Freier Flug"

var _places: Array[Dictionary] = []  # {id, name, site}
var _vehicle_names := {}  # vehicle file id -> name
var _pad_vehicles := {}  # pad id -> vehicle ids
var _missions: Array[Dictionary] = []  # {id, name, place, vehicle}


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
		result.append({"id": vehicle_id, "name": _vehicle_names.get(vehicle_id, vehicle_id)})
	return result


func missions_for(place_id: String, vehicle_id: String) -> Array[Dictionary]:
	var result: Array[Dictionary] = []
	if (_pad_vehicles.get(place_id, []) as Array).has(vehicle_id):
		result.append({"id": FREE_FLIGHT, "name": FREE_FLIGHT_NAME})
	var matching: Array[Dictionary] = []
	for mission in _missions:
		if mission["place"] == place_id and mission["vehicle"] == vehicle_id:
			matching.append({"id": mission["id"], "name": mission["name"]})
	matching.sort_custom(func(a: Dictionary, b: Dictionary) -> bool: return a["name"] < b["name"])
	result.append_array(matching)
	return result


func _load(config_dir: String) -> void:
	for file_name in _sorted_files(config_dir.path_join("vehicles")):
		var vehicle: Dictionary = TomlLite.load_file(config_dir.path_join("vehicles").path_join(file_name)).get("vehicle", {})
		var id := file_name.get_basename()
		_vehicle_names[id] = vehicle.get("display_name", vehicle.get("name", id))
	for file_name in _sorted_files(config_dir.path_join("missions")):
		var mission: Dictionary = TomlLite.load_file(config_dir.path_join("missions").path_join(file_name)).get("mission", {})
		if not mission.has("start_place") or not mission.has("start_vehicle"):
			continue  # not offered by the panel
		_missions.append({
			"id": file_name.get_basename(),
			"name": mission.get("display_name", mission.get("name", file_name.get_basename())),
			"place": mission["start_place"],
			"vehicle": mission["start_vehicle"],
		})
	var pads: Array[Dictionary] = []
	for site: Dictionary in TomlLite.load_file(config_dir.path_join("launch_sites.toml")).get("site", []):
		if site.has("kind"):
			continue  # landing zones are no start places
		_pad_vehicles[site["id"]] = site.get("vehicles", [])
		pads.append({"id": site["id"], "name": site.get("name", site["id"]), "site": site})
	var flight_places: Array[Dictionary] = []
	for place: Dictionary in TomlLite.load_file(config_dir.path_join("start_places.toml")).get("place", []):
		flight_places.append({"id": place["id"], "name": place.get("name", place["id"]), "site": {}})
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
