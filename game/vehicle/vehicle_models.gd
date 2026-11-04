class_name VehicleModels
## Run-time loader of the generated vehicle models (tools/structures/build_models.py --vehicle <id> ->
## data/models/<id>/) and the pose maths that places them from the flight snapshot. Without a detail file, fresh
## models or a vehicle that uses them it returns {} and VehicleView keeps its cylinders (one log line per vehicle).
## The glTFs keep Blender axes = the config's BODY frame (X station axis, nose +X, belly azimuth 270 = -Z; manifest
## "frame"). The poses below are body coordinates relative to the stage's base; the stage node carries the one
## permutation to render axes (body_to_render, the cyclic permutation of Frames.to_render).

## The vehicles that have detail models (a named list, not a config key, like Pad B's). The others keep their cylinders.
## A vehicle on the list whose models are missing or stale also keeps them (one log line); the generators are listed in
## tools/structures/blender/generators.toml.
static var MODEL_VEHICLE_IDS := PackedStringArray(["starship_block3", "starship_tanker", "starship_depot", "starship_hls", "falcon9_block5", "crew_dragon", "iss"])
## Calls of load_for (tests: a view must not ask again every frame).
static var load_calls := 0
const MANIFEST := "manifest.json"
## Per vehicle: the glTF files the loader needs (no extension) and the sources whose change makes the set stale
## (tools/structures/blender/ scripts and tools/structures/ bakers); keep in step with generators.toml.
const MODEL_FILES := {
	"starship_block3": ["booster", "ship", "flap_fwd", "flap_aft", "grid_fin", "raptor", "rvac"],
	# ship variants (plan 2026-11-04): the depot draws no flaps, the HLS has no booster (generators.toml "outputs")
	"starship_tanker": ["booster", "ship", "flap_fwd", "flap_aft", "grid_fin", "raptor", "rvac", "dorsal_hardware"],
	"starship_depot": ["booster", "ship", "grid_fin", "raptor", "rvac"],
	"starship_hls": ["ship", "raptor", "rvac"],
	"falcon9_block5": ["falcon9_stage1", "falcon9_grid_fin", "falcon9_leg", "falcon9_stage2", "mvac", "merlin"],
	"crew_dragon": ["dragon_capsule", "dragon_nose", "dragon_trunk", "idss_mechanism"], "iss": ["iss_station", "iss_arrays", "iss_rings"],
}
const SOURCE_SCRIPTS := {
	"starship_block3": ["vehicle_common.py", "hull_profile.py", "gen_hull.py", "gen_flap.py", "gen_grid_fin.py", "gen_engine.py"],
	"starship_tanker": ["vehicle_common.py", "hull_profile.py", "dorsal_layout.py", "gen_hull.py", "gen_flap.py", "gen_grid_fin.py", "gen_engine.py", "gen_dorsal_hardware.py"],
	"starship_depot": ["vehicle_common.py", "hull_profile.py", "gen_hull.py", "gen_grid_fin.py", "gen_engine.py"],
	"starship_hls": ["vehicle_common.py", "hull_profile.py", "gen_hull.py", "gen_engine.py"],
	"falcon9_block5": ["vehicle_common.py", "gen_falcon9_hull.py", "gen_falcon9_grid_fin.py", "gen_falcon9_leg.py", "falcon9_second_stage.py", "gen_engine.py"],
	"crew_dragon": ["vehicle_common.py", "gen_dragon.py", "idss_mechanism.py"], "iss": ["vehicle_common.py", "idss_mechanism.py", "iss_station.py", "iss_arrays.py", "idss_passive_ring.py"],
}
const SOURCE_BAKERS := {"starship_block3": ["hull_textures.py", "hull_tiles.py"], "starship_tanker": ["hull_textures.py", "hull_tiles.py"],
		"starship_depot": ["hull_textures.py", "hull_tiles.py"], "starship_hls": ["hull_textures.py", "hull_tiles.py"], "falcon9_block5": ["surface_textures.py"], "crew_dragon": ["surface_textures.py"], "iss": ["iss_textures.py"]}
## Ship variants take the detail tables they do not list, and the stages they take over, from Block 3: its files also make
## their model set stale (tools/structures/vehicle_spec.py, VARIANT_BASE).
const VARIANT_VEHICLES: Array[String] = ["starship_tanker", "starship_depot", "starship_hls"]
const VARIANT_BASE := "starship_block3"
## Which vehicle's model set draws a stage: a stage a vehicle takes over unchanged is drawn by the vehicle it comes from
## (config `[[stage]] from = "<vehicle>:<stage>"`), the others by the vehicle itself.
const MODEL_SET_OF_STAGE := {
	"starship_tanker": {"booster": "starship_block3"},
	"starship_hls_launch": {"booster": "starship_block3", "hls": "starship_hls"},
}


## The vehicle whose model set draws `stage_id` of `vehicle_id` (the snapshot's: "a", or "a/<stage>" after a separation,
## the part before "/" counts). The snapshot reports the composite vehicle ("starship_tanker"), not the source of a stage.
static func model_set_for_stage(vehicle_id: String, stage_id: String) -> String:
	var vehicle := vehicle_id.get_slice("/", 0)
	return String((MODEL_SET_OF_STAGE.get(vehicle, {}) as Dictionary).get(stage_id, vehicle))


## Engine kinds: the snapshot's "vacuum" flag selects the model and the manifest's engine id.
const ENGINE_FILE := {false: "raptor", true: "rvac"}
const ENGINE_ID := {false: "raptor3", true: "raptor_vacuum3"}
## Nozzle exit below the gimbal point when no model says better (the former view's value).
const DEFAULT_EXIT_DEPTH_M := 3.0
const HULL_SHADER := preload("res://shaders/vehicle_hull.gdshader")
const MAPPED_SHADER := preload("res://shaders/lit_aerial_mapped.gdshader")
const PAINTED_SHADER := preload("res://shaders/vehicle_painted.gdshader")
## "booster" = the booster hull: mode 0 with the baked steel map (the ship's hull carries it in mode 1).
const HULL_MODE := {"steel": 0, "booster": 0, "ship": 1, "flap": 2, "fin_frame": 0, "fin_cells": 0, "fin_band": 0, "dark": 0}
## [E] Sunlit frames (NSF photo 2026-03, aerial 2026-04, f12 file 1804 s): the fin rim reads dark grey, the cells black. Albedo
## linear from the sRGB of the sunlit rim (~75 / 255) against the bright hull: 0.07 +-0.03; cells 0.025 +-0.01.
const DARK_PARTS := {
	"fin_frame": {"albedo": Color(0.29, 0.29, 0.30), "roughness": 0.7, "metallic": 0.2, "mottle": 0.18, "scale": 1.3},
	"fin_cells": {"albedo": Color(0.17, 0.17, 0.175), "roughness": 0.8, "metallic": 0.1, "mottle": 0.1, "scale": 1.3},
	## Light flat rim face and catch bar: the installed-fin photo (user 2026-10-31) shows a white band along the upper edge.
	"fin_band": {"albedo": Color(0.52, 0.52, 0.54), "roughness": 0.45, "metallic": 0.5, "mottle": 0.05, "scale": 1.3},
	"dark": {"albedo": Color(0.2, 0.2, 0.21), "roughness": 0.6, "metallic": 0.2, "mottle": 0.0, "scale": 0.12},
}
## Beyond this distance a fin draws its opaque `plate` instead of the walls and spikes (wall 4 cm = 1 px of the Pimax at
## about 100 m; the walls alias against the sky). Rim, hooks and shaft stay.
const FIN_LATTICE_RANGE_M := 120.0
const ENGINE_SHADER := preload("res://shaders/vehicle_engine.gdshader")
const RIB_FILE_SIZE := 32
## [E] Bells in sky light (f14 file 2563 s, booster, soot free after the boostback) and hot-staging light (f14 file 2515 s): sea-level
## bells near black, albedo 0.05 +-0.03; RVac grey metal with flutes (0.25 +-0.1): the RVac is NOT black in those frames.
const ENGINE_LOOK := {
	false: {"albedo": Color(0.25, 0.25, 0.26), "roughness": 0.55, "metallic": 0.3, "rib_slope": 0.25, "rib_top_x_m": -0.75},
	true: {"albedo": Color(0.54, 0.54, 0.55), "roughness": 0.5, "metallic": 0.7, "rib_slope": 0.4, "rib_top_x_m": -1.74},
}
## Kinds whose surface is the hull's UV (arc metres, station metres) and so can carry the steel map.
const STEEL_MAP_KINDS: Array[String] = ["booster", "ship"]
## Entries of the shader's coverage table arrays.
const HULL_TABLE_SIZE := 8
const AZIMUTH_TOLERANCE_DEG := 0.5
## Painted parts have no rings: a pitch longer than any stage keeps the shader's per-ring tint to one panel.
const PAINT_RING_PITCH_M := 1000.0
const STATION_TOLERANCE_M := 0.01

static var _logged: Dictionary = {}
## Vehicle -> why the cylinders are shown: "stale" | "missing" | "none" (no models for this vehicle, by design); absent: models loaded.
static var _reasons: Dictionary = {}
static var _cache: Dictionary = {}
## Reason of the last failed glTF load (set by _meshes_of), for the fallback log line.
static var _load_error := ""
## Tests: a scratch repo folder (data/, config/ inside) instead of the real one; clear_cache resets it.
static var root_override := ""
## Number of log lines this class printed (tests assert "exactly one line" per fallback).
static var log_count := 0


## {} or {"manifest": Dictionary, "hulls": {stage_id: {node: Mesh}}, "flap": {"fwd"|"aft": {node: Mesh}},
## "grid_fin": {node: Mesh}, "engine": {false|true: Mesh}, "materials": {"steel"|"booster"|"ship"|"flap": ShaderMaterial}} (the
## engine meshes merged to one multi-surface mesh).
static func load_for(vehicle_id: String) -> Dictionary:
	load_calls += 1
	if not MODEL_VEHICLE_IDS.has(vehicle_id):
		_reasons[vehicle_id] = "none"
		if vehicle_id != "" and not _logged.has(vehicle_id):
			_logged[vehicle_id] = true
			_say("VehicleModels: %s has no models (cylinders by design)" % vehicle_id)
		return {}
	if not _cache.has(vehicle_id):
		_cache[vehicle_id] = _load(vehicle_id)  # a failure is cached too: the view asks again whenever it rebuilds
	return _cache[vehicle_id]


## Drops the cached results (tests that regenerate the models).
static func clear_cache() -> void:
	_cache.clear()
	_logged.clear()
	_reasons.clear()
	root_override = ""


static func _say(line: String) -> void:
	log_count += 1
	print(line)


static func _repo_dir() -> String:
	return root_override if root_override != "" else Paths.repo_dir()


static func _load(vehicle_id: String) -> Dictionary:
	var model_dir := _repo_dir().path_join("data/models").path_join(vehicle_id)
	var missing := _missing_or_stale(vehicle_id, model_dir)
	if missing != "":
		return _fallback(vehicle_id, missing)
	if (MODEL_FILES[vehicle_id] as Array).is_empty():  # fresh manifest, but this vehicle's generators do not exist yet
		return _fallback(vehicle_id, "no model files defined for this vehicle yet")
	var manifest: Variant = JSON.parse_string(FileAccess.get_file_as_string(model_dir.path_join(MANIFEST)))
	if not (manifest is Dictionary and manifest.has("flaps") and manifest.has("engines")):
		return _fallback(vehicle_id, "manifest unreadable")
	var parts := {}
	for file: String in MODEL_FILES[vehicle_id]:
		parts[file] = _meshes_of(model_dir.path_join(file + ".gltf"))
		if (parts[file] as Dictionary).is_empty():
			return _fallback(vehicle_id, _load_error)
	if vehicle_id == "falcon9_block5":
		return {
			"manifest": manifest,
			"hulls": {"first_stage": parts["falcon9_stage1"], "second_stage": parts["falcon9_stage2"]},
			"flap": {},
			"grid_fin": parts["falcon9_grid_fin"],
			"leg": parts["falcon9_leg"],
			"engine": {false: _merged(parts["merlin"]), true: _merged(parts["mvac"])},
			"materials": _hull_materials(manifest.get("look", {}), model_dir, vehicle_id),
		}
	if vehicle_id == "iss":
		var fixed: Dictionary = parts["iss_station"].duplicate()
		fixed.merge(parts["iss_rings"])
		return {
			"manifest": manifest,
			"hulls": {"station": fixed},
			"jointed": parts["iss_arrays"],  # relative to their joints' origins (manifest "joints"): VehicleView._build_joints
			"flap": {}, "grid_fin": {}, "engine": {},
			"materials": _hull_materials(manifest.get("look", {}), model_dir, vehicle_id),
		}
	if vehicle_id == "crew_dragon":
		return {
			"manifest": manifest,
			"hulls": {"trunk": parts["dragon_trunk"], "capsule": parts["dragon_capsule"]},
			"nose": parts["dragon_nose"],  # origin = the hinge point, closed = identity (nose_transform)
			"mechanism": parts["idss_mechanism"],  # DockingMechanismView: frames in idss_mechanism.py
			"flap": {}, "grid_fin": {}, "engine": {},
			"materials": _hull_materials(manifest.get("look", {}), model_dir, vehicle_id),
		}
	for engine_file: String in ["raptor", "rvac"]:
		parts[engine_file] = _merged(parts[engine_file])
	var hulls := {}
	var stage_ids: Dictionary = manifest.get("look", {}).get("stage_ids", {"booster": "booster", "ship": "ship"})
	for role: String in ["booster", "ship"]:
		if parts.has(role):  # a variant names its ship stage (depot, hls) and may have no booster
			hulls[String(stage_ids.get(role, role))] = parts[role]
	return {
		"manifest": manifest,
		"hulls": hulls,
		"flap": {"fwd": parts["flap_fwd"], "aft": parts["flap_aft"]} if parts.has("flap_fwd") else {},
		"grid_fin": parts.get("grid_fin", {}),
		"engine": {false: parts["raptor"], true: parts["rvac"]},
		"dorsal": parts.get("dorsal_hardware", {}),  # ProbeDrogueView: frames in gen_dorsal_hardware.py; {} on a ship with none
		"materials": _hull_materials(manifest.get("look", {}), model_dir, vehicle_id),
	}


## Hull shader materials (shaders/vehicle_hull.gdshader) sharing the manifest's "look" numbers (ring pitch, tile size and
## gap, the coverage table around the belly): "steel" (bare), "ship" (steel + tiles in the coverage), "flap" (tiles on the
## outer face). Opaque, like every vehicle surface.
static func _hull_materials(look: Dictionary, model_dir: String, vehicle_id: String) -> Dictionary:
	var materials := {}
	var no_maps: bool = look.get("no_maps", false)  # a vehicle drawn in plain paint: no baked maps, no log line about them
	var steel_map: ImageTexture = null if no_maps else _steel_map(look.get("steel_map", {}), model_dir, vehicle_id)
	var no_tiles: bool = not (look.get("variant", {}) as Dictionary).get("tiles", true)  # a variant without a heat shield
	var tile_maps: Dictionary = {} if no_maps or no_tiles else _tile_maps(look.get("tile_maps", {}), model_dir, vehicle_id)
	for key: String in ["steel", "booster", "ship", "flap", "fin_frame", "fin_cells", "fin_band", "dark"]:
		var material := ShaderMaterial.new()
		material.shader = HULL_SHADER
		material.set_shader_parameter("mode", HULL_MODE[key])
		material.set_shader_parameter("ring_pitch_m", float(look.get("ring_pitch_m", 1.8)))
		material.set_shader_parameter("barrel_end_m", float(look.get("ship_barrel_end_m", 1.0e9)))
		material.set_shader_parameter("weld_first_m", float(look.get("weld_first_m", 0.35)))
		if steel_map != null and STEEL_MAP_KINDS.has(key):
			var info: Dictionary = look["steel_map"]
			material.set_shader_parameter("steel_textured", true)
			material.set_shader_parameter("steel_map", steel_map)
			material.set_shader_parameter("steel_period_m", Vector2(info["period_u_m"], info["period_v_m"]))
			material.set_shader_parameter("steel_slope_range", float(info["slope_range"]))
			material.set_shader_parameter("steel_energy_range", float(info["slope_energy_range"]))
			material.set_shader_parameter("steel_shift_steps", float(info.get("shift_steps", 0)))
		if key == "flap" and steel_map != null:  # the panel lines of the lee face (same channels, no ring shift, no weld gate)
			var info: Dictionary = look["steel_map"]
			var flap_map := _steel_map({"file": info.get("flap_file", "")}, model_dir, vehicle_id)
			if flap_map != null:
				material.set_shader_parameter("steel_textured", true)
				material.set_shader_parameter("steel_map", flap_map)
				material.set_shader_parameter("steel_period_m", Vector2(info["flap_period_m"][0], info["flap_period_m"][1]))
				material.set_shader_parameter("steel_slope_range", float(info["slope_range"]))
				material.set_shader_parameter("steel_energy_range", float(info["slope_energy_range"]))
				material.set_shader_parameter("weld_first_m", -1.0e6)
		if DARK_PARTS.has(key):
			var dark: Dictionary = DARK_PARTS[key]
			material.set_shader_parameter("steel_albedo", dark["albedo"])
			material.set_shader_parameter("steel_roughness", dark["roughness"])
			material.set_shader_parameter("steel_metallic", dark["metallic"])
			material.set_shader_parameter("steel_mottle", dark["mottle"])
			material.set_shader_parameter("mottle_scale", dark["scale"])
		var tiles: Dictionary = look.get("tiles", {})
		if not tiles.is_empty():
			var stations := PackedFloat32Array(tiles["coverage_station_m"])
			var half_angles := PackedFloat32Array()
			for degrees: float in tiles["coverage_half_angle_deg"]:
				half_angles.append(deg_to_rad(degrees))
			var count := mini(stations.size(), HULL_TABLE_SIZE)
			stations.resize(HULL_TABLE_SIZE)
			half_angles.resize(HULL_TABLE_SIZE)
			material.set_shader_parameter("tile_across_flats_m", float(tiles["hex_across_flats_m"]))
			if not tile_maps.is_empty():
				var maps_info: Dictionary = look["tile_maps"]
				material.set_shader_parameter("tiles_textured", true)
				material.set_shader_parameter("tile_patch", tile_maps["patch"])
				material.set_shader_parameter("tile_sel", tile_maps["sel"])
				material.set_shader_parameter("tile_ids", tile_maps["ids"])
				material.set_shader_parameter("tile_period_m", Vector2(maps_info["period_m"][0], maps_info["period_m"][1]))
				material.set_shader_parameter("tile_ids_size", Vector2(maps_info["ids_size"][0], maps_info["ids_size"][1]))
				material.set_shader_parameter("tile_tilt_energy_range", float(maps_info["tilt_energy_range"]))
			material.set_shader_parameter("coverage_count", count)
			material.set_shader_parameter("coverage_station_m", stations)
			material.set_shader_parameter("coverage_half_angle_rad", half_angles)
			material.set_shader_parameter("belly_azimuth_rad",
					deg_to_rad(float(look.get("ship_belly_azimuth_deg", 270.0))))
		materials[key] = material
	for key: String in look.get("paint", {}):  # plain painted parts (manifest): the hull shader's bare mode, no ring tint
		var paint: Dictionary = look["paint"][key]
		var painted := ShaderMaterial.new()
		painted.shader = HULL_SHADER
		var albedo: Array = paint["albedo"]
		painted.set_shader_parameter("mode", 0)
		painted.set_shader_parameter("ring_pitch_m", PAINT_RING_PITCH_M)
		painted.set_shader_parameter("steel_albedo", Color(albedo[0], albedo[1], albedo[2]))
		painted.set_shader_parameter("steel_roughness", float(paint["roughness"]))
		painted.set_shader_parameter("steel_metallic", float(paint["metallic"]))
		painted.set_shader_parameter("steel_mottle", 0.0)
		materials[key] = painted
	var textured: Dictionary = look.get("textured", {})
	for key: String in textured:  # a painted part with a baked albedo map (the ISS's cells, panel lines): lit_aerial_mapped, the paint's numbers
		var paint: Dictionary = look["paint"][key]
		var image := Image.load_from_file(model_dir.path_join(String(textured[key]["file"])))
		if image == null:
			continue
		image.convert(Image.FORMAT_RGBA8)
		image.generate_mipmaps()
		var mapped := ShaderMaterial.new()
		mapped.shader = MAPPED_SHADER
		var albedo: Array = paint["albedo"]
		mapped.set_shader_parameter("albedo", Color(albedo[0], albedo[1], albedo[2]))
		mapped.set_shader_parameter("roughness", float(paint["roughness"]))
		mapped.set_shader_parameter("metallic", float(paint["metallic"]))
		mapped.set_shader_parameter("albedo_map", ImageTexture.create_from_image(image))
		mapped.set_shader_parameter("map_period_m", Vector2(textured[key]["period_m"][0], textured[key]["period_m"][1]))
		materials[key] = mapped
	var surface: Dictionary = look.get("surface", {})
	var loaded := {}  # file -> texture: parts sharing a map share its memory
	for key: String in surface:  # a painted part with its baked surface (Falcon 9, Crew Dragon); a missing file keeps the plain paint, one log line
		var entry: Dictionary = surface[key]
		var detail: ImageTexture = _surface_map(String(entry["file"]), model_dir, Image.FORMAT_RGBA8, loaded)
		if detail == null:
			_say("VehicleModels: %s: surface map %s missing, plain paint" % [vehicle_id, entry["file"]])
			continue
		var paint: Dictionary = look["paint"][key]
		var albedo: Array = paint["albedo"]
		var finished := ShaderMaterial.new()
		finished.shader = PAINTED_SHADER
		finished.set_shader_parameter("albedo", Color(albedo[0], albedo[1], albedo[2]))
		finished.set_shader_parameter("roughness", float(paint["roughness"]))
		finished.set_shader_parameter("metallic", float(paint["metallic"]))
		finished.set_shader_parameter("detail_map", detail)
		finished.set_shader_parameter("detail_period_m", Vector2(entry["period_m"][0], entry["period_m"][1]))
		finished.set_shader_parameter("detail_gain", float(entry["gain"]))
		if entry.has("wear_file"):
			var wear: ImageTexture = _surface_map(String(entry["wear_file"]), model_dir, Image.FORMAT_R8, loaded)
			if wear != null:
				var soot: Array = entry["wear_albedo"]
				finished.set_shader_parameter("wear_map", wear)
				finished.set_shader_parameter("wear_period_m", Vector2(entry["wear_period_m"][0], entry["wear_period_m"][1]))
				finished.set_shader_parameter("wear_albedo", Vector3(soot[0], soot[1], soot[2]))
				finished.set_shader_parameter("wear_roughness", float(entry["wear_roughness"]))
		materials[key] = finished
	return materials


## One baked surface map with its mip chain (box averages of linear data); null without the file.
static func _surface_map(file: String, model_dir: String, format: Image.Format, loaded: Dictionary) -> ImageTexture:
	if loaded.has(file):
		return loaded[file]
	var path := model_dir.path_join(file)
	if not FileAccess.file_exists(path):
		return null
	var image := Image.load_from_file(path)
	if image == null or image.is_empty():
		return null
	image.convert(format)
	image.generate_mipmaps()
	var texture := ImageTexture.create_from_image(image)
	loaded[file] = texture
	return texture


## The baked steel map (tools/structures/hull_textures.py) with a full mip chain made here: a plain box average of
## slope, slope energy and weld coverage, which is what the map's channels are designed for. null without the file
## (plain steel, one log line).
## Bell material of one engine kind: ribs map (one rib, mipmapped, baked here: a plain 1D profile), dark per ENGINE_LOOK.
static func engine_material(vacuum: bool, models: Dictionary) -> ShaderMaterial:
	var look: Dictionary = ENGINE_LOOK[vacuum]
	var override: Dictionary = models["manifest"].get("look", {}).get("engine_look", {}).get("vacuum" if vacuum else "sea_level", {})
	if not override.is_empty():  # a vehicle with its own engine look (the Falcon 9's Merlin Vacuum): the manifest's numbers
		var a: Array = override["albedo"]
		look = {"albedo": Color(a[0], a[1], a[2]), "roughness": override["roughness"], "metallic": override["metallic"],
				"rib_slope": override["rib_slope"], "rib_top_x_m": override["rib_top_x_m"]}
	var m := ShaderMaterial.new()
	m.shader = ENGINE_SHADER
	m.set_shader_parameter("albedo", look["albedo"])
	m.set_shader_parameter("base_roughness", look["roughness"])
	m.set_shader_parameter("metallic_value", look["metallic"])
	m.set_shader_parameter("rib_slope", look["rib_slope"])
	var manifest_look: Dictionary = models["manifest"].get("look", {})
	var counts: Dictionary = manifest_look.get("rib_counts", {})
	var engine: Dictionary = models["manifest"]["engines"].get(engine_id(models, vacuum), {})
	var depth := -float(engine.get("exit_plane_x_m", -DEFAULT_EXIT_DEPTH_M))
	m.set_shader_parameter("rib_count", float(counts.get(ENGINE_FILE[vacuum], 0.0)))
	m.set_shader_parameter("exit_x_m", -depth)
	m.set_shader_parameter("rib_top_x_m", float(look["rib_top_x_m"]))  # Raptor: the whole bell; RVac: the lower 62 % (f14 2515 s)
	m.set_shader_parameter("rib_map", _rib_texture())
	return m


## One rib across the texture: R the slope along the circumference, G its energy; the box mips average the slope to zero
## and keep the energy, so a far bell only gets rougher.
static func _rib_texture() -> ImageTexture:
	var data := PackedByteArray()
	for i in RIB_FILE_SIZE:
		var slope := cos(TAU * (i + 0.5) / RIB_FILE_SIZE)
		data.append(int(round((0.5 + 0.5 * slope) * 255.0)))
		data.append(int(round(slope * slope * 255.0)))
	var image := Image.create_from_data(RIB_FILE_SIZE, 1, false, Image.FORMAT_RG8, data)
	image.generate_mipmaps()
	return ImageTexture.create_from_image(image)


static func _steel_map(info: Dictionary, model_dir: String, vehicle_id: String) -> ImageTexture:
	var path := model_dir.path_join(String(info.get("file", "hull_steel.png")))
	var image := Image.load_from_file(path) if not info.is_empty() and FileAccess.file_exists(path) else null
	if image == null:
		if not _logged.has(vehicle_id + "/steel_map"):
			_logged[vehicle_id + "/steel_map"] = true
			_say("VehicleModels: %s has no steel map (%s): plain steel" % [vehicle_id, path.get_file()])
		return null
	image.convert(Image.FORMAT_RGBA8)
	image.generate_mipmaps()
	return ImageTexture.create_from_image(image)


## The baked tile maps (tools/structures/hull_tiles.py): the patch with a full mip chain made here (box averages, which its
## channels are designed for), the selector and id maps as they are (nearest sampled, no mips: ids cannot be averaged).
## {} when a file is missing (plain dark tiles without gaps, one log line).
static func _tile_maps(info: Dictionary, model_dir: String, vehicle_id: String) -> Dictionary:
	var maps := {}
	for key: String in ["patch", "sel", "ids"]:
		var path := model_dir.path_join(String(info.get("files", {}).get(key, "")))
		var image := Image.load_from_file(path) if not info.is_empty() and FileAccess.file_exists(path) else null
		if image == null:
			if not _logged.has(vehicle_id + "/tile_maps"):
				_logged[vehicle_id + "/tile_maps"] = true
				_say("VehicleModels: %s has no tile maps (%s): plain dark tiles" % [vehicle_id, path.get_file()])
			return {}
		image.convert(Image.FORMAT_RGBA8)
		if key == "patch":
			image.generate_mipmaps()
		maps[key] = ImageTexture.create_from_image(image)
	return maps


## "stale" | "missing" | "none" while the vehicle shows cylinders, else "".
static func fallback_reason(vehicle_id: String) -> String:
	return String(_reasons.get(vehicle_id, ""))


## German notice for the view while the cylinders stand for stale or missing models, else "" (also for "none": by design).
static func notice(vehicle_id: String) -> String:
	return ModelNotice.text_for(fallback_reason(vehicle_id))


static func _fallback(vehicle_id: String, reason: String) -> Dictionary:
	_reasons[vehicle_id] = "stale" if reason.begins_with("models older") else "missing"
	if not _logged.has(vehicle_id):
		_logged[vehicle_id] = true
		_say("VehicleModels: %s keeps its cylinders (%s; run python tools/structures/build_models.py --vehicle %s)"
				% [vehicle_id, reason, vehicle_id])
	return {}


## "" when manifest and every glTF exist and are not older than the sources; otherwise the reason.
static func _missing_or_stale(vehicle_id: String, model_dir: String) -> String:
	var manifest := model_dir.path_join(MANIFEST)
	if not FileAccess.file_exists(manifest):
		return "no models in " + model_dir
	var built := FileAccess.get_modified_time(manifest)
	var sources: Array[String] = [_repo_dir().path_join("config/vehicles/detail/%s.toml" % vehicle_id),
			_repo_dir().path_join("config/vehicles/%s.toml" % vehicle_id)]
	if vehicle_id == "iss":  # the station's rings are drawn from Dragon's [idss]
		sources.append(_repo_dir().path_join("config/vehicles/detail/crew_dragon.toml"))
	if VARIANT_VEHICLES.has(vehicle_id):  # the rest of the detail tables and the taken-over stages
		sources.append(_repo_dir().path_join("config/vehicles/detail/%s.toml" % VARIANT_BASE))
		sources.append(_repo_dir().path_join("config/vehicles/%s.toml" % VARIANT_BASE))
	for script: String in SOURCE_SCRIPTS[vehicle_id]:
		sources.append(_repo_dir().path_join("tools/structures/blender").path_join(script))
	for baker: String in SOURCE_BAKERS[vehicle_id]:
		sources.append(_repo_dir().path_join("tools/structures").path_join(baker))
	for path in sources:
		if FileAccess.file_exists(path) and FileAccess.get_modified_time(path) > built:
			return "models older than " + path.get_file()
	for file: String in MODEL_FILES[vehicle_id]:
		for extension in [".gltf", ".bin"]:
			if not FileAccess.file_exists(model_dir.path_join(file + extension)):
				return "model file %s missing" % (file + extension)
	return ""


## Meshes of a glTF by node name (in the file's frame: the nodes carry identity transforms); the scene is freed.
static func _meshes_of(path: String) -> Dictionary:
	var meshes := {}
	var document := GLTFDocument.new()
	var state := GLTFState.new()
	var err := document.append_from_file(path, state)
	if err != OK:
		_load_error = "%s: glTF load failed (%s)" % [path.get_file(), error_string(err)]
		return meshes
	var scene := document.generate_scene(state)
	if scene == null:
		_load_error = "%s: glTF scene could not be generated" % path.get_file()
		return meshes
	for node in scene.find_children("*", "MeshInstance3D", true, false):
		var mesh_node := node as MeshInstance3D
		meshes[String(mesh_node.name)] = mesh_node.mesh
	scene.free()
	if meshes.is_empty():
		_load_error = "%s: no mesh nodes" % path.get_file()
	return meshes


## One multi-surface mesh from several (a MultiMesh draws one mesh); the engine nodes sit at the origin.
static func _merged(parts: Dictionary) -> ArrayMesh:
	var merged := ArrayMesh.new()
	for mesh: Mesh in parts.values():
		for surface in mesh.get_surface_count():
			merged.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, mesh.surface_get_arrays(surface))
	return merged


# ------------------------------------------------------------------------------------------------ pose maths

## Body (x, y, z) -> render (y, z, x), as Frames.to_render, as a basis (the stage node's basis).
static func body_to_render() -> Basis:
	return Basis(Frames.to_render(Vector3.RIGHT), Frames.to_render(Vector3.UP), Frames.to_render(Vector3.BACK))


## Engine model (origin = gimbal point, thrust +X) in stage-local body coordinates; the snapshot's thrust direction
## carries the real gimbal (nothing is faked here).
## `axial_offset_m`: the visual offset along the station axis (see axial_offset_m below); the gimbal direction stays
## the snapshot's.
static func engine_transform(position_m: Vector3, direction: Vector3, base_station_m: float,
		axial_offset_m := 0.0) -> Transform3D:
	var axis := direction.normalized() if direction.length() > 0.5 else Vector3.RIGHT
	return Transform3D(Basis(Quaternion(Vector3.RIGHT, axis)),
			position_m + Vector3(axial_offset_m - base_station_m, 0.0, 0.0))


## How far up the station axis an engine is DRAWN from the physics gimbal point: the ship's engines hang inside the hull
## (manifest, per engine kind; the physics keeps its gimbal at the ship's station 0: docs "Stage interface"). Applied in
## one place for the bell pose and the plume. 0 for the booster and without models.
static func axial_offset_m(models: Dictionary, stage_id: String, vacuum: bool) -> float:
	if models.is_empty():
		return 0.0
	var entry: Dictionary = models["manifest"]["engines"].get(engine_id(models, vacuum), {})
	if entry.has("axial_offset_m_by_stage"):
		return float(entry["axial_offset_m_by_stage"].get(stage_id, 0.0))
	if stage_id != ship_stage_id(models) or not entry.has("ship_axial_offset_m"):
		return 0.0
	return float(entry["ship_axial_offset_m"])


## The id of the stage drawn as the Starship's ship hull: "ship", for a variant the manifest's (depot, hls).
static func ship_stage_id(models: Dictionary) -> String:
	if models.is_empty():
		return "ship"
	return String(models["manifest"].get("look", {}).get("stage_ids", {}).get("ship", "ship"))


## Exhaust start below the gimbal point along the thrust axis: the model's exit plane, else the former 3 m.
static func exit_depth_m(models: Dictionary, vacuum: bool) -> float:
	if models.is_empty():
		return DEFAULT_EXIT_DEPTH_M
	var entry: Dictionary = models["manifest"]["engines"].get(engine_id(models, vacuum), {})
	return -float(entry["exit_plane_x_m"]) if entry.has("exit_plane_x_m") else DEFAULT_EXIT_DEPTH_M


## The manifest's engine id of a snapshot engine kind: its own table (manifest look.engine_ids), else the Starship's. A kind
## with no drawn model has no manifest entry: callers fall back to the former values.
static func engine_id(models: Dictionary, vacuum: bool) -> String:
	var ids: Dictionary = models["manifest"].get("look", {}).get("engine_ids", {})
	if ids.is_empty():
		return ENGINE_ID[vacuum]
	return String(ids.get("vacuum" if vacuum else "sea_level", ""))


## The manifest entry of a snapshot flap (same stage id, hinge azimuth and aft station relative to the stage), or {}.
static func flap_entry(models: Dictionary, stage_id: String, flap: Dictionary, base_station_m: float) -> Dictionary:
	for entry: Dictionary in models["manifest"]["flaps"].get(stage_id, []):
		if absf(fposmod(float(entry["hinge_azimuth_deg"]) - float(flap["hinge_azimuth_deg"]) + 180.0, 360.0)
				- 180.0) < AZIMUTH_TOLERANCE_DEG \
				and absf(float(entry["station_aft_m"]) - (float(flap["station_aft_m"]) - base_station_m)) \
				< STATION_TOLERANCE_M:
			return entry
	return {}


## Flap model in stage-local body coordinates, placed by its manifest entry (the drawn hinge = the physics hinge of the
## vehicle config: aft end station and radius, lean; pinned by a Python test). The hinge (model X axis, origin at its aft end) sits at that point at the hinge azimuth and leans towards the
## station axis going forward by the lean l, in the meridian plane: X' = cos l X - sin l e_r, Y' = sin l X + cos l e_r
## (l = 0: the station axis and the radial). The model Z (panel at rest) is the stow direction of core/src/aero/flap.cpp
## (hull tangent away from the belly: X x e_r, or its opposite = the mirrored basis when the manifest's rotation sign
## about +X is +1); the deflection d turns the panel about the hinge X' by rotation_sign * d, towards Y' (d = 90 deg:
## in the meridian plane, broadside, span perpendicular to the hinge).
static func flap_transform(flap: Dictionary, entry: Dictionary) -> Transform3D:
	var rotation_sign := int(entry["rotation_sign_about_plus_x"])
	var azimuth := deg_to_rad(float(flap["hinge_azimuth_deg"]))
	var lean := deg_to_rad(float(entry["drawn_hinge_lean_deg"]))
	var e_r := Vector3(0.0, cos(azimuth), sin(azimuth))
	var hinge_axis := Vector3.RIGHT * cos(lean) - e_r * sin(lean)
	var normal := Vector3.RIGHT * sin(lean) + e_r * cos(lean)
	var stow := Vector3.RIGHT.cross(e_r) * (-float(rotation_sign))
	var rest := Basis(hinge_axis, normal, stow)  # model (X, Y, Z) -> (hinge, normal, stow); det -1 = mirrored
	var hinge := Vector3(float(entry["drawn_hinge_station_aft_m"]), 0.0, 0.0) 			+ e_r * float(entry["drawn_hinge_radius_aft_m"])
	var turn := Basis(hinge_axis, float(rotation_sign) * deg_to_rad(float(flap["deflection_deg"])))
	return Transform3D(turn * rest, hinge)


## The Crew Dragon's nose cone in capsule-local body coordinates: its model's origin is the hinge point (manifest `nose`:
## station, azimuth, radius of the hinge, the tangential axis, the open angle), so the hinge stays where it is at every
## opening `fraction` (0 closed .. 1 open) and the cone turns about the tangent, +X towards the outward radial.
static func nose_transform(nose: Dictionary, fraction: float) -> Transform3D:
	var azimuth := deg_to_rad(float(nose["hinge_azimuth_deg"]))
	var hinge := Vector3(float(nose["hinge_station_m"]), 0.0, 0.0) + Vector3(0.0, cos(azimuth), sin(azimuth)) * float(nose["hinge_radius_m"])
	var axis := Vector3(nose["axis"][0], nose["axis"][1], nose["axis"][2])
	return Transform3D(Basis(axis, deg_to_rad(float(nose["open_angle_deg"])) * clampf(fraction, 0.0, 1.0)), hinge)


## Grid fin model in stage-local body coordinates: pivot on the hull at the fin's station and azimuth, model Y radial;
## the deflection d is a right-handed rotation by +d about the radial axis (core/src/aero/grid_fin.cpp: the lattice
## normal turns from the tangent toward +X).
static func fin_transform(fin: Dictionary, base_station_m: float, hull_radius_m: float) -> Transform3D:
	var azimuth := deg_to_rad(float(fin["azimuth_deg"]))
	var around_x := Basis(Vector3.RIGHT, azimuth)
	var pivot := Vector3(float(fin["station_m"]) - base_station_m, cos(azimuth) * hull_radius_m,
			sin(azimuth) * hull_radius_m)
	return Transform3D(around_x * Basis(Vector3.UP, deg_to_rad(float(fin["deflection_deg"]))), pivot)


## The folded pose of a fin while another stage stands on its stage (F_ORBCOMM_2, F_Falcon_9_first_stage_in_hangar): panel flat
## against the hull, hinge on top, tip toward the base. The physics has no fold; a view rule from the snapshot's stage list.
static func fin_stowed_transform(fin: Dictionary, base_station_m: float, hull_radius_m: float, standoff_m: float) -> Transform3D:
	var azimuth := deg_to_rad(float(fin["azimuth_deg"]))
	var e_r := Vector3(0.0, cos(azimuth), sin(azimuth))
	var tangent := Vector3(0.0, -sin(azimuth), cos(azimuth))
	return Transform3D(Basis(e_r, Vector3.LEFT, tangent),
			Vector3(float(fin["station_m"]) - base_station_m, 0.0, 0.0) + e_r * (hull_radius_m + standoff_m))


## Rig of a landing leg, a function of the snapshot's deployment only (stage-local body coordinates): {"main": the rigid leg
## turned about its hinge from the stowed to the deployed angle, "pusher": the tube from the hull anchor to its point on the
## leg, its +X scaled to that distance}. `rig`: manifest "leg" (generated by vehicle_spec.falcon9_leg).
static func leg_poses(rig: Dictionary, azimuth_deg: float, deployment: float) -> Dictionary:
	var azimuth := deg_to_rad(azimuth_deg)
	var around_x := Basis(Vector3.RIGHT, azimuth)
	var e_r := Vector3(0.0, cos(azimuth), sin(azimuth))
	var angle := deg_to_rad(lerpf(float(rig["stowed_angle_deg"]), float(rig["deployed_angle_deg"]), clampf(deployment, 0.0, 1.0)))
	var hinge := Vector3(float(rig["hinge_station_m"]), 0.0, 0.0) + e_r * float(rig["hinge_radius_m"])
	var main := Transform3D(around_x * Basis(Vector3.BACK, angle), hinge)
	var anchor := Vector3(float(rig["pusher_anchor_station_m"]), 0.0, 0.0) + e_r * float(rig["pusher_anchor_radius_m"])
	var on_leg := main * Vector3(float(rig["pusher_leg_u_m"]), 0.0, 0.0)
	var along := on_leg - anchor
	var x := along.normalized()
	var z := around_x * Vector3.BACK  # the hinge axis: normal to the leg's plane
	return {"main": main, "pusher": Transform3D(Basis(along, z.cross(x), z), anchor)}
