class_name VehicleModels
## Run-time loader of the generated vehicle models (tools/structures/build_models.py --vehicle <id> ->
## data/models/<id>/) and the pose maths that places them from the flight snapshot. Without a detail file, fresh
## models or a vehicle that uses them it returns {} and VehicleView keeps its cylinders (one log line per vehicle).
## The glTFs keep Blender axes = the config's BODY frame (X station axis, nose +X, belly azimuth 270 = -Z; manifest
## "frame"). The poses below are body coordinates relative to the stage's base; the stage node carries the one
## permutation to render axes (body_to_render, the cyclic permutation of Frames.to_render).

## Prototype: only this vehicle has models. A named constant, not a config key, until the full build.
const PROTOTYPE_VEHICLE_ID := "starship_block3"
const MANIFEST := "manifest.json"
const MODEL_FILES: Array[String] = ["booster", "ship", "flap_fwd", "flap_aft", "grid_fin", "raptor", "rvac"]
const SOURCE_SCRIPTS: Array[String] = ["vehicle_common.py", "gen_hull.py", "gen_flap.py", "gen_grid_fin.py",
		"gen_engine.py"]
## Engine kinds: the snapshot's "vacuum" flag selects the model and the manifest's engine id.
const ENGINE_FILE := {false: "raptor", true: "rvac"}
const ENGINE_ID := {false: "raptor3", true: "raptor_vacuum3"}
## Nozzle exit below the gimbal point when no model says better (the former view's value).
const DEFAULT_EXIT_DEPTH_M := 3.0
const HULL_SHADER := preload("res://shaders/vehicle_hull.gdshader")
## "booster" = the booster hull: mode 0 with the baked steel map (the ship's hull carries it in mode 1).
const HULL_MODE := {"steel": 0, "booster": 0, "ship": 1, "flap": 2}
## Kinds whose surface is the hull's UV (arc metres, station metres) and so can carry the steel map.
const STEEL_MAP_KINDS: Array[String] = ["booster", "ship"]
## Entries of the shader's coverage table arrays.
const HULL_TABLE_SIZE := 8
const AZIMUTH_TOLERANCE_DEG := 0.5
const STATION_TOLERANCE_M := 0.01

static var _logged: Dictionary = {}
static var _cache: Dictionary = {}
## Reason of the last failed glTF load (set by _meshes_of), for the fallback log line.
static var _load_error := ""


## {} or {"manifest": Dictionary, "hulls": {stage_id: {node: Mesh}}, "flap": {"fwd"|"aft": {node: Mesh}},
## "grid_fin": {node: Mesh}, "engine": {false|true: Mesh}, "materials": {"steel"|"booster"|"ship"|"flap": ShaderMaterial}} (the
## engine meshes merged to one multi-surface mesh).
static func load_for(vehicle_id: String) -> Dictionary:
	if vehicle_id != PROTOTYPE_VEHICLE_ID:
		return {}
	if not _cache.has(vehicle_id):
		_cache[vehicle_id] = _load(vehicle_id)  # a failure is cached too: the view asks again whenever it rebuilds
	return _cache[vehicle_id]


## Drops the cached results (tests that regenerate the models).
static func clear_cache() -> void:
	_cache.clear()
	_logged.clear()


static func _load(vehicle_id: String) -> Dictionary:
	var model_dir := Paths.data_dir().path_join("models").path_join(vehicle_id)
	var missing := _missing_or_stale(vehicle_id, model_dir)
	if missing != "":
		return _fallback(vehicle_id, missing)
	var manifest: Variant = JSON.parse_string(FileAccess.get_file_as_string(model_dir.path_join(MANIFEST)))
	if not (manifest is Dictionary and manifest.has("flaps") and manifest.has("engines")):
		return _fallback(vehicle_id, "manifest unreadable")
	var parts := {}
	for file: String in MODEL_FILES:
		parts[file] = _meshes_of(model_dir.path_join(file + ".gltf"))
		if (parts[file] as Dictionary).is_empty():
			return _fallback(vehicle_id, _load_error)
	for engine_file: String in ["raptor", "rvac"]:
		parts[engine_file] = _merged(parts[engine_file])
	return {
		"manifest": manifest,
		"hulls": {"booster": parts["booster"], "ship": parts["ship"]},
		"flap": {"fwd": parts["flap_fwd"], "aft": parts["flap_aft"]},
		"grid_fin": parts["grid_fin"],
		"engine": {false: parts["raptor"], true: parts["rvac"]},
		"materials": _hull_materials(manifest.get("look", {}), model_dir, vehicle_id),
	}


## Hull shader materials (shaders/vehicle_hull.gdshader) sharing the manifest's "look" numbers (ring pitch, tile size and
## gap, the coverage table around the belly): "steel" (bare), "ship" (steel + tiles in the coverage), "flap" (tiles on the
## outer face). Opaque, like every vehicle surface.
static func _hull_materials(look: Dictionary, model_dir: String, vehicle_id: String) -> Dictionary:
	var materials := {}
	var steel_map := _steel_map(look.get("steel_map", {}), model_dir, vehicle_id)
	var tile_maps := _tile_maps(look.get("tile_maps", {}), model_dir, vehicle_id)
	for key: String in ["steel", "booster", "ship", "flap"]:
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
	return materials


## The baked steel map (tools/structures/hull_textures.py) with a full mip chain made here: a plain box average of
## slope, slope energy and weld coverage, which is what the map's channels are designed for. null without the file
## (plain steel, one log line).
static func _steel_map(info: Dictionary, model_dir: String, vehicle_id: String) -> ImageTexture:
	var path := model_dir.path_join(String(info.get("file", "hull_steel.png")))
	var image := Image.load_from_file(path) if not info.is_empty() and FileAccess.file_exists(path) else null
	if image == null:
		if not _logged.has(vehicle_id + "/steel_map"):
			_logged[vehicle_id + "/steel_map"] = true
			print("VehicleModels: %s has no steel map (%s): plain steel" % [vehicle_id, path.get_file()])
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
				print("VehicleModels: %s has no tile maps (%s): plain dark tiles" % [vehicle_id, path.get_file()])
			return {}
		image.convert(Image.FORMAT_RGBA8)
		if key == "patch":
			image.generate_mipmaps()
		maps[key] = ImageTexture.create_from_image(image)
	return maps


static func _fallback(vehicle_id: String, reason: String) -> Dictionary:
	if not _logged.has(vehicle_id):
		_logged[vehicle_id] = true
		print("VehicleModels: %s keeps its cylinders (%s; run python tools/structures/build_models.py --vehicle %s)"
				% [vehicle_id, reason, vehicle_id])
	return {}


## "" when manifest and every glTF exist and are not older than the sources; otherwise the reason.
static func _missing_or_stale(vehicle_id: String, model_dir: String) -> String:
	var manifest := model_dir.path_join(MANIFEST)
	if not FileAccess.file_exists(manifest):
		return "no models in " + model_dir
	for file: String in MODEL_FILES:
		for extension in [".gltf", ".bin"]:
			if not FileAccess.file_exists(model_dir.path_join(file + extension)):
				return "model file %s missing" % (file + extension)
	var built := FileAccess.get_modified_time(manifest)
	var sources: Array[String] = [Paths.config_dir().path_join("vehicles/detail/%s.toml" % vehicle_id),
			Paths.config_dir().path_join("vehicles/%s.toml" % vehicle_id)]
	for script in SOURCE_SCRIPTS:
		sources.append(Paths.repo_dir().path_join("tools/structures/blender").path_join(script))
	sources.append(Paths.repo_dir().path_join("tools/structures/hull_textures.py"))
	sources.append(Paths.repo_dir().path_join("tools/structures/hull_tiles.py"))
	for path in sources:
		if FileAccess.file_exists(path) and FileAccess.get_modified_time(path) > built:
			return "models older than " + path.get_file()
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
static func engine_transform(position_m: Vector3, direction: Vector3, base_station_m: float) -> Transform3D:
	var axis := direction.normalized() if direction.length() > 0.5 else Vector3.RIGHT
	return Transform3D(Basis(Quaternion(Vector3.RIGHT, axis)), position_m - Vector3(base_station_m, 0.0, 0.0))


## Exhaust start below the gimbal point along the thrust axis: the model's exit plane, else the former 3 m.
static func exit_depth_m(models: Dictionary, vacuum: bool) -> float:
	if models.is_empty():
		return DEFAULT_EXIT_DEPTH_M
	return -float(models["manifest"]["engines"][ENGINE_ID[vacuum]]["exit_plane_x_m"])


## The manifest entry of a snapshot flap (same stage id, hinge azimuth and aft station relative to the stage), or {}.
static func flap_entry(models: Dictionary, stage_id: String, flap: Dictionary, base_station_m: float) -> Dictionary:
	for entry: Dictionary in models["manifest"]["flaps"].get(stage_id, []):
		if absf(fposmod(float(entry["hinge_azimuth_deg"]) - float(flap["hinge_azimuth_deg"]) + 180.0, 360.0)
				- 180.0) < AZIMUTH_TOLERANCE_DEG \
				and absf(float(entry["station_aft_m"]) - (float(flap["station_aft_m"]) - base_station_m)) \
				< STATION_TOLERANCE_M:
			return entry
	return {}


## Flap model in stage-local body coordinates. The hinge (model X axis, origin at its aft end) sits on the hull at the
## hinge azimuth; the model Z (panel at rest) is the stow direction of core/src/aero/flap.cpp (hull tangent away from
## the belly: X x e_r, or its opposite = the mirrored basis when the manifest's rotation sign about +X is +1); the
## deflection d turns the panel about +X by rotation_sign * d, towards e_r (d = 90 deg: radial, broadside).
static func flap_transform(flap: Dictionary, base_station_m: float, hull_radius_m: float,
		rotation_sign: int) -> Transform3D:
	var azimuth := deg_to_rad(float(flap["hinge_azimuth_deg"]))
	var e_r := Vector3(0.0, cos(azimuth), sin(azimuth))
	var stow := Vector3.RIGHT.cross(e_r) * (-float(rotation_sign))
	var rest := Basis(Vector3.RIGHT, e_r, stow)  # model (X, Y, Z) -> (station axis, radial, stow); det -1 = mirrored
	var hinge := Vector3(float(flap["station_aft_m"]) - base_station_m, e_r.y * hull_radius_m, e_r.z * hull_radius_m)
	var turn := Basis(Vector3.RIGHT, float(rotation_sign) * deg_to_rad(float(flap["deflection_deg"])))
	return Transform3D(turn * rest, hinge)


## Grid fin model in stage-local body coordinates: pivot on the hull at the fin's station and azimuth, model Y radial;
## the deflection d is a right-handed rotation by +d about the radial axis (core/src/aero/grid_fin.cpp: the lattice
## normal turns from the tangent toward +X).
static func fin_transform(fin: Dictionary, base_station_m: float, hull_radius_m: float) -> Transform3D:
	var azimuth := deg_to_rad(float(fin["azimuth_deg"]))
	var around_x := Basis(Vector3.RIGHT, azimuth)
	var pivot := Vector3(float(fin["station_m"]) - base_station_m, cos(azimuth) * hull_radius_m,
			sin(azimuth) * hull_radius_m)
	return Transform3D(around_x * Basis(Vector3.UP, deg_to_rad(float(fin["deflection_deg"]))), pivot)
