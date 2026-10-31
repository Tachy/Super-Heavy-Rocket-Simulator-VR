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
	return materials


## The baked steel map (tools/structures/hull_textures.py) with a full mip chain made here: a plain box average of
## slope, slope energy and weld coverage, which is what the map's channels are designed for. null without the file
## (plain steel, one log line).
## Bell material of one engine kind: ribs map (one rib, mipmapped, baked here: a plain 1D profile), dark per ENGINE_LOOK.
static func engine_material(vacuum: bool, models: Dictionary) -> ShaderMaterial:
	var look: Dictionary = ENGINE_LOOK[vacuum]
	var m := ShaderMaterial.new()
	m.shader = ENGINE_SHADER
	m.set_shader_parameter("albedo", look["albedo"])
	m.set_shader_parameter("base_roughness", look["roughness"])
	m.set_shader_parameter("metallic_value", look["metallic"])
	m.set_shader_parameter("rib_slope", look["rib_slope"])
	var manifest_look: Dictionary = models["manifest"].get("look", {})
	var counts: Dictionary = manifest_look.get("rib_counts", {})
	var engine: Dictionary = models["manifest"]["engines"].get(ENGINE_ID[vacuum], {})
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
	if models.is_empty() or stage_id != "ship":
		return 0.0
	return float(models["manifest"]["engines"][ENGINE_ID[vacuum]]["ship_axial_offset_m"])


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
