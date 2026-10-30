class_name StructureModels
## Run-time loader of the generated Pad B models (tools/structures/build_models.py -> data/models/starbase_olp2/) and of
## the detail file they come from (config/structures/starbase_olp2.toml). Without a detail file, fresh models or a site
## that uses them it returns {} and the caller keeps its boxes (one log line per site).
## The glTFs keep Blender axes (X along the arm, Y across, Z up; manifest "frame"): BLENDER_TO_RENDER is the one place
## that maps them to render axes, the same cyclic permutation as Frames.to_render.

## Prototype: only this site shows the Pad B models, placed at its tower box (spec H). A named constant, not a config key,
## until the full build gives Pad B a site of its own.
const PROTOTYPE_SITE_ID := "starbase_olp1"
const DETAIL_FILE := "structures/starbase_olp2.toml"
const MODEL_DIR := "models/starbase_olp2"
const MANIFEST := "manifest.json"
## Generator sources a model must not be older than (repo checkouts only; a packaged install has none and counts as fresh).
const SOURCE_FILES: Array[String] = [DETAIL_FILE]
const SOURCE_SCRIPTS: Array[String] = ["common.py", "gen_profiles.py", "gen_chopstick.py"]
## Blender (X, Y, Z) -> render (Y, Z, X): arm along +X becomes +Z (the arm pivot's forward axis), Z up becomes +Y.
const BLENDER_TO_RENDER := Basis(Vector3(0.0, 0.0, 1.0), Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0))

static var _logged: Dictionary = {}


## {} or {"detail": TomlLite dictionary, "profiles": {name: Mesh}, "chopstick": {"truss": Mesh, "rail": Mesh}}.
static func load_for(site_id: String) -> Dictionary:
	if site_id != PROTOTYPE_SITE_ID:
		return {}
	var detail_path := Paths.config_dir().path_join(DETAIL_FILE)
	var model_dir := Paths.data_dir().path_join(MODEL_DIR)
	if not FileAccess.file_exists(detail_path):
		return _fallback(site_id, "no detail file " + detail_path)
	var missing := _missing_or_stale(model_dir)
	if missing != "":
		return _fallback(site_id, missing)
	var detail := TomlLite.load_file(detail_path)
	var profiles := _meshes_of(model_dir.path_join("profiles.gltf"), "profile_")
	var chopstick := _meshes_of(model_dir.path_join("chopstick.gltf"), "")
	for section: Dictionary in detail.get("tower.sections", []):
		for key: String in ["leg_profile", "brace_profile"]:
			if not profiles.has(section[key]):
				return _fallback(site_id, "profile %s missing in profiles.gltf" % section[key])
	if not (chopstick.has("truss") and chopstick.has("rail")):
		return _fallback(site_id, "chopstick.gltf lacks the nodes truss / rail")
	return {"detail": detail, "profiles": profiles, "chopstick": chopstick}


## The chopstick as a node in the arm pivot's frame (hinge at the origin, +Z along the arm, +Y up, top rails at Y = 0).
static func chopstick_node(chopstick: Dictionary, material: Material) -> Node3D:
	var root := Node3D.new()
	root.name = "Chopstick"
	root.basis = BLENDER_TO_RENDER
	for part: String in ["truss", "rail"]:
		var node := MeshInstance3D.new()
		node.name = part
		node.mesh = chopstick[part]
		node.material_override = material
		root.add_child(node)
	return root


static func _fallback(site_id: String, reason: String) -> Dictionary:
	if not _logged.has(site_id):
		_logged[site_id] = true
		print("StructureModels: %s keeps its boxes (%s; run python tools/structures/build_models.py)" % [site_id, reason])
	return {}


## "" when manifest and both glTFs exist and are not older than the sources; otherwise the reason.
static func _missing_or_stale(model_dir: String) -> String:
	var manifest := model_dir.path_join(MANIFEST)
	if not FileAccess.file_exists(manifest):
		return "no models in " + model_dir
	for file in ["profiles.gltf", "profiles.bin", "chopstick.gltf", "chopstick.bin"]:
		if not FileAccess.file_exists(model_dir.path_join(file)):
			return "model file %s missing" % file
	var built := FileAccess.get_modified_time(manifest)
	var sources: Array[String] = [Paths.config_dir().path_join(DETAIL_FILE)]
	for script in SOURCE_SCRIPTS:
		sources.append(Paths.repo_dir().path_join("tools/structures/blender").path_join(script))
	for path in sources:
		if FileAccess.file_exists(path) and FileAccess.get_modified_time(path) > built:
			return "models older than " + path.get_file()
	return ""


## Meshes of a glTF by node name (prefix stripped when it starts with `prefix`); the loaded scene is freed.
static func _meshes_of(path: String, prefix: String) -> Dictionary:
	var meshes := {}
	var document := GLTFDocument.new()
	var state := GLTFState.new()
	if document.append_from_file(path, state) != OK:
		return meshes
	var scene := document.generate_scene(state)
	if scene == null:
		return meshes
	for node in scene.find_children("*", "MeshInstance3D", true, false):
		var mesh_node := node as MeshInstance3D
		if prefix == "" or String(mesh_node.name).begins_with(prefix):
			meshes[String(mesh_node.name).trim_prefix(prefix)] = mesh_node.mesh
	scene.free()
	return meshes
