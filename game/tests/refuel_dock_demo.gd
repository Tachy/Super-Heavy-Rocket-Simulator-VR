# Demo of the refuelling dock: a tanker approaches the depot's dorsal side, its probes extend and capture, driven ONLY through ProbeDrogueView.set_state
# (the tanker's range is the demo's own pose of the scene, the way the sim moves a body; no physics here). The depot's drogues are the passive half.
# Needs a GPU (no --headless) and the generated models (build_models.py --vehicle starship_tanker / starship_depot).
# godot --path game --script res://tests/refuel_dock_demo.gd -- --out_dir=<abs dir> [--prefix=depot_demo]
# Writes <prefix>_<phase>_<view>.png. The docked pose is the sim's (core/src/vehicle/port_mating.cpp, mirrored in probe_drogue_test.gd mated_transform).
# Ranges and the state per phase are illustrative, not a flown case; the real snapshot mapping is ProbeDrogueView.extension_for.
extends SceneTree

const FRAMES_TO_SETTLE := 8
const PairTest := preload("res://tests/probe_drogue_test.gd")
## [name, range of the tanker from the mated pose along the port axis (m), probe extension 0..1, captured]
## (extension -1 = the view's seated extension: heads in their seats). Contact is at 0.3 m (the core's extension_m): the extended heads reach the seats, then the draw-in closes the gap.
const PHASES := [["approach", 40.0, 0.0, false], ["final", 6.0, 1.0, false], ["probes_out", 2.5, 1.0, false], ["contact", 0.3, 1.0, false], ["captured", 0.0, -1.0, true]]
## [name, camera position, aim point, fov]: the whole pair from the side, and the interface between the hulls (dorsal gap) at the first probe/drogue.
const VIEWS := {"pair": [Vector3(20.0, -110.0, 20.0), Vector3(20.0, 0.0, 14.0), 30.0],
		"close": [Vector3(8.5, -9.0, 5.9), Vector3(12.0, -1.5, 5.0), 24.0]}

var _depot_models: Dictionary
var _tanker_models: Dictionary
var _tanker_node: Node3D
var _tanker_view: ProbeDrogueView
var _depot_view: ProbeDrogueView
var _pose: Transform3D
var _camera: Camera3D
var _out_dir := ""
var _prefix := "depot_demo"
var _phase := 0
var _view_index := 0
var _wait := 0


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--out_dir="):
			_out_dir = arg.trim_prefix("--out_dir=")
		elif arg.begins_with("--prefix="):
			_prefix = arg.trim_prefix("--prefix=")
	VehicleModels.clear_cache()
	_depot_models = VehicleModels.load_for("starship_depot")
	_tanker_models = VehicleModels.load_for("starship_tanker")
	if _depot_models.is_empty() or _tanker_models.is_empty() or _out_dir == "":
		print("refuel_dock_demo: models missing or no --out_dir (%s / %s)" % [VehicleModels.fallback_reason("starship_depot"), VehicleModels.fallback_reason("starship_tanker")])
		quit(1)
		return
	DirAccess.make_dir_recursive_absolute(_out_dir)
	var scene := Node3D.new()
	root.add_child(scene)
	var depot_node := _ship_node(_depot_models)
	scene.add_child(depot_node)
	_depot_view = _dorsal(_depot_models, depot_node)
	_tanker_node = _ship_node(_tanker_models)
	scene.add_child(_tanker_node)
	_tanker_view = _dorsal(_tanker_models, _tanker_node)
	_pose = PairTest.mated_transform(_tanker_models["manifest"]["dorsal_port"], _depot_models["manifest"]["dorsal_port"])
	var sun := DirectionalLight3D.new()
	sun.light_energy = 1.5
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 400.0
	scene.add_child(sun)
	sun.look_at_from_position(Vector3(-40.0, -80.0, 40.0), Vector3(20.0, 0.0, 5.0), Vector3.BACK)  # from the cameras' side (-y), a little from the nose side and above
	var fill := DirectionalLight3D.new()  # [E] a soft second light: the gap between the hulls lies in shadow otherwise
	fill.light_energy = 0.5
	scene.add_child(fill)
	fill.look_at_from_position(Vector3(80.0, 60.0, -30.0), Vector3(20.0, 0.0, 5.0), Vector3.BACK)
	var env := WorldEnvironment.new()
	env.environment = Environment.new()
	env.environment.background_mode = Environment.BG_COLOR
	env.environment.background_color = Color(0.02, 0.02, 0.03)
	env.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.environment.ambient_light_color = Color(0.6, 0.62, 0.7)
	env.environment.ambient_light_energy = 0.4
	scene.add_child(env)
	_camera = Camera3D.new()
	_camera.near = 0.05
	_camera.far = 400.0
	scene.add_child(_camera)
	_camera.current = true
	root.size = Vector2i(1600, 1000)
	_apply()


## A ship's hull parts (no engines) with the model set's own hull shader materials, in the ship's body frame.
func _ship_node(models: Dictionary) -> Node3D:
	var node := Node3D.new()
	var look: Dictionary = models["manifest"].get("look", {})
	var part_materials: Dictionary = look.get("part_materials", {})
	var parts: Dictionary = models["hulls"][VehicleModels.ship_stage_id(models)]
	for part: String in parts:
		var instance := MeshInstance3D.new()
		instance.mesh = parts[part]
		instance.material_override = (models["materials"] as Dictionary)[_kind(part, part_materials)]
		node.add_child(instance)
	return node


static func _kind(part: String, part_materials: Dictionary) -> String:
	if part_materials.has(part):
		return part_materials[part]
	if part == "hull" or part == "hinge_covers":
		return "ship"
	return "steel"


func _dorsal(models: Dictionary, parent: Node3D) -> ProbeDrogueView:
	var view := ProbeDrogueView.new()
	parent.add_child(view)
	var part_materials: Dictionary = models["manifest"].get("look", {}).get("part_materials", {})
	view.build(models, func(part: String) -> Material: return (models["materials"] as Dictionary)[_kind(part, part_materials)])
	return view


## The phase: the tanker's pose (mated, then moved out along the depot's dorsal axis by the range) and the probe state through set_state only.
func _apply() -> void:
	var phase: Array = PHASES[_phase]
	_tanker_node.transform = Transform3D(Basis(), Vector3(0.0, 0.0, float(phase[1]))) * _pose
	_tanker_view.set_state(float(phase[2]) if float(phase[2]) >= 0.0 else _tanker_view.seated_extension(), bool(phase[3]))
	_depot_view.set_state(0.0, false)  # the drogues are passive
	var view: Array = VIEWS[VIEWS.keys()[_view_index]]
	_camera.fov = float(view[2])
	_camera.look_at_from_position(view[0], view[1], Vector3.BACK)  # the dorsal axis is up in these views
	_wait = FRAMES_TO_SETTLE


func _process(_delta: float) -> bool:
	if _tanker_view == null:
		return false
	_wait -= 1
	if _wait > 0:
		return false
	var name_of := "%s_%s_%s.png" % [_prefix, PHASES[_phase][0], VIEWS.keys()[_view_index]]
	root.get_texture().get_image().save_png(_out_dir.path_join(name_of))
	print("refuel_dock_demo: ", name_of)
	_view_index += 1
	if _view_index >= VIEWS.size():
		_view_index = 0
		_phase += 1
		if _phase >= PHASES.size():
			quit(0)
			return true
	_apply()
	return false
