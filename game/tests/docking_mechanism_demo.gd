# Demo of the Crew Dragon's docking mechanism: extend, contact, retract, close the hooks, driven only through DockingMechanismView.set_state, with the passive port
# of a second mechanism opposite (the IDSS is androgynous; that port's ring stays retracted). Needs a GPU (no --headless) and the generated models.
# godot --path game --script res://tests/docking_mechanism_demo.gd -- --out_dir=<abs dir> [--sun=<deg>]
# Writes <phase>_<view>.png per phase and view. Numbers: the manifest's idss block; contact tilt and stroke of this scene are illustrative, not a flown case.
extends SceneTree

const FRAMES_TO_SETTLE := 6
const CONTACT_TILT_DEG := 3.0  # [E] illustrative, inside the IDD's 4 deg envelope
const RETRACT_FRACTION := 0.4  # [E] the middle of the retraction shown
const FAR_APART_M := 0.9  # [E] the passive port's distance before contact
const VIEWS := {"side": [Vector3(0.0, 0.0, 3.2), Vector3(0.1, 0.0, 0.0), 40.0], "quarter": [Vector3(1.6, 1.3, 2.0), Vector3(0.1, 0.0, 0.0), 40.0],
		"top": [Vector3(3.2, 0.25, 0.2), Vector3(0.0, 0.0, 0.0), 40.0], "hook": []}  # aimed at hook 0, the passive port hidden

var _view: DockingMechanismView
var _passive: DockingMechanismView
var _camera: Camera3D
var _out_dir := ""
var _phases: Array = []
var _phase := 0
var _view_index := 0
var _wait := 0
var _full := 0.3


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--out_dir="):
			_out_dir = arg.trim_prefix("--out_dir=")
	VehicleModels.clear_cache()
	var models := VehicleModels.load_for("crew_dragon")
	if models.is_empty() or _out_dir == "":
		print("docking_mechanism_demo: no models or no --out_dir (%s)" % VehicleModels.fallback_reason("crew_dragon"))
		quit(1)
		return
	DirAccess.make_dir_recursive_absolute(_out_dir)
	var scene := Node3D.new()
	root.add_child(scene)
	var look: Dictionary = models["manifest"]["look"]
	var material_of := func(part: String) -> Material:
		var paint: Dictionary = look["paint"][look["part_materials"][part]]
		var m := StandardMaterial3D.new()
		var a: Array = paint["albedo"]
		m.albedo_color = Color(a[0], a[1], a[2])
		m.roughness = float(paint["roughness"])
		m.metallic = float(paint["metallic"])
		m.cull_mode = BaseMaterial3D.CULL_DISABLED
		return m
	var body := Node3D.new()  # the capsule's body frame, the port at the origin of the scene
	scene.add_child(body)
	_view = DockingMechanismView.new()
	body.add_child(_view)
	_view.build(models, material_of)
	var port_x := _view.position.x
	_view.position.x = 0.0
	var capsule := Node3D.new()
	capsule.position = Vector3(-port_x, 0.0, 0.0)
	body.add_child(capsule)
	for part: String in models["hulls"]["capsule"]:
		var instance := MeshInstance3D.new()
		instance.mesh = models["hulls"]["capsule"][part]
		instance.material_override = _hull_paint(look, part, material_of)
		capsule.add_child(instance)
	var nose := Node3D.new()
	nose.transform = VehicleModels.nose_transform(models["manifest"]["nose"], 1.0)
	capsule.add_child(nose)
	for part: String in models["nose"]:
		var instance := MeshInstance3D.new()
		instance.mesh = models["nose"][part]
		instance.material_override = _hull_paint(look, part, material_of)
		nose.add_child(instance)
	_passive = DockingMechanismView.new()
	body.add_child(_passive)
	_passive.build(models, material_of)
	_full = float(models["manifest"]["idss"]["ring_extension_m"])
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-35.0, 40.0, 0.0)
	sun.light_energy = 2.2
	sun.shadow_enabled = true
	scene.add_child(sun)
	var fill := DirectionalLight3D.new()  # [E] a soft second light: the hooks' faces lie in the sun's shadow otherwise
	fill.rotation_degrees = Vector3(-20.0, -120.0, 0.0)
	fill.light_energy = 0.8
	scene.add_child(fill)
	var env := WorldEnvironment.new()
	env.environment = Environment.new()
	env.environment.background_mode = Environment.BG_COLOR
	env.environment.background_color = Color(0.02, 0.02, 0.03)
	env.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.environment.ambient_light_color = Color(0.6, 0.62, 0.7)
	env.environment.ambient_light_energy = 0.5
	scene.add_child(env)
	_camera = Camera3D.new()
	_camera.near = 0.02
	scene.add_child(_camera)
	_camera.current = true
	root.size = Vector2i(1400, 1000)
	var tilt := deg_to_rad(CONTACT_TILT_DEG)
	# [name, extension, tilt, hooks, passive port distance]
	_phases = [["ready", 0.0, Vector2.ZERO, 0.0, FAR_APART_M], ["extended", _full, Vector2.ZERO, 0.0, FAR_APART_M], ["contact", _full, Vector2(tilt, 0.0), 0.0, _full],
			["retracting", RETRACT_FRACTION * _full, Vector2.ZERO, 0.0, RETRACT_FRACTION * _full], ["mated_open", 0.0, Vector2.ZERO, 0.0, 0.0], ["hooks_closed", 0.0, Vector2.ZERO, 1.0, 0.0]]
	_apply_phase()


func _hull_paint(look: Dictionary, part: String, material_of: Callable) -> Material:
	return material_of.call(part) if look["part_materials"].has(part) else StandardMaterial3D.new()


## The passive port: the mechanism turned half about its up, its plane `distance` above this port. Only set_state moves the parts.
func _apply_phase() -> void:
	var phase: Array = _phases[_phase]
	_view.set_state(float(phase[1]), phase[2] as Vector2, float(phase[3]))
	_passive.set_state(0.0, Vector2.ZERO, 0.0)
	_passive.basis = Basis(Vector3(0.0, 0.0, 1.0), PI)
	_passive.position = Vector3(float(phase[4]), 0.0, 0.0)
	_set_camera()


func _set_camera() -> void:
	var view: Array = VIEWS[VIEWS.keys()[_view_index]]
	_passive.visible = not view.is_empty()
	if view.is_empty():
		var at := _view.hook_transform(0, 0.0).origin
		_camera.fov = 20.0
		var outward := Vector3(0.0, at.y, at.z).normalized()
		_camera.look_at_from_position(at + outward * 0.4 + Vector3.RIGHT * 0.05, at + Vector3.RIGHT * 0.03, Vector3.RIGHT)
		_wait = FRAMES_TO_SETTLE
		return
	_camera.fov = float(view[2])
	_camera.look_at_from_position(view[0], view[1], Vector3.RIGHT if absf((view[0] - view[1]).normalized().x) < 0.9 else Vector3.BACK)
	_wait = FRAMES_TO_SETTLE


func _process(_delta: float) -> bool:
	if _phases.is_empty():
		return false
	_wait -= 1
	if _wait > 0:
		return false
	var name_of: String = "%s_%s.png" % [_phases[_phase][0], VIEWS.keys()[_view_index]]
	root.get_texture().get_image().save_png(_out_dir.path_join(name_of))
	print("docking_mechanism_demo: ", name_of)
	_view_index += 1
	if _view_index >= VIEWS.size():
		_view_index = 0
		_phase += 1
		if _phase >= _phases.size():
			quit(0)
			return true
		_apply_phase()
	else:
		_set_camera()
	return false
