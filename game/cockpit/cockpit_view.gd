class_name CockpitView
extends Node3D
## The Crew Dragon's cabin (docs/subsystems/cockpit.md) as a child of the capsule stage node: wall, floor, seats, console with
## screens / buttons / tablets, hatch, windows, the HOTAS props, the cabin lamps and the crew figures. Everything is in the
## capsule stage frame (the node's own transform is the identity); the stage node maps it to render axes. The cabin is part
## of the vehicle, so it exists in every view; the pilot camera (a later task) sits at `eye_node`.

const CABIN_SHADER := preload("res://shaders/cabin.gdshader")
const GLASS_SHADER := preload("res://shaders/cabin_glass.gdshader")
const DISPLAY_SHADER := preload("res://shaders/display_colour.gdshader")
const VEHICLE_ID := "crew_dragon"
const FIGURE_FILE := "crew_figure"
## Seats in the order the crew fills them: commander (the player body), pilot, then the outer seats.
const CREW_ORDER: Array[String] = ["s2", "s3", "s1", "s4"]
## [call] Look pass: luminous flux of one ceiling lamp panel and of the cabin fill (stands in for the light the white walls bounce).
const LAMP_LUMENS := 3000.0
const FILL_LUMENS := 7000.0
const LAMP_RANGE_M := 6.0
## [call] Look pass 2: the light the white walls bounce around the closed cabin (no GI): a flat illuminance on every cabin
## surface as emission albedo x E / pi, scene-referred like the lamps (so the eye adaptation sees a lit cabin at any hour).
const FILL_LUX := 450.0
## Lamp height below the ceiling mesh (the lamp housing hangs 0.08 m under the panel).
const LAMP_DROP_M := 0.08
## [E] The ceiling ring light around the forward tunnel (crew1_1: a bright white annulus, r ~0.45..0.9 m at the ceiling):
## RING_LAMPS omnis on r = RING_RADIUS_M (mid-annulus), RING_LUMENS in total (an area light's stand-in; its emissive mesh is the generator's).
const RING_LAMPS := 8
const RING_RADIUS_M := 0.72
const RING_LUMENS := 6000.0
const RING_DROP_M := 0.10
## [E] Screen placeholder: a dark Dragon-blue UI background, display-referred (no exposure, as every emitter).
const SCREEN_COLOUR := Color(0.025, 0.07, 0.17)

var config: CockpitConfig
var eye_node := Node3D.new()
var _nodes := {}  # cabin node name -> MeshInstance3D
var _lights: Array[OmniLight3D] = []
var _light_cd: Array[float] = []
var _figures := {}  # seat id -> Node3D (figure root)
var _skeletons := {}  # seat id -> Skeleton3D
var _crew := 0
var _materials := {}


## Builds the cabin from the model set's `models` (VehicleModels.load_for("crew_dragon")) and the cockpit definition.
func build(models: Dictionary, cockpit: CockpitConfig) -> void:
	config = cockpit
	name = "cockpit"
	eye_node.name = "eye"
	eye_node.transform = config.eye
	add_child(eye_node)
	var cabin: Dictionary = models.get("cabin", {})
	for part: String in cabin:
		var instance := MeshInstance3D.new()
		instance.name = part
		instance.mesh = cabin[part]
		instance.material_override = _material_of(part)
		if part.begins_with("window_glass") or part.begins_with("light_") or part.begins_with("screen_"):
			instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		add_child(instance)
		_nodes[part] = instance
	_build_lights()
	_build_figures(models)
	set_hotas_props(config.hotas_props_enabled)
	set_crew(config.seats.size())


## The mesh of a screen ("left", "centre", "right").
func screen_surface(id: String) -> MeshInstance3D:
	return _nodes.get("screen_" + id)


## The node of a button (config id; the centre handle is "handle").
func button_node(id: String) -> Node3D:
	return _nodes.get("handle" if id == "handle" else "button_" + id)


func cabin_node(node_name: String) -> MeshInstance3D:
	return _nodes.get(node_name)


## Crew of `count` (0..seats): figures in the seats in CREW_ORDER; the first is the player body (commander).
func set_crew(count: int) -> void:
	_crew = clampi(count, 0, CREW_ORDER.size())
	for i in CREW_ORDER.size():
		var figure: Node3D = _figures.get(CREW_ORDER[i])
		if figure != null:
			figure.visible = i < _crew


func crew_count() -> int:
	return _crew


func visible_figures() -> int:
	var n := 0
	for figure: Node3D in _figures.values():
		n += 1 if figure.visible else 0
	return n


## The figure's skeleton of a seat ("s1".."s4"; null without the model): the arm poses of the pointer (later task) set its bones.
func figure_skeleton(seat_id: String) -> Skeleton3D:
	return _skeletons.get(seat_id)


## Hides (collapses) the head, helmet and visor of the player's body (the commander's figure): the camera sits inside that
## helmet. The pilot camera switches it on; the external views show the whole figure.
func set_player_head_hidden(hidden: bool) -> void:
	var skeleton: Skeleton3D = _skeletons.get(CREW_ORDER[0])
	if skeleton == null:
		return
	var bone := skeleton.find_bone("head")
	if bone >= 0:
		skeleton.set_bone_pose_scale(bone, Vector3.ONE * (0.001 if hidden else 1.0))


## The stick and throttle props on the commander's armrests (the deliberate deviation, switchable).
func set_hotas_props(enabled: bool) -> void:
	for part in ["hotas_stick", "hotas_throttle"]:
		var node: MeshInstance3D = _nodes.get(part)
		if node != null:
			node.visible = enabled


func _process(_delta: float) -> void:
	var scene_per_cd: float = ShaderGlobals.value(&"atmosphere_scene_per_cd_m2")
	for i in _lights.size():
		_lights[i].light_energy = TowerLights.energy_of(_light_cd[i], scene_per_cd)
	var fill := FILL_LUX / PI * scene_per_cd
	for material: Material in _materials.values():
		if material is ShaderMaterial and (material as ShaderMaterial).shader == CABIN_SHADER:
			(material as ShaderMaterial).set_shader_parameter("fill", fill)


func _build_lights() -> void:
	for i in config.lights.size():
		var row: Dictionary = config.lights[i]
		var p: Vector3 = CockpitConfig._vec(row.get("position_m"))
		_add_omni("light_%s" % row.get("id", i), p + Vector3(-LAMP_DROP_M, 0.0, 0.0), LAMP_LUMENS).shadow_enabled = true  # two lamps: cheap shadows
	var ceiling_x := float(config.cabin.get("ceiling_x_m", 3.35))
	for k in RING_LAMPS:
		var az := TAU * float(k) / RING_LAMPS
		_add_omni("ring_%d" % k, Vector3(ceiling_x - RING_DROP_M, RING_RADIUS_M * cos(az), RING_RADIUS_M * sin(az)), RING_LUMENS / RING_LAMPS)
	_add_omni("fill_ceiling", Vector3(3.1, 0.0, 0.0), FILL_LUMENS)
	_add_omni("fill_seats", Vector3(2.0, 0.55, 0.0), FILL_LUMENS)


func _add_omni(light_name: String, at: Vector3, lumens: float) -> OmniLight3D:
	var omni := OmniLight3D.new()
	omni.name = light_name
	omni.omni_attenuation = 2.0
	omni.omni_range = LAMP_RANGE_M
	omni.light_specular = 0.4
	omni.shadow_enabled = false
	omni.position = at
	add_child(omni)
	_lights.append(omni)
	_light_cd.append(TowerLights.candela_omni(lumens))
	return omni


func _build_figures(models: Dictionary) -> void:
	var manifest: Dictionary = (models.get("manifest", {}) as Dictionary).get("crew_figure", {})
	var look: Dictionary = manifest.get("materials", {})
	var template := VehicleModels.scene_for(VEHICLE_ID, FIGURE_FILE)  # parsed once, duplicated per seat
	if template == null:
		return
	for seat: Dictionary in config.seats:
		var scene := template.duplicate()
		var seat_id := String(seat["id"])
		var angle := deg_to_rad(float(seat.get("back_angle_deg", 0.0)))
		var root := Node3D.new()
		root.name = "figure_" + seat_id
		root.transform = Transform3D(Basis(Vector3(cos(angle), -sin(angle), 0.0), Vector3(sin(angle), cos(angle), 0.0),
				Vector3(0.0, 0.0, 1.0)), seat["hip"])
		root.add_child(scene)
		add_child(root)
		_figures[seat_id] = root
		var skeletons := scene.find_children("*", "Skeleton3D", true, false)
		if not skeletons.is_empty():
			_skeletons[seat_id] = skeletons[0]
		for node in scene.find_children("*", "MeshInstance3D", true, false):
			var mesh_node := node as MeshInstance3D
			for s in mesh_node.mesh.get_surface_count():
				var source := mesh_node.mesh.surface_get_material(s)
				var key := source.resource_name if source != null else ""
				if look.has(key):
					var entry: Dictionary = look[key]
					var c: Array = entry["albedo"]
					mesh_node.set_surface_override_material(s, _cabin_material(Color(c[0], c[1], c[2]).linear_to_srgb(),
							float(entry["roughness"]), float(entry["metallic"]), 0.03, 0.0))
	template.free()


func _material_of(part: String) -> Material:
	if part.begins_with("screen_"):
		return _display(SCREEN_COLOUR)
	if part.begins_with("window_glass"):
		var glass := ShaderMaterial.new()
		glass.shader = GLASS_SHADER
		return glass
	if part.begins_with("light_"):
		return _display(Color(0.95, 0.95, 0.9))
	if part == "cabin_shell":
		return _cabin_material(Color(0.8, 0.8, 0.79), 0.85, 0.0, 0.03, 0.5)
	if part == "floor" or part == "hatch_frame" or part.begins_with("window_"):
		return _cabin_material(Color(0.8, 0.8, 0.79), 0.8, 0.0, 0.03, 0.0)
	if part == "console_housing":
		return _cabin_material(Color(0.045, 0.045, 0.05), 0.55, 0.1, 0.03, 0.0)
	if part.begins_with("seat_"):
		return _cabin_material(Color(0.03, 0.03, 0.032), 0.7, 0.0, 0.03, 0.0)
	if part.begins_with("tablet_"):
		return _cabin_material(Color(0.1, 0.1, 0.11), 0.35, 0.6, 0.03, 0.0)
	if part.begins_with("hotas_"):
		return _cabin_material(Color(0.06, 0.06, 0.065), 0.5, 0.4, 0.03, 0.0)
	return _cabin_material(Color(0.09, 0.09, 0.1), 0.45, 0.3, 0.03, 0.0)  # buttons, handle


func _display(colour: Color) -> ShaderMaterial:
	var key := "display_%s" % colour
	if not _materials.has(key):
		var m := ShaderMaterial.new()
		m.shader = DISPLAY_SHADER
		m.set_shader_parameter("colour", colour)
		_materials[key] = m
	return _materials[key]


func _cabin_material(albedo: Color, roughness: float, metallic: float, ambient: float, seams: float) -> ShaderMaterial:
	var key := "cabin_%s_%s_%s_%s_%s" % [albedo, roughness, metallic, ambient, seams]
	if not _materials.has(key):
		var m := ShaderMaterial.new()
		m.shader = CABIN_SHADER
		m.set_shader_parameter("albedo", albedo)
		m.set_shader_parameter("roughness", roughness)
		m.set_shader_parameter("metallic", metallic)
		m.set_shader_parameter("ambient_visibility", ambient)
		m.set_shader_parameter("seam_depth", seams)
		_materials[key] = m
	return _materials[key]
