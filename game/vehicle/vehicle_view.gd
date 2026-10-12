class_name VehicleView
extends Node3D
## Visual of one flying body (stack, booster or ship) built from its stage geometry, with engine plumes.
## Placed every frame relative to the floating origin; mesh coordinates are body coordinates
## (x = vehicle axis) mapped to render axes and relative to the centre of gravity.

const PLUME_SEA_LEVEL_LENGTH_M := 35.0
const PLUME_VACUUM_LENGTH_M := 160.0
const HIDE_BEYOND_M := 2.0e6  # vehicles are sub-pixel beyond this distance

var body_index := 0
var _stage_root := Node3D.new()
var _plumes: Array[MeshInstance3D] = []
var _steel := StandardMaterial3D.new()
var _plume_material := StandardMaterial3D.new()
var _built_signature := ""


func _ready() -> void:
	add_child(_stage_root)
	_steel.albedo_color = Color(0.74, 0.75, 0.77)
	_steel.metallic = 0.65   # stainless steel; no reflection probes yet
	_steel.roughness = 0.38
	_plume_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_plume_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_plume_material.blend_mode = BaseMaterial3D.BLEND_MODE_ADD
	_plume_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	_plume_material.albedo_color = Color(1.0, 0.55, 0.25, 0.55)
	_plume_material.emission_enabled = true
	_plume_material.emission = Color(1.0, 0.6, 0.3)
	_plume_material.emission_energy_multiplier = 6.0


## Rebuilds geometry when the stage list changed (e.g. after separation), then places the body.
func update_view(sim: FlightSim, camera_gcrf_m: Vector3) -> void:
	var stages: Array = sim.body_stages(body_index)
	var signature := ",".join(stages.map(func(s: Dictionary) -> String: return s["id"]))
	if signature != _built_signature:
		_build(stages)
		_built_signature = signature

	var offset := Frames.to_render(sim.body_position_gcrf_m(body_index) - camera_gcrf_m)
	visible = offset.length() < HIDE_BEYOND_M
	if not visible:
		return
	var attitude := Basis(sim.body_attitude_gcrf(body_index))
	transform = Transform3D(Frames.basis_to_render(attitude), offset)
	var cg := sim.body_cg_station_m(body_index)
	_stage_root.position = Frames.to_render(Vector3(-cg, 0.0, 0.0))
	_update_plumes(sim.body_engines(body_index), sim.telemetry(body_index).get("altitude_m", 0.0))


func _build(stages: Array) -> void:
	for child in _stage_root.get_children():
		child.queue_free()
	_plumes.clear()
	var axis_rotation := Basis(Vector3.RIGHT, PI / 2.0)  # CylinderMesh +Y -> render +Z (= body +x)
	for stage: Dictionary in stages:
		var radius: float = stage["diameter_m"] * 0.5
		var nose: float = stage["nose_length_m"]
		var barrel: float = stage["length_m"] - nose
		_add_cylinder(radius, radius, barrel, stage["base_station_m"] + barrel * 0.5, axis_rotation)
		if nose > 0.0:
			_add_cylinder(radius * 0.08, radius, nose, stage["base_station_m"] + barrel + nose * 0.5, axis_rotation)


func _add_cylinder(top_radius: float, bottom_radius: float, height: float, center_station: float, rotation: Basis) -> void:
	var mesh := CylinderMesh.new()
	mesh.top_radius = top_radius
	mesh.bottom_radius = bottom_radius
	mesh.height = height
	mesh.radial_segments = 48
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	instance.material_override = _steel
	instance.transform = Transform3D(rotation, Frames.to_render(Vector3(center_station, 0.0, 0.0)))
	_stage_root.add_child(instance)


func _update_plumes(engines: Array, altitude_m: float) -> void:
	while _plumes.size() < engines.size():
		var plume := MeshInstance3D.new()
		var cone := CylinderMesh.new()
		cone.radial_segments = 16
		cone.rings = 1
		plume.mesh = cone
		plume.material_override = _plume_material
		plume.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		_stage_root.add_child(plume)
		_plumes.append(plume)
	# Plumes widen and lengthen as ambient pressure drops (under-expanded exhaust).
	var expansion := clampf(altitude_m / 60000.0, 0.0, 1.0)
	for i in engines.size():
		var engine: Dictionary = engines[i]
		var plume := _plumes[i]
		plume.visible = engine["firing"]
		if not plume.visible:
			continue
		var strength: float = engine["spool"] * (0.4 + 0.6 * engine["throttle"])
		var base_length := PLUME_VACUUM_LENGTH_M if engine["vacuum"] else PLUME_SEA_LEVEL_LENGTH_M
		var length := base_length * (0.6 + 1.4 * expansion) * strength
		var cone := plume.mesh as CylinderMesh
		cone.height = length
		cone.top_radius = 0.6 if not engine["vacuum"] else 1.1
		cone.bottom_radius = (1.0 + 6.0 * expansion) * (1.6 if engine["vacuum"] else 1.0)
		# Exhaust leaves opposite to the thrust direction, starting at the nozzle exit (~3 m below the gimbal).
		var direction: Vector3 = engine["direction"]
		var exhaust := Frames.to_render(-direction).normalized()
		var nozzle_exit: Vector3 = engine["position_m"] - direction * 3.0
		var center := Frames.to_render(nozzle_exit) + exhaust * length * 0.5
		plume.transform = Transform3D(_basis_from_y(-exhaust), center)


## Basis whose +Y axis points along `y` (CylinderMesh: top at +Y = nozzle side).
static func _basis_from_y(y: Vector3) -> Basis:
	var x := y.cross(Vector3.FORWARD if absf(y.z) < 0.9 else Vector3.RIGHT).normalized()
	return Basis(x, y, x.cross(y))
