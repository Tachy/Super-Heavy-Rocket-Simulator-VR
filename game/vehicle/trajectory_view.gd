class_name TrajectoryView
extends MeshInstance3D
## Planned (target orbit of the guidance, green) and predicted (current state coasting, orange)
## trajectories. GCRF samples are refreshed periodically and placed relative to the camera every frame
## with the same scaled-space mapping as the bodies.

const REFRESH_S := 0.5
const PREDICTION_S := 6000.0
const STEP_S := 15.0

var _predicted := PackedVector3Array()
var _planned := PackedVector3Array()
var _age := INF
var _immediate := ImmediateMesh.new()
var _predicted_material := StandardMaterial3D.new()
var _planned_material := StandardMaterial3D.new()


func _ready() -> void:
	mesh = _immediate
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	for pair in [[_predicted_material, Color(1.0, 0.45, 0.15)], [_planned_material, Color(0.3, 1.0, 0.45)]]:
		var material: StandardMaterial3D = pair[0]
		material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		material.albedo_color = pair[1]
		material.no_depth_test = true
		material.render_priority = 10


func update_view(sim: FlightSim, camera_gcrf_m: Vector3, delta: float) -> void:
	_immediate.clear_surfaces()
	if not sim.is_active():
		return
	_age += delta
	if _age >= REFRESH_S:
		_age = 0.0
		_predicted = sim.predict_trajectory(sim.controlled_index(), PREDICTION_S, STEP_S)
		_planned = sim.target_orbit_points(180)
	_draw_line(_planned, camera_gcrf_m, _planned_material)
	_draw_line(_predicted, camera_gcrf_m, _predicted_material)


func _draw_line(points: PackedVector3Array, camera_gcrf_m: Vector3, material: Material) -> void:
	if points.size() < 2:
		return
	_immediate.surface_begin(Mesh.PRIMITIVE_LINE_STRIP, material)
	for p in points:
		var offset := Frames.to_render(p - camera_gcrf_m)
		var distance := offset.length()
		if distance > CelestialBody.SCALED_SPACE_DISTANCE_M:
			offset *= CelestialBody.SCALED_SPACE_DISTANCE_M / distance
		_immediate.surface_add_vertex(offset)
	_immediate.surface_end()
