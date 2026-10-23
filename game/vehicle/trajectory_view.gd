class_name TrajectoryView
extends MeshInstance3D
## Planned trajectory (target orbit of the guidance, green). GCRF samples are refreshed periodically and placed relative
## to the camera every frame with the same scaled-space mapping as the bodies. The coasting prediction of the current
## state (orange) is not drawn (user 2026-10-23): re-predicted every refresh under thrust, it jumped constantly.

const REFRESH_S := 0.5
var _planned := PackedVector3Array()
var _age := INF
var _immediate := ImmediateMesh.new()
var _planned_material := StandardMaterial3D.new()


func _ready() -> void:
	mesh = _immediate
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_planned_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_planned_material.albedo_color = Color(0.3, 1.0, 0.45)
	_planned_material.no_depth_test = true
	_planned_material.render_priority = 10


func update_view(sim: FlightSim, camera_gcrf_m: Vector3, delta: float) -> void:
	_immediate.clear_surfaces()
	if not sim.is_active():
		return
	_age += delta
	if _age >= REFRESH_S:
		_age = 0.0
		_planned = sim.target_orbit_points(180)
	_draw_line(_planned, camera_gcrf_m, _planned_material)


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
