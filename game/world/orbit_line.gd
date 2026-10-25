class_name OrbitLine
extends MeshInstance3D
## The Moon's path over +-14 days around the current instant (map view). Samples are recomputed when
## time has moved on; every frame they are placed relative to the camera with the same scaled-space
## mapping as the bodies, so the line passes exactly through the rendered Moon.

const HALF_SPAN_S := 14.0 * 86400.0
const SAMPLES := 400
const RESAMPLE_AFTER_S := 3600.0

var _samples_gcrf := PackedVector3Array()
var _sampled_at := INF
var _immediate := ImmediateMesh.new()


func _ready() -> void:
	mesh = _immediate
	# A display-referred UI colour, past night vision's rod step unchanged (shaders/display_colour.gdshader).
	var material := ShaderMaterial.new()
	material.shader = preload("res://shaders/display_colour.gdshader")
	material.set_shader_parameter("colour", Color(0.45, 0.6, 0.9, 0.7))
	material_override = material
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF


func update_line(core: StarsimCore, tdb_s: float, camera_gcrf_m: Vector3) -> void:
	if absf(tdb_s - _sampled_at) > RESAMPLE_AFTER_S:
		_samples_gcrf.resize(SAMPLES + 1)
		for i in range(SAMPLES + 1):
			var t := tdb_s - HALF_SPAN_S + 2.0 * HALF_SPAN_S * i / SAMPLES
			_samples_gcrf[i] = core.body_position_gcrf_m("moon", t)
		_sampled_at = tdb_s

	_immediate.clear_surfaces()
	_immediate.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for p in _samples_gcrf:
		var offset := Frames.to_render(p - camera_gcrf_m)
		var distance := offset.length()
		if distance > CelestialBody.SCALED_SPACE_DISTANCE_M:
			offset *= CelestialBody.SCALED_SPACE_DISTANCE_M / distance
		_immediate.surface_add_vertex(offset)
	_immediate.surface_end()
