class_name TrajectoryView
extends MeshInstance3D
## Planned trajectory (target orbit of the guidance, green). GCRF samples are refreshed periodically and placed relative
## to the camera every frame with the same scaled-space mapping as the bodies. The coasting prediction of the current
## state (orange) is not drawn (user 2026-10-23): re-predicted every refresh under thrust, it jumped constantly.
## Drawn as a ribbon of constant pixel width (res://shaders/orbit_ribbon.gdshader): depth-tested, cut behind the Earth's
## limb and faded where the line of sight grazes the atmosphere (user 2026-10-23: the 1-pixel line looked stepped and was
## drawn over the Earth and the vehicles).

const REFRESH_S := 0.5
const SAMPLES := 720  ## 0.5 deg per segment on a circular orbit: no visible corners at the ribbon's width
## [E] fade band above the surface for lines of sight through the atmosphere: the Karman line (100 km), where the
## visible limb haze ends
const ATMOSPHERE_FADE_HEIGHT_M := 100000.0
const SHADER := preload("res://shaders/orbit_ribbon.gdshader")

var _planned := PackedVector3Array()
var _age := INF
var _mesh := ArrayMesh.new()
var _material := ShaderMaterial.new()


func _ready() -> void:
	mesh = _mesh
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_material.shader = SHADER
	_material.set_shader_parameter("earth_radius_m", World.EARTH_MEAN_RADIUS_M)
	_material.set_shader_parameter("atmosphere_height_m", ATMOSPHERE_FADE_HEIGHT_M)


func update_view(sim: FlightSim, camera_gcrf_m: Vector3, delta: float) -> void:
	_mesh.clear_surfaces()
	if not sim.is_active():
		return
	_age += delta
	if _age >= REFRESH_S:
		_age = 0.0
		_planned = sim.target_orbit_points(SAMPLES)
	_material.set_shader_parameter("earth_center_world", Frames.to_render(-camera_gcrf_m))  # GCRF is Earth-centred
	_draw_ribbon(_planned, camera_gcrf_m)


## Two triangles per segment; each vertex carries a point further along its segment (CUSTOM0), its true distance and its
## side (UV) for the shader. The orbit is closed with a last segment back to the first sample.
func _draw_ribbon(points: PackedVector3Array, camera_gcrf_m: Vector3) -> void:
	if points.size() < 2:
		return
	var placed := PackedVector3Array()
	var distances := PackedFloat32Array()
	for p in points:
		var offset := Frames.to_render(p - camera_gcrf_m)
		var distance := offset.length()
		distances.append(distance)
		if distance > CelestialBody.SCALED_SPACE_DISTANCE_M:
			offset *= CelestialBody.SCALED_SPACE_DISTANCE_M / distance
		placed.append(offset)
	var vertices := PackedVector3Array()
	var ahead := PackedFloat32Array()
	var uvs := PackedVector2Array()
	var count := placed.size()
	for i in count:
		var j := (i + 1) % count
		var segment := placed[j] - placed[i]
		# corners: start left, start right, end left, end right; "ahead" always points along the segment
		for corner in [[i, -1.0], [i, 1.0], [j, -1.0], [j, 1.0]]:
			var k: int = corner[0]
			vertices.append(placed[k])
			var along := placed[k] + segment
			ahead.append_array([along.x, along.y, along.z, 0.0])
			uvs.append(Vector2(distances[k], corner[1]))
	var indices := PackedInt32Array()
	for s in count:
		var b := 4 * s
		indices.append_array([b, b + 1, b + 2, b + 1, b + 3, b + 2])
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_CUSTOM0] = ahead
	arrays[Mesh.ARRAY_INDEX] = indices
	_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays, [], {},
			Mesh.ARRAY_CUSTOM_RGBA_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM0_SHIFT)
	_mesh.surface_set_material(0, _material)
