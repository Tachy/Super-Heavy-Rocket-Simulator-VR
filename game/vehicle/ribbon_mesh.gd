class_name RibbonMesh
## Builds the triangles of the line shaders (res://shaders/orbit_ribbon.gdshader, trail_ribbon.gdshader): two triangles
## per segment, each vertex carrying a point further along its segment (CUSTOM0), the side of the ribbon (UV.y), the
## true distance (UV.x, camera-relative lines only) and a length along the line (UV2.x, for the trail's fade-in).


## Camera-relative line from GCRF points, placed with the same scaled-space mapping as the bodies (rebuilt every frame).
static func add(mesh: ArrayMesh, points_gcrf: PackedVector3Array, camera_gcrf_m: Vector3, closed: bool,
		material: Material) -> void:
	var placed := PackedVector3Array()
	placed.resize(points_gcrf.size())
	var distances := PackedFloat32Array()
	distances.resize(points_gcrf.size())
	for i in points_gcrf.size():
		var offset := Frames.to_render(points_gcrf[i] - camera_gcrf_m)
		var distance := offset.length()
		distances[i] = distance
		if distance > CelestialBody.SCALED_SPACE_DISTANCE_M:
			offset *= CelestialBody.SCALED_SPACE_DISTANCE_M / distance
		placed[i] = offset
	_build(mesh, placed, distances, PackedFloat32Array(), closed, material)


## Line in the shader's own frame (the trail: Earth-fixed metres; the shader places it), open, with the length along
## the line per point.
static func add_local(mesh: ArrayMesh, points: PackedVector3Array, path_m: PackedFloat32Array, material: Material) -> void:
	_build(mesh, points, PackedFloat32Array(), path_m, false, material)


static func _build(mesh: ArrayMesh, placed: PackedVector3Array, distances: PackedFloat32Array,
		path_m: PackedFloat32Array, closed: bool, material: Material) -> void:
	var count := placed.size()
	if count < 2:
		return
	var segments := count if closed else count - 1
	# Preallocated and filled by index (VR performance: the per-corner appends of a 720-segment orbit took ~4 ms).
	var vertices := PackedVector3Array()
	vertices.resize(4 * segments)
	var ahead := PackedFloat32Array()
	ahead.resize(16 * segments)  # RGBA float per corner; .w stays 0
	var uvs := PackedVector2Array()
	uvs.resize(4 * segments)
	var uv2s := PackedVector2Array()
	uv2s.resize(4 * segments)
	var indices := PackedInt32Array()
	indices.resize(6 * segments)
	var has_distances := not distances.is_empty()
	var has_path := not path_m.is_empty()
	for i in segments:
		var j := (i + 1) % count
		var start := placed[i]
		var end := placed[j]
		var segment := end - start
		var b := 4 * i
		# corners: start left, start right, end left, end right; "ahead" always points along the segment
		vertices[b] = start
		vertices[b + 1] = start
		vertices[b + 2] = end
		vertices[b + 3] = end
		var start_ahead := start + segment
		var end_ahead := end + segment
		for c in 2:
			var k := 4 * (b + c)
			ahead[k] = start_ahead.x
			ahead[k + 1] = start_ahead.y
			ahead[k + 2] = start_ahead.z
			k = 4 * (b + 2 + c)
			ahead[k] = end_ahead.x
			ahead[k + 1] = end_ahead.y
			ahead[k + 2] = end_ahead.z
		var start_distance := distances[i] if has_distances else 0.0
		var end_distance := distances[j] if has_distances else 0.0
		uvs[b] = Vector2(start_distance, -1.0)
		uvs[b + 1] = Vector2(start_distance, 1.0)
		uvs[b + 2] = Vector2(end_distance, -1.0)
		uvs[b + 3] = Vector2(end_distance, 1.0)
		var start_path := path_m[i] if has_path else 0.0
		var end_path := path_m[j] if has_path else 0.0
		uv2s[b] = Vector2(start_path, 0.0)
		uv2s[b + 1] = Vector2(start_path, 0.0)
		uv2s[b + 2] = Vector2(end_path, 0.0)
		uv2s[b + 3] = Vector2(end_path, 0.0)
		var t := 6 * i
		indices[t] = b
		indices[t + 1] = b + 1
		indices[t + 2] = b + 2
		indices[t + 3] = b + 1
		indices[t + 4] = b + 3
		indices[t + 5] = b + 2
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_TEX_UV2] = uv2s
	arrays[Mesh.ARRAY_CUSTOM0] = ahead
	arrays[Mesh.ARRAY_INDEX] = indices
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays, [], {},
			Mesh.ARRAY_CUSTOM_RGBA_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM0_SHIFT)
	mesh.surface_set_material(mesh.get_surface_count() - 1, material)
