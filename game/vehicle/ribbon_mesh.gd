class_name RibbonMesh
## Builds the triangles of the line shaders (res://shaders/orbit_ribbon.gdshader, trail_ribbon.gdshader): two triangles
## per segment, each vertex carrying a point further along its segment (CUSTOM0), the side of the ribbon (UV.y), the
## true distance (UV.x, camera-relative lines only) and a length along the line (UV2.x, for the trail's fade-in).


## Camera-relative line from GCRF points, placed with the same scaled-space mapping as the bodies (rebuilt every frame).
static func add(mesh: ArrayMesh, points_gcrf: PackedVector3Array, camera_gcrf_m: Vector3, closed: bool,
		material: Material) -> void:
	var placed := PackedVector3Array()
	var distances := PackedFloat32Array()
	for p in points_gcrf:
		var offset := Frames.to_render(p - camera_gcrf_m)
		var distance := offset.length()
		distances.append(distance)
		if distance > CelestialBody.SCALED_SPACE_DISTANCE_M:
			offset *= CelestialBody.SCALED_SPACE_DISTANCE_M / distance
		placed.append(offset)
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
	var vertices := PackedVector3Array()
	var ahead := PackedFloat32Array()
	var uvs := PackedVector2Array()
	var uv2s := PackedVector2Array()
	for i in segments:
		var j := (i + 1) % count
		var segment := placed[j] - placed[i]
		# corners: start left, start right, end left, end right; "ahead" always points along the segment
		for corner in [[i, -1.0], [i, 1.0], [j, -1.0], [j, 1.0]]:
			var k: int = corner[0]
			vertices.append(placed[k])
			var along := placed[k] + segment
			ahead.append_array([along.x, along.y, along.z, 0.0])
			uvs.append(Vector2(distances[k] if not distances.is_empty() else 0.0, corner[1]))
			uv2s.append(Vector2(path_m[k] if not path_m.is_empty() else 0.0, 0.0))
	var indices := PackedInt32Array()
	for s in segments:
		var b := 4 * s
		indices.append_array([b, b + 1, b + 2, b + 1, b + 3, b + 2])
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
