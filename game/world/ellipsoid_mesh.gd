class_name EllipsoidMesh
## Builds a UV ellipsoid in body-fixed coordinates (mapped to render axes) for equirectangular
## textures: u = (longitude + 180 deg) / 360 deg, v = (90 deg - latitude) / 180 deg.
## The seam column is duplicated so texture coordinates never wrap inside a triangle.


static func build(semi_major_axis_m: float, flattening: float, lon_segments := 256, lat_segments := 128) -> ArrayMesh:
	var a := semi_major_axis_m
	var b := semi_major_axis_m * (1.0 - flattening)
	var e2 := flattening * (2.0 - flattening)

	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs := PackedVector2Array()
	var indices := PackedInt32Array()

	for i in range(lat_segments + 1):
		var v := float(i) / lat_segments
		var lat := PI * (0.5 - v)  # geodetic latitude
		var sin_lat := sin(lat)
		var cos_lat := cos(lat)
		var n := a / sqrt(1.0 - e2 * sin_lat * sin_lat)
		for j in range(lon_segments + 1):
			var u := float(j) / lon_segments
			var lon := TAU * u - PI
			var fixed := Vector3(n * cos_lat * cos(lon), n * cos_lat * sin(lon), n * (1.0 - e2) * sin_lat)
			var normal := Vector3(cos_lat * cos(lon), cos_lat * sin(lon), sin_lat)  # geodetic normal
			vertices.append(Frames.to_render(fixed))
			normals.append(Frames.to_render(normal))
			uvs.append(Vector2(u, v))

	var row := lon_segments + 1
	for i in range(lat_segments):
		for j in range(lon_segments):
			var k := i * row + j
			indices.append_array([k, k + 1, k + row, k + 1, k + row + 1, k + row])  # clockwise = front in Godot

	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	mesh.custom_aabb = AABB(-Vector3.ONE * a * 1.01, Vector3.ONE * a * 2.02)
	return mesh
