class_name StarField
extends MeshInstance3D
## Stars to magnitude 7.5 from the HYG catalogue (ICRS = GCRF axes) as a point cloud that always
## surrounds the camera. Proper motion and annual aberration (<= 20.5") are below one pixel here.

const STAR_DISTANCE_M := 2.5e7
const REFERENCE_MAGNITUDE := 1.0  # magnitude rendered at full brightness


func load_catalogue(csv_path: String) -> int:
	var file := FileAccess.open(csv_path, FileAccess.READ)
	if file == null:
		push_error("StarField: cannot open %s" % csv_path)
		return 0
	file.get_csv_line()  # header
	var vertices := PackedVector3Array()
	var colors := PackedColorArray()
	while not file.eof_reached():
		var row := file.get_csv_line()
		if row.size() < 4:
			continue
		var ra := deg_to_rad(row[0].to_float())
		var dec := deg_to_rad(row[1].to_float())
		var direction := Vector3(cos(dec) * cos(ra), cos(dec) * sin(ra), sin(dec))
		vertices.append(Frames.to_render(direction) * STAR_DISTANCE_M)
		var brightness := pow(10.0, -0.4 * (row[2].to_float() - REFERENCE_MAGNITUDE))
		var color := _color_from_bv(row[3].to_float())
		color.a = brightness
		colors.append(color)

	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_COLOR] = colors
	var point_mesh := ArrayMesh.new()
	point_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)
	point_mesh.custom_aabb = AABB(-Vector3.ONE * STAR_DISTANCE_M, Vector3.ONE * STAR_DISTANCE_M * 2.0)
	mesh = point_mesh
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/stars.gdshader")
	material_override = material
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	return vertices.size()


## Approximate stellar colour from the B-V colour index.
static func _color_from_bv(bv: float) -> Color:
	var blue := Color(0.62, 0.72, 1.0)
	var white := Color(1.0, 0.97, 0.92)
	var orange := Color(1.0, 0.72, 0.45)
	if bv < 0.4:
		return blue.lerp(white, clampf((bv + 0.3) / 0.7, 0.0, 1.0))
	return white.lerp(orange, clampf((bv - 0.4) / 1.2, 0.0, 1.0))
