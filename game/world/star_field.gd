class_name StarField
extends MeshInstance3D
## Stars to magnitude 7.5 from the HYG catalogue (ICRS = GCRF axes) as a point cloud that always
## surrounds the camera. Proper motion and annual aberration (<= 20.5") are below one pixel here.
## Each star carries its illuminance above the atmosphere (CUSTOM0); shaders/stars.gdshader attenuates it through the
## atmosphere and hides it below the eye's threshold over the sky.

const STAR_DISTANCE_M := 2.5e7
const REFERENCE_MAGNITUDE := 1.0  # magnitude rendered at full brightness
## Illuminance of a magnitude-0 star outside the atmosphere (lux) [S: zero point of the V magnitude,
## E = 2.54e-6 x 10^(-0.4 m) lux].
const V_ZERO_POINT_LUX := 2.54e-6


## Illuminance (lux) of a star of visual magnitude `magnitude` above the atmosphere.
static func illuminance_lux(magnitude: float) -> float:
	return V_ZERO_POINT_LUX * pow(10.0, -0.4 * magnitude)


func load_catalogue(csv_path: String) -> int:
	var file := FileAccess.open(csv_path, FileAccess.READ)
	if file == null:
		push_error("StarField: cannot open %s" % csv_path)
		return 0
	file.get_csv_line()  # header
	var directions := PackedVector3Array()
	var magnitudes := PackedFloat32Array()
	var color_indices := PackedFloat32Array()
	while not file.eof_reached():
		var row := file.get_csv_line()
		if row.size() < 4:
			continue
		var ra := deg_to_rad(row[0].to_float())
		var dec := deg_to_rad(row[1].to_float())
		directions.append(Vector3(cos(dec) * cos(ra), cos(dec) * sin(ra), sin(dec)))
		magnitudes.append(row[2].to_float())
		color_indices.append(row[3].to_float())
	build(directions, magnitudes, color_indices)
	return directions.size()


## Builds the point mesh: unit directions (GCRF), visual magnitudes and B-V colour indices, one entry per star.
func build(directions_gcrf: PackedVector3Array, magnitudes: PackedFloat32Array,
		color_indices: PackedFloat32Array) -> void:
	var vertices := PackedVector3Array()
	var colors := PackedColorArray()
	var illuminances := PackedFloat32Array()
	for i in range(directions_gcrf.size()):
		vertices.append(Frames.to_render(directions_gcrf[i]) * STAR_DISTANCE_M)
		colors.append(_color_from_bv(color_indices[i]))
		illuminances.append(illuminance_lux(magnitudes[i]))

	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_COLOR] = colors
	arrays[Mesh.ARRAY_CUSTOM0] = illuminances
	var point_mesh := ArrayMesh.new()
	point_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays, [], {},
			Mesh.ARRAY_CUSTOM_R_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM0_SHIFT)
	point_mesh.custom_aabb = AABB(-Vector3.ONE * STAR_DISTANCE_M, Vector3.ONE * STAR_DISTANCE_M * 2.0)
	mesh = point_mesh
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/stars.gdshader")
	material.set_shader_parameter("reference_illuminance_lux", illuminance_lux(REFERENCE_MAGNITUDE))
	material_override = material
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF


## Approximate stellar colour from the B-V colour index.
static func _color_from_bv(bv: float) -> Color:
	var blue := Color(0.62, 0.72, 1.0)
	var white := Color(1.0, 0.97, 0.92)
	var orange := Color(1.0, 0.72, 0.45)
	if bv < 0.4:
		return blue.lerp(white, clampf((bv + 0.3) / 0.7, 0.0, 1.0))
	return white.lerp(orange, clampf((bv - 0.4) / 1.2, 0.0, 1.0))
