class_name CelestialBody
extends MeshInstance3D
## A planet or moon rendered at its true position relative to the floating origin (the camera).
##
## Scaled space: bodies farther than SCALED_SPACE_DISTANCE_M are drawn closer and proportionally
## smaller, preserving their exact angular size and direction. Lighting uses body-local uniforms in
## true metres, so it is unaffected by the scaling.

const SCALED_SPACE_DISTANCE_M := 1.5e7

var center_gcrf_m := Vector3.ZERO      ## body centre relative to the Earth's centre (GCRF)
var fixed_to_gcrf := Basis.IDENTITY    ## body orientation (rows = rotation matrix rows)
var render_basis := Basis.IDENTITY     ## same orientation in render axes
## Draws the ellipsoid mesh; false while a TerrainQuadtree child draws the surface (children stay visible).
var body_mesh_visible := true:
	set(value):
		if value != body_mesh_visible:
			body_mesh_visible = value
			mesh = _body_mesh if value else null

var _body_mesh: Mesh


func setup(semi_major_axis_m: float, flattening: float, shader_material: ShaderMaterial) -> void:
	_body_mesh = EllipsoidMesh.build(semi_major_axis_m, flattening)
	mesh = _body_mesh if body_mesh_visible else null
	material_override = shader_material
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF


## Places the body for a camera at `origin_gcrf_m` (floating origin).
func place(origin_gcrf_m: Vector3) -> void:
	render_basis = Frames.basis_to_render(fixed_to_gcrf)
	var offset := Frames.to_render(center_gcrf_m - origin_gcrf_m)
	var distance := offset.length()
	var scale_factor := 1.0
	if distance > SCALED_SPACE_DISTANCE_M:
		scale_factor = SCALED_SPACE_DISTANCE_M / distance
	transform = Transform3D(render_basis.scaled(Vector3.ONE * scale_factor), offset * scale_factor)


## GCRF vector (relative to this body's centre) -> body-local render frame for shader uniforms.
func to_local_frame(gcrf_vector_m: Vector3) -> Vector3:
	return render_basis.transposed() * Frames.to_render(gcrf_vector_m)
