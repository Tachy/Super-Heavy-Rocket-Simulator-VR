class_name VrOverlayPanel
extends MeshInstance3D
## VR display of the webcast overlay and the menus (pattern of the sibling's VrEditorHost): a sphere section centred on
## the eye (the XROrigin3D, i.e. the base pose), facing -Z. Child of XrManager.origin, never of the XRCamera3D: the
## panel stays put when the head turns and follows the orbit camera's base pose only. A mesh in the scene, so the
## desktop window's mirror shows it too; res://shaders/vr_panel.gdshader keeps the near plane from clipping it.

const RADIUS_M := 2.0
const H_FOV_DEG := 64.0  ## [E] fits the Pimax / Quest comfort zone; tune in the VR session
const ASPECT := 16.0 / 9.0  ## the overlay viewport's aspect (1600x900)
const SEGMENTS_H := 64
const SEGMENTS_V := 36
## Godot's largest cull margin: the CPU frustum cull uses the camera's near plane (up to 1 km), which the panel at 2 m
## lies in front of; the shader draws it regardless.
const CULL_MARGIN_M := 16384.0
const SHADER := preload("res://shaders/vr_panel.gdshader")


func _init() -> void:
	name = "VrOverlayPanel"
	mesh = build_mesh()
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	extra_cull_margin = CULL_MARGIN_M


## Centre the panel on the head pose of the last recentre (origin space): its world pose is then the base pose,
## i.e. in front of the head at the render origin (world = vr_origin_transform(base, recenter) * recenter = base).
func place(recenter_pose: Transform3D) -> void:
	transform = recenter_pose


func setup(texture: ViewportTexture) -> void:
	material_override = build_material(texture)


## Sphere section around the origin: azimuth +-H_FOV/2, elevation from the 16:9 span, front faces towards the origin.
static func build_mesh() -> ArrayMesh:
	var half_h := deg_to_rad(H_FOV_DEG) / 2.0
	var half_v := half_h / ASPECT
	var vertices := PackedVector3Array()
	var uvs := PackedVector2Array()
	var indices := PackedInt32Array()
	for j in SEGMENTS_V + 1:
		var v := float(j) / SEGMENTS_V
		var elevation_rad := lerpf(half_v, -half_v, v)
		for i in SEGMENTS_H + 1:
			var u := float(i) / SEGMENTS_H
			var azimuth_rad := lerpf(-half_h, half_h, u)
			vertices.append(RADIUS_M * Vector3(sin(azimuth_rad) * cos(elevation_rad), sin(elevation_rad),
					-cos(azimuth_rad) * cos(elevation_rad)))
			uvs.append(Vector2(u, v))
	var stride := SEGMENTS_H + 1
	for j in SEGMENTS_V:
		for i in SEGMENTS_H:
			var top_left := j * stride + i
			var bottom_left := top_left + stride
			# Clockwise seen from the origin = front face in Godot.
			indices.append_array([top_left, top_left + 1, bottom_left + 1, top_left, bottom_left + 1, bottom_left])
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_INDEX] = indices
	var array_mesh := ArrayMesh.new()
	array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return array_mesh


static func build_material(texture: Texture2D) -> ShaderMaterial:
	var material := ShaderMaterial.new()
	material.shader = SHADER
	material.render_priority = RenderingServer.MATERIAL_RENDER_PRIORITY_MAX
	# The SubViewport texture has no mipmaps (the shader samples it with plain bilinear filtering, which keeps the MSDF
	# text sharp at the panel's near-native resolution).
	material.set_shader_parameter("overlay", texture)
	return material
