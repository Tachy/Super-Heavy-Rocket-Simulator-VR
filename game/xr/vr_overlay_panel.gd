class_name VrOverlayPanel
extends MeshInstance3D
## VR display of the webcast overlay: a sphere section centred on the eye (the XROrigin3D, i.e. the base pose), facing
## -Z. Child of XrManager.origin, never of the XRCamera3D: the panel stays put when the head turns and follows the
## orbit camera's base pose only. The overlay texture holds premultiplied alpha, hence premultiplied blending.

const RADIUS_M := 2.0
const H_FOV_DEG := 64.0  ## [E] fits the Pimax / Quest comfort zone; tune in the VR session
const ASPECT := 16.0 / 9.0  ## the overlay viewport's aspect (1600x900)
const SEGMENTS_H := 64
const SEGMENTS_V := 36


func _init() -> void:
	name = "VrOverlayPanel"
	mesh = build_mesh()
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	# Camera-fixed content: never culled by the (mesh-centred) bounding box when the head sits off the origin.
	extra_cull_margin = RADIUS_M


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


static func build_material(texture: Texture2D) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.blend_mode = BaseMaterial3D.BLEND_MODE_PREMULT_ALPHA
	material.no_depth_test = true
	material.cull_mode = BaseMaterial3D.CULL_BACK
	material.render_priority = RenderingServer.MATERIAL_RENDER_PRIORITY_MAX
	material.albedo_texture = texture
	# The SubViewport texture has no mipmaps (a mipmap filter would sample a missing chain); the panel is drawn near its
	# native resolution, so plain bilinear filtering keeps the MSDF text sharp.
	material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR
	return material
