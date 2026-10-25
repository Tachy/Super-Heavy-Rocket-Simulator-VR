class_name MapLabel
extends MeshInstance3D
## Screen-space label for the map view, where bodies are only a few pixels large: the look of the former Label3D
## (billboard, fixed screen size, drawn over everything, offset above the body) as a TextMesh with
## shaders/map_label.gdshader, whose display-referred colour passes night vision's rod step unchanged (Label3D's 8-bit
## vertex colours cannot carry that inverse).

const SHADER := preload("res://shaders/map_label.gdshader")
## Label3D's former settings: pixel size (m per font pixel at unit depth), font size (Label3D's default), offset (px).
const PIXEL_SIZE := 0.0012
const FONT_SIZE := 32
const OFFSET_PX := Vector2(0, 22)


func setup(text: String, colour: Color) -> void:
	var text_mesh := TextMesh.new()
	text_mesh.text = text
	text_mesh.font_size = FONT_SIZE
	text_mesh.pixel_size = PIXEL_SIZE
	text_mesh.depth = 0.0
	text_mesh.offset = OFFSET_PX
	mesh = text_mesh
	var material := ShaderMaterial.new()
	material.shader = SHADER
	material.set_shader_parameter("colour", colour)
	material_override = material
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
