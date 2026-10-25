class_name MapLabel
extends MeshInstance3D
## Screen-space label for the map view, where bodies are only a few pixels large: the look of the former Label3D
## (billboard, fixed screen size, drawn over everything, offset above the body, a black outline) as a TextMesh with
## shaders/map_label.gdshader, whose display-referred colour passes night vision's rod step unchanged (Label3D's 8-bit
## vertex colours cannot carry that inverse).

const SHADER := preload("res://shaders/map_label.gdshader")
## Label3D's former settings: pixel size (m per font pixel at unit depth), font size (Label3D's default), offset (px).
const PIXEL_SIZE := 0.0012
const FONT_SIZE := 32
const OFFSET_PX := Vector2(0, 22)
## [call] The outline (night-vision review M2: Label3D draws a 12 px black outline by default, outline_size 12 = a
## 6 px rim at this font size; a TextMesh has none): black copies of the text shifted by this many font pixels in
## OUTLINE_DIRECTIONS directions, drawn before the text.
const OUTLINE_PX := 2.5
const OUTLINE_DIRECTIONS := 8


func setup(text: String, colour: Color) -> void:
	mesh = _text_mesh(text, OFFSET_PX)
	material_override = _material(colour, 1)
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	var outline_material := _material(Color(0, 0, 0, colour.a), 0)
	for k in range(OUTLINE_DIRECTIONS):
		var angle := TAU * k / OUTLINE_DIRECTIONS
		var copy := MeshInstance3D.new()
		copy.mesh = _text_mesh(text, OFFSET_PX + Vector2(cos(angle), sin(angle)) * OUTLINE_PX)
		copy.material_override = outline_material
		copy.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		add_child(copy)


static func _text_mesh(text: String, offset: Vector2) -> TextMesh:
	var text_mesh := TextMesh.new()
	text_mesh.text = text
	text_mesh.font_size = FONT_SIZE
	text_mesh.pixel_size = PIXEL_SIZE
	text_mesh.depth = 0.0
	text_mesh.offset = offset
	return text_mesh


## render_priority: the outline (0) under the text (1); both draw over everything (depth test off).
static func _material(colour: Color, priority: int) -> ShaderMaterial:
	var material := ShaderMaterial.new()
	material.shader = SHADER
	material.set_shader_parameter("colour", colour)
	material.render_priority = priority
	return material
