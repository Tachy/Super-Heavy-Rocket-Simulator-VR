class_name TowerMaterial
extends RefCounted
## Materials of the Pad B models: the tower's steel (shaders/tower_steel.gdshader: CC0 ambientCG Metal032 colour map from
## data/materials/metal032, fetched by `python tools/data_pipeline/fetch.py ambientcg_metal032`, plus procedural grime;
## missing texture: plain tinted steel), the dark decks / column bundle and the chopstick's matte black paint.

const SHADER := preload("res://shaders/tower_steel.gdshader")
const TEXTURE_FILE := "materials/metal032/Metal032_1K-JPG_Color.jpg"
## [E] light galvanised steel of the tower (padb_tower_full_with_chopsticks_2025-08): the texture's mean is ~(0.49, 0.53,
## 0.58) sRGB, this lifts it to the photo's bright steel.
const STEEL_TINT := Color(1.25, 1.2, 1.15)
const PLAIN_STEEL := Color(0.62, 0.64, 0.67)
const DARK := Color(0.07, 0.075, 0.08)
## [E] chopstick arms and carriage are painted matte black (padb_chopsticks_around_booster_top_2026-03).
const BLACK_PAINT := Color(0.035, 0.035, 0.04)

static var _texture: Texture2D
static var _texture_tried := false


static func steel() -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = SHADER
	var texture := _steel_texture()
	m.set_shader_parameter("use_texture", texture != null)
	if texture != null:
		m.set_shader_parameter("steel_albedo", texture)
		m.set_shader_parameter("tint", STEEL_TINT)
	else:
		m.set_shader_parameter("tint", PLAIN_STEEL)
	return m


static func dark() -> ShaderMaterial:
	return LitAerial.material(DARK, 0.7, 0.3)


static func black_paint() -> ShaderMaterial:
	return LitAerial.material(BLACK_PAINT, 0.75, 0.2)


## The texture, or null when the data file is missing (loaded once; mipmaps for the far view).
static func _steel_texture() -> Texture2D:
	if _texture_tried:
		return _texture
	_texture_tried = true
	var path := Paths.data_dir().path_join(TEXTURE_FILE)
	if not FileAccess.file_exists(path):
		print("TowerMaterial: no %s: plain steel (run python tools/data_pipeline/fetch.py ambientcg_metal032)" % path)
		return null
	var image := Image.load_from_file(path)
	if image == null or image.is_empty():
		return null
	image.generate_mipmaps()
	_texture = ImageTexture.create_from_image(image)
	return _texture
