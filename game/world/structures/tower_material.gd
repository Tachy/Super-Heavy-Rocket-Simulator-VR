class_name TowerMaterial
extends RefCounted
## Materials of the Pad B models: the tower's steel (shaders/tower_steel.gdshader: CC0 ambientCG Metal032 colour map from
## data/materials/metal032, fetched by `python tools/data_pipeline/fetch.py ambientcg_metal032`, plus procedural grime;
## missing texture: plain tinted steel), the dark decks / column bundle and the chopstick's matte black paint.

const SHADER := preload("res://shaders/tower_steel.gdshader")
const TEXTURE_FILE := "materials/metal032/Metal032_1K-JPG_Color.jpg"
## [E] Tower steel, user ruling 2026-10-31 on the real frames (vehicle-models.md "Black parts", real-frames item 11): the
## legs are bare steel of linear albedo ~0.15 (+-0.08), bright in direct sun and near black in shade; the braces, decks
## and platforms are dark, linear albedo ~0.04. The tints below give those means with the texture (linear mean
## (0.205, 0.248, 0.294), `tools` check 2026-10-31: tint = albedo / 0.91 grime / texture mean) and are sRGB values for the
## shader's source_color uniform. Without the texture the plain colours are the albedos themselves.
const LEG_TINT := Color(0.908, 0.834, 0.774)
const BRACE_TINT := Color(0.500, 0.458, 0.423)
const PLAIN_LEG := Color(0.42, 0.42, 0.42)
const PLAIN_BRACE := Color(0.22, 0.22, 0.22)
const DARK := Color(0.035, 0.038, 0.042)
## [E] Light-grey plinth, cladding plates and roof block: linear albedo ~0.30 (frames: light-grey plates, lit grey in sun).
const CLAD := Color(0.58, 0.58, 0.57)
## [E] chopstick arms and carriage are painted matte black (padb_chopsticks_around_booster_top_2026-03).
const BLACK_PAINT := Color(0.035, 0.035, 0.04)

static var _texture: Texture2D
static var _texture_tried := false


## Bare steel of the legs.
static func steel() -> ShaderMaterial:
	return _steel_material(LEG_TINT, PLAIN_LEG)


## Dark steel of the braces.
static func brace_steel() -> ShaderMaterial:
	return _steel_material(BRACE_TINT, PLAIN_BRACE)


static func clad() -> ShaderMaterial:
	return LitAerial.material(CLAD, 0.7, 0.3)


## The material set of LatticeBuilder.build.
static func lattice_set() -> Dictionary:
	return {"leg": steel(), "brace": brace_steel(), "dark": dark(), "clad": clad()}


static func _steel_material(textured_tint: Color, plain: Color) -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = SHADER
	var texture := _steel_texture()
	m.set_shader_parameter("use_texture", texture != null)
	if texture != null:
		m.set_shader_parameter("steel_albedo", texture)
		m.set_shader_parameter("tint", textured_tint)
	else:
		m.set_shader_parameter("tint", plain)
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
