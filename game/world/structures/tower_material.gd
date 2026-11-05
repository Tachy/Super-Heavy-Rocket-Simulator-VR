class_name TowerMaterial
extends RefCounted
## Materials of the Pad B models: the tower's steel (shaders/tower_steel.gdshader: CC0 ambientCG Metal032 colour map from
## data/materials/metal032, fetched by `python tools/data_pipeline/fetch.py ambientcg_metal032`, plus procedural grime;
## missing texture: plain tinted steel), the dark decks / column bundle and the chopstick's matte black paint.

const SHADER := preload("res://shaders/tower_steel.gdshader")
const TEXTURE_FILE := "materials/metal032/Metal032_1K-JPG_Color.jpg"
const NORMAL_FILE := "materials/metal032/Metal032_1K-JPG_NormalGL.jpg"
## [E] Tower steel after the Pad B webcast frames (user 2026-11-05 "nach Webcast-Bildern", tower-steel-frames.md): weathered
## galvanised steel, mid-grey slightly brown-warm. Legs: f14_2095 right leg sunlit mean sRGB (122, 104, 90) in low warm sun,
## f13_2040 / f12_1860 mid-grey with narrow bright edges; the sun's warm cast (about 1.2 : 1.0 : 0.85 at that elevation) divided
## out gives linear albedo ~(0.16, 0.14, 0.125). Braces: darker than the legs in every frame (f13_2040 crop_f13_tower, f14_2095
## truss pixels sRGB 37-56, lit faces to 90-100): linear ~0.06, the same warm tint. The tint IS the albedo (sRGB values of the
## shader's source_color uniform); the Metal032 map modulates it around its mean (`texture_contrast`). Rough and matte
## (user 2026-11-05, frames: no mirror highlight on any member): metallic 0 in the shader. Earlier cool silver legs 0.30 / braces
## 0.04 (photo nsf_padb_chopsticks_2025-02): docs/history/launch-structures.md.
const LEG_TINT := Color(0.44, 0.41, 0.385)
const BRACE_TINT := Color(0.28, 0.265, 0.25)
const LEG_CONTRAST := 0.25
const BRACE_CONTRAST := 0.5
const LEG_GRIME := 0.08
const BRACE_GRIME := 0.3
const PLAIN_LEG := LEG_TINT
const PLAIN_BRACE := BRACE_TINT
const DARK := Color(0.035, 0.038, 0.042)
## [E] Plinth, f14_2095 crop_f14_plinth: plate-clad steel, warm grey-tan, satin-matte; sunlit plates sRGB (104, 102, 95) mean
## in warm low sun, shaded faces 60-80, bright rims only on the ramp edge: linear albedo ~(0.17, 0.15, 0.12), metallic 0.
const PLINTH := Color(0.46, 0.43, 0.38)
## [E] Cladding plates on the pad-side face, f13_2040 (grey plates between the legs, lit grey in high sun, no sheen): neutral
## mid-grey, linear ~0.13, metallic 0.
const CLAD := Color(0.40, 0.39, 0.37)
## [E] QD arm: part of the dark truss in f13_2040 / f14_2095 (sRGB 37-56, lit faces to ~90), dark grey matte, linear ~0.04.
const ARM_GREY := Color(0.22, 0.22, 0.225)
## [E] Tower head, padb_tower_top_cabin_2026-04: weathered dark-grey plate wall, two tones so the panels read; light roof and
## bottom band; floor and beams dark steel (the black-paint default).
const HEAD_WALL_A := Color(0.22, 0.21, 0.20)
const HEAD_WALL_B := Color(0.17, 0.165, 0.16)
const HEAD_BAND := Color(0.30, 0.30, 0.30)
## [E] chopstick arms and carriage are painted matte black (padb_chopsticks_around_booster_top_2026-03).
const BLACK_PAINT := Color(0.035, 0.035, 0.04)

const MOUNT_SHADER := preload("res://shaders/mount_shell.gdshader")
const MOUNT_MAP := "models/starbase_olp2/mount_panels.png"
const CONCRETE_MAP := "models/starbase_olp2/pad_concrete.png"
const APRON_MASK_MAP := "models/starbase_olp2/apron_mask.png"
## [E] Mount plates without their generated map: grey to tan, linear (person-scale frame, images.jpeg).
const MOUNT_PLAIN := Color(0.34, 0.30, 0.23)
## [E] Hardstand without its generated map: light concrete, linear (images.jpeg).
const CONCRETE_PLAIN := Color(0.45, 0.43, 0.38)

static var _texture: Texture2D
static var _texture_tried := false
static var _normal: Texture2D
static var _normal_tried := false


## Bare steel of the legs.
static func steel() -> ShaderMaterial:
	return _steel_material(LEG_TINT, PLAIN_LEG, LEG_CONTRAST, LEG_GRIME)


## Dark steel of the braces.
static func brace_steel() -> ShaderMaterial:
	return _steel_material(BRACE_TINT, PLAIN_BRACE, BRACE_CONTRAST, BRACE_GRIME)


## Plates of the launch mount's faces: the generated map (data/models/starbase_olp2/mount_panels.png), tile in metres.
static func mount_shell(tile_m: float) -> Material:
	return _mapped(MOUNT_MAP, MOUNT_PLAIN, tile_m)


## Light concrete of the hardstand and the exit ramps: the generated map (pad_concrete.png), tile in metres.
static func pad_concrete(tile_m: float) -> Material:
	return _mapped(CONCRETE_MAP, CONCRETE_PLAIN, tile_m)


## Coverage map of the hardstand (apron_mask.png, terrain_earth.gdshader), null without the generated file.
static func apron_mask() -> Texture2D:
	var path := Paths.data_dir().path_join(APRON_MASK_MAP)
	var image := Image.load_from_file(path) if FileAccess.file_exists(path) else null
	if image == null or image.is_empty():
		return null
	return ImageTexture.create_from_image(image)


static func _mapped(map: String, plain: Color, tile_m: float) -> Material:
	var path := Paths.data_dir().path_join(map)
	var image := Image.load_from_file(path) if FileAccess.file_exists(path) else null
	if image == null or image.is_empty():
		return LitAerial.material(plain, 0.75, 0.15)
	image.generate_mipmaps()
	var m := ShaderMaterial.new()
	m.shader = MOUNT_SHADER
	m.set_shader_parameter("panels", ImageTexture.create_from_image(image))
	m.set_shader_parameter("tile_m", tile_m)
	return m


## Node -> material of the head's parts in the rails model.
static func head_set() -> Dictionary:
	var band := LitAerial.material(HEAD_BAND, 0.8, 0.0)
	return {"head_wall_a": LitAerial.material(HEAD_WALL_A, 0.85, 0.0), "head_wall_b": LitAerial.material(HEAD_WALL_B, 0.85, 0.0),
			"head_band": band, "head_roof": band}


static func clad() -> ShaderMaterial:
	return LitAerial.material(CLAD, 0.8, 0.0)


## Plate-clad plinth of the tower (its own material, apart from the cladding plates).
static func plinth() -> ShaderMaterial:
	return LitAerial.material(PLINTH, 0.8, 0.0)


## Dark grey matte steel of the QD arm.
static func arm_grey() -> ShaderMaterial:
	return LitAerial.material(ARM_GREY, 0.85, 0.0)


## The material set of LatticeBuilder.build.
static func lattice_set() -> Dictionary:
	return {"leg": steel(), "brace": brace_steel(), "dark": dark(), "clad": clad(), "plinth": plinth()}


static func _steel_material(textured_tint: Color, plain: Color, contrast: float, grime: float) -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = SHADER
	m.set_shader_parameter("texture_contrast", contrast)
	m.set_shader_parameter("grime_strength", grime)
	var normal := _steel_normal()
	m.set_shader_parameter("use_normal", normal != null)
	if normal != null:
		m.set_shader_parameter("steel_normal", normal)
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


## The Metal032 normal map (OpenGL convention), or null when the data file is missing (the steel is then smooth-shaded).
static func _steel_normal() -> Texture2D:
	if _normal_tried:
		return _normal
	_normal_tried = true
	var path := Paths.data_dir().path_join(NORMAL_FILE)
	if not FileAccess.file_exists(path):
		return null
	var image := Image.load_from_file(path)
	if image == null or image.is_empty():
		return null
	image.generate_mipmaps()
	_normal = ImageTexture.create_from_image(image)
	return _normal
