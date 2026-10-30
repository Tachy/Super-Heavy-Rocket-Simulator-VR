# Visual check of the loading screen's layout (needs a GPU, no --headless): renders the LoadingView with sample steps
# at rest and at three moments of a scroll, next to the boot splash, into screenshots/loading_screen/ (repo root):
# loading_view_*.png (1600x900 each) and loading_view_sheet.png (splash + the four frames).
# Run: godot --path game --script res://tests/loading_view_shot.gd
extends SceneTree

const OUT_DIR := "res://../screenshots/loading_screen"
const SIZE := Vector2i(1600, 900)  ## the project's window size, like the splash
const TEXTS := ["SPICE-Kernel (Ephemeriden, Erdorientierung) laden", "Startplätze und Fahrzeugdaten laden",
		"Himmel und Atmosphäre vorbereiten", "Wetterdienst und Wolken einrichten", "Erdtextur laden",
		"Erdgelände und Satellitenbilder einrichten", "Startplätze und Fangtürme aufbauen"]
## (name, scroll position in 0..1 of the animation; -1 = at rest before the last step arrives)
const STAGES := [["rest", -1.0], ["scroll_25", 0.25], ["scroll_55", 0.55], ["scroll_85", 0.85]]

var _viewport := SubViewport.new()
var _view := LoadingView.new()
var _steps := LoadingSteps.new()
var _stage := 0
var _frames := 0
var _images: Array[Image] = []


func _initialize() -> void:
	_steps.title = GameTitle.DISPLAY
	for i in TEXTS.size() - 1:
		_steps.add(TEXTS[i], 0)
	_view.set_process(false)
	_viewport.size = SIZE
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_viewport.add_child(_view)
	root.add_child(_viewport)
	_view.show_steps(_steps)


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames == 1:
		return false
	if _frames == 2:
		_enter_stage()
		return false
	if _frames < 5:
		return false
	var dir := ProjectSettings.globalize_path(OUT_DIR).simplify_path()
	DirAccess.make_dir_recursive_absolute(dir)
	var image := _viewport.get_texture().get_image()
	_images.append(image)
	image.save_png(dir.path_join("loading_view_%s.png" % STAGES[_stage][0]))
	_stage += 1
	if _stage >= STAGES.size():
		_save_sheet(dir)
		return true
	_frames = 1
	return false


## The last step arrives at the first scrolling stage; the scroll then stands at the stage's fraction.
func _enter_stage() -> void:
	var fraction: float = STAGES[_stage][1]
	if fraction < 0.0:
		return
	if _stage == 1:
		_steps.add(TEXTS[-1], 0)
		_view.show_steps(_steps)
	_view._scroll = 1.0 - fraction
	_view._layout()


func _save_sheet(dir: String) -> void:
	var sheet := Image.create(SIZE.x, SIZE.y / 2 * 3, false, Image.FORMAT_RGB8)
	var tiles: Array[Image] = []
	var splash := Image.load_from_file(ProjectSettings.globalize_path("res://assets/boot_splash.png"))
	splash.convert(Image.FORMAT_RGB8)
	tiles.append(splash)
	tiles.append_array(_images)
	for i in tiles.size():
		var tile := tiles[i].duplicate() as Image
		tile.convert(Image.FORMAT_RGB8)
		tile.resize(SIZE.x / 2, SIZE.y / 2, Image.INTERPOLATE_LANCZOS)
		sheet.blit_rect(tile, Rect2i(Vector2i.ZERO, tile.get_size()), Vector2i((i % 2) * SIZE.x / 2, (i / 2) * SIZE.y / 2))
	print("saved ", dir, " (", sheet.save_png(dir.path_join("loading_view_sheet.png")), ")")
