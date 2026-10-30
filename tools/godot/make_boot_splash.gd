# Renders the engine boot splash (game/assets/boot_splash.png): black, the game's name and "wird gestartet ..." in the
# loading screen's look. Needs a GPU (no --headless). Re-run when the name changes:
#   godot --path game --script res://../tools/godot/make_boot_splash.gd
extends SceneTree

const OUT := "res://assets/boot_splash.png"
const SIZE := Vector2i(1600, 900)  ## the project's window size (the splash is shown unscaled, centred)

var _viewport := SubViewport.new()
var _frames := 0


func _initialize() -> void:
	_viewport.size = SIZE
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	var background := ColorRect.new()
	background.color = LoadingView.BACKGROUND
	background.set_anchors_preset(Control.PRESET_FULL_RECT)
	_viewport.add_child(background)
	_add_row(str(ProjectSettings.get_setting("application/config/name")), LoadingView.CURRENT_SIZE,
			LoadingView.CURRENT_COLOR, -LoadingView.CURRENT_SIZE * 0.4)
	_add_row("wird gestartet ...", LoadingView.HISTORY_SIZE, LoadingView.TITLE_COLOR, LoadingView.CURRENT_SIZE * 0.6)
	root.add_child(_viewport)


func _add_row(text: String, font_size: int, color: Color, offset_y: float) -> void:
	var label := Label.new()
	label.text = text
	LoadingView._style(label, font_size, color)
	LoadingView._place_row(label, offset_y, font_size)
	_viewport.add_child(label)


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames < 3:
		return false
	var path := ProjectSettings.globalize_path(OUT)
	print("saved ", path, " (", _viewport.get_texture().get_image().save_png(path), ")")
	return true
