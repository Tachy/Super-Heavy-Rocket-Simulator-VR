# Visual check of the loading screen's layout (needs a GPU, no --headless): renders the LoadingView with sample steps
# to screenshots/loading_screen/loading_view.png (repo root).
# Run: godot --path game --script res://tests/loading_view_shot.gd
extends SceneTree

const OUT_DIR := "res://../screenshots/loading_screen"

var _viewport := SubViewport.new()
var _frames := 0


func _initialize() -> void:
	var steps := LoadingSteps.new()
	steps.title = str(ProjectSettings.get_setting("application/config/name"))
	for text in ["SPICE-Kernel (Ephemeriden, Erdorientierung) laden", "Startplätze und Fahrzeugdaten laden",
			"Himmel und Atmosphäre vorbereiten", "Wetterdienst und Wolken einrichten", "Erdtextur laden",
			"Erdgelände und Satellitenbilder einrichten"]:
		steps.add(text, 0)
	var view := LoadingView.new()
	view.show_steps(steps)
	_viewport.size = Vector2i(1920, 1080)
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_viewport.add_child(view)
	root.add_child(_viewport)


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames < 3:
		return false
	var dir := ProjectSettings.globalize_path(OUT_DIR).simplify_path()
	DirAccess.make_dir_recursive_absolute(dir)
	var error := _viewport.get_texture().get_image().save_png(dir.path_join("loading_view.png"))
	print("saved ", dir, " (", error, ")")
	return true
