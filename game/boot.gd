class_name Boot
extends Node
## The project's start scene (`run/main_scene`): a few lines that put the loading screen up at once and then bring in
## `main.tscn`. Creating the Main node blocks this thread for a moment (the cloud compute shaders are built on
## workers since 2026-10-30; it took about 20 s before), so the screen is visible about a second after the process start
## and says what happens ("Hauptszene laden", "Hauptszene aufbauen"). The later boot steps (World._ready,
## Main._ready) report into the same screen, which moves to the root when Main takes over.
## Tests and tools that load main.tscn themselves (--script) never see this scene; without a screen (headless,
## --self-check, --no-loading-screen) it just loads and switches.

const MAIN_SCENE := "res://main.tscn"
const STEP_LOAD := "Hauptszene laden"
const STEP_BUILD := "Hauptszene aufbauen"


func _ready() -> void:
	var started_ms := Time.get_ticks_msec()
	var screen := LoadingScreen.start(self)
	var packed: PackedScene
	if screen == null:
		packed = load(MAIN_SCENE)
	else:
		await LoadingScreen.step(STEP_LOAD)
		packed = await _load_threaded(MAIN_SCENE)
		await LoadingScreen.step(STEP_BUILD)  # the next call blocks this thread: say so first
	if packed == null:
		push_error("Boot: %s could not be loaded" % MAIN_SCENE)
		get_tree().quit(1)
		return
	var main := packed.instantiate()
	if screen != null:
		print("[Boot] main scene created %.2f s after the boot scene started" % ((Time.get_ticks_msec() - started_ms) / 1000.0))
		screen.reparent(get_tree().root)  # survives this scene
	switch_to(get_tree(), main, self)


## Makes `main` the current scene and frees `boot`. Public for the test.
static func switch_to(tree: SceneTree, main: Node, boot: Node) -> void:
	tree.root.add_child.call_deferred(main)  # the root is still busy setting up Boot when this runs synchronously
	tree.set_deferred("current_scene", main)
	boot.queue_free()


## Loads a resource on a loader thread and polls it each frame (the frames keep the loading screen alive). Null on error.
func _load_threaded(path: String) -> Resource:
	if ResourceLoader.load_threaded_request(path) != OK:
		return load(path)
	while true:
		var status := ResourceLoader.load_threaded_get_status(path)
		if status == ResourceLoader.THREAD_LOAD_LOADED:
			return ResourceLoader.load_threaded_get(path)
		if status != ResourceLoader.THREAD_LOAD_IN_PROGRESS:
			return null
		await get_tree().process_frame
	return null
