# Headless test of the startup loading screen's logic (LoadingSteps, LoadingView, the no-screen path of LoadingScreen).
# Run: godot --headless --path game --script res://tests/loading_steps_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	var steps := LoadingSteps.new()
	_check(steps.current() == "" and steps.history().is_empty() and steps.count() == 0, "empty before the first step")
	steps.add("Eins", 1000)
	steps.add("Eins", 1500)  # the same step reported twice
	steps.add("", 1600)
	_check(steps.count() == 1 and steps.current() == "Eins", "repeated and empty texts are ignored")
	for i in range(2, 9):
		steps.add("Schritt %d" % i, 1000 * i)
	_check(steps.current() == "Schritt 8", "the newest step is the current one")
	var history := steps.history()
	_check(history.size() == LoadingSteps.HISTORY_MAX and history[0] == "Schritt 4" and history[-1] == "Schritt 7",
			"history: the last %d finished steps, oldest first (%s)" % [LoadingSteps.HISTORY_MAX, history])
	_check(steps.duration_ms(0, 9000) == 1000 and steps.duration_ms(7, 9500) == 1500,
			"durations: to the next step; the running one up to now")
	steps.finish(10000)
	steps.add("Nach dem Ende", 11000)
	_check(steps.finished and steps.current() == "Schritt 8" and steps.duration_ms(7, 99999) == 2000,
			"after finish() nothing is added and the last step ends there")
	_check(steps.report_lines(10000).size() == steps.count(), "one report line per step")

	var view := LoadingView.new()
	steps.title = "Titel"
	view.show_steps(steps)
	var labels := view.find_children("*", "Label", false, false)
	var texts: Array[String] = []
	for label: Label in labels:
		texts.append(label.text)
	_check(texts.has("Titel") and texts.has("Schritt 8") and texts.has("Schritt 7") and texts.has("Schritt 4"),
			"view: title, current and history rows filled (%s)" % [texts])
	view.free()

	# Without a screen (tests, headless): reporting is a no-op, also when awaited.
	_check(LoadingScreen.start(root) == null and not LoadingScreen.yielding(), "headless: no loading screen")
	var frames := Engine.get_process_frames()
	await LoadingScreen.step("ignoriert")
	_check(Engine.get_process_frames() == frames, "step() without a screen does not suspend the caller")

	# The boot scene's hand-over: Main becomes the current scene, the boot scene goes, a screen reparented to the root stays.
	var boot := Node.new()
	root.add_child(boot)
	var kept := Node.new()
	boot.add_child(kept)
	kept.reparent(root)
	var main := Node.new()
	main.name = "FakeMain"
	Boot.switch_to(self, main, boot)
	await process_frame
	_check(current_scene == main and main.get_parent() == root, "boot: the main scene is the current scene")
	_check(not is_instance_valid(boot) and is_instance_valid(kept), "boot: the boot scene is freed, the reparented node stays")
	main.free()
	kept.free()

	await _check_vr_lifecycle()
	await _check_vr_timeout()

	print("loading_steps_test: ", "FAILED" if _failures > 0 else "OK")
	quit(1 if _failures > 0 else 0)


## Stand-in for the XrManager autoload: what the VR screen uses (origin, camera; no OpenXR interface).
class FakeXr extends Node:
	var origin := XROrigin3D.new()
	var camera := Camera3D.new()
	var interface: Object = null


const CULL_MASK := 0x5F


func _fake_xr() -> FakeXr:
	var xr := FakeXr.new()
	root.add_child(xr)
	xr.add_child(xr.origin)
	xr.origin.add_child(xr.camera)
	xr.camera.cull_mask = CULL_MASK
	xr.camera.environment = Environment.new()
	xr.camera.compositor = Compositor.new()
	return xr


func _wait_frames(n: int) -> void:
	for i in n:
		await process_frame


func _layers(xr: FakeXr) -> int:
	return xr.origin.find_children("*", "VrLoadingLayer", false, false).size()


func _camera_restored(xr: FakeXr, environment: Environment, compositor: Compositor) -> bool:
	return xr.camera.cull_mask == CULL_MASK and xr.camera.environment == environment \
			and xr.camera.compositor == compositor


## The VR path of the boot (the user's 2026-10-30 hang): the boot scene's screen survives the reparent to the root,
## Main gets the same screen (no second quad that blanks the camera again), the steps run, and after arming the camera
## gets its world back and the quad and the screen are freed.
func _check_vr_lifecycle() -> void:
	var xr := _fake_xr()
	var environment := xr.camera.environment
	var compositor := xr.camera.compositor
	var boot := Node.new()
	root.add_child(boot)
	var screen := LoadingScreen.create(boot, LoadingScreen.Mode.VR, xr)
	_check(LoadingScreen.yielding() and _layers(xr) == 1 and xr.camera.cull_mask == 0
			and xr.camera.environment != environment and xr.camera.compositor != compositor,
			"vr: the screen builds one quad and blanks the XR camera")
	var frames := Engine.get_process_frames()
	await LoadingScreen.step("Hauptszene laden")
	_check(Engine.get_process_frames() > frames, "vr: a step waits for a rendered frame")
	screen.reparent(root)  # the boot scene's hand-over
	boot.free()
	var main := Node.new()
	root.add_child(main)
	_check(LoadingScreen.current == screen and LoadingScreen.start(main) == screen and _layers(xr) == 1,
			"vr: after the reparent Main gets the same screen, still one quad")
	await LoadingScreen.step("SPICE-Kernel laden")
	var hold := ["Atmosphäre wird geladen"]
	screen.finish_when_idle(func() -> String: return hold[0])
	await _wait_frames(3)
	_check(is_instance_valid(screen) and screen.steps.current() == hold[0] and xr.camera.cull_mask == 0,
			"vr: armed, the screen holds while the atmosphere loads (camera still blank)")
	hold[0] = ""
	await _wait_frames(LoadingScreen.READY_FRAMES + 4)
	_check(_camera_restored(xr, environment, compositor), "vr: after the boot the XR camera is restored")
	_check(not is_instance_valid(screen) and LoadingScreen.current == null and _layers(xr) == 0,
			"vr: the screen and its quad are freed")
	main.free()
	xr.free()


## Safety: an armed screen that keeps waiting is removed after the timeout and gives the camera back.
func _check_vr_timeout() -> void:
	var xr := _fake_xr()
	var environment := xr.camera.environment
	var compositor := xr.camera.compositor
	var screen := LoadingScreen.create(root, LoadingScreen.Mode.VR, xr)
	await LoadingScreen.step("Oberfläche aufbauen")
	screen.armed_timeout_ms = 50
	screen.finish_when_idle(func() -> String: return "hängt")
	var started := Time.get_ticks_msec()
	while is_instance_valid(screen) and Time.get_ticks_msec() - started < 2000:
		await process_frame
	_check(not is_instance_valid(screen) and LoadingScreen.current == null and _layers(xr) == 0
			and _camera_restored(xr, environment, compositor),
			"vr: timeout removes the stuck screen and restores the camera")
	xr.free()


func _check(ok: bool, what: String) -> void:
	print(("ok   " if ok else "FAIL ") + what)
	if not ok:
		_failures += 1
