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

	print("loading_steps_test: ", "FAILED" if _failures > 0 else "OK")
	quit(1 if _failures > 0 else 0)


func _check(ok: bool, what: String) -> void:
	print(("ok   " if ok else "FAIL ") + what)
	if not ok:
		_failures += 1
