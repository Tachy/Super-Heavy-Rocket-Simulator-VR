class_name LoadingScreen
extends Node
## Startup loading screen: shows which boot step runs right now (light text on black, no progress bar). The boot code
## reports each step with `await LoadingScreen.step("Wetter laden ...")` (a no-op without a screen, so tests and
## headless runs never notice). Main creates it (`start`) before the world is built and arms it at the end
## (`finish_when_idle`); it disappears for good after the first rendered world frames.
##   Desktop: a full-screen CanvasLayer; the text reaches the window at once (RenderingServer.force_draw), so the
##            blocking boot steps stay synchronous.
##   VR:      a native OpenXR quad layer (VrLoadingLayer). Only rendered frames reach the headset, so a step waits one
##            frame after its text is set (`yielding()`: World._ready then suspends and Main waits for it).
## Every step is also logged with its start and duration (`[Boot] ...`, the table at the end).

enum Mode { NONE, DESKTOP, VR }

const DISABLE_ARG := "--no-loading-screen"  ## user argument: no screen and no boot log
const LAYER := 100  ## above every other canvas layer
const READY_FRAMES := 6  ## rendered world frames under the screen before it goes (the cloud shader warm-up takes 4)
const FADE_S := 0.3  ## desktop only
const SESSION_WAIT_MS := 4000  ## VR: how long the first step waits for the headset session to be visible

static var current: LoadingScreen

var steps := LoadingSteps.new()
var mode := Mode.NONE

var _view: LoadingView
var _layer: CanvasLayer  ## desktop
var _vr: VrLoadingLayer
var _xr: Node  ## the XrManager autoload
var _hold: Callable
var _armed := false
var _world_shown := false
var _frames_left := READY_FRAMES
var _placed := false
var _session_visible := false


## Creates the screen as a child of `host` (null result: none wanted - self-check, tests, headless). The boot scene
## starts it first; Main's call then returns that one.
static func start(host: Node) -> LoadingScreen:
	if current != null:
		return current
	var wanted := _mode_for(host)
	if wanted == Mode.NONE:
		return null
	var screen := LoadingScreen.new()
	screen.mode = wanted
	host.add_child(screen)
	screen._build()
	current = screen
	return screen


## Reports the step that starts now. A coroutine only in VR (one frame); elsewhere it returns at once.
static func step(text: String) -> void:
	if current != null:
		await current._report(text)


## True while steps wait for rendered frames (VR): the boot code must then be awaited.
static func yielding() -> bool:
	return current != null and current.mode == Mode.VR


## Which mode this start gets. NONE for headless runs, `--script` runs (tests), the self-check and the disable
## argument; VR only on a real OpenXR session (not the test harness' stand-in headset).
static func _mode_for(host: Node) -> Mode:
	var engine_args := OS.get_cmdline_args()
	var user_args := OS.get_cmdline_user_args()
	if DisplayServer.get_name() == "headless" or engine_args.has("--script") or engine_args.has("-s") \
			or user_args.has(DISABLE_ARG) or user_args.has("--self-check"):
		return Mode.NONE
	var xr := host.get_node_or_null("/root/XrManager")
	if xr != null and xr.xr_active:
		return Mode.VR if xr.interface != null and xr.interface.get_name() == "OpenXR" else Mode.NONE
	return Mode.DESKTOP


func _build() -> void:
	name = "LoadingScreen"
	process_mode = Node.PROCESS_MODE_ALWAYS
	steps.title = str(ProjectSettings.get_setting("application/config/name"))
	_view = LoadingView.new()
	if mode == Mode.VR:
		_xr = get_node("/root/XrManager")
		_vr = VrLoadingLayer.new()
		_xr.origin.add_child(_vr)
		_vr.setup(_view, _xr.camera)
		if _xr.interface.has_signal("session_visible"):
			_xr.interface.connect("session_visible", func() -> void: _session_visible = true)
		else:
			_session_visible = true
	else:
		_layer = CanvasLayer.new()
		_layer.layer = LAYER
		_layer.add_child(_view)
		add_child(_layer)
		get_viewport().disable_3d = true  # the half-built world is neither drawn nor are its render effects run
	_view.show_steps(steps)


func _report(text: String) -> void:
	var first := steps.count() == 0
	var now_ms := Time.get_ticks_msec()
	steps.add(text, now_ms)
	print("[Boot] %+7.2f s  %s" % [now_ms / 1000.0, text])
	_view.show_steps(steps)
	if mode == Mode.DESKTOP:
		RenderingServer.force_draw()
	elif mode == Mode.VR:
		await _present_vr(first)


## VR: the text set above reaches the headset with the next rendered frame; the step's work starts after it.
func _present_vr(first: bool) -> void:
	if not _placed:
		_placed = true
		var waited_ms := Time.get_ticks_msec()
		while not _session_visible and Time.get_ticks_msec() - waited_ms < SESSION_WAIT_MS:
			await get_tree().process_frame
		await get_tree().process_frame  # one frame with a valid head pose
		_vr.place(_xr.camera.transform)
	await get_tree().process_frame
	if first:
		await get_tree().process_frame


## The boot is built: the screen stays while `hold_text` returns a step text (e.g. the atmosphere's first-run
## precomputation) and goes READY_FRAMES rendered frames after it returns "".
func finish_when_idle(hold_text: Callable) -> void:
	_hold = hold_text
	_armed = true


func _process(_delta: float) -> void:
	if not _armed:
		return
	var holding := _hold.call() as String if _hold.is_valid() else ""
	if not holding.is_empty():
		var now_ms := Time.get_ticks_msec()
		if steps.current() != holding:
			print("[Boot] %+7.2f s  %s" % [now_ms / 1000.0, holding])
		steps.add(holding, now_ms)
		_view.show_steps(steps)
		return
	if not _world_shown:
		_world_shown = true
		_show_world()
		return  # the world renders from the next frame on
	_frames_left -= 1
	if _frames_left <= 0:
		_finish()


func _show_world() -> void:
	if mode == Mode.VR:
		_vr.release()
	else:
		get_viewport().disable_3d = false


func _finish() -> void:
	_armed = false
	var now_ms := Time.get_ticks_msec()
	steps.finish(now_ms)
	print("[Boot] done after %.2f s:" % (now_ms / 1000.0))
	for line in steps.report_lines(now_ms):
		print("[Boot] ", line)
	current = null
	if mode == Mode.VR:
		_vr.queue_free()
		queue_free()
	else:
		var tween := create_tween()
		tween.tween_property(_view, "modulate:a", 0.0, FADE_S)
		tween.tween_callback(queue_free)


func _exit_tree() -> void:
	if current == self:
		current = null
