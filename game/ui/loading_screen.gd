class_name LoadingScreen
extends Node
## Startup loading screen: shows which boot step runs right now (light text on black, no progress bar). The boot code
## reports each step with `await LoadingScreen.step("Wetter laden ...")` (a no-op without a screen, so tests and
## headless runs never notice). The boot scene creates it (`start`) before Main exists, Main's `start` call returns the
## same screen, and Main arms it at the end (`finish_when_idle`); it disappears for good after the first rendered
## world frames.
##   Desktop: a full-screen CanvasLayer; the text reaches the window at once (RenderingServer.force_draw), so the
##            blocking boot steps stay synchronous.
##   VR:      a native OpenXR quad layer (VrLoadingLayer). Only rendered frames reach the headset, so a step waits one
##            frame after its text is set (`yielding()`: World._ready then suspends and Main waits for it).
## Every step is also logged with its start and duration (`[Boot] ...`, the table at the end), and so is the screen's
## life (created, world shown, removed; in VR the quad and the XR camera's blanking).
## Safety: once armed, the screen goes at the latest ARMED_TIMEOUT_S later, whatever it still waits for.

enum Mode { NONE, DESKTOP, VR }

const DISABLE_ARG := "--no-loading-screen"  ## user argument: no screen and no boot log
const LAYER := 100  ## above every other canvas layer
const READY_FRAMES := 6  ## rendered world frames under the screen before it goes (the cloud shader warm-up takes 4)
const FADE_S := 0.3  ## desktop only
const SESSION_WAIT_MS := 4000  ## VR: how long the first step waits for the headset session to be visible
## [E] latest removal after arming; covers the atmosphere's first-run precomputation (about a minute) with margin.
const ARMED_TIMEOUT_S := 180.0

## The one screen of this start. Survives the reparent to the root at the boot scene's hand-over (cleared only when the
## screen finishes or is deleted, not on leaving the tree), so Main's `start` finds it and never builds a second one.
static var current: LoadingScreen

var steps := LoadingSteps.new()
var mode := Mode.NONE
var armed_timeout_ms := int(ARMED_TIMEOUT_S * 1000.0)  ## a variable for the test

var _view: LoadingView
var _layer: CanvasLayer  ## desktop
var _vr: VrLoadingLayer
var _xr: Node  ## the XrManager autoload (origin, camera, interface)
var _hold: Callable
var _armed := false
var _armed_ms := 0
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
	return create(host, wanted, host.get_node_or_null("/root/XrManager") if wanted == Mode.VR else null)


## Builds the screen in `wanted` mode under `host`; `xr` provides origin, camera and interface (VR). Used by `start`
## and directly by the test (with a stand-in for the XrManager).
static func create(host: Node, wanted: Mode, xr: Node) -> LoadingScreen:
	var screen := LoadingScreen.new()
	screen.mode = wanted
	screen._xr = xr
	host.add_child(screen)
	screen._build()
	current = screen
	print("[Boot] loading screen created (%s)" % ("VR" if wanted == Mode.VR else "desktop"))
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
	steps.title = GameTitle.DISPLAY
	_view = LoadingView.new()
	if mode == Mode.VR:
		_vr = VrLoadingLayer.new()
		_xr.origin.add_child(_vr)
		_vr.setup(_view, _xr.camera)
		var interface: Object = _xr.interface
		if interface != null and interface.has_signal("session_visible"):
			interface.connect("session_visible", func() -> void: _session_visible = true)
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
		_view.snap()  # the step blocks the main thread right after: no half-scrolled picture
		RenderingServer.force_draw()
	elif mode == Mode.VR:
		await _present_vr(first)


## True once the headset shows the app's frames. The state is read directly too: the signal may have come before the
## screen connected to it.
func _session_is_visible() -> bool:
	if _session_visible:
		return true
	var interface: Object = _xr.interface
	if interface != null and interface.has_method("get_session_state"):
		var state: int = interface.get_session_state()
		_session_visible = state == OpenXRInterface.SESSION_STATE_VISIBLE or state == OpenXRInterface.SESSION_STATE_FOCUSED
	return _session_visible


## VR: the text set above reaches the headset with the next rendered frame; the step's work starts after it.
func _present_vr(first: bool) -> void:
	if not _placed:
		_placed = true
		var waited_ms := Time.get_ticks_msec()
		while not _session_is_visible() and Time.get_ticks_msec() - waited_ms < SESSION_WAIT_MS:
			await get_tree().process_frame
		await get_tree().process_frame  # one frame with a valid head pose
		_vr.place(_xr.camera.transform)
		print("[Boot] VR loading quad placed (session visible: %s, waited %.2f s)"
				% [_session_visible, (Time.get_ticks_msec() - waited_ms) / 1000.0])
	await get_tree().process_frame
	if first:
		await get_tree().process_frame
	while _view.is_animating():  # the history scrolls up (0.3 s) before the step's blocking work starts
		await get_tree().process_frame


## The boot is built: the screen stays while `hold_text` returns a step text (e.g. the atmosphere's first-run
## precomputation) and goes READY_FRAMES rendered frames after it returns "".
func finish_when_idle(hold_text: Callable) -> void:
	_hold = hold_text
	_armed = true
	_armed_ms = Time.get_ticks_msec()


## True while the screen is armed and has not gone yet.
func is_armed() -> bool:
	return _armed


func _process(_delta: float) -> void:
	if not _armed:
		return
	if Time.get_ticks_msec() - _armed_ms > armed_timeout_ms:
		push_warning("LoadingScreen: still up %.0f s after the boot - removed" % (armed_timeout_ms / 1000.0))
		print("[Boot] loading screen timeout after %.0f s: removing it" % (armed_timeout_ms / 1000.0))
		if not _world_shown:
			_world_shown = true
			_show_world()
		_finish()
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
	print("[Boot] %+7.2f s  world shown under the loading screen" % (Time.get_ticks_msec() / 1000.0))
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
	if current == self:
		current = null
	if mode == Mode.VR:
		_vr.remove()
		_vr = null
		print("[Boot] loading screen removed")
		queue_free()
	else:
		var tween := create_tween()
		tween.tween_property(_view, "modulate:a", 0.0, FADE_S)
		tween.tween_callback(queue_free)
		print("[Boot] loading screen fading out")


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE:
		if current == self:
			current = null
		if _vr != null and is_instance_valid(_vr):  # never leave the quad up or the XR camera blank
			_vr.remove()
