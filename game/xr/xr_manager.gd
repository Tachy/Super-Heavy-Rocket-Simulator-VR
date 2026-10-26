extends Node
## Autoload `XrManager`: owns the XR rig (XROrigin3D + XRCamera3D) and decides the start mode
## (pattern of ../Stunt-Track-Racer-VR/scripts/autoload/xr_manager.gd). VR and desktop are separate start modes,
## no switching at run time:
##   VR:      Start-VR.cmd   (--xr-mode on -- --vr): OpenXR + headset required, else a message and exit code 1
##   Desktop: Start-Desktop.cmd (--xr-mode off -- --no-xr); every other start, `--script` and headless runs too
## OpenXR is off in project.godot so that test scripts and editor runs never touch the headset runtime.
## In VR the OrbitCamera feeds the base pose each frame (set_base); the headset moves in 6DOF on top of it.
## F12 recentres: the current head yaw + position become the base pose (camera control untouched).

signal recentered

const VR_ARG := "--vr"
const DESKTOP_ARG := "--no-xr"
const AUTO_RECENTER_DELAY_S := 2.0  ## [E] one recentre after the start (pattern of the sibling project), so the view is in front

var origin: XROrigin3D
var camera: XRCamera3D  ## VR only (null on the desktop, where the OrbitCamera is the current camera)
var interface: XRInterface
var xr_active := false

var _base := Basis.IDENTITY
var _recenter := Transform3D.IDENTITY
var _auto_recenter_timer_s := 0.0


func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
	origin = XROrigin3D.new()
	origin.name = "XROrigin3D"
	origin.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
	add_child(origin)
	interface = XRServer.find_interface("OpenXR")
	var xr_ready := interface != null and interface.is_initialized()
	var want_vr := vr_requested()
	if xr_ready and not want_vr:
		# OpenXR started although the desktop was requested: shut it down, its frame loop throttles the desktop
		interface.uninitialize()
		xr_ready = false
	if want_vr and not xr_ready:
		# VR start without a usable headset: no silent fallback to the desktop
		OS.alert("Kein VR-Headset gefunden.\n\nHeadset einschalten und seine OpenXR-Runtime starten\n"
				+ "oder die Desktop-Version starten (Start-Desktop.cmd).", "Starship-Simulator")
		get_tree().quit(1)
		return
	if xr_ready:
		camera = XRCamera3D.new()
		camera.name = "XRCamera3D"
		origin.add_child(camera)
		origin.current = true
		camera.current = true
		if interface.has_signal("pose_recentered"):
			interface.connect("pose_recentered", recenter)
		get_viewport().use_xr = true
		xr_active = true
		DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)  # the headset paces the frames
		_auto_recenter_timer_s = AUTO_RECENTER_DELAY_S
		print("[XR] VR mode")
	else:
		origin.current = false
		get_viewport().use_xr = false
		xr_active = false
		print("[XR] desktop mode")
	_apply()


## True when this start asked for VR: the `--vr` user argument, not overridden by `--no-xr`, never in headless or
## `--script` runs.
static func vr_requested() -> bool:
	var user_args := OS.get_cmdline_user_args()
	var engine_args := OS.get_cmdline_args()
	if DisplayServer.get_name() == "headless" or engine_args.has("--script") or engine_args.has("-s"):
		return false
	return user_args.has(VR_ARG) and not user_args.has(DESKTOP_ARG)


## Base orientation of the view (the OrbitCamera's basis); the origin stays at the render origin.
func set_base(basis: Basis) -> void:
	_base = basis
	_apply()


## F12 / runtime recentre: the current head yaw and position become the base pose (VR only).
func recenter() -> void:
	if not xr_active:
		return
	_recenter = CameraMath.recenter_offset(camera.transform)
	_apply()
	print("[XR] recentred at head pose ", camera.transform.origin)
	recentered.emit()


## Head pose (yaw + position) in origin space that the last recentre took as the base pose; identity before the first.
func recenter_pose() -> Transform3D:
	return _recenter


func _process(delta: float) -> void:
	FrameProfiler.begin(&"xr")
	_process_unprofiled(delta)
	FrameProfiler.end(&"xr")


func _process_unprofiled(delta: float) -> void:
	if _auto_recenter_timer_s > 0.0:
		_auto_recenter_timer_s -= delta
		if _auto_recenter_timer_s <= 0.0:
			recenter()


func _apply() -> void:
	origin.transform = CameraMath.vr_origin_transform(_base, _recenter if xr_active else Transform3D.IDENTITY)


func _input(event: InputEvent) -> void:
	if xr_active and event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_F12:
		recenter()
		get_viewport().set_input_as_handled()
