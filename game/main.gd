extends Node
## Application entry: world plus UI. The webcast overlay (OverlayHost) is the external view's display; the old flight
## HUD (text telemetry, NavBall, flight director) stays available behind F1 (off by default) until the cockpit exists.

## Canvas layers, bottom to top.
const WEBCAST_LAYER := 1
const PANEL_LAYER := 2
const LEGACY_HUD_LAYER := 3

## User argument (after `--`): report whether the world found its data, then quit (tools/package.ps1 runs it on the
## exported build).
const SELF_CHECK_ARG := "--self-check"

var world := World.new()
var overlay_host := OverlayHost.new()
var legacy_hud := CanvasLayer.new()  ## FlightOverlay, NavBall, FlightHud (F1)


func _ready() -> void:
	world.name = "World"
	add_child(world)
	get_window().title = "%s %s" % [ProjectSettings.get_setting("application/config/name"), GameVersion.VERSION]
	if OS.get_cmdline_user_args().has(SELF_CHECK_ARG):
		_self_check()
		return
	overlay_host.overlay.world = world
	add_child(overlay_host)
	var xr := get_node_or_null("/root/XrManager")
	var panel := TimePanel.new()
	panel.world = world
	if xr != null and xr.xr_active:
		# VR: canvas layers are not drawn in the headset. The overlay and the menu (top left, as on the desktop) share a
		# curved panel fixed to the base pose (not the head), driven by a mouse pointer.
		overlay_host.viewport.add_child(panel)
		var mouse := VrPanelMouse.new(overlay_host.viewport, panel)
		var vr_panel := VrOverlayPanel.new()
		xr.origin.add_child(vr_panel)
		vr_panel.setup(overlay_host.viewport.get_texture())
		vr_panel.set_pointer(mouse.pointer_viewport.get_texture(), Vector2(VrPanelMouse.POINTER_SIZE))
		mouse.pointer_moved.connect(vr_panel.move_pointer)
		add_child(mouse)
		vr_panel.place(xr.recenter_pose())
		xr.recentered.connect(func() -> void: vr_panel.place(xr.recenter_pose()))
	else:
		overlay_host.show_on_desktop(WEBCAST_LAYER)
		var ui := CanvasLayer.new()
		ui.layer = PANEL_LAYER
		add_child(ui)
		ui.add_child(panel)
	legacy_hud.name = "LegacyHud"
	legacy_hud.layer = LEGACY_HUD_LAYER
	legacy_hud.visible = false
	add_child(legacy_hud)
	var overlay := FlightOverlay.new()
	overlay.world = world
	legacy_hud.add_child(overlay)
	world.flight.overlay = overlay
	var navball := NavBall.new()
	navball.anchor_left = 0.5
	navball.anchor_right = 0.5
	navball.anchor_top = 1.0
	navball.anchor_bottom = 1.0
	navball.offset_left = -NavBall.RADIUS - 10.0
	navball.offset_top = -2.0 * NavBall.RADIUS - 30.0
	navball.visible = false
	legacy_hud.add_child(navball)
	world.flight.navball = navball
	var hud := FlightHud.new()
	hud.world = world
	legacy_hud.add_child(hud)


## One line `SELF-CHECK OK ...` (exit code 0) or `SELF-CHECK FAIL <reason>` (exit code 1).
func _self_check() -> void:
	var reason := ""
	if not world.spice_ok:
		reason = "no SPICE kernels in " + Paths.data_dir().path_join("spice")
	elif world.sites.is_empty():
		reason = "no launch sites in " + Paths.config_dir().path_join("launch_sites.toml")
	if not reason.is_empty():
		print("SELF-CHECK FAIL ", reason)
		get_tree().quit(1)
		return
	var yes_no := func(flag: bool) -> String: return "yes" if flag else "no"
	print("SELF-CHECK OK version=%s spice=ok sites=%d earth_terrain=%s imagery=%s" % [GameVersion.VERSION,
			world.sites.size(), yes_no.call(world.terrain_loaded.get("earth", false)),
			yes_no.call(FileAccess.file_exists(Paths.data_dir().path_join("imagery/earth/index.toml")))])
	get_tree().quit(0)


## F1 shows / hides the old flight HUD.
func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key != null and key.pressed and not key.echo and key.keycode == KEY_F1:
		legacy_hud.visible = not legacy_hud.visible
		get_viewport().set_input_as_handled()
