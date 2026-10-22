extends Node
## Application entry: world plus UI. The webcast overlay (OverlayHost) is the external view's display; the old flight
## HUD (text telemetry, NavBall, flight director) stays available behind F1 (off by default) until the cockpit exists.

## Canvas layers, bottom to top.
const WEBCAST_LAYER := 1
const PANEL_LAYER := 2
const LEGACY_HUD_LAYER := 3

var world := World.new()
var overlay_host := OverlayHost.new()
var legacy_hud := CanvasLayer.new()  ## FlightOverlay, NavBall, FlightHud (F1)


func _ready() -> void:
	world.name = "World"
	add_child(world)
	overlay_host.overlay.world = world
	add_child(overlay_host)
	var xr := get_node_or_null("/root/XrManager")
	if xr != null and xr.xr_active:
		# VR: the overlay sits on a curved panel fixed to the base pose (not the head)
		var vr_panel := VrOverlayPanel.new()
		xr.origin.add_child(vr_panel)
		vr_panel.setup(overlay_host.viewport.get_texture())
	else:
		overlay_host.show_on_desktop(WEBCAST_LAYER)
	var ui := CanvasLayer.new()
	ui.layer = PANEL_LAYER
	add_child(ui)
	var panel := TimePanel.new()
	panel.world = world
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


## F1 shows / hides the old flight HUD.
func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key != null and key.pressed and not key.echo and key.keycode == KEY_F1:
		legacy_hud.visible = not legacy_hud.visible
		get_viewport().set_input_as_handled()
