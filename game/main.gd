extends Node
## Application entry: world plus UI overlay.

var world := World.new()


func _ready() -> void:
	world.name = "World"
	add_child(world)
	var ui := CanvasLayer.new()
	add_child(ui)
	var panel := TimePanel.new()
	panel.world = world
	ui.add_child(panel)
	var overlay := FlightOverlay.new()
	overlay.world = world
	ui.add_child(overlay)
	world.flight.overlay = overlay
	var navball := NavBall.new()
	navball.anchor_left = 0.5
	navball.anchor_right = 0.5
	navball.anchor_top = 1.0
	navball.anchor_bottom = 1.0
	navball.offset_left = -NavBall.RADIUS - 10.0
	navball.offset_top = -2.0 * NavBall.RADIUS - 30.0
	navball.visible = false
	ui.add_child(navball)
	world.flight.navball = navball
	var hud := FlightHud.new()
	hud.world = world
	ui.add_child(hud)
