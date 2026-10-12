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
