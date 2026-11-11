class_name CockpitUiGate
extends Node
## In the cockpit view the 2D UI is gone completely (user 2026-11-11): the VR panel (webcast overlay + menu on the sphere section),
## the desktop overlay / menu canvas layers and the legacy HUD are hidden, and the VR panel's mouse (VrPanelMouse) is suspended, so
## the mouse reaches the ConsolePointer alone. Leaving the cockpit restores what each part showed before. `apply` is idempotent.

var hidden_nodes: Array[Node] = []  ## nodes with a `visible` property (CanvasLayer, MeshInstance3D, Control): hidden in the cockpit
var vr_mouse: VrPanelMouse  ## null on the desktop
var in_cockpit := false
var _saved := {}  ## node -> visible before the cockpit


func apply(cockpit: bool) -> void:
	if cockpit == in_cockpit:
		return
	in_cockpit = cockpit
	if cockpit:
		_saved.clear()
		for node in hidden_nodes:
			if is_instance_valid(node):
				_saved[node] = node.get("visible")
				node.set("visible", false)
	else:
		for node in _saved:
			if is_instance_valid(node):
				node.set("visible", _saved[node])
		_saved.clear()
	if vr_mouse != null:
		vr_mouse.suspended = cockpit
