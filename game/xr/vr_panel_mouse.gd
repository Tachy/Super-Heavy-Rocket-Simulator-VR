class_name VrPanelMouse
extends Node
## VR mouse control of the overlay viewport shown on the VR panel (pattern of the sibling's VrEditorHost): the mouse is
## captured and moves a pointer drawn into that viewport; its events go on to the viewport's controls (the menus).
## The right button still orbits the camera (the pointer stays put meanwhile); the wheel goes to the menus while the
## pointer is over one or a popup is open, else it zooms the camera. Keys go to the viewport first (LineEdit typing,
## Space, +/-) and on to the game when it leaves them unhandled.

var viewport: SubViewport
var menu: Control  ## the wheel stays with the camera outside of it
var cursor := Vector2.ZERO
var _pointer := Pointer.new()


func _init(target: SubViewport, menu_control: Control) -> void:
	name = "VrPanelMouse"
	viewport = target
	menu = menu_control
	cursor = Vector2(viewport.size) * 0.5
	viewport.gui_disable_input = false
	viewport.gui_embed_subwindows = true  # option popups render inside the viewport, i.e. on the panel
	_pointer.mouse = self
	viewport.add_child(_pointer)


func _ready() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	# The mouse is always "in" the viewport (a SubViewportContainer would tell it so), else hover is dropped.
	viewport.notification(Viewport.NOTIFICATION_VP_MOUSE_ENTER)


func _exit_tree() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE


func _input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		if Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
			return  # camera orbit: the pointer stays put
		var motion := event as InputEventMouseMotion
		cursor = (cursor + motion.relative).clamp(Vector2.ZERO, Vector2(viewport.size) - Vector2.ONE)
		_pointer.queue_redraw()
		_push_at_cursor(motion)
	elif event is InputEventMouseButton:
		var button := event as InputEventMouseButton
		if button.button_index == MOUSE_BUTTON_RIGHT:
			return  # camera orbit
		if button.button_index in [MOUSE_BUTTON_WHEEL_UP, MOUSE_BUTTON_WHEEL_DOWN] and not _over_menu():
			return  # camera zoom
		_push_at_cursor(button)
	elif event is InputEventKey:
		_pointer.move_to_front()
		viewport.push_input(event)
		if viewport.is_input_handled():
			get_viewport().set_input_as_handled()


## The pointer's own position: over the menu panel or an open (embedded) popup.
func _over_menu() -> bool:
	return menu.get_global_rect().has_point(cursor) or not viewport.get_embedded_subwindows().is_empty()


func _push_at_cursor(event: InputEventMouse) -> void:
	var moved := event.duplicate() as InputEventMouse
	moved.position = cursor
	moved.global_position = cursor
	_pointer.move_to_front()
	viewport.push_input(moved)
	get_viewport().set_input_as_handled()


## The mouse pointer drawn into the viewport (the captured system cursor is not in the headset or the mirror).
class Pointer:
	extends Control

	const SHAPE: Array[Vector2] = [Vector2(0, 0), Vector2(0, 22), Vector2(6, 17), Vector2(10, 26), Vector2(14, 24),
			Vector2(10, 16), Vector2(17, 16)]

	var mouse: VrPanelMouse

	func _init() -> void:
		name = "VrPointer"
		mouse_filter = Control.MOUSE_FILTER_IGNORE
		set_anchors_preset(Control.PRESET_FULL_RECT)
		z_index = RenderingServer.CANVAS_ITEM_Z_MAX

	func _draw() -> void:
		var points := PackedVector2Array()
		for p in SHAPE:
			points.append(mouse.cursor + p)
		draw_colored_polygon(points, Color.WHITE)
		points.append(points[0])
		draw_polyline(points, Color.BLACK, 1.5, true)
