class_name VrPanelMouse
extends Node
## VR mouse control of the overlay viewport shown on the VR panel (pattern of the sibling's VrEditorHost): the mouse is
## captured and moves a pointer; its events go on to the viewport's controls (the menus). The pointer is drawn into its
## own small viewport and composited by the panel's shader (pointer_moved): embedded popups (option lists) are drawn
## over every control of the overlay viewport, so a pointer drawn there vanished behind an open list.
## The right button still orbits the camera (the pointer stays put meanwhile); the wheel goes to the menus while the
## pointer is over one or a popup is open, else it zooms the camera. Keys go to the viewport first (LineEdit typing,
## Space, +/-) and on to the game when it leaves them unhandled.

signal pointer_moved(cursor_px: Vector2)

const POINTER_SIZE := Vector2i(32, 32)  ## covers the arrow shape (Pointer.SHAPE)

var viewport: SubViewport
var menu: Control  ## the wheel stays with the camera outside of it
var cursor := Vector2.ZERO
var pointer_viewport := SubViewport.new()  ## the arrow alone, transparent; its texture goes to the panel shader


func _init(target: SubViewport, menu_control: Control) -> void:
	name = "VrPanelMouse"
	viewport = target
	menu = menu_control
	cursor = Vector2(viewport.size) * 0.5
	viewport.gui_disable_input = false
	viewport.gui_embed_subwindows = true  # option popups render inside the viewport, i.e. on the panel
	pointer_viewport.name = "PointerViewport"
	pointer_viewport.size = POINTER_SIZE
	pointer_viewport.transparent_bg = true
	pointer_viewport.disable_3d = true
	pointer_viewport.gui_disable_input = true
	pointer_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS  # 32 x 32: negligible, and drawn whenever it enters the tree
	pointer_viewport.add_child(Pointer.new())
	add_child(pointer_viewport)


func _ready() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	# The mouse is always "in" the viewport (a SubViewportContainer would tell it so), else hover is dropped.
	viewport.notification(Viewport.NOTIFICATION_VP_MOUSE_ENTER)
	pointer_moved.emit(cursor)


func _exit_tree() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE


func _input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		if Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
			return  # camera orbit: the pointer stays put
		var motion := event as InputEventMouseMotion
		cursor = (cursor + motion.relative).clamp(Vector2.ZERO, Vector2(viewport.size) - Vector2.ONE)
		pointer_moved.emit(cursor)
		_push_at_cursor(motion)
	elif event is InputEventMouseButton:
		var button := event as InputEventMouseButton
		if button.button_index == MOUSE_BUTTON_RIGHT:
			return  # camera orbit
		if button.button_index in [MOUSE_BUTTON_WHEEL_UP, MOUSE_BUTTON_WHEEL_DOWN] and not _over_menu():
			return  # camera zoom
		_push_at_cursor(button)
	elif event is InputEventKey:
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
	viewport.push_input(moved)
	get_viewport().set_input_as_handled()


## The mouse pointer's arrow, tip at the viewport's origin (the captured system cursor is not in the headset or the
## mirror).
class Pointer:
	extends Control

	const SHAPE: Array[Vector2] = [Vector2(1, 1), Vector2(1, 23), Vector2(7, 18), Vector2(11, 27), Vector2(15, 25),
			Vector2(11, 17), Vector2(18, 17)]  ## 1 px inset: the outline stays inside the pointer viewport

	func _init() -> void:
		name = "VrPointer"
		mouse_filter = Control.MOUSE_FILTER_IGNORE
		set_anchors_preset(Control.PRESET_FULL_RECT)

	func _draw() -> void:
		var points := PackedVector2Array(SHAPE)
		draw_colored_polygon(points, Color.WHITE)
		points.append(points[0])
		draw_polyline(points, Color.BLACK, 1.5, true)
