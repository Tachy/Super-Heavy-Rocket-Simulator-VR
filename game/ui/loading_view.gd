class_name LoadingView
extends Control
## The loading screen's picture: light text on pure black, the title at the top, the current boot step large in the
## middle and the last finished ones smaller and dimmed above it. Size-agnostic (fills its parent), so the desktop
## layer and the VR quad's SubViewport use the same view.

const BACKGROUND := Color.BLACK
const TITLE_COLOR := Color(0.55, 0.55, 0.55)
const CURRENT_COLOR := Color(0.95, 0.95, 0.95)
const HISTORY_COLOR := Color(0.7, 0.7, 0.7)
const TITLE_SIZE := 44
const CURRENT_SIZE := 64
const HISTORY_SIZE := 32
## Opacity of the history rows, nearest to the current step first.
const HISTORY_ALPHA := [0.55, 0.38, 0.24, 0.12]

var _title := Label.new()
var _history: Array[Label] = []
var _current := Label.new()


func _init() -> void:
	name = "LoadingView"
	set_anchors_preset(Control.PRESET_FULL_RECT)
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	var background := ColorRect.new()
	background.color = BACKGROUND
	background.set_anchors_preset(Control.PRESET_FULL_RECT)
	background.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(background)
	_style(_title, TITLE_SIZE, TITLE_COLOR)
	_title.set_anchors_preset(Control.PRESET_TOP_WIDE)
	_title.offset_top = 48.0
	_title.offset_bottom = 48.0 + TITLE_SIZE * 1.6
	add_child(_title)
	# The current step sits at the vertical centre; the history rows stack upwards from it.
	_style(_current, CURRENT_SIZE, CURRENT_COLOR)
	_place_row(_current, 0.0, CURRENT_SIZE)
	add_child(_current)
	for i in LoadingSteps.HISTORY_MAX:
		var row := Label.new()
		_style(row, HISTORY_SIZE, HISTORY_COLOR)
		_place_row(row, -(CURRENT_SIZE * 0.8 + (i + 0.5) * HISTORY_SIZE * 1.6), HISTORY_SIZE)
		row.modulate.a = HISTORY_ALPHA[i]
		add_child(row)
		_history.append(row)  # index 0 = nearest to the current step


func show_steps(steps: LoadingSteps) -> void:
	_title.text = steps.title
	_current.text = steps.current()
	var finished := steps.history()  # oldest first
	for i in _history.size():
		var from_end := finished.size() - 1 - i
		_history[i].text = finished[from_end] if from_end >= 0 else ""


static func _style(label: Label, font_size: int, color: Color) -> void:
	label.add_theme_font_size_override("font_size", font_size)
	label.add_theme_color_override("font_color", color)
	label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	label.mouse_filter = Control.MOUSE_FILTER_IGNORE


## A full-width row centred vertically, `offset_y` pixels from the middle.
static func _place_row(label: Label, offset_y: float, font_size: int) -> void:
	label.anchor_left = 0.0
	label.anchor_right = 1.0
	label.anchor_top = 0.5
	label.anchor_bottom = 0.5
	var height := font_size * 1.6
	label.offset_left = 40.0
	label.offset_right = -40.0
	label.offset_top = offset_y - height * 0.5
	label.offset_bottom = offset_y + height * 0.5
