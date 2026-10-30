class_name LoadingView
extends Control
## The loading screen's picture, in the style of the engine boot splash (game/assets/boot_splash.png): light text on
## pure black, the game's name where the splash has it, the running boot step below it in a slightly larger font than
## the splash's "wird gestartet ..." line, and the finished steps above the running one, each older one fainter. When
## a new step comes, the finished ones scroll up (SCROLL_S) and fade out below the title. Everything is laid out
## relative to the view's height (REFERENCE_HEIGHT_PX = the splash's window height), so the desktop layer and the VR
## quad's SubViewport show the same picture. Animates in _process: it needs frames to pass (VR waits for it,
## see `is_animating()`; the desktop's blocking steps call `snap()`).

const BACKGROUND := Color.BLACK
const REFERENCE_HEIGHT_PX := 900.0  ## the splash is made for the project's 1600x900 window
## The boot splash's rows (tools/godot/make_boot_splash.gd uses these): the name and the "wird gestartet ..." line.
const TITLE_SIZE := 64
const TITLE_COLOR := Color(0.95, 0.95, 0.95)
const TITLE_OFFSET := -0.4 * TITLE_SIZE  ## title centre relative to the view's middle (reference pixels)
const SUBTITLE_SIZE := 32
const SUBTITLE_COLOR := Color(0.55, 0.55, 0.55)
const SUBTITLE_OFFSET := 0.6 * TITLE_SIZE
## The step rows (reference pixels).
const CURRENT_SIZE := 40  ## slightly larger than the splash's SUBTITLE_SIZE
const CURRENT_COLOR := Color(0.95, 0.95, 0.95)
const HISTORY_COLOR := Color(0.75, 0.75, 0.75)
const HISTORY_SCALE := 0.8  ## finished rows are drawn at this fraction of CURRENT_SIZE
const CURRENT_OFFSET := 220.0  ## centre of the running step relative to the view's middle
const ROW_PITCH := 50.0  ## distance between two rows; older rows are higher
## Opacity by row slot (0 = running step, 1 = newest finished, ...); in between it is interpolated, so a row
## scrolling up fades smoothly. The last slot is 0: the row has vanished below the title.
const SLOT_ALPHA := [1.0, 0.62, 0.38, 0.18, 0.0]
const SCROLL_S := 0.3  ## duration of one scroll step

var _scale := 1.0
var _title := Label.new()
var _rows: Dictionary = {}  ## step index -> Label
var _texts: Array[String] = []
var _scroll := 0.0  ## slots the rows still have to move (new steps start it at 1)


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
	add_child(_title)
	resized.connect(_on_resized)


func show_steps(steps: LoadingSteps) -> void:
	_title.text = steps.title
	var before := _texts.size()
	_texts.clear()
	for i in steps.count():
		_texts.append(steps.text_at(i))
	if before > 0 and _texts.size() > before:
		_scroll = minf(_scroll + _texts.size() - before, float(SLOT_ALPHA.size()))
	_layout()


## Ends a running scroll at once (the caller blocks the main thread right after, a half-moved picture would stand).
func snap() -> void:
	_scroll = 0.0
	_layout()


func is_animating() -> bool:
	return _scroll > 0.0


func _process(delta: float) -> void:
	if _scroll > 0.0:
		# A backlog of several steps scrolls faster, so it never lags behind the boot.
		_scroll = maxf(0.0, _scroll - delta * maxf(1.0, _scroll) / SCROLL_S)
		_layout()


func _on_resized() -> void:
	_scale = size.y / REFERENCE_HEIGHT_PX
	_title.add_theme_font_size_override("font_size", roundi(TITLE_SIZE * _scale))
	for label: Label in _rows.values():
		label.add_theme_font_size_override("font_size", roundi(CURRENT_SIZE * _scale))
	_layout()


## Opacity of a row `slot` slots below the title side of the running step (slot 0 = running, negative = still
## arriving from below).
static func slot_alpha(slot: float) -> float:
	if slot <= -1.0:
		return 0.0
	if slot < 0.0:
		return 1.0 + slot
	var last := SLOT_ALPHA.size() - 1
	if slot >= last:
		return 0.0
	return lerpf(SLOT_ALPHA[int(slot)], SLOT_ALPHA[int(slot) + 1], slot - floorf(slot))


func _layout() -> void:
	if size.y <= 0.0:
		return
	_scale = size.y / REFERENCE_HEIGHT_PX
	_place(_title, TITLE_OFFSET * _scale, TITLE_SIZE * _scale)
	var count := _texts.size()
	var oldest := maxi(0, count - SLOT_ALPHA.size() - 1)
	for index: int in _rows.keys():
		if index < oldest:
			_rows[index].queue_free()
			_rows.erase(index)
	for index in range(oldest, count):
		var label: Label = _rows.get(index)
		if label == null:
			label = Label.new()
			_style(label, roundi(CURRENT_SIZE * _scale), CURRENT_COLOR)
			add_child(label)
			_rows[index] = label
		var slot := float(count - 1 - index) - _scroll
		var alpha := slot_alpha(slot)
		label.text = _texts[index]
		label.visible = alpha > 0.004
		var finished := clampf(slot, 0.0, 1.0)  ## 0 = running step, 1 = fully a history row
		label.add_theme_color_override("font_color", CURRENT_COLOR.lerp(HISTORY_COLOR, finished))
		label.modulate.a = alpha
		_place(label, (CURRENT_OFFSET - slot * ROW_PITCH) * _scale, CURRENT_SIZE * _scale)
		label.pivot_offset = label.size * 0.5
		label.scale = Vector2.ONE * lerpf(1.0, HISTORY_SCALE, finished)


static func _style(label: Label, font_size: int, color: Color) -> void:
	label.add_theme_font_size_override("font_size", font_size)
	label.add_theme_color_override("font_color", color)
	label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	label.mouse_filter = Control.MOUSE_FILTER_IGNORE


## A full-width row centred vertically, `offset_y` pixels from the middle (anchor based: used by the splash script).
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


## Same placement on this view (its own size known): the row's middle `offset_y` below the view's middle.
func _place(label: Label, offset_y: float, font_size: float) -> void:
	_place_row(label, offset_y, roundi(font_size))
