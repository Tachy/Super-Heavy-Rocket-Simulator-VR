class_name WebcastStyle
extends RefCounted
## Look of the webcast overlay: fonts, colours, stroke widths and the drawing helpers every widget shares
## (thin white strokes on a soft dark halo, white text with a soft dark outline, no opaque panels).

## Fonts from the data pipeline (tools/data_pipeline/sources.toml: font_barlow, font_barlow_condensed); the default
## theme font is used when a file is missing.
const LABEL_FONT_FILE := "fonts/Barlow-Medium.ttf"
const VALUE_FONT_FILE := "fonts/BarlowCondensed-SemiBold.ttf"

## [E] Stroke width at the 1600x900 reference layout (px), and the dark halo drawn under it for contrast.
const STROKE_PX := 2.0
const HALO_PX := 5.0
const TEXT_OUTLINE_PX := 5

const WHITE := Color(1.0, 1.0, 1.0, 1.0)
## [E] Unfilled tracks and unlit elements.
const DIM := Color(1.0, 1.0, 1.0, 0.42)
const HALO := Color(0.0, 0.0, 0.0, 0.38)
const TEXT_OUTLINE := Color(0.0, 0.0, 0.0, 0.5)

## Horizontal alignment of draw_text.
enum Align { LEFT, CENTER, RIGHT }

static var _fonts := {}


static func label_font() -> Font:
	return _font(LABEL_FONT_FILE)


static func value_font() -> Font:
	return _font(VALUE_FONT_FILE)


## Text with its baseline at `baseline.y`, aligned at `baseline.x`; the dark outline first, then the fill.
static func draw_text(item: CanvasItem, baseline: Vector2, text: String, font: Font, size: int, color := WHITE,
		align := Align.CENTER) -> void:
	var width := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, size).x
	var x := baseline.x - (width * 0.5 if align == Align.CENTER else width if align == Align.RIGHT else 0.0)
	var outline := Color(TEXT_OUTLINE, TEXT_OUTLINE.a * color.a)
	item.draw_string_outline(font, Vector2(x, baseline.y), text, HORIZONTAL_ALIGNMENT_LEFT, -1, size, TEXT_OUTLINE_PX, outline)
	item.draw_string(font, Vector2(x, baseline.y), text, HORIZONTAL_ALIGNMENT_LEFT, -1, size, color)


## Baseline offset that centres a line of `size` vertically on a point.
static func center_offset(font: Font, size: int) -> float:
	return (font.get_ascent(size) - font.get_descent(size)) * 0.5


## Arc stroke (angles in radians, clockwise on screen) on its halo.
static func draw_arc_stroke(item: CanvasItem, center: Vector2, radius: float, from_rad: float, to_rad: float,
		color := WHITE, width := STROKE_PX) -> void:
	if absf(to_rad - from_rad) < 1e-4:
		return
	var points := maxi(24, int(absf(to_rad - from_rad) * radius / 4.0))
	item.draw_arc(center, radius, from_rad, to_rad, points, Color(HALO, HALO.a * color.a), width + HALO_PX - STROKE_PX, true)
	item.draw_arc(center, radius, from_rad, to_rad, points, color, width, true)


## Filled dot on its halo.
static func draw_dot(item: CanvasItem, center: Vector2, radius: float, color := WHITE) -> void:
	item.draw_circle(center, radius + (HALO_PX - STROKE_PX) * 0.5, Color(HALO, HALO.a * color.a), true, -1.0, true)
	item.draw_circle(center, radius, color, true, -1.0, true)


## Hollow ring on its halo.
static func draw_ring(item: CanvasItem, center: Vector2, radius: float, color := WHITE) -> void:
	draw_arc_stroke(item, center, radius, 0.0, TAU, color)


static func _font(file: String) -> Font:
	if _fonts.has(file):
		return _fonts[file]
	var font: Font = ThemeDB.fallback_font
	var path := Paths.data_dir().path_join(file)
	if FileAccess.file_exists(path):
		var loaded := FontFile.new()
		loaded.multichannel_signed_distance_field = true  # crisp when rotated (arc labels) or scaled (VR panel)
		if loaded.load_dynamic_font(path) == OK:
			font = loaded
	else:
		push_warning("WebcastStyle: font %s missing (python tools/data_pipeline/fetch.py) - default font" % path)
	_fonts[file] = font
	return font
