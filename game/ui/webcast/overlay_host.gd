class_name OverlayHost
extends Node
## Renders the WebcastOverlay into a transparent SubViewport at the 1600x900 reference size. The desktop shows the
## viewport texture full-window (show_on_desktop); in VR the same texture goes onto the curved panel.
## The texture holds premultiplied alpha (2D drawing blends onto the cleared transparent target), so it is shown with
## premultiplied blending; straight alpha blending would darken every semi-transparent pixel a second time.

const SIZE := Vector2i(1600, 900)

var viewport := SubViewport.new()
var overlay := WebcastOverlay.new()


func _init() -> void:
	name = "OverlayHost"
	viewport.name = "WebcastViewport"
	viewport.size = SIZE
	viewport.transparent_bg = true
	viewport.disable_3d = true
	viewport.gui_disable_input = true  # display only: the mouse belongs to the camera
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	add_child(viewport)
	overlay.name = "WebcastOverlay"
	viewport.add_child(overlay)


## Desktop: the overlay texture over the whole window (aspect kept, centred), in its own canvas layer `layer`.
func show_on_desktop(layer: int) -> void:
	var canvas := CanvasLayer.new()
	canvas.name = "WebcastLayer"
	canvas.layer = layer
	add_child(canvas)
	var rect := TextureRect.new()
	rect.texture = viewport.get_texture()
	rect.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	rect.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	rect.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	rect.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var material := CanvasItemMaterial.new()
	material.blend_mode = CanvasItemMaterial.BLEND_MODE_PREMULT_ALPHA
	rect.material = material
	canvas.add_child(rect)
