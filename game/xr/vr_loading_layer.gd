class_name VrLoadingLayer
extends Node3D
## The startup loading screen in the headset: a native OpenXR quad layer (the runtime composites it, so it stays sharp
## and needs no 3-D rendering) whose texture is a SubViewport holding the LoadingView. Child of XrManager.origin,
## placed once in front of the start head pose and then world-locked. While it is up, the XR camera sees nothing
## (cull mask 0, a black environment, no compositor effects), so the half-built world is neither drawn nor can its
## render effects run before they are set up; `release()` gives the camera back.

const QUAD_DISTANCE_M := 2.0  ## [E] a comfortable reading distance
const QUAD_WIDTH_M := 2.0  ## about 53 deg wide at 2 m
const VIEWPORT_SIZE := Vector2i(1920, 1080)

var supported := false  ## the runtime offers the quad layer (else nothing is shown in the headset)
var _viewport := SubViewport.new()
var _layer: OpenXRCompositionLayerQuad
var _camera: Camera3D
var _saved_cull_mask := 0
var _saved_environment: Environment
var _saved_compositor: Compositor


func _init() -> void:
	name = "VrLoadingLayer"


## Builds the layer with `view` as its picture and blanks `camera`'s 3-D view.
func setup(view: Control, camera: Camera3D) -> void:
	_viewport.size = VIEWPORT_SIZE
	_viewport.transparent_bg = false
	_viewport.disable_3d = true
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_viewport.add_child(view)
	add_child(_viewport)
	_layer = OpenXRCompositionLayerQuad.new()
	supported = _layer.is_natively_supported()
	if supported:
		_layer.layer_viewport = _viewport
		_layer.quad_size = Vector2(QUAD_WIDTH_M, QUAD_WIDTH_M * VIEWPORT_SIZE.y / VIEWPORT_SIZE.x)
		_layer.sort_order = 1  # in front of the (black) projection layer
		add_child(_layer)
	else:
		push_warning("VrLoadingLayer: the OpenXR runtime has no quad layer - no loading screen in the headset")
	_camera = camera
	_saved_cull_mask = camera.cull_mask
	_saved_environment = camera.environment
	_saved_compositor = camera.compositor
	var black := Environment.new()
	black.background_mode = Environment.BG_COLOR
	black.background_color = Color.BLACK
	camera.environment = black
	camera.compositor = Compositor.new()
	camera.cull_mask = 0


## Puts the quad QUAD_DISTANCE_M in front of `head` (origin space), upright, facing the head.
func place(head: Transform3D) -> void:
	var forward := -head.basis.z
	forward.y = 0.0
	forward = forward.normalized() if forward.length() > 1e-3 else Vector3.FORWARD
	var centre := head.origin + forward * QUAD_DISTANCE_M
	transform = Transform3D(Basis.looking_at(forward, Vector3.UP), centre)


## Gives the camera its world back (the quad stays until `free()`).
func release() -> void:
	if _camera == null:
		return
	_camera.cull_mask = _saved_cull_mask
	_camera.environment = _saved_environment
	_camera.compositor = _saved_compositor
	_camera = null
