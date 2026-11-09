class_name ScreenCamera
extends Node
## The Crew Dragon's left screen: external cameras (docs/subsystems/cockpit.md, "External cameras"). A SubViewport with its own
## Camera3D in the same World3D renders the scene from the selected camera (CameraCatalog) and the image goes onto
## CockpitView.screen_surface("left") through the display-referred path (shaders/screen_feed.gdshader). The camera sits at the
## mount's GCRF pose minus the main camera's GCRF position (the floating origin), converted in double: far cameras are
## accepted coarse (terrain, clouds and exposure are centred on the main camera; spec "Far cameras").
## Cost: one extra scene render at `size_px` every 1 / `update_hz` s, only while the cabin is the active view.

const SHADER := preload("res://shaders/screen_feed.gdshader")
const NEAR_M := 0.1
const FAR_M := 3.0e7
## The feed's own auto-exposure [call] (user decision, Task 7 fix round 1): a video camera meters its own image, not the cabin the
## pilot sits in. The shaders still scale the scene by the pilot eye's gain (large at night in the lit cabin), but the camera's
## auto-exposure (Godot's luminance reduction: the mean of the feed's own buffer) divides any such constant out. Paced like a
## video camera: AUTO_SPEED sets a ~1.5-3 s time constant; the clamp only guards the division.
const AUTO_SCALE := 0.4
const AUTO_SPEED := 1.4  ## Godot steps its meter by speed x frame time per feed render: ~1.5 s at 60 fps, ~3 s at 90 fps
## The clamp on the metered mean luminance (Godot's ISO-like properties: luminance = sensitivity x 0.125 / 100). The buffer holds the
## scene times the pilot eye's gain (the shaders apply it: ~1 by day, up to several 1000 at night), so the clamp is deliberately wide and
## fixed: the floor only keeps an all-black frame from dividing by zero (exposure at most 0.4 / 1.25e-3 = 320), the ceiling is out of reach.
## A gain-scaled clamp was tried and dropped: every change of an attribute makes Godot re-meter at once (a snap), and the gain moves all the time.
const AUTO_MIN_SENSITIVITY := 1.0
const AUTO_MAX_SENSITIVITY := 1.0e9
const LABEL_MARGIN_PX := 56  ## the label sits inside the part of the feed the screen shows (crop_y 0.9 cuts 32 px at the top)

var size_px := Vector2i(1024, 640)  ## feed resolution [E] (plan Task 7); lowered when the VR budget needs it
var update_hz := 30.0  ## feed rate [E]; measured in VR (Task 9)
var current_id := CameraCatalog.DEFAULT_ID  ## the selected camera; falls back to the default when its body is gone
var render_always := false  ## render although the cabin is not the active view (shots, tests)
var render_count := 0  ## feed renders requested (tests, perf)

var viewport: SubViewport
var camera: Camera3D
var _material: ShaderMaterial
var _config: CockpitConfig
var _sim: Object
var _available: Array[String] = []
var _since_render_s := 1.0e9
var _screen: MeshInstance3D  ## the mesh the material is on (re-applied when the cabin is rebuilt)
var _camera_fov_set := -1.0
var _label: Label
var _label_id := ""
var _label_language := ""


func _init() -> void:
	name = "ScreenCamera"
	viewport = SubViewport.new()
	viewport.name = "Feed"
	viewport.size = size_px
	viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	viewport.own_world_3d = false
	viewport.msaa_3d = Viewport.MSAA_DISABLED
	viewport.handle_input_locally = false
	viewport.gui_disable_input = true
	camera = Camera3D.new()
	camera.name = "FeedCamera"
	camera.keep_aspect = Camera3D.KEEP_WIDTH  # fov_deg of the config is the horizontal field
	camera.near = NEAR_M
	camera.far = FAR_M
	var attributes := CameraAttributesPractical.new()
	attributes.auto_exposure_enabled = true
	attributes.auto_exposure_scale = AUTO_SCALE
	attributes.auto_exposure_speed = AUTO_SPEED
	attributes.auto_exposure_min_sensitivity = AUTO_MIN_SENSITIVITY
	attributes.auto_exposure_max_sensitivity = AUTO_MAX_SENSITIVITY
	camera.attributes = attributes
	camera.cull_mask = 0xFFFFF & ~RenderLayers.OVERLAY & ~CockpitView.CONSOLE_LAYER  # no guidance overlays / HUD geometry in a camera image
	camera.compositor = Compositor.new()  # none of the main view's passes (cloud pass, eye-adaptation meter, display tone)
	viewport.add_child(camera)
	add_child(viewport)
	_label = Label.new()
	_label.name = "CameraName"
	_label.position = Vector2(LABEL_MARGIN_PX, LABEL_MARGIN_PX)
	_label.add_theme_font_size_override("font_size", 28)
	_label.add_theme_color_override("font_color", Color(1, 1, 1))
	_label.add_theme_color_override("font_outline_color", Color(0, 0, 0))
	_label.add_theme_constant_override("outline_size", 6)
	viewport.add_child(_label)
	_material = ShaderMaterial.new()
	_material.shader = SHADER
	_material.set_shader_parameter("feed", viewport.get_texture())
	_material.set_shader_parameter("live", 0.0)


## Selects a camera by id (offered or not: an id without a body falls back to the default at the next update).
func select(id: String) -> void:
	if CameraCatalog.entry_of(id, _config).is_empty():
		return
	current_id = id


## The next offered camera in config order after the current one (the first one when the current one is not offered).
func next() -> void:
	if _available.is_empty():
		return
	var at := _available.find(current_id)
	current_id = _available[(at + 1) % _available.size()] if at >= 0 else _available[0]


## The ids offered at the last update.
func available() -> Array[String]:
	return _available


## One frame. `cockpit`: the cabin view carrying the screen (null: no cabin shown); `active`: the cabin is the view being
## looked at (otherwise nothing renders). `camera_gcrf_m`: the main camera (floating origin). `_scene_gain` (the eye's pre-exposure,
## Atmosphere.exposure_gain x brightness) is unused: the feed meters its own image; kept so the caller's signature stays.
func update(sim: Object, config: CockpitConfig, cockpit: CockpitView, camera_gcrf_m: Vector3, delta_s: float, active: bool,
		_scene_gain := 1.0) -> void:
	_sim = sim
	_config = config
	_available = CameraCatalog.available(sim, config)
	if not _available.has(current_id):
		current_id = CameraCatalog.DEFAULT_ID
	var screen: MeshInstance3D = cockpit.screen_surface("left") if cockpit != null else null
	if screen != _screen:
		_screen = screen
		if _screen != null:
			_attach(config)
	var entry := CameraCatalog.entry_of(current_id, config)
	_update_label(entry)
	var live := _available.has(current_id)
	_material.set_shader_parameter("live", 1.0 if live else 0.0)
	if not live or not (active or render_always) or _screen == null:
		viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
		return
	var pose := CameraCatalog.mount_pose_gcrf(sim, current_id, config)
	var render_basis := Basis(Frames.to_render(pose.basis.x), Frames.to_render(pose.basis.y), Frames.to_render(pose.basis.z))
	camera.transform = Transform3D(render_basis, Frames.to_render(pose.origin - camera_gcrf_m))
	var fov := float(entry.get("fov_deg", 60.0))
	if fov != _camera_fov_set:
		camera.fov = fov
		_camera_fov_set = fov
	if viewport.size != size_px:
		viewport.size = size_px
		_update_crop(config)
	_since_render_s += delta_s
	if _since_render_s >= 1.0 / update_hz:
		_since_render_s = 0.0
		viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
		render_count += 1


## The camera's name (name_key through I18n) in the feed's top left corner; follows the language.
func _update_label(entry: Dictionary) -> void:
	_label.show()  # a Control under a SubViewport came up hidden: shown every frame
	var language := I18n.language()
	if _label_id == current_id and _label_language == language:
		return
	_label_id = current_id
	_label_language = language
	_label.text = I18n.text(String(entry.get("name_key", "")))


func _attach(config: CockpitConfig) -> void:
	_update_crop(config)
	_screen.material_override = _material  # CockpitView set its placeholder as the override, which beats surface materials


## The feed is less wide than the screen: it shows the feed's middle part of the height (no stretching).
func _update_crop(config: CockpitConfig) -> void:
	var screen: Dictionary = config.screens.get("left", {})
	var size: Vector2 = screen.get("size", Vector2(16.0, 9.0))
	var feed_aspect := float(size_px.x) / float(size_px.y)
	_material.set_shader_parameter("crop_y", minf(1.0, feed_aspect / (size.x / size.y)))
