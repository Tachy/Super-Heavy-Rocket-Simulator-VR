class_name StereoMockXr
extends XRInterfaceExtension
## A stand-in for the headset's OpenXR interface in an offscreen run (render_check --stereo=<json>): two views whose
## projections, eye transforms and render target size come from a recorded JSON, so the multiview render path (and
## the cloud pass's per-eye march) runs without a headset. The JSON is written by PerfLog at a real VR start
## (data/perf/xr_projections_<stamp>.json, committed as game/tests/data/pimax_dream_air_projections.json):
##   {"render_target": [w, h], "views": [{"projection": [16 floats, column-major], "eye_from_head": [12 floats: the
##    basis' three columns, then the origin]}, {...}], "near_m": n, "far_m": f}
## The projection is taken apart into its frustum tangents and rebuilt for the near / far the renderer asks for (as the
## OpenXR interface does from the runtime's field of view), so the recorded matrix itself comes back for the recorded
## near / far.
## Capture: with capture_requested set the next drawn frame's eye images are read back (after _post_draw_viewport).

const CAPTURE_NONE := 0

var render_target_px := Vector2i(1024, 1024)
var near_m := 0.2
var far_m := 3.0e7
## Per view: [tan_left, tan_right, tan_bottom, tan_top] of the recorded frustum.
var _tangents: Array[PackedFloat64Array] = []
var _eye_from_head: Array[Transform3D] = []
var _projections: Array[Projection] = []  ## as recorded (tests)
var _initialized := false

var capture_requested := false  ## set by the caller: the next drawn frame's views are read back into captured
var captured: Array[Image] = []  ## per view (after a capture)
var capture_error := ""
var capture_info := ""  ## the captured texture's format, layers and size
var frames_drawn := 0


## Loads the recorded JSON; false (with a push_error) when the file is missing or malformed.
func load(path: String) -> bool:
	var text := FileAccess.get_file_as_string(path)
	if text == "":
		push_error("[StereoMockXr] cannot read %s" % path)
		return false
	var parsed: Variant = JSON.parse_string(text)
	if not parsed is Dictionary:
		push_error("[StereoMockXr] %s is not a JSON object" % path)
		return false
	return load_dictionary(parsed)


func load_dictionary(data: Dictionary) -> bool:
	var views: Array = data.get("views", [])
	var target: Array = data.get("render_target", [])
	if views.is_empty() or target.size() != 2:
		push_error("[StereoMockXr] needs render_target [w, h] and at least one view")
		return false
	_tangents.clear()
	_eye_from_head.clear()
	_projections.clear()
	for view: Dictionary in views:
		var p: Array = view.get("projection", [])
		var t: Array = view.get("eye_from_head", [])
		if p.size() != 16 or t.size() != 12:
			push_error("[StereoMockXr] a view needs 16 projection and 12 eye_from_head values")
			return false
		var projection := Projection(Vector4(p[0], p[1], p[2], p[3]), Vector4(p[4], p[5], p[6], p[7]),
				Vector4(p[8], p[9], p[10], p[11]), Vector4(p[12], p[13], p[14], p[15]))
		_projections.append(projection)
		_tangents.append(_frustum_tangents(projection))
		_eye_from_head.append(Transform3D(Basis(Vector3(t[0], t[1], t[2]), Vector3(t[3], t[4], t[5]),
				Vector3(t[6], t[7], t[8])), Vector3(t[9], t[10], t[11])))
	render_target_px = Vector2i(int(target[0]), int(target[1]))
	near_m = float(data.get("near_m", near_m))
	far_m = float(data.get("far_m", far_m))
	return true


## The frustum edges at unit distance from a perspective projection (Projection.create_frustum's layout, depth
## correction not applied): [left, right, bottom, top] tangents.
static func _frustum_tangents(p: Projection) -> PackedFloat64Array:
	var sx: float = p.x.x
	var sy: float = p.y.y
	return PackedFloat64Array([(p.z.x - 1.0) / sx, (p.z.x + 1.0) / sx, (p.z.y - 1.0) / sy, (p.z.y + 1.0) / sy])


func view_projection(view: int, near: float, far: float) -> Projection:
	var t := _tangents[view]
	return Projection.create_frustum(t[0] * near, t[1] * near, t[2] * near, t[3] * near, near, far)


# --- XRInterfaceExtension -------------------------------------------------------------------------------------------

func _get_name() -> StringName:
	return &"StereoMock"


func _get_capabilities() -> int:
	return XRInterface.XR_STEREO


func _is_initialized() -> bool:
	return _initialized


func _initialize() -> bool:
	_initialized = not _tangents.is_empty()
	return _initialized


func _uninitialize() -> void:
	_initialized = false


func _get_system_info() -> Dictionary:
	return {"XRRuntimeName": "StereoMockXr"}


func _supports_play_area_mode(mode: int) -> bool:
	return mode == XRInterface.XR_PLAY_AREA_3DOF


func _get_play_area_mode() -> int:
	return XRInterface.XR_PLAY_AREA_3DOF


func _set_play_area_mode(_mode: int) -> bool:
	return true


func _get_tracking_status() -> int:
	return XRInterface.XR_NORMAL_TRACKING


func _get_render_target_size() -> Vector2:
	return Vector2(render_target_px)


func _get_view_count() -> int:
	return _tangents.size()


func _get_camera_transform() -> Transform3D:
	return Transform3D.IDENTITY


func _get_transform_for_view(view: int, cam_transform: Transform3D) -> Transform3D:
	return cam_transform * _eye_from_head[view]


func _get_projection_for_view(view: int, _aspect: float, z_near: float, z_far: float) -> PackedFloat64Array:
	var p := view_projection(view, z_near, z_far)
	return PackedFloat64Array(XrProjectionRecord.projection_to_array(p))


## Render thread, after the viewport is drawn: the capture of the eye images (a 2D array texture, one layer per view).
func _post_draw_viewport(render_target: RID, _screen_rect: Rect2) -> void:
	frames_drawn += 1
	if not capture_requested:
		return
	capture_requested = false
	captured = _read_views(render_target)


func _read_views(render_target: RID) -> Array[Image]:
	var images: Array[Image] = []
	var rd := RenderingServer.get_rendering_device()
	var texture := get_render_target_texture(render_target)
	if rd == null or not texture.is_valid():
		capture_error = "no render target texture"
		return images
	var format := rd.texture_get_format(texture)
	capture_info = "format %d, %d layers, %d x %d" % [format.format, format.array_layers, format.width, format.height]
	var image_format := Image.FORMAT_RGBA8
	var swap_rb := false
	match format.format:
		RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM, RenderingDevice.DATA_FORMAT_R8G8B8A8_SRGB:
			pass
		RenderingDevice.DATA_FORMAT_B8G8R8A8_UNORM, RenderingDevice.DATA_FORMAT_B8G8R8A8_SRGB:
			swap_rb = true
		RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT:
			image_format = Image.FORMAT_RGBAH
		_:
			capture_error = "unsupported render target format %d" % format.format
			return images
	for layer in range(maxi(format.array_layers, 1)):
		var bytes := rd.texture_get_data(texture, layer)
		var image := Image.create_from_data(format.width, format.height, false, image_format, bytes)
		if swap_rb:
			for y in range(image.get_height()):
				for x in range(image.get_width()):
					var c := image.get_pixel(x, y)
					image.set_pixel(x, y, Color(c.b, c.g, c.r, c.a))
		if image_format == Image.FORMAT_RGBAH:
			image.convert(Image.FORMAT_RGBA8)
		images.append(image)
	return images


## Provisional stand-in for the recorded headset (until game/tests/data/pimax_dream_air_projections.json exists): the
## stereo test's FOV_LEFT_EYE (cloud_vr_stereo_test.gd, [S: Meta Quest 2 runtime]) mirrored for the right eye, IPD
## 0.064 m, at the Pimax Dream Air's expected per-eye target 3534 x 2838.
static func provisional_dictionary(target := Vector2i(3534, 2838), ipd_m := 0.064, near := 0.2,
		far := 3.0e7) -> Dictionary:
	var fov := {"left": -0.9424778, "right": 0.6981317, "up": 0.8203047, "down": -0.8377581}
	var views := []
	for view in range(2):
		var left: float = fov["left"] if view == 0 else -fov["right"]
		var right: float = fov["right"] if view == 0 else -fov["left"]
		var projection := Projection.create_frustum(tan(left) * near, tan(right) * near, tan(fov["down"]) * near,
				tan(fov["up"]) * near, near, far)
		var sign := -1.0 if view == 0 else 1.0
		views.append({"projection": XrProjectionRecord.projection_to_array(projection),
				"eye_from_head": XrProjectionRecord.transform_to_array(
						Transform3D(Basis.IDENTITY, Vector3(sign * ipd_m / 2.0, 0.0, 0.0)))})
	return {"render_target": [target.x, target.y], "views": views, "near_m": near, "far_m": far}
