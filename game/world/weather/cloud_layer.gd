class_name CloudLayer
extends Node3D
## The volumetric clouds near the camera (M8 weather and clouds, Task 8): owns the full-screen quad with
## shaders/cloud_layer.gdshader and every `cloud_*` global uniform (project.godot [shader_globals]) - the two field
## textures with their blend, the noise textures, the coverage table, the optics numbers, the camera's float64-derived
## height and noise coordinates, and the quality preset.
##
## Field textures: the GPU form of a snapshot (StarsimCore.weather_load_snapshot -> cloud_texels, one RGBA8
## ImageTexture3D of columns x rows x 32 layers, no mipmaps). Only two are resident (a 0.25-deg snapshot is 133 MB).
## No pop on a pair or source change (anti-goal; controller ruling 2026-10-25): the visible state is
## mix(slot 0, slot 1, w1). When WeatherService announces a pair the slots do not hold, the weight first moves (FADE_S per
## unit weight) onto one slot - the wanted one if a slot holds a wanted snapshot - then the missing target snapshot is
## uploaded into the slot without weight and the weight moves to the target pair's blend; a pair that shares no snapshot
## with the slots goes through its dominant snapshot first (old -> new snapshot, then the real pair). Then w1 tracks
## the service's blend every frame. Weather off is the empty snapshot (a 2 x 2 x 32 texture of zeros): clouds fade out
## and in the same way.
## Uploads: the per-layer Images are cut on a worker (WorkerThreadPool), the texture is created on the main thread.

## [call] Seconds per unit weight of a cross-fade (pair / source change, weather on / off).
const FADE_S := 2.0
## Clouds are raymarched while the Earth is drawn in true space (CelestialBody.SCALED_SPACE_DISTANCE_M); beyond it the
## scaled-space Earth (Task 10) shows them.
const RAYMARCH_MAX_DISTANCE_M := CelestialBody.SCALED_SPACE_DISTANCE_M
## The shape and detail tiles (m): the same numbers as CLOUD_SHAPE_TILE_H_M / _V_M / CLOUD_DETAIL_TILE_M in
## shaders/clouds.gdshaderinc (the camera's noise coordinates are formed here in float64).
const SHAPE_TILE_H_M := 32000.0
const SHAPE_TILE_V_M := 16000.0
const DETAIL_TILE_M := 2000.0
const WGS84_A_M := 6378137.0
## a / b of WGS84 (the model space's polar stretch, as Atmosphere.POLAR_STRETCH).
const POLAR_STRETCH := 1.0 / (1.0 - 1.0 / 298.257223563)
const LAYERS := 32

enum Quality { DESKTOP, VR }
## [E] Quality presets: coarse steps spanning the segment, the cap on all steps, the fine step (fraction of the
## distance) inside the field's cloudy region, samples toward the light per in-cloud sample, the detail noise's
## amplitude at the camera and the distance where it has faded to 0 (cloud_check.gd and terrain_perf.gd measure them).
const PRESETS := {
	Quality.DESKTOP: {"steps": 128, "max_steps": 256, "fine_step_fraction": 0.012, "light_steps": 6,
			"detail_amplitude": 1.0, "detail_distance_m": 30000.0},
	Quality.VR: {"steps": 64, "max_steps": 128, "fine_step_fraction": 0.025, "light_steps": 4,
			"detail_amplitude": 1.0, "detail_distance_m": 15000.0},
}

var quality: Quality = Quality.DESKTOP:
	set(value):
		quality = value
		_apply_quality()
var noise_ready := false  ## noise and optics uploaded
var last_upload_ms := 0.0  ## main-thread time of the last field texture creation
var field_texture_bytes := 0  ## both resident field textures (RGBA8 texels)
var pinned := false  ## show_now was used: set_target is ignored

var _core: StarsimCore
var _quad := MeshInstance3D.new()
var _material := ShaderMaterial.new()
var _noise_task := -1
var _noise_result := {}
var _noise_textures: Array = []
## Slots: {id, size: Vector3 (columns, rows, layers), texture, bytes}.
var _slots: Array = [{}, {}]
var _w1 := 0.0  ## weight of slot 1 (cloud_blend)
var _tracking := false
var _target: Array = [{}, {}]  ## the service's pair (loaded snapshot dictionaries; {} = none)
var _target_blend := 0.0
var _upload := {}  ## the running upload: {slot, id, size, bytes, task, slot_data}
var _empty_texture: ImageTexture3D
var _frame := 0


func _init() -> void:
	name = "CloudLayer"


## Starts the noise generation on a worker and sets the optics; the quad joins the scene.
func setup(core: StarsimCore) -> void:
	_core = core
	var quad := QuadMesh.new()
	quad.size = Vector2(2.0, 2.0)
	quad.custom_aabb = AABB(Vector3(-1e12, -1e12, -1e12), Vector3(2e12, 2e12, 2e12))  # never culled
	_material.shader = load("res://shaders/cloud_layer.gdshader")
	_material.render_priority = Material.RENDER_PRIORITY_MIN  # first in the transparent pass: plumes draw over it
	_quad.mesh = quad
	_quad.material_override = _material
	_quad.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_quad.extra_cull_margin = 16384.0
	add_child(_quad)
	_empty_texture = _make_texture(PackedByteArray(), 2, 2)
	for i in range(2):
		_slots[i] = {"id": "", "size": Vector3(2, 2, LAYERS), "texture": _empty_texture, "bytes": 0}
	_bind_slots()
	var optics: Dictionary = core.cloud_optics_parameters()
	var rs := RenderingServer
	rs.global_shader_parameter_set(&"cloud_phase", Vector4(optics["hg_g"], optics["draine_g"], optics["draine_alpha"],
			optics["draine_weight"]))
	rs.global_shader_parameter_set(&"cloud_asymmetry_g", float(optics["asymmetry_g"]))
	rs.global_shader_parameter_set(&"cloud_extinction_step_per_m", float(optics["extinction_step_per_m"]))
	rs.global_shader_parameter_set(&"cloud_depth_code", Vector2(optics["depth_code_min"], optics["depth_code_max"]))
	rs.global_shader_parameter_set(&"cloud_enabled", false)
	rs.global_shader_parameter_set(&"cloud_debug_output", 0)
	_apply_quality()
	_noise_task = WorkerThreadPool.add_task(func() -> void: _noise_result = core.cloud_noise(), false, "cloud noise")


## Blocks until the noise is uploaded (checks).
func wait_until_ready() -> bool:
	if _noise_task >= 0:
		WorkerThreadPool.wait_for_task_completion(_noise_task)
		_noise_task = -1
		_upload_noise()
	return noise_ready


func shutdown() -> void:
	if _noise_task >= 0:
		WorkerThreadPool.wait_for_task_completion(_noise_task)
		_noise_task = -1
	if _upload.has("task"):
		WorkerThreadPool.wait_for_task_completion(_upload["task"])
		_upload = {}


## WeatherService.snapshots_changed: the pair to show (dictionaries of weather_load_snapshot; {} = none).
func set_target(before: Dictionary, after: Dictionary) -> void:
	if not pinned:
		_target = [before, after]


## Checks and render_check: shows a pair at once (synchronous uploads, no cross-fade) and pins it (the service's
## announcements are ignored from then on).
func show_now(before: Dictionary, after: Dictionary, blend: float) -> void:
	pinned = true
	_target = [before, after]
	for i in range(2):
		var data := _build_slot(_snapshot_id(_target[i]), _target[i], _slice_images(_target[i]))
		_slots[i] = data
	_w1 = clampf(blend, 0.0, 1.0)
	_target_blend = _w1
	_tracking = true
	_bind_slots()
	RenderingServer.global_shader_parameter_set(&"cloud_blend", _w1)


## Per frame: the cross-fade, the camera's float64 terms and whether to draw. camera_fixed_m: the camera in the Earth
## fixed frame (ITRF, m); blend: WeatherService.blend().
func update(camera_fixed_m: Vector3, blend: float, delta_s: float) -> void:
	if _noise_task >= 0 and WorkerThreadPool.is_task_completed(_noise_task):
		WorkerThreadPool.wait_for_task_completion(_noise_task)
		_noise_task = -1
		_upload_noise()
	if not pinned:
		_target_blend = clampf(blend, 0.0, 1.0)
	_poll_upload()
	_advance(maxf(delta_s, 0.0))
	var rs := RenderingServer
	rs.global_shader_parameter_set(&"cloud_blend", _w1)
	var camera_render := Frames.to_render(camera_fixed_m)
	var stretched := Vector3(camera_fixed_m.x, camera_fixed_m.y, camera_fixed_m.z * POLAR_STRETCH)
	var height := stretched.length() - WGS84_A_M
	rs.global_shader_parameter_set(&"cloud_camera_height_m", height)
	var k := 1.0 / SHAPE_TILE_V_M - 1.0 / SHAPE_TILE_H_M
	rs.global_shader_parameter_set(&"cloud_noise_origin",
			_wrap(camera_render / SHAPE_TILE_H_M + camera_render.normalized() * (height * k)))
	rs.global_shader_parameter_set(&"cloud_detail_origin", _wrap(camera_render / DETAIL_TILE_M))
	_frame = (_frame + 1) % 1024
	rs.global_shader_parameter_set(&"cloud_frame", _frame)
	var visible_weight := (1.0 - _w1) * float(_slots[0]["id"] != "") + _w1 * float(_slots[1]["id"] != "")
	rs.global_shader_parameter_set(&"cloud_enabled", noise_ready and visible_weight > 0.0
			and camera_fixed_m.length() < RAYMARCH_MAX_DISTANCE_M)


## The ids of the resident slots and the weight of slot 1 (tests).
func state() -> Dictionary:
	return {"slots": [_slots[0]["id"], _slots[1]["id"]], "w1": _w1, "tracking": _tracking,
			"uploading": not _upload.is_empty()}


# ---------------------------------------------------------------------------------------------------------------------
# Cross-fade state machine (class comment).

func _advance(delta_s: float) -> void:
	var step := delta_s / FADE_S
	var goal := _goal_weight()
	if not is_nan(goal):
		if _tracking:
			_w1 = goal
		else:
			_w1 = move_toward(_w1, goal, step)
			_tracking = is_equal_approx(_w1, goal)
		_release_unused()
		return
	_tracking = false
	if not _upload.is_empty():
		return  # an upload into the slot without weight is running
	var ids := [_snapshot_id(_target[0]), _snapshot_id(_target[1])]
	var holds := [ids.has(_slots[0]["id"]), ids.has(_slots[1]["id"])]
	var keep := 0 if _w1 <= 0.5 else 1
	if not holds[keep] and holds[1 - keep]:
		keep = 1 - keep  # fade toward the slot that holds a wanted snapshot
	var full := float(keep)
	if not is_equal_approx(_w1, full):
		_w1 = move_toward(_w1, full, step)
		return
	_w1 = full
	var kept_id: String = _slots[keep]["id"]
	var want: Dictionary
	if kept_id == ids[0]:
		want = _target[1]
	elif kept_id == ids[1]:
		want = _target[0]
	else:
		want = _target[0] if _target_blend <= 0.5 else _target[1]  # the dominant snapshot first
	_start_upload(1 - keep, want)


## A slot without weight that holds no wanted snapshot gives its texture back (weather off, one snapshot twice).
func _release_unused() -> void:
	var ids := [_snapshot_id(_target[0]), _snapshot_id(_target[1])]
	for s in range(2):
		var weight := _w1 if s == 1 else 1.0 - _w1
		if _slots[s]["id"] != "" and not ids.has(_slots[s]["id"]) and weight <= 0.0:
			_slots[s] = {"id": "", "size": Vector3(2, 2, LAYERS), "texture": _empty_texture, "bytes": 0}
			_bind_slots()


## The weight of slot 1 that shows the target pair at the service's blend, or NAN if the slots do not hold it.
func _goal_weight() -> float:
	var a := _snapshot_id(_target[0])
	var b := _snapshot_id(_target[1])
	var s0: String = _slots[0]["id"]
	var s1: String = _slots[1]["id"]
	if a == b:  # one snapshot twice (or none): one slot holding it suffices
		if s0 == a and s1 == a:
			return _w1
		if s0 == a:
			return 0.0
		if s1 == a:
			return 1.0
		return NAN
	if s0 == a and s1 == b:
		return _target_blend
	if s0 == b and s1 == a:
		return 1.0 - _target_blend
	return NAN


static func _snapshot_id(snapshot: Dictionary) -> String:
	if snapshot.is_empty() or not snapshot.get("ok", false):
		return ""
	return "%s|%s|%d" % [snapshot.get("kind", ""), snapshot.get("cache_key", ""), int(snapshot.get("rows", 0))]


func _start_upload(slot: int, snapshot: Dictionary) -> void:
	var id := _snapshot_id(snapshot)
	if id == "":
		_slots[slot] = {"id": "", "size": Vector3(2, 2, LAYERS), "texture": _empty_texture, "bytes": 0}
		_bind_slots()
		return
	var holder := {"images": []}
	var task := WorkerThreadPool.add_task(func() -> void: holder["images"] = _slice_images(snapshot), false,
			"cloud field images")
	_upload = {"slot": slot, "id": id, "snapshot": snapshot, "task": task, "holder": holder}


func _poll_upload() -> void:
	if _upload.is_empty() or not WorkerThreadPool.is_task_completed(_upload["task"]):
		return
	WorkerThreadPool.wait_for_task_completion(_upload["task"])
	var slot: int = _upload["slot"]
	_slots[slot] = _build_slot(_upload["id"], _upload["snapshot"], _upload["holder"]["images"])
	_upload = {}
	_bind_slots()


## The per-layer images of a snapshot's GPU form (worker-safe: no rendering calls).
static func _slice_images(snapshot: Dictionary) -> Array[Image]:
	var images: Array[Image] = []
	if _snapshot_id(snapshot) == "":
		return images
	var columns := int(snapshot["columns"])
	var rows := int(snapshot["rows"])
	var texels: PackedByteArray = snapshot["cloud_texels"]
	var layer_bytes := columns * rows * 4
	for layer in range(int(snapshot["layers"])):
		images.append(Image.create_from_data(columns, rows, false, Image.FORMAT_RGBA8,
				texels.slice(layer * layer_bytes, (layer + 1) * layer_bytes)))
	return images


func _build_slot(id: String, snapshot: Dictionary, images: Array[Image]) -> Dictionary:
	if id == "" or images.size() != LAYERS:
		return {"id": "", "size": Vector3(2, 2, LAYERS), "texture": _empty_texture, "bytes": 0}
	var start := Time.get_ticks_usec()
	var columns := int(snapshot["columns"])
	var rows := int(snapshot["rows"])
	var texture := ImageTexture3D.new()
	texture.create(Image.FORMAT_RGBA8, columns, rows, LAYERS, false, images)
	last_upload_ms = (Time.get_ticks_usec() - start) / 1000.0
	return {"id": id, "size": Vector3(columns, rows, LAYERS), "texture": texture, "bytes": columns * rows * LAYERS * 4}


func _make_texture(texels: PackedByteArray, columns: int, rows: int) -> ImageTexture3D:
	var images: Array[Image] = []
	for layer in range(LAYERS):
		var data := texels.slice(layer * columns * rows * 4, (layer + 1) * columns * rows * 4) if not texels.is_empty() \
				else PackedByteArray()
		if data.is_empty():
			data.resize(columns * rows * 4)
		images.append(Image.create_from_data(columns, rows, false, Image.FORMAT_RGBA8, data))
	var texture := ImageTexture3D.new()
	texture.create(Image.FORMAT_RGBA8, columns, rows, LAYERS, false, images)
	return texture


func _bind_slots() -> void:
	var rs := RenderingServer
	rs.global_shader_parameter_set(&"cloud_field_before", _slots[0]["texture"])
	rs.global_shader_parameter_set(&"cloud_field_after", _slots[1]["texture"])
	rs.global_shader_parameter_set(&"cloud_field_before_size", _slots[0]["size"])
	rs.global_shader_parameter_set(&"cloud_field_after_size", _slots[1]["size"])
	field_texture_bytes = int(_slots[0]["bytes"]) + int(_slots[1]["bytes"])


func _upload_noise() -> void:
	var noise := _noise_result
	if not noise.has("shape"):
		push_error("Wolken: Rauschtexturen fehlen: " + str(noise.get("error", "")))
		return
	_noise_textures = [_noise_texture(noise["shape"], int(noise["shape_size"])),
			_noise_texture(noise["detail"], int(noise["detail_size"]))]
	var thresholds: PackedFloat32Array = noise["coverage_threshold"]
	var table := ImageTexture.create_from_image(Image.create_from_data(thresholds.size(), 1, false, Image.FORMAT_RF,
			thresholds.to_byte_array()))
	_noise_textures.append(table)
	var rs := RenderingServer
	rs.global_shader_parameter_set(&"cloud_noise_shape", _noise_textures[0])
	rs.global_shader_parameter_set(&"cloud_noise_detail", _noise_textures[1])
	rs.global_shader_parameter_set(&"cloud_coverage_threshold", table)
	rs.global_shader_parameter_set(&"cloud_detail_mean", _detail_mean(noise["detail"]))
	_noise_result = {}
	noise_ready = true


static func _noise_texture(texels: PackedByteArray, size: int) -> ImageTexture3D:
	var images: Array[Image] = []
	var slice := size * size * 4
	for z in range(size):
		images.append(Image.create_from_data(size, size, false, Image.FORMAT_RGBA8, texels.slice(z * slice,
				(z + 1) * slice)))
	var texture := ImageTexture3D.new()
	texture.create(Image.FORMAT_RGBA8, size, size, size, false, images)
	return texture


## Mean of the detail value 0.625 R + 0.25 G + 0.125 B (clouds.gdshaderinc) over the texels: the filtered mean over
## uniform random positions equals the texel mean (trilinear weights sum to one over the wrapping texture).
static func _detail_mean(texels: PackedByteArray) -> float:
	var sum := Vector3.ZERO
	var count := texels.size() / 4
	for i in range(count):
		sum += Vector3(texels[4 * i], texels[4 * i + 1], texels[4 * i + 2])
	return (0.625 * sum.x + 0.25 * sum.y + 0.125 * sum.z) / (255.0 * count)


func _apply_quality() -> void:
	var preset: Dictionary = PRESETS[quality]
	var rs := RenderingServer
	rs.global_shader_parameter_set(&"cloud_steps", int(preset["steps"]))
	rs.global_shader_parameter_set(&"cloud_max_steps", int(preset["max_steps"]))
	rs.global_shader_parameter_set(&"cloud_fine_step_fraction", float(preset["fine_step_fraction"]))
	rs.global_shader_parameter_set(&"cloud_light_steps", int(preset["light_steps"]))
	rs.global_shader_parameter_set(&"cloud_detail_amplitude", float(preset["detail_amplitude"]))
	rs.global_shader_parameter_set(&"cloud_detail_distance_m", float(preset["detail_distance_m"]))


static func _wrap(q: Vector3) -> Vector3:
	return Vector3(q.x - floor(q.x), q.y - floor(q.y), q.z - floor(q.z))
