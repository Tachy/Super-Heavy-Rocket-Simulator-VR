class_name CloudLayer
extends Node3D
## The volumetric clouds near the camera (M8 weather and clouds, Task 8): owns the cloud pass (CloudEffect, a
## compositor effect: raymarch at reduced resolution, per-eye temporal accumulation, depth-aware upsampling), the
## full-screen composite quad (shaders/cloud_layer.gdshader: blends the pass's result over the scene first in the
## transparent pass) and every `cloud_*` global uniform (project.godot [shader_globals]) - the two field textures and
## their ice shares with the blend, the noise textures and tables, the optics numbers, the camera's float64-derived
## height and noise coordinates, and the quality preset. The world adds `effect` to its Compositor.
##
## Field textures: the GPU form of a snapshot (StarsimCore.weather_load_snapshot -> cloud_texels RGBA8 and
## cloud_ice_texels R8, columns x rows x 32 layers, no mipmaps; CloudFieldTexture uploads them in chunks of layers over
## frames). Only two snapshots are resident (a 0.25-deg one is 133 + 33 MB).
## No pop on a pair or source change (anti-goal; controller ruling 2026-10-25): the visible state is
## mix(slot 0, slot 1, w1). When WeatherService announces a pair the slots do not hold, the weight first moves (FADE_S per
## unit weight) onto one slot - the wanted one if a slot holds a wanted snapshot - then the missing target snapshot is
## uploaded into the slot without weight and the weight moves to the target pair's blend; a pair that shares no snapshot
## with the slots goes through its dominant snapshot first (old -> new snapshot, then the real pair). Then w1 tracks
## the service's blend every frame. Weather off is the empty snapshot (a 2 x 2 x 32 texture of zeros): clouds fade out
## and in the same way.

## [call] Seconds per unit weight of a cross-fade (pair / source change, weather on / off).
const FADE_S := 2.0
## Clouds are raymarched while the Earth is drawn in true space (CelestialBody.SCALED_SPACE_DISTANCE_M); beyond it the
## scaled-space Earth (Task 10) shows them.
const RAYMARCH_MAX_DISTANCE_M := CelestialBody.SCALED_SPACE_DISTANCE_M
## The shape, ice-sheet and detail tiles (m): the same numbers as CLOUD_SHAPE_TILE_H_M / _V_M, CLOUD_ICE_TILE_V_M and
## CLOUD_DETAIL_TILE_M in shaders/clouds.gdshaderinc (the camera's noise coordinates are formed here in float64).
const SHAPE_TILE_H_M := 32000.0
const SHAPE_TILE_V_M := 16000.0
const ICE_TILE_V_M := 2000.0
const DETAIL_TILE_M := 2000.0
const WGS84_A_M := 6378137.0
## a / b of WGS84 (the model space's polar stretch, as Atmosphere.POLAR_STRETCH).
const POLAR_STRETCH := 1.0 / (1.0 - 1.0 / 298.257223563)
const LAYERS := 32

enum Quality { DESKTOP, VR }
## [E] Quality presets: coarse steps spanning the segment, the cap on all steps, the fine step (fraction of the
## distance) inside the field's cloudy region; the light march toward the Sun / Moon (near samples over near_m through
## the shapes, geometric from first_m; far samples to the cloud top through the grid mean); the detail noise's
## amplitude at the camera and the distance where it has faded to 0; the cloud pass's resolution divisor and the
## temporal weight of a new frame (cloud_check.gd and terrain_perf.gd measure them).
const PRESETS := {
	Quality.DESKTOP: {"steps": 128, "max_steps": 256, "fine_step_fraction": 0.012, "light_near": 6, "light_far": 3,
			"light_first_m": 20.0, "light_near_m": 6000.0, "detail_amplitude": 1.0, "detail_distance_m": 30000.0,
			"divisor": 2, "temporal_weight": 0.1},
	Quality.VR: {"steps": 64, "max_steps": 128, "fine_step_fraction": 0.025, "light_near": 5, "light_far": 2,
			"light_first_m": 20.0, "light_near_m": 6000.0, "detail_amplitude": 1.0, "detail_distance_m": 15000.0,
			"divisor": 2, "temporal_weight": 0.1},
}

var quality: Quality = Quality.DESKTOP:
	set(value):
		quality = value
		_apply_quality()
var noise_ready := false  ## noise and optics uploaded
var last_upload_ms := 0.0  ## the longest frame step of the last field upload (main / render thread)
var field_texture_bytes := 0  ## both resident snapshots (field + ice share texels)
var pinned := false  ## show_now was used: set_target is ignored
var effect := CloudEffect.new()  ## the cloud pass (the world's Compositor)

var _core: StarsimCore
var _quad := MeshInstance3D.new()
var _material := ShaderMaterial.new()
var _noise_task := -1
var _noise_result := {}
var _noise_textures: Array = []
## Slots: {id, size: Vector3 (columns, rows, layers), field / ice: CloudFieldTexture or null (empty)}.
var _slots: Array = [{}, {}]
var _w1 := 0.0  ## weight of slot 1 (cloud_blend)
var _tracking := false
var _target: Array = [{}, {}]  ## the service's pair (loaded snapshot dictionaries; {} = none)
var _target_blend := 0.0
var _upload := {}  ## the running upload: {slot, id, size, field, ice}
var _empty_field: ImageTexture3D
var _empty_ice: ImageTexture3D
var _output_generation := -1


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
	_material.set_shader_parameter(&"cloud_image", effect.output)
	_quad.mesh = quad
	_quad.material_override = _material
	_quad.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_quad.extra_cull_margin = 16384.0
	add_child(_quad)
	_empty_field = _zero_texture(Image.FORMAT_RGBA8, 4)
	_empty_ice = _zero_texture(Image.FORMAT_R8, 1)
	for i in range(2):
		_slots[i] = _empty_slot()
	_bind_slots()
	var optics: Dictionary = core.cloud_optics_parameters()
	ShaderGlobals.set_value(&"cloud_phase", Vector4(optics["hg_g"], optics["draine_g"], optics["draine_alpha"],
			optics["draine_weight"]))
	ShaderGlobals.set_value(&"cloud_asymmetry_g", float(optics["asymmetry_g"]))
	ShaderGlobals.set_value(&"cloud_extinction_step_per_m", float(optics["extinction_step_per_m"]))
	ShaderGlobals.set_value(&"cloud_depth_code", Vector2(optics["depth_code_min"], optics["depth_code_max"]))
	ShaderGlobals.set_value(&"cloud_enabled", false)
	ShaderGlobals.set_value(&"cloud_debug_output", 0)
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
	if not _upload.is_empty():
		_upload["field"].release()
		_upload["ice"].release()
		_upload = {}


## WeatherService.snapshots_changed: the pair to show (dictionaries of weather_load_snapshot; {} = none).
func set_target(before: Dictionary, after: Dictionary) -> void:
	if not pinned:
		_target = [before, after]


## Checks and render_check: shows a pair at once (uploads completed now, no cross-fade, the history reset) and pins it
## (the service's announcements are ignored from then on).
func show_now(before: Dictionary, after: Dictionary, blend: float) -> void:
	pinned = true
	_target = [before, after]
	var olds: Array = [_slots[0], _slots[1]]
	for i in range(2):
		if i == 1 and _snapshot_id(_target[1]) == _snapshot_id(_target[0]):
			_slots[1] = _slots[0]  # one snapshot twice: one upload
			continue
		_slots[i] = _new_slot(_target[i])
		if _slots[i]["field"] != null:
			_slots[i]["field"].step(true)
			_slots[i]["ice"].step(true)
	for old: Dictionary in olds:
		_release_slot(old)
	_w1 = clampf(blend, 0.0, 1.0)
	_target_blend = _w1
	_tracking = true
	_bind_slots()
	ShaderGlobals.set_value(&"cloud_blend", _w1)
	effect.reset_history()


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
	if effect.output_generation != _output_generation:  # the pass re-created its result (a new size)
		_output_generation = effect.output_generation
		_material.set_shader_parameter(&"cloud_image", effect.output)
	ShaderGlobals.set_value(&"cloud_blend", _w1)
	var camera_render := Frames.to_render(camera_fixed_m)
	var stretched := Vector3(camera_fixed_m.x, camera_fixed_m.y, camera_fixed_m.z * POLAR_STRETCH)
	var height := stretched.length() - WGS84_A_M
	ShaderGlobals.set_value(&"cloud_camera_height_m", height)
	ShaderGlobals.set_value(&"cloud_noise_origin", _shape_origin(camera_render, height, SHAPE_TILE_V_M))
	ShaderGlobals.set_value(&"cloud_ice_origin", _shape_origin(camera_render, height, ICE_TILE_V_M))
	ShaderGlobals.set_value(&"cloud_detail_origin", _wrap(camera_render / DETAIL_TILE_M))
	var visible_weight := (1.0 - _w1) * float(_slots[0]["id"] != "") + _w1 * float(_slots[1]["id"] != "")
	ShaderGlobals.set_value(&"cloud_enabled", noise_ready and visible_weight > 0.0
			and camera_fixed_m.length() < RAYMARCH_MAX_DISTANCE_M)


## The ids of the resident slots and the weight of slot 1 (tests).
func state() -> Dictionary:
	return {"slots": [_slots[0]["id"], _slots[1]["id"]], "w1": _w1, "tracking": _tracking,
			"uploading": not _upload.is_empty()}


## Shape coordinates of the camera for the vertical tile tile_v_m (clouds.gdshaderinc cloud_shape_coordinates_for).
static func _shape_origin(camera_render: Vector3, height: float, tile_v_m: float) -> Vector3:
	var k := 1.0 / tile_v_m - 1.0 / SHAPE_TILE_H_M
	return _wrap(camera_render / SHAPE_TILE_H_M + camera_render.normalized() * (height * k))


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
			var old: Dictionary = _slots[s]
			_slots[s] = _empty_slot()
			_bind_slots()
			_release_slot(old)


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


func _empty_slot() -> Dictionary:
	return {"id": "", "size": Vector3(2, 2, LAYERS), "field": null, "ice": null}


## A slot for the snapshot with its volumes created (not yet uploaded); the empty slot for none.
func _new_slot(snapshot: Dictionary) -> Dictionary:
	var id := _snapshot_id(snapshot)
	if id == "":
		return _empty_slot()
	var columns := int(snapshot["columns"])
	var rows := int(snapshot["rows"])
	var ice: PackedByteArray = snapshot.get("cloud_ice_texels", PackedByteArray())
	if ice.size() != columns * rows * LAYERS:
		ice = PackedByteArray()
		ice.resize(columns * rows * LAYERS)  # no ice share given: liquid shapes
	return {"id": id, "size": Vector3(columns, rows, LAYERS),
		"field": CloudFieldTexture.create(snapshot["cloud_texels"], columns, rows, 4),
		"ice": CloudFieldTexture.create(ice, columns, rows, 1)}


## Frees a slot's volumes unless a resident slot still shows them (show_now shares one upload for a snapshot twice).
func _release_slot(slot: Dictionary) -> void:
	var field: Variant = slot.get("field")
	if field != null and field != _slots[0]["field"] and field != _slots[1]["field"]:
		slot["field"].release()
		slot["ice"].release()


func _start_upload(slot: int, snapshot: Dictionary) -> void:
	var data := _new_slot(snapshot)
	if data["id"] == "":
		var old: Dictionary = _slots[slot]
		_slots[slot] = data
		_bind_slots()
		_release_slot(old)
		return
	_upload = {"slot": slot, "data": data}


## One chunk of the running upload per frame; the slot is replaced when both volumes are complete.
func _poll_upload() -> void:
	if _upload.is_empty():
		return
	var data: Dictionary = _upload["data"]
	var field: CloudFieldTexture = data["field"]
	var ice: CloudFieldTexture = data["ice"]
	if not field.done:
		field.step()
	elif not ice.done:
		ice.step()
	if not (field.done and ice.done):
		return
	last_upload_ms = maxf(field.max_step_ms, ice.max_step_ms)
	var slot: int = _upload["slot"]
	var old: Dictionary = _slots[slot]
	_slots[slot] = data
	_upload = {}
	_bind_slots()
	_release_slot(old)


static func _zero_texture(format: Image.Format, texel_bytes: int) -> ImageTexture3D:
	var images: Array[Image] = []
	for layer in range(LAYERS):
		var data := PackedByteArray()
		data.resize(2 * 2 * texel_bytes)
		images.append(Image.create_from_data(2, 2, false, format, data))
	var texture := ImageTexture3D.new()
	texture.create(format, 2, 2, LAYERS, false, images)
	return texture


func _bind_slots() -> void:
	var names := [[&"cloud_field_before", &"cloud_ice_before", &"cloud_field_before_size"],
		[&"cloud_field_after", &"cloud_ice_after", &"cloud_field_after_size"]]
	field_texture_bytes = 0
	for s in range(2):
		var slot: Dictionary = _slots[s]
		var has := slot["field"] != null
		ShaderGlobals.set_value(names[s][0], slot["field"].texture if has else _empty_field)
		ShaderGlobals.set_value(names[s][1], slot["ice"].texture if has else _empty_ice)
		ShaderGlobals.set_value(names[s][2], slot["size"])
		if has and not (s == 1 and slot["field"] == _slots[0]["field"]):
			field_texture_bytes += slot["field"].bytes + slot["ice"].bytes


func _upload_noise() -> void:
	var noise := _noise_result
	if not noise.has("shape"):
		push_error("Wolken: Rauschtexturen fehlen: " + str(noise.get("error", "")))
		return
	var blue_size := int(noise["blue_noise_size"])
	_noise_textures = [_noise_texture(noise["shape"], int(noise["shape_size"])),
			_noise_texture(noise["detail"], int(noise["detail_size"])), _table(noise["coverage_threshold"]),
			_table(noise["edge_mean_factor"]),
			ImageTexture.create_from_image(Image.create_from_data(blue_size, blue_size, false, Image.FORMAT_R8,
					noise["blue_noise"]))]
	ShaderGlobals.set_value(&"cloud_noise_shape", _noise_textures[0])
	ShaderGlobals.set_value(&"cloud_noise_detail", _noise_textures[1])
	ShaderGlobals.set_value(&"cloud_coverage_threshold", _noise_textures[2])
	ShaderGlobals.set_value(&"cloud_edge_mean", _noise_textures[3])
	ShaderGlobals.set_value(&"cloud_blue_noise", _noise_textures[4])
	ShaderGlobals.set_value(&"cloud_edge_width", float(noise["edge_width_value"]))
	ShaderGlobals.set_value(&"cloud_detail_mean", _detail_mean(noise["detail"]))
	_noise_result = {}
	noise_ready = true


static func _table(values: PackedFloat32Array) -> ImageTexture:
	return ImageTexture.create_from_image(Image.create_from_data(values.size(), 1, false, Image.FORMAT_RF,
			values.to_byte_array()))


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
	ShaderGlobals.set_value(&"cloud_steps", int(preset["steps"]))
	ShaderGlobals.set_value(&"cloud_max_steps", int(preset["max_steps"]))
	ShaderGlobals.set_value(&"cloud_fine_step_fraction", float(preset["fine_step_fraction"]))
	var near := int(preset["light_near"])
	var first_m := float(preset["light_first_m"])
	var near_m := float(preset["light_near_m"])
	ShaderGlobals.set_value(&"cloud_light_march", Vector4(first_m, light_march_ratio(first_m, near_m, near),
			near_m, 0.0))
	ShaderGlobals.set_value(&"cloud_light_samples", Vector2i(near, int(preset["light_far"])))
	ShaderGlobals.set_value(&"cloud_detail_amplitude", float(preset["detail_amplitude"]))
	ShaderGlobals.set_value(&"cloud_detail_distance_m", float(preset["detail_distance_m"]))
	effect.set_quality(int(preset["divisor"]), float(preset["temporal_weight"]))


## Ratio r of the light march's geometric intervals: first_m (r^n - 1) / (r - 1) = near_m (core
## cloud_light_march_ratio, change both together); 1 for equal intervals.
static func light_march_ratio(first_m: float, near_m: float, n: int) -> float:
	var target := near_m / maxf(first_m, 1e-9)
	if target <= n:
		return 1.0
	var lo := 1.0
	var hi := 2.0
	while (pow(hi, n) - 1.0) / (hi - 1.0) < target:
		hi *= 2.0
	for i in range(100):
		var mid := 0.5 * (lo + hi)
		if (pow(mid, n) - 1.0) / (mid - 1.0) < target:
			lo = mid
		else:
			hi = mid
	return 0.5 * (lo + hi)


static func _wrap(q: Vector3) -> Vector3:
	return Vector3(q.x - floor(q.x), q.y - floor(q.y), q.z - floor(q.z))
