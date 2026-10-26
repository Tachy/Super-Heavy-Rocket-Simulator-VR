class_name CloudLayer
extends Node3D
## The volumetric clouds near the camera (M8 weather and clouds, Task 8): owns the cloud pass (CloudEffect, a
## compositor effect: raymarch at reduced resolution, per-eye temporal accumulation, depth-aware upsampling), the
## full-screen composite quad (shaders/cloud_layer.gdshader: blends the pass's result over the scene first in the
## transparent pass) and every `cloud_*` global uniform (project.godot [shader_globals]) - the two field textures and
## their ice shares with the blend, the noise textures and tables, the optics numbers, the camera's float64-derived
## height and noise coordinates, and the quality preset. The world adds `effect` to its Compositor.
##
## Field textures: the GPU form of a snapshot (StarsimCore.weather_load_snapshot -> cloud_texels RGBA8,
## cloud_ice_texels R8 and cloud_column_texels RG8 (the grid-mean column and its cover above each layer centre: the
## light under the clouds, Task 9), columns x rows x 32 layers, no mipmaps; CloudFieldTexture uploads them in chunks of
## layers over frames). Only two snapshots are resident (a 0.25-deg one is 133 + 33 + 66 MB).
## Light on the ground (Task 9): the cloud shadow maps (shadow_effect, CloudShadowEffect before the opaque pass) and
## their frame (`cloud_shadow_*` globals, set here every frame: anchors near the ground under the camera, re-anchored
## only when the camera moved SHADOW_*_REANCHOR_M away - the maps are Earth-fixed in between, no swimming), the slab
## transmittance table of the Sun's diffuse light (`cloud_slab_transmittance`).
## No pop on a pair or source change (anti-goal; controller ruling 2026-10-25): the visible state is
## mix(slot 0, slot 1, w1). When WeatherService announces a pair the slots do not hold, the weight first moves (FADE_S per
## unit weight) onto one slot - the wanted one if a slot holds a wanted snapshot - then the missing target snapshot is
## uploaded into the slot without weight and the weight moves to the target pair's blend; a pair that shares no snapshot
## with the slots goes through its dominant snapshot first (old -> new snapshot, then the real pair). Then w1 tracks
## the service's blend every frame. Weather off is the empty snapshot (a 2 x 2 x 32 texture of zeros): clouds fade out
## and in the same way.
## The CPU light (StarsimCore.weather_cloud_light: vehicles, eye, ambient) follows the very same visible state: this node is
## the only writer of the core's active pair. It pushes (slot 0 file, slot 1 file, w1) whenever a slot's content changes or
## w1 moved by LIGHT_PUSH_STEP; an empty slot is "" (the core blends toward zero depths, as the GPU's zero column texture).
## A file that fell out of the core's load cache is not read on the main thread: `core_cache_missed` asks the owner for
## a worker load and the push is retried every frame.

## [call] Seconds per unit weight of a cross-fade (pair / source change, weather on / off).
const FADE_S := 2.0
## [call] w1 must move this much before the CPU light's pair is pushed again (slot changes are pushed at once).
const LIGHT_PUSH_STEP := 0.01
## Clouds are raymarched while the Earth is drawn in true space (CelestialBody.SCALED_SPACE_DISTANCE_M, the same
## camera distance from the Earth's centre that switches the terrain quadtree to the scaled-space mesh); beyond it the
## scaled-space Earth draws them (World switches it to earth_clouds.gdshader while far_enabled; cloud_far.gdshaderinc,
## Task 10). One limit, so exactly one of the two is drawn.
const RAYMARCH_MAX_DISTANCE_M := CelestialBody.SCALED_SPACE_DISTANCE_M
## The shape, ice-sheet and level-0 detail tiles (m) and the stratiform stretch: the same numbers as
## CLOUD_SHAPE_TILE_H_M / _V_M, CLOUD_ICE_TILE_V_M, CLOUD_DETAIL_TILE_M and CLOUD_STRATIFORM_STRETCH in
## shaders/clouds.gdshaderinc (the camera's noise coordinates are formed here in float64).
const SHAPE_TILE_H_M := 32000.0
const SHAPE_TILE_V_M := 16000.0
const ICE_TILE_V_M := 2000.0
const STRATIFORM_STRETCH := 4.0
const DETAIL_TILE_M := 1000.0
const WGS84_A_M := 6378137.0
## a / b of WGS84 (the model space's polar stretch, as Atmosphere.POLAR_STRETCH).
const POLAR_STRETCH := 1.0 / (1.0 - 1.0 / 298.257223563)
const LAYERS := 32
## The Monte Carlo transfer table of the multiply scattered light (core cloud_transfer_table.hpp), in config/.
const TRANSFER_TABLE := "clouds/cloud_transfer_table.bin"
## [E] Cloud shadow maps (shaders/cloud_light.gdshaderinc): half extents (the near map 512 texels of 62.5 m: the
## shapes' edges 25-200 m), re-anchoring distances (the camera stays well inside each map) and the distance rule's fades
## (horizontal distance of the receiver from the camera): near -> far, far -> the column.
const SHADOW_MAP_SIZE := 512
const SHADOW_NEAR_HALF_M := 16000.0
const SHADOW_FAR_HALF_M := 200000.0
const SHADOW_NEAR_REANCHOR_M := 4000.0
const SHADOW_FAR_REANCHOR_M := 50000.0
const SHADOW_NEAR_FADE_M := Vector2(9000.0, 11000.0)
const SHADOW_FAR_FADE_M := Vector2(120000.0, 140000.0)

## A file of a resident slot is not in the core's load cache (the CPU light cannot use it): load it on a worker.
signal core_cache_missed(file: String)

enum Quality { DESKTOP, VR }
## The multiple-scattering model (fidelity flag, shader global cloud_scattering_model): the Monte Carlo transfer table
## (the reference) or the closed form (TMS on the delta-Eddington source function) for comparison.
enum ScatteringModel { CLOSED_FORM, TABLE }
## [E] Quality presets: coarse steps spanning the segment, the cap on all steps, the fine step (fraction of the
## distance) inside the field's cloudy region; the light march toward the Sun / Moon (near samples over near_m through
## the shapes, geometric from first_m; far samples to the cloud top through the grid mean); the detail levels used
## (each band-limited at the pixel footprint, cloud_noise.hpp); the cloud pass's resolution divisor and the floor of the
## temporal weight of a new frame (a running mean of up to 32 frames; cloud_check.gd, terrain_perf.gd measure them).
## VR divisor 3 of the centre view (VR performance part 2): one march for both eyes over ~1.24 x one eye's width at
## 1/9 of its pixels costs about what two eyes at divisor 4 did (1/16 each; divisor 2 per eye cost 11-13 ms of GPU at
## the headset's 20 MP, docs/subsystems/vr-performance.md), sharper and the same in both eyes. VR flicker (user
## 2026-10-26: the clouds need not look new every frame, only volumetric and in perspective): the history keeps up to 96
## frames (noise floor sqrt(w / (2 - w)): 7 % of one frame's instead of 13 % at 32) and is clipped to the
## neighbourhood's mean +- clip_gamma standard deviations instead of its min / max (which threw the history away at
## every motion: the flicker); clip_gamma 0 keeps the min / max (the desktop's approved look). The temporal weight and
## clip apply only to the jittered march (deterministic_march off). VR steps (static clouds Task 3, deterministic
## march): 48 coarse steps (4.3 % of the distance) lost cumulus bases at a fixed sample point - cloud_check case 9 off
## by 5 px, the deck view at 9 km thinned: 96 / 192 coarse / max steps. The fine fraction stays 0.025 (case 9 passes
## with it; 0.012 was a look choice at about twice the in-cloud samples, cost in docs/subsystems/vr-performance.md).
const PRESETS := {
	Quality.DESKTOP: {"steps": 128, "max_steps": 256, "fine_step_fraction": 0.012, "light_near": 6, "light_far": 3,
			"light_first_m": 20.0, "light_near_m": 6000.0, "detail_levels": 3, "divisor": 2,
			"temporal_weight": 1.0 / 32.0, "clip_gamma": 0.0},
	Quality.VR: {"steps": 96, "max_steps": 192, "fine_step_fraction": 0.025, "light_near": 5, "light_far": 2,
			"light_first_m": 20.0, "light_near_m": 6000.0, "detail_levels": 3, "divisor": 3,
			"temporal_weight": 1.0 / 96.0, "clip_gamma": 1.5},
}

var quality: Quality = Quality.DESKTOP:
	set(value):
		quality = value
		_apply_quality()
var scattering_model: ScatteringModel = ScatteringModel.TABLE:
	set(value):
		scattering_model = value
		_apply_scattering_model()
var noise_ready := false  ## noise and optics uploaded
var transfer_ready := false  ## the transfer table is uploaded (else the closed form is used)
var last_upload_ms := 0.0  ## the longest frame step of the last field upload (main / render thread)
var field_texture_bytes := 0  ## both resident snapshots (field + ice share texels)
var pinned := false  ## show_now was used: set_target is ignored
## The scaled-space Earth draws the clouds (weather shown, the camera beyond RAYMARCH_MAX_DISTANCE_M; Task 10).
var far_enabled := false
var effect := CloudEffect.new()  ## the cloud pass (the world's Compositor)
const SHADOW_RUN_WINDOW_MSEC := 250  ## how long after its last run the shadow effect still counts as running
var shadow_effect := CloudShadowEffect.new(SHADOW_MAP_SIZE)  ## the cloud shadow maps (the world's Compositor)
## The light cache (the Sun's tau_light looked up, not marched; planned here every frame, filled by the effect on the
## render thread; its layout and refresh rules: CloudLightCacheSchedule).
var light_cache := CloudLightCache.new()
## Fidelity flag (shader global cloud_light_cache_enabled): false marches the light per sample everywhere (the
## reference the cache reproduces, cloud_check.gd section 8).
var light_cache_enabled := true:
	set(value):
		light_cache_enabled = value
		ShaderGlobals.set_value(&"cloud_light_cache_enabled", value)
## Fidelity flag (shader global cloud_deterministic_march; static clouds Task 3): true marches every ray at fixed
## sample points (the step phase on Earth-fixed height shells, the stratified midpoints, the silhouette refined by
## bisection, the shape prefilter at its footprint's centre) - an unchanged view gives a bit-identical image - and the
## temporal kernel only copies the current frame; false: the per-pixel and per-frame jitter averaged by the temporal
## accumulation (the reference of the converged image, cloud_check.gd case 9).
var deterministic_march := true:
	set(value):
		deterministic_march = value
		ShaderGlobals.set_value(&"cloud_deterministic_march", value)

var _core: StarsimCore
var _quad := MeshInstance3D.new()
var _material := ShaderMaterial.new()
var _noise_task := -1
var _noise_result := {}
var _noise_textures: Array = []
var _transfer_result := {}
var _transfer_texture: ImageTexture3D
## Slots: {id, size: Vector3 (columns, rows, layers), field / ice: CloudFieldTexture or null (empty)}.
var _slots: Array = [{}, {}]
var _w1 := 0.0  ## weight of slot 1 (cloud_blend)
var _tracking := false
var _target: Array = [{}, {}]  ## the service's pair (loaded snapshot dictionaries; {} = none)
var _target_blend := 0.0
var _upload := {}  ## the running upload: {slot, id, size, field, ice}
var _empty_field: ImageTexture3D
var _empty_ice: ImageTexture3D
var _empty_column: ImageTexture3D  ## zero (cover 0, depth 0) LA8: an R8 texture would read as cover 1 through .ra
var _pushed_light := {"files": ["", ""], "w1": 0.0, "ok": true}  ## the core's pair as last accepted
var _slab_transmittance: ImageTexture
## Earth-fixed anchors of the shadow maps (render axes, float64; NAN: none yet).
var _near_anchor := Vector3(NAN, NAN, NAN)
var _far_anchor := Vector3(NAN, NAN, NAN)
## The light cache's view of the field: the settings it was filled with and the slots' ids (_update_light_cache).
var _cache_key := []
var _cache_slot_ids := ["", ""]
var _clock_jumped := false  ## notify_clock_jump since the last plan
var _cache_usec := -1  ## wall clock of the last plan (the cross-fade runs in real time)


func _init() -> void:
	name = "CloudLayer"
	effect.light_cache = light_cache


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
	_empty_column = _zero_texture(Image.FORMAT_LA8, 2)
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
	ShaderGlobals.set_value(&"cloud_shadow_valid", false)
	ShaderGlobals.set_value(&"cloud_debug_output", 0)
	ShaderGlobals.set_value(&"cloud_light_cache_enabled", light_cache_enabled)
	ShaderGlobals.set_value(&"cloud_deterministic_march", deterministic_march)
	ShaderGlobals.set_value(&"cloud_shadow_near", shadow_effect.near_texture)
	ShaderGlobals.set_value(&"cloud_shadow_far", shadow_effect.far_texture)
	ShaderGlobals.set_value(&"cloud_shadow_extent", Vector4(SHADOW_NEAR_HALF_M, SHADOW_FAR_HALF_M,
			SHADOW_NEAR_FADE_M.x, SHADOW_NEAR_FADE_M.y))
	ShaderGlobals.set_value(&"cloud_shadow_far_fade", SHADOW_FAR_FADE_M)
	_apply_quality()
	_apply_scattering_model()
	var table_file := Paths.config_dir().path_join(TRANSFER_TABLE)
	var load_tables := func() -> void:
		_noise_result = core.cloud_noise()
		_transfer_result = core.cloud_transfer_table(table_file)
	_noise_task = WorkerThreadPool.add_task(load_tables, false, "cloud noise")


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
	_cancel_upload()
	# The resident volumes (a Texture3DRD does not own its RID): unbound first, then freed.
	var olds: Array = [_slots[0], _slots[1]]
	_slots = [_empty_slot(), _empty_slot()]
	_bind_slots()
	for old: Dictionary in olds:
		_release_slot(old)


## Drops the running upload (its volumes are freed; no slot shows them).
func _cancel_upload() -> void:
	if _upload.is_empty():
		return
	var data: Dictionary = _upload["data"]
	data["field"].release()
	data["ice"].release()
	data["column"].release()
	_upload = {}


## WeatherService.snapshots_changed: the pair to show (dictionaries of weather_load_snapshot; {} = none).
func set_target(before: Dictionary, after: Dictionary) -> void:
	if not pinned:
		_target = [before, after]
		for snapshot: Dictionary in _target:  # a snapshot a slot already holds needs no texels
			var id := _snapshot_id(snapshot)
			if id != "" and (id == _slots[0]["id"] or id == _slots[1]["id"]):
				_strip_texels(snapshot)


## Checks and render_check: shows a pair at once (uploads completed now, no cross-fade, the history reset) and pins it
## (the service's announcements are ignored from then on). The caller's dictionaries keep their texels (checks show the
## same one again).
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
			_slots[i]["column"].step(true)
	for old: Dictionary in olds:
		_release_slot(old)
	_w1 = clampf(blend, 0.0, 1.0)
	_target_blend = _w1
	_tracking = true
	_bind_slots()
	ShaderGlobals.set_value(&"cloud_blend", _w1)
	effect.reset_history()
	_push_light()


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
	_push_light()
	if effect.swap_output():  # the pass re-created its result (a new size): shown from the next frame on
		# Unset first: the same resource again would leave the material's uniform set on the old RD texture, which the
		# pass frees a few frames later (CloudEffect.FREE_DELAY_CALLBACKS).
		_material.set_shader_parameter(&"cloud_image", null)
		_material.set_shader_parameter(&"cloud_image", effect.output)
	ShaderGlobals.set_value(&"cloud_blend", _w1)
	var camera_render := Frames.to_render(camera_fixed_m)
	var stretched := Vector3(camera_fixed_m.x, camera_fixed_m.y, camera_fixed_m.z * POLAR_STRETCH)
	var height := stretched.length() - WGS84_A_M
	ShaderGlobals.set_value(&"cloud_camera_height_m", height)
	ShaderGlobals.set_value(&"cloud_noise_origin", _shape_origin(camera_render, height, SHAPE_TILE_H_M, SHAPE_TILE_V_M))
	ShaderGlobals.set_value(&"cloud_ice_origin", _shape_origin(camera_render, height, SHAPE_TILE_H_M, ICE_TILE_V_M))
	ShaderGlobals.set_value(&"cloud_stratiform_origin",
			_shape_origin(camera_render, height, SHAPE_TILE_H_M * STRATIFORM_STRETCH, SHAPE_TILE_V_M))
	ShaderGlobals.set_value(&"cloud_detail_origin", _wrap(camera_render / DETAIL_TILE_M))
	var visible_weight := (1.0 - _w1) * float(_slots[0]["id"] != "") + _w1 * float(_slots[1]["id"] != "")
	var near := camera_fixed_m.length() < RAYMARCH_MAX_DISTANCE_M
	var enabled := noise_ready and visible_weight > 0.0 and near
	ShaderGlobals.set_value(&"cloud_enabled", enabled)
	# The far field needs only the field and column textures (no noise, no shadow maps).
	far_enabled = visible_weight > 0.0 and not near
	_update_shadow_frame(camera_render, height)
	_update_light_cache(camera_render, height)
	# The maps count as valid only while the effect keeps running (its render callback stamps ran_msec; the render
	# thread may trail the main thread by a frame or two, hence the window rather than a per-frame flag): a stopped
	# effect (compositor detached, device lost) turns the shadows back to the column the next moments.
	var shadow_ran := Time.get_ticks_msec() - shadow_effect.ran_msec <= SHADOW_RUN_WINDOW_MSEC
	ShaderGlobals.set_value(&"cloud_shadow_valid", enabled and shadow_effect.bind_textures() and shadow_ran)


## The ids of the resident slots and the weight of slot 1 (tests).
func state() -> Dictionary:
	return {"slots": [_slots[0]["id"], _slots[1]["id"]], "w1": _w1, "tracking": _tracking,
			"uploading": not _upload.is_empty()}


## Drops the big texel arrays of a loaded snapshot once its volumes are uploaded (they were the only copy besides the
## GPU's; the dictionary is shared with the service, which reloads from the core's cache if it needs them again).
static func _strip_texels(snapshot: Dictionary) -> void:
	for key: String in ["cloud_texels", "cloud_ice_texels", "cloud_column_texels"]:
		snapshot.erase(key)


## Pushes the visible state to the core's CPU light (class comment). Retried every frame while the core cannot take it.
func _push_light() -> void:
	if _core == null:
		return
	var files := ["", ""]
	for i in range(2):
		files[i] = String(_slots[i].get("file", ""))
	var pushed_files: Array = _pushed_light["files"]
	var settled := files == pushed_files and bool(_pushed_light["ok"])
	if settled and absf(_w1 - float(_pushed_light["w1"])) < LIGHT_PUSH_STEP 			and not (_w1 != float(_pushed_light["w1"]) and (_w1 <= 0.0 or _w1 >= 1.0)):
		return
	var result: Dictionary = _core.weather_set_active(files[0], files[1], _w1)
	var ok := bool(result.get("ok", false))
	_pushed_light = {"files": files, "w1": _w1, "ok": ok}
	if not ok:
		for file: String in files:
			if file != "" and not (file in _core.weather_cached_files()):
				core_cache_missed.emit(file)


## The shadow maps' frame (cloud_light.gdshaderinc): axes perpendicular to the Sun, anchors on the ground under the
## camera, kept while the camera stays within the re-anchoring distance (the maps stay Earth-fixed: no swimming).
func _update_shadow_frame(camera_render: Vector3, height: float) -> void:
	var value: Variant = ShaderGlobals.value(&"atmosphere_sun_direction")
	if not value is Vector3 or (value as Vector3).length_squared() < 0.5:
		return
	var sun := (value as Vector3).normalized()
	var up := Vector3(camera_render.x, camera_render.y * POLAR_STRETCH * POLAR_STRETCH, camera_render.z).normalized()
	var ground := camera_render - up * height
	if is_nan(_near_anchor.x) or _horizontal(ground - _near_anchor, up) > SHADOW_NEAR_REANCHOR_M:
		_near_anchor = ground
	if is_nan(_far_anchor.x) or _horizontal(ground - _far_anchor, up) > SHADOW_FAR_REANCHOR_M:
		_far_anchor = ground
	var e1 := sun.cross(Vector3.UP if absf(sun.y) < 0.9 else Vector3.RIGHT).normalized()
	ShaderGlobals.set_value(&"cloud_shadow_sun", sun)
	ShaderGlobals.set_value(&"cloud_shadow_e1", e1)
	ShaderGlobals.set_value(&"cloud_shadow_e2", sun.cross(e1).normalized())
	ShaderGlobals.set_value(&"cloud_shadow_near_anchor", _near_anchor - camera_render)
	ShaderGlobals.set_value(&"cloud_shadow_far_anchor", _far_anchor - camera_render)


## Plans the light cache's frame (CloudLightCacheSchedule): the ground point under the camera, the Sun, the blend; a
## reset when what the fill depends on changed - the quality, the detail levels, the debug output, the noise, or the
## content of a slot that has weight (a slot is only refilled without weight; its weight then comes through the blend
## steps). Runs with the fidelity flag off too (the flag gates only the lookup: switched back on, the cache is current).
func _update_light_cache(camera_render: Vector3, height: float) -> void:
	var value: Variant = ShaderGlobals.value(&"atmosphere_sun_direction")
	if not value is Vector3 or (value as Vector3).length_squared() < 0.5:
		return
	var sun := (value as Vector3).normalized()
	var up := Vector3(camera_render.x, camera_render.y * POLAR_STRETCH * POLAR_STRETCH, camera_render.z).normalized()
	var ground := camera_render - up * height
	var key := [noise_ready, quality, ShaderGlobals.value(&"cloud_detail_levels"),
			ShaderGlobals.value(&"cloud_debug_output")]
	var reset := key != _cache_key
	for i in range(2):
		var weight := _w1 if i == 1 else 1.0 - _w1
		if _slots[i]["id"] != _cache_slot_ids[i] and weight > 0.0:
			reset = true
		_cache_slot_ids[i] = _slots[i]["id"]
	if reset:
		_cache_key = key
		light_cache.reset()
	var now := Time.get_ticks_usec()
	var delta_s := 0.0 if _cache_usec < 0 else (now - _cache_usec) * 1.0e-6
	_cache_usec = now
	light_cache.plan(ground, sun, _w1, _clock_jumped, delta_s)
	_clock_jumped = false


## The clock jumped (SimClock.time_jumped): the light cache re-anchors and refills (the Sun and the field moved).
func notify_clock_jump() -> void:
	_clock_jumped = true


static func _horizontal(v: Vector3, up: Vector3) -> float:
	return (v - up * v.dot(up)).length()


## Shape coordinates of the camera for the tiles tile_h_m across and tile_v_m up (clouds.gdshaderinc
## cloud_shape_coordinates_for).
static func _shape_origin(camera_render: Vector3, height: float, tile_h_m: float, tile_v_m: float) -> Vector3:
	var k := 1.0 / tile_v_m - 1.0 / tile_h_m
	return _wrap(camera_render / tile_h_m + camera_render.normalized() * (height * k))


# ---------------------------------------------------------------------------------------------------------------------
# Cross-fade state machine (class comment).

func _advance(delta_s: float) -> void:
	var step := delta_s / FADE_S
	var goal := _goal_weight()
	if not is_nan(goal):
		# The slots show the target: an upload still running is not needed (and must never replace a slot that now
		# carries weight).
		_cancel_upload()
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
	return {"id": "", "file": "", "size": Vector3(2, 2, LAYERS), "field": null, "ice": null, "column": null}


## A slot for the snapshot with its volumes created (not yet uploaded); the empty slot for none.
func _new_slot(snapshot: Dictionary) -> Dictionary:
	var id := _snapshot_id(snapshot)
	if id == "":
		return _empty_slot()
	if not snapshot.has("cloud_texels"):  # stripped after an earlier upload (the service reloads before announcing again)
		push_warning("Wolken: Texel-Daten von %s fehlen" % id)
		return _empty_slot()
	var columns := int(snapshot["columns"])
	var rows := int(snapshot["rows"])
	var ice: PackedByteArray = snapshot.get("cloud_ice_texels", PackedByteArray())
	if ice.size() != columns * rows * LAYERS:
		ice = PackedByteArray()
		ice.resize(columns * rows * LAYERS)  # no ice share given: liquid shapes
	var column: PackedByteArray = snapshot.get("cloud_column_texels", PackedByteArray())
	if column.size() != columns * rows * LAYERS * 2:
		column = PackedByteArray()
		column.resize(columns * rows * LAYERS * 2)  # none given: no light taken under the clouds
	return {"id": id, "file": String(snapshot.get("file", "")), "size": Vector3(columns, rows, LAYERS),
		"field": CloudFieldTexture.create(snapshot["cloud_texels"], columns, rows, 4),
		"ice": CloudFieldTexture.create(ice, columns, rows, 1),
		"column": CloudFieldTexture.create(column, columns, rows, 2)}


## Frees a slot's volumes unless a resident slot still shows them (show_now shares one upload for a snapshot twice).
func _release_slot(slot: Dictionary) -> void:
	var field: Variant = slot.get("field")
	if field != null and field != _slots[0]["field"] and field != _slots[1]["field"]:
		slot["field"].release()
		slot["ice"].release()
		slot["column"].release()


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
	var column: CloudFieldTexture = data["column"]
	if not field.done:
		field.step()
	elif not ice.done:
		ice.step()
	elif not column.done:
		column.step()
	if not (field.done and ice.done and column.done):
		return
	last_upload_ms = maxf(field.max_step_ms, maxf(ice.max_step_ms, column.max_step_ms))
	var slot: int = _upload["slot"]
	if (_w1 if slot == 1 else 1.0 - _w1) > 0.0:
		_cancel_upload()  # the slot gained weight meanwhile: replacing it would pop (the state machine starts anew)
		return
	var old: Dictionary = _slots[slot]
	_slots[slot] = data
	_upload = {}
	_bind_slots()
	_release_slot(old)
	for snapshot: Dictionary in _target:
		if _snapshot_id(snapshot) == data["id"]:
			_strip_texels(snapshot)


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
	var names := [[&"cloud_field_before", &"cloud_ice_before", &"cloud_field_before_size", &"cloud_column_before"],
		[&"cloud_field_after", &"cloud_ice_after", &"cloud_field_after_size", &"cloud_column_after"]]
	field_texture_bytes = 0
	for s in range(2):
		var slot: Dictionary = _slots[s]
		var has := slot["field"] != null
		ShaderGlobals.set_value(names[s][0], slot["field"].texture if has else _empty_field)
		ShaderGlobals.set_value(names[s][1], slot["ice"].texture if has else _empty_ice)
		ShaderGlobals.set_value(names[s][2], slot["size"])
		ShaderGlobals.set_value(names[s][3], slot["column"].texture if has else _empty_column)
		if has and not (s == 1 and slot["field"] == _slots[0]["field"]):
			field_texture_bytes += slot["field"].bytes + slot["ice"].bytes + slot["column"].bytes


func _upload_noise() -> void:
	var noise := _noise_result
	if not noise.has("shape"):
		push_error("Wolken: Rauschtexturen fehlen: " + str(noise.get("error", "")))
		return
	var blue_size := int(noise["blue_noise_size"])
	var states := int(noise["detail_states"])
	_noise_textures = [_noise_texture(noise["shape"], int(noise["shape_size"])),
			_noise_texture(noise["detail"], int(noise["detail_size"])), _table(noise["coverage_threshold"], states),
			_table(noise["edge_mean_factor"], states),
			ImageTexture.create_from_image(Image.create_from_data(blue_size, blue_size, false, Image.FORMAT_R8,
					noise["blue_noise"]))]
	ShaderGlobals.set_value(&"cloud_noise_shape", _noise_textures[0])
	ShaderGlobals.set_value(&"cloud_noise_detail", _noise_textures[1])
	ShaderGlobals.set_value(&"cloud_coverage_threshold", _noise_textures[2])
	ShaderGlobals.set_value(&"cloud_edge_mean", _noise_textures[3])
	ShaderGlobals.set_value(&"cloud_blue_noise", _noise_textures[4])
	# The shape prefilter (Task 10 fix round 1, cloud_noise.hpp): the mip levels 1.. of the shape and their tables.
	var prefilter_rows := int(noise["prefilter_rows"])
	_noise_textures.append_array([_mip_texture(noise["shape_mips"], int(noise["shape_size"]) / 2),
			_table(noise["prefilter_threshold"], prefilter_rows), _table(noise["prefilter_edge_mean"], prefilter_rows)])
	ShaderGlobals.set_value(&"cloud_noise_shape_mips", _noise_textures[5])
	ShaderGlobals.set_value(&"cloud_prefilter_threshold", _noise_textures[6])
	ShaderGlobals.set_value(&"cloud_prefilter_edge_mean", _noise_textures[7])
	ShaderGlobals.set_value(&"cloud_edge_width", Vector2(noise["edge_width_min_value"], noise["edge_width_max_value"]))
	ShaderGlobals.set_value(&"cloud_detail_mean", float(noise["detail_mean"]))
	ShaderGlobals.set_value(&"cloud_detail_weights", noise["detail_weights"])
	var amplitude: PackedFloat32Array = noise["level_amplitude"]
	ShaderGlobals.set_value(&"cloud_level_amplitude", Vector3(amplitude[0], amplitude[1], amplitude[2]))
	_noise_result = {}
	noise_ready = true
	_upload_transfer()


## The transfer table's GPU form (RGBA half floats; StarsimCore.cloud_transfer_table) and its node maps.
func _upload_transfer() -> void:
	var table := _transfer_result
	_transfer_result = {}
	if not table.get("ok", false):
		push_warning("Wolken: Transfertabelle fehlt, geschlossene Form: " + str(table.get("error", "")))
		return
	var size: Vector3i = table["size"]
	var texels: PackedByteArray = table["texels"]
	var slice := size.x * size.y * 8
	var images: Array[Image] = []
	for z in range(size.z):
		images.append(Image.create_from_data(size.x, size.y, false, Image.FORMAT_RGBAH,
				texels.slice(z * slice, (z + 1) * slice)))
	_transfer_texture = ImageTexture3D.new()
	_transfer_texture.create(Image.FORMAT_RGBAH, size.x, size.y, size.z, false, images)
	var nodes: Vector4i = table["nodes"]
	ShaderGlobals.set_value(&"cloud_transfer", _transfer_texture)
	ShaderGlobals.set_value(&"cloud_transfer_nodes", Vector4(nodes.x, nodes.y, nodes.z, nodes.w))
	ShaderGlobals.set_value(&"cloud_transfer_depth_map", table["depth_map"])
	ShaderGlobals.set_value(&"cloud_transfer_theta_map", table["theta_map"])
	# The slab's total transmittance (the Sun's diffuse light under the clouds).
	var lut_size: Vector2i = table["transmittance_lut_size"]
	var lut: PackedFloat32Array = table["transmittance_lut"]
	_slab_transmittance = ImageTexture.create_from_image(Image.create_from_data(lut_size.x, lut_size.y, false,
			Image.FORMAT_RF, lut.to_byte_array()))
	ShaderGlobals.set_value(&"cloud_slab_transmittance", _slab_transmittance)
	ShaderGlobals.set_value(&"cloud_slab_transmittance_map", table["transmittance_lut_map"])
	transfer_ready = true
	_apply_scattering_model()


func _apply_scattering_model() -> void:
	var table := scattering_model == ScatteringModel.TABLE and transfer_ready
	ShaderGlobals.set_value(&"cloud_scattering_model", 1 if table else 0)


## A table of rows of 256 (one per detail state): a 256 x rows R32F texture, filtered linearly in both directions.
static func _table(values: PackedFloat32Array, rows: int) -> ImageTexture:
	return ImageTexture.create_from_image(Image.create_from_data(values.size() / rows, rows, false, Image.FORMAT_RF,
			values.to_byte_array()))


## A 3D RGBA8 texture with mipmaps from its levels (levels[0] of size^3, each next one half as large, down to 1).
static func _mip_texture(levels: Array, size: int) -> ImageTexture3D:
	var images: Array[Image] = []
	var s := size
	for level: PackedByteArray in levels:
		var slice := s * s * 4
		for z in range(s):
			images.append(Image.create_from_data(s, s, false, Image.FORMAT_RGBA8,
					level.slice(z * slice, (z + 1) * slice)))
		s = maxi(s / 2, 1)
	var texture := ImageTexture3D.new()
	texture.create(Image.FORMAT_RGBA8, size, size, size, true, images)
	return texture


static func _noise_texture(texels: PackedByteArray, size: int) -> ImageTexture3D:
	var images: Array[Image] = []
	var slice := size * size * 4
	for z in range(size):
		images.append(Image.create_from_data(size, size, false, Image.FORMAT_RGBA8, texels.slice(z * slice,
				(z + 1) * slice)))
	var texture := ImageTexture3D.new()
	texture.create(Image.FORMAT_RGBA8, size, size, size, false, images)
	return texture




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
	ShaderGlobals.set_value(&"cloud_detail_levels", int(preset["detail_levels"]))
	light_cache.march_first_m = first_m
	light_cache.march_near_m = near_m
	effect.set_quality(int(preset["divisor"]), float(preset["temporal_weight"]), float(preset["clip_gamma"]))


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
