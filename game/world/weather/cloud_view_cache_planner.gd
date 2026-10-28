class_name CloudViewCachePlanner
extends RefCounted
## The cloud view cache's per-frame plan on the main thread (cloud view cache Task 4; CloudLayer owns it, CloudEffect
## runs its batches): the layout from the eyes' pixel density, the anchor, the schedule's pick under a budget of tile
## equivalents (a level's cost relative to a full-level tile: its texels), the tiles' shown state for the lookup and its
## cross-fades, and the restart after a jump.
##
## Shown state (the lookup's per-tile buffer, LEVEL / FROM / SLOT bits + the fade's start): a tile shows the finest
## level it was filled with since the last reset, cross-faded over CloudViewCache CROSSFADE_S from what it showed before
## - nothing (T = 1) for a first fill, the coarser level for a refinement, and for a refresh at the same level (light,
## weather, anchor) a fade slot: the fill copies the tile's old texels there before it overwrites them (one Sun state
## per tile at any time, no tile switches its light abruptly). Slots are a ring of SLOT_COUNT cells; one still fading
## when the ring comes round again ends that tile's fade early (slot_overflows counts it).
## Batches: plan() returns {layout, anchor, tiles: PackedInt32Array of (tile, level, slot), states (bytes or empty:
## unchanged), now_s, epoch}; the effect fills the tiles, then shows the states. Picked tiles are completed in the
## schedule at once (the effect runs every batch it is handed: CloudLayer plans only when the last one was taken).

## [E] Fade slots (one full-level tile of colour each): the same-level refreshes in flight during one cross-fade - at
## 45 Hz a quarter second is ~11 frames of up to a few dozen full tiles; 32 x 32 cells of 64^2 RGBA16F = 32 MB.
const SLOT_COUNT := 1024
const SLOTS_PER_ROW := 32
## [E] The cost of a tile at each level relative to the full level (its texels: 1/4 and 1/2 of the density per axis).
const LEVEL_COST := [1.0 / 16.0, 1.0 / 4.0, 1.0]
## [E] pick() calls per frame at most and the main-thread time after which no further one starts (us): a pick returns at
## most 128 tiles (CloudViewCacheSchedule.MAX_BLOCK_EXPANSIONS) and costs up to ~0.5 ms; with the completions the
## schedule's share stays near its 1 ms per-frame bound (measured: FrameProfiler section cloud_view_cache).
const MAX_PICKS := 4
const PICK_TIME_US := 400
## [E] The largest face (texels per side): the cache's VRAM - 6 x 4096^2 x 12 B x (1 + 1/4 + 1/16) = 1.6 GB of the 2 GB
## cap with the fade slots; finer views (a narrow desktop field of view) get texels a little larger than their pixels.
const FACE_TEXELS_MAX := 4096
## [E] Relative change of the wanted face size that re-creates the layout (and refills the cache): the runtime's
## field-of-view jitter and small zooms keep the cache.
const FACE_HYSTERESIS := 0.1
## [E] Interim (until the progressive re-anchor, cloud view cache Task 5): the cache restarts from the camera once it
## moved this far from the anchor - the parallax lookup holds clouds beyond ~2 km within 1e-8 eye px at 50 m, so 500 m
## keeps far clouds exact and near ones within a few pixels.
const RESTART_DISTANCE_M := 500.0

var layout: CloudViewCacheLayout
var schedule: CloudViewCacheSchedule
var anchor_earth_m := Vector3(NAN, NAN, NAN)
var now_s := 0.0  ## the planner's clock (the frames' delta, s): the fades' time base, handed to the lookup
var resets := 0  ## restarts so far (tests, diagnostics)
var epoch := 0  ## counts layouts and restarts: a read-back distance of an older epoch is dropped
var slot_overflows := 0  ## refreshes whose fade slot was still in use (that tile switched without a fade)
var last_tiles := 0  ## tiles of the last plan
var last_equivalents := 0.0  ## their cost in full-level tiles
var last_fades: Array[int] = []  ## tiles whose fade started in the last plan (tests)
var last_fade_s := -INF  ## when the latest fade started (now_s; its end: + CloudViewCache.CROSSFADE_S)

var _reset_pending := false
var _shown := PackedInt32Array()  ## per tile: the level shown (-1: none)
var _states := PackedByteArray()  ## per tile 8 bytes: the lookup's word and the fade's start (float)
var _states_dirty := true
var _slot_end_s := PackedFloat64Array()  ## per slot: when the fade reading it ends
var _slot_owner := PackedInt32Array()  ## per slot: the tile fading from it
var _slot_cursor := 0
var _full_count := 0  ## tiles showing the full level
var _building := -1  ## the worker task building a layout (-1: none)
var _built := {}  ## its result: {layout, schedule}


## Main thread: restart the cache at the next plan (a view or clock jump, a pinned snapshot).
func request_reset() -> void:
	_reset_pending = true


## The plan of one frame. view: {dirs: Array[Vector3] (Earth-local unit sample rays of the eyes), sample_rad,
## face_texels (the eyes' density)}; light: {sun_dir, weather_weight, light_generation}; budget: tile equivalents.
## {} without a view.
func plan(camera_earth_m: Vector3, view: Dictionary, light: Dictionary, budget: float, delta_s: float) -> Dictionary:
	now_s += maxf(delta_s, 0.0)
	last_tiles = 0
	last_equivalents = 0.0
	last_fades.clear()
	if view.is_empty():
		return {}
	var wanted := mini(int(view["face_texels"]), FACE_TEXELS_MAX)
	if _building < 0 and (layout == null or absf(wanted - layout.face_texels) > FACE_HYSTERESIS * layout.face_texels):
		# A new layout and schedule are built on a worker (~10 ms of GDScript at the VR face: no frame hitch); until
		# then no batch - the effect keeps showing what it has (the march before the first batch).
		_building = WorkerThreadPool.add_task(_build.bind(wanted), false, "cloud view cache layout")
	if _building >= 0:
		if not WorkerThreadPool.is_task_completed(_building):
			return {}
		WorkerThreadPool.wait_for_task_completion(_building)
		_building = -1
		_adopt_layout()
		_restart(camera_earth_m)
	elif _reset_pending or is_nan(anchor_earth_m.x) \
			or camera_earth_m.distance_to(anchor_earth_m) > RESTART_DISTANCE_M:
		_restart(camera_earth_m)
	var light_sun: Vector3 = light["sun_dir"]
	schedule.mark_light(light_sun, light["weather_weight"], light["light_generation"])
	var state := {"anchor_earth_m": anchor_earth_m, "sun_dir": light_sun.normalized(),
		"weather_weight": light["weather_weight"], "light_generation": light["light_generation"]}
	var dirs: Array[Vector3] = view["dirs"]
	var tiles := PackedInt32Array()  # (tile, level, slot) per entry
	var remaining := maxf(budget, 0.0)
	var started := Time.get_ticks_usec()
	for call in range(MAX_PICKS):
		if remaining <= 0.0 or (call > 0 and Time.get_ticks_usec() - started > PICK_TIME_US):
			break
		# As many tiles as the budget holds at the cheapest level (pick() caps itself); the costlier ones cut below.
		var picked := schedule.pick(camera_earth_m, anchor_earth_m, dirs,
				maxi(ceili(remaining / LEVEL_COST[0]), 1), float(view["sample_rad"]))
		if picked.is_empty():
			break
		var spent := false
		var picked_tiles := PackedInt32Array()
		var picked_levels := PackedInt32Array()
		for p in picked:
			var level: int = p["level"]
			var cost: float = LEVEL_COST[level]
			if cost > remaining and not (tiles.is_empty() and picked_tiles.is_empty()):  # at least one tile a frame
				spent = true
				break
			remaining -= cost
			last_equivalents += cost
			picked_tiles.append(p["tile"])
			picked_levels.append(level)
		schedule.completed_batch(picked_tiles, picked_levels, state)
		for i in range(picked_tiles.size()):
			tiles.append_array([picked_tiles[i], picked_levels[i], _shown_now(picked_tiles[i], picked_levels[i])])
		if spent:
			break
	last_tiles = tiles.size() / 3
	var batch := {"layout": layout, "anchor": anchor_earth_m, "tiles": tiles, "now_s": now_s, "epoch": epoch,
		"states": _states.duplicate() if _states_dirty else PackedByteArray()}
	_states_dirty = false
	return batch


## Read-back nearest cloud distances of filled tiles (CloudEffect, an earlier epoch's are dropped): the float bits of
## each entry's distance (+inf: no cloud).
func apply_distances(batch_epoch: int, tiles: PackedInt32Array, bits: PackedByteArray) -> void:
	if batch_epoch != epoch or schedule == null:
		return
	for i in range(mini(tiles.size(), bits.size() / 4)):
		var near := bits.decode_float(4 * i)
		schedule.set_tile_distance(tiles[i], near if near > 0.0 else INF)


## The level a tile shows (-1: none).
## The fraction of the cube's tiles showing the full level (0 without a layout).
func full_fraction() -> float:
	return float(_full_count) / layout.tile_count() if layout != null else 0.0


func shown_level(tile: int) -> int:
	return _shown[tile]


## The cross-fade weight of a tile's shown content at time t (1: complete).
func fade_weight(tile: int, t: float) -> float:
	var start := _states.decode_float(8 * tile + 4)
	return clampf((t - start) / CloudViewCache.CROSSFADE_S, 0.0, 1.0)


## What a tile fades from (0 nothing, 1-3 that level + 1, 4 its fade slot).
func fade_source(tile: int) -> int:
	return (_states.decode_u32(8 * tile) >> 2) & 7


## Worker: the layout of a face size (rounded up to whole tiles) and its schedule.
func _build(face_texels: int) -> void:
	var tile_texels := CloudViewCache.TILE_TEXELS
	var built_layout := CloudViewCacheLayout.new(maxi(ceili(float(face_texels) / tile_texels), 1) * tile_texels,
			tile_texels)
	_built = {"layout": built_layout, "schedule": CloudViewCacheSchedule.new(built_layout)}


## Main thread: the built layout and schedule taken over (the shown states sized for them).
func _adopt_layout() -> void:
	layout = _built["layout"]
	schedule = _built["schedule"]
	_built = {}
	var n := layout.tile_count()
	_shown.resize(n)
	_states.resize(8 * n)


## Everything restarts from nothing at the camera: no tile shows anything (no old content after a jump).
func _restart(camera_earth_m: Vector3) -> void:
	_reset_pending = false
	anchor_earth_m = camera_earth_m
	resets += 1
	epoch += 1
	schedule.invalidate_all()
	_shown.fill(-1)
	_states.fill(0)
	_slot_end_s.resize(SLOT_COUNT)
	_slot_end_s.fill(-INF)
	_slot_owner.resize(SLOT_COUNT)
	_slot_owner.fill(-1)
	_full_count = 0
	_states_dirty = true


## A tile filled at `level` this frame: its shown state and fade; the fade slot the fill copies the old texels to (-1:
## none).
func _shown_now(tile: int, level: int) -> int:
	var old := _shown[tile]
	var from := 0  # nothing: a first fill fades in from T = 1
	var slot := -1
	if old >= 0 and level > old:
		from = old + 1
	elif old >= 0:
		# A refresh: the shown texels go to a slot first (a slot still fading ends that fade: the ring is too short).
		slot = _slot_cursor
		_slot_cursor = (_slot_cursor + 1) % SLOT_COUNT
		var owner := _slot_owner[slot]
		if _slot_end_s[slot] > now_s and owner >= 0 and owner != tile and fade_source(owner) == 4 				and _states.decode_u32(8 * owner) >> 8 == slot:
			slot_overflows += 1
			_states.encode_float(8 * owner + 4, now_s - CloudViewCache.CROSSFADE_S)  # its fade ends now
		_slot_end_s[slot] = now_s + CloudViewCache.CROSSFADE_S
		_slot_owner[slot] = tile
		from = 4
	if level == CloudViewCacheSchedule.MAX_LEVEL and old != level:
		_full_count += 1
	elif old == CloudViewCacheSchedule.MAX_LEVEL and level != old:
		_full_count -= 1
	_shown[tile] = level
	_states.encode_u32(8 * tile, (level + 1) | (from << 2) | (maxi(slot, 0) << 8))
	_states.encode_float(8 * tile + 4, now_s)
	_states_dirty = true
	last_fades.append(tile)
	last_fade_s = now_s
	return slot
