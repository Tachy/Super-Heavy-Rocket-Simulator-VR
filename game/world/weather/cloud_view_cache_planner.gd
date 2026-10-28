class_name CloudViewCachePlanner
extends RefCounted
## The cloud view cache's per-frame plan (cloud view cache Task 4; CloudLayer owns it, CloudEffect runs its batches),
## computed on a worker (fix round 1: the pick, completions and shown states cost 0.5 - 1.4 ms of GDScript a frame,
## over the 1.0 ms main-thread bound; the main thread only hands over the frame's inputs and takes the last run's
## batch - one frame later): the layout from the eyes' pixel density, the anchor, the schedule's pick under a GPU budget
## (ms, each tile's cost predicted from its own measured fills), the tiles' shown state for the lookup and its
## cross-fades, the restart after a jump, and whether the per-frame march still covers the view (covering).
##
## Shown state (the lookup's per-tile buffer, LEVEL / FROM / SLOT bits + the fade's start): a tile shows the finest
## level it was filled with since the last reset, cross-faded over CloudViewCache CROSSFADE_S from what it showed before
## - nothing (T = 1, or the covering march's result) for a first fill, the coarser level for a refinement, and for a
## refresh at the same level (light, weather, anchor) a fade slot: the fill copies the tile's old texels there before it
## overwrites them (one Sun state per tile at any time, no tile switches its light abruptly). Slots are a ring of
## SLOT_COUNT cells; one still fading when the ring comes round again ends that tile's fade early (slot_overflows counts
## it).
## Batches: plan() returns {layout, anchor, tiles: PackedInt32Array of (tile, level, slot), states (bytes or empty:
## unchanged), now_s, epoch, fill_id, covering, anchors (the ring's positions), anchor_index (the current one),
## anchor_tiles (per ring entry the tiles showing it)}; the effect fills the tiles (GPU timestamp named by fill_id),
## then shows the states. Picked tiles are completed in the schedule at once (the effect runs every batch it is handed:
## CloudLayer plans only when the last one was taken).
## Covering (controller ruling, Task 4 fix round 1): while a tile in the view's cone has no level yet, or a first fill
## there is still fading in, the effect runs the per-frame march at the preset divisor too and the lookup shows it where
## a tile has nothing (after a start, a jump, a layout re-creation, the interim restart): no frame without clouds.
## Cost model (fix round 1): a fill's GPU time is set first by its slowest ray, then by its work - a few tiles of a deck
## seen from above take as long as a thousand work groups of clear sky (measured, desktop deck view: deck ~8 ms + 6 us
## per work group, sky ~0.6 ms + 0.3 us; both ~ latency x (1 + groups / ~1300)). So each tile has a latency l (ms) per
## level and a fill of tiles i with g_i work groups is predicted as max l_i + sum g_i l_i / FILL_CAPACITY_GROUPS. A
## measured fill scales its tiles' latencies by measured / predicted (relative differences kept: the picks are spatially
## coherent). A level not yet measured takes the tile's coarser one times that level's ratio (the finer texels march
## more detail: learned per level from the tiles measured at both, the median of the recent fills' medians); a tile
## never measured since the restart takes the largest of the recent fills' slowest latency at that level (its
## neighbours, nearest the view first; UNKNOWN_LATENCY_MS before any). Tiles join a fill in priority order while the
## prediction fits the budget; the first one always, and when it alone exceeds the budget the tiles that lengthen its
## fill by at most FREE_RIDE (a slow tile's fill holds more of its kind almost for free).
## Progressive re-anchor (cloud view cache Task 5): once the camera is CloudViewCacheSchedule.REANCHOR_FRACTION of the
## near shell's radius from the anchor, the camera becomes the anchor - the next entry of a ring of ANCHOR_RING anchors.
## No tile is dropped: each keeps showing what it holds, from the anchor it was rendered from (its state names the ring
## entry, the lookup reprojects it from there), until the schedule refills it from the new anchor (its parallax and
## staleness classes: nearest and most displaced first, cross-faded from its fade slot). A tile is dropped by its own
## parallax (fix round 1): once the camera is farther from its anchor than CloudViewCacheSchedule.keep_distance_m of its
## nearest cached cloud (the march covers it if it is in view), as are the tiles of a ring entry about to be reused.

## [E] Fade slots (one full-level tile of colour each): the same-level refreshes in flight during one cross-fade - at
## 45 Hz a quarter second is ~11 frames of up to a few dozen full tiles; 32 x 32 cells of 64^2 RGBA16F = 32 MB.
const SLOT_COUNT := 1024
const SLOTS_PER_ROW := 32
## [E] pick() calls per plan at most and the blocks each expands (CloudViewCacheSchedule: 64 tiles, ~0.2 - 0.7 ms of
## GDScript); a further pick starts only within RUN_PICK_US of the first (the run must end well before the next frame:
## 11 ms at 90 Hz).
const MAX_PICKS := 8
const PICK_EXPANSIONS := 1
const RUN_PICK_US := 3000
## [E] The work groups (8 x 8 texels) of one latency in a fill's work term: RTX 4090, the fill kernel - deck and sky
## fills both took about latency x (1 + groups / 1300) (measured, fix round 1); 1024 keeps the prediction on the safe
## side.
const FILL_CAPACITY_GROUPS := 1024.0
## [E] The latency of a tile without a measurement before any fill was measured (ms): the slowest small fills measured
## (the desktop deck view: 8.8 ms).
const UNKNOWN_LATENCY_MS := 9.0
## [E] Recent fills whose latency predicts a tile never measured (the picks go nearest the view first: these are its
## neighbours), and fills waiting for their GPU time at most (the timestamps come back a few frames later; a fill whose
## time never comes is dropped).
const LATENCY_HISTORY := 8
const PENDING_MAX := 16
## [E] A level's latency over its coarser one's before any tile was measured at both.
const LEVEL_RATIO_PRIOR := 1.0
## [E] A first tile that alone exceeds the budget takes other tiles along while they lengthen its fill by at most this
## factor (~100 work groups: tiles of its kind fill at once instead of one per frame).
const FREE_RIDE := 1.1
## [E] Work groups of one fill at most: one FILL_CAPACITY_GROUPS - the prediction of larger fills missed by up to 3 x
## (desktop deck view: 1 in 4 fills of 1024 - 4096 groups over budget, those below 1024 within 1.4 x at p90; a cap of
## 2048 left the VR proxy's deck view fills up to 19 ms at its 11.4 ms budget, 1024 at most 9.4 ms).
const MAX_FILL_GROUPS := 1024
## [E] Measured fills kept for take_fill_log (a few minutes of play at 45 fills / s would be ~10^4; the checks take them
## every frame, so this only bounds a session nobody reads).
const FILL_LOG_MAX := 4096
## [E] The largest face (texels per side): the cache's VRAM - 6 x 4096^2 x 12 B x (1 + 1/4 + 1/16) = 1.6 GB of the 2 GB
## cap with the fade slots; finer views (a narrow desktop field of view) get texels a little larger than their pixels.
const FACE_TEXELS_MAX := 4096
## [E] Relative change of the wanted face size that re-creates the layout (and refills the cache): the runtime's
## field-of-view jitter and small zooms keep the cache.
const FACE_HYSTERESIS := 0.1
## [E] Anchors in the ring (a tile state's 7 bits): 128 re-anchors of 100 m - 12.8 km of flight - before an entry is
## reused; the eviction drops a tile long before that in a straight flight (CloudViewCacheSchedule.keep_distance_m: at
## most 1 km).
const ANCHOR_RING := 128

var layout: CloudViewCacheLayout
var schedule: CloudViewCacheSchedule
var anchor_earth_m := Vector3(NAN, NAN, NAN)
var now_s := 0.0  ## the planner's clock (the frames' delta, s): the fades' time base, handed to the lookup
var resets := 0  ## restarts so far (tests, diagnostics)
var epoch := 0  ## counts layouts and restarts: a read-back distance of an older epoch is dropped
var slot_overflows := 0  ## refreshes whose fade slot was still in use (that tile switched without a fade)
var last_tiles := 0  ## tiles of the last plan
var last_predicted_ms := 0.0  ## their predicted GPU time
var last_covering := false  ## the last plan's covering flag
var view_missing := true  ## the last plan found a tile in the view's cone without a level (the march covers it)
var last_fades: Array[int] = []  ## tiles whose fade started in the last plan (tests)
var last_fade_s := -INF  ## when the latest fade started (now_s; its end: + CloudViewCache.CROSSFADE_S)
var anchor_index := 0  ## the current anchor's ring entry
var reanchors := 0  ## re-anchors so far (tests, diagnostics)
var evicted_tiles := 0  ## tiles dropped so far because their anchor was left too far behind or reused
## Measured fills (diagnostics, render_check; take_fill_log): (measured ms, predicted ms, budget ms) each; a ring of
## the last FILL_LOG_MAX [E] (a run without a check taking them keeps the newest).
var fill_log: Array[Vector3] = []

var _reset_pending := false
var _shown := PackedInt32Array()  ## per tile: the level shown (-1: none)
var _states := PackedByteArray()  ## per tile 8 bytes: the lookup's word and the fade's start (float)
var _states_dirty := true
var _slot_end_s := PackedFloat64Array()  ## per slot: when the fade reading it ends
var _slot_owner := PackedInt32Array()  ## per slot: the tile fading from it
var _slot_cursor := 0
var _full_count := 0  ## tiles showing the full level
var _built := {}  ## a built layout: {layout, schedule, dirs}
var _tile_dirs := PackedVector3Array()  ## the layout's tile centre directions
var _anchor_ring := PackedVector3Array()  ## the ring's anchors (Earth-local float64)
var _anchor_tiles := PackedInt32Array()  ## per ring entry: the tiles showing it
var _tile_anchor := PackedInt32Array()  ## per tile: the ring entry it shows (-1: none)
## Per ring entry: the tiles that took it (an entry whose tile has moved on to another anchor is skipped and dropped at
## the entry's next scan) and the nearest cached cloud among them (m; INF: none known) - _evict_far scans an entry's
## tiles only when the camera may have left the keep distance of its nearest one.
var _anchor_list: Array[PackedInt32Array] = []
var _anchor_min_near_m := PackedFloat64Array()
## Per tile and level (tile x LEVELS + level): its latency (ms; 0: not measured since the restart).
var _tile_latency_ms := PackedFloat32Array()
## Per level: the recent fills' median ratio of a tile's latency to its coarser level's, and their slowest latency.
var _level_ratios: Array = [[], [], []]
var _level_groups := PackedInt32Array()  ## per level: a tile's work groups in the fill
var _fill_id := 0
var _pending := {}  ## fill id -> {epoch, tiles, levels, estimates, latency_ms, predicted_ms, budget_ms}
var _recent_latency_ms: Array = [[], [], []]
var _view_centre := Vector3.ZERO  ## the view's cone this frame (the covering test, first fills inside it)
var _view_cos := 2.0
var _cover_until_s := -INF  ## a first fill in the view is fading in until then
## The worker hand-over (main thread): the running plan's task (-1: none), its batch (written by the worker), the last
## finished run's batch not yet handed out, the frame time and the feedback gathered for the next run.
var _task := -1
var _result := {}
var _ready := {}
var _held_s := 0.0
var _inbox_distances: Array[Dictionary] = []
var _inbox_fills: Array[Vector2] = []


## Main thread: restart the cache at the next plan (a view or clock jump, a pinned snapshot). Until the restarted run's
## batch, plan() hands out {reset: true, covering: true}: the effect shows the per-frame march (no old tile).
func request_reset() -> void:
	_reset_pending = true


## Main thread, once a frame: the last finished run's batch ({} while it runs or without one; a reset batch while a
## restart is pending), and the next run started with this frame's inputs. view: {dirs: Array[Vector3] (Earth-local unit
## sample rays of the eyes), sample_rad, face_texels (the eyes' density)}; light: {sun_dir, weather_weight,
## light_generation}; budget_ms: the fill's GPU time.
func plan(camera_earth_m: Vector3, view: Dictionary, light: Dictionary, budget_ms: float, delta_s: float) -> Dictionary:
	_held_s += maxf(delta_s, 0.0)
	if _task >= 0 and WorkerThreadPool.is_task_completed(_task):
		wait()
	if _task >= 0:
		return {"reset": true, "covering": true} if _reset_pending else {}
	var batch := _ready
	_ready = {}
	if _reset_pending:
		batch = {"reset": true, "covering": true}  # the run's batch was of the old place / instant
	if view.is_empty():
		return batch
	var inputs := {"camera": camera_earth_m, "view": view, "light": light, "budget_ms": budget_ms, "delta_s": _held_s,
		"reset": _reset_pending, "distances": _inbox_distances, "fills": _inbox_fills}
	_reset_pending = false
	_held_s = 0.0
	_inbox_distances = []
	_inbox_fills = []
	_task = WorkerThreadPool.add_task(_run.bind(inputs), false, "cloud view cache plan")
	return batch


## Main thread: waits for the running plan (tests and checks read the planner's state after it).
func wait() -> void:
	if _task < 0:
		return
	WorkerThreadPool.wait_for_task_completion(_task)
	_task = -1
	_ready = _result
	_result = {}


## Worker: one run - the feedback applied, then the plan of the frame.
func _run(inputs: Dictionary) -> void:
	for r: Dictionary in inputs["distances"]:
		_apply_distances(r["epoch"], r["tiles"], r["bits"])
	for fill: Vector2 in inputs["fills"]:
		_apply_fill_time(int(fill.x), fill.y)
	_result = _plan_now(inputs["camera"], inputs["view"], inputs["light"], inputs["budget_ms"], inputs["delta_s"],
			inputs["reset"])


## Worker: the plan of one frame (plan()'s inputs; reset: a restart was requested).
func _plan_now(camera_earth_m: Vector3, view: Dictionary, light: Dictionary, budget_ms: float, delta_s: float,
		reset: bool) -> Dictionary:
	now_s += maxf(delta_s, 0.0)
	last_tiles = 0
	last_predicted_ms = 0.0
	last_fades.clear()
	var wanted := mini(int(view["face_texels"]), FACE_TEXELS_MAX)
	if layout == null or absf(wanted - layout.face_texels) > FACE_HYSTERESIS * layout.face_texels:
		# A new layout and schedule (~10 ms of GDScript at the VR face, here on the worker: the main thread hands out
		# nothing meanwhile and the effect keeps showing what it has - the march before the first batch).
		_build(wanted)
		_adopt_layout()
		_restart(camera_earth_m)
	elif reset or is_nan(anchor_earth_m.x):
		_restart(camera_earth_m)
	elif schedule.reanchor_needed(camera_earth_m):
		_reanchor(camera_earth_m)
	_evict_far(camera_earth_m)
	var light_sun: Vector3 = light["sun_dir"]
	schedule.mark_light(light_sun, light["weather_weight"], light["light_generation"])
	var state := {"anchor_earth_m": anchor_earth_m, "sun_dir": light_sun.normalized(),
		"weather_weight": light["weather_weight"], "light_generation": light["light_generation"]}
	var dirs: Array[Vector3] = view["dirs"]
	var sample_rad := float(view["sample_rad"])
	_view_cone(dirs, sample_rad)
	var tiles := PackedInt32Array()  # (tile, level, slot) per entry
	var ratio := PackedFloat64Array()
	var unknown_ms := PackedFloat64Array()
	for level in range(CloudViewCacheLayout.LEVELS):
		ratio.append(_level_ratio(level))
		unknown_ms.append(_unknown_latency_ms(level, unknown_ms[level - 1] * ratio[level] if level > 0 else 0.0))
	var latency := 0.0  # the fill's slowest tile so far (ms)
	var work := 0.0  # its work term: sum of groups x latency / FILL_CAPACITY_GROUPS (ms)
	var groups := 0  # its work groups
	var estimates := PackedFloat32Array()  # per entry: the tile's predicted latency
	var limit := maxf(budget_ms, 0.0)  # raised to the first tile's prediction when that alone exceeds the budget
	var full := false
	# pick() returns what its block expansions hold (the budget cut below, in priority order).
	var pick_tiles := CloudViewCacheSchedule.BLOCK_TILES * CloudViewCacheSchedule.BLOCK_TILES * PICK_EXPANSIONS
	var started := Time.get_ticks_usec()
	for call in range(MAX_PICKS):
		if full or (call > 0 and Time.get_ticks_usec() - started > RUN_PICK_US):
			break
		var picked := schedule.pick(camera_earth_m, anchor_earth_m, dirs, pick_tiles, sample_rad, PICK_EXPANSIONS)
		if picked.is_empty():
			break
		var picked_tiles := PackedInt32Array()
		var picked_levels := PackedInt32Array()
		for p in picked:
			var tile: int = p["tile"]
			var level: int = p["level"]
			var tile_latency := _latency_of(tile, level, ratio, unknown_ms)
			var with_latency := maxf(latency, tile_latency)
			var with_work := work + _level_groups[level] * tile_latency / FILL_CAPACITY_GROUPS
			var predicted := with_latency + with_work
			if tiles.is_empty() and picked_tiles.is_empty():
				limit = maxf(limit, FREE_RIDE * predicted)  # at least one tile a frame (and its free riders)
			elif predicted > limit or groups + _level_groups[level] > MAX_FILL_GROUPS:
				full = true
				break
			latency = with_latency
			work = with_work
			groups += _level_groups[level]
			last_predicted_ms = predicted
			picked_tiles.append(tile)
			picked_levels.append(level)
			estimates.append(tile_latency)
		schedule.completed_batch(picked_tiles, picked_levels, state)
		for i in range(picked_tiles.size()):
			tiles.append_array([picked_tiles[i], picked_levels[i], _shown_now(picked_tiles[i], picked_levels[i])])
	last_tiles = tiles.size() / 3
	view_missing = schedule.missing_in_cone(_view_centre, acos(clampf(_view_cos, -1.0, 1.0)))
	last_covering = view_missing or now_s < _cover_until_s
	_fill_id += 1
	if last_tiles > 0:
		var levels := PackedInt32Array()
		var filled := PackedInt32Array()
		for i in range(0, tiles.size(), 3):
			filled.append(tiles[i])
			levels.append(tiles[i + 1])
		_pending[_fill_id] = {"epoch": epoch, "tiles": filled, "levels": levels, "estimates": estimates,
			"latency_ms": latency, "predicted_ms": last_predicted_ms, "budget_ms": budget_ms}
		if _pending.size() > PENDING_MAX:
			_pending.erase(_pending.keys().min())
	var batch := {"layout": layout, "anchor": anchor_earth_m, "tiles": tiles, "now_s": now_s, "epoch": epoch,
		"fill_id": _fill_id, "covering": last_covering, "anchors": _anchor_ring.duplicate(),
		"anchor_index": anchor_index, "anchor_tiles": _anchor_tiles.duplicate(),
		"states": _states.duplicate() if _states_dirty else PackedByteArray()}
	_states_dirty = false
	return batch


## Main thread: read-back nearest cloud distances of filled tiles (CloudEffect; applied by the next run, an earlier
## epoch's dropped): the float bits of each entry's distance (+inf: no cloud).
func apply_distances(batch_epoch: int, tiles: PackedInt32Array, bits: PackedByteArray) -> void:
	_inbox_distances.append({"epoch": batch_epoch, "tiles": tiles, "bits": bits})


## Main thread: a fill's measured GPU time (CloudEffect: its timestamp, named by the batch's fill_id; applied by the
## next run).
func apply_fill_time(fill_id: int, ms: float) -> void:
	_inbox_fills.append(Vector2(fill_id, ms))


func _apply_distances(batch_epoch: int, tiles: PackedInt32Array, bits: PackedByteArray) -> void:
	if batch_epoch != epoch or schedule == null:
		return
	schedule.set_tile_distances(tiles, bits)
	for tile in tiles:  # the anchors' nearest clouds (the eviction's scan trigger)
		var index := _tile_anchor[tile]
		if index >= 0:
			_anchor_min_near_m[index] = minf(_anchor_min_near_m[index], schedule.near_of(tile))


## Worker: a fill's measured GPU time scales its tiles' latencies by measured / predicted (an older epoch's fill only
## joins the recent latencies).
func _apply_fill_time(fill_id: int, ms: float) -> void:
	var fill: Dictionary = _pending.get(fill_id, {})
	if fill.is_empty():
		return
	_pending.erase(fill_id)
	var predicted: float = fill["predicted_ms"]
	fill_log.append(Vector3(ms, predicted, fill["budget_ms"]))
	if fill_log.size() > FILL_LOG_MAX:
		fill_log.remove_at(0)
	if ms <= 0.0 or predicted <= 0.0:
		return
	var scale := ms / predicted
	if OS.has_environment("STARSIM_CACHE_FILL_TRACE"):
		var counts := [0, 0, 0]
		for level in (fill["levels"] as PackedInt32Array):
			counts[level] += 1
		print("fill_trace %d ms %.3f predicted %.3f levels %s latency %.3f budget %.2f" % [fill_id, ms, predicted,
				counts, float(fill["latency_ms"]) * scale, fill["budget_ms"]])
	var levels: PackedInt32Array = fill["levels"]
	var tiles: PackedInt32Array = fill["tiles"]
	var estimates: PackedFloat32Array = fill["estimates"]
	var current: bool = fill["epoch"] == epoch
	var slowest := [0.0, 0.0, 0.0]
	var ratios := [[], [], []]
	var n := CloudViewCacheLayout.LEVELS
	for i in range(tiles.size()):
		var level := levels[i]
		var latency := estimates[i] * scale
		slowest[level] = maxf(slowest[level], latency)
		if not current:
			continue
		_tile_latency_ms[tiles[i] * n + level] = latency
		var coarser := _tile_latency_ms[tiles[i] * n + level - 1] if level > 0 else 0.0
		if coarser > 0.0:
			ratios[level].append(latency / coarser)
	for level in range(n):
		if slowest[level] > 0.0:
			_push(_recent_latency_ms[level], slowest[level])
		if not ratios[level].is_empty():
			ratios[level].sort()
			_push(_level_ratios[level], ratios[level][ratios[level].size() / 2])


## The measured fills since the last call (diagnostics): (measured ms, predicted ms, budget ms) each.
func take_fill_log() -> Array[Vector3]:
	var log := fill_log
	fill_log = []
	return log


## The fraction of the cube's tiles showing the full level (0 without a layout).
func full_fraction() -> float:
	return float(_full_count) / layout.tile_count() if layout != null else 0.0


## The level a tile shows (-1: none).
func shown_level(tile: int) -> int:
	return _shown[tile]


## The cross-fade weight of a tile's shown content at time t (1: complete).
func fade_weight(tile: int, t: float) -> float:
	var start := _states.decode_float(8 * tile + 4)
	return clampf((t - start) / CloudViewCache.CROSSFADE_S, 0.0, 1.0)


## The ring entry of the anchor a tile shows (-1: none), and a ring entry's anchor (tests).
func tile_anchor(tile: int) -> int:
	return _tile_anchor[tile]


func anchor_at(index: int) -> Vector3:
	return _anchor_ring[index]


## What a tile fades from (0 nothing, 1-3 that level + 1, 4 its fade slot).
func fade_source(tile: int) -> int:
	return (_states.decode_u32(8 * tile) >> 2) & 7


## A tile's predicted latency at a level (ms): measured, else its coarser level's times the ratios, else unknown_ms.
func _latency_of(tile: int, level: int, ratio: PackedFloat64Array, unknown_ms: PackedFloat64Array) -> float:
	var n := CloudViewCacheLayout.LEVELS
	var scale := 1.0
	for at in range(level, -1, -1):
		var latency := _tile_latency_ms[tile * n + at]
		if latency > 0.0:
			return latency * scale
		scale *= ratio[at]
	return unknown_ms[level]


## A level's latency over its coarser one's: the median of the recent fills' medians (LEVEL_RATIO_PRIOR before any;
## their largest over-predicted the VR proxy's fills ~3 x, the fill cap bounds the error of the median).
func _level_ratio(level: int) -> float:
	var ratios: Array = _level_ratios[level]
	if ratios.is_empty():
		return LEVEL_RATIO_PRIOR
	var sorted := ratios.duplicate()
	sorted.sort()
	return sorted[sorted.size() / 2]


## The predicted latency of a tile never measured at a level (ms): the largest of the recent fills' slowest there, else
## from_coarser (the coarser level's times the ratio; 0: none), else UNKNOWN_LATENCY_MS.
func _unknown_latency_ms(level: int, from_coarser: float) -> float:
	var recent: Array = _recent_latency_ms[level]
	if recent.is_empty():
		return from_coarser if from_coarser > 0.0 else UNKNOWN_LATENCY_MS
	var most := 0.0
	for ms: float in recent:
		most = maxf(most, ms)
	return most


## Appends to a bounded history (LATENCY_HISTORY entries).
static func _push(history: Array, value: float) -> void:
	history.append(value)
	if history.size() > LATENCY_HISTORY:
		history.remove_at(0)


## The view's cone (the covering test): the sample rays' mean direction and the angle to the farthest of them plus the
## sample spacing and a tile's radius (a tile touching the view has its centre inside).
func _view_cone(dirs: Array[Vector3], sample_rad: float) -> void:
	var centre := Vector3.ZERO
	for d in dirs:
		centre += d
	if dirs.is_empty() or centre.length() < 1.0e-6:
		_view_centre = Vector3(0.0, 0.0, 1.0)
		_view_cos = -1.0
		return
	centre = centre.normalized()
	var least := 1.0
	for d in dirs:
		least = minf(least, centre.dot(d))
	_view_centre = centre
	_view_cos = cos(minf(acos(clampf(least, -1.0, 1.0)) + sample_rad + layout.tile_radius_rad(), PI))


## The layout of a face size (rounded up to whole tiles), its schedule and tile directions.
func _build(face_texels: int) -> void:
	var tile_texels := CloudViewCache.TILE_TEXELS
	var built_layout := CloudViewCacheLayout.new(maxi(ceili(float(face_texels) / tile_texels), 1) * tile_texels,
			tile_texels)
	_built = {"layout": built_layout, "schedule": CloudViewCacheSchedule.new(built_layout),
		"dirs": built_layout.tile_dirs()}


## The built layout and schedule taken over (the shown states sized for them).
func _adopt_layout() -> void:
	layout = _built["layout"]
	schedule = _built["schedule"]
	_tile_dirs = _built["dirs"]
	_built = {}
	var n := layout.tile_count()
	_shown.resize(n)
	_tile_anchor.resize(n)
	_states.resize(8 * n)
	_tile_latency_ms.resize(n * CloudViewCacheLayout.LEVELS)
	_level_groups.resize(CloudViewCacheLayout.LEVELS)
	for level in range(CloudViewCacheLayout.LEVELS):
		var side := (layout.tile_texels >> (CloudViewCacheSchedule.MAX_LEVEL - level)) / CloudViewCache.BLOCK
		_level_groups[level] = side * side


## Everything restarts from nothing at the camera: no tile shows anything (no old content after a jump), no tile's cost
## is known (another place, another light).
func _restart(camera_earth_m: Vector3) -> void:
	# (_reset_pending belongs to the main thread: plan() clears it when it hands the request to the run.)
	anchor_earth_m = camera_earth_m
	anchor_index = 0
	_anchor_ring.resize(ANCHOR_RING)
	_anchor_ring.fill(camera_earth_m)
	_anchor_tiles.resize(ANCHOR_RING)
	_anchor_tiles.fill(0)
	_anchor_list.resize(ANCHOR_RING)
	for index in range(ANCHOR_RING):
		_anchor_list[index] = PackedInt32Array()
	_anchor_min_near_m.resize(ANCHOR_RING)
	_anchor_min_near_m.fill(INF)
	_tile_anchor.fill(-1)
	resets += 1
	epoch += 1
	schedule.invalidate_all()
	_shown.fill(-1)
	_states.fill(0)
	_tile_latency_ms.fill(0.0)
	_slot_end_s.resize(SLOT_COUNT)
	_slot_end_s.fill(-INF)
	_slot_owner.resize(SLOT_COUNT)
	_slot_owner.fill(-1)
	_full_count = 0
	_states_dirty = true
	last_covering = true  # nothing in view yet
	view_missing = true


## The camera becomes the anchor: the ring's next entry (its tiles, if any still show it, dropped first). Every tile
## keeps what it shows; the schedule sees the new anchor at the next pick (its tiles all off: refilled by parallax and
## staleness).
func _reanchor(camera_earth_m: Vector3) -> void:
	var next := (anchor_index + 1) % ANCHOR_RING
	_evict(next)
	anchor_index = next
	_anchor_ring[next] = camera_earth_m
	_anchor_min_near_m[next] = INF
	anchor_earth_m = camera_earth_m
	reanchors += 1


## Drops each tile the camera left farther from its anchor than the tile's keep distance
## (CloudViewCacheSchedule.keep_distance_m of its nearest cached cloud: its own parallax), then hands the schedule the
## largest offset of a kept tile's anchor from the current one (its parallax class's bound). An entry's tiles are
## scanned only when the camera may have left the keep distance of the entry's nearest cloud.
func _evict_far(camera_earth_m: Vector3) -> void:
	var cap_m := CloudViewCacheSchedule.keep_distance_m(INF)
	var deviation_m := 0.0
	for index in range(ANCHOR_RING):
		if _anchor_tiles[index] <= 0 or index == anchor_index:
			continue
		var offset_m := camera_earth_m.distance_to(_anchor_ring[index])
		if offset_m > CloudViewCacheSchedule.keep_distance_m(_anchor_min_near_m[index]) or offset_m > cap_m:
			var kept := PackedInt32Array()
			var dropped := PackedInt32Array()
			var nearest_m := INF
			for tile in _anchor_list[index]:
				if _tile_anchor[tile] != index:
					continue  # refilled from another anchor since
				var near_m := schedule.near_of(tile)
				if offset_m > CloudViewCacheSchedule.keep_distance_m(near_m):
					dropped.append(tile)
				else:
					kept.append(tile)
					nearest_m = minf(nearest_m, near_m)
			_anchor_list[index] = kept
			_anchor_min_near_m[index] = nearest_m
			_drop(dropped)
		if _anchor_tiles[index] > 0:
			deviation_m = maxf(deviation_m, anchor_earth_m.distance_to(_anchor_ring[index]))
	schedule.set_anchor_deviation(anchor_earth_m, deviation_m)


## The tiles showing a ring entry lose their content (no level: filled again like new ones, the march covers them in
## view).
func _evict(index: int) -> void:
	var dropped := PackedInt32Array()
	for tile in _anchor_list[index]:
		if _tile_anchor[tile] == index:
			dropped.append(tile)
	_drop(dropped)
	_anchor_list[index] = PackedInt32Array()
	_anchor_min_near_m[index] = INF


## Tiles lose their content (no level: filled again like new ones; the march covers them in view).
func _drop(tiles: PackedInt32Array) -> void:
	if tiles.is_empty():
		return
	for tile in tiles:
		if _shown[tile] == CloudViewCacheSchedule.MAX_LEVEL:
			_full_count -= 1
		_shown[tile] = -1
		_anchor_tiles[_tile_anchor[tile]] -= 1
		_tile_anchor[tile] = -1
		_states.encode_u32(8 * tile, 0)
		_states.encode_float(8 * tile + 4, 0.0)
	schedule.forget(tiles)
	evicted_tiles += tiles.size()
	_states_dirty = true


## A tile filled at `level` this frame: its shown state and fade; the fade slot the fill copies the old texels to (-1:
## none).
func _shown_now(tile: int, level: int) -> int:
	var old := _shown[tile]
	var from := 0  # nothing: a first fill fades in from T = 1 (or the covering march)
	var slot := -1
	if old >= 0 and level > old:
		from = old + 1
	elif old >= 0:
		# A refresh: the shown texels go to a slot first (a slot still fading ends that fade: the ring is too short).
		slot = _slot_cursor
		_slot_cursor = (_slot_cursor + 1) % SLOT_COUNT
		var owner := _slot_owner[slot]
		if _slot_end_s[slot] > now_s and owner >= 0 and owner != tile and fade_source(owner) == 4 \
				and (_states.decode_u32(8 * owner) >> 8) & 1023 == slot:
			slot_overflows += 1
			_states.encode_float(8 * owner + 4, now_s - CloudViewCache.CROSSFADE_S)  # its fade ends now
		_slot_end_s[slot] = now_s + CloudViewCache.CROSSFADE_S
		_slot_owner[slot] = tile
		from = 4
	elif _tile_dirs[tile].dot(_view_centre) >= _view_cos:
		_cover_until_s = now_s + CloudViewCache.CROSSFADE_S  # the march covers until it has faded in
	if level == CloudViewCacheSchedule.MAX_LEVEL and old != level:
		_full_count += 1
	elif old == CloudViewCacheSchedule.MAX_LEVEL and level != old:
		_full_count -= 1
	_shown[tile] = level
	# The anchor: the current one; what it fades from was rendered from its previous one (the lookup reads that at the
	# same point of the eye ray).
	var old_anchor := _tile_anchor[tile]
	var from_anchor := old_anchor if old >= 0 and old_anchor >= 0 else anchor_index
	if old_anchor >= 0:
		_anchor_tiles[old_anchor] -= 1
	if old_anchor != anchor_index:
		_anchor_list[anchor_index].append(tile)
	_tile_anchor[tile] = anchor_index
	_anchor_tiles[anchor_index] += 1
	_states.encode_u32(8 * tile, (level + 1) | (from << 2) | (maxi(slot, 0) << 8) | (anchor_index << 18)
			| (from_anchor << 25))
	_states.encode_float(8 * tile + 4, now_s)
	_states_dirty = true
	last_fades.append(tile)
	last_fade_s = now_s
	return slot
