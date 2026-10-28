class_name CloudWorldVolumeSchedule
extends RefCounted
## What the cloud world volume bakes and lights next (world-volume plan Task 3; pure logic, no GPU - CloudWorldVolume
## runs the batches on the render thread). Patterns of cloud_light_cache_schedule.gd: a tangent frame (here the layout's
## azimuthal-equidistant frame), re-anchoring (re-frame), budgeted jobs.
##
## plan() returns one frame's batch, executed in this order on the GPU: bake (density of both snapshots, or of one
## snapshot's channel pair), provisional light (copied from the next coarser level; the coarsest level's from the
## grid-mean column), light (real). `lookup` is the state AFTER the batch (per level origin / active / ready / frame);
## the render thread adopts it only once the batch ran.
##
## Scroll (Task 5 review I2: bake ahead of exposure): a level's texture holds its storage box, the window the lookup
## reads is CloudWorldVolumeLayout.GUARD_CELLS smaller on each side. When the camera moves, the storage moves toward
## the window's target grown by the guard (only as far as it still holds the shown window), and the slabs it exposes
## are queued (`scroll`). They are baked earliest deadline first: a slab's deadline is the frames until the window,
## at the camera's speed per axis (MOTION_DECAY peak hold), would reach it; the frame's scroll budget is the rate that
## meets every deadline times SCROLL_MARGIN, capped at SCROLL_CAP_MS (the spike cap less the light's share), and the
## normal bake share left after fills tops it up (baking further ahead). The window then advances toward the camera
## only onto baked cells (never late: no frame reads an unbaked cell; a slab that misses its deadline makes the window
## wait - lag, the coarser level shows the clouds there). Nesting (review round 2, N1): the windows advance coarse to
## fine and a level nested in the next coarser one moves only to positions that keep it nested (it waits for it); a
## level is shown (lookup `shown`) while it is ready and the next coarser level is shown with it nested. Baked slabs
## are lit first (priority 1: they have no light yet); whatever of them the light budget cannot cover gets provisional
## light and stays first in line (PROVISIONAL_MAX_S is the gate's bound on how long that may last).
## Fill (cold start, jump, level switched on, whole-window moves): the window is "dirty" and baked under the budget,
## coarse to fine, the coarsest level first (world-volume Task 5: its 8.4 M cells of 8448 taps take the GPU far longer
## than a frame's UPKEEP_SPIKE_MS, so it is budgeted too; until it is `ready` nothing of the volume is shown - the march's
## fallback, today's per-sample march, draws the frame). A level is `ready` (filled) once its dirty list is empty. Pair
## switch: a snapshot replaced in slot s is rebaked into slot s's channel pair alone, under the bake budget, coarse to
## fine - for a level still filling only what its fill has baked already (the rest bakes both snapshots anyway);
## `slot_ready[s]` tells when it may take weight (the spare included).
## Light refresh: per level the Sun's turn (shadow shift H dtheta / sin^2 e < 1/4 cell), the Moon's (only while the Sun is
## below MOON_SUN_ELEVATION_RAD at the level centre), the blend's step (WEATHER_REFRESH_STEP) start a new generation:
## the level's window is queued as tiles, nearest (and in view) first, behind what an older generation left in the
## queue (no tile starves under time warp), served in equal shares finest level first.
## Re-frame: the camera's ground point > REFRAME_M from the frame origin -> a new frame at the camera; levels are
## rebuilt one at a time, coarsest first, into a spare version in the new frame, shown (the old version stays shown
## meanwhile), cross-faded over REFRAME_FADE_S and swapped.
## Budget: UPKEEP_BUDGET_MS per frame (LIGHT_BUDGET_MS of it light); UPKEEP_SPIKE_MS in cold-start, jump and re-frame
## frames until the fill is complete (half bake, half light). The bake's cost is counted in taps (a cell of level k
## costs CloudWorldVolumeLayout.cell_taps(k): 256 at level 0 to 8448 at level 7), its rate in taps per ms; the scroll
## bake may exceed the normal bake share up to SCROLL_CAP_MS (fast flight: 7.8 km/s needs ~1-1.5 ms at 4.3e7 taps/ms).

const UPKEEP_BUDGET_MS := 1.5
const LIGHT_BUDGET_MS := 1.0
const UPKEEP_SPIKE_MS := 3.0
## The scroll bake's cap per frame: the spike cap less the light's share (a frame's upkeep stays <= UPKEEP_SPIKE_MS).
const SCROLL_CAP_MS := UPKEEP_SPIKE_MS - LIGHT_BUDGET_MS
## [E] The scroll budget is the deadline-meeting rate times this: the speed may grow between plans and a slab is
## baked in whole cells.
const SCROLL_MARGIN := 1.5
## [E] Per-axis speed estimate (grid m per plan): the larger of this plan's step and the last estimate times this
## (a frame without motion, e.g. a paused clock, does not drop the deadlines to infinity at once).
const MOTION_DECAY := 0.9
## [E] Deadline (frames) of a slab while the camera stands still along its axes: baked from the spare budget only.
const IDLE_FRAMES := 1.0e6
const PROVISIONAL_FADE_S := 0.25
const PROVISIONAL_MAX_S := 1.0
const REFRAME_FADE_S := 0.5
const WEATHER_REFRESH_STEP := 0.02
## [E] The 1 % front bound: a refresh moves a cell's exp(-tau) by at most this (the GPU's step per refresh).
const LIGHT_STEP_T := 0.01
const MOON_SUN_ELEVATION_RAD := deg_to_rad(-6.0)

## [E] Light-refresh tile (cells per axis): ~0.03 ms of light at the calibrated rate; 405 tiles at most per level.
const TILE_CELLS := 32
## [E] The elevation the Sun's / Moon's refresh threshold is evaluated at, at lowest: below it sin^2 e -> 0 would
## demand a continuous refresh at sunrise / sunset; the budget makes it lag instead (never a front).
const MIN_REFRESH_ELEVATION_RAD := deg_to_rad(5.0)
## [E] Lowest H (m) in the threshold: a level whose window lies above the cloud top has nothing to refresh anyway.
const MIN_HEIGHT_M := 100.0
## [E] Tiles in view are served before the ones behind the camera: behind counts twice as far.
const BEHIND_FACTOR := 2
## [E] Start rates, replaced by feedback(): the spec's 67 M light cells in 60 ms; the bake's taps per ms half the RTX
## 4090's measured cold-start rate (4.3e7, cloud_check.gd section 16) - feedback() raises it by at most 20 % a frame,
## and a first frame below its budget only lags.
const INITIAL_LIGHT_CELLS_PER_MS := 67.0e6 / 60.0
const INITIAL_BAKE_TAPS_PER_MS := 2.0e7
## [E] feedback() moves the rates by this fraction of the measured error per call, at most by the limits.
const CALIBRATION_GAIN := 0.2
const CALIBRATION_LIMITS := Vector2(0.5, 2.0)
const MIN_CELLS_PER_MS := 1.0e4
## Cells of a tile's key order inside the sort key (tiles per level <= 4096).
const _KEY_BITS := 12

## Calibrated by feedback().
var bake_taps_per_ms := INITIAL_BAKE_TAPS_PER_MS
var light_cells_per_ms := INITIAL_LIGHT_CELLS_PER_MS
## Optional: the camera's view direction (Earth-local); tiles behind it are refreshed later. ZERO = ignore.
var view_dir_earth := Vector3.ZERO
## Checks only (cloud_check.gd section 16: a cold fill in few frames): scales every per-frame budget.
var budget_scale := 1.0


class _Level extends RefCounted:
	var k := 0
	var layout: CloudWorldVolumeLayout
	var shown := true  # false: a re-frame's spare while it is built
	var active := false
	var ready := false
	## Shown to the march (lookup "shown"): ready, and every coarser level drawn with this window nested (N1).
	var drawn := false
	var origin := Vector3i.ZERO  # the window's (what the lookup reads)
	var storage := Vector3i.ZERO  # the texture's cells (CloudWorldVolumeLayout.storage_box)
	var scroll: Array = []  # storage slabs {from, to} exposed by a scroll, still to bake (both snapshots)
	var dirty: Array = []  # boxes {from, to} still to bake (both snapshots)
	var baked: Array = []  # boxes a fill in progress has baked (a slot change rebakes these alone)
	var slot_dirty: Array = [[], []]  # per snapshot slot: boxes to rebake (channel pair of that slot)
	var prov: Array = []  # baked boxes {from, to, emitted, age_s} still to get real light
	var queue: Array = []  # tile keys (Vector3i) of the refresh generation(s) in progress
	var queued := {}
	var partial: Array = []  # boxes: the rest of the tile being served
	var need_queue := false
	var lit_valid := false
	var lit_sun := Vector3.ZERO
	var lit_moon := Vector3.ZERO
	var lit_blend := 0.0
	var lit_moon_on := false
	var moon_on := false


var _layout: CloudWorldVolumeLayout
var _next: CloudWorldVolumeLayout
var _levels: Array = []
var _spare: _Level
var _todo: Array[int] = []
var _fade_s := 0.0
var _started := false
var _spike := false
var _time_s := 0.0
var _slot_ids: Array = [0, 1]
var _slot_ready: Array = [true, true]
var _bake_cells := 0
var _bake_taps := 0
var _scroll_cells := 0
var _scroll_taps := 0
var _light_cells := 0
var _prov_cells := 0
var _prov_age_max := 0.0
var _prov_age_session_max := 0.0
var _camera_grid := Vector3.ZERO
var _camera_grid_next := Vector3.ZERO
var _camera_prev := Vector3.ZERO
var _has_prev := false
## The camera's speed per grid axis (m per plan, MOTION_DECAY peak hold).
var _motion := Vector3.ZERO
var _waiting := 0


func _init(layout: CloudWorldVolumeLayout = null) -> void:
	_layout = layout if layout != null else CloudWorldVolumeLayout.new()


## Forgets everything but the calibration: the next plan is a cold start.
func reset() -> void:
	_started = false
	_next = null
	_spare = null
	_todo.clear()
	_levels.clear()


## Shadow shift H dtheta / sin^2 e kept below 1/4 cell: the Sun may turn this far (rad) before a level is relit.
## H = the cloud top above the level's lowest cell (m).
static func sun_threshold_rad(k: int, sun_elevation_rad: float, height_above_m: float) -> float:
	var s := sin(sun_elevation_rad)
	return 0.25 * CloudWorldVolumeLayout.CELL_H_M[k] * s * s / height_above_m


## Rate calibration from GPU timestamps: the measured upkeep time of a frame against the model's (bake taps, light
## cells as the frame ran them).
func feedback(upkeep_ms: float, bake_taps: int, light_cells: int) -> void:
	if upkeep_ms <= 0.0:
		return
	var model := bake_taps / bake_taps_per_ms + light_cells / light_cells_per_ms
	if model <= 0.0:
		return
	var f := clampf(model / upkeep_ms, CALIBRATION_LIMITS.x, CALIBRATION_LIMITS.y)
	var g := 1.0 + CALIBRATION_GAIN * (f - 1.0)
	bake_taps_per_ms = maxf(bake_taps_per_ms * g, MIN_CELLS_PER_MS)
	light_cells_per_ms = maxf(light_cells_per_ms * g, MIN_CELLS_PER_MS)


## Cold-start / jump / re-frame fill and the spare version are done; nothing is waiting to be baked or lit first
## (queued scroll slabs do not count: a moving camera always has some, and the spike must end).
func complete() -> bool:
	if not _started or _spare != null or _next != null:
		return false
	for lv: _Level in _levels:
		if lv.active and (not lv.ready or not lv.dirty.is_empty() or not lv.prov.is_empty()
				or not lv.slot_dirty[0].is_empty() or not lv.slot_dirty[1].is_empty()):
			return false
	return true


## True while level k's light is behind its target (a refresh generation or provisional light is pending).
func light_stale(k: int) -> bool:
	var lv: _Level = _levels[k]
	return lv.need_queue or not lv.queue.is_empty() or not lv.partial.is_empty() or not lv.prov.is_empty()


func plan(camera_earth_m: Vector3, sun_dir_earth: Vector3, moon_dir_earth: Vector3, blend: float,
		cloud_top_m: float, jumped: bool, delta_s: float, slot_ids: Array = [0, 1]) -> Dictionary:
	_time_s += delta_s
	var out := {"bake": [], "light": [], "provisional": []}
	_bake_cells = 0
	_bake_taps = 0
	_scroll_cells = 0
	_scroll_taps = 0
	_light_cells = 0
	_prov_cells = 0
	_prov_age_max = 0.0
	_waiting = 0
	if jumped or not _started:
		_cold_start(camera_earth_m, slot_ids)
	_apply_slot_ids(slot_ids)
	_reframe_begin(camera_earth_m)
	_camera_grid = _layout.grid_of(camera_earth_m)
	if _next != null:
		_camera_grid_next = _next.grid_of(camera_earth_m)
	_update_motion(camera_earth_m)
	var order := _levels.duplicate()
	order.reverse()  # coarse to fine
	if _spare != null:
		order.append(_spare)
	for lv: _Level in order:
		_scroll(lv, out["bake"])
	var scroll_items := _scroll_items(order)
	_bake_scroll(scroll_items, _scroll_budget(scroll_items), out["bake"])  # what the deadlines need
	_bake_fill(order, out["bake"])
	# the normal bake share the fills left: further ahead (deadlines recomputed: the first pass split boxes)
	_bake_scroll(_scroll_items(order), int(_bake_ms() * bake_taps_per_ms) - _bake_taps, out["bake"])
	for lv: _Level in order:
		_advance_window(lv)
	_light(order, sun_dir_earth, moon_dir_earth, blend, cloud_top_m, delta_s, out)
	var reframe := _reframe_end(delta_s)
	if _spike and complete():
		_spike = false
	out["reframe"] = reframe
	out["lookup"] = _lookup()
	out["bake_cells"] = _bake_cells
	out["bake_taps"] = _bake_taps
	out["bake_scroll_cells"] = _scroll_cells
	out["bake_scroll_taps"] = _scroll_taps
	out["windows_waiting"] = _waiting
	out["light_cells"] = _light_cells
	out["provisional_cells"] = _prov_cells
	out["provisional_age_max_s"] = _prov_age_max
	out["provisional_max_s"] = _prov_age_session_max
	out["spike"] = _spike
	return out


# ---------------------------------------------------------------- start, slots, scroll

func _cold_start(camera_earth_m: Vector3, slot_ids: Array) -> void:
	_layout.set_frame(camera_earth_m)
	_next = null
	_spare = null
	_todo.clear()
	_levels.clear()
	for k in CloudWorldVolumeLayout.LEVELS:
		var lv := _Level.new()
		lv.k = k
		lv.layout = _layout
		_levels.append(lv)
	_slot_ids = slot_ids.duplicate()
	_slot_ready = [true, true]
	_started = true
	_spike = true
	_fade_s = 0.0
	_has_prev = false  # a jump is no motion
	_motion = Vector3.ZERO


## A snapshot replaced in a slot: its channel pair is rebaked alone, under the bake budget - the whole window of a
## ready level, the part a fill has baked so far of a level still filling (its dirty rest bakes both snapshots).
func _apply_slot_ids(slot_ids: Array) -> void:
	for s in 2:
		if slot_ids[s] == _slot_ids[s]:
			continue
		_slot_ids[s] = slot_ids[s]
		for lv: _Level in _all_levels():
			if lv.active:
				lv.slot_dirty[s] = lv.baked.duplicate() if not lv.dirty.is_empty() \
						else [CloudWorldVolumeLayout.storage_box(lv.storage)]


func _all_levels() -> Array:
	var all := _levels.duplicate()
	if _spare != null:
		all.append(_spare)
	return all


func _grid_of_level(lv: _Level) -> Vector3:
	return _camera_grid_next if lv.layout == _next else _camera_grid


## The camera's speed per grid axis (m per plan) in the current frame, MOTION_DECAY peak hold.
func _update_motion(camera_earth_m: Vector3) -> void:
	var step := Vector3.ZERO
	if _has_prev:
		step = (_camera_grid - _layout.grid_of(_camera_prev)).abs()
	_motion = Vector3(maxf(step.x, _motion.x * MOTION_DECAY), maxf(step.y, _motion.y * MOTION_DECAY),
			maxf(step.z, _motion.z * MOTION_DECAY))
	_camera_prev = camera_earth_m
	_has_prev = true


## A level's window follows the camera: a filling level's window and storage at once (the exposed cells join the fill);
## a ready level's storage toward the window's target grown by the guard, as far as it still holds the shown window,
## the cells it exposes queued for the scroll bake (the window follows in _advance_window).
func _scroll(lv: _Level, _bake_out: Array) -> void:
	var L := CloudWorldVolumeLayout
	var cg := _grid_of_level(lv)
	if not lv.layout.level_active(lv.k, cg.z):
		if lv.active:
			_clear(lv)
		return
	var target := lv.layout.window_origin(lv.k, cg)
	if not lv.active:
		_restart_fill(lv, target)
		return
	var whole := Vector3i(L.CELLS_H, L.CELLS_H, L.CELLS_V)
	var d := (target - lv.origin).abs()
	if d.x >= whole.x or d.y >= whole.y or d.z >= whole.z:  # a whole window: a fill, not a scroll
		_restart_fill(lv, target)
		return
	var storage := L.storage_target(lv.k, target)
	if lv.ready:
		var limits := L.storage_range(lv.k, lv.origin)
		storage = storage.clamp(limits[0], limits[1])
	if storage == lv.storage:
		if not lv.ready:
			_set_origin(lv, target)
		return
	var slabs := L.exposed_slabs(lv.k, lv.storage, storage)
	lv.storage = storage
	var box := L.storage_box(storage)
	if not lv.ready:
		lv.dirty.append_array(slabs)
		_set_origin(lv, target)
		return
	var kept: Array = []
	for b: Dictionary in lv.scroll:  # cells that left the storage are not baked any more (their texels moved on)
		var c := L.box_clip(b, box)
		if L.box_cells(c) > 0:
			kept.append(c)
	lv.scroll = kept + slabs


func _restart_fill(lv: _Level, target: Vector3i) -> void:
	_clear(lv)
	lv.active = true
	_set_origin(lv, target)
	lv.storage = CloudWorldVolumeLayout.storage_target(lv.k, target)
	lv.dirty = [CloudWorldVolumeLayout.storage_box(lv.storage)]


## Every level's queued scroll slabs [deadline frames, level, box], earliest first.
func _scroll_items(order: Array) -> Array:
	var items: Array = []
	for lv: _Level in order:
		if not lv.active or not lv.ready:
			continue
		var target := lv.layout.window_origin(lv.k, _grid_of_level(lv))
		for b: Dictionary in lv.scroll:
			items.append([_deadline_frames(lv, b, target), lv, b])
	items.sort_custom(func(a: Array, b: Array) -> bool: return a[0] < b[0])
	return items


## Frames (this one included) until the window, at the camera's speed, would hold a cell of `box`: 1 if the target
## window already does. Along each axis the box lies apart from the target window, the camera must cross the rest of
## its cell and gap - 1 more cells; the window reaches the box when all those axes are crossed.
func _deadline_frames(lv: _Level, box: Dictionary, target: Vector3i) -> float:
	var L := CloudWorldVolumeLayout
	var win := L.window_box(lv.k, target)
	var cg := _grid_of_level(lv)
	var cell := Vector3(L.CELL_H_M[lv.k], L.CELL_H_M[lv.k], L.CELL_V_M[lv.k])
	var u := Vector3(cg.x / cell.x, cg.y / cell.y, (cg.z - L.SHELL_BOTTOM_M) / cell.z)
	var frames := 1.0
	for axis in 3:
		var gap := 0
		var partial := 0.0
		if box["from"][axis] >= win["to"][axis]:
			gap = box["from"][axis] - win["to"][axis] + 1
			partial = ceilf(u[axis]) - u[axis] if u[axis] != floorf(u[axis]) else 1.0
		elif box["to"][axis] <= win["from"][axis]:
			gap = win["from"][axis] - box["to"][axis] + 1
			partial = u[axis] - floorf(u[axis])
		if gap == 0:
			continue
		var metres := ((gap - 1) + partial) * cell[axis]
		var f := IDLE_FRAMES if _motion[axis] <= 0.0 else 1.0 + floorf(metres / _motion[axis])
		frames = maxf(frames, minf(f, IDLE_FRAMES))
	return frames


## The taps that meet every deadline at a constant rate (earliest deadline first), times SCROLL_MARGIN, capped.
func _scroll_budget(items: Array) -> int:
	var need := 0.0
	var cum := 0.0
	for it: Array in items:
		var lv: _Level = it[1]
		cum += CloudWorldVolumeLayout.box_cells(it[2]) * CloudWorldVolumeLayout.cell_taps(lv.k)
		need = maxf(need, cum / float(it[0]))
	return int(minf(need * SCROLL_MARGIN, budget_scale * SCROLL_CAP_MS * bake_taps_per_ms))


## Bakes queued scroll slabs earliest deadline first, up to `budget` taps (at least one cell when the budget is
## positive); baked cells get light first (prov list).
func _bake_scroll(items: Array, budget: int, bake_out: Array) -> void:
	var L := CloudWorldVolumeLayout
	for it: Array in items:
		if budget <= 0:
			return
		var lv: _Level = it[1]
		var box: Dictionary = it[2]
		var i := lv.scroll.find(box)
		if i < 0:
			continue  # baked whole by an earlier call this plan
		var sp := L.box_split(box, _cells_for(lv.k, budget))
		_emit_bake(lv, sp[0], -1, bake_out)
		var cells := L.box_cells(sp[0])
		_scroll_cells += cells
		_scroll_taps += cells * L.cell_taps(lv.k)
		budget -= cells * L.cell_taps(lv.k)
		lv.prov.append({"from": sp[0]["from"], "to": sp[0]["to"], "emitted": false, "age_s": 0.0})
		lv.scroll.remove_at(i)
		var rest: Array = sp[1]
		for j in rest.size():
			lv.scroll.insert(i + j, rest[j])
		it[2] = rest[0] if not rest.is_empty() else {}
		if not rest.is_empty():
			return  # the budget is spent (box_split took what it paid for)


## Moves a ready level's window toward the camera one cell at a time per axis, only while the window stays inside the
## storage and holds no queued (unbaked) cell, and - while the level is nested in the next coarser one, shown -
## only onto positions that keep it nested (N1: a finer level waits for its coarser one; called coarse to fine); a
## window left behind its target is counted (waiting: lag). Then whether the level is drawn.
func _advance_window(lv: _Level) -> void:
	lv.drawn = false
	if not lv.active or not lv.ready:
		return
	var up := _up_level(lv)
	var keep := up != null and up.drawn and CloudWorldVolumeLayout.nested(lv.k, lv.origin, up.origin)
	var target := lv.layout.window_origin(lv.k, _grid_of_level(lv))
	var o := lv.origin
	for axis in 3:
		while o[axis] != target[axis]:
			var c := o
			c[axis] += signi(target[axis] - o[axis])
			if not _window_holds(lv, c) or (keep and not CloudWorldVolumeLayout.nested(lv.k, c, up.origin)):
				break
			o = c
	if o != lv.origin:
		_set_origin(lv, o)
	if o != target:
		_waiting += 1
	lv.drawn = _nests_in_up(lv)


## The next coarser level a level's fade band blends (null for the coarsest and for a re-frame's spare).
func _up_level(lv: _Level) -> _Level:
	if lv == _spare or lv.k + 1 >= CloudWorldVolumeLayout.LEVELS:
		return null
	return _levels[lv.k + 1]


## The N1 invariant for a ready level: the coarsest one, or the next coarser level drawn and (in the same frame) holding
## this window where its weight is 1. Levels of different frames (a re-frame in progress) are not compared here.
func _nests_in_up(lv: _Level) -> bool:
	var up := _up_level(lv)
	if up == null:
		return true
	return up.drawn and (up.layout != lv.layout or CloudWorldVolumeLayout.nested(lv.k, lv.origin, up.origin))


func _window_holds(lv: _Level, origin: Vector3i) -> bool:
	var L := CloudWorldVolumeLayout
	var limits := L.storage_range(lv.k, origin)
	if lv.storage != lv.storage.clamp(limits[0], limits[1]):
		return false
	var win := L.window_box(lv.k, origin)
	for b: Dictionary in lv.scroll:
		if L.box_overlaps(win, b):
			return false
	return true


## The level's window origin, also in its layout (fade_weight reads it there; a spare's goes to the new frame's layout).
func _set_origin(lv: _Level, origin: Vector3i) -> void:
	lv.origin = origin
	lv.layout.set_origin(lv.k, origin)


func _clear(lv: _Level) -> void:
	lv.active = false
	lv.ready = false
	lv.dirty = []
	lv.scroll = []
	lv.baked = []
	lv.slot_dirty = [[], []]
	lv.prov = []
	lv.queue = []
	lv.queued = {}
	lv.partial = []
	lv.need_queue = false
	lv.lit_valid = false


func _emit_bake(lv: _Level, box: Dictionary, channel: int, bake_out: Array) -> void:
	bake_out.append({"level": lv.k, "from": box["from"], "to": box["to"], "snapshot_channel": channel,
			"spare": lv == _spare})
	var cells := CloudWorldVolumeLayout.box_cells(box)
	_bake_cells += cells
	_bake_taps += cells * CloudWorldVolumeLayout.cell_taps(lv.k)


# ---------------------------------------------------------------- bake

func _bake_ms() -> float:
	return budget_scale * (UPKEEP_SPIKE_MS * 0.5 if _spike else UPKEEP_BUDGET_MS - LIGHT_BUDGET_MS)


## The budget (taps) after the scroll slabs, spent coarse to fine on the fills, then the slot rebakes, then the spare;
## a finer level starts only once the coarser one is done (strictly coarse first, the budget's remainder of less than
## a cell left).
func _bake_fill(order: Array, bake_out: Array) -> void:
	var budget := maxi(int(_bake_ms() * bake_taps_per_ms) - _scroll_taps, 0)
	for lv: _Level in order:
		if budget <= 0:
			break
		if lv.active and not lv.dirty.is_empty() and lv != _spare:
			budget -= _bake_dirty(lv, budget, bake_out)
			if not lv.dirty.is_empty():
				break
	var slots_done := false
	for lv: _Level in order:
		for s in 2:
			if budget <= 0 or slots_done:
				break
			if lv.active and not lv.slot_dirty[s].is_empty():
				budget -= _bake_slot(lv, s, budget, bake_out)
				slots_done = not lv.slot_dirty[s].is_empty()
	if _spare != null and _spare.active and not _spare.dirty.is_empty() and budget > 0:
		_bake_dirty(_spare, budget, bake_out)
	for s in 2:
		var ready := true
		for lv: _Level in _all_levels():
			ready = ready and lv.slot_dirty[s].is_empty()
		_slot_ready[s] = ready


## Cells of level k a budget of `taps` pays for (at least one: a fill always moves).
static func _cells_for(k: int, taps: int) -> int:
	return maxi(taps / CloudWorldVolumeLayout.cell_taps(k), 1)


## Bakes up to `budget` taps of the level's dirty boxes (clipped to its window) and hands them to the light (prov
## list); returns the taps spent. The level is ready once nothing is dirty.
func _bake_dirty(lv: _Level, budget: int, bake_out: Array) -> int:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var n := _cells_for(lv.k, budget)
	var used := 0
	while used < n and not lv.dirty.is_empty():
		var box := L.box_clip(lv.dirty.pop_front(), window)
		if L.box_cells(box) == 0:
			continue
		var sp := L.box_split(box, n - used)
		_emit_bake(lv, sp[0], -1, bake_out)
		lv.prov.append({"from": sp[0]["from"], "to": sp[0]["to"], "emitted": false, "age_s": 0.0})
		lv.baked.append(sp[0])
		used += L.box_cells(sp[0])
		lv.dirty = sp[1] + lv.dirty
	if lv.dirty.is_empty():
		lv.ready = true
		lv.baked = []
	return used * L.cell_taps(lv.k)


func _bake_slot(lv: _Level, s: int, budget: int, bake_out: Array) -> int:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var list: Array = lv.slot_dirty[s]
	var n := _cells_for(lv.k, budget)
	var used := 0
	while used < n and not list.is_empty():
		var box := L.box_clip(list.pop_front(), window)
		if L.box_cells(box) == 0:
			continue
		var sp := L.box_split(box, n - used)
		_emit_bake(lv, sp[0], s, bake_out)
		used += L.box_cells(sp[0])
		list = sp[1] + list
	lv.slot_dirty[s] = list
	return used * L.cell_taps(lv.k)


# ---------------------------------------------------------------- light

func _light(order: Array, sun: Vector3, moon: Vector3, blend: float, cloud_top_m: float, delta_s: float,
		out: Dictionary) -> void:
	var L := CloudWorldVolumeLayout
	var fine_first := order.duplicate()
	fine_first.reverse()
	var built := false
	for lv: _Level in fine_first:
		if not lv.active:
			continue
		_light_env(lv, sun, moon, blend, cloud_top_m)
		for b: Dictionary in lv.prov:
			if b["emitted"]:
				b["age_s"] += delta_s
				_prov_age_max = maxf(_prov_age_max, b["age_s"])
		if lv.need_queue and not built:
			_enqueue_tiles(lv)  # one level per plan: the sort is the main thread's cost
			built = true
	var light_ms := budget_scale * LIGHT_BUDGET_MS
	if _spike:
		light_ms = maxf(budget_scale * UPKEEP_SPIKE_MS - _bake_taps / bake_taps_per_ms, 0.0)
	var budget := int(light_ms * light_cells_per_ms)
	# 1. the freshly baked cells (provisional ones first in line), coarse to fine.
	for lv: _Level in order:
		if lv.active and not lv.prov.is_empty():
			budget -= _light_prov(lv, budget, out["light"])
	# 2. the refresh generations in equal shares, finest level first (nearest tiles first inside a level).
	var stale: Array = []
	for lv: _Level in fine_first:
		if lv.active and (not lv.queue.is_empty() or not lv.partial.is_empty()):
			stale.append(lv)
	for i in stale.size():
		if budget <= 0:
			break
		var share := (budget + stale.size() - i - 1) / (stale.size() - i)
		budget -= _light_queue(stale[i], share, out["light"])
	# 3. what got no real light yet gets provisional light now (once).
	for lv: _Level in order:
		if not lv.active or not lv.shown:
			continue
		var window := L.storage_box(lv.storage)
		for b: Dictionary in lv.prov:
			if b["emitted"]:
				continue
			var box := L.box_clip(b, window)
			if L.box_cells(box) > 0:
				out["provisional"].append({"level": lv.k, "from": box["from"], "to": box["to"],
						"from_column": lv.k == L.LEVELS - 1})
				_prov_cells += L.box_cells(box)
			b["emitted"] = true


func _light_job(lv: _Level, box: Dictionary) -> Dictionary:
	_light_cells += CloudWorldVolumeLayout.box_cells(box)
	return {"level": lv.k, "from": box["from"], "to": box["to"], "moon": lv.moon_on, "spare": lv == _spare}


func _light_prov(lv: _Level, budget: int, jobs: Array) -> int:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var used := 0
	while used < budget and not lv.prov.is_empty():
		var b: Dictionary = lv.prov.pop_front()
		var box := L.box_clip(b, window)
		if L.box_cells(box) == 0:
			continue
		var sp := L.box_split(box, budget - used)
		jobs.append(_light_job(lv, sp[0]))
		used += L.box_cells(sp[0])
		if b["emitted"]:
			_prov_age_max = maxf(_prov_age_max, b["age_s"])
			_prov_age_session_max = maxf(_prov_age_session_max, b["age_s"])
		var rest: Array = []
		for r: Dictionary in sp[1]:
			rest.append({"from": r["from"], "to": r["to"], "emitted": b["emitted"], "age_s": b["age_s"]})
		lv.prov = rest + lv.prov
	return used


func _light_queue(lv: _Level, budget: int, jobs: Array) -> int:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var used := 0
	while used < budget:
		if lv.partial.is_empty():
			if lv.queue.is_empty():
				break
			var key: Vector3i = lv.queue.pop_front()
			lv.queued.erase(key)
			var lo := key * TILE_CELLS
			var box := L.box_clip({"from": lo, "to": lo + Vector3i(TILE_CELLS, TILE_CELLS, TILE_CELLS)}, window)
			if L.box_cells(box) > 0:
				lv.partial = [box]
			continue
		var sp := L.box_split(lv.partial.pop_front(), budget - used)
		jobs.append(_light_job(lv, sp[0]))
		used += L.box_cells(sp[0])
		lv.partial = sp[1] + lv.partial
	return used


## The level's light environment and refresh triggers (a trigger starts a new generation: the whole window is
## queued again, behind what is still queued).
func _light_env(lv: _Level, sun: Vector3, moon: Vector3, blend: float, cloud_top_m: float) -> void:
	var L := CloudWorldVolumeLayout
	var cx := (lv.storage.x + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	var cy := (lv.storage.y + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	var up := lv.layout.up_of_grid(cx, cy)
	var sin_e := sun.dot(up)
	lv.moon_on = sin_e < sin(MOON_SUN_ELEVATION_RAD)
	if not lv.ready or not lv.lit_valid:
		# Filling: the light computed now already is the current one.
		lv.lit_valid = lv.ready or lv.lit_valid
		lv.lit_sun = sun
		lv.lit_moon = moon
		lv.lit_blend = blend
		lv.lit_moon_on = lv.moon_on
		lv.lit_valid = true
		return
	var h := maxf(cloud_top_m - (L.SHELL_BOTTOM_M + lv.storage.z * L.CELL_V_M[lv.k]), MIN_HEIGHT_M)
	var trigger := false
	if (sun - lv.lit_sun).length() > sun_threshold_rad(lv.k, _refresh_elevation(sin_e), h):
		trigger = true
	if lv.moon_on != lv.lit_moon_on:
		trigger = true
	elif lv.moon_on and (moon - lv.lit_moon).length() > sun_threshold_rad(lv.k, _refresh_elevation(moon.dot(up)), h):
		trigger = true
	if absf(blend - lv.lit_blend) >= WEATHER_REFRESH_STEP - 1e-9:
		trigger = true
	if trigger:
		lv.lit_sun = sun
		lv.lit_moon = moon
		lv.lit_blend = blend
		lv.lit_moon_on = lv.moon_on
		lv.need_queue = true


static func _refresh_elevation(sin_e: float) -> float:
	return maxf(absf(asin(clampf(sin_e, -1.0, 1.0))), MIN_REFRESH_ELEVATION_RAD)


func _enqueue_tiles(lv: _Level) -> void:
	var L := CloudWorldVolumeLayout
	lv.need_queue = false
	var o := lv.storage
	var t0 := Vector3i(floori(o.x / float(TILE_CELLS)), floori(o.y / float(TILE_CELLS)), floori(o.z / float(TILE_CELLS)))
	var t1 := Vector3i(floori((o.x + L.CELLS_H - 1) / float(TILE_CELLS)), floori((o.y + L.CELLS_H - 1) / float(TILE_CELLS)),
			floori((o.z + L.CELLS_V - 1) / float(TILE_CELLS)))
	var cg := _grid_of_level(lv)
	var ch: float = L.CELL_H_M[lv.k]
	var cv: float = L.CELL_V_M[lv.k]
	var view := lv.layout.tangent_of(view_dir_earth)
	var use_view := view.length_squared() > 0.0
	if use_view:
		view = view.normalized()
	var tiles: Array[Vector3i] = []
	var keys := PackedInt64Array()
	var half := 0.5 * TILE_CELLS
	for tz in range(t0.z, t1.z + 1):
		for ty in range(t0.y, t1.y + 1):
			for tx in range(t0.x, t1.x + 1):
				var key := Vector3i(tx, ty, tz)
				if lv.queued.has(key):
					continue
				var dx := (tx * TILE_CELLS + half) * ch - cg.x
				var dy := (ty * TILE_CELLS + half) * ch - cg.y
				var dz := L.SHELL_BOTTOM_M + (tz * TILE_CELLS + half) * cv - cg.z
				var score := int(sqrt(dx * dx + dy * dy + dz * dz) / ch * 8.0)
				if use_view and dx * view.x + dy * view.y < 0.0:
					score *= BEHIND_FACTOR
				keys.append((score << _KEY_BITS) | tiles.size())
				tiles.append(key)
	keys.sort()
	for key in keys:
		var tile := tiles[key & ((1 << _KEY_BITS) - 1)]
		lv.queue.append(tile)
		lv.queued[tile] = true


# ---------------------------------------------------------------- re-frame

func _reframe_begin(camera_earth_m: Vector3) -> void:
	if _next == null and _layout.reframe_needed(camera_earth_m):
		_next = CloudWorldVolumeLayout.new()
		_next.set_frame(camera_earth_m)
		_todo.clear()
		for k in range(CloudWorldVolumeLayout.LEVELS - 1, -1, -1):
			_todo.append(k)
		_spike = true
	if _next != null and _spare == null and not _todo.is_empty():
		_spare = _Level.new()
		_spare.k = _todo.pop_front()
		_spare.layout = _next
		_spare.shown = false
		_fade_s = 0.0


## The spare version: shown once built and lit, faded in over REFRAME_FADE_S, then swapped in; returns the plan's
## `reframe` entry {level, spare, fade}.
func _reframe_end(delta_s: float) -> Dictionary:
	var rf := {"level": -1, "spare": false, "fade": 0.0}
	if _spare != null:
		var sp := _spare
		if not sp.active and sp.layout.level_active(sp.k, _camera_grid_next.z) == false:
			_swap_spare()  # a level that is off needs no spare
			rf = {"level": sp.k, "spare": false, "fade": 1.0}
		elif sp.shown:
			_fade_s += delta_s
			if _fade_s >= REFRAME_FADE_S:
				_swap_spare()
				rf = {"level": sp.k, "spare": false, "fade": 1.0}
			else:
				rf = {"level": sp.k, "spare": true, "fade": _fade_s / REFRAME_FADE_S}
		else:
			if sp.active and sp.ready and sp.prov.is_empty() and sp.queue.is_empty() and sp.partial.is_empty() \
					and not sp.need_queue:
				sp.shown = true
				_fade_s = 0.0
			rf = {"level": sp.k, "spare": true, "fade": 0.0}
	if _next != null and _spare == null and _todo.is_empty():
		_layout = _next
		_next = null
	return rf


func _swap_spare() -> void:
	_levels[_spare.k] = _spare
	_spare.shown = true
	_spare = null


# ---------------------------------------------------------------- lookup

func _lookup() -> Dictionary:
	var L := CloudWorldVolumeLayout
	var levels: Array = []
	for lv: _Level in _levels:
		levels.append({"level": lv.k, "origin": lv.origin, "storage": lv.storage, "active": lv.active,
				"ready": lv.ready, "shown": lv.drawn,
				"frame_origin_earth_m": lv.layout.origin_earth_m, "vertical_fade": lv.k < L.WINDOWED_LEVELS,
				"clamped_bottom": lv.origin.z == 0})
	var lookup := {"levels": levels, "slot_ready": _slot_ready.duplicate(), "slot_ids": _slot_ids.duplicate()}
	if _spare != null:  # the spare being built (its jobs' frame), shown or not
		lookup["build_spare"] = {"level": _spare.k, "origin": _spare.origin, "storage": _spare.storage,
				"frame_origin_earth_m": _spare.layout.origin_earth_m}
	if _spare != null and _spare.shown:
		lookup["spare"] = {"level": _spare.k, "origin": _spare.origin, "storage": _spare.storage,
				"frame_origin_earth_m": _spare.layout.origin_earth_m, "fade": minf(_fade_s / REFRAME_FADE_S, 1.0),
				"vertical_fade": _spare.k < L.WINDOWED_LEVELS, "clamped_bottom": _spare.origin.z == 0}
	return lookup
