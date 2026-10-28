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
## Scroll: a level whose window moved bakes the exposed slabs in the plan that adopts the new origin (never late, not
## budgeted) and lights them first (priority 1: they have no light yet); whatever of them the light budget cannot cover
## gets provisional light and stays first in line (PROVISIONAL_MAX_S is the gate's bound on how long that may last).
## Fill (cold start, jump, level switched on, whole-window moves): the window is "dirty" and baked under the budget, coarse
## to fine; only the coarsest level is baked whole at once (it is always complete). A level is `ready` (shown) once
## its dirty list is empty. Pair switch: a snapshot replaced in slot s is rebaked into slot s's channel pair alone,
## under the bake budget, coarse to fine; `slot_ready[s]` tells when it may take weight.
## Light refresh: per level the Sun's turn (shadow shift H dtheta / sin^2 e < 1/4 cell), the Moon's (only while the Sun is
## below MOON_SUN_ELEVATION_RAD at the level centre), the blend's step (WEATHER_REFRESH_STEP) start a new generation:
## the level's window is queued as tiles, nearest (and in view) first, behind what an older generation left in the
## queue (no tile starves under time warp), served in equal shares finest level first.
## Re-frame: the camera's ground point > REFRAME_M from the frame origin -> a new frame at the camera; levels are
## rebuilt one at a time, coarsest first, into a spare version in the new frame, shown (the old version stays shown
## meanwhile), cross-faded over REFRAME_FADE_S and swapped.
## Budget: UPKEEP_BUDGET_MS per frame (LIGHT_BUDGET_MS of it light); UPKEEP_SPIKE_MS in cold-start, jump and re-frame
## frames until the fill is complete (half bake, half light). The first plan's bake of the whole coarsest level is not
## budgeted (the frame must show clouds) - its cost is the GPU's to measure.

const UPKEEP_BUDGET_MS := 1.5
const LIGHT_BUDGET_MS := 1.0
const UPKEEP_SPIKE_MS := 3.0
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
## [E] Start rates, replaced by feedback(): the spec's 67 M light cells in 60 ms; baking about as fast.
const INITIAL_LIGHT_CELLS_PER_MS := 67.0e6 / 60.0
const INITIAL_BAKE_CELLS_PER_MS := 1.0e6
## [E] feedback() moves the rates by this fraction of the measured error per call, at most by the limits.
const CALIBRATION_GAIN := 0.2
const CALIBRATION_LIMITS := Vector2(0.5, 2.0)
const MIN_CELLS_PER_MS := 1.0e4
## Cells of a tile's key order inside the sort key (tiles per level <= 4096).
const _KEY_BITS := 12

## Calibrated by feedback().
var bake_cells_per_ms := INITIAL_BAKE_CELLS_PER_MS
var light_cells_per_ms := INITIAL_LIGHT_CELLS_PER_MS
## Optional: the camera's view direction (Earth-local); tiles behind it are refreshed later. ZERO = ignore.
var view_dir_earth := Vector3.ZERO


class _Level extends RefCounted:
	var k := 0
	var layout: CloudWorldVolumeLayout
	var shown := true  # false: a re-frame's spare while it is built
	var active := false
	var ready := false
	var origin := Vector3i.ZERO
	var dirty: Array = []  # boxes {from, to} still to bake (both snapshots)
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
var _mandatory_cells := 0
var _light_cells := 0
var _prov_cells := 0
var _prov_age_max := 0.0
var _prov_age_session_max := 0.0
var _camera_grid := Vector3.ZERO
var _camera_grid_next := Vector3.ZERO


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


## Cells-per-ms calibration from GPU timestamps: the measured upkeep time of a frame against the model's.
func feedback(upkeep_ms: float, bake_cells: int, light_cells: int) -> void:
	if upkeep_ms <= 0.0:
		return
	var model := bake_cells / bake_cells_per_ms + light_cells / light_cells_per_ms
	if model <= 0.0:
		return
	var f := clampf(model / upkeep_ms, CALIBRATION_LIMITS.x, CALIBRATION_LIMITS.y)
	var g := 1.0 + CALIBRATION_GAIN * (f - 1.0)
	bake_cells_per_ms = maxf(bake_cells_per_ms * g, MIN_CELLS_PER_MS)
	light_cells_per_ms = maxf(light_cells_per_ms * g, MIN_CELLS_PER_MS)


## Cold-start / jump / re-frame fill and the spare version are done; nothing is waiting to be baked or lit first.
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
	_mandatory_cells = 0
	_light_cells = 0
	_prov_cells = 0
	_prov_age_max = 0.0
	if jumped or not _started:
		_cold_start(camera_earth_m, slot_ids)
	_apply_slot_ids(slot_ids)
	_reframe_begin(camera_earth_m)
	_camera_grid = _layout.grid_of(camera_earth_m)
	if _next != null:
		_camera_grid_next = _next.grid_of(camera_earth_m)
	var order := _levels.duplicate()
	order.reverse()  # coarse to fine
	if _spare != null:
		order.append(_spare)
	for lv: _Level in order:
		_scroll(lv, out["bake"])
	_bake_fill(order, out["bake"])
	_light(order, sun_dir_earth, moon_dir_earth, blend, cloud_top_m, delta_s, out)
	var reframe := _reframe_end(delta_s)
	if _spike and complete():
		_spike = false
	out["reframe"] = reframe
	out["lookup"] = _lookup()
	out["bake_cells"] = _bake_cells
	out["bake_mandatory_cells"] = _mandatory_cells
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


## A snapshot replaced in a slot: its channel pair is rebaked alone, in full, under the bake budget.
func _apply_slot_ids(slot_ids: Array) -> void:
	for s in 2:
		if slot_ids[s] == _slot_ids[s]:
			continue
		_slot_ids[s] = slot_ids[s]
		for lv: _Level in _all_levels():
			if lv.active:
				lv.slot_dirty[s] = [CloudWorldVolumeLayout.window_box(lv.origin)]


func _all_levels() -> Array:
	var all := _levels.duplicate()
	if _spare != null:
		all.append(_spare)
	return all


func _grid_of_level(lv: _Level) -> Vector3:
	return _camera_grid_next if lv.layout == _next else _camera_grid


func _scroll(lv: _Level, bake_out: Array) -> void:
	var L := CloudWorldVolumeLayout
	var cg := _grid_of_level(lv)
	if not lv.layout.level_active(lv.k, cg.z):
		if lv.active:
			_clear(lv)
		return
	var new_origin := lv.layout.window_origin(lv.k, cg)
	if not lv.active:
		_clear(lv)
		lv.active = true
		lv.origin = new_origin
		lv.dirty = [L.window_box(new_origin)]
		return
	if new_origin == lv.origin:
		return
	var slabs := L.exposed_slabs(lv.k, lv.origin, new_origin)
	lv.origin = new_origin
	for slab in slabs:
		if not lv.ready or L.box_cells(slab) >= L.CELLS_H * L.CELLS_H * L.CELLS_V:
			if L.box_cells(slab) >= L.CELLS_H * L.CELLS_H * L.CELLS_V:  # a whole window: a fill, not a scroll
				_clear(lv)
				lv.active = true
				lv.origin = new_origin
				lv.dirty = [L.window_box(new_origin)]
				return
			lv.dirty.append(slab)
		else:
			_emit_bake(lv, slab, -1, bake_out)
			_mandatory_cells += L.box_cells(slab)
			lv.prov.append({"from": slab["from"], "to": slab["to"], "emitted": false, "age_s": 0.0})


func _clear(lv: _Level) -> void:
	lv.active = false
	lv.ready = false
	lv.dirty = []
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
	_bake_cells += CloudWorldVolumeLayout.box_cells(box)


# ---------------------------------------------------------------- bake

func _bake_ms() -> float:
	return UPKEEP_SPIKE_MS * 0.5 if _spike else UPKEEP_BUDGET_MS - LIGHT_BUDGET_MS


func _bake_fill(order: Array, bake_out: Array) -> void:
	var L := CloudWorldVolumeLayout
	# The coarsest shown level is always complete: baked whole at once, not budgeted.
	var coarsest: _Level = _levels[L.LEVELS - 1]
	if coarsest.active and not coarsest.ready:
		var n := _bake_dirty(coarsest, 1 << 60, bake_out)
		_mandatory_cells += n
	var budget := maxi(int((_bake_ms() - _mandatory_cells / bake_cells_per_ms) * bake_cells_per_ms), 0)
	for lv: _Level in order:
		if budget <= 0:
			break
		if lv.active and not lv.dirty.is_empty() and lv != _spare:
			budget -= _bake_dirty(lv, budget, bake_out)
	for lv: _Level in order:
		for s in 2:
			if budget <= 0:
				break
			if lv.active and not lv.slot_dirty[s].is_empty():
				budget -= _bake_slot(lv, s, budget, bake_out)
	if _spare != null and _spare.active and not _spare.dirty.is_empty() and budget > 0:
		_bake_dirty(_spare, budget, bake_out)
	for s in 2:
		var ready := true
		for lv: _Level in _levels:
			ready = ready and lv.slot_dirty[s].is_empty()
		_slot_ready[s] = ready


## Bakes up to `budget` cells of the level's dirty boxes (clipped to its window) and hands them to the light
## (prov list); returns the cells baked. The level is ready once nothing is dirty.
func _bake_dirty(lv: _Level, budget: int, bake_out: Array) -> int:
	var L := CloudWorldVolumeLayout
	var window := L.window_box(lv.origin)
	var used := 0
	while used < budget and not lv.dirty.is_empty():
		var box := L.box_clip(lv.dirty.pop_front(), window)
		if L.box_cells(box) == 0:
			continue
		var sp := L.box_split(box, budget - used)
		_emit_bake(lv, sp[0], -1, bake_out)
		lv.prov.append({"from": sp[0]["from"], "to": sp[0]["to"], "emitted": false, "age_s": 0.0})
		used += L.box_cells(sp[0])
		lv.dirty = sp[1] + lv.dirty
	if lv.dirty.is_empty():
		lv.ready = true
	return used


func _bake_slot(lv: _Level, s: int, budget: int, bake_out: Array) -> int:
	var L := CloudWorldVolumeLayout
	var window := L.window_box(lv.origin)
	var list: Array = lv.slot_dirty[s]
	var used := 0
	while used < budget and not list.is_empty():
		var box := L.box_clip(list.pop_front(), window)
		if L.box_cells(box) == 0:
			continue
		var sp := L.box_split(box, budget - used)
		_emit_bake(lv, sp[0], s, bake_out)
		used += L.box_cells(sp[0])
		list = sp[1] + list
	lv.slot_dirty[s] = list
	return used


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
	var light_ms := UPKEEP_BUDGET_MS - (UPKEEP_BUDGET_MS - LIGHT_BUDGET_MS)
	if _spike:
		light_ms = maxf(UPKEEP_SPIKE_MS - _bake_cells / bake_cells_per_ms, 0.0)
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
		var window := L.window_box(lv.origin)
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
	var window := L.window_box(lv.origin)
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
	var window := L.window_box(lv.origin)
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
	var cx := (lv.origin.x + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	var cy := (lv.origin.y + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
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
	var h := maxf(cloud_top_m - (L.SHELL_BOTTOM_M + lv.origin.z * L.CELL_V_M[lv.k]), MIN_HEIGHT_M)
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
	var o := lv.origin
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
		levels.append({"level": lv.k, "origin": lv.origin, "active": lv.active, "ready": lv.ready,
				"frame_origin_earth_m": lv.layout.origin_earth_m, "vertical_fade": lv.k < L.WINDOWED_LEVELS,
				"clamped_bottom": lv.origin.z == 0})
	var lookup := {"levels": levels, "slot_ready": _slot_ready.duplicate(), "slot_ids": _slot_ids.duplicate()}
	if _spare != null and _spare.shown:
		lookup["spare"] = {"level": _spare.k, "origin": _spare.origin, "frame_origin_earth_m": _spare.layout.origin_earth_m,
				"fade": minf(_fade_s / REFRAME_FADE_S, 1.0), "vertical_fade": _spare.k < L.WINDOWED_LEVELS,
				"clamped_bottom": _spare.origin.z == 0}
	return lookup
