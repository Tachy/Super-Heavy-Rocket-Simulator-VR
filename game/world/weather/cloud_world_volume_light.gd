class_name CloudWorldVolumeLight
extends RefCounted
## The cloud world volume's light schedule (world-volume plan Tasks 3 / 6; pure logic, split out of
## CloudWorldVolumeSchedule, which owns the levels and calls run() once per plan after the bake and the windows' advance;
## the GPU side is CloudWorldVolume / shaders/cloud_volume_light.gdshaderinc).
##
## A level is lit only once it is filled (`ready`: its rays read its own window, then the coarser SHOWN levels; a level
## still filling would light its cells from the coarser levels alone). Per plan, in this order and under the light
## budget (LIGHT_BUDGET_MS, in spike frames what the bake left of UPKEEP_SPIKE_MS):
## 0. blends in progress: a box that was SHOWN with provisional light gets its real light blended in over
##    PROVISIONAL_FADE_S (the light kernel recomputed each plan, exp(-tau) mixed by dt / the time left: linear in time);
## 1. freshly baked cells (the `prov` list, coarse to fine): written directly, except a box shown with provisional light
##    outside a spike (cold start, jump, re-frame), which starts a blend;
## 2. refresh generations in equal shares, finest level first, nearest tiles first: rate-limited (exp(-tau) of every
##    channel moves by at most LIGHT_STEP_T per pass: lag, never a front). The GPU counts the cells each job clamped
##    (light_feedback): a box that clamped any is served again once the level's queue is through ("settle"), until it
##    clamps none;
## 3. what got no real light yet gets provisional light once (copied from the next coarser level, the coarsest level's
##    from the grid-mean column), written directly.
## Triggers per ready level (a new generation: its storage queued as tiles behind what is still queued): the Sun's turn
## since the level's light (shadow shift H dtheta / sin^2 e < 1/4 cell), the Moon's (only with the Sun below
## MOON_SUN_ELEVATION_RAD at the level centre: then jobs carry `moon`), the Moon switching, the blend's step. The
## level's light reference (Sun, Moon, blend) is the one of its first real light after a fill (the oldest its cells hold).

## Write modes (shaders/cloud_volume_light.gdshaderinc CLOUD_VOLUME_LIGHT_*; change together).
const MODE_DIRECT := 0
const MODE_RATE := 1
const MODE_BLEND := 2
## Light jobs not of a refresh generation (no settle feedback).
const NO_GENERATION := -1

## This plan's work (CloudWorldVolumeSchedule.plan copies them into its batch).
var light_cells := 0
var moon_cells := 0
var prov_cells := 0
var prov_age_max := 0.0
var prov_age_session_max := 0.0

var _sun := Vector3.ZERO
var _moon := Vector3.ZERO
var _blend := 0.0


## Shadow shift H dtheta / sin^2 e kept below 1/4 cell: the Sun may turn this far (rad) before a level is relit.
## H = the cloud top above the level's lowest cell (m).
static func sun_threshold_rad(k: int, sun_elevation_rad: float, height_above_m: float) -> float:
	var s := sin(sun_elevation_rad)
	return 0.25 * CloudWorldVolumeLayout.CELL_H_M[k] * s * s / height_above_m


## True while level k's light is behind its target (a refresh generation, a settle pass, provisional light or a blend
## is pending).
static func stale(lv: CloudWorldVolumeSchedule._Level) -> bool:
	return lv.need_queue or not lv.settle.is_empty() or not lv.queue.is_empty() or not lv.partial.is_empty() \
			or not lv.prov.is_empty() or not lv.blend.is_empty()


## True while cells of the level have no real light yet or are still blending it in.
static func pending(lv: CloudWorldVolumeSchedule._Level) -> bool:
	return not lv.prov.is_empty() or not lv.blend.is_empty()


## A rate-limited job of generation `generation` clamped cells of `box` (CloudWorldVolume.take_light_feedback): the box
## is served again once the level's queue is through (an older generation's box: the newer one recomputes it anyway).
static func feedback(lv: CloudWorldVolumeSchedule._Level, generation: int, box: Dictionary) -> void:
	if generation == lv.generation:
		lv.settle.append({"from": box["from"], "to": box["to"]})


func run(s: CloudWorldVolumeSchedule, order: Array, sun: Vector3, moon: Vector3, blend: float, cloud_top_m: float,
		delta_s: float, out: Dictionary) -> void:
	var L := CloudWorldVolumeLayout
	var S := CloudWorldVolumeSchedule
	light_cells = 0
	moon_cells = 0
	prov_cells = 0
	prov_age_max = 0.0
	_sun = sun
	_moon = moon
	_blend = blend
	var spike: bool = s.spike_frame()
	var fine_first := order.duplicate()
	fine_first.reverse()
	var built := false
	for lv: CloudWorldVolumeSchedule._Level in fine_first:
		if not lv.active:
			continue
		_env(lv, sun, moon, blend, cloud_top_m)
		for b: Dictionary in lv.prov:
			if b["emitted"]:
				b["age_s"] += delta_s
				prov_age_max = maxf(prov_age_max, b["age_s"])
		if lv.need_queue and not built:
			_enqueue_tiles(s, lv)  # one level per plan: the sort is the main thread's cost
			built = true
	var light_ms: float = s.budget_scale * S.LIGHT_BUDGET_MS
	if spike:
		light_ms = maxf(s.budget_scale * S.UPKEEP_SPIKE_MS - s.bake_ms_planned(), 0.0)
	var budget := int(light_ms * s.light_budget_scale * s.light_cells_per_ms)
	var jobs: Array = out["light"]
	# 0. blends in progress.
	for lv: CloudWorldVolumeSchedule._Level in order:
		if lv.active and not lv.blend.is_empty():
			budget -= _light_blend(s, lv, budget, delta_s, jobs)
	# 1. the freshly baked cells of filled levels (provisional ones first in line), coarse to fine.
	for lv: CloudWorldVolumeSchedule._Level in order:
		if lv.active and lv.ready and not lv.prov.is_empty():
			budget -= _light_prov(s, lv, budget, spike, delta_s, jobs)
	# 2. the refresh generations in equal shares, finest level first (nearest tiles first inside a level).
	var stale_levels: Array = []
	for lv: CloudWorldVolumeSchedule._Level in fine_first:
		if lv.active and lv.ready and (not lv.queue.is_empty() or not lv.partial.is_empty()):
			stale_levels.append(lv)
	for i in stale_levels.size():
		if budget <= 0:
			break
		var share := (budget + stale_levels.size() - i - 1) / (stale_levels.size() - i)
		budget -= _light_queue(s, stale_levels[i], share, jobs)
	# 3. what got no real light yet gets provisional light now (once); boxes left with provisional light on a shown
	# level are marked: their real light will be blended in.
	for lv: CloudWorldVolumeSchedule._Level in order:
		if not lv.active or not lv.shown:
			continue
		var window := L.storage_box(lv.storage)
		for b: Dictionary in lv.prov:
			if b["emitted"]:
				b["seen"] = b["seen"] or lv.drawn
				continue
			var box := L.box_clip(b, window)
			if L.box_cells(box) > 0:
				out["provisional"].append({"level": lv.k, "from": box["from"], "to": box["to"],
						"from_column": lv.k == L.LEVELS - 1, "moon": lv.moon_on, "spare": s.is_spare(lv)})
				prov_cells += L.box_cells(box)
			b["emitted"] = true
			b["seen"] = lv.drawn


func _job(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, box: Dictionary, mode: int,
		weight: float, generation: int) -> Dictionary:
	var cells := CloudWorldVolumeLayout.box_cells(box)
	light_cells += cells
	if lv.moon_on:
		moon_cells += cells
	if not lv.lit_valid:  # the level's light reference: its first real light after a fill
		lv.lit_valid = true
		lv.lit_sun = _sun
		lv.lit_moon = _moon
		lv.lit_blend = _blend
		lv.lit_moon_on = lv.moon_on
	return {"level": lv.k, "from": box["from"], "to": box["to"], "moon": lv.moon_on, "spare": s.is_spare(lv),
		"mode": mode, "weight": weight, "generation": generation}


## A blend step for every box in the level's blend list the budget covers: weight dt / the time left, so exp(-tau) moves
## linearly from the provisional to the real light over PROVISIONAL_FADE_S; a box not served waits (its time too).
func _light_blend(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, budget: int, delta_s: float,
		jobs: Array) -> int:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var used := 0
	var kept: Array = []
	for b: Dictionary in lv.blend:
		var box := L.box_clip(b, window)
		if L.box_cells(box) == 0:
			continue
		if used >= budget:
			kept.append(b)
			continue
		var sp := L.box_split(box, budget - used)
		var weight := _blend_weight(b["t_s"], delta_s)
		jobs.append(_job(s, lv, sp[0], MODE_BLEND, weight, NO_GENERATION))
		used += L.box_cells(sp[0])
		if weight < 1.0:
			kept.append({"from": sp[0]["from"], "to": sp[0]["to"], "t_s": b["t_s"] + delta_s})
		for r: Dictionary in sp[1]:
			kept.append({"from": r["from"], "to": r["to"], "t_s": b["t_s"]})
	lv.blend = kept
	return used


## The blend's weight for a box t_s into its fade: the rest of the way in dt / (PROVISIONAL_FADE_S - t_s).
static func _blend_weight(t_s: float, delta_s: float) -> float:
	var left := CloudWorldVolumeSchedule.PROVISIONAL_FADE_S - t_s
	return 1.0 if left <= delta_s else clampf(delta_s / left, 0.0, 1.0)


func _light_prov(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, budget: int, spike: bool,
		delta_s: float, jobs: Array) -> int:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var used := 0
	while used < budget and not lv.prov.is_empty():
		var b: Dictionary = lv.prov.pop_front()
		var box := L.box_clip(b, window)
		if L.box_cells(box) == 0:
			continue
		var sp := L.box_split(box, budget - used)
		if b["emitted"] and b["seen"] and not spike:
			var weight := _blend_weight(0.0, delta_s)
			jobs.append(_job(s, lv, sp[0], MODE_BLEND, weight, NO_GENERATION))
			if weight < 1.0:
				lv.blend.append({"from": sp[0]["from"], "to": sp[0]["to"], "t_s": delta_s})
		else:
			jobs.append(_job(s, lv, sp[0], MODE_DIRECT, 1.0, NO_GENERATION))
		used += L.box_cells(sp[0])
		if b["emitted"]:
			prov_age_max = maxf(prov_age_max, b["age_s"])
			prov_age_session_max = maxf(prov_age_session_max, b["age_s"])
		var rest: Array = []
		for r: Dictionary in sp[1]:
			rest.append({"from": r["from"], "to": r["to"], "emitted": b["emitted"], "age_s": b["age_s"],
					"seen": b["seen"]})
		lv.prov = rest + lv.prov
	return used


func _light_queue(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, budget: int, jobs: Array) -> int:
	var L := CloudWorldVolumeLayout
	var T := CloudWorldVolumeSchedule.TILE_CELLS
	var window := L.storage_box(lv.storage)
	var used := 0
	while used < budget:
		if lv.partial.is_empty():
			if lv.queue.is_empty():
				break
			var key: Vector3i = lv.queue.pop_front()
			lv.queued.erase(key)
			var lo := key * T
			var box := L.box_clip({"from": lo, "to": lo + Vector3i(T, T, T)}, window)
			if L.box_cells(box) > 0:
				lv.partial = [box]
			continue
		var sp := L.box_split(lv.partial.pop_front(), budget - used)
		jobs.append(_job(s, lv, sp[0], MODE_RATE, 1.0, lv.generation))
		used += L.box_cells(sp[0])
		lv.partial = sp[1] + lv.partial
	return used


## The level's light environment and refresh triggers (a trigger starts a new generation: the whole storage is queued
## again, behind what is still queued); boxes whose rate-limited write clamped cells are served again once the queue is
## through (settle). Nothing before the level's first real light (its reference is set then: _job).
func _env(lv: CloudWorldVolumeSchedule._Level, sun: Vector3, moon: Vector3, blend: float, cloud_top_m: float) -> void:
	var L := CloudWorldVolumeLayout
	var S := CloudWorldVolumeSchedule
	var cx := (lv.storage.x + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	var cy := (lv.storage.y + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	var up := lv.layout.up_of_grid(cx, cy)
	var sin_e := sun.dot(up)
	lv.moon_on = sin_e < sin(S.MOON_SUN_ELEVATION_RAD)
	if not lv.ready or not lv.lit_valid:
		return
	var h := maxf(cloud_top_m - (L.SHELL_BOTTOM_M + lv.storage.z * L.CELL_V_M[lv.k]), S.MIN_HEIGHT_M)
	var trigger := false
	if (sun - lv.lit_sun).length() > sun_threshold_rad(lv.k, _refresh_elevation(sin_e), h):
		trigger = true
	if lv.moon_on != lv.lit_moon_on:
		trigger = true
	elif lv.moon_on and (moon - lv.lit_moon).length() > sun_threshold_rad(lv.k, _refresh_elevation(moon.dot(up)), h):
		trigger = true
	if absf(blend - lv.lit_blend) >= S.WEATHER_REFRESH_STEP - 1e-9:
		trigger = true
	if trigger:
		lv.lit_sun = sun
		lv.lit_moon = moon
		lv.lit_blend = blend
		lv.lit_moon_on = lv.moon_on
		lv.need_queue = true
		lv.settle = []  # the new generation recomputes every cell anyway
	elif not lv.settle.is_empty() and not lv.need_queue and lv.queue.is_empty() and lv.partial.is_empty():
		lv.partial = lv.settle
		lv.settle = []


static func _refresh_elevation(sin_e: float) -> float:
	return maxf(absf(asin(clampf(sin_e, -1.0, 1.0))), CloudWorldVolumeSchedule.MIN_REFRESH_ELEVATION_RAD)


func _enqueue_tiles(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level) -> void:
	var L := CloudWorldVolumeLayout
	var S := CloudWorldVolumeSchedule
	var T := S.TILE_CELLS
	lv.need_queue = false
	lv.generation += 1
	var o := lv.storage
	var t0 := Vector3i(floori(o.x / float(T)), floori(o.y / float(T)), floori(o.z / float(T)))
	var t1 := Vector3i(floori((o.x + L.CELLS_H - 1) / float(T)), floori((o.y + L.CELLS_H - 1) / float(T)),
			floori((o.z + L.CELLS_V - 1) / float(T)))
	var cg: Vector3 = s.camera_grid_of(lv)
	var ch: float = L.CELL_H_M[lv.k]
	var cv: float = L.CELL_V_M[lv.k]
	var view := lv.layout.tangent_of(s.view_dir_earth)
	var use_view := view.length_squared() > 0.0
	if use_view:
		view = view.normalized()
	var tiles: Array[Vector3i] = []
	var keys := PackedInt64Array()
	var half := 0.5 * T
	for tz in range(t0.z, t1.z + 1):
		for ty in range(t0.y, t1.y + 1):
			for tx in range(t0.x, t1.x + 1):
				var key := Vector3i(tx, ty, tz)
				if lv.queued.has(key):
					continue
				var dx := (tx * T + half) * ch - cg.x
				var dy := (ty * T + half) * ch - cg.y
				var dz := L.SHELL_BOTTOM_M + (tz * T + half) * cv - cg.z
				var score := int(sqrt(dx * dx + dy * dy + dz * dz) / ch * 8.0)
				if use_view and dx * view.x + dy * view.y < 0.0:
					score *= S.BEHIND_FACTOR
				keys.append((score << S.KEY_BITS) | tiles.size())
				tiles.append(key)
	keys.sort()
	for key in keys:
		var tile := tiles[key & ((1 << S.KEY_BITS) - 1)]
		lv.queue.append(tile)
		lv.queued[tile] = true
