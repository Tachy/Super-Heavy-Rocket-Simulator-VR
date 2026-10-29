class_name CloudWorldVolumeLight
extends RefCounted
## The cloud world volume's light schedule (world-volume plan Tasks 3 / 6; pure logic, split out of
## CloudWorldVolumeSchedule, which owns the levels and calls run() once per plan after the bake and the windows' advance;
## the GPU side is CloudWorldVolume / shaders/cloud_volume_light.gdshaderinc).
##
## The light is a distance-stepped march through baked density (Task 6 review round 1): a level's rays march a fixed
## number of cells of their own level, then as many of each coarser level - no level reads another's light, so the
## levels are lit independently; a level whose coarser level turned drawn (its rays held other density before: its
## `light_epoch`) is relit once. Its cost per cell is a constant per level: the budget is in ms, a job's cost its cells
## and its column cells at its level's calibrated rates (CloudWorldVolumeSchedule.light_ms_of) plus, once per level and
## plan, the level's dispatch latency (light_latency_of: a level whose latency does not fit what is left of the budget
## is skipped and a level that fits is served); jobs are column-shaped (whole columns of their box: the column pass sums
## a level's whole column once per job).
##
## A level is lit only once it is filled (`ready`: its rays read its own window, then the coarser SHOWN levels; a level
## still filling would light its cells from the coarser levels alone). Per plan, in this order and under the light
## budget (LIGHT_BUDGET_MS, in spike frames LIGHT_SPIKE_SHARE of what the bake left of UPKEEP_SPIKE_MS with
## BAKE_SPIKE_MARGIN; halved after an overrun):
## 0. blends in progress: a box that was SHOWN with provisional light gets its real light blended in over
##    PROVISIONAL_FADE_S (the light kernel recomputed each plan, exp(-tau) mixed by dt / the time left: linear in time);
## 1. freshly baked cells (the `prov` list, coarse to fine): written directly, except a box shown with provisional light
##    outside a spike (cold start, jump, re-frame), which starts a blend;
## 2. refresh generations one level after the other, fine to coarse, nearest tiles (whole columns) first: rate-limited
##    (exp(-tau) of every channel moves by at most LIGHT_STEP_T per pass: lag, never a front; into or out of "no light"
##    - Earth's shadow, the Moon off - directly, the shader's header). The GPU counts the cells each job clamped
##    (light_feedback): a box that clamped any is served again once the level's queue is through ("settle"), until it
##    clamps none;
## 3. what got no real light yet gets provisional light once (copied from the next coarser level, the coarsest level's
##    from the grid-mean column), written directly.
## Triggers per ready level (a new generation: its storage queued as tiles behind what is still queued): the Sun's turn
## since the level's light (shadow shift H dtheta / sin^2 e < 1/4 cell), the Moon's (only with the Sun below
## MOON_SUN_ELEVATION_RAD at the level centre: then jobs carry `moon`), the Moon switching, the blend's step, a coarser
## level turning drawn. The level's light reference (Sun, Moon, blend, coarser epochs) is the one of its first real
## light after a fill (the oldest its cells hold).

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
## This plan's light GPU time at the calibrated rates (ms).
var ms_planned := 0.0

var _sun := Vector3.ZERO
var _moon := Vector3.ZERO
var _blend := 0.0
var _charged := {}  ## levels whose light lists' latency this plan's budget paid for (light_latency_of)
var _left := {}  ## level -> light cells the level may still take this plan (light_cells_cap; absent: uncapped)
var _share_ms := 0.0  ## this plan's whole light budget (a partial column only where a whole one never fits it)
var _hard_left_ms := 0.0  ## what UPKEEP_SPIKE_MS leaves this plan after the bake and the light so far (a probe's bound)


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


## The sum of the coarser levels' light epochs (each counts the times its level turned drawn): a change means the
## level's rays now hold other density beyond their own segment.
static func coarser_epoch(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level) -> int:
	var sum := 0
	for j in range(lv.k + 1, mini(CloudWorldVolumeLayout.LEVELS, s._levels.size())):
		sum += (s._levels[j] as CloudWorldVolumeSchedule._Level).light_epoch
	return sum


func run(s: CloudWorldVolumeSchedule, order: Array, sun: Vector3, moon: Vector3, blend: float, cloud_top_m: float,
		delta_s: float, out: Dictionary) -> void:
	var L := CloudWorldVolumeLayout
	var S := CloudWorldVolumeSchedule
	light_cells = 0
	moon_cells = 0
	prov_cells = 0
	prov_age_max = 0.0
	_charged = {}
	_left = {}
	for k in CloudWorldVolumeLayout.LEVELS:
		s.rate_age[k] += 1
	for k in CloudWorldVolumeLayout.LEVELS:
		if s.light_cells_cap(k) >= 0:
			_left[k] = s.light_cells_cap(k)
	_sun = sun
	_moon = moon
	_blend = blend
	var spike: bool = s.spike_frame()
	for lv: CloudWorldVolumeSchedule._Level in order:
		if lv.active:
			lv.moon_on = _moon_wanted(lv, sun)
	var built := false
	var fine_first := order.duplicate()
	fine_first.reverse()
	for lv: CloudWorldVolumeSchedule._Level in fine_first:  # the finest level's tiles queued first (the nearest light)
		if not lv.active:
			continue
		_env(lv, coarser_epoch(s, lv), sun, moon, blend, cloud_top_m)
		for b: Dictionary in lv.prov:
			if b["emitted"]:
				b["age_s"] += delta_s
				prov_age_max = maxf(prov_age_max, b["age_s"])
		if lv.need_queue and not built:
			_enqueue_tiles(s, lv)  # one level per plan: the sort is the main thread's cost
			built = true
	var light_ms: float = s.budget_scale * S.LIGHT_BUDGET_MS
	if spike:
		light_ms = S.LIGHT_SPIKE_SHARE * maxf(s.budget_scale * S.UPKEEP_SPIKE_MS - S.BAKE_SPIKE_MARGIN * s.bake_ms_planned(),
				0.0)
	light_ms *= s.light_budget_scale * s.overrun_factor()
	var left := light_ms
	_share_ms = light_ms
	_hard_left_ms = s.budget_scale * S.UPKEEP_SPIKE_MS * s.overrun_factor() - s.bake_ms_planned()
	var jobs: Array = out["light"]
	# -1. the level longest unmeasured (at least LIGHT_PROBE_FRAMES plans, its share cannot reach a large dispatch) gets
	# a probe (N1: a slow sample or a latency above the share must not freeze its plan for good); one level a plan.
	var probed: CloudWorldVolumeSchedule._Level = null
	for lv: CloudWorldVolumeSchedule._Level in fine_first:
		if lv.active and lv.ready and _probe_due(s, lv, left) 				and (probed == null or s.rate_age[lv.k] > s.rate_age[probed.k]):
			probed = lv
	if probed != null:
		left -= _probe(s, probed, spike, delta_s, jobs)
	# 0. blends in progress.
	for lv: CloudWorldVolumeSchedule._Level in order:
		if lv.active and not lv.blend.is_empty():
			left -= _light_blend(s, lv, left, delta_s, jobs)
	# 1. the freshly baked cells of filled levels (provisional ones first in line), coarse to fine.
	for lv: CloudWorldVolumeSchedule._Level in order:
		if lv.active and lv.ready and not lv.prov.is_empty():
			left -= _light_prov(s, lv, left, spike, delta_s, jobs)
	# 2. the refresh generations one level after the other, fine to coarse (the nearest light first; nearest tiles first
	# inside a level): a level's light dispatch costs at least the latency of its longest ray (~0.25 ms on cloudy content:
	# cloud_check section 18), so equal shares over many stale levels (a blend step makes all eight stale) paid that floor
	# eight times a frame.
	for lv: CloudWorldVolumeSchedule._Level in fine_first:
		if left <= 0.0:
			break
		if lv.active and lv.ready and (not lv.queue.is_empty() or not lv.partial.is_empty()):
			left -= _light_queue(s, lv, left, jobs)
	ms_planned = light_ms - left
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
	# The epochs: a level that turned drawn relights the finer ones (_env; M6).
	for lv: CloudWorldVolumeSchedule._Level in s._levels:
		if lv.drawn and not lv.was_drawn:
			lv.light_epoch += 1
		lv.was_drawn = lv.drawn


## True when the Sun is below MOON_SUN_ELEVATION_RAD at the level's centre.
static func _moon_wanted(lv: CloudWorldVolumeSchedule._Level, sun: Vector3) -> bool:
	var L := CloudWorldVolumeLayout
	var cx := (lv.storage.x + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	var cy := (lv.storage.y + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	return sun.dot(lv.layout.up_of_grid(cx, cy)) < sin(CloudWorldVolumeSchedule.MOON_SUN_ELEVATION_RAD)


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
		lv.lit_up_epoch = coarser_epoch(s, lv)
	return {"level": lv.k, "from": box["from"], "to": box["to"], "moon": lv.moon_on, "spare": s.is_spare(lv),
		"mode": mode, "weight": weight, "generation": generation}


## The column-shaped part of `box` a light budget of `ms` pays for at level k ([taken, rest]; taken empty when nothing
## fits) and its cost (ms; the level's dispatch latency the first time this plan); a level takes at most
## CloudWorldVolumeSchedule.light_cells_cap a plan (LIGHT_GROWTH). Where not one whole column fits, a partial column
## (the first z cells of its first column, LIGHT_PARTIAL_MIN_CELLS at least): a level is always served what is left.
func _split(s: CloudWorldVolumeSchedule, k: int, box: Dictionary, ms: float) -> Array:
	var over := 0.0 if _charged.has(k) else s.light_latency_of(k)
	var n := s.light_cells_for(k, box, ms - over)
	if _left.has(k):  # the model is exact only near the sizes it measured (LIGHT_GROWTH)
		n = mini(n, _left[k])
	var sp := CloudWorldVolumeLayout.box_split_columns(box, n)
	if CloudWorldVolumeLayout.box_cells(sp[0]) == 0:
		sp = _split_partial(s, k, box, ms - over)
	if CloudWorldVolumeLayout.box_cells(sp[0]) == 0:
		return [sp[0], sp[1], 0.0]
	_charged[k] = true
	if _left.has(k):
		_left[k] -= CloudWorldVolumeLayout.box_cells(sp[0])
	return [sp[0], sp[1], s.light_ms_of(k, sp[0]) + over]


## A partial column of `box` (z slice of its first column) that `ms` pays for: the column pass whole, the light kernel
## for the slice - only where a whole column (and the level's latency) never fits the plan's whole light budget (else a
## sliver of the budget would fragment the jobs). [taken, rest]; taken empty when fewer than LIGHT_PARTIAL_MIN_CELLS (or
## the box's cells) fit.
func _split_partial(s: CloudWorldVolumeSchedule, k: int, box: Dictionary, ms: float) -> Array:
	var d: Vector3i = box["to"] - box["from"]
	var none := [{"from": box["from"], "to": box["from"]}, [box]]
	if d.z <= 0 or d.x <= 0 or d.y <= 0:
		return none
	if s.light_latency_of(k) + s.light_ms_of(k, {"from": box["from"], "to": box["from"] + Vector3i(1, 1, d.z)}) 			<= _share_ms:
		return none
	var nz := int(floorf((ms - CloudWorldVolumeLayout.window_size(k).z / s.column_cells_per_ms[k])
			* s.light_cells_per_ms[k]))
	if _left.has(k):
		nz = mini(nz, _left[k])
	if nz < mini(CloudWorldVolumeSchedule.LIGHT_PARTIAL_MIN_CELLS, d.z):
		return none
	return CloudWorldVolumeLayout.box_split_column_z(box, mini(nz, d.z))


## True when level lv is due a probe (LIGHT_PROBE_FRAMES): it has light pending, its light kernel was not measured by a
## large dispatch for that many plans and `left` (what is left of the share) pays for fewer than
## LIGHT_CALIBRATION_MIN_CELLS cells at the level's rates and latency.
func _probe_due(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, left: float) -> bool:
	var S := CloudWorldVolumeSchedule
	var pending := not lv.prov.is_empty() or not lv.queue.is_empty() or not lv.partial.is_empty()
	if not pending or s.rate_age[lv.k] < S.LIGHT_PROBE_FRAMES:
		return false
	var column := {"from": Vector3i.ZERO, "to": Vector3i(S.TILE_CELLS, S.TILE_CELLS,
			CloudWorldVolumeLayout.window_size(lv.k).z)}
	var over := 0.0 if _charged.has(lv.k) else s.light_latency_of(lv.k)
	return s.light_cells_for(lv.k, column, left - over) < S.LIGHT_CALIBRATION_MIN_CELLS


## The probe: level lv is served LIGHT_CALIBRATION_MIN_CELLS cells (two columns more: whole columns) even beyond its
## share, bounded by what UPKEEP_SPIKE_MS leaves less BAKE_SPIKE_MARGIN (_hard_left_ms; a plan is measured up to that
## much over) - its measurement refreshes the level's rate and latency. Returns the ms spent.
func _probe(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, spike: bool, delta_s: float,
		jobs: Array) -> float:
	var S := CloudWorldVolumeSchedule
	var L := CloudWorldVolumeLayout
	var k := lv.k
	var cells := S.LIGHT_CALIBRATION_MIN_CELLS + 2 * L.CELLS_V
	var columns := ceili(float(cells) / L.window_size(k).z)
	var need := s.light_latency_of(k) + cells / s.light_cells_per_ms[k] + columns * L.window_size(k).z 			/ s.column_cells_per_ms[k]
	var budget := minf(need, _hard_left_ms / S.BAKE_SPIKE_MARGIN)
	if budget <= 0.0:
		return 0.0
	var capped := _left.has(k)
	if capped:
		_left.erase(k)  # a probe is the one dispatch that may exceed the growth bound
	var used := _light_prov(s, lv, budget, spike, delta_s, jobs) if not lv.prov.is_empty() 			else _light_queue(s, lv, budget, jobs)
	if capped:
		_left[k] = 0
	if used > 0.0:
		s.rate_age[k] = 0  # its measurement arrives a few frames late
	return used


## A blend step for every box in the level's blend list the budget covers: weight dt / the time left, so exp(-tau) moves
## linearly from the provisional to the real light over PROVISIONAL_FADE_S; a box not served waits (its time too).
## Returns the ms spent.
func _light_blend(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, budget_ms: float, delta_s: float,
		jobs: Array) -> float:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var used := 0.0
	var kept: Array = []
	for b: Dictionary in lv.blend:
		var box := L.box_clip(b, window)
		if L.box_cells(box) == 0:
			continue
		var sp := _split(s, lv.k, box, budget_ms - used)
		if L.box_cells(sp[0]) == 0:
			kept.append(b)
			continue
		var weight := _blend_weight(b["t_s"], delta_s)
		jobs.append(_job(s, lv, sp[0], MODE_BLEND, weight, NO_GENERATION))
		used += sp[2]
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


func _light_prov(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, budget_ms: float, spike: bool,
		delta_s: float, jobs: Array) -> float:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var used := 0.0
	while used < budget_ms and not lv.prov.is_empty():
		var b: Dictionary = lv.prov.pop_front()
		var box := L.box_clip(b, window)
		if L.box_cells(box) == 0:
			continue
		var sp := _split(s, lv.k, box, budget_ms - used)
		if L.box_cells(sp[0]) == 0:
			lv.prov.push_front(b)
			break
		if b["emitted"] and b["seen"] and not spike:
			var weight := _blend_weight(0.0, delta_s)
			jobs.append(_job(s, lv, sp[0], MODE_BLEND, weight, NO_GENERATION))
			if weight < 1.0:
				lv.blend.append({"from": sp[0]["from"], "to": sp[0]["to"], "t_s": delta_s})
		else:
			jobs.append(_job(s, lv, sp[0], MODE_DIRECT, 1.0, NO_GENERATION))
		used += sp[2]
		if b["emitted"]:
			prov_age_max = maxf(prov_age_max, b["age_s"])
			prov_age_session_max = maxf(prov_age_session_max, b["age_s"])
		var rest: Array = []
		for r: Dictionary in sp[1]:
			rest.append({"from": r["from"], "to": r["to"], "emitted": b["emitted"], "age_s": b["age_s"],
					"seen": b["seen"]})
		lv.prov = rest + lv.prov
	return used


func _light_queue(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level, budget_ms: float,
		jobs: Array) -> float:
	var L := CloudWorldVolumeLayout
	var T := CloudWorldVolumeSchedule.TILE_CELLS
	var window := L.storage_box(lv.storage)
	var used := 0.0
	while used < budget_ms:
		if lv.partial.is_empty():
			if lv.queue.is_empty():
				break
			var key: Vector3i = lv.queue.pop_front()
			lv.queued.erase(key)
			var lo := Vector3i(key.x * T, key.y * T, lv.storage.z)  # a tile: whole columns of the storage
			var box := L.box_clip({"from": lo, "to": lo + Vector3i(T, T, L.CELLS_V)}, window)
			if L.box_cells(box) > 0:
				lv.partial = [box]
			continue
		var sp := _split(s, lv.k, lv.partial[0], budget_ms - used)
		if L.box_cells(sp[0]) == 0:
			break
		lv.partial.pop_front()
		jobs.append(_job(s, lv, sp[0], MODE_RATE, 1.0, lv.generation))
		used += sp[2]
		lv.partial = sp[1] + lv.partial
	return used


## The level's light environment and refresh triggers (a trigger starts a new generation: the whole storage is queued
## again, behind what is still queued); boxes whose rate-limited write clamped cells are served again once the queue is
## through (settle). Nothing before the level's first real light (its reference is set then: _job). up_epoch: the
## coarser levels' epochs (coarser_epoch).
func _env(lv: CloudWorldVolumeSchedule._Level, up_epoch: int, sun: Vector3, moon: Vector3,
		blend: float, cloud_top_m: float) -> void:
	var L := CloudWorldVolumeLayout
	var S := CloudWorldVolumeSchedule
	if not lv.ready or not lv.lit_valid:
		return
	var cx := (lv.storage.x + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	var cy := (lv.storage.y + L.CELLS_H / 2) * L.CELL_H_M[lv.k]
	var level_up := lv.layout.up_of_grid(cx, cy)
	var sin_e := sun.dot(level_up)
	var h := maxf(cloud_top_m - (L.SHELL_BOTTOM_M + lv.storage.z * L.CELL_V_M[lv.k]), S.MIN_HEIGHT_M)
	var trigger := false
	if (sun - lv.lit_sun).length() > sun_threshold_rad(lv.k, _refresh_elevation(sin_e), h):
		trigger = true
	if lv.moon_on != lv.lit_moon_on:
		trigger = true
	elif lv.moon_on and (moon - lv.lit_moon).length() > sun_threshold_rad(lv.k, _refresh_elevation(moon.dot(level_up)),
			h):
		trigger = true
	if absf(blend - lv.lit_blend) >= S.WEATHER_REFRESH_STEP - 1e-9:
		trigger = true
	var idle := not lv.need_queue and lv.queue.is_empty() and lv.partial.is_empty()
	if up_epoch != lv.lit_up_epoch and idle:
		trigger = true  # a coarser level turned drawn: the rays' part beyond the level reads it now (M6)
	if trigger:
		lv.lit_sun = sun
		lv.lit_moon = moon
		lv.lit_blend = blend
		lv.lit_moon_on = lv.moon_on
		lv.lit_up_epoch = up_epoch
		lv.need_queue = true
		lv.settle = []  # the new generation recomputes every cell anyway
	elif not lv.settle.is_empty() and not lv.need_queue and lv.queue.is_empty() and lv.partial.is_empty():
		lv.partial = lv.settle
		lv.settle = []


static func _refresh_elevation(sin_e: float) -> float:
	return maxf(absf(asin(clampf(sin_e, -1.0, 1.0))), CloudWorldVolumeSchedule.MIN_REFRESH_ELEVATION_RAD)


## The level's storage queued as tiles of whole columns (TILE_CELLS x TILE_CELLS, the storage's height), nearest the
## camera first, behind the camera later.
func _enqueue_tiles(s: CloudWorldVolumeSchedule, lv: CloudWorldVolumeSchedule._Level) -> void:
	var L := CloudWorldVolumeLayout
	var S := CloudWorldVolumeSchedule
	var T := S.TILE_CELLS
	lv.need_queue = false
	lv.generation += 1
	var o := lv.storage
	var t0 := Vector2i(floori(o.x / float(T)), floori(o.y / float(T)))
	var t1 := Vector2i(floori((o.x + L.CELLS_H - 1) / float(T)), floori((o.y + L.CELLS_H - 1) / float(T)))
	var cg: Vector3 = s.camera_grid_of(lv)
	var ch: float = L.CELL_H_M[lv.k]
	var view := lv.layout.tangent_of(s.view_dir_earth)
	var use_view := view.length_squared() > 0.0
	if use_view:
		view = view.normalized()
	var tiles: Array[Vector3i] = []
	var keys := PackedInt64Array()
	var half := 0.5 * T
	for ty in range(t0.y, t1.y + 1):
		for tx in range(t0.x, t1.x + 1):
			var key := Vector3i(tx, ty, 0)
			if lv.queued.has(key):
				continue
			var dx := (tx * T + half) * ch - cg.x
			var dy := (ty * T + half) * ch - cg.y
			var score := int(sqrt(dx * dx + dy * dy) / ch * 8.0)
			if use_view and dx * view.x + dy * view.y < 0.0:
				score *= S.BEHIND_FACTOR
			keys.append((score << S.KEY_BITS) | tiles.size())
			tiles.append(key)
	keys.sort()
	for key in keys:
		var tile := tiles[key & ((1 << S.KEY_BITS) - 1)]
		lv.queue.append(tile)
		lv.queued[tile] = true
