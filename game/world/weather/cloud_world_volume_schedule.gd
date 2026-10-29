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
## Light (CloudWorldVolumeLight, world-volume Task 6): filled levels only; blends, freshly baked cells, rate-limited
## refresh generations (Sun / Moon / blend / a coarser level drawn, settle passes from the GPU's clamp counts),
## provisional light. Fill and scroll bakes are column-shaped (whole columns of their box: the light's jobs follow them).
## Re-frame: the camera's ground point > REFRAME_M from the frame origin -> a new frame at the camera; levels are
## rebuilt one at a time, coarsest first, into a spare version in the new frame, shown (the old version stays shown
## meanwhile), cross-faded over REFRAME_FADE_S and swapped.
## Budget: UPKEEP_BUDGET_MS per frame (LIGHT_BUDGET_MS of it light); UPKEEP_SPIKE_MS in cold-start, jump and re-frame
## frames until the fill is complete (half bake, half light). The bake's work is counted in taps (a cell of level k
## costs CloudWorldVolumeLayout.cell_taps(k): 256 at level 0 to 8448 at level 7), its cost in ms at the level's rate in
## taps per ms (the cost per tap differs between levels by up to 2.4 x: bake_ms_of); the scroll bake may exceed the
## normal bake share up to SCROLL_CAP_MS (fast flight: 7.8 km/s needs ~1-1.5 ms at 4.3e7 taps/ms).
## The light's cost (latency-aware, Task 6 review round 1c): a level's light lists cost its latency (light_latency_of,
## once per plan) plus its cells at the light kernel's rate plus its column cells at the column pass's rate
## (light_ms_of) - a small dispatch is bound by its longest ray, not by its cells. feedback_split learns them from the
## level's own GPU intervals: the rates from large dispatches net of the latency, the latency from small ones net of
## their cells, each planned at the slow end of its recent samples (LIGHT_RATE_QUANTILE / LIGHT_LATENCY_QUANTILE; an
## overrun adopts the measurement at once); a level's light grows by at most LIGHT_GROWTH a plan over the sizes it was
## measured at; an upkeep over OVERRUN_HALVE x its plan halves the next plan's budgets. In spike frames a filled level
## with light still due (cold start: its whole window) holds the fill bake to SPIKE_BAKE_SHARE_LIGHT_DUE of the cap:
## each level's light completes within ~1 s of its density shown.

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
## [E] Start rates, replaced by feedback_split() (Task 6 review round 1, I1: the light's cost per cell is a constant -
## the march's fixed samples per level - so it is budgeted in cells at a rate per level): the light kernel's cells per
## ms and the column pass's column cells (columns x the window's height) per ms, below the RTX 4090's measured rates on
## cloud_check section 17's slab (vr-performance.md); the bake's taps per ms (per level) half the measured cold-start
## rate (4.3e7, section 16). A rate below the measured one only lags until the first measurement raises it.
const INITIAL_LIGHT_CELLS_PER_MS := 1.0e6
const INITIAL_COLUMN_CELLS_PER_MS := 1.0e7
const INITIAL_BAKE_TAPS_PER_MS := 2.0e7
## [E] Start latency (ms per level and plan: its two light lists), replaced by feedback_split() once small dispatches
## measure it: Task 6 round 1c's cold start on the cloudy slab measured ~0.05 ms (level 7) to ~0.2-0.3 ms (levels
## 2-6) and none at level 0 (its time is proportional to its cells). The rates are learned net of it: at the sizes
## measured the model holds whatever the split; growing a dispatch (LIGHT_GROWTH) it errs by the latency's error - a
## start value above the true latency then plans too little time, one below it too much, so the low end is taken.
const INITIAL_LIGHT_LATENCY_MS := 0.1
## [E] The bake's calibration (feedback_split): an overrun (measured slower than the rate) adopts the measured rate at
## once; an underrun raises the rate by CALIBRATION_GAIN of the measured gap, at most by CALIBRATION_MAX_RAISE per
## measurement.
const CALIBRATION_GAIN := 0.2
const CALIBRATION_MAX_RAISE := 2.0
## [E] Work shorter than this - measured and at the rate - calibrates no rate (a compute list and its timestamp cost a
## few us by themselves).
const CALIBRATION_MIN_MS := 0.05
const MIN_CELLS_PER_MS := 1.0e4
## [E] A light dispatch of at least this many cells (half a tile of whole columns, 16 x 32 x 128: 1024 workgroups of
## 4^3, about one wave of the GPU's threads) measures the level's rate; a smaller one its latency (its time is that of
## its longest ray: adopting it as a rate spiralled the plans down to one column per level, Task 6 round 1b).
const LIGHT_CALIBRATION_MIN_CELLS := 65536
## [E] Growth of a level's light per plan (light_cells_cap): at most LIGHT_GROWTH x the largest of its measured
## dispatches in the window, LIGHT_PROBE_CELLS (twice LIGHT_CALIBRATION_MIN_CELLS: a large dispatch) before any - the
## model is exact only at the sizes it was measured at: a level not measured yet plans at the coarser levels' cost,
## 2.5-3.5 x too fast for the next finer level at N = 32 (round 1c: 3.65 ms against 1.50 planned at level 1, 4.93
## against 2.34 at level 0), and a rate taken net of a latency estimated too high overestimates it (at most
## 1 / LIGHT_MIN_THROUGHPUT_SHARE: one doubling of the measured time).
const LIGHT_PROBE_CELLS := 2 * LIGHT_CALIBRATION_MIN_CELLS
const LIGHT_GROWTH := 2
## [E] The light's cost samples kept per level and kernel (a second at 45 Hz) and the quantiles planned at: the rate at
## its slow end, the latency at its high end - the per-level rates jitter by up to ~65 % between frames, faster than
## the feedback (two frames late) follows (Task 6 round 1b: 3.07 ms measured against 1.95 planned at level 0); the
## latest sample is adopted at once when it is worse (an overrun).
const LIGHT_COST_WINDOW := 45
const LIGHT_RATE_QUANTILE := 0.1
const LIGHT_LATENCY_QUANTILE := 0.9
## [E] A large dispatch's rate is its cells over its time less the level's latency, that time taken as at least this
## share of the measured one (a latency estimated above a fast level's true one - the start value - cannot make the
## rate infinite; LIGHT_GROWTH bounds what the remaining optimism costs).
const LIGHT_MIN_THROUGHPUT_SHARE := 0.5
## Light kernels (the rate windows' index; CloudWorldVolume's two lists per level).
## [E] Guaranteed progress of a level's light (Task 6 review round 2, N1): a level whose light kernel was not measured by
## a large dispatch for this many plans (a second at 45 Hz, LIGHT_COST_WINDOW) while it has light pending gets one probe
## of LIGHT_CALIBRATION_MIN_CELLS cells even where its planned share does not cover it (bounded by what UPKEEP_SPIKE_MS
## leaves): its rate and latency are measured again, so a slow sample cannot freeze a level's plan.
const LIGHT_PROBE_FRAMES := 45
## [E] The least z extent of a partial column (a job that does not cover a whole column when one column's cost exceeds
## what is left): the column pass is paid whole, so a smaller slice is not worth its dispatch (a 4^3 block's depth x 4).
const LIGHT_PARTIAL_MIN_CELLS := 16
const KERNEL_LIGHT := 0
const KERNEL_COLUMN := 1
## [E] The bake's cost per tap varies with the content (clear cells skip their taps; up to 1.9 x within a level in
## cloud_check section 17's cold start): each level's bake plans at the slowest rate of its last BAKE_RATE_WINDOW
## measurements (a second at 45 Hz), and a spike frame's light gets the spike cap less the bake's plan times
## BAKE_SPIKE_MARGIN.
const BAKE_RATE_WINDOW := 45
const BAKE_SPIKE_MARGIN := 1.5
## [E] A spike frame's bake (the fills; the scroll has its own cap) gets this share of UPKEEP_SPIKE_MS - while a filled
## level's light is due, SPIKE_BAKE_SHARE_LIGHT_DUE (Task 6 review round 1c): at N = 32 cells per level a level's light
## costs up to ~70 ms on the cloudy slab (level 0 at ~1.2e5 cells / ms, level 1 ~3e5), which the light's ~0.5 ms beside
## a half-cap bake could not finish within 45 frames (level 1: 125 frames); the bake of the next finer level waits.
const SPIKE_BAKE_SHARE := 0.5
const SPIKE_BAKE_SHARE_LIGHT_DUE := 0.2
## [E] A spike frame's light plans this share of what the bake leaves of the cap: the margin left for the jitter the
## cost model's quantiles do not cover (round 1b, a rate-only model: 3.18 ms measured against 1.95 planned at level 0;
## round 1c, latency-aware at the slow end of the window: 3.04 against 2.70 at a share of 0.9, level 0's first dispatch
## after its probes; level 0's 8.4 M cells at ~1e5 cells / ms need ~1.9 ms a frame to complete within 45 frames).
const LIGHT_SPIKE_SHARE := 0.85
## [E] A measured upkeep above this many times its plan's halves the next plan's budgets (a guard on the model).
const OVERRUN_HALVE := 2.0
## Cells of a tile's key order inside the sort key (tiles per level <= 4096).
const KEY_BITS := 12

## Calibrated by feedback_split(): per level the bake's taps per ms, the light kernel's cells per ms and the column
## pass's column cells per ms (set_light_rates sets the light's for every level).
var bake_level_taps_per_ms := PackedFloat64Array([INITIAL_BAKE_TAPS_PER_MS, INITIAL_BAKE_TAPS_PER_MS,
		INITIAL_BAKE_TAPS_PER_MS, INITIAL_BAKE_TAPS_PER_MS, INITIAL_BAKE_TAPS_PER_MS, INITIAL_BAKE_TAPS_PER_MS,
		INITIAL_BAKE_TAPS_PER_MS, INITIAL_BAKE_TAPS_PER_MS])
## The slowest level's bake rate; set: every level's (schedule tests, checks).
var bake_taps_per_ms: float:
	get:
		var r := INF
		for v in bake_level_taps_per_ms:
			r = minf(r, v)
		return r
	set(value):
		bake_level_taps_per_ms.fill(value)
var light_cells_per_ms := PackedFloat64Array()
var column_cells_per_ms := PackedFloat64Array()
## Per level the latency of its light lists (ms once per plan; set_light_rates, feedback_split).
var light_latency_ms := PackedFloat64Array()
## Optional: the camera's view direction (Earth-local); tiles behind it are refreshed later. ZERO = ignore.
var view_dir_earth := Vector3.ZERO
## Checks only (cloud_check.gd section 16: a cold fill in few frames): scales every per-frame budget.
var budget_scale := 1.0
## Checks only (cloud_check.gd section 17: the light at the game's budget while the bake runs at budget_scale):
## scales the light's budget alone.
var light_budget_scale := 1.0


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
	var prov: Array = []  # baked boxes {from, to, emitted, age_s, seen} still to get real light
	var blend: Array = []  # boxes {from, to, t_s} blending their real light in over the provisional (Task 6)
	var queue: Array = []  # tile keys (Vector3i) of the refresh generation(s) in progress
	var queued := {}
	var partial: Array = []  # boxes: the rest of the tile being served
	var need_queue := false
	var generation := 0  # refresh generations queued so far (the GPU's clamp counts name theirs)
	var settle: Array = []  # boxes {from, to} whose rate-limited write clamped cells: served again once through
	var lit_valid := false
	var lit_sun := Vector3.ZERO
	var lit_moon := Vector3.ZERO
	var lit_blend := 0.0
	var lit_moon_on := false
	var moon_on := false
	## M6 (Task 6 review round 1): the level turning drawn counts up `light_epoch`; a finer level whose `lit_up_epoch`
	## (the coarser levels' epochs summed at its last relight: CloudWorldVolumeLight.coarser_epoch) differs relights once
	## (its rays' part beyond its own segment reads the drawn level's density now).
	var light_epoch := 0
	var lit_up_epoch := 0
	var was_drawn := false


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
var _bake_ms_planned := 0.0
var _scroll_cells := 0
var _scroll_taps := 0
var _scroll_ms := 0.0
var _light := CloudWorldVolumeLight.new()
var _camera_grid := Vector3.ZERO
var _camera_grid_next := Vector3.ZERO
var _camera_prev := Vector3.ZERO
var _has_prev := false
## The camera's speed per grid axis (m per plan, MOTION_DECAY peak hold).
var _motion := Vector3.ZERO
var _waiting := 0
## The last measured upkeep overran its plan by more than OVERRUN_HALVE: the next plan's budgets are halved (once).
var _overrun := false
## Per level the bake's measured rates of the last BAKE_RATE_WINDOW measurements (its planning rate is at most their
## minimum).
var _bake_window: Array = [[], [], [], [], [], [], [], []]
## Per level: its light cost was calibrated by a measurement (else it follows the slowest measured level's).
var _rates_measured: Array[bool] = [false, false, false, false, false, false, false, false]
## Per level the last LIGHT_COST_WINDOW samples: per kernel (KERNEL_LIGHT, KERNEL_COLUMN) its rates (units per ms,
## from large dispatches), the level's latencies (ms, from small ones), its dispatches' cells (light_cells_cap).
var _rate_window: Array = []
var _latency_window: Array = []
var _cells_window: Array = []
## Per level the plans since its light kernel was last measured by a large dispatch (LIGHT_PROBE_FRAMES; the light
## schedule counts them up, a probe or a measurement resets them).
var rate_age := PackedInt32Array([0, 0, 0, 0, 0, 0, 0, 0])
## Per level: its light per plan is not capped (rates set as known: schedule tests).
var _growth_free: Array[bool] = [false, false, false, false, false, false, false, false]


func _init(layout: CloudWorldVolumeLayout = null) -> void:
	_layout = layout if layout != null else CloudWorldVolumeLayout.new()
	set_light_rates(INITIAL_LIGHT_CELLS_PER_MS, INITIAL_COLUMN_CELLS_PER_MS, INITIAL_LIGHT_LATENCY_MS, false)


## Every level's light cost: rates (cells per ms; INF: free - schedule tests) and latency (ms per level and plan),
## known (`measured`: no growth bound) or start values (light_cells_cap). Forgets the cost samples.
func set_light_rates(cells_per_ms: float, column_cells_per_ms_all: float, latency_ms := 0.0, measured := true) -> void:
	var n := CloudWorldVolumeLayout.LEVELS
	light_cells_per_ms.resize(n)
	column_cells_per_ms.resize(n)
	light_latency_ms.resize(n)
	light_cells_per_ms.fill(cells_per_ms)
	column_cells_per_ms.fill(column_cells_per_ms_all)
	light_latency_ms.fill(latency_ms)
	_rate_window.clear()
	_latency_window.clear()
	_cells_window.clear()
	rate_age.fill(0)
	for k in n:
		_rate_window.append([[], []])
		_latency_window.append([])
		_cells_window.append([])
		_rates_measured[k] = measured
		_growth_free[k] = measured


## The most light cells level k may take in one plan (LIGHT_GROWTH; -1: no cap).
func light_cells_cap(k: int) -> int:
	if _growth_free[k]:
		return -1
	var window: Array = _cells_window[k]
	return LIGHT_PROBE_CELLS if window.is_empty() else LIGHT_GROWTH * int(window.max())


## The latency (ms) a plan charges once for level k's light lists (its column pass and its light kernel).
func light_latency_of(k: int) -> float:
	return light_latency_ms[k]


## The GPU time (ms at the calibrated rates, without the level's latency: light_latency_of) of a light job of level k
## over `box`: the light kernel per cell, the column pass per column cell read (each column of the box sums the level's
## whole window column).
func light_ms_of(k: int, box: Dictionary) -> float:
	var d: Vector3i = box["to"] - box["from"]
	var columns := maxi(d.x, 0) * maxi(d.y, 0)
	if columns == 0 or d.z <= 0:
		return 0.0
	return columns * (d.z / light_cells_per_ms[k] + CloudWorldVolumeLayout.window_size(k).z / column_cells_per_ms[k])


## The cells of `box` (whole columns) a light budget of `ms` pays for at level k's rates.
func light_cells_for(k: int, box: Dictionary, ms: float) -> int:
	var d: Vector3i = box["to"] - box["from"]
	if d.z <= 0 or ms <= 0.0:
		return 0
	var per_column := d.z / light_cells_per_ms[k] + CloudWorldVolumeLayout.window_size(k).z / column_cells_per_ms[k]
	if not per_column < INF:
		return 0
	return int(floorf(ms / per_column)) * d.z


## The budget factor of this plan: 1, or 1/2 after an overrun (OVERRUN_HALVE).
func overrun_factor() -> float:
	return 0.5 if _overrun else 1.0


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
	return CloudWorldVolumeLight.sun_threshold_rad(k, sun_elevation_rad, height_above_m)


## Rate calibration from one frame's measured upkeep (CloudWorldVolume.take_feedback; Task 6 review round 1, I1):
## {bake_ms, light_ms, planned_ms, bake_levels: {k: {taps, ms}}, levels: {k: {cells, cells_ms, column_cells,
## column_ms}}} - each level's bake rate and light cost (_light_sample: latency and rate per kernel) from their own
## intervals (an interval shorter than CALIBRATION_MIN_MS calibrates no rate; an overrun adopts the measurement at once;
## the bake then plans at most at its level's slowest rate of the window); an unmeasured level's bake keeps its start
## rate (below every measured one), its light the slowest measured level's cost; an upkeep above OVERRUN_HALVE x its
## plan halves the next plan's budgets.
func feedback_split(m: Dictionary) -> void:
	var bake_ms := float(m.get("bake_ms", 0.0))
	var bake_levels: Dictionary = m.get("bake_levels", {})
	for k: int in bake_levels:
		var e: Dictionary = bake_levels[k]
		var taps := int(e.get("taps", 0))
		var ms := float(e.get("ms", 0.0))
		var window: Array = _bake_window[k]
		var bake := _measured(bake_level_taps_per_ms[k], taps, ms)
		if bake != bake_level_taps_per_ms[k] or (taps > 0 and ms >= CALIBRATION_MIN_MS):
			window.append(taps / maxf(ms, 1e-9))
			if window.size() > BAKE_RATE_WINDOW:
				window.remove_at(0)
		bake_level_taps_per_ms[k] = minf(bake, window.min()) if not window.is_empty() else bake
	var levels: Dictionary = m.get("levels", {})
	for k: int in levels:
		var e: Dictionary = levels[k]
		var measured := _light_sample(k, int(e.get("cells", 0)), float(e.get("cells_ms", 0.0)),
				int(e.get("column_cells", 0)), float(e.get("column_ms", 0.0)))
		_rates_measured[k] = _rates_measured[k] or measured
	# A level not measured yet takes the slowest measured level's light cost (its start values may be far off; the
	# finer levels march more segments: slower, measured on their first jobs).
	var slowest := Vector2(INF, INF)
	var longest := 0.0
	for k in CloudWorldVolumeLayout.LEVELS:
		if _rates_measured[k]:
			slowest = Vector2(minf(slowest.x, light_cells_per_ms[k]), minf(slowest.y, column_cells_per_ms[k]))
			longest = maxf(longest, light_latency_ms[k])
	for k in CloudWorldVolumeLayout.LEVELS:
		if not _rates_measured[k] and slowest.x < INF:
			light_cells_per_ms[k] = slowest.x
			column_cells_per_ms[k] = slowest.y
			light_latency_ms[k] = longest
	var planned := float(m.get("planned_ms", 0.0))
	_overrun = planned > 0.0 and bake_ms + float(m.get("light_ms", 0.0)) > OVERRUN_HALVE * planned


## One level's light lists' sample (Task 6 review round 1c): `cells` took `cells_ms` in the light kernel,
## `column_cells` `column_ms` in the column pass. A large dispatch (cells >= LIGHT_CALIBRATION_MIN_CELLS) measures the
## light kernel's rate net of the level's latency (LIGHT_MIN_THROUGHPUT_SHARE) and the column pass's as its units over
## its time (its own latency is a few us; a kernel whose time and time at the rate are below CALIBRATION_MIN_MS: none);
## a small one measures the level's latency: the two lists' time less their units at the rates. Each is planned at its
## window's LIGHT_RATE_QUANTILE / LIGHT_LATENCY_QUANTILE, or at the sample itself when that is worse (an overrun is
## adopted at once). Returns true when it measured the light kernel's rate.
func _light_sample(k: int, cells: int, cells_ms: float, column_cells: int, column_ms: float) -> bool:
	if cells <= 0 or cells_ms + column_ms <= 0.0:
		return false
	var sizes: Array = _cells_window[k]
	sizes.append(cells)
	if sizes.size() > LIGHT_COST_WINDOW:
		sizes.remove_at(0)
	if cells < LIGHT_CALIBRATION_MIN_CELLS:
		var sample := maxf(cells_ms + column_ms - cells / light_cells_per_ms[k] - column_cells / column_cells_per_ms[k],
				0.0)
		light_latency_ms[k] = maxf(_window_quantile(_latency_window[k], sample, LIGHT_LATENCY_QUANTILE), sample)
		return false
	rate_age[k] = 0
	var measured := maxf(cells / light_cells_per_ms[k], cells_ms) >= CALIBRATION_MIN_MS
	light_cells_per_ms[k] = _rate_sample(_rate_window[k][KERNEL_LIGHT], light_cells_per_ms[k], light_latency_ms[k],
			cells, cells_ms)
	column_cells_per_ms[k] = _rate_sample(_rate_window[k][KERNEL_COLUMN], column_cells_per_ms[k], 0.0, column_cells,
			column_ms)
	return measured


## A kernel's planning rate after `units` took `ms` (net of `latency_ms`, LIGHT_MIN_THROUGHPUT_SHARE): the window's
## LIGHT_RATE_QUANTILE, or the sample when slower; `rate` unchanged when the interval is too short to measure.
static func _rate_sample(window: Array, rate: float, latency_ms: float, units: int, ms: float) -> float:
	if units <= 0 or ms <= 0.0 or maxf(units / rate, ms) < CALIBRATION_MIN_MS:
		return rate
	var sample := maxf(units / maxf(ms - latency_ms, LIGHT_MIN_THROUGHPUT_SHARE * ms), MIN_CELLS_PER_MS)
	return minf(_window_quantile(window, sample, LIGHT_RATE_QUANTILE), sample)


## `sample` appended to `window` (the last LIGHT_COST_WINDOW kept); returns the window's q-quantile.
static func _window_quantile(window: Array, sample: float, q: float) -> float:
	window.append(sample)
	if window.size() > LIGHT_COST_WINDOW:
		window.remove_at(0)
	var sorted := window.duplicate()
	sorted.sort()
	return sorted[clampi(int(roundf(q * (sorted.size() - 1))), 0, sorted.size() - 1)]


## A rate after `work` units took `ms`: calibrated when the work's time at the rate or the measured time reaches
## CALIBRATION_MIN_MS (a short interval is the timestamps' own overhead; a rate too low to plan that much is raised).
static func _measured(rate: float, work: int, ms: float) -> float:
	if work <= 0 or ms <= 0.0 or maxf(work / rate, ms) < CALIBRATION_MIN_MS:
		return rate
	return _calibrated(rate, work / ms)


## A rate after one measurement: an overrun (measured below the rate) adopts the measured rate at once, an underrun
## raises it by CALIBRATION_GAIN of the gap (at most x CALIBRATION_MAX_RAISE).
static func _calibrated(rate: float, measured: float) -> float:
	if measured < rate:
		return maxf(measured, MIN_CELLS_PER_MS)
	if not rate < INF:
		return rate
	var f := minf(measured / rate, CALIBRATION_MAX_RAISE)
	return maxf(rate * (1.0 + CALIBRATION_GAIN * (f - 1.0)), MIN_CELLS_PER_MS)


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


## True while level k's light is behind its target (a refresh generation, a settle pass, provisional light or a blend
## is pending).
func light_stale(k: int) -> bool:
	return CloudWorldVolumeLight.stale(_levels[k])


## True while cells of level k have no real light yet or still blend it in.
func light_pending(k: int) -> bool:
	return CloudWorldVolumeLight.pending(_levels[k])


## True while level k has a refresh generation queued or in progress (not counting a settle pass not yet started).
func light_refreshing(k: int) -> bool:
	var lv: _Level = _levels[k]
	return lv.need_queue or not lv.queue.is_empty() or not lv.partial.is_empty()


## Level k's refresh generation (the light jobs' `generation`).
func light_generation(k: int) -> int:
	return _levels[k].generation


## A rate-limited light job whose write clamped cells (CloudWorldVolume.take_light_feedback): level k, the generation
## it named, its box - served again once the level's queue is through. A spare (re-frame) is written directly.
func light_feedback(k: int, generation: int, box: Dictionary) -> void:
	if _started and k >= 0 and k < _levels.size():
		CloudWorldVolumeLight.feedback(_levels[k], generation, box)


## True in cold-start, jump and re-frame frames until the fill is complete (the light writes directly then).
func spike_frame() -> bool:
	return _spike


## The bake's planned time this plan (ms at the calibrated rate).
func bake_ms_planned() -> float:
	return _bake_ms_planned


## The bake's GPU time (ms at level k's calibrated rate) of `cells` cells of level k.
func bake_ms_of(k: int, cells: int) -> float:
	return cells * CloudWorldVolumeLayout.cell_taps(k) / bake_level_taps_per_ms[k]


func is_spare(lv: _Level) -> bool:
	return lv == _spare


## The camera's grid point in the level's frame.
func camera_grid_of(lv: _Level) -> Vector3:
	return _grid_of_level(lv)


func plan(camera_earth_m: Vector3, sun_dir_earth: Vector3, moon_dir_earth: Vector3, blend: float,
		cloud_top_m: float, jumped: bool, delta_s: float, slot_ids: Array = [0, 1]) -> Dictionary:
	_time_s += delta_s
	var out := {"bake": [], "light": [], "provisional": []}
	_bake_cells = 0
	_bake_taps = 0
	_bake_ms_planned = 0.0
	_scroll_cells = 0
	_scroll_taps = 0
	_scroll_ms = 0.0
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
	_bake_scroll(_scroll_items(order), _bake_ms() - _bake_ms_planned, out["bake"])
	for lv: _Level in order:
		_advance_window(lv)
	_light.run(self, order, sun_dir_earth, moon_dir_earth, blend, cloud_top_m, delta_s, out)
	_overrun = false  # the halving holds for one plan (the next measurement decides again)
	var reframe := _reframe_end(delta_s)
	if _spike and complete():
		_spike = false
	out["reframe"] = reframe
	out["lookup"] = _lookup()
	out["bake_cells"] = _bake_cells
	out["bake_taps"] = _bake_taps
	out["bake_scroll_cells"] = _scroll_cells
	out["bake_scroll_taps"] = _scroll_taps
	out["bake_scroll_ms"] = _scroll_ms
	out["cloud_top_m"] = cloud_top_m
	out["windows_waiting"] = _waiting
	out["light_cells"] = _light.light_cells
	out["light_ms_planned"] = _light.ms_planned
	out["bake_ms_planned"] = bake_ms_planned()
	out["planned_ms"] = bake_ms_planned() + _light.ms_planned
	out["light_moon_cells"] = _light.moon_cells
	out["provisional_cells"] = _light.prov_cells
	out["provisional_age_max_s"] = _light.prov_age_max
	out["provisional_max_s"] = _light.prov_age_session_max
	out["sun"] = sun_dir_earth
	out["moon"] = moon_dir_earth
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
	# The light's growth starts again from LIGHT_PROBE_CELLS (round 1c: the sizes measured on other content and at
	# another GPU load let a cold start's first level-0 dispatches run 2 x over their plan, 4.56 ms against 2.55); the
	# rates are kept (a slower measurement is adopted at once).
	for sizes: Array in _cells_window:
		sizes.clear()


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


## The bake ms that meet every deadline at a constant rate (earliest deadline first), times SCROLL_MARGIN, capped.
func _scroll_budget(items: Array) -> float:
	var need := 0.0
	var cum := 0.0
	for it: Array in items:
		var lv: _Level = it[1]
		cum += bake_ms_of(lv.k, CloudWorldVolumeLayout.box_cells(it[2]))
		need = maxf(need, cum / float(it[0]))
	return minf(need * SCROLL_MARGIN, budget_scale * SCROLL_CAP_MS)


## Bakes queued scroll slabs earliest deadline first, up to `budget` ms (at least one column when the budget is
## positive); baked cells get light first (prov list).
func _bake_scroll(items: Array, budget: float, bake_out: Array) -> void:
	var L := CloudWorldVolumeLayout
	var any := false
	for it: Array in items:
		if budget <= 0:
			return
		var lv: _Level = it[1]
		var box: Dictionary = it[2]
		var i := lv.scroll.find(box)
		if i < 0:
			continue  # baked whole by an earlier call this plan
		var sp := L.box_split_columns(box, _cells_for(lv.k, budget))  # column-shaped: the light's column pass
		if L.box_cells(sp[0]) == 0:
			if any:
				return  # the rest of the budget is less than a column
			sp = L.box_split_columns(box, box["to"].z - box["from"].z)  # one column when the budget is positive
		any = true
		_emit_bake(lv, sp[0], -1, bake_out)
		var cells := L.box_cells(sp[0])
		_scroll_cells += cells
		_scroll_taps += cells * L.cell_taps(lv.k)
		_scroll_ms += bake_ms_of(lv.k, cells)
		budget -= bake_ms_of(lv.k, cells)
		lv.prov.append({"from": sp[0]["from"], "to": sp[0]["to"], "emitted": false, "age_s": 0.0,
				"seen": false})
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
	lv.blend = []
	lv.settle = []
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
	_bake_ms_planned += bake_ms_of(lv.k, cells)


# ---------------------------------------------------------------- bake

func _bake_ms() -> float:
	var spike_share := SPIKE_BAKE_SHARE_LIGHT_DUE if _light_due() else SPIKE_BAKE_SHARE
	return budget_scale * overrun_factor() * (UPKEEP_SPIKE_MS * spike_share if _spike
			else UPKEEP_BUDGET_MS - LIGHT_BUDGET_MS)


## True while a filled level has cells without real light or still blending it in (Task 6 review round 1c: the light
## of a level shown is due within ~1 s).
func _light_due() -> bool:
	for lv: _Level in _all_levels():
		if lv.active and lv.ready and CloudWorldVolumeLight.pending(lv):
			return true
	return false


## The budget (ms) after the scroll slabs, spent coarse to fine on the fills, then the slot rebakes, then the spare;
## a finer level starts only once the coarser one is done (strictly coarse first, the budget's remainder of less than
## a cell left).
func _bake_fill(order: Array, bake_out: Array) -> void:
	var budget := maxf(_bake_ms() - _scroll_ms, 0.0)
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


## Cells of level k a budget of `ms` pays for at its bake rate (at least one: a fill always moves).
func _cells_for(k: int, ms: float) -> int:
	return maxi(int(ms * bake_level_taps_per_ms[k] / CloudWorldVolumeLayout.cell_taps(k)), 1)


## Bakes up to `budget` ms of the level's dirty boxes (clipped to its window) and hands them to the light (prov
## list); returns the ms spent. The level is ready once nothing is dirty.
func _bake_dirty(lv: _Level, budget: float, bake_out: Array) -> float:
	var L := CloudWorldVolumeLayout
	var window := L.storage_box(lv.storage)
	var n := _cells_for(lv.k, budget)
	var used := 0
	while used < n and not lv.dirty.is_empty():
		var box := L.box_clip(lv.dirty.pop_front(), window)
		if L.box_cells(box) == 0:
			continue
		var sp := L.box_split_columns(box, n - used)
		if L.box_cells(sp[0]) == 0:
			if used > 0:  # the rest of the budget is less than a column
				lv.dirty.push_front(box)
				break
			sp = L.box_split_columns(box, box["to"].z - box["from"].z)  # one column: a fill always moves
		_emit_bake(lv, sp[0], -1, bake_out)
		lv.prov.append({"from": sp[0]["from"], "to": sp[0]["to"], "emitted": false, "age_s": 0.0,
				"seen": false})
		lv.baked.append(sp[0])
		used += L.box_cells(sp[0])
		lv.dirty = sp[1] + lv.dirty
	if lv.dirty.is_empty():
		lv.ready = true
		lv.baked = []
	return bake_ms_of(lv.k, used)


func _bake_slot(lv: _Level, s: int, budget: float, bake_out: Array) -> float:
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
	return bake_ms_of(lv.k, used)


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
