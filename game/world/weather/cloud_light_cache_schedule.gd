class_name CloudLightCacheSchedule
extends RefCounted
## What the light cache fills next (static clouds Task 2; pure logic, no GPU - CloudLightCache runs the jobs on the
## render thread, cloud_light_cache.gdshaderinc reads the result).
##
## Layout: CASCADES camera-anchored boxes over the cloud shell (SHELL_BOTTOM_M .. SHELL_TOP_M), all on one Earth-fixed
## frame - the tangent plane at `origin` (east, north, the geodetic up there): a voxel is the point at plane coordinates
## ((i + 0.5) dx, (j + 0.5) dx) and the height of its layer centre. Global voxel indices (i, j) never move with the
## camera; a cascade's window is the size.x x size.y block of them centred on the camera's ground point, stored
## toroidally (texel = index mod size), so a re-anchor - the ground point more than REANCHOR_VOXELS from the window's
## centre along an axis - re-centres the window and fills only the uncovered slab; the rest keeps its texels (and a
## strip in progress its finished layers).
## Progressive fill: a slot's held part (valid) is a rectangle that grows strip by strip - a STRIP_VOXELS square around
## the ground point first, then strips at most STRIP_VOXELS wide along the side nearest the ground point - so a first
## fill shows its inner part within a few frames and never restarts on a re-anchor or a small Sun turn.
## Generations: two slots per cascade (the two cache textures). Slot `front` is shown; a refresh (the Sun turned by
## REFRESH_ANGLE_RAD, the snapshot blend moved by REFRESH_BLEND_STEP) fills the other slot completely with the new light
## and blend - the old one stays shown alone meanwhile - and then the lookup cross-fades to it over CROSSFADE_S. One
## generation is built at a time, the stalest cascade's first (no cascade starves under time warp). A clock jump, a
## reset (the field changed: CloudLayer) or the ground point leaving the frame (REFRAME_M) set a new frame at the
## ground point and invalidate everything; a cascade whose shown light is more than RESET_ANGLE_RAD off the Sun drops it
## (its generation in progress takes over, else it starts empty). Until filled the lookup falls back to the march.
## Serving: each frame's budget goes first to the shown slots' missing parts (shared fairly: the cascades needing less
## than an equal share get all they need, the rest split the remainder - flight faster than a fine cascade's fill
## leaves that one behind, not the coarse ones), then the fading-out slots', then the generation being built.
## Lookup weights: inside a cascade's camera square (half size radius_voxels around the ground point, the same point as
## the re-anchoring, so the square always lies inside the old and the new window) the cascade has weight 1, falling to
## 0 over BAND_VOXELS at its edge (and at the edge of what its slots hold); the rest goes to the next cascade, after the
## last to the march - no seam anywhere.
## Jobs per update: {cascade, region: AABB (x, y: global voxel indices, z: layer; whole layers, rows of one layer or
## part of one row), generation (slot), light (unit, Earth-local), blend (the field's w1 to fill with)}, exactly
## min(budget_voxels, what is missing) voxels.

## [E] The cascades (spec A; tuned in Task 10): horizontal extent and voxels (east, north, up). 62.5 m across and 40 m
## up near the camera (static clouds Task 1: cloud_check.gd section 8); x2 per cascade, a clipmap (8 / 16 / 32 / 64 /
## 128 km; controller ruling 2026-10-26): each level sees about the first's voxel / pixel-footprint ratio at its
## distance. Weight 1 to 34 km from the ground point, 0 at 42 km: the volume march's full range (Task 8's near / far
## hand-over at <= 34 km, ledgered). The vertical stays 40 m in every cascade: the light's kink at a cloud's top does
## not get coarser with distance.
## near_m (hybrid, > 0): the sample marches the first near_m of its light path through the shapes and reads the
## cascade there (a cumulus' self-shadowing and the shadow of its neighbours lie below a far cascade's voxel;
## controller ruling 2026-10-26; the lengths: NEAR_INTERVAL_M). Measured (case 8, broken cloud): read at the sample,
## cascades 1-4 were 13-108 % off at a pixel (a x8 layout 42-1005 %). Cascade 0 (62.5 m) is read at the sample (0,
## pure): case 8 passes so (2.7 / 4.6 %), it serves the most samples, and pure is the cheaper path.
const CASCADES := [
	{"extent_m": 8000.0, "size": Vector3i(128, 128, 512), "near_m": 0.0},
	{"extent_m": 16000.0, "size": Vector3i(128, 128, 512), "near_m": 1000.0},
	{"extent_m": 32000.0, "size": Vector3i(128, 128, 512), "near_m": 3000.0},
	{"extent_m": 64000.0, "size": Vector3i(128, 128, 512), "near_m": 2000.0},
	{"extent_m": 128000.0, "size": Vector3i(128, 128, 512), "near_m": 2000.0},
]
## [E] The hybrid cascades' near march (near_m above; cloud_light_cache.gdshaderinc cloud_near_length): its first
## NEAR_GEOMETRIC_SAMPLES samples on geometric intervals over NEAR_GEOMETRIC_M from CloudLightCache.march_first_m (the
## round-3 set: cascade 1 passes with it alone), then NEAR_INTERVAL_M each (a cumulus edge's scale) up to the cascade's
## length; the stratified midpoints. Measured (cloud_check.gd section 8, broken cloud, Sun 20 deg; the worst pixel by
## the cascade holding its slab middle, over all five views; tolerance 10 %): 1 km in cascades 1-4: c2 32.1 / c3 39.6 /
## c4 15.1 % (cascades 3-4: the voxels at the 1-km point smear the shadow of the next cumulus over +-dx); cascades 2-4
## at 2 km with 208-m intervals (the geometric part's last): c2 11.5-13.3, c3 <= 5.6, c4 <= 1.6 %; c2 at 3-5 km with
## 208 m: 20.7 % (one deep-shadow pixel - the near march's own intervals, not the cache); with 100 m: c2 at 2 / 2.5 /
## 3 km 13.3 / 10.7 / 9.0 %, c3-c4 at 1.5 km 18.2 / 5.3 %, at 2 km 3.6 / 1.6 %. So 1 / 3 / 2 / 2 km (12 / 32 / 22 / 22
## samples at the desktop preset's 20 m first interval). Cost: vr-performance.md.
const NEAR_GEOMETRIC_M := 1000.0
const NEAR_GEOMETRIC_SAMPLES := 12
const NEAR_INTERVAL_M := 100.0
## The shell (cloud_pass.gdshaderinc CLOUD_BOTTOM_M .. CLOUD_TOP_M: below every ground .. above every cloud).
const SHELL_BOTTOM_M := -500.0
const SHELL_TOP_M := 20000.0
## [E] Re-anchor when the ground point is this many voxels from the window's centre along an axis (cascade 0: 1.25 km;
## a slab of 20 x 128 x 512 voxels, ~13 frames of the budget).
const REANCHOR_VOXELS := 20
## [E] The camera square's distance from the window's edge beyond the re-anchoring distance: the voxel-centre range ends
## half a voxel inside the window, and the render thread's camera trails the main thread's by up to a frame.
const MARGIN_VOXELS := 2
## [E] The blend band at a cascade's edge (cascade 0: 500 m): the step between two cascades (each within case 8's
## tolerance of the march) spread over many pixels (cloud_check.gd section 10).
const BAND_VOXELS := 8
## [E] A refresh when the Sun turned by more than this (the light at a cloud edge moves < 1 voxel: 0.25 deg x 20 km of
## light path = 87 m against 62.5 m voxels; spec Task 2's start value).
const REFRESH_ANGLE_RAD := 0.25 * PI / 180.0
## [E] A refresh when the field's blend moved by more than this (Task 1's value: a 5 % step of the snapshot mix).
const REFRESH_BLEND_STEP := 0.05
## [E] Beyond this Sun turn a shown generation is too far off to show (the clock jumped without a flag, the checks' Sun
## changes, fast time warp): the cascade drops it at once - a fading-out one ends its fade, a front gives way to the
## generation in progress (shown as far as filled) or to an empty slot (the march until filled). The bound on the shown
## light: one generation at a time, a light is taken when its fill starts and shown until the next one has faded in -
## at most two full refreshes and a fade (~10 s): 4.2 deg at 100x warp (measured 2.2-3.6 deg, never dropped), ~42 deg
## at 1000x (4.2 deg/s), so there this bound holds it (measured: every cascade within 25 deg, its ground point held in
## every frame; cloud_light_cache_test.gd).
const RESET_ANGLE_RAD := 25.0 * PI / 180.0
## [E] The cross-fade of a new generation (the light of 0.25 deg Sun or a 5 % blend: half a second is no visible pop).
const CROSSFADE_S := 0.5
## [E] A new frame (everything re-anchored, invalid) when the ground point is this far from the frame's origin in the
## plane: the plane's up then tilts <= 9 deg from the local vertical and its voxels stretch < 1.3 % (the fill solves the
## height exactly).
const REFRAME_M := 1.0e6
## [E] Voxels filled per frame: <= 0.5 ms GPU in every frame (spec decision 6: inside the 2.5 ms p99). Measured
## (cloud_check.gd section 8, RTX 4090): the same jobs cost 0.18 ms per 262 144 voxels at the GPU's full clock and
## 0.37-0.51 ms when it clocks down in a light scene (the 0.45-0.80 ms spread at 327 680: the GPU's clock state, not
## the jobs - one run steady at 0.37, another 0.18 with single frames at 0.38-0.51); real weather costs more per voxel
## (terrain_perf --render_scale=1.65, ERA5 2025-10-26 15 UTC, per-second means: 196 608 voxels up to 0.45 ms, 131 072
## up to 0.48 ms in a run where the whole GPU ran ~2x slower - the system's state, every pass alike). So 4 layers of
## cascade 0 (65 536 voxels, <= 0.5 ms also in that slow state). A full refresh of all 42 M voxels takes 640 frames (7.1 s at
## 90 Hz), cross-faded.
const BUDGET_VOXELS := 65536

## [E] The progressive fill's strip width and first square (cascade 0: 1 km; 131 072 voxels, ~7 frames at a fifth of the
## budget): a first fill or a refill after a reset shows its inner part within a few frames, and a re-anchor's slab is
## at most this wide per step.
const STRIP_VOXELS := 16

var budget_voxels := BUDGET_VOXELS

## The frame (Earth-local render axes, m; float64): NaN origin until the first update.
var origin := Vector3(NAN, NAN, NAN)
var east := Vector3.RIGHT
var north := Vector3.BACK
var up := Vector3.UP

## Per cascade: {size, centre: Vector2i (the window spans centre - size / 2 .. centre + size / 2), ground: Vector2
## (the ground point in voxel units), front: int, fade: float (1: done), slots: [{active, building, valid: Rect2i,
## light, blend, box: Rect2i (the strip in progress), z, row, col (its fill cursor: layer, row, column)}, ...]}.
var _cascades: Array[Dictionary] = []
var _reset := true


## The near march's samples of cascade k (0: pure) with first interval first_m: NEAR_GEOMETRIC_SAMPLES geometric ones
## over NEAR_GEOMETRIC_M, then NEAR_INTERVAL_M each until the cascade's near_m is reached.
static func near_samples(k: int, first_m: float) -> int:
	var target: float = CASCADES[k]["near_m"]
	var n := 0
	while target > 0.0 and near_length_m(n, first_m) < target - 1e-6:
		n += 1
	return n


## The length of the near march's first n samples (cloud_light_cache.gdshaderinc cloud_near_length).
static func near_length_m(n: int, first_m: float) -> float:
	var ratio := CloudLayer.light_march_ratio(first_m, NEAR_GEOMETRIC_M, NEAR_GEOMETRIC_SAMPLES)
	var m := mini(n, NEAR_GEOMETRIC_SAMPLES)
	var geometric := first_m * (pow(ratio, m) - 1.0) / (ratio - 1.0) if ratio > 1.0 else first_m * m
	return geometric + maxi(n - NEAR_GEOMETRIC_SAMPLES, 0) * NEAR_INTERVAL_M


func _init() -> void:
	assert(float(CASCADES[0]["near_m"]) == 0.0, "cascade 0 is read at the sample (the shader's near weights: 1..4)")
	for c: Dictionary in CASCADES:
		_cascades.append({"size": c["size"], "centre": Vector2i.ZERO, "ground": Vector2.ZERO, "front": 0, "fade": 1.0,
				"slots": [_idle_slot(), _idle_slot()]})


## All voxels of one generation (every cascade).
static func voxel_count() -> int:
	var n := 0
	for c: Dictionary in CASCADES:
		var s: Vector3i = c["size"]
		n += s.x * s.y * s.z
	return n


## The geodetic up of an Earth-local point (render axes: polar axis +Y; WGS84).
static func geodetic_up(p: Vector3) -> Vector3:
	var stretch := CloudLayer.POLAR_STRETCH
	return Vector3(p.x, p.y * stretch * stretch, p.z).normalized()


## Invalidates everything at the next update (the field changed: slots, quality, detail, debug output).
func reset() -> void:
	_reset = true


## One frame: re-anchors, drops stale light, refreshes, advances the cross-fades and returns this frame's fill jobs
## (class comment). camera_earth_m: the camera's ground point (Earth-local render axes, m); light_dir: unit toward the
## Sun; blend: the field's w1; clock_jumped: the clock jumped since the last update; delta_s: wall time since the last
## update.
func update(camera_earth_m: Vector3, light_dir: Vector3, blend: float, clock_jumped: bool,
		delta_s := 0.0) -> Array[Dictionary]:
	var light := light_dir.normalized()
	if is_nan(origin.x) or _plane(camera_earth_m).length() > REFRAME_M:
		_reset = true
	if clock_jumped or _reset:
		_invalidate(camera_earth_m, light, blend)
	_reanchor(camera_earth_m)
	for c: Dictionary in _cascades:
		_drop_stale(c, light, blend)
	_refresh(light, blend)
	for c: Dictionary in _cascades:
		_advance_fade(c, delta_s)
	return _jobs()


## True when every cascade's shown slot holds its whole window and nothing fills or fades.
func complete() -> bool:
	for c: Dictionary in _cascades:
		var front: Dictionary = c["slots"][c["front"]]
		if c["fade"] < 1.0 or c["slots"][1 - c["front"]]["active"] or front["valid"] != _window(c):
			return false
	return true


## The lookup's state after this update's jobs (CloudLightCache hands it to the shader with them): the frame and per
## cascade {dx_m, size, window, centre (the ground point, voxel units), radius_voxels, band_voxels, valid: [slot 0,
## slot 1] (Rect2i of global indices; empty: none), weight (of slot 1), lights: [slot 0, slot 1] (the light each was
## filled with; checks)}.
func lookup_state() -> Dictionary:
	var cascades: Array[Dictionary] = []
	for k in range(_cascades.size()):
		var c: Dictionary = _cascades[k]
		var size: Vector3i = CASCADES[k]["size"]
		var valid: Array[Rect2i] = []
		var lights: Array[Vector3] = []
		for s: Dictionary in c["slots"]:
			valid.append(s["valid"] if s["active"] and not s["building"] else Rect2i())
			lights.append(s["light"])
		var fade: float = c["fade"]
		cascades.append({"dx_m": _dx(k), "size": size, "near_m": CASCADES[k]["near_m"], "window": _window(c),
				"centre": c["ground"],
				"radius_voxels": float(size.x / 2 - REANCHOR_VOXELS - MARGIN_VOXELS), "band_voxels": float(BAND_VOXELS),
				"valid": valid, "weight": fade if c["front"] == 1 else 1.0 - fade, "lights": lights})
	return {"origin": origin, "east": east, "north": north, "up": up, "cascades": cascades}


func _set_frame(ground: Vector3) -> void:
	origin = ground
	up = geodetic_up(ground)
	east = Vector3.UP.cross(up)
	east = east.normalized() if east.length_squared() > 1e-12 else Vector3.RIGHT
	north = up.cross(east).normalized()


## Every cascade re-centred on the ground point, slot 0 shown and empty (filled with this light and blend), slot 1 idle.
func _invalidate(ground: Vector3, light: Vector3, blend: float) -> void:
	_reset = false
	_set_frame(ground)
	for k in range(_cascades.size()):
		var c: Dictionary = _cascades[k]
		var g := _plane(ground) / _dx(k)
		c["centre"] = Vector2i(roundi(g.x), roundi(g.y))
		c["front"] = 0
		c["fade"] = 1.0
		c["slots"] = [_new_slot(light, blend), _idle_slot()]


func _reanchor(ground: Vector3) -> void:
	for k in range(_cascades.size()):
		var c: Dictionary = _cascades[k]
		var g := _plane(ground) / _dx(k)
		c["ground"] = g
		var centre: Vector2i = c["centre"]
		if absf(g.x - centre.x) > REANCHOR_VOXELS:
			centre.x = roundi(g.x)
		if absf(g.y - centre.y) > REANCHOR_VOXELS:
			centre.y = roundi(g.y)
		if centre == c["centre"]:
			continue
		c["centre"] = centre
		var window := _window(c)
		for s: Dictionary in c["slots"]:
			if not s["active"]:
				continue
			s["valid"] = _intersect(s["valid"], window)
			# The strip in progress keeps its finished layers where it stays inside the window (valid and the strip
			# stay one rectangle: both are cut by the same window); a partly filled layer is filled again.
			var box := _intersect(s["box"], window)
			if box != s["box"]:
				s["box"] = box
				s["row"] = 0
				s["col"] = 0
				if not box.has_area():
					s["z"] = 0


## A shown slot whose light is more than RESET_ANGLE_RAD off the Sun is dropped: a fading-out one ends its fade, the
## front gives way to the generation in progress (shown as far as it is filled) or to an empty slot with this light.
func _drop_stale(c: Dictionary, light: Vector3, blend: float) -> void:
	var f: int = c["front"]
	var other: Dictionary = c["slots"][1 - f]
	if c["fade"] < 1.0 and light.angle_to(other["light"]) > RESET_ANGLE_RAD:
		c["fade"] = 1.0
		c["slots"][1 - f] = _idle_slot()
		other = c["slots"][1 - f]
	var front: Dictionary = c["slots"][f]
	if not front["active"] or light.angle_to(front["light"]) <= RESET_ANGLE_RAD:
		return
	if other["building"]:
		other["building"] = false
		c["front"] = 1 - f
		c["slots"][f] = _idle_slot()
	else:
		c["slots"][f] = _new_slot(light, blend)
	c["fade"] = 1.0


## New light where the Sun or the blend moved beyond the thresholds: a front that has not received a voxel yet takes it
## at once; otherwise one new generation at a time, for the stalest cascade whose front is complete and not fading.
func _refresh(light: Vector3, blend: float) -> void:
	var building := false
	var best := -1
	var best_score := 1.0
	for k in range(_cascades.size()):
		var c: Dictionary = _cascades[k]
		var newest := _newest(c)
		var score := maxf(light.angle_to(newest["light"]) / REFRESH_ANGLE_RAD,
				absf(blend - float(newest["blend"])) / REFRESH_BLEND_STEP)
		var front: Dictionary = c["slots"][c["front"]]
		var back: Dictionary = c["slots"][1 - c["front"]]
		building = building or back["building"]
		if score > 1.0 and not _started(front):
			c["slots"][c["front"]] = _new_slot(light, blend)
			continue
		if score > best_score and c["fade"] >= 1.0 and not back["active"] and front["valid"] == _window(c):
			best = k
			best_score = score
	if building or best < 0:
		return
	var c: Dictionary = _cascades[best]
	var back := _new_slot(light, blend)
	back["building"] = true
	c["slots"][1 - c["front"]] = back


func _advance_fade(c: Dictionary, delta_s: float) -> void:
	if c["fade"] >= 1.0:
		return
	c["fade"] = minf(float(c["fade"]) + delta_s / CROSSFADE_S, 1.0)
	if c["fade"] >= 1.0:
		c["slots"][1 - c["front"]] = _idle_slot()


## This frame's jobs (class comment, Serving): the shown slots' missing parts, the fading-out slots', the generation
## being built.
func _jobs() -> Array[Dictionary]:
	var jobs: Array[Dictionary] = []
	var budget := budget_voxels
	for pass_index in range(3):  # 0: shown, 1: fading out, 2: building
		var wanted: Array[Vector3i] = []  # (cascade, slot, voxels missing)
		for k in range(_cascades.size()):
			var c: Dictionary = _cascades[k]
			var slot_index: int = c["front"] if pass_index == 0 else 1 - c["front"]
			var s: Dictionary = c["slots"][slot_index]
			if not s["active"] or pass_index > 0 and bool(s["building"]) != (pass_index == 2):
				continue
			var missing := _missing(k, slot_index)
			if missing > 0:
				wanted.append(Vector3i(k, slot_index, missing))
		budget = _share(wanted, budget, jobs)
	for c: Dictionary in _cascades:
		var back: Dictionary = c["slots"][1 - c["front"]]
		if back["building"] and back["valid"] == _window(c):  # the new generation is complete: fade to it
			back["building"] = false
			c["front"] = 1 - c["front"]
			c["fade"] = 0.0
	return jobs


## Splits budget among wanted (cascade, slot, voxels missing) fairly: in order of need, each gets min(need, an equal
## share of what is left); appends the jobs, returns the budget left.
func _share(wanted: Array[Vector3i], budget: int, jobs: Array[Dictionary]) -> int:
	wanted.sort_custom(func(a: Vector3i, b: Vector3i) -> bool: return a.z < b.z or a.z == b.z and a.x < b.x)
	var gets := {}
	var left := budget
	for i in range(wanted.size()):
		var n := mini(wanted[i].z, left / (wanted.size() - i))
		gets[wanted[i].x] = n
		left -= n
	for i in range(wanted.size()):  # the integer shares' remainder, to the first that still miss voxels
		if left <= 0:
			break
		var more := mini(wanted[i].z - int(gets[wanted[i].x]), left)
		gets[wanted[i].x] = int(gets[wanted[i].x]) + more
		left -= more
	wanted.sort_custom(func(a: Vector3i, b: Vector3i) -> bool: return a.x < b.x)  # jobs in cascade order
	for w in wanted:
		if int(gets[w.x]) > 0:
			_fill(w.x, w.y, gets[w.x], jobs)
	return left


## The voxels slot slot_index of cascade k still misses (window minus valid, less the strip's finished part).
func _missing(k: int, slot_index: int) -> int:
	var c: Dictionary = _cascades[k]
	var s: Dictionary = c["slots"][slot_index]
	var layers: int = CASCADES[k]["size"].z
	var valid: Rect2i = s["valid"]
	var n := (_window(c).get_area() - valid.get_area()) * layers
	var box: Rect2i = s["box"]
	if box.has_area():
		n -= (int(s["z"]) * box.size.y + int(s["row"])) * box.size.x + int(s["col"])
	return n


## Appends the jobs filling n voxels of slot slot_index of cascade k toward its window (n <= its missing voxels): the
## strips of window minus valid toward the ground point (_next_box), each layer by layer, row by row within a layer,
## voxel by voxel within a row (the cursor z, row, col; z counts the layers from cascade k's first, k x layers /
## cascades, wrapping: in a first fill all cascades progress alike, and the layers in and below the clouds cost
## several times the clear ones - so a frame mixes both); valid grows by a whole strip at a time (stays a rectangle).
func _fill(k: int, slot_index: int, n: int, jobs: Array[Dictionary]) -> void:
	var c: Dictionary = _cascades[k]
	var s: Dictionary = c["slots"][slot_index]
	var window := _window(c)
	var layers: int = CASCADES[k]["size"].z
	var offset := k * layers / CASCADES.size()  # cascade k's first layer (its cursor z counts layers from there)
	while n > 0 and s["valid"] != window:
		if not (s["box"] as Rect2i).has_area():
			s["box"] = _next_box(window, s["valid"], c["ground"])
			s["z"] = 0
			s["row"] = 0
			s["col"] = 0
		var box: Rect2i = s["box"]
		var nx := box.size.x
		var ny := box.size.y
		while n > 0 and int(s["z"]) < layers:
			var z: int = s["z"]
			var row: int = s["row"]
			var col: int = s["col"]
			var region := AABB()
			var layer := (z + offset) % layers
			if col > 0 or n < nx:  # part of one row
				var m := mini(n, nx - col)
				region = AABB(Vector3(box.position.x + col, box.position.y + row, layer), Vector3(m, 1, 1))
				col += m
				if col >= nx:
					col = 0
					row += 1
			elif row > 0 or n < nx * ny:  # rows of one layer
				var rows := mini(ny - row, n / nx)
				region = AABB(Vector3(box.position.x, box.position.y + row, layer), Vector3(nx, rows, 1))
				row += rows
			else:  # whole layers (up to the top layer: the next job wraps to layer 0)
				var count := mini(mini(layers - z, layers - layer), n / (nx * ny))
				region = AABB(Vector3(box.position.x, box.position.y, layer), Vector3(nx, ny, count))
				z += count
			if row >= ny:
				row = 0
				z += 1
			s["z"] = z
			s["row"] = row
			s["col"] = col
			n -= int(region.get_volume())
			jobs.append({"cascade": k, "region": region, "generation": slot_index, "light": s["light"],
					"blend": s["blend"]})
		if int(s["z"]) >= layers:
			s["valid"] = box if not (s["valid"] as Rect2i).has_area() else (s["valid"] as Rect2i).merge(box)
			s["box"] = Rect2i()


## The next strip of window minus valid (valid: a sub-rectangle of window, or empty) toward the ground point g (voxel
## units): a STRIP_VOXELS square around g (inside the window) first, then strips along valid's side nearest g that still
## has room - left / right spanning valid's rows, bottom / top its columns - each at most STRIP_VOXELS wide.
static func _next_box(window: Rect2i, valid: Rect2i, g: Vector2) -> Rect2i:
	if not valid.has_area():
		var seed := Vector2i(mini(STRIP_VOXELS, window.size.x), mini(STRIP_VOXELS, window.size.y))
		var corner := Vector2i(roundi(g.x), roundi(g.y)) - seed / 2
		corner = corner.clamp(window.position, window.end - seed)
		return Rect2i(corner, seed)
	var side := -1
	var nearest := INF
	var distances := [g.x - valid.position.x, valid.end.x - g.x, g.y - valid.position.y, valid.end.y - g.y]
	var room := [valid.position.x > window.position.x, valid.end.x < window.end.x,
			valid.position.y > window.position.y, valid.end.y < window.end.y]
	for i in range(4):
		if room[i] and float(distances[i]) < nearest:
			side = i
			nearest = distances[i]
	match side:
		0:
			var w := mini(STRIP_VOXELS, valid.position.x - window.position.x)
			return Rect2i(valid.position.x - w, valid.position.y, w, valid.size.y)
		1:
			return Rect2i(valid.end.x, valid.position.y, mini(STRIP_VOXELS, window.end.x - valid.end.x), valid.size.y)
		2:
			var h := mini(STRIP_VOXELS, valid.position.y - window.position.y)
			return Rect2i(valid.position.x, valid.position.y - h, valid.size.x, h)
	return Rect2i(valid.position.x, valid.end.y, valid.size.x, mini(STRIP_VOXELS, window.end.y - valid.end.y))


static func _intersect(a: Rect2i, b: Rect2i) -> Rect2i:
	var r := a.intersection(b)
	return r if r.has_area() else Rect2i()


## True when the slot received a voxel (valid or a strip in progress).
static func _started(s: Dictionary) -> bool:
	return (s["valid"] as Rect2i).has_area() or (s["box"] as Rect2i).has_area()


## The newest generation's slot (the one filling, else the shown one).
static func _newest(c: Dictionary) -> Dictionary:
	var back: Dictionary = c["slots"][1 - c["front"]]
	return back if back["building"] else c["slots"][c["front"]]


static func _window(c: Dictionary) -> Rect2i:
	var size: Vector3i = c["size"]
	var centre: Vector2i = c["centre"]
	return Rect2i(centre.x - size.x / 2, centre.y - size.y / 2, size.x, size.y)


func _dx(k: int) -> float:
	return float(CASCADES[k]["extent_m"]) / float(CASCADES[k]["size"].x)


## The plane coordinates (east, north; m) of an Earth-local point.
func _plane(p: Vector3) -> Vector2:
	var d := p - origin
	return Vector2(d.dot(east), d.dot(north))


static func _new_slot(light: Vector3, blend: float) -> Dictionary:
	return {"active": true, "building": false, "valid": Rect2i(), "light": light, "blend": blend, "box": Rect2i(),
			"z": 0, "row": 0, "col": 0}


static func _idle_slot() -> Dictionary:
	return {"active": false, "building": false, "valid": Rect2i(), "light": Vector3.ZERO, "blend": 0.0,
			"box": Rect2i(), "z": 0, "row": 0, "col": 0}
