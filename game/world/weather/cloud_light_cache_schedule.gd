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
## centre along an axis - re-centres the window and fills only the uncovered slab; the rest keeps its texels.
## Generations: two slots per cascade (the two cache textures). Slot `front` is shown; a refresh (the Sun turned by
## REFRESH_ANGLE_RAD, the snapshot blend moved by REFRESH_BLEND_STEP) fills the other slot completely with the new light
## and blend - the old one stays shown alone meanwhile - and then the lookup cross-fades to it over CROSSFADE_S. A clock
## jump, a reset (the field changed: CloudLayer), a Sun jump beyond RESET_ANGLE_RAD or the ground point leaving the
## frame (REFRAME_M) set a new frame at the ground point and invalidate everything: until filled the lookup falls back
## to the per-sample march.
## Lookup weights: inside a cascade's camera square (half size radius_voxels around the ground point, the same point as
## the re-anchoring, so the square always lies inside the old and the new window) the cascade has weight 1, falling to
## 0 over BAND_VOXELS at its edge (and at the edge of what its slots hold); the rest goes to the next cascade, after the
## last to the march - no seam anywhere.
## Jobs per update: {cascade, region: AABB (x, y: global voxel indices, z: layer; whole layers or rows of one layer),
## generation (slot), light (unit, Earth-local), blend (the field's w1 to fill with)}, at most budget_voxels voxels.

## [E] The cascades (spec A; tuned in Task 10): horizontal extent and voxels (east, north, up). 62.5 m across and 40 m
## up near the camera (static clouds Task 1: cloud_check.gd section 8); x2 per cascade, a clipmap (8 / 16 / 32 / 64 /
## 128 km; controller ruling 2026-10-26): each level sees about the first's voxel / pixel-footprint ratio at its
## distance. Weight 1 to 34 km from the ground point, 0 at 42 km: the volume march's full range (Task 8's near / far
## hand-over at <= 34 km, ledgered). The vertical stays 40 m in every cascade: the light's kink at a cloud's top does
## not get coarser with distance.
## hybrid: the sample marches the first NEAR_M of its light path through the shapes and reads the cascade there (a
## cumulus' self-shadowing lies below a far cascade's voxel; controller ruling 2026-10-26). Measured (case 8 at each
## level's scale, broken cloud): read at the sample, cascades 1-4 were 13-108 % off at a pixel (a x8 layout 42-1005 %);
## hybrid, 1-2 pass and 3-4 at a 20-deg Sun keep ~40 % at one edge pixel (open). Cascade 0 (62.5 m) is read at the
## sample (pure): case 8 passes so (2.7 / 4.6 %), it serves the most samples, and pure is the cheaper path.
const CASCADES := [
	{"extent_m": 8000.0, "size": Vector3i(128, 128, 512), "hybrid": false},
	{"extent_m": 16000.0, "size": Vector3i(128, 128, 512), "hybrid": true},
	{"extent_m": 32000.0, "size": Vector3i(128, 128, 512), "hybrid": true},
	{"extent_m": 64000.0, "size": Vector3i(128, 128, 512), "hybrid": true},
	{"extent_m": 128000.0, "size": Vector3i(128, 128, 512), "hybrid": true},
]
## [E] The hybrid cascades' near march: its length (a cumulus' self-shadowing scale) and samples (geometric intervals
## from CloudLightCache.march_first_m, the stratified midpoints). Measured (cloud_check.gd section 8, broken cloud,
## worst pixel of cascades 1 / 2 / 3 / 4 at Sun 20 deg): 500 m / 8 7.4 / 14.5 / 29.7 / 23.3 %; 2 km / 8 24.7 / 153 /
## 47 / 61 % (the near march's own intervals too coarse); 1 km / 12 9.6 / 9.4 / 39.6 / 39.8 %; 1 km / 16 7.4 / 9.4 /
## 39.6 / 39.2 % (no gain for 4 more samples). Cost (terrain_perf --render_scale=1.65, the VR preset): the cloud pass
## +1.1-1.4 ms over the pure lookup (vr-performance.md).
const NEAR_M := 1000.0
const NEAR_SAMPLES := 12
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
## [E] Beyond this Sun turn the shown generation is too far off to fade from (the clock jumped without a flag, the
## checks' Sun changes): invalidate at once, the march takes over until filled. Under 1000x time warp the Sun turns
## ~4.2 deg/s; a refresh (the full fill ~4.7 s + the fade 0.5 s) then lags by up to ~22 deg - below this, so warp
## still cross-fades.
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
## up to 0.48 ms in a run where the whole GPU ran ~2x slower - the system's state, every pass alike). So 6 layers of
## cascade 0 (98 304 voxels, ~0.36 ms in that slow state). A full refresh of all 42 M voxels takes 427 frames (4.7 s at
## 90 Hz), cross-faded.
const BUDGET_VOXELS := 98304

var budget_voxels := BUDGET_VOXELS

## The frame (Earth-local render axes, m; float64): NaN origin until the first update.
var origin := Vector3(NAN, NAN, NAN)
var east := Vector3.RIGHT
var north := Vector3.BACK
var up := Vector3.UP

## Per cascade: {size, centre: Vector2i (the window spans centre - size / 2 .. centre + size / 2), ground: Vector2
## (the ground point in voxel units), front: int, fade: float (1: done), slots: [{active, building, valid: Rect2i,
## light, blend, box: Rect2i, z, row}, ...]}.
var _cascades: Array[Dictionary] = []
var _reset := true


func _init() -> void:
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


## One frame: re-anchors, refreshes, advances the cross-fades and returns this frame's fill jobs (class comment).
## camera_earth_m: the camera's ground point (Earth-local render axes, m); light_dir: unit toward the Sun; blend: the
## field's w1; clock_jumped: the clock jumped since the last update; delta_s: wall time since the last update.
func update(camera_earth_m: Vector3, light_dir: Vector3, blend: float, clock_jumped: bool,
		delta_s := 0.0) -> Array[Dictionary]:
	var light := light_dir.normalized()
	if is_nan(origin.x) or _plane(camera_earth_m).length() > REFRAME_M:
		_reset = true
	if not _reset and light.angle_to(_newest(_cascades[0])["light"]) > RESET_ANGLE_RAD:
		_reset = true
	if clock_jumped or _reset:
		_invalidate(camera_earth_m, light, blend)
	_reanchor(camera_earth_m)
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
## slot 1] (Rect2i of global indices; empty: none), weight (of slot 1)}.
func lookup_state() -> Dictionary:
	var cascades: Array[Dictionary] = []
	for k in range(_cascades.size()):
		var c: Dictionary = _cascades[k]
		var size: Vector3i = CASCADES[k]["size"]
		var valid: Array[Rect2i] = []
		for s: Dictionary in c["slots"]:
			valid.append(s["valid"] if s["active"] and not s["building"] else Rect2i())
		var fade: float = c["fade"]
		cascades.append({"dx_m": _dx(k), "size": size, "hybrid": CASCADES[k]["hybrid"], "window": _window(c),
				"centre": c["ground"],
				"radius_voxels": float(size.x / 2 - REANCHOR_VOXELS - MARGIN_VOXELS), "band_voxels": float(BAND_VOXELS),
				"valid": valid, "weight": fade if c["front"] == 1 else 1.0 - fade})
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
			if s["active"]:
				s["valid"] = _intersect(s["valid"], window)
				s["box"] = Rect2i()  # a box in progress restarts (its texels lie outside valid: never read)


## A new generation where the Sun or the blend moved beyond the thresholds and the cascade is not busy with one.
func _refresh(light: Vector3, blend: float) -> void:
	for c: Dictionary in _cascades:
		var newest := _newest(c)
		if light.angle_to(newest["light"]) <= REFRESH_ANGLE_RAD \
				and absf(blend - float(newest["blend"])) <= REFRESH_BLEND_STEP:
			continue
		var front: Dictionary = c["slots"][c["front"]]
		if not (front["valid"] as Rect2i).has_area():  # nothing shown yet: refill the front with the new light
			c["slots"][c["front"]] = _new_slot(light, blend)
		elif c["fade"] >= 1.0 and not c["slots"][1 - c["front"]]["active"]:
			var back := _new_slot(light, blend)
			back["building"] = true
			c["slots"][1 - c["front"]] = back


func _advance_fade(c: Dictionary, delta_s: float) -> void:
	if c["fade"] >= 1.0:
		return
	c["fade"] = minf(float(c["fade"]) + delta_s / CROSSFADE_S, 1.0)
	if c["fade"] >= 1.0:
		c["slots"][1 - c["front"]] = _idle_slot()


## This frame's jobs: the shown slots' missing parts first (near cascades first), then the fading-out slots', then the
## new generations'.
func _jobs() -> Array[Dictionary]:
	var jobs: Array[Dictionary] = []
	var budget := budget_voxels
	for pass_index in range(3):  # 0: shown, 1: fading out, 2: building
		for k in range(_cascades.size()):
			var c: Dictionary = _cascades[k]
			var slot_index: int = c["front"] if pass_index == 0 else 1 - c["front"]
			var s: Dictionary = c["slots"][slot_index]
			if not s["active"] or pass_index > 0 and bool(s["building"]) != (pass_index == 2):
				continue
			budget = _fill(k, slot_index, budget, jobs)
			if s["building"] and s["valid"] == _window(c):  # the new generation is complete: fade to it
				s["building"] = false
				c["front"] = slot_index
				c["fade"] = 0.0
	return jobs


## Appends the jobs filling slot slot_index of cascade k toward its window within budget; returns the budget left.
## Order: the strips of window minus valid (left, right, then the full-width bottom and top), each layer by layer (or
## row by row when less than a layer fits); valid grows by a whole strip at a time (stays a rectangle).
func _fill(k: int, slot_index: int, budget: int, jobs: Array[Dictionary]) -> int:
	var c: Dictionary = _cascades[k]
	var s: Dictionary = c["slots"][slot_index]
	var window := _window(c)
	var layers: int = CASCADES[k]["size"].z
	while s["valid"] != window:
		var box := _next_box(window, s["valid"])
		if box != s["box"]:
			s["box"] = box
			s["z"] = 0
			s["row"] = 0
		var nx := box.size.x
		var ny := box.size.y
		var z: int = s["z"]
		var row: int = s["row"]
		var region := AABB()
		if row == 0 and budget >= nx * ny:
			var n := mini(layers - z, budget / (nx * ny))
			region = AABB(Vector3(box.position.x, box.position.y, z), Vector3(nx, ny, n))
			s["z"] = z + n
		else:
			var rows := mini(ny - row, budget / nx)
			if rows <= 0:
				break
			region = AABB(Vector3(box.position.x, box.position.y + row, z), Vector3(nx, rows, 1))
			s["row"] = row + rows
			if s["row"] >= ny:
				s["row"] = 0
				s["z"] = z + 1
		budget -= int(region.get_volume())
		jobs.append({"cascade": k, "region": region, "generation": slot_index, "light": s["light"],
				"blend": s["blend"]})
		if s["z"] >= layers:
			s["valid"] = box if not (s["valid"] as Rect2i).has_area() else (s["valid"] as Rect2i).merge(box)
			s["box"] = Rect2i()
		if budget <= 0:
			break
	return budget


## The next strip of window minus valid (valid: a sub-rectangle of window, or empty).
static func _next_box(window: Rect2i, valid: Rect2i) -> Rect2i:
	if not valid.has_area():
		return window
	if valid.position.x > window.position.x:
		return Rect2i(window.position.x, valid.position.y, valid.position.x - window.position.x, valid.size.y)
	if valid.end.x < window.end.x:
		return Rect2i(valid.end.x, valid.position.y, window.end.x - valid.end.x, valid.size.y)
	if valid.position.y > window.position.y:
		return Rect2i(window.position.x, window.position.y, window.size.x, valid.position.y - window.position.y)
	return Rect2i(window.position.x, valid.end.y, window.size.x, window.end.y - valid.end.y)


static func _intersect(a: Rect2i, b: Rect2i) -> Rect2i:
	var r := a.intersection(b)
	return r if r.has_area() else Rect2i()


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
			"z": 0, "row": 0}


static func _idle_slot() -> Dictionary:
	return {"active": false, "building": false, "valid": Rect2i(), "light": Vector3.ZERO, "blend": 0.0,
			"box": Rect2i(), "z": 0, "row": 0}
