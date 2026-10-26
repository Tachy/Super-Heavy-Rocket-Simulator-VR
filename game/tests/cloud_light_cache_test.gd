# The light cache's schedule (static clouds Task 2; CloudLightCacheSchedule: what to fill next - pure logic, no GPU).
# Headless: godot --headless --path game --script res://tests/cloud_light_cache_test.gd
# Checked: the initial fill completes and every frame's jobs stay within the voxel budget; a camera moving 1 km inside
# the central part schedules nothing, leaving it schedules exactly the uncovered slab (toroidal: the rest stays); the
# Sun turning 0.1 deg schedules nothing, 0.3 deg a full new generation in the other slot, shown only once complete and
# then cross-faded monotonically over CROSSFADE_S; a blend step above REFRESH_BLEND_STEP schedules a refill; a clock
# jump (and a Sun jump beyond RESET_ANGLE_RAD) re-anchors every cascade and leaves them invalid until filled (the
# lookup then falls back to the per-sample march). The fill is progressive: a held rectangle around the ground point
# within a few frames, growing outward until it holds the window, kept through re-anchors and Sun turns below
# RESET_ANGLE_RAD. Under time warp (the Sun turning at 100x / 1000x) no cascade starves: every cascade's shown light
# stays within the derived staleness bound (two full refreshes and a fade at the warp's rate, at most RESET_ANGLE_RAD);
# a moving camera (1, 2, 7.8 km/s) keeps the coarse cascades' ground point held.
extends SceneTree

const DT := 1.0 / 90.0
const MAX_FRAMES := 4000
## Earth's rotation rate (IERS Conventions 2010: 7.292115e-5 rad/s): the Sun's turn in the Earth-local frame.
const EARTH_RATE_RAD_S := 7.292115e-5
## [E] The scenarios' length: covers the first fill and five full refreshes at 100x / 1000x.
const SCENARIO_S := 30.0
## The seed of the progressive fill (CloudLightCacheSchedule.STRIP_VOXELS square, all layers) of every cascade at once.
const SEED_SLACK_FRAMES := 1
## Faster than this the re-anchor slabs of all cascades need more than the budget (cascade k's slab every
## REANCHOR_VOXELS x dx_k of flight: ~15.8 M voxels/s at 7.8 km/s against the budget's 8.8 M voxels/s at 90 Hz).
const FAST_FLIGHT_MPS := 4000.0

var _failures := 0
var _schedule := CloudLightCacheSchedule.new()
var _ground := Vector3(0.0, 0.0, CloudLayer.WGS84_A_M)  # on the equator (render axes: east +X, north +Y here)
var _sun := Vector3(0.3, 0.5, 0.8).normalized()


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	var c0: Dictionary = CloudLightCacheSchedule.CASCADES[0]
	var dx0: float = c0["extent_m"] / float(c0["size"].x)
	var n0: int = c0["size"].x
	var layers0: int = c0["size"].z

	# The first frame: jobs, nothing held yet; then the whole fill within the budget.
	var first := _schedule.update(_ground, _sun, 0.0, false, DT)
	_check(not first.is_empty() and _volume(first) <= _schedule.budget_voxels, "first frame: jobs within the budget")
	_check(_nothing_held(), "first frame: no cascade holds a point yet")
	var initial := _drain(_ground, _sun, 0.0)
	initial["volume"] += _volume(first)
	_check(initial["complete"] and initial["within_budget"], "initial fill completes within the budget per frame "
			+ "(%d frames, %d voxels)" % [initial["frames"], initial["volume"]])
	_check(initial["volume"] == _schedule.voxel_count(), "initial fill: every voxel once (%d of %d)"
			% [initial["volume"], _schedule.voxel_count()])

	# A camera moving 1 km inside the central part: nothing.
	var east := _east()
	var moved := _ground + east * 1000.0
	var jobs := _schedule.update(moved, _sun, 0.0, false, DT)
	_check(jobs.is_empty(), "1 km inside the central part: nothing scheduled (%d jobs)" % jobs.size())

	# Leaving it (1.6 km east of the anchor): exactly the uncovered slab of cascade 0, nothing else.
	var leave_m := 1600.0
	var shift := roundi(leave_m / dx0)
	var slab := AABB(Vector3(n0 / 2, -n0 / 2, 0), Vector3(shift, n0, layers0))
	var left := _drain(_ground + east * leave_m, _sun, 0.0)
	var inside := true
	var only_c0 := true
	for job: Dictionary in left["jobs"]:
		only_c0 = only_c0 and int(job["cascade"]) == 0 and int(job["generation"]) == 0
		inside = inside and slab.encloses(job["region"])
	_check(left["complete"] and left["within_budget"] and only_c0 and inside
			and left["volume"] == int(slab.get_volume()), "leaving the central part: exactly the uncovered slab "
			+ "(%d voxels of %d, cascade 0 only %s, inside %s)" % [left["volume"], int(slab.get_volume()), only_c0, inside])
	var held: Rect2i = _schedule.lookup_state()["cascades"][0]["valid"][0]
	_check(held == Rect2i(-n0 / 2 + shift, -n0 / 2, n0, n0), "cascade 0 holds the moved window: %s" % held)
	var here := _ground + east * leave_m

	# The Sun turning 0.1 deg: nothing; 0.3 deg (from the filled light): a full new generation.
	var axis := _sun.cross(Vector3.UP).normalized()
	jobs = _schedule.update(here, _sun.rotated(axis, deg_to_rad(0.1)), 0.0, false, DT)
	_check(jobs.is_empty(), "Sun turned 0.1 deg: nothing scheduled")
	var turned := _sun.rotated(axis, deg_to_rad(0.3))
	var regen := _drain(here, turned, 0.0, true)
	var in_slot1 := true
	for job: Dictionary in regen["jobs"]:
		in_slot1 = in_slot1 and int(job["generation"]) == 1
	_check(regen["complete"] and regen["within_budget"] and in_slot1
			and regen["volume"] == _schedule.voxel_count(), "Sun turned 0.3 deg: a full new generation in slot 1 "
			+ "(%d of %d voxels)" % [regen["volume"], _schedule.voxel_count()])
	_check(regen["old_kept"], "the old generation is shown alone until the new one is complete")
	_check(regen["fade_monotonic"], "the cross-fade runs monotonically over CROSSFADE_S (%d fade frames, %.2f s)"
			% [regen["fade_frames"], regen["fade_frames"] * DT])

	# A blend step: 0.04 nothing, 0.06 a refill.
	jobs = _schedule.update(here, turned, 0.04, false, DT)
	_check(jobs.is_empty(), "blend step 0.04 (<= REFRESH_BLEND_STEP): nothing scheduled")
	var blend_refill := _drain(here, turned, 0.06)
	_check(blend_refill["volume"] == _schedule.voxel_count() and blend_refill["complete"],
			"blend step 0.06 (> REFRESH_BLEND_STEP): a full refill (%d voxels)" % blend_refill["volume"])

	# A clock jump: every cascade re-anchored at the new ground point and invalid until filled.
	var far_site := _ground + east * 300000.0
	jobs = _schedule.update(far_site, turned, 0.06, true, DT)
	var state := _schedule.lookup_state()
	var anchored := true
	for k in range(CloudLightCacheSchedule.CASCADES.size()):
		var cascade: Dictionary = state["cascades"][k]
		var window: Rect2i = cascade["window"]
		var centre: Vector2 = cascade["centre"]
		anchored = anchored and window.has_point(Vector2i(floori(centre.x), floori(centre.y))) \
				and absf(window.get_center().x - centre.x) <= 1.0 and absf(window.get_center().y - centre.y) <= 1.0
	_check(anchored and (state["origin"] as Vector3).distance_to(far_site) < 1.0,
			"clock jump: every cascade re-anchored at the new ground point")
	_check(_nothing_held() and not jobs.is_empty(), "clock jump: every cascade invalid (the march) until filled")
	var after_jump := _drain(far_site, turned, 0.06, false, true)
	_check(after_jump["complete"] and after_jump["c0_grows"],
			"after the jump: cascade 0 holds a point only once filled there, its held part only grows (around the "
			+ "ground point, first at frame %d), then everything completes" % after_jump["c0_first_hold"])

	# A Sun jump beyond RESET_ANGLE_RAD (no clock jump flagged): invalid at once (no stale light shown).
	_schedule.update(far_site, _sun.rotated(axis, CloudLightCacheSchedule.RESET_ANGLE_RAD * 1.5), 0.06, false, DT)
	_check(_nothing_held(), "Sun jump beyond RESET_ANGLE_RAD: every cascade invalid until filled")

	# The budget is respected for any budget (here 1/16 of the voxels: complete after 16 frames).
	_schedule.budget_voxels = _schedule.voxel_count() / 16
	_schedule.update(far_site, _sun, 0.06, true, DT)
	var sixteenth := _drain(far_site, _sun, 0.06)
	_check(sixteenth["complete"] and sixteenth["within_budget"] and sixteenth["frames"] + 1 == 16,
			"budget 1/16 of the voxels: complete after 16 frames (%d)" % (sixteenth["frames"] + 1))

	_schedule.budget_voxels = CloudLightCacheSchedule.BUDGET_VOXELS
	_progressive_checks()
	_warp_checks()
	_flight_checks()

	if _failures == 0:
		print("CLOUD LIGHT CACHE TEST PASSED")
	else:
		printerr("CLOUD LIGHT CACHE TEST FAILED: %d" % _failures)
	quit(1 if _failures > 0 else 0)


## The progressive first fill (static camera and Sun): every cascade holds its ground point within the frames of the
## seed square of all cascades; a Sun turn above REFRESH_ANGLE_RAD (below RESET_ANGLE_RAD) during the fill keeps the
## filled part; a camera re-anchoring during the fill keeps the strip in progress.
func _progressive_checks() -> void:
	var seed_frames := _seed_frames()
	var still := _scenario(0.0, 0.0, 2.0 * seed_frames * DT)
	var first: Array = still["first_hold"]
	var ok := true
	for f: int in first:
		ok = ok and f >= 0 and f <= seed_frames + SEED_SLACK_FRAMES
	_check(ok and still["within_budget"], "first fill: every cascade holds its ground point within %d frames (%s)"
			% [seed_frames + SEED_SLACK_FRAMES, first])

	var s := CloudLightCacheSchedule.new()
	var axis := _sun.cross(Vector3.UP).normalized()
	for i in range(seed_frames + 4):
		s.update(_ground, _sun, 0.0, false, DT)
	var before := _held(s)
	var jobs := s.update(_ground, _sun.rotated(axis, deg_to_rad(1.0)), 0.0, false, DT)
	var after := _held(s)
	var kept := true
	for k in range(before.size()):
		kept = kept and (not before[k].has_area() or after[k].encloses(before[k]))
	var front_light := true
	for job: Dictionary in jobs:
		front_light = front_light and (job["light"] as Vector3).is_equal_approx(_sun)
	_check(kept and front_light, "first fill: a 1-deg Sun turn keeps the filled part and the front's light "
			+ "(refreshed through the back slot; held before %s after %s)" % [before, after])

	# A re-anchor while a strip is in progress: the strip's finished layers are kept (the cascade's missing voxels drop
	# by the frame's budget, not grow back by the strip's).
	var m := CloudLightCacheSchedule.new()
	m.budget_voxels = 4096  # a fifth each: cascade 0's seed square (16 x 16 x 512) takes 160 frames
	for i in range(10):
		m.update(_ground, _sun, 0.0, false, DT)
	var dx0: float = CloudLightCacheSchedule.CASCADES[0]["extent_m"] / CloudLightCacheSchedule.CASCADES[0]["size"].x
	var jump := _ground + _east() * dx0 * (CloudLightCacheSchedule.REANCHOR_VOXELS + 1)
	var first_layer := 1 << 30
	for job: Dictionary in m.update(jump, _sun, 0.0, false, DT):
		if int(job["cascade"]) == 0:
			first_layer = mini(first_layer, int((job["region"] as AABB).position.z))
	_check(first_layer > 0 and first_layer < 1 << 30, "a re-anchor during the first fill resumes the strip in progress "
			+ "(cascade 0's next job at layer %d)" % first_layer)


## Time warp: the Sun turning at 100x and 1000x for SCENARIO_S from the first fill. Every cascade's shown light (a
## held slot with weight > 0) stays within the bound: the light is taken when a fill starts and shown until the next
## one has faded in - at most two full refreshes and a fade (the first fill, then the stalest-first round), at most
## RESET_ANGLE_RAD (a staler slot is dropped).
func _warp_checks() -> void:
	var frames_all := ceili(float(CloudLightCacheSchedule.voxel_count()) / CloudLightCacheSchedule.BUDGET_VOXELS)
	var fade_frames := ceili(CloudLightCacheSchedule.CROSSFADE_S / DT)
	for warp: float in [100.0, 1000.0]:
		var r := _scenario(warp, 0.0, SCENARIO_S)
		var rate := EARTH_RATE_RAD_S * warp
		var bound := minf((2 * frames_all + fade_frames) * DT * rate + CloudLightCacheSchedule.REFRESH_ANGLE_RAD,
				CloudLightCacheSchedule.RESET_ANGLE_RAD)
		var worst: Array = r["worst_deg"]
		var ok: bool = r["within_budget"]
		for w: float in worst:
			ok = ok and w <= rad_to_deg(bound) + 1e-6
		for f: int in r["first_hold"]:
			ok = ok and f >= 0 and f <= _seed_frames() + SEED_SLACK_FRAMES
		_check(ok, "%dx warp: every cascade's shown light within %.1f deg of the Sun (worst %s deg; ground point held %s "
				% [warp, rad_to_deg(bound), _fmt(worst), _fmt(r["held_share"])] + "of the frames; kept after first hold %s)"
				% [r["held_after_first"]])
		if warp == 100.0:  # below RESET: once held, the ground point stays held (no drops)
			_check(_all_true(r["held_after_first"]), "100x warp: every cascade keeps its ground point once held (%s)"
					% [r["held_after_first"]])


## A moving camera (Sun fixed) at 1, 2 and 7.8 km/s for SCENARIO_S: at 1 and 2 km/s every cascade holds its ground
## point within the seed's frames and then keeps it (each re-anchor's slab filled before the next); at 7.8 km/s (the
## re-anchor slabs of all cascades need ~1.8x the budget: the fine ones fall behind, the march or the next cascade takes
## their samples) the coarse cascades 2-4 do.
func _flight_checks() -> void:
	var seed_frames := _seed_frames()
	for speed: float in [1000.0, 2000.0, 7800.0]:
		var r := _scenario(0.0, speed, SCENARIO_S)
		var first: Array = r["first_hold"]
		var keep: Array = r["held_after_first"]
		var from := 0 if speed < FAST_FLIGHT_MPS else 2
		var ok: bool = r["within_budget"] and _all_true(keep.slice(from))
		for f: int in first.slice(from):
			ok = ok and f >= 0 and f <= seed_frames + SEED_SLACK_FRAMES
		_check(ok, "camera at %.1f km/s: first held at frames %s, ground point kept %s (held %s of the frames)"
				% [speed / 1000.0, first, keep, _fmt(r["held_share"])])


## Runs a fresh schedule for `seconds` (the Sun turning about the polar axis at `warp` x Earth's rate, the camera moving
## east along the equator at speed_mps). Per cascade: worst_deg (the shown light's worst angle to the Sun), first_hold
## (the first frame whose shown slots hold the ground point), held_share (frames holding it), held_after_first (held in
## every frame since); within_budget.
func _scenario(warp: float, speed_mps: float, seconds: float) -> Dictionary:
	var s := CloudLightCacheSchedule.new()
	var n := CloudLightCacheSchedule.CASCADES.size()
	var worst: Array[float] = []
	var first: Array[int] = []
	var held_frames: Array[int] = []
	var kept: Array[bool] = []
	for k in range(n):
		worst.append(0.0)
		first.append(-1)
		held_frames.append(0)
		kept.append(true)
	var within := true
	var frames := int(seconds / DT)
	for f in range(frames):
		var t := f * DT
		var ground := _ground.rotated(Vector3.UP, speed_mps * t / CloudLayer.WGS84_A_M)
		var sun := _sun.rotated(Vector3.UP, EARTH_RATE_RAD_S * warp * t)
		var jobs := s.update(ground, sun, 0.0, false, DT)
		within = within and _volume(jobs) <= s.budget_voxels
		var state := s.lookup_state()
		for k in range(n):
			var cascade: Dictionary = state["cascades"][k]
			var g: Vector2 = cascade["centre"]
			var held := false
			for slot in range(2):
				var w: float = cascade["weight"] if slot == 1 else 1.0 - float(cascade["weight"])
				var valid: Rect2i = cascade["valid"][slot]
				if w <= 0.0 or not valid.has_area():
					continue
				worst[k] = maxf(worst[k], rad_to_deg(sun.angle_to(cascade["lights"][slot])))
				held = held or valid.has_point(Vector2i(floori(g.x), floori(g.y)))
			if held:
				held_frames[k] += 1
				if first[k] < 0:
					first[k] = f
			elif first[k] >= 0:
				kept[k] = false
	var share: Array[float] = []
	for k in range(n):
		share.append(float(held_frames[k]) / frames)
	return {"worst_deg": worst, "first_hold": first, "held_share": share, "held_after_first": kept,
			"within_budget": within}


## The frames until the seed squares of all cascades are filled (they share the budget equally).
static func _seed_frames() -> int:
	var seed := 0
	for c: Dictionary in CloudLightCacheSchedule.CASCADES:
		seed += CloudLightCacheSchedule.STRIP_VOXELS * CloudLightCacheSchedule.STRIP_VOXELS * (c["size"] as Vector3i).z
	return ceili(float(seed) / CloudLightCacheSchedule.BUDGET_VOXELS)


## The rectangle each cascade's shown slots hold (the front's; empty: none).
static func _held(s: CloudLightCacheSchedule) -> Array[Rect2i]:
	var held: Array[Rect2i] = []
	for cascade: Dictionary in s.lookup_state()["cascades"]:
		var slot := 1 if float(cascade["weight"]) >= 0.5 else 0
		held.append(cascade["valid"][slot])
	return held


static func _all_true(a: Array) -> bool:
	for v: bool in a:
		if not v:
			return false
	return true


static func _fmt(a: Array) -> String:
	var parts := PackedStringArray()
	for v: float in a:
		parts.append("%.2f" % v)
	return "[" + ", ".join(parts) + "]"


## Updates the schedule every DT until it schedules nothing and is complete. Returns the jobs, their volume, frames,
## whether every frame stayed within the budget, and (watch_fade) whether cascade 0's old generation was shown alone
## while its new one filled and the fade then ran monotonically; (watch_c0) whether cascade 0 first holds a point in the
## frame of its last fill job (a state holds what that frame's jobs filled: they run before the lookup).
func _drain(ground: Vector3, light: Vector3, blend: float, watch_fade := false, watch_c0 := false) -> Dictionary:
	var all: Array[Dictionary] = []
	var volume := 0
	var within := true
	var frames := 0
	var old_kept := true
	var monotonic := true
	var fade_frames := 0
	var last_weight := -1.0
	var first_hold := -1
	var grows := true
	var last_held := Rect2i()
	while frames < MAX_FRAMES:
		var jobs := _schedule.update(ground, light, blend, false, DT)
		var v := _volume(jobs)
		within = within and v <= _schedule.budget_voxels
		volume += v
		all.append_array(jobs)
		var cascade: Dictionary = _schedule.lookup_state()["cascades"][0]
		var weight: float = cascade["weight"]
		var c0_jobs := false
		for job: Dictionary in jobs:
			c0_jobs = c0_jobs or int(job["cascade"]) == 0
		if watch_fade:
			if c0_jobs and weight > 0.0:
				old_kept = false  # the new generation shown while still filling
			if weight > 0.0 and weight < 1.0:
				fade_frames += 1
			if last_weight >= 0.0 and weight < last_weight:
				monotonic = false
			last_weight = weight
		if watch_c0:
			var held: Rect2i = cascade["valid"][0] if (cascade["valid"][0] as Rect2i).has_area() else cascade["valid"][1]
			if held.has_area() and first_hold < 0:
				first_hold = frames
				var g: Vector2 = cascade["centre"]
				grows = held.has_point(Vector2i(floori(g.x), floori(g.y)))
			grows = grows and (not last_held.has_area() or held.encloses(last_held))
			last_held = held
		frames += 1
		if jobs.is_empty() and _schedule.complete():
			break
	var expected_fade := int(CloudLightCacheSchedule.CROSSFADE_S / DT)
	return {"jobs": all, "volume": volume, "within_budget": within, "frames": frames - 1,
			"complete": _schedule.complete(), "old_kept": old_kept,
			"fade_monotonic": monotonic and absi(fade_frames - expected_fade) <= 2, "fade_frames": fade_frames,
			"c0_grows": first_hold > 0 and grows, "c0_first_hold": first_hold}


func _volume(jobs: Array[Dictionary]) -> int:
	var v := 0
	for job: Dictionary in jobs:
		v += int((job["region"] as AABB).get_volume())
	return v


func _nothing_held() -> bool:
	for cascade: Dictionary in _schedule.lookup_state()["cascades"]:
		for r: Rect2i in cascade["valid"]:
			if r.has_area():
				return false
	return true


func _east() -> Vector3:
	return Vector3.UP.cross(CloudLightCacheSchedule.geodetic_up(_ground)).normalized()


func _check(ok: bool, what: String) -> void:
	if ok:
		print("  ok    %s" % what)
	else:
		_failures += 1
		printerr("  FAIL  %s" % what)
