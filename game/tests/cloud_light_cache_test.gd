# The light cache's schedule (static clouds Task 2; CloudLightCacheSchedule: what to fill next - pure logic, no GPU).
# Headless: godot --headless --path game --script res://tests/cloud_light_cache_test.gd
# Checked: the initial fill completes and every frame's jobs stay within the voxel budget; a camera moving 1 km inside
# the central part schedules nothing, leaving it schedules exactly the uncovered slab (toroidal: the rest stays); the
# Sun turning 0.1 deg schedules nothing, 0.3 deg a full new generation in the other slot, shown only once complete and
# then cross-faded monotonically over CROSSFADE_S; a blend step above REFRESH_BLEND_STEP schedules a refill; a clock
# jump (and a Sun jump beyond RESET_ANGLE_RAD) re-anchors every cascade and leaves them invalid until filled (the
# lookup then falls back to the per-sample march).
extends SceneTree

const DT := 1.0 / 90.0
const MAX_FRAMES := 4000

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
	_check(after_jump["complete"] and after_jump["c0_invalid_until_filled"],
			"after the jump: cascade 0 stays invalid until its fill is complete, then everything completes")

	# A Sun jump beyond RESET_ANGLE_RAD (no clock jump flagged): invalid at once (no stale light shown).
	_schedule.update(far_site, _sun.rotated(axis, CloudLightCacheSchedule.RESET_ANGLE_RAD * 1.5), 0.06, false, DT)
	_check(_nothing_held(), "Sun jump beyond RESET_ANGLE_RAD: every cascade invalid until filled")

	# The budget is respected for any budget (here 1/16 of the voxels: complete after 16 frames).
	_schedule.budget_voxels = _schedule.voxel_count() / 16
	_schedule.update(far_site, _sun, 0.06, true, DT)
	var sixteenth := _drain(far_site, _sun, 0.06)
	_check(sixteenth["complete"] and sixteenth["within_budget"] and sixteenth["frames"] + 1 == 16,
			"budget 1/16 of the voxels: complete after 16 frames (%d)" % (sixteenth["frames"] + 1))

	if _failures == 0:
		print("CLOUD LIGHT CACHE TEST PASSED")
	else:
		printerr("CLOUD LIGHT CACHE TEST FAILED: %d" % _failures)
	quit(1 if _failures > 0 else 0)


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
	var last_c0_jobs := -1
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
			var holds: bool = (cascade["valid"][0] as Rect2i).has_area() or (cascade["valid"][1] as Rect2i).has_area()
			if holds and first_hold < 0:
				first_hold = frames
			if c0_jobs:
				last_c0_jobs = frames
		frames += 1
		if jobs.is_empty() and _schedule.complete():
			break
	var expected_fade := int(CloudLightCacheSchedule.CROSSFADE_S / DT)
	return {"jobs": all, "volume": volume, "within_budget": within, "frames": frames - 1,
			"complete": _schedule.complete(), "old_kept": old_kept,
			"fade_monotonic": monotonic and absi(fade_frames - expected_fade) <= 2, "fade_frames": fade_frames,
			"c0_invalid_until_filled": first_hold >= 0 and first_hold == last_c0_jobs}


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
