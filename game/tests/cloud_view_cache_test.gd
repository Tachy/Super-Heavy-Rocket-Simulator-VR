# Headless test of the cloud view cache's pure logic: CloudViewCacheLayout (cube faces, tiles, resolution from the
# renderer's projection) and CloudViewCacheSchedule (staleness, priority, the per-frame pick under a budget).
# Run: godot --headless --path game --script res://tests/cloud_view_cache_test.gd
extends SceneTree

## cloud_vr_stereo_test.gd's headset (Meta Quest 2 runtime, XrFovf of the left eye, rad) and the run's per-eye target.
const FOV_LEFT_EYE := {"left": -0.9424778, "right": 0.6981317, "up": 0.8203047, "down": -0.8377581}
const VIEW_SIZE := Vector2i(3534, 2838)
const FAR_M := 3.0e7
const TILE_TEXELS := 64
## The view rays a caller passes: a grid every 10 deg, each ray covering half the grid diagonal.
## A pick returns at most MAX_BLOCK_EXPANSIONS blocks' tiles (128): the first pick of a cold cache asks for fewer.
const FIRST_PICK_TILES := 120
const VIEW_STEP_RAD := 0.17453293
const VIEW_SAMPLE_RAD := 0.13089969
const ROUND_TRIP_TOLERANCE_RAD := 1.0e-9

var _failures := 0
var _rng := RandomNumberGenerator.new()


func _initialize() -> void:
	_rng.seed = 20261027
	_test_round_trip()
	_test_tiles()
	_test_face_texels_for()
	_test_first_fill()
	_test_light_staleness()
	_test_parallax()
	_test_budget_and_levels()
	_test_timing()
	print("cloud_view_cache_test: %s" % ("PASS" if _failures == 0 else "FAIL (%d)" % _failures))
	quit(1 if _failures > 0 else 0)


func _layout(face_texels := 3200) -> CloudViewCacheLayout:
	return CloudViewCacheLayout.new(face_texels, TILE_TEXELS)


func _random_dir() -> Vector3:
	return Vector3(_rng.randfn(), _rng.randfn(), _rng.randfn()).normalized()


## Random directions plus every face edge and corner (the major-axis ties) and the axes.
func _test_dirs() -> Array[Vector3]:
	var dirs: Array[Vector3] = []
	for i in range(1000):
		dirs.append(_random_dir())
	for sx in [-1.0, 0.0, 1.0]:
		for sy in [-1.0, 0.0, 1.0]:
			for sz in [-1.0, 0.0, 1.0]:
				if sx != 0.0 or sy != 0.0 or sz != 0.0:
					dirs.append(Vector3(sx, sy, sz).normalized())
	return dirs


func _test_round_trip() -> void:
	var layout := _layout()
	var worst := 0.0
	for d in _test_dirs():
		var t := layout.texel_of(d)
		var back := layout.dir_of(int(t.z), Vector2(t.x, t.y) / float(layout.face_texels))
		worst = maxf(worst, back.angle_to(d))
	_check(worst < ROUND_TRIP_TOLERANCE_RAD, "dir_of(texel_of(d)) = d for 1000 random + 26 edge / corner / axis "
			+ "directions (worst %s rad)" % String.num_scientific(worst))


func _test_tiles() -> void:
	var layout := _layout()
	var in_range := true
	for d in _test_dirs():
		var tile := layout.tile_of(d)
		in_range = in_range and tile >= 0 and tile < layout.tile_count()
	_check(in_range, "every direction maps to a tile in 0..%d" % layout.tile_count())
	# One tile per direction, and the tiles partition the sphere: every tile's centre maps back to that very tile.
	var identity := true
	for tile in range(layout.tile_count()):
		identity = identity and layout.tile_of(layout.tile_dir(tile)) == tile
	_check(identity, "each tile's centre direction lies in that tile (%d tiles)" % layout.tile_count())
	var sizes_ok := layout.level_texels(0) == TILE_TEXELS / 4 and layout.level_texels(1) == TILE_TEXELS / 2 \
			and layout.level_texels(2) == TILE_TEXELS
	_check(sizes_ok, "level texels: 1/4, 1/2, full of the tile")
	# Texel density: the face centre (the coarsest point of a face) has face_texels / 2 texels per radian.
	var centre := layout.texel_of(Vector3(0.0, 0.0, -1.0))
	var next := layout.texel_of(Vector3(1.0e-4, 0.0, -1.0).normalized())
	var per_rad := absf(next.x - centre.x) / 1.0e-4
	_check(absf(per_rad - layout.face_texels / 2.0) < 1.0, "face centre density %.1f texels/rad = face_texels / 2"
			% per_rad)


## The run's projection: the left eye of cloud_vr_stereo_test.gd's headset (asymmetric frustum, RD depth correction).
func _run_projection() -> Projection:
	var near_m := 1.0
	var frustum := Projection.create_frustum(tan(FOV_LEFT_EYE["left"]) * near_m, tan(FOV_LEFT_EYE["right"]) * near_m,
			tan(FOV_LEFT_EYE["down"]) * near_m, tan(FOV_LEFT_EYE["up"]) * near_m, near_m, FAR_M)
	return Projection.create_depth_correction(true) * frustum


func _test_face_texels_for() -> void:
	var face := CloudViewCacheLayout.face_texels_for(_run_projection(), VIEW_SIZE, TILE_TEXELS)
	_check(absf(face - 3200.0) <= 320.0 and face % TILE_TEXELS == 0,
			"face_texels_for the run's projection: %d (3200 +- 10 %%, a multiple of %d)" % [face, TILE_TEXELS])
	var finer := CloudViewCacheLayout.face_texels_for(_run_projection(), VIEW_SIZE * 2, TILE_TEXELS)
	_check(absf(finer - 2.0 * face) <= 2.0 * TILE_TEXELS, "twice the pixels, twice the face (%d)" % finer)


## A grid of view rays over a cone of half-angle `half_rad` around -Z (spacing `step_rad`).
func _view_grid(half_rad: float, step_rad: float) -> Array[Vector3]:
	var dirs: Array[Vector3] = []
	var n := int(half_rad / step_rad)
	for ix in range(-n, n + 1):
		for iy in range(-n, n + 1):
			var d := Vector3(tan(ix * step_rad), tan(iy * step_rad), -1.0).normalized()
			if d.angle_to(Vector3(0.0, 0.0, -1.0)) <= half_rad:
				dirs.append(d)
	return dirs


func _state(sun_dir: Vector3, weight := 0.5, generation := 1, anchor := Vector3(1.0e6, 2.0e6, 3.0e6)) -> Dictionary:
	return {"anchor_earth_m": anchor, "sun_dir": sun_dir, "weather_weight": weight, "light_generation": generation}


func _complete_all(schedule: CloudViewCacheSchedule, level: int, state: Dictionary) -> void:
	for tile in range(schedule.layout.tile_count()):
		schedule.completed(tile, level, state)


func _test_first_fill() -> void:
	var layout := _layout()
	var schedule := CloudViewCacheSchedule.new(layout)
	schedule.invalidate_all()
	var view := _view_grid(deg_to_rad(50.0), VIEW_STEP_RAD)
	var sample_rad := VIEW_SAMPLE_RAD
	var anchor := Vector3(1.0e6, 2.0e6, 3.0e6)
	var started := Time.get_ticks_usec()
	var picks := schedule.pick(anchor, anchor, view, FIRST_PICK_TILES, sample_rad)
	var pick_ms := (Time.get_ticks_usec() - started) / 1000.0
	var all_level_0 := picks.size() == FIRST_PICK_TILES
	var in_margin := true
	var reach := deg_to_rad(50.0) + CloudViewCacheSchedule.VIEW_MARGIN_RAD + sample_rad + layout.tile_radius_rad()
	var distinct := {}
	for p in picks:
		all_level_0 = all_level_0 and p["level"] == 0
		in_margin = in_margin and layout.tile_dir(p["tile"]).angle_to(Vector3(0.0, 0.0, -1.0)) <= reach
		distinct[p["tile"]] = true
	_check(all_level_0 and distinct.size() == picks.size(), "after invalidate_all the pick is %d distinct level-0 tiles"
			% picks.size())
	_check(in_margin, "the first picks lie in the view plus its margin")
	# The view's own tiles come before any tile of the margin ring.
	var view_tiles := {}
	for d in view:
		view_tiles[layout.tile_of(d)] = true
	var first_outside := picks.size()
	var last_inside := -1
	for i in range(picks.size()):
		var visible := false
		for d in view:
			if layout.tile_dir(picks[i]["tile"]).angle_to(d) <= sample_rad + layout.tile_radius_rad():
				visible = true
				break
		if visible:
			last_inside = i
		elif first_outside == picks.size():
			first_outside = i
	_check(last_inside < first_outside, "tiles of the view come before the margin ring (last visible %d, first margin %d)"
			% [last_inside, first_outside])
	# Everything in the view and margin gets level 0 before anything refines: while an in-margin tile lacks level 0
	# (and is not in the batch itself at level 0), no pick may carry a level above 0.
	var margin_reach := sample_rad + layout.tile_radius_rad() + CloudViewCacheSchedule.VIEW_MARGIN_RAD
	var margin_tiles: Array[int] = []
	for tile in range(layout.tile_count()):
		if _max_angle_dot(layout.tile_dir(tile), view) >= cos(margin_reach):
			margin_tiles.append(tile)
	var guard := 0
	var refined_early := false
	var rounds_with_open_margin := 0
	while guard < 200:
		guard += 1
		var batch := schedule.pick(anchor, anchor, view, 400, sample_rad)
		var batch_level_0 := {}
		for p in batch:
			if p["level"] == 0:
				batch_level_0[p["tile"]] = true
		var lacking := 0
		for tile in margin_tiles:
			if schedule.level_of(tile) < 0 and not batch_level_0.has(tile):
				lacking += 1
		if lacking > 0:
			rounds_with_open_margin += 1
			for p in batch:
				refined_early = refined_early or p["level"] > 0
		for tile in batch_level_0:
			schedule.completed(tile, 0, _state(Vector3.UP))
		if lacking == 0:
			break
	_check(not refined_early and rounds_with_open_margin > 1 and guard < 200,
			"no pick refines while the view and margin (%d tiles) lack level 0 (%d such rounds)"
			% [margin_tiles.size(), rounds_with_open_margin])
	var refine := schedule.pick(anchor, anchor, view, 50, sample_rad)
	var all_up := not refine.is_empty()
	for p in refine:
		all_up = all_up and p["level"] == schedule.level_of(p["tile"]) + 1
	_check(all_up, "then visible tiles go up one level at a time")
	print("  info  pick over %d tiles, %d view rays: %.1f ms" % [layout.tile_count(), view.size(), pick_ms])


func _test_light_staleness() -> void:
	var layout := _layout()
	var schedule := CloudViewCacheSchedule.new(layout)
	var sun := Vector3(0.3, 0.8, -0.5).normalized()
	_complete_all(schedule, CloudViewCacheLayout.LEVELS - 1, _state(sun))
	_settle(schedule, sun, 0.5, 1)
	_check(_stale_count(schedule) == 0, "same light: no tile stale")
	var axis := sun.cross(Vector3.UP).normalized()
	_settle(schedule, sun.rotated(axis, deg_to_rad(0.05)), 0.5, 1)
	_check(_stale_count(schedule) == 0, "Sun 0.05 deg off: no tile stale (%d)" % _stale_count(schedule))
	_settle(schedule, sun.rotated(axis, deg_to_rad(0.5)), 0.5, 1)
	_check(_stale_count(schedule) == layout.tile_count(), "Sun 0.5 deg off: all %d tiles stale (%d)"
			% [layout.tile_count(), _stale_count(schedule)])
	_settle(schedule, sun, 0.5, 1)
	_check(_stale_count(schedule) == 0, "Sun back: not stale")
	_settle(schedule, sun, 0.51, 1)
	_check(_stale_count(schedule) == 0, "weather weight 0.01 off: not stale")
	_settle(schedule, sun, 0.53, 1)
	_check(_stale_count(schedule) == layout.tile_count(), "weather weight 0.03 off: all stale")
	_settle(schedule, sun, 0.5, 2)
	_check(_stale_count(schedule) == layout.tile_count(), "another light cache generation: all stale")
	# A stale tile is picked (class 4) at its own level; completing it with the new light clears the flag.
	var anchor := Vector3(1.0e6, 2.0e6, 3.0e6)
	var picks := schedule.pick(anchor, anchor, [Vector3(0.0, 0.0, -1.0)], 5, deg_to_rad(10.0))
	var level_kept := not picks.is_empty()
	for p in picks:
		level_kept = level_kept and p["level"] == CloudViewCacheLayout.LEVELS - 1
	_check(level_kept, "stale tiles are re-marched at their level")
	schedule.completed(picks[0]["tile"], 2, _state(sun, 0.5, 2))
	_check(not schedule.is_stale(picks[0]["tile"]), "a tile completed with the current light is not stale")
	var moved := Vector3(anchor.x + 50.0, anchor.y, anchor.z)
	_settle(schedule, sun, 0.5, 2)
	schedule.pick(moved, moved, [Vector3(0.0, 0.0, -1.0)], 1, 0.1)
	_check(schedule.is_stale(picks[0]["tile"]), "a new anchor makes a tile rendered from the old one stale")


## mark_light until the amortised light pass has finished (the staleness flags are complete).
func _settle(schedule: CloudViewCacheSchedule, sun: Vector3, weight: float, generation: int) -> void:
	for i in range(1000):
		schedule.mark_light(sun, weight, generation)
		if not schedule.light_scan_pending():
			return
	_check(false, "the light pass finishes")


func _stale_count(schedule: CloudViewCacheSchedule) -> int:
	var n := 0
	for tile in range(schedule.layout.tile_count()):
		n += 1 if schedule.is_stale(tile) else 0
	return n


func _test_parallax() -> void:
	var layout := _layout(4032)  # 63 tiles of 64: 2016 texels per radian at a face centre (35.2 px/deg)
	var schedule := CloudViewCacheSchedule.new(layout)
	var tile := layout.tile_of(Vector3(0.1, 0.2, -1.0))
	var dir := layout.tile_dir(tile)
	var anchor := Vector3(1.0e6, 2.0e6, 3.0e6)
	schedule.set_tile_distance(tile, 2000.0)
	var side := dir.cross(Vector3.UP).normalized()
	var error := schedule.parallax_error_px(tile, anchor + side * 10.0, anchor)
	_check(absf(error / 10.0 - 1.0) <= 0.05, "parallax error: cloud at 2 km, camera 10 m off: %.2f px (~10)" % error)
	var along := schedule.parallax_error_px(tile, anchor + dir * 10.0, anchor)
	_check(along < 1.0e-6, "parallax error: an offset along the view direction makes none (%s)"
			% String.num_scientific(along))
	_check(schedule.parallax_error_px(tile + 1, anchor + side * 10.0, anchor) == 0.0,
			"parallax error: no known cloud distance, no error")
	# A visible tile over the threshold is re-marched ahead of the finer levels, nearer first.
	var all := _layout()
	var s2 := CloudViewCacheSchedule.new(all)
	_complete_all(s2, 2, _state(Vector3.UP, 0.5, 1, anchor))
	_settle(s2, Vector3.UP, 0.5, 1)
	var near_tile := all.tile_of(Vector3(0.0, 0.0, -1.0))
	var far_tile := all.tile_of(Vector3(0.05, 0.0, -1.0))
	s2.set_tile_distance(near_tile, 500.0)
	s2.set_tile_distance(far_tile, 5000.0)
	# The anchor moves 5 m: the tiles rendered from the old one show that offset (the other tiles are merely stale).
	var moved := anchor + Vector3(0.0, 5.0, 0.0)
	var picks := s2.pick(moved, moved, [Vector3(0.0, 0.0, -1.0)], 10, deg_to_rad(10.0))
	_check(picks.size() >= 2 and picks[0]["tile"] == near_tile and picks[1]["tile"] == far_tile
			and picks[0]["level"] == 2,
			"visible tiles over %.1f px: the nearer cloud first, at its level (%s)"
			% [CloudViewCacheSchedule.PARALLAX_REFRESH_PX, str(picks.slice(0, 2))])
	_test_parallax_reanchor()


## Class 2 compares the tile's rendered anchor with the CURRENT anchor: a camera off a fixed anchor changes nothing
## (refilling from the same anchor cannot reduce it); moving the anchor makes the old tiles picked.
func _test_parallax_reanchor() -> void:
	var layout := _layout()
	var schedule := CloudViewCacheSchedule.new(layout)
	var anchor := Vector3(1.0e6, 2.0e6, 3.0e6)
	_complete_all(schedule, 2, _state(Vector3.UP, 0.5, 1, anchor))
	_settle(schedule, Vector3.UP, 0.5, 1)
	var view := _view_grid(deg_to_rad(30.0), VIEW_STEP_RAD)
	var tile := layout.tile_of(Vector3(0.14, 0.0, -1.0))  # ~8 deg off the view centre: nothing nearer to the centre
	schedule.set_tile_distance(tile, 500.0)
	var camera := anchor + Vector3(50.0, 0.0, 0.0)
	var picks := schedule.pick(camera, anchor, view, 100, VIEW_SAMPLE_RAD)
	_check(picks.is_empty(), "converged cache, camera 50 m off the anchor: nothing to march (%d picks)" % picks.size())
	var camera_error := schedule.parallax_error_px(tile, camera, anchor)
	_check(camera_error > 100.0, "parallax_error_px still reports the camera's offset from an anchor (%.0f px)"
			% camera_error)
	picks = schedule.pick(camera, camera, view, 100, VIEW_SAMPLE_RAD)
	_check(not picks.is_empty() and picks[0]["tile"] == tile and picks[0]["level"] == 2,
			"anchor moved 50 m: the tile rendered from the old anchor comes first, at its level (%s)"
			% str(picks.slice(0, 1)))
	schedule.completed(tile, 2, _state(Vector3.UP, 0.5, 1, camera))
	picks = schedule.pick(camera, camera, view, 100, VIEW_SAMPLE_RAD)
	var again := false
	for p in picks:
		again = again or p["tile"] == tile
	_check(not again, "the tile re-marched from the new anchor is not picked again")


func _test_budget_and_levels() -> void:
	var layout := _layout()
	var schedule := CloudViewCacheSchedule.new(layout)
	var anchor := Vector3(1.0e6, 2.0e6, 3.0e6)
	var view := _view_grid(deg_to_rad(40.0), VIEW_STEP_RAD)
	var ok := true
	for budget in [0, 1, 7, 64, 1000, 100000]:
		var picks := schedule.pick(anchor, anchor, view, budget, VIEW_SAMPLE_RAD)
		ok = ok and picks.size() <= budget and (budget == 0 or not picks.is_empty())
	_check(ok, "pick never returns more than budget_tiles")
	# Tiles complete over frames; at the end nothing is left to do.
	var frames := 0
	while frames < 2000:
		var picks := schedule.pick(anchor, anchor, view, 2000, VIEW_SAMPLE_RAD)
		if picks.is_empty():
			break
		for p in picks:
			schedule.completed(p["tile"], p["level"], _state(Vector3.UP))
		frames += 1
	_check(frames < 2000 and schedule.level_of(layout.tile_of(Vector3(0.0, 0.0, -1.0))) == 2,
			"repeated picks end with every tile at the full level (%d rounds)" % frames)
	var all_full := true
	for tile in range(layout.tile_count()):
		all_full = all_full and schedule.level_of(tile) == CloudViewCacheLayout.LEVELS - 1
	_check(all_full, "all %d tiles reached the full level" % layout.tile_count())


## [E] Main-thread budgets (the 5 ms main-thread target is shared by many sections), per frame, every frame: a
## converged cache's pick plus mark_light, and a cold start / jump frame.
const WARM_BUDGET_MS := 1.0
const COLD_BUDGET_MS := 10.0
const TIMING_CALLS := 20
## Tiles one frame may march (each costs the caller a GPU march and pick() a result entry).
const FRAME_BUDGET_TILES := 128
const SUN_SWEEP_FRAMES := 200
const SUN_SWEEP_DEG := 0.05
## Headless timing noise: at most this many frames, each above OUTLIER_FACTOR x the median, are dropped from a maximum
## (and reported).
const MAX_OUTLIERS := 2
const OUTLIER_FACTOR := 3.0


## The run's layout: the face the projection asks for.
func _run_layout() -> CloudViewCacheLayout:
	return _layout(CloudViewCacheLayout.face_texels_for(_run_projection(), VIEW_SIZE, TILE_TEXELS))


func _test_timing() -> void:
	var layout := _run_layout()
	var anchor := Vector3(1.0e6, 2.0e6, 3.0e6)
	var view := _view_grid(deg_to_rad(50.0), VIEW_STEP_RAD)  # ~50 deg half-angle
	var sun := Vector3(0.3, 0.8, -0.5).normalized()
	var schedule := CloudViewCacheSchedule.new(layout)
	var cold: Array[float] = []
	var cold_count := 0
	for i in range(TIMING_CALLS):
		var started := Time.get_ticks_usec()
		var picks := schedule.pick(anchor, anchor, view, FRAME_BUDGET_TILES, VIEW_SAMPLE_RAD)
		cold.append((Time.get_ticks_usec() - started) / 1000.0)
		cold_count = picks.size()
	var cold_ms := _median(cold)
	_check(cold_count == FRAME_BUDGET_TILES and cold_ms <= COLD_BUDGET_MS, "cold pick (%d tiles open, %d view rays, budget 128): "
			% [layout.tile_count(), view.size()] + "median %.2f ms <= %.0f ms" % [cold_ms, COLD_BUDGET_MS])
	var converged := _converged_schedule(layout, sun, anchor)
	var axis := sun.cross(Vector3.UP).normalized()
	var warm: Array[float] = []
	var warm_empty := true
	for i in range(TIMING_CALLS):
		var started := Time.get_ticks_usec()
		# The Sun turns a little every frame (well below a pass's step in total), the head moves.
		converged.mark_light(sun.rotated(axis, 1.0e-5 * i), 0.5, 1)
		var picks := converged.pick(anchor + Vector3(0.0, 0.1 * i, 0.0), anchor, view, FRAME_BUDGET_TILES, VIEW_SAMPLE_RAD)
		warm.append((Time.get_ticks_usec() - started) / 1000.0)
		warm_empty = warm_empty and picks.is_empty()
	var warm_ms := _median(warm)
	_check(warm_empty and warm_ms <= WARM_BUDGET_MS, "warm pick + mark_light on a converged cache: median %.3f ms <= "
			% warm_ms + "%.1f ms (nothing to march: %s)" % [WARM_BUDGET_MS, str(warm_empty)])
	_test_sun_sweep(layout, view, sun, anchor)
	_test_reanchor_timing(layout, view, sun, anchor)
	_test_cold_start_timing(layout, view, anchor)


## A schedule on the layout with every tile at the full level, a known cloud distance everywhere, the light settled
## and the anchor adopted (the first pick, untimed).
func _converged_schedule(layout: CloudViewCacheLayout, sun: Vector3, anchor: Vector3) -> CloudViewCacheSchedule:
	var schedule := CloudViewCacheSchedule.new(layout)
	_complete_all(schedule, CloudViewCacheLayout.LEVELS - 1, _state(sun, 0.5, 1, anchor))
	for tile in range(layout.tile_count()):
		schedule.set_tile_distance(tile, 2000.0 + tile)
	_settle(schedule, sun, 0.5, 1)
	schedule.pick(anchor, anchor, [Vector3(0.0, 0.0, -1.0)], 1, 0.1)
	return schedule


## The maximum of the frame times after dropping up to MAX_OUTLIERS frames above OUTLIER_FACTOR x the median; the
## dropped ones are printed.
func _max_without_outliers(times: Array[float], what: String) -> float:
	var median := _median(times)
	var sorted := times.duplicate()
	sorted.sort()
	var dropped: Array[float] = []
	while dropped.size() < MAX_OUTLIERS and sorted.back() > OUTLIER_FACTOR * median and sorted.back() > 1.0:
		dropped.append(sorted.pop_back())
	print("  info  %s: median %.3f ms, max %.3f ms, excluded outliers %s" % [what, median, sorted.back(), str(dropped)])
	return sorted.back()


## (a) A Sun turning 0.05 deg per frame on a converged cache: the light passes run back to back and the stale tiles
## come back as picks (completed in the same frame, untimed); every frame's mark_light + pick stays in the bound.
func _test_sun_sweep(layout: CloudViewCacheLayout, view: Array[Vector3], sun: Vector3, anchor: Vector3) -> void:
	var schedule := _converged_schedule(layout, sun, anchor)
	var axis := sun.cross(Vector3.UP).normalized()
	var times: Array[float] = []
	var picked_total := 0
	for frame in range(1, SUN_SWEEP_FRAMES + 1):
		var turned := sun.rotated(axis, deg_to_rad(SUN_SWEEP_DEG) * frame)
		var started := Time.get_ticks_usec()
		schedule.mark_light(turned, 0.5, 1)
		var picks := schedule.pick(anchor, anchor, view, FRAME_BUDGET_TILES, VIEW_SAMPLE_RAD)
		times.append((Time.get_ticks_usec() - started) / 1000.0)
		picked_total += picks.size()
		for p in picks:
			schedule.completed(p["tile"], p["level"], _state(turned, 0.5, 1, anchor))
	var worst := _max_without_outliers(times, "Sun sweep %.2f deg/frame, %d frames" % [SUN_SWEEP_DEG, SUN_SWEEP_FRAMES])
	_check(worst <= WARM_BUDGET_MS and picked_total > 0, "Sun sweep: every frame's mark_light + pick <= %.1f ms "
			% WARM_BUDGET_MS + "(max %.3f ms, %d stale tiles re-marched)" % [worst, picked_total])


## (b) A re-anchor by 50 m on a converged cache: every picked tile is re-marched in the same frame until nothing is
## left, then 50 more frames; the converged frames pick nothing.
func _test_reanchor_timing(layout: CloudViewCacheLayout, view: Array[Vector3], sun: Vector3,
		anchor: Vector3) -> void:
	var schedule := _converged_schedule(layout, sun, anchor)
	var moved := anchor + Vector3(50.0, 0.0, 0.0)
	var times: Array[float] = []
	var frames := 0
	var converged_frames := 0
	var converged_empty := true
	var marched := 0
	while converged_frames < 50 and frames < 5000:
		frames += 1
		var started := Time.get_ticks_usec()
		schedule.mark_light(sun, 0.5, 1)
		var picks := schedule.pick(moved, moved, view, FRAME_BUDGET_TILES, VIEW_SAMPLE_RAD)
		times.append((Time.get_ticks_usec() - started) / 1000.0)
		marched += picks.size()
		if picks.is_empty():
			converged_frames += 1
		else:
			converged_empty = converged_empty and converged_frames == 0
		for p in picks:
			schedule.completed(p["tile"], p["level"], _state(sun, 0.5, 1, moved))
	var worst := _max_without_outliers(times, "re-anchor 50 m, %d frames (%d tiles re-marched)" % [frames, marched])
	_check(worst <= WARM_BUDGET_MS and converged_frames == 50 and converged_empty and marched >= layout.tile_count(),
			"re-anchor: every frame <= %.1f ms (max %.3f ms), then %d converged frames pick nothing"
			% [WARM_BUDGET_MS, worst, converged_frames])


## (c) A cold start (invalidate_all) with partial fills each frame (half of each pick completes) until converged.
func _test_cold_start_timing(layout: CloudViewCacheLayout, view: Array[Vector3], anchor: Vector3) -> void:
	var sun := Vector3(0.3, 0.8, -0.5).normalized()
	var schedule := _converged_schedule(layout, sun, anchor)
	schedule.invalidate_all()
	var times: Array[float] = []
	var frames := 0
	while frames < 5000:
		frames += 1
		var started := Time.get_ticks_usec()
		schedule.mark_light(sun, 0.5, 1)
		var picks := schedule.pick(anchor, anchor, view, FRAME_BUDGET_TILES, VIEW_SAMPLE_RAD)
		times.append((Time.get_ticks_usec() - started) / 1000.0)
		if picks.is_empty():
			break
		for i in range((picks.size() + 1) / 2):
			schedule.completed(picks[i]["tile"], picks[i]["level"], _state(sun, 0.5, 1, anchor))
	var worst := _max_without_outliers(times, "cold start, %d frames" % frames)
	_check(frames < 5000 and worst <= COLD_BUDGET_MS, "cold start with partial fills: every frame <= %.0f ms "
			% COLD_BUDGET_MS + "(max %.3f ms, converged after %d frames)" % [worst, frames])


static func _median(values: Array[float]) -> float:
	var sorted := values.duplicate()
	sorted.sort()
	return sorted[sorted.size() / 2]


static func _max_angle_dot(dir: Vector3, views: Array[Vector3]) -> float:
	var best := -1.0
	for d in views:
		best = maxf(best, dir.dot(d))
	return best


func _check(ok: bool, message: String) -> void:
	if ok:
		print("  ok    %s" % message)
	else:
		_failures += 1
		print("  FAIL  %s" % message)
