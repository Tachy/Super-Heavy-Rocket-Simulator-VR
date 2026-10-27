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
	var seen := PackedByteArray()
	seen.resize(layout.tile_count())
	var identity := true
	for tile in range(layout.tile_count()):
		identity = identity and layout.tile_of(layout.tile_dir(tile)) == tile
		seen[tile] += 1
	_check(identity, "each tile's centre direction lies in that tile (%d tiles)" % layout.tile_count())
	var sizes_ok := layout.level_texels(0) == TILE_TEXELS / 4 and layout.level_texels(1) == TILE_TEXELS / 2 \
			and layout.level_texels(2) == TILE_TEXELS
	_check(sizes_ok, "level texels: 1/4, 1/2, full of the tile")
	# Texel density: the face centre has face_texels / 2 per radian, no direction fewer than that / sqrt(3)^... (corner).
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
	var picks := schedule.pick(anchor, anchor, view, 200, sample_rad)
	var pick_ms := (Time.get_ticks_usec() - started) / 1000.0
	var all_level_0 := picks.size() == 200
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
	# Everything in the view and margin gets level 0 before anything refines.
	var order_ok := true
	var done := {}
	var guard := 0
	var refined_early := false
	var margin_open := true
	while margin_open and guard < 200:
		guard += 1
		var batch := schedule.pick(anchor, anchor, view, 400, sample_rad)
		margin_open = false
		for p in batch:
			if p["level"] == 0 and schedule.level_of(p["tile"]) < 0:
				margin_open = true
			elif schedule.level_of(p["tile"]) < 0:
				refined_early = true
		for p in batch:
			if p["level"] == 0:
				schedule.completed(p["tile"], 0, _state(Vector3.UP))
	_check(not refined_early and guard < 200,
			"the view and margin reach level 0 before a tile refines (%d rounds)" % guard)
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
	schedule.mark_light(sun, 0.5, 1)
	_check(_stale_count(schedule) == 0, "same light: no tile stale")
	var axis := sun.cross(Vector3.UP).normalized()
	schedule.mark_light(sun.rotated(axis, deg_to_rad(0.05)), 0.5, 1)
	_check(_stale_count(schedule) == 0, "Sun 0.05 deg off: no tile stale (%d)" % _stale_count(schedule))
	schedule.mark_light(sun.rotated(axis, deg_to_rad(0.5)), 0.5, 1)
	_check(_stale_count(schedule) == layout.tile_count(), "Sun 0.5 deg off: all %d tiles stale (%d)"
			% [layout.tile_count(), _stale_count(schedule)])
	schedule.mark_light(sun, 0.5, 1)
	_check(_stale_count(schedule) == 0, "Sun back: not stale")
	schedule.mark_light(sun, 0.51, 1)
	_check(_stale_count(schedule) == 0, "weather weight 0.01 off: not stale")
	schedule.mark_light(sun, 0.53, 1)
	_check(_stale_count(schedule) == layout.tile_count(), "weather weight 0.03 off: all stale")
	schedule.mark_light(sun, 0.5, 2)
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
	schedule.mark_light(sun, 0.5, 2)
	schedule.pick(moved, moved, [Vector3(0.0, 0.0, -1.0)], 1, 0.1)
	_check(schedule.is_stale(picks[0]["tile"]), "a new anchor makes a tile rendered from the old one stale")


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
	s2.mark_light(Vector3.UP, 0.5, 1)
	var near_tile := all.tile_of(Vector3(0.0, 0.0, -1.0))
	var far_tile := all.tile_of(Vector3(0.05, 0.0, -1.0))
	s2.set_tile_distance(near_tile, 500.0)
	s2.set_tile_distance(far_tile, 5000.0)
	var camera := anchor + Vector3(0.0, 5.0, 0.0)
	var picks := s2.pick(camera, anchor, [Vector3(0.0, 0.0, -1.0)], 10, deg_to_rad(10.0))
	_check(picks.size() == 2 and picks[0]["tile"] == near_tile and picks[1]["tile"] == far_tile,
			"visible tiles over %.1f px: the nearer cloud first (%s)" % [CloudViewCacheSchedule.PARALLAX_REFRESH_PX,
			str(picks)])


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


func _check(ok: bool, message: String) -> void:
	if ok:
		print("  ok    %s" % message)
	else:
		_failures += 1
		print("  FAIL  %s" % message)
