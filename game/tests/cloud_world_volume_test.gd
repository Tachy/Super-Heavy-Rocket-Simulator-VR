# The cloud world volume's layout and upkeep schedule (world-volume plan Task 3; pure logic, no GPU).
# Headless: godot --headless --path game --script res://tests/cloud_world_volume_test.gd
# Checked: the azimuthal-equidistant grid (round trip, distances), the level on / off heights and vertical windows,
# toroidal addressing, exposed slabs, the fade band (also vertical for levels 0-1), the re-frame test; the schedule:
# cold start (the coarsest level baked first under the spike budget and shown once whole, light provisional from the
# column, finer levels coarse to fine), scroll slabs baked ahead of the window (1 km/s, 7.8 km/s; with the budget
# capped the windows wait instead of reading unbaked cells and the shown levels stay nested), the
# layout's origins kept by the schedule, Sun / Moon / weather-blend triggers, light budget and order, the light's write
# modes (Task 6: real light only for filled levels, direct in spike frames, rate-limited refresh passes with settle
# passes from the GPU's clamp counts, provisional light blended out over PROVISIONAL_FADE_S), pair switch (also
# during a fill: no cell baked twice), jump, re-frame (one level at a time, coarsest first, a shown version in every
# plan, spike cap), plan() cost. The schedules run at TEST_TAPS_PER_MS (world-volume Task 5: the budget counts taps).
extends SceneTree

const A_M := 6378137.0
const F := 1.0 / 298.257223563
const DT := 1.0 / 45.0
const SUN_30 := 30.0
const CLOUD_TOP_M := 11500.0  # with the camera at 100 m: H = 12000 m above level 0's lowest cell (-500 m)
const FILL_FRAMES_MAX := 400
## A GPU baking this many taps per ms fills the whole volume (1.24e11 taps) in ~85 spike frames: the logic's tests.
const TEST_TAPS_PER_MS := 1.0e9
## The RTX 4090's measured cold-start bake rate (cloud_check.gd section 16, Task 5): the scroll flights plan at it.
const GAME_TAPS_PER_MS := 4.3e7
## The schedules' light rate (cells per ms) for the logic's tests (the spec's 67 M cells in 60 ms; cloud_check.gd
## section 17 measures the GPU's).
const TEST_LIGHT_CELLS_PER_MS := 67.0e6 / 60.0

var _failures := 0
var _rng := RandomNumberGenerator.new()


func _initialize() -> void:
	_rng.seed = 20261028
	_run.call_deferred()


func _run() -> void:
	_test_projection()
	_test_levels()
	_test_toroid_and_slabs()
	_test_fade()
	_test_reframe_and_memory()
	_test_cold_start()
	_test_scroll()
	_test_triggers()
	_test_light_budget_and_order()
	_test_moon()
	_test_light_modes()
	_test_pair_switch()
	_test_jump()
	_test_reframe_schedule()
	if _failures == 0:
		print("cloud_world_volume_test: PASS")
	else:
		printerr("cloud_world_volume_test: FAIL (%d)" % _failures)
	quit(1 if _failures > 0 else 0)


# ---------------------------------------------------------------- layout

func _geodetic_point(lat: float, lon: float, h: float) -> Vector3:
	var e2 := F * (2.0 - F)
	var s := sin(lat)
	var n := A_M / sqrt(1.0 - e2 * s * s)
	return Vector3((n + h) * cos(lat) * cos(lon), (n * (1.0 - e2) + h) * s, (n + h) * cos(lat) * sin(lon))


func _normal(lat: float, lon: float) -> Vector3:
	return Vector3(cos(lat) * cos(lon), sin(lat), cos(lat) * sin(lon))


func _test_projection() -> void:
	print("projection")
	var lat0 := deg_to_rad(25.99)
	var lon0 := deg_to_rad(-97.18)
	var origin := _geodetic_point(lat0, lon0, 0.0)
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(origin)
	var n0 := _normal(lat0, lon0)
	var s0 := sin(lat0)
	var e2 := F * (2.0 - F)
	var r_g := A_M * sqrt(1.0 - e2) / (1.0 - e2 * s0 * s0)
	var worst_round := 0.0
	var worst_rel := 0.0
	var n_ok := 0
	while n_ok < 1000:
		var lat := lat0 + _rng.randf_range(-0.24, 0.24)
		var lon := lon0 + _rng.randf_range(-0.24, 0.24)
		var n := _normal(lat, lon)
		var ang := atan2(n.cross(n0).length(), n.dot(n0))
		if ang * r_g > 1.5e6:
			continue
		var h := _rng.randf_range(-500.0, 400000.0)
		var p := _geodetic_point(lat, lon, h)
		var g := layout.grid_of(p)
		worst_round = maxf(worst_round, layout.earth_of(g).distance_to(p))
		if ang > 1e-6:
			worst_rel = maxf(worst_rel, absf(Vector2(g.x, g.y).length() - r_g * ang) / (r_g * ang))
		worst_round = maxf(worst_round, absf(g.z - h))
		n_ok += 1
	_check(worst_round < 1e-6, "earth_of(grid_of(p)) within 1e-6 m and z = height (worst %s m)" % worst_round)
	_check(worst_rel < 1e-9, "|(x, y)| = great-circle distance on the projection sphere (worst %s)" % worst_rel)
	_check(layout.grid_of(origin).length() < 1e-6, "the frame origin maps to (0, 0, 0)")


func _test_levels() -> void:
	print("levels")
	var L := CloudWorldVolumeLayout
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(Vector3(A_M, 0.0, 0.0))
	_check(L.LEVELS == 8 and L.CELLS_H == 256 and L.CELLS_V == 128, "8 levels of 256 x 256 x 128")
	_check(layout.level_active(0, 22000.0) and not layout.level_active(0, 22200.0), "level 0 on at 22.0 km, off at 22.2")
	_check(layout.level_active(1, 23600.0) and not layout.level_active(1, 23800.0), "level 1 off at 23.8 km")
	var all_on := true
	for k in range(2, 8):
		all_on = all_on and layout.level_active(k, 400000.0)
	_check(all_on, "levels 2-7 on at 400 km")
	var o := layout.window_origin(0, Vector3(0.0, 0.0, 100.0))
	var bottom := L.SHELL_BOTTOM_M + o.z * L.CELL_V_M[0]
	_check(is_equal_approx(bottom, -500.0) and is_equal_approx(bottom + L.window_size(0).z * L.CELL_V_M[0], 2600.0),
			"level 0 window at camera height 100 m = [-500, 2600] m (bottom clamp; storage to 2700 m)")
	_check(L.window_size(0) == Vector3i(252, 252, 124) and L.window_size(2) == Vector3i(252, 252, 128),
			"windows: the storage less GUARD_CELLS per side of each scrolling axis")
	_check(L.storage_target(0, o) == Vector3i(o.x - 2, o.y - 2, 0) and L.storage_target(5, Vector3i(7, -9, 0))
			== Vector3i(5, -11, 0), "storage target: the window grown by the guard, not below the shell")
	var o1 := layout.window_origin(1, Vector3(0.0, 0.0, 100.0))
	_check(o1.z == 0, "level 1 clamped at the shell bottom")
	var oh := layout.window_origin(0, Vector3(0.0, 0.0, 22200.0))
	_check(L.SHELL_BOTTOM_M + oh.z * L.CELL_V_M[0] >= L.SHELL_TOP_M, "at the off height level 0's window is above the shell")
	var o_mid := layout.window_origin(0, Vector3(0.0, 0.0, 9000.0))
	var lo := L.SHELL_BOTTOM_M + o_mid.z * L.CELL_V_M[0]
	var half_v := 0.5 * L.window_size(0).z * L.CELL_V_M[0]
	_check(lo < 9000.0 and lo + 2.0 * half_v > 9000.0 and absf((lo + half_v) - 9000.0) <= L.CELL_V_M[0],
			"level 0 is centred on the camera height away from the shell bottom")
	var o7 := layout.window_origin(7, Vector3(0.0, 0.0, 9000.0))
	_check(o7.z == 0, "levels 2-7 span the whole shell")
	_check(L.CELL_V_M[2] * L.CELLS_V == L.SHELL_TOP_M - L.SHELL_BOTTOM_M, "164.0625 m x 128 = the shell")


func _test_toroid_and_slabs() -> void:
	print("toroid, slabs")
	var L := CloudWorldVolumeLayout
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(Vector3(A_M, 0.0, 0.0))
	var ok := true
	for i in 100:
		var k := _rng.randi_range(0, 7)
		var o := Vector3i(_rng.randi_range(-100000, 100000), _rng.randi_range(-100000, 100000), _rng.randi_range(0, 400))
		for axis in 3:
			var n := L.CELLS_H if axis < 2 else L.CELLS_V
			var seen := {}
			for i_c in n:
				var c := o
				c[axis] += i_c
				seen[layout.texel_of(k, c)[axis]] = true
			ok = ok and seen.size() == n
			for key: int in seen:
				ok = ok and key >= 0 and key < n
	_check(ok, "texel_of is a bijection of every window axis onto the level's texels (100 random origins)")
	var old_o := Vector3i(1000, -500, 4)
	var slabs := layout.exposed_slabs(2, old_o, old_o + Vector3i(3, 0, 0))
	_check(slabs.size() == 1 and _cells(slabs[0]) == 3 * 256 * 128, "+3 cells in x: one slab of 3 x 256 x 128")
	var new_o := old_o + Vector3i(5, -7, 2)
	slabs = layout.exposed_slabs(0, old_o, new_o)
	var total := 0
	var disjoint := true
	for i in slabs.size():
		total += _cells(slabs[i])
		for j in range(i + 1, slabs.size()):
			disjoint = disjoint and not _overlap(slabs[i], slabs[j])
	var overlap := (256 - 5) * (256 - 7) * (128 - 2)
	_check(disjoint and total == 256 * 256 * 128 - overlap, "diagonal move: disjoint slabs = new window minus overlap")
	var inside := true
	for s: Dictionary in slabs:
		inside = inside and _inside_window(s, new_o) and not _overlap_window(s, old_o)
	_check(inside, "the slabs lie in the new window and outside the old one")
	slabs = layout.exposed_slabs(4, old_o, old_o + Vector3i(256, 0, 0))
	_check(slabs.size() == 1 and _cells(slabs[0]) == 256 * 256 * 128, "a move of >= 256 cells exposes the whole level")
	_check(layout.exposed_slabs(3, old_o, old_o).is_empty(), "no move, no slab")


func _cells(box: Dictionary) -> int:
	var d: Vector3i = box["to"] - box["from"]
	return maxi(d.x, 0) * maxi(d.y, 0) * maxi(d.z, 0)


func _overlap(a: Dictionary, b: Dictionary) -> bool:
	var f := Vector3i(maxi(a["from"].x, b["from"].x), maxi(a["from"].y, b["from"].y), maxi(a["from"].z, b["from"].z))
	var t := Vector3i(mini(a["to"].x, b["to"].x), mini(a["to"].y, b["to"].y), mini(a["to"].z, b["to"].z))
	return t.x > f.x and t.y > f.y and t.z > f.z


func _window_box(o: Vector3i) -> Dictionary:
	return {"from": o, "to": o + Vector3i(256, 256, 128)}


func _inside_window(box: Dictionary, o: Vector3i) -> bool:
	var w := _window_box(o)
	return box["from"].x >= w["from"].x and box["from"].y >= w["from"].y and box["from"].z >= w["from"].z \
			and box["to"].x <= w["to"].x and box["to"].y <= w["to"].y and box["to"].z <= w["to"].z


func _overlap_window(box: Dictionary, o: Vector3i) -> bool:
	return _overlap(box, _window_box(o))


func _test_fade() -> void:
	print("fade")
	var L := CloudWorldVolumeLayout
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(Vector3(A_M, 0.0, 0.0))
	var cam := Vector3(12345.0, -6789.0, 3000.0)
	var o := layout.window_origin(3, cam)
	layout.set_origin(3, o)
	var cell := L.CELL_H_M[3]
	var size := L.window_size(3)
	var y := (o.y + 100) * cell
	var z := 3000.0
	_check(layout.fade_weight(3, Vector3((o.x + 128) * cell, y, z)) == 1.0, "weight 1 inside")
	_check(layout.fade_weight(3, Vector3((o.x + 0.5) * cell, y, z)) == 0.0, "weight 0 at the outermost cell")
	_check(layout.fade_weight(3, Vector3((o.x + size.x - 0.5) * cell, y, z)) == 0.0,
			"weight 0 at the outermost cell (far side)")
	_check(is_equal_approx(layout.fade_weight(3, Vector3((o.x + 4.5) * cell, y, z)), 0.5), "linear: half-way over 8 cells")
	_check(layout.fade_weight(3, Vector3((o.x + 8.5) * cell, y, z)) == 1.0, "weight 1 8 cells in")
	_check(layout.fade_weight(3, Vector3((o.x - 2) * cell, y, z)) == 0.0, "weight 0 outside")
	# Vertical fade of levels 0-1 at the window's top edge; none at the shell bottom (clamped window).
	var low := layout.window_origin(0, Vector3(0.0, 0.0, 100.0))
	layout.set_origin(0, low)
	var c0 := L.CELL_H_M[0]
	var x := (low.x + 128) * c0
	var yy := (low.y + 128) * c0
	_check(layout.fade_weight(0, Vector3(x, yy, -499.0)) == 1.0, "no vertical fade at the shell bottom (clamped)")
	var top := L.SHELL_BOTTOM_M + (low.z + L.window_size(0).z) * L.CELL_V_M[0]
	_check(layout.fade_weight(0, Vector3(x, yy, top - 0.5 * L.CELL_V_M[0])) == 0.0, "level 0 fades at its window top")
	_check(layout.fade_weight(0, Vector3(x, yy, top - 4.5 * L.CELL_V_M[0])) > 0.4, "...linearly over 8 cells")
	var mid := layout.window_origin(0, Vector3(0.0, 0.0, 9000.0))
	layout.set_origin(0, mid)
	var bot := L.SHELL_BOTTOM_M + mid.z * L.CELL_V_M[0]
	_check(layout.fade_weight(0, Vector3((mid.x + 128) * c0, (mid.y + 128) * c0, bot + 0.5 * L.CELL_V_M[0])) == 0.0,
			"level 0 fades at its window bottom away from the shell bottom")
	# Each level's window lies inside the next coarser level's region with weight 1.
	var bad := 0
	for i in 1000:
		var cg := Vector3(_rng.randf_range(-900000.0, 900000.0), _rng.randf_range(-900000.0, 900000.0),
				_rng.randf_range(-500.0, 23500.0))
		for k in 7:
			if not layout.level_active(k, cg.z):
				continue
			layout.set_origin(k, layout.window_origin(k, cg))
			layout.set_origin(k + 1, layout.window_origin(k + 1, cg))
			var w := layout.window_origin(k, cg)
			var w1 := layout.window_origin(k + 1, cg)
			if not L.nested(k, w, w1) or not _nests(k, w, w1):
				bad += 1
			# k + 1 left behind (horizontally; level 1 vertically when it is not clamped): no longer nested
			if L.nested(k, w, w1 - Vector3i(60, 0, 0)) or _nests(k, w, w1 - Vector3i(60, 0, 0)):
				bad += 1
			var up_z := w1 + Vector3i(0, 0, 30)
			if k == 0 and w1.z > 40 and (L.nested(k, w, up_z) or _nests(k, w, up_z)):
				bad += 1
			var ch := L.CELL_H_M[k]
			var cv := L.CELL_V_M[k]
			var ws := L.window_size(k)
			for cx in [0.5, ws.x - 0.5]:
				for cy in [0.5, ws.y - 0.5]:
					for cz in [0.5, ws.z - 0.5]:
						var gz: float = L.SHELL_BOTTOM_M + (w.z + cz) * cv
						if gz < L.SHELL_BOTTOM_M or gz > L.SHELL_TOP_M:
							continue
						if layout.fade_weight(k + 1, Vector3((w.x + cx) * ch, (w.y + cy) * ch, gz)) != 1.0:
							bad += 1
	_check(bad == 0, "1000 random cameras: each level's window lies inside the next coarser level with weight 1, "
			+ "nested() agrees, and not with level k + 1 left 60 cells behind (%d bad)" % bad)


func _test_reframe_and_memory() -> void:
	print("reframe, memory")
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(Vector3(A_M, 0.0, 0.0))
	_check(layout.reframe_needed(layout.earth_of(Vector3(1.0001e6, 0.0, 0.0))), "re-frame needed at 1000.1 km")
	_check(not layout.reframe_needed(layout.earth_of(Vector3(9.999e5, 0.0, 0.0))), "not at 999.9 km")
	_check(not layout.reframe_needed(layout.earth_of(Vector3(0.0, 0.0, 400000.0))), "height does not count")
	_check(CloudWorldVolumeLayout.memory_bytes() == 8 * 256 * 256 * 128 * 16, "memory_bytes = 8 x 256 x 256 x 128 x 16")


# ---------------------------------------------------------------- schedule

func _schedule(layout := CloudWorldVolumeLayout.new()) -> CloudWorldVolumeSchedule:
	var s := CloudWorldVolumeSchedule.new(layout)
	s.bake_taps_per_ms = TEST_TAPS_PER_MS
	s.light_cells_per_ms = TEST_LIGHT_CELLS_PER_MS
	return s


func _cam(lam: float, h: float) -> Vector3:
	return (A_M + h) * Vector3(cos(lam), 0.0, sin(lam))


func _up(lam: float) -> Vector3:
	return Vector3(cos(lam), 0.0, sin(lam))


func _sun(lam: float, elevation_deg: float) -> Vector3:
	var e := deg_to_rad(elevation_deg)
	return (_up(lam) * sin(e) + Vector3.UP * cos(e)).normalized()


func _plan(s: CloudWorldVolumeSchedule, lam: float, h: float, sun: Vector3, blend := 0.3, jumped := false,
		moon := Vector3(0.0, 0.6, 0.8), slots: Array = [0, 1]) -> Dictionary:
	return s.plan(_cam(lam, h), sun, moon.normalized(), blend, CLOUD_TOP_M, jumped, DT, slots)


## Plans a still camera until the schedule is complete and its light queues are empty; returns the frames used.
func _settle(s: CloudWorldVolumeSchedule, lam: float, h: float, sun: Vector3, blend := 0.3) -> int:
	var n := 0
	while n < 3 * FILL_FRAMES_MAX:
		var p := _plan(s, lam, h, sun, blend)
		n += 1
		if s.complete() and p["light"].is_empty() and p["bake"].is_empty():
			return n
	return -1


func _sum_cells(jobs: Array, level := -1) -> int:
	var n := 0
	for j: Dictionary in jobs:
		if level < 0 or j["level"] == level:
			n += _cells(j)
	return n


func _test_cold_start() -> void:
	print("cold start")
	var layout := CloudWorldVolumeLayout.new()
	var s := _schedule(layout)
	var sun := _sun(0.0, SUN_30)
	var p := _plan(s, 0.0, 100.0, sun)
	var cap: float = CloudWorldVolumeSchedule.UPKEEP_SPIKE_MS * 0.5 * s.bake_taps_per_ms
	_check(_sum_cells(p["bake"]) == _sum_cells(p["bake"], 7) and _sum_cells(p["bake"], 7) > 0
			and p["bake_taps"] <= cap + CloudWorldVolumeLayout.cell_taps(7),
			"the first plan bakes the coarsest level first, within the spike's bake budget (%d taps)" % p["bake_taps"])
	var channels_ok := true
	for j: Dictionary in p["bake"]:
		channels_ok = channels_ok and j["snapshot_channel"] == -1
	_check(channels_ok, "...both snapshots")
	_check(not p["lookup"]["levels"][7]["ready"], "...and does not show it before it is baked whole")
	_check(p["light_cells"] <= CloudWorldVolumeSchedule.UPKEEP_SPIKE_MS * s.light_cells_per_ms, "first-plan light within the spike cap")
	# Until the coarsest level is whole: every plan lights what it baked (real or provisional, the provisional light
	# from the grid-mean column); finer levels follow coarse to fine; all within the spike cap.
	var first_seen := {}
	var order_ok := true
	var spike_ok := true
	var lit_ok := true
	var from_column := true
	var prov7 := 0
	var ready7_at := -1
	var frames := 1
	while true:
		for j: Dictionary in p["bake"]:
			if not first_seen.has(j["level"]):
				first_seen[j["level"]] = frames
		if ready7_at < 0:
			lit_ok = lit_ok and _sum_cells(p["light"], 7) + _sum_cells(p["provisional"], 7) >= _sum_cells(p["bake"], 7)
			prov7 += _sum_cells(p["provisional"], 7)
			if p["lookup"]["levels"][7]["ready"]:
				ready7_at = frames
		for j: Dictionary in p["provisional"]:
			if j["level"] == 7:
				from_column = from_column and j["from_column"]
		var est: float = p["bake_taps"] / s.bake_taps_per_ms + p["light_cells"] / s.light_cells_per_ms
		spike_ok = spike_ok and est <= CloudWorldVolumeSchedule.UPKEEP_SPIKE_MS + 1e-6
		if s.complete() or frames >= FILL_FRAMES_MAX:
			break
		p = _plan(s, 0.0, 100.0, sun)
		frames += 1
	_check(ready7_at > 0 and first_seen.get(6, 0) >= ready7_at - 1, "the coarsest level is shown once baked whole (plan %d), "
			% ready7_at + "level 6 starts in its last plan or after (%d)" % first_seen.get(6, -1))
	_check(lit_ok and from_column, "its baked cells get real or provisional light in their plan (%d " % prov7
			+ "provisional, from the grid-mean column)")
	for k in range(7, 0, -1):
		order_ok = order_ok and first_seen.has(k) and first_seen.has(k - 1) and first_seen[k] <= first_seen[k - 1]
	_check(order_ok and first_seen.size() == 8, "finer levels are baked coarse to fine")
	_check(spike_ok, "every fill plan stays within UPKEEP_SPIKE_MS")
	_check(s.complete() and frames <= 120, "schedule logic: whole volume filled in %d frames at test rate %s "
			% [frames,
			String.num_scientific(TEST_TAPS_PER_MS)] + "taps / ms (not the game's rate: section 16 measures that)")
	var all_ready := true
	var origins_ok := true
	for e: Dictionary in _plan(s, 0.0, 100.0, sun)["lookup"]["levels"]:
		all_ready = all_ready and e["ready"]
		origins_ok = origins_ok and layout.origins[e["level"]] == e["origin"]
	_check(all_ready, "every level shown after the fill")
	_check(origins_ok, "the layout holds every level's window origin (fade_weight)")
	_check(not s.light_stale(0), "...and its light is up to date")


func _test_scroll() -> void:
	print("scroll")
	var sun := _sun(0.0, SUN_30)
	# Bake ahead of exposure (Task 5 review I2) at the game's measured rate: no frame reads an unbaked cell, the windows
	# never wait, every plan within the spike cap; 1 km/s low and climbing (all 8 levels, the vertical scroll of levels
	# 0-1), 7.8 km/s in orbit (levels 2-7: 7.8 km/s occurs only above the shell). Diagonal headings: both axes scroll.
	_scroll_flight("1 km/s at 10 km, climbing 150 m/s", 1000.0, 10000.0, 150.0, 1400)
	_scroll_flight("7.8 km/s at 120 km", 7800.0, 120000.0, 0.0, 1200)
	# The bake budget binding (Task 5 review round 2, I2 / N1): the windows wait (lag) instead of reading unbaked
	# cells, and the levels stay nested in every frame (N1 ruling).
	_capped_flight("7.8 km/s at 120 km, budget x 0.1", 7800.0, 120000.0, 0.0, 1500, 0.1)
	_capped_flight("7.8 km/s at 120 km, budget x 0.3", 7800.0, 120000.0, 0.0, 1500, 0.3)
	_capped_flight("1 km/s at 10 km climbing 150 m/s, budget x 0.05", 1000.0, 10000.0, 150.0, 1500, 0.05)
	# A coarser level not ready (its fill restarted): every finer level is hidden with it (N1), shown again after.
	var sc := _schedule()
	_settle(sc, 0.0, 100.0, sun)
	sc._levels[5].ready = false
	var hid: Array = _plan(sc, 0.0, 100.0, sun)["lookup"]["levels"]
	sc._levels[5].ready = true
	var back: Array = _plan(sc, 0.0, 100.0, sun)["lookup"]["levels"]
	var chain_ok := true
	for k in CloudWorldVolumeLayout.LEVELS:
		chain_ok = chain_ok and hid[k]["shown"] == (k > 5) and back[k]["shown"] and (k == 5 or hid[k]["ready"])
	_check(chain_ok, "a coarser level not ready hides the finer ones (still filled), shown again once it is")
	# 7.8 km/s at 120 km: bake cells per frame follow the layout (levels 2-7); p99 plan() cost <= 1 ms.
	var s := _schedule()
	_settle(s, 0.0, 120000.0, sun)
	var ref := CloudWorldVolumeLayout.new()
	ref.set_frame(_cam(0.0, 120000.0))
	var rate := 7800.0 * DT / (A_M + 120000.0)
	var step_m := ref.grid_of(_cam(rate, 120000.0)).distance_to(ref.grid_of(_cam(0.0, 120000.0)))
	var formula := 0.0
	for k in range(2, 8):
		formula += step_m / CloudWorldVolumeLayout.CELL_H_M[k] * 256.0 * 128.0
	var total := 0
	var times := PackedFloat64Array()
	var frames := 1000
	var lam := 0.0
	var ok_levels := true
	for f in frames:
		lam += rate
		var t0 := Time.get_ticks_usec()
		var p := _plan(s, lam, 120000.0, sun)
		times.append(float(Time.get_ticks_usec() - t0))
		total += _sum_cells(p["bake"])
		ok_levels = ok_levels and _sum_cells(p["bake"], 0) == 0 and _sum_cells(p["bake"], 1) == 0
	var mean := float(total) / frames
	_check(absf(mean - formula) <= 0.2 * formula,
			"7.8 km/s at 120 km: %.0f bake cells per frame vs %.0f from the layout (+-20 %%)" % [mean, formula])
	_check(ok_levels, "levels 0-1 are off at 120 km")
	times.sort()
	var p99 := times[int(0.99 * times.size())]
	_check(p99 <= 1000.0, "plan() p99 %.0f us <= 1000 us over %d frames (median %.0f us)" % [p99, frames, times[times.size() / 2]])


## A flight at `speed_mps` (heading 30 deg east of north, climbing at `climb_mps`) from a settled volume, planned at the
## game's bake rate: an independent model of the unbaked cells per level (the cells the storage gained, less every bake
## job) must never meet a shown window; the windows never wait; every plan's estimate within UPKEEP_SPIKE_MS.
func _scroll_flight(label: String, speed_mps: float, h0_m: float, climb_mps: float, frames: int) -> void:
	var L := CloudWorldVolumeLayout
	var sun := _sun(0.0, SUN_30)
	var lat0 := deg_to_rad(25.99)
	var lon0 := deg_to_rad(-97.18)
	var s := _schedule()
	var cam := _geodetic_point(lat0, lon0, h0_m)
	var n := 0
	while n < 3 * FILL_FRAMES_MAX:
		var p := s.plan(cam, sun, Vector3(0.0, 0.6, 0.8), 0.3, CLOUD_TOP_M, false, DT, [0, 1])
		n += 1
		if s.complete() and p["bake"].is_empty() and p["light"].is_empty():
			break
	s.bake_taps_per_ms = GAME_TAPS_PER_MS
	var storage: Array = []
	var unbaked: Array = []
	var first := s.plan(cam, sun, Vector3(0.0, 0.6, 0.8), 0.3, CLOUD_TOP_M, false, DT, [0, 1])
	for e: Dictionary in first["lookup"]["levels"]:
		storage.append(e["storage"])
		unbaked.append([])
	var heading := deg_to_rad(30.0)
	var lat := lat0
	var lon := lon0
	var h := h0_m
	var read_unbaked := 0
	var waiting := 0
	var not_ready := 0
	var worst_ms := 0.0
	var scroll_ms_sum := 0.0
	var worst_scroll_ms := 0.0
	var prov_max := 0.0
	var moves := 0
	for f in frames:
		var r := A_M + h
		lat += speed_mps * cos(heading) * DT / r
		lon += speed_mps * sin(heading) * DT / (r * cos(lat))
		h += climb_mps * DT
		var p := s.plan(_geodetic_point(lat, lon, h), sun, Vector3(0.0, 0.6, 0.8), 0.3, CLOUD_TOP_M, false, DT, [0, 1])
		var est: float = p["bake_taps"] / GAME_TAPS_PER_MS + p["light_cells"] / s.light_cells_per_ms
		worst_ms = maxf(worst_ms, est)
		var scroll_ms: float = p["bake_scroll_taps"] / GAME_TAPS_PER_MS
		scroll_ms_sum += scroll_ms
		worst_scroll_ms = maxf(worst_scroll_ms, scroll_ms)
		waiting += 1 if p["windows_waiting"] > 0 else 0
		prov_max = maxf(prov_max, p["provisional_age_max_s"])
		for k in L.LEVELS:
			var e: Dictionary = p["lookup"]["levels"][k]
			if not e["active"]:
				continue
			if not e["ready"]:
				not_ready += 1
				continue
			var st: Vector3i = e["storage"]
			var list: Array = unbaked[k]
			if st != storage[k]:
				moves += 1
				list.append_array(L.exposed_slabs(k, storage[k], st))
				storage[k] = st
			var kept: Array = []
			for b: Dictionary in list:  # cells that left the storage
				var c := L.box_clip(b, L.storage_box(st))
				if L.box_cells(c) > 0:
					kept.append(c)
			list = kept
			for j: Dictionary in p["bake"]:
				if j["level"] == k and not j["spare"]:
					var rest: Array = []
					for b: Dictionary in list:
						rest.append_array(_box_subtract(b, j))
					list = rest
			unbaked[k] = list
			var win := L.window_box(k, e["origin"])
			var inside: bool = L.box_cells(L.box_clip(win, L.storage_box(st))) == L.box_cells(win)
			for b: Dictionary in list:
				inside = inside and not L.box_overlaps(win, b)
			read_unbaked += 0 if inside else 1
	_check(moves > 0 and read_unbaked == 0 and not_ready == 0,
			"%s: no shown window holds an unbaked or foreign cell (%d storage moves, %d frames)"
			% [label, moves, frames])
	_check(waiting == 0, "%s: the windows never wait (%d frames with a window behind the camera)" % [label, waiting])
	_check(worst_ms <= CloudWorldVolumeSchedule.UPKEEP_SPIKE_MS + 1e-6,
			"%s: every plan <= %.1f ms at %s taps / ms (worst %.2f ms; scroll bake mean %.2f ms, worst %.2f ms)"
			% [label, CloudWorldVolumeSchedule.UPKEEP_SPIKE_MS, String.num_scientific(GAME_TAPS_PER_MS), worst_ms,
			scroll_ms_sum / frames, worst_scroll_ms])
	_check(prov_max <= CloudWorldVolumeSchedule.PROVISIONAL_MAX_S,
			"%s: provisional light never older than %.1f s (%.2f s)"
			% [label, CloudWorldVolumeSchedule.PROVISIONAL_MAX_S, prov_max])


## A flight from a settled volume with the bake budget scaled by `scale` after settling, so the scroll cap binds
## (Task 5 review round 2: I2 test gap, N1 ruling). Independent models: the unbaked cells per level (as in
## _scroll_flight, followed while the level is `ready` = filled) must never meet the window of a SHOWN level; every
## frame the shown levels nest - level k shown => level k + 1 shown, and level k's window (its fade band included)
## lies where level k + 1's weight is 1 (fade_weight_at at the window's corners). The windows must wait (> 0 frames:
## the cap binds), counted apart from levels that are not ready (a fill restarted after a whole-extent lag).
func _capped_flight(label: String, speed_mps: float, h0_m: float, climb_mps: float, frames: int, scale: float) -> void:
	var L := CloudWorldVolumeLayout
	var sun := _sun(0.0, SUN_30)
	var moon := Vector3(0.0, 0.6, 0.8)
	var lat := deg_to_rad(25.99)
	var lon := deg_to_rad(-97.18)
	var h := h0_m
	var s := _schedule()
	var cam := _geodetic_point(lat, lon, h)
	for i in 3 * FILL_FRAMES_MAX:
		var p0 := s.plan(cam, sun, moon, 0.3, CLOUD_TOP_M, false, DT, [0, 1])
		if s.complete() and p0["bake"].is_empty() and p0["light"].is_empty():
			break
	s.bake_taps_per_ms = GAME_TAPS_PER_MS
	s.budget_scale = scale
	var first := s.plan(cam, sun, moon, 0.3, CLOUD_TOP_M, false, DT, [0, 1])
	var storage: Array = []
	var unbaked: Array = []
	var was_ready: Array = []
	for e: Dictionary in first["lookup"]["levels"]:
		storage.append(e["storage"])
		unbaked.append([])
		was_ready.append(e["ready"])
	var heading := deg_to_rad(30.0)
	var read_unbaked := 0
	var waiting := 0
	var not_ready := 0
	var hidden := 0
	var not_nested := 0
	var shown_over_hidden := 0
	for f in frames:
		var r := A_M + h
		lat += speed_mps * cos(heading) * DT / r
		lon += speed_mps * sin(heading) * DT / (r * cos(lat))
		h += climb_mps * DT
		var p := s.plan(_geodetic_point(lat, lon, h), sun, moon, 0.3, CLOUD_TOP_M, false, DT, [0, 1])
		waiting += 1 if p["windows_waiting"] > 0 else 0
		var lv: Array = p["lookup"]["levels"]
		for k in L.LEVELS:
			var e: Dictionary = lv[k]
			var shown: bool = e.get("shown", e["ready"])
			if not e["active"]:
				was_ready[k] = false
				continue
			if not e["ready"]:
				not_ready += 1
				was_ready[k] = false
				unbaked[k] = []
				storage[k] = e["storage"]
				continue
			if not was_ready[k]:  # filled again: its whole storage was baked by the fill
				was_ready[k] = true
				storage[k] = e["storage"]
				unbaked[k] = []
			var st: Vector3i = e["storage"]
			var list: Array = unbaked[k]
			if st != storage[k]:
				list.append_array(L.exposed_slabs(k, storage[k], st))
				storage[k] = st
			var kept: Array = []
			for b: Dictionary in list:
				var c := L.box_clip(b, L.storage_box(st))
				if L.box_cells(c) > 0:
					kept.append(c)
			list = kept
			for j: Dictionary in p["bake"]:
				if j["level"] == k and not j["spare"]:
					var rest: Array = []
					for b: Dictionary in list:
						rest.append_array(_box_subtract(b, j))
					list = rest
			unbaked[k] = list
			if not shown:
				hidden += 1
				continue
			var win := L.window_box(k, e["origin"])
			var inside: bool = L.box_cells(L.box_clip(win, L.storage_box(st))) == L.box_cells(win)
			for b: Dictionary in list:
				inside = inside and not L.box_overlaps(win, b)
			read_unbaked += 0 if inside else 1
			if k + 1 < L.LEVELS:
				var up: Dictionary = lv[k + 1]
				if not (up["active"] and up.get("shown", up["ready"])):
					shown_over_hidden += 1
				elif not _nests(k, e["origin"], up["origin"]):
					not_nested += 1
	_check(read_unbaked == 0, "%s: no shown window holds an unbaked or foreign cell (%d level-frames do, %d frames)"
			% [label, read_unbaked, frames])
	_check(waiting > 0, "%s: the cap binds - the windows wait (%d frames; %d level-frames not ready, %d hidden)"
			% [label, waiting, not_ready, hidden])
	_check(shown_over_hidden == 0 and not_nested == 0, "%s: shown levels nest every frame (%d level-frames shown over a "
			% [label, shown_over_hidden] + "coarser level not shown, %d with the window outside the coarser level's "
			% not_nested + "weight-1 region)")


## Level k's window at `origin` lies where level k + 1's weight (window at `origin_up`) is 1: its 8 corners (window
## edges, grid m; levels 2-7 have no vertical fade: they hold the whole shell and clamp vertically).
func _nests(k: int, origin: Vector3i, origin_up: Vector3i) -> bool:
	var L := CloudWorldVolumeLayout
	var size := L.window_size(k)
	var ch: float = L.CELL_H_M[k]
	var cv: float = L.CELL_V_M[k]
	for cx in [0, size.x]:
		for cy in [0, size.y]:
			for cz in [0, size.z]:
				var g := Vector3((origin.x + cx) * ch, (origin.y + cy) * ch, L.SHELL_BOTTOM_M + (origin.z + cz) * cv)
				if L.fade_weight_at(k + 1, g, origin_up) < 1.0:
					return false
	return true


## a less b as disjoint boxes (a itself when they do not overlap).
static func _box_subtract(a: Dictionary, b: Dictionary) -> Array:
	var L := CloudWorldVolumeLayout
	if not L.box_overlaps(a, b):
		return [a]
	var out: Array = []
	var f: Vector3i = a["from"]
	var t: Vector3i = a["to"]
	for axis in 3:
		if b["from"][axis] > f[axis]:
			var hi := t
			hi[axis] = b["from"][axis]
			out.append({"from": f, "to": hi})
			f[axis] = b["from"][axis]
		if b["to"][axis] < t[axis]:
			var lo := f
			lo[axis] = b["to"][axis]
			out.append({"from": lo, "to": t})
			t[axis] = b["to"][axis]
	return out


func _origins(p: Dictionary) -> Array:
	var r := []
	for e: Dictionary in p["lookup"]["levels"]:
		r.append(e["origin"])
	return r


func _slabs_of(bake: Array, level: int) -> Array:
	var r := []
	for j: Dictionary in bake:
		if j["level"] == level:
			r.append(j)
	return r


func _has_box(boxes: Array, box: Dictionary) -> bool:
	for b: Dictionary in boxes:
		if b["from"] == box["from"] and b["to"] == box["to"]:
			return true
	return false


## The Sun turned by `angle` rad (on the sky) about the vertical: same elevation.
func _rotate(sun: Vector3, up: Vector3, angle: float) -> Vector3:
	return sun.rotated(up, angle / sqrt(1.0 - sun.dot(up) * sun.dot(up)))


func _test_triggers() -> void:
	print("triggers")
	var sun := _sun(0.0, SUN_30)
	var thr := CloudWorldVolumeSchedule.sun_threshold_rad(0, deg_to_rad(30.0), 12000.0)
	_check(absf(thr - 2.6e-4) < 0.05e-4, "sun_threshold_rad(0, 30 deg, 12 km) = %s rad" % thr)
	var s := _schedule()
	_check(_settle(s, 0.0, 100.0, sun) > 0, "settled (fill and light queues empty)")
	var p := _plan(s, 0.0, 100.0, _rotate(sun, _up(0.0), 0.9 * thr))
	_check(p["light"].is_empty() and not s.light_stale(0), "Sun moved by 0.9 x threshold: no light jobs, level 0 not stale")
	p = _plan(s, 0.0, 100.0, _rotate(sun, _up(0.0), 1.1 * thr))
	_check(s.light_stale(0) or _sum_cells(p["light"], 0) > 0, "Sun moved by 1.1 x threshold: level 0 stale")
	var only0 := true
	for j: Dictionary in p["light"]:
		only0 = only0 and j["level"] == 0
	_check(only0, "...and only level 0 (the coarser levels' thresholds are larger)")
	_check(_settle(s, 0.0, 100.0, _rotate(sun, _up(0.0), 1.1 * thr)) > 0, "the refresh completes")
	# Weather blend.
	s = _schedule()
	_settle(s, 0.0, 100.0, sun, 0.3)
	p = _plan(s, 0.0, 100.0, sun, 0.319)
	_check(p["light"].is_empty() and not s.light_stale(0), "blend moved 0.019: no light jobs")
	p = _plan(s, 0.0, 100.0, sun, 0.32)
	_check(not p["light"].is_empty() and s.light_stale(7), "blend moved 0.02: every level stale")


func _test_light_budget_and_order() -> void:
	print("light budget, order")
	var sun := _sun(0.0, SUN_30)
	var s := _schedule()
	_settle(s, 0.0, 100.0, sun, 0.3)
	s.feedback(2.0, 400000, 800000)  # fixed GPU timings: calibrates the cells-per-ms rates
	var cap := CloudWorldVolumeSchedule.LIGHT_BUDGET_MS * s.light_cells_per_ms
	var worst := 0
	var jobs := 0
	for i in 12:
		var p := _plan(s, 0.0, 100.0, sun, 0.3 + 0.03 * (i + 1))
		worst = maxi(worst, _sum_cells(p["light"]))
		jobs += p["light"].size()
	_check(jobs > 0 and worst <= cap, "light cells per plan %d <= the calibrated budget %.0f" % [worst, cap])
	# Order: provisional slab first, then the nearest tile of the finest stale level.
	s = _schedule()
	_settle(s, 0.0, 100.0, sun, 0.3)
	var lam := 600.0 / (A_M + 100.0)  # 3 cells of level 2, 12 of level 0
	var p := _plan(s, lam, 100.0, sun, 0.5)
	var light: Array = p["light"]
	_check(not light.is_empty() and not p["bake"].is_empty(), "scroll and blend step in one plan")
	var first_in_slab := false
	for b: Dictionary in p["bake"]:
		if light[0]["level"] == b["level"] and _overlap(light[0], b):
			first_in_slab = true
	_check(first_in_slab, "the first light job serves a freshly baked (provisional) slab")
	var cam_cell := Vector2i.ZERO
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(_cam(0.0, 100.0))
	var g := layout.grid_of(_cam(lam, 100.0))
	cam_cell = Vector2i(floori(g.x / 50.0), floori(g.y / 50.0))
	var near_ok := false
	for j: Dictionary in light:
		var is_slab := false
		for b: Dictionary in p["bake"]:
			is_slab = is_slab or (j["level"] == b["level"] and _inside_box(j, b))
		if j["level"] == 0 and not is_slab:
			# (a camera on a tile edge is a tie: one cell of slack)
			near_ok = cam_cell.x >= j["from"].x - 1 and cam_cell.x <= j["to"].x \
					and cam_cell.y >= j["from"].y - 1 and cam_cell.y <= j["to"].y
			break
	_check(near_ok, "then the tile under the camera first (in view and near)")


func _inside_box(inner: Dictionary, outer: Dictionary) -> bool:
	return inner["from"].x >= outer["from"].x and inner["from"].y >= outer["from"].y \
			and inner["from"].z >= outer["from"].z and inner["to"].x <= outer["to"].x \
			and inner["to"].y <= outer["to"].y and inner["to"].z <= outer["to"].z


func _test_moon() -> void:
	print("moon")
	var moon := Vector3(0.2, 0.5, 0.8).normalized()
	var day := _sun(0.0, SUN_30)
	var s := _schedule()
	_settle(s, 0.0, 100.0, day)
	var any_moon := false
	var moved := moon.rotated(_up(0.0), 0.3)
	var p := s.plan(_cam(0.0, 100.0), day, moved, 0.3, CLOUD_TOP_M, false, DT, [0, 1])
	for j: Dictionary in p["light"]:
		any_moon = any_moon or j["moon"]
	_check(p["light"].is_empty() and not any_moon, "Sun above -6 deg: the Moon's motion schedules nothing")
	var night := _sun(0.0, -10.0)
	var jobs: Array = []
	for i in 40:
		p = s.plan(_cam(0.0, 100.0), night, moon, 0.3, CLOUD_TOP_M, false, DT, [0, 1])
		jobs.append_array(p["light"])
	var all_moon := not jobs.is_empty()
	for j: Dictionary in jobs:
		all_moon = all_moon and j["moon"]
	_check(all_moon, "Sun below -6 deg at the level centre: light jobs carry the Moon")
	for i in 400:
		s.plan(_cam(0.0, 100.0), night, moon, 0.3, CLOUD_TOP_M, false, DT, [0, 1])
	p = s.plan(_cam(0.0, 100.0), night, moon.rotated(_up(0.0), 0.3), 0.3, CLOUD_TOP_M, false, DT, [0, 1])
	_check(not p["light"].is_empty(), "...and the Moon's motion then relights")


## World-volume Task 6: which light the schedule writes how.
func _test_light_modes() -> void:
	print("light modes")
	var M := CloudWorldVolumeLight
	var sun := _sun(0.0, SUN_30)
	# Cold start: real light only for levels filled in that plan's state, written directly (spike).
	var s := _schedule()
	var ready_ok := true
	var direct_ok := true
	var frames := 0
	while frames < FILL_FRAMES_MAX:
		var p := _plan(s, 0.0, 100.0, sun)
		frames += 1
		for j: Dictionary in p["light"]:
			ready_ok = ready_ok and bool(p["lookup"]["levels"][j["level"]]["ready"])
			direct_ok = direct_ok and j["mode"] == M.MODE_DIRECT
		if s.complete() and p["light"].is_empty() and p["bake"].is_empty():
			break
	_check(ready_ok, "cold start: real light only for filled levels (their rays read their own window)")
	_check(direct_ok, "...written directly (spike)")
	_check(not s.light_pending(0) and not s.light_stale(7), "...and every level lit once the fill completes")
	# A refresh generation: rate-limited, named by its generation.
	var thr := CloudWorldVolumeSchedule.sun_threshold_rad(0, deg_to_rad(30.0), 12000.0)
	var turned := _rotate(sun, _up(0.0), 1.1 * thr)
	var gen := s.light_generation(0)
	var rate_ok := true
	var any0 := false
	for i in 200:
		var p := _plan(s, 0.0, 100.0, turned)
		for j: Dictionary in p["light"]:
			rate_ok = rate_ok and j["mode"] == M.MODE_RATE and j["generation"] == s.light_generation(j["level"])
			any0 = any0 or j["level"] == 0
		if not s.light_stale(0):
			break
	_check(any0 and rate_ok and s.light_generation(0) == gen + 1, "a Sun trigger: one generation of rate-limited "
			+ "refresh jobs (generation %d)" % s.light_generation(0))
	# Settle: a box whose rate-limited write clamped cells is served again, alone; an older generation's box is not.
	var box := {"from": Vector3i(10, 10, 10), "to": Vector3i(20, 20, 20)}
	s.light_feedback(0, gen + 1, box)
	s.light_feedback(0, gen, {"from": Vector3i(100, 100, 10), "to": Vector3i(110, 110, 20)})  # older: ignored
	_check(s.light_stale(0), "the GPU clamped cells of a box: level 0 stale again")
	var more := 0
	var settle_ok := true
	for i in 20:
		var p := _plan(s, 0.0, 100.0, turned)
		for j: Dictionary in p["light"]:
			more += _cells(j)
			settle_ok = settle_ok and j["level"] == 0 and j["generation"] == gen + 1 and j["mode"] == M.MODE_RATE \
					and _inside_box(j, box)
		if not s.light_stale(0):
			break
	_check(settle_ok and more == _cells(box) and s.light_generation(0) == gen + 1, "...that box alone again, "
			+ "rate-limited, same generation (%d cells of %d)" % [more, _cells(box)])
	var after := 0
	for i in 20:
		after += _sum_cells(_plan(s, 0.0, 100.0, turned)["light"])
	_check(after == 0 and not s.light_stale(0), "no clamp reported: nothing more")
	# Provisional light shown, then blended out: a scroll with the light starved.
	s = _schedule()
	_settle(s, 0.0, 100.0, sun)
	s.light_cells_per_ms = 0.0
	var lam := 0.0
	var seen_blend := false
	for i in 30:
		lam += 200.0 / (A_M + 100.0)
		var p := _plan(s, lam, 100.0, sun)
		for j: Dictionary in p["light"]:
			seen_blend = seen_blend or j["mode"] == M.MODE_BLEND
	_check(not seen_blend and _any_pending(s), "light starved: provisional light, no blend before real light")
	s.light_cells_per_ms = TEST_LIGHT_CELLS_PER_MS
	var spans := {}  # box -> [first plan, last plan, first weight, last weight]
	var weights_ok := true
	for i in 200:
		var p := _plan(s, lam, 100.0, sun)
		for j: Dictionary in p["light"]:
			if j["mode"] != M.MODE_BLEND:
				continue
			weights_ok = weights_ok and j["weight"] > 0.0 and j["weight"] <= 1.0
			var key := "%d %s %s" % [j["level"], j["from"], j["to"]]
			if not spans.has(key):
				spans[key] = [i, i, j["weight"], j["weight"]]
			spans[key][1] = i
			spans[key][3] = j["weight"]
		var pending := false
		for k in CloudWorldVolumeLayout.LEVELS:
			pending = pending or s.light_pending(k)
		if not pending:
			break
	var fade_frames := int(ceil(CloudWorldVolumeSchedule.PROVISIONAL_FADE_S / DT))
	var span_ok := not spans.is_empty()
	var first_ok := true
	var last_ok := true
	var longest := 0
	for key: String in spans:
		var e: Array = spans[key]
		longest = maxi(longest, e[1] - e[0] + 1)
		span_ok = span_ok and e[1] - e[0] + 1 == fade_frames
		first_ok = first_ok and absf(e[2] - DT / CloudWorldVolumeSchedule.PROVISIONAL_FADE_S) < 1e-6
		last_ok = last_ok and e[3] >= 1.0
	_check(weights_ok and first_ok and last_ok, "provisional light that was shown is blended out: %d boxes, first "
			% spans.size() + "weight dt / %.2f s, last 1" % CloudWorldVolumeSchedule.PROVISIONAL_FADE_S)
	_check(span_ok, "...each over PROVISIONAL_FADE_S: %d plans (longest %d)" % [fade_frames, longest])


## True while some level has cells with provisional light only.
func _any_pending(s: CloudWorldVolumeSchedule) -> bool:
	for k in CloudWorldVolumeLayout.LEVELS:
		if s.light_pending(k):
			return true
	return false


func _test_pair_switch() -> void:
	print("pair switch")
	var sun := _sun(0.0, SUN_30)
	var s := _schedule()
	_settle(s, 0.0, 100.0, sun)
	var p := _plan(s, 0.0, 100.0, sun, 0.3, false, Vector3(0.0, 0.6, 0.8), [2, 1])
	_check(not p["lookup"]["slot_ready"][0] and p["lookup"]["slot_ready"][1], "a new snapshot in slot 0: slot 0 not ready")
	var only0 := true
	var first_level := -1
	var total := 0
	var coarse_first := true
	var last_level := 8
	var frames := 0
	while not p["lookup"]["slot_ready"][0] and frames < 3 * FILL_FRAMES_MAX:
		for j: Dictionary in p["bake"]:
			only0 = only0 and j["snapshot_channel"] == 0
			total += _cells(j)
			coarse_first = coarse_first and j["level"] <= last_level
			last_level = j["level"]
		p = _plan(s, 0.0, 100.0, sun, 0.3, false, Vector3(0.0, 0.6, 0.8), [2, 1])
		frames += 1
	for j: Dictionary in p["bake"]:
		only0 = only0 and j["snapshot_channel"] == 0
		total += _cells(j)
	_check(p["lookup"]["slot_ready"][0] and only0, "slot 0's channel pair is rebaked alone (slot 1 untouched)")
	_check(total == 8 * 256 * 256 * 128, "...every active level in full (%d cells, %d frames)" % [total, frames])
	_check(coarse_first, "...coarse to fine")
	# During a cold fill: the slot rebake covers only what the fill baked already - no cell baked twice for slot 0.
	s = _schedule()
	var baked := 0
	for i in 20:
		baked += _sum_cells(_plan(s, 0.0, 100.0, sun)["bake"])
	var both := baked
	var slot_only := 0
	frames = 0
	p = _plan(s, 0.0, 100.0, sun, 0.3, false, Vector3(0.0, 0.6, 0.8), [2, 1])
	while frames < FILL_FRAMES_MAX:
		for j: Dictionary in p["bake"]:
			if j["snapshot_channel"] == 0:
				slot_only += _cells(j)
			elif j["snapshot_channel"] == -1:
				both += _cells(j)
		if s.complete():
			break
		p = _plan(s, 0.0, 100.0, sun, 0.3, false, Vector3(0.0, 0.6, 0.8), [2, 1])
		frames += 1
	var twice := both + slot_only - 8 * 256 * 256 * 128
	_check(s.complete() and slot_only == baked and twice == baked, "a slot change during a fill rebakes the %d cells "
			% baked + "baked before it alone (%d rebaked, %d both-snapshot cells beyond one fill)" % [slot_only,
			twice - baked])


func _test_jump() -> void:
	print("jump")
	var sun := _sun(0.0, SUN_30)
	var s := _schedule()
	_settle(s, 0.0, 100.0, sun)
	var p := _plan(s, 0.7, 9000.0, sun, 0.3, true)
	var lv: Array = p["lookup"]["levels"]
	_check(_sum_cells(p["bake"], 7) > 0 and _sum_cells(p["bake"]) == _sum_cells(p["bake"], 7) and not lv[7]["ready"],
			"jumped: the coarsest level is baked first, nothing of the old place shown")
	_check(not lv[3]["ready"] and not lv[0]["ready"], "...the finer levels follow")
	_check(_sum_cells(p["light"], 7) + _sum_cells(p["provisional"], 7) >= _sum_cells(p["bake"], 7),
			"...with real or provisional light for what it baked")
	var frames := 1
	while not s.complete() and frames < FILL_FRAMES_MAX:
		_plan(s, 0.7, 9000.0, sun)
		frames += 1
	_check(s.complete() and frames <= 120, "schedule logic: full volume back in %d frames at test rate %s " % [frames,
			String.num_scientific(TEST_TAPS_PER_MS)] + "taps / ms")


func _test_reframe_schedule() -> void:
	print("re-frame")
	var sun := _sun(0.0, SUN_30)
	var s := _schedule()
	_settle(s, 0.0, 100.0, sun)
	var old_frame: Vector3 = _plan(s, 0.0, 100.0, sun)["lookup"]["levels"][7]["frame_origin_earth_m"]
	var rate := 7800.0 * DT / (A_M + 100.0)
	var lam := 0.0
	var frames := 0
	while lam * (A_M + 100.0) < 1.0e6 and frames < 20000:
		lam += rate
		_plan(s, lam, 100.0, sun)
		frames += 1
	var ok_shown := true
	var spike_ok := true
	var seq: Array[int] = []
	var swaps := 0
	var done_frames := 0
	var finished := false
	while done_frames < 800 and not finished:
		lam += rate
		var p := _plan(s, lam, 100.0, sun)
		done_frames += 1
		var rf: Dictionary = p["reframe"]
		if rf["level"] >= 0 and (seq.is_empty() or seq[-1] != rf["level"]):
			seq.append(rf["level"])
		for e: Dictionary in p["lookup"]["levels"]:
			ok_shown = ok_shown and (e["ready"] or not e["active"])
		var est: float = p["bake_taps"] / s.bake_taps_per_ms + p["light_cells"] / s.light_cells_per_ms
		spike_ok = spike_ok and est <= CloudWorldVolumeSchedule.UPKEEP_SPIKE_MS + 1e-6
		finished = rf["level"] < 0 and s.complete() and not (p["lookup"]["levels"][7]["frame_origin_earth_m"] == old_frame)
	_check(finished, "the re-frame finishes (%d frames)" % done_frames)
	var coarse_first := not seq.is_empty() and seq[0] == 7
	for i in range(1, seq.size()):
		coarse_first = coarse_first and seq[i] < seq[i - 1]
	_check(coarse_first and seq.size() == 8, "one level at a time, coarsest first: %s" % str(seq))
	_check(ok_shown, "every level has a shown version in every plan")
	_check(spike_ok, "re-frame plans stay within UPKEEP_SPIKE_MS")
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(_plan(s, lam, 100.0, sun)["lookup"]["levels"][7]["frame_origin_earth_m"])
	_check(not layout.reframe_needed(_cam(lam, 100.0)), "the new frame is at the camera")


func _check(ok: bool, what: String) -> void:
	if ok:
		print("  ok    %s" % what)
	else:
		_failures += 1
		printerr("  FAIL  %s" % what)
