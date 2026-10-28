class_name CloudViewCacheSchedule
extends RefCounted
## What the cloud view cache marches next (pure logic, no GPU; pattern of CloudLightCacheSchedule). Per tile of a
## CloudViewCacheLayout it holds the level reached (-1: nothing yet), the state the tile was rendered with (anchor, Sun
## direction, weather weight, light cache generation), whether that state is stale against the current light, and the
## nearest cached cloud distance (the parallax depth, read back from the cache by the caller). pick() returns the
## tiles to march this frame in priority order under a budget.
##
## Priority (spec C): (1) tiles in the view (plus VIEW_MARGIN_RAD: a fast head turn) without level 0; (2) visible tiles
## whose parallax error exceeds PARALLAX_REFRESH_PX, by error times nearness; (3) visible tiles below the full level;
## (4) tiles stale in light / weather / anchor; (5) the rest below the full level. Each tile counts in the first
## class it fits; within a class nearer the view first. A tile that pick() returned stays as it was until completed()
## reports it: the caller completes what it marches in the same frame.
##
## The parallax of class 2 is the one a new march would remove: the offset between the anchor the tile was rendered
## from and the CURRENT anchor (the viewpoint a march starts from). A camera that left the anchor is the re-anchor's
## business (it moves the anchor, and every tile then shows that offset); refilling from the same anchor cannot reduce
## it.

## [E] A tile is stale when the Sun turned more than this from the direction it was rendered with: ~0.1 deg, one pixel
## of the evening light's gradient (a cloud edge's colour moves < 1 pixel).
const SUN_REFRESH_RAD := 0.002
## [E] A tile is stale when the weather weight (the field's blend) moved more than this from the one it was rendered
## with: a 2 % step of the snapshot mix (the light cache refreshes at 5 %; the view cache sits well below it).
const WEATHER_REFRESH := 0.02
## [E] mark_light() rescans the tiles when the light moved this fraction of the thresholds above since the last scan
## started (or the generation changed): the staleness is noticed within 1.5 thresholds, and a Sun drifting every frame
## starts a pass only every ~0.05 deg.
const LIGHT_SCAN_FRACTION := 0.5
## [E] Blocks one mark_light() call rescans (a pass over the ~300 blocks spans several frames: a block of 64 tiles costs
## ~40 us in GDScript of the 1 ms per-frame bound; at 90 Hz a pass takes ~3.3 s, the Sun moves ~0.02 deg in that time).
const LIGHT_SCAN_BLOCKS := 1
## [E] Padding (rad, ~6 deg) of the cached list of blocks around the view's cone: the list is rebuilt only when the view
## turns more than this (a pass over all blocks), not every frame.
const CONE_PAD_RAD := 0.1
## [E] Blocks one pick() call expands (classifies tile by tile) at most: a block costs 0.1 - 0.25 ms, so the call stays
## well inside the 1 ms per-frame bound whatever is open; the rest follows in the next frames. So a pick returns at most
## 2 blocks = 128 tiles, however large the budget.
const MAX_BLOCK_EXPANSIONS := 2
## [E] Parallax error (cache texels ~ eye pixels) above which a visible tile is re-marched: half an eye pixel.
const PARALLAX_REFRESH_PX := 0.5
## [E] The margin around the view (rad, ~20 deg) filled with at least level 0: what a fast head turn reveals within
## the time the cache needs to refine it.
const VIEW_MARGIN_RAD := 0.35
## [E] A tile rendered from an anchor farther than this from the current one is stale (the anchor moves by a
## re-anchor; 1 m is far below any parallax the anchor offset makes at cloud distances).
const ANCHOR_MOVE_M := 1.0
## The full level (CloudViewCacheLayout.LEVELS - 1).
const MAX_LEVEL := CloudViewCacheLayout.LEVELS - 1
## [E] Tiles per side of a block: the coarse grid pick() culls with (a block holds 64 tiles: ~300 blocks cover the
## cube, so a converged cache costs a few hundred array reads; smaller blocks cost more reads, larger ones classify
## coarser).
const BLOCK_TILES := 8

## Keys of the sort: a 30-bit quantised priority above the 21-bit tile index (exact in a double: 2^51 < 2^53; 21 bits
## hold 2 M tiles, the layouts have < 20 k).
const _KEY_MAX := 1073741823
const _TILE_RANGE := 2097152.0
## [E] Quantisation of 1 - cos(angle) (1e-9 ~ 4.5e-5 rad at 1 rad: far below a tile's 0.02 rad) and of error x nearness
## (m^-1; 1e7 keeps the ordering of errors from 1e-7 to 100 px per metre inside the 30 bits).
const _DOT_SCALE := 5.0e8
const _NEARNESS_SCALE := 1.0e7

var layout: CloudViewCacheLayout

var _dirs := PackedVector3Array()  # the layout's tile centre directions
var _parallax_px_per_rad := 0.0  # cache texels per radian at a face centre (what the parallax error is measured in)

var _level := PackedInt32Array()
var _anchor_earth_m := PackedVector3Array()
var _sun_dir := PackedVector3Array()
var _weather_weight := PackedFloat64Array()
var _light_generation := PackedInt32Array()
var _stale := PackedByteArray()
var _near_m := PackedFloat64Array()
## The anchor epoch a tile was rendered in (-1: from a different anchor than the epoch's). A tile is "off" when it holds a
## level and its stamp is not the current epoch: it was rendered from an anchor farther than ANCHOR_MOVE_M from the
## current one (each anchor move starts a new epoch: O(blocks), no pass over the tiles).
var _stamp := PackedInt32Array()

var _current_sun := Vector3.ZERO
var _current_weight := 0.0
var _current_generation := 0
var _current_anchor := Vector3(NAN, NAN, NAN)
## The light of the last tile scan (mark_light gates the scan on the movement since); no scan yet: _have_light false.
var _scanned_sun := Vector3.ZERO
var _scanned_weight := 0.0
var _scanned_generation := 0
var _have_light := false
## The amortised light pass: mark_light() rescans LIGHT_SCAN_BLOCKS blocks from _scan_cursor per call while active.
var _scan_active := false
var _scan_cursor := 0
var _anchor_epoch := 0
var _cos_sun_refresh := cos(SUN_REFRESH_RAD)
var _cos_sun_scan := cos(SUN_REFRESH_RAD * LIGHT_SCAN_FRACTION)

var _blocks_per_side := 0
var _block_dirs := PackedVector3Array()
var _block_radius := PackedFloat64Array()
var _block_cos_radius := PackedFloat64Array()
var _block_sin_radius := PackedFloat64Array()
var _block_first_tile := PackedInt32Array()
var _block_width := PackedInt32Array()
var _block_height := PackedInt32Array()
var _block_open := PackedInt32Array()
var _block_missing := PackedInt32Array()  # tiles without a level
var _block_off := PackedInt32Array()  # tiles rendered from another anchor than the current one
var _block_min_near_m := PackedFloat64Array()
var _block_near_dirty := PackedByteArray()
## Upper bound of the offset (m) between the current anchor and the anchor of any off tile; exactly 0 while no tile is off.
## It grows by the length of every anchor move and returns to 0 when the last off tile is re-marched.
var _max_anchor_dev_m := 0.0
var _total_off := 0
var _cone_valid := false
var _cone_centre := Vector3.ZERO
var _cone_angle := 0.0
var _cone_blocks := PackedInt32Array()


func _init(layout_: CloudViewCacheLayout) -> void:
	layout = layout_
	_dirs = layout.tile_dirs()
	_parallax_px_per_rad = 0.5 * layout.face_texels
	var n := layout.tile_count()
	_level.resize(n)
	_anchor_earth_m.resize(n)
	_sun_dir.resize(n)
	_weather_weight.resize(n)
	_light_generation.resize(n)
	_stale.resize(n)
	_near_m.resize(n)
	_stamp.resize(n)
	_build_blocks()
	invalidate_all()


## Forgets everything (a camera or clock jump, a field change): every tile restarts from nothing, nearest distances
## unknown.
func invalidate_all() -> void:
	_level.fill(-1)
	_stale.fill(0)
	_near_m.fill(INF)
	_block_min_near_m.fill(INF)
	_block_near_dirty.fill(0)
	_block_off.fill(0)
	_total_off = 0
	_max_anchor_dev_m = 0.0
	for block in range(_block_open.size()):
		_block_open[block] = _block_width[block] * _block_height[block]
		_block_missing[block] = _block_open[block]


## Sets the cloud distance (m) the tile's parallax error uses: the nearest cached cloud distance of the tile
## (INF: no cloud, no parallax error).
func set_tile_distance(tile: int, near_m: float) -> void:
	var old := _near_m[tile]
	_near_m[tile] = near_m
	var block := _block_of(tile)
	# The block's nearest distance follows incrementally; it is recomputed (lazily) only when its holder moved away.
	if near_m > 0.0 and near_m <= _block_min_near_m[block]:
		_block_min_near_m[block] = near_m
	elif old <= _block_min_near_m[block]:
		_block_near_dirty[block] = 1


## set_tile_distance for a fill's read-back (cloud view cache Task 4: up to a few hundred tiles a frame on the main
## thread): tiles[i] gets the float at byte 4 i of bits (<= 0: no cloud, INF).
func set_tile_distances(tiles: PackedInt32Array, bits: PackedByteArray) -> void:
	var per_side := layout.tiles_per_side
	for i in range(mini(tiles.size(), bits.size() / 4)):
		var tile := tiles[i]
		var near_m := bits.decode_float(4 * i)
		if not near_m > 0.0:
			near_m = INF
		var old := _near_m[tile]
		_near_m[tile] = near_m
		var block := (tile / (per_side * per_side) * _blocks_per_side + (tile / per_side) % per_side / BLOCK_TILES) \
				* _blocks_per_side + tile % per_side / BLOCK_TILES
		if near_m <= _block_min_near_m[block]:
			_block_min_near_m[block] = near_m
		elif old <= _block_min_near_m[block]:
			_block_near_dirty[block] = 1


## The current light: tiles whose rendered state differs by more than SUN_REFRESH_RAD or WEATHER_REFRESH, or by the
## light cache generation, become stale (compared with the state they were rendered with: slow drift adds up). The
## comparison is a pass over the blocks, amortised: a pass starts when the light moved LIGHT_SCAN_FRACTION of a
## threshold (or the generation changed) since the previous pass started, and every call rescans LIGHT_SCAN_BLOCKS
## blocks from a persistent cursor, so a pass spans several frames (light_scan_pending()). A tile whose staleness is
## not yet noticed is merely picked later; a light that keeps moving keeps passes running back to back.
func mark_light(sun_dir: Vector3, weather_weight: float, light_generation: int) -> void:
	var sun := sun_dir.normalized()
	_current_sun = sun
	_current_weight = weather_weight
	_current_generation = light_generation
	if not _scan_active:
		if _have_light and light_generation == _scanned_generation \
				and absf(weather_weight - _scanned_weight) <= WEATHER_REFRESH * LIGHT_SCAN_FRACTION \
				and sun.dot(_scanned_sun) >= _cos_sun_scan:
			return
		_scanned_sun = sun
		_scanned_weight = weather_weight
		_scanned_generation = light_generation
		_have_light = true
		_scan_active = true
		_scan_cursor = 0
	var last := mini(_scan_cursor + LIGHT_SCAN_BLOCKS, _block_open.size())
	for block in range(_scan_cursor, last):
		_scan_block(block)
	_scan_cursor = last
	_scan_active = last < _block_open.size()


## True while a light pass has blocks left to rescan (the staleness flags are complete once it is false).
func light_scan_pending() -> bool:
	return _scan_active


## Reports a finished march: the tile now holds level, rendered with state {anchor_earth_m: Vector3, sun_dir: Vector3,
## weather_weight: float, light_generation: int}.
func completed(tile: int, level: int, state: Dictionary) -> void:
	var block := _block_of(tile)
	var was_open := _is_open(tile)
	var was_off := _is_off(tile)
	if _level[tile] < 0 and level >= 0:
		_block_missing[block] -= 1
	_level[tile] = level
	var anchor: Vector3 = state["anchor_earth_m"]
	_anchor_earth_m[tile] = anchor
	_stamp[tile] = _anchor_epoch if is_nan(_current_anchor.x) or anchor.distance_to(_current_anchor) <= ANCHOR_MOVE_M \
			else -1
	_sun_dir[tile] = (state["sun_dir"] as Vector3).normalized()
	_weather_weight[tile] = state["weather_weight"]
	_light_generation[tile] = state["light_generation"]
	_stale[tile] = 1 if _differs(tile) else 0
	var is_open := _is_open(tile)
	if was_open != is_open:
		_block_open[block] += 1 if is_open else -1
	var is_off := _is_off(tile)
	if was_off != is_off:
		var step := 1 if is_off else -1
		_block_off[block] += step
		_total_off += step
		if _total_off == 0:
			_max_anchor_dev_m = 0.0


## completed() for many tiles of one state (tiles[i] now holds levels[i]): the same result, the state read once and the
## per-tile helpers inlined (cloud view cache Task 4: ~128 tiles a frame on the main thread).
func completed_batch(tiles: PackedInt32Array, levels: PackedInt32Array, state: Dictionary) -> void:
	var anchor: Vector3 = state["anchor_earth_m"]
	var stamp := _anchor_epoch if is_nan(_current_anchor.x) or anchor.distance_to(_current_anchor) <= ANCHOR_MOVE_M \
			else -1
	var sun := (state["sun_dir"] as Vector3).normalized()
	var weight: float = state["weather_weight"]
	var generation: int = state["light_generation"]
	var stale := 1 if _have_light and (sun.dot(_current_sun) < _cos_sun_refresh \
			or absf(weight - _current_weight) > WEATHER_REFRESH or generation != _current_generation) else 0
	var per_side := layout.tiles_per_side
	for i in range(tiles.size()):
		var tile := tiles[i]
		var level := levels[i]
		var block := (tile / (per_side * per_side) * _blocks_per_side + (tile / per_side) % per_side / BLOCK_TILES) \
				* _blocks_per_side + tile % per_side / BLOCK_TILES
		var old := _level[tile]
		var was_open := old < MAX_LEVEL or _stale[tile] != 0 or _stamp[tile] != _anchor_epoch
		var was_off := old >= 0 and _stamp[tile] != _anchor_epoch
		if old < 0 and level >= 0:
			_block_missing[block] -= 1
		_level[tile] = level
		_anchor_earth_m[tile] = anchor
		_stamp[tile] = stamp
		_sun_dir[tile] = sun
		_weather_weight[tile] = weight
		_light_generation[tile] = generation
		_stale[tile] = stale
		var is_open := level < MAX_LEVEL or stale != 0 or stamp != _anchor_epoch
		if was_open != is_open:
			_block_open[block] += 1 if is_open else -1
		var is_off := level >= 0 and stamp != _anchor_epoch
		if was_off != is_off:
			var step := 1 if is_off else -1
			_block_off[block] += step
			_total_off += step
			if _total_off == 0:
				_max_anchor_dev_m = 0.0


## True while a tile whose centre lies within cone_angle (rad) of centre (a unit direction) holds no level: the blocks
## with tiles missing are tested against the cone, those on its boundary tile by tile (cloud view cache Task 4: the
## per-frame march covers the view until it has level 0 everywhere).
func missing_in_cone(centre: Vector3, cone_angle: float) -> bool:
	var cos_cone := cos(minf(cone_angle, PI))
	for block in range(_block_missing.size()):
		if _block_missing[block] <= 0:
			continue
		var angle := acos(clampf(_block_dirs[block].dot(centre), -1.0, 1.0))
		if angle - _block_radius[block] > cone_angle:
			continue
		if angle + _block_radius[block] <= cone_angle:
			return true
		var per_side := layout.tiles_per_side
		for dy in range(_block_height[block]):
			var row := _block_first_tile[block] + dy * per_side
			for tile in range(row, row + _block_width[block]):
				if _level[tile] < 0 and _dirs[tile].dot(centre) >= cos_cone:
					return true
	return false


## The level a tile holds (-1: none).
func level_of(tile: int) -> int:
	return _level[tile]


## Stale in light / weather, or rendered from another anchor than the current one.
func is_stale(tile: int) -> bool:
	return _stale[tile] != 0 or _is_off(tile)


## The parallax error (cache texels ~ eye pixels) of a tile that was rendered from anchor_earth_m and is looked at
## from camera_earth_m: the viewpoint's offset across the tile's direction over the cloud distance, at the cache's
## centre density. pick() uses the CURRENT anchor as the viewpoint (what a new march would remove); the re-anchor
## (Task 5) passes the camera (what moving the anchor to it would remove). 0 without a known cloud distance.
func parallax_error_px(tile: int, camera_earth_m: Vector3, anchor_earth_m: Vector3) -> float:
	var near := _near_m[tile]
	if is_inf(near) or near <= 0.0:
		return 0.0
	var offset := camera_earth_m - anchor_earth_m
	var dir := _dirs[tile]
	var across := (offset - dir * offset.dot(dir)).length()
	return across / near * _parallax_px_per_rad


## The tiles to march this frame, at most budget_tiles, in priority order: [{tile, level}]. anchor_earth_m: the anchor
## a march starts from now (a tile rendered from another anchor is stale, and its offset is the parallax of class 2);
## the camera does not matter here (see parallax_error_px). view_dirs: unit sample rays of the view (Earth-local; tens
## of them: the frustum's corners, edges and centre per eye) - a tile is visible when its centre is within
## view_sample_rad (+ the tile's own radius) of one (the samples' spacing at most twice view_sample_rad), and in the
## margin when within VIEW_MARGIN_RAD more.
## Cost, bounded per call: only blocks (BLOCK_TILES squared tiles) with something open are visited (a converged cache
## costs a pass over the cone's block counters), nearest to the view's cone centre first (a list cached while the view
## stays inside CONE_PAD_RAD: approximate between blocks), and at most max_expansions (MAX_BLOCK_EXPANSIONS) of them
## are classified tile by tile - blocks with tiles missing or a
## possible parallax inside the margin first, then the other blocks of the margin, then those outside it - each group
## only while the budget is not met and no earlier group was cut off by the cap (so no refinement is returned while a
## nearer block may still hold tiles without level 0). Within the expanded blocks the order is exact.
func pick(_camera_earth_m: Vector3, anchor_earth_m: Vector3, view_dirs: Array[Vector3], budget_tiles: int,
		view_sample_rad := 0.0, max_expansions := MAX_BLOCK_EXPANSIONS) -> Array[Dictionary]:
	var picked: Array[Dictionary] = []
	if budget_tiles <= 0:
		return picked
	_sync_anchor(anchor_earth_m)
	var block_count := _block_open.size()
	var has_view := not view_dirs.is_empty()
	var tile_radius := layout.tile_radius_rad()
	var visible_angle := minf(view_sample_rad + tile_radius, PI)
	var margin_angle := minf(view_sample_rad + tile_radius + VIEW_MARGIN_RAD, PI)
	var zones := Vector4(visible_angle, margin_angle, cos(visible_angle), cos(margin_angle))
	# The view's cone: its centre and the margin's reach from it; a block outside the cone is outside the margin.
	var centre := Vector3.ZERO
	var cone_angle := PI
	if has_view:
		for d in view_dirs:
			centre += d
		if centre.length() > 1.0e-6:
			centre = centre.normalized()
			var least := 1.0
			for d in view_dirs:
				least = minf(least, centre.dot(d))
			cone_angle = minf(acos(clampf(least, -1.0, 1.0)) + margin_angle, PI)
	var urgent_blocks := PackedInt32Array()  # in the margin cone, tiles missing or a parallax possible
	var later_blocks := PackedInt32Array()  # in the margin cone, only stale or below-full tiles
	var cos_cone := cos(cone_angle)
	if has_view:
		_update_cone(centre, cone_angle)
		# The blocks a parallax can show in: the nearest cloud of the block closer than this (m).
		var parallax_reach_m := _max_anchor_dev_m * _parallax_px_per_rad / PARALLAX_REFRESH_PX
		# _cone_blocks is ordered nearest first: only as many blocks as pick() can expand are collected (one more
		# urgent block says that the group is cut off), so a pass ends as soon as the nearest ones are found.
		for block in _cone_blocks:
			if _block_open[block] <= 0:
				continue
			if _block_missing[block] > 0 or (_block_off[block] > 0 and (_block_min_near_m[block] < parallax_reach_m
					if _block_near_dirty[block] == 0 else _parallax_possible(block))):
				urgent_blocks.append(block)
				if urgent_blocks.size() > max_expansions:
					break
			elif later_blocks.size() < max_expansions:
				later_blocks.append(block)
	var classes: Array[PackedFloat64Array] = [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array(),
			PackedFloat64Array(), PackedFloat64Array()]
	var expansions := 0
	var cut_off := false
	for block in urgent_blocks:
		if classes[0].size() + classes[1].size() >= budget_tiles:
			break
		if expansions >= max_expansions:
			cut_off = true
			break
		_classify_block(block, view_dirs, zones, classes)
		expansions += 1
	if not cut_off and classes[0].size() + classes[1].size() < budget_tiles:
		for block in later_blocks:
			if classes[0].size() + classes[1].size() + classes[2].size() + classes[3].size() >= budget_tiles:
				break
			if expansions >= max_expansions:
				cut_off = true
				break
			_classify_block(block, view_dirs, zones, classes)
			expansions += 1
	if cut_off:
		# Urgent blocks are left unexpanded and may hold more tiles without level 0: until the expanded ones gave
		# some, only first-level marches go out (the lower classes would refine while tiles still lack level 0).
		# Tiles of classes 2 - 4 go out only when the expanded blocks hold nothing else (progress).
		var urgent_first := not classes[0].is_empty() or not classes[1].is_empty()
		var first_levels := PackedFloat64Array()
		if not urgent_first:
			for entry in classes[4]:
				if _level[int(fmod(entry, _TILE_RANGE))] < 0:
					first_levels.append(entry)
		if urgent_first or not first_levels.is_empty():
			classes[2].clear()
			classes[3].clear()
			classes[4] = first_levels
	var wanted := classes[0].size() + classes[1].size() + classes[2].size() + classes[3].size()
	if not cut_off and wanted < budget_tiles:
		var far_blocks := _far_blocks(centre, cone_angle, cos_cone, has_view)
		far_blocks.sort()
		for entry in far_blocks:
			if wanted >= budget_tiles or expansions >= max_expansions:
				break
			wanted += _collect_far_block(int(fmod(entry, _TILE_RANGE)), int(entry / _TILE_RANGE), classes)
			expansions += 1
	for class_index in range(classes.size()):
		var keys := classes[class_index]
		keys.sort()
		for i in range(mini(keys.size(), budget_tiles - picked.size())):
			var tile := int(fmod(keys[i], _TILE_RANGE))
			var level := _level[tile]
			# Missing and the finer-level classes go one level up; refreshes (parallax, stale) keep the level.
			var next := 0 if level < 0 else (level if class_index == 1 or class_index == 3 else level + 1)
			picked.append({"tile": tile, "level": next})
		if picked.size() >= budget_tiles:
			break
	return picked


## Sorts the tiles of a block into the priority classes. zones: (visible angle, margin angle, their cosines).
func _classify_block(block: int, view_dirs: Array[Vector3], zones: Vector4, classes: Array[PackedFloat64Array]) -> void:
	var block_dot := _max_dot(_block_dirs[block], view_dirs) if not view_dirs.is_empty() else -1.0
	var block_angle := acos(clampf(block_dot, -1.0, 1.0))
	var radius := _block_radius[block]
	var block_key := _key(1.0 - block_dot, _DOT_SCALE)
	var touches_visible := block_angle - radius <= zones.x
	var inside_visible := block_angle + radius <= zones.x
	var inside_margin := block_angle + radius <= zones.y
	var outside_margin := block_angle - radius > zones.y
	# Tiles of a block on a zone's boundary get their own distance to the view; only the rays that can be nearest to
	# any tile of the block count (within the block's best angle plus two block radii of its centre).
	var per_tile := not (inside_visible or outside_margin or (inside_margin and not touches_visible))
	var near_rays: Array[Vector3] = []
	if per_tile:
		var cos_reach := cos(minf(block_angle + 2.0 * radius, PI))
		for d in view_dirs:
			if _block_dirs[block].dot(d) >= cos_reach:
				near_rays.append(d)
	var check_parallax := touches_visible and _parallax_possible(block)
	var last_anchor := Vector3(NAN, NAN, NAN)  # the offset to the current anchor is shared by tiles of one anchor
	var offset := Vector3.ZERO
	var offset_sq := 0.0
	var per_side := layout.tiles_per_side
	for dy in range(_block_height[block]):
		var row := _block_first_tile[block] + dy * per_side
		for tile in range(row, row + _block_width[block]):
			var level := _level[tile]
			if level == MAX_LEVEL and _stale[tile] == 0 and _stamp[tile] == _anchor_epoch:
				continue  # nothing left to do for this tile (the common case of a converged block)
			var visible := inside_visible
			var in_margin := inside_margin and not outside_margin
			var key := block_key
			if per_tile:
				var dot := _max_dot(_dirs[tile], near_rays)
				key = _key(1.0 - dot, _DOT_SCALE)
				visible = dot >= zones.z
				in_margin = dot >= zones.w
			var off := level >= 0 and _stamp[tile] != _anchor_epoch
			var error_px := 0.0
			if check_parallax and visible and off:
				var near := _near_m[tile]  # inline of parallax_error_px (the hot loop)
				if near > 0.0 and near < INF:
					if _anchor_earth_m[tile] != last_anchor:
						last_anchor = _anchor_earth_m[tile]
						offset = _current_anchor - last_anchor
						offset_sq = offset.length_squared()
					var along := offset.dot(_dirs[tile])
					var across_sq := offset_sq - along * along
					error_px = sqrt(across_sq) / near * _parallax_px_per_rad if across_sq > 0.0 else 0.0
			if level < 0 and in_margin:
				classes[0].append(float(key) * _TILE_RANGE + float(tile))
			elif error_px > PARALLAX_REFRESH_PX:
				var score := int(error_px / _near_m[tile] * _NEARNESS_SCALE)  # error times nearness
				classes[1].append(float(_KEY_MAX - (score if score < _KEY_MAX else _KEY_MAX)) * _TILE_RANGE + float(tile))
			elif visible and level < MAX_LEVEL:
				classes[2].append(float(key) * _TILE_RANGE + float(tile))
			elif level >= 0 and (_stale[tile] != 0 or off):
				classes[3].append(float(key) * _TILE_RANGE + float(tile))
			elif level < MAX_LEVEL:
				classes[4].append(float(key) * _TILE_RANGE + float(tile))


## Keeps _cone_blocks: the blocks that can touch the margin cone (centre, cone_angle), nearest to its centre first. The
## list is built with a padding (CONE_PAD_RAD) and reused while the view stays inside it, so the per-frame pass visits
## only those blocks.
func _update_cone(centre: Vector3, cone_angle: float) -> void:
	if _cone_valid and cone_angle + acos(clampf(centre.dot(_cone_centre), -1.0, 1.0)) <= _cone_angle:
		return
	_cone_valid = true
	_cone_centre = centre
	_cone_angle = minf(cone_angle + CONE_PAD_RAD, PI)
	var entries := PackedFloat64Array()
	for block in range(_block_dirs.size()):
		var dot := _block_dirs[block].dot(centre)
		if acos(clampf(dot, -1.0, 1.0)) - _block_radius[block] <= _cone_angle:
			entries.append(_encode(_key(1.0 - dot, _DOT_SCALE), block))
	entries.sort()  # nearest to the cone's centre first
	_cone_blocks = PackedInt32Array()
	for entry in entries:
		_cone_blocks.append(int(fmod(entry, _TILE_RANGE)))


## The open blocks outside the margin cone, keyed by their distance to the cone's centre.
func _far_blocks(centre: Vector3, cone_angle: float, cos_cone: float, has_view: bool) -> PackedFloat64Array:
	var far_blocks := PackedFloat64Array()
	for block in range(_block_open.size()):
		if _block_open[block] <= 0:
			continue
		var dot := _block_dirs[block].dot(centre) if has_view else -1.0
		if not has_view or (cone_angle + _block_radius[block] < PI and dot * _block_cos_radius[block]
				+ sqrt(maxf(1.0 - dot * dot, 0.0)) * _block_sin_radius[block] < cos_cone):
			far_blocks.append(_encode(_key(1.0 - dot, _DOT_SCALE), block))
	return far_blocks


## Fills classes 3 / 4 from a block outside the margin cone (its tiles are ordered by their block's distance to the
## view); returns the number of tiles gathered.
func _collect_far_block(block: int, block_key: int, classes: Array[PackedFloat64Array]) -> int:
	var gathered := 0
	var per_side := layout.tiles_per_side
	for dy in range(_block_height[block]):
		var row := _block_first_tile[block] + dy * per_side
		for tile in range(row, row + _block_width[block]):
			if _level[tile] >= 0 and (_stale[tile] != 0 or _stamp[tile] != _anchor_epoch):
				classes[3].append(_encode(block_key, tile))
				gathered += 1
			elif _level[tile] < MAX_LEVEL:
				classes[4].append(_encode(block_key, tile))
				gathered += 1
	return gathered


## A new anchor starts a new epoch: every tile that holds a level is off (rendered from another anchor) and so open.
## O(blocks). The first anchor ever seen adopts the tiles completed before it with one pass over them.
func _sync_anchor(anchor_earth_m: Vector3) -> void:
	if not is_nan(_current_anchor.x) and _current_anchor.distance_to(anchor_earth_m) <= ANCHOR_MOVE_M:
		return
	var previous := _current_anchor
	_current_anchor = anchor_earth_m
	_anchor_epoch += 1
	if is_nan(previous.x):
		_adopt_anchor()
		return
	_max_anchor_dev_m += previous.distance_to(anchor_earth_m)
	_total_off = 0
	for block in range(_block_open.size()):
		var tiles := _block_width[block] * _block_height[block]
		_block_off[block] = tiles - _block_missing[block]
		_total_off += _block_off[block]
		_block_open[block] = tiles


## The first anchor: tiles completed before any anchor was known are re-stamped against it (one pass, once).
func _adopt_anchor() -> void:
	_total_off = 0
	_max_anchor_dev_m = 0.0
	var per_side := layout.tiles_per_side
	for block in range(_block_open.size()):
		var off := 0
		for dy in range(_block_height[block]):
			var row := _block_first_tile[block] + dy * per_side
			for tile in range(row, row + _block_width[block]):
				if _level[tile] < 0:
					continue
				var dev := _anchor_earth_m[tile].distance_to(_current_anchor)
				if dev > ANCHOR_MOVE_M:
					_stamp[tile] = -1
					off += 1
					_max_anchor_dev_m = maxf(_max_anchor_dev_m, dev)
				else:
					_stamp[tile] = _anchor_epoch
		_block_off[block] = off
		_total_off += off
		_scan_block(block)


## Light staleness only (the anchor is the epoch's business).
func _differs(tile: int) -> bool:
	if not _have_light:  # no light marked yet: nothing to compare with
		return false
	return _sun_dir[tile].dot(_current_sun) < _cos_sun_refresh \
			or absf(_weather_weight[tile] - _current_weight) > WEATHER_REFRESH \
			or _light_generation[tile] != _current_generation


## Rescans one block against the current light, recounting its open tiles.
func _scan_block(block: int) -> void:
	var open := 0
	var per_side := layout.tiles_per_side
	for dy in range(_block_height[block]):
		var row := _block_first_tile[block] + dy * per_side
		for tile in range(row, row + _block_width[block]):
			if _level[tile] >= 0:
				_stale[tile] = 1 if _differs(tile) else 0
				if _stale[tile] != 0 or _level[tile] < MAX_LEVEL or _stamp[tile] != _anchor_epoch:
					open += 1
			else:
				open += 1
	_block_open[block] = open


func _is_off(tile: int) -> bool:
	return _level[tile] >= 0 and _stamp[tile] != _anchor_epoch


## Open: something left for the background classes (below the full level, stale, or rendered from another anchor).
func _is_open(tile: int) -> bool:
	return _level[tile] < MAX_LEVEL or _stale[tile] != 0 or _stamp[tile] != _anchor_epoch


static func _key(one_minus_dot: float, scale: float) -> int:
	return clampi(int(maxf(one_minus_dot, 0.0) * scale), 0, _KEY_MAX)


static func _encode(key: int, index: int) -> float:
	return float(key) * _TILE_RANGE + float(index)


static func _max_dot(dir: Vector3, view_dirs: Array[Vector3]) -> float:
	var best := -1.0
	for d in view_dirs:
		var dot := dir.dot(d)
		if dot > best:
			best = dot
	return best


func _build_blocks() -> void:
	var per_side := layout.tiles_per_side
	_blocks_per_side = ceili(float(per_side) / BLOCK_TILES)
	var count := CloudViewCacheLayout.FACES * _blocks_per_side * _blocks_per_side
	_block_dirs.resize(count)
	_block_radius.resize(count)
	_block_first_tile.resize(count)
	_block_width.resize(count)
	_block_height.resize(count)
	_block_open.resize(count)
	_block_missing.resize(count)
	_block_off.resize(count)
	_block_cos_radius.resize(count)
	_block_sin_radius.resize(count)
	_block_min_near_m.resize(count)
	_block_near_dirty.resize(count)
	var tile_uv := float(layout.tile_texels) / float(layout.face_texels)
	for block in range(count):
		var face := block / (_blocks_per_side * _blocks_per_side)
		var bx := block % _blocks_per_side
		var by := (block / _blocks_per_side) % _blocks_per_side
		var lo := Vector2(bx, by) * BLOCK_TILES * tile_uv
		var hi := Vector2(mini((bx + 1) * BLOCK_TILES, per_side), mini((by + 1) * BLOCK_TILES, per_side)) * tile_uv
		var centre := layout.dir_of(face, (lo + hi) * 0.5)
		var radius := 0.0
		for corner in [lo, hi, Vector2(lo.x, hi.y), Vector2(hi.x, lo.y)]:
			radius = maxf(radius, centre.angle_to(layout.dir_of(face, corner)))
		_block_dirs[block] = centre
		_block_radius[block] = radius
		_block_cos_radius[block] = cos(radius)
		_block_sin_radius[block] = sin(radius)
		_block_first_tile[block] = (face * per_side + by * BLOCK_TILES) * per_side + bx * BLOCK_TILES
		_block_width[block] = mini((bx + 1) * BLOCK_TILES, per_side) - bx * BLOCK_TILES
		_block_height[block] = mini((by + 1) * BLOCK_TILES, per_side) - by * BLOCK_TILES


## Whether a tile of the block can show a parallax over PARALLAX_REFRESH_PX against the current anchor: it holds a tile
## rendered from another anchor, and the largest such offset over the block's nearest cloud distance is large enough.
func _parallax_possible(block: int) -> bool:
	if _block_off[block] <= 0 or _max_anchor_dev_m <= 0.0:
		return false
	if _block_near_dirty[block] != 0:
		_block_near_dirty[block] = 0
		var nearest := INF
		var per_side := layout.tiles_per_side
		for dy in range(_block_height[block]):
			var row := _block_first_tile[block] + dy * per_side
			for tile in range(row, row + _block_width[block]):
				if _near_m[tile] > 0.0:
					nearest = minf(nearest, _near_m[tile])
		_block_min_near_m[block] = nearest
	return _max_anchor_dev_m / _block_min_near_m[block] * 0.5 * layout.face_texels > PARALLAX_REFRESH_PX


func _block_of(tile: int) -> int:
	var per_side := layout.tiles_per_side
	var face := tile / (per_side * per_side)
	var ty := (tile / per_side) % per_side
	var tx := tile % per_side
	return (face * _blocks_per_side + ty / BLOCK_TILES) * _blocks_per_side + tx / BLOCK_TILES
