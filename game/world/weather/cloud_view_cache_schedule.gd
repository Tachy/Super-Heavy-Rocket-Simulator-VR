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
## [E] mark_light() rescans the tiles when the light moved this fraction of the thresholds above since the last scan:
## the staleness is noticed within 1.5 thresholds, and a Sun drifting every frame costs a scan only every ~0.05 deg.
const LIGHT_SCAN_FRACTION := 0.5
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

var _level := PackedInt32Array()
var _anchor_earth_m := PackedVector3Array()
var _sun_dir := PackedVector3Array()
var _weather_weight := PackedFloat64Array()
var _light_generation := PackedInt32Array()
var _stale := PackedByteArray()
var _near_m := PackedFloat64Array()

var _current_sun := Vector3.ZERO
var _current_weight := 0.0
var _current_generation := 0
var _current_anchor := Vector3(NAN, NAN, NAN)
## The light of the last tile scan (mark_light gates the scan on the movement since); no scan yet: _have_light false.
var _scanned_sun := Vector3.ZERO
var _scanned_weight := 0.0
var _scanned_generation := 0
var _have_light := false
var _cos_sun_refresh := cos(SUN_REFRESH_RAD)
var _cos_sun_scan := cos(SUN_REFRESH_RAD * LIGHT_SCAN_FRACTION)

var _blocks_per_side := 0
var _block_dirs := PackedVector3Array()
var _block_radius := PackedFloat64Array()
var _block_first_tile := PackedInt32Array()
var _block_width := PackedInt32Array()
var _block_height := PackedInt32Array()
var _block_open := PackedInt32Array()
var _block_missing := PackedInt32Array()  # tiles without a level
var _block_min_near_m := PackedFloat64Array()
var _block_near_dirty := PackedByteArray()
var _max_anchor_dev_m := 0.0
var _scratch_dot := PackedFloat64Array()  # pick(): per block, the best dot with the view rays


func _init(layout_: CloudViewCacheLayout) -> void:
	layout = layout_
	var n := layout.tile_count()
	_level.resize(n)
	_anchor_earth_m.resize(n)
	_sun_dir.resize(n)
	_weather_weight.resize(n)
	_light_generation.resize(n)
	_stale.resize(n)
	_near_m.resize(n)
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
	_max_anchor_dev_m = 0.0
	for block in range(_block_open.size()):
		_block_open[block] = _block_width[block] * _block_height[block]
		_block_missing[block] = _block_open[block]


## Sets the cloud distance (m) the tile's parallax error uses: the nearest cached cloud distance of the tile
## (INF: no cloud, no parallax error).
func set_tile_distance(tile: int, near_m: float) -> void:
	_near_m[tile] = near_m
	_block_near_dirty[_block_of(tile)] = 1


## The current light: tiles whose rendered state differs by more than SUN_REFRESH_RAD or WEATHER_REFRESH, or by the
## light cache generation, become stale (compared with the state they were rendered with: slow drift adds up). The
## tiles are rescanned only when the light moved LIGHT_SCAN_FRACTION of a threshold since the last scan (or the
## generation changed), so a Sun turning every frame costs a scan only now and then.
func mark_light(sun_dir: Vector3, weather_weight: float, light_generation: int) -> void:
	var sun := sun_dir.normalized()
	_current_sun = sun
	_current_weight = weather_weight
	_current_generation = light_generation
	if _have_light and light_generation == _scanned_generation \
			and absf(weather_weight - _scanned_weight) <= WEATHER_REFRESH * LIGHT_SCAN_FRACTION \
			and sun.dot(_scanned_sun) >= _cos_sun_scan:
		return
	_scanned_sun = sun
	_scanned_weight = weather_weight
	_scanned_generation = light_generation
	_have_light = true
	_restale()


## Reports a finished march: the tile now holds level, rendered with state {anchor_earth_m: Vector3, sun_dir: Vector3,
## weather_weight: float, light_generation: int}.
func completed(tile: int, level: int, state: Dictionary) -> void:
	var was_open := _is_open(tile)
	if _level[tile] < 0 and level >= 0:
		_block_missing[_block_of(tile)] -= 1
	_level[tile] = level
	_anchor_earth_m[tile] = state["anchor_earth_m"]
	_sun_dir[tile] = (state["sun_dir"] as Vector3).normalized()
	_weather_weight[tile] = state["weather_weight"]
	_light_generation[tile] = state["light_generation"]
	_stale[tile] = 1 if _differs(tile) else 0
	if not is_nan(_current_anchor.x):
		_max_anchor_dev_m = maxf(_max_anchor_dev_m, _anchor_earth_m[tile].distance_to(_current_anchor))
	var is_open := _is_open(tile)
	if was_open != is_open:
		_block_open[_block_of(tile)] += 1 if is_open else -1


## The level a tile holds (-1: none).
func level_of(tile: int) -> int:
	return _level[tile]


func is_stale(tile: int) -> bool:
	return _stale[tile] != 0


## The parallax error (cache texels ~ eye pixels) of a tile that was rendered from anchor_earth_m and is looked at
## from camera_earth_m: the viewpoint's offset across the tile's direction over the cloud distance, at the cache's
## centre density. pick() uses the CURRENT anchor as the viewpoint (what a new march would remove); the re-anchor
## (Task 5) passes the camera (what moving the anchor to it would remove). 0 without a known cloud distance.
func parallax_error_px(tile: int, camera_earth_m: Vector3, anchor_earth_m: Vector3) -> float:
	var near := _near_m[tile]
	if is_inf(near) or near <= 0.0:
		return 0.0
	var offset := camera_earth_m - anchor_earth_m
	var dir := layout.tile_dir(tile)
	var across := (offset - dir * offset.dot(dir)).length()
	return across / near * 0.5 * layout.face_texels


## The tiles to march this frame, at most budget_tiles, in priority order: [{tile, level}]. anchor_earth_m: the anchor
## a march starts from now (a tile rendered from another anchor is stale, and its offset is the parallax of class 2);
## the camera does not matter here (see parallax_error_px). view_dirs: unit sample rays of the view (Earth-local; tens
## of them: the frustum's corners, edges and centre per eye) - a tile is visible when its centre is within
## view_sample_rad (+ the tile's own radius) of one (the samples' spacing at most twice view_sample_rad), and in the
## margin when within VIEW_MARGIN_RAD more.
## Cost: only blocks (BLOCK_TILES squared tiles) with something open, or with a possible parallax, are visited at all
## (a converged cache costs a pass over the block counters). Blocks with tiles missing or a possible parallax are
## expanded nearest first until class 1 fills the budget; the others (classes 3 / 4 only) nearest first until
## classes 1 - 3 do; blocks wholly outside the margin hold only classes 4 / 5 and come last, likewise nearest first
## and only while the budget is not yet met. So within a class the order is exact for the tiles picked from the
## blocks expanded, and approximate (by the block's distance to the view) between blocks.
func pick(_camera_earth_m: Vector3, anchor_earth_m: Vector3, view_dirs: Array[Vector3], budget_tiles: int,
		view_sample_rad := 0.0) -> Array[Dictionary]:
	var picked: Array[Dictionary] = []
	if budget_tiles <= 0:
		return picked
	_sync_anchor(anchor_earth_m)
	var candidates := PackedInt32Array()
	for block in range(_block_open.size()):
		if _block_open[block] > 0 or _block_parallax_bound_px(block) > PARALLAX_REFRESH_PX:
			candidates.append(block)
	if candidates.is_empty():
		return picked
	var tile_radius := layout.tile_radius_rad()
	var visible_angle := minf(view_sample_rad + tile_radius, PI)
	var margin_angle := minf(view_sample_rad + tile_radius + VIEW_MARGIN_RAD, PI)
	var cos_visible := cos(visible_angle)
	var cos_margin := cos(margin_angle)
	var zones := Vector4(visible_angle, margin_angle, cos_visible, cos_margin)
	var classes: Array[PackedFloat64Array] = [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array(),
			PackedFloat64Array(), PackedFloat64Array()]
	var far_blocks := PackedFloat64Array()
	var urgent_blocks := PackedFloat64Array()  # tiles missing, or a parallax possible: classes 1 / 2 can be in them
	var later_blocks := PackedFloat64Array()  # only stale or below-full tiles: classes 3 / 4 and 5
	var has_view := not view_dirs.is_empty()
	for block in candidates:
		var block_dot := _max_dot(_block_dirs[block], view_dirs) if has_view else -1.0
		_scratch_dot[block] = block_dot
		var entry := _encode(_key(1.0 - block_dot, _DOT_SCALE), block)
		if not has_view or acos(clampf(block_dot, -1.0, 1.0)) - _block_radius[block] > margin_angle:
			if _block_open[block] > 0:
				far_blocks.append(entry)
		elif _block_missing[block] > 0 or _block_parallax_bound_px(block) > PARALLAX_REFRESH_PX:
			urgent_blocks.append(entry)
		else:
			later_blocks.append(entry)
	# Nearest block first. Without the budget met, every block of a group is expanded; once it is, the farther
	# blocks of that group are not (within a class the order is the blocks' distance to the view, tiles on a zone's
	# boundary exactly).
	urgent_blocks.sort()
	for entry in urgent_blocks:
		if classes[0].size() >= budget_tiles:
			break
		_classify_block(int(fmod(entry, _TILE_RANGE)), view_dirs, zones, classes)
	if classes[0].size() < budget_tiles:
		later_blocks.sort()
		for entry in later_blocks:
			if classes[0].size() + classes[1].size() + classes[2].size() >= budget_tiles:
				break
			_classify_block(int(fmod(entry, _TILE_RANGE)), view_dirs, zones, classes)
	var wanted := classes[0].size() + classes[1].size() + classes[2].size() + classes[3].size()
	if wanted < budget_tiles and not far_blocks.is_empty():
		_collect_far_tiles(far_blocks, classes, budget_tiles)
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


## Sorts the tiles of a block (within the margin or touching it) into the priority classes. zones: (visible angle,
## margin angle, their cosines).
func _classify_block(block: int, view_dirs: Array[Vector3], zones: Vector4, classes: Array[PackedFloat64Array]) -> void:
	var block_dot := _scratch_dot[block]
	var block_angle := acos(clampf(block_dot, -1.0, 1.0))
	var radius := _block_radius[block]
	var block_key := _key(1.0 - block_dot, _DOT_SCALE)
	var touches_visible := block_angle - radius <= zones.x
	var inside_visible := block_angle + radius <= zones.x
	var inside_margin := block_angle + radius <= zones.y
	var check_parallax := touches_visible and _block_parallax_bound_px(block) > PARALLAX_REFRESH_PX
	var per_side := layout.tiles_per_side
	for dy in range(_block_height[block]):
		var row := _block_first_tile[block] + dy * per_side
		for tile in range(row, row + _block_width[block]):
			var level := _level[tile]
			var visible := inside_visible
			var in_margin := inside_margin
			var key := block_key
			if not (inside_visible or inside_margin and not touches_visible):
				var dot := _max_dot(layout.tile_dir(tile), view_dirs)  # on a zone's boundary: exact
				key = _key(1.0 - dot, _DOT_SCALE)
				visible = dot >= zones.z
				in_margin = dot >= zones.w
			var error_px := 0.0
			if check_parallax and visible and level >= 0:
				error_px = parallax_error_px(tile, _current_anchor, _anchor_earth_m[tile])
			if level < 0 and in_margin:
				classes[0].append(_encode(key, tile))
			elif error_px > PARALLAX_REFRESH_PX:
				var score := error_px / _near_m[tile]  # error times nearness
				classes[1].append(_encode(_KEY_MAX - mini(int(score * _NEARNESS_SCALE), _KEY_MAX), tile))
			elif visible and level < MAX_LEVEL:
				classes[2].append(_encode(key, tile))
			elif level >= 0 and _stale[tile] != 0:
				classes[3].append(_encode(key, tile))
			elif level < MAX_LEVEL:
				classes[4].append(_encode(key, tile))


## Fills classes 3 / 4 from blocks outside the margin, nearest block first, until budget_tiles tiles are gathered
## (tiles there are ordered by their block's distance to the view).
func _collect_far_tiles(far_blocks: PackedFloat64Array, classes: Array[PackedFloat64Array], budget_tiles: int) -> void:
	far_blocks.sort()
	var per_side := layout.tiles_per_side
	var gathered := 0
	for entry in far_blocks:
		var block := int(fmod(entry, _TILE_RANGE))
		var block_key := int(entry / _TILE_RANGE)
		for dy in range(_block_height[block]):
			var row := _block_first_tile[block] + dy * per_side
			for tile in range(row, row + _block_width[block]):
				if _level[tile] >= 0 and _stale[tile] != 0:
					classes[3].append(_encode(block_key, tile))
					gathered += 1
				elif _level[tile] < MAX_LEVEL:
					classes[4].append(_encode(block_key, tile))
					gathered += 1
		if gathered >= budget_tiles:
			return


## A new anchor makes every tile rendered from another one stale.
func _sync_anchor(anchor_earth_m: Vector3) -> void:
	if _current_anchor.distance_to(anchor_earth_m) <= ANCHOR_MOVE_M:
		return
	_current_anchor = anchor_earth_m
	_restale()


func _anchor_moved(tile: int) -> bool:
	return not is_nan(_current_anchor.x) and _anchor_earth_m[tile].distance_to(_current_anchor) > ANCHOR_MOVE_M


func _differs(tile: int) -> bool:
	if not _have_light:  # no light marked yet: nothing to compare with
		return _anchor_moved(tile)
	return _anchor_moved(tile) or _sun_dir[tile].dot(_current_sun) < _cos_sun_refresh \
			or absf(_weather_weight[tile] - _current_weight) > WEATHER_REFRESH \
			or _light_generation[tile] != _current_generation


## Compares every rendered tile with the current light and anchor, recounting the open tiles of every block on the way.
func _restale() -> void:
	_max_anchor_dev_m = 0.0
	var have_anchor := not is_nan(_current_anchor.x)
	var per_side := layout.tiles_per_side
	for block in range(_block_open.size()):
		var open := 0
		for dy in range(_block_height[block]):
			var row := _block_first_tile[block] + dy * per_side
			for tile in range(row, row + _block_width[block]):
				if _level[tile] >= 0:
					_stale[tile] = 1 if _differs(tile) else 0
					if have_anchor:
						_max_anchor_dev_m = maxf(_max_anchor_dev_m, _anchor_earth_m[tile].distance_to(_current_anchor))
				if _level[tile] < MAX_LEVEL or _stale[tile] != 0:
					open += 1
		_block_open[block] = open


## Open: something left for the background classes (below the full level, or stale).
func _is_open(tile: int) -> bool:
	return _level[tile] < MAX_LEVEL or _stale[tile] != 0


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
	_scratch_dot.resize(count)
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
		_block_first_tile[block] = (face * per_side + by * BLOCK_TILES) * per_side + bx * BLOCK_TILES
		_block_width[block] = mini((bx + 1) * BLOCK_TILES, per_side) - bx * BLOCK_TILES
		_block_height[block] = mini((by + 1) * BLOCK_TILES, per_side) - by * BLOCK_TILES


## The most parallax (px) any tile of the block can show against the current anchor: the largest offset of an
## anchor its tiles were rendered from, over the block's nearest cloud distance.
func _block_parallax_bound_px(block: int) -> float:
	if _max_anchor_dev_m <= 0.0:
		return 0.0
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
	return _max_anchor_dev_m / _block_min_near_m[block] * 0.5 * layout.face_texels


func _block_of(tile: int) -> int:
	var per_side := layout.tiles_per_side
	var face := tile / (per_side * per_side)
	var ty := (tile / per_side) % per_side
	var tx := tile % per_side
	return (face * _blocks_per_side + ty / BLOCK_TILES) * _blocks_per_side + tx / BLOCK_TILES
