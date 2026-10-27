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

## [E] A tile is stale when the Sun turned more than this from the direction it was rendered with: ~0.1 deg, one pixel
## of the evening light's gradient (a cloud edge's colour moves < 1 pixel).
const SUN_REFRESH_RAD := 0.002
## [E] A tile is stale when the weather weight (the field's blend) moved more than this from the one it was rendered
## with: a 2 % step of the snapshot mix (the light cache refreshes at 5 %; the view cache sits well below it).
const WEATHER_REFRESH := 0.02
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
## Tiles per side of a block: the coarse grid pick() culls with (a block outside the margin and with nothing open is
## not visited).
const BLOCK_TILES := 8

## Keys of the sort: a 30-bit quantised priority above the 21-bit tile index (exact in a double).
const _KEY_MAX := 1073741823
const _TILE_RANGE := 2097152.0
## Quantisation of 1 - cos(angle) and of error x nearness (m^-1).
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
var _have_light := false
var _current_anchor := Vector3(NAN, NAN, NAN)

var _blocks_per_side := 0
var _block_dirs := PackedVector3Array()
var _block_radius := PackedFloat64Array()
var _block_open := PackedInt32Array()
var _block_min_near_m := PackedFloat64Array()
var _block_near_dirty := PackedByteArray()
var _max_anchor_dev_m := 0.0


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
	_block_open.fill(0)
	for block in range(_block_open.size()):
		_block_open[block] = _block_tile_count(block)


## Sets the cloud distance (m) the tile's parallax error uses: the nearest cached cloud distance of the tile
## (INF: no cloud, no parallax error).
func set_tile_distance(tile: int, near_m: float) -> void:
	_near_m[tile] = near_m
	_block_near_dirty[_block_of(tile)] = 1


## The current light: tiles whose rendered state differs by more than SUN_REFRESH_RAD or WEATHER_REFRESH, or by the
## light cache generation, become stale (compared with the state they were rendered with: slow drift adds up).
func mark_light(sun_dir: Vector3, weather_weight: float, light_generation: int) -> void:
	var sun := sun_dir.normalized()
	if _have_light and sun == _current_sun and weather_weight == _current_weight \
			and light_generation == _current_generation:
		return
	_current_sun = sun
	_current_weight = weather_weight
	_current_generation = light_generation
	_have_light = true
	_restale()


## Reports a finished march: the tile now holds level, rendered with state {anchor_earth_m: Vector3, sun_dir: Vector3,
## weather_weight: float, light_generation: int}.
func completed(tile: int, level: int, state: Dictionary) -> void:
	_level[tile] = level
	_anchor_earth_m[tile] = state["anchor_earth_m"]
	_sun_dir[tile] = (state["sun_dir"] as Vector3).normalized()
	_weather_weight[tile] = state["weather_weight"]
	_light_generation[tile] = state["light_generation"]
	_stale[tile] = 1 if _differs(tile) else 0
	if not is_nan(_current_anchor.x):
		_max_anchor_dev_m = maxf(_max_anchor_dev_m, _anchor_earth_m[tile].distance_to(_current_anchor))
	_update_open(tile)


## The level a tile holds (-1: none).
func level_of(tile: int) -> int:
	return _level[tile]


func is_stale(tile: int) -> bool:
	return _stale[tile] != 0


## The parallax error (cache texels ~ eye pixels) of a tile rendered from anchor_earth_m and seen from camera_earth_m:
## the camera's offset across the tile's direction over the cloud distance, at the cache's centre density. 0 without a
## known cloud distance.
func parallax_error_px(tile: int, camera_earth_m: Vector3, anchor_earth_m: Vector3) -> float:
	var near := _near_m[tile]
	if is_inf(near) or near <= 0.0:
		return 0.0
	var offset := camera_earth_m - anchor_earth_m
	var dir := layout.tile_dir(tile)
	var across := (offset - dir * offset.dot(dir)).length()
	return across / near * 0.5 * layout.face_texels


## The tiles to march this frame, at most budget_tiles, in priority order: [{tile, level}]. camera_earth_m: the eyes'
## head; anchor_earth_m: the anchor a march starts from now (a tile rendered from another anchor is stale);
## view_dirs: unit sample rays of the view (Earth-local; tens of them: the frustum's corners, edges and centre per
## eye) - a tile is visible when its centre is within view_sample_rad (+ the tile's own radius) of one (the samples'
## spacing at most twice view_sample_rad), and in the margin when within VIEW_MARGIN_RAD more.
## Cost: blocks (BLOCK_TILES squared tiles) wholly inside or outside a zone are classified at once and skipped when
## nothing is open in them and no parallax error is possible, so a converged cache costs the block pass only.
func pick(camera_earth_m: Vector3, anchor_earth_m: Vector3, view_dirs: Array[Vector3], budget_tiles: int,
		view_sample_rad := 0.0) -> Array[Dictionary]:
	var picked: Array[Dictionary] = []
	if budget_tiles <= 0:
		return picked
	_sync_anchor(anchor_earth_m)
	var tile_radius := layout.tile_radius_rad()
	var visible_angle := minf(view_sample_rad + tile_radius, PI)
	var margin_angle := minf(view_sample_rad + tile_radius + VIEW_MARGIN_RAD, PI)
	var cos_visible := cos(visible_angle)
	var cos_margin := cos(margin_angle)
	var classes: Array[PackedFloat64Array] = [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array(),
			PackedFloat64Array(), PackedFloat64Array()]
	var anchor_offset_m := camera_earth_m.distance_to(_current_anchor) + _max_anchor_dev_m
	var has_view := not view_dirs.is_empty()
	for block in range(_block_dirs.size()):
		var block_dot := _max_dot(_block_dirs[block], view_dirs) if has_view else -1.0
		var block_angle := acos(clampf(block_dot, -1.0, 1.0))
		var radius := _block_radius[block]
		var touches_margin := has_view and block_angle - radius <= margin_angle
		var touches_visible := has_view and block_angle - radius <= visible_angle
		var inside_visible := has_view and block_angle + radius <= visible_angle
		var inside_margin := has_view and block_angle + radius <= margin_angle
		var parallax_possible := touches_visible \
				and _block_parallax_bound_px(block, anchor_offset_m) > PARALLAX_REFRESH_PX
		if _block_open[block] == 0 and not parallax_possible:
			continue
		var block_key := _key(1.0 - block_dot, _DOT_SCALE)
		for tile in _block_tiles(block):
			var level := _level[tile]
			var visible := inside_visible
			var in_margin := inside_margin
			var key := block_key
			if touches_margin and not (inside_visible or inside_margin and not touches_visible):
				var dot := _max_dot(layout.tile_dir(tile), view_dirs)  # on a zone's boundary: exact
				key = _key(1.0 - dot, _DOT_SCALE)
				visible = dot >= cos_visible
				in_margin = dot >= cos_margin
			if level < 0 and in_margin:
				classes[0].append(_encode(key, tile))
			elif visible and level >= 0 and _near_m[tile] > 0.0:
				var error_px := parallax_error_px(tile, camera_earth_m, _anchor_earth_m[tile])
				if error_px > PARALLAX_REFRESH_PX:
					var score := error_px / _near_m[tile]  # error times nearness
					classes[1].append(_encode(_KEY_MAX - mini(int(score * _NEARNESS_SCALE), _KEY_MAX), tile))
				elif level < MAX_LEVEL:
					classes[2].append(_encode(key, tile))
				elif _stale[tile] != 0:
					classes[3].append(_encode(key, tile))
			elif visible and level < MAX_LEVEL:
				classes[2].append(_encode(key, tile))
			elif level >= 0 and _stale[tile] != 0:
				classes[3].append(_encode(key, tile))
			elif level < MAX_LEVEL:
				classes[4].append(_encode(key, tile))
	for class_index in range(classes.size()):
		var keys := classes[class_index]
		keys.sort()
		for i in range(mini(keys.size(), budget_tiles - picked.size())):
			var tile := int(fmod(keys[i], _TILE_RANGE))
			var level := _level[tile]
			# Missing and the finer-level classes go one level up; refreshes (parallax, stale) keep the level.
			var next := 0 if level < 0 else (level if class_index in [1, 3] else level + 1)
			picked.append({"tile": tile, "level": next})
		if picked.size() >= budget_tiles:
			break
	return picked


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
	return _anchor_moved(tile) or _sun_dir[tile].dot(_current_sun) < cos(SUN_REFRESH_RAD) \
			or absf(_weather_weight[tile] - _current_weight) > WEATHER_REFRESH \
			or _light_generation[tile] != _current_generation


## Compares every rendered tile with the current light and anchor, then recounts the open tiles of every block.
func _restale() -> void:
	_max_anchor_dev_m = 0.0
	for tile in range(_level.size()):
		if _level[tile] >= 0:
			_stale[tile] = 1 if _differs(tile) else 0
			if not is_nan(_current_anchor.x):
				_max_anchor_dev_m = maxf(_max_anchor_dev_m, _anchor_earth_m[tile].distance_to(_current_anchor))
	for block in range(_block_open.size()):
		_recount_block(block)


## Open: something left for the background classes (below the full level, or stale).
func _is_open(tile: int) -> bool:
	return _level[tile] < MAX_LEVEL or _stale[tile] != 0


func _update_open(tile: int) -> void:
	_recount_block(_block_of(tile))


func _recount_block(block: int) -> void:
	var open := 0
	for t in _block_tiles(block):
		open += 1 if _is_open(t) else 0
	_block_open[block] = open


static func _key(one_minus_dot: float, scale: float) -> int:
	return clampi(int(maxf(one_minus_dot, 0.0) * scale), 0, _KEY_MAX)


static func _encode(key: int, tile: int) -> float:
	return float(key) * _TILE_RANGE + float(tile)


static func _max_dot(dir: Vector3, view_dirs: Array[Vector3]) -> float:
	var best := -1.0
	for d in view_dirs:
		best = maxf(best, dir.dot(d))
	return best


func _build_blocks() -> void:
	var per_side := layout.tiles_per_side
	_blocks_per_side = ceili(float(per_side) / BLOCK_TILES)
	var count := CloudViewCacheLayout.FACES * _blocks_per_side * _blocks_per_side
	_block_dirs.resize(count)
	_block_radius.resize(count)
	_block_open.resize(count)
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


## The most parallax (px) any tile of the block can show for a camera anchor_offset_m from the anchors its tiles were
## rendered from: the offset over the block's nearest cloud distance.
func _block_parallax_bound_px(block: int, anchor_offset_m: float) -> float:
	if _block_near_dirty[block] != 0:
		_block_near_dirty[block] = 0
		var nearest := INF
		for tile in _block_tiles(block):
			if _near_m[tile] > 0.0:
				nearest = minf(nearest, _near_m[tile])
		_block_min_near_m[block] = nearest
	return anchor_offset_m / _block_min_near_m[block] * 0.5 * layout.face_texels


func _block_of(tile: int) -> int:
	var per_side := layout.tiles_per_side
	var face := tile / (per_side * per_side)
	var ty := (tile / per_side) % per_side
	var tx := tile % per_side
	return (face * _blocks_per_side + ty / BLOCK_TILES) * _blocks_per_side + tx / BLOCK_TILES


func _block_tiles(block: int) -> PackedInt32Array:
	var per_side := layout.tiles_per_side
	var face := block / (_blocks_per_side * _blocks_per_side)
	var bx := block % _blocks_per_side
	var by := (block / _blocks_per_side) % _blocks_per_side
	var tiles := PackedInt32Array()
	for ty in range(by * BLOCK_TILES, mini((by + 1) * BLOCK_TILES, per_side)):
		for tx in range(bx * BLOCK_TILES, mini((bx + 1) * BLOCK_TILES, per_side)):
			tiles.append((face * per_side + ty) * per_side + tx)
	return tiles


func _block_tile_count(block: int) -> int:
	return _block_tiles(block).size()
