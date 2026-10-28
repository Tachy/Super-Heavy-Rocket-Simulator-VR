class_name CloudWorldVolumeLayout
extends RefCounted
## The cloud world volume's clipmap layout (world-volume plan Task 3; pure logic, no GPU; the GLSL mirror
## `cloud_volume_grid` of Task 5 evaluates the same projection).
##
## LEVELS toroidal levels of CELLS_H x CELLS_H x CELLS_V cells over the cloud shell (SHELL_BOTTOM_M .. SHELL_TOP_M).
## Horizontal axes: an Earth-fixed azimuthal-equidistant projection about a frame origin (east / north of the geodetic
## normal there), on the sphere of the WGS84 Gaussian radius at the origin's latitude; vertical axis: height above the
## WGS84 ellipsoid (a flat tangent grid would miss the shell by 13 km at 410 km). Earth-local points are in render axes
## (polar axis +Y). A cell's global index is floor(coordinate / cell size) (vertical: from SHELL_BOTTOM_M); a level's
## window is the CELLS_H x CELLS_H x CELLS_V block of them centred on the camera's ground point, snapped to its cell
## size and stored toroidally (texel = index mod size), so a move exposes only a slab (exposed_slabs).
## Storage and window (world-volume Task 5 review I2): the texture holds the STORAGE box of CELLS_H x CELLS_H x CELLS_V
## cells; what the lookup reads (the WINDOW, window_size) is GUARD_CELLS smaller on each side of every scrolling axis,
## so the slabs ahead of the camera are baked before the window reaches them (CloudWorldVolumeSchedule bakes them
## amortised; the window advances only onto baked cells). The storage's target is the window grown by the guard
## (storage_target); the fade band lies inside the window.
## Vertical: levels 2-7 hold the whole shell (164.0625 m x 128 = 21 km; no vertical scroll, no guard); levels 0-1 a
## 3.1 / 6.2 km window centred on the camera height, clamped at the shell bottom only (no top clamp: level 0 must not
## pop when it switches off, see level_active); both fade at their vertical window edges (fade_weight) except at the
## clamped shell bottom.

const LEVELS := 8
const CELLS_H := 256
const CELLS_V := 128
## Horizontal cell size per level (m): doubling.
const CELL_H_M: Array[float] = [50.0, 100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0]
## Vertical cell size per level (m): levels 0-1 fine windows, levels 2-7 the shell in 128 cells.
const CELL_V_M: Array[float] = [25.0, 50.0, 164.0625, 164.0625, 164.0625, 164.0625, 164.0625, 164.0625]
## The cloud shell (cloud_pass.gdshaderinc CLOUD_BOTTOM_M .. CLOUD_TOP_M plus the 500 m margin the light cache keeps).
const SHELL_BOTTOM_M := -500.0
const SHELL_TOP_M := 20500.0
## Fade band at a window's edge (cells of the level): weight 0 at the outermost cell, 1 FADE_CELLS cells in.
const FADE_CELLS := 8
## Re-frame when the ground point is further than this from the frame origin (as the light cache's REFRAME_M).
const REFRAME_M := 1.0e6
## Density RGBA16 (both snapshots) + light RGBA16F.
const BYTES_PER_CELL := 16
## The max-extinction mip and the distance field are stored at 1/MIP_DIV resolution per axis (R16F / R8).
const MIP_DIV := 4
## Levels 0-1 have a vertical window; their vertical fade runs on the window edges.
const WINDOWED_LEVELS := 2
## [E] Cells of storage beyond the window on each side of a scrolling axis (Task 5 review I2): the slab the window
## enters next is baked while the camera crosses the guard. 2 cells: the window may move up to 2 x GUARD_CELLS cells
## a frame (level 2 at 7.8 km/s: 0.87, level 0 at 1 km/s: 0.44) and the slab's bake is spread over GUARD_CELLS cells
## of motion (an L7 slab over 74 frames at 7.8 km/s); costs 1.6 % of each level's reach, no memory.
const GUARD_CELLS := 2
## Iterations of the geodetic latitude fixed point (contraction e^2 = 0.0067 per step: exact to double precision).
const GEODETIC_ITERATIONS := 12

const E2 := 1.0 - 1.0 / (CloudLayer.POLAR_STRETCH * CloudLayer.POLAR_STRETCH)

## The cell bake's tap rule (core cloud_volume_bake.hpp cloud_cell_bake_taps and its constants, mirrored by
## cloud_volume_bake.gdshaderinc cloud_volume_taps; change together): the upkeep's cost unit (CloudWorldVolumeSchedule).
const BAKE_HORIZONTAL_SPACING_M := 200.0
const BAKE_MIN_HORIZONTAL_TAPS := 4
const BAKE_MAX_HORIZONTAL_TAPS := 8
const BAKE_WIDEST_CELL_M := 4800.0
const BAKE_WIDEST_CELL_HORIZONTAL_TAPS := 16
const BAKE_VERTICAL_SPACING_M := 5.0
const BAKE_MIN_VERTICAL_TAPS := 16
const BAKE_TAP_COUNT_SLACK := 0.01

var origin_earth_m := Vector3.ZERO
## The current window origin (cells) of every level, as adopted by the render thread (set_origin).
var origins: Array[Vector3i] = []
## The projection sphere's radius (m): the WGS84 Gaussian radius at the frame origin.
var sphere_radius_m := CloudLayer.WGS84_A_M

var _n0 := Vector3.UP
var _east := Vector3.RIGHT
var _north := Vector3.BACK


func _init() -> void:
	origins.resize(LEVELS)
	origins.fill(Vector3i.ZERO)
	set_frame(Vector3(CloudLayer.WGS84_A_M, 0.0, 0.0))


static func memory_bytes() -> int:
	return LEVELS * CELLS_H * CELLS_H * CELLS_V * BYTES_PER_CELL


## Texels of a level's max-extinction mip / distance field (per axis /MIP_DIV).
static func mip_size() -> Vector3i:
	return Vector3i(CELLS_H, CELLS_H, CELLS_V) / MIP_DIV


## Taps per horizontal axis and vertically of a cell horizontal_m x vertical_m (the bake's tap rule).
static func bake_taps(horizontal_m: float, vertical_m: float) -> Vector2i:
	var slack := 1.0 - BAKE_TAP_COUNT_SLACK
	var cap := BAKE_WIDEST_CELL_HORIZONTAL_TAPS if horizontal_m > BAKE_WIDEST_CELL_M else BAKE_MAX_HORIZONTAL_TAPS
	return Vector2i(clampi(ceili(horizontal_m / BAKE_HORIZONTAL_SPACING_M * slack), BAKE_MIN_HORIZONTAL_TAPS, cap),
			maxi(BAKE_MIN_VERTICAL_TAPS, ceili(vertical_m / BAKE_VERTICAL_SPACING_M * slack)))


## Taps of one cell of level k (256, 256, 528, 528, 528, 2112, 2112, 8448).
static func cell_taps(k: int) -> int:
	var t := bake_taps(CELL_H_M[k], CELL_V_M[k])
	return t.x * t.x * t.y


func set_frame(frame_origin_earth_m: Vector3) -> void:
	origin_earth_m = frame_origin_earth_m
	var g := _geodetic(frame_origin_earth_m)
	_n0 = Vector3(g.x, g.y, g.z)
	_east = Vector3.UP.cross(_n0)
	_east = _east.normalized() if _east.length_squared() > 1e-12 else Vector3.RIGHT
	_north = _n0.cross(_east).normalized()
	sphere_radius_m = CloudLayer.WGS84_A_M * sqrt(1.0 - E2) / (1.0 - E2 * _n0.y * _n0.y)


## (x, y) in the projection (m east / north of the frame origin along the sphere), z = height above the ellipsoid.
func grid_of(p_earth_m: Vector3) -> Vector3:
	var g := _geodetic(p_earth_m)
	var n := Vector3(g.x, g.y, g.z)
	var a := n.dot(_east)
	var b := n.dot(_north)
	var r := sqrt(a * a + b * b)
	if r < 1e-300:
		return Vector3(0.0, 0.0, g.w)
	var s := sphere_radius_m * atan2(r, n.dot(_n0)) / r
	return Vector3(a * s, b * s, g.w)


func earth_of(grid_m: Vector3) -> Vector3:
	var r := sqrt(grid_m.x * grid_m.x + grid_m.y * grid_m.y)
	var n := _n0
	if r > 0.0:
		var c := r / sphere_radius_m
		n = (_n0 * cos(c) + (_east * grid_m.x + _north * grid_m.y) * (sin(c) / r)).normalized()
	var ns := CloudLayer.WGS84_A_M / sqrt(1.0 - E2 * n.y * n.y)
	return Vector3(n.x * (ns + grid_m.z), n.y * (ns * (1.0 - E2) + grid_m.z), n.z * (ns + grid_m.z))


## The geodetic normal over the grid point (x, y).
func up_of_grid(x_m: float, y_m: float) -> Vector3:
	var r := sqrt(x_m * x_m + y_m * y_m)
	if r <= 0.0:
		return _n0
	var c := r / sphere_radius_m
	return (_n0 * cos(c) + (_east * x_m + _north * y_m) * (sin(c) / r)).normalized()


## An Earth-local direction in the frame's tangent plane (east, north components), not normalised.
func tangent_of(dir_earth: Vector3) -> Vector2:
	return Vector2(dir_earth.dot(_east), dir_earth.dot(_north))


## The frame's axes: the origin's normal, east, north.
func axes() -> PackedVector3Array:
	return PackedVector3Array([_n0, _east, _north])


## (n.x, n.y, n.z, h): the geodetic normal and height of an Earth-local point.
func geodetic_of(p: Vector3) -> Vector4:
	return _geodetic(p)


func _geodetic(p: Vector3) -> Vector4:
	var q := sqrt(p.x * p.x + p.z * p.z)
	var phi := atan2(p.y, q * (1.0 - E2))
	for i in GEODETIC_ITERATIONS:
		var s := sin(phi)
		var nu := CloudLayer.WGS84_A_M / sqrt(1.0 - E2 * s * s)
		var next := atan2(p.y + E2 * nu * s, q)
		var done := absf(next - phi) < 1e-15
		phi = next
		if done:
			break
	var sp := sin(phi)
	var cp := cos(phi)
	var h := q * cp + p.y * sp - CloudLayer.WGS84_A_M * sqrt(1.0 - E2 * sp * sp)
	var cx := p.x / q if q > 1e-9 else 1.0
	var cz := p.z / q if q > 1e-9 else 0.0
	return Vector4(cp * cx, sp, cp * cz, h)


## True while the level's window can meet the shell (levels 0-1 are off above ~22 / ~24 km; 2-7 always on).
func level_active(k: int, camera_height_m: float) -> bool:
	if k >= WINDOWED_LEVELS:
		return true
	return camera_height_m - 0.5 * CELL_V_M[k] * CELLS_V <= SHELL_TOP_M


## The window's size (cells): the storage less GUARD_CELLS on each side of the scrolling axes.
static func window_size(k: int) -> Vector3i:
	return Vector3i(CELLS_H - 2 * GUARD_CELLS, CELLS_H - 2 * GUARD_CELLS,
			CELLS_V - 2 * GUARD_CELLS if k < WINDOWED_LEVELS else CELLS_V)


## The window's lower corner (cells): centred on the camera's ground point; levels 0-1 vertically centred on the camera
## height and clamped at the shell bottom (not at the top), levels 2-7 the whole shell.
func window_origin(k: int, camera_grid_m: Vector3) -> Vector3i:
	var size := window_size(k)
	var ox := floori(camera_grid_m.x / CELL_H_M[k]) - size.x / 2
	var oy := floori(camera_grid_m.y / CELL_H_M[k]) - size.y / 2
	var oz := 0
	if k < WINDOWED_LEVELS:
		oz = maxi(floori((camera_grid_m.z - SHELL_BOTTOM_M) / CELL_V_M[k]) - size.z / 2, 0)
	return Vector3i(ox, oy, oz)


## The storage origin a window at `origin` wants: the window grown by the guard (vertically not below the shell).
static func storage_target(k: int, origin: Vector3i) -> Vector3i:
	if k < WINDOWED_LEVELS:
		return Vector3i(origin.x - GUARD_CELLS, origin.y - GUARD_CELLS, maxi(origin.z - GUARD_CELLS, 0))
	return Vector3i(origin.x - GUARD_CELLS, origin.y - GUARD_CELLS, 0)


## The storage origins that still hold the whole window at `origin` (per axis [lo, hi]): the window's cells must not
## leave the texture while the window waits.
static func storage_range(k: int, origin: Vector3i) -> Array[Vector3i]:
	var spare := Vector3i(CELLS_H, CELLS_H, CELLS_V) - window_size(k)
	var lo := origin - spare
	lo.z = maxi(lo.z, 0)
	return [lo, origin]


func set_origin(k: int, origin: Vector3i) -> void:
	origins[k] = origin


## The texture's cells for a storage origin.
static func storage_box(origin: Vector3i) -> Dictionary:
	return {"from": origin, "to": origin + Vector3i(CELLS_H, CELLS_H, CELLS_V)}


## The cells the lookup reads for a window origin.
static func window_box(k: int, origin: Vector3i) -> Dictionary:
	return {"from": origin, "to": origin + window_size(k)}


static func texel_of(_k: int, cell: Vector3i) -> Vector3i:
	return Vector3i(posmod(cell.x, CELLS_H), posmod(cell.y, CELLS_H), posmod(cell.z, CELLS_V))


## The cells of the new storage box that were not in the old one, as disjoint half-open boxes {from, to}; the whole
## new storage box when the move is a full storage extent or more along an axis.
static func exposed_slabs(_k: int, old_origin: Vector3i, new_origin: Vector3i) -> Array[Dictionary]:
	var slabs: Array[Dictionary] = []
	var d := new_origin - old_origin
	if d == Vector3i.ZERO:
		return slabs
	if absi(d.x) >= CELLS_H or absi(d.y) >= CELLS_H or absi(d.z) >= CELLS_V:
		slabs.append(storage_box(new_origin))
		return slabs
	var size := Vector3i(CELLS_H, CELLS_H, CELLS_V)
	var lo := Vector3i(maxi(old_origin.x, new_origin.x), maxi(old_origin.y, new_origin.y), maxi(old_origin.z, new_origin.z))
	var hi := Vector3i(mini(old_origin.x, new_origin.x), mini(old_origin.y, new_origin.y), mini(old_origin.z, new_origin.z)) + size
	var from := new_origin
	var to := new_origin + size
	# One axis at a time: the slab spans the not yet reduced extent of the other axes (disjoint by construction).
	for axis in 3:
		if d[axis] == 0:
			continue
		var s_from := from
		var s_to := to
		if d[axis] > 0:
			s_from[axis] = old_origin[axis] + size[axis]
		else:
			s_to[axis] = old_origin[axis]
		slabs.append({"from": s_from, "to": s_to})
		from[axis] = lo[axis]
		to[axis] = hi[axis]
	return slabs


## Weight of the level at a grid point for the window at `origin`: 1 inside, 0 at the outermost cell, linear over
## FADE_CELLS cells; levels 0-1 also fade at their vertical window edges, except where the window is clamped at the
## shell bottom.
static func fade_weight_at(k: int, grid_m: Vector3, origin: Vector3i) -> float:
	var size := window_size(k)
	var ux := grid_m.x / CELL_H_M[k]
	var uy := grid_m.y / CELL_H_M[k]
	var d := minf(minf(ux - origin.x, origin.x + size.x - ux), minf(uy - origin.y, origin.y + size.y - uy))
	if k < WINDOWED_LEVELS:
		var uz := (grid_m.z - SHELL_BOTTOM_M) / CELL_V_M[k]
		d = minf(d, origin.z + size.z - uz)
		if origin.z > 0:
			d = minf(d, uz - origin.z)
	return clampf((d - 0.5) / float(FADE_CELLS), 0.0, 1.0)


## True when level k's window at `origin` lies where level k + 1's weight (window at `origin_up`) is 1 (Task 5 review
## round 2, N1): level k's fade band blends level k + 1, so its trilinear reads, block bounds (<= 1.5 cells of k + 1
## past the window) and free balls stay inside k + 1's window, and the blend meets k + 1 at full weight (no seam).
## Levels 2-7 hold the whole shell (no vertical fade, reads clamped); level 1's clamped bottom does not fade.
static func nested(k: int, origin: Vector3i, origin_up: Vector3i) -> bool:
	var size := window_size(k)
	var size_up := window_size(k + 1)
	var band := FADE_CELLS + 0.5  # fade_weight_at reaches 1 this many cells inside the window edge
	var r := CELL_H_M[k] / CELL_H_M[k + 1]
	for axis in 2:
		if origin[axis] * r < origin_up[axis] + band or (origin[axis] + size[axis]) * r > origin_up[axis] \
				+ size_up[axis] - band:
			return false
	if k + 1 < WINDOWED_LEVELS:
		var rv := CELL_V_M[k] / CELL_V_M[k + 1]
		if (origin.z + size.z) * rv > origin_up.z + size_up.z - band:
			return false
		if origin_up.z > 0 and origin.z * rv < origin_up.z + band:
			return false
	return true


func fade_weight(k: int, grid_m: Vector3) -> float:
	return fade_weight_at(k, grid_m, origins[k])


func reframe_needed(ground_earth_m: Vector3) -> bool:
	var g := grid_of(ground_earth_m)
	return Vector2(g.x, g.y).length() > REFRAME_M


## Cell count of a half-open box {from, to}.
static func box_cells(box: Dictionary) -> int:
	var d: Vector3i = box["to"] - box["from"]
	return maxi(d.x, 0) * maxi(d.y, 0) * maxi(d.z, 0)


## True when two half-open boxes share a cell.
static func box_overlaps(a: Dictionary, b: Dictionary) -> bool:
	return box_cells(box_clip(a, b)) > 0


## The intersection of two boxes (empty box when disjoint).
static func box_clip(a: Dictionary, b: Dictionary) -> Dictionary:
	var f := Vector3i(maxi(a["from"].x, b["from"].x), maxi(a["from"].y, b["from"].y), maxi(a["from"].z, b["from"].z))
	var t := Vector3i(mini(a["to"].x, b["to"].x), mini(a["to"].y, b["to"].y), mini(a["to"].z, b["to"].z))
	return {"from": f, "to": t}


## Splits `box` into the largest prefix of at most n cells that is itself a box (whole z planes, else whole rows of one
## plane, else a partial row; n >= cells takes everything) and the remaining boxes: [taken, rest]. Callers loop while
## budget is left.
static func box_split(box: Dictionary, n: int) -> Array:
	var f: Vector3i = box["from"]
	var t: Vector3i = box["to"]
	var d := t - f
	if n >= box_cells(box):
		return [box, []]
	var rest: Array[Dictionary] = []
	var plane := d.x * d.y
	if n >= plane:
		var zs := n / plane
		rest.append({"from": Vector3i(f.x, f.y, f.z + zs), "to": t})
		return [{"from": f, "to": Vector3i(t.x, t.y, f.z + zs)}, rest]
	var rows := n / d.x
	if rows >= 1:
		rest.append({"from": Vector3i(f.x, f.y + rows, f.z), "to": Vector3i(t.x, t.y, f.z + 1)})
		if d.z > 1:
			rest.append({"from": Vector3i(f.x, f.y, f.z + 1), "to": t})
		return [{"from": f, "to": Vector3i(t.x, f.y + rows, f.z + 1)}, rest]
	rest.append({"from": Vector3i(f.x + n, f.y, f.z), "to": Vector3i(t.x, f.y + 1, f.z + 1)})
	if d.y > 1:
		rest.append({"from": Vector3i(f.x, f.y + 1, f.z), "to": Vector3i(t.x, t.y, f.z + 1)})
	if d.z > 1:
		rest.append({"from": Vector3i(f.x, f.y, f.z + 1), "to": t})
	return [{"from": f, "to": Vector3i(f.x + n, f.y + 1, f.z + 1)}, rest]
