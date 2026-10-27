class_name CloudViewCacheLayout
extends RefCounted
## The geometry of the world-fixed cloud view cache (pure logic, no GPU): a cube map around the anchor, direction
## <-> texel, the tiles the cache is filled and refreshed by, and the resolution levels of a tile.
##
## Cube faces 0..5 = +X, -X, +Y, -Y, +Z, -Z of the Earth-local render axes. Tangent mapping: a face coordinate is
## a / |major axis| (-1..1), so a face spans 90 deg and its centre has face_texels / 2 texels per radian (the face edges
## have more: the centre is the coarsest, so the cache is nowhere coarser than that). A tile is a tile_texels square
## block of a face; tile index =
## (face * tiles_per_side + ty) * tiles_per_side + tx. Face coordinates uv are 0..1 across a face, texel coordinates
## uv * face_texels (continuous).

## [E] Resolution levels of a tile: level 0 = 1/4 of the full texel density per axis (about today's 1/3 march divisor at
## the first fill), 1 = 1/2, 2 = full. The coarse level shows something at once, the finer ones follow (spec A).
const LEVELS := 3
const FACES := 6

var face_texels: int
var tile_texels: int
var tiles_per_side: int

var _tile_dirs := PackedVector3Array()


## face_texels must be a multiple of tile_texels, tile_texels a multiple of 2 ^ (LEVELS - 1).
func _init(face_texels_: int, tile_texels_: int) -> void:
	assert(tile_texels_ > 0 and tile_texels_ % (1 << (LEVELS - 1)) == 0, "tile size must divide into the levels")
	assert(face_texels_ > 0 and face_texels_ % tile_texels_ == 0, "face size must be a multiple of the tile size")
	face_texels = face_texels_
	tile_texels = tile_texels_
	tiles_per_side = face_texels_ / tile_texels_
	_tile_dirs.resize(tile_count())
	for tile in range(tile_count()):
		var tx := tile % tiles_per_side
		var ty := (tile / tiles_per_side) % tiles_per_side
		_tile_dirs[tile] = dir_of(tile / (tiles_per_side * tiles_per_side),
				Vector2(tx + 0.5, ty + 0.5) * float(tile_texels) / float(face_texels))


## The face size (texels) whose centre has the eyes' pixel density: the finest density over the view centre (the
## direction of ndc (0, 0)) along both axes, from the projection's own rays, times 2 (tangent mapping: a face centre has
## face_texels / 2 texels per radian), rounded up to a multiple of tile_texels. projection: the eye's clip projection
## (head space -> clip, as the renderer hands it over); view_size: the eye target in pixels.
static func face_texels_for(projection: Projection, view_size: Vector2i, tile_texels_: int) -> int:
	var inverse := projection.inverse()
	var density_per_rad := 0.0
	for axis in [Vector2.RIGHT, Vector2.DOWN]:
		var step: Vector2 = axis / Vector2(view_size)  # +- half a pixel in ndc
		var angle := _ray(inverse, -step).angle_to(_ray(inverse, step))
		density_per_rad = maxf(density_per_rad, 1.0 / angle)
	return ceili(2.0 * density_per_rad / tile_texels_) * tile_texels_


func tile_count() -> int:
	return FACES * tiles_per_side * tiles_per_side


## The texels per side of a tile at a level (level 2 = tile_texels).
func level_texels(level: int) -> int:
	return tile_texels >> (LEVELS - 1 - level)


## Angular radius (rad) every tile fits into around its centre direction (the face centre's, the largest).
func tile_radius_rad() -> float:
	return atan(sqrt(2.0) * float(tile_texels) / float(face_texels))


## The tile's centre direction (unit, Earth-local).
func tile_dir(tile: int) -> Vector3:
	return _tile_dirs[tile]


## All tile centre directions (index = tile): for hot loops that cannot afford a call per tile.
func tile_dirs() -> PackedVector3Array:
	return _tile_dirs


## The tile holding a direction (not necessarily unit).
func tile_of(dir: Vector3) -> int:
	var t := texel_of(dir)
	var tx := clampi(int(t.x) / tile_texels, 0, tiles_per_side - 1)
	var ty := clampi(int(t.y) / tile_texels, 0, tiles_per_side - 1)
	return (int(t.z) * tiles_per_side + ty) * tiles_per_side + tx


## (x, y, face): the continuous texel coordinates of a direction in its face (dir not zero).
func texel_of(dir: Vector3) -> Vector3:
	assert(dir != Vector3.ZERO, "a direction is needed")
	var a := dir.abs()
	var face: int
	var s: float
	var t: float
	var major: float
	if a.x >= a.y and a.x >= a.z:
		major = a.x
		face = 0 if dir.x > 0.0 else 1
		s = -dir.z if dir.x > 0.0 else dir.z
		t = -dir.y
	elif a.y >= a.z:
		major = a.y
		face = 2 if dir.y > 0.0 else 3
		s = dir.x
		t = dir.z if dir.y > 0.0 else -dir.z
	else:
		major = a.z
		face = 4 if dir.z > 0.0 else 5
		s = dir.x if dir.z > 0.0 else -dir.x
		t = -dir.y
	var scale := 0.5 * face_texels / major
	return Vector3(clampf((s * scale + 0.5 * face_texels), 0.0, face_texels),
			clampf((t * scale + 0.5 * face_texels), 0.0, face_texels), face)


## The unit direction of face coordinates uv (0..1 across the face).
func dir_of(face: int, uv: Vector2) -> Vector3:
	var s := 2.0 * uv.x - 1.0
	var t := 2.0 * uv.y - 1.0
	var d: Vector3
	match face:
		0:
			d = Vector3(1.0, -t, -s)
		1:
			d = Vector3(-1.0, -t, s)
		2:
			d = Vector3(s, 1.0, t)
		3:
			d = Vector3(s, -1.0, -t)
		4:
			d = Vector3(s, -t, 1.0)
		_:
			d = Vector3(-s, -t, -1.0)
	return d.normalized()


## Head-space ray direction of an ndc through an inverse projection (the pass's two-depth difference).
static func _ray(inverse: Projection, ndc: Vector2) -> Vector3:
	var a: Vector4 = inverse * Vector4(ndc.x, ndc.y, 0.5, 1.0)
	var b: Vector4 = inverse * Vector4(ndc.x, ndc.y, 1.0, 1.0)
	return (Vector3(a.x, a.y, a.z) / a.w - Vector3(b.x, b.y, b.z) / b.w).normalized()
