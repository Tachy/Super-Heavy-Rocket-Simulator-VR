class_name ImagerySource
extends RefCounted
## Earth imagery tile pyramids of tools/data_pipeline/imagery.py (data/imagery/earth/index.toml): EPSG:4326 TMS,
## 256 px JPEG tiles at <id>/<z>/<x>/<y>.jpg, tile size 180 / 2^z deg, x east from 180 W, y south from 90 N. Colour
## fits are baked into the tiles; tiles are black outside a source's coverage, so blending uses the index coverage
## boxes (never pixel values). Reads only after construction: crop() is safe on worker threads.

const TILE_PX := 256
## Mosaics are at most MAX_TILES x MAX_TILES tiles; a larger request takes a coarser level.
const MAX_TILES := 3

## Sources, finest first: {id, min_level, max_level, coverage: Rect2 (deg), fade_m, dir}.
var _sources: Array[Dictionary] = []


## index_path: the index.toml (default: data/imagery/earth/index.toml); absent: no imagery.
func _init(index_path := "") -> void:
	if index_path.is_empty():
		index_path = Paths.data_dir().path_join("imagery/earth/index.toml")
	if not FileAccess.file_exists(index_path):
		return
	for entry: Dictionary in TomlLite.load_file(index_path).get("source", []):
		var west: float = entry["west_deg"]
		var south: float = entry["south_deg"]
		_sources.append({
			"id": String(entry["id"]),
			"min_level": int(entry["min_level"]),
			"max_level": int(entry["max_level"]),
			"coverage": Rect2(west, south, float(entry["east_deg"]) - west, float(entry["north_deg"]) - south),
			"fade_m": float(entry["fade_m"]),
			"dir": index_path.get_base_dir().path_join(String(entry["id"])),
		})
	_sources.sort_custom(func(a: Dictionary, b: Dictionary) -> bool: return a["max_level"] > b["max_level"])


func is_empty() -> bool:
	return _sources.is_empty()


## Imagery for a lat/lon box (deg, east may exceed 180) at a wanted texel size (deg): the finest source (with
## max_level < coarser_than_level) whose coverage intersects the box; level = the finest <= max_level whose tile size is
## >= texel_deg * 256, coarser while the mosaic over box-and-coverage needs more than 3 x 3 tiles; a missing tile tries
## the next coarser level of the source, then the next source. Returns {image: Image (RGB8, mipmaps), box_deg: Rect2
## (the mosaic extent, in the request's longitude frame), fade_m, coverage_deg: Rect2 (same frame), source_id, level,
## max_level} or {}.
func crop(box_deg: Rect2, texel_deg: float, coarser_than_level := 1 << 30) -> Dictionary:
	var wanted := int(floor(log(180.0 / (texel_deg * TILE_PX)) / log(2.0)))
	for source: Dictionary in _sources:
		if source["max_level"] >= coarser_than_level:
			continue
		var coverage: Rect2 = source["coverage"]
		for shift: float in [0.0, -360.0, 360.0]:  # the box in the coverage's longitude frame
			var shifted := Rect2(box_deg.position + Vector2(shift, 0.0), box_deg.size)
			if not shifted.intersects(coverage):
				continue
			var area := shifted.intersection(coverage)
			var z := clampi(wanted, source["min_level"], source["max_level"])
			while z >= source["min_level"]:
				var tiles := _tile_range(area, z)
				if tiles.size.x <= MAX_TILES and tiles.size.y <= MAX_TILES:
					var image := _mosaic(source, z, tiles)
					if image != null:
						var size := 180.0 / (1 << z)
						var back := Vector2(-shift, 0.0)
						return {
							"image": image,
							"box_deg": Rect2(tiles.position.x * size - 180.0 + back.x,
									90.0 - tiles.end.y * size, tiles.size.x * size, tiles.size.y * size),
							"fade_m": source["fade_m"],
							"coverage_deg": Rect2(coverage.position + back, coverage.size),
							"source_id": source["id"],
							"level": z,
							"max_level": source["max_level"],
						}
				z -= 1
			break
	return {}


## Tiles (x east from 180 W, y south from 90 N; x unwrapped) touching a box at level z.
static func _tile_range(box: Rect2, z: int) -> Rect2i:
	var size := 180.0 / (1 << z)
	var x0 := int(floor((box.position.x + 180.0) / size))
	var x1 := maxi(int(ceil((box.end.x + 180.0) / size)) - 1, x0)
	var y0 := int(floor((90.0 - box.end.y) / size))
	var y1 := maxi(int(ceil((90.0 - box.position.y) / size)) - 1, y0)
	return Rect2i(x0, y0, x1 - x0 + 1, y1 - y0 + 1)


## The tiles blitted into one RGB8 image with mipmaps; null if a tile is missing.
static func _mosaic(source: Dictionary, z: int, tiles: Rect2i) -> Image:
	var columns := 2 << z
	var image := Image.create_empty(tiles.size.x * TILE_PX, tiles.size.y * TILE_PX, false, Image.FORMAT_RGB8)
	for dy in tiles.size.y:
		for dx in tiles.size.x:
			var path: String = source["dir"].path_join("%d/%d/%d.jpg" % [z, posmod(tiles.position.x + dx, columns),
					tiles.position.y + dy])
			if not FileAccess.file_exists(path):
				return null
			var tile := Image.load_from_file(path)
			if tile == null or tile.get_width() != TILE_PX or tile.get_height() != TILE_PX:
				return null
			if tile.get_format() != Image.FORMAT_RGB8:
				tile.convert(Image.FORMAT_RGB8)
			image.blit_rect(tile, Rect2i(0, 0, TILE_PX, TILE_PX), Vector2i(dx * TILE_PX, dy * TILE_PX))
	image.generate_mipmaps()
	return image
