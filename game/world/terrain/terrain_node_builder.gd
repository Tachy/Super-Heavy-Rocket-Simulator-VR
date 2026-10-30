class_name TerrainNodeBuilder
## Builds the mesh data of one cube-sphere terrain node (pure data: Packed arrays and Images, no GPU resources), safe on
## worker threads. The main thread turns the result into an ArrayMesh / textures (TerrainQuadtree, Task 6).
##
## Vertex layout: 65 x 65 grid vertices (index b * ROW + a, a along the face's s axis, b along t), then the skirt ring
## (SKIRT_EDGE[k] is the grid vertex above skirt vertex k). Positions are relative to `center_fixed` in render axes.
## Per-vertex channels: CUSTOM0 = `morph` (xyz: offset to the parent-level surface, w: 1 on skirt vertices),
## CUSTOM1 = `custom1` (2/vertex: global equirect u, v), CUSTOM2 = `custom2` (water mask uv; on a body with horizon
## maps the vertex's grid position (a, b) instead: the texel of the horizon map, see `_horizon_image`),
## CUSTOM3 = `custom3` (3/vertex: the parent-level normal the shader blends towards with the morph factor).

const CELLS := 64
const ROW := CELLS + 1
## Lattice with one ring outside the node (seamless normals by central differences).
const LATTICE := CELLS + 3
const GRID_VERTICES := ROW * ROW
## [E] Skirt depth in grid cells of the node edge: hides cracks between neighbours one level apart.
const SKIRT_DEPTH_CELLS := 2.0
const WATER_MASK_PX := 128
## Horizon azimuths per vertex (k * 2 pi / N_AZ clockwise from north); N_AZ / 4 = HORIZON_BLOCKS RGBA texels per vertex.
const N_AZ := 16
const HORIZON_BLOCKS := N_AZ / 4
## [E] Nodes with a vertex spacing above this get no horizon map: the bake's cost grows steeply with the spacing (IM-2,
## one node: 0.06 s at 666 m, 0.2 s at 1.3 km, 0.8-1.4 s from 2.7 km up to the 40 km of level 1; the first load of a
## site went 5 s -> 40 s with all levels), and a grid coarser than ~1 km (the LDEM_16 data: 1.9 km) holds no shadow
## detail a viewer of such a node (an orbit view of a whole region) could see. Such nodes light without terrain shadows.
const HORIZON_MAX_SPACING_M := 1000.0
## [E] Imagery texels wanted across a node (4 per grid cell); ImagerySource picks the level from it.
const IMAGERY_TEXELS := 256.0
const INV_360 := 1.0 / 360.0
const INV_180 := 1.0 / 180.0

static var SKIRT_EDGE: PackedInt32Array = _make_skirt_edge()
static var INDICES: PackedInt32Array = _make_indices()


## Mesh data of node `key` (CubeSphere key) on a body of mean radius radius_m. imagery may be null (no imagery).
static func build(key: Vector4i, heights: HeightSource, imagery: ImagerySource, radius_m: float) -> Dictionary:
	var face := key.x
	var level := key.y
	var box := CubeSphere.lat_lon_box(key)
	var box_center_lon := box.get_center().x
	var center_lon := CubeSphere.lat_lon_deg(CubeSphere.node_center(key)).y

	# Lattice directions (geodetic normals), geodetic coordinates and heights (one sample call).
	var warp_s := PackedFloat64Array()
	var warp_t := PackedFloat64Array()
	for n in LATTICE:
		warp_s.append(CubeSphere.warp(CubeSphere.lattice_coordinate(key.z, level, n - 1, CELLS)))
		warp_t.append(CubeSphere.warp(CubeSphere.lattice_coordinate(key.w, level, n - 1, CELLS)))
	var count := LATTICE * LATTICE
	var dirs := PackedVector3Array()
	var lats := PackedFloat64Array()
	var lons := PackedFloat64Array()  # raw atan2 longitudes (sampling); a pole takes the node centre's
	dirs.resize(count)
	lats.resize(count)
	lons.resize(count)
	var k := 0
	for lb in LATTICE:
		var b_warp := warp_t[lb]
		for la in LATTICE:
			var d := CubeSphere.direction_warped(face, warp_s[la], b_warp)
			var horizontal := sqrt(d.x * d.x + d.y * d.y)
			dirs[k] = d
			lats[k] = rad_to_deg(atan2(d.z, horizontal))
			lons[k] = rad_to_deg(atan2(d.y, d.x)) if horizontal > CubeSphere.POLE_EPSILON else center_lon
			k += 1
	var height_m: PackedFloat64Array = heights.sample(lats, lons)["height_m"]
	var fixed := heights.positions_fixed(dirs, height_m)
	var center_fixed := fixed[(CELLS / 2 + 1) * LATTICE + CELLS / 2 + 1]
	# Lattice relative to the centre in render axes (Frames.to_render inlined: (x, y, z) -> (y, z, x)); a rotation, so
	# differences, cross products and means below can work in render axes directly.
	var rel := PackedVector3Array()
	rel.resize(count)
	for i in count:
		var q := fixed[i] - center_fixed
		rel[i] = Vector3(q.y, q.z, q.x)

	# Imagery crops: the fine one and the base (next coarser source) under it.
	var texel_deg := box.size.y / IMAGERY_TEXELS
	var fine := {}
	var base := {}
	if imagery != null:
		fine = imagery.crop(box, texel_deg)
		if not fine.is_empty():
			base = imagery.crop(box, texel_deg, fine["max_level"])
	var fine_map := _crop_map(fine)
	var base_map := _crop_map(base)
	var mask_u_scale := 1.0 / box.size.x
	var mask_v_scale := 1.0 / box.size.y

	var horizon_start_us := Time.get_ticks_usec()
	var horizon := _horizon_image(key, heights, radius_m, lats, lons, height_m)
	var horizon_ms := (Time.get_ticks_usec() - horizon_start_us) / 1000.0

	var total := GRID_VERTICES + SKIRT_EDGE.size()
	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var uv := PackedVector2Array()
	var uv2 := PackedVector2Array()
	var morph := PackedFloat32Array()
	var custom1 := PackedFloat32Array()
	var custom2 := PackedFloat32Array()
	vertices.resize(total)
	normals.resize(total)
	uv.resize(total)
	uv2.resize(total)
	morph.resize(4 * total)
	custom1.resize(2 * total)
	custom2.resize(2 * total)
	morph.fill(0.0)
	# Hot loop: crop constants in locals (Packed array / Rect2 lookups per vertex cost more than the arithmetic).
	var f_su := fine_map[0]
	var f_ou := fine_map[1]
	var f_sv := fine_map[2]
	var f_ov := fine_map[3]
	var b_su := base_map[0]
	var b_ou := base_map[1]
	var b_sv := base_map[2]
	var b_ov := base_map[3]
	var mask_west := box.position.x
	var mask_north := box.end.y
	var grid_uv := heights.has_horizon()  # CUSTOM2: the grid position (horizon map texel) instead of the water mask uv
	# Raw longitudes are already continuous unless the node touches the antimeridian.
	var unwrap := box.position.x <= -180.0 or box.end.x >= 180.0
	var v := 0
	for b in ROW:
		var odd_b := b & 1
		for a in ROW:
			k = (b + 1) * LATTICE + a + 1
			var p := rel[k]
			vertices[v] = p
			normals[v] = (rel[k + 1] - rel[k - 1]).cross(rel[k + LATTICE] - rel[k - LATTICE]).normalized()
			# Morph target: the parent level's surface, zero (resize fill) on even vertices: the mean of the two even
			# neighbours along an edge, inside a parent cell the mean of the parent triangles' shared diagonal
			# ((a + 1, b - 1) - (a - 1, b + 1), the split INDICES uses), which is where the parent's mesh passes.
			var i4 := 4 * v
			if a & 1 or odd_b:
				var m: Vector3
				if a & 1 and odd_b:
					m = (rel[k + 1 - LATTICE] + rel[k - 1 + LATTICE]) * 0.5 - p
				elif odd_b:
					m = (rel[k - LATTICE] + rel[k + LATTICE]) * 0.5 - p
				else:
					m = (rel[k - 1] + rel[k + 1]) * 0.5 - p
				morph[i4] = m.x
				morph[i4 + 1] = m.y
				morph[i4 + 2] = m.z
			var lat := lats[k]
			var lon := lons[k]
			if unwrap:
				lon = box_center_lon + wrapf(lon - box_center_lon, -180.0, 180.0)  # continuous in the node
			uv[v] = Vector2(lon * f_su + f_ou, lat * f_sv + f_ov)
			uv2[v] = Vector2(lon * b_su + b_ou, lat * b_sv + b_ov)
			custom1[2 * v] = (lon + 180.0) * INV_360
			custom1[2 * v + 1] = (90.0 - lat) * INV_180
			if grid_uv:
				custom2[2 * v] = a
				custom2[2 * v + 1] = b
			else:
				custom2[2 * v] = (lon - mask_west) * mask_u_scale
				custom2[2 * v + 1] = (mask_north - lat) * mask_v_scale
			v += 1

	# Parent-level normals: even vertices keep their own; odd ones the normalised mean of the neighbours their morph target
	# interpolates (odd vertices never lie on the border, so every neighbour is a grid vertex).
	var custom3 := PackedFloat32Array()
	custom3.resize(3 * total)
	for b in ROW:
		for a in ROW:
			v = b * ROW + a
			var n := normals[v]
			if a & 1 and b & 1:
				n = (normals[v + 1 - ROW] + normals[v - 1 + ROW]).normalized()
			elif b & 1:
				n = (normals[v - ROW] + normals[v + ROW]).normalized()
			elif a & 1:
				n = (normals[v - 1] + normals[v + 1]).normalized()
			custom3[3 * v] = n.x
			custom3[3 * v + 1] = n.y
			custom3[3 * v + 2] = n.z

	# Skirts: the edge vertices lowered along the geodetic normal; same channels, CUSTOM0.w = 1.
	var depth_m := CubeSphere.node_size_m(key, radius_m) / CELLS * SKIRT_DEPTH_CELLS
	var skirt_dirs := PackedVector3Array()
	var skirt_heights := PackedFloat64Array()
	for e in SKIRT_EDGE:
		k = (e / ROW + 1) * LATTICE + e % ROW + 1
		skirt_dirs.append(dirs[k])
		skirt_heights.append(height_m[k] - depth_m)
	var skirt_fixed := heights.positions_fixed(skirt_dirs, skirt_heights)
	for s in SKIRT_EDGE.size():
		v = GRID_VERTICES + s
		var e := SKIRT_EDGE[s]
		vertices[v] = Frames.to_render(skirt_fixed[s] - center_fixed)
		normals[v] = normals[e]
		uv[v] = uv[e]
		uv2[v] = uv2[e]
		morph[4 * v] = morph[4 * e]
		morph[4 * v + 1] = morph[4 * e + 1]
		morph[4 * v + 2] = morph[4 * e + 2]
		morph[4 * v + 3] = 1.0
		custom1[2 * v] = custom1[2 * e]
		custom1[2 * v + 1] = custom1[2 * e + 1]
		custom2[2 * v] = custom2[2 * e]
		custom2[2 * v + 1] = custom2[2 * e + 1]
		for c in 3:
			custom3[3 * v + c] = custom3[3 * e + c]

	return {
		"key": key,
		"center_fixed": center_fixed,
		"vertices": vertices,
		"normals": normals,
		"indices": INDICES,
		"morph": morph,
		"uv": uv,
		"uv2": uv2,
		"custom1": custom1,
		"custom2": custom2,
		"custom3": custom3,
		"imagery_fine": fine,
		"imagery_base": base,
		"water_mask": _water_mask(heights, box) if heights.has_water() else null,  # null: a body without water
		"horizon": horizon,  # null: a body without horizon maps (or without terrain data)
		"horizon_ms": horizon_ms,
		"lat_lon_box": box,
		# The finest data anywhere on the lattice: the quadtree stops splitting where even this is coarser than the mesh;
		# a site tile covering only part of the node still counts (a lattice cell or more of it).
		"max_resolution_m": heights.finest_resolution_m(lats, lons),
	}


## The node's horizon map: RGBAH, (ROW * HORIZON_BLOCKS) x ROW texels; texel (a * HORIZON_BLOCKS + c, b) holds the
## elevations (rad above the local horizontal, half floats: 0.004 deg at 5 deg, range +-pi/2 needs no clamp) of vertex
## (a, b) at azimuths 4 c .. 4 c + 3 (k * 2 pi / N_AZ clockwise from north), the core's order (vertex-major,
## azimuth-minor) reinterpreted without a copy. The observer is the vertex on the terrain surface. The shader filters
## bilinearly between the vertices itself (texelFetch), so a shared edge reads identical texels. Null without horizon
## support and on nodes coarser than HORIZON_MAX_SPACING_M.
static func _horizon_image(key: Vector4i, heights: HeightSource, radius_m: float, lats: PackedFloat64Array,
		lons: PackedFloat64Array, height_m: PackedFloat64Array) -> Image:
	if not heights.has_horizon():
		return null
	var grid_lats := PackedFloat64Array()
	var grid_lons := PackedFloat64Array()
	var grid_heights := PackedFloat64Array()
	for b in ROW:
		var k := (b + 1) * LATTICE + 1
		grid_lats.append_array(lats.slice(k, k + ROW))
		grid_lons.append_array(lons.slice(k, k + ROW))
		grid_heights.append_array(height_m.slice(k, k + ROW))
	var spacing_m := CubeSphere.node_size_m(key, radius_m) / CELLS
	if spacing_m > HORIZON_MAX_SPACING_M:
		return null
	var elevations := heights.horizon_elevations(grid_lats, grid_lons, grid_heights, N_AZ, spacing_m)
	if elevations.size() != GRID_VERTICES * N_AZ:
		return null
	var image := Image.create_from_data(ROW * HORIZON_BLOCKS, ROW, false, Image.FORMAT_RGBAF,
			elevations.to_byte_array())
	image.convert(Image.FORMAT_RGBAH)
	return image


## Elevation (rad) of vertex (a, b) at azimuth index k of a node's horizon map (the encoding of _horizon_image).
static func horizon_value(horizon: Image, a: int, b: int, k: int) -> float:
	return horizon.get_pixel(a * HORIZON_BLOCKS + (k >> 2), b)[k & 3]


## Per-crop constants for the vertex loop: [u scale, u offset, v scale, v offset] (uv = lon * su + ou, lat * sv + ov).
## No crop: all zero (uv 0; the shader's coverage fade is 0 there).
static func _crop_map(crop: Dictionary) -> PackedFloat64Array:
	var map := PackedFloat64Array()
	map.resize(4)
	map.fill(0.0)
	if crop.is_empty():
		return map
	var box: Rect2 = crop["box_deg"]
	map[0] = 1.0 / box.size.x
	map[1] = -box.position.x / box.size.x
	map[2] = -1.0 / box.size.y
	map[3] = box.end.y / box.size.y
	return map


## L8 water mask (255 = water) over the node's lat/lon box, pixel centres, one sample call.
static func _water_mask(heights: HeightSource, box: Rect2) -> Image:
	var count := WATER_MASK_PX * WATER_MASK_PX
	var row_lons := PackedFloat64Array()
	for c in WATER_MASK_PX:
		row_lons.append(wrapf(box.position.x + (c + 0.5) / WATER_MASK_PX * box.size.x, -180.0, 180.0))
	var row_lat := PackedFloat64Array()
	row_lat.resize(WATER_MASK_PX)
	var lats := PackedFloat64Array()
	var lons := PackedFloat64Array()
	for r in WATER_MASK_PX:
		row_lat.fill(box.end.y - (r + 0.5) / WATER_MASK_PX * box.size.y)
		lats.append_array(row_lat)
		lons.append_array(row_lons)
	var water: PackedByteArray = heights.sample(lats, lons)["water"]
	var pixels := PackedByteArray()
	pixels.resize(count)
	pixels.fill(0)
	if water.count(0) != count:  # some water: flag pixels (dry nodes skip the loop)
		for k in count:
			if water[k] != 0:
				pixels[k] = 255
	return Image.create_from_data(WATER_MASK_PX, WATER_MASK_PX, false, Image.FORMAT_L8, pixels)


## Grid vertex above each skirt vertex: the node border once around (b = 0, a = 64, b = 64, a = 0).
static func _make_skirt_edge() -> PackedInt32Array:
	var edge := PackedInt32Array()
	for a in CELLS:
		edge.append(a)
	for b in CELLS:
		edge.append(b * ROW + CELLS)
	for a in range(CELLS, 0, -1):
		edge.append(CELLS * ROW + a)
	for b in range(CELLS, 0, -1):
		edge.append(b * ROW)
	return edge


## Grid triangles (clockwise seen from outside = Godot front faces; each cell split along (a + 1, b) - (a, b + 1)) and
## double-sided skirt quads between the border and the skirt ring.
static func _make_indices() -> PackedInt32Array:
	var indices := PackedInt32Array()
	for b in CELLS:
		for a in CELLS:
			var k := b * ROW + a
			indices.append_array([k, k + ROW, k + 1, k + 1, k + ROW, k + ROW + 1])
	var ring := _make_skirt_edge()
	for s in ring.size():
		var e0 := ring[s]
		var e1 := ring[(s + 1) % ring.size()]
		var s0 := GRID_VERTICES + s
		var s1 := GRID_VERTICES + (s + 1) % ring.size()
		indices.append_array([e0, s0, e1, e1, s0, s1, e0, e1, s0, e1, s1, s0])
	return indices
