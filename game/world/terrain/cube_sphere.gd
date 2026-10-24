class_name CubeSphere
## Cube-sphere quadtree over a body (pure math, float64, thread-safe).
##
## Node key: Vector4i(face, level, i, j). Faces 0-5 are +X, -X, +Y, -Y, +Z, -Z of the body-fixed frame; level 0 is the
## whole face; i (along the face's s axis) and j (along t) run over [0, 2^level). A face point (s, t) in [-1, 1]^2 maps
## to the unit direction of normal + tan(pi/4 s) u + tan(pi/4 t) v (tangent-adjusted gnomonic cube: near-equal node
## areas, node edges are great-circle arcs). The direction is the geodetic normal: lat = asin(z), lon = atan2(y, x) on
## the Earth (WGS84) as on the Moon (sphere).
## Directions on shared face edges are bit-identical on both faces (exact warp at s = +-1, odd-symmetric tangent,
## a length symmetric in the two warped coordinates), so neighbouring nodes sample the same surface points along
## their seam.

const FACE_NORMAL: Array[Vector3] = [Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 1, 0), Vector3(0, -1, 0),
		Vector3(0, 0, 1), Vector3(0, 0, -1)]
## Face axes with u x v = normal (outward), so (s, t) increasing is counter-clockwise seen from outside.
const FACE_U: Array[Vector3] = [Vector3(0, 1, 0), Vector3(0, -1, 0), Vector3(-1, 0, 0), Vector3(1, 0, 0),
		Vector3(1, 0, 0), Vector3(1, 0, 0)]
const FACE_V: Array[Vector3] = [Vector3(0, 0, 1), Vector3(0, 0, 1), Vector3(0, 0, 1), Vector3(0, 0, 1),
		Vector3(0, 1, 0), Vector3(0, -1, 0)]
## Boundary samples per node edge for lat_lon_box (edges are great-circle arcs: latitude / longitude extremes of the
## node lie on its boundary, except a pole inside it).
const BOX_EDGE_SAMPLES := 32
## Horizontal component below which a direction counts as a pole (longitude undefined).
const POLE_EPSILON := 1e-15


## Unit direction of face point (s, t), s and t in [-1, 1] (beyond +-1 continues the face's projection).
static func direction(face: int, s: float, t: float) -> Vector3:
	return direction_warped(face, warp(s), warp(t))


## Unit direction from already warped face coordinates (a = warp(s), b = warp(t)); for loops that cache the warp.
## The components are +-1, +-a, +-b in some order; the length uses 1 + (a^2 + b^2), symmetric in a and b, so a seam
## point seen from either face (where one of a, b is exactly +-1) gets the identical length and direction.
static func direction_warped(face: int, a: float, b: float) -> Vector3:
	return (FACE_NORMAL[face] + FACE_U[face] * a + FACE_V[face] * b) / sqrt(1.0 + (a * a + b * b))


## tan(pi/4 s), exactly +-1 at s = +-1 and odd-symmetric (bit-identical seams between faces).
static func warp(s: float) -> float:
	var m := absf(s)
	var w := 1.0 if m == 1.0 else tan(PI / 4.0 * m)
	return -w if s < 0.0 else w


## Face coordinate of lattice line k of a node: node index `index` at `level`, `cells` cells per node edge. Exact
## dyadic value, so every node of a level computes the same coordinate for a shared lattice line.
static func lattice_coordinate(index: int, level: int, k: int, cells: int) -> float:
	return -1.0 + 2.0 * float(index * cells + k) / float(cells << level)


## Corner directions (s0 t0, s1 t0, s1 t1, s0 t1).
static func node_corners(key: Vector4i) -> PackedVector3Array:
	var s0 := lattice_coordinate(key.z, key.y, 0, 1)
	var s1 := lattice_coordinate(key.z, key.y, 1, 1)
	var t0 := lattice_coordinate(key.w, key.y, 0, 1)
	var t1 := lattice_coordinate(key.w, key.y, 1, 1)
	return PackedVector3Array([direction(key.x, s0, t0), direction(key.x, s1, t0), direction(key.x, s1, t1),
			direction(key.x, s0, t1)])


## Direction of the node centre.
static func node_center(key: Vector4i) -> Vector3:
	var s := lattice_coordinate(key.z, key.y, 1, 2)
	var t := lattice_coordinate(key.w, key.y, 1, 2)
	return direction(key.x, s, t)


## Arc length of the node's longest edge on a sphere of radius_m (m).
static func node_size_m(key: Vector4i, radius_m: float) -> float:
	var c := node_corners(key)
	var angle := 0.0
	for k in 4:
		var a := c[k]
		var b := c[(k + 1) % 4]
		angle = maxf(angle, atan2(a.cross(b).length(), a.dot(b)))
	return angle * radius_m


## The four children in the order (0, 0), (1, 0), (0, 1), (1, 1) of (i, j).
static func children(key: Vector4i) -> Array[Vector4i]:
	var level := key.y + 1
	return [Vector4i(key.x, level, 2 * key.z, 2 * key.w), Vector4i(key.x, level, 2 * key.z + 1, 2 * key.w),
			Vector4i(key.x, level, 2 * key.z, 2 * key.w + 1), Vector4i(key.x, level, 2 * key.z + 1, 2 * key.w + 1)]


## The parent node; a level-0 node is its own parent.
static func parent(key: Vector4i) -> Vector4i:
	if key.y == 0:
		return key
	return Vector4i(key.x, key.y - 1, key.z >> 1, key.w >> 1)


## The node at `level` that contains the (not necessarily unit) direction.
static func key_at(dir: Vector3, level: int) -> Vector4i:
	var m := dir.abs()
	var face := 0
	if m.y >= m.x and m.y >= m.z:
		face = 2
	elif m.z >= m.x and m.z >= m.y:
		face = 4
	var major := dir.dot(FACE_NORMAL[face])
	if major < 0.0:
		face += 1
		major = -major
	var count := 1 << level
	var s := atan(dir.dot(FACE_U[face]) / major) * 4.0 / PI
	var t := atan(dir.dot(FACE_V[face]) / major) * 4.0 / PI
	return Vector4i(face, level, clampi(int(floor((s + 1.0) * 0.5 * count)), 0, count - 1),
			clampi(int(floor((t + 1.0) * 0.5 * count)), 0, count - 1))


## Geodetic latitude and longitude (deg) of a unit direction: Vector2(lat, lon); a pole has longitude 0.
static func lat_lon_deg(dir: Vector3) -> Vector2:
	var horizontal := Vector2(dir.x, dir.y).length()
	return Vector2(rad_to_deg(atan2(dir.z, horizontal)), rad_to_deg(atan2(dir.y, dir.x)))


## Unit direction (geodetic normal) of geodetic latitude / longitude (deg); the inverse of lat_lon_deg. Use it, not
## the normalised position, to find a geodetic point's node (they differ by up to 0.19 deg on the WGS84 ellipsoid).
static func geodetic_direction(lat_deg: float, lon_deg: float) -> Vector3:
	var la := deg_to_rad(lat_deg)
	var lo := deg_to_rad(lon_deg)
	return Vector3(cos(la) * cos(lo), cos(la) * sin(lo), sin(la))


## Latitude / longitude box of the node (deg): position = (west, south), size = (east - west, north - south).
## Longitudes are continuous across the node: an antimeridian node gets east > 180. A node with a pole strictly inside
## spans all longitudes; a pole on its corner extends the box to +-90 latitude.
static func lat_lon_box(key: Vector4i) -> Rect2:
	var center_lon := lat_lon_deg(node_center(key)).y
	var s0 := lattice_coordinate(key.z, key.y, 0, 1)
	var t0 := lattice_coordinate(key.w, key.y, 0, 1)
	var span := s0 - lattice_coordinate(key.z, key.y, -1, 1)
	var south := INF
	var north := -INF
	var west := INF
	var east := -INF
	for edge in 4:
		for k in BOX_EDGE_SAMPLES:
			var f := float(k) / BOX_EDGE_SAMPLES
			var st: Vector2 = [Vector2(f, 0.0), Vector2(1.0, f), Vector2(1.0 - f, 1.0), Vector2(0.0, 1.0 - f)][edge]
			var d := direction(key.x, s0 + span * st.x, t0 + span * st.y)
			var ll := lat_lon_deg(d)
			south = minf(south, ll.x)
			north = maxf(north, ll.x)
			if Vector2(d.x, d.y).length() > POLE_EPSILON:
				var lon := center_lon + wrapf(ll.y - center_lon, -180.0, 180.0)
				west = minf(west, lon)
				east = maxf(east, lon)
	if key.x >= 4 and s0 < 0.0 and s0 + span > 0.0 and t0 < 0.0 and t0 + span > 0.0:  # pole strictly inside
		west = -180.0
		east = 180.0
		if key.x == 4:
			north = 90.0
		else:
			south = -90.0
	if west < -180.0:
		west += 360.0
		east += 360.0
	return Rect2(west, south, east - west, north - south)
