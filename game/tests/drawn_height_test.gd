# Headless test of the drawn-surface height under a camera (bug 2026-11-09: an eye arriving at the lunar ground before
# the terrain under it had loaded stood under the coarse drawn surface and saw the nodes' skirts and inner sides):
# CubeSphere.face_coordinates inverts CubeSphere.direction; TerrainQuadtree.drawn_height_m is the drawn leaf's mesh
# (its triangle) at the camera on every frame of the loading, coarse levels hundreds of metres off the true ground
# included, a leaf in its first frame at its parent's surface; the envelope is never below either; SurfaceClearance.drawn_lift_m lifts an eye 2 m over the true ground to MIN_CLEARANCE_M above the drawn
# surface where that lies higher. Nodes are built synchronously on a rolling-relief source.
# Run: godot --headless --path game --script res://tests/drawn_height_test.gd
extends SceneTree

const EARTH_RADIUS_M := 6378137.0
const CELLS := TerrainNodeBuilder.CELLS
const ROW := TerrainNodeBuilder.ROW
const EYE_HEIGHT_M := 2.0
const MAX_FRAMES := 4000
## [call] the drawn height against the rebuilt mesh (float32 grid heights against float64 mesh positions: < 1 mm)
const MESH_TOLERANCE_M := 0.01
## [call] the lift against its definition (the eye's geodetic height: Bowring, sub-mm)
const LIFT_TOLERANCE_M := 1.0e-3

var _failures := 0


## Rolling relief (+-300 m, ~20 km wavelength): coarse and fine surfaces differ by tens to hundreds of metres.
class WaveSource extends HeightSource:
	const AMPLITUDE_M := 300.0
	const WAVES_PER_DEG := 5.0

	func sample(lats_deg: PackedFloat64Array, lons_deg: PackedFloat64Array) -> Dictionary:
		var heights := PackedFloat64Array()
		heights.resize(lats_deg.size())
		var water := PackedByteArray()
		water.resize(lats_deg.size())
		for k in lats_deg.size():
			heights[k] = AMPLITUDE_M * sin(TAU * WAVES_PER_DEG * lats_deg[k]) * cos(TAU * WAVES_PER_DEG * lons_deg[k])
		return {"height_m": heights, "water": water}


func _initialize() -> void:
	_check_face_coordinates()
	_check_drawn_height()
	print("DRAWN HEIGHT TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check_face_coordinates() -> void:
	var worst := 0.0
	var faded := 0  ## frames whose leaf under the eye had just appeared (morph floor 1)
	var envelope_ok := true
	var faces_ok := true
	for face in 6:
		for s in [-0.97, -0.4, 0.0, 0.31, 0.88]:
			for t in [-0.9, -0.2, 0.15, 0.6, 0.99]:
				var c := CubeSphere.face_coordinates(CubeSphere.direction(face, s, t) * 1234.5)
				faces_ok = faces_ok and int(c.x) == face
				worst = maxf(worst, maxf(absf(c.y - s), absf(c.z - t)))
	_check(faces_ok and worst < 1e-12, "face_coordinates inverts direction (worst %.3f nano)" % (worst * 1e9))


func _check_drawn_height() -> void:
	var heights := WaveSource.new()
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain_earth.gdshader")
	var tree := TerrainQuadtree.new()
	tree.synchronous = true
	root.add_child(tree)
	tree.setup(heights, null, EARTH_RADIUS_M, material)
	var lat := 10.03
	var lon := 20.07
	var ground: float = heights.sample(PackedFloat64Array([lat]), PackedFloat64Array([lon]))["height_m"][0]
	var eye := heights.position_fixed(lat, lon, ground + EYE_HEIGHT_M)
	var dir := CubeSphere.geodetic_direction(lat, lon)
	tree.update_for_camera(eye, 1.0)  # active; the roots drawn, their children requested
	var lift0 := SurfaceClearance.drawn_lift_m(eye, heights, tree)
	var root_drawn := tree.drawn_height_m(dir)
	var expected0 := maxf(root_drawn + SurfaceClearance.MIN_CLEARANCE_M - ground - EYE_HEIGHT_M, 0.0)
	_check(absf(lift0 - expected0) < LIFT_TOLERANCE_M and lift0 > 0.0,
			"roots only: the eye 2 m over the true ground (%.1f m) rises %.1f m over the root's surface (%.1f m)"
			% [ground, lift0, root_drawn])
	var worst := 0.0
	var faded := 0  ## frames whose leaf under the eye had just appeared (morph floor 1)
	var envelope_ok := true
	var farthest := 0.0  ## the drawn surface's largest distance from the true ground on the way
	var frames := 0
	var levels := {}
	for frame in MAX_FRAMES:
		tree.update_for_camera(eye, 1.0)  # 1 s: the fade-ins end at once (the morph floor is tested apart)
		frames += 1
		var key := _leaf_at(tree, dir)
		levels[key.y] = true
		var drawn := tree.drawn_height_m(dir)
		var own := _mesh_height_m(tree, key, lat, lon)
		var mesh := own
		var floor_now := float(tree.node_info(key)["morph_floor"])
		if floor_now > 0.0:  # a leaf in its first frame: drawn at the parent's surface (the fades end within 1 s)
			faded += 1
			mesh = lerpf(own, _mesh_height_m(tree, CubeSphere.parent(key), lat, lon), floor_now)
		worst = maxf(worst, absf(drawn - mesh))
		var envelope := tree.drawn_height_m(dir, true)
		envelope_ok = envelope_ok and envelope >= maxf(drawn, own) - MESH_TOLERANCE_M
		farthest = maxf(farthest, absf(drawn - ground))
		if tree.stats["nodes_loading"] == 0 and tree.stats["uploads_last_frame"] == 0:
			break
	_check(worst <= MESH_TOLERANCE_M and levels.size() > 5 and farthest > 100.0 and faded > 0,
			"loading (%d frames, %d leaf levels under the eye, drawn surface up to %.0f m off the ground): the drawn height "
			% [frames, levels.size(), farthest] + "is the leaf's mesh, a new leaf's first frame its parent's (%d frames; "
			% faded + "worst %.5f m)" % worst)
	_check(envelope_ok, "the envelope (the lift's surface) is never below the drawn surface nor the leaf's own")
	var final := tree.drawn_height_m(dir)
	var leaf := _leaf_at(tree, dir)
	var mesh_final := _mesh_height_m(tree, leaf, lat, lon)
	var lift := SurfaceClearance.drawn_lift_m(eye, heights, tree)
	var expected := maxf(final + SurfaceClearance.MIN_CLEARANCE_M - ground - EYE_HEIGHT_M, 0.0)
	_check(absf(final - mesh_final) <= MESH_TOLERANCE_M and absf(lift - expected) < LIFT_TOLERANCE_M,
			"converged (leaf level %d): drawn %.3f m over the mesh %.3f m, lift %.3f m (the eye %.1f m over the true ground)"
			% [leaf.y, final, mesh_final, lift, EYE_HEIGHT_M])
	tree.queue_free()


## The drawn leaf containing the direction.
func _leaf_at(tree: TerrainQuadtree, dir: Vector3) -> Vector4i:
	var drawn := tree.drawn_keys()
	for level in range(TerrainQuadtree.MAX_LEVEL, -1, -1):
		var key := CubeSphere.key_at(dir, level)
		if drawn.has(key):
			return key
	return Vector4i(-1, -1, -1, -1)


## The height of node `key`'s mesh (its triangles, as drawn unmorphed) above (lat, lon).
func _mesh_height_m(tree: TerrainQuadtree, key: Vector4i, lat: float, lon: float) -> float:
	var node := TerrainNodeBuilder.build(key, tree.height_source, null, tree.radius_m)
	var vertices: PackedVector3Array = node["vertices"]
	var center: Vector3 = node["center_fixed"]
	var origin := tree.height_source.position_fixed(lat, lon, 0.0)
	var up := CubeSphere.geodetic_direction(lat, lon)
	var c := CubeSphere.face_coordinates(up)
	var scale := float(CELLS << key.y) * 0.5
	var a0 := int(floor((c.y + 1.0) * scale)) - key.z * CELLS
	var b0 := int(floor((c.z + 1.0) * scale)) - key.w * CELLS
	for b in range(maxi(b0 - 1, 0), mini(b0 + 2, CELLS)):
		for a in range(maxi(a0 - 1, 0), mini(a0 + 2, CELLS)):
			var k := b * ROW + a
			for tri: Array in [[k, k + ROW, k + 1], [k + 1, k + ROW, k + ROW + 1]]:  # TerrainNodeBuilder.INDICES
				var p0: Vector3 = center + Frames.from_render(vertices[tri[0]])
				var p1: Vector3 = center + Frames.from_render(vertices[tri[1]])
				var p2: Vector3 = center + Frames.from_render(vertices[tri[2]])
				var n := (p1 - p0).cross(p2 - p0)
				var h := n.dot(p0 - origin) / n.dot(up)
				var q := origin + up * h
				var w1 := (q - p0).cross(p2 - p0).dot(n) / n.length_squared()
				var w2 := (p1 - p0).cross(q - p0).dot(n) / n.length_squared()
				if w1 >= -1e-9 and w2 >= -1e-9 and w1 + w2 <= 1.0 + 1e-9:
					return h
	return NAN


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1
