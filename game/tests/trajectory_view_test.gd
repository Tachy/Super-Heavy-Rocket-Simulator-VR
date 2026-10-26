# Headless test of TrajectoryView's placement: the ribbon is rebuilt only with new points or after the camera moved
# REBUILD_DISTANCE_M from the build's anchor; in between the built mesh is moved with the camera (VR performance).
# Run: godot --headless --path game --script res://tests/trajectory_view_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	var view := TrajectoryView.new()
	root.add_child(view)
	await process_frame  # _ready assigns the mesh
	var points := PackedVector3Array()
	for i in 36:
		var a := TAU * i / 36.0
		points.append(Vector3(cos(a), sin(a), 0.0) * 6.8e6)
	var camera := Vector3(6.5e6, 1000.0, 2000.0)
	view.place(points, true, camera)
	var first := _first_vertex(view)
	_check(view.mesh.get_surface_count() == 1 and view.position == Vector3.ZERO, "new points: built at the camera")

	var moved := camera + Vector3(300.0, -200.0, 100.0)
	view.place(points, false, moved)
	_check(_first_vertex(view) == first, "small camera move: the mesh is kept")
	_check(view.position.is_equal_approx(Frames.to_render(camera - moved)), "small camera move: moved with the camera")

	var far := camera + Vector3(TrajectoryView.REBUILD_DISTANCE_M * 1.5, 0.0, 0.0)
	view.place(points, false, far)
	_check(_first_vertex(view) != first and view.position == Vector3.ZERO, "camera beyond the rebuild distance: rebuilt")

	view.place(points, true, far)
	_check(view.position == Vector3.ZERO and view.mesh.get_surface_count() == 1, "new points again: rebuilt, one surface")
	view.place(PackedVector3Array(), true, far)
	_check(view.mesh.get_surface_count() == 0, "no points: nothing drawn")
	_test_ribbon_arrays()
	print("TRAJECTORY VIEW TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


## RibbonMesh's vertex layout (pinned before its loop was made faster): per segment four corners - start left, start
## right, end left, end right - each with the point one segment further along (CUSTOM0), the true distance and side (UV),
## the length along the line (UV2); two triangles per segment.
func _test_ribbon_arrays() -> void:
	var placed := PackedVector3Array([Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(1, 2, 0)])
	var mesh := ArrayMesh.new()
	RibbonMesh.add_local(mesh, placed, PackedFloat32Array([0.0, 1.0, 3.0]), null)
	var a := mesh.surface_get_arrays(0)
	var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
	_check(v.size() == 8 and v[0] == placed[0] and v[1] == placed[0] and v[2] == placed[1] and v[3] == placed[1]
			and v[4] == placed[1] and v[7] == placed[2], "ribbon: four corners per open segment")
	var uv: PackedVector2Array = a[Mesh.ARRAY_TEX_UV]
	_check(uv[0] == Vector2(0.0, -1.0) and uv[1] == Vector2(0.0, 1.0) and uv[3] == Vector2(0.0, 1.0), "ribbon: side in UV.y")
	var uv2: PackedVector2Array = a[Mesh.ARRAY_TEX_UV2]
	_check(uv2[0].x == 0.0 and uv2[2].x == 1.0 and uv2[7].x == 3.0, "ribbon: length along the line in UV2.x")
	var custom: PackedFloat32Array = a[Mesh.ARRAY_CUSTOM0]
	# corner 0 (start of segment 0): ahead = start + (end - start) = (1, 0, 0); corner 6 (end of segment 1): (1, 4, 0)
	_check(custom.size() == 32 and Vector3(custom[0], custom[1], custom[2]) == Vector3(1, 0, 0)
			and Vector3(custom[24], custom[25], custom[26]) == Vector3(1, 4, 0) and custom[3] == 0.0,
			"ribbon: the point one segment ahead in CUSTOM0")
	var idx: PackedInt32Array = a[Mesh.ARRAY_INDEX]
	_check(idx == PackedInt32Array([0, 1, 2, 1, 3, 2, 4, 5, 6, 5, 7, 6]), "ribbon: two triangles per segment")
	var closed := ArrayMesh.new()
	RibbonMesh.add(closed, placed, Vector3.ZERO, true, null)
	var cv: PackedVector3Array = closed.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
	var cuv: PackedVector2Array = closed.surface_get_arrays(0)[Mesh.ARRAY_TEX_UV]
	_check(cv.size() == 12 and cv[11] == Frames.to_render(placed[0]) and is_equal_approx(cuv[2].x, 1.0),
			"ribbon: closed line wraps to the first point; true distance in UV.x")


func _first_vertex(view: TrajectoryView) -> Vector3:
	if view.mesh.get_surface_count() == 0:
		return Vector3.INF
	return view.mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX][0]


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
