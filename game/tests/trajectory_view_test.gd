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
	print("TRAJECTORY VIEW TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


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
