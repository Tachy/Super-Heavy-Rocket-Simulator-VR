# Headless test of the external-camera math (CameraMath) and the desktop start mode of XrManager.
# Run: godot --headless --path game --script res://tests/camera_math_test.gd
extends SceneTree

const EPS := 1e-9
const RANDOM_POSES := 20

var _failures := 0
var _frames := 0


func _initialize() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = 20261022
	var up := Vector3(0.3, 0.8, -0.52).normalized()
	var frame := CameraMath.horizon_frame(up, Vector3.FORWARD)
	_check(frame.y.is_equal_approx(up) and _orthonormal(frame), "horizon frame: y = up, orthonormal")
	_check((-frame.z).dot(Vector3.UP) > 0.0 and absf(frame.z.dot(up)) < EPS, "horizon frame: -z points north")
	_check(frame.determinant() > 0.0, "horizon frame is right-handed")

	# No roll: the camera x-axis stays horizontal for any yaw / pitch; the camera looks at the target.
	var max_roll := 0.0
	var max_length_error := 0.0
	var max_aim_error := 0.0
	for i in RANDOM_POSES:
		var yaw := rng.randf_range(-PI, PI)
		var pitch := rng.randf_range(-CameraMath.PITCH_LIMIT, CameraMath.PITCH_LIMIT)
		var distance := rng.randf_range(1.0, 1e9)
		var view := CameraMath.orbit_basis(frame, yaw, pitch)
		var offset := CameraMath.orbit_offset(frame, yaw, pitch, distance)
		max_roll = maxf(max_roll, absf(view.x.dot(up)))
		max_length_error = maxf(max_length_error, absf(offset.length() - distance) / distance)
		max_aim_error = maxf(max_aim_error, (-view.z).distance_to(-offset.normalized()))
	_check(max_roll < EPS, "camera x-axis perpendicular to up (max |x.up| %s)" % String.num_scientific(max_roll))
	_check(max_length_error < EPS, "orbit_offset length == distance (max rel. error %s)" % String.num_scientific(max_length_error))
	_check(max_aim_error < EPS, "camera looks at the target (max error %s)" % String.num_scientific(max_aim_error))

	# Zoom: x1.15 per notch, clamped.
	_check(CameraMath.zoom(100.0, 1, 1.0, 1e9) == 100.0 / 1.15, "zoom(100, 1) == 100 / 1.15")
	_check(CameraMath.zoom(100.0, -1, 1.0, 1e9) == 100.0 * 1.15, "zoom(100, -1) == 100 * 1.15")
	_check(CameraMath.zoom(1.05, 3, 1.0, 1e9) == 1.0, "zoom clamps at the minimum")
	_check(CameraMath.zoom(9.9e8, -3, 1.0, 1e9) == 1e9, "zoom clamps at the maximum")

	# Blend between two horizon frames (Earth <-> Moon reference change).
	var other := CameraMath.horizon_frame(Vector3(-0.7, 0.1, 0.7).normalized(), Vector3.FORWARD)
	_check(CameraMath.blend_frame(frame, other, 0.0).is_equal_approx(frame), "blend_frame(t=0) == from")
	_check(CameraMath.blend_frame(frame, other, 1.0).is_equal_approx(other), "blend_frame(t=1) == to")
	var half := CameraMath.blend_frame(frame, other, 0.5)
	_check(_orthonormal(half) and half.determinant() > 0.0, "blend_frame(t=0.5) is a rotation")
	_check(CameraMath.blend_frame(frame, other, -1.0).is_equal_approx(frame)
			and CameraMath.blend_frame(frame, other, 2.0).is_equal_approx(other), "blend_frame clamps t to [0, 1]")

	# Review Focus 5: up parallel to the pole axis and the view at the pitch limit stay finite, no roll.
	var pole_frame := CameraMath.horizon_frame(Vector3.UP, Vector3.RIGHT)
	_check(_finite(pole_frame) and _orthonormal(pole_frame), "horizon frame at the pole is finite")
	_check((-pole_frame.z).is_equal_approx(Vector3.RIGHT), "horizon frame at the pole keeps the previous north")
	var south_frame := CameraMath.horizon_frame(Vector3.DOWN, Vector3.ZERO)
	_check(_finite(south_frame) and _orthonormal(south_frame), "horizon frame at the south pole without a previous north")
	var zero_frame := CameraMath.horizon_frame(Vector3.ZERO, Vector3.FORWARD)
	_check(_finite(zero_frame) and _orthonormal(zero_frame), "horizon frame of a zero up vector is finite")
	for limit_frame: Basis in [frame, pole_frame]:
		for pitch: float in [CameraMath.PITCH_LIMIT, -CameraMath.PITCH_LIMIT, PI / 2.0, -PI / 2.0]:
			var view := CameraMath.orbit_basis(limit_frame, 0.7, pitch)
			var offset := CameraMath.orbit_offset(limit_frame, 0.7, pitch, 1e6)
			_check(_finite(view) and _orthonormal(view) and offset.is_finite()
					and absf(view.x.dot(limit_frame.y)) < EPS, "pitch %.3f: finite basis without roll" % pitch)
	var pole_blend := CameraMath.blend_frame(frame, pole_frame, 0.5)
	_check(_finite(pole_blend) and _orthonormal(pole_blend), "blend towards the pole frame is finite")

	# VR recentre: only yaw and position of the head; the head then sits at the base pose.
	var head := Transform3D(Basis.from_euler(Vector3(0.3, 1.1, -0.2)), Vector3(0.2, 1.6, -0.4))
	var recenter := CameraMath.recenter_offset(head)
	_check(recenter.basis.y.is_equal_approx(Vector3.UP) and recenter.origin == head.origin,
			"recenter offset: yaw and position only")
	var base := CameraMath.orbit_basis(frame, 0.4, -0.3)
	var origin := CameraMath.vr_origin_transform(base, recenter)
	var head_global := origin * Transform3D(Basis(Vector3.UP, 1.1), head.origin)
	_check(head_global.origin.length() < 1e-9 and head_global.basis.is_equal_approx(base),
			"after recentre the head sits at the base pose (render origin)")

	_check_vr_panel_mesh()


# VR overlay panel: every vertex on the 2 m sphere around the eye, inside +-H_FOV/2 of -Z, front faces inward.
func _check_vr_panel_mesh() -> void:
	var arrays := VrOverlayPanel.build_mesh().surface_get_arrays(0)
	var vertices: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
	var indices: PackedInt32Array = arrays[Mesh.ARRAY_INDEX]
	var half_h := deg_to_rad(VrOverlayPanel.H_FOV_DEG) / 2.0
	var max_radius_error := 0.0
	var max_azimuth := 0.0
	var max_elevation := 0.0
	for v in vertices:
		max_radius_error = maxf(max_radius_error, absf(v.length() - VrOverlayPanel.RADIUS_M))
		max_azimuth = maxf(max_azimuth, absf(atan2(v.x, -v.z)))
		max_elevation = maxf(max_elevation, absf(asin(v.y / v.length())))
	_check(vertices.size() == (VrOverlayPanel.SEGMENTS_H + 1) * (VrOverlayPanel.SEGMENTS_V + 1),
			"panel mesh: 64 x 36 segments")
	_check(max_radius_error < 1e-3, "panel mesh: every vertex 2.0 m from the eye (max error %s m)"
			% String.num_scientific(max_radius_error))
	_check(max_azimuth <= half_h + 1e-6, "panel mesh: azimuth within +-H_FOV/2 (max %.3f deg)" % rad_to_deg(max_azimuth))
	_check(max_elevation <= half_h * 9.0 / 16.0 + 1e-6, "panel mesh: 16:9 vertical span (max %.3f deg)" % rad_to_deg(max_elevation))
	var inward := true
	for t in indices.size() / 3:
		var a := vertices[indices[3 * t]]
		var normal := (vertices[indices[3 * t + 1]] - a).cross(vertices[indices[3 * t + 2]] - a)
		inward = inward and normal.dot(a) > 0.0  # clockwise seen from the origin = outward right-hand normal
	_check(inward, "panel mesh: all triangles wound as front faces seen from the eye")
	var material := VrOverlayPanel.build_material(null)
	_check(material.blend_mode == BaseMaterial3D.BLEND_MODE_PREMULT_ALPHA and material.no_depth_test
			and material.cull_mode == BaseMaterial3D.CULL_BACK
			and material.render_priority == RenderingServer.MATERIAL_RENDER_PRIORITY_MAX,
			"panel material: premultiplied, no depth test, cull back, max priority")


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames < 2:
		return false
	# Headless and --script runs are always desktop.
	var xr := root.get_node_or_null("XrManager")
	_check(xr != null, "XrManager autoload present")
	if xr != null:
		_check(not xr.xr_active, "headless run is desktop (xr_active false)")
		_check(xr.origin is XROrigin3D, "XrManager owns an XROrigin3D")
		xr.set_base(Basis(Vector3.UP, 0.5))
		_check(xr.origin.transform.origin == Vector3.ZERO, "set_base keeps the origin at the render origin")
	_check(_check_up_blend(), "OrbitCamera up-blend checks ran to the end")
	print("CAMERA MATH TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)
	return false


# OrbitCamera: Earth -> Moon reference change blends "up" over UP_BLEND_S without a jump or roll (Review Focus 5).
func _check_up_blend() -> bool:
	var camera := OrbitCamera.new()
	root.add_child(camera)
	_check(camera.current, "desktop: the OrbitCamera is the current camera")
	var moon_center := Vector3(-2.9e8, -2.7e8, -7.6e7)
	camera.target_gcrf_m = Vector3(1.0e8, 2.0e8, 5.0e7)  # between Earth and Moon, far from both poles
	camera.reference_center_gcrf_m = Vector3.ZERO
	camera.reference_body = "earth"
	camera.update_pose(0.016)
	var earth_view := camera.basis
	camera.reference_center_gcrf_m = moon_center
	camera.reference_body = "moon"
	camera.update_pose(0.0)
	_check(camera.basis.is_equal_approx(earth_view), "reference change: no jump at the start of the blend")
	var moon_up := Frames.to_render(camera.target_gcrf_m - moon_center).normalized()
	var moon_view := CameraMath.orbit_basis(CameraMath.horizon_frame(moon_up, Vector3.FORWARD), camera.yaw, camera.pitch)
	var steps := 10
	var previous_angle := INF
	var monotonic := true
	for i in steps - 1:
		camera.update_pose(CameraMath.UP_BLEND_S / steps)
		var angle := camera.basis.get_rotation_quaternion().angle_to(moon_view.get_rotation_quaternion())
		monotonic = monotonic and angle < previous_angle and _finite(camera.basis) and _orthonormal(camera.basis)
		previous_angle = angle
	_check(monotonic, "blend turns the view steadily towards the Moon frame")
	_check(previous_angle > 1e-3, "blend still running at 90 %% of UP_BLEND_S (%.4f rad left)" % previous_angle)
	camera.update_pose(CameraMath.UP_BLEND_S / steps)
	_check(camera.basis.is_equal_approx(moon_view), "after UP_BLEND_S the Moon's local vertical is up")
	camera.queue_free()
	return true


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1


static func _orthonormal(b: Basis) -> bool:
	return (b * b.transposed()).is_equal_approx(Basis.IDENTITY)


static func _finite(b: Basis) -> bool:
	return b.x.is_finite() and b.y.is_finite() and b.z.is_finite()
