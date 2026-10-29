# Headless test of the free camera's curves, mapping signs and pose frames (FreeCameraControl, FreeCameraPose).
# Run: godot --headless --path game --script res://tests/free_camera_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	# Logarithmic speed: dead zone, endpoints, monotone, signed.
	var dz := 0.05
	_check(FreeCameraControl.log_speed_mps(0.0, dz, 0.1, 1.0e7) == 0.0, "zero deflection: standstill")
	_check(FreeCameraControl.log_speed_mps(0.05, dz, 0.1, 1.0e7) == 0.0, "edge of the dead zone: standstill")
	_check(is_equal_approx(FreeCameraControl.log_speed_mps(0.0501, dz, 0.1, 1.0e7), 0.1 * pow(1.0e8, 0.0001 / 0.95)),
			"just outside the dead zone: about v_min")
	_check(is_equal_approx(FreeCameraControl.log_speed_mps(1.0, dz, 0.1, 1.0e7), 1.0e7), "full deflection: v_max")
	_check(is_equal_approx(FreeCameraControl.log_speed_mps(-1.0, dz, 0.1, 1.0e7), -1.0e7), "negative full deflection: -v_max")
	var middle := FreeCameraControl.log_speed_mps((1.0 + dz) * 0.5, dz, 0.1, 1.0e7)
	_check(is_equal_approx(middle, sqrt(0.1 * 1.0e7)), "half way: the geometric mean of v_min and v_max")
	var previous := 0.0
	var monotone := true
	for i in range(1, 101):
		var speed := FreeCameraControl.log_speed_mps(i / 100.0, dz, 0.1, 1.0e7)
		monotone = monotone and speed >= previous
		previous = speed
	_check(monotone, "speed grows with the deflection")

	# Rotation: dead zone, expo, full rate.
	var max_rate := deg_to_rad(90.0)
	_check(FreeCameraControl.rotation_rate_rad_s(0.04, 0.05, 0.3, max_rate) == 0.0, "rotation dead zone")
	_check(is_equal_approx(FreeCameraControl.rotation_rate_rad_s(1.0, 0.05, 0.3, max_rate), max_rate), "full deflection: 90 deg/s")
	_check(is_equal_approx(FreeCameraControl.rotation_rate_rad_s(-1.0, 0.05, 0.3, max_rate), -max_rate), "signed")
	var half := FreeCameraControl.rotation_rate_rad_s(0.525, 0.05, 0.3, max_rate)  # x = 0.5
	_check(is_equal_approx(half, (0.7 * 0.5 + 0.3 * 0.125) * max_rate), "expo curve at x = 0.5")

	# Rotation about the local axes: a quarter turn of yaw (positive about +y) turns the view (-z) to -x.
	var turned := FreeCameraControl.rotate_local(Basis.IDENTITY, Vector3(0.0, PI / 2.0, 0.0), 1.0)
	_check((turned * Vector3.FORWARD).is_equal_approx(Vector3.LEFT), "positive yaw rate turns the view to the left")
	_check(turned.is_equal_approx(turned.orthonormalized()), "rotation stays orthonormal")

	# Signs of the commands: simulate the axes through configure + a fake device is not possible headless, so the sign
	# convention is checked on the composed pose instead.
	var pose := FreeCameraPose.new()
	pose.reset_to(Vector3(7.0e6, 0.0, 0.0), Basis.IDENTITY, Basis.IDENTITY, Basis.IDENTITY, Vector3(3.8e8, 0.0, 0.0))
	_check(pose.frame == FreeCameraPose.FRAME_EARTH, "near the Earth: its frame")
	var out := pose.step(Vector3.ZERO, Vector3(0.0, 0.0, -100.0), 1.0, Basis.IDENTITY, Basis.IDENTITY, Vector3(3.8e8, 0.0, 0.0))
	# Identity basis looks along render -z = GCRF -x? Frames.from_render maps render z to GCRF x: -z -> -x.
	_check((out["position_gcrf_m"] - Vector3(7.0e6 - 100.0, 0.0, 0.0)).length() < 1e-6, "forward moves along the view")

	# Co-rotation: the Earth turns 90 deg about z; the camera in its frame goes with the ground.
	var turn := Basis(Vector3(0.0, 0.0, 1.0), PI / 2.0)
	var moved := pose.step(Vector3.ZERO, Vector3.ZERO, 0.0, turn, Basis.IDENTITY, Vector3(3.8e8, 0.0, 0.0))
	_check((moved["position_gcrf_m"] - turn * Vector3(7.0e6 - 100.0, 0.0, 0.0)).length() < 1e-6, "stays over the same ground")
	_check((moved["basis"] * Vector3.FORWARD).is_equal_approx(Frames.basis_to_render(turn) * Vector3.FORWARD),
			"the view turns with the ground")

	# Frame switching: no jump, hysteresis.
	var far := FreeCameraPose.new()
	far.reset_to(Vector3(1.0e8, 0.0, 0.0), Basis.IDENTITY, Basis.IDENTITY, Basis.IDENTITY, Vector3(3.8e8, 0.0, 0.0))
	_check(far.frame == FreeCameraPose.FRAME_INERTIAL, "far from both: inertial")
	var near_moon := FreeCameraPose.new()
	near_moon.reset_to(Vector3(3.8e8 + 5.0e6, 0.0, 0.0), Basis.IDENTITY, Basis.IDENTITY, Basis.IDENTITY, Vector3(3.8e8, 0.0, 0.0))
	_check(near_moon.frame == FreeCameraPose.FRAME_MOON, "near the Moon: its frame")
	var leaving := FreeCameraPose.new()
	leaving.reset_to(Vector3(2.9e7, 0.0, 0.0), Basis.IDENTITY, Basis.IDENTITY, Basis.IDENTITY, Vector3(3.8e8, 0.0, 0.0))
	var jump := leaving.step(Vector3.ZERO, Vector3(0.0, 0.0, 2.0e6), 1.0, Basis.IDENTITY, Basis.IDENTITY, Vector3(3.8e8, 0.0, 0.0))
	_check(leaving.frame == FreeCameraPose.FRAME_EARTH, "3.1e7 m: still in the Earth frame (hysteresis)")
	leaving.step(Vector3.ZERO, Vector3(0.0, 0.0, 3.0e6), 1.0, Basis.IDENTITY, Basis.IDENTITY, Vector3(3.8e8, 0.0, 0.0))
	_check(leaving.frame == FreeCameraPose.FRAME_INERTIAL, "3.4e7 m: inertial")
	_check(jump["position_gcrf_m"].x > 3.0e7, "moved on")

	_test_calibration()

	print("[free_camera_test] %s (%d failures)" % ["OK" if _failures == 0 else "FAIL", _failures])
	quit(1 if _failures > 0 else 0)


func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		print("[free_camera_test] FAIL: ", what)


## Wizard logic with synthetic samples: axis "0:1" rests at +0.2 and goes to -0.9 (asked direction) and +0.7 (opposite);
## "0:3" jitters, "0:4" is an axis that moved in an earlier step (excluded).
func _test_calibration() -> void:
	var cal := FreeCameraCalibration.new()
	var rest := {"0:1": 0.2, "0:3": 0.0, "0:4": 0.0}
	cal.begin(rest, ["0:4"])
	var script := [-0.1, -0.5, -0.9, -0.2, 0.3, 0.7, 0.4, 0.21, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
	var settled_early := false
	for i in script.size():
		cal.feed({"0:1": script[i], "0:3": 0.03 * (i % 2), "0:4": 1.0 if i == 4 else 0.0}, 0.1)
		if i < 8 and cal.settled():
			settled_early = true
	_check(cal.best_key() == "0:1", "wizard: the axis with the largest excursion wins, jitter and excluded axes lose")
	_check(not settled_early and cal.settled(), "wizard: settles only after returning to rest for a while")
	var result := cal.result()
	_check(is_equal_approx(result["center"], 0.2), "wizard: off-centre rest value is the centre")
	_check(is_equal_approx(result["raw_pos"], -0.9) and is_equal_approx(result["raw_neg"], 0.7),
			"wizard: asked direction first -> raw_pos is that extreme (the axis is inverted), the other end raw_neg")
	_check(is_equal_approx(FreeCameraCalibration.normalize(0.2, result), 0.0), "normalize: centre -> 0")
	_check(is_equal_approx(FreeCameraCalibration.normalize(-0.9, result), 1.0), "normalize: asked extreme -> +1")
	_check(is_equal_approx(FreeCameraCalibration.normalize(0.7, result), -1.0), "normalize: opposite extreme -> -1")
	_check(is_equal_approx(FreeCameraCalibration.normalize(-0.35, result), 0.5), "normalize: asked side scaled by its own extent")
	_check(is_equal_approx(FreeCameraCalibration.normalize(0.45, result), -0.5), "normalize: asymmetric opposite side scaled separately")
	_check(FreeCameraCalibration.normalize(-5.0, result) == 1.0 and FreeCameraCalibration.normalize(5.0, result) == -1.0,
			"normalize: clamped")
	# Nothing moved: no result. One-sided use: mirrored about the centre.
	var idle := FreeCameraCalibration.new()
	idle.begin({"0:0": 0.0}, [])
	idle.feed({"0:0": 0.1}, 0.1)
	_check(idle.best_key() == "" and idle.result().is_empty() and not idle.settled(), "wizard: no movement, no axis")
	var one_sided := FreeCameraCalibration.new()
	one_sided.begin({"1:5": -0.1}, [])
	one_sided.feed({"1:5": 0.8}, 0.1)
	var single := one_sided.result()
	_check(is_equal_approx(single["raw_pos"], 0.8) and is_equal_approx(single["raw_neg"], -1.0), "wizard: unused side mirrored")
	# Save / load round trip.
	var path := "user://free_camera_test_%d.json" % Time.get_ticks_usec()
	var entries := {"roll": {"device_name": "T16000M", "axis": 1, "center": 0.2, "raw_pos": -0.9, "raw_neg": 0.7}}
	_check(FreeCameraCalibration.save(path, entries), "calibration file written")
	var loaded := FreeCameraCalibration.load_file(path)
	_check(loaded.has("roll") and int(loaded["roll"]["axis"]) == 1 and is_equal_approx(loaded["roll"]["raw_neg"], 0.7),
			"calibration file round trip")
	DirAccess.remove_absolute(path)
