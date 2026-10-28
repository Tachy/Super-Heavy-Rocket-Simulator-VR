# Headless test of the offscreen stereo harness: StereoMockXr loads a recorded-projections JSON (here written from the
# stereo test's FOV_LEFT_EYE mirrored, IPD 0.064 m) and returns its projections / eye transforms; CloudViews' ids.
# Run: godot --headless --path game --script res://tests/stereo_mock_xr_test.gd
extends SceneTree

const TOLERANCE := 1.0e-12
const TARGET := Vector2i(1234, 987)
const NEAR_M := 0.2
const FAR_M := 3.0e7

var _failures := 0


func _initialize() -> void:
	var data := StereoMockXr.provisional_dictionary(TARGET, 0.064, NEAR_M, FAR_M)
	var path := OS.get_temp_dir().path_join("starsim_stereo_mock_test_%d.json" % OS.get_process_id())
	var file := FileAccess.open(path, FileAccess.WRITE)
	file.store_string(JSON.stringify(data))
	file.close()

	var mock := StereoMockXr.new()
	_check(mock.load(path), "the JSON loads")
	DirAccess.remove_absolute(path)
	_check(mock._get_view_count() == 2, "view count 2")
	_check(mock._get_render_target_size() == Vector2(TARGET), "render target as given: %s" % mock._get_render_target_size())
	_check(mock._get_capabilities() == XRInterface.XR_STEREO, "stereo capability")
	for view in range(2):
		var expected: Array = data["views"][view]["projection"]
		var got := mock._get_projection_for_view(view, 1.0, NEAR_M, FAR_M)
		_check(got.size() == 16, "view %d: 16 values" % view)
		var worst := 0.0
		for i in range(mini(got.size(), 16)):
			worst = maxf(worst, absf(got[i] - float(expected[i])) / maxf(1.0, absf(float(expected[i]))))
		_check(worst < TOLERANCE, "view %d: projection within %s of the recorded (worst %s)" % [view, TOLERANCE, worst])
		# a different near / far keeps the frustum's angles: the same tangents at the new near plane
		var other := mock.view_projection(view, 1.0, 1.0e6)
		var tangents := StereoMockXr._frustum_tangents(other)
		var recorded := StereoMockXr._frustum_tangents(mock._projections[view])
		for k in range(4):
			_check(absf(tangents[k] - recorded[k]) < TOLERANCE, "view %d: tangent %d kept at another near / far" % [view, k])
	# the eyes at -+IPD/2 relative to the head, and relative to a moved head
	var head := Transform3D(Basis.from_euler(Vector3(0.1, 0.2, 0.3)), Vector3(1.0, 2.0, 3.0))
	var eye_l := mock._get_transform_for_view(0, head)
	var eye_r := mock._get_transform_for_view(1, head)
	_check(((eye_r.origin - eye_l.origin).length() - 0.064) < TOLERANCE, "eye separation 0.064 m")
	_check((eye_l.origin - head * Vector3(-0.032, 0.0, 0.0)).length() < TOLERANCE, "left eye 32 mm left of the head")
	# the recording of an initialised interface round-trips (mock -> JSON dictionary -> mock)
	_check(XRServer.find_interface("StereoMock") == null, "not registered by default")
	XRServer.add_interface(mock)
	_check(mock.initialize(), "initialises")
	var recorded_dict := StereoMockXr.record(mock, NEAR_M, FAR_M)
	_check(recorded_dict["render_target"] == [TARGET.x, TARGET.y], "recorded render target")
	var mock2 := StereoMockXr.new()
	_check(mock2.load_dictionary(recorded_dict), "the recording loads")
	var worst_round := 0.0
	for view in range(2):
		var a := mock.view_projection(view, NEAR_M, FAR_M)
		var b := mock2.view_projection(view, NEAR_M, FAR_M)
		for i in range(16):
			worst_round = maxf(worst_round, absf(StereoMockXr.projection_to_array(a)[i] - StereoMockXr.projection_to_array(b)[i]))
	_check(worst_round < TOLERANCE, "record -> load round trip (worst %s)" % worst_round)
	mock.uninitialize()
	XRServer.remove_interface(mock)

	_check("--eye_height=9000" in CloudViews.args("V3"), "CloudViews.args(\"V3\") has --eye_height=9000")
	_check("--weather=analog" in CloudViews.args("V3_analog"), "V3_analog has the analog weather")
	_check(not "--weather=analog" in CloudViews.args("V3"), "V3 keeps its slab")
	_check(_any(CloudViews.args("V3_era5"), func(a: String) -> bool: return a.begins_with("--weather=") and a.ends_with(".wthr")),
			"V3_era5 has a .wthr weather file")
	_check(CloudViews.args("V4_80km")[CloudViews.args("V4_80km").size() - 1] != "", "V4_80km exists")
	_check(CloudViews.args("nonsense").is_empty(), "an unknown id gives an empty list")
	_check(_any(CloudViews.args("V7"), func(a: String) -> bool: return a.begins_with("--view=")),
			"V7 has a fixed --view index")
	for id in CloudViews.ids():
		for argument in CloudViews.args(id):
			_check(argument.begins_with("--") and not "<" in argument and not "{" in argument,
					"%s: a plain argument (%s)" % [id, argument])
	print("stereo_mock_xr_test: %s" % ("PASS" if _failures == 0 else "FAIL (%d)" % _failures))
	quit(1 if _failures > 0 else 0)


func _any(list: PackedStringArray, test: Callable) -> bool:
	for item in list:
		if test.call(item):
			return true
	return false


func _check(condition: bool, message: String) -> void:
	if not condition:
		_failures += 1
		print("FAIL: ", message)
