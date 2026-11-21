# GPU test (needs a window, the generated Dragon models and a mission session): the cockpit shots show the left screen's feed.
# Runs cockpit_shots.gd for the pilot_eye shot in a child Godot process and measures the mean luminance of the screen_left region
# of the PNG (the shot prints the region it projected). The placeholder is black: the feed must lift it by more than 0.02. Also
# checks config/cockpits/photo_cameras.toml (rows, unit axes, models).
# Run: godot --path game --script res://tests/cockpit_shots_feed_test.gd   (or tools/run_godot_tests.ps1 -Only cockpit_shots_feed_test -Gpu)
extends SceneTree

const MIN_FEED_LUMINANCE := 0.02
const TIMEOUT_MS := 240000

var _failures := 0


func _initialize() -> void:
	_check_photo_cameras()
	_check_feed()
	if _failures > 0:
		print("cockpit_shots_feed_test: FAILED (%d)" % _failures)
		quit(1)
	else:
		print("cockpit_shots_feed_test: OK")
		quit(0)


func _check(condition: bool, what: String) -> void:
	if not condition:
		_failures += 1
		push_error("cockpit_shots_feed_test: FAIL " + what)


func _check_photo_cameras() -> void:
	var rows: Array = TomlLite.load_file(Paths.config_dir().path_join("cockpits/photo_cameras.toml")).get("photo", [])
	_check(rows.size() >= 2, "photo_cameras.toml has the crew1_1 and discovery2 rows")
	var ids: Array = []
	for row: Dictionary in rows:
		ids.append(row["id"])
		var forward := Vector3(row["forward"][0], row["forward"][1], row["forward"][2])
		var up := Vector3(row["up"][0], row["up"][1], row["up"][2])
		_check(absf(forward.length() - 1.0) < 1.0e-3 and absf(up.length() - 1.0) < 1.0e-3, "%s: unit axes" % row["id"])
		_check(absf(forward.dot(up)) < 1.0e-3, "%s: forward and up are orthogonal" % row["id"])
		_check(["pinhole", "fisheye_equidistant"].has(row["model"]), "%s: known model" % row["id"])
		_check(float(row["fov_h_deg"]) > 10.0 and float(row["fov_h_deg"]) < 180.0, "%s: field of view" % row["id"])
		_check(String(row["file"]).begins_with("screenshots/m7-cockpit-refs/"), "%s: file relative to the repo" % row["id"])
	_check(ids.has("crew1_1") and ids.has("discovery2"), "rows crew1_1 and discovery2")


func _check_feed() -> void:
	var out_dir := ProjectSettings.globalize_path("res://").path_join("../screenshots/m7-cockpit/fidelity/t1/feed_test").simplify_path()
	DirAccess.make_dir_recursive_absolute(out_dir)
	var output: Array = []
	var args := PackedStringArray(["--path", ProjectSettings.globalize_path("res://"), "--script", "res://tests/cockpit_shots.gd", "--",
			"--out_dir=" + out_dir, "--only=pilot_eye", "--t_plus=-60"])
	var code := OS.execute(OS.get_executable_path(), args, output, true)
	var text := "\n".join(PackedStringArray(output))
	_check(code == 0, "cockpit_shots ran (exit %d)" % code)
	var rect := Rect2i()
	for line in text.split("\n"):
		if line.contains("screen_left_rect pilot_eye"):
			var numbers := line.split("rect pilot_eye")[1].strip_edges().split(" ")
			rect = Rect2i(int(numbers[0]), int(numbers[1]), int(numbers[2]), int(numbers[3]))
	_check(rect.size.x > 20 and rect.size.y > 20, "the shot reported the left screen region (%s)" % rect)
	var image := Image.load_from_file(out_dir.path_join("pilot_eye.png"))
	_check(image != null, "pilot_eye.png written")
	if image == null:
		return
	# the projected box also covers the bezel / console around the glass: measure its central 40 % (always glass)
	rect = Rect2i(rect.position + rect.size * 3 / 10, rect.size * 4 / 10).intersection(Rect2i(Vector2i.ZERO, image.get_size()))
	var sum := 0.0
	var n := 0
	for y in range(rect.position.y, rect.end.y, 2):
		for x in range(rect.position.x, rect.end.x, 2):
			var c := image.get_pixel(x, y)
			sum += (c.r + c.g + c.b) / 3.0
			n += 1
	var mean := sum / maxf(n, 1)
	print("cockpit_shots_feed_test: screen_left mean luminance %.4f over %d samples" % [mean, n])
	_check(mean > MIN_FEED_LUMINANCE, "the left screen shows the feed (mean %.4f > %.2f)" % [mean, MIN_FEED_LUMINANCE])
