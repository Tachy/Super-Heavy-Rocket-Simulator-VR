# Headless test of WeatherFetcher's completion branches (Task 7 fix round): the Range header, which answers are accepted,
# temp file + rename and the `.part` cleanup - with synthetic results, no network.
# Run: godot --headless --path game --script res://tests/weather_fetcher_test.gd
extends SceneTree

var _failures := 0
var _dir := ""


func _initialize() -> void:
	_dir = OS.get_user_data_dir().path_join("weather_test_fetcher_%d" % OS.get_process_id())
	DirAccess.make_dir_recursive_absolute(_dir)
	_test_range_header()
	_test_accepts()
	_test_settle_success()
	_test_settle_failures()
	_test_timeout_constants()
	for name: String in DirAccess.get_files_at(_dir):
		DirAccess.remove_absolute(_dir.path_join(name))
	DirAccess.remove_absolute(_dir)
	print("WEATHER FETCHER TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _write(path: String, bytes: PackedByteArray) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	f.store_buffer(bytes)
	f.close()


func _test_range_header() -> void:
	_check(WeatherFetcher.range_header(100, 199) == "Range: bytes=100-199", "Range header with an end")
	_check(WeatherFetcher.range_header(5, -1) == "Range: bytes=5-", "Range header open ended")


func _test_accepts() -> void:
	var ok := HTTPRequest.RESULT_SUCCESS
	_check(WeatherFetcher.accepts(ok, 200, false), "unranged: 200 accepted")
	_check(not WeatherFetcher.accepts(ok, 206, false), "unranged: 206 rejected")
	_check(WeatherFetcher.accepts(ok, 206, true), "ranged: 206 accepted")
	_check(not WeatherFetcher.accepts(ok, 200, true), "ranged: a 200 full-file answer is rejected")
	_check(not WeatherFetcher.accepts(ok, 404, false) and not WeatherFetcher.accepts(ok, 416, true), "404 / 416 rejected")
	_check(not WeatherFetcher.accepts(HTTPRequest.RESULT_TIMEOUT, 200, false), "a non-success result is rejected")
	_check(not WeatherFetcher.accepts(HTTPRequest.RESULT_BODY_SIZE_LIMIT_EXCEEDED, 206, true), "body size error rejected")


func _test_settle_success() -> void:
	var dest := _dir.path_join("good.bin")
	_write(dest + ".part", PackedByteArray([1, 2, 3]))
	_check(WeatherFetcher.settle_file(HTTPRequest.RESULT_SUCCESS, 206, true, dest), "ranged 206: settled")
	_check(FileAccess.file_exists(dest) and not FileAccess.file_exists(dest + ".part"), "renamed, no .part left")
	# An existing file is replaced.
	_write(dest + ".part", PackedByteArray([9, 9]))
	_check(WeatherFetcher.settle_file(HTTPRequest.RESULT_SUCCESS, 200, false, dest), "replaces an existing file")
	_check(FileAccess.get_file_as_bytes(dest) == PackedByteArray([9, 9]), "new content under the final name")


func _test_settle_failures() -> void:
	var cases := [
		["404", HTTPRequest.RESULT_SUCCESS, 404, false, PackedByteArray([1])],
		["200 on a range", HTTPRequest.RESULT_SUCCESS, 200, true, PackedByteArray([1])],
		["timeout", HTTPRequest.RESULT_TIMEOUT, 0, false, PackedByteArray([1])],
		["empty file", HTTPRequest.RESULT_SUCCESS, 200, false, PackedByteArray()],
	]
	for c: Array in cases:
		var dest := _dir.path_join("bad_%s.bin" % String(c[0]).replace(" ", "_"))
		_write(dest + ".part", c[4])
		_check(not WeatherFetcher.settle_file(c[1], c[2], c[3], dest), "%s: rejected" % c[0])
		_check(not FileAccess.file_exists(dest) and not FileAccess.file_exists(dest + ".part"),
				"%s: no final file, .part removed" % c[0])
	var missing := _dir.path_join("never.bin")
	_check(not WeatherFetcher.settle_file(HTTPRequest.RESULT_SUCCESS, 200, false, missing), "no .part at all: rejected")


func _test_timeout_constants() -> void:
	_check(WeatherFetcher.STALL_TIMEOUT_S >= 10.0, "a stall is a long silence, not a whole-request cap")


func _check(condition: bool, message: String) -> void:
	if condition:
		print("ok   ", message)
	else:
		print("FAIL ", message)
		_failures += 1
