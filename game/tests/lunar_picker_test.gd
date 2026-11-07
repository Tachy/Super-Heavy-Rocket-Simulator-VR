# Headless test of the interim lunar picker (lunar sites plan Task 12) with a fake site service, fake simulation and fake core.
# Run: godot --headless --path game --script res://tests/lunar_picker_test.gd
extends SceneTree

var _failures := 0


class FakeService:
	extends RefCounted
	signal progress(fraction: float)
	signal site_ready(lat_deg: float, lon_deg: float)
	signal failed(report: Dictionary)
	var prepared: Array = []
	var cancels := 0

	func prepare(lat_deg: float, lon_deg: float) -> void:
		prepared.append([lat_deg, lon_deg])

	func cancel() -> void:
		cancels += 1


class FakeSim:
	extends RefCounted
	var requests: Array = []
	var cancelled: Array = []
	var answer := {"done": true, "ok": true, "report": {}, "windows": []}

	func find_lunar_windows(mission_id: String, lat_deg: float, lon_deg: float, stay_s: float, start_tdb_s: float) -> int:
		requests.append({"mission": mission_id, "lat": lat_deg, "lon": lon_deg, "stay_s": stay_s, "start": start_tdb_s})
		return requests.size()

	func result(_id: int) -> Dictionary:
		return answer

	func cancel(id: int) -> bool:
		cancelled.append(id)
		return true


class FakeCore:
	extends RefCounted

	func tdb_s_to_utc(tdb_s: float) -> String:
		return Time.get_datetime_string_from_unix_time(int(tdb_s) + 946728000, false)  # J2000 = 2000-01-01T12:00 (leap seconds ignored)


class FakeClock:
	extends RefCounted
	var tdb_s := 7.0e8


class Starter:
	extends RefCounted
	var calls: Array = []
	var switched: Array = []
	var switch_ok := true

	func start(mission_id: String, options: Dictionary) -> bool:
		calls.append([mission_id, options])
		return true

	func switch(lat_deg: float, lon_deg: float) -> bool:
		switched.append([lat_deg, lon_deg])
		return switch_ok


func _initialize() -> void:
	var _events: I18nEvents = I18n.events
	I18n.settings_path = "user://lunar_picker_test_%d.cfg" % OS.get_process_id()
	I18n.set_language("en")
	_test_invalid_latitude()
	_test_refusal_listed()
	_test_start_passes_window()
	_test_presets_and_cancel()
	print("LUNAR PICKER TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _make(sim_answer := {}) -> Dictionary:
	var h := {"service": FakeService.new(), "sim": FakeSim.new(), "starter": Starter.new(), "picker": LunarPicker.new()}
	if not sim_answer.is_empty():
		h["sim"].answer = sim_answer
	(h["picker"] as LunarPicker).setup(FakeCore.new(), h["service"], h["sim"], FakeClock.new(),
			Callable(h["starter"], "start"), Callable(h["starter"], "switch"), LunarPicker.load_presets(Paths.config_dir()))
	(h["picker"] as LunarPicker).set_mission("hls_lunar_llo", 24.0)
	return h


func _test_invalid_latitude() -> void:
	var h := _make()
	var picker: LunarPicker = h["picker"]
	picker.set_fields("95", "10", "24")
	picker.search()
	_check((h["service"] as FakeService).prepared.is_empty() and (h["sim"] as FakeSim).requests.is_empty(),
			"invalid latitude 95 is rejected before any request")
	_check(picker.status_text() == I18n.text("ui.lunar_site.invalid_latitude"), "the latitude refusal is shown")
	picker.set_fields("10", "10", "0")
	picker.search()
	_check((h["service"] as FakeService).prepared.is_empty() and picker.status_text() == I18n.text("ui.lunar_site.invalid_stay"),
			"a stay of 0 h is rejected")
	picker.set_fields("abc", "10", "24")
	picker.search()
	_check((h["service"] as FakeService).prepared.is_empty(), "a non-number is rejected")


func _test_refusal_listed() -> void:
	var refusal := {"code": "lunar.no_window", "params": {}, "kinds": {}, "causes": [{"code": "lunar.nrho_site_unreachable",
			"params": {"latitude": 0.1, "budget_short": 3000.0}, "kinds": {"latitude": "angle", "budget_short": "mass"}, "causes": []}]}
	var h := _make({"done": true, "ok": false, "report": refusal, "windows": []})
	var picker: LunarPicker = h["picker"]
	picker.set_fields("26.13222", "3.63386", "24")
	picker.search()
	var service: FakeService = h["service"]
	_check(service.prepared == [[26.13222, 3.63386]], "the exact coordinates go to the service")
	service.site_ready.emit(26.13222, 3.63386)
	picker.poll()
	var expected := I18n.text("ui.lunar_site.refused", {"reason": I18n.report_text(refusal)})
	_check(picker.status_text() == expected and I18n.report_text(refusal) != "", "the window list shows the refusal text when no window exists")
	_check(picker.list_texts().is_empty(), "no dates listed with a refusal")
	var request: Dictionary = (h["sim"] as FakeSim).requests[0]
	_check(request["mission"] == "hls_lunar_llo" and is_equal_approx(request["stay_s"], 86400.0) and request["start"] == 7.0e8,
			"the search gets mission, stay in seconds and the clock")


func _test_start_passes_window() -> void:
	var windows := [
		{"mission_start_tdb_s": 7.0e8, "touchdown_tdb_s": 7.0e8 + 3.0e5, "liftoff_tdb_s": 7.0e8 + 4.0e5, "sun_elevation_deg": 12.3,
				"sun_azimuth_deg": 95.0, "daylight_after_touchdown_s": 9.0e5, "propellant_reserve_kg": 12000.0,
				"llo_inclination_deg": 28.0, "orbit": "llo"},
		{"mission_start_tdb_s": 7.1e8, "touchdown_tdb_s": 7.1e8 + 3.0e5, "liftoff_tdb_s": 7.1e8 + 4.0e5, "sun_elevation_deg": 8.0,
				"sun_azimuth_deg": 100.0, "daylight_after_touchdown_s": 8.0e5, "propellant_reserve_kg": 11000.0,
				"llo_inclination_deg": 28.0, "orbit": "llo"}]
	var h := _make({"done": true, "ok": true, "report": {}, "windows": windows})
	var picker: LunarPicker = h["picker"]
	picker.select_preset("apollo_15")
	picker.set_fields("26.13222", "3.63386", "24")
	picker.search()
	(h["service"] as FakeService).site_ready.emit(26.13222, 3.63386)
	picker.poll()
	_check(picker.list_texts().size() == 2, "two dates listed")
	_check(picker.list_texts()[0].contains("UTC") and picker.list_texts()[0].contains("12.3"), "the line shows date and Sun elevation")
	picker.select_window(1)
	picker.start_chosen()  # the fakes do not await: runs through
	var starter: Starter = h["starter"]
	_check(starter.switched == [[26.13222, 3.63386]], "the terrain is switched to the exact site first")
	var call: Array = starter.calls[0] if not starter.calls.is_empty() else []
	var lunar: Dictionary = call[1]["lunar"] if not call.is_empty() else {}
	_check(not call.is_empty() and call[0] == "hls_lunar_llo", "start passes the mission")
	_check(lunar.get("mission_start_tdb_s", 0.0) == 7.1e8 and lunar.get("touchdown_tdb_s", 0.0) == 7.1e8 + 3.0e5
			and lunar.get("lat_deg", 0.0) == 26.13222 and lunar.get("lon_deg", 0.0) == 3.63386 and lunar.get("stay_s", 0.0) == 86400.0,
			"start passes the chosen window")


func _test_presets_and_cancel() -> void:
	var h := _make({"done": false, "ok": false, "report": {}, "progress": 0.4, "windows": []})
	var picker: LunarPicker = h["picker"]
	picker.select_preset("apollo_17")
	picker.search()
	var service: FakeService = h["service"]
	_check(service.prepared.size() == 1 and is_equal_approx(service.prepared[0][0], 20.19089), "a preset fills the coordinates")
	picker.cancel()
	_check(service.cancels == 1, "changing away cancels the preparation")
	picker.search()
	service.site_ready.emit(20.19089, 30.77168)
	_check(picker.state == LunarPicker.State.SEARCHING, "searching after the site is ready")
	picker.set_mission("")
	_check((h["sim"] as FakeSim).cancelled == [1], "leaving the mission cancels the search")
	_check(not picker.visible, "a non-lunar mission hides the picker")


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
