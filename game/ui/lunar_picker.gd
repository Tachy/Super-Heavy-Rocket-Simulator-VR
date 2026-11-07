class_name LunarPicker
extends VBoxContainer
## Interim picker for a lunar landing mission (the graphical Moon view of M7 replaces it): a site (preset of
## config/lunar_sites.toml or free coordinates), the stay on the surface, then "search dates": the terrain tiles of the
## site (LunarSiteService.prepare), the landing window search (FlightSim.find_lunar_windows) and the list of dates (or the
## refusal text). Choosing a date prepares the terrain (World.switch_lunar_site) and starts the mission with the window.
## Every text is a table key or a report (I18n); both are kept as data and drawn at draw time (language switch).

signal start_finished(ok: bool)  ## the picker's own start finished (not accepted / terrain failed); a mission outcome comes from the flight

enum State { IDLE, PREPARING, SEARCHING, LISTED, STARTING }

const CUSTOM := "custom"  ## the preset entry "Coordinates"
const MAX_STAY_H := 24.0 * 365.0  ## sanity bound of the stay field

var state := State.IDLE
var windows: Array = []  ## the listed windows (FlightSim.result(id)["windows"])

var _core: Object
var _service: Object  ## LunarSiteService (setup / prepare / cancel, signals progress, site_ready, failed)
var _sim: Object  ## FlightSim (find_lunar_windows, result, cancel)
var _clock: Object  ## has tdb_s
var _starter: Callable  ## (mission_id, options) -> bool: World.start_mission
var _switcher: Callable  ## (lat_deg, lon_deg) -> bool, may await: World.switch_lunar_site and its outcome
var _presets: Array = []  ## [{id, latitude_deg, longitude_deg}]
var _mission_id := ""
var _request := {}  ## the running request {lat_deg, lon_deg, stay_s}
var _request_kept := {}  ## the request of the listed windows (site and stay of the start)
var _search_id := 0
var _chosen := -1
var _start_pending := false  ## STARTING before the mission was handed over: a mission change or hide aborts it
var _start_serial := 0

var _preset_box := OptionButton.new()
var _lat_edit := LineEdit.new()
var _lon_edit := LineEdit.new()
var _stay_edit := LineEdit.new()
var _search_button := Button.new()
var _progress := ProgressBar.new()
var _status := Label.new()
var _list := ItemList.new()
var _start_button := Button.new()
var _status_key := ""
var _status_params := {}
var _status_report := {}  ## a refusal report (takes precedence over the key)


## Sites of config/lunar_sites.toml: [{id, latitude_deg, longitude_deg}].
static func load_presets(config_dir: String) -> Array:
	var result: Array = []
	for site: Dictionary in TomlLite.load_file(config_dir.path_join("lunar_sites.toml")).get("site", []):
		result.append({"id": site["id"], "latitude_deg": float(site["latitude_deg"]), "longitude_deg": float(site["longitude_deg"])})
	return result


func setup(core: Object, service: Object, sim: Object, clock: Object, starter: Callable, switcher: Callable, presets: Array) -> void:
	_core = core
	_service = service
	_sim = sim
	_clock = clock
	_starter = starter
	_switcher = switcher
	_presets = presets
	_build()
	_service.progress.connect(_on_prepare_progress)
	_service.site_ready.connect(_on_site_ready)
	_service.failed.connect(_on_prepare_failed)
	visibility_changed.connect(func() -> void:
		if not is_visible_in_tree():
			cancel())
	I18n.events.language_changed.connect(func(_code: String) -> void: _refresh_texts())
	visible = false


## The selected mission: a lunar landing shows the picker (`stay_hours` > 0 its default stay), anything else hides it.
## Any running preparation or search is cancelled when the mission changes.
func set_mission(mission_id: String, stay_hours := 0.0) -> void:
	if mission_id == _mission_id:
		return
	cancel()
	_mission_id = mission_id
	visible = mission_id != ""
	if mission_id != "":
		_stay_edit.text = I18n.number(stay_hours, 1) if stay_hours > 0.0 else ""
		_clear_list()
		_set_status("", {})


## Stops the site preparation and the window search (site, mission or wizard changed).
func cancel() -> void:
	if state == State.PREPARING and _service != null:
		_service.cancel()
	if state == State.SEARCHING and _search_id > 0 and _sim != null:
		_sim.cancel(_search_id)
	_search_id = 0
	_request = {}
	if state == State.STARTING and _start_pending:
		_start_serial += 1  # the awaiting start_chosen ends without starting anything
		_start_pending = false
		state = State.IDLE
	if state != State.STARTING:
		state = State.IDLE
		_progress.visible = false
		_update_buttons()


func _exit_tree() -> void:
	cancel()


## The numbers of the fields as {ok, lat_deg, lon_deg, stay_s} or {ok = false, key} (a table key of the refusal).
func read_fields() -> Dictionary:
	var lat_text := _lat_edit.text.strip_edges().replace(",", ".")
	var lon_text := _lon_edit.text.strip_edges().replace(",", ".")
	var stay_text := _stay_edit.text.strip_edges().replace(",", ".")
	if not lat_text.is_valid_float() or absf(lat_text.to_float()) > 90.0:
		return {"ok": false, "key": "ui.lunar_site.invalid_latitude"}
	if not lon_text.is_valid_float() or lon_text.to_float() < -180.0 or lon_text.to_float() > 360.0:
		return {"ok": false, "key": "ui.lunar_site.invalid_longitude"}
	if not stay_text.is_valid_float() or stay_text.to_float() < 0.0 or stay_text.to_float() > MAX_STAY_H:
		return {"ok": false, "key": "ui.lunar_site.invalid_stay"}
	return {"ok": true, "lat_deg": lat_text.to_float(), "lon_deg": lon_text.to_float(), "stay_s": stay_text.to_float() * 3600.0}


## "Search dates": validates the fields (nothing is requested when one is invalid), then the site's tiles, then the search.
func search() -> void:
	cancel()
	_clear_list()
	var fields := read_fields()
	if not fields["ok"]:
		_set_status(fields["key"], {})
		return
	_request = {"lat_deg": fields["lat_deg"], "lon_deg": fields["lon_deg"], "stay_s": fields["stay_s"]}
	state = State.PREPARING
	_progress.value = 0.0
	_progress.visible = true
	_set_status("ui.lunar_site.preparing_tiles", {})
	_update_buttons()
	_service.prepare(_request["lat_deg"], _request["lon_deg"])


func _on_prepare_progress(fraction: float) -> void:
	if state == State.PREPARING:
		_progress.value = fraction * 100.0


func _on_prepare_failed(report: Dictionary) -> void:
	if state != State.PREPARING:
		return
	_finish_search({"report": report})


func _on_site_ready(lat_deg: float, lon_deg: float) -> void:
	if state != State.PREPARING or not is_equal_approx(lat_deg, _request["lat_deg"]) \
			or not is_equal_approx(lon_deg, _request["lon_deg"]):
		return  # a superseded site
	state = State.SEARCHING
	_progress.value = 0.0
	_set_status("ui.lunar_site.searching", {})
	_search_id = int(_sim.find_lunar_windows(_mission_id, lat_deg, lon_deg, _request["stay_s"], _clock.tdb_s))
	if _search_id <= 0:
		_finish_search({"key": "ui.lunar_site.search_not_accepted"})
	_update_buttons()


## Polls the running search (called every frame by _process; tests call it directly).
func poll() -> void:
	if state != State.SEARCHING or _search_id <= 0:
		return
	var result: Dictionary = _sim.result(_search_id)
	if not result.get("done", false):
		_progress.value = float(result.get("progress", 0.0)) * 100.0
		return
	_search_id = 0
	if result.get("ok", false):
		windows = result.get("windows", [])
		if windows.is_empty():
			_finish_search({"key": "ui.lunar_site.no_windows"})
		else:
			_finish_search({"windows": windows})
	else:
		var report: Dictionary = result.get("report", {})
		_finish_search({"report": report if not report.is_empty() else {"code": "command.internal_error"}})


## Waits for the outcome of a terrain switch: `start` begins it, `ok_signal` / `fail_signal` of `source` end it. A superseded
## switch emits nothing, so the wait also ends (false) when `serial_of` (the source's switch counter) changes.
static func await_switch(tree: SceneTree, source: Object, ok_signal: StringName, fail_signal: StringName,
		serial_of: Callable, start: Callable) -> bool:
	var outcome := {"done": false, "ok": false}
	var on_ok := func(_lat: float, _lon: float) -> void:
		outcome["done"] = true
		outcome["ok"] = true
	var on_failed := func(_report: Dictionary) -> void: outcome["done"] = true
	source.connect(ok_signal, on_ok, CONNECT_ONE_SHOT)
	source.connect(fail_signal, on_failed, CONNECT_ONE_SHOT)
	start.call()
	var serial: int = serial_of.call()
	while not outcome["done"] and int(serial_of.call()) == serial:
		await tree.process_frame
	if source.is_connected(ok_signal, on_ok):
		source.disconnect(ok_signal, on_ok)
	if source.is_connected(fail_signal, on_failed):
		source.disconnect(fail_signal, on_failed)
	return outcome["ok"]


func _process(_delta: float) -> void:
	poll()


## The end of a preparation / search: the windows, a refusal report or a table key as the line.
func _finish_search(outcome: Dictionary) -> void:
	state = State.IDLE
	_progress.visible = false
	_request_clear()
	if outcome.has("windows"):
		state = State.LISTED
		_fill_list()
		_set_status("ui.lunar_site.choose", {})
	elif outcome.has("report"):
		_set_status("", {}, outcome["report"])
	else:
		_set_status(outcome["key"], {})
	_update_buttons()


func _request_clear() -> void:
	_request_kept = _request
	_request = {}


## "Start": prepares the terrain of the site (World.switch_lunar_site, ~5 s on the main thread: one frame of the
## "preparing terrain" state is drawn first), then starts the mission with the chosen window.
func start_chosen() -> void:
	var index := _list.get_selected_items()[0] if not _list.get_selected_items().is_empty() else -1
	if state != State.LISTED or index < 0 or index >= windows.size():
		return
	var window: Dictionary = windows[index]
	var site := _request_kept
	var mission_id := _mission_id  # the snapshot: mission, site and window are started together or not at all
	_start_serial += 1
	var serial := _start_serial
	_start_pending = true
	state = State.STARTING
	_set_status("ui.lunar_site.preparing_terrain", {})
	_update_buttons()
	if is_inside_tree():
		await get_tree().process_frame  # the frame with the status text, before the main thread blocks
	if serial != _start_serial:
		return
	var switched: bool = await _switcher.call(site["lat_deg"], site["lon_deg"])
	if serial != _start_serial:
		return  # the mission changed or the picker was hidden meanwhile: nothing is started
	_start_pending = false
	if not switched:
		state = State.LISTED
		_set_status("ui.lunar_site.terrain_failed", {})
		_update_buttons()
		start_finished.emit(false)
		return
	var options := {"lunar": {"lat_deg": site["lat_deg"], "lon_deg": site["lon_deg"], "stay_s": site["stay_s"],
			"mission_start_tdb_s": window["mission_start_tdb_s"], "touchdown_tdb_s": window["touchdown_tdb_s"]}}
	var accepted: bool = _starter.call(mission_id, options)
	if not accepted:
		state = State.LISTED
		_set_status("ui.lunar_site.start_not_accepted", {})
		_update_buttons()
		start_finished.emit(false)
		return
	_set_status("ui.time_panel.start_planning", {})


## The mission's start outcome (FlightController.mission_started): a refusal returns to the list.
func on_mission_started(ok: bool, report: Dictionary) -> void:
	if state != State.STARTING:
		return
	state = State.LISTED
	if ok:
		_set_status("", {})
	else:
		_set_status("", {}, report if not report.is_empty() else {"code": "ok"})
	_update_buttons()
	start_finished.emit(ok)


func _build() -> void:
	for entry: Dictionary in _presets:
		_preset_box.add_item(I18n.text("lunar_site.%s.name" % entry["id"]))
	_preset_box.add_item(I18n.text("ui.lunar_site.coordinates"))
	_preset_box.item_selected.connect(_on_preset_selected)
	add_child(_preset_box)
	var grid := GridContainer.new()
	grid.columns = 2
	for entry: Array in [["ui.lunar_site.latitude", _lat_edit], ["ui.lunar_site.longitude", _lon_edit], ["ui.lunar_site.stay", _stay_edit]]:
		var label := Label.new()
		I18n.bind_text(label, &"text", entry[0])
		grid.add_child(label)
		var edit: LineEdit = entry[1]
		edit.custom_minimum_size.x = 120
		grid.add_child(edit)
	add_child(grid)
	I18n.bind_text(_search_button, &"text", "ui.lunar_site.search")
	_search_button.pressed.connect(search)
	add_child(_search_button)
	_progress.show_percentage = false
	_progress.visible = false
	add_child(_progress)
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_status.custom_minimum_size.x = 460
	add_child(_status)
	_list.custom_minimum_size = Vector2(460, 190)  # five rows
	_list.item_selected.connect(func(_index: int) -> void: _update_buttons())
	add_child(_list)
	I18n.bind_text(_start_button, &"text", "ui.lunar_site.start")
	_start_button.pressed.connect(start_chosen)
	add_child(_start_button)
	_on_preset_selected(0)
	_update_buttons()


func _on_preset_selected(index: int) -> void:
	var custom := index >= _presets.size()
	_lat_edit.editable = custom
	_lon_edit.editable = custom
	if not custom:
		_lat_edit.text = I18n.number(_presets[index]["latitude_deg"], 5)
		_lon_edit.text = I18n.number(_presets[index]["longitude_deg"], 5)
	cancel()
	_clear_list()
	_set_status("", {})


## The selected preset's id, or CUSTOM.
func selected_preset() -> String:
	var index := _preset_box.selected
	return _presets[index]["id"] if index >= 0 and index < _presets.size() else CUSTOM


func select_preset(id: String) -> void:
	for i in _presets.size():
		if _presets[i]["id"] == id:
			_preset_box.select(i)
			_on_preset_selected(i)
			return
	_preset_box.select(_presets.size())
	_on_preset_selected(_presets.size())


func set_fields(lat_text: String, lon_text: String, stay_text: String) -> void:
	_lat_edit.text = lat_text
	_lon_edit.text = lon_text
	_stay_edit.text = stay_text


func select_window(index: int) -> void:
	_list.select(index)
	_update_buttons()


func status_text() -> String:
	return _status.text


func list_texts() -> PackedStringArray:
	var result := PackedStringArray()
	for i in _list.item_count:
		result.append(_list.get_item_text(i))
	return result


func _clear_list() -> void:
	windows = []
	_request_kept = {}
	_list.clear()
	_update_buttons()


func _fill_list() -> void:
	_list.clear()
	for window: Dictionary in windows:
		_list.add_item(window_text(window))


## One list line of a window: UTC date of the touchdown, Sun elevation / azimuth, daylight left, propellant reserve.
func window_text(window: Dictionary) -> String:
	var touchdown_unix_s := _unix_s(float(window["touchdown_tdb_s"]))
	return I18n.text("ui.lunar_site.window", {
		"date": I18n.format_value(touchdown_unix_s, "utc", "utc"),
		"sun": I18n.format_value(deg_to_rad(float(window["sun_elevation_deg"])), "angle", "deg1"),
		"azimuth": I18n.format_value(deg_to_rad(float(window["sun_azimuth_deg"])), "angle", "deg0"),
		"daylight": I18n.format_value(float(window["daylight_after_touchdown_s"]), "time", "h0"),
		"reserve": I18n.format_value(float(window["propellant_reserve_kg"]), "mass", "t0")})


## UTC Unix seconds of a TDB epoch through the core's UTC string.
func _unix_s(tdb_s: float) -> float:
	var utc: String = _core.tdb_s_to_utc(tdb_s)
	return Time.get_unix_time_from_datetime_string(utc.substr(0, 19))


func _update_buttons() -> void:
	var busy := state == State.PREPARING or state == State.SEARCHING or state == State.STARTING
	_search_button.disabled = busy
	_start_button.disabled = state != State.LISTED or _list.get_selected_items().is_empty()


func _set_status(key: String, params: Dictionary, report := {}) -> void:
	_status_key = key
	_status_params = params
	_status_report = report
	_refresh_texts()


func _refresh_texts() -> void:
	if not _status_report.is_empty():
		var reason := I18n.report_text(_status_report)
		_status.text = I18n.text("ui.lunar_site.refused", {"reason": reason}) if reason != "" \
				else I18n.text("ui.lunar_site.refused_unknown")
	else:
		_status.text = I18n.text(_status_key, _status_params) if _status_key != "" else ""
	for i in _presets.size():
		_preset_box.set_item_text(i, I18n.text("lunar_site.%s.name" % _presets[i]["id"]))
	if not _presets.is_empty() or _preset_box.item_count > 0:
		_preset_box.set_item_text(_presets.size(), I18n.text("ui.lunar_site.coordinates"))
	if _list.item_count == windows.size():
		for i in windows.size():
			_list.set_item_text(i, window_text(windows[i]))
