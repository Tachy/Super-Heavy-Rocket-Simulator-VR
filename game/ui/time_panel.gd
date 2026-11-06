class_name TimePanel
extends PanelContainer
## Time and view control: UTC display/input, time warp, camera focus, eclipse readout.
## Keys: Space = pause/resume, +/- = warp faster/slower.

## [table key of the name, focus]
const FOCUS_OPTIONS := [
	["ui.focus.earth", World.FOCUS_EARTH],
	["ui.focus.moon", World.FOCUS_MOON],
	["ui.focus.earth_moon", World.FOCUS_EARTH_MOON],
	["ui.focus.vehicle", World.FOCUS_VEHICLE],
	["ui.focus.free", World.FOCUS_FREE],  ## HOTAS 6 DOF (config/input_hotas.toml [free_camera])
]
const VEHICLE_FOCUS_INDEX := 3  ## FOCUS_OPTIONS entry "ui.focus.vehicle"
## [table key of the name, UTC ("" = now)]
const PRESETS := [
	["ui.preset.now", ""],
	["ui.preset.solar_eclipse", "2024-04-08T18:42:00"],
	["ui.preset.full_moon", "2024-04-23T23:49:00"],
	["ui.preset.lunar_eclipse", "2025-03-14T06:58:00"],
]

## The frame-rate line in the info text (render_check --repeat_identical hides it: it changes from frame to frame).
static var show_frame_rate := true

var world: World

var _utc_label := Label.new()
var _info_label := Label.new()
var _utc_input := LineEdit.new()
var _focus := OptionButton.new()
var _presets := OptionButton.new()
var _catalogue: MissionCatalogue
var _places := OptionButton.new()
var _vehicles := OptionButton.new()
var _missions := OptionButton.new()
var _place_list: Array[Dictionary] = []
var _vehicle_list: Array[Dictionary] = []
var _mission_list: Array[Dictionary] = []
var _start_button: Button
var _start_status := Label.new()
var _start_key := ""  ## table key of the start status text ("" none); a refusal (_start_refusal) takes precedence
var _start_refusal: Dictionary = {}  ## report of a refused start; resolved at draw time so a language switch re-renders it
var _warp_before_pause := 1.0
var _axes_button: Button
var _wizard := FreeCameraWizard.new()
var _rate := UiRate.new()  ## the readouts at 15 Hz (UiRate)


func _ready() -> void:
	position = Vector2(12, 12)
	var box := VBoxContainer.new()
	add_child(box)

	_utc_label.add_theme_font_size_override("font_size", 22)
	box.add_child(_utc_label)
	box.add_child(_info_label)

	var time_row := HBoxContainer.new()
	_utc_input.placeholder_text = "YYYY-MM-DDTHH:MM:SS (UTC)"
	_utc_input.custom_minimum_size.x = 230
	_utc_input.text_submitted.connect(func(_text: String) -> void: _apply_utc_input())
	time_row.add_child(_utc_input)
	time_row.add_child(_button("ui.panel.set", _apply_utc_input))
	box.add_child(time_row)

	var warp_row := HBoxContainer.new()
	warp_row.add_child(_button("", func() -> void: world.clock.step_warp(-1), "<<"))
	warp_row.add_child(_button("ui.panel.pause", _toggle_pause))
	warp_row.add_child(_button("", func() -> void: world.clock.step_warp(1), ">>"))
	warp_row.add_child(_button("ui.panel.reverse", func() -> void: world.clock.warp = -world.clock.warp))
	box.add_child(warp_row)

	_fill_focus()
	_focus.item_selected.connect(_on_focus_selected)
	box.add_child(_focus)
	_axes_button = _button("ui.panel.axes_setup", func() -> void: _wizard.open())
	_axes_button.visible = false
	box.add_child(_axes_button)
	_wizard.control = world.free_control
	box.add_child(_wizard)

	_fill_presets()
	_presets.item_selected.connect(_on_preset_selected)
	box.add_child(_presets)

	var weather_box := CheckBox.new()
	I18n.bind_text(weather_box, &"text", "ui.panel.weather")
	weather_box.button_pressed = world.weather.enabled
	weather_box.toggled.connect(func(on: bool) -> void:
		world.weather.enabled = on
		WeatherService.save_enabled_setting(on))
	box.add_child(weather_box)
	# Fidelity setting moon_near_shadows (MoonNearShadow): the Sun's shadow map near the lunar ground.
	var moon_shadow_box := CheckBox.new()
	I18n.bind_text(moon_shadow_box, &"text", "ui.panel.moon_shadow")
	I18n.bind_text(moon_shadow_box, &"tooltip_text", "ui.panel.moon_shadow_tip")
	moon_shadow_box.button_pressed = MoonNearShadow.enabled
	moon_shadow_box.toggled.connect(func(on: bool) -> void:
		MoonNearShadow.enabled = on
		MoonNearShadow.save_enabled_setting(on))
	box.add_child(moon_shadow_box)
	# Fidelity setting earth_near_shadows (EarthNearShadow, Task 16): the Sun's shadow map near the Earth's ground.
	var earth_shadow_box := CheckBox.new()
	I18n.bind_text(earth_shadow_box, &"text", "ui.panel.earth_shadow")
	I18n.bind_text(earth_shadow_box, &"tooltip_text", "ui.panel.earth_shadow_tip")
	earth_shadow_box.button_pressed = EarthNearShadow.enabled
	earth_shadow_box.toggled.connect(func(on: bool) -> void:
		EarthNearShadow.enabled = on
		EarthNearShadow.save_enabled_setting(on))
	box.add_child(earth_shadow_box)
	box.add_child(_language_row())

	box.add_child(HSeparator.new())
	var launch_label := Label.new()
	I18n.bind_text(launch_label, &"text", "ui.panel.prepare_launch")
	box.add_child(launch_label)
	_catalogue = MissionCatalogue.load_from(Paths.config_dir())
	_place_list = _catalogue.places()
	for place in _place_list:
		_places.add_item(place["name"])
	_places.item_selected.connect(func(_index: int) -> void: _refill_vehicles())
	_vehicles.item_selected.connect(func(_index: int) -> void: _refill_missions())
	for option: OptionButton in [_places, _vehicles, _missions]:
		box.add_child(option)
	_start_button = _button("ui.panel.start", _on_start_pressed)
	box.add_child(_start_button)
	_start_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_start_status.custom_minimum_size.x = 300
	box.add_child(_start_status)
	world.flight.mission_started.connect(_on_start_finished)
	world.flight.sim.command_finished.connect(_on_command_finished)
	I18n.events.language_changed.connect(_on_language_changed)
	_refill_vehicles()


func _process(_delta: float) -> void:
	FrameProfiler.begin(&"ui_time_panel")
	_process_unprofiled(_delta)
	FrameProfiler.end(&"ui_time_panel")


func _process_unprofiled(_delta: float) -> void:
	if not world.core.is_open():
		_utc_label.text = I18n.text("ui.panel.spice_missing")
		return
	_axes_button.visible = world.focus == World.FOCUS_FREE
	if world.focus != World.FOCUS_FREE:
		_wizard.cancel()
	elif world.free_wizard_requested:  # first use without a saved mapping
		world.free_wizard_requested = false
		_wizard.open()
	var t := world.clock.tdb_s
	if not _rate.due(_delta, t, world.clock.warp):
		return
	_utc_label.text = "%s UTC" % world.core.tdb_s_to_utc(t).substr(0, 19).replace("T", "  ")
	var lines := PackedStringArray()
	lines.append(I18n.text("ui.panel.warp", {"warp": _format_warp(world.clock.warp)}))
	lines.append(I18n.text("ui.panel.moon_distance", {"distance": world.moon_gcrf_m.length()}))
	if world.focus == World.FOCUS_FREE:
		var p := world.camera.position_gcrf_m
		var on_moon := world.free_pose.frame == FreeCameraPose.FRAME_MOON
		var height := camera_height_line((p - world.moon_gcrf_m).length() - World.MOON_RADIUS_M if on_moon
				else world.core.sea_level_height_m(p, t), on_moon)
		if height != "":
			lines.append(height)
	for site in world.sites:
		var fraction: float = world.core.solar_eclipse_fraction(site["latitude_deg"], site["longitude_deg"], site["height_m"], t)
		if fraction > 0.0:
			lines.append(I18n.text("ui.panel.sun_coverage", {"site": I18n.text("site.%s.name" % site["id"]), "fraction": fraction}))
	if world.flight.sim.is_active():
		var sea := sea_state_line(world.flight.sim.landing_zone())
		if sea != "":
			lines.append(sea)
	if world.atmosphere.status_key != "":
		lines.append(world.atmosphere.status_text())
	lines.append(world.weather.status_text())
	lines.append(world.clouds.status_line())  # the cloud march's live toggle (F8, CloudLayer)
	var frame_rate := frame_rate_line(Engine.get_frames_per_second()) if show_frame_rate else ""
	if frame_rate != "":
		lines.append(frame_rate)
	_info_label.text = "\n".join(lines)


## The frame rate (Engine.get_frames_per_second: the frames of the last second) and its frame time, else "" (first
## second). In VR a rate pinned at half the headset's (45 at 90 Hz) means the runtime is reprojecting.
static func frame_rate_line(fps: float) -> String:
	if fps <= 0.0:
		return ""
	return I18n.text("ui.panel.frame_rate", {"fps": roundi(fps), "ms": 1000.0 / fps})


## The free camera's height (user 2026-10-30): above mean sea level (EGM2008 geoid), or - in the Moon's frame
## (FreeCameraPose.FRAME_MOON) - above the mean lunar sphere; metres below 10 km. "" for a NAN height (no geoid).
static func camera_height_line(height_m: float, on_moon: bool) -> String:
	if is_nan(height_m):
		return ""
	var value := I18n.format_value(height_m, "length", "m0" if absf(height_m) < 1.0e4 else "km1")
	return I18n.text("ui.panel.camera_height", {"height": value,
			"reference": I18n.text("ui.panel.height_moon" if on_moon else "ui.panel.height_msl")})


## The sea state and deck motion while a drone ship is the landing zone (FlightSim.landing_zone), else "".
static func sea_state_line(zone: Dictionary) -> String:
	if not zone.get("has_deck", false):
		return ""
	var sea: Dictionary = zone.get("sea_state", {})
	if sea.is_empty():
		return I18n.text("ui.panel.sea_calm")
	return I18n.text("ui.panel.sea_state", {"hs": sea["hs_m"], "tp": sea["tp_s"],
			"heave": _signed(zone.get("deck_heave_m", 0.0), 2), "roll": _signed(zone.get("deck_roll_deg", 0.0), 1),
			"pitch": _signed(zone.get("deck_pitch_deg", 0.0), 1)})


## A number with an explicit sign ("+0,25"), as %+.nf.
static func _signed(value: float, decimals: int) -> String:
	var text := I18n.number(value, decimals)
	return text if text.begins_with("-") else "+" + text


func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed:
		return
	match key.keycode:
		KEY_SPACE:
			_toggle_pause()
		KEY_PLUS, KEY_KP_ADD:
			world.clock.step_warp(1)
		KEY_MINUS, KEY_KP_SUBTRACT:
			world.clock.step_warp(-1)


func _apply_utc_input() -> void:
	var text := _utc_input.text.strip_edges()
	if text.is_empty():
		return
	world.clock.set_time(world.core.utc_to_tdb_s(text))


func _toggle_pause() -> void:
	if world.clock.warp == 0.0:
		world.clock.warp = _warp_before_pause
	else:
		_warp_before_pause = world.clock.warp
		world.clock.warp = 0.0


func _on_focus_selected(index: int) -> void:
	if index < FOCUS_OPTIONS.size():
		world.set_focus(FOCUS_OPTIONS[index][1])
	else:
		world.set_focus(world.sites[index - FOCUS_OPTIONS.size()]["id"])


func _on_preset_selected(index: int) -> void:
	var utc: String = PRESETS[index][1]
	if utc.is_empty():
		utc = Time.get_datetime_string_from_system(true)
	world.clock.set_time(world.core.utc_to_tdb_s(utc))


## The focus drop-down: the fixed entries, then the sites (a site with a [site.viewpoint] is seen from there,
## World._viewpoint_site, else from its pad). Keeps the selection.
func _fill_focus() -> void:
	var selected := _focus.selected
	_focus.clear()
	for option in FOCUS_OPTIONS:
		_focus.add_item(I18n.text(option[0]))
	for site in world.sites:
		_focus.add_item(I18n.text("ui.focus.site_viewpoint" if site.has("viewpoint") else "ui.focus.site_ground",
				{"site": I18n.text("site.%s.name" % site["id"])}))
	if selected >= 0:
		_focus.select(selected)


func _fill_presets() -> void:
	var selected := _presets.selected
	_presets.clear()
	for preset in PRESETS:
		_presets.add_item(I18n.text(preset[0]))
	if selected >= 0:
		_presets.select(selected)


## Refills the vehicle box for the selected place (keeping the selected vehicle when it is offered there), then the
## mission box.
func _refill_vehicles() -> void:
	var previous := _selected_id(_vehicle_list, _vehicles)
	_vehicle_list = _catalogue.vehicles_at(_selected_id(_place_list, _places))
	_fill(_vehicles, _vehicle_list, previous)
	_refill_missions()


func _refill_missions() -> void:
	var previous := _selected_id(_mission_list, _missions)
	_mission_list = _catalogue.missions_for(_selected_id(_place_list, _places), _selected_id(_vehicle_list, _vehicles))
	_fill(_missions, _mission_list, previous)


static func _fill(option: OptionButton, list: Array[Dictionary], keep_id: String) -> void:
	option.clear()
	for entry in list:
		option.add_item(entry["name"])
	if not list.is_empty():
		option.select(maxi(MissionCatalogue.index_of(list, keep_id), 0))


static func _selected_id(list: Array[Dictionary], option: OptionButton) -> String:
	var index := option.selected
	return list[index]["id"] if index >= 0 and index < list.size() else ""


## Free flight: the vehicle on the selected pad (FlightController.launch); else the selected mission. The button stays
## disabled until the start's outcome (a mission or a planned launch takes up to ~45 s).
func _on_start_pressed() -> void:
	if _mission_list.is_empty() or _missions.selected < 0:
		return
	var mission_id := _selected_id(_mission_list, _missions)
	# disabled before posting: a fast outcome may already arrive inside the call (_on_start_finished)
	_start_button.disabled = true
	_set_start_status("ui.time_panel.start_planning", {})
	var started := false
	if mission_id == MissionCatalogue.FREE_FLIGHT:
		var site: Dictionary = _place_list[_places.selected]["site"]
		var vehicle: Dictionary = _vehicle_list[_vehicles.selected]
		started = world.launch_vehicle(vehicle["id"], site, FlightController.LAUNCH_COUNTDOWN_S, vehicle.get("name_key", ""))
		if started:
			_focus.select(VEHICLE_FOCUS_INDEX)  # world.launch_vehicle follows the vehicle at once
	else:
		started = world.start_mission(mission_id, not (_place_list[_places.selected]["site"] as Dictionary).is_empty())
	if not started:
		_start_button.disabled = false
		_set_start_status("ui.time_panel.start_not_accepted", {})


func _on_command_finished(_id: int, command: String, ok: bool, report: Dictionary) -> void:
	if command == "launch_from_site" and _start_button.disabled:
		_on_start_finished(ok, report)


func _on_start_finished(ok: bool, report: Dictionary) -> void:
	_start_button.disabled = false
	if ok:
		_set_start_status("", {})
		_focus.select(VEHICLE_FOCUS_INDEX)  # World follows the started vehicle
	else:
		_set_start_status("", report if report.get("code", "ok") != "ok" else {"code": "ok"}, true)


## The start status line: a table key, or (refused) a report. Both are kept as data and drawn by _render_start_status.
func _set_start_status(key: String, report: Dictionary, refused := false) -> void:
	_start_key = key
	_start_refusal = report if refused else {}
	_render_start_status()


func _render_start_status() -> void:
	if not _start_refusal.is_empty():
		var reason := I18n.report_text(_start_refusal)
		_start_status.text = I18n.text("ui.time_panel.start_refused", {"reason": reason}) if reason != "" \
				else I18n.text("ui.time_panel.start_refused_unknown")
	else:
		_start_status.text = I18n.text(_start_key) if _start_key != "" else ""


func _on_language_changed(_language: String) -> void:
	_fill_focus()
	_fill_presets()
	_refill_places()
	_render_start_status()


## The place / vehicle / mission drop-downs in the new language, keeping the selected entries.
func _refill_places() -> void:
	var place := _selected_id(_place_list, _places)
	_place_list = _catalogue.places()
	_places.clear()
	for entry in _place_list:
		_places.add_item(entry["name"])
	if not _place_list.is_empty():
		_places.select(maxi(MissionCatalogue.index_of(_place_list, place), 0))
	var mission := _selected_id(_mission_list, _missions)
	_refill_vehicles()
	var index := MissionCatalogue.index_of(_mission_list, mission)
	if index >= 0:
		_missions.select(index)


## The language row: label and a drop-down with the languages (saved in user://settings.cfg [ui] language).
func _language_row() -> HBoxContainer:
	var row := HBoxContainer.new()
	var label := Label.new()
	I18n.bind_text(label, "text", "ui.language.label")
	row.add_child(label)
	var option := OptionButton.new()
	for code in I18n.LANGUAGES:
		option.add_item(I18n.text("ui.language." + code))
	option.select(I18n.LANGUAGES.find(I18n.language()))
	option.item_selected.connect(func(index: int) -> void: I18n.set_language(I18n.LANGUAGES[index]))
	row.add_child(option)
	return row


## A button labelled with the text of `key` (follows the language), or with `symbol` (no words) when the key is "".
func _button(key: String, action: Callable, symbol := "") -> Button:
	var button := Button.new()
	if key != "":
		I18n.bind_text(button, &"text", key)
	else:
		button.text = symbol  # i18n-ok: an arrow symbol, no words
	button.pressed.connect(action)
	return button


static func _format_warp(warp: float) -> String:
	if absf(warp) >= 1000.0:
		return "%dk" % int(warp / 1000.0)
	return str(int(warp)) if warp == floorf(warp) else str(warp)
