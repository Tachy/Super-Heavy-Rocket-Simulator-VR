class_name TimePanel
extends PanelContainer
## Time and view control: UTC display/input, time warp, camera focus, eclipse readout.
## Keys: Space = pause/resume, +/- = warp faster/slower.

const FOCUS_OPTIONS := [
	["Erde", World.FOCUS_EARTH],
	["Mond", World.FOCUS_MOON],
	["Erde-Mond-System", World.FOCUS_EARTH_MOON],
	["Fahrzeug", World.FOCUS_VEHICLE],
]
const VEHICLE_FOCUS_INDEX := 3  ## FOCUS_OPTIONS entry "Fahrzeug"
const PRESETS := [
	["Jetzt", ""],
	["Totale Sonnenfinsternis 08.04.2024 (Dallas)", "2024-04-08T18:42:00"],
	["Vollmond 23.04.2024", "2024-04-23T23:49:00"],
	["Totale Mondfinsternis 14.03.2025", "2025-03-14T06:58:00"],
]

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
var _warp_before_pause := 1.0


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
	time_row.add_child(_button("Setzen", _apply_utc_input))
	box.add_child(time_row)

	var warp_row := HBoxContainer.new()
	warp_row.add_child(_button("<<", func() -> void: world.clock.step_warp(-1)))
	warp_row.add_child(_button("Pause", _toggle_pause))
	warp_row.add_child(_button(">>", func() -> void: world.clock.step_warp(1)))
	warp_row.add_child(_button("Rückwärts", func() -> void: world.clock.warp = -world.clock.warp))
	box.add_child(warp_row)

	for option in FOCUS_OPTIONS:
		_focus.add_item(option[0])
	for site in world.sites:
		_focus.add_item("%s (Boden)" % site["name"])
	_focus.item_selected.connect(_on_focus_selected)
	box.add_child(_focus)

	for preset in PRESETS:
		_presets.add_item(preset[0])
	_presets.item_selected.connect(_on_preset_selected)
	box.add_child(_presets)

	box.add_child(HSeparator.new())
	var launch_label := Label.new()
	launch_label.text = "Start vorbereiten"
	box.add_child(launch_label)
	_catalogue = MissionCatalogue.load_from(Paths.config_dir())
	_place_list = _catalogue.places()
	for place in _place_list:
		_places.add_item(place["name"])
	_places.item_selected.connect(func(_index: int) -> void: _refill_vehicles())
	_vehicles.item_selected.connect(func(_index: int) -> void: _refill_missions())
	for option: OptionButton in [_places, _vehicles, _missions]:
		box.add_child(option)
	_start_button = _button("Starten", _on_start_pressed)
	box.add_child(_start_button)
	_start_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_start_status.custom_minimum_size.x = 300
	box.add_child(_start_status)
	world.flight.mission_started.connect(_on_start_finished)
	world.flight.sim.command_finished.connect(_on_command_finished)
	_refill_vehicles()


func _process(_delta: float) -> void:
	if not world.core.is_open():
		_utc_label.text = "SPICE-Daten fehlen: python tools/data_pipeline/fetch.py"
		return
	var t := world.clock.tdb_s
	_utc_label.text = "%s UTC" % world.core.tdb_s_to_utc(t).substr(0, 19).replace("T", "  ")
	var lines := PackedStringArray()
	lines.append("Zeitraffer: %sx" % _format_warp(world.clock.warp))
	lines.append("Mond-Distanz: %.0f km" % (world.moon_gcrf_m.length() / 1000.0))
	for site in world.sites:
		var fraction: float = world.core.solar_eclipse_fraction(site["latitude_deg"], site["longitude_deg"], site["height_m"], t)
		if fraction > 0.0:
			lines.append("Sonnenbedeckung %s: %.1f %%" % [site["name"], fraction * 100.0])
	if world.flight.sim.is_active():
		var sea := sea_state_line(world.flight.sim.landing_zone())
		if sea != "":
			lines.append(sea)
	if world.atmosphere.status_text != "":
		lines.append(world.atmosphere.status_text)
	_info_label.text = "\n".join(lines)


## The sea state and deck motion while a drone ship is the landing zone (FlightSim.landing_zone), else "".
static func sea_state_line(zone: Dictionary) -> String:
	if not zone.get("has_deck", false):
		return ""
	var sea: Dictionary = zone.get("sea_state", {})
	if sea.is_empty():
		return "Seegang ruhig - Deck in Ruhe"
	return "Seegang Hs %.1f m, Tp %.0f s - Deck: Hub %+.2f m, Rollen %+.1f°, Stampfen %+.1f°" % [sea["hs_m"], sea["tp_s"],
			zone.get("deck_heave_m", 0.0), zone.get("deck_roll_deg", 0.0), zone.get("deck_pitch_deg", 0.0)]


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
	_start_status.text = "Start wird geplant ..."
	var started := false
	if mission_id == MissionCatalogue.FREE_FLIGHT:
		var site: Dictionary = _place_list[_places.selected]["site"]
		var vehicle: Dictionary = _vehicle_list[_vehicles.selected]
		started = world.launch_vehicle(vehicle["id"], site, FlightController.LAUNCH_COUNTDOWN_S, vehicle["name"])
		if started:
			_focus.select(VEHICLE_FOCUS_INDEX)  # world.launch_vehicle follows the vehicle at once
	else:
		started = world.start_mission(mission_id, not (_place_list[_places.selected]["site"] as Dictionary).is_empty())
	if not started:
		_on_start_finished(false, "Befehl nicht angenommen")


func _on_command_finished(_id: int, command: String, ok: bool, message: String) -> void:
	if command == "launch_from_site" and _start_button.disabled:
		_on_start_finished(ok, message)


func _on_start_finished(ok: bool, message: String) -> void:
	_start_button.disabled = false
	if ok:
		_start_status.text = ""
		_focus.select(VEHICLE_FOCUS_INDEX)  # World follows the started vehicle
	else:
		_start_status.text = "Start abgelehnt: %s" % (message if message != "" else "unbekannter Grund")


func _button(text: String, action: Callable) -> Button:
	var button := Button.new()
	button.text = text
	button.pressed.connect(action)
	return button


static func _format_warp(warp: float) -> String:
	if absf(warp) >= 1000.0:
		return "%dk" % int(warp / 1000.0)
	return str(int(warp)) if warp == floorf(warp) else str(warp)
