# Headless test of the language service (I18n): tables, reports, formatting, the live switch and the setting.
# Run: godot --headless --path game --script res://tests/i18n_test.gd
extends SceneTree

var _failures := 0
var _signals := 0


func _initialize() -> void:
	var settings := "user://i18n_test_%d.cfg" % OS.get_process_id()
	# the first access to the class runs its static initialisers (they would reset the path): touch it first
	var events: I18nEvents = I18n.events
	I18n.settings_path = settings
	_check(I18n.settings_path == settings, "the settings path of this test is set (%s)" % I18n.settings_path)
	DirAccess.remove_absolute(ProjectSettings.globalize_path(settings))
	events.language_changed.connect(_on_language_changed)

	_check(I18n.saved_language() == "de", "no settings file: German")
	_check(I18n.language() == "de", "the default language is German")
	_check(I18n.text("ui.language.label") == "Sprache", "real table, de: ui.language.label")
	_check(I18n.has_key("report.internal_error") and I18n.has_key("report.command.internal_error"), "first report keys exist")
	# real phase table (de): the ids the bindings send resolve to today's German words, not to their keys
	for pair: Array in [["autopilot_state.off", "aus"], ["autopilot_state.upper_stage", "Oberstufe"],
			["landing_outcome.flying", "fliegt"], ["docking_phase.soft_capture", "Weichfang"],
			["proximity_phase.final_approach", "Endanflug"], ["catch_phase.aborted", "Abbruch"],
			["mission_event_kind.meco", "MECO"], ["guidance_mode.upright", "aufrecht"]]:
		_check(I18n.phase_text(pair[0]) == pair[1], "real phase table, de: %s -> %s" % [pair[0], I18n.phase_text(pair[0])])

	# real report table (de): the rendezvous codes read as before
	var waiting := {"code": "rendezvous.waiting", "params": {"kind": "phase.phasing_burn_kind.boost", "wait": 5400.0},
			"kinds": {"kind": "ref", "wait": "time"}, "causes": []}
	_check(I18n.report_text(waiting) == "Phasing: Anhebung in 1,5 h", "real report, de: %s" % I18n.report_text(waiting))
	var window := {"code": "launch_window.none", "params": {"days": 3.0}, "kinds": {"days": "number"}, "causes": [
			{"code": "phasing.orbit_too_close", "params": {}, "kinds": {}, "causes": []}]}
	_check(I18n.report_text(window) == "Kein Startfenster in 3 Tagen: Kein Rendezvous: Phasing-Bahn zu nah an der Zielbahn",
			"real report, de: %s" % I18n.report_text(window))

	var de := {
		"report.t.steep": "zu steil: {slope:deg1} (Grenze {limit})",
		"report.t.outer": "außen: {cause}",
		"report.t.ref": "Phase {what}",
		"report.internal_error": "Interner Fehler",
		"report.t.oem": "OEM vom {date:date}, {date}: {maneuvers}",
		"phase.t.coast": "Freiflug",
		"only.de": "nur",
	}
	var en := {
		"report.t.steep": "too steep: {slope:deg1} (limit {limit})",
		"report.t.outer": "outer: {cause}",
		"report.t.ref": "phase {what}",
		"report.internal_error": "Internal error",
		"phase.t.coast": "coast",
	}
	I18n.use_tables(de, en)
	var steep := {"code": "t.steep", "params": {"slope": 0.1449, "limit": 0.1396},
			"kinds": {"slope": "angle", "limit": "angle"}, "causes": []}
	_check(I18n.report_text(steep) == "zu steil: 8,3° (Grenze 8,0°)", "report de: %s" % I18n.report_text(steep))
	var ok_report := {"code": "ok", "params": {}, "kinds": {}, "causes": []}
	_check(I18n.report_text(ok_report) == "", "ok report is empty")
	_check(I18n.text("no.such.key") == "no.such.key", "missing key returns the key")
	var ref := {"code": "t.ref", "params": {"what": "phase.t.coast"}, "kinds": {"what": "ref"}, "causes": []}
	_check(I18n.report_text(ref) == "Phase Freiflug", "ref parameter resolves: %s" % I18n.report_text(ref))
	var outer := {"code": "t.outer", "params": {}, "kinds": {}, "causes": [steep]}
	_check(I18n.report_text(outer) == "außen: zu steil: 8,3° (Grenze 8,0°)", "cause renders nested: %s" % I18n.report_text(outer))
	var internal := {"code": "internal_error", "params": {}, "kinds": {}, "causes": []}
	_check(I18n.report_text(internal) == "Interner Fehler", "internal_error report text")
	_check(I18n.phase_text("t.coast") == "Freiflug" and I18n.phase_text("") == "", "phase text")

	_check(I18n.format_value(12345.0, "mass") == "12,3 t", "mass default: %s" % I18n.format_value(12345.0, "mass"))
	_check(I18n.format_value(123456.0, "length", "km1") == "123,5 km", "km1")
	_check(I18n.format_value(3725.0, "time", "clock") == "1:02:05", "clock 1:02:05")
	_check(I18n.format_value(90061.0, "time", "clock") == "25:01:01", "clock beyond 24 h")
	_check(I18n.format_value(65.0, "time", "clock") == "01:05", "clock below 1 h")
	_check(I18n.format_value(NAN, "number") == "–" and I18n.format_value(INF, "length") == "–", "NaN and inf: dash")
	_check(I18n.format_value(-0.0, "number", "n1") == "0,0", "negative zero: %s" % I18n.format_value(-0.0, "number", "n1"))
	_check(I18n.format_value(3.0, "count") == "3", "count")
	_check(I18n.format_value(1760466601.0, "utc") == "2025-10-14 18:30 UTC", "utc default: %s" % I18n.format_value(1760466601.0, "utc"))
	_check(I18n.format_value(1760466601.0, "utc", "date") == "2025-10-14", "date spec")
	_check(I18n.format_value(NAN, "utc") == "–" and I18n.format_value(NAN, "utc", "date") == "–", "utc NaN: dash")
	var oem := {"code": "t.oem", "params": {"date": 1760466601.0, "maneuvers": "A1, B2"}, "kinds": {"date": "utc", "maneuvers": "data_name"}, "causes": []}
	_check(I18n.report_text(oem) == "OEM vom 2025-10-14, 2025-10-14 18:30 UTC: A1, B2", "station report: %s" % I18n.report_text(oem))
	_check(I18n.format_value(0.25, "number", "pct0") == "25 %", "pct0")

	# the live switch
	I18n.reload_tables()  # real tables again
	var label := Label.new()
	I18n.bind_text(label, "text", "ui.language.label")
	_check(label.text == "Sprache", "bound label starts German")
	I18n.set_language("en")
	_check(I18n.language() == "en" and _signals == 1, "switch to en emits once (%d)" % _signals)
	_check(label.text == "Language", "bound label follows the switch: %s" % label.text)
	_check(I18n.report_text({"code": "command.internal_error", "params": {}, "kinds": {}, "causes": []})
			== "Internal error in command", "stored report re-rendered in English")
	I18n.use_tables(de, en)
	_check(I18n.report_text(steep) == "too steep: 8.3° (limit 8.0°)", "report en: %s" % I18n.report_text(steep))
	_check(I18n.text("only.de") == "nur", "en falls back to de")
	_check(I18n.format_value(1234.0, "number", "n1") == "1234.0", "en decimal point")
	var config := ConfigFile.new()
	_check(config.load(settings) == OK and config.get_value("ui", "language", "") == "en", "setting saved as [ui] language")
	I18n.set_language("fr")
	_check(I18n.language() == "en" and _signals == 1, "unknown language changes nothing")
	_check(I18n.saved_language() == "en", "saved_language reads en")
	config.set_value("ui", "language", "fr")
	config.save(settings)
	_check(I18n.saved_language() == "de", "unknown stored value reads as de")
	DirAccess.remove_absolute(ProjectSettings.globalize_path(settings))
	_check(I18n.saved_language() == "de", "no file: de")
	I18n.set_language("de")

	# stored HUD events resolve at draw time (Review Focus 1): the same data reads German, then English
	I18n.reload_tables()
	var ignition := {"key": "event.ignition"}
	var refused := {"key": "event.start_refused", "params": {"reason": {"code": "command.internal_error", "params": {},
			"kinds": {}, "causes": []}}}
	var launched := {"key": "event.launch_at", "params": {"vehicle": {"literal": "Mk3"}, "site": {"key": "event.ignition"}}}
	var planned := {"key": "event.return_planned_ship", "then": {"key": "event.dispersion_warning",
			"params": {"note": {"code": "command.internal_error", "params": {}, "kinds": {}, "causes": []}}}}
	var core_report := {"code": "command.internal_error", "params": {}, "kinds": {}, "causes": []}
	_check(FlightController.event_text(ignition) == "Zündung", "event de: %s" % FlightController.event_text(ignition))
	_check(FlightController.event_text(refused) == "Start abgelehnt: Interner Fehler im Befehl",
			"event with a nested report, de: %s" % FlightController.event_text(refused))
	_check(FlightController.event_text(launched) == "Mk3 auf Zündung", "event with literal and key params, de")
	_check(FlightController.event_text(planned).begins_with("Rückkehr geplant: ") and FlightController.event_text(planned).contains(" - Achtung: "),
			"event with a then-part, de: %s" % FlightController.event_text(planned))
	_check(FlightController.event_text({}) == "" and FlightController.event_text(core_report) == "Interner Fehler im Befehl",
			"empty event is empty, a core report renders itself")
	_check(FlightHud.key_help().contains("Tab / Umschalt+Tab") and FlightHud.key_help().count("\n") == 2, "key help de")
	I18n.set_language("en")
	_check(FlightController.event_text(ignition) == "Ignition", "the stored event re-renders in English: %s" % FlightController.event_text(ignition))
	_check(FlightController.event_text(refused) == "Start refused: Internal error in command", "nested report in English: %s" % FlightController.event_text(refused))
	_check(FlightHud.key_help().contains("Tab / Shift+Tab"), "key help en")
	_check(ModelNotice.text_for("stale") == "notice.stale" and I18n.text("notice.stale").begins_with("Placeholder model"), "model notice en")
	var notice := ModelNotice.label("notice.stale")
	_check(notice.text.begins_with("Placeholder model"), "notice label starts in the active language")
	I18n.set_language("de")
	_check(notice.text.begins_with("Platzhaltermodell"), "notice label follows the switch back to German: %s" % notice.text)
	_check(FlightController.event_text(ignition) == "Zündung", "back to German")
	notice.free()
	# panels, wizard, world and status texts (Task 15): every one reads in the active language at call time
	I18n.set_language("de")
	var map_label := MapLabel.new()
	map_label.setup_key("ui.map.earth", Color.WHITE)
	var zone := {"offset_s": -18000, "abbreviation": "CDT"}
	var weather := {"kind": "era5", "valid_hour": 1000000}
	var entry := {"device_name": "Stick", "axis": 2, "center": 0.0, "raw_pos": 0.5, "raw_neg": -0.5}
	var atmosphere := Atmosphere.new()
	atmosphere.status_key = "loading.atmosphere_load"
	var panel_de := [TimePanel.camera_height_line(2500.0, false), LocalTime.format_line(zone, 1700000000),
			WeatherService.snapshot_label(weather), FreeCameraControl.role_label("roll"), atmosphere.status_text(),
			(map_label.mesh as TextMesh).text, FreeCameraWizard._describe_entry(entry), TimePanel.sea_state_line({"has_deck": true})]
	I18n.set_language("en")
	var panel_en := [TimePanel.camera_height_line(2500.0, false), LocalTime.format_line(zone, 1700000000),
			WeatherService.snapshot_label(weather), FreeCameraControl.role_label("roll"), atmosphere.status_text(),
			(map_label.mesh as TextMesh).text, FreeCameraWizard._describe_entry(entry), TimePanel.sea_state_line({"has_deck": true})]
	for i in panel_de.size():
		_check(panel_de[i] != "" and panel_en[i] != "" and panel_de[i] != panel_en[i],
				"text %d differs by language: '%s' / '%s'" % [i, panel_de[i], panel_en[i]])
	_check(panel_en[0] == "Camera height: 2500 m above MSL" and panel_en[5] == "Earth" and panel_en[3] == "Roll",
			"English panel texts: %s" % [panel_en])
	_check(MissionCatalogue.load_from(Paths.config_dir()).missions_for("ccsfs_slc40", "falcon9_block5")[0]["name"] == "Free flight",
			"catalogue free flight in English")
	var named := {"key": "event.launch_at", "params": {"vehicle": {"key": "vehicle.starship_block3.display_name"},
			"site": {"key": "site.starbase_pad2.name"}}}
	_check(FlightController.event_text(named).begins_with("Heavy Launcher Mk3 at "), "launch event re-renders the vehicle name: %s"
			% FlightController.event_text(named))
	I18n.set_language("de")
	_check(map_label.mesh != null and (map_label.mesh as TextMesh).text == "Erde", "map label follows the switch to German")
	map_label.free()
	DirAccess.remove_absolute(ProjectSettings.globalize_path(settings))
	label.free()

	print("i18n_test: %s" % ("PASS" if _failures == 0 else "FAIL (%d)" % _failures))
	quit(1 if _failures > 0 else 0)


func _on_language_changed(_code: String) -> void:
	_signals += 1


func _check(condition: bool, message: String) -> void:
	if not condition:
		_failures += 1
		print("FAIL: ", message)
