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
	_check(I18n.has_key("report.legacy") and I18n.has_key("report.command.internal_error"), "first report keys exist")
	# real phase table (de): the ids the bindings send resolve to today's German words, not to their keys
	for pair: Array in [["autopilot_state.off", "aus"], ["autopilot_state.upper_stage", "Oberstufe"],
			["landing_outcome.flying", "fliegt"], ["docking_phase.soft_capture", "Weichfang"],
			["proximity_phase.final_approach", "Endanflug"], ["catch_phase.aborted", "Abbruch"],
			["mission_event_kind.meco", "MECO"], ["guidance_mode.upright", "aufrecht"]]:
		_check(I18n.phase_text(pair[0]) == pair[1], "real phase table, de: %s -> %s" % [pair[0], I18n.phase_text(pair[0])])

	var de := {
		"report.t.steep": "zu steil: {slope:deg1} (Grenze {limit})",
		"report.t.outer": "außen: {cause}",
		"report.t.ref": "Phase {what}",
		"report.legacy": "{text}",
		"phase.t.coast": "Freiflug",
		"only.de": "nur",
	}
	var en := {
		"report.t.steep": "too steep: {slope:deg1} (limit {limit})",
		"report.t.outer": "outer: {cause}",
		"report.t.ref": "phase {what}",
		"report.legacy": "{text}",
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
	var legacy := {"code": "legacy", "params": {"text": "Altlast"}, "kinds": {"text": "legacy_text"}, "causes": []}
	_check(I18n.report_text(legacy) == "Altlast", "legacy report is its text")
	_check(I18n.phase_text("t.coast") == "Freiflug" and I18n.phase_text("") == "", "phase text")

	_check(I18n.format_value(12345.0, "mass") == "12,3 t", "mass default: %s" % I18n.format_value(12345.0, "mass"))
	_check(I18n.format_value(123456.0, "length", "km1") == "123,5 km", "km1")
	_check(I18n.format_value(3725.0, "time", "clock") == "1:02:05", "clock 1:02:05")
	_check(I18n.format_value(90061.0, "time", "clock") == "25:01:01", "clock beyond 24 h")
	_check(I18n.format_value(65.0, "time", "clock") == "01:05", "clock below 1 h")
	_check(I18n.format_value(NAN, "number") == "–" and I18n.format_value(INF, "length") == "–", "NaN and inf: dash")
	_check(I18n.format_value(-0.0, "number", "n1") == "0,0", "negative zero: %s" % I18n.format_value(-0.0, "number", "n1"))
	_check(I18n.format_value(3.0, "count") == "3", "count")
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
