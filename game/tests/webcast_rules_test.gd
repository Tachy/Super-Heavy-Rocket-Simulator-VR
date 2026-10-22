# Headless test of the webcast overlay logic (gauge rules, timeline window).
# Run: godot --headless --path game --script res://tests/webcast_rules_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	_test_gauges()
	_test_window()
	_test_labels()
	print("WEBCAST RULES TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _telemetry(over: Dictionary = {}) -> Dictionary:
	var t := {
		"altitude_m": 50.0e3, "knudsen": 0.001, "dynamic_pressure_pa": 5000.0, "clamped": false,
		"heat_rate_w_m2": 0.0, "apoapsis_altitude_km": 120.0, "periapsis_altitude_km": -2000.0,
	}
	t.merge(over, true)
	return t


func _body(over: Dictionary = {}) -> Dictionary:
	var b := {"engines": [1], "stages": [], "docking": {}}
	b.merge(over, true)
	return b


func _test_gauges() -> void:
	var state := {}
	var hot := _telemetry({"heat_rate_w_m2": 2e3})
	var coasting := _body({"engines": []})  # with engines the 4-per-side limit would cut heat flux
	var sel := GaugeRules.select(hot, coasting, state, 0.0)
	_check("heat_flux" in sel["left"], "heat flux appears at 2e3 W/m2")
	var cool := _telemetry()
	sel = GaugeRules.select(cool, coasting, state, 4.0)
	_check("heat_flux" in sel["left"], "heat flux stays 4 s after dropping to 0")
	sel = GaugeRules.select(cool, coasting, state, 6.0)
	_check(not ("heat_flux" in sel["left"]), "heat flux gone at 6 s")

	sel = GaugeRules.select(_telemetry({"knudsen": 1.0}), _body(), {}, 0.0)
	_check(not ("mach" in sel["left"]), "Mach hidden at Knudsen 1")
	sel = GaugeRules.select(_telemetry({"clamped": true}), _body(), {}, 0.0)
	_check(not ("g_load" in sel["left"]), "a clamped body has no g load")
	sel = GaugeRules.select(_telemetry(), _body({"engines": []}), {}, 0.0)
	_check(not ("engines" in sel["left"]), "no engines gauge without engines")

	# Everything true at once: limits and priority order.
	var docking := {"present": true, "range_m": 50.0, "docked": false}
	sel = GaugeRules.select(_telemetry({"heat_rate_w_m2": 5e4, "altitude_m": 200.0e3}), _body({"docking": docking}),
			{}, 0.0)
	_check(sel["left"].size() <= GaugeRules.MAX_PER_SIDE and sel["right"].size() <= GaugeRules.MAX_PER_SIDE,
			"never more than 4 per side")
	_check(sel["left"] == ["engines", "g_load", "dynamic_pressure", "mach"], "left side by priority")
	_check(sel["right"] == ["speed_surface", "speed_inertial", "altitude", "apoapsis"], "right side by priority")
	sel = GaugeRules.select(_telemetry({"apoapsis_altitude_km": INF, "altitude_m": 200.0e3}), _body(), {}, 0.0)
	_check(not ("apoapsis" in sel["right"]) and not ("periapsis" in sel["right"]), "no apsides on an open orbit")
	_check(GaugeRules.docking_active(docking) and not GaugeRules.docking_active({"present": true, "range_m": 1.0,
			"docked": true}) and not GaugeRules.docking_active({}), "docking active only when approaching")


func _test_window() -> void:
	var now := 1.0e9
	var events := PackedFloat64Array([now - 70.0, now + 30.0, now + 400.0, now + 3.0 * 86400.0])
	var w := TimelineWindow.target_span(events, now)
	_check(w.x <= now - 70.0 and w.y >= now + 400.0 and w.y < now + 3.0 * 86400.0,
			"window contains -70 s and +400 s, ends before +3 d (%s)" % str(w - Vector2(now, now)))
	w = TimelineWindow.target_span(PackedFloat64Array(), now)
	_check(w.y - w.x >= TimelineWindow.MIN_SPAN_S, "empty event list: span >= 60 s")
	w = TimelineWindow.target_span(PackedFloat64Array([now - 10.0]), now)
	_check(w.y - w.x >= TimelineWindow.MIN_SPAN_S and w.y > now, "no future event: end after now")
	var near := TimelineWindow.target_span(PackedFloat64Array([now - 1000.0, now + 100.0, now + 200.0, now + 300.0,
			now + 900.0]), now)
	_check(absf(near.y - (now + 300.0 + 130.0)) < 1e-6 and absf(near.x - (now - 1000.0 - 130.0)) < 1e-6,
			"third next event plus 10 %% margin (%s)" % str(near - Vector2(now, now)))
	var e := TimelineWindow.ease(Vector2(0, 10), Vector2(100, 200), 2.0)
	_check(absf(e.x - 100.0 * (1.0 - exp(-1.0))) < 1e-9, "ease: one time constant per 2 s")
	_check(TimelineWindow.ease(Vector2(0, 10), Vector2(5, 20), 0.0) == Vector2(0, 10), "ease: zero delta keeps window")


func _test_labels() -> void:
	var t0 := 1.0e9
	var events: Array = []
	var kinds := ["ignition", "cutoff", "max_q", "meco", "docking"]
	for i in 60:
		events.append({"kind": kinds[i % kinds.size()], "tdb_s": t0 + i * 52.0 * 86400.0 / 59.0})
	var window := Vector2(t0, t0 + 52.0 * 86400.0)
	var shown := TimelineWindow.visible_labels(events, window, 1000.0)
	_check(shown.size() < events.size() and shown.size() > 0, "60 events thinned (%d shown)" % shown.size())
	var ok := true
	for i in range(1, shown.size()):
		var dpx: float = (shown[i]["tdb_s"] - shown[i - 1]["tdb_s"]) / (window.y - window.x) * 1000.0
		ok = ok and dpx >= TimelineWindow.MIN_LABEL_PX
	_check(ok, "no two labels closer than 70 px")

	var pair := [{"kind": "ignition", "tdb_s": 10.0}, {"kind": "meco", "tdb_s": 12.0}]
	shown = TimelineWindow.visible_labels(pair, Vector2(0.0, 100.0), 500.0)
	_check(shown.size() == 1 and shown[0]["kind"] == "meco", "higher priority kind survives")
	pair = [{"kind": "max_q", "tdb_s": 10.0}, {"kind": "max_q", "tdb_s": 12.0}]
	shown = TimelineWindow.visible_labels(pair, Vector2(0.0, 100.0), 500.0)
	_check(shown.size() == 1 and shown[0]["tdb_s"] == 10.0, "equal priority keeps the earlier event")
	shown = TimelineWindow.visible_labels(pair, Vector2(50.0, 100.0), 500.0)
	_check(shown.is_empty(), "events outside the window are dropped")


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
