# Headless test of the webcast overlay logic (gauge rules, timeline window).
# Run: godot --headless --path game --script res://tests/webcast_rules_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	_test_gauges()
	_test_config_matches_defaults()
	_test_config_errors()
	_test_window()
	_test_labels()
	print("WEBCAST RULES TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _telemetry(over: Dictionary = {}) -> Dictionary:
	var t := {
		"altitude_m": 50.0e3, "knudsen": 0.001, "dynamic_pressure_pa": 5000.0, "clamped": false,
		"heat_rate_w_m2": 0.0, "apoapsis_altitude_km": 120.0, "periapsis_altitude_km": -2000.0,
		"propellant_kg": 500.0, "propellant_capacity_kg": 1000.0,
	}
	t.merge(over, true)
	return t


func _engine(firing: bool, auxiliary := false) -> Dictionary:
	return {"firing": firing, "auxiliary": auxiliary, "stage": 0, "position_m": Vector3.ZERO}


func _body(over: Dictionary = {}) -> Dictionary:
	var b := {"engines": [_engine(false)], "stages": [], "docking": {}}
	b.merge(over, true)
	return b


func _test_gauges() -> void:
	var state := {}
	var hot := _telemetry({"heat_rate_w_m2": 2e3})
	var coasting := _body()  # entry-like: engines present, g, q > 100 Pa, Knudsen < 0.1
	var sel := GaugeRules.select(hot, coasting, state, 0.0)
	_check("heat_flux" in sel["left"] and "heat_load" in sel["left"], "heat flux and load appear at 2e3 W/m2 with engines")
	var cool := _telemetry()
	sel = GaugeRules.select(cool, coasting, state, 4.0)
	_check("heat_flux" in sel["left"], "heat flux stays 4 s after dropping to 0")
	sel = GaugeRules.select(cool, coasting, state, 6.0)
	_check(not ("heat_flux" in sel["left"]), "heat flux gone at 6 s")

	sel = GaugeRules.select(_telemetry({"knudsen": 1.0}), _body(), {}, 0.0)
	_check(not ("mach" in sel["left"]), "Mach hidden at Knudsen 1")
	sel = GaugeRules.select(_telemetry({"clamped": true}), _body(), {}, 0.0)
	_check(not ("g_load" in sel["left"]) and not ("mach" in sel["left"]), "on the pad: no g load, no Mach")
	var coast := _telemetry({"dynamic_pressure_pa": 0.0, "knudsen": 1.0e6, "altitude_m": 300.0e3})
	sel = GaugeRules.select(coast, _body({"engines": [_engine(false)]}), {}, 0.0)
	_check(not ("g_load" in sel["left"]), "orbital coast: no constant 0 g")
	sel = GaugeRules.select(coast, _body({"engines": [_engine(true)]}), {}, 0.0)
	_check("g_load" in sel["left"], "a burn in orbit: g load shown")
	sel = GaugeRules.select(_telemetry({"dynamic_pressure_pa": 50.0}), _body(), {}, 0.0)
	_check(not ("mach" in sel["left"]), "Mach needs dynamic pressure above its minimum")
	sel = GaugeRules.select(_telemetry({"dynamic_pressure_pa": 30.0e3, "mach": 1.5}), _body({"engines": [_engine(true)]}),
			{}, 0.0)
	_check("g_load" in sel["left"] and "mach" in sel["left"], "ascent: g load and Mach shown")
	sel = GaugeRules.select(_telemetry(), _body({"engines": []}), {}, 0.0)
	_check(not ("engines" in sel["left"]), "no engines gauge without engines")
	sel = GaugeRules.select(_telemetry(), _body({"engines": [_engine(true, true)]}), {}, 0.0)
	_check(not ("engines" in sel["left"]), "abort-only / landing thrusters are no engines gauge")
	sel = GaugeRules.select(_telemetry({"propellant_capacity_kg": 1000.0}), _body(), {}, 0.0)
	_check(sel["left"].slice(0, 2) == ["engines", "propellant"], "propellant right after the engines")
	sel = GaugeRules.select(_telemetry({"propellant_capacity_kg": 0.0}), _body(), {}, 0.0)
	_check(not ("propellant" in sel["left"]), "no propellant gauge without a tank load")

	# Everything true at once: limits and priority order.
	var docking := {"present": true, "range_m": 50.0, "docked": false}
	sel = GaugeRules.select(_telemetry({"heat_rate_w_m2": 5e4, "altitude_m": 200.0e3}), _body({"docking": docking}),
			{}, 0.0)
	_check(sel["left"].size() <= GaugeConfig.shared().max_per_side and sel["right"].size() <= GaugeConfig.shared().max_per_side,
			"never more than 4 per side")
	_check(sel["left"] == ["engines", "propellant", "heat_flux", "heat_load"], "left side by priority")
	_check(sel["right"] == ["range", "closing_rate", "altitude", "speed_inertial"], "docking: range and closing rate shown")
	sel = GaugeRules.select(_telemetry({"altitude_m": 400.0e3}), _body(), {}, 0.0)
	_check(sel["right"] == ["speed_inertial", "altitude", "apoapsis", "periapsis"], "orbit: apsides shown")
	sel = GaugeRules.select(_telemetry({"altitude_m": 20.0e3}), _body(), {}, 0.0)
	_check(sel["right"] == ["speed_surface", "altitude", "vertical_speed", "speed_inertial"],
			"ascent: surface speed, altitude, vertical speed shown")
	sel = GaugeRules.select(_telemetry({"altitude_m": 200.0e3, "apoapsis_altitude_km": INF}), _body(), {}, 0.0)
	_check(sel["right"] == ["speed_surface", "altitude", "speed_inertial"], "open orbit: no apsides, no vertical speed")
	sel = GaugeRules.select(_telemetry({"apoapsis_altitude_km": INF, "altitude_m": 200.0e3}), _body(), {}, 0.0)
	_check(not ("apoapsis" in sel["right"]) and not ("periapsis" in sel["right"]), "no apsides on an open orbit")
	_check(GaugeRules.docking_active(docking) and not GaugeRules.docking_active({"present": true, "range_m": 1.0,
			"docked": true}) and not GaugeRules.docking_active({}), "docking active only when approaching")
	_test_transfer()


func _test_config_matches_defaults() -> void:
	var cfg := GaugeConfig.shared()
	_check(cfg.errors.is_empty(), "shipped config loads without errors (%s)" % [cfg.errors])
	_check(cfg.max_per_side == 4 and cfg.hide_delay_s == 5.0, "rules from the file")
	_check(cfg.situations["orbit"] == ["speed_inertial", "altitude", "apoapsis", "periapsis"], "orbit situation list")
	_check(cfg.gauges["altitude"]["log_min"] == 0.1, "altitude log_min")


func _test_config_errors() -> void:
	var cfg := GaugeConfig.parse("""
[rules]
hide_delay_s = 5.0
max_per_side = 4
[left]
gauges = ["engines", "bar"]
[gauge.engines]
label = "E"
unit = ""
full_scale = 1.0
decimals = 0
condition = "has_engines"
[gauge.foo]
label = "F"
unit = ""
full_scale = 1.0
decimals = 0
condition = "nonsense"
""")
	_check(cfg.errors.size() == 2, "bad condition, unknown left id: 2 errors (%s)" % [cfg.errors])
	var sel := GaugeRules.select(_telemetry(), _body(), {}, 0.0, cfg)
	_check(sel["left"] == ["engines"] and sel["right"].is_empty(), "bad entries dropped, engines still shown (%s)" % [sel])
	var missing := GaugeConfig.load_file("res://does_not_exist.toml")
	_check(missing.errors.size() == 1, "missing file: one error")
	sel = GaugeRules.select(_telemetry(), _body(), {}, 0.0, missing)
	_check(sel["left"].is_empty() and sel["right"].is_empty(), "missing file: empty selection")


## Task 4c: the Earth-Moon transfer situation (AROW-like gauges).
func _test_transfer() -> void:
	var transfer_ids := ["distance_earth", "distance_moon", "speed_earth", "speed_moon"]
	var moon := {"distance_moon_m": 300.0e6, "speed_moon_mps": 1500.0, "distance_earth_m": 90.0e6,
			"speed_earth_mps": 3000.0, "dynamic_pressure_pa": 0.0, "knudsen": 1.0e6}
	# Translunar coast: Earth reference, 84,000 km up, apogee near the Moon's distance.
	var tli := _telemetry(moon)
	tli.merge({"altitude_m": 84.0e6, "apoapsis_altitude_km": 390.0e3, "periapsis_altitude_km": 200.0}, true)
	var sel := GaugeRules.select(tli, _body(), {}, 0.0)
	_check(sel["right"] == transfer_ids, "translunar coast: transfer gauges (%s)" % [sel["right"]])
	# Trans-Earth coast near the Moon: open about the Moon.
	var tei := _telemetry(moon)
	tei.merge({"altitude_m": 20.0e6, "apoapsis_altitude_km": INF}, true)
	_check(GaugeRules.select(tei, _body(), {}, 0.0)["right"] == transfer_ids, "open orbit far from the Moon: transfer")
	# LLO, NRHO, LEO and GEO stay "orbit".
	var llo := _telemetry(moon)
	llo.merge({"altitude_m": 100.0e3 + 1.0, "apoapsis_altitude_km": 110.0, "periapsis_altitude_km": 90.0}, true)
	var orbit_ids := ["speed_inertial", "altitude", "apoapsis", "periapsis"]
	_check(GaugeRules.select(llo, _body(), {}, 0.0)["right"] == orbit_ids, "LLO: orbit with the inertial speed")
	var nrho := _telemetry(moon)
	nrho.merge({"altitude_m": 68.0e6, "apoapsis_altitude_km": 68.5e3, "periapsis_altitude_km": 1500.0}, true)
	_check(GaugeRules.select(nrho, _body(), {}, 0.0)["right"] == orbit_ids, "NRHO near its apolune: orbit")
	var geo := _telemetry({"altitude_m": 35.786e6, "apoapsis_altitude_km": 35786.0, "periapsis_altitude_km": 35786.0})
	_check(GaugeRules.select(geo, _body(), {}, 0.0)["right"] == orbit_ids, "GEO: orbit")
	var leo := _telemetry({"altitude_m": 400.0e3, "apoapsis_altitude_km": 420.0, "periapsis_altitude_km": 380.0})
	_check(GaugeRules.select(leo, _body(), {}, 0.0)["right"] == orbit_ids, "LEO: orbit, unchanged")
	# Just after a TLI: apogee far out but still low: not yet transfer.
	var after_tli := tli.duplicate()
	after_tli["altitude_m"] = 1.0e6
	_check(GaugeRules.select(after_tli, _body(), {}, 0.0)["right"] == orbit_ids, "1,000 km after TLI: still orbit")
	# Hysteresis: once in transfer it holds slightly below the entry limit and ends clearly below it.
	var state := {}
	GaugeRules.select(tli, _body(), state, 0.0)
	var edge := tli.duplicate()
	edge["altitude_m"] = 0.9 * GaugeConfig.shared().threshold("transfer_min_altitude_m")
	_check(GaugeRules.select(edge, _body(), state, 1.0)["right"] == transfer_ids, "transfer holds inside the band")
	_check(GaugeRules.select(edge, _body(), {}, 1.0)["right"] != transfer_ids, "the band is not entered from outside")
	edge["altitude_m"] = 0.7 * GaugeConfig.shared().threshold("transfer_min_altitude_m")
	_check(GaugeRules.select(edge, _body(), state, 2.0)["right"][0] == "speed_inertial", "transfer ends below the band")
	# Without a Moon in the session: no Moon gauges.
	var earth_only := tli.duplicate()
	earth_only.erase("distance_moon_m")
	earth_only.erase("speed_moon_mps")
	_check(GaugeRules.select(earth_only, _body(), {}, 0.0)["right"] == ["distance_earth", "speed_earth"],
			"no Moon in the session: Earth distance and speed only")


func _test_window() -> void:
	var now := 1.0e9
	var edge := TimelineWindow.visible_half_fraction()  # inner edge of the fade zone (half-arc fraction)
	var scale := Vector2(100.0, 1000.0)
	var ok := true
	var last := -INF
	for i in 41:
		var dt := -2000.0 + 100.0 * i
		var p := TimelineWindow.arc_position(dt, scale)
		ok = ok and p > last and absf(p + TimelineWindow.arc_position(-dt, scale)) < 1e-12
		last = p
	_check(ok, "arc position monotone and symmetric for the past")
	_check(TimelineWindow.arc_position(0.0, scale) == 0.0, "now at the apex")
	_check(absf(TimelineWindow.arc_position(1000.0, scale) - edge) < 1e-12, "the reach at the fade zone's inner edge")

	var events := PackedFloat64Array([now - 70.0, now + 30.0, now + 400.0, now + 900.0, now + 3.0 * 86400.0])
	var sc := TimelineWindow.target_scale(events, now)
	_check(absf(TimelineWindow.arc_position(900.0, sc) - edge) < 1e-9, "third next event at the fade edge")
	_check(TimelineWindow.arc_position(3.0 * 86400.0, sc) > 1.0, "a fourth event lies off the arc")
	_check(TimelineWindow.arc_position(30.0, sc) >= TimelineWindow.NEAR_MIN_POSITION - 1e-6, "next event clear of the apex")

	# Launch: a next event 30 s and one 3 d ahead - both on the arc, the 30 s one clearly nearer the apex.
	sc = TimelineWindow.target_scale(PackedFloat64Array([now + 30.0, now + 3.0 * 86400.0]), now)
	var p30 := TimelineWindow.arc_position(30.0, sc)
	var p3d := TimelineWindow.arc_position(3.0 * 86400.0, sc)
	_check(p30 > 0.0 and p3d <= edge + 1e-9 and p30 < 0.5 * p3d, "30 s and 3 d both visible, 30 s nearer (%.3f, %.3f)"
			% [p30, p3d])
	_check(p30 >= TimelineWindow.NEAR_MIN_POSITION - 1e-6, "the 30 s event gets room (%.3f)" % p30)
	# Rough proportions: 60 s and 160 s ahead are spaced about linearly (no compression needed).
	sc = TimelineWindow.target_scale(PackedFloat64Array([now + 60.0, now + 160.0]), now)
	var ratio := TimelineWindow.arc_position(60.0, sc) / TimelineWindow.arc_position(160.0, sc)
	_check(ratio > 60.0 / 160.0 and ratio < 0.6, "near events keep rough proportions (%.3f)" % ratio)

	sc = TimelineWindow.target_scale(PackedFloat64Array(), now)
	_check(absf(sc.y - TimelineWindow.MIN_SPAN_S) < 1e-9, "no events: MIN_SPAN_S")
	sc = TimelineWindow.target_scale(PackedFloat64Array([now - 1000.0]), now)
	_check(absf(TimelineWindow.arc_position(-1000.0, sc) + edge) < 1e-9, "no next event: the last past event at the left edge")
	sc = TimelineWindow.target_scale(PackedFloat64Array([now - 10.0 * 86400.0, now + 6.0 * 3600.0]), now)
	_check(absf(TimelineWindow.arc_position(6.0 * 3600.0, sc) - edge) < 1e-9, "a single event 6 h ahead: scale of hours")
	_test_glide(now)
	var e := TimelineWindow.ease(Vector2(0, 10), Vector2(100, 200), 2.0)
	_check(absf(e.x - 100.0 * (1.0 - exp(-1.0))) < 1e-9, "ease: one time constant per 2 s")
	_check(TimelineWindow.ease(Vector2(0, 10), Vector2(5, 20), 0.0) == Vector2(0, 10), "ease: zero delta keeps the scale")


## Fix 2: with fixed events the scale stays put, so the nearest event glides steadily to the apex (no jump at the end);
## it is chosen anew once that event has passed.
func _test_glide(now: float) -> void:
	var events := PackedFloat64Array([now - 40.0, now + 60.0, now + 160.0, now + 3.0 * 86400.0, now + 4.0 * 86400.0])
	var holder := TimelineWindow.new()
	var first := holder.scale_target(events, now)
	var edge := TimelineWindow.visible_half_fraction()
	_check(absf(TimelineWindow.arc_position(3.0 * 86400.0, first) - edge) < 1e-9, "furthest of the next 3 at the fade edge")
	var step_s := 0.5
	var last := INF
	var last_step := 0.0
	var max_step := 0.0
	var max_growth := 0.0
	var steady := true
	var t := now
	while t <= now + 60.0 - step_s:
		var scale := holder.scale_target(events, t)
		steady = steady and scale == first
		var p := TimelineWindow.arc_position(now + 60.0 - t, scale)
		if is_finite(last):
			var step := last - p
			steady = steady and step > 0.0
			max_step = maxf(max_step, step)
			if last_step > 0.0:
				max_growth = maxf(max_growth, step / last_step)
			last_step = step
		last = p
		t += step_s
	var final_step := last  # the last 0.5 s down to the apex
	_check(steady, "scale fixed and the nearest event moving monotonically towards the apex")
	_check(maxf(max_step, final_step) < 0.06 and max_growth < 1.6,
			"smooth glide: largest step %.4f of the half-arc per 0.5 s (last %.4f), growth %.2f per step"
			% [max_step, final_step, max_growth])
	var after := holder.scale_target(events, now + 61.0)
	_check(after != first and absf(TimelineWindow.arc_position(4.0 * 86400.0 - 61.0, after) - edge) < 1e-9,
			"after it passed, a new scale for the next three")
	var moved := events.duplicate()
	moved[2] = now + 160.0 + 0.5 * TimelineWindow.RESCALE_TOLERANCE * first.y
	holder.reset()
	holder.scale_target(events, now)
	_check(holder.scale_target(moved, now + 1.0) == first, "a small prediction change keeps the scale")
	moved[2] = now + 160.0 + 2.0 * TimelineWindow.RESCALE_TOLERANCE * first.y
	_check(holder.scale_target(moved, now + 1.0) != first, "a prediction moving by more than the tolerance rescales")


func _test_labels() -> void:
	var now := 1.0e9
	var events: Array = []
	var kinds := ["ignition", "cutoff", "max_q", "meco", "docking"]
	for i in 60:
		events.append({"kind": kinds[i % kinds.size()], "tdb_s": now + (i - 30) * 3600.0})
	var scale := Vector2(3600.0, 30.0 * 3600.0)
	var shown := TimelineWindow.visible_labels(events, now, scale, 1000.0)
	_check(shown.size() < events.size() and shown.size() > 0, "60 events thinned (%d shown)" % shown.size())
	var ok := true
	for i in range(1, shown.size()):
		var a := TimelineWindow.arc_position(float(shown[i - 1]["tdb_s"]) - now, scale)
		var b := TimelineWindow.arc_position(float(shown[i]["tdb_s"]) - now, scale)
		ok = ok and 0.5 * (b - a) * 1000.0 >= TimelineWindow.MIN_LABEL_PX
	_check(ok, "no two labels closer than 70 px on the arc")

	var linear := Vector2(1.0e6, 100.0)  # knee >> reach: nearly linear
	var pair := [{"kind": "ignition", "tdb_s": now + 10.0}, {"kind": "meco", "tdb_s": now + 12.0}]
	shown = TimelineWindow.visible_labels(pair, now, linear, 500.0)
	_check(shown.size() == 1 and shown[0]["kind"] == "meco", "higher priority kind survives")
	pair = [{"kind": "max_q", "tdb_s": now + 10.0}, {"kind": "max_q", "tdb_s": now + 12.0}]
	shown = TimelineWindow.visible_labels(pair, now, linear, 500.0)
	_check(shown.size() == 1 and shown[0]["tdb_s"] == now + 10.0, "equal priority keeps the earlier event")
	shown = TimelineWindow.visible_labels([{"kind": "max_q", "tdb_s": now + 500.0}], now, linear, 500.0)
	_check(shown.is_empty(), "events off the arc are dropped")


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
