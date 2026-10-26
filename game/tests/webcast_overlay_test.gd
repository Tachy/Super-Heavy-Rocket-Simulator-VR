# Headless test of the webcast overlay widgets' pure helpers (formatting, arc fade, engine layout, gauge values).
# Run: godot --headless --path game --script res://tests/webcast_overlay_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	_test_clock()
	_test_numbers()
	_test_fade()
	_test_engines()
	_test_values()
	_test_arc()
	_test_label_layout()
	_test_overlay()
	_test_host()
	_test_vr_pointer()
	print("WEBCAST OVERLAY TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


## Arc labels near "now" must not cover the mission clock and name under the apex (Task 4c follow-up).
func _test_label_layout() -> void:
	var font := WebcastStyle.label_font()
	var clock := MissionClock.text_rect("T+ 1 d 00:00:22", "Rückkehr vom Mond (LLO)", WebcastStyle.value_font(), font)
	_check(clock.has_area() and clock.position.y > 0.0, "clock area lies below the apex")
	var free := true
	var moved_up := false
	var both_sides := [false, false]
	for i in range(-40, 41):
		var f := 0.5 + 0.005 * i  # events from just left to just right of the apex
		for prefer_below: bool in [true, false]:
			var side := TimelineArc.label_side("Zündung", f, prefer_below, font, clock)
			if side == TimelineArc.SIDE_NONE:
				continue
			free = free and not TimelineArc.label_rect("Zündung", f, side == TimelineArc.SIDE_BELOW, font).intersects(clock)
			if prefer_below and side == TimelineArc.SIDE_ABOVE:
				moved_up = true
				both_sides[0 if f < 0.5 else 1] = true
	_check(free, "no placed label covers the clock")
	_check(moved_up and both_sides[0] and both_sides[1], "labels below the line near the apex move above, both sides")
	_check(TimelineArc.label_side("Zündung", 0.1, true, font, clock) == TimelineArc.SIDE_BELOW,
			"far from the apex a label keeps its side")
	_check(TimelineArc.label_side("Zündung", 0.5, true, font, Rect2()) == TimelineArc.SIDE_BELOW,
			"no clock: the alternation is kept")
	var everywhere := Rect2(-2000.0, -2000.0, 4000.0, 4000.0)
	_check(TimelineArc.label_side("Zündung", 0.5, true, font, everywhere) == TimelineArc.SIDE_NONE,
			"both sides colliding: the label is left out")


func _test_clock() -> void:
	_check(MissionClock.format_mission_clock(-41.3) == "T- 00:00:42", "T- counts down to zero (%s)" % MissionClock.format_mission_clock(-41.3))
	_check(MissionClock.format_mission_clock(0.0) == "T+ 00:00:00", "T-0 is T+ 00:00:00")
	_check(MissionClock.format_mission_clock(75.9) == "T+ 00:01:15", "T+ 75.9 s (%s)" % MissionClock.format_mission_clock(75.9))
	var days := 3.0 * 86400.0 + 4.0 * 3600.0 + 12.0 * 60.0 + 9.0
	_check(MissionClock.format_mission_clock(days) == "T+ 3 d 04:12:09", "days (%s)" % MissionClock.format_mission_clock(days))
	_check(MissionClock.format_mission_clock(-days - 0.5) == "T- 3 d 04:12:10", "long wait before T-0 (%s)"
			% MissionClock.format_mission_clock(-days - 0.5))
	_check(TopBar.format_utc("2024-04-08T18:42:07.512") == "2024-04-08 18:42:07 UTC", "UTC line")


func _test_numbers() -> void:
	_check(RoundGauge.format_number(12345.6, 0) == "12.346", "thousands separator (%s)" % RoundGauge.format_number(12345.6, 0))
	_check(RoundGauge.format_number(1.254, 2) == "1,25", "decimal comma (%s)" % RoundGauge.format_number(1.254, 2))
	_check(RoundGauge.format_number(-1234567.0, 1) == "-1.234.567,0", "negative, grouped (%s)"
			% RoundGauge.format_number(-1234567.0, 1))
	_check(RoundGauge.format_number(-0.04, 1) == "0,0", "no negative zero (%s)" % RoundGauge.format_number(-0.04, 1))
	# VR performance: a round gauge redraws when its text changes or its arc moves by half a pixel or more
	var gauge := RoundGauge.new()
	_check(gauge.show_value("1,0", 0.5), "round gauge: first value drawn")
	_check(not gauge.show_value("1,0", 0.5 + 0.5 * RoundGauge.FRACTION_STEP), "sub-pixel arc change: no redraw")
	_check(gauge.show_value("1,0", 0.5 + 2.0 * RoundGauge.FRACTION_STEP), "arc moves a pixel: redraw")
	_check(gauge.show_value("1,1", gauge.fraction), "text changes: redraw")
	_check(gauge.show_value("1,1", 1.0) and gauge.fraction == 1.0, "full track: redraw, exact end value")
	gauge.free()
	_check(RoundGauge.format_number(999.96, 1) == "1.000,0", "rounding carries into the grouping")


func _test_fade() -> void:
	_check(TimelineArc.fade_alpha(0.0) == 0.0 and TimelineArc.fade_alpha(1.0) == 0.0, "arc ends invisible")
	_check(TimelineArc.fade_alpha(0.5) == 1.0 and TimelineArc.fade_alpha(TimelineArc.ARC_FADE_FRACTION) == 1.0,
			"full opacity inside the fade zones")
	var quarter := TimelineArc.fade_alpha(TimelineArc.ARC_FADE_FRACTION * 0.5)
	_check(absf(quarter - 0.5) < 1e-9, "smoothstep: half opacity in the middle of the fade (%s)" % quarter)
	_check(TimelineArc.fade_alpha(0.03) < TimelineArc.fade_alpha(0.06) and
			absf(TimelineArc.fade_alpha(0.03) - TimelineArc.fade_alpha(0.97)) < 1e-9, "monotonic and symmetric")
	_check(TimelineArc.fade_alpha(-0.2) == 0.0 and TimelineArc.fade_alpha(1.3) == 0.0, "outside the arc: hidden")


func _test_engines() -> void:
	var engines := [
		{"stage": 1, "position_m": Vector3(-30.0, 0.0, 2.0), "firing": true},
		{"stage": 0, "position_m": Vector3(0.0, 3.0, 0.0), "firing": true},
		{"stage": 0, "position_m": Vector3(0.0, 0.0, -1.5), "firing": false, "vacuum": true},
	]
	_check(EngineGauge.lowest_engine_stage(engines) == 0, "lowest stage with engines")
	_check(EngineGauge.lowest_engine_stage([]) == -1, "no engines: -1")
	var dots := EngineGauge.layout(engines, 0, 4.5)
	_check(dots.size() == 2, "only the lowest stage's engines")
	_check(dots[0]["position"].is_equal_approx(Vector2(3.0, 0.0) / 4.5) and dots[0]["firing"] and not dots[0]["vacuum"],
			"body y -> right")
	_check(dots[1]["position"].is_equal_approx(Vector2(0.0, -1.5) / 4.5) and not dots[1]["firing"]
			and dots[1]["vacuum"],
			"body z -> down on the gauge (z = -1.5 m: up)")
	# VR performance: the gauge redraws only when what it shows changes, the dot radius only with the layout
	var gauge := EngineGauge.new()
	var stages := [{"diameter_m": 9.0}, {"diameter_m": 9.0}]
	_check(gauge.show_stage(engines, stages), "first show: drawn")
	var radius := gauge.dot_radius_px
	_check(not gauge.show_stage(engines.duplicate(true), stages), "same engines again: no redraw")
	var lit := engines.duplicate(true)
	lit[2]["firing"] = true
	_check(gauge.show_stage(lit, stages) and gauge.dot_radius_px == radius, "an engine lights: redraw, same dot radius")
	var moved := lit.duplicate(true)
	moved[1]["position_m"] = Vector3(0.0, 1.0, 0.0)
	_check(gauge.show_stage(moved, stages) and gauge.dot_radius_px != radius, "another layout: new dot radius")
	gauge.free()


func _test_values() -> void:
	var t := {"speed_relative_mps": 100.0, "speed_inertial_mps": 500.0, "altitude_m": 12500.0, "vertical_speed_mps": -20.0,
		"apoapsis_altitude_km": 210.0, "periapsis_altitude_km": 190.0, "sensed_acceleration_g": 1.5,
		"dynamic_pressure_pa": 31000.0, "mach": 1.2, "heat_rate_w_m2": 250000.0, "heat_load_j_m2": 3.0e7}
	var docking := {"range_m": 42.0, "range_rate_mps": -0.12}
	_check(absf(WebcastOverlay.gauge_value("speed_surface", t, docking) - 360.0) < 1e-9, "surface speed in km/h")
	_check(absf(WebcastOverlay.gauge_value("speed_inertial", t, docking) - 1800.0) < 1e-9, "inertial speed in km/h")
	_check(absf(WebcastOverlay.gauge_value("altitude", t, docking) - 12.5) < 1e-9, "altitude in km")
	# on the pad: the ellipsoidal altitude is negative (pad below WGS84), the gauge shows the height above the ground
	var pad := {"altitude_m": -21.11 + 40.0, "altitude_terrain_m": 40.0}
	_check(absf(WebcastOverlay.gauge_value("altitude", pad, docking) - 0.04) < 1e-9,
			"altitude gauge: above the ground, not above the ellipsoid")
	_check(WebcastOverlay.gauge_value("vertical_speed", t, docking) == -20.0, "vertical speed in m/s")
	_check(WebcastOverlay.gauge_value("apoapsis", t, docking) == 210.0, "apoapsis in km")
	_check(WebcastOverlay.gauge_value("periapsis", t, docking) == 190.0, "periapsis in km")
	_check(WebcastOverlay.gauge_value("g_load", t, docking) == 1.5, "g-load in g")
	_check(absf(WebcastOverlay.gauge_value("dynamic_pressure", t, docking) - 31.0) < 1e-9, "q in kPa")
	_check(WebcastOverlay.gauge_value("mach", t, docking) == 1.2, "Mach")
	_check(absf(WebcastOverlay.gauge_value("heat_flux", t, docking) - 250.0) < 1e-9, "heat flux in kW/m2")
	_check(absf(WebcastOverlay.gauge_value("heat_load", t, docking) - 30.0) < 1e-9, "heat load in MJ/m2")
	_check(WebcastOverlay.gauge_value("range", t, docking) == 42.0, "range in m")
	_check(absf(WebcastOverlay.gauge_value("closing_rate", t, docking) - 0.12) < 1e-12, "closing rate positive when closing")
	var tr := {"distance_earth_m": 250.0e6, "distance_moon_m": 140.0e6, "speed_earth_mps": 1200.0,
		"speed_moon_mps": 900.0}
	_check(absf(WebcastOverlay.gauge_value("distance_earth", tr, docking) - 250000.0) < 1e-6, "Earth distance in km")
	_check(absf(WebcastOverlay.gauge_value("distance_moon", tr, docking) - 140000.0) < 1e-6, "Moon distance in km")
	_check(absf(WebcastOverlay.gauge_value("speed_earth", tr, docking) - 4320.0) < 1e-9, "speed about the Earth in km/h")
	_check(absf(WebcastOverlay.gauge_value("speed_moon", tr, docking) - 3240.0) < 1e-9, "speed about the Moon in km/h")
	for id in GaugeRules.select({"altitude_m": 1.0e8, "apoapsis_altitude_km": INF, "distance_moon_m": 1.0}, {}, {},
			0.0)["right"]:
		_check(GaugeConfig.shared().gauges.has(id), "transfer gauge %s has a spec" % id)
	for id in GaugeConfig.shared().gauges:
		var spec: Dictionary = GaugeConfig.shared().gauges[id]
		_check(spec.has("label") and spec.has("unit") and float(spec["full_scale"]) > 0.0, "gauge spec %s" % id)
	var p := {"propellant_kg": 250.0, "propellant_capacity_kg": 1000.0}
	_check(absf(WebcastOverlay.gauge_value("propellant", p, docking) - 25.0) < 1e-12, "propellant in % of the full load")
	_check(WebcastOverlay.gauge_value("propellant", {}, docking) == 0.0, "no load: 0 %")


func _test_arc() -> void:
	var now := 1.0e9
	var events := PackedFloat64Array([now - 50.0, now + 12.0, now + 80.0, now + 400.0])
	var sc := TimelineWindow.target_scale(events, now)
	_check(TimelineArc.fraction_of(0.0, sc) == 0.5 and TimelineArc.fade_alpha(0.5) == 1.0, "now at the apex, opaque")
	_check(TimelineArc.point_of(0.5).is_equal_approx(Vector2.ZERO), "the apex is the control's position")
	var nearest := TimelineArc.fade_alpha(TimelineArc.fraction_of(12.0, sc))
	_check(nearest >= 0.9, "the nearest next event is (nearly) opaque (%.3f)" % nearest)
	_check(TimelineArc.fraction_of(12.0, sc) > 0.5 and TimelineArc.fraction_of(-50.0, sc) < 0.5,
			"future right of the apex, past left")
	var far := TimelineArc.point_of(1.0)
	_check(far.y > 0.0 and far.y < 100.0 and far.x > 250.0, "flat and wide arc (end at %s)" % str(far))
	_check(TimelineArc.label_sides(3, 3) == [false, true, false] and TimelineArc.label_sides(4, 2) == [true, false],
			"labels alternate below / above, anchored on the event index")


func _test_overlay() -> void:
	var overlay := WebcastOverlay.new()
	var t := {"altitude_m": 1000.0, "propellant_kg": 1.0, "propellant_capacity_kg": 2.0}
	var engine := {"stage": 0, "position_m": Vector3(0.0, 1.0, 0.0), "firing": true}
	overlay.show_gauges({"left": ["engines", "g_load"], "right": ["altitude"]}, t, [engine], [{"diameter_m": 9.0}], {},
			"Booster Mk3")
	var shown: Array = []
	for id in overlay.gauges:
		if overlay.gauges[id].visible:
			shown.append(id)
	_check(shown.size() == 2 and "g_load" in shown and "altitude" in shown and overlay.engine_gauge.visible,
			"exactly the selected gauges shown (%s)" % str(shown))
	_check(overlay.group_name == "Booster Mk3", "the body's display name under its gauges")
	# VR performance: showing the same selection again toggles no visibility (each toggle redraws the gauge)
	var toggles := [0]
	overlay.engine_gauge.visibility_changed.connect(func() -> void: toggles[0] += 1)
	overlay.gauges["altitude"].visibility_changed.connect(func() -> void: toggles[0] += 1)
	overlay.show_gauges({"left": ["engines", "g_load"], "right": ["altitude"]}, t, [engine], [{"diameter_m": 9.0}], {},
			"Booster Mk3")
	_check(toggles[0] == 0, "same selection again: no visibility toggles (%d)" % toggles[0])
	overlay.show_gauges({"left": [], "right": ["altitude"]}, t, [], [], {}, "Booster Mk3")
	_check(not overlay.engine_gauge.visible and not overlay.gauges["g_load"].visible, "deselected gauges hidden")
	overlay.show_gauges({"left": [], "right": []}, {}, [], [], {}, "Booster Mk3")
	_check(overlay.group_name == "", "no gauges: no name")
	overlay.show_clock({"t0_tdb_s": null, "events": []}, 100.0, "Testflug")
	_check(overlay.mission_clock.clock == "" and overlay.mission_clock.mission_name == "Testflug", "no T-0: no clock")
	overlay.show_clock({"t0_tdb_s": 25.0, "events": []}, 100.0, "Testflug")
	_check(overlay.mission_clock.clock == "T+ 00:01:15", "T+ from the timeline's T-0")
	overlay.top_bar.show_weather("Wetter: aus")
	_check(overlay.top_bar.weather == "Wetter: aus", "the weather line is kept by the top bar")
	overlay.free()


func _test_host() -> void:
	var host := OverlayHost.new()
	root.add_child(host)
	_check(host.viewport is SubViewport and host.viewport.transparent_bg, "transparent SubViewport")
	_check(host.viewport.size == Vector2i(1600, 900), "1600x900 reference size")
	_check(host.viewport.render_target_update_mode == SubViewport.UPDATE_ALWAYS, "updated every frame")
	_check(host.overlay is WebcastOverlay and host.overlay.get_parent() == host.viewport, "overlay inside the viewport")
	host.free()


## VR pointer: the arrow's tip (inset in its texture) lands on the cursor, not 1 px beside it.
func _test_vr_pointer() -> void:
	_check(VrPanelMouse.Pointer.SHAPE[0] == VrPanelMouse.Pointer.TIP, "the arrow's first point is its tip")
	var panel := VrOverlayPanel.new()
	panel.material_override = VrOverlayPanel.build_material(null)
	panel.move_pointer(Vector2(400.0, 300.0))
	var origin: Vector2 = panel.material_override.get_shader_parameter("pointer_px")
	_check(origin + VrPanelMouse.Pointer.TIP == Vector2(400.0, 300.0), "pointer tip at the cursor (origin %s)" % str(origin))
	panel.free()


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
