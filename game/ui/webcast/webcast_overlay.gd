class_name WebcastOverlay
extends Control
## SpaceX-webcast-style overlay at the 1600x900 reference layout (rendered by OverlayHost into a transparent
## SubViewport): round gauges of the viewed object bottom left (vehicle) and bottom right (trajectory), chosen by
## GaugeRules; the mission timeline arc bottom centre (whole mission, all bodies); name, mission clock and UTC on top.
## Reads world state only.

const SIZE := Vector2(1600.0, 900.0)
const GAUGE_Y_PX := 800.0
const GAUGE_SPACING_PX := 106.0
const OUTER_MARGIN_PX := 76.0      ## screen edge to the outermost round gauge's centre
const ENGINE_GAUGE_X_PX := 100.0   ## the engine gauge is the outermost element on the left
const ENGINE_GAUGE_Y_PX := 790.0
const ENGINE_TO_GAUGE_PX := 148.0  ## engine gauge centre to the first round gauge's centre
const ARC_TOP_Y_PX := 765.0        ## apex of the timeline arc

## Gauge specs: label (German), unit, full scale of the arc, log_min > 0 = logarithmic arc from log_min to full scale,
## maximum decimals (fewer for large values), signed = the arc fills left/right from the top.
const GAUGES := {
	"speed_surface": {"label": "Geschwindigkeit", "unit": "km/h", "full_scale": 30000.0, "decimals": 0},
	"speed_inertial": {"label": "Inertial", "unit": "km/h", "full_scale": 40000.0, "decimals": 0},
	"altitude": {"label": "Höhe", "unit": "km", "full_scale": 400000.0, "log_min": 0.1, "decimals": 1},
	"vertical_speed": {"label": "Vertikal", "unit": "m/s", "full_scale": 2000.0, "decimals": 1, "signed": true},
	"apoapsis": {"label": "Apoapsis", "unit": "km", "full_scale": 400000.0, "log_min": 10.0, "decimals": 0},
	"periapsis": {"label": "Periapsis", "unit": "km", "full_scale": 400000.0, "log_min": 10.0, "decimals": 0},
	"range": {"label": "Abstand", "unit": "m", "full_scale": 1.0e6, "log_min": 0.1, "decimals": 1},
	"closing_rate": {"label": "Annäherung", "unit": "m/s", "full_scale": 1.0, "decimals": 2, "signed": true},
	"g_load": {"label": "Beschleunigung", "unit": "g", "full_scale": 6.0, "decimals": 2},
	"dynamic_pressure": {"label": "Staudruck", "unit": "kPa", "full_scale": 40.0, "decimals": 1},
	"mach": {"label": "Mach", "unit": "", "full_scale": 30.0, "decimals": 2},
	"heat_flux": {"label": "Wärmestrom", "unit": "kW/m²", "full_scale": 2000.0, "decimals": 0},
	"heat_load": {"label": "Wärmelast", "unit": "MJ/m²", "full_scale": 1000.0, "decimals": 1},
}

var world: World

var top_bar := TopBar.new()
var timeline_arc := TimelineArc.new()
var engine_gauge := EngineGauge.new()
var gauges := {}  ## gauge id -> RoundGauge

var _rule_state := {}
var _serial := -1
var _window := Vector2.ZERO
var _window_t0: Variant = null  ## T-0 the window belongs to
var _snap := true


func _init() -> void:
	size = SIZE
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	top_bar.position = Vector2(SIZE.x * 0.5, 0.0)
	add_child(top_bar)
	timeline_arc.position = Vector2(SIZE.x * 0.5, ARC_TOP_Y_PX)
	add_child(timeline_arc)
	add_child(engine_gauge)
	for id: String in GAUGES:
		var gauge := RoundGauge.new()
		gauge.label = GAUGES[id]["label"]
		gauge.unit = GAUGES[id]["unit"]
		gauge.signed = GAUGES[id].get("signed", false)
		add_child(gauge)
		gauges[id] = gauge
	_hide_body_gauges()


## Shows the timeline window at its target at once (no easing), e.g. after a time jump.
func snap_timeline() -> void:
	_snap = true


func _process(delta: float) -> void:
	if world == null or not world.core.is_open():
		visible = false
		return
	visible = true
	var sim := world.flight.sim
	var now := world.clock.tdb_s
	var active := sim.is_active()
	var timeline: Dictionary = sim.mission_timeline() if active else {}
	var t0 = timeline.get("t0_tdb_s")
	var index := world.view_target.resolve(sim) if active and world.focus == World.FOCUS_VEHICLE else -1
	var viewed := object_name(sim.body_stages(index)) if index >= 0 else ""
	top_bar.show_values(viewed, TopBar.format_mission_clock(now - float(t0)) if t0 != null else "",
			TopBar.format_utc(world.core.tdb_s_to_utc(now)))
	_update_timeline(timeline, now, t0, delta)
	if index < 0:
		_hide_body_gauges()
		return
	_update_gauges(sim, index)


func _update_timeline(timeline: Dictionary, now: float, t0: Variant, delta: float) -> void:
	var events: Array = timeline.get("events", [])
	timeline_arc.visible = not timeline.is_empty()
	if not timeline_arc.visible:
		_snap = true
		return
	var times := PackedFloat64Array()
	for e: Dictionary in events:
		times.append(float(e["tdb_s"]))
	var target := TimelineWindow.target_span(times, now)
	if _snap or now < _window.x or now > _window.y or t0 != _window_t0:
		_window = target  # first frame, a time jump or a new mission: no easing
		_window_t0 = t0
		_snap = false
	else:
		_window = TimelineWindow.ease(_window, target, delta)
	timeline_arc.show_timeline(_window, now, events)


func _update_gauges(sim: FlightSim, index: int) -> void:
	var serial := sim.body_serial(index)
	if serial != _serial:  # another object: its own gauges, no hysteresis carried over
		_serial = serial
		_rule_state.clear()
	var telemetry: Dictionary = sim.telemetry(index)
	var engines: Array = sim.body_engines(index)
	var stages: Array = sim.body_stages(index)
	var docking: Dictionary = sim.docking_state() if index == sim.controlled_index() else {}
	var body := {"engines": engines, "stages": stages, "docking": docking}
	var selection := GaugeRules.select(telemetry, body, _rule_state, Time.get_ticks_msec() / 1000.0)
	_hide_body_gauges()
	var left: Array[String] = selection["left"]
	var x := OUTER_MARGIN_PX
	if left.has("engines"):
		engine_gauge.visible = true
		engine_gauge.position = Vector2(ENGINE_GAUGE_X_PX, ENGINE_GAUGE_Y_PX)
		engine_gauge.show_stage(engines, stages)
		x = ENGINE_GAUGE_X_PX + ENGINE_TO_GAUGE_PX
	for id in left:
		if id != "engines":
			_place(id, x, telemetry, docking)
			x += GAUGE_SPACING_PX
	var right: Array[String] = selection["right"]
	x = SIZE.x - OUTER_MARGIN_PX - GAUGE_SPACING_PX * (right.size() - 1)
	for id in right:
		_place(id, x, telemetry, docking)
		x += GAUGE_SPACING_PX


func _place(id: String, x: float, telemetry: Dictionary, docking: Dictionary) -> void:
	var gauge: RoundGauge = gauges[id]
	var value := gauge_value(id, telemetry, docking)
	gauge.visible = true
	gauge.position = Vector2(x, GAUGE_Y_PX)
	gauge.show_value(RoundGauge.format_number(value, _decimals(GAUGES[id], value)), gauge_fraction(id, value))


func _hide_body_gauges() -> void:
	engine_gauge.visible = false
	for gauge: RoundGauge in gauges.values():
		gauge.visible = false


## The gauge's value in its display unit.
static func gauge_value(id: String, t: Dictionary, docking: Dictionary) -> float:
	match id:
		"speed_surface": return float(t.get("speed_relative_mps", 0.0)) * 3.6
		"speed_inertial": return float(t.get("speed_inertial_mps", 0.0)) * 3.6
		"altitude": return float(t.get("altitude_m", 0.0)) / 1000.0
		"vertical_speed": return float(t.get("vertical_speed_mps", 0.0))
		"apoapsis": return float(t.get("apoapsis_altitude_km", 0.0))
		"periapsis": return float(t.get("periapsis_altitude_km", 0.0))
		"range": return float(docking.get("range_m", 0.0))
		"closing_rate": return -float(docking.get("range_rate_mps", 0.0))
		"g_load": return float(t.get("sensed_acceleration_g", 0.0))
		"dynamic_pressure": return float(t.get("dynamic_pressure_pa", 0.0)) / 1000.0
		"mach": return float(t.get("mach", 0.0))
		"heat_flux": return float(t.get("heat_rate_w_m2", 0.0)) / 1000.0
		"heat_load": return float(t.get("heat_load_j_m2", 0.0)) / 1.0e6
	return 0.0


## Arc fill of the gauge: 0..1, signed gauges -1..1.
static func gauge_fraction(id: String, value: float) -> float:
	var spec: Dictionary = GAUGES[id]
	var full := float(spec["full_scale"])
	if spec.get("signed", false):
		return clampf(value / full, -1.0, 1.0)
	var log_min := float(spec.get("log_min", 0.0))
	if log_min > 0.0:
		if value <= log_min:
			return 0.0
		return clampf(log(value / log_min) / log(full / log_min), 0.0, 1.0)
	return clampf(value / full, 0.0, 1.0)


## Name of the viewed object: its bottom and top stage ("Super Heavy Block 3 + Starship Block 3"); stages in between
## are left out so that a long stack (Falcon 9 with Crew Dragon and its trunk) still fits on one line.
static func object_name(stages: Array) -> String:
	if stages.is_empty():
		return ""
	var bottom := str(stages[0].get("name", ""))
	return bottom if stages.size() == 1 else "%s + %s" % [bottom, str(stages[-1].get("name", ""))]


## Fewer decimals for larger values, so the number stays short.
static func _decimals(spec: Dictionary, value: float) -> int:
	var decimals := int(spec["decimals"])
	var magnitude := absf(value)
	if magnitude >= 100.0:
		return 0
	if magnitude >= 10.0:
		return mini(decimals, 1)
	return decimals
