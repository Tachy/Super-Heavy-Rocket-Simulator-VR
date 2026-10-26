class_name WebcastOverlay
extends Control
## SpaceX-webcast-style overlay at the 1600x900 reference layout (rendered by OverlayHost into a transparent
## SubViewport): round gauges of the viewed object bottom left (vehicle) and bottom right (trajectory), chosen by
## GaugeRules, with the object's display name under them; the mission timeline arc bottom centre (whole mission, all
## bodies) with the mission clock and mission name under its apex; the UTC on top, the camera's local time under it
## (LocalTime, only within 100 km of the Earth's surface). Reads world state only.

const SIZE := Vector2(1600.0, 900.0)
const GAUGE_Y_PX := 800.0
const GAUGE_SPACING_PX := 106.0
const OUTER_MARGIN_PX := 76.0       ## screen edge to the outermost gauge's centre
const ARC_APEX_Y_PX := 770.0        ## apex of the timeline arc ("now"); the mission clock hangs below it
const GROUP_NAME_BASELINE_PX := 888.0
const GROUP_NAME_SIZE := 15
const GROUP_NAME_ALPHA := 0.9

var world: World

var top_bar := TopBar.new()
var timeline_arc := TimelineArc.new()
var mission_clock := MissionClock.new()
var engine_gauge := EngineGauge.new()
var gauges := {}  ## gauge id -> RoundGauge
var group_name := ""  ## display name of the viewed object, under its gauges
var group_name_x := 0.0

var _rule_state := {}
var _serial := -1
var _log_scale := Vector2.ZERO  ## log of the shown timeline scale (knee_s, reach_s): eased in log space
var _scale_t0: Variant = null   ## T-0 the scale belongs to
var _scale_target := TimelineWindow.new()  ## the scale chosen for the current set of next events
var _last_now := 0.0
var _snap := true


func _init() -> void:
	size = SIZE
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	top_bar.position = Vector2(SIZE.x * 0.5, 0.0)
	add_child(top_bar)
	timeline_arc.position = Vector2(SIZE.x * 0.5, ARC_APEX_Y_PX)
	add_child(timeline_arc)
	mission_clock.position = timeline_arc.position
	add_child(mission_clock)
	add_child(engine_gauge)
	var specs := GaugeConfig.shared().gauges  ## gauge specs: see config/ui/webcast_gauges.toml
	for id: String in specs:
		if id == "engines":  # drawn by the engine gauge
			continue
		var gauge := RoundGauge.new()
		gauge.label = specs[id]["label"]
		gauge.unit = specs[id]["unit"]
		gauge.signed = specs[id].get("signed", false)
		add_child(gauge)
		gauges[id] = gauge
	_hide_body_gauges()


## Shows the timeline scale at its target at once (no easing), e.g. after a time jump.
func snap_timeline() -> void:
	_snap = true


func _process(delta: float) -> void:
	FrameProfiler.begin(&"ui_webcast")
	_process_unprofiled(delta)
	FrameProfiler.end(&"ui_webcast")


func _process_unprofiled(delta: float) -> void:
	if world == null or not world.core.is_open():
		visible = false
		return
	visible = true
	var sim := world.flight.sim
	var now := world.clock.tdb_s
	var active := sim.is_active()
	var timeline: Dictionary = sim.mission_timeline() if active else {}
	top_bar.show_utc(TopBar.format_utc(world.core.tdb_s_to_utc(now)))
	top_bar.show_local_time(LocalTime.shared().camera_line(world.core, world.camera.position_gcrf_m, now))
	top_bar.show_weather(world.weather.status_text())
	var index := world.view_target.resolve(sim) if active and world.focus == World.FOCUS_VEHICLE else -1
	var mission := sim.mission_display_name() if active else ""
	if mission == "" and active:  # no mission name configured: the viewed (else the controlled) vehicle's
		mission = sim.body_display_name(index if index >= 0 else sim.controlled_index())
	if sim.autopilot_status().get("planning", false):  # also a mission start planned before its vehicle is placed
		mission = FlightController.PLANNING_EVENT  # the countdown holds while the onboard computer plans
	show_clock(timeline, now, mission)
	_update_timeline(timeline, now, delta)
	if index < 0:
		show_gauges({"left": [], "right": []}, {}, [], [], {}, "")
		return
	var serial := sim.body_serial(index)
	if serial != _serial:  # another object: its own gauges, no hysteresis carried over
		_serial = serial
		_rule_state.clear()
	var telemetry: Dictionary = sim.telemetry(index)
	var engines := GaugeRules.main_engines(sim.body_engines(index))
	var stages: Array = sim.body_stages(index)  # read once per frame
	var docking: Dictionary = sim.docking_state() if index == sim.controlled_index() else {}
	var body := {"engines": engines, "stages": stages, "docking": docking}
	var selection := GaugeRules.select(telemetry, body, _rule_state, Time.get_ticks_msec() / 1000.0)
	show_gauges(selection, telemetry, engines, stages, docking, sim.body_display_name(index))


## Mission clock from the timeline's T-0 (none without one) and the mission name under the arc's apex.
func show_clock(timeline: Dictionary, now: float, mission: String) -> void:
	var t0 = timeline.get("t0_tdb_s")
	mission_clock.show_values(MissionClock.format_mission_clock(now - float(t0)) if t0 != null else "", mission)
	timeline_arc.keep_out = mission_clock.occupied_rect()  # same position (the apex): arc labels stay off the clock


## Shows exactly the selected gauges ({"left": ids, "right": ids}) of a body, and its name under them.
func show_gauges(selection: Dictionary, telemetry: Dictionary, engines: Array, stages: Array, docking: Dictionary,
		body_name: String) -> void:
	_hide_body_gauges()
	var left: Array = selection["left"]
	var right: Array = selection["right"]
	var x := OUTER_MARGIN_PX
	for id: String in left:
		if id == "engines":
			engine_gauge.visible = true
			engine_gauge.position = Vector2(x, GAUGE_Y_PX)
			engine_gauge.show_stage(engines, stages)
		else:
			_place(id, x, telemetry, docking)
		x += GAUGE_SPACING_PX
	var right_start := SIZE.x - OUTER_MARGIN_PX - GAUGE_SPACING_PX * (right.size() - 1)
	x = right_start
	for id: String in right:
		_place(id, x, telemetry, docking)
		x += GAUGE_SPACING_PX
	# One name per body: under its vehicle gauges (left), else under its trajectory gauges.
	var name_x := right_start + 0.5 * GAUGE_SPACING_PX * (right.size() - 1)
	if not left.is_empty():
		name_x = OUTER_MARGIN_PX + 0.5 * GAUGE_SPACING_PX * (left.size() - 1)
	var new_name := "" if left.is_empty() and right.is_empty() else body_name
	if new_name != group_name or name_x != group_name_x:
		group_name = new_name
		group_name_x = name_x
		queue_redraw()


func _draw() -> void:
	if group_name != "":
		WebcastStyle.draw_text(self, Vector2(group_name_x, GROUP_NAME_BASELINE_PX), group_name.to_upper(),
				WebcastStyle.value_font(), GROUP_NAME_SIZE, Color(1, 1, 1, GROUP_NAME_ALPHA))


func _update_timeline(timeline: Dictionary, now: float, delta: float) -> void:
	var events: Array = timeline.get("events", [])
	timeline_arc.visible = not timeline.is_empty()
	if not timeline_arc.visible:
		_snap = true
		return
	var times := PackedFloat64Array()
	for e: Dictionary in events:
		times.append(float(e["tdb_s"]))
	var t0 = timeline.get("t0_tdb_s")
	var jumped := absf(now - _last_now) > exp(_log_scale.y)  # more than the shown reach in one frame
	var fresh: bool = _snap or jumped or t0 != _scale_t0
	if fresh:
		_scale_target.reset()
	var target := _scale_target.scale_target(times, now)  # fixed while the next events stay the same
	var target_log := Vector2(log(target.x), log(target.y))
	if fresh:
		_log_scale = target_log  # first frame, a time jump or a new mission: no easing
		_scale_t0 = t0
		_snap = false
	else:
		_log_scale = TimelineWindow.ease(_log_scale, target_log, delta)
	_last_now = now
	timeline_arc.show_timeline(Vector2(exp(_log_scale.x), exp(_log_scale.y)), now, events)


func _place(id: String, x: float, telemetry: Dictionary, docking: Dictionary) -> void:
	var gauge: RoundGauge = gauges[id]
	var value := gauge_value(id, telemetry, docking)
	gauge.visible = true
	gauge.position = Vector2(x, GAUGE_Y_PX)
	gauge.show_value(RoundGauge.format_number(value, _decimals(GaugeConfig.shared().gauges[id], value)), gauge_fraction(id, value))


func _hide_body_gauges() -> void:
	engine_gauge.visible = false
	for gauge: RoundGauge in gauges.values():
		gauge.visible = false


## The gauge's value in its display unit.
static func gauge_value(id: String, t: Dictionary, docking: Dictionary) -> float:
	match id:
		"propellant":
			var capacity := float(t.get("propellant_capacity_kg", 0.0))
			return 100.0 * float(t.get("propellant_kg", 0.0)) / capacity if capacity > 0.0 else 0.0
		"speed_surface": return float(t.get("speed_relative_mps", 0.0)) * 3.6
		"speed_inertial": return float(t.get("speed_inertial_mps", 0.0)) * 3.6
		# above the ground below (the terrain, else the flat pad ground): 0 on the pad like the webcast dial; the
		# ellipsoidal altitude_m is negative there (pads lie below WGS84 on the US coasts)
		"altitude": return float(t.get("altitude_terrain_m", t.get("altitude_m", 0.0))) / 1000.0
		"vertical_speed": return float(t.get("vertical_speed_mps", 0.0))
		"apoapsis": return float(t.get("apoapsis_altitude_km", 0.0))
		"periapsis": return float(t.get("periapsis_altitude_km", 0.0))
		"distance_earth": return float(t.get("distance_earth_m", 0.0)) / 1000.0
		"distance_moon": return float(t.get("distance_moon_m", 0.0)) / 1000.0
		"speed_earth": return float(t.get("speed_earth_mps", 0.0)) * 3.6
		"speed_moon": return float(t.get("speed_moon_mps", 0.0)) * 3.6
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
	var spec: Dictionary = GaugeConfig.shared().gauges[id]
	var full := float(spec["full_scale"])
	if spec.get("signed", false):
		return clampf(value / full, -1.0, 1.0)
	var log_min := float(spec.get("log_min", 0.0))
	if log_min > 0.0:
		if value <= log_min:
			return 0.0
		return clampf(log(value / log_min) / log(full / log_min), 0.0, 1.0)
	return clampf(value / full, 0.0, 1.0)


## Fewer decimals for larger values, so the number stays short.
static func _decimals(spec: Dictionary, value: float) -> int:
	var decimals := int(spec["decimals"])
	var magnitude := absf(value)
	if magnitude >= 100.0:
		return 0
	if magnitude >= 10.0:
		return mini(decimals, 1)
	return decimals
