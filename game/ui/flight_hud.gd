class_name FlightHud
extends PanelContainer
## Telemetry of the controlled vehicle (top right) plus the pilot's key reference. Every text comes from the text
## tables (hud.* keys) and is rebuilt at 15 Hz from the stored data, so a language switch shows at once.

## Unit suffixes of the value column (symbols, the same in every language).
const UNIT_DEG := " °"  # i18n-ok: unit symbol
const UNIT_KMH := " km/h"
const UNIT_MPS := " m/s"
const UNIT_KPA := " kPa"
const UNIT_G := " g"
const UNIT_KW_M2 := " kW/m²"  # i18n-ok: unit symbol
const UNIT_MJ_M2 := " MJ/m²"  # i18n-ok: unit symbol
const UNIT_KM := " km"
const UNIT_MN := " MN"
const UNIT_T := " t"
const UNIT_S := " s"
const UNIT_PCT := " %"
const VALUE_WIDTH := 9  ## the right-aligned number field of a telemetry row
const KEY_HELP_KEYS: Array[String] = ["hud.key_help_1", "hud.key_help_2", "hud.key_help_3"]

var world: World
var _label := Label.new()


func _ready() -> void:
	_label.add_theme_font_size_override("font_size", 15)
	add_child(_label)
	I18n.events.language_changed.connect(func(_code: String) -> void: _rate.invalidate())

var _rate := UiRate.new()  ## the readout at 15 Hz (UiRate)


func _process(_delta: float) -> void:
	FrameProfiler.begin(&"ui_hud")
	_process_unprofiled(_delta)
	FrameProfiler.end(&"ui_hud")


## The key reference at the end of the HUD (one line per key), in the active language.
static func key_help() -> String:
	var lines := PackedStringArray()
	for key in KEY_HELP_KEYS:
		lines.append(I18n.text(key))
	return "\n".join(lines)


## A number in a right-aligned field of `width` characters (active decimal separator), then the unit suffix.
static func _val(value: float, decimals: int, unit := "", width := VALUE_WIDTH) -> String:
	return I18n.number(value, decimals).lpad(width) + unit


## Adds a labelled row: `label_key` is a table key, `value` the finished value text. Padded rows share one label
## column (the widest label of the active language); the others (explicit value widths) keep their natural label.
static func _row(entries: Array, label_key: String, value: String, padded := true) -> void:
	entries.append({"label": I18n.text(label_key), "value": value, "padded": padded})


## The rows laid out: the label column as wide as the widest padded label, one space, the value.
static func _render(entries: Array) -> PackedStringArray:
	var column := 0
	for entry in entries:
		if entry is Dictionary and entry["padded"]:
			column = maxi(column, (entry["label"] as String).length())
	var lines := PackedStringArray()
	for entry in entries:
		if entry is Dictionary:
			var label: String = entry["label"]
			lines.append((label.rpad(column) if entry["padded"] else label) + " " + (entry["value"] as String))
		else:
			lines.append(entry)
	return lines


func _process_unprofiled(_delta: float) -> void:
	var sim := world.flight.sim
	visible = sim.is_active()
	if not visible:
		_rate.invalidate()
		return
	if not _rate.due(_delta, world.clock.tdb_s, world.clock.warp):
		return
	var index := sim.controlled_index()
	var t: Dictionary = sim.telemetry(index)
	var mission := sim.mission_time_s()
	var entries: Array = []  ## Strings (finished lines) and rows {label, value, padded}
	var clock_key := "hud.clock_pad" if t["clamped"] else "hud.clock_met"
	var clock := "%02d:%02d" % [int(absf(mission)) / 60, int(absf(mission)) % 60]
	entries.append("%s   %s" % [I18n.text(clock_key, {"clock": clock}),
			sim.body_stages(index).map(func(s: Dictionary) -> String: return I18n.text(s["name_key"]))[-1] if sim.body_count() > 0 else ""])
	_row(entries, "hud.row.altitude_agl", _val(t["altitude_terrain_m"] / 1000.0, 2, UNIT_KM))
	_row(entries, "hud.row.altitude_wgs84", _val(t["altitude_m"] / 1000.0, 2, UNIT_KM))
	_row(entries, "hud.row.speed", _val(t["speed_relative_mps"] * 3.6, 0, UNIT_KMH))
	_row(entries, "hud.row.speed_inertial_body", _val(t["speed_inertial_mps"], 0, UNIT_MPS), false)
	_row(entries, "hud.row.speed_inertial_earth", _val(t.get("speed_earth_mps", 0.0), 0, UNIT_MPS))
	_row(entries, "hud.row.speed_vertical", _val(t["vertical_speed_mps"], 0, UNIT_MPS))
	_row(entries, "hud.row.flight_path", _val(t["flight_path_angle_deg"], 1, UNIT_DEG))
	# Mach only means something in continuum flow; above ~90 km the gas is rarefied.
	# Above Kn ~ 0.1 the continuum assumption (and with it the Mach number) no longer holds.
	if t["knudsen"] < 0.1:
		_row(entries, "hud.row.mach", _val(t["mach"], 2))
	elif t["knudsen"] < 10.0:
		_row(entries, "hud.row.mach", I18n.DASH.lpad(6) + "   " + I18n.text("hud.mach_transition", {"kn": I18n.number(t["knudsen"], 2)}))
	else:
		_row(entries, "hud.row.mach", I18n.DASH.lpad(6) + "   " + I18n.text("hud.mach_free_molecular"))
	_row(entries, "hud.row.dynamic_pressure", _val(t["dynamic_pressure_pa"] / 1000.0, 1, UNIT_KPA))
	_row(entries, "hud.row.angle_of_attack", _val(t["angle_of_attack_deg"], 1, UNIT_DEG))
	_row(entries, "hud.row.acceleration", _val(t["sensed_acceleration_g"], 2, UNIT_G))
	if t.get("heat_load_j_m2", 0.0) > 0.0:
		_row(entries, "hud.row.heat_rate", _val(t["heat_rate_w_m2"] / 1000.0, 0, UNIT_KW_M2))
		_row(entries, "hud.row.heat_load", _val(t["heat_load_j_m2"] / 1.0e6, 0, UNIT_MJ_M2))
	if t.get("skipped_out", false):
		entries.append(I18n.text("hud.skipped_out"))
	if t.get("heat_warning", false):
		entries.append(I18n.text("hud.heat_warning"))
	if t.has("entry_phase"):  # entry guidance setpoint (flight director)
		_row(entries, "hud.row.entry", I18n.phase_text(t["entry_phase"]) + (I18n.text("hud.entry_constrained") if t["entry_constrained"] else ""))
		_row(entries, "hud.row.entry_alpha", _val(t["entry_alpha_deg"], 1, UNIT_DEG, 8) + I18n.text("hud.entry_alpha_actual",
				{"alpha": I18n.number(t["angle_of_attack_deg"], 1) + UNIT_DEG}), false)
		_row(entries, "hud.row.entry_bank", _val(t["entry_bank_deg"], 1, UNIT_DEG, 7), false)
		_row(entries, "hud.row.entry_downrange", _val(t["entry_downrange_km"], 1, UNIT_KM) + I18n.text("hud.entry_crossrange",
				{"cross": I18n.number(t["entry_crossrange_km"], 1) + UNIT_KM}), false)
	_row(entries, "hud.row.thrust", _val(t["thrust_n"] / 1.0e6, 1, UNIT_MN))
	_row(entries, "hud.row.mass", _val(t["mass_kg"] / 1000.0, 0, UNIT_T))
	_row(entries, "hud.row.propellant", _val(t["propellant_kg"] / 1000.0, 0, UNIT_T))
	_row(entries, "hud.row.engines", _val(t["engines_running"], 0))
	_row(entries, "hud.row.throttle", _val(world.flight.throttle * 100.0, 0, UNIT_PCT)
			+ ("" if world.flight.lever_armed else I18n.text("hud.throttle_lever")))
	_row(entries, "hud.row.control", I18n.text("hud.control_rate" if world.flight.rate_mode else "hud.control_direct"))
	if t["clamped"]:
		entries.append(I18n.text("hud.clamped"))
	if t.get("breakup_reason", {}).get("code", "ok") != "ok":
		entries.append(I18n.text("hud.breakup", {"reason": I18n.report_text(t["breakup_reason"])}))
	elif t["impacted"]:
		entries.append(I18n.text("hud.ground_contact"))
	var tower: Dictionary = sim.tower_state()
	if tower.get("present", false) and sim.body_count() > 1:
		_row(entries, "hud.row.tower", I18n.phase_text(tower["phase"]))
	for i in sim.body_count():  # returning vehicles: the booster (body 0 after staging) and the ship's flip and catch
		var ret: Dictionary = sim.return_state(i)
		if ret.get("present", false):
			var label_key := "hud.vehicle_booster" if i == 0 and sim.body_count() > 1 else "hud.vehicle_ship"
			_row(entries, label_key, "%s (%s)" % [I18n.phase_text(ret["phase"]), I18n.phase_text(ret["guidance"])])
			if ret["abort"] and not tower.get("present", false):
				_row(entries, "hud.row.tower", I18n.text("hud.tower_abort"))
			if ret["outcome"] != "landing_outcome.flying":
				_row(entries, "hud.row.result", I18n.phase_text(ret["outcome"]))
	var ap: Dictionary = sim.autopilot_status()
	if ap.get("state", "autopilot_state.off") != "autopilot_state.off":
		_row(entries, "hud.row.autopilot", I18n.phase_text(ap["state"]))
		if ap["state"] in ["autopilot_state.booster", "autopilot_state.upper_stage"]:
			_row(entries, "hud.row.phase", I18n.phase_text(ap["phase"]))
		if ap["state"] == "autopilot_state.upper_stage":
			_row(entries, "hud.row.time_to_go", _val(ap["time_to_go_s"], 1, UNIT_S))
	if ap.has("burn"):
		var b: Dictionary = ap["burn"]
		var ignition: float = b["time_to_ignition_s"]
		_row(entries, "hud.row.maneuver", I18n.text("hud.maneuver_dv", {"remaining": I18n.number(b["remaining_dv_mps"], 1),
				"total": I18n.number(b["total_dv_mps"], 1)}))
		entries.append("  %s" % _dv_bar(b["remaining_dv_mps"], b["total_dv_mps"]))
		if ignition > 0.0:
			_row(entries, "hud.row.ignition_in", I18n.text("hud.ignition_value", {"clock": I18n.format_value(ignition, "time", "clock"),
					"burn": I18n.number(b["burn_duration_s"], 1)}))
		else:
			_row(entries, "hud.row.burning", I18n.text("hud.burning_value", {"burn": I18n.number(b["burn_duration_s"], 1)}))
	if ap.has("return") and ap["return"]["feasible"]:
		var r: Dictionary = ap["return"]
		if r["entry_in_s"] > 0.0:
			_row(entries, "hud.row.return", I18n.text("hud.return_value", {"dv": I18n.number(r["delta_v_mps"], 1),
					"prop": I18n.number(r["propellant_t"], 1), "cross": I18n.number(r["crossrange_km"], 0)}))
			_row(entries, "hud.row.entry_in", I18n.format_value(r["entry_in_s"], "time", "clock"))
			# Task 12b: main-tank residue the flaps cannot trim in the entry is vented before the interface
			if r.get("entry_vent_remaining_t", 0.0) > 0.0:
				_row(entries, "hud.row.vent", I18n.text("hud.vent_open", {"vented": I18n.number(r["entry_vented_t"], 1),
						"open": I18n.number(r["entry_vent_remaining_t"], 1)}))
			elif r.get("entry_vented_t", 0.0) > 0.0:
				_row(entries, "hud.row.vent", I18n.text("hud.vent_done", {"vented": I18n.number(r["entry_vented_t"], 1)}))
			elif r.get("entry_vent_planned_t", 0.0) > 0.0:
				_row(entries, "hud.row.vent", I18n.text("hud.vent_planned", {"planned": I18n.number(r["entry_vent_planned_t"], 1)}))
			# the pass's dispersion note concerns the deorbit and the interface: shown until the entry (Task 7 M16).
			# Also a return the mission planned (ISS undocking, capsule mission).
			if not r.get("dispersion_robust", true):
				_row(entries, "hud.row.caution", I18n.report_text(r.get("dispersion_note", {})))
	var event_line := FlightController.event_text(world.flight.last_event_report)
	if event_line != "":
		entries.append(I18n.text("hud.event", {"event": event_line}))
	var lines := _render(entries)
	lines.append("")
	lines.append(key_help())
	_label.text = "\n".join(lines)
	position = Vector2(get_viewport_rect().size.x - size.x - 12.0, 12.0)


static func _dv_bar(remaining: float, total: float) -> String:
	var cells := 24
	var filled := int(round(cells * clampf(remaining / maxf(total, 1e-6), 0.0, 1.0)))
	return "[" + "█".repeat(filled) + "·".repeat(cells - filled) + "]"  # i18n-ok: glyph bar, no words
