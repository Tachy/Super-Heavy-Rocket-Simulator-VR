class_name FlightHud
extends PanelContainer
## Telemetry of the controlled vehicle (top right) plus the pilot's key reference.

const KEY_HELP := "I Zünden · L Klammern lösen · K Aus · G Stufentrennung · M Steuermodus\n" \
		+ "W/S Nicken · A/D Gieren · Q/E Rollen · Shift/Strg Schub (oder HOTAS)\n" \
		+ "P Aufstiegs-Autopilot · O Zirkularisieren an Apoapsis · X Autopilot aus"

var world: World
var _label := Label.new()


func _ready() -> void:
	_label.add_theme_font_size_override("font_size", 15)
	add_child(_label)


func _process(_delta: float) -> void:
	var sim := world.flight.sim
	visible = sim.is_active()
	if not visible:
		return
	var index := sim.controlled_index()
	var t: Dictionary = sim.telemetry(index)
	var mission := sim.mission_time_s()
	var lines := PackedStringArray()
	var clock_prefix := "Rampe " if t["clamped"] else "T+"
	lines.append("%s%02d:%02d   %s" % [clock_prefix, int(absf(mission)) / 60, int(absf(mission)) % 60,
			sim.body_stages(index).map(func(s: Dictionary) -> String: return s["name"])[-1] if sim.body_count() > 0 else ""])
	lines.append("Höhe            %9.2f km" % (t["altitude_m"] / 1000.0))
	lines.append("Geschwindigkeit %9.0f km/h" % (t["speed_relative_mps"] * 3.6))
	lines.append("  inertial      %9.0f m/s" % t["speed_inertial_mps"])
	lines.append("  vertikal      %9.0f m/s" % t["vertical_speed_mps"])
	lines.append("Bahnwinkel      %9.1f °" % t["flight_path_angle_deg"])
	# Mach only means something in continuum flow; above ~90 km the gas is rarefied.
	# Above Kn ~ 0.1 the continuum assumption (and with it the Mach number) no longer holds.
	if t["knudsen"] < 0.1:
		lines.append("Mach            %9.2f" % t["mach"])
	elif t["knudsen"] < 10.0:
		lines.append("Mach                 –   (Übergangsströmung, Kn %.2f)" % t["knudsen"])
	else:
		lines.append("Mach                 –   (freie Molekularströmung)")
	lines.append("Staudruck       %9.1f kPa" % (t["dynamic_pressure_pa"] / 1000.0))
	lines.append("Anstellwinkel   %9.1f °" % t["angle_of_attack_deg"])
	lines.append("Beschleunigung  %9.2f g" % t["sensed_acceleration_g"])
	lines.append("Schub           %9.1f MN" % (t["thrust_n"] / 1.0e6))
	lines.append("Masse           %9.0f t" % (t["mass_kg"] / 1000.0))
	lines.append("Treibstoff      %9.0f t" % (t["propellant_kg"] / 1000.0))
	lines.append("Triebwerke      %9d" % t["engines_running"])
	lines.append("Schubregler     %9.0f %%%s" % [world.flight.throttle * 100.0,
			"" if world.flight.lever_armed else "  (Hebel einmal bewegen)"])
	lines.append("Steuerung       %s" % ("Ratensteuerung (SAS)" if world.flight.rate_mode else "Direkt (Gimbal)"))
	if t["clamped"]:
		lines.append("--- am Startplatz gehalten ---")
	if t["impacted"]:
		lines.append("*** Bodenkontakt ***")
	var tower: Dictionary = sim.tower_state()
	if tower.get("present", false) and sim.body_count() > 1:
		lines.append("Turm            %s" % tower["phase"])
	var ap: Dictionary = sim.autopilot_status()
	if ap.get("state", "aus") != "aus":
		lines.append("Autopilot       %s" % ap["state"])
		if ap["state"] in ["Booster", "Oberstufe"]:
			lines.append("  Phase         %s" % ap["phase"])
		if ap["state"] == "Oberstufe":
			lines.append("  Restbrennzeit %9.1f s" % ap["time_to_go_s"])
	if ap.has("burn"):
		var b: Dictionary = ap["burn"]
		var ignition: float = b["time_to_ignition_s"]
		lines.append("Manöver         Δv %.1f / %.1f m/s" % [b["remaining_dv_mps"], b["total_dv_mps"]])
		lines.append("  %s" % _dv_bar(b["remaining_dv_mps"], b["total_dv_mps"]))
		if ignition > 0.0:
			lines.append("  Zündung in    %02d:%02d  (Brenndauer %.1f s)" % [int(ignition) / 60, int(ignition) % 60, b["burn_duration_s"]])
		else:
			lines.append("  BRENNT        Rest %.1f s" % b["burn_duration_s"])
	if world.flight.last_event != "":
		lines.append("Ereignis: %s" % world.flight.last_event)
	lines.append("")
	lines.append(KEY_HELP)
	_label.text = "\n".join(lines)
	position = Vector2(get_viewport_rect().size.x - size.x - 12.0, 12.0)


static func _dv_bar(remaining: float, total: float) -> String:
	var cells := 24
	var filled := int(round(cells * clampf(remaining / maxf(total, 1e-6), 0.0, 1.0)))
	return "[" + "█".repeat(filled) + "·".repeat(cells - filled) + "]"
