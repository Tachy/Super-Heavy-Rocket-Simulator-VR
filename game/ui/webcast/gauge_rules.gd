class_name GaugeRules
extends RefCounted
## Which gauges of the webcast overlay are shown for a body. Pure rules, no nodes. The gauge lists, thresholds and
## each gauge's condition name come from GaugeConfig (config/ui/webcast_gauges.toml); the conditions themselves are
## computed here. `state` is the caller-owned hysteresis memory: gauge id -> last time (s) its condition was true.

## The condition names the TOML may use (see `_conditions`).
const CONDITIONS: Array[String] = ["always", "has_engines", "has_propellant", "heating", "powered_or_aero",
		"continuum_aero", "aero", "has_moon", "low_altitude", "in_orbit", "docking"]
## Key of the transfer situation in the hysteresis `state` (not a gauge id).
const TRANSFER_STATE_KEY := "situation:transfer"


## Returns {"left": Array[String], "right": Array[String]}.
## Left (vehicle) priority comes from the config's `left` list. The right (trajectory) list depends on the situation:
## docking, else transfer, else orbit, else default. Each gauge also needs its own condition; at most
## `max_per_side` per side. `config` null = GaugeConfig.shared().
## `body` = {"engines": Array, "stages": Array, "docking": Dictionary}; "docking" is the docking state of this body
## (empty when the session's docking state belongs to another body).
static func select(telemetry: Dictionary, body: Dictionary, state: Dictionary, now_s: float,
		config: GaugeConfig = null) -> Dictionary:
	if config == null:
		config = GaugeConfig.shared()
	var conditions := _conditions(telemetry, body, state, config)  # also keeps the transfer hysteresis current
	var situation := "default"
	if conditions["docking"]:
		situation = "docking"
	elif state.get(TRANSFER_STATE_KEY, false):
		situation = "transfer"
	elif conditions["in_orbit"]:
		situation = "orbit"
	var right_ids: Array = config.situations.get(situation, [])
	return {
		"left": _pick(config.left, conditions, config, state, now_s),
		"right": _pick(right_ids, conditions, config, state, now_s),
	}


## Docking state counts as active for the body when it is present, reports a range to the port and the
## vehicles are not yet docked (after docking there is no closing rate).
static func docking_active(docking: Dictionary) -> bool:
	return docking.get("present", false) and docking.has("range_m") and not docking.get("docked", false)


## The engines of main propulsion: launch escape (Crew Dragon SuperDracos, `abort_only`) and landing thrusters
## (`landing_only`) are flagged `auxiliary` by FlightSim.body_engines and are not part of the engine gauge.
static func main_engines(engines: Array) -> Array:
	return engines.filter(func(e: Dictionary) -> bool: return not e.get("auxiliary", false))


## Coasting between Earth and Moon: high above the reference body on an open orbit or one reaching far out (see the
## `transfer_*` thresholds). Remembers the result in `state` for its hysteresis.
static func in_transfer(t: Dictionary, state: Dictionary, config: GaugeConfig = null) -> bool:
	if config == null:
		config = GaugeConfig.shared()
	var factor := config.threshold("transfer_exit_factor") if state.get(TRANSFER_STATE_KEY, false) else 1.0
	var apoapsis_km := float(t.get("apoapsis_altitude_km", INF))
	var transfer := float(t.get("altitude_m", 0.0)) > config.threshold("transfer_min_altitude_m") * factor \
			and (not is_finite(apoapsis_km) or apoapsis_km > config.threshold("transfer_min_apoapsis_km") * factor)
	state[TRANSFER_STATE_KEY] = transfer
	return transfer


## Every named condition (CONDITIONS) of the telemetry / body: name -> bool.
static func _conditions(t: Dictionary, body: Dictionary, state: Dictionary, config: GaugeConfig) -> Dictionary:
	var altitude_m := float(t.get("altitude_m", 0.0))
	var engines := main_engines(body.get("engines", []))
	var firing: bool = engines.any(func(e: Dictionary) -> bool: return e.get("firing", false))
	var free: bool = not t.get("clamped", false)
	var aero: bool = float(t.get("dynamic_pressure_pa", 0.0)) > config.threshold("dynamic_pressure_min_pa")
	in_transfer(t, state, config)  # evaluated every frame: keeps its hysteresis state current
	return {
		"always": true,
		"has_engines": not engines.is_empty(),
		"has_propellant": float(t.get("propellant_capacity_kg", 0.0)) > 0.0,
		"heating": float(t.get("heat_rate_w_m2", 0.0)) > config.threshold("heat_rate_min_w_m2"),
		"powered_or_aero": free and (firing or aero),  # no constant 0 g on the pad or in free fall
		"continuum_aero": free and aero and float(t.get("knudsen", INF)) < config.threshold("knudsen_continuum_max"),
		"aero": aero,
		"has_moon": t.has("distance_moon_m"),  # only sessions with the Moon report it
		"low_altitude": altitude_m < config.threshold("vertical_speed_max_altitude_m"),
		"in_orbit": altitude_m > config.threshold("orbit_min_altitude_m") \
				and is_finite(float(t.get("apoapsis_altitude_km", INF))),
		"docking": docking_active(body.get("docking", {})),
	}


## Applies the hysteresis and the per-side limit; `ids` is in priority order.
static func _pick(ids: Array, conditions: Dictionary, config: GaugeConfig, state: Dictionary, now_s: float) \
		-> Array[String]:
	var shown: Array[String] = []
	for id: String in ids:
		if conditions[config.gauges[id]["condition"]]:
			state[id] = now_s
		if state.has(id) and now_s - float(state[id]) < config.hide_delay_s and shown.size() < config.max_per_side:
			shown.append(id)
	return shown
