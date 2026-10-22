class_name GaugeRules
extends RefCounted
## Which gauges of the webcast overlay are shown for a body. Pure rules, no nodes.
## `state` is the caller-owned hysteresis memory: gauge id -> last time (s) its condition was true.

## A gauge hides only after its condition has been false for this long (s). [E] readability
const HIDE_DELAY_S := 5.0
const MAX_PER_SIDE := 4
## Dynamic pressure above which the gauge is useful (Pa). [E]
const DYNAMIC_PRESSURE_MIN_PA := 100.0
## Free-molecular edge: Mach is meaningless above this Knudsen number (same rule as flight_hud.gd). [D]
const KNUDSEN_CONTINUUM_MAX := 0.1
## Aerodynamic heating worth showing (W/m^2). [E]
const HEAT_RATE_MIN_W_M2 := 1.0e3
## Vertical speed is shown in the atmosphere / near the ground only (m). [E]
const VERTICAL_SPEED_MAX_ALTITUDE_M := 150.0e3
## Orbit apsides are shown above this altitude (m). [E]
const ORBIT_MIN_ALTITUDE_M := 100.0e3


## Returns {"left": Array[String], "right": Array[String]}.
## Left (vehicle) priority: engines, propellant, heat_flux, heat_load, g_load, mach, dynamic_pressure.
## Right (trajectory) priority depends on the situation: docking [range, closing_rate, altitude, speed_inertial];
## orbit [speed_inertial, altitude, apoapsis, periapsis]; otherwise [speed_surface, altitude, vertical_speed,
## speed_inertial]. Each gauge also needs its own condition; at most MAX_PER_SIDE per side.
## `body` = {"engines": Array, "stages": Array, "docking": Dictionary}; "docking" is the docking state of this body
## (empty when the session's docking state belongs to another body).
static func select(telemetry: Dictionary, body: Dictionary, state: Dictionary, now_s: float) -> Dictionary:
	var left: Array[String] = _pick(_left_conditions(telemetry, body), state, now_s)
	var right: Array[String] = _pick(_right_conditions(telemetry, body), state, now_s)
	return {"left": left, "right": right}


## Docking state counts as active for the body when it is present, reports a range to the port and the
## vehicles are not yet docked (after docking there is no closing rate).
static func docking_active(docking: Dictionary) -> bool:
	return docking.get("present", false) and docking.has("range_m") and not docking.get("docked", false)


static func _left_conditions(t: Dictionary, body: Dictionary) -> Array:
	var heat: bool = float(t.get("heat_rate_w_m2", 0.0)) > HEAT_RATE_MIN_W_M2
	var engines := main_engines(body.get("engines", []))
	var firing: bool = engines.any(func(e: Dictionary) -> bool: return e.get("firing", false))
	var free: bool = not t.get("clamped", false)
	var aero: bool = float(t.get("dynamic_pressure_pa", 0.0)) > DYNAMIC_PRESSURE_MIN_PA
	return [
		["engines", not engines.is_empty()],
		["propellant", float(t.get("propellant_capacity_kg", 0.0)) > 0.0],
		["heat_flux", heat],  # heating before g / q / Mach: it must not be cut during entry by the side limit
		["heat_load", heat],
		["g_load", free and (firing or aero)],  # no constant 0 g on the pad or in free fall
		["mach", free and aero and float(t.get("knudsen", INF)) < KNUDSEN_CONTINUUM_MAX],
		["dynamic_pressure", aero],  # after Mach: with engines and propellant an ascent shows g and Mach (fix 1)
	]


## The engines of main propulsion: launch escape (Crew Dragon SuperDracos, `abort_only`) and landing thrusters
## (`landing_only`) are flagged `auxiliary` by FlightSim.body_engines and are not part of the engine gauge.
static func main_engines(engines: Array) -> Array:
	return engines.filter(func(e: Dictionary) -> bool: return not e.get("auxiliary", false))


## The right side depends on the flight situation: docking approach, orbit, or ascent / entry / landing / pad.
static func _right_conditions(t: Dictionary, body: Dictionary) -> Array:
	var altitude_m := float(t.get("altitude_m", 0.0))
	var docking := docking_active(body.get("docking", {}))
	var in_orbit := altitude_m > ORBIT_MIN_ALTITUDE_M and is_finite(float(t.get("apoapsis_altitude_km", INF)))
	var conditions := {
		"speed_surface": true,
		"speed_inertial": true,
		"altitude": true,
		"vertical_speed": altitude_m < VERTICAL_SPEED_MAX_ALTITUDE_M,
		"apoapsis": in_orbit,
		"periapsis": in_orbit,
		"range": docking,
		"closing_rate": docking,
	}
	var ids: Array[String]
	if docking:
		ids = ["range", "closing_rate", "altitude", "speed_inertial"]
	elif in_orbit:
		ids = ["speed_inertial", "altitude", "apoapsis", "periapsis"]
	else:
		ids = ["speed_surface", "altitude", "vertical_speed", "speed_inertial"]
	var result: Array = []
	for id in ids:
		result.append([id, conditions[id]])
	return result


## Applies the hysteresis and the per-side limit; `conditions` is in priority order.
static func _pick(conditions: Array, state: Dictionary, now_s: float) -> Array[String]:
	var shown: Array[String] = []
	for entry in conditions:
		var id: String = entry[0]
		if entry[1]:
			state[id] = now_s
		if state.has(id) and now_s - float(state[id]) < HIDE_DELAY_S and shown.size() < MAX_PER_SIDE:
			shown.append(id)
	return shown
