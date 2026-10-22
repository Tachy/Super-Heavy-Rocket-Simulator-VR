class_name ViewTarget
extends Node
## The simulated body the external view shows (and whose gauges the webcast overlay reads). Bodies are identified by
## their stable serial (indices shift when bodies appear or vanish).
## Follow mode (the default, set by a new flight): the view follows the controlled body, also across separations (after
## hot staging the ship, on Falcon 9 + Dragon the Dragon). Tab / Shift+Tab cycle through the bodies ordered by serial
## and pin the view on that serial (follow off); cycling back onto the controlled body turns follow on again, as does a
## vanished viewed body.

## [E] On a switch the camera distance is clamped to this range of the new object's length.
const MIN_DISTANCE_PER_LENGTH := 0.6
const MAX_DISTANCE_PER_LENGTH := 40.0

var serial := 0  ## serial of the viewed body (0 until the first resolve)
var follow_controlled := true  ## true: the view follows the controlled body; resolve() never clears it


## Follow the controlled body again (a new flight).
func follow() -> void:
	follow_controlled = true
	serial = 0


## Index of the viewed body: the controlled body in follow mode (its serial is adopted), else the body with `serial`;
## when that is gone, follow mode again. -1 when there is no flight.
func resolve(sim: FlightSim) -> int:
	if not sim.is_active() or sim.body_count() == 0:
		return -1
	if not follow_controlled:
		for i in sim.body_count():
			if sim.body_serial(i) == serial:
				return i
		follow_controlled = true
	var controlled := sim.controlled_index()
	serial = sim.body_serial(controlled)
	return controlled


## Moves the view to the next (step > 0) or previous (step < 0) body ordered by serial, wrapping around. Landing on the
## controlled body turns follow mode on, any other body pins the view on its serial.
func cycle(sim: FlightSim, step: int) -> void:
	var current := resolve(sim)
	if current < 0 or step == 0:
		return
	var serials: Array[int] = []
	for i in sim.body_count():
		serials.append(sim.body_serial(i))
	serials.sort()
	var at := serials.find(serial)
	serial = serials[posmod(at + (1 if step > 0 else -1), serials.size())]
	follow_controlled = serial == sim.body_serial(sim.controlled_index())


## GCRF point at `station_m` on the body axis (body +x; stage stations as in vehicle_view.gd, where the body position
## is the centre of gravity and a stage at station s is drawn s - cg ahead of it).
func station_gcrf_m(sim: FlightSim, index: int, station_m: float) -> Vector3:
	var axis := Basis(sim.body_attitude_gcrf(index)) * Vector3.RIGHT
	return sim.body_position_gcrf_m(index) + axis * (station_m - sim.body_cg_station_m(index))


## The camera target: the object's centre of gravity.
func cg_gcrf_m(sim: FlightSim, index: int) -> Vector3:
	return station_gcrf_m(sim, index, sim.body_cg_station_m(index))


## Sum of the stage lengths.
func object_length_m(sim: FlightSim, index: int) -> float:
	var length := 0.0
	for stage: Dictionary in sim.body_stages(index):
		length += float(stage["length_m"])
	return length


## Camera distance after a switch to an object of `length_m`.
static func clamp_distance_m(distance_m: float, length_m: float) -> float:
	return clampf(distance_m, MIN_DISTANCE_PER_LENGTH * length_m, MAX_DISTANCE_PER_LENGTH * length_m)
