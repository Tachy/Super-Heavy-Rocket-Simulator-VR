class_name SimClock
extends Node
## Simulation time in TDB seconds past J2000 (float64) with time warp.

signal time_jumped(tdb_s: float)

const WARP_STEPS: Array[float] = [0.0, 1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0]

var tdb_s := 0.0
var warp := 1.0


func _process(delta: float) -> void:
	tdb_s += delta * warp


## Jump to a new instant (e.g. user entered a UTC time).
func set_time(new_tdb_s: float) -> void:
	tdb_s = new_tdb_s
	time_jumped.emit(tdb_s)


func step_warp(direction: int) -> void:
	var index := WARP_STEPS.find(absf(warp))
	if index < 0:
		index = 1
	index = clampi(index + direction, 0, WARP_STEPS.size() - 1)
	warp = WARP_STEPS[index] * (signf(warp) if warp != 0.0 else 1.0)
