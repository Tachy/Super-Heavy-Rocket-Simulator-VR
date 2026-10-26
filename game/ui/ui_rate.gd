class_name UiRate
extends RefCounted
## Update rate of text readouts and round gauges (user 2026-10-26: 15 Hz is plenty; every frame cost several ms of a
## VR frame). An update is due PERIOD_S after the last one, at once after a clock jump (the shown time ran ahead of the
## expected warp advance by more than JUMP_S, or moved against it) and after invalidate() (another view or body). A
## clock that advanced less than expected (held back for the sim, pinned by a pending mission start) is no jump.

const PERIOD_S := 1.0 / 15.0
## [E] Clock change against warp x elapsed time that counts as a jump (time panel, Tab, mission start).
const JUMP_S := 1.0

var _age_s := INF
var _last_tdb_s := NAN


func due(delta_s: float, tdb_s: float, warp: float) -> bool:
	_age_s += delta_s
	var advance := tdb_s - _last_tdb_s
	var expected := _age_s * warp  # signed: running backwards expects a negative advance
	var jumped := not is_nan(_last_tdb_s) and is_finite(_age_s) \
			and (advance > maxf(expected, 0.0) + JUMP_S or advance < minf(expected, 0.0) - JUMP_S)
	if _age_s < PERIOD_S and not jumped:
		return false
	_age_s = 0.0
	_last_tdb_s = tdb_s
	return true


func invalidate() -> void:
	_age_s = INF
