class_name UiRate
extends RefCounted
## Update rate of text readouts and round gauges (user 2026-10-26: 15 Hz is plenty; every frame cost several ms of a
## VR frame). An update is due PERIOD_S after the last one, at once after a clock jump (the shown time moved by more
## than JUMP_S against the expected warp advance) and after invalidate() (another view or body).

const PERIOD_S := 1.0 / 15.0
## [E] Clock change against warp x elapsed time that counts as a jump (time panel, Tab, mission start).
const JUMP_S := 1.0

var _age_s := INF
var _last_tdb_s := NAN


func due(delta_s: float, tdb_s: float, warp: float) -> bool:
	_age_s += delta_s
	var jumped := not is_nan(_last_tdb_s) and is_finite(_age_s) \
			and absf(tdb_s - _last_tdb_s - _age_s * warp) > JUMP_S
	if _age_s < PERIOD_S and not jumped:
		return false
	_age_s = 0.0
	_last_tdb_s = tdb_s
	return true


func invalidate() -> void:
	_age_s = INF
