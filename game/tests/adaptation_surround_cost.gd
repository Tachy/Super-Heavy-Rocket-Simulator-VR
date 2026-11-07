# Main-thread cost of the adaptation surround's complete store rebuild (the view jump), pad noon against a solar-eclipse
# penumbra (look fixes T4 fix round 1): headless, needs the built core and the SPICE kernels / atmosphere tables.
# godot --headless --path game --script res://tests/adaptation_surround_cost.gd  (prints ms; exit code 1 over the budget)
extends SceneTree

const AU_M := 149597870700.0
const EPOCH_UTC := "2024-04-08T18:42:00"
const BUDGET_MS := 11.0  ## one frame at 90 Hz
const REPEATS := 5

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()


func _initialize() -> void:
	_run.call_deferred()


## [worst frame ms, total ms, frames] of a view jump's rebuild (the best of REPEATS by the worst frame).
func _rebuild(camera_itrf: Vector3, to_gcrf: Basis, sun_gcrf: Vector3, moon_apparent: Vector3, tdb: float) -> Array:
	var best := [INF, 0.0, 0]
	for i in range(REPEATS):
		var surround := _atmosphere.adaptation_surround
		surround.reset()
		var worst := 0.0
		var total := 0.0
		var frames := 0
		while not surround.is_ready() and frames < 200:
			var before := surround.refresh_usec
			var visible := 1.0 if is_nan(tdb) else 1.0 - _core.solar_eclipse_fraction_gcrf(to_gcrf * camera_itrf, tdb)
			_atmosphere.update(to_gcrf * camera_itrf, Vector3.ZERO, to_gcrf, sun_gcrf, 0.0, visible, moon_apparent, tdb)
			var ms := (surround.refresh_usec - before) / 1000.0
			worst = maxf(worst, ms)
			total += ms
			frames += 1
		if worst < best[0]:
			best = [worst, total, frames]
	return best


func _run() -> void:
	if not _core.open(Paths.data_dir().path_join("spice")):
		print("adaptation_surround_cost: SPICE kernels missing")
		quit(1)
		return
	_atmosphere.start_loading(_core, Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	if not _atmosphere.wait_until_loaded():
		print("adaptation_surround_cost: atmosphere tables not loaded: ", _atmosphere.status_text())
		quit(1)
		return
	AdaptationSurround.enabled = true
	_atmosphere.eye_adaptation_enabled = true
	_atmosphere.umbra_light_synchronous = true
	var t := _core.utc_to_tdb_s(EPOCH_UTC)
	var to_gcrf := _core.earth_fixed_to_gcrf(t)
	var sun_gcrf: Vector3 = _core.apparent_body_position_gcrf_m("sun", t, Vector3.ZERO, Vector3.ZERO)
	var moon_gcrf: Vector3 = _core.apparent_body_position_gcrf_m("moon", t, Vector3.ZERO, Vector3.ZERO)
	# Pad noon: a Sun 60 deg up, no Moon shadow.
	var pad := HeightSource.geodetic_to_fixed(25.9967, -97.1543, 2.0)
	var up := pad.normalized()
	var sun_noon: Vector3 = to_gcrf * (up * 0.866 + Vector3(0, 0, 1).cross(up).normalized() * 0.5) * AU_M
	var noon := _rebuild(pad, to_gcrf, sun_noon, Vector3.ZERO, NAN)
	# Dallas under the 2024-04-08 eclipse (totality): the penumbra on the atmosphere, every ray segmented.
	var dallas := HeightSource.geodetic_to_fixed(32.7767, -96.7970, 2.0)
	var eclipse := _rebuild(dallas, to_gcrf, sun_gcrf, moon_gcrf, t)
	print("adaptation_surround_cost: view jump rebuild (576 cells), worst frame / total / frames: pad noon %.2f / %.2f ms / %d, eclipse %.2f / %.2f ms / %d (segments %d)" % [
			noon[0], noon[1], noon[2], eclipse[0], eclipse[1], eclipse[2], _atmosphere.eclipse_segments])
	quit(1 if maxf(noon[0], eclipse[0]) > BUDGET_MS else 0)
