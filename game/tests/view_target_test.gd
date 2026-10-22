# Headless test of the viewed object (Tab switching, CG target) with a real FlightSim.
# Run: godot --headless --path game --script res://tests/view_target_test.gd
extends SceneTree

const KERNEL_DIR := "../data/spice"

var _failures := 0


func _initialize() -> void:
	var core := StarsimCore.new()
	var kernel_dir := ProjectSettings.globalize_path("res://").path_join(KERNEL_DIR).simplify_path()
	_check(core.open(kernel_dir), "SPICE kernels loaded")
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	var tdb := core.utc_to_tdb_s("2026-10-12T12:00:00")
	_check(_run(config_dir, tdb) == true, "view target checks ran to the end")
	print("VIEW TARGET TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1


func _ok(sim: FlightSim, id: int) -> bool:
	return id > 0 and sim.wait_idle(120.0) and bool(sim.result(id).get("ok", false))


func _run(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	var target := ViewTarget.new()
	_check(target.resolve(sim) == -1, "no flight: nothing to view")
	_check(_ok(sim, sim.launch_from_site("starship_block3", "starbase_olp1", 0.0, tdb)), "Starship launches")
	_check(sim.body_count() == 1, "one body on the pad")
	var index := target.resolve(sim)
	_check(index == sim.controlled_index() and target.serial == sim.body_serial(index), "serial 0: the controlled body")
	target.cycle(sim, 1)
	target.cycle(sim, -1)
	_check(target.resolve(sim) == index, "cycling with one body stays")

	# Geometry: the body position is the CG (vehicle_view draws the stages around it).
	var stages: Array = sim.body_stages(index)
	var length := 0.0
	for stage: Dictionary in stages:
		length += float(stage["length_m"])
	_check(length > 50.0 and absf(target.object_length_m(sim, index) - length) < 1e-9, "object length %.1f m" % length)
	_check(target.cg_gcrf_m(sim, index).is_equal_approx(sim.body_position_gcrf_m(index)), "CG point is the body position")
	var nose := target.station_gcrf_m(sim, index, sim.body_cg_station_m(index) + 10.0)
	var axis := Basis(sim.body_attitude_gcrf(index)) * Vector3.RIGHT
	_check((nose - sim.body_position_gcrf_m(index)).distance_to(axis * 10.0) < 1e-6, "station 10 m forward of the CG lies along the body axis")

	# Staging: two bodies, Tab reaches the other serial, ordered by serial.
	var first_serial := target.serial
	_check(_ok(sim, sim.stage()), "stage separation")
	_check(sim.body_count() == 2, "two bodies after staging (%d)" % sim.body_count())
	var seen := {}
	for i in 3:
		target.cycle(sim, 1)
		seen[target.serial] = true
	_check(seen.size() == 2 and seen.has(first_serial), "Tab reaches both serials")
	var before := target.serial
	target.cycle(sim, 1)
	var forward := target.serial
	target.cycle(sim, -1)
	_check(target.serial == before and forward != before, "Shift+Tab goes back")

	# Mission restart: the viewed serial is gone -> the controlled body.
	var old_serial := target.serial
	_check(_ok(sim, sim.start_mission("starship_leo", tdb)), "new mission")
	var resolved := target.resolve(sim)
	_check(resolved == sim.controlled_index() and target.serial == sim.body_serial(resolved)
			and target.serial != old_serial, "gone serial: falls back to the controlled body")

	# Distance clamp on a switch.
	_check(is_equal_approx(ViewTarget.clamp_distance_m(1.0, 100.0), 60.0), "close distance clamped to 0.6 lengths")
	_check(is_equal_approx(ViewTarget.clamp_distance_m(1.0e6, 100.0), 4000.0), "far distance clamped to 40 lengths")
	_check(is_equal_approx(ViewTarget.clamp_distance_m(500.0, 100.0), 500.0), "distance in range is kept")
	target.free()
	return true
