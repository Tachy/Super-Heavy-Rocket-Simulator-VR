# Headless smoke test: the GDExtension loads in the double-precision engine and delivers float64 data.
# Run: godot --headless --path game --script res://tests/smoke_test.gd
extends SceneTree

const KERNEL_DIR := "../data/spice"
# Horizons (DE441) geocentric Moon at JD 2451545.0 TDB, km (tests/validation/fixtures).
const MOON_J2000_REF_M := Vector3(-291608.3841877129, -266716.8338540655, -76102.48730658794) * 1000.0

var _failures := 0


func _initialize() -> void:
	_check(OS.has_feature("double"), "engine is a double-precision build")

	var core := StarsimCore.new()
	var kernel_dir := ProjectSettings.globalize_path("res://").path_join(KERNEL_DIR).simplify_path()
	_check(core.open(kernel_dir), "SPICE kernels loaded from %s" % kernel_dir)

	var j2000 := core.utc_to_tdb_s("2000-01-01T11:58:55.816")
	_check(absf(j2000) < 2e-3, "UTC -> TDB at J2000 (%.6f s)" % j2000)

	var moon := core.body_position_gcrf_m("moon", 0.0)
	var moon_error := moon.distance_to(MOON_J2000_REF_M)
	_check(moon_error < 3.5, "Moon at J2000 vs Horizons: %.3f m" % moon_error)

	# float64 end to end: a 1 mm height step at Starbase must be visible in GCRF coordinates.
	var tdb := core.utc_to_tdb_s("2026-10-12T12:00:00")
	var pad := core.geodetic_to_gcrf_m(25.99686, -97.15394, 10.0, tdb)
	var pad_up := core.geodetic_to_gcrf_m(25.99686, -97.15394, 10.001, tdb)
	var step := pad.distance_to(pad_up)
	_check(absf(step - 0.001) < 1e-7, "1 mm step at Starbase resolved as %.9f m" % step)
	print("Starbase GCRF at 2026-10-12T12:00:00 UTC: ", pad)
	print("Moon distance now: %.3f km" % (core.body_position_gcrf_m("moon", tdb).length() / 1000.0))

	# Catch tower geometry comes from the core's config (one source of truth for physics and drawing).
	var tower_sim := FlightSim.new()
	tower_sim.setup(ProjectSettings.globalize_path("res://").path_join("../config").simplify_path())
	var tower: Dictionary = tower_sim.site_tower("starbase_olp1")
	_check(tower.get("present", false), "Starbase has a catch tower")
	_check(absf(float(tower.get("mount_height_m", 0.0)) - 20.0) < 1e-9, "tower geometry from the core config")

	print("SMOKE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1
