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
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	var tower_sim := FlightSim.new()
	tower_sim.setup(config_dir)
	var tower: Dictionary = tower_sim.site_tower("starbase_olp1")
	_check(tower.get("present", false), "Starbase has a catch tower")
	_check(absf(float(tower.get("mount_height_m", 0.0)) - 20.0) < 1e-9, "tower geometry from the core config")

	_check_falcon9(config_dir, tdb)

	print("SMOKE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1


# Falcon 9 missions (c5 game interface): mission start, landing zone with and without a deck, legs and grid fins.
func _check_falcon9(config_dir: String, tdb: float) -> void:
	var f9 := FlightSim.new()
	f9.setup(config_dir)
	_check(f9.start_mission("falcon9_rtls", tdb), "Falcon 9 RTLS mission starts")
	var zone: Dictionary = f9.landing_zone()
	_check(zone.get("present", false) and not zone.get("has_deck", true), "RTLS: landing zone without a deck")
	_check(absf(float(zone.get("pad_radius_m", 0.0)) - 42.7) < 1e-9, "RTLS: LZ-40 pad radius from the config")
	_check(f9.mission_site_id() == "ccsfs_slc40", "RTLS: launch site of the mission")
	_check(f9.start_mission("falcon9_asds", tdb), "Falcon 9 drone-ship mission starts on the same FlightSim")
	zone = f9.landing_zone()
	_check(zone.get("has_deck", false) and absf(float(zone.get("deck_length_m", 0.0)) - 91.0) < 1e-9
			and absf(float(zone.get("sea_height_m", 0.0)) + 35.0) < 1e-6, "ASDS: the drone ship's deck from the config")
	var legs: Dictionary = f9.body_legs(0)
	_check(legs.get("present", false) and (legs.get("azimuths_deg", []) as Array).size() == 4
			and float(legs.get("deployment", 1.0)) == 0.0 and legs.get("touchdown", "x") == "", "Falcon 9 stack: four stowed legs")
	var fins: Array = f9.body_grid_fins(0)
	_check(fins.size() == 4 and absf(float(fins[0].get("area_m2", 0.0)) - 1.8) < 1e-9, "Falcon 9 stack: four grid fins")
	_check((f9.return_state(0) as Dictionary).has("legs"), "return state carries the legs")
	_check(not f9.start_mission("no_such_mission", tdb), "unknown mission is refused")
	_check(f9.launch_from_site("falcon9_block5", "ccsfs_slc40", 0.0, tdb), "Falcon 9 launches without a mission")
	_check(not f9.landing_zone().get("present", true), "a plain launch has no landing zone")
	_check(f9.mission_site_id() == "ccsfs_slc40", "plain launch: its site")
	_check(f9.body_legs(7).is_empty() and f9.body_grid_fins(7).is_empty(), "invalid body index: empty results")
	var ship := FlightSim.new()
	ship.setup(config_dir)
	_check(ship.launch_from_site("starship_block3", "starbase_olp1", 0.0, tdb), "Starship launches")
	_check(not ship.landing_zone().get("present", true), "Starship: no landing zone")
	_check(not ship.body_legs(0).get("present", true), "Starship: no legs")
	_check(ship.body_grid_fins(0).size() == 3, "Super Heavy: three grid fins")
	_check(ship.start_mission("starship_leo", tdb) and ship.mission_site_id() == "starbase_olp1",
			"Starship LEO mission starts at Starbase")
	ship.abort_landing(0)  # no returning body yet: no effect, no error
	_check(ship.is_active() and ship.tower_state().get("phase", "") != "Abbruch",
			"abort before staging changes nothing (tower not aborted)")
