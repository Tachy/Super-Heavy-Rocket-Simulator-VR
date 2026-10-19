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

	# A script error aborts a check function (it then returns null) without failing a check: count it as a failure.
	_check(_check_falcon9(config_dir, tdb) == true, "Falcon 9 checks ran to the end")
	_check(_check_dragon(config_dir, tdb) == true, "Crew Dragon checks ran to the end")
	_check(_check_thread(config_dir, tdb) == true, "simulation thread checks ran to the end")

	print("SMOKE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1


## Waits for the simulation thread; true when command `id` finished ok (the former bool result).
func _ok(sim: FlightSim, id: int) -> bool:
	return id > 0 and sim.wait_idle(120.0) and bool(sim.result(id).get("ok", false))


## Advances to `tdb_s` and waits until the simulation thread has reached it.
func _advance(sim: FlightSim, tdb_s: float) -> void:
	sim.advance_to(tdb_s)
	sim.wait_idle(60.0)


# Simulation thread (l6): commands by id, results, idle wait, re-setup while busy.
func _check_thread(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	var id: int = sim.start_mission("starship_leo", tdb)
	_check(id > 0, "start_mission returns a command id")
	_check(sim.wait_idle(60.0), "the simulation thread becomes idle")
	var r: Dictionary = sim.result(id)
	_check(r.get("done", false) and r.get("ok", false), "start_mission finished ok")
	var reached := sim.advance_to(tdb + 2.0)
	_check(sim.wait_idle(30.0), "the simulation thread reaches the target")
	# 400 tick additions at ~8.5e8 s TDB round to a few microseconds
	var gap := (tdb + 2.0) - sim.advance_to(tdb + 2.0)
	_check(absf(gap) < 1e-4, "advance reaches the target (gap %s s)" % str(gap))
	_check(reached <= tdb + 2.0, "advance_to returns the snapshot's epoch, never ahead")
	var bad: int = sim.start_mission("no_such_mission", tdb)
	sim.wait_idle(10.0)
	_check(not sim.result(bad).get("ok", true) and str(sim.result(bad).get("message", "")) != "",
			"unknown mission: refused with a message")
	sim.setup(config_dir)  # re-setup joins the old thread
	_check(not sim.is_active(), "re-setup starts empty")
	return true


# Falcon 9 missions (c5 game interface): mission start, landing zone with and without a deck, legs and grid fins.
func _check_falcon9(config_dir: String, tdb: float) -> bool:
	var f9 := FlightSim.new()
	f9.setup(config_dir)
	_check(_ok(f9, f9.start_mission("falcon9_rtls", tdb)), "Falcon 9 RTLS mission starts")
	var zone: Dictionary = f9.landing_zone()
	_check(zone.get("present", false) and not zone.get("has_deck", true), "RTLS: landing zone without a deck")
	_check(absf(float(zone.get("pad_radius_m", 0.0)) - 42.7) < 1e-9, "RTLS: LZ-40 pad radius from the config")
	_check(f9.mission_site_id() == "ccsfs_slc40", "RTLS: launch site of the mission")
	_check(_ok(f9, f9.start_mission("falcon9_asds", tdb)), "Falcon 9 drone-ship mission starts on the same FlightSim")
	zone = f9.landing_zone()
	_check(zone.get("has_deck", false) and absf(float(zone.get("deck_length_m", 0.0)) - 91.0) < 1e-9
			and absf(float(zone.get("sea_height_m", 0.0)) + 35.0) < 1e-6, "ASDS: the drone ship's deck from the config")
	var legs: Dictionary = f9.body_legs(0)
	_check(legs.get("present", false) and (legs.get("azimuths_deg", []) as Array).size() == 4
			and float(legs.get("deployment", 1.0)) == 0.0 and legs.get("touchdown", "x") == "", "Falcon 9 stack: four stowed legs")
	var fins: Array = f9.body_grid_fins(0)
	_check(fins.size() == 4 and absf(float(fins[0].get("area_m2", 0.0)) - 1.8) < 1e-9, "Falcon 9 stack: four grid fins")
	_check((f9.return_state(0) as Dictionary).has("legs"), "return state carries the legs")
	_check(not _ok(f9, f9.start_mission("no_such_mission", tdb)), "unknown mission is refused")
	_check(_ok(f9, f9.launch_from_site("falcon9_block5", "ccsfs_slc40", 0.0, tdb)), "Falcon 9 launches without a mission")
	_advance(f9, tdb + 5.0)
	_check(f9.autopilot_status().get("state", "") == "aus", "a plain launch after a mission starts with the autopilot off")
	_check(not f9.landing_zone().get("present", true), "a plain launch has no landing zone")
	_check(f9.mission_site_id() == "ccsfs_slc40", "plain launch: its site")
	_check(f9.body_legs(7).is_empty() and f9.body_grid_fins(7).is_empty(), "invalid body index: empty results")
	var ship := FlightSim.new()
	ship.setup(config_dir)
	_check(_ok(ship, ship.launch_from_site("starship_block3", "starbase_olp1", 0.0, tdb)), "Starship launches")
	_check(not ship.landing_zone().get("present", true), "Starship: no landing zone")
	_check(not ship.body_legs(0).get("present", true), "Starship: no legs")
	_check(ship.body_grid_fins(0).size() == 3, "Super Heavy: three grid fins")
	_check(_ok(ship, ship.start_mission("starship_leo", tdb)) and ship.mission_site_id() == "starbase_olp1",
			"Starship LEO mission starts at Starbase")
	_check(not _ok(ship, ship.abort_landing(0)), "abort refused for a body that is not returning")
	_check(ship.is_active() and ship.tower_state().get("phase", "") != "Abbruch",
			"abort before staging changes nothing (tower not aborted)")
	return true


# Crew Dragon (d5 game interface): both missions via start_mission, thrusters, parachutes and the capsule state.
func _check_dragon(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	_check(sim.body_rcs(0).is_empty() and sim.body_parachutes(0).is_empty() and sim.capsule_state(0).is_empty()
			and not sim.splashdown_zone().get("present", true), "before a flight: empty thruster/parachute/capsule state")
	_check(_ok(sim, sim.start_mission("falcon9_crew_dragon", tdb)), "Falcon 9 with Crew Dragon starts")
	_check(absf(sim.mission_start_tdb_s() - tdb) < 1e-9, "an ascent starts at the requested time")
	var rcs: Array = sim.body_rcs(0)
	_check(rcs.size() == 8 + 8 + 16, "stack: 8 + 8 Falcon 9 thrusters and 16 Dracos")
	var dracos_blocked := true
	for t in rcs:
		if t.get("stage", 0) == 3 and not t.get("blocked", false):
			dracos_blocked = false
	_check(dracos_blocked, "stacked Dracos are blocked")
	_check((rcs[0] as Dictionary).has_all(["id", "position_m", "direction", "firing", "duty", "thrust_n", "starved"]),
			"thruster fields")
	_check(not sim.capsule_state(0).get("present", true), "a capsule stacked on its launcher is not a capsule body")
	_check(sim.body_parachutes(0).size() == 2, "the stacked capsule carries its two parachute sets")
	_check(not _ok(sim, sim.start_mission("ship_return_starbase", tdb)), "a ship return is not started by start_mission")

	var ret := FlightSim.new()
	ret.setup(config_dir)
	_check(_ok(ret, ret.start_mission("crew_dragon_return", tdb)), "Crew Dragon return mission starts")
	_check(absf(ret.mission_start_tdb_s() - tdb) > 1.0, "the return starts at its own epoch, not the requested one")
	var capsule: Dictionary = ret.capsule_state(ret.controlled_index())
	_check(capsule.get("present", false) and capsule.get("trunk_attached", false) and capsule.get("nose_cone_open", false),
			"capsule on its trunk with the nose cone open")
	_check(capsule.get("return_step", "") == "Deorbit-Brennvorgang", "the deorbit burn is planned")
	_check(capsule.get("parachute_phase", "x") == "", "no parachute sequencer while the trunk is attached")
	_check(ret.autopilot_status().get("return", {}).get("feasible", false), "the return plan is feasible")
	_check(ret.autopilot_status().get("capsule_step", "") == "Deorbit-Brennvorgang", "autopilot status: capsule step")
	var zone: Dictionary = ret.splashdown_zone()
	_check(zone.get("present", false) and absf(float(zone.get("radius_m", 0.0)) - 2000.0) < 1e-9, "splashdown zone radius 2 km")
	var chutes: Array = ret.body_parachutes(ret.controlled_index())
	_check(chutes.size() == 2 and chutes[0].get("state", "") == "verstaut" and float(chutes[0].get("inflation", 1.0)) == 0.0,
			"parachutes stowed")
	_check(ret.body_rcs(ret.controlled_index()).size() == 16, "Crew Dragon alone: 16 Dracos")
	_advance(ret, ret.mission_start_tdb_s() + 10.0)
	_check(ret.is_active(), "the return mission advances")
	_check(ret.body_rcs(9).is_empty() and ret.body_parachutes(9).is_empty() and ret.capsule_state(9).is_empty(),
			"invalid body index: empty results")
	_check(not _ok(ret, ret.plan_capsule_return()), "a second return plan is refused while one runs")
	_check(not _ok(ret, ret.set_splashdown_zone("no_such_zone")), "unknown splashdown zone is refused")
	return true
