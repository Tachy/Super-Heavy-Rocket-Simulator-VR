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
	_check(_check_lunar(config_dir, tdb) == true, "lunar mission checks ran to the end")
	_check(_check_iss(config_dir, tdb) == true, "ISS mission checks ran to the end")
	_check(_check_refuelling(config_dir, tdb) == true, "refuelling checks ran to the end")

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
	# External view: mission timeline, body serials, tank levels, countdown start.
	_advance(sim, tdb + 200.0)
	var timeline: Dictionary = sim.mission_timeline()
	var kinds: Array = []
	for e in timeline.get("events", []):
		kinds.append(e["kind"])
	_check(kinds.has("liftoff"), "mission timeline contains the liftoff (%s)" % str(kinds))
	_check(sim.body_serial(0) >= 1, "body serial is stable and positive")
	_check((sim.body_engines(0)[0] as Dictionary).has("stage"), "engines carry their stage index")
	_check(float((sim.body_stages(0)[0] as Dictionary).get("oxidizer_capacity_kg", 0.0)) > 0.0, "stages carry tank capacities")
	_check(sim.mission_display_name() == "Testflug in den Erdorbit", "mission display name (%s)" % sim.mission_display_name())
	_check(sim.body_display_name(0) != "" and not sim.body_display_name(0).contains("Starship"),
			"body display name without trademark (%s)" % sim.body_display_name(0))
	_check(float(sim.telemetry(0).get("propellant_capacity_kg", 0.0)) >= float(sim.telemetry(0).get("propellant_kg", 1.0)),
			"telemetry carries the propellant load's capacity")
	_check((sim.body_engines(0)[0] as Dictionary).has("auxiliary"), "engines carry the auxiliary flag")
	# a fresh sim: the old one still has its advance target
	var countdown_sim := FlightSim.new()
	countdown_sim.setup(config_dir)
	_check(_ok(countdown_sim, countdown_sim.launch_from_site("starship_block3", "starbase_olp1", 0.0, tdb, 30.0)), "countdown launch starts")
	var clock = countdown_sim.mission_timeline().get("clock_s")
	_check(clock != null and absf(float(clock) + 30.0) < 1e-3, "countdown launch: T-30 clock at the start (%s)" % str(clock))
	for i in 10:  # past T-0 without the autopilot: nobody releases the clamps
		_advance(countdown_sim, tdb + 45.0)
	clock = countdown_sim.mission_timeline().get("clock_s")
	_check(clock != null and float(clock) == 0.0, "no liftoff past T-0: the clock holds at T-0 (%s)" % str(clock))
	# launch_from_pad starts a fresh mission log (the LEO mission above had lifted off)
	_check(_ok(sim, sim.launch_from_pad("starship_block3", 25.99686, -97.15394, 10.0, 90.0, tdb + 300.0)),
			"launch_from_pad on a flown session")
	var fresh_kinds: Array = []
	for e in sim.mission_timeline().get("events", []):
		fresh_kinds.append(e["kind"])
	_check(not fresh_kinds.has("liftoff"), "launch_from_pad: a fresh mission log (%s)" % str(fresh_kinds))
	sim.setup(config_dir)  # re-setup joins the old thread
	_check(not sim.is_active(), "re-setup starts empty")
	var empty: Dictionary = sim.mission_timeline()
	_check(empty.has("t0_tdb_s") and empty["t0_tdb_s"] == null and empty["clock_s"] == null
			and empty.get("events") is Array and (empty["events"] as Array).is_empty(),
			"no flight: the mission log has the full shape, empty (%s)" % str(empty))
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
	# ascent Task 5: a free launch with a target is planned on the pad; a site without a tower refuses Super Heavy
	var refused := FlightSim.new()
	refused.setup(config_dir)
	var target := {"periapsis_km": 200.0, "apoapsis_km": 200.0, "inclination_deg": 26.5, "northerly": true}
	var refusal_id := refused.launch_from_site("starship_block3", "ccsfs_slc40", 0.0, tdb, 10.0, target)
	refused.wait_idle(60.0)
	var refusal: Dictionary = refused.result(refusal_id)
	_check(refusal.get("done", false) and not refusal.get("ok", true)
			and str(refusal.get("message", "")) == "Kein Fangturm am Startplatz.",
			"Starship at SLC-40: the launch is refused with the German reason (%s)" % str(refusal))
	_check(refused.is_active() and refused.autopilot_status().get("state", "") == "aus"
			and not refused.autopilot_status().get("planning", true), "refused launch: on the pad, autopilot off, not planning")
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


# Lunar missions (l6): both kinds start, lunar telemetry, terrain query, refusal, re-plan command.
func _check_lunar(config_dir: String, tdb: float) -> bool:
	var started := Time.get_ticks_msec()
	var ret := FlightSim.new()
	ret.setup(config_dir)
	var id: int = ret.start_mission("ship_lunar_return_llo", tdb)
	_check(ret.wait_idle(120.0), "lunar return planned within 120 s")
	_check(ret.result(id).get("ok", false), "lunar return mission starts: %s" % ret.result(id).get("message", ""))
	var t: Dictionary = ret.telemetry(ret.controlled_index())
	_check(t.get("reference_body", "") == "moon", "return ship is Moon-referenced")
	_check(absf(float(t.get("altitude_terrain_m", -1.0)) - float(t["altitude_m"])) < 1e-6,
			"no terrain: terrain altitude = shape altitude")
	_check(absf(float(t.get("periapsis_altitude_km", 0.0)) - 100.0) < 15.0, "LLO periselene ~100 km (osculating)")
	var lunar: Dictionary = ret.lunar_state()
	_check(lunar.get("present", false) and lunar["return"].get("feasible", false), "return plan in lunar_state")
	_check(float(lunar["return"].get("tei_delta_v_mps", 0.0)) > 700.0, "TEI delta-v plausible (> 700 m/s)")
	_check(not ret.moon_terrain(-84.8, 0.0).get("present", true), "no terrain loaded for the return mission")
	var rp: int = ret.replan_lunar_return()
	_check(ret.wait_idle(120.0) and ret.result(rp).get("done", false), "re-plan command finishes")
	print("  lunar return start: %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))

	started = Time.get_ticks_msec()
	var land := FlightSim.new()
	land.setup(config_dir)
	var lid: int = land.start_mission("hls_lunar_llo", tdb)
	_check(land.wait_idle(120.0) and land.result(lid).get("ok", false), "lunar landing mission starts")
	var site: Dictionary = land.lunar_site()
	_check(site.get("present", false), "landing site present")
	var ground: Dictionary = land.moon_terrain(site["latitude_deg"], site["longitude_deg"])
	_check(ground.get("present", false) and ground.get("in_tile", false), "terrain query at the site (tile)")
	_check(absf(float(ground["height_m"]) - float(site["height_m"])) < 1.0, "terrain height = site height")
	_check(land.telemetry(land.controlled_index()).get("reference_body", "") == "earth", "HLS starts in LEO")
	print("  lunar landing start: %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))

	started = Time.get_ticks_msec()
	var nrho := FlightSim.new()
	nrho.setup(config_dir)
	var nid: int = nrho.start_mission("hls_lunar_nrho", tdb)
	nrho.wait_idle(180.0)
	_check(not nrho.result(nid).get("ok", true) and "nicht durchführbar" in str(nrho.result(nid).get("message", "")),
			"NRHO landing at 120 t is refused honestly")
	print("  NRHO landing start (refused): %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))
	return true


# ISS missions (i6): the three kinds start, ISS / rendezvous / docking telemetry, station geometry, commands.
func _check_iss(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	var started := Time.get_ticks_msec()
	var id: int = sim.start_mission("crew_dragon_iss", tdb)
	_check(sim.wait_idle(120.0), "ISS rendezvous planned within 120 s")
	_check(sim.result(id).get("ok", false), "crew_dragon_iss starts: %s" % sim.result(id).get("message", ""))
	print("  ISS rendezvous start: %.1f s" % ((Time.get_ticks_msec() - started) / 1000.0))
	var iss: Dictionary = sim.iss_state()
	_check(iss.get("present", false) and iss["ports"].size() == 2, "iss_state with both ports")
	var plan: Dictionary = sim.rendezvous_plan()
	_check(plan.get("present", false) and plan.get("feasible", false), "rendezvous_plan feasible")
	_check(plan.get("burns", []).size() >= 3, "rendezvous_plan has its burns")
	_check(sim.iss_mission().get("leg", "") == "Phasenflug", "iss_mission in the phasing leg")
	_check(sim.iss_mission().get("separation_tdb_s", 0.0) > 0.0, "the start is a launch window's separation")
	var geo: Dictionary = sim.station_geometry()
	_check(geo.get("present", false) and geo["boxes"].size() + geo["capsules"].size() >= 9,
			"station_geometry primitives")
	_check(geo["ports"][0].get("mechanism", "") == "idss" and iss["ports"][0].get("mechanism", "") == "idss",
			"ISS ports export their mechanism")
	var hold: int = sim.proximity_hold()
	_check(sim.wait_idle(10.0) and not sim.result(hold).get("ok", true), "hold refused outside the approach")
	var und: int = sim.undock()
	_check(sim.wait_idle(10.0) and not sim.result(und).get("ok", true), "undock refused while not docked")
	_check(not sim.docking_state().get("docked", false), "not docked during phasing")
	_check(sim.iss_mission().get("leg", "") == "Phasenflug", "refused commands leave the sequence running")

	var pad := FlightSim.new()
	pad.setup(config_dir)
	var pid: int = pad.start_mission("falcon9_crew_dragon_iss", tdb)
	_check(pad.wait_idle(120.0) and pad.result(pid).get("ok", false), "falcon9_crew_dragon_iss starts")
	var liftoff: float = pad.iss_mission().get("liftoff_tdb_s", 0.0)
	_check(liftoff > 0.0, "launch window in iss_mission")
	_check(absf(pad.mission_start_tdb_s() - liftoff) < 1e-6, "the mission starts at the window")
	_check(pad.mission_site_id() == "ccsfs_slc40", "pad mission: its launch site")

	var dep := FlightSim.new()
	dep.setup(config_dir)
	var did: int = dep.start_mission("crew_dragon_iss_departure", tdb)
	_check(dep.wait_idle(120.0) and dep.result(did).get("ok", false), "crew_dragon_iss_departure starts")
	_check(dep.docking_state().get("docked", false), "departure mission starts docked")
	_check(dep.docking_state().get("hooks_closed", false), "hooks closed while docked")
	_check(dep.iss_mission().get("undock_tdb_s", 0.0) > dep.mission_start_tdb_s(), "undocking ahead")
	var ab: int = dep.proximity_abort()
	_check(dep.wait_idle(10.0) and not dep.result(ab).get("ok", true), "abort refused while docked")
	return true


# Refuelling r6: the campaign command, transfer and depot getters.
func _check_refuelling(config_dir: String, tdb: float) -> bool:
	var sim := FlightSim.new()
	sim.setup(config_dir)
	_check(not sim.transfer_state().get("present", true), "no transfer without a docked pair")
	_check(not sim.depot_state().get("present", true), "no depot before a refuelling mission")
	_check(not sim.campaign_flight_state().get("present", true), "no flown campaign before one is started")
	var id: int = sim.start_mission("tanker_flight_orbit", tdb)
	_check(sim.wait_idle(120.0) and sim.result(id).get("ok", false), "tanker_flight_orbit starts")
	var depot: Dictionary = sim.depot_state()
	_check(depot.get("present", false) and depot.get("main_propellant_kg", 0.0) > 0.0, "depot_state with its fill")
	_check(depot.get("capacity_kg", 0.0) > depot.get("main_propellant_kg", 0.0), "depot_state with its tank volume")
	var depot_geo: Dictionary = sim.station_geometry()
	_check(depot_geo.get("present", false) and depot_geo["ports"][0].get("mechanism", "") == "probe_drogue"
			and depot_geo["ports"][0].get("elements_m", []).size() == 4, "depot port with its four drogues")
	# Crew escape: uncrewed by default; a crew-escape config makes the flight crewed (four seats, nothing armed).
	_check(not sim.crew_escape_state().get("present", true), "no crew escape on an uncrewed flight")
	var crewed: int = sim.set_crew_escape("starship_crew")
	_check(sim.wait_idle(10.0) and sim.result(crewed).get("ok", false), "set_crew_escape starship_crew")
	var crew: Dictionary = sim.crew_escape_state()
	_check(crew.get("present", false) and not crew.get("armed", true) and crew.get("members", []).size() == 4,
			"crew_escape_state with four seats, not armed")
	var no_crew: int = sim.set_crew_escape("no_such_crew")
	_check(sim.wait_idle(10.0) and not sim.result(no_crew).get("ok", true), "an unknown crew escape is refused")
	var uncrewed: int = sim.set_crew_escape("")
	_check(sim.wait_idle(10.0) and sim.result(uncrewed).get("ok", false)
			and not sim.crew_escape_state().get("present", true), "set_crew_escape('') makes the flight uncrewed")
	var bad: int = sim.refuelling_campaign({"height_km": "x"})
	_check(sim.wait_idle(10.0) and not sim.result(bad).get("ok", true), "a wrongly typed lever is refused")
	var unknown: int = sim.refuelling_campaign({"no_such_lever": 1.0})
	_check(sim.wait_idle(10.0) and not sim.result(unknown).get("ok", true), "an unknown lever is refused")
	var zero: int = sim.refuelling_campaign({"insulation": 0.0})
	_check(sim.wait_idle(10.0) and not sim.result(zero).get("ok", true), "a lever out of range is refused")
	var started := Time.get_ticks_msec()
	var cid: int = sim.refuelling_campaign({"height_km": 200.0})
	_check(sim.wait_idle(600.0) and sim.result(cid).get("ok", false), "refuelling_campaign finishes")
	var data: Dictionary = sim.result(cid).get("data", {})
	_check(data.get("tanker_flights", 0) >= 4 and data.get("tankers", []).size() == data.get("tanker_flights", -1),
			"campaign result with its tankers")
	_check(data.get("losses", {}).has("boil_off_t"), "campaign result with its losses")
	print("  refuelling campaign: %d tankers, %.1f d (%.1f s)" % [data.get("tanker_flights", 0),
			data.get("duration_d", 0.0), (Time.get_ticks_msec() - started) / 1000.0])
	return true
