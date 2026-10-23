# Follow-ups after the external view and the ascent milestone - design

Date: 2026-10-23. Status: decisions taken with the user 2026-10-23; awaiting final approval.

## Intent

Close the follow-ups the user approved at the end of the external-view and ascent milestones before M8 starts
(order decided 2026-10-23: follow-ups -> mission selection in the panel -> M8 with tiles + DEM first -> M7 cockpit).
The goal is a cleaner base: faster filtered tests, data-driven webcast gauges, physically better Falcon 9 recovery
numbers, no planner header cycle, no UI flicker. Physics first: no tuning to webcast gauges that the physics cannot
explain (standing decision 2026-10-14).

## Out of scope (moved or already done)

- Heat shield + flaps on the vehicle model and alpha/bank gauges: moved to M8 (Starship model). User 2026-10-23: the
  simulation must know the correct attitude (it does); the drawing comes with M8.
- Falcon 9 MECO -> separation gap: the user wants the real sequence. It already exists: `[stage.staging]` in
  `config/vehicles/falcon9_block5.toml` (`separation_delay_s` 4.5 s [D], `upper_ignition_delay_s` 6.0 s [D], from the
  CRS-35 / Starlink webcast tables, `docs/subsystems.md` "Staging sequence"). The memory note was stale. No work.

## Parts

### F1 Parallel filtered tests (`tools/build.ps1`)
- Now: `-TestFilter` runs `starsim_core_tests.exe "<spec>"` serially in one process. Catch tags are already CTest
  labels (`catch_discover_tests(... ADD_TAGS_AS_LABELS)`).
- Wanted: `-TestFilter` with a tag list (e.g. `"[cr3bp]"`, `"[staging][ascent]"`, `"[staging],[ascent]"`) runs
  `ctest -L "^(tag1|tag2)$" -j $Jobs` - one process per case, union of the tags, `[slow]` cases of those tags
  included. `-Fast` keeps adding `-LE slow`. The `plan_speed` test (`RUN_SERIAL`) keeps working.
- A spec that is not a pure tag list (test names, wildcards, negations) falls back to the serial exe as now, with a
  one-line notice.
- Output to a log file with a short summary (failed cases, total time), matching the repo rule to keep long output
  out of the context.
- Update CLAUDE.md's build section (the "`-TestFilter` runs serially" note) and the memory note.

### F2 Webcast gauges from TOML (`config/ui/webcast_gauges.toml`)
- Now: `game/ui/webcast/gauge_rules.gd` hard-codes thresholds, left-side priority and right-side sets per situation;
  `webcast_overlay.gd:21-38` holds label/unit/full scale/log_min/signed per gauge.
- Wanted: one TOML file holds all of it:
  - `[gauge.<id>]`: label, unit, full scale, optional log_min, signed.
  - Situations (docking, transfer, orbit, default) with their ordered right-side gauge list; left-side priority list;
    `max_per_side`, `hide_delay_s`.
  - Named conditions with their thresholds (q > 100 Pa, Knudsen < 0.1, heat flux > 1e3 W/m2, vertical speed below
    150 km, orbit gauges above 100 km, transfer entry 5e6 m altitude + 1e5 km apoapsis with exit factor 0.8).
- Conditions stay GDScript functions selected by name (no expression language); the TOML picks which named
  condition gates a gauge and sets its numbers. Unknown names in the TOML are an error at load (logged), not a crash.
- Load with `game/util/toml_lite.gd`; extend it if needed (string arrays), with its own test.
- Behaviour identical to today: the existing `webcast_rules_test.gd` / `webcast_overlay_test.gd` pass unchanged in
  substance (they may load the file instead of constants).
- A later settings wizard will edit this file: keep it flat and commented, provenance tags not needed (UI data).

### F3 Falcon 9 RCS flip and `boostback_loss_fraction`
- Now: 8 x 450 N [E] cold-gas thrusters at the interstage give 25.8 kNm on 1.0e7 kg m2: the flip takes 50.8 s
  (CRS-35: boostback ~14 s after separation). `flip_duration_s` 50 s [D] and `boostback_loss_fraction` 0.19 [D] were
  identified on that slow flip. A 14 s flip needs ~15x the torque and more N2 than the 200 kg [E] tank.
- Wanted: the flip and boostback timing match the flights within the physics.
- User decision 2026-10-23: research first, then fix the vehicle data (option a), no new guidance mechanism:
  1. Research what the real stage has: cold-gas thruster count, thrust, placement, N2 supply (COPV count/size),
     the flip's timeline in webcasts (separation -> boostback ignition at several flights) and published analyses.
     Record every source in the TOML comment with its provenance.
  2. Set thruster thrust/count/placement and the N2 supply from sources [S]; what the sources do not give is derived
     from the measured flip time [D] (flip time from several webcast flights, not one) - the N2 supply must cover the
     flip, the trims and the landing with a documented margin, and its mass goes into the dry-mass bookkeeping
     (`falcon9_block5.toml` header masses) so the total stage mass stays as identified.
  3. If the research shows the real stage does something the model lacks (e.g. boostback lit mid-turn), record it in
     `docs/subsystems.md` as an open point and stop for the user - not built in this part.
- After the flip is settled: re-identify `boostback_loss_fraction` on the full RTLS run (landing-burn mass ~12.1 t
  planned), update `flip_duration_s`, the RTLS / drone-ship tests' numbers and `docs/subsystems.md`.

### F4 Cross-flow normal force in `descent_model`
- Now: `gnc/descent_model` has drag along the velocity only (Cd(Mach)). Drone-ship landing burns clip for 600-2600
  ticks at 5-9x the gimbal's reach at ~1.4-0.8 km: the real body sees 3-5 m/s2 lateral from the cross-flow normal
  force at ~10 deg angle of attack (`docs/subsystems.md` "Mid-burn clipping").
- Wanted: the descent model (used by the SCvx landing planner) includes the cross-flow normal force from the same
  aero data the flight model uses (slender body + cross-flow table), convexified or linearised as SCvx needs.
- My calls (builder check):
  - Shared physics for every vehicle that uses `descent_model` (Falcon 9, Super Heavy, ship): the planner mirrors
    the flight model, no per-vehicle switch. Super Heavy catches and the ship's landings must not get worse; their
    `[slow]` catch / flight-14 campaigns are re-run and re-validated, changed numbers are re-baselined in
    `docs/subsystems.md`. If the term is removable in isolation, a fidelity flag for A/B measurement is welcome but
    not required.
  - "Same aero data" = exactly what the flight model evaluates for that vehicle, including the built-in cross-flow
    fallback (eta 0.68 x C_dc 1.2 in `slender_body_aero.cpp`) when a vehicle has no `crossflow_table`. No new Falcon 9
    table in this part (it would change the flight model too). Grid-fin normal force stays out of the planner model.
- Success (goal, measured, not a hard gate): drone-ship and RTLS landing burns clip far less (clipped ticks
  before/after recorded; aim: no mid-burn clipping above the terminal phase), landings unchanged or better, catches
  not worse. If the convexified term only reduces clipping partly, the part closes with the measured reduction
  documented and the rest listed as a follow-up.

### F5 One stack builder, no FlightSession <-> planner cycle
- Now: three places build a vehicle stack (`sim::ascent_stack`, inline in `start_ascent_mission`,
  `FlightSession::engage_ascent_autopilot` from `pad_vehicle_id_`/`pad_stack_` assuming fairing = true).
  `flight_session.cpp` includes `ascent_profile_planner.hpp`, which includes `flight_session.hpp`; the planner builds
  trial `FlightSession`s.
- Wanted: one stack builder (payload, fairing, propellant load) used by all three; `FlightSession` no longer depends on
  the planner (e.g. the planner is handed in from above, or the planned engage moves out of `FlightSession`). The
  builder decides the mechanism. Behaviour identical (same plans, same memo keys).
- Done before F6 because F6 touches the same planner code.

### F6 Test CPU cost of per-process launch planning
- Now: the plan memo is a static in-process map; CTest runs one process per case, so ~60 cases each re-plan
  (+28 s each, +1700 s CPU; a Falcon 9 drone-ship plan ~45 s).
- Wanted: test runs plan each distinct request once per test run, not once per process.
- Approved by the user 2026-10-23: a persistent plan cache, enabled only by an environment variable that the test CMake sets
  (`STARSIM_PLAN_CACHE_DIR` under `build/`), never in the game. Key = today's memo key + a hash of the config files
  read + a build stamp of the test exe (a rebuild or a config change never reuses an old plan). Writes are atomic
  (temp file + rename); a concurrent duplicate plan is allowed (same result), a corrupt/unknown file is ignored and
  re-planned. This is a deliberate, documented exception to "tests must not share temp files": the files are
  write-once, content-addressed and identical for every writer.
- The planner tests themselves (`[plan_speed]`, planner unit tests) bypass the cache.
- Success: CPU time of the affected labels back near the pre-ascent level; results bit-identical with and without
  the cache.

### F7 Planning flag no longer flashes on orbit-start missions
- Now: `FlightSim::start_mission` sets the planning flag whenever the mission kind may plan an ascent
  (`may_plan_ascent(kind)`), also for missions of those kinds that start in orbit or placed
  (`hls_refuel_orbit`, `tanker_flight_orbit`, `refuelling_campaign_flown_placed`).
- Wanted: the flag is set only when the mission will actually plan an ascent (start from a pad). Decide from the
  mission's start mode, not the kind alone. Test in the bindings / smoke test.

### F8 Drone ship chosen from a list
- Now: `default_drone_ship_id = "asds_atlantic"` is the only drone ship; `config/drone_ships.toml` has one entry.
- Wanted: `config/drone_ships.toml` lists the real ships with their basin (Atlantic: two; Pacific: one) [S] names,
  deck sizes [S]/[E]; the planner picks a ship of the basin that contains the predicted impact (nearest home port if
  several), and refuses if no ship serves that basin. With today's sites (Cape, Starbase) the result is Atlantic as
  before; a Pacific launch site is not added (out of scope). Ship display names come from config (trademark rule).
- The station position logic (provisional 600 km, then moved to the impact) stays; no land mask (documented gap).

## Constraints

- Architecture rules of CLAUDE.md (core without Godot, thin bindings, config not code, SI, no magic numbers).
- Tests: only the tags a change affects, `[slow]` included; a long run in the background with checks every 10 min.
- Commit per part on `main`, Conventional Commits, push after each part with green tests.
- `docs/subsystems.md` updated with every physics change (F3, F4, F6, F8).

## Left to the builder

- F1 parsing details, F2 TOML layout details, F4 the exact formulation inside SCvx, F5 the mechanism, F6 the hash
  and file format, F8 the basin geometry (simple lat/lon polygons or ocean boxes are enough).

## Order

F1 (faster tests first) -> F2 -> F7 -> F5 -> F6 -> F8 -> F3 -> F4 (F3 changes the Falcon 9 descent numbers that F4
then measures against).
