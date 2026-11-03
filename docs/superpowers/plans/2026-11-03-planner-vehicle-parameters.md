# Planner to vehicle parameters Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every vehicle-specific value the planners and guidance laws use comes from `config/` (directly, or derived in code from config physics) instead of code defaults or fits; first the Raptor 3 nozzle exit at 1.30 m.

**Architecture:** The booster's pattern is the template: a parsed config struct in `core/include/starsim/config/vehicle_config.hpp`, parsed in `core/src/config/vehicle_config.cpp`, applied by a `gnc::*_parameters(const config::StageConfig&, ...)` function, reached through `FlightSession`. New shared derivations (inertia, angular acceleration, flip time, thrust bounds of an engine set) live in one small `gnc` module so every guidance law derives the same way.

**Tech Stack:** C++20 core (Catch2), TOML configs, Python model pipeline (`tools/structures`, pytest), Godot 4 (render check only in Task 1).

**Spec:** user ruling 2026-11-03 "auf Config umstellen ... und los" (campaigns may get worse, values follow config); item list = section "Planner audit" of `docs/superpowers/archive/specs/2026-11-03-test-audit.md`; facts with file:line = `.superpowers/sdd/2026-11-03-planner-vehicle-parameters/survey.md`; nozzle ruling = memory `project-raptor-nozzle` (copied into Task 1).

## Global Constraints

- One rule per value, decided per item in its task: (D) **derived** in code from config physics when the physics determines it; (K) **config key** with a provenance tag `[S]`/`[E]`/`[D]` in the TOML comment (see `config/README.md`) when it is a vehicle or mission property the config cannot derive (fitted/identified values keep `[D]` and say on what they were identified); (M) **mission config** when it is a mission choice, not a vehicle property.
- Production paths (FlightSession, planners, bindings, missions) never fall back to a struct default for a moved value. Struct defaults may stay only as unit-test fixtures. A vehicle that uses a guidance law but lacks one of its keys fails at load with a German message naming the key and file (Review Focus 1).
- Behaviour-neutral moves (K, M with the current value) must leave the affected tags bit-identical in outcome; derived values (D) may change results: re-measure, record before/after.
- User 2026-11-03: "Der Planer muss jetzt mit den neuen Werten automatisch so planen und im Flug nachplanen, dass die Flüge hinhauen ... für das ist der Planer ja da." A flight or campaign that turns red after a vehicle value changed is a planner defect: the planner / guidance must plan from the vehicle values and re-plan in flight so the flight works. Fix it in the planner logic (derive, search, re-plan from the live state), never by a fitted constant, a loosened limit or by changing the vehicle value back. Only a mission that is physically impossible with the vehicle (shown with numbers, e.g. no propellant margin at any feasible plan) goes to the user instead.
- Physics first; no webcast gates; tests follow physics (memory `feedback-tests-follow-physics`). No magic numbers: new constants with source in `core/include/starsim/math/constants.hpp`.
- SI units, frame/unit in names, angles in radians internally (`*_deg` keys converted at load, as the loader does today).
- After every vehicle config change: `python tools/vehicle_report.py <vehicle-id>` (starship_block3, falcon9_block5, starship_hls, starship_tanker as touched).
- Tests (user 2026-11-03: long tests collected at the end while the plan still changes values): per task `-Fast -TestFilter` of the touched tags plus single cases by name; push after those; every `[slow]` set and campaign once in Task 12, `-Nightly` there in the background. Task steps that name `[slow]` runs are moved to Task 12. Known reds stay red unless a task fixes them: the two settling cases, "Docked tanker: the ship return flies to the catch", "Lunar return campaign (direct): seed 2", the two flap-trim cases, `plan_speed`, #1147 (nightly).
- Each subsystem file `docs/subsystems/<slug>.md` touched is updated with the change (current state only; superseded values to `docs/history/<slug>.md`).
- Commit on `main` by path, push after green; never add `*.uid` or `cmake/build/`. Conventional Commits, English.

## Review Focus

1. A vehicle without the new keys (`crew_dragon`, `starship_depot`, `parachutist`) loads as before; a vehicle that uses the law and lacks a key fails with a German message — test in Task 3 (`test_vehicle_config` case "Ship without landing keys is refused").
2. `[[stage]] from = "<vehicle>:<stage>"` reuse: `starship_tanker`, `starship_hls_launch` inherit the new ship keys — test in Task 3 ("Tanker inherits the ship landing profile") and Task 4 (entry).
3. `[stage.recovery.drone_ship]` inherits new Super Heavy / Falcon keys left out there — test in Task 5.
4. The no-argument paths (`plan_capsule_return()`, `bindings/src/flight_sim.cpp:492` deorbit, tanker-reserve search) use config, not struct defaults — test in Task 7 and Task 8.
5. Derived values land in a physical band with a hand calculation in the test (e.g. Super Heavy flip time from inertia and gimbal torque); a stage without `inertia_kg_m2` uses the existing cylinder model, stated in the test — Task 2.

---

### Task 1: Raptor 3 nozzle exit 1.30 m, ring radii, no overlap (Opus)

User 2026-11-03: "1,30 m sind gesetzt ... Die Düsen dürfen sich nicht mehr überschneiden." "Die Glocken haben eine Wanddicke von 1 cm." 1.30 m is the inner exit diameter (flow, physics); drawn outer lip 1.32 m; 20 bells need the outer ring radius >= 1.32 / (2 sin 9 deg) = 4.22 m. Photo: `screenshots/raptor_nozzle/user_booster_engines_detail.jpg` (13 inner, 20 outer, bells close but not touching, outer bells reach beyond the 9 m hull at the bottom).

**Files:**
- Modify: `config/engines/raptor3.toml:20` (`nozzle_exit_diameter_m = 1.30` [S user 2026-11-03]); `config/vehicles/starship_block3.toml` ring radii (outer :98, middle/divert/centre and the ship sea-level ring 1.3 checked); `starship_hls.toml` if its rings share the values
- Modify: `blender/gen_engine.py:21`, `tools/structures/mount_layout.py:558` (bell with a 0.01 m wall: inner and outer surface, lip closed)
- Modify tests: `core/tests/test_vehicle_models.cpp:106-118` (vacuum Isp from the new area), `core/tests/test_peg_steering.cpp:29` (local 2.912e6/350 -> read the engine config or the new value with its derivation comment), `tools/structures/tests/test_vehicle_models.py`, `test_mount_layout.py`, `game/tests/vehicle_view_test.gd:154,871`
- Docs: `config/README.md`, `docs/subsystems/vehicle-models.md:447`, the propulsion subsystem file, `docs/known-issues.md` (remove the Raptor nozzle line)

**Interfaces:** Produces: `raptor3` exit area 1.3273 m2; SL thrust 2.7459e6 N and SL Isp 330 s unchanged; vacuum thrust and Isp follow (`engine_performance.cpp:12`), expected about 2.880 MN / about 346 s.

- [ ] Step 1: failing tests — C++ `"Raptor 3 vacuum performance follows the 1.30 m exit"` `[propulsion][config]`: area = pi/4*1.30^2 within 1e-9, F_vac = 2.7459e6 + 101325*area within 1 N, Isp_vac = F_vac / mdot / g0 within 0.01 s. Python `test_outer_ring_bells_do_not_touch` and `test_booster_bells_do_not_touch` (every pair of drawn outer lips, diameter 1.32 m, centre distance >= 1.32 m + 0 clearance, all rings of booster, ship, HLS). Run: red.
- [ ] Step 2: measure the ring radii from the photo (perspective-corrected against the 9 m hull and the 1.30 m bells) and the 1:100 STL (`feedback-model-reference-stl`); write the measurement with its uncertainty to `.superpowers/sdd/2026-11-03-planner-vehicle-parameters/task-1-rings.md`; choose radii >= the no-touch bound, tag [E] with the source. If photo/STL disagree beyond 0.1 m, stop and report (user decides).
- [ ] Step 3: config + bell wall + model rebuild (`tools/structures/build_models.py`); `python tools/vehicle_report.py starship_block3` and `starship_hls`.
- [ ] Step 4: tests green: `-Fast -TestFilter "[propulsion],[engines],[ascent],[booster_return],[peg]"` + pytest `tools/structures/tests` + Godot `vehicle_view_test.gd`.
- [ ] Step 5: render check (GPU, no headless) of the booster and ship engine bays from below and the side, PNGs to `screenshots/raptor_nozzle/`, sent to the user.
- [ ] Step 6: re-measure (vacuum thrust changed every Raptor burn): `[slow]` of `[ascent_planner]`, `[booster]`, `[catch]`, `[ship]`, `[lunar]` reduced cases; numbers before/after into the report; reds to known-issues with numbers. Commit `feat(propulsion): Raptor 3 nozzle exit 1.30 m, rings without bell overlap`.

### Task 1b: Mount opening fits the engine bay (Sonnet)

User 2026-11-03: "Beim Mount muss natürlich alles durchpassen." The pad's mount opening (9.0 m) is smaller than the outer bells (ring 4.40 m + lip 0.66 m = 5.06 m radius). Widen the opening (and whatever ring/clamp geometry sits around it) so the booster's whole engine bay incl. gimbal sweep passes with the existing 0.04 m margin; take the real opening from the photo / STL / pad references (`feedback-reality-over-physics-geometry`) when it is larger. `test_booster_stack_clears_deck_ring_and_qd` green (negative control kept), `docs/subsystems/launch-structures.md` current; render check of the mount from below and the side to `screenshots/raptor_nozzle/`. Physics pad collision shapes follow the drawing if they model the opening.

### Task 1d: Inner engine rings (Sonnet)

User 2026-11-03: "Innere Ringradien: so, dass es logisch und geometrisch sinnvoll aufgebaut ist." The 1:100 STL gives booster centre 0.88 m, middle (and divert) 2.55 m, ship sea-level 0.95 m (config 1.0 / 2.6 / 1.3, `task-1-report.md`). Set the rings to the STL values where the layout stays consistent: no two drawn lips (1.32 m) touch within or across rings incl. the gimbal sweep, equal angular spacing per ring, rings concentric with clear radial gaps, the ship's vacuum ring (3.0 m) and HLS rings checked the same way; where a STL value breaks a rule, the nearest value that keeps it, stated in the TOML comment [E, STL 1:100]. Extend the Task 1 no-touch pytest to all rings and across rings. Tests that pin a ring radius (`test_ship_landing.cpp:38` and the Python/Godot ones) follow the config. `vehicle_report.py`, render check, docs. Re-measure of flights is left to Task 1c.

### Task 1c: Planners fly the Task 1 reds again (Opus, systematic-debugging)

The reds from the 1.1 % lower vacuum Isp (`task-1-report.md`, `docs/known-issues.md`): flight 14 ascent plan infeasible, two ship-return dispersion cases miss the gate by 356 m and 638 m, lunar direct seed 15, aerocapture b4, booster abort at 800 m with 0.5 t left. Per the user's rule (Global Constraints) each is a planner defect: find why the planner did not absorb the change (plan from vehicle values, in-flight re-plan) and fix the planner logic, red test first where possible. A case that is physically impossible goes to the user with numbers. Known-issues updated; tags of each case with `[slow]`.

### Task 2: Shared vehicle capability derivations (Opus)

**Files:**
- Create: `core/include/starsim/gnc/vehicle_capability.hpp`, `core/src/gnc/vehicle_capability.cpp`, `core/tests/test_vehicle_capability.cpp` (`[gnc][capability]`)
- Modify: `core/CMakeLists.txt` / tests list as the repo does it

**Interfaces (produces, used by Tasks 3-8):**
- `double gnc::stage_pitch_inertia_kg_m2(const config::VehicleConfig&, const config::StageConfig&, double propellant_kg)` — about the stage's CG, propellant settled aft in its tanks: the flight dynamics' own parts (`vehicle::settled_stage_mass_properties`, which also gives the stage-local CG station for the levers). As built after the Task 2 fix round (was `(StageConfig, propellant_kg)` with a hull-long propellant cylinder).
- `struct EngineSetThrust { double min_n; double max_n; double gimbal_rad; double gimbal_rate_radps; double lever_m; };` `EngineSetThrust gnc::engine_set_thrust(const config::StageConfig&, const std::vector<std::string>& groups, double ambient_pa, double cg_station_m)`.
- `double gnc::max_pitch_acceleration_radps2(const EngineSetThrust&, double throttle, double inertia_kg_m2)` and `double gnc::rcs_pitch_acceleration_radps2(const config::StageConfig&, double inertia_kg_m2, double cg_station_m)`.
- `double gnc::flip_duration_s(double angle_rad, double accel_radps2, double max_rate_radps)` — bang-bang with a rate cap (accelerate, coast at the cap, decelerate).

- [ ] Step 1: tests with hand values: cylinder inertia of a 9 m x 71 m stage at a given mass; engine set of the 13 centre Raptors at min throttle 0.30 (thrust from the engine config at 0 Pa / 101325 Pa); flip of pi rad at 0.2 rad/s^2 without cap = 2*sqrt(pi/0.2) s; with cap 0.1 rad/s the trapezoid value; the Falcon 9 RCS turn against the comment's 1.12e7 kg m2 (`falcon9_block5.toml:149`). Red.
- [ ] Step 2: implement; green `-Fast -TestFilter "[capability]"`. Commit `feat(gnc): vehicle capability derivations from config`.

### Task 3: Ship landing from config (Opus)

Items: `ship_landing.cpp:49-54,66,70-74`, `ship_landing.hpp:19-37`.

**Files:** `core/include/starsim/config/vehicle_config.hpp` (ship uses `ReturnProfile`; add `ShipLandingProfile` fields only where `ReturnProfile` has no field), `core/src/config/vehicle_config.cpp` (parse, validate groups, German refusal), `core/src/gnc/ship_landing.cpp/.hpp`, `core/src/sim/flight_session*.cpp` (construction at `flight_session.cpp:994`, `flight_session_return.cpp:69,159,223`), `core/src/mission/lunar_return_mission.cpp:200`, `config/vehicles/starship_block3.toml:317-324`, tests `test_ship_landing.cpp`, `test_booster_return_config.cpp:98,106`, `test_vehicle_config.cpp`.

**Interfaces:** Produces `gnc::ShipLandingParameters gnc::ship_landing_parameters(const config::StageConfig& ship)` (replaces the no-arg default at every production call site).

Per value:
- engine groups and phases 3->2->2: (K) `landing_engine_phases` (existing key).
- reserve 1000 kg: (K) new `[stage.tanks]`-level `unusable_propellant_kg` per tank set [E], used as the landing reserve (also by Task 8).
- tilt limits per phase: (K) `landing_phase_max_tilt_deg` (existing) — [E] gimbal authority; tilt rate limits per phase: (D) from `max_pitch_acceleration_radps2` of the phase's engine set at its design throttle and the gimbal slew rate (the smaller).
- 3->2 switch window {20, 40} m/s and two-engine sink 20 m/s [D webcast]: (D) the sink at which two engines at max throttle still stop the ship within the remaining gate height and three at min throttle exceed weight; formula in the test with a hand case; the webcast value only in the report.
- flip throttle 0.8, flip tolerance 15 deg, settled rate 0.2, flaps until 30 deg, gate lead 220 m, margin 1.3: flip time and gate lead (D) from `flip_duration_s` with the flip engine set's acceleration and the sink speed at the gate; tolerance, settled rate, flaps-until, margin, flip throttle (K) `[stage.recovery]` keys `flip_throttle`, `flip_tolerance_deg`, `flip_settled_rate_dps`, `flaps_until_deg`, `gate_margin` [E/D with source].
- lateral effort 0.03 tuned on lunar seed 18: (K) `landing_lateral_effort` [D identified on lunar seed 18].

- [ ] Step 1: tests — "Ship landing parameters follow the vehicle config" (each K value read, each D value equals its hand formula); "Ship without landing keys is refused" (Review Focus 1); "Tanker inherits the ship landing profile" (`starship_tanker`, `starship_hls_launch`). Red.
- [ ] Step 2: implement, wire every production call site; K values = today's numbers (neutral).
- [ ] Step 3: `-Fast -TestFilter "[ship][landing],[config]"` green; then `[slow]` of `[catch]`, `[ship]` reduced lunar cases; before/after into the report. Commit `feat(gnc): ship landing parameters from vehicle config`.

### Task 3b: Realistic ship dry mass (Opus)

User 2026-11-03: "Leermasse realistischer." (asked after flight 14's ascent stayed ~60 m/s / 5.6 t short at every kick with 255 t dry and 52 t payload.) Evidence in `docs/known-issues.md` (ship aero frozen entry, ~:83-93): the 255 t [D] came from the rocket equation over webcast gauges that exclude the landing propellant, so it contains it; flight 12's one-engine phase allows m <= 209-218 t at 280 tf; judgement about 175 +-25 t. `docs/history/vehicle-data.md:83`, `tools/telemetry/mass_estimate.py`, `ship_dv.py`.

- Re-derive the dry mass from the flight data with the landing propellant separated (tool change, documented), cross-check with the one-engine bound and the 175 +-25 t judgement; set `starship_block3.toml` ship dry mass and dry CG (the CG must follow the mass distribution, not stay at 23.516 m by default) [D with source]; tanker, HLS, depot via `from =` checked (HLS keeps its own 120 t [E] if it overrides).
- `python tools/vehicle_report.py` for every ship-based vehicle.
- Re-fly with single cases by name: flight 14 ascent (must reach its orbit now), one ship catch, one entry replay per flight 12-14 (the `[nightly][webcast]` ship entry report case once, read not gated), tanker_flight. Reds follow the planner rule (fix in planners, or Task 4 for entry authority). Update known-issues (frozen-aero entry: what the new mass does to the replay table), subsystem docs, history (255 t).

### Task 4: Ship entry guidance from config (Opus)

Items: `entry_guidance.hpp:33-55`; `flight_session.cpp:440-441` uses raw defaults; lunar preset (q 32 kPa) in `lunar_return()`.

**Files:** new `[stage.entry]` in `vehicle_config.hpp/.cpp` (`EntryProfile`), `entry_guidance.hpp/.cpp`, `flight_session.cpp:440`, lunar return preset site, `starship_block3.toml`, tests `test_entry_guidance.cpp`, `test_vehicle_config.cpp`.

**Interfaces:** `gnc::EntryGuidanceParameters gnc::entry_guidance_parameters(const config::StageConfig&, EntryKind kind)` with `enum class EntryKind { Orbital, LunarReturn };`.

Per value: max q 25 kPa / lunar 32 kPa (K, structural [E]); terminal Mach 5/3 (K); min alpha 50 deg, alpha authority 20 deg (D from the flap travel in `[[stage.flap]]` and the trim table if the aero model offers the trim range; otherwise K [D identified on b1-b4] — the implementer states which in the report); bank rates 1/3 deg/s and alpha slew 3 deg/s (D: roll/pitch acceleration from `rcs_pitch_acceleration_radps2` and flap rate; cap = the smaller of derived and K `bank_rate_max_dps` [E]).

- [ ] Step 1: tests "Entry guidance parameters follow the vehicle config", "Lunar return entry uses the lunar q limit from config", tanker inheritance. Red.
- [ ] Step 2: implement and wire; green `-Fast -TestFilter "[entry],[guidance],[config]"`; `[slow]` `[ship][guidance]`, `[entry]` reduced lunar cases, #1147 report via nightly only in Task 12. Commit `feat(gnc): ship entry guidance from vehicle config`.

### Task 5: Booster returns from config (Opus)

Items: `booster_return.hpp:24-25` (flight time 230, flip 10), Super Heavy `[stage.recovery]` incomplete, `falcon9_block5.toml:149,164` (`flip_duration_s` 14.8, `boostback_loss_fraction` 0.157 [D fitted]), `scvx_landing.hpp:46-50` (defaults used by every vehicle without a profile), `booster_return_detail.hpp:27,39-42` (alpha 8 deg, leg margin 5 s, touchdown tilt 4 deg).

**Files:** `booster_return.cpp:59-121`, `booster_return_detail.hpp`, `scvx_landing.hpp`, `vehicle_config.hpp/.cpp` (grid fin and leg keys), `starship_block3.toml:123-130`, `falcon9_block5.toml:136-210`, tests `test_booster_return_config.cpp:38` (the case "Super Heavy keeps the default return parameters" becomes "Super Heavy return parameters come from its config"), `test_falcon9_rtls.cpp`, `test_booster_return.cpp`.

Per value: flip duration (D, both boosters) from `flip_duration_s` with the flip set (Super Heavy: centre engines at min throttle, gimbal; Falcon 9: RCS, its `flip_max_rate_radps`) — replaces the fitted 14.8 and the default 10; boostback loss fraction (D) from the gravity loss of the boostback burn at the stage's T/W over the Lambert arc if the derivation is closed-form, else K [D fitted] kept and reported; flight time (D) = the Lambert arc of the nominal MECO state (`nominal_meco_*` keys) — or K if the return planner already iterates it (state which); SCvx throttles/divert/switch for Super Heavy (K) `landing_phase_throttles`, `landing_switch_sink_mps` with today's numbers in `starship_block3.toml`, the 0.35 "13 engines" from the engine set (D: min throttle of the final set vs landing weight); alpha 8 deg (K) `[stage.grid_fins] max_return_alpha_deg` [E]; leg deploy margin and touchdown tilt (K) `[stage.landing_legs]` `deploy_margin_s`, `max_touchdown_tilt_deg` [E].

- [ ] Step 1: tests: config case above with hand values; drone-ship profile inherits the new keys (Review Focus 3). Red.
- [ ] Step 2: implement; green `-Fast -TestFilter "[booster],[falcon9],[scvx_landing],[config]"`; `[slow]` of `[booster]`, `[falcon9]`, `[catch]`; flip times and campaign outcomes before/after into the report. Commit `feat(gnc): booster return profiles from vehicle config`.

### Task 6: Lunar landing and ascent, lunar return burns (Sonnet)

Items: `lunar_landing_guidance.cpp:141-144` (guess {50,10,30} s, lateral effort 0.05), `lunar_ascent_guidance.hpp:35-45` (Raptor start 30 m, rise 150 m, abort margin 0.15), `lunar_return_mission.cpp:35-36` (aerocapture raise 120 m/s, deorbit 100 m/s).

Per value: the SCvx initial phase-time guess (D: from the HLS landing set's T/W and the approach state, a ballistic estimate); lateral effort 0.05, Raptor start height, rise height, abort margin (K) in `starship_hls.toml` `[stage.recovery]`/`[stage.lunar_ascent]` [E/D with source]; aerocapture raise and deorbit delta-v (M) in the lunar return mission TOML `[return]`.

- [ ] Steps: tests "HLS landing and ascent parameters follow config", "Lunar return burns come from the mission"; implement; `-Fast -TestFilter "[lunar_scvx],[lunar_ascent],[hls],[config]"`; `[slow]` `[lunar]` reduced cases. Commit `feat(gnc): lunar landing and ascent parameters from config`.

### Task 7: Ascent downrange and deorbit crossrange (Sonnet)

Items: `ascent_profile_planner.cpp:53` (provisional downrange 600 km), `flight_session_capsule.cpp:49-50` (Dragon bank 30 deg, crossrange 40 km), `deorbit_planner.hpp:21` (ship crossrange 60 km).

Per value: downrange (D) ballistic range of the nominal MECO state from the stage's `nominal_meco_*` keys; crossrange (D) from the vehicle's hypersonic L/D (aero config at the entry trim) with the standard crossrange approximation, bank (M) mission `[deorbit]`; the no-arg `plan_capsule_return()` and `bindings/src/flight_sim.cpp:492` read the default mission of the vehicle instead of struct defaults (Review Focus 4).

- [ ] Steps: tests "Provisional downrange from the MECO state", "Crossrange follows the vehicle L/D", "No-argument capsule return uses the mission config"; implement; `-Fast -TestFilter "[ascent_planner],[deorbit],[dragon],[config]"`, `[slow]` of the same. Commit `feat(gnc): downrange and crossrange from vehicle physics`.

### Task 8: Reserves and refuelling from config (Sonnet)

Items: `proximity_geometry.hpp:94` 600 kg, `phasing_schedule.hpp:24` 900 kg, `tanker_reserve.hpp:9-16` (margin 1.10, gas 3500 kg, boil-off 2000 kg, fuel share 0.56, unusable 1000 kg), `refuelling_campaign.hpp:48` vent 1.5 %.

Per value: proximity/phasing reserves (K) vehicle `[stage.recovery]` `rendezvous_reserve_kg` [E: the deorbit plan's need]; tanker unusable (from Task 3 `unusable_propellant_kg`), fuel share (D from the engine mixture ratio, already in `flight_session_transfer.cpp:268-290`: extend to the fuel share), boil-off (D from the thermal model over the planned phasing duration, as `phasing_boil_off_t_per_day` is), margin 1.10 and gas (K) [E]; vent (K) `config/refuelling` [E]. `reference_flights.toml` stays a measured file; it is re-measured in Task 12 with the existing tool.

- [ ] Steps: tests "Tanker reserve from the vehicle config", "Phasing and proximity reserves from config", tanker-reserve search without struct defaults (Review Focus 4); implement; `-Fast -TestFilter "[refuelling][config],[proximity],[phasing]"`, `[slow]` `[refuelling]`. Commit `feat(gnc): reserves and refuelling parameters from config`.

### Task 9: Settling cases, the drifting tanker (Opus, systematic-debugging)

Known reds since 47904b6: the two cases of `test_ignition_settling.cpp` (tanker in a ~100 km perigee orbit turns side-on, apoapsis 183 -> 117 km in one pass). Decide by physics: either the test orbit is unphysical for a tanker (then a real mission orbit from the tanker mission config) or the coast needs an attitude hold (then the drifting tanker holds a low-drag attitude through the RCS, as mission tankers do: check them first). No limit loosened.

- [ ] Steps: reproduce with numbers; check mission tankers' coast attitude; fix with a failing test first; `[settling]` (the file's tags) + `[refuelling]` `-Fast`. Commit `fix(...)`.

### Task 10: Falcon 9 flip case 4 and the 100 deg ignition trial (Opus, systematic-debugging)

After Task 5 (flip time derived). Case 4 lands upright 121 m off the pad after four unconverged SCvx re-plans; the expected off-pad list `{4}` in `test_falcon9_rtls.cpp`. Find the cause; trial of boostback ignition at 100 deg off the velocity instead of 114 deg (`flip_ignition_off_velocity_rad`), adopt only if the RTLS and ASDS campaigns stay green (user 2026-11-03). Add a pad rule to `landing_outcome.cpp` only if the user asks (not in this plan).

- [ ] Steps: reproduce; cause with numbers in the report; trial; `[falcon9]` `[slow]`; update the expected off-pad list to the measured set. Commit.

### Task 11: plan_speed reference plans

The stored reference plans differ (interface -68..-82 m/s, braking +78..+96 m/s, TEI -1.9/+3.4/-0.2 m/s). After Tasks 1-8 the physics changed: regenerate the reference plans with the existing tool/case, check the differences are explained by the changes (nozzle, flip, reserves), record before/after. No tolerance loosened.

- [ ] Steps: find the generator; regenerate; `plan_speed` green; commit `test: re-measure plan_speed reference plans`.

### Task 12: Re-measure, docs, final review

- [ ] `[slow]` of every touched tag, then `-Nightly` in the background (output to `.superpowers/sdd/2026-11-03-planner-vehicle-parameters/nightly.log`, check every 10 min); webcast reports read, not gated.
- [ ] `config/refuelling/reference_flights.toml` re-measured with its tool if the tanker numbers moved.
- [ ] `docs/known-issues.md`: every red with numbers; subsystem files (ship-landing, entry, booster-return, propulsion, refuelling, lunar) current; history files get the superseded values (the fitted F9 flip 14.8 s, Raptor 1.446 m, code defaults).
- [ ] Final review (Opus) on the whole diff; fix round; archive the plan; memory and ledger; ask for `/clear`.
