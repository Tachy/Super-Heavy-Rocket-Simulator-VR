# Ascent Trajectory and Staging Sequence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every launch from a pad flies a physics-optimised, continuous arc to SECO (kick chosen by trial flights,
throttle by a max-Q limit), with a realistic MECO -> separation -> ignition sequence and the recovery chosen by the
onboard computer.

**Architecture:** Three model changes in `core` - a staging state machine in `FlightSession` driven by `[stage.staging]`,
a dynamic-pressure throttle limiter in `gnc::AscentGuidance`, and a pre-launch planner `sim::plan_ascent_profile` that
flies trial ascents in a fresh `FlightSession` and picks the kick (and, for Falcon 9, the recovery). Missions and the
free launch both go through the planner; webcast-tuned `kick_deg` / `booster_throttle` leave the mission files. Each
task re-baselines the reference tests its change moves, with the reason in `docs/subsystems.md`.

**Tech Stack:** C++20 core (Catch2 tests, CMake/Ninja), TOML config, GDExtension bindings, Godot 4 GDScript.

**Spec:** `docs/superpowers/specs/2026-10-22-ascent-trajectory-staging-design.md`

## Global Constraints

- Physics in `core`, float64, SI units, frame + unit in names; bindings thin; the game only sends commands.
- Vehicle / mission data in TOML with provenance tags `[S]` / `[E]` / `[D]`; no magic numbers (constants with source).
- UI strings German, code / comments / commits English; Conventional Commits; commit + `git push origin main` after each
  task once its affected tests are green.
- Tests: only the tags a change affects (`[slow]` cases of those tags included). Build with
  `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoTests` (add `-NoBindings` for core-only tasks), then
  `ctest --test-dir build -j 24 -L "^(tag1|tag2)$" --output-on-failure > tmp/<task>.txt`; runs > 10 min in the
  background, checked every 10 min. New cases > ~40 s get `[slow]`.
- No webcast fitting: a reference number that moves is re-baselined to the new physics with its reason documented; a
  validation miss is investigated in the model (drag, thrust, masses), never closed by tuning the profile.
- Standing decisions: only Block 3 / V3 and Falcon 9 Block 5; Super Heavy always caught (never expendable); physics over
  gauges.

## Review Focus

1. A launch site without a tower (Starship at `ccsfs_slc40`) or without a landing zone (Falcon 9 at `starbase_olp1`):
   Starship refused with a German reason before liftoff, Falcon 9 falls back to the drone ship - Task 4 tests both.
2. A target the stack cannot reach (e.g. 2000 km circular with Starship + catch): refused, never flown expendable or
   with a broken catch reserve - Task 4 test "unreachable target is refused".
3. The countdown while the planner runs: the free launch must not lift off on the default profile; the clock holds on
   the pad ("Startvorbereitung") until the plan is in - Task 5 test.
4. Pilot / re-engaged autopilot after staging (existing test "Re-engaging the ascent autopilot after staging keeps a
   stacked payload ...") and a manual `stage()` command (key) during the sequence: the sequence must finish once, no
   double separation - Task 1 test.
5. An inclination below the site latitude (Florida with the 26.5 deg default target): the lowest reachable inclination
   (due east), not a refusal - Task 4 test.

---

### Task 1: Staging sequence (cold and hot) in `FlightSession`

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp` (parse `[stage.staging]`)
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (`AutopilotState::Staging`,
  `FlightSession::stage()` split into a sequence)
- Modify: `core/src/gnc/booster_return.cpp`, `core/include/starsim/gnc/booster_return.hpp` (flip engine groups)
- Modify: `core/src/sim/predicted_events.cpp` (separation / upper ignition predicted at their own instants)
- Modify: `config/vehicles/falcon9_block5.toml`, `config/vehicles/starship_block3.toml` (+ depot / tanker / HLS launch
  variants that stage), `config/README.md`
- Test: `core/tests/test_staging_sequence.cpp` (new, tag `[staging]`)

**Interfaces:**
- Produces: `config::StagingConfig { enum class Kind { Hot, Cold } kind; double separation_delay_s;
  double upper_ignition_delay_s; double separation_dv_mps; std::vector<std::string> run_on_engine_groups; }` in
  `StageConfig::staging` (lower stage). TOML `[stage.staging]`: `kind = "hot" | "cold"` (replaces the unused
  `separation = "hot_staging" | "cold_pneumatic"` key), `separation_delay_s`, `upper_ignition_delay_s`,
  `separation_dv_mps`, `run_on_engine_groups`.
- Produces: `[stage.recovery] flip_engine_groups` (default: the landing groups, today's behaviour; Falcon 9 `[]` = RCS
  flip, engines off).
- Produces: `AutopilotState::Staging` between `Booster` and `UpperStage`; `FlightSession::stage()` keeps its signature
  (manual staging) and starts the same sequence.

Sequence (all events recorded in `timeline_` at their own tick):
- Cold (Falcon 9): MECO (all booster groups off; the tail-off acts on the joined stack) -> `separation_delay_s` later
  `FlightBody::separate(1, separation_dv_mps)` -> `upper_ignition_delay_s` after separation `ignite()` on the upper stage.
  Values from the CRS-35 / Starlink 15-25 webcast event times (MECO -> stage separation -> second-engine start) [S],
  recorded in the TOML comment; separation push [E] with reason.
- Hot (Super Heavy): MECO (all groups off except `run_on_engine_groups` = the centre three, commanded to minimum
  throttle) -> the ship's engines start while joined (`ignite()` on stage index 1 of the stack: thrust acts on the
  stack) -> separate (dv 0) on the tick all the ship's non-auxiliary engines are `EngineMode::Running` -> the booster
  keeps the centre group into the flip. `separation_delay_s` / `upper_ignition_delay_s` unused (0). Flight 14 telemetry
  checks the MECO -> separation interval where it shows (else [E] = ship start-up time).

- [ ] **Step 1: Write the failing tests** in `core/tests/test_staging_sequence.cpp`, tag `[staging]`:
  - `"Cold staging: MECO, separation after the delay, upper ignition after its delay"` - Falcon 9 from
    `falcon9_rtls`: event epochs satisfy `sep - meco == Approx(separation_delay_s).margin(tick)` and
    `ignition - sep == Approx(upper_ignition_delay_s).margin(tick)`; right after separation
    `dot(v_booster - v_upper, unit(v_upper)) <= 0.5` m/s.
  - `"Hot staging: the ship lights while joined and separates at thrust; the booster falls back"` - Starship from
    `starship_leo`: separation tick has all ship engines Running; relative along-track speed booster - ship
    `<= 0.5` m/s at separation and decreasing over the next 5 s.
  - `"The Falcon 9 booster flips on RCS with its engines off"` - no booster engine Running between separation and the
    boostback ignition.
  - `"A manual stage command during the sequence separates once"` - `stage()` twice -> one StageSeparation event.
- [ ] **Step 2: Run** `ctest --test-dir build -L "^staging$"` after `-NoTests -NoBindings` build. Expected: FAIL
  (`[stage.staging]` unknown / separation in the MECO tick).
- [ ] **Step 3: Implement** config parsing, the `Staging` state, flip groups, predictions. The `Staging` state holds
  the stack attitude on the current guidance command (no steering change while joined).
- [ ] **Step 4: Run** `[staging]` green, then the moved references: `ctest -L "^(session|timeline|catch|rtls|asds|falcon9|booster|return|ascent|iss_mission|launch_window|refuelling|campaign_flown|hls)$"`.
  Re-baseline each moved number (MECO epochs, catch reserve at the landing burn, ISS window `insertion_s`, refuelling
  `reference_flights.toml` measured inputs) with a one-line reason in `docs/subsystems.md` (new section "Ascent
  trajectory and staging"). A catch that no longer succeeds is a defect to fix, not a re-baseline.
- [ ] **Step 5: Commit** `feat(sim): realistic staging sequence - cold with delays, hot with the ship lit while joined`, push.

### Task 2: Max-Q throttle limit in the ascent guidance

**Files:**
- Modify: `core/include/starsim/gnc/ascent_guidance.hpp`, `core/src/gnc/ascent_guidance.cpp`
- Modify: `core/src/sim/predicted_events.cpp` (MECO forecast uses the limiter instead of the schedule)
- Modify: vehicle TOMLs (`[stage] max_dynamic_pressure_pa` on first stages), `config/README.md`
- Create: `tools/telemetry/peak_dynamic_pressure.py` (peak q of a recorded ascent from speed + altitude + US76)
- Test: `core/tests/test_ascent_guidance.cpp` (tag `[ascent]`)

**Interfaces:**
- Produces: `PitchProgram::max_dynamic_pressure_pa` (from the first stage's config; 0 = no limit) and
  `double AscentGuidance::booster_throttle(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
  time::Epoch epoch) const` - full thrust below the limit, throttled to hold q <= limit (engine minimum respected).
  `PitchProgram::booster_throttle` (the time schedule) is removed with its config key.
- Values: `max_dynamic_pressure_pa` [D] = the peak q of flight 14 (Starship) and CRS-35 (Falcon 9) from
  `tests/validation/telemetry` via the new tool; command and result in the TOML comment.

- [ ] **Step 1: Failing tests** `"The ascent holds dynamic pressure at the vehicle limit"` (free Starship and Falcon 9
  ascents: `max q <= 1.03 * limit`; throttle 1.0 while `q < 0.9 * limit`) and `"MECO forecast follows the limiter"`
  (forecast vs flown MECO within the existing forecast tolerance).
- [ ] **Step 2: Run** `-L "^ascent$"`, expect FAIL.
- [ ] **Step 3: Implement** the limiter (feedback on the current q with the engines' throttle rate; document gain and
  why), the tool, the TOML values; remove `booster_throttle` from `PitchProgram`, `AscentMission` and all mission TOMLs.
- [ ] **Step 4: Run** `[ascent]` green, then the Task 1 reference tags; re-baseline with reasons.
- [ ] **Step 5: Commit** `feat(gnc): max-Q throttle limit replaces flight-fitted throttle schedules`, push.

### Task 3: Pre-launch ascent planner (kick optimisation)

**Files:**
- Create: `core/include/starsim/sim/ascent_profile_planner.hpp`, `core/src/sim/ascent_profile_planner.cpp`
- Modify: `core/CMakeLists.txt`
- Test: `core/tests/test_ascent_profile_planner.cpp` (tags `[ascent_planner]`, the full sweep `[slow]`)

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::sim {
  struct AscentProfileRequest {
      std::string vehicle_id; std::string site_id; double heading_rad; time::Epoch liftoff;
      gnc::AscentTarget target; std::optional<config::MissionPayload> payload; double payload_kg = 0.0; bool fairing = true;
      std::optional<LaunchSite> landing_zone;  // the recovery set up as today (Task 4 replaces it by RecoveryChoice)
  };
  struct AscentProfile {
      bool feasible = false; std::string reason;  // German, shown to the player when not feasible
      double kick_angle_rad = 0.0; double upper_propellant_at_seco_kg = 0.0; int trial_flights = 0;
  };
  AscentProfile plan_ascent_profile(const FlightSession& like, const AscentProfileRequest& request);
  }
  ```
  `like` supplies ephemeris, Earth orientation and config dir; each trial is a fresh `FlightSession` flown from the pad
  to SECO (upper-stage cutoff) with the candidate kick; the objective is the upper stage's propellant at SECO; a trial
  that does not reach the target orbit or whose booster recovery plan fails is infeasible.
- Search: golden section on `kick_angle_rad` in [4, 25] deg [E] to 0.2 deg (spec), bracketing first if the optimum
  sits at a bound; result memoised per process by the request (a repeated mission start costs nothing).

- [ ] **Step 1: Failing tests:** `"The planner's kick is the optimum of a kick sweep"` `[slow]` (Starship to 200 km /
  26.5 deg: planner kick within 0.2 deg of the best of a 1-deg sweep 6..22 deg, propellant at SECO >= every sweep
  point - 1 t); `"The optimised arc has no kink at staging"` (upper-stage pitch never more than 15 deg [E] below the
  flight-path angle in the first 60 s after separation; MECO flight-path angle between 10 and 40 deg [E]);
  `"A repeated request is answered from the memo"` (`trial_flights == 0` on the second call).
- [ ] **Step 2: Run** `-L "^ascent_planner$"`, expect FAIL (no symbol).
- [ ] **Step 3: Implement.** Trial speed: if a trial flight with the flight's 5 ms tick is too slow for the countdown
  rule, a coarser trial tick is allowed only with a test showing the chosen kick moves < 0.2 deg.
- [ ] **Step 4: Run** `[ascent_planner]` green; report trial count and wall time per plan in the commit message.
- [ ] **Step 5: Commit** `feat(sim): pre-launch ascent planner chooses the pitch kick by trial flights`, push.

### Task 4: Recovery choice, refusals and reachable inclination

**Files:**
- Modify: `core/include/starsim/sim/ascent_profile_planner.hpp`, `core/src/sim/ascent_profile_planner.cpp`
- Modify: `config/launch_sites.toml` (`landing_zone = "ccsfs_lz40"` on `ccsfs_slc40` [S: wiki / CRS-35]), launch-site
  config loader
- Test: `core/tests/test_ascent_profile_planner.cpp`

**Interfaces:**
- Produces (replaces `AscentProfileRequest::landing_zone` by `RecoveryChoice recovery`, filled by `plan_launch`):
  `enum class RecoveryKind { TowerCatch, LandingZone, DroneShip }`; `struct RecoveryChoice { RecoveryKind kind;
  std::string zone_or_ship_id; double latitude_rad, longitude_rad; }`;
  `AscentProfile plan_launch(const FlightSession& like, AscentProfileRequest request)` - picks the recovery, then
  `plan_ascent_profile`; sets `request.target.inclination_rad = max(target, |site latitude|)` when unreachable.
- Rules (spec): Super Heavy -> TowerCatch at the site's tower, no tower or infeasible -> `feasible = false` with
  "Kein Fangturm am Startplatz" / "Ziel mit Booster-Rückkehr nicht erreichbar". Falcon 9 -> LandingZone when the site
  has one and the plan is feasible with the RTLS reserve, else DroneShip `asds_atlantic` placed at the booster's
  predicted no-boostback impact point (`BoosterReturnGuidance::predict_impact` 10 s after MECO, the
  `falcon9_asds.toml` derivation), heading along the ground track; neither -> `feasible = false`.

- [ ] **Step 1: Failing tests** (`[ascent_planner]`): `"Falcon 9 from SLC-40 to the ISS plane returns to LZ-40"`;
  `"Falcon 9 with a Starlink-class payload to 53 deg lands on the drone ship at its predicted impact point"` (ship within
  2 km of the no-boostback impact); `"Falcon 9 from a site without a landing zone uses the drone ship"`;
  `"Starship at a site without a tower is refused"`; `"An unreachable target is refused"` (Starship 2000 km circular);
  `"An inclination below the site latitude flies due east"` (Starship from LC-39A with 26.5 deg -> 28.6 deg).
- [ ] **Step 2: Run**, expect FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `[ascent_planner]` green.
- [ ] **Step 5: Commit** `feat(sim): the onboard computer chooses the booster recovery and refuses unreachable launches`, push.

### Task 5: Every launch through the planner (missions, chains, free launch)

**Files:**
- Modify: `core/src/sim/ascent_mission.cpp` (`start_ascent_mission` calls `plan_launch`; a not feasible plan refuses the
  start with its reason), `core/include/starsim/config/mission_config.hpp`, `core/src/config/mission_config.cpp`
  (`kick_deg`, `[drone_ship]` and `landing_zone` removed: the recovery comes from the planner), all
  `config/missions/*.toml` with `[pitch_program]`
- Modify: `core/src/sim/flight_session.cpp` (`engage_ascent_autopilot(target)` on a clamped stack plans first)
- Modify: `bindings/src/flight_sim.cpp` (`launch_from_site` result carries the plan's reason; snapshot shows
  `autopilot_status()["planning"] = true` while planning), `game/vehicle/flight_controller.gd` (`last_event`
  "Startvorbereitung: Bahn wird berechnet", refusal text shown), `game/ui/webcast/*` only if the status line needs it
- Test: `core/tests/test_ascent_profile_planner.cpp`, `game/tests/smoke_test.gd`

**Interfaces:**
- Consumes: `plan_launch` (Task 4). Produces: no new API beyond the snapshot flag.

- [ ] **Step 1: Failing tests:** `"Mission starts plan their ascent; no mission file carries a kick"` (loader rejects
  `kick_deg` / `booster_throttle`); `"A free launch holds on the pad until the plan is in"` (scheduled T-0 before the plan
  completes -> liftoff no earlier than the plan's completion; no liftoff on the default `PitchProgram`); smoke test:
  launch_from_site of Starship at `ccsfs_slc40` reports the German refusal.
- [ ] **Step 2: Run**, expect FAIL.
- [ ] **Step 3: Implement**; remove the `[pitch_program]` / `[drone_ship]` sections and `landing_zone` keys from the
  mission files (their recovery now comes from the planner; comments with the old webcast derivations move to
  `docs/subsystems.md` as history).
- [ ] **Step 4: Run** the full reference tag set of Task 1 plus `[mission]`, `[iss]`, `[dragon]`, `[lunar]` cases that
  launch from a pad, and the Godot smoke / webcast tests; re-baseline with reasons. `refuelling_report.exe --fly-placed`
  once: the campaign result (tankers / days) before vs after into `docs/subsystems.md`.
- [ ] **Step 5: Commit** `feat: every launch plans its ascent profile; tuned kicks and throttle schedules removed`, push.

### Task 6: Validation against flight 14 and CRS-35, docs

**Files:**
- Create: `config/missions/flight14_ascent.toml`, `config/missions/crs35_ascent.toml` (targets, payloads, propellant
  loads of those flights [S]/[D]; no profile keys)
- Modify: `core/tests/test_starship_ascent.cpp`, `core/tests/test_falcon9_ascent.cpp` (the "follows the webcast
  telemetry" tests fly these missions through the planner)
- Modify: `docs/subsystems.md`, `CLAUDE.md` (implemented-modules list: ascent planner, staging sequence), `docs/ansichten.md`
  (planning status on the pad)

- [ ] **Step 1: Rewrite the telemetry tests** to compare the planned flight with the recordings: MECO time, altitude,
  speed, flight-path angle and SECO time with tolerances from the readings' accuracy (document each); engine-outs
  excluded and stated.
- [ ] **Step 2: Run** `-L "^(validation|ascent|falcon9)$"`. A miss beyond tolerance: investigate drag / thrust / masses;
  where the telemetry separates drag from thrust (engine counts known, the MECO coast), calibrate the ascent drag tables
  (`tools/telemetry/drag_estimate.py` on the ascent window) and record provenance [D]. Never adjust the profile.
- [ ] **Step 3: Docs** - section "Ascent trajectory and staging" complete (decisions, numbers, re-baselines, the
  validation result incl. any open discrepancy), memory note updated.
- [ ] **Step 4: Run** all affected tags once more (Task 1 set + `[ascent_planner]` + `[validation]`).
- [ ] **Step 5: Commit** `test: ascent validated against flight 14 and CRS-35; docs`, push.
