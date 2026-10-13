# Falcon 9 c3: return to the landing zone (RTLS) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Falcon 9 first stage separated on an RTLS ascent flips, boosts back, flies an entry burn, lands on SCvx
plans (1 -> 3 -> 1 engines) on its legs on LZ-40 and is judged "gelandet".

**Architecture:** `gnc/booster_return` becomes generic over the vehicle: its engine sets, return flight time, entry
burn and landing-burn profile come from `[stage.recovery]` (Super Heavy keeps the defaults). The SCvx descent planner
gets a third goal `Pad` (touchdown at leg-contact height, no tower); the session gets a landing zone next to the launch
site, routes `leg_landing` stages to it, deploys the legs on the guidance's command and cuts the engines on leg
contact.

**Tech Stack:** C++20 core (`gnc`, `sim`, `config`, `vehicle`), Catch2, TOML config.

**Spec:** `docs/superpowers/specs/2026-10-13-falcon9-landing-design.md` (section "c3 design").

## Global Constraints

- SI units internally; frame/unit in names; no Godot in `core`; modules depend downwards only.
- No magic numbers: tuned values are named constants or config entries with a provenance tag [S]/[E]/[D].
- Super Heavy behaviour unchanged: all existing tests (293) stay green, catch campaign still 21/21.
- UI strings in the core are German ("gelandet", "umgekippt", "Wiedereintritt").
- Evidence for Falcon 9 [D] comes from our CRS-35 webcast telemetry (`tests/validation/telemetry/f9_crs35_2026-10-13.csv`):
  - entry burn on the centre engine T+355-374 s, 56.6 -> 35.9 km, gauge 3.5 -> 3.1 % (~125 m/s);
  - landing burn ignition T+420 s at ~3.2 km, ~300 m/s on 1 engine; 3 engines T+425-429.5 s (2.0 -> 1.05 km,
    ~250 -> ~165 m/s); 1 engine to touchdown at T+442 s; gauge 2.9 % at ignition.
- Spec validation: from the ignition state < 1 m off the centre, sink at contact < 2 m/s; campaign +-10 m, +-3 m/s,
  +-5 % mass, +-3 deg, truth aero +-10 %: all land; full mission liftoff -> "gelandet".

## Review Focus

1. A Starship booster or ship return run (no `[stage.recovery]` keys) must behave exactly as before - test: the
   existing `[catch]`, `[booster]`, `[ship]` suites.
2. The single-engine final set has minimum thrust above the weight (cannot hover): the plan must not carry the
   soft exponential approach of the catch (infeasible at the end) - test: Task 2 planner test converges with
   zero virtual control.
3. A plan that ends a few decimetres above the legs' contact: the engines must still cut off (plan end + 0.5 s), and a
   drop from there must land - covered by Task 4's session test (cutoff reason reported).
4. Legs deploy on the way down only once (sticky) and early enough to be locked at touchdown (deploy time 2 s) - test
   in Task 4: legs fully out at first contact.
5. Leg landings of a body that is not returning (c2 drop tests) keep working - existing `[legs]` tests.

---

### Task 1: Return parameters from `[stage.recovery]`

**Files:**
- Modify: `core/include/starsim/config/vehicle_config.hpp`, `core/src/config/vehicle_config.cpp` (recovery keys)
- Modify: `core/include/starsim/gnc/booster_return.hpp`, `core/src/gnc/booster_return.cpp`
  (`booster_return_parameters(const config::StageConfig&)`, entry-burn fields)
- Modify: `config/vehicles/falcon9_block5.toml`, `config/README.md`
- Test: `core/tests/test_booster_return_config.cpp` (new, `[gnc][falcon9]`)

**Interfaces:**
- Produces: `config::RecoveryConfig` fields on `StageConfig`:
  `boostback_engine_groups`, `entry_engine_groups`, `landing_engine_phases` (vector of 3 group lists),
  `landing_phase_throttles` (3), `landing_switch_sink_mps` (2 x {min, max}), `return_flight_time_s`,
  `entry_burn_altitude_m`, `entry_burn_delta_v_mps`, `entry_burn_throttle`.
- Produces: `gnc::BoosterReturnParameters booster_return_parameters(const config::StageConfig& stage,
  BoosterReturnParameters base = {})` - copies every key that is set over `base`.
- Produces: `BoosterReturnParameters::entry_groups`, `entry_altitude_m` (0: no entry burn), `entry_delta_v_mps`,
  `entry_throttle`.

- [ ] Step 1: test: Falcon 9 config gives boostback `{first_stage/center, first_stage/pair}`, entry `{first_stage/center}`,
  landing phases `{center}`, `{center, pair}`, `{center}`, flight time 290 s, entry 56.6 km / 125 m/s; Starship config
  gives the unchanged defaults (`booster/center` ...). Run `starsim_core_tests "[falcon9]"`: FAIL (no function).
- [ ] Step 2: implement config parsing (arrays of arrays via toml++) and the mapping; Falcon 9 keys with tags.
- [ ] Step 3: run the test: PASS. Commit `feat: Falcon 9 return engine sets and entry burn from [stage.recovery]`.

### Task 2: SCvx pad goal

**Files:**
- Modify: `core/include/starsim/gnc/landing_site.hpp` (`PadTarget`), `descent_problem.hpp/.cpp`, `descent_planner.cpp`,
  `descent_snapshot.hpp/.cpp`, `scvx_landing.cpp`
- Test: `core/tests/test_descent_planner.cpp` (new case `[gnc][falcon9]`)

**Interfaces:**
- Produces: `struct PadTarget { double contact_bottom_height_m; double radius_m; }`; `LandingSite::pad`.
- Produces: `DescentGoal::Pad` - terminal like Catch (position, velocity, upright, no pitch/yaw rate), no corridor /
  final-set radius; the snapshot's landing target (`DescentSnapshot::catch_target`, documented as "catch or pad")
  is the CG at leg contact: `cg_station - foot_station` above the pad, sink `catch_sink_mps`.
- `ScvxLandingParameters`, `DescentLimits::approach_time_constant_s = 0` for Falcon 9 (Ruling: min thrust > weight).

- [ ] Step 1: test: an F9 stage (11.3 t propellant) at 3.2 km falling 300 m/s, phases center / center+pair / center,
  switch windows [230, 270] and [140, 190] m/s, plans to the pad: converged, end height = contact height +-0.1 m,
  sink 1 m/s, lateral < 0.1 m, final mass > dry + reserve. FAIL (no Pad goal).
- [ ] Step 2: implement; Catch-only code paths (`== Catch`) become `!= Abort` where the pad shares them.
- [ ] Step 3: PASS; `[descent]` and `[catch]` suites green. Commit `feat: SCvx pad touchdown goal`.

### Task 3: Generic return guidance with entry burn, leg deployment and leg cutoff

**Files:**
- Modify: `core/include/starsim/gnc/booster_return.hpp`, `core/src/gnc/booster_return.cpp`
- Test: `core/tests/test_falcon9_rtls.cpp` (new, `[gnc][falcon9][rtls]`)

**Interfaces:**
- Produces: `ReturnPhase::EntryBurn` ("Wiedereintritt"); `ReturnCommand::deploy_legs`;
  `BoosterReturnGuidance::set_leg_contact(bool)`.
- Entry burn: during Coast, descending, below `entry_altitude_m`, not yet flown: burn `entry_groups` at
  `entry_throttle` along -v (relative) until the burned delta-v (integrated thrust / mass) reaches `entry_delta_v_mps`.
- Landing burn on SCvx when the site has a catch target or a pad (else explicit laws); phase groups from parameters;
  legs deploy when the plan's time-to-go <= deploy time + `leg_deploy_margin_s` (5 s [E]); Landed on leg contact,
  resting/impacted, or plan end + 0.5 s.
- `meco_propellant_kg`: the entry burn's propellant by the rocket equation before the boostback term.

- [ ] Step 1: tests: (a) MECO propellant with an entry burn exceeds the one without by the rocket-equation amount;
  (b) a stage placed at 70 km falling over LZ-40 in Coast flies the entry burn on the centre engine only, burns
  ~125 m/s, ends EntryBurn -> Coast. FAIL.
- [ ] Step 2: implement. Step 3: PASS + `[booster]` green. Commit `feat: generic booster return: entry burn, legs`.

### Task 4: Session: landing zone, leg-landing return, touchdown

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp`,
  `core/include/starsim/sim/landing_outcome.hpp`, `core/src/sim/landing_outcome.cpp`,
  `core/include/starsim/vehicle/flight_body.hpp`, `core/src/vehicle/flight_body.cpp` (truth aero scale)
- Test: `core/tests/test_falcon9_rtls.cpp`

**Interfaces:**
- Produces: `FlightSession::set_landing_zone(const LaunchSite&)` (legs at the zone; the return target of
  `leg_landing` stages), `landing_zone()`; `returns_after_staging` true for `leg_landing` with a zone;
  `return_site` gives the zone + `PadTarget`; leg contact fed to the guidance; `deploy_legs` on command;
  `LandingOutcome::TippedOver` ("umgekippt") from the leg judgement; `landing_outcome` judged against the zone.
- Produces: `FlightBody::set_hull_aero_scale(double)` and `FlightSession::set_aero_dispersion(size_t, double)` (truth
  model error injection: scales the hull's aerodynamic force; the guidance's models stay nominal).

- [ ] Step 1: test "Falcon 9 lands on LZ-40 from the landing-burn ignition state": placed in LandingBurn at the
  burn prediction's ignition height for 300 m/s; expect `LegTouchdown::Landed`, offset < 1 m, sink at first foot
  contact < 2 m/s, outcome Landed. FAIL.
- [ ] Step 2: implement. Step 3: PASS. Commit `feat: Falcon 9 lands on the landing zone (RTLS touchdown)`.

### Task 5: RTLS campaign

**Files:** Test: `core/tests/test_falcon9_rtls.cpp` (`[gnc][falcon9][rtls][validation]`)

- [ ] 20 seeded dispersions (+-10 m, +-3 m/s, +-5 % propellant, +-3 deg, truth hull aero 0.9-1.1) + nominal: all
  land (< 5 m from the centre - spec success; report offset, sink, propellant left). Tune [E] parameters with rulings.
  Commit `test: Falcon 9 RTLS landing campaign`.

### Task 6: Mission and full flight

**Files:**
- Create: `config/missions/falcon9_rtls.toml`; Modify: `core/include/starsim/config/mission_config.hpp`,
  `core/src/config/mission_config.cpp` (`AscentMission`, `load_ascent_mission`)
- Test: `core/tests/test_falcon9_rtls.cpp` ("Falcon 9 RTLS mission: liftoff to landed")

- [ ] Mission: vehicle `falcon9_block5`, site `ccsfs_slc40`, landing zone `ccsfs_lz40`, payload 12.5 t [E], 200 km /
  51.6 deg, kick 14 deg, throttle bucket as c1. Test flies liftoff -> orbit and booster -> "gelandet"; prints MECO
  propellant (CRS-35: 61 t), entry burn and landing-burn ignition state vs the telemetry (3.2 km / 300 m/s).
  Commit `feat: Falcon 9 RTLS mission`.

### Task 7: Docs

- [ ] CLAUDE.md (Falcon 9 return), `config/README.md`, ledger; full ctest, smoke test. Commit `docs: Falcon 9 RTLS (c3)`.
