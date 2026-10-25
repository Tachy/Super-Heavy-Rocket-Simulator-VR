# Falcon 9 c4: landing on the drone ship (ASDS) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Falcon 9 first stage on a Starlink-class ascent from SLC-40 coasts without a boostback, turns engines
first on its RCS, flies a three-engine entry burn that steers its impact onto a stationary drone ship, lands on one
engine on SCvx plans on the deck and is judged "gelandet"; a miss of the deck ends in the water ("Wasserung").

**Architecture:** The drone ship is configuration (`config/drone_ships.toml`: deck size, freeboard) placed by the
mission at its nominal impact point [D]. A drone ship becomes a landing zone (`sim::LaunchSite` with a deck): the leg
landing gets the deck surface (`contact::LandingSurface::Deck`, from c2), the return guidance a `PadTarget` at deck
height and a `SeaSide` beside the ship for the abort. The Falcon 9 vehicle carries two return profiles: the RTLS one
(`[stage.recovery]`) and a drone-ship profile (`[stage.recovery.drone_ship]`, overrides) that has no boostback. Without
a boostback the guidance starts in its coast phase (RCS turn engines first) and the MECO planning counts only the
run-down, the entry burn and the landing propellant. A small `sim/ascent_mission` helper sets up a session from an
`AscentMission` (shared by the tests and, in c5, the game).

**Tech Stack:** C++20 core (`config`, `contact`, `gnc`, `sim`), Catch2, TOML config.

**Spec:** `docs/superpowers/specs/2026-10-13-falcon9-landing-design.md` (section "c4 design").

## Global Constraints

- SI units internally; frame/unit in names; no Godot in `core`; modules depend downwards only.
- No magic numbers: tuned values are named constants or config entries with a provenance tag [S]/[E]/[D].
- Super Heavy, ship and the c3 RTLS behave exactly as before: all 303 tests stay green.
- UI strings in the core are German ("Drohnenschiff", "gelandet", "Wasserung").
- Drone ship (spec): deck 91 x 52 m [S], ~3 m above the sea [E], stationary and level (no sea state), display name
  "Drohnenschiff"; placed at the mission's nominal impact point [D].
- Evidence [D] from our Starlink 15-25 webcast telemetry (`tests/validation/telemetry/f9_starlink15-25_2026-10-12.csv`):
  - MECO T+149 s, ~60 km, ~8,000 km/h, gauge ~11 % (~43 t);
  - entry burn: centre engine lit T+364 s at 67.4 km, three engines T+367.5-385 s, out by 45 km; speed 7,926 km/h
    (T+361) -> 5,245 km/h (T+392); gauge 6.1 % (24.1 t) at ignition;
  - landing burn on one engine from ~3.0 km (T+476 s, ~270 m/s) to touchdown at ~T+499 s; gauge 2.8 % (11.1 t) at
    ignition, 1.7 % at touchdown.
- Spec validation: campaign `[falcon9][asds][validation]` "as for RTLS plus dispersed MECO states; all land on the
  deck"; full mission lands on the deck.

## Review Focus

1. RTLS must keep its profile: the drone-ship overrides apply only when the landing zone is a deck - test: the c3
   `[rtls]` suite unchanged, plus a config test that the RTLS parameters still have the boostback and centre-engine
   entry burn.
2. A stage that touches the sea next to the deck (feet beyond the deck edge) is "Wasserung", not "gelandet" - test in
   Task 1 (drop beside the deck).
3. Without a boostback the guidance must not run the Flip/Boostback phases nor plan a Lambert arc at MECO - test in
   Task 2 (MECO plan = run-down + entry burn + landing propellant, independent of the target's position).
4. The RCS turn after separation (nose first at ~60 km, ~700 Pa) must end engines first before the entry burn - the
   full mission (Task 4) records the attitude error at the entry-burn ignition.
5. A deck that is out of reach aborts to the water beside the ship - test in Task 5.

---

### Task 1: Drone ship config and the deck as a landing zone

**Files:**
- Create: `config/drone_ships.toml`, `core/include/starsim/config/drone_ship_config.hpp`,
  `core/src/config/drone_ship_config.cpp`
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (`LaunchSite::deck`,
  `set_landing_zone` builds the deck surface, `return_site` gives a `PadTarget` at the deck and a `SeaSide` beside it),
  `core/CMakeLists.txt`
- Test: `core/tests/test_falcon9_asds.cpp` (new): config loads; a stage dropped onto the deck lands; one dropped beside
  the deck splashes down.

**Interfaces:**
- Produces: `config::DroneShipConfig{id, name, deck_length_m, deck_width_m, freeboard_m}`,
  `config::load_drone_ship(config_dir, id)`.
- Produces: `sim::LaunchSite::deck` (`std::optional<contact::LandingSurface::Deck>`),
  `sim::drone_ship_site(const config::DroneShipConfig&, latitude_rad, longitude_rad, sea_height_m, heading_rad)`:
  height = sea + freeboard, `pad_radius_m` = half the deck width, `sea` = shoreline at the deck's side edge
  (direction across the deck, distance half width) so the SCvx abort aims beside the ship.

- [ ] Write the config test and the two drop tests (stage 2 m above the deck, legs out, 2 m/s sink: "gelandet" on the
      deck; 40 m beside the long edge: "Wasserung"). Run: FAIL (symbols missing).
- [ ] Implement loader, `drone_ship_site`, deck surface in `set_landing_zone`, `return_site`. Run `[asds]`: PASS.
- [ ] Commit `feat: drone ship deck as a Falcon 9 landing zone`.

### Task 2: Drone-ship return profile without a boostback

**Files:**
- Modify: `vehicle_config.{hpp,cpp}` (return keys grouped in `config::ReturnProfile`; `StageConfig::return_profile`
  and `std::optional<ReturnProfile> drone_ship_return` from `[stage.recovery.drone_ship]`, keys override the RTLS
  profile; new key `boostback` (default true)), `booster_return.{hpp,cpp}` (`booster_return_parameters(stage_id,
  profile, base)`, `BoosterReturnParameters::boostback`, guidance starts in Coast without one, `meco_propellant_kg`
  without flip/boostback), `flight_session.cpp` (drone-ship profile when the landing zone has a deck),
  `config/vehicles/falcon9_block5.toml`, `config/README.md`
- Test: `test_booster_return_config.cpp` (drone-ship profile; RTLS unchanged), `test_falcon9_asds.cpp` (MECO plan
  without boostback; a stage placed at the entry-burn altitude flies the three-engine entry burn; a stage placed at
  ignition over the deck lands on one engine).

**Interfaces:**
- Produces: `config::ReturnProfile` (fields as the c3 return keys + `bool boostback = true`),
  `gnc::booster_return_parameters(const config::StageConfig&, bool drone_ship, BoosterReturnParameters base = {})`.
- Values [D] Starlink 15-25: entry burn on `["center", "pair"]` from 66 km [D: three engines from 64-67 km], delta-v
  ~880 m/s [D: 745 m/s slower over T+361-392 s plus ~125 m/s of gravity along the path, minus drag; the rocket
  equation from 24.1 t to 11.3 t at Isp_vac 311 s gives 880 m/s: the gauge agrees], throttle from the burn time
  (~18 s on three engines); landing burn on `["center"]` in all three phases; switch windows wide (no engine
  change).

- [ ] Write the tests. Run: FAIL.
- [ ] Implement. Run `[falcon9]`, `[rtls]`, `[asds]`: PASS.
- [ ] Commit `feat: Falcon 9 drone-ship return profile (no boostback, three-engine entry burn)`.

### Task 3: Ascent mission helper and the ASDS mission

**Files:**
- Create: `core/include/starsim/sim/ascent_mission.hpp`, `core/src/sim/ascent_mission.cpp`
  (`sim::start_ascent_mission(FlightSession&, const config::AscentMission&, const std::filesystem::path& config_dir,
  time::Epoch)`: vehicle with payload/fairing, pad, landing zone or drone ship, booster return, autopilot);
  `config/missions/falcon9_asds.toml`
- Modify: `mission_config.{hpp,cpp}` (`[drone_ship]` table: `id`, `latitude_deg`, `longitude_deg`, `heading_deg`,
  `sea_height_m`), `core/tests/test_falcon9_rtls.cpp` (full mission uses the helper)
- Test: `test_falcon9_asds.cpp` (mission loads; hidden diagnostic `[.]` prints the nominal impact point).

- [ ] Write the load test; run FAIL; implement; PASS.
- [ ] Run the nominal mission with the diagnostic, put the impact point into the mission file [D].
- [ ] Commit `feat: Falcon 9 drone-ship mission (Starlink class, SLC-40)`.

### Task 4: Full ASDS mission and campaign

**Files:**
- Test: `test_falcon9_asds.cpp`: full mission (liftoff -> "gelandet" on the deck, < 5 m from its centre; timeline
  printed against Starlink 15-25); campaign `[falcon9][asds][validation]`: from the nominal MECO state (recorded once
  per run from the nominal ascent) with dispersions (+-200 m position, +-5 m/s velocity, +-5 % propellant, truth aero
  +-10 %) plus from the ignition state as for RTLS; all land on the deck.
- Tune [E]/[D] values only from evidence; ledger every ruling.

- [ ] Write the tests; run; fix causes (systematic debugging), not symptoms.
- [ ] Commit `test: Falcon 9 drone-ship mission and landing campaign`.

### Task 5: Abort beside the ship, docs

**Files:**
- Test: `test_falcon9_asds.cpp`: a stage lit 900 m off the deck aborts and ends in the water.
- Modify: `CLAUDE.md` (Falcon 9 ASDS bullet, known issues), `config/README.md` (drone ships, profile, mission keys).

- [ ] Test, docs, full ctest, bindings build, smoke test.
- [ ] Commit `docs: Falcon 9 drone ship (c4)`.
