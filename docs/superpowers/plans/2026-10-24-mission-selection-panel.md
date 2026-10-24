# Mission Selection Panel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Choose place -> vehicle -> mission in the panel and start any configured mission (incl. CRS-35 / flight 14 at
their historical liftoff and the refuelling campaigns) from the game.

**Architecture:** Missions declare their start (`start_place`, `start_vehicle`), pads list their free-flight vehicles,
non-pad places live in `config/start_places.toml`. A GDScript `MissionCatalogue` reads that and filters; the panel shows
three boxes and calls `FlightSim.start_mission(id, tdb, true)` through `FlightController`. The core gains one optional
ascent key `liftoff_utc`, and a core test keeps the declared start vehicle / place in line with what `start_mission`
really starts.

**Tech Stack:** C++20 core (Catch2), GDScript (Godot 4.7 double), TOML.

**Spec:** `docs/superpowers/specs/2026-10-24-mission-selection-panel-design.md`

## Global Constraints

- `game/` reads TOML (TomlLite) and sends commands; no physics in GDScript; no hard-coded mission / vehicle lists.
- UI text German, code / comments / commits English (Conventional Commits), commits end with the session attribution.
- Shown names: missions by `[mission] display_name` (fallback `name`), vehicles by the vehicle TOML's top
  `display_name` (fallback `name`), places by their `name`; no trademarks on screen.
- Never offered as a vehicle: `ejection_seat`, `parachutist`, `iss`. Never listed as a mission: files without
  `start_place` (`ship_return_starbase`, `ship_return_starbase_crewed`, `leo_ascent`).
- Pad missions start with `wait_on_pad = true` (user 2026-10-24); real missions at their historical liftoff.
- Tests: only the touched tags (`-TestFilter "[mission]"` etc.), never the full suite; push after every task.

## Review Focus

1. A mission whose core start refuses (no launch window, plan infeasible): the panel shows the refusal text, the start
   button is enabled again, the previous flight state is not left half-replaced in the HUD. -> Task 4 manual check +
   `_on_command_finished` handling of `start_mission` with `ok == false`.
2. Clock jump backwards (crew_dragon_return 2026-10-14 while the clock is later): the world clock must be set to
   `mission_start_tdb_s()` after the command, warp 1. -> Task 4 step.
3. Box refill keeps the previous choice when still offered, and a place with only orbit missions never shows
   "Freier Flug". -> Task 3 test `refill keeps selection` is GDScript-pure: `MissionCatalogue.index_of(list, id)`.
4. Start pressed again while a start is planning (~45 s): ignored (button disabled). -> Task 4.
5. A mission file with a typo in `start_vehicle` / `start_place`: caught by the core consistency test (Task 2) and the
   catalogue test (Task 3), not silently hidden in the game.

---

### Task 1: Ascent `liftoff_utc` (historical liftoff)

**Files:**
- Modify: `core/include/starsim/config/mission_config.hpp` (`AscentMission`), `core/src/config/mission_config.cpp`
  (`load_ascent_mission`)
- Modify: `core/src/sim/mission_start.cpp` (Ascent case), `core/include/starsim/sim/mission_start.hpp` (doc comment)
- Modify: `config/missions/crs35_ascent.toml`, `config/missions/flight14_ascent.toml`, `config/README.md`
- Test: `core/tests/test_mission_start.cpp`

**Interfaces:**
- Produces: `std::optional<std::string> AscentMission::liftoff_utc;` (`[mission] liftoff_utc`, ISO UTC like the other
  `epoch_utc` keys).

- [ ] **Step 1: Failing tests** in `test_mission_start.cpp`, tag `[mission][sim]`:
  - `"An ascent mission with liftoff_utc lifts off at it, whatever the requested epoch"`: start `flight14_ascent` with
    `requested` = liftoff + 3 days, `wait_on_pad = true` -> the timeline's T-0 (or the start's liftoff epoch as the
    existing ascent test reads it) equals `time.from_utc(liftoff_utc)` within 1e-6 s, and `s.epoch` lies
    `countdown` (mission `countdown_s` or the option default) before it; with `wait_on_pad = false` T-0 equals the
    liftoff and `s.epoch` equals it.
  - `"liftoff_utc is loaded"`: `load_ascent_mission(...,"crs35_ascent").liftoff_utc` has the flights.toml date; an
    ascent mission without the key (`starship_leo`) has none (`[mission][config]`).
- [ ] **Step 2:** Run `tools/build.ps1 -NoBindings -TestFilter "[mission]"` -> the new cases fail.
- [ ] **Step 3: Implement.** Loader reads the optional string. In the Ascent case: `liftoff = m.liftoff_utc ?
  time.from_utc(*m.liftoff_utc) : requested + countdown_s`; `placed = m.liftoff_utc ? liftoff - countdown_s : requested`;
  pass `liftoff` and `wait_from = wait_on_pad ? placed : nullopt` to `start_ascent_mission_for_liftoff`; `s.epoch =
  placed`. The sea origin (`set_sea_state(..., requested)` above the switch) must use the same `placed` epoch for such a
  mission: compute the ascent's effective start before `set_sea_state` (load the ascent mission there only for
  `MissionKind::Ascent`). Keep the existing T-0 lead rule (T-0 >= placement + `liftoff_ignition_lead_s()`).
- [ ] **Step 4: Config.** `liftoff_utc` from `tests/validation/telemetry/flights.toml` [S]: flight 14
  `"2026-09-28T12:48:59"`; CRS-35 the entry whose outcome is "landed at LZ-40 (RTLS)" (`"2026-10-13T10:33:00"`) -
  verify by the entry's id/name before writing; cite `[S:flights.toml]`. Document the key in `config/README.md`.
- [ ] **Step 5:** Run `-TestFilter "[mission],[validation]"` (the CRS-35 / flight 14 validation cases read these files;
  if their tag differs, find it with `grep -l crs35 core/tests` and add it) -> all pass.
- [ ] **Step 6: Commit + push** `feat(sim): ascent missions lift off at their configured liftoff_utc`.

### Task 2: Start declarations in the config + core consistency test

**Files:**
- Create: `config/start_places.toml`
- Modify: every selectable `config/missions/*.toml` (`[mission] start_place`, `start_vehicle`),
  `config/launch_sites.toml` (`vehicles = [...]` on every pad), `config/README.md`
- Create: `core/tests/test_mission_start_places.cpp` (add to the tests CMake list like its neighbours)

**Interfaces:**
- Produces (data, read by Task 3):
  - `config/start_places.toml`: `[[place]]` with `id`, `name`:
    `earth_orbit` "Erdorbit", `iss_docked` "ISS (angedockt)", `lunar_orbit` "Mondumlaufbahn".
  - `[mission] start_place = "<pad id | place id>"`, `start_vehicle = "<vehicle file id>"`.
  - `[[site]] vehicles = ["..."]` on sites without `kind` (pads): `starbase_olp1` and `ksc_lc39a`: `starship_block3`,
    `starship_depot`, `starship_tanker`, `starship_hls_launch`; `ccsfs_slc40`: `falcon9_block5`. Landing zones get no
    key.

- [ ] **Step 1: Failing test** `test_mission_start_places.cpp`, tags `[mission][config]`, reads TOML with the core's
  TOML reader (as `mission_config.cpp` does), no flights:
  - `"Every mission file declares a start or is a known follow-on"`: for each file in `config/missions`: either both
    keys present, or the id is in `{ship_return_starbase, ship_return_starbase_crewed, leo_ascent}`.
  - `"Start places and vehicles exist"`: `start_place` is a pad id of launch_sites.toml or a start_places.toml id;
    `config/vehicles/<start_vehicle>.toml` exists; start_vehicle not in `{ejection_seat, parachutist, iss}`.
  - `"The declared start matches what start_mission starts"` - per kind, from the kind's loader (no session):
    Ascent: `vehicle_id == start_vehicle`, `site_id == start_place`; IssLaunch: its ascent mission's vehicle / site;
    DepotLaunch: its depot ascent mission's vehicle / site; TankerFlight / HlsRefuel: the chaser's ascent mission
    (`ascent_id`) vehicle / site when the start mode is pad, else `start_place == "earth_orbit"` and that vehicle; RefuellingCampaignFlown: `[depot] start`
    launch -> the depot ascent's vehicle + pad, placed -> `earth_orbit` + `starship_depot`; CapsuleReturn /
    IssRendezvous: `earth_orbit` + `crew_dragon`; IssDeparture: `iss_docked` + the mission's vehicle; LunarLanding:
    `earth_orbit` + its vehicle; LunarReturn: `lunar_orbit` + its vehicle. Use the loaders' existing fields (e.g.
    `RefuellingFlightMission::ascent_id`); where a loader lacks the vehicle, read the referenced ascent mission.
  - `"Every pad lists vehicles that exist and may stand there"`: no ejection_seat / parachutist / iss / crew_dragon /
    starship_hls in any `vehicles` list.
- [ ] **Step 2:** Run `-TestFilter "[mission]"` -> fails (keys missing).
- [ ] **Step 3: Data.** Add the keys by the spec's table (`docs/superpowers/specs/2026-10-24-mission-selection-panel-design.md`
  "Initial assignment"); where the test shows the core starts something else, the core wins - fix the table row in the
  spec too. Create `start_places.toml` with a header comment (what it is, who reads it). LC-39A gets `falcon9_block5`
  only if `launch_from_site("falcon9_block5", "ksc_lc39a")` is accepted by the core (check its site/tower rules;
  otherwise leave it out and say why in a comment).
- [ ] **Step 4:** Run `-TestFilter "[mission]"` -> pass.
- [ ] **Step 5: Commit + push** `feat(config): missions declare their start place and vehicle`.

### Task 3: `MissionCatalogue` (GDScript)

**Files:**
- Create: `game/ui/mission_catalogue.gd` (`class_name MissionCatalogue extends RefCounted`)
- Create: `game/tests/mission_catalogue_test.gd` (pattern of `game/tests/toml_lite_test.gd`)

**Interfaces:**
- Consumes: Task 2 data; `TomlLite.load_file`, `Paths.config_dir()`.
- Produces:
  - `static func load_from(config_dir: String) -> MissionCatalogue`
  - `func places() -> Array[Dictionary]`: `{id, name, site}` (`site`: the launch_sites.toml entry for a pad, `{}`
    otherwise); pads first in file order, then start_places.toml order; only places with >= 1 vehicle.
  - `func vehicles_at(place_id: String) -> Array[Dictionary]`: `{id, name}`, union of the pad's `vehicles` and the
    `start_vehicle`s of missions with that `start_place`, ordered by vehicle file id.
  - `func missions_for(place_id: String, vehicle_id: String) -> Array[Dictionary]`: `{id, name}`; first
    `{id: "", name: "Freier Flug"}` when the place is a pad listing that vehicle, then the matching missions by name.
  - `static func index_of(list: Array, id: String) -> int` (-1 when absent) - for keeping a selection on refill.
  - `const FREE_FLIGHT := ""`

- [ ] **Step 1: Failing test** `mission_catalogue_test.gd` against the real `config/`:
  - places contain `starbase_olp1`, `ccsfs_slc40`, `earth_orbit`, `iss_docked`, `lunar_orbit`; not `ccsfs_lz40`.
  - `vehicles_at("ccsfs_slc40")` ids == `["falcon9_block5"]`; `vehicles_at("earth_orbit")` contains `crew_dragon`,
    `starship_hls` and not `falcon9_block5`; no place offers `ejection_seat`, `parachutist`, `iss`.
  - `missions_for("ccsfs_slc40","falcon9_block5")[0].id == ""` and contains `falcon9_rtls`, `crs35_ascent`,
    `falcon9_crew_dragon_iss`; `missions_for("lunar_orbit","starship_block3")` has 4 entries, none free flight;
    no list contains `ship_return_starbase` or `leo_ascent`.
  - names: the RTLS entry's name is its `display_name` ("Medium Launcher: Landung am Startgelände"); the SLC-40
    vehicle's name is "Medium Launcher".
  - `index_of([{id:"a"},{id:"b"}], "b") == 1`, `index_of([], "x") == -1`.
- [ ] **Step 2:** Run `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script
  res://tests/mission_catalogue_test.gd` (after one `--headless --path game --import` for the new class_name) ->
  FAILED.
- [ ] **Step 3: Implement** the class (vehicle names from the vehicle TOML's first table holding `display_name`, as
  the file's top-level vehicle entry - check how `crew_dragon.toml` nests it).
- [ ] **Step 4:** Run the test -> `MISSION CATALOGUE TEST PASSED`; `toml_lite_test.gd` still passes.
- [ ] **Step 5: Commit + push** `feat(game): mission catalogue filtered by place and vehicle`.

### Task 4: Panel boxes, mission start, docs

**Files:**
- Modify: `game/ui/time_panel.gd` (replace the vehicle/site boxes + "Auf die Rampe stellen")
- Modify: `game/vehicle/flight_controller.gd` (mission start + its command outcome), `game/world/world.gd`
  (`start_mission` next to `launch_vehicle`)
- Modify: `docs/ansichten.md` (panel), `CLAUDE.md` (game section: one line on the catalogue), `game/tests/smoke_test.gd`
  only if it touched the removed boxes

**Interfaces:**
- Consumes: Task 3 `MissionCatalogue`; `FlightSim.start_mission(id, tdb_s, wait_on_pad) -> int`,
  `FlightSim.mission_start_tdb_s()`, `result(id)`, signal `command_finished(id, command, ok, message)`.
- Produces:
  - `FlightController.start_mission(mission_id: String, tdb_s: float) -> bool` (clears views like `launch`, posts with
    `wait_on_pad = true`, `_pending_mission = {id, mission_id}`, `last_event = PLANNING_EVENT`).
  - `FlightController.signal mission_started(ok: bool, message: String)` emitted from `_on_command_finished` for
    `"start_mission"`.
  - `World.start_mission(mission_id: String) -> bool`: calls the controller with `clock.tdb_s`; on `mission_started(ok)`
    sets `clock.set_time(flight.sim.mission_start_tdb_s())`, `clock.warp = 1.0`, `set_focus(FOCUS_VEHICLE)`.

- [ ] **Step 1: Panel.** Under "Start vorbereiten": `OptionButton`s Ort / Fahrzeug / Mission from
  `MissionCatalogue.load_from(Paths.config_dir().path_join(""))` (or the config dir path the catalogue expects), a
  button "Starten". `item_selected` of Ort refills Fahrzeug, of Fahrzeug refills Mission, keeping the previous id via
  `index_of`. "Starten": free flight -> `world.launch_vehicle(vehicle_id, place.site)` (unchanged behaviour), else
  `world.start_mission(mission_id)`; the button is disabled until `mission_started` / the launch outcome; a refusal goes
  to a status label under the button ("Start abgelehnt: <message>"), success clears it; while the start runs it shows
  "Start wird geplant ...".
- [ ] **Step 2: Controller + world** as in Interfaces; the existing `launch_from_site` handling stays. Note: the
  flown refuelling campaign's core start takes no `wait_from` (it starts at its own epoch); that is accepted, record it
  in the spec's amendments instead of changing the core.
- [ ] **Step 3: Checks.** Smoke test passes; `view_target_test.gd`, `webcast_overlay_test.gd` pass. Manual desktop
  check (`Start-Desktop.cmd`, user or Claude with render/flight_check if possible): SLC-40 -> Medium Launcher -> RTLS
  starts with the countdown; Erdorbit -> Crew Capsule -> "Rückkehr der Crew-Kapsel" jumps the clock to 2026-10-14 and
  follows the capsule; a free flight still works. Report what was checked by whom.
- [ ] **Step 4: Docs.** `docs/ansichten.md`: the panel's three boxes and their filtering; CLAUDE.md game section one
  line (`game/ui/mission_catalogue.gd`, config keys `start_place` / `start_vehicle`, `config/start_places.toml`).
- [ ] **Step 5: Commit + push** `feat(game): start missions from the panel by place, vehicle and mission`.

### Final

- [ ] Whole-change review (tags `[mission]` + the validation tag of Task 1 + the GDScript tests); update the memory
  (`project-milestone-order.md`) and ask the user for `/clear`, naming the next step (rest of M8).
