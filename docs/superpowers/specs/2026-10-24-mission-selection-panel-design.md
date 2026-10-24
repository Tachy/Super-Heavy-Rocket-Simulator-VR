# Mission selection in the panel - design

Date: 2026-10-24. Status: draft for the user's review.

## Intent

The core can start every configured mission (`sim::start_mission`, bound as `FlightSim.start_mission`), but the game
never calls it: the panel's "Start vorbereiten" block only places a vehicle on a pad (`world.launch_vehicle`), the
missions are reached only through tests or the onboard keys (P, O, R). The user wants to fly the configured missions -
including the real ones (CRS-35, flight 14) and the refuelling campaigns - from the game, chosen in the rudimentary
panel until the M7 cockpit planner replaces it (user 2026-10-24). The choice is the same one the planner will offer
later, so it is data-driven from `config/` and the game only reads it.

## The choice (user's decisions)

- Three boxes under the vehicle placement, filtered top-down: **place -> vehicle -> mission** (user: "erst Ort, dann
  passende Fahrzeuge und zuletzt dazu passende Missionen. dreistufig gefiltert").
- Places and vehicles are filtered sensibly: a vehicle appears only where it can really start (user: "der
  Schleudersitz oder die Kapsel kann ja nicht auf der Rampe stehen").
- Places are not only pads: orbit starts (Earth orbit, ISS docked, lunar orbit) are places too (user agreed).
- Real missions (validation flights) and the multi-ship campaigns are selectable (user: "will ja auch mal echte
  Missionen nachfliegen").

## Goals

1. Place box: the Earth pads of `config/launch_sites.toml` (kind pad, not landing zones) plus the non-pad start places
   below, each shown only if at least one vehicle can start there.
2. Vehicle box: the vehicles that can start at the chosen place - at a pad the vehicles the pad lists (free flight) and
   the start vehicles of the missions starting there; at a non-pad place only mission start vehicles.
3. Mission box: the missions whose start place and start vehicle match, by their `display_name` (no trademarks, as
   the webcast overlay; fallback `name`) [call]; at a pad additionally
   "Freier Flug" (today's "Auf die Rampe stellen": countdown, then P / O / R by key), listed first.
4. One start button. "Freier Flug" calls `world.launch_vehicle` as today; a mission calls `FlightSim.start_mission`
   with the world clock's TDB as `requested`. The world follows the vehicle (focus as after a pad launch), the clock
   jumps to the mission's epoch where the core places it at its own epoch (it does so today: orbit epochs, launch
   windows; it may jump backwards, e.g. crew_dragon_return), and a refusal or planning failure is shown in the panel's
   info line (the command's `result(id)`).
   - Missions that lift off from a pad start with `wait_on_pad = true` (user 2026-10-24): the vehicle stands on the
     pad through the countdown, as in free flight; with a launch window (iss_launch, depot_launch, tanker_flight,
     hls_refuel, the flown campaign) it waits on the pad until the window and the player warps ahead.
   - Real missions are flown at their historical liftoff (user 2026-10-24): `crs35_ascent` and `flight14_ascent` get
     `[mission] liftoff_utc` (the [S] dates of `tests/validation/telemetry/flights.toml`; builder matches the right
     entries) and the core's ascent start path uses it instead of `requested` (T-0 = liftoff_utc, placed the countdown
     before it with `wait_on_pad`); the clock jumps there. Ascent missions without the key keep `requested`.
5. Changing a box refills the boxes below it and keeps the previous selection when it is still offered.
6. All filtering comes from configuration through one GDScript catalogue (`game/` reads TOML only) that the M7 planner
   can reuse; the panel only displays it.

## Non-goals

- No free placement in orbit or at the Moon without a mission (orbit places offer only missions).
- No mission editing, parameters or a mission briefing beyond the name; no graphical planner (M7).
- No change to how missions fly; the core's `start_mission` changes only by the optional ascent `liftoff_utc`.
- Follow-on missions that are not starts stay onboard commands: `ship_return` (the core refuses it as a start) and the
  key-P target file `leo_ascent.toml` are never listed.

## Anti-goals

- A vehicle offered at a place where it cannot stand (capsule, ejection seat, parachutist, ISS, the bare lunar HLS on a
  pad).
- A mission in the list that the core refuses at once because of the chosen place or vehicle.
- Hard-coded mission or vehicle lists in GDScript.

## Data (Claude's calls, to be confirmed in review)

The existing `[mission] site` is ambiguous (a pad, the lunar landing site, the ship's return site) and several missions
have no top-level `vehicle` (iss_launch, depot_launch, tanker_flight, hls_refuel, campaigns). So the start is declared
explicitly:

- Every selectable mission gets under `[mission]`:
  `start_place = "<id>"` (a pad id of `launch_sites.toml` or a place id below) and
  `start_vehicle = "<vehicle id>"` (the vehicle the player sees at the start). A mission without `start_place` is not
  listed (ship_return, leo_ascent).
- Non-pad places in a new `config/start_places.toml` (`[[place]] id, name`): `earth_orbit` ("Erdorbit"),
  `iss_docked` ("ISS (angedockt)"), `lunar_orbit` ("Mondumlaufbahn"). Phasing, separation and depot orbits all count as
  `earth_orbit`; the mission name tells them apart.
- Free flight: each pad in `launch_sites.toml` gets `vehicles = [...]`, the vehicles that may stand on it (a pad
  knows its mount: Starship needs the tower pad). Initial values: Starbase OLP-1 and KSC LC-39A the Starship variants
  with a booster (`starship_block3`, `starship_depot`, `starship_tanker`, `starship_hls_launch`); CCSFS SLC-40
  `falcon9_block5`. LC-39A also lists `falcon9_block5` only if the core can launch it there (builder checks).

Initial assignment (builder verifies each against the mission's code path and fixes the table where the core says
otherwise):

| Mission file | start_place | start_vehicle |
|---|---|---|
| falcon9_rtls, falcon9_asds, falcon9_asds_sea, falcon9_crew_dragon, crs35_ascent, falcon9_crew_dragon_iss | ccsfs_slc40 | falcon9_block5 |
| starship_leo, flight14_ascent | starbase_olp1 | starship_block3 |
| depot_ascent, depot_launch, refuelling_campaign_flown | starbase_olp1 | starship_depot |
| tanker_ascent, tanker_flight | starbase_olp1 | starship_tanker |
| hls_ascent, hls_refuel | starbase_olp1 | starship_hls_launch |
| tanker_flight_orbit | earth_orbit | starship_tanker |
| hls_refuel_orbit | earth_orbit | starship_hls_launch |
| refuelling_campaign_flown_placed | earth_orbit | starship_depot |
| hls_lunar_llo, hls_lunar_llo_depot, hls_lunar_nrho | earth_orbit | starship_hls |
| crew_dragon_iss, crew_dragon_return | earth_orbit | crew_dragon |
| crew_dragon_iss_departure | iss_docked | crew_dragon |
| ship_lunar_return_llo, ship_lunar_return_llo_crewed, ship_lunar_return_nrho, ship_lunar_aircatch_llo | lunar_orbit | starship_block3 |
| ship_return_starbase(_crewed), leo_ascent | - (not listed) | - |

The vehicle box shows the vehicle's display name from its TOML (no trademarks on screen, as the webcast overlay), not
the file id as today.

## Constraints

- Architecture: `game/` reads state and sends commands; the catalogue is GDScript over `TomlLite`, no physics.
- UI text German, code English; the panel lives in `game/ui/time_panel.gd` (keep it small: the catalogue in its own
  script, e.g. `game/ui/mission_catalogue.gd`).
- Long starts: some missions plan for seconds to ~45 s (pad ascents, windows). The panel shows that the start is being
  planned (the snapshot's planning flag / `autopilot_status`, already exposed) and disables the start button until
  the command finishes.
- The VR overlay panel mirrors the desktop panel; the boxes must stay usable with the VR pointer (`vr_panel_mouse.gd`).

## Tests

- A headless GDScript test of the catalogue: every mission file either has a valid `start_place` + `start_vehicle`
  (place exists, vehicle file exists) or is a known non-start; no listed vehicle is ejection_seat / parachutist / iss;
  the filters return the expected sets for a pad, `earth_orbit` and `lunar_orbit`.
- A core test (or an extension of an existing one) that each listed mission's `start_vehicle` is the vehicle
  `sim::start_mission` actually puts in front of the player, so the table cannot drift - builder decides where this
  check lives cheapest (it must not fly the missions; reading the configs and the start dispatch is enough).
- Smoke test still passes; one manual desktop check: choose SLC-40 -> Falcon 9 -> RTLS, start, and an orbit start
  (Crew-Dragon-Rückkehr).

## Left to the builder

- Box layout details, the exact refusal / planning texts, whether the old "Auf die Rampe stellen" button disappears
  (it does in function: "Freier Flug" + start replaces it).
- How the clock and focus are set after a non-pad mission start (follow what the existing tests of each mission kind
  do).
- Docs: `docs/ansichten.md` (panel), CLAUDE.md game section one line, `config/README.md` for the new keys.

## Amendments

- 2026-10-24 (Task 4): the flown refuelling campaign's core start takes no `wait_from`: started from the panel it
  begins at its own epoch, not at the clock's instant. Accepted; the core is not changed.
- 2026-10-24 (Task 4): `FlightSim.start_mission` takes an optional `countdown_s` (`sim::StartOptions::countdown_s`); the
  panel passes `FlightController.LAUNCH_COUNTDOWN_S` with `wait_on_pad = true`, so pad missions count down like a free
  flight (a mission's own `[mission] countdown_s` wins) and wait on the pad for a launch window.
