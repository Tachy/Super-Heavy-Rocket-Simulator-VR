# Known issues - history

Entries removed from docs/known-issues.md because they were resolved or superseded; verbatim, with the date they were moved.

## Moved 2026-11-07

Resolved or superseded by later entries (trim margin green since 2026-11-06, entry residue vent Task 12b, the 2026-11-06 pre-existing-reds fixes, planner plan Tasks 3b / 4 / 9 / 12).

- Planner plan Task 12 re-measure (2026-11-04, `.superpowers/sdd/2026-11-03-planner-vehicle-parameters/task-12-report.md`):
  the plan's touched tags with `[slow]` (1143 cases, 1502 s) left 10 red; after the re-measures and test-premise fixes
  two stayed red, both entry trim with the 177 t ship (below): "The ship trims belly first with control margin on its
  flaps" (still red) and "Entry guidance brings the ship to the Starbase flip gate from dispersed entries" (cases 8, 9,
  11; green since Task 12b, the entry residue vent). Green
  in that run: the settling cases, "Lunar return campaign (direct): seed 2", "Docked tanker: the ship return flies to
  the catch", `plan_speed`, "Flaps pinned at their stops", every ship return / catch / lunar / Falcon 9 / Dragon campaign.
  Nightly (30 cases, 1931 s): 29 green incl. the full flown refuelling campaign against the calculator; red "Lunar
  return campaign (direct): seeds 10-12" (seed 11, below). The other 149 default-tier cases of the plan's tags green.

  (Moved because the trim-margin case is green since 2026-11-06 and the entry guidance cases since Task 12b; the open seed 11 red stays in the "Red list on the frozen aero state".)

- Sentence of the "Planner plan close-out" entry (resolved, comparison passed): Refuelling offsets re-calibrated 2026-11-04
  (7b8a42e): flown campaign 7 tankers / 32.1 d, calculator 7 / 25.25 d, the tool's
  comparison passed (`refuelling-recal-report.md`).

- Entry trim with main-tank residue: closed by planner plan Task 12b (controller ruling 2026-11-04, a planner item):
  the return planner vents the main-tank residue the flaps cannot trim before the entry interface
  (`gnc::entry_residue_limit`, 5.85 t main on top of the full headers for the 177 t ship; `ship-entry-return.md`).
  Entry guidance campaign 12 of 12 (cases 4 / 7 / 8 / 9 / 11 vent 5.0 / 3.2 / 20.7 / 13.6 / 15.5 t; gate <= 0.08 m,
  belly <= 0.13 deg, sink <= 85 m/s). The measurement before: `docs/history/guidance-missions.md`.
- Default-tier run after the test tiers (2026-11-03, `.superpowers/sdd/2026-11-03-test-tiers/full_after.log`): 7 of 1513 red at that run (the "Docked tanker" case is green since 2026-11-03); ctest numbers shift with every added case, so the reds are named by test: the two settling cases (green since planner plan Task 9, below), the reduced direct lunar case "Lunar return campaign (direct): seed 2" (green in the Task 12 run), the two flap-trim cases (one since Task 3b, above); `plan_speed` is green since the re-measure of 2026-11-03. The timeline sampling case (formerly #711) was fixed by 9fbb94a and is green; #1147 is nightly.
- Settling cases (red since 47904b6, ship hull CP table; green since planner plan Task 9, 2026-11-03): the fixture drifted free for 4 h at the 120 km insertion perigee (~101 km per pass with J2 against the two-body placement), where the tumbling hull fell at ~14,000 s; mission tankers raise that perigee at the first apoapsis and drift free (burn_drifting, no RCS) only in the 170 x 190 km phasing orbit, so the two cases fly there (from the mission configs), no attitude hold added, no limit changed; (the timeline sampling case, formerly #711, was fixed by 9fbb94a and is green); `plan_speed` (green since 2026-11-03: reference plans re-measured, see `docs/subsystems/lunar.md`).
- Reds since the Raptor 3 nozzle 1.30 m (2026-11-03, vacuum 2.912 MN / 350 s -> 2.880 MN / 346.2 s, nothing tuned
  back): all green in the planner plan Task 12 run (2026-11-04): flight 14 ascent, the ship return campaigns (phase
  0-135 / 180-315, dispersions 1-3 / 4-6; dispersed case 3 re-planned to the next pass, the campaign now follows the flown
  plan), the lunar seed-18 gate, the ship catch campaign, the direct lunar seeds 2 / 12 / 15 / 18, "Lunar return
  aerocapture: captured - perigee raised - caught by b4", the booster abort, the docked tanker return; refuelling
  reference flights re-measured (7 tankers / 25.25 d, `refuelling.md`). The entries as they stood:
  `docs/history/guidance-missions.md` "Known issues closed by the Task 12 re-measure".
- Ship dry mass 255 -> 177 t, dry CG 23.516 -> 22.03 m (planner plan Task 3b, 2026-11-03; with full headers 212 t at
  24.99 m instead of 290 t at 25.50 m; `vehicle-data.md`). Single cases and `-Fast` of the touched tags, before ->
  after (`.superpowers/sdd/2026-11-03-planner-vehicle-parameters/task-3b-report.md`):
  - Turned green: flight 14 ascent (above); "Flaps pinned at their stops come back into the range". The trim-margin case
    stays red but narrower: alpha 60 trims at forward 54.3 deg (was 77.2), alpha 70 trims (was none) at forward 79.7
    deg against the 75 deg limit.
  - Green since planner plan Task 4 (2026-11-03, `.superpowers/sdd/2026-11-03-planner-vehicle-parameters/task-4-report.md`):
    the yaw-demand fixture (its 3e5 N m lay beyond the 212 t ship's flap yaw reach of ~1.8-2.6e5; restated to 1.5e5
    within reach, 3e5 kept as beyond); the gate over Starbase (miss 189.8 -> 38.6 m: the nose turn's angular acceleration
    is the RCS's, 0.126 deg/s^2, not 0.5); the flip-and-catch gate prediction (1.34 -> 0.62 s: the entry aero table's
    flaps now trim where the allocation settles, drag area deficit 0.4-0.7 % -> 0.03-0.24 %). Before:
  - Green since Task 4 fix round 1: "A missed catch ejects the crew" - its fixed 27 t was the 290 t ship's first
    missed load; the 177 t ship catches down to 18 t (hidden "Ship catch: lowest catchable gate load": 17 t missed).
    The load now comes from the config (ideal-stop rocket-equation bound, 9.1 t). Fix round 2: the fixed
    `min_catch_propellant_kg` = 30 t is replaced by the ship's derived catch floor (`gnc::ship_catch_propellant_floor_kg`,
    13.9 t usable for Block 3), so catchable crewed ships are no longer given up; a catch that passes the floor and fails
    after the gate has its case again. Open: the seats' "Fang verfehlt" path (CG past the catch point without the abort
    flag) is not reached by any load (`docs/subsystems/crew-escape.md`).
  - Red then, entry authority (for plan Task 4): "A yaw demand does not drive a flap into the range where it has no
    authority" (Mach ~0.6, alpha 80.5, 35 t: trim aft 63.6 / forward 43.5 deg, was 54 / 44; a 3e5 N m yaw leaves an RCS
    fraction 1.6 and the aft pair moving 0.5 deg per call; the forward flap stops at the authority floor at 36 deg).
    "Session flies an entering ship to the gate over Starbase": gate miss 0.6 -> 189.8 m (limit 150): the nose turn
    at the basis freeze (Mach ~0.4) departs (axis error 7.6 deg, roll error -19 deg, RCS 13-22 x saturated, flaps on
    the 90 deg stops) and the ship drifts 180 m east, which the near-vertical fall cannot take back. The same start
    flown to the arms ("from Mach 2.6") is caught (0.96 m/s, 9.5 t, was 10.6 t).
  - Green in the Task 12 run (2026-11-04, `[timeline]` 29 of 29), before red, predictions: "A ship's flip and catch records one landing burn". Found after Task 3b (fix round 1): (1) the
    case placed the ship at 65 deg with the flaps at mid travel, the 290 t ship's state; the 212 t ship's steering
    commands 56.3 deg there (its first prediction at 65 deg fell 4.3 km short) and trims with the forward flaps ~20 deg
    further out: a ~10 s pitch transient the predictions do not fly. The case now starts at the commanded alpha with
    the trimmed flaps (derived per run). (2) The predicted catch was the plan's end, the pins 1 m above the arm tops at
    1 m/s: the touch comes clearance / sink = 1.0 s later (`ShipLandingGuidance::catch_epoch`). (3) The first landing
    plan after the flip ignored that the flip engines run (3 x 0.8 throttle, 0.5 / s down): it asked 8.7 m/s^2, the
    ship flew ~19 and braked ~9 m/s of sink off unplanned; the first plan now starts from the running thrust like a
    re-plan (`ScvxLandingGuidance::request`). Catch error 4.41 -> 3.11 s (limit 4.0, green). Still red: the gate
    -1.34 s at 207 s out (limit 1.0; was -2.15), -0.78 s after ~15 s: the online drag scale learns a 0.7 % drag
    deficit of the entry aero table below Mach 2.2 over that time (the predicted gate moves +0.57 s with it); the
    rest (~-0.8 s) is the pre-existing bias (-0.83 s at 290 t over a 179 s horizon, now 208 s). Entry aero table
    fidelity, for plan Task 4. Red again 2026-11-06 (catch 4.30 s, the last 10 s 2.13 s): the tracking held the ship
    on the chord between the plan's nodes (~1.9 s apart in the pair phase, ~3 m above the braking path), so it flew
    higher and slower than planned and every re-plan ended later; the reference now follows the plan's own path
    (`gnc::plan_path`, docs/subsystems/gnc-core.md). Catch 2.74 s at most, 1.00 s within the last 10 s.
    (The "Open, measured, all landing vehicles" part of this bullet stays in docs/known-issues.md as "Landing planner open points".)
  - Fixed on the way: the booster's landing-ignition forecast just past the apex (-54.7 s, a sink near zero; now the mean
    sink of the fall ahead is its floor: +8.6 s); `frames::fixed_to_geodetic` NaN within ~0.1 m of the axis (the
    yaw-demand fixture starts over the pole). Mass-relative fixtures restated: no usable corridor at 10.9 km/s (0.13
    deg wide now), aerocapture window speed 9,992 m/s (below the corridor's 10,273), lunar return tanks 1/16, aircatch
    propellant 186.5 t.
  - Refuelling measured numbers: green since the Task 12 re-measure (2026-11-04; tanker 171.15 t with stretched tanks,
    depot 158.5 t, `tanker_flight` insertion 497.7 s; numbers in `refuelling.md`).
- Duplicate of the Crew Dragon entry (merged): The Dragon sensitivity above stays parked.
- Part of "Lunar direct campaign" in the red list, superseded by the Task 12 nightly (seed 2 caught, own `[slow]` case green): Before, with the arms as keep-out and the gradual gate law (Tasks 6g, 6h): 20 of 21 (limit 21; since the test tiers 2026-11-03 the full 21 seeds are `[nightly]`, the per-push case "Lunar return campaign (direct): seed 2" stays red; seeds 12, 15, 18 are own green cases). Seed 2 upright 4.9 t, gate 838.9 m at 101.9 m/s, 127 m east of the corridor, why: not determined; it was lost 19.7 t with the keep-out alone, and its safe end is not understood. Seed 15 caught 4.2 t (keep-out alone: lost 20.6 t), seed 12 caught 11.5 t, no ship lost, no strike, the others caught 7.2-12.2 t. Peak over all 21 seeds is 1171 kW/m^2 (seed 21, limit 1200), not the 1096 first reported.
- `Docked tanker: the ship return flies to the catch`: green again after the Raptor 3 nozzle change (2026-11-03, Task 1 re-measure; red before it). The cause of the earlier loss (departure in the terminal approach, gate at 3,769 m with 217 m/s of sink) was not determined; it may return with other vehicle parameters. `tanker_flight` from the pad is caught, on the header tanks (main 0.0 t).
  (Green in the Task 12 run, tanker main at catch re-measured 1.82 t in 2a9e66d9.)
- Ship case 20: upright on 0.3 t, below the 1 t reserve (the 0.0 t was the braking feedforward; fixed in ca4d506).
- Red since the ship's entry value set (vehicle physics Task 5b, 2026-11-01, user's OK on the set; no limit changed; re-measured Task 6d):
  - `[ship][entry][validation]` flight 14 is no longer red (since 2026-11-03 a nightly report row, outside; see #1147): speed rms 8.33 % (report limit 5, unchanged; flights 13, 12: 4.05, 1.51 %), flying fast. The mesh's hull centre of pressure takes the aft flaps from the stop (88-90 deg) to 30-37 deg, flap CdA from 55-65 to 13 m^2, total -10 % at Mach 8-25. Nose and `ship_ascent.csv` move this by at most 0.3 points. An experiment with 11 t instead of 35 t of header propellant reaches 3.15 % there, but low rms 112.9 % and subsonic ratio 3.31 (departs, alpha 168 deg at Mach 1.46): not adopted, the header load is not established by it. The webcast flights share a CdA shortfall of the mesh hull (`aero-derivation.md`).
  - Trim margin, "The ship trims belly first with control margin" and "Flaps pinned at their stops" (`[entry][flap][trim]`): at 65 km, 35 t, alpha 60 the forward pair trims at 77.2 deg (limit 75); alpha 70 does not trim (forward at the 90 stop, aft 19.9, remainder 3.2 x, limit 0.01). The schedule flies 52-55 deg there, where the margin holds (forward 53-62 deg). The limit is not changed.
  - The flap allocation test at alpha 60 keeps its 1e5 N m: 3e5 was tried with the authority floor and still leaves RCS, the forward pair at 77 deg.
- (Superseded by Task 3b's derivation, 177 t +-20.) Dry mass 255 t is probably too high: the one-engine phase of flight 12 allows m <= 209-218 t at 280 tf (a weak source); 255 t would need >= 117 % of the engine rating (328 tf). Judgement: about 175 +-25 t (not a statistic). The propellant gauge excludes the landing propellant, so the rocket-equation 255 t contains it.
- Clause of the atmosphere / terrain entry, stale since the clouds exist (M8): no clouds yet;
- Clause of the C++ sources entry, stale since the i18n plan (2026-11-06, the core returns report codes, texts in `config/i18n`): UI strings in the core are German.
- Pad B physics alignment (user 2026-10-31): the final arm geometry (one hinge axis, arm 24.0 m, standby 7.1 deg, closing
  0.71 s) was re-flown in the vehicle plan's Task 6: booster catch campaign 21 of 21, ship catch campaign 20 of 21
  (`docs/subsystems/ship-entry-return.md`, `ascent-staging.md`). The lunar campaigns flew it in Task 6e: aerocapture 21 of 21; direct above.
- Plan Task 6 step 2 (`[slow]` catch cases) did not run by user decision; it moves to the vehicle plan's campaign gate.
  (Moved: the campaigns were re-flown later, see the previous entry.)
- (Quadtree hitch and Moon frame times: see the Task 13 section.)  [pointer, under "Moon procedural surface plan, Task 11 fix round B (2026-11-05)"]
- Section "Moon procedural surface plan, Task 12 (2026-11-05)": Near-field shadows' GPU cost and the boulder-field camera jump: superseded by the Task 13 section.
- Seed 14 sentence of the lunar campaign rocking entry (green since the preferred 5 deg touchdown slope, 2026-11-07): Seed 14 (1.52 m on 8.0 deg) is green
  since the preferred 5 deg touchdown slope (2026-11-07);
- First sentence of the `cloud_far_check.gd` entry (2026-10-27), replaced 2026-11-07 (`dbe01c3b`): the Moon-view case needs `data/weather/cache/era5_1089330.wthr`, which the cache
  eviction removed; it prints a SKIP line (environmental; refetch it or pick a cached epoch).
