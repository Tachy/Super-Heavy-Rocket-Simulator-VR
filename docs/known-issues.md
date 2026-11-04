# Known issues

Moved verbatim from `CLAUDE.md` on 2026-10-25. Per-subsystem open points also live in `docs/subsystems/<slug>.md`.

- #1147 (`Ship entry reproduces the webcast telemetry of flights 12-14`, flight 14 speed rms 8.33 % against 5 %): the webcast comparison is a report since 2026-11-03 (user: "report, not gate; physical gate kept"), `[nightly][webcast]` "Ship entry: agreement with the webcast telemetry of flights 12-14 (report)" writes `webcast_report/ship_entry_telemetry.md` (parameters: entry mass (177 t dry + 35 t headers), CG, crossflow Cd, flap CdA). The gate part (no impact, lift fraction saturated < 10 %, > 200 speed points) is green (measured 2026-11-03, 16 s) and red or green by physics only; the report row flight 14 speed rms was 0.083 (outside 0.05) at 255 t dry; since the 177 t of Task 3b it is 0.046 (inside), flights 12 / 13 0.061 / 0.091 (outside) and the Mach 3-1 rows of flights 13 / 14 0.28 / 0.25 (outside 0.1, slow); open vehicle-plan item, nothing tuned. The same applies to the other webcast bands (ascent flight 14 / CRS-35, descent rms, booster return timeline, ASDS and RTLS timelines, staging flip time, Crew Dragon chutes): reports, the physical gates stay in the normal tier. Dropped pins: the 77.5 deg bank of `test_entry_skip.cpp` (gate, miss and heat stay), the docking `KNOWN STATE` (report row).
- Planner plan Task 12 re-measure (2026-11-04, `.superpowers/sdd/2026-11-03-planner-vehicle-parameters/task-12-report.md`):
  the plan's touched tags with `[slow]` (1143 cases, 1502 s) left 10 red; after the re-measures and test-premise fixes
  two stayed red, both entry trim with the 177 t ship (below): "The ship trims belly first with control margin on its
  flaps" (still red) and "Entry guidance brings the ship to the Starbase flip gate from dispersed entries" (cases 8, 9,
  11; green since Task 12b, the entry residue vent). Green
  in that run: the settling cases, "Lunar return campaign (direct): seed 2", "Docked tanker: the ship return flies to
  the catch", `plan_speed`, "Flaps pinned at their stops", every ship return / catch / lunar / Falcon 9 / Dragon campaign.
  Nightly (30 cases, 1931 s): 29 green incl. the full flown refuelling campaign against the calculator; red "Lunar
  return campaign (direct): seeds 10-12" (seed 11, below). The other 149 default-tier cases of the plan's tags green.
- Planner plan close-out (2026-11-04, final review + re-review `final-review.md` / `final-rereview.md` in the plan's
  workspace), open minors: `vent_pass_attempts` = 3 is a search-loop cap, not a [D] value; the vent-fit check counts
  the wait from the snapshot without margin, so a marginal fit can enter up to ~3 t above the residue limit; the
  later-pass and refusal paths of the vent-fit search are untested; the first key R reads the alpha schedule file once
  on the sim thread; tanker reserve worker 41.5-50.4 s against its 60 s budget; tanker phasing floor 5.0 t against
  5.04 t flown (Task 8 N1) and the rendezvous floor without boil-off (N2); Task 4 N1 (yaw spill into saturated roll),
  N2 (arm-top source session vs test), the "Fang verfehlt" path untested; no drone-ship tilt-rate test; Falcon 9 flip
  case 4 cause pinned to a commit, not a mechanism; HUD "Manöver ... Brenndauer 0.0 s" while the burn is 55 min ahead
  (pre-existing, seen in `screenshots/entry_vent/hud_planned.png`). Refuelling offsets re-calibrated 2026-11-04
  (7b8a42e): flown campaign 7 tankers / 32.1 d, calculator 7 / 25.25 d, the tool's
  comparison passed (`refuelling-recal-report.md`).
- Entry trim with main-tank residue: closed by planner plan Task 12b (controller ruling 2026-11-04, a planner item):
  the return planner vents the main-tank residue the flaps cannot trim before the entry interface
  (`gnc::entry_residue_limit`, 5.85 t main on top of the full headers for the 177 t ship; `ship-entry-return.md`).
  Entry guidance campaign 12 of 12 (cases 4 / 7 / 8 / 9 / 11 vent 5.0 / 3.2 / 20.7 / 13.6 / 15.5 t; gate <= 0.08 m,
  belly <= 0.13 deg, sink <= 85 m/s). The measurement before: `docs/history/guidance-missions.md`.
- Hull drift with the burning CG (`flight_body.cpp`, Task 4b review minor, measured Task 12b with the hidden
  "Diagnostic: Super Heavy CG station over its propellant"): the state position stays where it is when consumption moves
  the CG along the hull, so the hull shifts against the trajectory (review estimate ~2.8 m over the ascent burn; bounded
  by the CG station's change, not measured in flight). Super Heavy CG station 30.15 m full, 18.30 m at the
  separation's 293 t, 21.78 m at the landing burn's 53.5 t, 23.03 / 23.49 m at 20 / 10 t: over the landing burn at
  most 1.3-1.7 m along the (then vertical) axis - the catch pins' height, not the lateral catch geometry; the catch ends 0.0 m off the axis, max pin load 1.30 MN, every catch / pad campaign green. Open (no red).
- Trim margin, "The ship trims belly first with control margin on its flaps": alpha 70 deg at Mach 21 / 65 km (15 deg
  above the schedule's 55) trims with the forward pair at 79.7 deg against the 75 deg margin limit (alpha 55 / 60:
  46.7 / 54.3 deg). An attitude the schedule does not fly there; the limit is not changed (open, vehicle data).
- Default-tier run after the test tiers (2026-11-03, `.superpowers/sdd/2026-11-03-test-tiers/full_after.log`): 7 of 1513 red at that run (the "Docked tanker" case is green since 2026-11-03); ctest numbers shift with every added case, so the reds are named by test: the two settling cases (green since planner plan Task 9, below), the reduced direct lunar case "Lunar return campaign (direct): seed 2" (green in the Task 12 run), the two flap-trim cases (one since Task 3b, above); `plan_speed` is green since the re-measure of 2026-11-03. The timeline sampling case (formerly #711) was fixed by 9fbb94a and is green; #1147 is nightly.
- Settling cases (red since 47904b6, ship hull CP table; green since planner plan Task 9, 2026-11-03): the fixture drifted free for 4 h at the 120 km insertion perigee (~101 km per pass with J2 against the two-body placement), where the tumbling hull fell at ~14,000 s; mission tankers raise that perigee at the first apoapsis and drift free (burn_drifting, no RCS) only in the 170 x 190 km phasing orbit, so the two cases fly there (from the mission configs), no attitude hold added, no limit changed; (the timeline sampling case, formerly #711, was fixed by 9fbb94a and is green); `plan_speed` (green since 2026-11-03: reference plans re-measured, see `docs/subsystems/lunar.md`).
- Real-time play (planner plan Task 11b, fix round 1): every planner that held the sim thread for >= 0.4 s runs on a
  worker (ship-entry-return.md "Planner work off the sim thread"). Left: the entry guidance's first cycle after a
  placement (~55 ms), the refuelling campaign's live re-plans (`plan_refuelling_campaign`, unmeasured); no
  cancellation - a new return request or the session's end (`FlightSim::setup` on the Godot main thread) waits for a
  running job (up to ~38 s for the tanker reserve); the hand-over holds cost propellant (b4 aircatch 9.1 t left
  against 11.6 t synchronous); the gate aim's very first refresh in real-time play uses the analytic flip model for
  one cycle (M2).
  Fix round 2: a background job lives with its owner (the far re-plan with the Burn state, the tanker reserve with the
  Docked leg); when the owner ends (reset, disengage, undock, mission end) it is orphaned: no coast clamp, its result is
  dropped when it is done (its failure too), and the owner's next job waits for it (a new ISS sequence is a new owner;
  a far re-plan due while an orphan runs starts on a tick after it is reaped, the sim thread never joins it). A booster already at its stop height when it is first
  seen starts its landing plan synchronously (~0.7 s sim-thread step) instead of one hold late. The refuelling first
  return's side flight on the worker interpolates the worker's interpolated ephemeris again (sub-mm): not bit-identical
  to the synchronous side flight (identical with real-time planning off).
  Late landing re-plans (fix round 3): a re-plan one interval past its epoch of effect is waited for on the sim thread
  (gnc-core.md); the hold is the solve's wall time less latency + interval - idle <= ~0.2 s, 6-12 s per late re-plan
  under the full parallel test load (in VR the sim stalls while rendering interpolates); `STARSIM_DEBUG_REALTIME`
  prints each ("late landing re-plan: waited ... ms") and a dropped orphan job's failure ("dropped background job
  failed"). Accepted limit: a late flip prediction has no age bound (its lead is recomputed from the live state each
  call; staleness = refresh period + worker time).
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
    fidelity, for plan Task 4.
  - Fixed on the way: the booster's landing-ignition forecast just past the apex (-54.7 s, a sink near zero; now the mean
    sink of the fall ahead is its floor: +8.6 s); `frames::fixed_to_geodetic` NaN within ~0.1 m of the axis (the
    yaw-demand fixture starts over the pole). Mass-relative fixtures restated: no usable corridor at 10.9 km/s (0.13
    deg wide now), aerocapture window speed 9,992 m/s (below the corridor's 10,273), lunar return tanks 1/16, aircatch
    propellant 186.5 t.
  - Refuelling measured numbers: green since the Task 12 re-measure (2026-11-04; tanker 171.15 t with stretched tanks,
    depot 158.5 t, `tanker_flight` insertion 497.7 s; numbers in `refuelling.md`).
- Crew Dragon entry: 1 cm of stage length moves a campaign case by up to 0.88 km (bound 2 km holds), source not determined (`.superpowers/sdd/2026-11-02-falcon-dragon-iss-models/task-5-report.md`); held until the rendering is done, by the user's wish.
- Falcon 9 / Crew Dragon / ISS detail models (plan 2026-11-02, end state; values in `.superpowers/sdd/2026-11-02-falcon-dragon-iss-models/final-values.md`), open for the user:
  - Port clocking: the `up` of the IDA-2 / IDA-3 ports is a photo-picked member [E, +-30 deg] of a measured class [D] (model D); no document states the real clock; IDA-3 aft vs forward rests on model D only. One config value, one tag run (`iss.md` "Port clocking").
  - Dragon port at 4.22 m [E] +-0.1 (was 4.0); capsule 4.85 m and trunk 3.25 m by measurement, over the cited 4.5 m capsule (`crew-dragon.md`).
  - Falcon 9 foot at -2.1 m from weak evidence; fins fold by a view rule (no physics state), leg drawn as one beam (`falcon9.md`).
  - ISS collision shapes are the old capsules and boxes and lag the drawing; CG 38 m is an unverified [E] (a rough tally gives about 33 m) (`iss.md`).
  - IDSS soft capture misses one envelope corner: 0.05 m/s axial, 0.10 m offset, +0.04 m/s lateral away from the axis ends "abgeprallt" at 30 and 45 deg petal lean (inward lateral captures); the undamped ring platform [E] reflects the chaser. Not tuned; the nightly report row "IDSS envelope corner" (captured 0 vs 1, outside) shows it, no test pins it. Capture at 0.05 m/s also narrowed with the 45 deg lean (largest offset 0.200 -> 0.135 m, margin over the 0.10 m envelope 0.035 m) (`iss.md`).
  - Docked Dragon shows ring retracted / hooks open (`DockingMechanismView` is not driven in flight).
  - The Dragon sensitivity above stays parked.
  - Dragon's flange OD: drawn 1.40 m [E], about 1.64 m in the photo (`vehicle-models.md`); axial fittings against model D unchecked; Dragon's campaign margin 1.61 km against the 2 km bound.
  - Falcon 9 separation: the drawn interstage and MVac bell no longer cross (hold 4 s, the turn over the ground, Task 14); the nozzle clears the interstage by only 0.035 m (`falcon9.md` "Stack", "The flip"). 13.8 m assumed to end at the trunk's base (gimbal lever: 1.2 m if not). Leg tips drawn 1.3 m low against `F_NHQ202210010026`. All for the user.
  - Baked surfaces, shimmer still red at the gate, nothing to change: the user checks it in the headset (user 2026-11-02). `tile_shimmer_check.gd`, 2 m patch, no limit changed: `array_sun` ratio 3.09, error 0.27; `array` shaded ratio 20.1, error 1.13; `skin` shaded ratio 1.07, error 0.78; `skin_sun` ratio 1.335, error 0.050. Green: `f9_interstage` 1.125 / 0.020, `f9_hull` and `radiator` <= 1.08 / <= 0.042; the temporal gate holds on all. Causes measured: the ratio is mostly the patch cap on a small hull (0.6 m patch: `skin_sun` 1.031, `skin` 1.127, `array_sun` still 1.54), the errors are geometry edges (edges masked, one frame: at most 0.068); the shaded skin is a sub-pixel sunlit sliver along ridges and rim. Views to look at: the station's arrays in the sun and shaded, the Dragon's skin in the sun and shaded (`vehicle-models.md` "Shimmer").
  - Falcon 9 stack without Dragon: fairing only as a mass, its hull ends 12.5 m below the real 70 m (`falcon9.md` "Stack"). Second stage stations other than the 9.8 m are [E].
- Red list on the frozen aero state (vehicle physics Task 6e, 2026-11-01; measured on 5167ce4 + the re-measure, findings not fixed):
  - Lunar direct campaign, planner plan Task 12 nightly (2026-11-04): 20 of 21 again, the red moved: seed 2 caught (also its own `[slow]` case), seed 11 aborts upright with 8.2 t (gate -0.05 / -0.62 km, 40.6 t at the entry, 845 m/s braking; why: not determined, no ship lost). Before, with the arms as keep-out and the gradual gate law (Tasks 6g, 6h): 20 of 21 (limit 21; since the test tiers 2026-11-03 the full 21 seeds are `[nightly]`, the per-push case "Lunar return campaign (direct): seed 2" stays red; seeds 12, 15, 18 are own green cases). Seed 2 upright 4.9 t, gate 838.9 m at 101.9 m/s, 127 m east of the corridor, why: not determined; it was lost 19.7 t with the keep-out alone, and its safe end is not understood. Seed 15 caught 4.2 t (keep-out alone: lost 20.6 t), seed 12 caught 11.5 t, no ship lost, no strike, the others caught 7.2-12.2 t. Peak over all 21 seeds is 1171 kW/m^2 (seed 21, limit 1200), not the 1096 first reported.
  - Falcon 9 landing outcome: "landed" only means upright (`landing_outcome.cpp` has no pad rule; the flip campaign counts the pad itself, all 13 separations on it since planner Task 10, case 4's former 121 m in `docs/history/guidance-missions.md`). A stage standing upright off the pad would show "gelandet". From the separation 0-2 of ~20-27 plans of a flight fail (from ignition 0 of 497): the placed-state campaign understates re-plan failures; cause not worked out. A pad rule needs the user's ruling.
  - Falcon 9 flip numbers that miss or are not measured: 14.7 s separation -> boostback misses 13.7 +-0.6 s by 0.4 s beyond the band (CRS-35 own record 13.0 s); 20 deg/s at ignition is not measured (16 +-4 extrapolated; flown 20.2); the drone ship hold of 15 s is a lower bound (camera view ends T+2:48); `flip_duration_s` and `boostback_loss_fraction` are identified on our simulation [D]. Second stage ignition looks like 8-9 s after separation in the frames against our 6 s [D] (`falcon9.md`). Report, not gate (user 2026-11-03: "report, not gate; physical gate kept"): the misses are rows of the `[nightly][webcast]` report `webcast_report/falcon9_rtls_timeline.md`, no longer pinned by `KNOWN MISS` assertions.
  - `Docked tanker: the ship return flies to the catch`: green again after the Raptor 3 nozzle change (2026-11-03, Task 1 re-measure; red before it). The cause of the earlier loss (departure in the terminal approach, gate at 3,769 m with 217 m/s of sink) was not determined; it may return with other vehicle parameters. `tanker_flight` from the pad is caught, on the header tanks (main 0.0 t).
  - `Flown campaign, placed depot`: 4 rows, the test wants 3: tanker 1 refused ("Kein Rendezvous: Phasenwinkel nicht innerhalb ..."), the campaign re-plans and completes. The full flown campaign passes its comparison but loses tankers 5, 9 ("Kein Rendezvous: Bahn tritt vor dem Annäherungsbeginn in das Annäherungsellipsoid ein", was 0 failures); its first tanker return ends upright (allowed by the test). Whether the re-measured file moved the placed case: not determined.
  - Ship abort scan (seeds 0-20, gate 950 and 800 shifted from the catch cases, not flown entry gates; abort CG 250-550 m, 273 runs per gate), struck / lost, before -> with the keep-out (Task 6g): gate 950: 85 / 94 -> 10 / 35; gate 800: 62 / 65 -> 20 / 24. Lost without a strike rose at gate 950 from 9 to 25 (several hit the ground with the headers dry, 0.0 t: going round costs propellant). Before, the tower itself was struck at dispersed gates too (-2.76 m); "never struck" held only for the nominal gate. Still: seed 19 at gate 800 strikes at every height (the abort in the flip, its last plan refused too late); some strikes have arm and tower clear (the mount, which has no hull row). Below 250 m: gate 950 no strike at 150-225 m; gate 800 1-3 of 21 at 150-225 m, so no lower edge there. Nominal gate 950, all heights: no strike; 375, 450 m lost (375 m: 0.0 t), the others upright. `tools/validation/abort_clearance.py` on the hidden `[.abort_clearance]` cases.
  - Aborts on the drawn pad, which the physics does not collide with: booster 400 m: landed 29 m from the centre, foot 12.2, block 9.1, annulus 8.45, tower box 17.4, arm 5.2 m, but 0.84 m into the 0.5 m trench wall. Ship aborts with the keep-out at 200-300 m come down at or over the drawn ramp, 1.1-4.3 m into the trench walls, except the 225 m abort (2.1 m clear, on the ramp over the drawn pit the flat physics ground does not know); 325 m: 0.3 m into the tower foot.
  - Ship case 20: upright on 0.3 t, below the 1 t reserve (the 0.0 t was the braking feedforward; fixed in ca4d506).
  - Low-propellant abort (21-23 t at the nominal gate): lost at all three loads, no test yet; numbers in `docs/subsystems/ship-entry-return.md`.

- Red since the ship's entry value set (vehicle physics Task 5b, 2026-11-01, user's OK on the set; no limit changed; re-measured Task 6d):
  - `[ship][entry][validation]` flight 14 is no longer red (since 2026-11-03 a nightly report row, outside; see #1147): speed rms 8.33 % (report limit 5, unchanged; flights 13, 12: 4.05, 1.51 %), flying fast. The mesh's hull centre of pressure takes the aft flaps from the stop (88-90 deg) to 30-37 deg, flap CdA from 55-65 to 13 m^2, total -10 % at Mach 8-25. Nose and `ship_ascent.csv` move this by at most 0.3 points. An experiment with 11 t instead of 35 t of header propellant reaches 3.15 % there, but low rms 112.9 % and subsonic ratio 3.31 (departs, alpha 168 deg at Mach 1.46): not adopted, the header load is not established by it. The webcast flights share a CdA shortfall of the mesh hull (`aero-derivation.md`).
  - Trim margin, "The ship trims belly first with control margin" and "Flaps pinned at their stops" (`[entry][flap][trim]`): at 65 km, 35 t, alpha 60 the forward pair trims at 77.2 deg (limit 75); alpha 70 does not trim (forward at the 90 stop, aft 19.9, remainder 3.2 x, limit 0.01). The schedule flies 52-55 deg there, where the margin holds (forward 53-62 deg). The limit is not changed.
  - The flap allocation test at alpha 60 keeps its 1e5 N m: 3e5 was tried with the authority floor and still leaves RCS, the forward pair at 77 deg.
  - The 64 t aerocapture is green: apoapsis 426 km, axis error 1.6 deg, peak 924 kW/m^2. At 64 t below Mach 2.5 the aft pair is at its stop (short 42-619 m^3 per q).
- Low-Mach control margin of the belly flop (Task 6d): the attitude step when the terminal basis freezes (gamma -70, Mach ~0.68) demands ~1e6 N m of yaw, above what the flaps deliver at zero roll (+-8.4e5 N m at alpha 80.5, forward flaps fixed about -1.3e5 / +0.7e5). Roll reaches 0.05-0.07 rad/s and makes sideslip; at 13 deg of it the hull's yaw moment nearly exhausts the flaps, at 20 deg it is beyond them. The authority floor of the allocation (`flap_lowest_effective_rad`, a quarter of the best slope) cleared every departure seen, and case 8 under flap table scalings up to +-0.2 %: 0 of 10 depart (without floor 8 of 10). The window is narrow: fraction 0.20 2 of 10, 0.15 and 0.30 9 of 10. 0.25 was set before the first run, from `flap_tiny_column_fraction`, and not moved. The step itself is untouched (no guidance change).

- Ship aero state frozen (2026-11-01, mass / CG / geometry audits and replays): crossflow `ship_crossflow.csv` 1.5-1.6 [D] and the old entry alpha table stay as on main (the dry mass and CG are the derived 177 t / 22.03 m since planner plan Task 3b, entry CG with full headers 24.99 m), because no tested combination of the 2026-11-01 audits passed all replays (flights 12-14 speed rms < 5 %, low-altitude rms < 10 %, saturated < 10 %, subsonic ratio 0.9-1.1; the guidance case 8). Replays, entry mass per flight 221 / 252 / 215 t:

  | Variant | f12 speed / sat % | f13 speed / low / sat % | f14 speed / low / sat % | Case 8 |
  |---|---|---|---|---|
  | main (255 t dry, crossflow 1.5, CG 25.50 m) | 1.51 / 5.5 | 4.06 / 4.7 / 0 | **8.33** / 9.0 / 0 | passes (0.11 m) |
  | measured masses, crossflow 1.5 | **5.14** / 1.0 | 2.66 / **12.5** / 0 | 3.75 / **22.2** / 0 | 0 of 50 green at 175 t dry |
  | measured masses, geometry crossflow 1.07-1.10 | 1.54 / 5.7 | **9.76** / **10.9** / 0 | **7.31** / 8.5 / 0 | passes at CG 25.50 m |
  | geometry + alpha 59.5 + masses + CG 24.85 m | 4.54 / **10.20** | 3.54 / **14.96** / **15.21** | 4.45 / **22.06** / 8.53 | departs, 10 of 10 fail (miss 5.2 km) |
  | main since Task 3b (177 t dry, 212 t entry, CG 24.99 m, crossflow 1.5) | **6.07** / - | **9.08** / **27.9** / - | 4.55 / **24.7** / - | departs with 61.6 t (Task 12; cases 9 / 11 2.0 / 1.3 km); since Task 12b vented to 40.9 t: 0.01 m |

  The last row is the nightly report
  "Ship entry: agreement with the webcast telemetry of flights 12-14 (report)" (speed rms above Mach 3 / rms Mach 3-1;
  subsonic ratio 0.83 / 0.82 on flights 13 / 14, outside 0.9-1.1): the lighter ship flies slow below Mach 3, as the
  "measured masses" row did. Crossflow and alpha table stay as frozen; the low-Mach part stays the next step.
  Facts behind it (audits are estimates, +-20 t unless noted):
  - (Superseded by Task 3b's derivation, 177 t +-20.) Dry mass 255 t is probably too high: the one-engine phase of flight 12 allows m <= 209-218 t at 280 tf (a weak source); 255 t would need >= 117 % of the engine rating (328 tf). Judgement: about 175 +-25 t (not a statistic). The propellant gauge excludes the landing propellant, so the rocket-equation 255 t contains it.
  - Entry masses (gauge still shows main-tank residue): flight 12 221 t, flight 13 252 t, flight 14 203-227 t (only if its payload was released, one weak source), each +-20 t.
  - CG: the trim window over Mach 0.7-25 is 24.49-25.88 m at the stops (15 deg margin 24.73-25.65 m). 25.50 m (entry) and 23.516 m (dry) are tuned values: the dry CG was 28 m, scanned to make the old hull model trim, then inverted to 23.516 m through the header tanks. A bottom-up estimate gives a dry CG of about 22.9 m (range 19.4-23.5 m). The trim margin test is green at an entry CG of 24.85 m at every dry mass tried, red at 25.50 m. 24.85 m costs aft flaps of 66-71 deg at Mach 0.7-1.5; the flight 14 frames fit it, flight 12's do not.
  - A lighter dry mass moves the CG more with the main-tank residue: at 175 t dry and 24.85 m the entry CG at Mach 1.1 is 23.75 m, below the window (24.42 m), and case 8 departs. At the old alpha with the measured masses and CG 25.50 m case 8 passes only with the geometry crossflow.
  - Crossflow gap: the drawn hull derives 1.07-1.10 at Mach 4-25 against the flights' 1.5. The gap in CdA (500-570 m^2 at 290 t, 381-433 m^2 at the measured masses) is a mass artefact; the table is then within +-13 % of the estimate above Mach 6 on flights 13, 14. The geometry set with alpha 59.5 fits the CdA (within 8 % on flights 13, 14) and the speed, but flies too slow at Mach 3-6 (mean -10 to -15 %): flight 13 low 14.96 %, flight 14 low 22.06 %.
  - Allocator window (Task 6d): fraction 0.25 passes case 8; failures out of 10 perturbed runs at fractions 0 / 0.15 / 0.20 / 0.25 / 0.30: 8 / 9 / 2 / 0 / 9. The window vanishes with the geometry crossflow (0 failures at every fraction) and with the table at 175 t dry it is 10 of 10 at every fraction. Cause: the yaw demand at the terminal basis freeze (gamma -70 deg, Mach ~0.68, ~1e6 N m) exceeds the zero-roll flap authority (+-8.4e5 N m); why one tick decides the sign is not determined. Chatter of the commands at that freeze remains.
  - Next step when picked up: the low Mach part (Mach 2-6, where the geometry set flies too slow and the aft flaps rise twice as fast as the frames), not a fit of the table. Open, not closed by the alpha the frames support.
- Forward RCS pod (for Task 6): floats 1.15 m off the drawn hull (radius 4.5 against 3.348 at station 46). Moved alone it misses the catch gate at 148 m (limit 100), so it stays.
- `optim` (SOCP solver, ECOS-style HSDE): dense Nesterov-Todd blocks per cone, so cones should stay small
  (dimension <= ~20); ordering is minimum degree on the explicit elimination graph (not full AMD). Solution
  vectors are accurate to roughly sqrt(tolerance) near cone boundaries; objective and feasibility to the tolerance.
- The very first `--import` of a fresh `game/.godot` segfaults on editor shutdown when the
  extension is present (import itself completes; later runs are fine). Not yet investigated.
- Solar radiation pressure is modelled for vehicles (FlightBody); the Artemis I coast test still uses the
  pure gravity propagator and shows SRP as a t^2 residual (~1.9e-8 m/s^2: 39 m after 18 h).
- Detailed subsystem notes - vehicle data (Block 3, Falcon 9, Crew Dragon, HLS), ascent, booster return and catch,
  SCvx landing, ship entry / flip / catch / return, legs, RCS, capsule entry and parachutes, lunar missions - with
  their decisions, validation numbers and `STARSIM_DEBUG_*` diagnostics live in `docs/subsystems.md`. Read the
  relevant section before changing a subsystem and update it with the change.
- Standing decisions from there: only Block 3 / V3 is modelled (2026-10-13); Super Heavy is always caught, Falcon 9
  lands on legs; physics first - webcast propellant gauges that the engines cannot explain are not followed
  (2026-10-14); Starship throttle schedules are guidance choices, not vehicle data.
- In scaled space the Earth is a 256x128 ellipsoid mesh (facets up to ~0.6 km below the true surface); closer, the
  streamed terrain quadtree (`game/world/terrain/`, M8 Plan B) draws it: with the Earth terrain loaded it follows
  `StarsimCore.surface_samples` (relief, site plateaus, sea at the geoid) plus NAIP / Sentinel-2 imagery, without data/
  the flat rule (the pad height near launch sites). Physics ground (M8 Plan A): with `FlightSession::set_earth_terrain` (the game, `[data]`
  tests) the real Earth terrain - flat plateaus at the derived site heights under every pad / landing zone, 3DEP /
  GLO-30 / ETOPO beyond, water at the EGM2008 geoid; without it (all non-`[data]` tests, the comparison mode) flat at
  the site height (FlightBody::ground_height_m) with the coarse committed geoid as the sea. Site heights in
  `launch_sites.toml` are ellipsoidal [D] (`tools/data_pipeline/site_heights.py`; Starbase -21.11 m). Gentle contact
  (<= 3 m/s, <= 10 deg) rests, harder contact is an impact. Landing legs creep downhill (~0.05 m/s) on slopes >= 2 deg
  and are never judged landed there (off-plateau only; not fixed). Sky, haze and limb come from the precomputed
  atmosphere (M8, `docs/subsystems.md` "Atmosphere rendering"): every Earth surface, the sea's reflected sky, Moon, Sun,
  stars and vehicles are seen through it, with eye adaptation (gain <= 25) and the eclipse sky; no clouds yet; one Sun
  light, evaluated at the followed body. Unverified: NAIP suggests the real OLP-1 mount stands some tens of metres WSW
  of our [E] site coordinates. Details: `docs/subsystems.md` "Earth terrain and geoid".
- Terrain-load draw stall (open, for M7 where the VR frame budget is real): ~1 `terrain_perf.gd` run in 3 shows one
  11-17 ms frame (once 33.7 ms) inside the RenderingServer draw (outside the viewport render) while a batch of terrain
  builds runs on the worker pool. Engine-internal, pool-size dependent: `threading/worker_pool/max_threads = 8`
  removed it (9 of 9 runs) but the Cape then loads in > 20 s instead of ~8 s - a trade-off, not changed. Also open:
  the eclipse epoch's p99 ~8.7 ms (GPU, the per-vertex sky eclipse factor). Details: `docs/subsystems.md`
  "Atmosphere rendering", Performance.
- C++ sources are UTF-8 (/utf-8); UI strings in the core are German.
- The TWCS lever reads 0 until first moved; FlightController only takes it over after a movement.
- Interactive regression check (keys injected, logs clock/altitude/throttle): `res://tests/flight_check.gd`.
- Horizons serves DE441; our DE440 Moon differs by up to 3.3 m (1990-2040). Tolerances reflect this.
- Weather and clouds (M8): the snapshot's layers are scaled to the data's total cloud cover (static clouds Task 9c;
  the 37 pressure levels lose the overlap of the model levels, 0.14-0.20 below tcc before), keeping the grid-mean
  water: the in-cloud water of decks and cirrus is thinner by that factor; the boundary-layer cumulus get a
  sub-adiabatic parcel's water (grid-mean liquid water path 50 -> 111 g/m^2 on the analog hours - still above the ocean
  climatology of order 60-90 g/m^2 quoted from memory for MAC-LWP, so the mean water is deliberately high (open: compare against the MAC-LWP monthly climatology on the same months / ocean regions); f_ad falls
  from 0.50 at the base to 0.20 at 2 km); low stratiform is not too dark by its renderer (body-core tau median 97 on the
  analog set, carried by > 4 km frontal runs; the < 1 km runs median 20, Task 10 - `weather-clouds.md` Validation); the limits (light between columns, shadow darkness under broken cloud, plumes always draw
  over clouds, earthshine at the constant Bond albedo, half-resolution staircase at sharp dark edges, no rain / fog /
  wind) are listed under "Open points and known limits" in `docs/subsystems/weather-clouds.md`.
- `cloud_far_check.gd` (2026-10-27): the Moon-view case needs `data/weather/cache/era5_1089330.wthr`, which the cache
  eviction removed; it prints a SKIP line (environmental; refetch it or pick a cached epoch). Two hand-over fade cases
  fail (listed in `docs/subsystems/weather-clouds.md`, Open points).
- Planned (user 2026-10-27): milestone M11 (after M7 cockpit and M10 sound design) holds the rest of the weather - part
  2 rain / snow / fog / lightning / weather-dependent haze (small variant: climatology AOD plus humidity growth, a few
  precomputed table sets) / city light on clouds; part 3 GOES patterns; part 4 wind / density and launch rules.
- Pad B physics alignment (user 2026-10-31): the final arm geometry (one hinge axis, arm 24.0 m, standby 7.1 deg, closing
  0.71 s) was re-flown in the vehicle plan's Task 6: booster catch campaign 21 of 21, ship catch campaign 20 of 21
  (`docs/subsystems/ship-entry-return.md`, `ascent-staging.md`). The lunar campaigns flew it in Task 6e: aerocapture 21 of 21; direct above.
- `tools/package.ps1` packs `data/models` unchecked: a stale or missing local model would ship in the installer. Open (review
  2026-11-01 F5; fix: stop packaging when the manifest hash does not match `build_models.input_hash`).

- Dorsal raceway vs transfer connectors (plan 2026-11-04, Tasks 3 / 4; USER DECISION OPEN): the Block 3 raceway tube lies on the
  dorsal centreline and the configs put both connectors on y = 0 (stations 8 and 24 m), so the tube would run into the connector
  pedestals. Until the user decides, the DRAWING routes the tube sideways around the pedestals on the tanker and the depot only
  (`[dorsal] raceway_detour_m` / `raceway_detour_ramp_m`, `dorsal_layout.raceway_offset_m`, `gen_hull.py`); physics and Block 3 are
  untouched. Option 2 awaiting the user: move the connectors off the centreline in the configs (`starship_tanker.toml`,
  `starship_depot.toml`, `starship_hls.toml`; the mated pair needs both ships' connectors mirrored), then drop the detour keys.

## Pad B drawing, open after the physics alignment plan (2026-11-01)
- Slits between the cladding plates on the pad-side tower face: the sky shows through in places (`screenshots/pad-b-plan/fix6_leg.png`).
- Tower braces have no gusset plates; the close-up photo shows them clearly (`nsf_padb_chopsticks_2025-02.jpg`).
- Hoist cables: the worker photo reads 2-3 cable widths apart across the block; only 1.2 fit in the 2 m block at 4 per group, so the pitch is 0.19 m and the group spacing 1.25 m (photo reading 1.5 m). From the pad side the two groups still read as two bars.
- Hinge axis direction (outward diagonal) is an assumption; the photos give only the 2 +-1 m. Rollers, third rail corner and second drum are the user's description, not resolved on photos.
- Plan Task 6 step 2 (`[slow]` catch cases) did not run by user decision; it moves to the vehicle plan's campaign gate.
- `tower_handover_check` is red (2026-11-01): 8 failures on fa226c8, 7 on aae1e5d, about 6 % near/mid against its limit. Cause not traced; not known whether it was green before the leg colour change e67af13.
- Tower head underside 140.5 m, 1.4 m above the lattice top where the photo reads level: at 139.1 m the sheave block would enter the head at +0.7 m carriage offset, the stroke allows +4 m, and it clears to +2.1 m as built.
