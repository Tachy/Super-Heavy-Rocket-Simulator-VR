# Known issues

Moved verbatim from `CLAUDE.md` on 2026-10-25. Per-subsystem open points also live in `docs/subsystems/<slug>.md`.

- #1147 (`Ship entry reproduces the webcast telemetry of flights 12-14`, flight 14 speed rms 8.33 % against 5 %): the webcast comparison is a report since 2026-11-03 (user: "report, not gate; physical gate kept"), `[nightly][webcast]` "Ship entry: agreement with the webcast telemetry of flights 12-14 (report)" writes `webcast_report/ship_entry_telemetry.md` (parameters: entry mass (177 t dry + 35 t headers), CG, crossflow Cd, flap CdA). The gate part (no impact, lift fraction saturated < 10 %, > 200 speed points) is green (measured 2026-11-03, 16 s) and red or green by physics only; the report row flight 14 speed rms was 0.083 (outside 0.05) at 255 t dry; since the 177 t of Task 3b it is 0.046 (inside), flights 12 / 13 0.061 / 0.091 (outside) and the Mach 3-1 rows of flights 13 / 14 0.28 / 0.25 (outside 0.1, slow); open vehicle-plan item, nothing tuned. The same applies to the other webcast bands (ascent flight 14 / CRS-35, descent rms, booster return timeline, ASDS and RTLS timelines, staging flip time, Crew Dragon chutes): reports, the physical gates stay in the normal tier. Dropped pins: the 77.5 deg bank of `test_entry_skip.cpp` (gate, miss and heat stay), the docking `KNOWN STATE` (report row).
- Planner plan close-out (2026-11-04, final review + re-review `final-review.md` / `final-rereview.md` in the plan's
  workspace), open minors: `vent_pass_attempts` = 3 is a search-loop cap, not a [D] value; the vent-fit check counts
  the wait from the snapshot without margin, so a marginal fit can enter up to ~3 t above the residue limit; the
  later-pass and refusal paths of the vent-fit search are untested; the first key R reads the alpha schedule file once
  on the sim thread; tanker reserve worker 41.5-50.4 s against its 60 s budget; tanker phasing floor 5.0 t against
  5.04 t flown (Task 8 N1) and the rendezvous floor without boil-off (N2); Task 4 N1 (yaw spill into saturated roll),
  N2 (arm-top source session vs test), the "Fang verfehlt" path untested; no drone-ship tilt-rate test; Falcon 9 flip
  case 4 cause pinned to a commit, not a mechanism; HUD "Manöver ... Brenndauer 0.0 s" while the burn is 55 min ahead
  (pre-existing, seen in `screenshots/entry_vent/hud_planned.png`).
- Hull drift with the burning CG (`flight_body.cpp`, Task 4b review minor, measured Task 12b with the hidden
  "Diagnostic: Super Heavy CG station over its propellant"): the state position stays where it is when consumption moves
  the CG along the hull, so the hull shifts against the trajectory (review estimate ~2.8 m over the ascent burn; bounded
  by the CG station's change, not measured in flight). Super Heavy CG station 30.15 m full, 18.30 m at the
  separation's 293 t, 21.78 m at the landing burn's 53.5 t, 23.03 / 23.49 m at 20 / 10 t: over the landing burn at
  most 1.3-1.7 m along the (then vertical) axis - the catch pins' height, not the lateral catch geometry; the catch ends 0.0 m off the axis, max pin load 1.30 MN, every catch / pad campaign green. Open (no red).
- Real-time play (planner plan Task 11b, fix round 1): every planner that held the sim thread for >= 0.4 s runs on a
  worker (ship-entry-return.md "Planner work off the sim thread"). Left: the entry guidance's first cycle after a
  placement (~55 ms), the refuelling campaign's live re-plans (`plan_refuelling_campaign`, unmeasured); no
  cancellation - a new return request or the session's end (`FlightSim::setup` on the Godot main thread) waits for a
  running job (up to ~38 s for the tanker reserve; likewise the lunar landing site check at mission start, U-7: the
  `std::async` touchdown search, up to ~30 s, held by the runner's deferred poll, which `FlightSim::~FlightSim` / `setup`
  destroy; the held session takes no cancel command meanwhile); the hand-over holds cost propellant (b4 aircatch 9.1 t left
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
- Landing planner open points (measured, all landing vehicles; docs/subsystems/gnc-core.md, "Open points"): the planner drifts +0.9 s by
  re-plan 14 even under perfect tracking (+0.5 s jump at ~5 s to go); the approach sink limit holds at nodes only
  (~3 m/s violated mid-interval on the first ship plan); the dropped engine's run-down thrust is missing from phase 1.
- Crew Dragon entry: 1 cm of stage length moves a campaign case by up to 0.88 km (bound 2 km holds), source not determined (`.superpowers/sdd/2026-11-02-falcon-dragon-iss-models/task-5-report.md`); held until the rendering is done, by the user's wish.
- Falcon 9 / Crew Dragon / ISS detail models (plan 2026-11-02, end state; values in `.superpowers/sdd/2026-11-02-falcon-dragon-iss-models/final-values.md`), open for the user:
  - Port clocking: the `up` of the IDA-2 / IDA-3 ports is a photo-picked member [E, +-30 deg] of a measured class [D] (model D); no document states the real clock; IDA-3 aft vs forward rests on model D only. One config value, one tag run (`iss.md` "Port clocking").
  - Dragon port at 4.22 m [E] +-0.1 (was 4.0); capsule 4.85 m and trunk 3.25 m by measurement, over the cited 4.5 m capsule (`crew-dragon.md`).
  - Falcon 9 foot at -2.1 m from weak evidence; fins fold by a view rule (no physics state), leg drawn as one beam (`falcon9.md`).
  - ISS collision shapes are the old capsules and boxes and lag the drawing; CG 38 m is an unverified [E] (a rough tally gives about 33 m) (`iss.md`).
  - IDSS soft capture misses one envelope corner: 0.05 m/s axial, 0.10 m offset, +0.04 m/s lateral away from the axis ends "abgeprallt" at 30 and 45 deg petal lean (inward lateral captures); the undamped ring platform [E] reflects the chaser. Not tuned; the nightly report row "IDSS envelope corner" (captured 0 vs 1, outside) shows it, no test pins it. Capture at 0.05 m/s also narrowed with the 45 deg lean (largest offset 0.200 -> 0.135 m, margin over the 0.10 m envelope 0.035 m) (`iss.md`).
  - Docked Dragon shows ring retracted / hooks open (`DockingMechanismView` is not driven in flight).
  - Dragon's flange OD: drawn 1.40 m [E], about 1.64 m in the photo (`vehicle-models.md`); axial fittings against model D unchecked; Dragon's campaign margin 1.61 km against the 2 km bound.
  - Falcon 9 separation: the drawn interstage and MVac bell no longer cross (hold 4 s, the turn over the ground, Task 14); the nozzle clears the interstage by only 0.035 m (`falcon9.md` "Stack", "The flip"). 13.8 m assumed to end at the trunk's base (gimbal lever: 1.2 m if not). Leg tips drawn 1.3 m low against `F_NHQ202210010026`. All for the user.
  - Baked surfaces, shimmer still red at the gate, nothing to change: the user checks it in the headset (user 2026-11-02). `tile_shimmer_check.gd`, 2 m patch, no limit changed: `array_sun` ratio 3.09, error 0.27; `array` shaded ratio 20.1, error 1.13; `skin` shaded ratio 1.07, error 0.78; `skin_sun` ratio 1.335, error 0.050. Green: `f9_interstage` 1.125 / 0.020, `f9_hull` and `radiator` <= 1.08 / <= 0.042; the temporal gate holds on all. Causes measured: the ratio is mostly the patch cap on a small hull (0.6 m patch: `skin_sun` 1.031, `skin` 1.127, `array_sun` still 1.54), the errors are geometry edges (edges masked, one frame: at most 0.068); the shaded skin is a sub-pixel sunlit sliver along ridges and rim. Views to look at: the station's arrays in the sun and shaded, the Dragon's skin in the sun and shaded (`vehicle-models.md` "Shimmer").
  - Falcon 9 stack without Dragon: fairing only as a mass, its hull ends 12.5 m below the real 70 m (`falcon9.md` "Stack"). Second stage stations other than the 9.8 m are [E].
- Red list on the frozen aero state (vehicle physics Task 6e, 2026-11-01; measured on 5167ce4 + the re-measure, findings not fixed):
  - Lunar direct campaign, planner plan Task 12 nightly (2026-11-04): 20 of 21 again, the red moved: seed 2 caught (also its own `[slow]` case), seed 11 aborts upright with 8.2 t (gate -0.05 / -0.62 km, 40.6 t at the entry, 845 m/s braking; why: not determined, no ship lost).
  - Falcon 9 landing outcome: "landed" only means upright (`landing_outcome.cpp` has no pad rule; the flip campaign counts the pad itself, all 13 separations on it since planner Task 10, case 4's former 121 m in `docs/history/guidance-missions.md`). A stage standing upright off the pad would show "gelandet". From the separation 0-2 of ~20-27 plans of a flight fail (from ignition 0 of 497): the placed-state campaign understates re-plan failures; cause not worked out. A pad rule needs the user's ruling.
  - Falcon 9 flip numbers that miss or are not measured: 14.7 s separation -> boostback misses 13.7 +-0.6 s by 0.4 s beyond the band (CRS-35 own record 13.0 s); 20 deg/s at ignition is not measured (16 +-4 extrapolated; flown 20.2); the drone ship hold of 15 s is a lower bound (camera view ends T+2:48); `flip_duration_s` and `boostback_loss_fraction` are identified on our simulation [D]. Second stage ignition looks like 8-9 s after separation in the frames against our 6 s [D] (`falcon9.md`). Report, not gate (user 2026-11-03: "report, not gate; physical gate kept"): the misses are rows of the `[nightly][webcast]` report `webcast_report/falcon9_rtls_timeline.md`, no longer pinned by `KNOWN MISS` assertions.
  - `Flown campaign, placed depot`: 4 rows, the test wants 3: tanker 1 refused ("Kein Rendezvous: Phasenwinkel nicht innerhalb ..."), the campaign re-plans and completes. The full flown campaign passes its comparison but loses tankers 5, 9 ("Kein Rendezvous: Bahn tritt vor dem Annäherungsbeginn in das Annäherungsellipsoid ein", was 0 failures); its first tanker return ends upright (allowed by the test). Whether the re-measured file moved the placed case: not determined.
  - Ship abort scan (seeds 0-20, gate 950 and 800 shifted from the catch cases, not flown entry gates; abort CG 250-550 m, 273 runs per gate), struck / lost, before -> with the keep-out (Task 6g): gate 950: 85 / 94 -> 10 / 35; gate 800: 62 / 65 -> 20 / 24. Lost without a strike rose at gate 950 from 9 to 25 (several hit the ground with the headers dry, 0.0 t: going round costs propellant). Before, the tower itself was struck at dispersed gates too (-2.76 m); "never struck" held only for the nominal gate. Still: seed 19 at gate 800 strikes at every height (the abort in the flip, its last plan refused too late); some strikes have arm and tower clear (the mount, which has no hull row). Below 250 m: gate 950 no strike at 150-225 m; gate 800 1-3 of 21 at 150-225 m, so no lower edge there. Nominal gate 950, all heights: no strike; 375, 450 m lost (375 m: 0.0 t), the others upright. `tools/validation/abort_clearance.py` on the hidden `[.abort_clearance]` cases.
  - Aborts on the drawn pad, which the physics does not collide with: booster 400 m: landed 29 m from the centre, foot 12.2, block 9.1, annulus 8.45, tower box 17.4, arm 5.2 m, but 0.84 m into the 0.5 m trench wall. Ship aborts with the keep-out at 200-300 m come down at or over the drawn ramp, 1.1-4.3 m into the trench walls, except the 225 m abort (2.1 m clear, on the ramp over the drawn pit the flat physics ground does not know); 325 m: 0.3 m into the tower foot.
  - Low-propellant abort (21-23 t at the nominal gate): lost at all three loads, no test yet; numbers in `docs/subsystems/ship-entry-return.md`.

- 64 t aerocapture (Task 5b value set), green: apoapsis 426 km, axis error 1.6 deg, peak 924 kW/m^2. At 64 t below Mach 2.5 the aft pair is at its stop (short 42-619 m^3 per q).
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
- Earth ground, open: landing legs creep downhill (~0.05 m/s) on slopes >= 2 deg and are never judged landed
  there (off-plateau only; not fixed). Unverified: NAIP suggests the real OLP-1 mount stands some tens of metres WSW
  of our [E] site coordinates (`docs/subsystems/earth-terrain.md`).
- Terrain-load draw stall (open, for M7 where the VR frame budget is real): ~1 `terrain_perf.gd` run in 3 shows one
  11-17 ms frame (once 33.7 ms) inside the RenderingServer draw (outside the viewport render) while a batch of terrain
  builds runs on the worker pool. Engine-internal, pool-size dependent: `threading/worker_pool/max_threads = 8`
  removed it (9 of 9 runs) but the Cape then loads in > 20 s instead of ~8 s - a trade-off, not changed. Also open:
  the eclipse epoch's p99 ~8.7 ms (GPU, the per-vertex sky eclipse factor). Details: `docs/subsystems.md`
  "Atmosphere rendering", Performance.
- The TWCS lever reads 0 until first moved; FlightController only takes it over after a movement.
- Weather and clouds (M8): the snapshot's layers are scaled to the data's total cloud cover (static clouds Task 9c;
  the 37 pressure levels lose the overlap of the model levels, 0.14-0.20 below tcc before), keeping the grid-mean
  water: the in-cloud water of decks and cirrus is thinner by that factor; the boundary-layer cumulus get a
  sub-adiabatic parcel's water, capped at the run's own path (grid-mean liquid water path 50 -> 76-78 g/m^2, inside the
  ocean climatology range ~60-90 g/m^2 set by the project owner, look fixes T7; open: compare against the MAC-LWP
  monthly climatology on the same months / ocean regions; the capped cumulus look flatter from below); f_ad falls
  from 0.50 at the base to 0.20 at 2 km); low stratiform is not too dark by its renderer (body-core tau median 97 on the
  analog set, carried by > 4 km frontal runs; the < 1 km runs median 20, Task 10 - `weather-clouds.md` Validation); the limits (light between columns, shadow darkness under broken cloud, plumes always draw
  over clouds, earthshine at the constant Bond albedo, half-resolution staircase at sharp dark edges, no rain / fog /
  wind) are listed under "Open points and known limits" in `docs/subsystems/weather-clouds.md`.
- Planned (user 2026-10-27): milestone M11 (after M7 cockpit and M10 sound design) holds the rest of the weather - part
  2 rain / snow / fog / lightning / weather-dependent haze (small variant: climatology AOD plus humidity growth, a few
  precomputed table sets) / city light on clouds; part 3 GOES patterns; part 4 wind / density and launch rules.

- Dorsal raceway vs transfer connectors (plan 2026-11-04, Tasks 3 / 4; USER DECISION OPEN): the Block 3 raceway tube lies on the
  dorsal centreline and the configs put both connectors on y = 0 (stations 8 and 24 m), so the tube would run into the connector
  pedestals. Until the user decides, the DRAWING routes the tube sideways around the pedestals on the tanker, the depot and the HLS only
  (`[dorsal] raceway_detour_m` / `raceway_detour_ramp_m`, `dorsal_layout.raceway_offset_m`, `gen_hull.py`); physics and Block 3 are
  untouched. Option 2 awaiting the user: move the connectors off the centreline in the configs (`starship_tanker.toml`,
  `starship_depot.toml`, `starship_hls.toml`; the mated pair needs both ships' connectors mirrored), then drop the detour keys.
- HLS nose docking port (plan 2026-11-04, Task 5; PLANNED, user 2026-11-04: crew transfer, the crew arrives on a Starship): NASA's docking page and the concept render show an IDSS-class port on the HLS nose, but
  `config/vehicles/starship_hls.toml` has only the dorsal `transfer_dock`. The nose port is drawn VISUAL ONLY (`[nose_port]` of
  `detail/starship_hls.toml`, node `nose_port`, manifest `hls_nose_port.visual_only`): no docking physics, no port frame, nothing in the core. The user wants a real (physics) nose port later,
  for crew transfer from a crew Starship; until then nothing can dock there. The drawn collar stands 0.10 m above the config length (52.1 m).
- HLS stowed leg pads vs core collision (Task 6b minor): the core's collision points do not sample the stowed pad rim (drawn pad 0.776 m from the depot's hull, core 1.817 m); stated, not fitted.
- HLS landing thrusters (plan 2026-11-04, Task 5): drawn exactly as the config's `landing` group resolves them (12, ring 4.6 m, station 40.0 m, cant 15 deg
  toward the axis). The thrust axis then lies 15 deg off the hull's station axis, so the exit plane stands across the hull (aft-facing pods), which the 2021 render's
  "port cutouts" do not show either way; the config is unchanged (user ruling 2026-11-04: no proposal approved).

## Pad B drawing, open after the physics alignment plan (2026-11-01)
- Slits between the cladding plates on the pad-side tower face: the sky shows through in places (`screenshots/pad-b-plan/fix6_leg.png`).
- Tower braces have no gusset plates; the close-up photo shows them clearly (`nsf_padb_chopsticks_2025-02.jpg`).
- Hoist cables: the worker photo reads 2-3 cable widths apart across the block; only 1.2 fit in the 2 m block at 4 per group, so the pitch is 0.19 m and the group spacing 1.25 m (photo reading 1.5 m). From the pad side the two groups still read as two bars.
- Hinge axis direction (outward diagonal) is an assumption; the photos give only the 2 +-1 m. Rollers, third rail corner and second drum are the user's description, not resolved on photos.
- `tower_handover_check` is red (2026-11-01): 8 failures on fa226c8, 7 on aae1e5d, about 6 % near/mid against its limit. Cause not traced; not known whether it was green before the leg colour change e67af13.
- Tower head underside 140.5 m, 1.4 m above the lattice top where the photo reads level: at 139.1 m the sheave block would enter the head at +0.7 m carriage offset, the stroke allows +4 m, and it clears to +2.1 m as built.

## Depot/tanker/HLS models, open after the final review (2026-11-04)
- Probes follow the controlled body's `docking_state` only (`game/vehicle/vehicle_view.gd` ~178): a docked tanker/HLS that is not the controlled body (e.g. after switching to the depot) shows stowed probes. Fix later: a per-body docked flag in the snapshot (read-only bindings field).
- Terrain horizon march near the lunar poles (LDEM_64, 2026-11-06): against a 1 m-step brute force (the 10 m brute was the inaccurate reference: it showed the march HIGHER by up to 0.61 deg on 2 rays) the march is never higher (0 of 1,024 rays beyond 0.1 deg) but lower on 5 of 637 rays with a horizon < 5 deg (all near the IM-2 site, -84 deg, worst 0.82 deg): steep bilinear LDEM_64 walls within the first 10 m of the observer fall between the observer and the march's first sample. The test asserts the never-higher invariant per ray and counts the lower rays (limit 8, worst 0.9 deg); the banded-lattice limits were re-measured on LDEM_64 (`core/tests/test_terrain_horizon.cpp`).
- Procedural Moon surface detail (Moon procedural surface plan Task 2, `docs/subsystems/moon-surface.md`): outside the
  site tiles the generated craters (to 2 km) and roughness (to ~1 km wavelength) are statistical, not the real features
  there (LOLA_16 does not resolve them); band_start uses the global grid's latitude spacing (LDEM_64: 0.47 km; the 1.9 km below is LOLA_16's), so near the poles,
  where its longitude spacing is finer (172 m at -84.8 deg), some 0.2-1.9 km detail is both in LOLA_16 and generated;
  the roughness is ~6 % rougher near the 8 cube corners (skewed face lattice; craters + roughness 4.09 mm there,
  above Guo's 4 mm before rocks); queries inside the +-13.6 km face blend band along the cube edges cost ~3-6 x more.
  Moon terrain horizons (Task 7): baked over LOLA from LOLA observer heights, so the procedural relief casts no terrain
  shadow beyond Task 12's near field and floors / rims read their horizon from the LOLA height (~1 deg over a few km
  at south-pole Sun elevations; `docs/subsystems/moon-surface.md` open points).

## Moon procedural surface plan, Task 11 fix round B (2026-11-05)
- Lunar ground luminance (`docs/subsystems/moon-surface.md`, Task 11 fix B report): the albedo texture (CGI Moon Kit
  colour mosaic, sRGB-decoded) makes the maria ~4.3 x darker than its global mean - about twice the contrast reported
  for mare / highland normal albedo - so mare sites (Apollo 11, 15, 17) render ~2-3 x darker than photometry suggests;
  and the eye adaptation (white plate under the Sun, 41 000 cd/m^2) shows the regolith ~6 stops below the plate (Apollo 11 view: ground 490 cd/m^2).
  Both await a decision (albedo source; scene-luminance adaptation T14).

## Moon procedural surface plan, Task 5 (2026-11-05)
- Refused landing site at mission start: the touchdown search checks every candidate of the 1 km search (IM-2:
  7,449) before it refuses. Parallel ring-wise search (i7-13700K, 22 threads): 15.8 s when every candidate fails at
  its first roll, 29.5 s worst case (every candidate through all rolls and the base check) - over the 20 s target
  (serial: 169 s / 250-290 s). No cap by ruling (a cap could refuse a site that has a safe point). Since follow-up
  U-7 (2026-11-06) it runs on a background thread at the mission start (game responsive, status line "Landeplatz wird
  geprüft"); the wait itself is unchanged. The refuelling continuation inside the flight runs it on a background
  thread too since 2026-11-07 (`lunar_landing_continuation`, real-time planning at warp <= 1). Only IM-2 ships (136 ms in the mission start of that
  run; 59 ms in the IM-2 hazard test on 2026-11-07, a point 28.3 m off). Since the preferred 5 deg slope (2026-11-07)
  the full scan also happens at reachable sites with no admissible point within 5 deg (or only far out); both the
  mission start and the refuelling continuation stay responsive (background). Under time warp (real-time planning
  off) the continuation still decides on the simulation thread, as the other planners do.
- Lunar campaign rocking (follow-up U-8, 2026-11-06, `docs/subsystems/moon-surface.md` open points): not foot slip
  but rigid-body rocking after the engines-off first contact on one uphill foot - the hull pivots through the slope,
  overshoots onto a single downhill foot and swings round that sticking foot. Seed 19 chooses a 4.8 deg point and settles 1.04 m off
  (gate 1 m, not loosened): the 15-21 case of `[lunar_campaign][slow]` is red - see "Flat touchdown point plan" below.

## Moon procedural surface plan, Task 16: Sun shadows on the Earth (2026-11-05)
- The Earth's near-field shadow map covers 500 m-4 km of view depth around the eye (terrain-rendering.md "Near-field Sun
  shadows (Earth)"); beyond it (and above 30 km, at night) nothing casts. A vehicle's own shadows on itself fade out
  while the eye climbs from 20 to 30 km over the ground (a smoothstep of the shadow opacity; the Moon keeps a step at
  20 km).
- The Earth terrain casts no shadow (dunes, levees): only structures and vehicles do; the relief keeps its normal shading.
- Cost in the stereo mock at the pad: vr-performance.md (Task 13 re-measure); not measured in the headset. With the
  analog weather on the pad view takes 17.2 ms (14.4 ms without the shadows, Task 13b): the clouds' cost, over the
  11.1 ms frame regardless of the shadows.
- Ground bounce in the radiance cube: the imagery's sector x ring profile (`GroundAlbedoNear`, 8 azimuth sectors x 5 rings
  to 1.5 km, read on a worker) replaces the global day texture's 6.7 km texel near the eye, within 2.2 % mean / 4.3 % max of
  the imagery's exact view-factor albedo per face azimuth at eye heights of 10-148 m (launch-structures.md); a 2 m eye at the
  launch mount is up to 20 % off (the 0-25 m ring averages the mount's top and shadow in the imagery by area). One profile at
  the eye: objects far from the eye get the eye's ground. Only where the two imagery areas (Starbase, Cape) cover the eye
  and below 6 km. Elsewhere on Earth the texel stays (the bounce at a site without imagery is the global mean of land, sea
  and snow).
- Orbit renders hang in `render_check` (follow-ups batch B, 2026-11-06): ground views at 128 m and 5 km render, but views
  from 400 km and 1500 km produced no frames for more than 4 min (CPU idle) and were killed; not investigated. The rim of the
  U-3 sea calibration's 100 km fade has therefore not been looked at from orbit. Open.
- Earth near shadows: the penumbra grain does not crawl at HIGH in a desktop head-motion test (0.02 deg per frame); not
  checked in the headset. The Falcon 9 ship's shadow on the sea is not
  examined (Task 13b).
- The tower's far tier (alpha-to-coverage box beyond 5 km) casts nothing; at that distance the tower's ground shadow lies
  beyond the shadow range anyway.

## Moon procedural surface plan, Task 13 (2026-11-06)
- Moon frame times (stereo mock, vr-performance.md "Task 13 frame times"): IM-2 with near shadows 17.5 / 16.9 / 11.7 ms
  at 2 m / 100 m / 2 km (within the 22.2 ms floor of ruling T11-U3); the Diviner boulder field 44.9 / 26.3 / 13.9 ms -
  on its ground over the floor already without shadows (38.2 ms; the game's floor guard then raises the rock threshold).
  The headset's larger target draws more (the mock is a lower bound). Not measured in the headset.
- Moon quadtree main thread after a 10 km camera jump at 2 m: the selection phase takes up to 21 ms (IM-2) / 52 ms
  (boulder field) in one frame, p99 14.4 / 16.6 ms - over the 5 ms hitch gate the rock layer meets (its share max
  4.8 ms since Task 13's pool warming). A time-sliced selection would be the fix; not built (terrain-rendering.md).
  Headset frame pacing while the view loads, the leaf `band_heights` retention (~70 MB) and the rock pool's real VRAM
  (the 512 MB bound is computed from instance counts) are not measured.
- Boulder fields at the 512 MB pool bound keep near nodes statistical (radiance-continuous; 353 nodes in the jump test):
  the hand-over eviction finds no far node with the near LOD buckets there.
- Boulder-field camera jump (moon_surface_check, 10 km at 2 m): the rock layer's main-thread share fails its 5 ms gate
  intermittently with one 21-26 ms pool allocation (the MultiMesh take) - final-review fix pass 2026-11-06: 4 of 5 runs
  with the LOD-0 shadow mesh, 2 of 4 with the former LOD-1 one (passing runs 3.7-5.0 ms). Task 13's single pass (4.75 ms)
  was a favourable run; the prewarm does not cover the bucket that allocates. Not fixed; the IM-2 jump passes.
- Summed roughness at L20 / L19 -7.6 / -6.7 % against the 10 % gate: a systematic deficit of the texture weights' mesh
  transfer, not one crater tile's realisation (moon-surface.md Validation); more crater tiles would lower it. The
  shipped crater realisation is favourable, so the expected L20 margin is smaller than the 2.4 % shown and may reach the
  gate; the real fix is the texture-weight transfer (open point), not more tiles.
- Fill light at a grazing Sun: faint steps at the horizon maps' node borders (0.07 deg Sun; Task 12 fix 1) - not fixed.
- Rock statistics against exact rasters of the drawn meshes (8 deg Sun, eye 60 deg): the hull excess X 0.025 against
  0.045, the rock flux per cover +3.8 %; the GPU rock patch at abundance 10 reads +1.52 % (gate 2 %), abundance 1
  -1.3..-1.5 %.
- Surface static: no plume interaction (craters, dust, rocks do not react to the engines; the spec's limit).

## Lunar sites plan, Task 7 fix round 1 (2026-11-06)

- NRHO variant: the start budget charges the site-dependent plane change (`nrho_plane_change`, ruling R12,
  `docs/subsystems/lunar.md`), but the flight still flies the polar transfer from the NRHO and waits for the pass;
  flying the budgeted plane change (departure burn or LLO) is open. No shipped mission is affected (the NRHO refuses
  at 120 t).
- NRHO `[budget] nrho_to_llo_delta_v_mps` is 1245 m/s [D] (user 2026-11-07; what `nrho_to_llo` plans from the 2026-11-13
  mission's first perilune after the insertion: 969 + 276, the polar plane through a perilune at 73.1 N is turned
  35.5 deg from the NRHO plane; the l3 case had 660 + 216; it was 956 before). The NRHO stays infeasible at 120 t
  (29.3 t short at Mons Mouton).

## Lunar sites plan, Task 8 fix round 1 (2026-11-07)

- The stay's plane change (leg `stay_plane_change`, ruling R15: the Moon turns under the landing plane over the
  stay) is budgeted in the start, the window reserve and the refuelling demand, but not flown: the ascent ends in the
  plane through the liftoff point and no turn back onto the landing plane is commanded. Mons Mouton 24 h: 33.9 m/s.

## Lunar sites plan, Task 11 (2026-11-07)

- `World.switch_lunar_site` blocks the main thread for ~5.0 s (`TerrainQuadtree.setup` ~4.3 s, rock layer ~0.4 s, detail configure ~0.25 s; game start ~7.7 s for the same setup). Accepted while the switch happens in the mission wizard before a flight (ruling R20); moving the quadtree setup off the main thread is open. Measured value: `World.last_site_rebuild_ms` (printed by `render_check`).

## Lunar sites plan, Task 13 (2026-11-07)

- Render checks: an eye placed by `--eye_height` over the site centre can stand next to a drawn boulder (Apollo 15
  LM-window view); the AS11/15/17 comparisons are at the photo azimuths only approximately (the camera stations are
  not reproduced).

## Flat touchdown point plan (2026-11-07)

- The touchdown search prefers points within 5 deg (user decision 2026-11-07); the 6-8 deg landings of the lunar sites plan Task 13 (Apollo 11 nightly 4 / 7, 1.69-2.38 m; seed 14 1.51 m) are gone. Two landings just under 5 deg still rock (U-8) just over the 1 m gate: dispersed seed 19 1.04 m on 4.8 deg (`[lunar_campaign][slow]` 15-21 red, 6 / 7) and Apollo 11 nightly seed 2 1.06 m on 4.9 deg (`[nightly]` red, 6 / 7). Campaigns 39 / 41 (was 37 / 41). Accepted by the user 2026-11-07 as real rocking near 5 deg: both stay known reds (gate 1 m not loosened); `docs/subsystems/lunar.md` open points.

## Look fixes T7c fix 3, clouds (2026-11-08)

- Accepted as the current look (user 2026-11-08), guarded by `game/tests/cloud_flyup_check.gd` (recorded baseline):
  from orbit the volume's small coarse cumulus cross-fade into the flat levels' larger patches over 205-410 km (a soft
  ring, no bright edge); fly-up region steps above the 4-code target up to 10-12.6 codes at 29-41 km (Gulf, cumulus)
  and 5-8 codes at 150-1100 km; the Sun beam rule is fitted with detail levels off (game's near shadows 4-10 % darker,
  `STARSIM_GAME_DETAIL=1` cloud_check fails 4). Details: `docs/subsystems/weather-clouds.md` open points.

## Horizon clouds at noon (bug hunt 2026-11-08, open: model decision)

- Noon cumulus near the horizon show salmon / peach (pad, 2 km, displayed linear B/R 0.64; far / near 0.755), also on
  the desktop; `tools/look_reference_suite.py` scene `horizon_clouds_noon_2km` fails until decided. No code defect
  found: the display chain reproduces the measured colour exactly from the scene value (white balance + Hunt S 1.50:
  0.652 computed vs 0.639 measured), local chromatic adaptation is inactive at noon (D = 1, K = identity), the
  bright-top term is gated off below 20 km, the short-path form acts below 200 m only, the air shadow adds 3 %, and a
  uniform deck rendered from 2 km follows the core's `sky_luminance_to_point` composite within the per-row noise
  (render no redder). The colour is the model's physics: sunlit cumulus sides (scene B/R 0.90 of the plate white:
  direct-beam dominated) x the 20-60 km boundary-layer path (Rayleigh + aerosol AOD 0.1, Angstrom 1.3, 1.2 km scale
  height: T_B / T_R 0.39 at 30 km, exact spectrum 0.42) + the bluish air light -> scene 0.76 of the plate white, then
  the Hunt step's OkLab chroma x 1.5 (0.755 -> 0.652). Levers (measured with the core): Angstrom 1.3 -> 0.5 (maritime,
  the coastal pad) raises a white cloud 30 km away 0.845 -> 0.937 of the plate white (60 km 1.0 -> 1.13); T9b channel
  wavelengths +~7 % on the direct beam (largely cancelled by the bluer plate white) and over-corrects long paths
  (T_B / T_R 30 km 0.444 vs exact 0.416); Mie g 0.8 -> 0.7 +6-10 % air light only; the Hunt factor's chroma boost.
  Suspected model defect (to check): the volume's sky-lit part of the clouds is 5-7 % of their luminance (diag
  sun-only / sky-only renders at 5 km) against the plate's 14 % expected for a thick top (Sun and sky light reflected
  alike) - clouds lit in the direct beam's colour.
  Report: `.superpowers/sdd/2026-11-07-look-fixes/bug-horizon-report.md`.

## Look-fixes plan, open gate misses (final review 2026-11-09)

- Suite gate `lunar_day_apollo11.ground_luma` (`tools/look_reference_suite.py`): 114.5 against the lower bound 115 (T13
  fix 1 state). Open user decision: the Gilchrist area rule or the Moon-day lever (eye-adaptation.md).
- `moon_surface_check` fails 2 checks, pre-existing (the same 2 at HEAD with the look-fixes T5 round stashed, T5 report,
  not colour): the rock layer's camera-jump timing (see "Boulder-field camera jump" above) and the detail-off leaf morph.

- `terrain_quadtree_test` (slow): 6 Moon data FAILs, pre-existing (identical at 3eee254e^ and after; noted 2026-11-09).
