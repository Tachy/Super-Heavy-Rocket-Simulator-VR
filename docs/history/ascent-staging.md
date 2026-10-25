# Ascent trajectory and staging - history

Verbatim record of this subsystem's notes as they stood in `docs/subsystems.md` on 2026-10-25, before the split. Current state: `docs/subsystems/ascent-staging.md`. Kept for the reasons behind decisions, superseded numbers and earlier approaches; read only when that history is needed.

Plan: `docs/superpowers/archive/plans/2026-10-22-ascent-trajectory-staging.md`, spec
`docs/superpowers/archive/specs/2026-10-22-ascent-trajectory-staging-design.md`.

### Staging sequence (Task 1)
- **Cause:** `FlightSession::stage()` cut the booster and separated in the same tick; the engines' shutdown tail-off
  (Raptor 0.5 s, Merlin 0.3 s) then pushed the light booster alone - right after separation it was 17 m/s (Falcon 9)
  to 34-61 m/s (Starship) faster than the upper stage.
- **Now** (`[stage.staging]` of the lower stage, `config::StagingConfig`; `sim::StagingSequence`,
  `flight_session_staging.cpp`): `stage()` starts a sequence, `AutopilotState::Staging` lies between Booster and
  UpperStage and holds the guidance's last command (no steering change while joined); every event at its own tick.
  - Cold (Falcon 9): MECO (every group off: the tail-off acts on the joined stack), `separation_delay_s` 4.5 s [D]
    later the separation with `separation_dv_mps` 1.0 m/s [E] (pushers), `upper_ignition_delay_s` 6.0 s [D] after it
    the second stage lights. [D] from our CRS-35 / Starlink 15-25 webcast tables (engine icon dark -> dials split ->
    stage 2 speed minimum less the MVac spool), see the TOML comment.
  - Hot (Super Heavy, also depot / tanker / HLS launch through `from`): MECO keeps `run_on_engine_groups` (the centre
    3) at minimum throttle, the ship's engines start while joined (thrust on the stack), the separation (no push)
    comes on the tick none of them is still starting: the ship's start-up 3.0 s [E] (flight 14's table has no samples
    from T+138.75 to T+146.25 s). The booster keeps the centre engines into its flip.
  - Measured (`[staging]`, `test_staging_sequence.cpp`): Falcon 9 booster -1.00 m/s along track right after the
    separation (the push); Starship booster -0.03 m/s at the separation, then -4.6 / -8.9 / -12.4 / -15.1 / -17.8 m/s
    over the next 5 s (it falls back).
  - Without a `[stage.staging]` (trunk, payload stacks) or with the lower stage not burning, `stage()` separates at
    once as before. A second `stage()` during a sequence is ignored (one separation).
  - Timeline: the hot MECO and the upper stage's ignition are recorded at their command (the ignition with the
    serial the upper part gets); the observer skips an Ignition the session recorded within two samples. Predictions:
    separation at MECO + `separation_after_meco_s`, the upper ignition at MECO + `upper_ignition_after_meco_s`.
  - Diagnostics: `STARSIM_DEBUG_STAGING=1` prints both parts' rates and relative speed at the split.
- **Booster flip:** `[stage.recovery] flip_engine_groups` (unset: the final landing set - Super Heavy's centre
  engines, as before; `[]`: engines off, the RCS turns the stage). MECO planning takes the flip from
  `flip_duration_s` (default 10 s [E]; Falcon 9: 13.6 s [D], follow-ups Task 7) plus the sequence's MECO -> separation interval (hot: the
  run-on engines burn at minimum throttle through it, as the flip engines do; starship_leo then reaches landing
  ignition with 52.2 t for 51.0 t planned - 48.6 t without the interval).
- **Falcon 9 flip time (closed by follow-ups Task 7, 2026-10-23).** Before: 8 x 450 N [E] at the interstage gave
  25.8 kNm on a pitch inertia of 1.0e7 kg m2; the flip (on `return_controller_`, no rate limit; the boostback lights at
  15 deg pointing error) took 50.8-53 s from separation to the boostback against ~14 s on CRS-35, and the 50 s was the
  planning value (boostback_loss_fraction 0.19). Research (user decision 2026-10-23: research first, then fix the
  vehicle data; no new guidance mechanism):

  | Quantity | Value | Source | Tag |
  |---|---|---|---|
  | Thrusters, layout | nitrogen cold gas on the interstage, 2 packs of 4 at the top of the booster; they flip the stage and point it in the coast | everydayastronaut.com "Definitive guide to Starship", Falcon 9 section (`ea_rcs`) | [S] |
  | Thrust per thruster | not published (SpaceX, Wikipedia, NASA/AIAA spec sheets, user guide: "nitrogen gas thrusters" only; the "~20 kN" in the same guide is Starship's hot gas thrusters) | - | 6.5 kN [D] |
  | N2 supply (COPV count / volume / pressure) | not published | - | 600 kg [D] |
  | Separation -> boostback startup, CRS-35 | T+2:18 -> T+2:31 = 13 s (MECO T+2:15, flip T+2:22) | SpaceX mission timeline via spacelaunchschedule.com (`tl_crs35`) | [S] |
  | Separation -> boostback startup, Bandwagon-3 (RTLS) | T+2:18 -> T+2:32 = 14 s | `tl_bandwagon3` | [S] |
  | Separation -> boostback startup, Crew-13 (RTLS LZ-40) | T+2:27 -> T+2:41 = 14 s | `tl_crew13` | [S] |
  | Mean | 13.7 s (the timelines give whole seconds, +-0.5 s per time: each interval +-1 s, the mean of three +-0.6 s) | - | [D] |
  | CRS-35 webcast clock (our speed table) | separation T+141.5 s (dials split); stage-1 speed leaves its ballistic -8..-9 km/h per 0.5 s at T+154.5 s (13.0 s), full deceleration (-47 km/h per 0.5 s) from T+160.5 s | `tests/validation/telemetry/f9_crs35_2026-10-13.csv` | [D] |
  | Boostback lit before the turn is complete? | not settled: the sources describe the flip on cold gas, then the boostback (the timelines list "flip" 4 s after the separation and the boostback startup as separate events); no source reports the engines helping the turn; the CRS-35 speed ramp (6 s to full deceleration) fits the 3 Merlins' start-up plus the last degrees of pointing - the model already lights at 15 deg error | `ea_rcs`, the three timelines, our telemetry | open point, no model gap found |
  | Pitch inertia of the empty stage | not published; the model gives 1.01e7 kg m2 at the separation (25.6 t dry + ~53 t propellant) | - | [D] (model) |

  Derived: the thrust is sized on the existing flip controller and the 15 deg gate so that the flown separation ->
  boostback is the 13.7 s mean: sweep (staging test, ample N2) 3.5 / 4 / 5 / 7 / 8 / 9 kN -> 20.4 / 19.0 / 17.0 / 14.3 /
  13.3 / 12.6 s; 6.5 kN with 600 kg N2 -> 13.6 s. Pitch at the separation state (`[staging]` test): one pack's two inward
  thrusters, 13 kN at 28.5 m from the CG = 3.71e5 Nm on 1.01e7 kg m2 = 0.037 rad/s2 (the 265 kNm capability listed
  below is the same pair at the full stage's CG, arm 20.4 m). The RCS alone, bang-bang over the ~150 deg to the 15 deg
  gate, would need ~17 s; the aero moment (q ~ 200 Pa at the separation, falling with height) carries a large share
  of the turn and brings it to 13.6 s (at 5 kN: 0.034 rad/s2 flown against 0.024 from the RCS early in the turn). The
  N2 at Isp 70 s [E]: the full RTLS uses 441 kg (ascent 70, flip 349, coast trims and landing 22 kg), 159 kg left at
  the touchdown = 36 % margin (>= 20 % [E]); 600 kg [D] carved out of the 25.6 t dry mass (dry_mass_kg 25.4 -> 25.0 t,
  total unchanged). A 6.5 kN cold-gas thruster is large (N2 flow 9.5 kg/s each) and rests only on the measured flip
  time: no source gives or bounds the thrust. `flip_duration_s` 50 -> 13.6 s [D] (the flown flip); `boostback_loss_fraction` 0.19 ->
  0.13 [D], re-identified on the full RTLS (0.19 -> 12.8 t, 0.165 -> 13.0 t, 0.13 -> 12.1 t at the landing-burn
  ignition for 12.1 t planned).
  - Re-baselined (old -> new): RTLS mission MECO T+137 / 59.9 t -> separation T+140 / 52.8 t, boostback T+153.8 s
    (13.6 s after the separation), entry burn T+356 s at 57.0 -> 58.2 km, landing burn T+422 -> 423 s at 2.83 -> 2.99 km
    / 282 -> 287 m/s / 12.0 -> 12.1 t, landed T+460 -> 456 s, 5.0 -> 5.8 t left; new checks: separation -> boostback
    within 3 s of 13.7 s (`test_falcon9_rtls`, `test_staging_sequence`), N2 left at the touchdown > 0 (159 kg). Staging
    test: flip 53.0 -> 13.6 s, 92.7 -> 348.5 kg N2. MECO plan from a fixed state: 64.1 / 60.5 -> 50.8 / 47.8 t (with /
    without entry burn; leverage 3.6 -> 3.0 t, bound 4.0 t kept). RCS capability (first stage, at the full stage's CG) roll / pitch /
    yaw 24 / 265 / 265 kNm (3.71e5 Nm pitch at the separation CG). Falcon 9 MECO prediction at T+30 s (`[timeline]`):
    +7.88 s (last recorded, ascent Task 2) -> +7.13 s, bound 9 s kept. Drone ship: MECO 24.6 -> 24.5 t, 8367 -> 8376 km/h; landing burn 2.97 -> 2.95 km; N2 426 kg left at
    the touchdown; campaigns unchanged (landed 21/21, 11/11, 10/10). ISS window (`falcon9_crew_dragon_iss.toml`):
    `insertion_s` 547 -> 540 s, `insertion_arc_deg` 14.78 -> 15.08 [D] (phase error 0.006 deg). Leg drop "steep ones are
    lost": 25 deg at 2 m/s now lands 6.3 m off (the 6.5 kN thrusters hold the attitude against the blow; 450 N could
    not) - the case runs at 3 m/s (impact, stroke 0.502 m).
  - **Open discrepancy - CRS-35 MECO (was INTERIM; resolved as far as sourced data reach by follow-ups Tasks 7b / 7c,
    allowance now 0.073):** history: the CRS-35 check
    (`test_falcon9_ascent`) MECO speed 6596 -> 6985 km/h against 6490 recorded (+1.6 -> +7.6 %; `meco_speed_allowance`
    0.03 -> 0.097 = the measured 7.63 % + 2 % = 9.63 % rounded up, bound 224 -> 659 km/h);
    MECO T+132.7 -> 135.5 s (134.6 recorded), 58.2 -> 59.7 km (57.4), path angle 41.3 -> 38.2 deg (36.6), kick 11.57 ->
    12.27 deg, 60.8 -> 53.6 t kept at MECO (gauge ~66 t). The 37 s shorter flip makes the return cheaper (the stage
    no longer coasts ~50 km further downrange before the boostback), so the planner gives the ascent more. The real
    stage keeps more: CRS-35's boostback burns ~55 s (speed table: lit ~T+154.5 s, stage-1 speed minimum T+199 s,
    burning until ~T+209.5 s, 621 m/s back; timeline T+2:31-3:25 = 54 s; Bandwagon-3 43 s, Crew-13 46 s) against our 43
    s (T+153.8-197.0 s, 676 m/s back at apogee 114.5 km). The 50 s flip had hidden this gap.
    **Task 7b diagnosis (2026-10-23): the boostback is not the cause** - CRS-35's boostback needs 2014 m/s against our
    2100 m/s and burns 38-41 t against our 38.8 t; its 55 s come from a soft start and a trim at the end, not from more
    delta-v; the MECO speed gap is the ascent's (section "Falcon 9 boostback against CRS-35 (follow-ups Task 7b)").
    Allowance kept at 0.097 until the ascent side is resolved. **Task 7c diagnosis:** most of the late excess is the
    Merlin's vacuum thrust (our 932 kN against SpaceX's 914 kN); the rest is the max-Q bucket, a ~3.9 g levelling in the
    last seconds and MECO 0.9 s late - section "Falcon 9 late ascent against CRS-35 (follow-ups Task 7c)". User decision
    (A): 914 kN taken; MECO speed +7.6 -> +5.3 %, allowance 0.097 -> 0.073.
- **SCvx landing (depot catch):** the depot's booster reaches ignition ~70 m and ~10 m/s across the approach (also at
  HEAD: its coast removes a ~200 m cross miss in the air by aero steering, which leaves the cross velocity), now ~25 m
  further out than before. Its first plan then dropped the 13 -> 5 engines 1.72 s into their 3 s start-up while the
  plan counted the 5 at full thrust (57 % spool): the tracking fell behind, the attitude swung up to 40 deg and the catch
  was aborted. Now ignition engines that the next set drops run until they are at thrust:
  `DescentLimits::phase0_min_s` = the rest of their start-up (from the plan's start: re-plans less the latency), set by
  `ScvxLandingGuidance` for start phase 0 when the next set drops engines (Super Heavy; not Falcon 9, whose next set adds
  engines; not in an abort). Caught; starship_leo and depot re-plans 22/22 and 23/23 converged (HEAD 23/23). Tried and
  rejected: the bound for every plan (the planner unit test from a 1.2 km ignition stalled, the Falcon 9 abort tipped
  over), waiting with the switch in the guidance only (still aborted), modelling the spool into the later sets
  (starship_leo re-plans stopped converging low down).

### Re-baselined reference numbers (Task 1)
- Falcon 9 `boostback_loss_fraction` 0.12 -> 0.19 [D]: identified again on the full RTLS with the 50 s RCS flip
  (0.12 arrived at the landing burn with 10.0 t instead of 12.1 t; now 12.0 t). `test_falcon9_rtls` entry-burn
  leverage bound 3.5 -> 4.0 t (3.6 t with the longer flip and loss).
- Falcon 9 RTLS mission: MECO T+137 s -> separation T+141 s, entry burn T+354 s, landing burn T+428 s at 2.74 km /
  276 m/s / 12.0 t, landed 0.00 m off at T+464 s (CRS-35 touchdown T+442 s).
- Falcon 9 drone ship (`falcon9_asds.toml`): the deck moves from 32.9272 / -76.0990 to 32.8819 / -76.1490 deg [D]
  (the hidden "nominal impact point" probe, now 10 s after the separation): the tail-off no longer pushes the stage
  alone (~7 km shorter). The ASDS campaigns start 10 s after the separation instead of after MECO.
- CRS-35 ascent check (`test_falcon9_ascent`): its MECO sample is the end of the booster phase (state Staging), not
  the first UpperStage tick (that is the upper ignition now, 10.5 s later).
- Falcon 9 MECO prediction at T+30 s: +7.35 -> +8.44 s (bound 8 -> 9 s): the cutoff propellant included the 54.5 s
  MECO -> boostback and the loss 0.19 (since follow-ups Task 7: 18.1 s MECO -> boostback, loss 0.13, +7.13 s).
- Starship booster return predictions (`[timeline]`): landing burn -5.4 -> -13.3 s and catch -16 -> -23.5 s at the
  apex (first sample, sink near 0: ill-conditioned), unchanged from 120 s on (4.9 / 4.6 s); bounds 7 / 20 -> 15 / 25 s
  plus 6 s within 120 s.
- ISS window (`falcon9_crew_dragon_iss.toml`): `insertion_s` 535 -> 547 s, `insertion_arc_deg` 15.0 -> 14.78 [D] (the
  cold sequence adds 4.5 + 6 s before the second stage burns).
- Refuelling reference flights (`config/refuelling/reference_flights.toml`, re-measured by the slow r5 tanker_flight):
  residue 97.2 -> 97.3 t, rendezvous 17.0 -> 15.64 t, delivered LOX / CH4 53.1 / 13.2 -> 54.1 / 13.6 t, settling gas
  0.92 -> 0.94 t, main at catch 1.88 -> 1.19 t (the tanker's ascent and staging moved its phasing start; the
  rendezvous cost varies 13-21 t between flights); depot and HLS numbers unchanged; `inputs_digest` a19eb2bbbaf8285c.
  With these the flown campaign needed 14 tankers against the calculator's 13 (tanker 8: depot 547.5 t flown, 560.4 t
  calculated): `campaign_offset_above_t` / `below_t` 2.25 / 1.7 -> 1.6 / -0.5 t [D], the mean flown - modelled delivery
  of the flights whose phasing class flown and calculator agree on (7 above, 3 below).

### Max-Q limit and operational ascent throttle (Task 2)
- **Before:** the missions carried first-stage throttle schedules over mission time (`booster_throttle`), fitted to
  one flight each (Starship: 0.85 from liftoff, 0.76-0.85 later, 0.76 before MECO, level fitted to the flight 13/14
  speed; Falcon 9: the CRS-35 bucket 0.72 at T+50-61 s, then 0.95). Removed with the key.
- **Now** (`gnc::AscentGuidance::booster_throttle`): the booster flies its operational throttle `[stage]
  max_ascent_throttle` and is throttled below it to hold the dynamic pressure at `max_dynamic_pressure_pa`
  (`FlightSession::engage_ascent_autopilot` copies both from the lowest stage into `PitchProgram`).
  - Limiter law (`gnc::max_q_thrust_acceleration_mps2`): feedback linearisation of q = rho v^2/2 along the path,
    dq/dt = rho v dv/dt + v^2/2 drho/dt = (limit - q) / tau with tau = `max_q_time_constant_s` 1 s [E] (short against
    q's own build-up near max-Q, ~20-30 s: full thrust holds to ~95 % of the limit; long enough that the commanded
    throttle moves by ~0.1-0.2 /s, inside the engines' 0.5 /s rate - no lag-driven overshoot). dv/dt besides thrust from
    the accelerometer (last tick) less the engines' thrust now, gravity and the centrifugal term; drho/dt from US76's
    gradient (scaled as the truth air) times the climb rate. Thrust to throttle: `gnc::booster_throttle_for_thrust`
    (the producing engines' common range, linear thrust).
  - Values: Starship `max_dynamic_pressure_pa` 38909 Pa [D] = flight 14's peak q (its q holds 38-39 kPa from T+58 to
    T+71 s, a max-Q hold: the gauge shows the engines throttled at Mach 1.39-1.70, flight 13 at Mach 1.62-1.88, both
    supersonic - no transonic hold, so Starship keeps one value). Falcon 9 over Mach (fix round 1 after the review):
    `[[0.80, 35567], [0.90, 29600], [1.12, 29600], [1.45, 35567]]` [D]. Both Falcon 9 records hold q transonically with
    the engines throttled (gauge flow) and let it rise once supersonic - CRS-35 flat at 28.7-29.6 kPa over Mach
    0.92-1.12 (throttled T+52-66 s), Starlink 15-25 flat at 31.0-31.5 kPa over Mach 0.97-1.13 (throttled T+56-68 s);
    at full thrust they reach 31.6 kPa (Mach 1.43) and 35.6 kPa (Mach 1.55). Transonic: the lower hold (CRS-35's);
    elsewhere the highest q flown at full thrust (no evidence of a lower limit); ramps where the records leave the hold.
    Transonic loads are the binding case of a launcher (why it buckets there); a single peak-q value (first version:
    CRS-35's 31.6 kPa, reached supersonic after the bucket) let the stack through the transonic band 2 kPa high. The
    post-bucket ~95 % of the old schedule was a G-dial fit: the gauge shows the rated flow after the bucket (0.997 /
    0.993, T+66-100 s), so full thrust there is data. New tool `tools/telemetry/peak_dynamic_pressure.py` (speed +
    altitude + US76, the core's model; with `--capacity-t` / `--rated-flow-tps` the gauge's throttled phases, their
    Mach range and the q held in them). Super Heavy
    `max_ascent_throttle` 0.88 [D] (user 2026-10-23: the flown throttle is vehicle data, 280 tf stays the engines'
    capability): the propellant gauge's mean flow T+20-56 s against the rated 28.00 t/s, 0.873 (flight 14) / 0.894
    (flight 13), `tools/telemetry/mass_estimate.py` (new summary line "mean flow"). After max-Q (T+70-125 s) the flow
    stays at 0.871 / 0.856 and the last ~10 s fall with the acceleration: no late-boost limit. Falcon 9: none (the
    425 t tanks' gauge flow is the rated flow; the "92-94 %" note of `falcon9_block5.toml` belongs to the rejected
    395.7 t load).
  - Measured (`[ascent]` "The ascent holds dynamic pressure at the vehicle limit", free launches): Falcon 9 max q
    32.2 kPa, at most 1.000 of the limit at its Mach (transonic hold at 29.6 kPa), lowest throttle 0.805, full thrust
    below 90 %; the free Starship (kick 10 deg, 0.88) peaks at 33.8 kPa (0.87 of the limit, never limited).
  - MECO forecast: the limiter and the cap run forward on a point mass (section "Mission timeline"); free launches
    within 0.02 s (Starship) / 0.32 s (Falcon 9) of the flown MECO over the booster phase from the kick's end, when the
    forecast is first shown ("MECO forecast follows the limiter", re-measured in the final review; before the
    suppression Falcon 9's samples during the rise and kick reached 1.2 s).
- **Open discrepancy - Starship vs flight 14** (resolved up to max-Q in Task 6: the dials' time base and flight 14's
  booster load; see "Validation against flight 14 and CRS-35"): at the gauge's own flow the stack is faster than the flights (test
  "Starship ascent follows the webcast telemetry ...", flown with the recorded 500 t MECO): T+30 / 60 / 90 / 120 s
  2.3 / 10.2 / 24.9 / 47.4 km, 579 / 1516 / 3029 / 5313 km/h against 1.6 / 8.4 / 20.5 / 38.2 km, 515 / 1406 / 2772 /
  4848 km/h (+12.4 / +7.8 / +9.3 / +9.6 %); MECO T+126 s / 52.9 km / 5869 km/h against ~T+140 s / ~53 km / ~5950
  km/h. Not tuned (bounds 0.12 -> 0.13 and 12 -> 15 s; Task 6 rewrites the check). The conclusion of that time ("the
  excess is in thrust per flow or drag, not mass") did not hold: Task 6 found the dials' time base (3.50 s late, the
  "+12 %" at T+30 s) and flight 14's propellant load (the mass_estimate fit's 5542-5603 t against our full 5715 t) as
  the causes - with both the model follows flight 14 within the readings up to max-Q, no thrust or drag error is needed
  there (section "Validation against flight 14 and CRS-35": the remaining gaps after max-Q are flight 14's throttling
  and its engine out).
- **Open discrepancy - Falcon 9 vs CRS-35** (review of Task 2; Task 6: the T+30 s excess was the dials' time base,
  the remaining gaps in "Validation against flight 14 and CRS-35"): `test_falcon9_ascent` "... CRS-35 webcast telemetry"
  T+30 / 60 / 90 / 120 s 529 / 1335 / 2864 / 5316 km/h against 485 / 1188 / 2474 / 4672 km/h (+9 / +12.4 / +15.8 /
  +13.8 %); with the former schedule +2.6 % / +2.9 % at T+60 / 120 s - its deep 72 % bucket and 95 % afterwards (a G-dial
  fit the gauge contradicts) hid the excess. The excess is there before any throttling (T+30 s +9 %, 2.1 against
  1.4 km) at the gauge's rated flow, the same sign and size as Starship's: a common cause in thrust per flow, drag or
  the webcast clock (the old docs saw the real stack ~2.5 s late), not in the q limit (the Mach-dependent transonic
  hold moved T+60 s only from +14.4 % to +12.4 %). Bound: speed at T+60 / 120 s within 15 % (catches a further
  regression). Follow-up: Task 6 (ascent validation against the flights) - drag table, Isp / thrust per flow, liftoff
  time of the tables.
- **SCvx fresh plans** 30 -> 60 iterations [E] (`ScvxSettings::fresh_max_iterations`): the Falcon 9 ASDS MECO campaign
  case seed 107 (aero x0.91, ignition 3.8 km high, 1.3 km short, 14.8 t) had its first deck plan refused at 30 and
  splashed down 172 m beside the deck; it converges in 42 and lands. Re-plans keep their 10. Cost: the simulation
  waits for a fresh plan, and one that does not converge is now refused after 60 iterations instead of 30 - twice the
  wait before the abort / give-up path starts. Measured (final review, replay of
  `falcon9_asds_seed117_741m.txt` as a fresh plan without the soft reserve: it never converges): ~25 ms per iteration,
  refused after 1.58 s instead of 0.81 s (the simulation thread stands that long; a converging plan pays only its
  own iterations, seed 107 42).

### Re-baselined reference numbers (Task 2)
- Falcon 9 drone ship (`falcon9_asds.toml`): deck 32.8819 / -76.1490 -> 33.2240 / -75.7724 deg [D] (the hidden
  "nominal impact point" probe: separation T+150.4 s at 75.3 km, 8249 km/h, 30.7 deg path - Starlink 15-25: 60 km,
  ~25 deg; ~43 km further). The widening below stays needed with the Mach-dependent limit (the deck moved only 1.5 km
  from the scalar limit's 33.2139 / -75.7839): the higher, steeper arc comes from the faster stack (open discrepancy
  above) and the 13 deg kick, not from the q limit. Mission
  landed 0.00 m off; entry burn T+429 s, landing burn T+514 s at 3.03 km / 274 m/s / 12.0 t (Starlink 15-25: T+364 /
  476 s; bounds 25 -> 75 / 50 s: the 13 deg kick and the higher arc - Task 3 optimises the kick).
- CRS-35 check (`test_falcon9_ascent`): MECO T+131 s / 58.0 km / 6412 km/h (recorded 137 / 56.6 / 6441, within the
  10 % bounds); T+60 / 120 s 1335 / 5316 km/h (recorded 1188 / 4672), now asserted within 15 %.
- Super Heavy return on starship_leo ("Block 3 plans MECO ..."): boostback T+148-169 s from 273 t, apogee 103.1 km at
  T+221 s, landing-burn ignition T+383 s with 52.4 t (51.0 t planned): apogee and ignition bounds 15 -> 25 s (flight 14
  T+240 / ~398 s; MECO earlier with the faster stack).
- MECO predictions: starship_leo +5.0 s at T+30 s (was +7.0), Falcon 9 RTLS +7.9 s (was +8.4); bounds unchanged.
- Refuelling windows: tanker `insertion_s` 445.5 -> 436.1 s, `insertion_arc_deg` 12.20 -> 11.93; HLS 428.3 -> 421.4 s,
  11.68 -> 11.37 (`tanker_flight(_orbit).toml`, `hls_refuel(_orbit).toml`, the flown ascents).
- Refuelling reference flights (`reference_flights.toml`, r5 flights): depot initial fill 55.5 -> 69.6 t, tanker
  residue 97.3 -> 93.0 t, rendezvous 15.64 -> 15.49 t, delivered LOX / CH4 54.1 / 13.6 -> 50.9 / 12.7 t, settling gas
  0.94 -> 0.90 t, main at catch 1.19 -> 1.08 t, HLS main at docking 193.6 -> 182.4 t; `inputs_digest` c924f219bd0caf00.
  The calculator: 14 tankers / 52.3 d (was 13 / 52.5). The full flown campaign: 14 tankers, 60.8 days, comparison
  passed (fills per flight against the calculator's model: above-depot class -0.4..+1.2 t, below -0.3..+2.3 t; the
  campaign offsets kept); placed campaign passed.

### Landing planner: propellant floor and gimbal allowance (Task 2b)
- **Failure:** Falcon 9 ASDS MECO campaign seed 117 (aero x1.07) hit the deck at 5.5 m/s after Task 2's MECO state.
  Its long entry burn (removing a ~2.2 km miss) leaves 9.8 t at landing-burn ignition (nominal 12.0 t), and every
  SCvx plan of the burn ends exactly on the propellant floor (dry mass + 5 t reserve [E]). The 848 m re-plan converged
  on the floor; the next one at 741 m (185 m / 24 m/s across, 103.5 m/s down, 12.9 s to go) stalled at virtual
  control 0.0066 - all of it in the **mass** of the last interval (~66 kg short), not in the gimbal cone, tilt or rate
  limits (all within). Four unconverged re-plans gave the deck up; the abort plans failed too; hard touchdown.
- **Diagnostics** (new): `STARSIM_DUMP_SCVX=<dir>` writes every plan request (model tables, state, target, settings,
  warm start; `gnc/descent_request_io`) as text; the hidden test `[scvx-diag]` replays one
  (`STARSIM_SCVX_REQUEST=<file>`, overrides `STARSIM_SCVX_OVERRIDE="limits.gimbal_fraction 0.45;..."`) and prints each
  node against its limits and its nonlinear defect with the offending state component. Replays reproduce the flight
  bit for bit. Findings at 741 m: floor lowered 0.5 t -> converges, 0.1 t into the reserve; gimbal fraction 0.45 or
  0.6 -> converges on the floor; final-lateral weight 0 or 60 iterations -> no change. Fuel optimum from the ignition
  with the floor removed: ~0.1 t below it (gimbal 0.3 .. 1.0 alike): the burn is propellant-marginal.
- **What the reserve is** (`SnapshotSettings::reserve_kg`, review fix round 1): our tanks burn to empty - no unusable
  residual is modelled - so for Super Heavy (5 t [E]), Falcon 9 (5 t, Super Heavy's value) and the HLS (5 t [E]) the
  reserve is planning margin. The ship is different: its landing is fed by the header tanks (the engines starve when
  they run dry) while the propellant mass also counts the main tanks' residue, so its 1 t stays a hard floor
  (`ship_landing_parameters`: both fractions below 0).
- **Fix 1 - soft part of the reserve** (`DescentLimits::soft_reserve_kg`, `DescentWeights::reserve_use` 100 per 1e4 kg
  [E]): a plan may end that far below the floor at 100x the propellant's own weight (exact L1 penalty: zero when the
  target is reachable above it; the virtual control stays 10x dearer); below it the floor is hard. Against the
  final-set lateral penalty (`final_lateral` 0.01 per m and node) the weights break even at 1 kg of reserve per metre
  of offset at one node (correction, final review: the first wording said the lateral penalty did not buy reserve - it
  does wherever 1 kg removes more than 1 m-node of offset; "Results" below: the ASDS plans spend the margin on it). Shares
  (`ScvxLandingParameters`): first plans 0; re-plans of the target `replan_reserve_fraction` 0.5 [E] - plans of a
  marginal burn sit on the floor, and a re-plan from a measured state slightly off the previous plan needs a little
  more; abort plans `abort_reserve_fraction` 1.0 [D] - the stage is given up, reaching the water is what matters. The
  741 m snapshot converges in 3 iterations, 101 kg into the reserve (fixture
  `tests/validation/fixtures/scvx/falcon9_asds_seed117_741m.txt`); the 848 m one stays on the floor.
- **Fix 2 - one Falcon 9 gimbal allowance** (`[stage.recovery] landing_gimbal_fraction` 0.45 [E] - the derivation below
  leaves a band 0.40-0.45, the value is chosen from it (final review); 0 in the file means the default; the whole landing
  burn, RTLS and drone ship; the default `DescentLimits::gimbal_fraction` 0.3 is Super Heavy's, from its catch
  campaign: 3 Raptors near minimum throttle, a limit cycle at 0.4). With the soft reserve alone seed 117 failed again at
  471 m - there the defect sits in the attitude at the start (the plan needs another tilt than the measured one:
  torque-limited reach, every final node on the 1.5 deg cone); even with no floor it stalls at 0.3.
  Derivation (`ScvxLandingGuidance::Statistics`: the LQR's commanded tilt acceleration - plan feed-forward plus
  feedback - over what the whole gimbal gives at the thrust, peak in the last 10 s of the burn):
  - Reach (lower bound): seed 117 at 0.30 ditches (12 failed re-plans), at 0.35 lands with 3 failed re-plans, at
    0.40 / 0.45 with none.
  - Authority (upper bound): at 0.45 the command peaks at <= 0.50 of the whole gimbal in 62 of 63 Falcon 9 campaign
    cases (RTLS <= 0.10, ASDS ignition <= 0.28, ASDS MECO <= 0.50), never clipped: half the gimbal stays for
    disturbances. The feedback alone grows with the allowance (seeds 106 / 116 / 118: 0.16-0.25 at 0.30, 0.43-0.59 at
    0.45) but largely opposes the feed-forward, so the sum is what counts. Seed 117 peaks at 1.06 (14 of 2486 ticks
    clipped; 1.35 and 92 ticks at 0.40, 3.8 at 0.35): at ~400 m its slanted descent still meets the cross-flow below.
  - 0.45: the lowest allowance with a reach margin over 0.40 that keeps the 50 % authority reserve; 0.40 left seed 117
    more saturated (it diverts later).
- **Mid-burn clipping (pre-existing, analysed):** on the drone-ship burns the LQR clips for 600-2600 ticks per case,
  peaks 5-9x the gimbal's reach, the same at 0.3 and 0.45 - not at ignition (no attitude hand-over effect) but at
  ~1.4-0.8 km, 120-150 m/s down, 20-30 m/s across. The accelerometer shows 3-5 m/s^2 of body-lateral specific force
  there, while the whole gimbal gives ~2 m/s^2 (845 kN sin 5 deg / 36 t): the slender body's cross-flow normal force
  at ~10 deg angle of attack, which the planning model lacks (axial drag along the motion only). The velocity error
  grows ~4 m/s^2 within each plan and the 1 s re-plans absorb it (plans lengthen by ~2 s each, then settle below
  ~600 m as q falls). A real authority shortfall against a model gap, not a margin question: no allowance fixes it.
  Follow-up: cross-flow normal force in `descent_model` - done (Task 8, next item).
- **Hull normal force in the planning model (follow-ups Task 8, 2026-10-23).** Shared physics for every vehicle that
  plans with `descent_model` (Falcon 9, Super Heavy, the ship, the HLS - in vacuum it is zero):
  - Model: the flight model's slender body (`aero::SlenderBodyAero`): the Jorgensen cross-flow term
    `q L D cd_x(M) sin^2 a` at mid-length (`crossflow_cd_by_mach` = eta C_dc from the stage's `CrossflowCurve`, else the
    built-in `aero::generic_crossflow_efficiency` 0.68 x `generic_crossflow_drag_coefficient` 1.2, now named in
    `slender_body_aero.hpp`, `SlenderBodyAero::continuum_crossflow_cd`) and, for a body with a nose (the ship), the
    potential term `q A |sin 2a| cos(a/2)` at the ogive's centre of pressure (`potential_station_m`), both against the
    air velocity's cross component, with their moments about the CG (`DescentModel::hull_normal_force`). Potential
    term choice: at 10 deg tail first it is ~12 % of the ship's normal force but acts at the nose (the long arm), so the
    ship's model carries it; the boosters' flat tops have none. Continuum only (no free-molecular blend: landing burns
    fly below 3 km). Grid fins and flaps stay out of the plan (fins feathered, flaps cancelled by the allocation).
    Checked against `SlenderBodyAero::compute` to 1e-6 (`[descent]`: Falcon 9 at 1.2 km / 250 m/s / 10 deg: 3.36 m/s^2
    lateral, the force the planner lacked; ship with the nose term; moment sign and value; Jacobians).
  - The return guidance's online drag estimate (`BoosterReturnGuidance::drag_scale`, frozen at landing-burn ignition)
    scales the planned normal force (`DescentModel::normal_force_scale`, via `SnapshotSettings`); the axial drag table
    of the plan stays unscaled as before (open: scaling it too).
  - The landing LQR is consistent with the plan: its command and the plan's feed-forward are the net tilt acceleration,
    which contains the hull moment; `ScvxLandingGuidance::update` cancels the model's moment at the measured state and
    gives the gimbals the rest (clipping and gimbal use count that part). A booster's torque goes to the gimbals as it
    is; the ship's allocation cancels its modelled moment (hull + flaps) itself, so it gets the net torque. Before, a
    booster's gimbals were asked for the plan's whole tilt acceleration and the real hull moment came on top: with the
    moment in the plan that counted it twice (first measurement: RTLS clipping 0 -> 446 ticks, peak 0.67 -> 4.25).
  - SCvx stall rule: fresh plans (no warm start) wait `ScvxSettings::fresh_stall_iterations` 4 [E] (re-plans keep 3).
    Super Heavy's sea abort from ~350 m (test "A commanded abort late in the landing burn ...") passes a transient of
    its virtual control with the normal force (37.3, 2.72, 6.19, 6.59, 2.27, converged at iteration 17); at 3 it was
    called stalled, the sea requirement was dropped and the pull-only abort landed the booster 31 m from the tower.
    Fixture `tests/validation/fixtures/scvx/superheavy_abort_sea_353m.txt`. Abort test (A -> B): 1500 m Wasserung
    630 m / 25.7 t -> 552 m / 12.4 t; 800 m 1372 m / 0.0 t -> 549 m / 10.0 t; 400 m 539 m / 5.7 t -> 532 m / 6.1 t.
    The 1500 m case burns more because it now flies its plan: in A the stage lost control in the divert (tilt ~140 deg,
    2 rad/s in the debug trace; its plan had no hull moment) and fell into the water at ~90 m/s with
    25.7 t unburned; in B it flies the planned powered divert and sets down in the water (12.4 t left).
  - A/B measurement (A = `STARSIM_DEBUG_NO_CROSSFLOW=1`: the planner without the normal force; B = all of the above;
    clipped LQR ticks summed over a test case's burns, of all lateral ticks; peak = largest gimbal use; no case clips
    in its last 10 s in A or B):

    | Test case | burns | A clipped | B clipped | A peak | B peak |
    |---|---|---|---|---|---|
    | Falcon 9 ASDS MECO states 1/2 | 11 | 11379 / 82038 | 0 / 82456 | 8.21 | 0.98 |
    | Falcon 9 ASDS MECO states 2/2 | 10 | 9421 / 72696 | 0 / 73036 | 7.89 | 0.95 |
    | Falcon 9 ASDS at ignition | 21 | 18970 / 151176 | 237 / 151618 | 7.93 | 1.97 |
    | Falcon 9 RTLS at ignition | 21 | 0 / 110642 | 0 / 110620 | 0.67 | 0.72 |
    | Super Heavy catch campaign | 21 | 0 / 126073 | 0 / 125972 | 0.71 | 0.74 |
    | Super Heavy nominal + deterministic | 3 | 0 | 0 | 0.68 | 0.66 |
    | Ship catch campaign (failed plans) | 21 | 0 / 86877 (6) | 39 / 85609 (1) | 0.80 | 1.09 |
    | Lunar-return seed-18 gate (ship) | 1 | 0 / 4195 | 20 / 4060 | 0.92 | 1.06 |
    | Ship flip/catch, abort, Mach 2.6 entry, return campaigns | 1-4 | 0 | 0 | <= 0.92 | <= 0.85 |

    Outcomes (A -> B): every campaign as before - ASDS 11/11, 10/10, 21/21 landed (>= 4983 -> >= 4986 kg), RTLS 21/21
    (>= 5747 -> >= 5744 kg, sink <= 1.02 -> 1.05 m/s: re-baselined - the campaign is deterministic, so this is the
    model change, not noise; the plans now use the normal force, the touchdown stays inside the sink limit), Super Heavy catch 21/21, ship catch campaign 20 of 21 (the
    known engine-failure case), full Falcon 9 RTLS mission (`test_falcon9_rtls`) 6.0 -> 6.4 t left at the touchdown - and all 198 cases of the labels descent, scvx, scvx_landing, landing, catch, asds,
    rtls, falcon9, ship, booster, replanner pass.
  - Result: the drone-ship mid-burn clipping is gone in the MECO campaigns and down 99 % at ignition (237 ticks, peak
    1.97, in a few dispersed cases); the ship now clips a little where it did not (39 + 20 ticks, peaks ~1.1, none in the
    last 10 s). Follow-up: those remaining ticks (the ship's flaps, which the allocation cancels from the flight model
    while the plan has none; the ASDS ignition dispersions), and the plan's axial drag under the online estimate.
  - Fidelity flag `SnapshotSettings::hull_normal_force` (default on; off plans with axial drag only).
  - Diagnostics: `STARSIM_DEBUG_NO_CROSSFLOW=1` (overrides the flag off: the A/B baseline),
    `STARSIM_DEBUG_SCVX_SOLVER=1` now also prints the virtual control per SCvx iteration (`[scvx it]`), with
    `STARSIM_VERBOSE=1` every `test::fly_landing` prints a `[scvx clip]` line.
- **Fix 3 - the abort decision** (the RTLS "cannot reach the landing zone" case, lit 900 m beyond the zone, broke at
  0.45: Aufprall on land 520 m from the zone instead of the water beyond 650 m). Replays: the first pad plan converges,
  but only by tilting the single-engine final phase to ~25 deg against its 10 deg limit (soft limit; fixture
  `falcon9_rtls_900m_beyond_zone.txt`); the stage flew that 30 s near-hover divert, the re-plans ran out of
  propellant at 626 m (that re-plan needs ~3.6 t of the 5 t reserve, the soft part is 2.5 t; only the full gimbal
  converges, exactly on the hard floor), the target was given
  up at 550 m, and the abort plan for the water then needed ~3.3 t of the reserve it was not allowed to touch (pull-only
  fallback: on land). Two changes: (1) abort plans may use the whole reserve (above); with that alone the abort plan
  converged but the stage, already 38 deg over, tipped over flying it. (2) A converged plan whose final set tilts
  more than `max_planned_tilt_excess_rad` 3 deg [E] beyond its soft limit (more than a start state on that set
  already does; `DescentPlan::planned_tilt_excess` from the subproblem's slacks, nodes after the switch node - that
  one still carries the outgoing set's attitude: the full CRS-35 RTLS mission's first plan switches at 15 deg against
  the final 10 and lands) does not reach the target within the
  vehicle's authority: where there is a sea it counts like an unreachable plan (first plan: abort at once; re-plans:
  towards `failures_before_abort`). The soft limits stay what they were for: re-plans from violating states. Final set
  only: seed 107's first drone-ship plan tilts its phase 0 to 37 deg against 20 and is flown fine (high thrust, 19 s).
  The case ditches again, 800 m from the zone.
- **Results** (`asds-single` seeds 0, 101-120): before 20 of 21 (seed 117 Aufprall, 9 failed re-plans); soft reserve
  only 20 of 21 (seed 117 Wasserung after a failed 471 m re-plan); gimbal 0.45 only: seed 117 Aufprall; with all fixes
  21 of 21, **no failed re-plan** in any case. Every ASDS case lands with ~5.0 t (the floor; seed 107 7.8 t): the
  plans spend the margin on the final-set lateral penalty, so the soft reserve matters for all of them.
- Re-validation (ctest labels asds, rtls, catch, landing, scvx, scvx_landing, descent, lunar_descent_plan,
  lunar_descent_session, lunar_scvx, lunar_descent_flight, lunar_campaign, crew_escape, falcon9, booster, ship,
  replanner, hls, return; [slow] included): 293 of 293 passed. Counts against before: Super Heavy SCvx catch 21 of 21
  (21), ship catch 20 of 21, none lost (20), Falcon 9 RTLS 21 of 21 (21), ASDS ignition 21 of 21 (21), ASDS MECO 21 of
  21 (20; seed 117 4.6 t left, 0.4 t of the reserve), lunar landing 21 of 21 (21), lunar return direct 21 of 21 and
  aerocapture 21 of 21 (21 / 21); the full CRS-35 RTLS mission lands 0.00 m off with 5.0 t; ship return campaigns,
  crew escape, the refuelling tanker flights passed. No failed re-plan in any ASDS case (MECO: 9 before).
- `ScvxSettings::fresh_max_iterations` 60 (Task 2) stays: seed 107's first plan is a fresh plan (no soft reserve) and
  still needs 44 iterations at gimbal 0.45 (41 at 0.3).

### Pre-launch ascent planner (Task 3)
- `sim::plan_ascent_profile(like, request)` (`ascent_profile_planner.cpp`) chooses the first-stage kick by trial
  ascents (`sim::fly_ascent_trial`): each a fresh `FlightSession` on `like`'s ephemeris, Earth orientation, gravity
  model and config dir (its interpolated sources read directly, no second interpolation layer), the request's stack on
  its pad, booster return on, the ascent autopilot with the candidate kick, at the flight's 5 ms tick (no coarser trial
  tick). Objective: the upper stage's propellant (oxidizer + fuel) once SECO's engine tail-off has ended (PEG counts
  on the tail-off impulse, ~9 m/s on Starship: read at the cutoff command the periapsis was 30 km short).
- Booster recovery as the hard constraint: the MECO plan (`BoosterReturnGuidance::meco_propellant_kg`) decides MECO
  in every trial, as in the flight; a non-finite plan is infeasible. The return itself is not flown: at the separation
  the trial ends the booster's guidance (`FlightSession::end_booster_return`) - it changes nothing for the upper stage
  (identical propellant at SECO to the gram) and saved ~45 % of a trial (11.5 -> 6.4 s serial).
- Feasible: the guidance cut off before the stage ran dry and the radius flown over the next revolution
  (`predict_trajectory`) lies within 5 km [E] of the target's periapsis and apoapsis. Not the osculating apsides at
  cutoff (199.7 x 212 km for a 200 km target, by design of the J2-aware insertion) nor `orbit::j2_mean_orbit`'s mean
  apsides (207 x 208 km: the averaged osculating a sits ~7 km above the flown radius).
- Search: kick in [4, 25] deg [E]; start 10.5 / 13 / 15.5 deg [E]; then the vertex of the parabola through the best
  trial and its two neighbours (a Newton step on d(propellant)/d(kick), slope and curvature from the three trials),
  also when the best is the outermost (clamped to 1.618 x the spacing beyond it); golden section where the fit is
  convex or leaves the bracket and towards an infeasible neighbour; stop when the next step is < 0.1 deg [E]. Memoised
  per process by the request without its liftoff epoch (the optimum does not depend on the hour), with the `like`
  session's gravity model in the key (final review: the trials fly that model; a session with another
  `PointMassGravityConfig` gets its own plan).
- Measured: Starship to 200 km / 26.5 deg (starship_leo's request) 14.85 deg, 88.5 t at SECO, 4 trials, 15-26 s wall
  (parallel test load / serial); the 1-deg sweep 6..22 deg peaks at 15 deg (88.5 t; 6 and 22 deg do not reach the
  orbit), refined at 0.2 deg at 14.8 deg (88.493 t) - smooth, no visible trial noise. MECO flight-path angle 31.2 deg;
  the ship never pitches below its flight-path angle in the first 60 s after the separation (no kink). Falcon 9 RTLS
  (falcon9_rtls's request, probe): 11.57 deg, 2.2 t, 4 trials, 10.8 s. Longer than the free launch's 10 s countdown:
  it must hold for the plan (spec).

### Recovery choice and refusals (Task 4)
- `sim::plan_launch(like, request)` raises a target inclination below the site's latitude to it (due east: the lowest
  reachable), chooses the recovery (`RecoveryChoice`: TowerCatch / LandingZone / DroneShip; replaces the request's
  landing site, part of the memo key), then plans the kick. By the first stage's `recovery_method`:
  - `tower_catch` (Super Heavy): the site's tower; none -> "Kein Fangturm am Startplatz."; no plan with the catch ->
    "Ziel mit Booster-Rückkehr nicht erreichbar (<trial reason>)". Never expendable.
  - `leg_landing` (Falcon 9): the site's `landing_zone` (launch_sites.toml; SLC-40 -> LZ-40 [S]) when the plan with the
    RTLS return (its MECO plan keeps the 12.1 t landing propellant) is feasible - also where the drone ship would leave
    more (SpaceX practice, spec) - else a drone ship from `config/drone_ships.toml` at the stage's impact predicted 10 s after the
    separation (`BoosterReturnGuidance::predict_impact`, drag and the entry burn, no steering: the falcon9_asds.toml
    derivation), deck along the ground track there (in a moving sea: the heading of least deck tilt, below); its sea the EGM2008 geoid at the station (`FlightSession::sea_height_m`, coarse grid -43.2 m at 32 N 76 W; M8 Plan A Task 5, was -35 m [E]). The kick is searched with the ship at
    a provisional station 600 km [E] downrange (the MECO plan without a boostback does not depend on the ship; near the
    pad its deck would replace the ground). Neither feasible: refused.
  - Drone ship list (follow-ups Task 6): `asds_atlantic`, `asds_atlantic_2` (home port Port Canaveral [S]) and
    `asds_pacific` (Long Beach [S]), each with a service area [E] (a lat/lon box). `config::drone_ship_for` picks the
    ship whose area holds the point (several: nearest home port, file order on a tie); `plan_launch` asks it for the
    provisional station (none: refused "Kein Drohnenschiff für dieses Seegebiet.", before any trial) and again at the
    predicted impact (the recovery's ship id; the memo key carries the provisional one). Cape launches still get
    `asds_atlantic`. The old fixed `default_drone_ship_id` is gone.
  - Limits (final review): the drone ship's station is not checked to lie at sea - there is no land mask until the
    M8 terrain (an impact predicted over land would place the deck there). Without a feasible trial the search probes
    the middle of the widest kick gap left and gives up once every gap is below 2.5 deg (`initial_step_rad`): a launch
    whose feasible kick band is narrower than that and falls between the samples can be refused - conservative (never
    a launch that cannot fly).
  - Sea state: a mission's `[sea_state]` can refuse the drone ship (`sim::sea_state_refusal`, checked on the
    provisional ship before the drone-ship trials and again on the chosen one) and turns the deck to its least-tilt
    heading - see "Sea state and deck motion".
- Recovery feasibility is checked for real in every trial (Task 3 review note): the session must return the stage
  (`returns_after_staging`), the MECO plan be finite, and with a boostback its Lambert arc be found from the stack's
  state at the last Booster tick (the trial single-ticks within two checks' propellant of the cutoff):
  `BoosterReturnGuidance::boostback_delta_v_mps` (split out of `meco_propellant_kg`, which counted a failed arc as no
  boostback - finite, but not flyable). What the reserve holds is the MECO plan's: the stage's `landing_propellant_kg`
  plus run-down, flip and the boostback by the rocket equation on the Lambert arc with `boostback_loss_fraction` on
  top - no return is flown in the trials. The Lambert-arc check is weak (an arc is found for almost every MECO state);
  a return that does not fit shows as an early MECO (more propellant kept) and the trial then fails the orbit check.
- Measured (parallel test load): Falcon 9 CRS-35 class (12.5 t, 200 km / 51.6 deg, SLC-40) -> LZ-40, 11.57 deg, 2.2 t,
  4 trials, 11.7 s. Starlink class (16.5 t, 200 x 280 km / 53 deg, SLC-40): RTLS infeasible -> drone ship at 33.1419 N
  / 75.8444 W, heading 44.8 deg, kick 13.93 deg, 1.0 t, 15 trials, 35.6 s; the configured falcon9_asds station
  (33.2240 / -75.7724, flown with the configured 13 deg kick) lies ~11 km away. The CRS-35 request from LC-39A (no
  landing zone): drone ship at 33.2590 / -75.4291, 14.94 deg, 7.0 t, 5 trials. Starship 2000 km circular: refused after
  10 trials (23.6 s). Starship from LC-39A to 26.5 deg: flown at 28.608 deg, 14.82 deg, 88.1 t.

### Every launch through the planner (Task 5)
- **Missions and chains:** `sim::start_ascent_mission` builds the planner request from the mission (vehicle, site,
  heading, target, payload) and calls `sim::engage_planned_ascent`: `plan_launch` with the session itself as `like` (on
  the simulation thread), then the chosen landing zone / drone ship, the booster's return on and the ascent autopilot
  with the target flown and the planned kick. A refused plan leaves the stack on the pad with the autopilot off and calls
  off a scheduled liftoff; `start_mission`, the ISS launch, depot launch, tanker / HLS flights and the flown campaign's
  pad launches return its German reason as their refusal. A study's changed vehicle (`start_ascent_mission(..., &vehicle)`)
  is planned as it stands (`AscentProfileRequest::stack`, never memoised: its id names the unchanged file).
- **Mission files:** `[pitch_program]`, `kick_deg`, `booster_throttle`, `landing_zone` and `[drone_ship]` are gone from
  every ascent mission and the loader refuses them (`AscentMission` lost `kick_angle_rad`, `landing_zone_id`,
  `drone_ship`). History of the removed values: starship_leo / depot / tanker / HLS ascent kick 14 deg [D] (the sweep
  "Starship pitch kick sweep": altitude at T+60/90/120 s within ~1 km of flights 13/14, 9.1 / 21.3 / 38.4 km vs
  8.4 / 20.5 / 38.2, MECO T+135 s 48 km; the tanker's "kick, throttle sweeps gave no gain" was a sweep on the old
  throttle schedule); falcon9_rtls 12 deg [E] (CRS-35 MECO T+137 s / 56.4 km vs 137 s / 56.6 km), falcon9_crew_dragon
  12 deg [E] (as RTLS); falcon9_asds 13 deg [D] (MECO path 24.7 deg, T+152 s, 64.7 km, 8373 km/h vs Starlink 15-25
  T+149 s, 60 km, ~8000 km/h, ~25 deg) and its drone ship at 33.2240 / -75.7724 [D] (the stage's impact 10 s after the
  separation; history 32.9272 / -76.0990 before the staging sequence, 32.8819 / -76.1490 with it), deck heading 45 deg
  [E], sea on the EGM2008 geoid (M8 Plan A Task 5, was -35 m [E]); landing_zone = ccsfs_lz40 on falcon9_rtls and falcon9_crew_dragon (now SLC-40's own landing zone).
- **Free launch:** the game's `launch_from_site` carries the target (`leo_ascent.toml`) unless manual control is on;
  the command places the stack, schedules T-0, publishes a snapshot with `autopilot_status()["planning"] = true` and
  plans (`sim::engage_pad_ascent(session, target)` on a clamped stack: the request from the placed stack - the
  vehicle file by id with its own payload, fairing kept, so it is memoised - or the loaded configuration as `stack`).
  While the command runs the session does not advance: the world clock holds at the pad snapshot's epoch, the T- clock
  stands ("Startvorbereitung: Bahn wird berechnet" as last_event and in the webcast's name line); the liftoff follows
  the plan, never the default `PitchProgram` (test "A free launch holds on the pad until the plan is in"). Refused:
  the command is not ok with the reason (game: "Start abgelehnt: Kein Fangturm am Startplatz."), the vehicle stays on
  its pad. Key P on a clamped stack plans the same way. `engage_ascent_autopilot(target, program)` flies a given
  program (trial flights, studies, guidance tests); re-engaged during a staging sequence of the controlled body
  (joined, or separated before the upper ignition) it keeps the state Staging (final review: it set Booster, and the
  joined stack would have cut off again). `FlightSim.start_mission` of a mission that launches from a pad (ascent,
  iss_launch, depot_launch, tanker_flight, hls_refuel, the flown campaign) publishes the planning flag before
  `sim::start_mission` (placement and planning are one call; the flag is in `autopilot_status()` also while the
  session holds no flight yet) and clears it afterwards; the flag is a scope guard (cleared on a throw too).
- **Orbit check of elliptic targets:** the trials judged every target on the radius flown over the next revolution;
  PEG inserts J2-aware only into circular targets and aims the osculating apsides otherwise, so the tankers'
  120 x 190 km reached 120 x 180 km flown and every trial was refused. An elliptic target is now judged on the
  osculating apsides at the end of the tail-off (the planner's `circular_target` is the guidance's own rule).
- **Planned kicks** (memo per process, measured 2026-10-23): starship_leo 14.85 deg (88.5 t at SECO, as Task 3);
  tanker_ascent 18.07 deg - 145.5 t at SECO against ~130 t at the old 14 deg: the 120 km perigee favours a flatter
  arc than the webcast-matched profile; the Falcon 9 missions as Task 4 (RTLS 11.57 deg; ASDS 13.93 deg with the ship
  at 33.1419 / -75.8444).
- **Re-baselined:**
  - Mission timeline (`A Starship ascent records ...`): during the vertical rise and the kick the return's MECO plan
    from the near-vertical state swings (+10 .. +96 s at the planned 14.85 deg; at 14 deg it stayed within 9 s by
    chance); the whole-flight bound of 9 s now starts at the kick's end (T+30 s: +5.6 s, within 120 s before MECO the
    swing ends ~T+16 s). Final review: the forecast is not shown before the kick's end (first seen 109.2 s before MECO,
    +5.19 s; at most 6.58 s off over every shown sample, 2.21 s within 10 s; T+30 s +5.59 s).
  - Windows: tanker_flight `insertion_s` 436.1 -> 432.2 s, `insertion_arc_deg` 11.93 -> 12.25; hls_refuel 421.4 ->
    415.7 s, 11.37 -> 11.78 deg (the flown ascents with their planned kicks).
  - Reference flights (`reference_flights.toml`, re-measured by the slow r5 flights): depot fill 69.6 -> 70.7 t;
    tanker residue 93.0 -> 105.6 t, rendezvous 15.49 -> 17.37 t, delivered LOX / CH4 50.9 / 12.7 -> 59.4 / 14.85 t,
    settling gas 0.90 -> 0.98 t, main at catch 1.08 -> 1.21 t; HLS main at docking 182.4 -> 202.8 t;
    `inputs_digest` afc1cbc547f04fbd.
  - Campaign offsets (the flown campaign against the calculator's model of the flown window, flights whose class
    both agree on): above 1.6 -> 2.0 t (4 flights -2.6..+2.1 t), below -0.5 -> 1.1 t (1 flight +1.6 t) [D].
  - Calculator (default): 14 tankers / 52.3 d -> **12 tankers / 50.6 d**.
  - Flown campaign, placed (`refuelling_report --fly-placed`): before 2 tankers, 8.6 d (HLS at the depot 8.59 d with
    1053.0 t, first return caught with 1.27 t); after 2 tankers, 8.6 d, comparison passed (deliveries 73.8 / 75.5 t
    against 60.5 / 63.3 t, HLS at the depot 8.62 d with 1062.7 t, first return caught with 2.57 t).
  - Full flown campaign (ctest `Flown campaign: the depot launched*`): 12 tankers flown, 52.9 d, every tanker docked,
    HLS at the depot 52.94 d with 1079.7 t (needs 1041.4 t), first return caught (2.58 t); first plan 12 tankers /
    44.7 d. Against the first plan's k-th (the comparison until now) the fill missed at tankers 5 and 8 (420.5 vs
    410.5 t, +2.4 %; 628.7 vs 616.3 t, +2.0 %): the k-th pairs drift apart in time once the first windows cascade
    (first plan 44.7 d, flown 52.9 d), so 7 of 12 flew the other phasing class than their pair (tankers 5, 6, 8-12).
    Ruling 2026-10-23: the fill against the calculator's model of each flown window - every tanker within 0.5 %
    (tanker 5: 420.5 vs 420.6 t, tanker 8: 628.7 vs 629.6 t, largest tanker 12: 907.6 vs 909.0 t); the class mix
    asserted instead of the pairwise flips: first plan 8 of 12 above the depot, flown 7 of 12 (difference 1, at most 3);
    7 pairwise flips reported. Comparison passed.
- **Test time:** every test process plans its own launches (the memo lives per process; the game has no on-disk cache - the spec
  wants planning at launch): 60 cases of the affected labels grew by ~28 s each (+1700 s CPU in all under the 24-way
  parallel load; a Starship plan 4-6 trials, ~4 s each serial, ~20-30 s under load; the Falcon 9 drone-ship request 15
  trials, ~45 s). Cases that only test pad mechanics through `start_mission` pay it too.
  Follow-ups Task 5: the test processes share their plans through a write-once disk cache
  (`sim/ascent_plan_cache.hpp`; only when `STARSIM_PLAN_CACHE_DIR` is set - the test CMake sets it to
  `build/plan_cache`, the game never does). Disk key = memo key + liftoff epoch (hexfloat) + FNV-1a hash of the config
  directory's contents; the file is written to a temp name and renamed into place; the directory is wiped on every
  relink of the test exe. The epoch is in the disk key although the memo leaves it out: a plan made by another process
  at another hour differs in the kick's last bits, and the tanker flight's rendezvous then moved by 1.6 t (17.37 ->
  18.97 t, the reference-flight check failed) and the kink test's `start_ascent_mission` kick no longer equalled its
  plan's - with the epoch a hit is bit-identical to the plan the process would have made itself (RTLS mission with
  `STARSIM_VERBOSE=1`: all 76 output lines equal with and without the cache). Tests that assert trial flights call
  `sim::disable_plan_cache()`; `plan_speed` runs without the cache. Measured 2026-10-23, `ctest -j 24` on the labels
  ascent / ascent_planner / staging / falcon9 / rtls / asds (69 cases, all passed in each run): no cache 174 s wall /
  1895 s summed per-test time; empty cache 171 s / 1772 s; warm cache 133 s / 973 s (-49 %). The wall time is bound by
  the tanker flight (133-174 s alone); the warm cache saves only on a rerun without relinking.

### Validation against flight 14 and CRS-35 (Task 6)
- **Missions** (`config/missions/flight14_ascent.toml`, `crs35_ascent.toml`): each recorded flight's target, payload
  and propellant load, flown through `sim::start_ascent_mission` - kick, recovery and MECO planned, nothing fitted.
  Tests "Starship ascent to flight 14's insertion orbit against its webcast telemetry" (`test_starship_ascent.cpp`) and
  "Falcon 9 ascent to CRS-35's orbit against its webcast telemetry" (`test_falcon9_ascent.cpp`; helpers
  `core/tests/ascent_validation.*`, `STARSIM_ASCENT_DUMP=<dir>` writes the flown samples). Every recorded sample is
  asserted (T+30 / 60 / 90 / 120 s speed and altitude, MECO time / speed / altitude / path angle, SECO time): bound =
  reading tolerance (speed 1 km/h, altitude 0.05 km, +-0.25 s per reading and +-0.25 s time base, times the local
  rate) + a named allowance per documented open discrepancy (measured + ~2 % [E]; 0 where the model follows). Final
  review: the T+30 / 60 / 90 / 120 s samples carry one allowance each (`test::SampleGap`: the measured gap +
  `test::allowance_margin` 2 % [E]) instead of one band per phase, so each sample gates on its own gap. The MECO
  reading tolerance is the local rate x 2 x the MECO's uncertainty on flight 14 (its last reading lies at the lower
  end of the MECO interval, the stack may accelerate over the whole interval after it) and x 1 on CRS-35 (read at the
  interval's centre).
- **New mission key `[propellant]`** (`<stage id> = t`, `AscentMission::propellant_load_kg`): loaded before the engines
  start (`config::fill_tanks`; an unknown stage or more than the tanks hold is refused by `start_ascent_mission`); the
  request is then planned as it stands (not memoised). Flight 14: booster 3,533.2 t [D] (gauge 96.8 % of 3,650 t from
  T-6 to T+5 s display) - 5,650 t on the pad, 5,602 t at the release (mass_estimate.py: 5,542-5,603 t; full tanks and
  no payload 5,715 t). Payload 52 t [E] (26 Starlink V3 at ~2 t, the ship dry-mass derivation's figure). CRS-35: full
  (the gauge extrapolates to 100 % at the release), 12.5 t [E].
- **Flight 14's target is its insertion orbit, not 262 x 275 km.** The ship's altitude after SECO (Kepler fit of 620
  readings T+515-1498 s, rms 0.3 km) gives **-44 x 276 km**: SECO at 165 km still climbing into a transatmospheric
  orbit, the perigee raised near the apogee at T+1520-1640 s (speed +89 m/s) to 262 x 277 km (fit T+1700-2600 s, rms
  0.2 km; flights.toml: 262 x 275 km). Tried: 262 x 275 km with the 52 t payload and the catch - refused by the planner
  (ship dry before cutoff; with full booster tanks too; with no payload 64 t left); -44 x 276 km - PEG inserts at the
  periapsis, the trials crash (a transatmospheric target would need a guidance change, a non-goal). Flown: **165 x 276
  km, 30.5 deg** [D] (the recorded apoapsis, the periapsis at the recorded SECO altitude; ~105 m/s more than flight 14's
  insertion). Measured: kick 16.33 deg, 37.2 t left in the ship, 165.0 x 276.7 km.
- **(1) Time base - the cause of the early "+9-12 %".** The dials show the state late against the release (our T+0):
  the speed dial's early rise (8-40 m/s, drag-free) extrapolates to zero at T+3.50 s (flight 14), 3.24 (flight 13),
  3.42 (CRS-35), 3.26 s (Starlink 15-25); before that it creeps from 0 at T+2-3 s (smoothing). The altitude dial lags
  the speed dial by another 1.95 / 2.00 / 2.45 / 2.05 s (the displayed speed integrated through the vertical rise matches
  the 0.1 km readings within 0-18 m) - the "speed and altitude graphics offset by ~1.5-2 s" of the descent analysis.
  New tool `tools/telemetry/liftoff_time.py <table> [--simulated <dump>]` (also the model-based shift: 3.53 s for
  flight 14). Calibrated [D] per table in the tests: speed 3.50 / 3.42 s, altitude 5.45 / 5.87 s, +-0.25 s. Without
  them Starship ran +12 % fast at T+30 s (a 3.5 s shift at ~20 km/h/s); with them -0.9 %. CRS-35's gauge confirms it
  independently: 98.8 % at T+5.25 s falling ~0.5-0.65 %/s puts 100 % at ~T+3 s display.
- **(2) Liftoff mass.** Starship with flight 14's load (above): the early acceleration matches (speed slope 4.73 /
  5.85 / 7.74 m/s^2 simulated against 4.91 / 5.86 / 7.68 recorded at T+10 / 25 / 40 s display); the old check's full
  tanks without payload made the stack ~1 % heavier on the pad. CRS-35 (full, 570.3 t): see (3).
- **(3) Thrust per mass flow.** Starship: no evidence of an error up to max-Q (above). Falcon 9: the recording
  accelerates ~0.5 m/s^2 harder before max-Q (4.42 against 3.95 m/s^2 at 8-40 m/s; G dial 1.5 against 1.40-1.48 g at
  T+10-20 s), while after max-Q the G dial times the gauge's mass (425 t capacity, 145.3 t above it) gives 0.98-1.00 of
  our thrust (T+100-135 s, G resolution +-1.5 %) and 1.03-1.07 before max-Q. A thrust or Isp scale moves both alike; a
  lighter liftoff (e.g. the commonly cited 395.7 t load) needs a mass flow ~10 % below the rated flow at the rated
  thrust (Isp ~315 s at sea level, beyond the Merlin's 282 s) to keep the late agreement - not separable, not
  calibrated. Our vacuum Merlin is 932 kN (from Isp 311 s and the 845 kN rating) against SpaceX's 914 kN [S: F9 page
  8,227 kN for nine] - 2 %, inside the G dial's resolution, kept. **Replaced in follow-ups Task 7c (user 2026-10-23):
  914 kN at the rated flow (Isp_vac 305 s), section "Falcon 9 late ascent against CRS-35".**
- **(4) Ascent drag.** Not separable: before max-Q the drag is < 0.3 m/s^2 (Cd 0.5 at q < 10 kPa), and through max-Q
  the thrust is throttled to an unknown level (the gauge's flow over +-4 s: ~+-3 %; the G dial's 0.05 g is Cd +-0.6 at
  30 kPa on Falcon 9). The ascent tables (`config/aero/*_ascent.csv`) stay [E]. No other drag cause is needed: Starship's
  deviation after max-Q is 2.5 m/s^2 at q 37 kPa (T+60 s display) - Cd ~4.5 if it were drag - a throttle difference.
- **Remaining discrepancies (open, asserted with their allowances):**
  - Starship after max-Q: flight 14 throttles to 0.78-0.90 of the rated flow from Mach 1.39 to MECO
    (`peak_dynamic_pressure.py --capacity-t 3650 --rated-flow-tps 28.0`) while our q limit never bites (our q peaks at
    38.7 kPa under the 38.9 kPa limit, so the stack keeps 0.88), its acceleration levels at ~20.4 m/s^2 from T+110 s
    while the mass falls 16 % (a throttle-down; not sourced, spec: no late-boost limit without a sourced g limit), and
    one centre engine is out from T+76 s (not modelled). T+60 / 90 / 120 s +1.7 / +6.9 / +9.0 % fast, +0.6 / +7.4 /
    +11.2 % high. MECO from the catch plan (294 t in the booster; flight 14 ~500 t by its gauge, not followed after
    staging): T+129.8 vs 136.0 s, 6559 vs 5943 km/h (+10.4 %), 52.5 vs 51.0 km, path 30.3 vs 27.8 deg. SECO T+447 vs
    ~503 s (-11 %): flight 14's RVac out from T+438 s, its slower MECO and its lower insertion.
  - Falcon 9 before max-Q: -9.5 % speed / -8.5 % altitude at T+30 s ((3)). After max-Q CRS-35 throttles earlier and
    deeper (G 1.4-1.6 at T+50-60 s; ours holds the transonic q from Mach 0.9 at >= 0.80): +2.3 / +4.6 / +4.2 % at
    T+60 / 90 / 120 s, altitude -8.2 / -2.1 / +2.6 %. MECO T+132.7 vs 134.6 s, 6596 vs 6490 km/h (+1.6 %), 58.2 vs
    57.4 km, path 41.3 vs 36.6 deg (the planned 11.6 deg kick flies steeper than CRS-35); SECO T+508.0 vs ~511.6 s
    (CRS-35's SECO lies in a reading gap: the rocket-equation fit of T+430-496 s reaches the final 27,012 km/h at
    T+515 +- 3 s display). LZ-40 chosen (CRS-35 landed there). These CRS-35 numbers are Task 6's; superseded by the
    follow-ups Task 7c (section "Falcon 9 late ascent against CRS-35") and M8 Plan A Task 7: now T+30 / 60 / 90 /
    120 s speed -9.8 / +1.9 / +3.1 / +2.6 %, altitude -7.2 / -8.7 / -3.4 / +0.1 %; MECO T+135.5 vs 134.6 s, 6830 vs
    6490 km/h (+5.2 %), 59.5 vs 57.4 km, path 39.1 vs 36.6 deg; SECO T+509.0 s.
- **M8 Plan A Task 7:** the samples' altitude is above the pad (was the ellipsoidal height; the derived site heights
  would have put it 21-23 m low) - numbers in "Earth terrain and geoid".
- **No re-baselines in Task 6:** no physics changed (the time base is the comparison's, the propellant load a mission
  input); the reference missions (starship_leo, the Falcon 9 missions, ISS, refuelling) flew as before. Later
  re-baselines: follow-ups Tasks 7b / 7c and M8 Plan A Task 7 (sections below and "Earth terrain and geoid").
- **Removed:** the old telemetry tests (starship_leo with the recorded 500 t MECO and the planned kick; CRS-35 on a
  fixed 12 deg kick, T+60 / 120 s within 15 % only) and the kick sweeps "[.tuning]" (kicks are planned).

### Falcon 9 boostback against CRS-35 (follow-ups Task 7b)
Question (Task 7): with the flown 13.6 s flip the planner keeps 53.6 t at MECO (gauge ~66 t), MECO comes 7.6 % fast,
and our boostback burns 43 s against CRS-35's ~55 s - does the boostback cost too little? **Answer: no.** Diagnosis
only, nothing changed in the model.
- **Method.** `tools/telemetry/boostback_fit.py`: a planar point mass (spherical Earth, no drag - q < 20 Pa above 80 km)
  with a free initial state and a piecewise-linear thrust acceleration (knots every 3-4 s) fitted to the speed and
  altitude readings (time bases 3.42 / 5.87 s, section "Validation against flight 14 and CRS-35"). The propellant of a
  burn follows from its delta-v and its end mass alone; the end mass is bounded below by the gauge after the burn
  (3.5 % = 14.9 t at the entry burn + 25.6 t dry with the N2 tanks = 40.8 t) and above by the thrust ceiling (in the
  steady phase, where 1/a falls by 1/c per second as for a constant thrust, the mass is at most 2796 kN / a; three
  Merlins in vacuum, our engine model). Checked on our own flight (hidden test "Falcon 9 CRS-35 first stage profile
  dump", `STARSIM_BOOSTBACK_DUMP=<csv>`): fitted 2101 m/s, end mass 39.1 t, start 77.9 t, 38.8 t burned against the
  flown 2100 m/s, 39.2 / 78.0 / 38.8 t. Fits on CRS-35: knots 3 s rms 0.64 m/s / 0.23 km, 2014 m/s; knots 5 s 2019
  m/s; constant-thrust model (worse fit, rms 6 m/s) 2026-2068 m/s.
- **Phase by phase** (CRS-35 in true time = display - 3.42 s; ours: `crs35_ascent`, the dump above):

  | Phase | CRS-35 | Ours |
  |---|---|---|
  | MECO | T+134.6 s, 6490 km/h, 57.4 km, path 36.6 deg; gauge 16.3 % = 69 t (display T+136.25 s) | T+135.5 s, 6985 km/h, 59.7 km, 38.2 deg; 53.6 t |
  | Separation | T+138 s (timeline 2:18) | T+140.0 s, 52.8 t, 62 km from LZ-40 |
  | State at T+143 s (fit) | 68.7 km, vertical 1016 m/s, horizontal 1461 m/s | 71.3 km, 1126 m/s, 1541 m/s |
  | Boostback ignition | ~T+151-152 s (timeline 2:31) | T+153.8 s (flip 13.6 s) |
  | Boostback thrust | ~T+152-170 s soft start: 24-31 m/s2 at 75-85 t = ~1.8-2.3 MN; T+172-202 s steady: 38.5 -> 53 m/s2, growing as a constant thrust (1/c within ~2 %) of 2.57-2.80 MN = 0.92-1.00 of three Merlins (two give at most 1.86 MN: three engines); ~T+202-207 s thrust-down | 2.80 MN (three Merlins, full) from ignition to cutoff, no trim |
  | Boostback duration | ~55 s (T+151-206.5 s) | 43.2 s (T+153.8-197.0 s) |
  | Thrust delta-v | 2014 m/s (2014-2068 over the fits), tilted 1.5 -> 5.6 deg down | 2100 m/s, tilted 5.8 deg down |
  | Mass start -> end, burned | 79.0-85.8 t -> 40.8-44.3 t, 38.2-41.4 t (end mass at the gauge / at the thrust ceiling) | 78.0 -> 39.2 t, 38.8 t |
  | Propellant at boostback ignition | 53.7-60.5 t (start mass - 25.0 t structure - ~0.3 t N2) | 52.8 t |
  | End of burn | 616-621 m/s at 112 km (T+206.5 s); ~555 m/s horizontal back; apogee ~116.4 km at ~T+237 s | 677 m/s at 114.9 km (T+197 s; 633 m/s at T+206.5 s); 555 m/s back; apogee 122.9 km at T+239 s |
  | Entry burn (display times) | T+355 s, 56.6 -> 35.9 km, 125 m/s, gauge 3.5 -> 3.1 % (~1.7 t) | T+356.5 s, 57.8 -> 38.7 km, 2.0 t |
  | Landing burn (display times) | T+420 s, ~3.2 km, ~300 m/s, 12.1 t; touchdown T+442 s | T+423.5 s, 2.94 km, 287 m/s, 12.06 t; touchdown T+453.5 s |

- **Findings.**
  - The flight's boostback needs **less** delta-v than ours (2014 against 2100 m/s): its MECO is slower (~75 m/s less
    horizontal speed at T+143 s), and both end with ~555 m/s back. The aim (Lambert arc, `return_flight_time_s` 260 s)
    and the end state agree; aero during the burn is negligible on both (q < 20 Pa); a slower burn adds no gravity loss
    to a horizontal burn (the vertical speeds follow gravity plus the small downward thrust share on both).
  - Its 55 s are a **thrust profile, not a cost**: ~18 s of soft start at ~2/3 thrust and ~5 s of trim at the end;
    the steady part is three engines at 0.92-1.00. The propellant is set by delta-v and end mass: 38.2-41.4 t against
    our 38.8 t - at most 2.6 t more than ours (end mass at the thrust ceiling), as little as ours at the gauge's end mass.
    `boostback_loss_fraction` 0.13 stays (it is identified on the landing-burn propellant, which matches).
  - The **gauge's 66-69 t at MECO is out of reach**: the thrust ceiling bounds the propellant at boostback ignition
    to <= 60.5 t (MECO <= ~61 t with the run-down) - the same pattern as Starlink 15-25 (47 t at MECO -> 25.9 t at the
    entry burn without a burn). Not followed (physics first).
  - The **MECO speed gap is the ascent's**: at equal true time the flight is slower before MECO (T+130.8 s: 6111
    against our 6409 km/h, +4.9 %; T+120 s +4.2 %, Task 6), and our stack accelerates harder near MECO (along the path
    T+125.8-130.8 s: 35.7 m/s2 sensed for the flight - 107.8 km/h/s + g sin(path) - against our 37.8 m/s2: with our
    thrust a stack ~13 t (6 %) heavier, or ~6 % less thrust; the G dial reads 2-3 % below ours at T+129.5-130.8 s). MECO itself comes 0.9 s
    later than recorded (+1.7 % speed). Before Task 7 the 50 s flip made MECO 1.9 s early (T+132.7 s), which hid the
    ascent's excess (+1.6 % at MECO). Even the bracket's upper end (2.6 t more at MECO) would move MECO ~1 s earlier,
    ~1.8 % of the 7.6 %.
- **Open (user decision 2026-10-23, Task 7b gate):** the ascent's late acceleration excess (Task 6 open item (3):
  thrust / mass flow against liftoff and upper-stack mass, not separable from the dials alone). New evidence for it: the
  boostback's thrust ceiling bounds the first stage's propellant at MECO to 54-61 t (ours 53.6 t), so a heavier stack
  near MECO must come from the upper stack or the thrust, not from the first stage's propellant. The CRS-35
  `meco_speed_allowance` stays 0.097 (INTERIM) until then. Diagnosed and resolved in Task 7c (next section).

### Falcon 9 late ascent against CRS-35 (follow-ups Task 7c)
Question (Task 7b): near MECO our stack accelerates ~6 % harder than CRS-35 - a heavier real stack, or less thrust?
Diagnosis first (gate: the candidate fix chooses between two contradicting sourced values); the user chose (A) - see
"Resolution" at the end of this section.
- **Method.** `tools/telemetry/late_ascent_fit.py`: the sensed acceleration along the path from the dials, a = dv/dt +
  g(h) sin(gamma) (least-squares slopes over +-2 s speed / +-3 s altitude, time bases 3.42 / 5.87 s), at equal true
  time against our flight (`STARSIM_ASCENT_DUMP`), with the same estimator run on our dump (bias against our own F / m:
  level x0.998-1.004). With constant thrust and flow 1/a = m / F falls by 1/c per second, so a line through 1/a gives
  m / F (level) and c (slope). Drag is < 0.5 % of the thrust above 30 km on both.
- **Level.** From T+90 to T+124 s the flight's m / F is x1.035-1.041 ours over every window (90-110, 100-124, 110-130 s):
  with our thrust a stack ~10 t heavier, or ~3.5-4 % less thrust. The ratio is flat over that span; it rises only in
  the last ~8 s (1.05 at T+126 s, 1.08 at T+132 s) - see "Last seconds". The slope (c) is not usable: the dials'
  noise moves it from 2720 to 3230 m/s between windows (Isp 278-329 s).
- **(c) First stage - flow and mass.** The propellant gauge falls 0.646-0.660 %/s in every window from T+70 s to its
  last reading (display T+136.25 s) = 2.75-2.80 t/s on 425 t: the rated flow of nine Merlins, no throttle-down before
  MECO. At equal true time the gauge shows 8.5-10.6 t more than our first stage from T+20 s to MECO (3.4 t of it our
  start-up burn before the release, which the gauge counts as 100 %); the offset is flat through max-Q (the deeper real
  throttle bucket saves at most ~1-2 t) - but the boostback's thrust ceiling allows at most ~61 t at MECO (Task 7b), i.e.
  at most ~3-5 t above our ~56 t at the flight's MECO time. A first-stage mass excess is bounded to a few tonnes.
- **(a) Upper stack - sources.** Second stage: Falcon 9 FT tank capacities 75.2 t LOX + 32.3 t RP-1 = 107.5 t, empty
  4.0 t [S: en.wikipedia.org/wiki/Falcon_9_Full_Thrust, "Rocket specifications", citing Espace & Exploration No. 39,
  2017] - ours 106.7 t [D] + 3.9 t: the derived value matches the source within 0.9 t. Dragon: CRS-35 cargo 2,860 kg
  (6,300 lb) [S: space.com, CRS-35 launch report 2026-10-13]; Dragon 2 dry mass 7,700 kg and propellant 2,562 kg [S:
  en.wikipedia.org/wiki/SpaceX_Dragon_2 - the crew capsule; cargo-Dragon and trunk masses are not published] - 13.1 t
  plus the trunk against our 12.5 t [E]. A heavier upper stack cannot be the main cause: in our model CRS-35's RTLS
  closes only with <= ~14 t (payload 17.5 t: the planner chooses the drone ship, MECO T+146 s; 22.5 t: infeasible).
- **(b) First-stage thrust - sources.** SpaceX's Falcon 9 page (via en.wikipedia.org/wiki/Falcon_9_Full_Thrust, ref.
  [3]): nine Merlins 7,607 kN at sea level, 8,227 kN in vacuum = 845 / 914 kN per engine; the same page gives Isp 282 /
  311 s. The two pairs contradict each other: 845 kN / 282 s is a flow of 305.6 kg/s, 914 kN / 311 s of 299.7 kg/s.
  Ours keeps the sea-level pair (845 kN, 282 s - the gauge confirms the flow) and the 311 s (nozzle exit 1.045 m), so
  our vacuum thrust is 932 kN, 2.0 % above SpaceX's 914 kN. With 914 kN at the rated flow (Isp_vac 305 s; nozzle exit
  0.932 m; the sensitivity runs below used 0.931 m, 0.2 kN less) the flight's m / F over T+90-124 s is x1.008-1.026
  ours (mean ~1.018: +5 t or -1.8 %, at the dials' resolution) - the vacuum thrust explains about half of the level, the
  dials leave the rest unresolved.
- **Last seconds (T+~126 s to MECO).** The flight's acceleration levels off: the G dial holds 3.8-3.9 g (display
  T+132.75-135.25 s) and falls to 3.6 g at T+136.25 s with nine engines lit and the gauge at the rated flow; Starlink
  15-25 holds 3.8-3.9 g over its last 7 s before MECO (display T+141.25-147.75 s) while its mass falls ~8 %. Ours rises
  to 4.2 g at MECO (4.16 g with 914 kN). This looks like an acceleration limit near ~3.9 g (no source found: no
  published Falcon 9 g limit; the gauge shows no flow reduction, so it may be a dial or guidance effect). Worth only
  ~0.35 % of MECO speed (~0.13 g over ~5 s).
- **Sensitivity runs** (the CRS-35 test with a temporary config change, reverted; MECO speed against 6490 km/h):

  | Case | Kick | MECO | T+90 / T+120 s speed | MECO speed | First stage at MECO | Second stage at SECO |
  |---|---|---|---|---|---|---|
  | as is (932 kN, 12.5 t) | 12.27 deg | T+135.5 s | +4.8 / +4.8 % | 6985 km/h, +7.6 % | 53.6 t | 3.0 t |
  | 914 kN | 11.80 deg | T+135.5 s | +3.1 / +2.6 % | 6830 km/h, +5.2 % | 53.0 t | 2.7 t |
  | 914 kN, payload 13.1 t | 11.75 deg | T+135.5 s | +2.9 / +2.3 % | 6811 km/h, +4.9 % | 52.9 t | 2.1 t |
  | 932 kN, payload 14.0 t | 11.98 deg | T+135.7 s | +4.1 / +4.0 % | 6957 km/h, +7.2 % | 52.7 t | 1.6 t |
  | 914 kN, payload 14.0 t | 11.62 deg | T+135.7 s | +2.4 / +1.8 % | 6809 km/h, +4.9 % | 52.1 t | 1.3 t |
  | 932 kN, payload 17.5 t | 14.12 deg | T+146.0 s (drone ship) | +3.4 / +4.6 % | +30 % | 25.6 t | 2.2 t |

- **Budget of the remaining +4.9-5.2 % (~95 m/s) with 914 kN:** the max-Q bucket (+2.9-3.1 % already at T+90 s, ~21
  m/s; Task 6 open item: CRS-35 throttles earlier and deeper), T+90-124 s ~1-2 % m / F (~20 m/s), the last seconds'
  levelling (~15 m/s by the ratio, ~6 m/s by the G dial), and MECO 0.9 s later than recorded (~35 m/s: the planner's
  MECO from the return reserve; the flight keeps up to ~61 t, ours 53 t).
- **Decision needed (gate):** (A) take SpaceX's 914 kN vacuum thrust at the rated flow (Merlin 1D Isp_vac 311 -> 305 s
  [D], nozzle exit 1.045 -> 0.932 m) - sourced, but it drops the equally sourced 311 s; it moves every Falcon 9 return
  (boostback/entry/landing propellant, `boostback_loss_fraction`, ISS window calibration) and reaches ~5 %, not 3 %;
  (B) an ascent acceleration limit ~3.9 g (guidance, unsourced); (C) keep 932 kN and the INTERIM allowance.
- **Resolution (user 2026-10-23: (A), no g limit).** `config/engines/merlin1d.toml`: vacuum thrust 914 kN [S: SpaceX
  Falcon 9 page, 8,227 kN / 9] at the unchanged rated flow (845 kN / 282 s at sea level = 305.6 kg/s, pinned by the burn
  time and both flights' gauges): `nozzle_exit_diameter_m` 1.045 -> 0.932 [D] (exit area 0.682 m2), Isp_vac 311 ->
  305.1 s [D]. The 311 s on the same SpaceX page contradicts its thrusts at one flow and is dropped (the Merlin page
  flags it "needs update", a 2012 value). Only the Merlin 1D changes: the second stage's Merlin Vacuum (981 kN [S:wiki_merlin]; the
  old SpaceX page lists 934 kN for the second stage - a separate question, not part of this decision) is not covered
  by the 8,227 kN. Comments derived from 311 s updated (entry burn 1.7 t: 131 -> 128 m/s; drone-ship gauge rocket
  equation 950 -> 933 m/s).
  - CRS-35 (`test_falcon9_ascent`): kick 12.27 -> 11.80 deg, MECO T+135.5 s (unchanged) with 53.6 -> 53.0 t, MECO speed
    6985 -> 6831 km/h (+7.6 -> +5.3 %), altitude 59.7 -> 59.5 km (+4.0 -> +3.7 %), path 38.2 -> 39.1 deg (36.6), SECO
    T+507.7 -> 509.0 s with 3.0 -> 2.7 t left. Samples T+30 / 60 / 90 / 120 s: speed -9.5 / +2.2 / +4.8 / +4.8 % ->
    -9.8 / +2.0 / +3.1 / +2.6 %, altitude -8.5 / -8.5 / -3.3 / +0.4 % -> -8.7 / -8.9 / -3.5 / +0.1 % (`crs35_sample_gaps`:
    the gaps that shrank are tightened to these - T+60 / 90 / 120 s speed 0.023 / 0.046 / 0.042 -> 0.020 / 0.031 /
    0.026, T+120 s altitude 0.026 -> 0.001 -; the ones that grew keep their Task 6 constants, which with the 2 % margin
    still hold: T+30 s 0.095 / 0.085, T+60 s altitude 0.082, T+90 s altitude 0.021). `meco_speed_allowance` 0.097
    (INTERIM) -> 0.073 = 5.26 % + 2 %, rounded up (bound = 0.073 x the recorded 6490 km/h + 29 km/h reading tolerance =
    503 km/h; was 659 km/h); the remaining gap is an
    open discrepancy (budget above: max-Q bucket, MECO 0.9 s late from the planner's return reserve, ~1-2 % m / F at
    the dials' resolution, Dragon / trunk mass >= 13.1 t sourced without the trunk against 12.5 t [E] flown).
  - Falcon 9 RTLS mission (`test_falcon9_rtls`): MECO T+140 s with 52.8 -> 52.1 t; landing burn T+423 s at 2.99 -> 2.97
    km, 287 m/s, 12.1 t (inside its 0.2 t window: `boostback_loss_fraction` 0.13 stays); landed T+456 -> 455 s, 5.8 ->
    6.0 t left (later 6.4 t with the hull normal force in the landing planner, Task 8); separation -> boostback 13.6 -> 13.5 s (`flip_duration_s` 13.6 kept, the measured mean is 13.7 s); N2
    530 -> 527 kg at the separation, 159 -> 164 kg at the touchdown; dispersed campaign 21/21 (>= 5697 -> 5747 kg
    left). MECO plan from a fixed state: 50.8 / 47.8 -> 51.9 / 48.7 t (with / without entry burn).
  - Staging (`test_staging_sequence`): flip 13.6 -> 13.5 s, 348.5 -> 344.7 kg N2, pitch inertia 1.01e7 -> 9.98e6 kg m2.
  - Drone ship (`test_falcon9_asds`): MECO T+151 -> 150 s, 71.3 -> 70.3 km, 8376 -> 8160 km/h, 24.5 -> 24.8 t; ship
    682 -> 652 km from the pad (placed at the predicted impact); entry burn T+404 -> 397 s, 12.5 -> 12.7 t; landing burn
    T+496 -> 490 s at 2.95 km, 271 m/s, 11.8 t; landed T+535 -> 529 s, 5.0 t left; second stage 1.0 -> 0.5 t left at
    the orbit (16.5 t to 200 x 281 km); campaigns 21/21, 11/11, 10/10 unchanged.
  - Fixed-kick ascent (10 t payload, 12 deg): staging 66.7 -> 65.1 km, 2030 -> 1993 m/s, 4.2 -> 3.9 t left in stage 2.
    Max-Q limiter: flown max q 32173 -> 31997 Pa, lowest throttle 0.805 -> 0.814, MECO T+137.9 -> 137.7 s.
  - Falcon 9 MECO prediction at T+30 s (`[timeline]`): +7.13 -> +6.59 s, bound 9 s kept.
  - ISS window (`falcon9_crew_dragon_iss.toml`): `insertion_s` 540 -> 542 s, `insertion_arc_deg` 15.08 -> 15.01 [D]
    (flown separation 542.0 s after liftoff; phase error 0.207 deg before, 0.004 deg after).
  - Tests: `-TestFilter "[falcon9],[rtls],[asds],[ascent],[ascent_planner],[staging],[iss_mission],[timeline],[booster],
    [legs],[rcs],[dragon],[propulsion],[config]"` (slow included) 301/301 passed before the recalibration and the
    allowance change; after them with `[iss]` added 452/452.

