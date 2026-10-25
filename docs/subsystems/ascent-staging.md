# Ascent trajectory and staging

History and the reasons behind superseded decisions: `docs/history/ascent-staging.md`.
Plan `docs/superpowers/archive/plans/2026-10-22-ascent-trajectory-staging.md`, spec
`docs/superpowers/archive/specs/2026-10-22-ascent-trajectory-staging-design.md`. Milestone COMPLETE 2026-10-23.

## Model

### Staging sequence
`stage()` starts a sequence (`[stage.staging]` of the lower stage, `config::StagingConfig`; `sim::StagingSequence`,
`flight_session_staging.cpp`); `AutopilotState::Staging` lies between Booster and UpperStage and holds the guidance's last
command (no steering change while joined); every event at its own tick.
- **Cold (Falcon 9):** MECO (every group off: the tail-off acts on the joined stack), `separation_delay_s` 4.5 s [D]
  later the separation with `separation_dv_mps` 1.0 m/s [E] (pushers), `upper_ignition_delay_s` 6.0 s [D] after it the
  second stage lights. [D] from our CRS-35 / Starlink 15-25 webcast tables (see the TOML comment).
- **Hot (Super Heavy, also depot / tanker / HLS launch through `from`):** MECO keeps `run_on_engine_groups` (centre 3) at
  minimum throttle, the ship's engines start while joined, the separation (no push) comes on the tick none of them is
  still starting: ship start-up 3.0 s [E]. The booster keeps the centre engines into its flip.
- Without a `[stage.staging]` (trunk, payload stacks) or with the lower stage not burning, `stage()` separates at once.
  A second `stage()` during a sequence is ignored. Re-engaging the ascent autopilot during a sequence keeps the state
  Staging.
- **Timeline:** the hot MECO and the upper stage's ignition are recorded at their command (ignition with the serial the
  upper part gets); the observer skips an Ignition recorded within two samples. Predictions: separation at MECO +
  `separation_after_meco_s`, upper ignition at MECO + `upper_ignition_after_meco_s`.
- **Booster flip:** `[stage.recovery] flip_engine_groups` (unset: the final landing set - Super Heavy's centre engines;
  `[]`: engines off, the RCS turns the stage). MECO planning takes the flip from `flip_duration_s` (default 10 s [E];
  Falcon 9 13.6 s [D]) plus the sequence's MECO -> separation interval (hot: run-on engines burn at minimum throttle).
- **Falcon 9 flip** (follow-ups Task 7, user 2026-10-23: research first, fix vehicle data, no new guidance mechanism).
  Nitrogen cold gas thrusters, 2 packs of 4 on the interstage [S: everydayastronaut.com Starship guide, Falcon 9
  section]; thrust 6.5 kN each [D], N2 600 kg [D] (carved out of the 25.6 t dry mass: dry_mass_kg 25.0 t, total
  unchanged), Isp 70 s [E]. Neither thrust nor supply is published; the thrust is sized so the flown separation ->
  boostback is the mean of three timelines, 13.7 s (CRS-35 13 s, Bandwagon-3 14 s, Crew-13 14 s; spacelaunchschedule.com
  timelines, +-0.6 s on the mean); the CRS-35 webcast clock gives 13.0 s. `flip_duration_s` 13.6 s [D],
  `boostback_loss_fraction` 0.13 [D] (identified on the full RTLS: 12.1 t at landing-burn ignition for 12.1 t planned).
  The full RTLS uses 441 kg N2 (ascent 70, flip 349, trims and landing 22), 159 kg left (36 % margin, >= 20 % [E]).
  Pitch inertia of the empty stage 1.01e7 kg m2 [D] (model). One pack's two inward thrusters: 13 kN at 28.5 m from the CG
  = 3.71e5 Nm = 0.037 rad/s2 at the separation; RCS capability at the full stage's CG roll / pitch / yaw 24 / 265 / 265
  kNm; aero moment (q ~200 Pa at the separation) carries a large share of the turn. Whether the boostback lights before
  the turn completes is not settled (the model lights at 15 deg pointing error).

### Max-Q limit and operational ascent throttle
`gnc::AscentGuidance::booster_throttle`: the booster flies its operational throttle `[stage] max_ascent_throttle` and is
throttled below it to hold the dynamic pressure at `max_dynamic_pressure_pa` (`FlightSession::engage_ascent_autopilot`
copies both from the lowest stage into `PitchProgram`). No first-stage throttle schedules (`booster_throttle` key
removed).
- **Limiter law** (`gnc::max_q_thrust_acceleration_mps2`): feedback linearisation of q = rho v^2/2 along the path, dq/dt =
  rho v dv/dt + v^2/2 drho/dt = (limit - q)/tau with tau = `max_q_time_constant_s` 1 s [E] (full thrust holds to ~95 % of
  the limit; throttle moves ~0.1-0.2 /s, inside the engines' 0.5 /s rate). dv/dt besides thrust from the accelerometer
  (last tick) less the engines' thrust now, gravity and the centrifugal term; drho/dt from US76's gradient (scaled as
  the truth air) times the climb rate. Thrust to throttle: `gnc::booster_throttle_for_thrust` (the producing engines'
  common range, linear thrust).
- **Starship:** `max_dynamic_pressure_pa` 38909 Pa [D] = flight 14's peak q (held 38-39 kPa T+58-71 s at Mach 1.39-1.70;
  flight 13 Mach 1.62-1.88; supersonic, so one value). Super Heavy `max_ascent_throttle` 0.88 [D] (user 2026-10-23: the
  flown throttle is vehicle data, 280 tf stays the engines' capability): gauge mean flow T+20-56 s against rated 28.00
  t/s, 0.873 (flight 14) / 0.894 (flight 13), `tools/telemetry/mass_estimate.py` ("mean flow"); T+70-125 s 0.871 / 0.856,
  no late-boost limit.
- **Falcon 9** over Mach: `[[0.80, 35567], [0.90, 29600], [1.12, 29600], [1.45, 35567]]` [D]. Both records hold q
  transonically with the engines throttled and let it rise once supersonic (CRS-35 flat 28.7-29.6 kPa over Mach
  0.92-1.12; Starlink 15-25 flat 31.0-31.5 kPa over 0.97-1.13; full thrust reaches 31.6 kPa at Mach 1.43 and 35.6 kPa at
  1.55). Transonic = the lower hold (CRS-35's); elsewhere the highest q flown at full thrust. No `max_ascent_throttle`
  (425 t tanks' gauge flow is the rated flow). Tool: `tools/telemetry/peak_dynamic_pressure.py` (speed + altitude + US76;
  with `--capacity-t` / `--rated-flow-tps` the gauge's throttled phases, Mach range and held q).
- **MECO forecast:** the limiter and the cap run forward on a point mass (see `docs/subsystems/mission-timeline.md`); free
  launches within 0.02 s (Starship) / 0.32 s (Falcon 9) of the flown MECO over the booster phase from the kick's end,
  when the forecast is first shown.

### Pre-launch ascent planner
- `sim::plan_ascent_profile(like, request)` (`ascent_profile_planner.cpp`) chooses the first-stage kick by trial ascents
  (`sim::fly_ascent_trial`): each a fresh `FlightSession` on `like`'s ephemeris, Earth orientation, gravity model and
  config dir, the request's stack on its pad, booster return on, the ascent autopilot with the candidate kick, at the
  flight's 5 ms tick. Objective: the upper stage's propellant (oxidizer + fuel) once SECO's tail-off has ended (PEG counts
  on the tail-off impulse, ~9 m/s on Starship).
- Booster recovery is the hard constraint: the MECO plan (`BoosterReturnGuidance::meco_propellant_kg`) decides MECO in
  every trial; a non-finite plan is infeasible; the session must return the stage (`returns_after_staging`) and with a
  boostback its Lambert arc be found from the last Booster tick's state (`BoosterReturnGuidance::boostback_delta_v_mps`).
  The reserve holds the MECO plan's `landing_propellant_kg` plus run-down, flip and boostback by the rocket equation on
  the Lambert arc with `boostback_loss_fraction` on top. The return is not flown: at the separation the trial ends the
  booster's guidance (`FlightSession::end_booster_return`; identical upper-stage propellant, saves ~45 %). The
  Lambert-arc check is weak; a return that does not fit shows as an early MECO and the trial then fails the orbit check.
- Feasible: the guidance cut off before the stage ran dry and the radius flown over the next revolution
  (`predict_trajectory`) within 5 km [E] of the target's periapsis and apoapsis (circular targets); an elliptic target is
  judged on the osculating apsides at the end of the tail-off (the planner's `circular_target` is the guidance's rule).
- Search: kick in [4, 25] deg [E]; start 10.5 / 13 / 15.5 deg [E]; then the vertex of the parabola through the best trial
  and its two neighbours (also when the best is the outermost, clamped to 1.618 x the spacing); golden section where the
  fit is convex, leaves the bracket, or towards an infeasible neighbour; stop when the next step is < 0.1 deg [E]. Without
  a feasible trial it probes the middle of the widest kick gap and gives up once every gap is < 2.5 deg
  (`initial_step_rad`): a launch whose feasible band is narrower and falls between samples can be refused (conservative).
  Memoised per process by the request without its liftoff epoch, with the `like` session's gravity model in the key.
- **Recovery choice** (`sim::plan_launch(like, request)`): raises a target inclination below the site's latitude to it,
  chooses the recovery (`RecoveryChoice`: TowerCatch / LandingZone / DroneShip; replaces the request's landing site, part of
  the memo key), then plans the kick. By `recovery_method`:
  - `tower_catch` (Super Heavy): the site's tower; none -> "Kein Fangturm am Startplatz."; no plan with the catch ->
    "Ziel mit Booster-Rückkehr nicht erreichbar (<trial reason>)". Never expendable.
  - `leg_landing` (Falcon 9): the site's `landing_zone` (launch_sites.toml; SLC-40 -> LZ-40 [S]) when the plan with the
    RTLS return (MECO keeps the 12.1 t landing propellant) is feasible, also where the drone ship would leave more
    (SpaceX practice); else a drone ship from `config/drone_ships.toml` at the stage's impact predicted 10 s after the
    separation (`BoosterReturnGuidance::predict_impact`, drag and entry burn, no steering), deck along the ground track
    (in a moving sea: the heading of least deck tilt); its sea the EGM2008 geoid at the station
    (`FlightSession::sea_height_m`). The kick is searched with the ship at a provisional station 600 km [E] downrange.
    Neither feasible: refused.
  - Drone ships: `asds_atlantic`, `asds_atlantic_2` (Port Canaveral [S]), `asds_pacific` (Long Beach [S]), each with a
    service area [E] (lat/lon box). `config::drone_ship_for` picks the ship whose area holds the point (several: nearest
    home port, file order on a tie); none: refused "Kein Drohnenschiff für dieses Seegebiet." before any trial. Cape
    launches get `asds_atlantic`. The station is not checked to lie at sea (no land mask).
  - A mission's `[sea_state]` can refuse the drone ship (`sim::sea_state_refusal`, on the provisional and the chosen
    ship) and turns the deck to its least-tilt heading (`docs/subsystems/sea-state.md`).
- **Every launch through the planner:** `sim::start_ascent_mission` builds the request from the mission (vehicle, site,
  heading, target, payload) and calls `sim::engage_planned_ascent`: `plan_launch` with the session as `like` (on the
  simulation thread), then the chosen landing zone / drone ship, booster return on, ascent autopilot with the target and
  the planned kick. A refused plan leaves the stack on the pad, autopilot off, calls off a scheduled liftoff;
  `start_mission`, the ISS launch, depot launch, tanker / HLS flights and the flown campaign's pad launches return its
  German reason. A study's changed vehicle (`start_ascent_mission(..., &vehicle)`) is planned as it stands
  (`AscentProfileRequest::stack`, never memoised).
- **Mission files:** `[pitch_program]`, `kick_deg`, `booster_throttle`, `landing_zone` and `[drone_ship]` are gone from
  ascent missions and the loader refuses them (`AscentMission` has no `kick_angle_rad`, `landing_zone_id`, `drone_ship`).
  falcon9_asds deck 33.2240 / -75.7724 deg [D] was the hidden "nominal impact point" probe of the configured station,
  deck heading 45 deg [E]; `landing_zone = ccsfs_lz40` on falcon9_rtls and falcon9_crew_dragon.
- **Free launch:** the game's `launch_from_site` carries the target (`leo_ascent.toml`) unless manual control is on; the
  command places the stack, schedules T-0, publishes a snapshot with `autopilot_status()["planning"] = true` and plans
  (`sim::engage_pad_ascent(session, target)` on a clamped stack: the request from the placed stack - vehicle file by id
  with its own payload, fairing kept, so memoised - or the loaded configuration as `stack`). While the command runs the
  session does not advance: the world clock holds at the pad snapshot's epoch, the T- clock stands ("Startvorbereitung:
  Bahn wird berechnet" as last_event and in the webcast's name line); the liftoff follows the plan, never the default
  `PitchProgram`. Refused: the command is not ok with the reason (game: "Start abgelehnt: Kein Fangturm am Startplatz."),
  the vehicle stays on its pad. Key P on a clamped stack plans the same way. `engage_ascent_autopilot(target, program)`
  flies a given program (trial flights, studies, guidance tests). `FlightSim.start_mission` of a pad-launching mission
  (ascent, iss_launch, depot_launch, tanker_flight, hls_refuel, the flown campaign) publishes the planning flag before
  `sim::start_mission` and clears it afterwards (scope guard, cleared on a throw too).
- **Plan cache for tests** (`sim/ascent_plan_cache.hpp`): test processes share plans through a write-once disk cache,
  active only when `STARSIM_PLAN_CACHE_DIR` is set (the test CMake: `build/plan_cache`; the game never does). Disk key =
  memo key + liftoff epoch (hexfloat) + FNV-1a hash of the config directory's contents (a plan from another hour differs
  in the kick's last bits: tanker rendezvous moved by 1.6 t); temp name + rename; wiped on every relink of the test exe.
  Tests that assert trial flights call `sim::disable_plan_cache()`; `plan_speed` runs without the cache. Warm cache -49 %
  summed test time on the ascent labels.

### Validation missions
`config/missions/flight14_ascent.toml`, `crs35_ascent.toml`: each recorded flight's target, payload and propellant load,
flown through `sim::start_ascent_mission` - kick, recovery and MECO planned, nothing fitted.
- **`[propellant]` mission key** (`<stage id> = t`, `AscentMission::propellant_load_kg`): loaded before the engines start
  (`config::fill_tanks`; unknown stage or more than the tanks hold is refused); the request is then planned as it stands
  (not memoised). Flight 14: booster 3,533.2 t [D] (gauge 96.8 % of 3,650 t) - 5,650 t on the pad, 5,602 t at the release
  (mass_estimate.py: 5,542-5,603 t; full tanks and no payload 5,715 t); payload 52 t [E] (26 Starlink V3 at ~2 t). CRS-35:
  full, 12.5 t [E].
- **Flight 14's target is its insertion orbit, not 262 x 275 km:** Kepler fit of 620 readings T+515-1498 s (rms 0.3 km)
  gives -44 x 276 km (SECO 165 km, transatmospheric; the perigee raised near apogee at T+1520-1640 s, +89 m/s, to 262 x 277
  km). 262 x 275 km with 52 t and the catch is refused by the planner; -44 x 276 km would need a guidance change (non-goal).
  Flown: **165 x 276 km, 30.5 deg** [D] (~105 m/s more than flight 14); kick 16.33 deg, 37.2 t left in the ship, 165.0 x
  276.7 km.
- **Dials' time base:** the speed dial lags the release (our T+0) by 3.50 s (flight 14) / 3.24 (flight 13) / 3.42
  (CRS-35) / 3.26 s (Starlink 15-25); the altitude dial lags the speed dial by another 1.95 / 2.00 / 2.45 / 2.05 s.
  Tool `tools/telemetry/liftoff_time.py <table> [--simulated <dump>]` (model-based shift 3.53 s for flight 14).
  Calibrated [D] per table in the tests: speed 3.50 / 3.42 s, altitude 5.45 / 5.87 s, +-0.25 s.
- **Not separable, kept as [E]:** Falcon 9 thrust per flow before max-Q (recording accelerates ~0.5 m/s^2 harder, 4.42
  vs 3.95 m/s^2 at 8-40 m/s) and ascent drag (before max-Q < 0.3 m/s^2; through max-Q the throttle is unknown). Ascent
  tables `config/aero/*_ascent.csv` stay [E].

### Falcon 9 engine data (Merlin 1D, decision 2026-10-23)
`config/engines/merlin1d.toml`: vacuum thrust 914 kN [S: SpaceX Falcon 9 page, 8,227 kN / 9] at the unchanged rated flow
(845 kN / 282 s at sea level = 305.6 kg/s, pinned by burn time and both flights' gauges): `nozzle_exit_diameter_m` 0.932
[D] (exit area 0.682 m2), Isp_vac 305.1 s [D]. The 311 s on the same page contradicts its thrusts at one flow and is
dropped (user 2026-10-23: option A, no g limit). Merlin Vacuum (second stage) 981 kN [S:wiki_merlin] is a separate
question, unchanged. Second stage 106.7 t propellant [D] + 3.9 t dry against Falcon 9 FT 75.2 t LOX + 32.3 t RP-1 =
107.5 t, empty 4.0 t [S: en.wikipedia.org/wiki/Falcon_9_Full_Thrust citing Espace & Exploration No. 39, 2017]. Dragon:
CRS-35 cargo 2,860 kg [S: space.com 2026-10-13]; Dragon 2 dry 7,700 kg + propellant 2,562 kg [S: Wikipedia SpaceX Dragon
2]; cargo-Dragon and trunk masses unpublished (13.1 t plus trunk against 12.5 t [E] flown).

### Landing planner (SCvx) rules from this milestone
- **Staging effect on the depot catch:** ignition engines that the next set drops run until they are at thrust:
  `DescentLimits::phase0_min_s` = the rest of their start-up (from the plan's start; re-plans less the latency), set by
  `ScvxLandingGuidance` for start phase 0 when the next set drops engines (Super Heavy; not Falcon 9; not in an abort).
- **`ScvxSettings::fresh_max_iterations` 60 [E]** (re-plans keep 10): ASDS MECO seed 107 needs 42-44 iterations. A
  non-converging fresh plan is refused after ~1.58 s (~25 ms/iteration; the simulation thread stands that long).
  **`fresh_stall_iterations` 4 [E]** (re-plans keep 3): Super Heavy's sea abort from ~350 m passes a transient of its
  virtual control (fixture `tests/validation/fixtures/scvx/superheavy_abort_sea_353m.txt`).
- **Reserve** (`SnapshotSettings::reserve_kg`): our tanks burn to empty, so for Super Heavy (5 t [E]), Falcon 9 (5 t) and
  the HLS (5 t [E]) the reserve is planning margin; the ship's landing is fed by the header tanks, so its 1 t stays a hard
  floor (`ship_landing_parameters`: both fractions below 0). **Soft reserve** `DescentLimits::soft_reserve_kg`,
  `DescentWeights::reserve_use` 100 per 1e4 kg [E] (exact L1 penalty; the virtual control stays 10x dearer); shares
  (`ScvxLandingParameters`): first plans 0, re-plans `replan_reserve_fraction` 0.5 [E], abort plans
  `abort_reserve_fraction` 1.0 [D]. Against `final_lateral` 0.01 per m and node the weights break even at 1 kg of reserve
  per metre of offset at one node (ASDS plans spend the margin on the lateral penalty).
- **Falcon 9 gimbal allowance** `[stage.recovery] landing_gimbal_fraction` 0.45 [E] (band 0.40-0.45 from reach and
  authority; 0 in the file = default; whole landing burn, RTLS and drone ship); the default
  `DescentLimits::gimbal_fraction` 0.3 is Super Heavy's (3 Raptors near minimum throttle, limit cycle at 0.4). At 0.45 the
  LQR command peaks <= 0.50 of the whole gimbal in 62 of 63 Falcon 9 campaign cases, never clipped (seed 117 peaks 1.06,
  14 of 2486 ticks clipped).
- **Hull normal force in the plan** (follow-ups Task 8, 2026-10-23), shared by every vehicle that plans with
  `descent_model` (zero in vacuum): the flight model's slender body (`aero::SlenderBodyAero`): Jorgensen cross-flow
  `q L D cd_x(M) sin^2 a` at mid-length (`crossflow_cd_by_mach` = eta C_dc from the stage's `CrossflowCurve`, else
  `aero::generic_crossflow_efficiency` 0.68 x `generic_crossflow_drag_coefficient` 1.2, `SlenderBodyAero::
  continuum_crossflow_cd`) and, for a body with a nose (the ship), the potential term `q A |sin 2a| cos(a/2)` at the
  ogive's centre of pressure (`potential_station_m`), with their moments about the CG (`DescentModel::hull_normal_force`).
  Continuum only; grid fins and flaps stay out of the plan. Checked against `SlenderBodyAero::compute` to 1e-6
  (`[descent]`: Falcon 9 at 1.2 km / 250 m/s / 10 deg: 3.36 m/s^2 lateral; ship nose term; moment sign; Jacobians). The
  return guidance's online drag estimate (`BoosterReturnGuidance::drag_scale`, frozen at landing-burn ignition) scales the
  planned normal force (`DescentModel::normal_force_scale`, via `SnapshotSettings`); the axial drag table stays unscaled.
  The landing LQR is consistent with the plan: `ScvxLandingGuidance::update` cancels the model's moment at the measured
  state and gives the gimbals the rest (a booster's torque goes to the gimbals as is; the ship's allocation cancels its
  modelled moment hull + flaps itself). Fidelity flag `SnapshotSettings::hull_normal_force` (default on).
- **Abort decision** (RTLS "cannot reach the landing zone"): abort plans may use the whole reserve; a converged plan
  whose final set tilts more than `max_planned_tilt_excess_rad` 3 deg [E] beyond its soft limit
  (`DescentPlan::planned_tilt_excess`, nodes after the switch node; final set only) does not reach the target within the
  vehicle's authority: where there is a sea it counts like an unreachable plan (first plan: abort at once; re-plans: towards
  `failures_before_abort`). Fixture `falcon9_rtls_900m_beyond_zone.txt`.

## Code
- Staging: `sim::StagingSequence`, `flight_session_staging.cpp`, `config::StagingConfig`; tests `test_staging_sequence.cpp`.
- Ascent guidance: `gnc::AscentGuidance`, `PitchProgram`, `gnc::max_q_thrust_acceleration_mps2`,
  `gnc::booster_throttle_for_thrust`, PEG (`docs/subsystems/guidance-missions.md`).
- Planner: `core/src/sim/ascent_profile_planner.cpp` (`plan_ascent_profile`, `fly_ascent_trial`, `plan_launch`,
  `engage_planned_ascent`, `engage_pad_ascent`), `sim/ascent_plan_cache.hpp`.
- Return: `gnc::BoosterReturnGuidance` (`meco_propellant_kg`, `boostback_delta_v_mps`, `predict_impact`, `drag_scale`),
  `ScvxLandingGuidance`, `descent_model`, `gnc/descent_request_io`.
- Validation helpers: `core/tests/ascent_validation.*`; tools `tools/telemetry/{peak_dynamic_pressure,mass_estimate,
  liftoff_time,boostback_fit,late_ascent_fit}.py`.
- Data: `config/vehicles`, `config/engines/merlin1d.toml`, `config/missions/*_ascent.toml`, `config/drone_ships.toml`,
  `launch_sites.toml`.

## Standing decisions
- Planned kick everywhere; recovery as a hard constraint of the planner; Super Heavy never expendable (user 2026-10-22/23).
- Super Heavy `max_ascent_throttle` 0.88 is vehicle data, 280 tf stays the engines' capability (user 2026-10-23).
- Falcon 9: 914 kN vacuum at the rated flow, no ascent g limit (user 2026-10-23, option A); flip 13.6 s from research, no
  new guidance mechanism (user 2026-10-23).
- Physics over gauges: Falcon 9 gauge's 66-69 t at MECO is out of reach (thrust ceiling bounds the first stage's
  propellant at boostback ignition to <= 60.5 t) and is not followed; flight 14's late throttling and engine out are not
  modelled (no late-boost limit without a sourced g limit).
- Campaign comparison ruling 2026-10-23: the refuelling fill is compared against the calculator's model of each flown
  window (every tanker within 0.5 %), with the class mix asserted (see `docs/subsystems/refuelling.md`).
- The forecast of MECO is not shown before the kick's end (the return's MECO plan from the near-vertical state swings).

## Validation
Tolerance scheme (`test_starship_ascent.cpp`, `test_falcon9_ascent.cpp`): bound = reading tolerance (speed 1 km/h,
altitude 0.05 km, +-0.25 s per reading and +-0.25 s time base, times the local rate) + a named allowance per documented
open discrepancy (measured + `test::allowance_margin` 2 % [E]; each T+30/60/90/120 s sample has its own
`test::SampleGap`); MECO reading tolerance = local rate x 2 x the MECO's uncertainty (flight 14) / x 1 (CRS-35).
`STARSIM_ASCENT_DUMP=<dir>` writes the flown samples. Samples' altitude is above the pad (`docs/subsystems/earth-terrain.md`).
- **Staging** (`[staging]`): Falcon 9 booster -1.00 m/s along track right after separation; Starship booster -0.03 m/s at
  separation then -4.6 / -8.9 / -12.4 / -15.1 / -17.8 m/s over the next 5 s; Falcon 9 flip 13.5 s, 344.7 kg N2, pitch
  inertia 9.98e6 kg m2; separation -> boostback within 3 s of 13.7 s (`test_falcon9_rtls`, `test_staging_sequence`).
- **Max-Q hold** (`[ascent]` "The ascent holds dynamic pressure at the vehicle limit"): Falcon 9 max q 31997 Pa (at most
  1.000 of the limit at its Mach), lowest throttle 0.814, full thrust below 90 %, MECO T+137.7 s (fixed-kick 10 t, 12 deg);
  free Starship (kick 10 deg, 0.88) peaks at 33.8 kPa (0.87 of the limit, never limited).
- **Starship vs flight 14** (`flight14_ascent`, planned kick, recorded flight's load): the model follows within the readings
  up to max-Q. After max-Q (flight 14 throttles to 0.78-0.90 from Mach 1.39, centre engine out from T+76 s, ours stays
  0.88): T+60 / 90 / 120 s +1.7 / +6.9 / +9.0 % fast, +0.6 / +7.4 / +11.2 % high; MECO from the catch plan (294 t in the
  booster; flight 14 ~500 t by its gauge, not followed after staging) T+129.8 vs 136.0 s, 6559 vs 5943 km/h (+10.4 %), 52.5
  vs 51.0 km, path 30.3 vs 27.8 deg; SECO T+447 vs ~503 s (flight 14's RVac out from T+438 s, slower MECO, lower insertion).
- **Falcon 9 vs CRS-35** (`test_falcon9_ascent`, with 914 kN): kick 11.80 deg, MECO T+135.5 vs 134.6 s, 6831 vs 6490 km/h
  (+5.3 %), 59.5 vs 57.4 km (+3.7 %), path 39.1 vs 36.6 deg, 53.0 t kept at MECO; SECO T+509.0 s with 2.7 t left. Samples
  T+30 / 60 / 90 / 120 s speed -9.8 / +2.0 / +3.1 / +2.6 %, altitude -8.7 / -8.9 / -3.5 / +0.1 %. `crs35_sample_gaps`: T+60 /
  90 / 120 s speed 0.020 / 0.031 / 0.026, T+120 s altitude 0.001, T+30 s 0.095 / 0.085, T+60 s altitude 0.082, T+90 s
  altitude 0.021. `meco_speed_allowance` 0.073 = 5.26 % + 2 %, rounded up (bound 503 km/h = 0.073 x 6490 + 29 km/h reading
  tolerance). Remaining gap (open): max-Q bucket (CRS-35 throttles earlier and deeper, +2.9-3.1 % already at T+90 s), MECO
  0.9 s late from the planner's return reserve (flight keeps up to ~61 t, ours 53 t), ~1-2 % m/F at the dials' resolution,
  the flight's acceleration levelling at ~3.8-3.9 g in the last seconds (no source for a limit; ~0.35 % of MECO speed),
  Dragon + trunk mass.
- **Falcon 9 boostback vs CRS-35** (diagnosis, no model change; tool `tools/telemetry/boostback_fit.py`, hidden test "Falcon
  9 CRS-35 first stage profile dump", `STARSIM_BOOSTBACK_DUMP=<csv>`): CRS-35's boostback needs 2014 m/s (2014-2068 over
  fits) against our 2100 m/s and burns 38.2-41.4 t against our 38.8 t; its ~55 s (T+151-206.5 s) are a soft start (~18 s
  at ~2/3 thrust) and ~5 s trim, not more delta-v; ours 43.2 s (T+153.8-197.0 s), 677 m/s at 114.9 km, apogee 122.9 km. The
  MECO speed gap is the ascent's, not the boostback's. `boostback_loss_fraction` 0.13 stays.
- **Falcon 9 RTLS mission** (`test_falcon9_rtls`): MECO T+140 s with 52.1 t; boostback 13.5 s after separation; landing
  burn T+423 s at 2.97 km, 287 m/s, 12.1 t (0.2 t window); landed 0.00 m off T+455 s, 6.4 t left (with the hull normal
  force); N2 164 kg at the touchdown; dispersed campaign 21/21 (>= 5747 kg left); MECO plan from a fixed state 51.9 / 48.7 t
  (with / without entry burn; leverage bound 4.0 t); entry-burn leverage 3.6 t.
- **Falcon 9 drone ship** (`test_falcon9_asds`, deck 33.2240 / -75.7724): MECO T+150 s, 70.3 km, 8160 km/h, 24.8 t; ship 652 km
  from the pad; entry burn T+397 s, 12.7 t; landing burn T+490 s at 2.95 km, 271 m/s, 11.8 t; landed T+529 s, 5.0 t left;
  second stage 0.5 t left at the orbit (16.5 t to 200 x 281 km); campaigns 21/21, 11/11, 10/10.
- **Planner measurements** (parallel test load): Starship to 200 km / 26.5 deg 14.85 deg, 88.5 t at SECO, 4 trials, 15-26 s
  (1-deg sweep peaks at 15 deg; MECO flight-path angle 31.2 deg; the ship never pitches below its flight-path angle in the
  first 60 s after separation); Falcon 9 CRS-35 class (12.5 t, 200 km / 51.6 deg, SLC-40) -> LZ-40, 11.57 deg, 2.2 t, 4
  trials, 11.7 s (kick re-baselined after 914 kN: 11.80 deg in the CRS-35 check); Starlink class (16.5 t, 200 x 280 km / 53 deg,
  SLC-40): RTLS infeasible -> drone ship at 33.1419 N / 75.8444 W, heading 44.8 deg, 13.93 deg, 1.0 t, 15 trials, 35.6 s
  (configured station ~11 km away); CRS-35 request from LC-39A: drone ship 33.2590 / -75.4291, 14.94 deg, 7.0 t, 5 trials;
  Starship 2000 km circular refused after 10 trials (23.6 s); Starship from LC-39A to 26.5 deg flown at 28.608 deg, 14.82 deg,
  88.1 t; tanker_ascent 18.07 deg, 145.5 t at SECO (120 km perigee favours a flatter arc). Re-measure after changes.
- **Mission timeline** (`[timeline]`): MECO prediction at T+30 s Falcon 9 +6.59 s (bound 9 s); starship_leo +5.6 s at T+30 s,
  whole-flight bound 9 s from the kick's end (shown 109.2 s before MECO, +5.19 s, at most 6.58 s off, 2.21 s within 10 s).
  Starship booster return predictions: landing burn / catch bounds 15 / 25 s at the apex plus 6 s within 120 s.
- **Super Heavy return on starship_leo** ("Block 3 plans MECO ..."): boostback T+148-169 s from 273 t, apogee 103.1 km at
  T+221 s, landing-burn ignition T+383 s with 52.4 t (51.0 t planned); apogee and ignition bounds 25 s.
- **Hull normal force A/B** (clipped LQR ticks): ASDS MECO states 0 / 82456 and 0 / 73036, ASDS at ignition 237 / 151618
  (peak 1.97), RTLS 0 / 110620, Super Heavy catch 0 / 125972, ship catch campaign 39 / 85609, lunar-return seed-18 gate 20 /
  4060; no case clips in its last 10 s. Outcomes: ASDS 11/11, 10/10, 21/21 landed, RTLS 21/21 (sink <= 1.05 m/s), Super
  Heavy catch 21/21, ship catch campaign 20 of 21 (known engine-failure case). Abort test: 1500 m Wasserung 552 m / 12.4 t;
  800 m 549 m / 10.0 t; 400 m 532 m / 6.1 t. Results of the SCvx rules above: `asds-single` seeds 0, 101-120 21/21 with no
  failed re-plan (ASDS cases land with ~5.0 t, seed 107 7.8 t).
- **Windows and reference flights** (re-baselined by the ascent changes; re-measure after changes to ascent, staging or
  engine data): tanker_flight `insertion_s` 432.2 s / `insertion_arc_deg` 12.25; hls_refuel 415.7 s / 11.78 deg;
  `falcon9_crew_dragon_iss.toml` 542 s / 15.01 deg (`docs/subsystems/iss.md`). `reference_flights.toml` (slow r5 flights):
  depot fill 70.7 t; tanker residue 105.6 t, rendezvous 17.37 t, delivered LOX / CH4 59.4 / 14.85 t, settling gas 0.98 t,
  main at catch 1.21 t; HLS main at docking 202.8 t; `inputs_digest` afc1cbc547f04fbd; `campaign_offset_above_t` 2.0 /
  `below_t` 1.1 t [D]. Calculator 12 tankers / 50.6 d; full flown campaign (ctest `Flown campaign: the depot launched*`)
  12 tankers, 52.9 d, HLS at the depot 1079.7 t (needs 1041.4 t), first return caught 2.58 t; placed campaign
  (`refuelling_report --fly-placed`) 2 tankers, 8.6 d. Details: `docs/subsystems/refuelling.md`.
- **Test tags** for this subsystem: `[falcon9],[rtls],[asds],[ascent],[ascent_planner],[staging],[iss_mission],[timeline],
  [booster],[legs],[rcs],[dragon],[propulsion],[config]` plus for the landing planner `descent, scvx, scvx_landing,
  landing, catch, lunar_descent_*, lunar_scvx, crew_escape, ship, replanner, hls, return` (last full re-validation of the
  Task 2b labels: 293/293; of the Task 7c tags 452/452 with `[iss]`).

## Diagnostics
- `STARSIM_DEBUG_STAGING=1`: both parts' rates and relative speed at the split.
- `STARSIM_DUMP_SCVX=<dir>`: writes every plan request (model tables, state, target, settings, warm start;
  `gnc/descent_request_io`); hidden test `[scvx-diag]` replays one (`STARSIM_SCVX_REQUEST=<file>`, overrides
  `STARSIM_SCVX_OVERRIDE="limits.gimbal_fraction 0.45;..."`), prints each node against its limits and its nonlinear defect;
  replays are bit for bit. Fixtures in `tests/validation/fixtures/scvx/` (`falcon9_asds_seed117_741m.txt`,
  `falcon9_rtls_900m_beyond_zone.txt`, `superheavy_abort_sea_353m.txt`).
- `STARSIM_DEBUG_NO_CROSSFLOW=1`: planner without the hull normal force (A/B baseline). `STARSIM_DEBUG_SCVX_SOLVER=1`: also
  the virtual control per SCvx iteration (`[scvx it]`); with `STARSIM_VERBOSE=1` every `test::fly_landing` prints a
  `[scvx clip]` line. `ScvxLandingGuidance::Statistics` (commanded tilt acceleration over the gimbal's).
- `STARSIM_ASCENT_DUMP`, `STARSIM_BOOSTBACK_DUMP` (above); `tools/telemetry/*` (above).

## Open points and known limits
- Falcon 9 vs CRS-35 and Starship vs flight 14 gaps listed under Validation (allowances asserted).
- Whether the real Falcon 9 boostback lights before the flip is complete is not settled; the 6.5 kN flip thrust rests only
  on the measured flip time.
- Falcon 9 drone ship station is not checked to lie at sea (no land mask); a launch whose feasible kick band is narrower
  than 2.5 deg and falls between samples can be refused.
- Remaining LQR clipping after the hull normal force: the ship's flaps (the allocation cancels them from the flight model
  while the plan has none) and the ASDS ignition dispersions; the plan's axial drag under the online estimate is unscaled.
- A fresh plan that does not converge costs ~1.58 s of simulation-thread wait before the abort / give-up path starts.
- The fixed-kick bound: Falcon 9 mid-burn model gap is closed; Super Heavy `gimbal_fraction` 0.3 limit-cycles at 0.4.
