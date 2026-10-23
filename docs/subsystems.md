# Subsystems, decisions and validation record

Detailed notes moved out of `CLAUDE.md` (2026-10-15) to keep the per-session context small. Each entry records
a subsystem's design, the decisions behind it (with dates and who decided), its validation numbers and its
diagnostics. Keep it up to date when a subsystem changes; `CLAUDE.md` keeps only the rules and a short index.

## Vehicle data, ascent and booster recovery

- Only the current configuration (Block 3 / V3, Raptor 3) is modelled (decision 2026-10-13); its data come from
  the flights that flew it (12-14, `tests/validation/telemetry/flights.toml`). Earlier blocks were removed.
- The booster's ascent throttle is vehicle data since ascent Task 2 (user 2026-10-23): `[stage] max_ascent_throttle`
  0.88 [D] (the flights 13/14 propellant gauge's flow before max-Q against the rated flow) and the max-Q limit
  `max_dynamic_pressure_pa` 38909 Pa [D] (flight 14's peak q); the speed-fitted time schedule of the missions is gone
  (section "Ascent trajectory and staging"). Every launch plans its kick (ascent Task 3-5). Validation (ascent Task 6,
  test "Starship ascent to flight 14's insertion orbit against its webcast telemetry"): on the dials' own time base
  (speed 3.50 s, altitude 5.45 s late) and with flight 14's booster load (96.8 %) the planned ascent follows flight 14
  within the readings to max-Q; from there flight 14 throttles below 0.88 and loses an engine: +7-9 % fast at T+90-120 s
  (open, section "Ascent trajectory and staging"). Staging mass vs the gauge (decision 2026-10-14: physics first, the gauge after staging is not
  followed): on this ascent our return plans MECO with ~310 t (301 t at the boostback start, ~2.2 km/s on 13 engines),
  flight 14's gauge shows 11.8 % (431 t) there. The gauge is not consistent with the engines after staging: it falls
  ~15 t/s on 13 engines (full mass flow ~10.5 t/s) and 3.5 -> 1.4 % (77 t) in the last ~12 s on 3 engines (~30 t);
  Falcon 9's falls 21 t in a coast without a burn. The timeline matches (test "Block 3 plans MECO ..."): boostback
  T+152-176 s (flight 14 ~152-187 s), apogee 91.8 km at T+236 s (98.9 km, T+240 s), landing-burn ignition T+390 s
  (~T+398 s) at 287 m/s / 2.2 km (flights 13/14: ~355 m/s at ~2.9 km; subsonic drag is not measured). The earlier
  "~207 t, boostback ~1.5 km/s" came from the default full-thrust ascent (MECO at ~97 km climbing at 1.7 km/s).
- **Scheduled liftoff and pad hold** (external view, Task 4): `FlightSession::schedule_liftoff(t0)` gives the stack on
  the pad a T-0. The ascent autopilot (`PreLaunch`) then holds: the engines start `liftoff_ignition_lead_s()` before
  T-0 (longest first-stage `startup_s` + `liftoff_hold_margin_s` 0.5 s [E]; Raptor 3: 3.5 s) and the clamps open at
  the first tick with all engines running at or after T-0 (test: engine start T-3.49 s, liftoff at T-0 within a tick).
  While waiting the clamped stack coasts (`FlightBody::coast` keeps a clamped body on its pad, co-rotating) until
  `burn_preparation_s` before the ignition, so hours on the pad cost a few coast segments. Without a schedule the
  liftoff follows the engine start at once (unchanged). `sim::StartOptions{wait_on_pad, countdown_s}` of
  `start_mission`: launch missions (ascent, iss_launch, depot_launch, tanker_flight / hls_refuel from the pad) stand on
  the pad from `requested` and lift off at the window's liftoff (depot_launch: its epoch; ascent: `requested` +
  `[mission] countdown_s`, else the option's `countdown_s`); their stations are placed at the session epoch. T-0 is
  never less than the ignition lead after the placement (fix round 1: a T-0 the engines cannot meet made the clock jump
  back at the real liftoff): a window's liftoff or a configured epoch is kept and the vehicle placed the lead before it,
  a countdown T-0 moves to placement + lead. iss_launch waits only where the ISS comes from the same source as at the
  liftoff (the OEM file the window used, else the synthetic orbit): the placement moves up to that file's coverage.
  Disengaging the autopilot during the hold drops the schedule (no T-0, no predicted liftoff). The default
  (`wait_on_pad = false`) places the vehicle at the liftoff as before.
- Circular ascent targets are J2-aware (`gnc/j2_insertion`): when the gravity model has J2
  (`GravityModel::central_j2`), PEG aims at the radius, radial and horizontal speed at the predicted cutoff point whose
  next revolution stays closest to the target radius (Gauss-Newton on a point-mass + J2 revolution, re-solved when the
  cutoff point moves > 2 deg). A Kepler-circular insertion swung 5-20 km in radius (Starship 26.5 deg: 190.8-200.0 km);
  now the flown revolution keeps 199.7-201.6 km - the J2 short-period swing left is 0.7 / 2.1 / 3.3 km at 26.5 / 51.6 /
  97 deg. Circular-orbit tests judge the flown revolution; elliptical targets (Starlink 200 x 280 km) are still judged
  on osculating elements at cutoff (Kepler). Burn planning helpers are two-body.
- Booster recovery (decision 2026-10-13): Super Heavy is always caught (no legs; `[stage.recovery] method =
  "tower_catch"`): return guidance only at sites with a tower. Falcon 9 boosters land on the pad or the drone ship
  ("leg_landing", milestone c: `docs/superpowers/specs/2026-10-13-falcon9-landing-design.md`;
  vehicle, legs, the RTLS to LZ-40 (c3), the drone ship (c4) and the game interface (c5) done; the drawing and UI wait
  for the user's UI specification). If the first SCvx catch plan fails, an SCvx sea abort is planned at once;
  the explicit laws are the last fallback (over the arms, ~1-3 m: not accurate enough to be caught); with
  `scvx_landing = false` the whole return aims beyond the shoreline and ditches (test: 1.05 m/s, 650 m off).
- Booster return: at a catch site the landing burn flies SCvx plans (`gnc/scvx_landing`), V3 sequence 13 -> 5 (divert pair) -> 3 engines,
  re-planned every 1 s on a worker thread (effective at snapshot + 0.5 s: a re-plan starts from the snapshot
  predicted over that latency with the active plan's thrust, its start thrust within the throttle-rate reach of the
  measured thrust; tests and time warp wait, real-time play never waits - `FlightSession::set_realtime_planning`).
  The approach into the arms is slow (sink <= 1 m/s + height / 3 s, soft). From an ignition state (scenario:
  355 m/s at 2.88 km as on flights 13/14 [D]; the ignition profile flies 13 engines at 35 %) the dispersion campaign
  `[gnc][catch][validation]` (+-10 m, +-3 m/s, +-5 % mass, +-3 deg, engine failure) catches 21 of 21 (< 0.1 m, < 0.05 m/s lateral, 0.95-0.97 m/s sink at pin contact). The landing
  throttle accounts for the nozzle pressure loss (thrust = throttle x F_vac - p A_e; a throttle proportional to sea
  level thrust gave ~10 % too little thrust at low throttle). Lateral tracking is an LQR on the
  gimbaled-rocket dynamics (`gnc/tvc_lateral`); plans use <= 30 % of the gimbal limit, the LQR effort is 0.2 x the
  gimbal authority. The coast phase already rolls the pins towards the arms. Planning model: constant CG/inertia,
  no grid fins, no wind; no re-plans in the last 3 s. The explicit laws remain the fallback.
  Switch windows [E]: 13 -> 5 at 180-220 m/s sink (5 engines seen at ~200 m/s on flights 13/14), 5 -> 3 calm at
  15-30 m/s (satnews 2025-10-05: V3 ends in a hover on 3 engines).
  Full return: caught, 51.0 t at landing-burn ignition (planned 51 t, flight 14 gauge; the MECO plan budgets 1.7 t [D] of
  main-tank RCS gas, `[stage.recovery] rcs_propellant_kg`, since d1; the catch and MECO tests fly the flight-like
  ascent `starship_leo`: 52.6 t); it arrives slower than flights 13/14 (287 m/s at ignition, ~2.2 km). Abort plans must end beyond the shoreline (`DescentTarget::sea_min_distance_m`
  = shoreline + `sea_clearance_m` 30 m [E], a hard constraint) and pull towards the sea target (soft); only when the
  water is out of reach do the plans fall back to the soft pull (before: the soft pull alone, 100 kg per metre, set the
  booster down beside the tower). `FlightSession::abort_landing` / `FlightSim.abort_landing` is the pilot's abort
  (refused - false, arms untouched - without a sea, once down or with the pins on the arms; on the explicit laws the
  descent re-aims beyond the shoreline; before the landing burn it is remembered for it); test
  "A commanded abort late in the landing burn ditches the booster at sea" (1500 / 800 / 400 m: water, 25.7 / 14.6 /
  5.7 t left).
- Descent drag (tail first): body table `aero/super_heavy_descent.csv` [D] plus the grid fins' axial drag
  (`aero/grid_fin.csv` column `ca_axial` [E], 3 x 13.5 m^2). Identified from our webcast telemetry of flights 13 and
  14 (`tests/validation/telemetry`, `tools/telemetry/webcast_telemetry.py` + `drag_estimate.py`; below ~9 km the
  speed dial gives way to the engine icon, speed then from the 0.1 km altitude of the nearly vertical fall): the
  unguided booster (210 t dry + 51 t) follows both descents from ~27 km within 0.6 / 2.6 % rms and loses altitude to
  the landing burn within 415 / 158 m (test "Booster descent drag reproduces the webcast telemetry of flights 13
  and 14"). The webcast's speed and altitude graphics are offset by ~1.5-2 s. Subsonic drag is not measured.
- Block 3 masses [D] from flights 12-14 (`tools/telemetry/mass_estimate.py`, `ship_dv.py`): ship dry 255 t
  (rocket equation staging -> cutoff with the propellant gauge, 248-260 t), booster dry 210 t +-40 t (stack mass
  balance and descent ballistic coefficient), liftoff ~5,550 t; tank capacities 3,650 / 1,600 t [S]; Raptor 3
  280 tf consistent (peaks ~270 tf sea-level equivalent; 250 tf is contradicted), minimum throttle 30 % [D] (V3
  hovers on 3 engines). MECO of a returning booster is planned by the ascent autopilot from the state
  (`BoosterReturnGuidance::meco_propellant_kg`: run-down + flip + boostback by the rocket equation on the Lambert
  delta-v after the flip, +8.6 % [D] for the drag-shifted aim point and finite burn) so that `landing_propellant_kg`
  = 51 t [D] (flight 14 gauge) remain at landing-burn ignition (test: 51.0 t with the RCS gas budget; 30 / 70 t targets within 0.4 t);
  on the flight-like ascent it stages with ~310 t. `reserve_propellant_kg` (225 t) is the MECO without return guidance.
  The flights' gauge shows ~500 t (~13.5 %) at MECO and 1.4 % (51 t) after the boostback (video T+188-195 s; the
  cleaned table drops these readings) - more than the engines can burn in between: the gauge is not followed after
  staging (see the Starship throttle note).
- Catch tower dimensions and azimuths (`[site.tower]`, Starbase 315 deg, LC-39A 0 deg) and the catch pins
  (`[stage.catch_pins]`, 5 MN per pin) are estimates [E]; verify the tower azimuths against imagery.


## Guidance, vehicles and missions (M5 onwards)

- GNC (M5): `gnc/ascent_guidance` (pitch program + PEG + terminal v_go steering), `gnc/burn_node`
  (closed-loop burns), autopilot state machine in `sim::FlightSession` (`core/src/sim/flight_session.cpp`: construction,
  placement, autopilot loop, tick, pilot actions; `_command.cpp` the per-tick vehicle command; `_burns.cpp` burn
  configurations, execution, circularisation; `_return.cpp` ship and booster return; `_capsule.cpp` capsule entry and
  parachutes; `_lunar.cpp` the lunar flight). Burns pick the strongest engine
  configuration whose start/shutdown impulse is <= 1/4 of the delta-v; smaller burns fly on RCS translation along +x
  (the thrusters' force and mass flow) when the vehicle has thrusters along +x (the ship's aft ullage thrusters,
  Falcon 9 second stage), else they are rejected. Game keys: P ascent autopilot (config/missions/leo_ascent.toml),
  O circularize at apoapsis, R ship return to the site's tower (b4), X autopilot off. HUD: NavBall (`ui/nav_ball.gd`), flight director
  (`ui/flight_overlay.gd`), trajectories (`vehicle/trajectory_view.gd`: green planned, orange predicted).
- Catch tower (M6, `contact/*`, `sim/catch_tower_sim`): a site loaded with `config::load_launch_site` /
  `sim::launch_site_from_config` carries its tower; stacks then stand on the launch mount (20 m). The tower
  closes its arms automatically (time to pin contact <= closing time + 1 s, axis offset <= 3 m), applies
  penalty contact forces (pins and a hull ring vs the arm boxes, friction 0.3) through
  `FlightBody::apply_external_wrench`, and judges: caught (both pins, engines off, at rest 2 s ->
  `FlightBody::hold_fixed_to_earth`), impact (tower/mount hit, pin load > `max_load_n`, carriage hard stop
  > 1 m/s, an arm > 0.2 m deep in the hull) or missed. With a tower the booster's landing burn aims at the
  catch point, rolls its pins towards the arms and cuts off on pin contact. Fidelity flag
  `FlightSession::set_towers`; `FlightSession::place_body` places a vehicle freely (scenarios).
  Game: `game/world/tower_view.gd` draws the tower from `FlightSim.site_tower`, arms from `tower_state`
  (render_check `--tower_arms_deg=`); a new `class_name` needs one `godot --headless --import`.
- Booster return (M6, `gnc/booster_return`): `FlightSession::set_booster_return(true)` gives the separated
  booster its own guidance (flip, Lambert boostback with drag-aware aim-point shooting, coast: RCS impact
  trim above ~2 kPa dynamic pressure, below it grid-fin steering - angle of attack found by bisection on
  the aero model (for the blunt tail-first booster the tilted axial drag dominates: the force points to the
  side the top leans to) -, landing burn (explicit fallback 13 -> 3 engines; SCvx 13 -> 5 -> 3): 1-D burn simulation
  for ignition, ZEM/ZEV vertical to the gate, then the approach profile sink = 1 m/s + height / 3 s, braking-profile
  divert). It uses its own faster attitude controller with grid fins enabled. Files: `booster_return.cpp` (parameters,
  `update` state machine), `_prediction.cpp` (impact / landing-burn / MECO predictions, drag estimate), `_burns.cpp`
  (entry burn, explicit ZEM/ZEV landing burn), private `booster_return_detail.hpp` (shared constants and helpers).
  The attitude controller limits the rate command to what the available angular acceleration (gimbal or
  RCS, plus fins) can still stop. Diagnostics: `STARSIM_VERBOSE=1`, `STARSIM_DEBUG_LANDING=1`,
  `STARSIM_DEBUG_HOLD=1` with test "[booster]".
- SCvx landing planner (M6, Plan 3a; flown since Plan 3b): `gnc/descent_*` plans the landing burn in
  `descent_phases` = 3 engine phases (13 -> 5 -> 3; 10 nodes per phase, neighbours share a node)
  in the plan frame (origin pad centre on the ground, x up, y east, z north; upright = identity attitude):
  `descent_model` (14-state rigid body, tabulated air/drag, Coriolis + centrifugal, numeric Jacobians;
  `make_descent_model` builds it from the booster on the simulation thread), `descent_discretization` (FOH +
  RK4 sensitivities), `descent_problem` (convex subproblem in scaled deviations: thrust/gimbal/tilt/rate/glide
  cones, catch or sea-abort goal with tower keep-out; a shared phase node has the next set's bounds, engine
  switches have sink windows, the last one is calm),
  `descent_planner` (`plan_descent`: penalised trust region, warm start, converged when the virtual control is
  small, the step or final mass has settled and the nonlinear interval defects are < 5 cm). Uses the reusable
  `optim::SocpSolver`. Nominal block 3 case: 6 iterations, ~0.5 s fresh; re-plans 1-4 iterations, < 0.5 s. In flight (Plan 3b): `gnc/descent_snapshot`
  (sim-thread snapshot; the worker never calls SPICE), `gnc/descent_replanner` (worker thread, fixed sim-time
  latency), `gnc/descent_tracking` (plan reference sampling), `gnc/tvc_lateral` (LQR gains), `gnc/scvx_landing`
  (first plan, re-plans, abort to the sea after repeated unreachable catches, commands incl. a body torque);
  `sim/landing_outcome` (shoreline `[site.sea]`, "Wasserung"); tower `CatchPhase::Aborted` (arms stay open).
  Diagnostics: `STARSIM_DEBUG_SCVX=1`, `STARSIM_DEBUG_SCVX_SOLVER=1`, `STARSIM_DEBUG_CATCH=1`; `STARSIM_DUMP_SCVX=<dir>`
  + hidden `[scvx-diag]` replay one plan request. Re-plans may draw on half the propellant reserve at a penalty,
  aborts on all of it (not the ship); a converged plan that tilts the final set > 3 deg beyond its soft limit counts as
  unreachable where there is a sea (Task 2b, "Landing planner: propellant floor and gimbal allowance").
- Grid fins (`aero/grid_fin`, `[stage.grid_fins]`): flat-plate style normal force from the local flow at
  each fin (incl. omega x r: aerodynamic damping), slope/limit over Mach from a table (transonic choking).
  Actuators: deflection and rate limits (`Vehicle::update_grid_fins`). The controller allocates torque to
  the fins first (minimum-norm least squares), the remainder to gimbals/RCS, using the air data of the
  last tick (`FlightBody::air_data()`); fins are used only when `AttitudeController::Gains::grid_fins`.
- Ship entry (b1 of `docs/superpowers/specs/2026-10-13-ship-return-design.md`): belly first. Hull =
  `SlenderBodyAero` with the ship's cross-flow table `aero/ship_crossflow.csv` [D] (used when the ship flies alone;
  shaped like a cylinder in cross-flow: transonic rise to 1.9, 1.5-1.6 hypersonic). Four flaps (`aero/flap`,
  `[[stage.flap]]` [E] aft 2 x 48 m^2, forward 2 x 24 m^2 moved 20 deg leeward; Newtonian flat plates,
  `aero/flap.csv`; 0 deg folded leeward, 90 deg broadside; 20 deg/s). `AttitudeController::Gains::flaps`: the flaps
  cancel the modelled aero moment (hull + flaps) and steer by a Newton step on their linearised moments
  (minimum-norm, per-flap clamp, null space drawn to mid-range), the rest goes to RCS. `gnc/entry_attitude`
  builds the setpoint from angle of attack and bank (belly at `[stage.aero] belly_azimuth_deg`). Nominal alpha
  over Mach `aero/ship_entry_alpha.csv` [D] (55 deg hypersonic, 50 near Mach 8, 60-80 towards the belly flop).
  Ship dry CG 23.516 m [D] since the header tanks (was 28 m, the trim requirement: at 20 m it sat ~9 m behind the
  hull's centre of pressure): it keeps the validated landing-mass CG, dry + 35 t at 25.50 m from the ship's base.
  Entry control: 0.5 rad/s and bank changes <= 1 deg/s (0.8 rad/s limit-cycles in roll with 20 deg/s flaps;
  faster bank changes depart in alpha). Validation: replay of flights 12-14 (bank follows the recorded altitude)
  - speed over time above Mach 3 within 1.6 / 2.3 / 4.9 % rms, below Mach 3 over altitude 6.0 / 5.2 %,
  belly-flop speed within 2-3 % (test "Ship entry reproduces the webcast telemetry of flights 12-14").
  Corridor (`[stage.heat_shield]`, judged while the ship flies alone): Sutton-Graves stagnation heat rate
  (`aero/entry_heating`, nose radius 6 m [E]), heat load, aerodynamic load (thrust not counted); beyond a limit the
  ship breaks up ("Hitzeschild versagt" / "Strukturüberlast", outcome "zerbrochen"), warning from 80 %; skip-out
  event ("Abprall": below 80 km, then above 100 km). Heating is integrated in `step()` only. Entry mass in tests:
  dry + full header tanks (`landing_propellant_kg`, 35 t [E]) + main-tank residue; `config::fill_tanks` fills the
  headers first. The headers are filled at launch and feed only from the flip, so a returning ship never carries less
  than 35 t: below that the CG moves AFT as the ship gets lighter (11 t: 24.16 m, the pitch trim at peak q is lost
  below ~24.5 m; with the old dry CG 28 m in the main tanks it moved forward) - the entry campaign once drew 11-39 t
  and five cases departed in pitch. Campaigns: entry 35 t + 0-28 t residue, catch gate 35 t + 0-3.5 t.
  Propellant settling (header-tank plan, `vehicle/liquid_settling`, fidelity flag
  `FlightEnvironment::propellant_settling`, on): oxidizer and fuel lie below a surface perpendicular to the apparent
  gravity (minus the specific force from the first RK4 stage), lagged by tau = sqrt(2 D / a) in [0.5, 60] s and held
  in coasts (< 0.01 m/s^2); per tank a tilted-cylinder table (centroid and variances). At entry attitude the residue
  lies forward and on the belly (64 t: CG +0.95 m forward) - this fixed the aerocapture departure (below). Engine,
  grid fin and flap moments use the axial CG [E] unless `FlightEnvironment::actuator_moments_about_cg_point` (off; the
  full CG point moved the Falcon 9 / Crew Dragon ascent apoapsis 210 -> 223 km - turning it on needs its own
  validation over the ascent and return campaigns). Main-fed burns check the main tanks only
  (`Vehicle::main_propellant_kg`: lunar return plan, aircatch capture burns); the headers carry the landing.
  The liquid does not turn with a round tank (no roll inertia; wall pressure through the axis): moments act in roll
  about `MassProperties::roll_offset_m` (the hull's parts only), in pitch / yaw about the full CG. Before that rule
  the aero normal force times a 4.4 t residue swinging ~0.1 m sideways gave up to 0.05 MNm of roll - the ship's whole
  RCS roll authority (1.5 kN [E] thrusters) - and lunar-return seed 8 lost its landing burn. Tanks have flat ends in
  this model (real domes would keep a small residue in the sump); the RCS allocator still uses the full CG point.
  The ship alone carries the stack's axial drag table [E] (`ascent_drag_table`; before b2 a ship built alone had none).
- Ship entry guidance (b2, `gnc/entry_guidance`, `gnc/entry_predictor`, `gnc/entry_aero_table`): the onboard computer
  predicts the rest of the entry as a 3-DOF point mass (Earth-fixed, J2, US76, the ship's trimmed C_D A / C_L A over
  Mach and alpha with flaps in pitch trim, online drag / lift scale factors from the accelerometer
  `FlightBody::aero_acceleration_body_mps2()`; pure, no SPICE) every 2 s and solves the bank magnitude for zero range
  error (Illinois); heat-rate, load and dynamic-pressure peaks over 90 % of a limit cut the bank back. Bank reversals are
  timed by the predictor (hold vs reverse-now end on opposite sides). Bank rate 1 deg/s above Mach 10, 3 deg/s below
  Mach 5 [E]. From Mach 5 (latest Mach 3, once a reversal has settled) the terminal phase ("Endanflug") steers the
  lift in the velocity frame relative to the alpha schedule (Levenberg-Marquardt steps on the predicted gate miss,
  starting from the entry's bank); the gate ("Tor") is 800 m [E] above the pad, where b3 flips (flight 13 relit at
  ~0.95 km [D]; at 950 m campaign case 4 misses by 212 m). `FlightSession::place_entering_ship` (scenarios),
  `entry_status` (setpoint; bindings telemetry `entry_*`, HUD lines). Campaign `[ship][guidance][validation]`
  (12 dispersed entries to Starbase, truth aero +-10 %): 11 within 0.11 m, case 4 marginal (roundoff moves it between
  ~0.1 and ~1.2 km: open; the test holds it to < 1.5 km since d1, 0.38 km there), belly <= 30 deg off down at the gate; load limit held.
  Tests use `predictor_j2 = false` (the scenario Earth has central gravity; J2 alone shifts a long glide ~2 %).
- Ship flip and catch (b3, `gnc/ship_landing`): at the gate the three sea-level Raptors relight at 80 % [E], the aft
  flaps fold and the forward flaps go broadside until the tilt is < 30 deg (the tail drops while the engines spool);
  the flip takes ~4 s (flights: 2-3 s; Raptor start-up 3 s [E]) and ~8 t, and pushes the ship ~100 m along its nose -
  the gate lies back along the pad -> tower line with the nose to the tower, so the side pins (0/180 deg) lie across the
  arms without a roll. Once upright (< 15 deg, < 0.2 rad/s) SCvx plans (`ship_landing_parameters()`) fly 3 -> 2 engines:
  the belly engine shuts down at 20-40 m/s sink [D: flight 14 video, two engines from ~200 m], the leeward pair lands
  (layout [D] from the flight 13 splashdown video; its thrust centroid lies in the symmetry plane). Thrust of an
  asymmetric set goes through the CG along an axis tilted against the hull (~1.5 deg for the pair): plan states carry
  that thrust-axis attitude (`gnc::thrust_axis_body`), else the LQR (no integrator) holds a ~2 m offset. Ship LQR
  effort 0.03 of the gimbal authority [E] (0.1 until the ship catch robustness plan, see below), attitude controller with
  `Gains::actuator_delay_s` 0.75 s (gimbal slew in the braking; no overshoot). Flaps folded in the burn.
  `FlightSession::place_landing_ship` (scenarios); the entry hands over at the gate. Test from the gate: caught 0.01 m,
  0.94 m/s sink, 5.7 t left; campaign `[ship][catch][validation]` (+-10 m, +-10 m/s, propellant 35 t + 0-3.5 t
  main-tank residue, +-5 deg alpha / bank, belly engine failed in the flip): 20 of 21 caught, the engine-failure case aborts upright (none lost).
  Gate lead: `gnc::ship_entry_target` puts the entry's target 220 m [E] back from the catch axis on the line tower -> pad
  with `EntryTarget::nose_direction_itrf` to the tower. Only that approach is robust: the arms stand across the line,
  a burn arriving from the side (azimuth 45/225 deg) runs into an arm at arm height; flip-push 20 deg off the line
  already ends there. The terminal phase turns the nose once the fall is steep (basis frozen, -70 deg): the steering
  turns to the nearest edge of the nose sector at fraction 0.2 (alpha ~86 deg), the lift direction turns about the air
  velocity at 3 deg/s [E] (a shortest-path attitude slew passes through low alpha: ~1 km miss) with the steering held,
  then Levenberg-Marquardt steps in sector coordinates (fraction 0.02-0.5; the sector narrows from +-30 deg 4 km above
  the gate to +-10 deg at it [E]). The predictor flies the same turn at its rate (an instant turn hid the lift swept in
  it: 0.4-0.8 km overshoots). The commanded attitude slews at 1.5x the turn rate and its rate is fed forward to the
  entry controller (`EntryCommand::rate_gcrf_radps`; without it the PD loop lagged the turn by ~18 deg of alpha), which
  has an actuator delay of 0.5 s [E] (2x slew or 5 deg/s set off a roll limit cycle on the flaps: departures). Session
  test from Mach 2.6, 28 km west: gate < 1 m off, caught 0.02 m, 0.95 m/s, 2.1 t left. Energy-short terminal phase (36 km
  from Mach 2.6; not seen from orbit, the deorbit plan arrives with energy to spare): the gate is hit (0.3 m), but the fall
  turns steep too late for the nose turn (137 deg off the tower direction): the flip pushes away from the tower, the burn
  aborts and the ship ditches at sea (1.4 t left) - safe, no catch. Ship catch propellant margin:
  5 of 21 dispersed gates arrive with <= 0.7 t (gate +-10 m/s: up to ~39 m/s across after the flip, braked high inside
  the glide cone); campaign case 12 (seed 13) reached the catch point before the arms had closed, ran dry and dropped
  ~1 m onto the arms at 4.5 m/s (judged "gefangen"); with a partly empty header (the old -5 % draw) it dropped at
  5.5 m/s ("verfehlt"), with full headers it is caught normally. Refuelling f3 (settling pushes before TEI and the Earth
  braking, ~0.45 t each) shifted the lunar-return gates by ~1.4 s / 20 m: seed 1 left the flip with ~27 m/s across, the
  lateral phase saturated at its 15 deg tilt, overshot ~35 m, hovered and ran the headers dry -> the lateral phase's tilt
  limit is 20 deg [E] since (seed 1 caught, 3.9 t left; `[ship][catch]` unchanged at 20 of 21). Ship catch robustness
  (2026-10-21, plan `docs/superpowers/plans/2026-10-21-ship-catch-robustness.md`): the numbers above for the remaining
  f3 miss (25 m short at 29 m/s across) were seed 12's; the replayed seed-18 gate (`[ship][catch][lunar]`, printed GCRF
  state with the flight's truth factors) showed a different cause: its flip ends 148 m from the axis at 32 m/s towards
  it and the first plan converges, but the lateral LQR (its model has no actuator delay, 0.75 s on the ship) out-tilted
  the plans (hull 0.23 vs reference 0.18 rad), braked ~4 m/s too much across, the re-plans jumped between a 9 s and a
  23 s final phase and stopped converging near the 3 -> 2 switch, the catch aborted and the unconverged abort plans
  swung the hull to 45-54 deg. Lateral effort on the replay: 0.1 / 0.07 miss, 0.05-0.02 catch (17 plans, none failed,
  axis on the reference to ~0.005 rad) -> 0.03. A braking flip (thrust against the cross speed before the hand-over)
  was designed in the spec and dropped: no failing case needs it.
  Upright abort (`gnc::upright_descent`, a deterministic law: tilt <= 15 deg, away from the tower inside 60 m, sink
  profile sqrt(2^2 + 2 * 0.5 * (a_max - g) * h) with its deceleration fed forward, never below the engines' minimum
  thrust): it takes over an abort with no converged abort plan 2 s after the abort, an abort leaning beyond 35 deg, an
  abort whose usable propellant falls below 1.3 x a vertical stop's need (`upright_stop_propellant_kg`), and a flip
  whose first three plans reach neither the catch nor the sea. Pilot abort 1 s into the flip: the abort plans flew
  ~460 m for the water and ran the headers dry (water at 9.5 m/s, 18.7 deg); now water at 2.85 m/s, 14.9 deg, 1.7 t
  left. Return result `sim::ReturnResult` (caught / upright abort: contact tilt < 20 deg and sink <= 5 m/s [E] / lost),
  `FlightSession::ship_return_result`. Open: a low-propellant abort (21-23 t at the gate) is carried by the flip push
  next to the tower; the descent then starts ~22 m from it at ~200 m / 42 m/s and strikes it - an abort-planning
  problem (keep-out with unconverged re-plans) for a later plan. Campaigns: direct 21 of 21, aerocapture 21 of 21.
  Diagnostics: `STARSIM_DEBUG_FLIP=1` (also `[upright]` lines), `STARSIM_DEBUG_GATE=1` (gate and flip-end state
  relative to the catch axis), `STARSIM_DEBUG_BURN=1`, `STARSIM_DEBUG_ENTRY=1` with the SCvx / catch flags;
  hidden test "Ship catch single case" with `STARSIM_SHIP_SEED=<1-20>`.
- Ship return from orbit (b4, `config/missions/ship_return_starbase.toml`, `config::load_ship_return_mission`):
  `FlightSession::plan_ship_return` (game key R) plans the deorbit burn on the onboard computer (`gnc/deorbit_planner`:
  coast samples over 48 h, passes = closest approaches of the ground track; per pass the burn epoch is bracketed on the
  entry predictor's downrange error and burn epoch + retrograde delta-v are shot (Newton) to the 120 km entry interface
  at -1 deg with the prediction at a nominal 50 deg bank (both signs averaged) ending at the gate; the first pass within
  60 km crossrange [E] is taken; refused without the landing reserve or without a pass). The burn flies on
  `[stage.recovery] deorbit_engine_groups` - the ship: the belly sea-level Raptor alone (user 2026-10-13), the hull turned
  so that its thrust axis through the CG (`gnc::thrust_axis_body`, ~3 deg off the hull) points along the burn; the burn
  executor senses along that axis (`FlightSession::execute_burn(node, groups)`; it flies on the named groups only - no
  RCS fallback, the deorbit burn is far above the RCS's reach). Any burn more than 30 min [E]
  (`FlightSession::burn_drift_lead_s`) + the 120 s preparation away waits in free drift (no RCS, coasted at any rate);
  the ship then turns to the burn attitude on the slow coast controller (0.5 deg/s) and holds it until the preparation
  (l5 Task 9b: holding the burn attitude through the aircatch's 93-108 h wait against the gravity gradient spent 6-8 t
  of RCS gas and left too little for the landing; drift: ~0 t, the turn ~1 t). Onboard tracking of a far deorbit (l5
  Task 10): planned more than 4 h ahead, it is planned again every 24 h [E] and 3 h before its burn with the orbital drag
  `DeorbitParameters::drag_area_per_mass_m2pkg` (C_D A / m, US76 air co-rotating) that
  `gnc::estimate_drag_area_per_mass` solves from the arc since the first plan (along-track lead, secant); a re-plan may
  take a pass up to 72 h [E] beyond the old one. Drag-free, a 4.7-day coast from the aircatch orbit put the pass ~80 km
  off its plan (out of the 60 km reach; gates 1,300-4,900 km short); tracked: C_D A / m 1.9-2.5e-3 m2/kg, later
  re-plans move the burn by 0-16 s, the daily pass drifts ~40 km a day. `AutopilotState::Return` ("Rückkehr")
  coasts to 300 s before the entry interface, then the entry guidance (b2) and the flip and catch (b3) take over.
  From 200 km / 26.5 deg: burn after ~21-22 h (one pass a day passes Starbase within reach), 65-78 m/s, 5.6-6.8 t.
  Campaign `[ship][mission][validation]` (orbit phase 0-315 deg, orbit 190-230 km, 42-48 t, deorbit execution
  +-1 % / 0.5 deg via `FlightSession::apply_velocity_error`): 14 of 14 caught, gate ~1 m, 8.9-13.9 t left (~40 s each).
  Diagnostics: `STARSIM_DEBUG_DEORBIT=1`; test "Ship return single case" (hidden, `STARSIM_RETURN_PHASE=<deg>` or
  `STARSIM_RETURN_DISPERSED=<i>`, with `STARSIM_VERBOSE=1`).
- `FlightBody::resting()`: carried by the ground after a gentle touchdown; resting bodies coast analytically
  (follow the Earth), impacted/resting bodies do not block coast segments. A freely falling body below 5 km above its
  ground is ticked (a coast segment would fall through the ground before its single contact check).
- Falcon 9 (milestone c, `config/vehicles/falcon9_block5.toml`, Block 5 only): octaweb groups `center` / `pair` /
  `ring`, 4 grid fins, cold-gas RCS, `[stage.landing_legs]`; sites `ccsfs_slc40` (launch, no tower) and `ccsfs_lz40`
  (`kind = "landing_zone"`, ~340 m east of the pad [E], 85 m pad [S]). Evidence: our webcast telemetry of CRS-35 (RTLS,
  LZ-40) and Starlink 15-25 (drone ship) - same round-dial layout as Starship (`webcast_telemetry.py clean --vehicle
  falcon9`: stage 1 left / stage 2 right after separation, G-FORCE and the stage-1 engine icon + propellant gauge
  before; the icon replaces the speed dial in the entry and landing burns). Derived [D]: first-stage propellant 425 t
  (decision 2026-10-14: the gauge falls 0.63 %/s at full thrust - nine Merlins at their 845 kN rating burn 2.75 t/s; the
  telemetry fixes only capacity x mass flow: the alternative, the commonly cited 395.7 t with the engines at ~93 % all
  ascent, keeps no official figure; `tools/telemetry/mass_estimate.py --area-m2 10.52`; with 425 t the simulated G
  matches the dial before max-Q), second-stage propellant 106.7 t, landing propellant 12.1 t (gauge 2.8-2.9 % at
  landing-burn ignition on both flights; touchdown 1.7 % = 7.2 t), MECO gauge 66 t RTLS / 47 t drone ship, descent drag
  table `aero/falcon9_descent.csv` (unguided replay from after the entry burn: 0.9 / 1.5 % rms, test "Falcon 9 descent
  drag reproduces ..."). Burns seen: CRS-35 entry burn on the centre engine only, landing burn 1 -> 3 -> 1; Starlink 15-25
  entry burn on 3, landing burn on 1 (~40 % throttle by the gauge). The drone-ship descent stays ~2 km higher than a
  zero-lift fall (grid-fin lift). Ascent (since ascent Task 2): full thrust (no `max_ascent_throttle`: the 425 t
  tanks' gauge flow is the rated flow) under the Mach-dependent q limit `max_dynamic_pressure_pa` [D] (transonic hold
  29.6 kPa, 35.6 kPa supersonic: section "Ascent trajectory and staging"); the CRS-35 schedule (bucket ~72 % T+50-61 s,
  then ~95 %) is gone (`STARSIM_DEBUG_ASCENT_G=1` prints the simulated G); the kick is planned (ascent Task 3-5).
  Validation against CRS-35 (ascent Task 6, `crs35_ascent.toml`, dials' time base 3.42 / 5.87 s): -9.5 % at T+30 s,
  +2-5 % after max-Q, MECO T+132.7 s / 58.2 km / 6596 km/h vs 134.6 / 57.4 / 6490, SECO T+508 vs ~512 s (section
  "Ascent trajectory and staging"). The upper stage's plane steering nulls out-of-plane speed and offset together
  (ZEM/ZEV) and the terminal v_go removes the rest; inclinations of steep planes are judged against the true equator of
  date (GCRF elements carry ~0.15 deg of precession by 2026). `FlightSession::launch_from_pad` also takes a loaded
  `VehicleConfig` (mission payload; Dragon flies without a fairing); every launch starts with the autopilot off (no ascent
  guidance, burn or ship return of a previous flight; `set_booster_return` is a session setting and stays).
- Landing legs (c2, `contact/landing_legs`, `contact/landing_surface`, `sim/leg_landing_sim`): `Vehicle::deploy_legs`
  (sticky, `deploy_time_s`; `FlightSession::deploy_legs` / `set_leg_deployment`); once the legs move,
  `LegLandingSim` owns the body's ground contact: four feet as spring-dampers in series with plastic crush cores
  (`stroke_m`, `crush_force_n`), friction capped at what stops a foot's slip within a tick (the small roll inertia made
  plain regularised friction limit-cycle). Surface: the site's ground plane with its shoreline, or a deck over the sea
  (`FlightSession::set_landing_surface`). Judgement (`FlightSession::leg_touchdown`): "gelandet" (>= 3 feet, engines
  off, at rest 2 s -> `FlightBody::hold_landed`), "umgekippt" (> 30 deg on a foot), "Aufprall" (core used up, base on
  the surface, legs not fully out), "Wasserung". Drop tests `[legs]`: 1/3/6 m/s land (6 m/s crushes 0.36 m), 10 m/s and
  25 deg at 2 m/s are lost. Diagnostics: `STARSIM_DEBUG_LEGS=1`.
- Falcon 9 RTLS (c3, `config/missions/falcon9_rtls.toml`, `config::load_ascent_mission`): `FlightSession::set_landing_zone`
  (after `launch_from_pad`; the legs land on the zone's ground) makes a `"leg_landing"` stage return to the zone
  (`returns_after_staging`, `return_site` with a `gnc::PadTarget`). The return guidance is generic over the vehicle:
  `gnc::booster_return_parameters(stage)` takes engine sets, return flight time, boostback loss fraction, entry burn
  and the landing-burn profile from `[stage.recovery]` (Super Heavy has none of these keys: defaults unchanged).
  Phases Flip -> Boostback -> Coast -> EntryBurn ("Wiedereintritt", Falcon 9 only: retrograde on the centre engine from
  56.6 km for 125 m/s [D], leaning <= 5 deg towards the impact miss; it ends after 5 s [E] without thrust - engines out -
  and the landing burn still comes) -> Coast -> LandingBurn. The impact prediction
  carries the entry burn still ahead (a finite burn since c4). The landing burn flies SCvx plans with the goal `DescentGoal::Pad`
  (terminal state of a catch at leg-contact height, sink 1 m/s, no corridor, no exponential approach: one Merlin at
  minimum throttle exceeds the weight), 1 -> 3 -> 1 engines with switch windows 230-270 / 140-190 m/s [D]; the legs
  deploy 5 s [E] + deploy time before the planned touchdown, the engines stop on leg contact or 0.5 s after the plan's
  end; an unreachable zone aborts to the sea ("Wasserung"). `LandingOutcome::TippedOver` ("umgekippt") comes from the
  leg judgement. Validation: from ignition (3.3 km, 300 m/s, 12.1 t) lands 0.01 m off at 0.88 m/s; campaign
  `[falcon9][rtls][validation]` (+-10 m, +-3 m/s, +-5 % propellant, +-3 deg, truth hull aero +-10 % via
  `FlightSession::set_aero_dispersion`) 21 of 21 within 0.01 m, <= 0.99 m/s, >= 5.7 t left; full mission: MECO T+141 s,
  entry burn T+356 s, landing burn T+423 s at 3.0 km / 287 m/s / 11.7 t, touchdown T+459 s, 0.00 m off (CRS-35: 137 /
  355 / 420 at ~3.2 km, ~300 m/s / 442 s). `return_flight_time_s` 260 s and `boostback_loss_fraction` 0.12 [D] were
  identified on that mission with the 395.7 t stage (kept). CRS-35's gauge shows ~66 t at MECO against our 55 t (not
  followed: the Falcon 9 gauge drops without burns, see the drone ship).
- SCvx landing aborts (all vehicles; the pilot's `abort_landing` too): a first plan that finds the target unreachable aborts at once; `failures_before_abort`
  (2) re-plans with a virtual control above the stall threshold, or `unconverged_before_abort` (4) unconverged re-plans of
  any kind (a marginal first plan the vehicle could not follow), give the target up and plan for the sea.
- Falcon 9 drone ship (c4, `config/drone_ships.toml`, `config/missions/falcon9_asds.toml`): a drone ship is a landing zone
  with a deck (`sim::drone_ship_site`: deck 91 x 52 m [S], 3 m freeboard [E], stationary and level; `sea` marks the water
  beside the long edge for the abort); the mission's `[drone_ship]` places it at the nominal impact point of our ascent
  [D] (hidden test "... nominal impact point ...": drag + finite entry burn, 10 s after MECO). `sim::start_ascent_mission`
  sets a session up from an `AscentMission` (payload, pad, landing zone or drone ship, booster return, autopilot).
  `[stage.recovery.drone_ship]` overrides the RTLS profile when the landing zone has a deck (`StageConfig::return_profile_for`,
  `FlightSession::return_parameters`): `boostback = false` (the guidance starts coasting, turns engines first on its RCS;
  MECO plans run-down + entry burn + landing propellant only: 24.7 t), entry burn on centre + pair from 66 km for
  880 m/s [D] at 0.72, landing burn on the centre engine in all three phases [D] (`landing_phase_max_tilt_deg` 20).
  What the slanted approach (~90 m/s across at ignition) needed, all generic: the impact prediction flies the pending
  entry burn as a finite burn with its lean (`PendingBurn`; the impulse model was 3.5 km off); the entry burn solves its
  lean by shooting on that prediction (`entry_max_lean_rad` 5 deg); leg landings aim the ballistic impact beyond the pad by
  the landing burn's lead (`landing_burn_lead_itrf`: the retrograde burn bends the fall to the vertical and stops short);
  SCvx landing burns feather the grid fins into their local flow (`AttitudeController::feathered_grid_fins`: neutral fins
  acted as a tail the planning model does not know - Super Heavy too, catch campaign still 21/21); the return guidance
  estimates the drag online from the accelerometer (`FlightBody::set_accelerometer`, `drag_scale()`). Validation: full
  mission (kick 13 deg: MECO T+152 s, 64.8 km, 8373 km/h, flight path 24.7 deg vs Starlink 15-25 T+149 s, 60 km,
  ~8000 km/h, ~25 deg; 16.5 t to 200 x 281 km - 17 t arrives with 0.3 t left): entry burn T+379-399 s (T+364-386), landing
  burn T+476 s at 3.0 km / 271 m/s / 12.0 t (T+476 s, ~3.0 km, ~270 m/s, 11.9 t), landed T+515 s (T+499 s) 0.01 m off;
  campaigns `[falcon9][asds][validation]` from ignition (as RTLS) and from MECO states (+-200 m, +-5 m/s, +-5 %, aero
  +-10 %; two halves) 21 of 21 each; a deck out of reach ditches beside the ship. Our MECO leaves 25 t; the gauge shows
  ~47 t, but it falls from 47 t at MECO to 25.9 t at the entry burn while the speed stays ~8000 km/h (no burn between) -
  our MECO matches the gauge after that drop (physics first, decision 2026-10-14). Known gap: landing burns end at the SCvx reserve (5 t [E], Super Heavy's value) after ~36 s: the
  gauge shows 7.2 t at touchdown after a ~23 s burn of ~4.9 t, and a 7.2 t planner reserve is infeasible from the recorded
  ignition state in our model (our burn decelerates harder early - unmeasured subsonic and power-on drag [E]).
  Diagnostics: hidden test "Falcon 9 ASDS single MECO case" with `STARSIM_ASDS_SEED=<101-120>`.
- Game interface for Falcon 9 (c5; the drawing and UI are specified by the user first): `FlightSim.start_mission(id, tdb)`
  (ascent missions of `config/missions`), `mission_site_id`, `landing_zone()` (pad radius or drone-ship deck, sea height,
  site frame in ITRF), `body_legs(i)` (geometry, deployment, touchdown), `body_grid_fins(i)` (geometry, deflection);
  `return_state(i)["legs"]`. Covered by the smoke test. The bindings build with C++ exceptions on
  (`GODOTCPP_DISABLE_EXCEPTIONS OFF`): godot-cpp's default turned every core config error into an engine crash.
- RCS (d1 of `docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md`): individual thrusters from `[stage.rcs]` pods
  (Super Heavy 12 x 3 kN at the forward dome, ship 26 x 1.5 kN fore and aft (two of each direction per pod: the pre-d1 per-axis estimate the ship campaigns were
  validated with - roll 20, pitch / yaw 117 kNm - available on all axes at once; with one of each, roll and pitch shared
  their thrusters, the entry's nose turn ran out of torque and catches from entries hit the arms) incl. two aft ullage
  thrusters along +x,
  Falcon 9 8 x 450 N at the interstage and 8 x 200 N on the second stage; layouts [E]), on/off with a fixed thrust
  (constant pressure; user 2026-10-14: no continuous mode, the autopilots fly on the pulses). Per tick
  `Vehicle::update_rcs` allocates the commanded torque / force fractions (`propulsion/rcs_allocator`: box QP solved by
  an exact active set on the 6 x n effectiveness about the current CG, rebuilt when the CG moves > 1 cm or a supply runs
  dry) and `propulsion/rcs_modulator` turns the duties into pulses of >= 20 ms (4 ticks). Torque comes first; orbit
  manoeuvres set `VehicleCommand::rcs_keep_force` (session Burn state): balanced layouts (pure couples on every axis: the
  ship) then keep the net force what was asked for, so turns do not translate the vehicle (the ship's saturated burn turn
  borrowed its ullage thrusters: +55 m on a 0.5 m/s burn; keeping the force in the entry's thin air instead cut torque
  and made catches from entries fail). One-station layouts (Super Heavy, Falcon 9) always turn with a push: they cannot
  translate in the coast, so the Falcon 9 entry burn removes the impact miss (user 2026-10-14): in the coast its ignition
  altitude is shot on the miss along the track (+-8 km [E]: an earlier burn shortens the rest of the fall, free of
  propellant and of speed at the landing burn), in the burn its length (+-25 % [E]) and lean across (<= 10 deg [E])
  trim the rest; MECO campaigns 21 / 21 within 0.04 m. A returning booster's coast slews (pitch / yaw on RCS) are limited to 2 deg/s [E]
  (a full-authority turn emptied the Falcon 9 nitrogen). Capability per axis (`max_rcs_torque_nm`, the fraction scale of the controllers) and per
  direction (`rcs_force_capability`) come from the geometry. Propellant: Falcon 9 N2 tanks (200 / 80 kg carved out of
  the dry masses; not in the gauge `propellant_kg()`), Starship / Super Heavy ullage gas from the main tanks (Isp 70 s
  [E]). The burn executor counts the realised RCS force while a translation is commanded. Not modelled: thrust rise /
  tail-off, short-pulse Isp loss, pressure blowdown.
- Crew Dragon (d2 of the same spec, `config/vehicles/crew_dragon.toml`, test file `core/tests/test_crew_dragon.cpp`):
  capsule + trunk, 12.5 t [S] (capsule dry 8.4 t [D], 2,562 kg MMH/NTO [S] in an RCS tank, trunk 1.5 t [E]). 16 Dracos
  (400 N, Isp 300 s, 10 ms pulses) [E layout]: 4 under the nose cone push aft (usable only with the cone open,
  `Vehicle::set_nose_cone_open`, `FlightSession::set_nose_cone`), 4 side pods (tangential +/-, forward) give pure couples
  on all axes; no lateral translation (docking, M9). 8 SuperDracos are an `abort_only` group: data only, never lit by
  the session (`ignite`, burn configurations). Payload vehicles: a mission's `[payload]` stacks a vehicle on the launcher
  (`config::stack_payload`); its stages are `payload` stages whose thrusters are blocked until it flies alone.
  `FlightSession::set_payload_separation` / `separate_payload`: released at cutoff + 30 s with a 0.3 m/s push [E]
  (`AutopilotState::PayloadSeparation` "Nutzlasttrennung", the stack holds its attitude meanwhile); the capsule becomes
  the controlled body in an inertial hold and opens its nose cone 600 s later [E]. Orbit attitude modes
  (`gnc/orbital_attitude`: Inertial, Lvlh, Direction, Sun; `FlightSession::set_attitude_mode`) are flown by
  `orbit_controller_`: slews <= 1 deg/s on every axis (roll too: `Gains::rcs_max_roll_rate_radps`; an unlimited roll of
  the capsule spent 3.5 kg on a 180 deg turn), hold deadband 0.5 deg / 0.05 deg/s [E]; any mode but Free keeps the
  session ticking (Free coasts). RCS burns fly along +x when the vehicle has thrust along it, else along -x (the hull
  turns half about body y), steered by the orbit controller. Mission `falcon9_crew_dragon` (SLC-40, 51.6 deg,
  200 x 210 km, RTLS to LZ-40): SECO T+503 s, release T+533 s, 200.0 x 210.8 km, 51.74 deg (GCRF elements), +18 m in the
  first 60 s, 0.42 kg in the first 10 min, first stage landed. Inertial hold after a 1.3 deg/s tumble: <= 0.71 deg,
  1.9 kg/h (pulse quantum: not grams per hour); LVLH trunk forward: <= 0.71 deg, 2.4 kg/h; Draco burns 1 / 10 m/s within
  0.002 / 0.014 m/s (5.0 / 43.4 kg, ideal 4.3 / 42.4), -x 2 m/s within 0.007 m/s.
- Crew Dragon entry (d3 of the same spec, plan `docs/superpowers/plans/2026-10-14-crew-dragon-d3.md`, tests
  `core/tests/test_capsule_aero.cpp`, `test_capsule_entry.cpp`): capsule aerodynamics `aero/capsule_aero` from
  `aero/crew_dragon_aero.csv` (modified Newtonian [D] by `tools/aero/gen_capsule_aero.py`: spherical-section shield
  R_n = 1.2 D [E], 15 deg sidewall, C_p,max from the Rayleigh pitot formula; [E] below Mach 2, subsonic C_A ~0.97;
  hypersonic C_A 1.68 at 0 deg, ~10 % above Apollo/Orion's measured values - Newtonian over-predicts flat shields; no
  rarefied bridging, no damping derivatives). Used once the capsule is the lowest stage (`Vehicle::capsule_aero`,
  `hull_aero`); `AeroResult::moment_body_nm` carries its pure moment. Lateral CG: `[[stage]] dry_cg_offset_m` [D] 9.6 cm
  (`MassProperties::cg_offset_m`, used by the aerodynamic and RCS moments only; inertia on the axis) trims the 10.5 t
  entry capsule at 12.5 deg, L/D 0.20 [E: spec], C_D 1.57. Entry control `gnc/capsule_entry_control`: the Dracos roll the
  lift about the air velocity (bank rate <= 10 deg/s [E], reversals through lift up) and damp pitch / yaw (deadband
  1 deg/s); the aero trim holds alpha; below 200 Pa the full trim attitude is held (a 90 deg / 2 deg/s tumble at 110 km
  is on trim by 1 kPa); ~31 kg of MMH/NTO per entry (Apollo ~30-40 kg). Entry guidance: `EntryGuidance(capsule, target)`
  with `EntryGuidanceParameters::capsule()` (`EntryAeroTable::capsule`: trim over Mach; no terminal phase; gate 5.5 km
  [S: NASA Crew-2, drogues ~18,000 ft]; reversals down to 500 m/s with a 0.5 km deadband; the bank is held below
  500 m/s - the bank from the vertical plane degenerates in the steep subsonic fall; load limit = crew 4 g [E]
  (`[stage.heat_shield] crew_load_limit_g`, `FlightBody::crew_load_warning`) x 0.9). An L/D 0.2 capsule peaks at 3.3 g
  lift up and 4.7 g at 60 deg of bank whatever the entry angle: under 3.6 g only ~0-45 deg of bank remain, so the nominal
  entry is -2.0 deg at 120 km with 30 deg of bank [E] (at -1.2 deg the range moves ~2400 km per degree of entry angle,
  at -2.0 deg ~1000 km). Splashdown zone `config/splashdown_zones.toml` (`pacific_san_diego` 32.6 N 117.8 W [E], off San
  Diego where Crew-10 / Ax-4 splashed down in 2025 [S], radius 2 km - user 2026-10-14). Deorbit: `gnc::plan_deorbit` with
  `DeorbitParameters::burn_thrust_n` (finite burn centred on the node along its fixed direction) and `entry_mass_kg` (the
  capsule after the trunk jettison: 45 km of downrange otherwise). `FlightSession::plan_capsule_return` (steps
  `CapsuleReturnStep`): Draco burn along +x (4 side Dracos, 1.6 kN), trunk jettison 120 s after it, nose cone closed
  120 s later [E; order [S]: SpaceX deorbits before the trunk jettison since the West Coast return], turn to the trim
  attitude, entry guidance `entry_lead_s` before the interface, gate; `place_entering_capsule` (scenarios). From 420 km /
  51.6 deg: 139 m/s / 598 kg after 37 h, burn 1058 s (published 7.5-12 min: the Draco layout is an estimate), interface
  1135 s after it, gate 0.58 km off at 166 m/s (597 km/h; published ~560), 3.5 g, 33 kg of Dracos in the entry. Campaign
  `[dragon][entry][validation]` (entry angle +-0.1 deg, mass +-3 %, truth aero +-10 % from the planned interface state):
  12 of 12 at the gate within 1.2 km, 160-178 m/s, <= 3.6 g. The gate is where d4 opens the drogues; until then the
  capsule falls on (bank 0, damped). Diagnostics: `STARSIM_DEBUG_ENTRY=1` (one line per guidance cycle),
  hidden test "Capsule entry single case" with `STARSIM_CAPSULE_SEED=<1-12>`.
- Crew Dragon parachutes and splashdown (d4 of the same spec, plan `docs/superpowers/plans/2026-10-14-crew-dragon-d4.md`,
  tests `core/tests/test_parachute.cpp`, `test_capsule_splashdown.cpp`): `[[stage.parachute]]` sets (`aero/parachute`: one
  canopy's drag area grows as t^2 through its reefing stages; massless canopy on a riser point on the axis - force along
  the point's air velocity, moment from the offset, omega x r damps the hanging capsule; no opening-shock factor, no
  pendulum body, no wind). Crew Dragon: 2 drogues 28 m^2 each [D: 53 m/s at 1.8 km, 119 mph S], reefed 35 % for 4 s, and
  4 mains 815 m^2 each [D: 7.0 m/s at sea level, 15-16 mph S], reefed 6 / 20 / 45 % for 4 s each (10.0 t capsule).
  Reefing and load limits [E]: the main limit 120 kN per canopy is 2x the nominal reefed peak (60 kN), so mains opened
  without drogues (~136 m/s) tear - with 150 kN they survived at 6.4 g. Opening loads per canopy are judged every tick
  (`Vehicle::record_parachute_load`, "Fallschirm gerissen": the set tears). `gnc/parachute_sequencer`: drogues at 5.5 km
  [S: 18,000 ft] once q <= 15 kPa [E], mains at 1.8 km [S: 6,000 ft] whatever the drogues did (backup; the drogues are
  released), mains released on the water; only descending flight deploys. `FlightSession` runs a sequencer for each body
  whose lowest stage has sets (a capsule alone, never stacked), the Dracos stay off once a set is out (user 2026-10-14),
  and the session ticks below the highest deployment height + 1 km (no coast past it). Splashdown: a body whose lowest
  stage recovers by `"splashdown"` rests on the water at any tilt up to `max_splashdown_speed_mps` 9 m/s [E: spec]
  (`FlightBody::touchdown_sink_mps`), outcome `SplashedDown` "gewassert"; faster is "Aufprall" (no separate mains-open
  flag: 9 m/s is out of reach without mains). Capsule return steps go on Gate -> Drogues -> Mains -> SplashedDown
  ("Bremsschirme", "Hauptschirme", "gewassert"); `FlightSession::splashdown_miss_m`, `parachute_sequencer(i)`.
  From the gate (166 m/s): drogues 5.50 km / 164 m/s, mains 1.80 km / 53 m/s 57 s later, water 205 s later at 6.9 m/s,
  peak 2.8 g. Return mission `config/missions/crew_dragon_return.toml` (`sim::place_capsule_return_mission`, 420 km /
  51.6 deg, RAAN 60 deg [D]: the first plane with a pass within reach in 48 h): burn after 37.1 h (1005 s), water 0.29 km
  from the zone centre at 6.9 m/s, deorbit ignition -> splashdown 50.5 min (Crew-10 2025-08-09: 54 min [S]). Campaign
  `[dragon][mission][validation]` (8 cases: orbit phase +-3 deg about the mission's, MMH/NTO +-10 %, deorbit execution
  +-1 % / 0.5 deg): 8 of 8 gewassert within 1.09 km, 6.9-7.0 m/s, <= 2.97 g. An ISS-like orbit passes the zone within the
  capsule's 40 km crossrange only every few days: of the orbit phases 0-315 deg (45 deg steps) only one has a pass in
  48 h, two in 96 h (45 deg of phase moves the ground track ~270 km at 32.6 N), so the campaign disperses about the
  mission's pass instead of over all phases (b4). Entry guidance (all vehicles, d4): a reversal due within the next cycle
  is flown only if reversing now ends closer across than holding (a capsule overshot +5.9 km vs -2.1 km held at 536 m/s
  and came down 5.8 km off). Webcast telemetry of a Dragon return is not used (published figures cover the tolerances;
  no free licence). Diagnostics: hidden tests "Crew Dragon return single case" (`STARSIM_DRAGON_CASE=<0-7>`) and
  "Crew Dragon return: the first feasible orbit plane". A capsule that impacts or breaks up ends its return as "verloren"
  (`CapsuleReturnStep::Lost`, autopilot off; test "A capsule return without mains ends lost, ...").
- Game interface for Crew Dragon (d5 of the same spec, plan `docs/superpowers/plans/2026-10-14-crew-dragon-d5.md`; the
  drawing and UI are specified by the user first): `FlightSim.start_mission` dispatches on `[mission] kind`
  (`sim::start_mission`: "ascent" default; "capsule_return" starts at the mission's own epoch - `mission_start_tdb_s()`,
  the world clock jumps there - with the deorbit planned; "ship_return" is refused: key R from orbit). `body_rcs(i)` (per
  thruster: stage, position, direction, thrust, firing, duty `Vehicle::rcs_duty()`, starved, blocked),
  `body_parachutes(i)` (state German incl. "Fallschirm gerissen", drag area, inflation, riser station, canopy direction
  against the riser point's air velocity `FlightBody::point_air_velocity_body_mps`, loads), `capsule_state(i)` (a capsule
  flying alone, not stacked: nose cone, trunk, return step, parachute phase, splashdown miss, outcome),
  `splashdown_zone()`, `set_splashdown_zone(id)`, `plan_capsule_return()`, `autopilot_status()["capsule_step"]`. Covered
  by the smoke test, whose check functions must run to their end (a GDScript error used to abort one silently).
- Lunar missions (spec `docs/superpowers/specs/2026-10-14-lunar-missions-design.md`), l1 reference bodies: vehicle states
  stay Earth-centred GCRF everywhere; `environment::ReferenceBody` (Earth: ITRF93 + WGS84 + US76; Moon: MOON_ME, sphere
  1737.4 km, vacuum) gives what is relative to a body. A `FlightBody` takes the Moon within 60,000 km of it and the Earth
  again beyond 100,000 km [E] (the NRHO's apolune stays inside; a body just placed counts as coming from the Moon);
  bodies on the ground keep theirs, and a ground impact keeps the wreck where it struck on its body. Air, ground contact,
  resting (it follows the Moon), telemetry (`reference_body`, altitude, osculating periapsis / apoapsis and inclination to
  the body's equator) and the session's tick-near-ground decision and LVLH holds use it. `FlightSession` takes a Moon
  orientation (5th constructor argument, `spice::SpiceOrientation::moon_mean_earth()`; the game's `FlightSim` passes it
  since l6); without it everything is relative to the Earth as before. Ground at the Moon: the sphere (LOLA tile: l2). Validation
  (`test_lunar_propagation.cpp`): 100 km LLO and a 3,000 x 70,000 km orbit one day Earth-centred vs a Moon-centred
  integration of the same forces (the DE440 Moon's acceleration as the frame's) 0.020 / 0.025 m; 200 Hz ticks 20 min
  5e-5 m. A Moon-centred `PointMassGravity` is a different model (its frame accelerates as point-mass Earth and Sun pull
  the Moon; DE440's Moon also feels the planets, Earth J2 and tides): 0.33 / 4.4 m in a day.
  l2 HLS and terrain (plan `docs/superpowers/plans/2026-10-15-lunar-l2-hls-terrain.md`): `config/vehicles/starship_hls.toml`
  (120 t dry [E], Block 3 tanks and Raptors, no heat shield / flaps / pins, ship RCS; 12 landing thrusters [E] 46 kN at
  40 m canted 15 deg - 1.5 lunar weights of the ~221 t landing mass, hover from 131 t; re-sized in l4, below -
  `landing_only`: never in burns or
  `ignite()`). Engine groups take `station_m` / `cant_deg`. Legs [E] fixed, six, footprint radius 12 m (user 2026-10-15):
  a vertical touchdown on a slope lands on the uphill feet and pivots freely through the slope angle (CG ~21 m up), so the
  slope limit is set by the stance - 8 m ~5 deg, 10 m 7 deg, 12 m >= 9 deg (NASA's Artemis limit ~8-10 deg); absorber
  1e7 N/m, 1e6 N s/m, crush 1.5 MN (no crush at 1.5 m/s on flat ground). Drop tests (1 m/s + 0.5 m): 0 / 5 / 8 deg land,
  25 deg tips over; static tip-over 26 deg. Terrain: `config/lunar_sites.toml` (default `mons_mouton_im2`, IM-2's landing
  point [S], 5,322 m above the sphere); `fetch.py` crops LOLA `LDEM_80S_20M` (20 m/px, polar stereographic) to
  20 x 20 km per site by one HTTP range request (61 MB of the 1.8 GB) and fetches the global `LDEM_16` (16 px/deg) -
  `data/terrain/`; the tile's orientation is checked against LDEM_16 (rms 40 m; mirrored 117-304 m).
  `environment::LunarTerrain` (tile inside, global grid outside, 1 km smoothstep blend; normals by central differences
  over 10 m) through `ReferenceBody::set_terrain` / `surface_height_m` / `surface_normal_fixed`;
  `FlightSession::set_moon_terrain`, `set_lunar_landing_site` (`sim::load_lunar_terrain`, `sim::lunar_site`). Leg landings
  work on any reference body (`sim::LegSite` with a tracking radius - the site's tile; outside it, or without a lunar
  site, `FlightBody`'s own ground contact, `set_legs_on_site`) and press the feet along the surface normal
  (`contact::SurfaceField`: `PlaneField`, `TerrainField`); Earth landings are bit-identical. FlightBody's ground at the
  Moon is the terrain (the sphere without one). Earth DEM tiles (e.g. Copernicus GLO-30) come with M8 (user 2026-10-15;
  `ReferenceBody::set_terrain` takes a `LunarTerrain` so far).
  l3 outbound (plan `docs/superpowers/plans/2026-10-15-lunar-l3-outbound.md`): `mission/position_targeter` (fixed-time
  position in the full model; needs a patched-conic first guess on long arcs - Newton from zero diverges over many
  revolutions; finite burns via `set_finite_burn` / `mission::apply_finite_burn`), `orbit/cr3bp` (CR3BP, fixed-period
  periodic orbits, instantaneous Earth-Moon rotating frame), `mission/nrho_reference` (9:2 southern L2 NRHO [S: Zimovan
  2017 seed] by multiple shooting in the full model: perilune anchors from the CR3BP, 4 patches per half revolution
  crowded to the perilunes, continuation revolution by revolution, free-coast fallback when a revolution's anchor fails;
  periods 6.25-6.71 d, perilune 2,861-3,670 km over the north pole, apolune 69,804-71,706 km [Gateway: 6.26-6.76 d, rp
  3,195-3,557 km, Davis et al. AAS 2020]; phase set by the first perilune - no Gateway phasing; only `first_perilune` is
  exact, use `perilune_epochs()` (true closest approaches), never `first + k P`), `mission/lunar_outbound` (LLO: TLI
  3,142 m/s impulsive / 3,182 m/s finite, LOI 856 m/s, ~1,147 t propellant used; NRHO via a powered lunar swing-by (user 2026-10-15: a
  direct transfer meets the NRHO perilune head-on, 3.5 km/s) after Kikuchi et al. ISSFD 2024: TLI 3,162 m/s, swing-by
  243 m/s at 100 km, insertion 224 m/s ~1.3 d later; MCCs, LLO trims, propellant check), `mission/lunar_orbit_transfer`
  (NRHO -> LLO 660 + 216 = 876 m/s; LLO -> NRHO arrives anywhere on the reference (user 2026-10-15) in one-day windows:
  725 m/s right after the LLO arrival, ~0.7 km/s within ~2.5 d or 21-30 d, 0.9-1.65 km/s in between - the NRHO plane
  turns ~86 deg per revolution with the Moon's orbit, the LLO plane stays inertial; NRHO station keeping at apolune
  when the predicted perilune miss exceeds 1 km [E]: 8.2 m/s per 3 revolutions planned, 1.1-1.4 m/s per burn flown).
  `FlightSession::plan_lunar_outbound` / `plan_lunar_orbit_transfer` (also from autopilot state Orbit),
  `AutopilotState::Lunar` ("Mondflug", `core/src/sim/flight_session_lunar.cpp`), `LunarStep`, `lunar_burns()`. The TLI is
  re-solved for the finite burn at departure (flown as an impulse the 7 min TLI missed the Moon: MCC 1 333 m/s, now
  3.4 m/s); LOI is timed on the periselene predicted at the last coast; burns too small for the vacuum Raptors fall back
  to RCS. Coasts are Sun-pointed with a re-aim every 6 h [E] on their own controller (0.5 deg/s slews, 0.002 deg/s rate
  deadband): ~0.9 t cold gas per day (free coast between: no disturbance torques); `gnc::SunHold` ends a hold that has
  not settled (1 deg, 0.005 deg/s) after 30 min [E] - the vehicle then coasts freely until the next re-aim instead of
  spending cold gas. Leads: trims and station keeping are planned at least the 120 s burn preparation + 60 s [E] ahead
  (an LLO trim then takes the next apsis after that); the TLI centre is never planned earlier than 30 min [E] after
  the plan (`FlightSession::min_departure_lead_s`; preparation + half the ~7 min burn), also for
  `earliest_departure_h = 0`. NRHO insertion and the LLO -> NRHO arrival burn match the reference velocity only; the
  position error at the perilune is left to the station keeping. `plan_lunar_outbound` refuses a vehicle already
  Moon-referenced ("Kein Mondflug: schon beim Mond"). Pilot-flown MCCs (user 2026-10-15,
  `pilot_corrections`): call 600 s before the MCC, `lunar_pilot_cue()` (remaining delta-v, attitude error, window),
  `complete_pilot_correction()`, the next MCC re-plans from the real state; game command since l6, display with the UI
  specification (M7). Missions
  `hls_lunar_llo` / `hls_lunar_nrho` (`kind = "lunar_landing"`, outbound only until l4; in the game since l6). Results: LLO 99.97-100.00 km after one trim; NRHO 0.58 km worst perilune offset over 3
  revolutions, NRHO -> LLO 99.97-100.03 km; campaign (TLI +-1 % and 0.5 deg, 8 / 8): MCC 1 183-311 m/s (a 1 % / 0.5 deg TLI error is ~60 m/s, grown ~5x by a
  day of coast), MCC 2 0.3-0.4 m/s, LLO 99.97-100.01 km, 1,177-1,194 t propellant used (not the mass in the LLO - ~573 t, see l4); NRHO worst perilune offset 0.47-2.88 km,
  1,124-1,150 t propellant used. Bug fixed on the way: the
  default Earth leg-site plane cut through LEO and wrecked the HLS - leg tracks start only near the body's own ground.
  Debug: `STARSIM_DEBUG_NRHO`, `STARSIM_DEBUG_OUTBOUND`, `STARSIM_DEBUG_LUNAR_TRANSFER`, `STARSIM_DEBUG_LUNAR_SESSION`.
  Open: the plan's propellant check uses the impulsive TLI (~40 m/s gravity loss inside the 5 % reserve).
  l4 landing and ascent (plans `docs/superpowers/plans/2026-10-15-lunar-l4-landing-ascent.md` and
  `...-lunar-l4-mass-correction.md`): mission 1 (`hls_lunar_llo`) flies LEO -> LLO -> descent -> surface stay -> ascent
  -> LLO end to end (`sim::start_lunar_landing_mission`, legs chained by `FlightSession`). Masses (mass correction
  2026-10-15): l3's "~1,185 t" was the propellant USED; the real HLS has ~573 t in the LLO, ~566 t at PDI (~500 t NRHO
  variant) and lands ~315 t. Landing thrusters re-sized [E] to 12 x 90 kN (throttle 35-100 %), canted 15 deg at 40 m,
  hover 225-642 t; legs crush force 4.5 MN [E]; leg damping 3e6 N s/m [E] (Task 10: at 1e6 the hull rocked on 1-2 feet
  for ~15 s after the engine cut on a 5 deg slope and walked 1.2 m; the regularised friction (0.1 m/s) lets the
  footprint creep only while the hull settles (~15 s; 0.17 m of the 1.18 m walk on 5.2 deg); a resting body is held fixed
  to the Moon, so a stay adds none). Descent: site pass - the LLO waits for a pass of
  the polar site within `max_crossrange_km` (20 km) and trims the plane ~90 deg before it (user 2026-10-15); the wait
  cannot be shortened by the mission epoch (Task 9 finding, user 2026-10-16: "refuse honestly" rather than re-target):
  the Moon is tidally locked, so a polar LLO's plane, which contains the arrival asymptote, sits at the same place in
  the Moon frame for any epoch - 5.8 d wait for Mons Mouton (any polar plane passes within 158 km of -84.8 deg).
  DOI on the vacuum Raptors at minimum (periselene 15 km above the site), trims / DOI / circularisation on the single
  sea-level Raptor (`lunar_small_burn`, transients <= 60 % of the burn [E]). Braking (`gnc/apollo_braking`, Klumpp
  R-695 quartic targets; Apollo 11 validation 508.5 s vs 507 s flown) on the 3 RVacs + RCS attitude (user 2026-10-15:
  Isp 380 vs 350), then sea_level_1, then the landing thrusters (user 2026-10-15: the set drops at
  max(0.9 x next max, running minimum)); braking range from the mass (`mission::braking_range_m`, thrust fraction
  0.55, factor 1.0, floor 150 km: 164 km at 566 t). Approach: SCvx on the landing thrusters from the high gate
  (700 m / 800 m / 25 m/s - lower than the spec's ~2 km), "equivalent gimbal" simplification: the canted, ungimballed
  thrusters steer by differential throttling (~20x the RCS torque, equivalent gimbal 9.4 deg; user 2026-10-15), modelled
  for SCvx as a gimbal. Terminal descent (Apollo P66 style, `LunarDescentPhase::Terminal`, "Endanflug") from SCvx's
  last phase: 1 m/s sink, throttle kept >= 0.1 above the minimum (no differential authority at a limit) and <= 0.95 of
  the weight floor. Engines off on leg contact; judgement as Falcon 9's. Surface stay: resting on the legs for
  `stay_hours`; no boil-off. Ascent (`gnc/lunar_ascent_guidance`, PEG extracted to `gnc/peg_steering`): vertical rise to 150 m, PEG to
  15 km x 100 km on the 3 RVacs (user 2026-10-15), the landing thrusters steer by differential throttling until the RVac
  axis is within 2 deg of PEG (RCS alone left the ~340 t HLS 60-90 deg behind), acceleration limit 1.5 g [D] (sweep
  1-2.5 g within 0.15 t, user "find the optimum"; 3 g+ crashes: PEG t_go collapses from a vertical start), circularised
  at the aposelene. RVac engine-out (braking or ascent): the other two cut, the 3 sea-level Raptors take over. Aborts
  (`abort_landing`, automatic when the approach cannot be planned): Arrest (climb) then PEG once a fresh solve steers
  below 0.85 radial or the ballistic apex reaches the insertion radius; refused when the propellant cannot reach orbit
  ("Abbruch nicht möglich", the landing continues). Leg bug fixed (l2): a site's antipode was judged "on site". Budget
  (`mission::lunar_budget`, mission `[budget]` [D] from the flights): 120 t LLO closes (64 t propellant left in the
  flight); 204 t LLO refused (9.3 t short); NRHO variant refused at 120 t (2.2 t short: NRHO -> LLO 956 m/s and
  LLO -> NRHO 990 m/s flown vs 876 / 725 nominal - the return departs 5.3 d after the LLO arrival, outside the cheap
  window). Results: descent from PDI 566 t: 2068 m/s (380 / 350 s split), landed 0.20 m, 1.04 m/s; with DOI 18 m/s
  ~2.09 km/s vs Apollo's ~2.06 km/s (DOI ~22 m/s + PDI ~2.04 km/s [S: Bennett, NASA TN D-6846]) - within 1.5 %
  (not broken down further). Ascent from 340 t: 1955 m/s into 100 x 100 km (ideal Hohmann from the 5.3 km high site
  ~1.72 km/s) vs Apollo ~1.85 km/s into 17 x 83 km (ideal ~1.69 km/s): the higher, circular target costs ~30 m/s, the
  rest is the vertical rise, the PEG start from 150 m and the steering seconds - not broken down further. Campaign
  (`test_lunar_landing_campaign.cpp`, PDI state +-1 km / +-5 m/s per axis, thrust +-2 % via
  `FlightSession::set_thrust_dispersion`, propellant +-5 %, touchdown spot within 1 km on the LOLA terrain): 21 / 21
  landed, 0.02-0.78 m (the offset grows with the slope: the hull settles onto the downhill feet), sink 0.98-1.03 m/s,
  237-256 t from PDI; aborts from 3 / 1 / 0.3 km reach the LLO (148 / 142 / 135 t). Debug:
  `STARSIM_DEBUG_LUNAR_DESCENT`, `STARSIM_DEBUG_LUNAR_ASCENT`, `STARSIM_DEBUG_SCVX`, `STARSIM_DEBUG_LEGS`; single
  campaign case `"[.lunar_case]"` with `STARSIM_LUNAR_CASE=<seed>`. Open: no DOI re-plan after the trim (0.2 m flown
  without), boil-off, a degraded profile for automatic aborts from fast low states (PEG saturates: 111 x 115 km), the
  status at burn end shows osculating elements (95 x 100 km for a 100 km LLO; the l6 telemetry shows the same osculating values), the sub-orbital cutoff
  guard is untested (no scenario reproduces it).
  l5 return to Earth (plan `docs/superpowers/plans/2026-10-16-lunar-l5-return.md`, ledger
  `.superpowers/sdd/2026-10-16-lunar-l5-return/progress.md`): mission 2 - a Block 3 ship in the LLO or the NRHO
  (tanked by tankers, M9; the start state carries just the planned propellant) flies TEI -> MCCs -> braking burn ->
  direct entry and catch at Starbase (`ship_lunar_return_llo`, `_nrho`), or an aerocapture ("Aircatch") into a ~400 km
  orbit -> perigee raise -> circularisation -> b4 (`ship_lunar_aircatch_llo`); `sim::start_lunar_return_mission`,
  `FlightSession::plan_lunar_return`, steps in `core/src/sim/flight_session_lunar_return.cpp`.
  Radiative heating: NASA TM-102652 (Sutton & Hartung 1990) equilibrium stagnation-point tables transcribed
  (`config/aero/radiative_heating_earth.csv`, 624 rows, 30-84 km, 8-18 km/s, nose radius 0.05-10 m; log-linear in
  density, linear in speed and radius); Tauber-Sutton cross-check +3 % (60 km, 11 km/s, 2.3 m). `FlightBody` heat rate =
  Sutton-Graves convective + radiative (Earth air, from 8 km/s). At 60 km / 5 m: 6.8 MW/m^2 at 11 km/s, 0.63 at 9.5 km/s -
  the reason for the braking burn (user 2026-10-16): no direct lunar-return entry stays below the 1.2 MW/m^2 [E] limit.
  Known gaps: equilibrium air only (no non-equilibrium radiation, no ablation coupling - the tiles do not ablate),
  Starship's heat-flux limit and the 6 m nose radius are estimates (no published figure; tiles ~1650 C ~0.65 MW/m^2
  radiative equilibrium), Tauber-Sutton itself is outside its range (Rn 0.3-3 m).
  Corridor (`gnc/entry_corridor`; speeds relative to the rotating Earth, east, 120 km): direct (1 deg wide) max 9,688 m/s
  (-5.53 .. -4.51 deg, steep side limited by Hitzestrom), aerocapture to 400 km (0.5 deg) 10,086 m/s; the real return
  (~10.5 km/s relative, ~11.0 inertial) has ~0.2 deg; Dragon at 11 km/s 1.68 deg around -6.6 deg (Apollo-like). The
  planned entry angle is the corridor angle with the widest bank window (`gnc::widest_bank_window`); the aerocapture's
  entry speed comes from a bank window of >= 10 deg (9.78 km/s). Interface state placed at the geodetic 120 km.
  Earth return planner (`mission/earth_return`): LLO TEI ~880 m/s (aligned plane, 3.4 d; the Moon moves ~13 deg/day:
  835 m/s at +15 h for a 13 deg misaligned LLO), MCC 4 m/s for 0.1 % / 34 m/s for 1 % TEI errors, braking burn on the
  vacuum Raptors centred at ~2,000 km, 1.03-1.15 km/s by vis-viva (the plan's 1.4-1.5 km/s assumed 10.9 relative),
  entry ~9.7 km/s relative; aerocapture braking ~0.93 km/s. The gate is aimed by moving the arrival epoch (downrange is
  badly conditioned in the TEI components) with a gate window scan +-12 h by continuation (2 h steps halved to 15 min,
  warm start extrapolated); the plan aims ONE bank sign's predicted track (lateral offset fixed point, ~957 km).
  The direct entry reaches Starbase only at epochs where the arrival brings the site within reach (Nov 12 2026 works,
  Nov 2 not); the user's rule (2026-10-18): plan as mission planners do - find the departure window, no epoch hopping.
  NRHO: a single perilune TEI costs 3.4 km/s (perilune motion head-on to the return asymptote) -> NRHO departure + 100 km
  return powered flyby (Artemis architecture, mirror of l3), up to 8 patched-conic departure options tried cheapest
  first: Nov 13 2026 575.7 m/s (spec 400-500; the epoch sweep -48..+48 h gives 427-742 m/s, 17/17 planned).
  Predictor (`gnc/entry_predictor`): radiative heating, skips carried, exit orbit, truth air-density factor
  (`FlightSession::set_air_density_factor`); great-circle range unwrapped by the swept angle (a 28,000 km skip + glide
  read as 10,857 km) and a target behind the start's track counted as a turn less its distance ahead (the NRHO flight's
  spurious plan, red since 884bfdd).
  Entry guidance (`gnc/entry_guidance`, `EntryGuidanceParameters::lunar_return()` / `aerocapture()`): bank linear in
  energy (FNPEG principle, Lu JGCD 2014; final 40 deg [E], user 2026-10-16) - no constant bank both captures and stays in
  the limits at 9.7 km/s; capture preferred to the predicted limits (lowest capturing bank + 2 deg); q limit 32 kPa [E]
  (LEO entries keep 25 kPa; user 2026-10-17); no reversal above 8.5 km/s [E] nor while the range is missed by > 50 km
  [E]; a lateral funnel (300 km (v / 9.7 km/s)^3 [E], closed at 5 km/s); reversals timed within the 2 s cycle; the bank
  roll is fed forward to the controller (`EntryCommand::rate_gcrf_radps`, also the roll axis: without it the PD loop
  lagged ~3 deg in roll and yaw and the sideslip carried the b4 entry 9 km off) and ramped (`bank_step_smooth`,
  0.5 deg/s^2 [E]: a rate step fed forward at 18 kPa departed the ship); ~180 deg reversals keep their roll direction.
  Aerocapture: one bank sign, no plane steering (reversals through lift up lofted the pass to 17,800 km), phase Exit; a
  second pass (up to 3) when perigee raise + circularisation would cost > 150 m/s [E] (dense air).
  Entry attitude control (Task 10 findings): the flaps cancel the modelled hull moment, so a dispersed hull (x0.92) left
  an alpha error growing with q (up to 11 deg) - `FlightSession::update_hull_moment_scale` estimates k = true / modelled
  hull pitch moment from the pitch error (Mach > 10, q > 200 Pa, 0.3 deg deadband, 2 % apply deadband, no integration
  with a flap within 2 deg of a stop; user 2026-10-17: a scale, not a torque integral, which broke the LEO catches);
  flap allocation rewritten as a box-constrained active set (`AttitudeController`: free flaps re-solved, box = stops
  and the 0.25 s [E] flap-rate reach, all-flap moment-neutral centring, a secant only for a flap AT a stop - the sin^2
  force has a zero derivative there and the old min-norm + clamp chattered 0/90 deg); below 200 Pa [E] the flaps rest in
  the coming entry's pitch trim (`EntryAeroTable::trim_flap_rad`; user 2026-10-18, no public data on a thermally best
  rest angle).
  Braking cut off closed-loop on the entry-interface state (speed and flight-path angle), then an RCS entry trim
  (threshold 0.5 m/s); the burn executor honours the thrust dispersion. Catch from lunar gates (b3): catch plans keep a
  soft half-plane 18 m off the tower while the hull bottom is below the tower top (overshoot into the lattice above the
  arms), and the gate hands over higher when the sink exceeds 100 m/s [E] (gate x (sink / 100)^2). Aircatch b4: burns
  more than 30 min away wait in free drift; the far deorbit is re-planned every 24 h and 3 h before its burn with the
  onboard-estimated orbital drag; a re-plan short only of the landing reserve is flown.
  Re-plan from flight data (user 2026-10-18): `FlightSession::replan_lunar_return` (gate window scan from the live
  coast via `EarthReturnPlanner::recovery`), automatic when an MCC does not converge or exceeds its 90 m/s allowance and
  the new plan is cheaper; `lunar_return_replans()`; game command `replan_lunar_return()` since l6 (key and display with the UI specification). Botched TEI (velocity error = fraction of
  the TEI delta-v): 1-3 % MCC 22-66 m/s, 5 % re-planned 109 m/s caught 2.2 t, 7 % re-planned 152 m/s caught 0.0 t, 10 %
  refused honestly (impact).
  Results: LLO direct flight (Nov 12 2026): TEI ~876-888 m/s, MCCs < 1 m/s, braking ~1,030-1,043 m/s at 2,000 km, entry
  9,695 m/s -4.8 deg, peak 0.75-1.06 MW/m^2 (radiative ~0.2), 2.6-4.2 g, caught (8-13 t left); aircatch: braking 926 m/s,
  exit ~53-55 x ~422-427 km (936 kW/m^2), perigee raise 43 m/s + circularisation 62 m/s, b4 deorbit 63 m/s ~93 h later,
  caught 7.6 t; NRHO direct caught 6.1 t (269.3 + 306.4 m/s). Campaigns (`test_lunar_return_campaign.cpp`, seeds 1-21
  each: TEI and braking +-0.2 % / 0.1 deg [E] - the closed-loop cutoff residual, user 2026-10-17 -, entry angle +-0.1 deg
  at the interface, truth hull aero +-10 %, density +-15 %, thrust +-2 %): direct 21 / 21 caught (gates <= 0.03 km,
  5.9-13.2 t left, peak 0.93-1.16 MW/m^2 - up to 97 % of the limit, 3.4-4.2 g); aircatch 21 / 21 caught (exit perigee
  41-61 km, apoapsis 413-438 km, b4 gates <= 0.04 km, 0.7-9.6 t left, peak 0.83-1.08 MW/m^2, 2.3-2.5 g). A braking burn
  short by 500 m/s or not flown at all ends honestly in "Hitzeschild versagt" (final review: the sink-scaled gate
  height used to hand such a dive over to the flip 1,322 km up - now only below 40 km). Artemis I comparison (magnitude only): Orion entered at ~11 km/s (Dec 2022) and skipped; our ship
  brakes ~1.0-1.1 km/s and enters at ~9.7 km/s relative (~10.1 inertial), one direct glide. Density x3.0 breaks the
  aircatch ship up (Hitzeschild versagt), x0.5 exits to a ~10,000 km apoapsis - outside the campaign's +-15 %.
  Plan time (l6 Task 1, `[plan_speed]`, hidden probe `[.plan_timing]`): a whole mission start (propellant fixed point
  + the session's gate plan) takes LLO ~7 s, NRHO ~8 s, aircatch ~4.5 s (before: 130 / 125 / 15 s). The profile
  (`STARSIM_DEBUG_RETURN` prints a `return profile` line per plan / recovery) showed 99 % in the coasts and the
  predictor at 0.3 %; 75 % of the time was CSPICE (Moon, Sun, Earth orientation per force evaluation) - now
  interpolated by the session (`InterpolatedEphemeris` / `InterpolatedOrientation`) - and the NRHO flyby search
  spent ~19 s per plan in `solve_lambert`'s 4,000-sample bracket scan (now 100 samples first, 4,000 only as the
  fallback). A looser tolerance for the scan's samples saved nothing (6.8 -> 6.4 Newton iterations per sample) and
  was not kept. The plans move by <= 0.13 m/s TEI, 3 s of interface epoch.
  Aerocapture attitude at peak q (l6 Task 1 finding, root cause 2026-10-18, CLOSED by propellant settling): the aircatch campaign moved by tonnes on
  round-off (seeds 4 / 7 short of the deorbit after the interpolated ephemeris and the Lambert bracket, both exact to
  < 1 mm / < 1e-9): the entry attitude loop diverged at 73 km / 9.8 km/s / ~6.7 kPa (roll error to +45 deg, axis error
  35 deg, yaw rate 0.09 rad/s against a saturated yaw RCS), the guidance answered with the 150 deg bank limit, the
  pass exited at 12,787 km and the second pass spent the deorbit propellant. Root cause: these seeds carry 61-64 t at
  the interface - all in the main tanks, LOX aft, so the CG sits aft - and the pitch trim at peak q drove BOTH AFT
  flaps to the broadside stop (90 deg), where the sin^2 flap force has no derivative. Pitch, roll and yaw were left to
  the two small forward flaps (roll and yaw coupled) and the RCS (demand 3-10x its torque): a small roll disturbance
  grew. (The `STARSIM_DEBUG_ENTRY_ATTITUDE` flaps print in config order: aft right, aft left, forward right, forward
  left.) The aft CG also costs cold gas: the pass alone burns 2.9 / 6.4 / 8.4 t of main-tank RCS gas with 55 / 61 /
  64 t aboard (the RCS holds the pitch the flaps cannot trim). Tried and dropped (user 2026-10-18): a 10 deg allocation
  margin below the broadside stop (`flap_authority`: upper box min(max - 10 deg, d + reach)) held the attitude (64 t with a 0.02 rad/s roll
  kick: axis error 6 deg, apoapsis 428 km) but cost 0.6-1.9 t more gas per pass - 6 of 21 aircatch seeds then lacked
  1-2 t for the deorbit, and the Mach 2.6 tower catch struck the tower. Header tanks alone moved the 64 t CG by 3 cm
  (the surplus sits in the main tanks); the real cause was the liquid model - always a column on the aft domes. With
  propellant settling the residue lies forward on the belly at entry: fixture "An aerocapture pass with 64 t aboard
  damps a roll disturbance at peak q" (`[header_tanks][aerocapture]`) 1.3 deg axis error (was 166), apoapsis 420 km,
  peak 904 kW/m2, 60.9 t left; aircatch seeds 1-7 back to 7 of 7.
  Debug: `STARSIM_DEBUG_RETURN`, `STARSIM_DEBUG_ENTRY` (incl. constraint and reversal lines), `STARSIM_DEBUG_ENTRY_ATTITUDE`
  (1 Hz axis error, rates, RCS, flaps), `STARSIM_DEBUG_FLIP` ('[gate] state'), `STARSIM_DEBUG_DEORBIT`; hidden tests
  `"[.lunar_return_case]"` (`STARSIM_RETURN_SEED`, `STARSIM_RETURN_AIRCATCH=1`), `"[.lunar_gate_replay]"`
  (`STARSIM_GATE_STATE`), `"[.lunar_handover_replay]"` (`STARSIM_HANDOVER_STATE`), `"[.lunar_return_epoch_probe]"`,
  `"Aerocapture dispersion probe"` (`STARSIM_PROBE_DENSITY`, `_AERO`, `_PROPELLANT_KG`, `_KICK` "h_km,wx,wy,wz": a
  peak-q attitude repro in ~13 s).
  l6 game interface (plan `docs/superpowers/plans/2026-10-18-lunar-l6-game-interface.md`, ledger
  `.superpowers/sdd/2026-10-18-lunar-l6-game-interface/progress.md`): the game's `FlightSim` runs its session on a
  simulation thread (`sim::SessionRunner`): commands (launch, mission start, plans, pilot actions) are queued and
  return a command id (0 when refused before posting); `result(id)` -> {done, ok, message} and the signal
  `command_finished(id, name, ok, message)` (emitted on the main thread from `advance_to` / `wait_idle`) report the
  outcome. Pilot input goes to a single "latest" slot read before each chunk; `advance_to` only sets the target
  epoch, the worker advances in chunks of at most 4,000 ticks and builds a snapshot after every command and chunk -
  all getters read that snapshot (shared containers, read-only); the target counts as reached within one tick.
  `planning_s()` gives the wall time of a running plan (the world clock stands meanwhile). CSPICE is serialised by a
  process-wide recursive lock (`detail::lock_spice`, held by every adapter call; without it a second thread corrupted
  CSPICE's trace stack, `[threads]`). `FlightSim` passes the Moon orientation, so lunar missions run in the game:
  `start_mission` kinds `lunar_landing` / `lunar_return` (all legs planned at the start; a refusal's reason is the
  command's message), `lunar_state()` (step, status, burns, descent plan, touchdown, return plan with re-plan count,
  pilot cue), `lunar_site()`, `moon_terrain(lat, lon)` (calling thread; the terrain is immutable), commands
  `complete_pilot_correction()` and `replan_lunar_return()`. Telemetry carries `reference_body`, `altitude_terrain_m`
  (above the lunar terrain, `FlightTelemetry::altitude_terrain_m`) and osculating periapsis / apoapsis / inclination
  about the reference body (inclination against its equator: a polar LLO reads ~90 deg). Start wall times (smoke):
  lunar return 6.9 s, lunar landing ~0.1 s, NRHO refusal 2.9 s. Display of these items: with the UI specification
  (M6 / M7, `docs/ansichten.md`). `[plan_speed]` checks wall time: it failed under the full suite's parallel load
  (LLO 12-20 s against 10 s; alone ~7 s), so CTest runs it as one `RUN_SERIAL` test. Final review: `idle()` also
  waits for a running input / chunk; an advance that throws stalls the runner until the next command (no re-throw
  every chunk), the snapshot is still built, and `FlightSim` reports input / advance / snapshot errors once via
  `push_error` (`SessionRunner::take_error`).

## ISS rendezvous and docking (milestone i)

Spec `docs/superpowers/specs/2026-10-19-iss-docking-design.md` (parts i1-i6; only Crew Dragon docks).

### ISS target body (i1)
Plan `docs/superpowers/plans/2026-10-19-iss-i1-target-body.md`, ledger
`.superpowers/sdd/2026-10-19-iss-i1-target-body/progress.md`.
- **Data source and archive.** NASA "ISS Trajectory Data" (CCSDS OEM, public domain,
  `tools/data_pipeline/sources.toml`): `fetch.py` downloads the current file and archives it as
  `data/iss_oem/ISS.OEM_J2K_EPH_<CREATION_DATE>.txt` (`tools/data_pipeline/iss_oem.py`; an already archived creation
  date is skipped), so the archive grows with every run. Committed fixture: the full 2026-10-14 file in
  `tests/validation/fixtures/iss_oem/`.
- **Format facts** (file of 2026-10-14): CCSDS OEM 2.0 KVN, `REF_FRAME = EME2000` (taken as GCRF axes; frame bias
  ~23 mas, < 1 m at the ISS, not modelled), `TIME_SYSTEM = UTC` (converted to TDB at load time), km and km/s, 15 days on
  a 4 min grid that restarts at three rows: a 90 s step onto 2026-10-17T17:57:30 and 2026-10-21T04:35:00, a 210 s
  step onto 2026-10-28T08:42:30; a last 90 s step reaches the stop time 2026-10-29T12:00 (no new segment). The
  trajectory is continuous across the seams (extrapolation from either side matches to 7-250 m, like ordinary rows).
  CRLF line ends. Header comments `MASS=460320.00`, `DRAG_AREA=1177.76`, `DRAG_COEFF=1.50`,
  `SOLAR_RAD_AREA=0.00` and the "TRAJECTORY EVENT SUMMARY" table (event, TIG as day-of-year GMT, optional ORB, DV m/s,
  HA/HP km; imperial "(...)" second lines skipped).
- **`orbit/oem`.** `Oem` parses one file (rejects other frames, unordered rows) and interpolates by Lagrange degree 8
  (CCSDS recommendation) inside runs of equal steps ("segments"): a segment starts at row k when the step after k
  differs from the step before and the next step equals the step after (a single odd step makes no 1-row segment);
  the seam row is shared, and no window reaches across a seam. Leave-one-out on the thinned 8 min grid: median 20 m,
  worst 529 m (one-sided window after a seam), 1.3 m/s; the error scales ~h^9, so the published 4 min grid gives
  ~4 cm (~1 m one-sided). `OemArchive` (one directory) picks the newest file (by CREATION_DATE) whose usable window
  contains the epoch; unparsable files are listed in `skipped()`.
- **`orbit/synthetic_station_orbit`.** For dates without OEM coverage: mean elements averaged over the first
  revolution of the newest OEM, advanced by the secular J2 rates (RAAN, argument of perigee, mean argument of
  latitude); the scenario sets the argument of latitude. The elements refer to the **true equator of date** (z = the
  ITRF pole, the J2 axis of the gravity model; x = the GCRF x axis projected onto that equator,
  `true_equator_to_gcrf`), not GCRF: in GCRF the OEM shows i = 51.773 deg (precession since J2000) and the advanced
  RAAN missed the OEM by 0.156 deg after 14 days; in the true-equator frame i = 51.633 deg and the RAAN error after
  14 days is +0.018 deg. The x axis was first the node of the true equator on the GCRF equator (final review): that
  lies near GCRF +y in 2026 (RAANs 90.7 deg off the usual value), swings with nutation and is undefined at J2000; the
  projected GCRF x keeps RAANs within a fraction of a degree of the usual GCRF RAAN today. The state uses the mean elements as
  osculating ones [E: J2 short-period terms ~+-10 km in radius not restored]. Without any OEM:
  `[stage.station.reference_orbit]` of `iss.toml` (415 km [E], 51.64 deg [S], RAAN 69.3148 deg [D] at
  2026-10-14T12:00 UTC; a test keeps it equal to the fixture's mean RAAN). The 2026-10-14 OEM has a mean a - R of 420.7 km (HA/HP 418.6/421.1 km).
- **ISS vehicle** (`config/vehicles/iss.toml`): one stage, no engines. Mass 460,320 kg, drag area 1,177.76 m^2 and
  C_D 1.5 [S: OEM header] (replaced by the header of the OEM in use); fixed drag C_D A along the oncoming flow
  (`aero/fixed_area_aero`, replaces the slender-body aero); SRP with a fixed `solar_area_m2` 2,500 m^2 [E] (the OEM
  uses 0); principal inertia [E] (not used while the attitude is ideal; needed by the compound body in i5); ports
  IDA-2 (Harmony forward, +XVV: V-bar) and IDA-3 (Harmony zenith: R-bar) with position, outward axis and clocking
  `up` [E]; collision capsules (modules) and boxes (truss, arrays) [E] - data for i4/i5. Drag acts at every height
  US76 covers (to 1,000 km); there was no aero cut-off to lift.
- **Ideal attitude** (documented simplification): a body with `[stage.station]` snaps to LVLH (+XVV: body x along
  the velocity, z nadir) rotated by the TEA `body_in_lvlh_deg` [E: yaw -3, pitch -6, roll 0.5 deg] right after each
  integrator step (`FlightBody::set_ideal_lvlh_attitude`, `frames/lvlh` - moved from `gnc` so `vehicle` stays below
  it); contact torques do not change it (CMGs taken as strong enough). Translation is free.
- **Session.** `sim::iss_start(iss, archive, time, earth, epoch, source, argument_of_latitude)` chooses the source by
  the fidelity flag `iss_orbit` (`IssOrbitSource`): `oem` outside every window is a German error (no fallback),
  `synthetic`, or `auto` (OEM where covered, else synthetic; `note` says which); `StationStart` carries the state's
  epoch. `FlightSession::add_station(config, state, epoch, impulses)` adds the station as an uncontrolled body (when
  it is alone it is the "controlled" index, but nothing commands it) - **call it after `place_body` /
  `launch_from_pad` / `place_in_orbit`**: they clear all bodies, the station included (`station_index()` is reset, no
  dangling index). Without bodies the session starts at the station's epoch; with bodies an epoch more than one tick
  off the session's is a `logic_error`, as are a second station and a vehicle without `[stage.station]`. A station alone still
  coasts: its orbital rate (1.1e-3 rad/s) exceeds the "rotation settled" threshold (1e-3 rad/s), so the coast check
  skips bodies in ideal attitude.
- **Impulses.** OEM events with DV > 0 after the start epoch become impulsive Delta-v along the velocity at their TIG
  [D] (a coast stops there). The sign comes from the OEM's own mean radius one revolution after vs before the TIG
  (32 samples each). Within one revolution of the window edges the sign cannot be read (a single sampled radius
  swings by several km, a 1 m/s burn moves a by ~1.8 km): the burn is assumed posigrade (reboosts are) and the
  `note` names it. The fixture has only a 0 m/s docking event; the sign paths are tested with generated two-body OEMs
  (retrograde, posigrade with "(" in the event name, near the window end). Event names may contain "(": only lines
  starting with "(", "|" or "=" are skipped.
- **OEM validation** (`[iss]`, SRP off as in the header, start 2026-10-16T00:00 UTC, our full force model with US76):
  after 6 h R 21.8 m, T -894.5 m, N -13.3 m; after 24 h R -11.7 m, T -3,228.7 m, N -43.0 m (tolerances 1,400 m /
  4,900 m). Almost all along track: the US76 density at ~420 km differs from NASA's (solar-activity dependent)
  model; NRLMSISE in M8 should shrink it. Station tests run < 0.5 s each.

### Crew Dragon translation and relative navigation (i2)
Plan `docs/superpowers/plans/2026-10-19-iss-i2-dragon-translation.md`, ledger
`.superpowers/sdd/2026-10-19-iss-i2-dragon-translation/progress.md`.
- **Draco layout kept** (user 2026-10-19): the [E] layout of the RCS milestone reaches the approach accelerations
  without changes. Planning probe (LP bound, cone open, 12.5 t): +-x 1534 N, +-y/+-z 576 N (0.046 m/s^2; 0.040-0.058
  m/s^2 for CG stations 4.2-4.8 m). Threshold for the test: lateral >= 0.04 m/s^2 [E] (1 m/s in 25 s).
- **Pure translation capability.** `RcsAllocator::force_capability_n` is not a pure translation: its saturating
  least squares trades a turn and a cross push (Dragon "lateral" 631 N came with 400 N axial and 92 N m, and ~400 N
  even with the cone closed). `RcsAllocator::pure_force_capability_n` bisects below it for the largest force whose
  allocation leaves torque and cross force below 1 % (of the force / of the gross thruster torque);
  `Vehicle::rcs_pure_force_capability`, `VehicleCommand::rcs_force_pure` (the force fraction refers to it). Burns keep
  the coupled capability (validated campaigns unchanged). Measured (cone open, 12.5 t): +x 1556.3 N = 0.1245 m/s^2,
  4.25 kg per m/s (ideal m/(Isp g0) 4.25); -x 1556.3 N, 4.55 kg/(m/s); +-y +-z 566.7 N = 0.0453 m/s^2,
  16.20 kg/(m/s) (3.8x the ideal: the axial pairs that cancel the side pods' couple fire too; bound 4.5x [E]);
  residual torque <= 0.01 N m.
- **Closed nose cone:** pure lateral and -x authority < 1 N. Lateral force needs the nose Dracos (under the cone);
  the side pods' tangential thrusters sit ~1.6 m forward of the CG and their couple can only be cancelled by
  differential axial thrust, which leaves no net lateral force.
- **Dragon port** (`crew_dragon.toml`): `idss` on the capsule, station 4.0 m [E] (top of the pressure vessel, under
  the cone hinge), axis body +x, clocking `up` body -z [E], `behind_nose_cone = true` [S]: covered while the cone is
  closed.
- **Port frame** (`vehicle/docking_port`, `port_kinematics(body, id)`): x = outward docking axis, z = the clocking
  reference `up`, y = z x x. Lever from the current CG (follows propellant use); port velocity = CG velocity +
  omega x lever. Two ports are mated with the axes opposed and the ups equal.
- **Relative state** (`gnc/relative_navigation`, `relative_port_state(chaser, target)`): chaser port relative to
  the target port in the target port frame, which turns with the target body - a co-moving chaser reads zero velocity
  (the frame's omega x r is removed). Closing speed = -d(range)/dt (> 0 approaching); misalignment = angle between
  the chaser axis and the reversed target axis; clocking = chaser up against target up about the target axis
  (positive about +x_target, (-pi, pi]). Fidelity flag `relative_nav`: only `"ideal"` (true states; GPS / LIDAR /
  thermal camera models later).
- **Session API** (`sim::FlightSession`): `set_rendezvous_target(target_index, chaser_port, target_port)`
  (out_of_range for a bad index or the controlled body, German invalid_argument for an unknown port),
  `clear_rendezvous_target()`, `relative_port_state()` (nullopt without a target), `set_translation_command(f)`
  (target port frame, fraction of the pure capability, |f| > 1 scaled to 1; logic_error without a target). Target and
  command are cleared with the bodies (`place_body` / `launch_from_pad` / `place_in_orbit`) and whenever the
  controlled body changes (`stage`, payload separation, the capsule's trunk jettison: the chaser port belonged to the
  old body). The command is flown only in an attitude hold or in rate mode outside the autopilot's own steering
  (autopilot Off / Orbit / Lunar; `translation_flown()`), and only then the session does not coast (the coast skips
  the RCS); a stale command elsewhere (a burn's drift wait, a capsule return) neither acts nor stops the time warp
  (final review). The command is applied next to the attitude hold's torque,
  **torque first** (`rcs_keep_force = false`): with the force kept (the plan's choice) a full pure translation
  saturated the Dracos, the hold's torque was cut and the attitude drifted 2.8-13.8 deg in 20 s (nose direction only;
  5 s settle did not recover it); at 80 % command the hold stayed < 0.7 deg, torque first at 100 % < 0.72 deg. Cost:
  the hold's turns may push slightly (cross <= 0.064 m/s over 20 s); a saturated translation is cut instead. A
  pilot's body-axis translation (`input_.translation`) takes precedence over the command in rate mode.
- **Translation campaign** (`[rendezvous][session][validation]`, Dragon 50 m in front of IDA-2, OEM ISS 2026-10-16,
  LVLH hold; the hold first rolls the placed Dragon in (~1 deg/s, settled to 0.43 deg after 100 s), then 20 s full
  command per target-port axis, 5 s settle; dv / expected, cross, propellant / expected, worst full attitude error
  incl. roll): +x 2.296/2.301 m/s, 0.064, 11.14/11.13 kg, 0.63 deg; -x 2.335/2.334, 0.039, 13.24/13.31, 0.71;
  +y 0.899/0.908, 0.007, 14.52/14.51, 0.58; -y 0.896/0.909, 0.006, 14.95/14.96, 0.68; +z 0.903/0.917, 0.024,
  15.13/15.13, 0.68; -z 0.912/0.917, 0.027, 14.29/14.27, 0.59 (tolerances 15 % dv, cross < 10 % + 0.01 m/s, 25 %
  propellant, 1 deg). Port +-x is the Dragon's body +-x here (both noses along the LVLH velocity). Runs in ~2 s.

### Phasing (i3)
Plan `docs/superpowers/plans/2026-10-19-iss-i3-phasing.md` (its header has the research and the four spec
corrections, approved by the user 2026-10-19), ledger `.superpowers/sdd/2026-10-19-iss-i3-phasing/progress.md`.
- **Profile and sources.** Crew-2 timeline (Spaceflight Now, 2021-04-22): separation, phase burn +37 min, boost +15.4 h,
  coelliptic burn, approach initiation **7.5 km** out, mid-course 25 min later, **Waypoint 0 400 m below the ISS**
  45 min after the approach initiation. Demo-2 burn sizes (everydayastronaut.com): phase 16.1, boost 44.2,
  circularisation 57.9 m/s. Derived [D]: coelliptic closing 4.9 m/s between 30 and 10 km = 1.5 n dh -> **dh 2.9 km**;
  coelliptic coast 2.8 h. All values are `mission::PhasingParameters` fields with tags.
- **Spec corrections (user-approved):** (1) i3 ends at Waypoint 0 braked to rest in the station's LVLH frame, not at an
  approach-ellipsoid entry point (the approach initiation transfer crosses the 4 x 2 x 2 km ellipsoid); i4 starts there.
  (2) i3 is port-independent (Crew-2 went to IDA-2 through Waypoint 0 below); the campaign runs over phase angles.
  (3) The phase burn is used only with `earliest_arrival_s`: phase burn + boost at the same perigee add up to the same
  Hohmann transfer, so every revolution count costs the same in-plane delta-v and the cheapest correct window is the
  earliest one (wait on the separation orbit, fastest catch-up). (4) Orbit plane: the J2 node regression is ~0.58
  deg/day faster at 205 km than at 415 km; the tests aim the start plane so that it drifts onto the ISS plane by the
  boost (`rendezvous_test_support.hpp`; the i6 launch-window planner must do the same), the rest is removed by the
  3D-targeted coelliptic and approach burns (no separate out-of-plane burn).
- **Clohessy-Wiltshire** (`orbit/clohessy_wiltshire`, LVLH x V-bar, y H-bar, z R-bar): propagation, two-impulse transfer,
  `lvlh_relative_state`. Only a first guess and timing aid; against two-body motion 42.7 m after the test arc (bound
  60 m: the rectilinear V-bar's curvature offset x^2/2r, ledger ruling Task 1).
- **Geometry** (`mission/rendezvous_geometry`): aim points behind/below the target, approach ellipsoid metric
  (semi-axes 2000/1000/1000 m [S: NASA]), keep-out sphere 200 m [S]. `PositionTargeter` got an optional drag term
  (`set_drag_area_per_mass`; 0 leaves the old path byte-identical).
- **Schedule** (`mission/phasing_schedule`): burn epochs from mean motions with the secular J2 rates. Default: wait on
  the start orbit until the phase is right, then boost; with `earliest_arrival_s` a prograde phase burn at
  `first_burn_lead_s` (30 min [E]) raises the apoapsis (at most to the coelliptic height: halves the catch-up rate,
  11.2 -> 5.7 deg/h) and the boost follows after `phasing_revolutions`. A phase that needs more than one catch-up lap
  adds `extra_laps`; beyond `mission_limit_s` (28 h [S]) it is refused in German. A start whose apoapsis reaches the coelliptic height
  or an open orbit is refused (no NaN epochs).
- **Planner** (`mission/phasing_planner`): each burn is targeted in the full force model with drag on both bodies
  (`gnc::coast_with_drag`, US76), flown as a finite Draco burn (`apply_finite_burn`) from the state the previous burn
  leaves. Phase and boost are prograde magnitudes solved by secant (the boost on the radius at the coelliptic epoch:
  a 180 deg position target is singular out of plane; a waiting plan first solves the boost epoch for the phase);
  coelliptic -> approach initiation point (7.5 km, dh below) and approach initiation -> Waypoint 0 by the position
  targeter (CW first guess); the mid-course correction is planned in flight only (same CW guess; the session skips
  it below 0.02 m/s [D] and reports a non-converged one as "Kurskorrektur nicht berechenbar" - the braking burn still
  follows; a skipped one carries ~24 m to Waypoint 0); braking leaves the chaser at rest in the target's LVLH frame.
  Budget by the rocket equation against the Dracos' propellant with `reserve_kg` 900 kg [E] (deorbit ~600 kg + proximity and abort);
  `min_ellipsoid_metric_before_ai` (planned arc sampled every minute) must stay > 1. `boost_phase_error_m` predicts
  the along-track miss of the boost geometry from live states.
- **Drag.** C_D A / m guesses: Dragon C_D 2.2 [E] (free molecular blunt body) on its cross-section; ISS from its OEM
  header (area x C_D 1.5 / mass). In flight the session tracks both from their arcs (`gnc::estimate_drag_area_per_mass`,
  the Dragon since its last burn, the station since the plan or its last impulse) once an arc is >= 3 h
  (`rendezvous_track_min_arc_s` [E]); the estimates replace the plan's fields before every burn. A waiting plan checks
  the boost geometry `rendezvous_check_lead_s` = 3 h [E] before the boost and re-plans from the live states beyond
  `rendezvous_replan_tolerance_m` = 2 km [E]; a burn that cannot be targeted before the boost re-plans once, later it
  aborts (memory feedback-replan-from-flight-data).
- **Session** (`flight_session_rendezvous.cpp`): `plan_rendezvous(PhasingParameters)` -> `AutopilotState::Rendezvous`,
  `RendezvousStep` None / Coast ("Phasing") / Burn ("Manöver") / Waypoint0 ("Wegpunkt 0"), `rendezvous_plan()`,
  German `rendezvous_status()`, `rendezvous_burns()` (`RendezvousBurnRecord`), `rendezvous_replans()`,
  `rendezvous_warnings()` (a station reboost inside the plan's span). Refused (German) without a station, with the
  station controlled, a busy autopilot, a rendezvous or lunar flight under way, a vehicle with main-engine burns (only
  Crew Dragon docks: the budget is the RCS propellant), or the planner's reason. Every burn is
  re-targeted from the live states at its command epoch and flown through `execute_burn` (Dracos along +x); big burns
  are commanded `burn_drift_lead_s` before their preparation, mid-course and braking 600 s
  (`rendezvous_small_turn_lead_s` [E]); between burns the vehicle drifts freely.
- **Nose cone:** `plan_rendezvous` opens Crew Dragon's nose cone (as the real Dragon after separation; restored on a
  refusal). With it closed the side Dracos pushed 0.02-0.28 m/s while turning to a burn and Waypoint 0 was missed by
  83-391 m; open: 2-12 m. A `BurnExecutor` change (sensing attitude pulses before ignition) was tried and reverted: it
  broke the ship return and lunar return campaigns and was not needed once the cone is open.
- **Validation.** Plan from 120 deg (`[validation]`): boost 64.65 m/s @+9.80 h, coelliptic 62.47 @+10.55 h, approach
  initiation 1.86 @+13.35 h, braking 0.13 @+14.10 h; total 129.1 m/s, 537 kg; ellipsoid metric before the approach
  initiation 20.4; Waypoint 0 miss 0.0 m. Crew day (120 deg, earliest arrival 20 h): phase burn 55.10 m/s, 11
  revolutions, total 137.1 m/s. Campaign 0-330 deg (48 h limit, 12 cases): 122.5-135.4 m/s, Waypoint 0 after
  6.5-35.9 h, metric >= 20.4, miss 0.0 m. Refusals: phase beyond the mission limit, without the reserve, chaser above
  the coelliptic height. Flights (OEM ISS, `[session][validation]`, bounds Waypoint 0 <= 100 m and <= 0.5 m/s [E],
  flown arc outside the ellipsoid before the approach initiation, never inside the keep-out sphere): 30 deg - boost
  63.94 @+2.23 h, coelliptic 57.57 @+2.98 h, approach initiation 1.69 @+5.78 h, mid-course skipped, braking 0.15
  @+6.53 h; Waypoint 0 LVLH (-5.0, 0.0, 396.9) m, 0.017 m/s, 539 kg. 150 deg with 1.5x truth air density: boost check
  error -75.9 km -> 1 re-plan; boost 64.25 @+12.69 h, coelliptic 61.00, approach initiation 1.27, braking 0.08
  @+17.00 h; Waypoint 0 (1.8, 0.1, 400.6) m, 0.027 m/s, 576 kg. Crew day (120 deg, earliest arrival 20 h) with 1.5x
  air: phase 55.06 @+0.50 h, boost 8.33 @+17.04 h, coelliptic 68.66, approach initiation 5.93, braking 0.13 @+21.35 h;
  Waypoint 0 (-7.9, 0.6, 399.9) m, 0.021 m/s, 603 kg. **Open:** a phase plan has no boost check - the coelliptic and
  approach burns absorb the phase orbit's drag error (~-34 km here, ~+13 m/s); a re-plan from the eccentric phase orbit
  was tried in the final review and cost more (635 kg: the boost leaves the perigee); a phase trim burn after the drag
  tracking would be the fix. Real Demo-2 in-plane total ~118 m/s.
- **Diagnostics:** `STARSIM_DEBUG_RENDEZVOUS=1` prints planning exceptions, each burn's execution error
  (delta-v along/radial, position), the boost phase error and a status line per step (time, step, range, mass).

### Proximity operations (i4)
Plan `docs/superpowers/plans/2026-10-19-iss-i4-proximity.md` (its header has the research and the five spec
corrections, approved by the user 2026-10-19), ledger `.superpowers/sdd/2026-10-19-iss-i4-proximity/progress.md`.
- **Profile and sources.** Crew-2 / Demo-1 timelines (Spaceflight Now): Waypoint 0 04:25 (400 m below the ISS), go to
  enter the keep-out sphere, **Waypoint 1 04:49 220 m in front**, **Waypoint 2 05:00 at 20 m**, contact at IDA-2
  05:08-05:10; Crew-5 (2022-10-05): hold and docking go/no-go at Waypoint 2. Demo-2 (NASA, 2020-05-31): final approach
  at **0.30 m/s** (0.98 ft/s), slowed to **0.10 m/s** before soft capture, nose cone open. Zenith port (Crew-4,
  NASASpaceflight 2022-04; Crew-2 relocation 2021-07-21): Waypoints 0/1/2 then a **fly-around** to the corridor above
  the complex (geometry unpublished: [E]). Derived [D]: Waypoint 0 -> 1 is 1140 s plus a 5 min hold; all values are
  `gnc::ProximityParameters` fields with tags.
- **Spec corrections (user-approved):** (1) Transfers outside the keep-out sphere are CW two-impulse transfers
  re-solved every tick from the live relative state and flown with a deadband (the multi-impulse corrections fall out
  of the loop), not targeted on the full model: CW vs full model Waypoint 0 -> 1 miss **0.95 m** (bound 2 m). (2) IDA-3
  (R-bar) gets a fly-around: Waypoint 0 -> 250 m in front on V-bar -> 30 deg legs on a 250 m arc about the station CG
  -> Waypoint 1 220 m above IDA-3. IDA-2: Waypoint 0 -> Waypoint 1 directly. (3) Inside the sphere a range-scheduled
  axial approach (0.30 m/s far, range / 600 s between the limits, 0.10 m/s at Waypoint 2, 0.10 -> 0.075 m/s in the
  final approach) with lateral convergence onto the axis. (4) i4 ends at "Kontakt", the crossing of the contact plane
  (`contact_plane_m` = 0 [E]; i5 sets the ring's reach), recorded with the relative port state; no contact physics
  before i5. Capture envelope [E] (i5 verifies against the IDSS IDD): axial 0.05-0.10 m/s, lateral <= 0.04 m/s,
  offset <= 0.10 m, angle <= 4 deg, rate <= 0.2 deg/s. (5) Contact speed target 0.075 m/s [E].
- **Geometry** (`gnc/proximity_geometry`): `proximity_path(port, parameters)` builds the waypoints (LVLH points outside,
  `on_axis` points on the port axis inside), `corridor_status` (cone of 10 deg [E] half angle about the port axis,
  never narrower than 0.3 m; beyond 2x the bound: retreat; x <= 0 counts as retreat), the docking attitude (chaser
  port axes opposed to the target port's, ups equal) and `capture_envelope_check`.
- **Transfer** (`gnc/proximity_transfer`): CW two-impulse transfer with keep-out clearance (sphere radius + 10 m margin,
  sampled; a path through the sphere is refused, the guidance then aborts "kein sicherer Transferweg"). The re-solve
  stops `transfer_final_s` 60 s before arrival; the hold law brakes onto the waypoint. **Breakout:** the smallest of
  0.3 / 0.5 / 1.0 / 1.5 m/s [E] along the candidate directions that leaves the sphere within 1800 s and stays outside
  for 24 h (CW, 30 s samples); Task 2: (45,0,0) -> 0.30 m/s +z, closest 222 m; full model 24 h closest after exit
  236 m.
- **Translation loop** (`gnc/translation_loop`): desired relative velocity -> translation fraction per axis with a
  hysteresis deadband (start above 0.02 m/s, stop below 0.005 m/s [E]; fraction = error / (authority x 1 s)) for Draco
  pulse economy; near the port (final approach) a finer loop (start 0.005 m/s) keeps the offset small. Approach laws:
  axial speed schedule, lateral velocity = -offset x 0.1/s (max 0.03 m/s), position hold velocity = -error / 60 s
  (max 0.05 m/s), retreat 0.10 m/s outward along the axis. The lateral gain was raised 0.02 -> 0.1/s: the contact offset
  is about the deadband's mean error / gain (0.26 m before, 0.03 m after).
- **State machine** (`gnc::ProximityGuidance`, German phases): "Wegpunkt" (hold, go/no-go 300 s each [E]),
  "Annäherung" (CW transfer), "Umflug" (fly-around legs), "Anflug" (axial approach to an on-axis waypoint),
  "Endanflug" (Waypoint 2 -> contact), "Halt", "Rückzug", "Abbruch", "Kontakt". Monitors each tick: leaving the corridor
  -> "Halt: Korridor verlassen" (beyond 2x the bound: retreat); closing faster than 1.5 x schedule + 0.02 m/s -> "Halt:
  zu schnell"; inside the sphere off the corridor -> "Abbruch: Sicherheitszone verletzt"; RCS propellant below the
  deorbit reserve (600 kg [E], d3: 598 kg from 420 km) -> "Abbruch: Treibstoffreserve". Commands `hold`, `retreat`
  (from a waypoint or a hold that interrupted one: to the previous waypoint; from a monitor: to the last reached),
  `abort`, `resume` latch a request resolved in the next update; a second command before then is refused. Every
  refusal sets a German `refusal()` (the session shows it as `proximity_status()`), e.g. retreat at Waypoint 0, resume
  without a hold, resume after an abort while the breakout is still flown, beyond 5 km, inside the sphere or below the
  deorbit reserve (it would abort again at once); the session refreshes the live state (`observe`) before every command
  because it coasts without updates after a breakout. A retreat to an on-axis waypoint flies the axial law also between
  the sphere and the waypoint (a CW transfer would start inside its clearance); a monitor retreat at an on-axis
  waypoint the chaser already sits at goes one waypoint further back (no retreat/arrival loop). A transfer starting
  inside the 10 m margin (outside the sphere) only has to stay outside its start range. A start within 10 m [E] of the
  first waypoint holds there (i3 arrives within 2-5 m). After the breakout burn the phase stays "Abbruch" (passive,
  status "Ausweichmanöver ausgeführt"[", nicht passiv sicher"]); `resume` flies back to Waypoint 0.
- **Session** (`flight_session_proximity.cpp`): `start_proximity(station_port, ProximityParameters)` ->
  `AutopilotState::Proximity` ("Anflug"); getters `proximity_phase()`, `proximity_status()`, `proximity_events()`,
  `proximity_contact()`, `proximity_warnings()` (station impulses within 3 h of the start); commands
  `proximity_hold/retreat/abort/resume()`. Refused (German) without a station, with the station controlled, a busy
  autopilot (not Off / Orbit), a rendezvous other than at Waypoint 0, a lunar flight, no docking port on the vehicle, an
  unknown or unsuitable station port, a start beyond 5 km, or inside the sphere off the start axis. Opens the nose cone
  and sets the rendezvous target (chaser port: the vehicle's first docking port). The docking attitude is an LVLH hold
  re-derived every tick from the port kinematics; the translation authority along the target port axes is measured
  once at the start in that attitude. The guidance runs in the autopilot dispatch before the tick's vehicle command,
  so its fraction is flown in the same tick; `advance_to` keeps ticking during the flight (no coast across a hold or
  the final approach) and may coast only after a completed breakout. The flight ends with the rendezvous target
  (`clear_rendezvous_target` and every path calling it: placing bodies, launch, staging, separation, trunk jettison),
  with `disengage_autopilot`, with `set_rendezvous_target` (the flight is bound to its ports), and at contact
  (autopilot off, the flight stays readable).
- **Validation.** Guidance alone (CW point mass, dt 0.05 s): IDA-2 62.8 min / 49.8 kg, IDA-3 98.8 min / 112.7 kg (the
  CW sim charges 16.2 kg/(m/s) on every axis: conservative). Session campaigns (full model, 200 Hz, OEM ISS,
  `[proximity]`, not `[slow]`: 10-24 s each):

  | case | time | propellant | axial | lateral | offset | angle | rate |
  |---|---|---|---|---|---|---|---|
  | IDA-2 from Waypoint 0 | 62.0 min | 31.2 kg | 0.079 m/s | 0.0010 m/s | 0.052 m | 0.61 deg | 0.022 deg/s |
  | IDA-3 (3 fly-around legs) | 98.1 min | 64.0 kg | 0.078 m/s | 0.0018 m/s | 0.035 m | 0.48 deg | - |
  | 8 dispersed Waypoint 2 starts | - | 1.7-2.0 kg | 0.078-0.080 | <= 0.0024 | 0.030-0.050 | <= 0.59 | <= 0.039 |

  All inside the envelope. From the i3 Waypoint 0 state in the braking attitude (180 deg off, drifting 0.02 m/s):
  aligned < 2 deg after 170 s, worst Waypoint 0 offset 1.16 m, contact inside, 24.4 kg. Station reboost 0.3 m/s at
  600 s into the transfer: one warning, Waypoint 1 at 30.2 min (28.8 without), contact inside, no abort. Waypoint 1
  hold across `advance_to(+290 s)` in one call: offset 1.20 m. Abort at x 9.6 m: breakout done in 10 s, 0.291 m/s,
  passively safe, 24 h drift closest 285 m. Hold at x 11.4 m for 120 s: drift 0.20 m, 0.0019 m/s; resume -> contact
  inside. The contact offsets 0.03-0.05 m are pulse effects (no coupled-torque issue).
- **Diagnostics:** `STARSIM_DEBUG_PROXIMITY=1` prints the guidance events and every 60 s the port state, command,
  attitude error and propellant.
- **Open / for i6:** the envelope is [S] since i5 (IDSS IDD E) and the contact plane is the petal mesh plane; manual
  proximity control is M7; the game interface (getters above, docking view) is i6.

### Docking mechanism, undocking and departure (i5)
Plan `docs/superpowers/plans/2026-10-19-iss-i5-docking.md` (its header has the research and the seven spec corrections,
approved by the user 2026-10-19), ledger `.superpowers/sdd/2026-10-19-iss-i5-docking/progress.md`.
- **Sources.** IDSS Interface Definition Document Rev. E (Oct 2016): Table 3.3.1.1-2 initial contact conditions
  (axial 0.05-0.10 m/s, lateral 0.04 m/s, pitch/yaw rate 0.2 deg/s and roll rate 0.2 deg/s, lateral misalignment
  0.10 m, pitch/yaw 4 deg, roll 4 deg - at the first physical touch), §3.2 SCS mating plane at the 1,200 mm petal
  outline, §3.2.2.1 three inward petals, §3.1.1.1 sequence (petals -> latches on full ring contact -> attenuation ->
  retraction -> guide pins -> 12 hooks), Table 3.3.1.4-1 SCS loads (compression 3,500 N static / 6,500 N up to 0.1 s,
  tension 3,900 N, shear 3,200 N, bending 2,800 N m, torsion 1,500 N m), §3.2.3.6 separation energy 39.2-47.5 N m,
  Table 3.3.1.2-1 IDSS-350T inertia (the ISS inertia is that scaled by mass, [D]). Demo-2: soft capture 10:16, hard
  capture 10:27 EDT (11 min). Crew-4 / Demo-1: undock burn 1 to 0.15 m/s, keep-out sphere left 6 min 48 s and the
  approach ellipsoid 17 min after undocking.
- **Interface** (`contact/docking_interface`, passive port frame, reduced order - spec correction 2): the active ring
  face 0.30 m [E] out of the Dragon port; ring-face penalty contact (2e5 N/m, 2.9e4 N s/m [E]: a 0.10 m/s impact on the
  Dragon-ISS pair peaks at ~4.0 kN, 0.25 m/s at ~10 kN); the three petal pairs as one cone (outline radius 0.60 m [S],
  height 0.20 m, lean 30 deg, meshed clearance 0.01 m [E]): capture radius 0.241 m - beyond it the tips strike the
  opposing petals (axial contact, no centring). The petals guide the light active ring, which gives way on its platform
  (2000 N/m up to 0.20 m [E]); the vehicle feels that spring, the stiff petal contact only beyond the range. The ring
  conforms in tilt up to 6 deg and roll up to 6 deg [E]. Latches: ring faces within 5 mm, deflection within the range,
  tilt and roll within their ranges. The interface acts only within 1.2 m laterally and 0.15 m ring penetration
  (beyond: the hull contact; a chaser behind the port read as an 80 MN compression without it).
- **Mechanism** (`contact/docking_mechanism`): Ready -> Kontakt -> Weichfang (damped 6-DOF joint: translation
  critically damped at 0.5 rad/s on the chaser mass; rotation from the IDSS loads - half the bending / torsion limit at
  4 deg, damping ratio 1 with the chaser inertia; axial stroke 0.10 m, then a 2e6 N/m end stop) -> Einzug (after
  >= 120 s and settled, 0.30 m at 1 mm/s) -> guide pins (5 mm, 0.5 deg) -> Haken schließen (180 s) -> angedockt; first
  contact -> hard capture 10.0-10.2 min (Demo-2: 11). Failures: no contact for 2 s with the ring 0.05 m beyond the
  petal tips: "abgeprallt" / "Versatz zu groß" (tips only) / "Winkel zu groß" (ring touched, latch blocked);
  compression above 6,500 N or above 3,500 N for > 0.1 s: "Überlast" (the latches release, spec correction 3).
  Tension / shear / bending / torsion above the IDSS limits only warn. Below 0.05 m/s it still captures (documented).
  Undock: Haken öffnen (180 s) -> getrennt; separation speed sqrt(2 E / mu) with E = 43.35 N m [E]: 0.0844 m/s for the
  Dragon on the ISS.
- **Hull collision** (`contact/station_collision`): hull rings every 0.5 m (16 points) over the trunk (3.7 m) and the
  capsule tapering 4.0 -> 1.8 m [E: `hull_top_diameter_m`], the top metre below the docking port left out; penalty
  1e6 N/m against the ISS capsules and boxes, within 120 m of the station CG. Any contact: "Kollision" with the
  impulse (sum |F| dt), the proximity flight ends (no structural model: the Dragon is not destroyed).
- **Session** (`sim/docking_sim`, `flight_session_docking.cpp`): `set_rendezvous_target` arms the mechanism for an
  active chaser port and a passive station port (`[[stage.docking_port]] soft_capture`); `start_proximity` refuses
  other pairs and sets the i4 contact plane to the petal mesh plane (0.70 m). Every tick before the bodies step the
  mechanism and hull wrenches act on both bodies (equal and opposite; the station torque is dropped - ideal
  attitude). Free drift (no RCS) from the first contact to the hooks (real Dragon; the session rate damping fought the
  joint otherwise). Coasting near the station is limited by the clearance (CG to the primitives minus the hull reach;
  port to port minus the petal reach) over the relative speed in the rotating station frame. **Compound** (spec
  correction 4): at hard capture the chaser becomes a point mass of the station vehicle at its CG
  (`Vehicle::set_attached_mass` - a stage extra mass shifted the station CG inside the body and every port by 0.44 m),
  the station state moves to the compound CG with the momentum-weighted velocity, the chaser is held on it
  (`FlightBody::hold_docked`) at the mated pose and follows after every tick and coast; `compound_mass()` reports the
  summed mass and the inertia tensor (unused while the attitude is ideal); the Dragon drag area is not added
  (~0.6 % of the station C_D A). A failed capture sends the proximity flight back to Waypoint 2
  (`ProximityGuidance::capture_failed`, status "Rückzug: Fangen fehlgeschlagen (<reason>)"); the mechanism re-arms 1 m
  beyond the petal tips. While docked or capturing: `start_proximity`, `plan_rendezvous`, `plan_capsule_return`,
  `ignite`, `stage` are refused (German); getters `docking_phase/status/events/loads`, `docked`, `docking_busy`,
  `compound_mass`, `collision_impulse_ns`, `docking_merge_record`.
- **Undocking and departure** (spec correction 5, `gnc/departure_guidance`): `undock()` (docked, autopilot Off / Orbit)
  -> hooks open -> the springs split the relative speed by mass along the port axis (momentum record exact) ->
  `AutopilotState::Departure` "Abflug": "Abkoppelimpuls" to 0.15 m/s opening, "Rückzug vom Port" on the axis to 10 m,
  "Abflugmanöver" = the i4 breakout with the approach-ellipsoid exit required within 3600 s (passively safe 24 h),
  "Abflug" drift (coasting, a guidance tick every 60 s), "Abflug abgeschlossen" at the ellipsoid exit (autopilot off,
  rendezvous target cleared). Later departure burns are not modelled; the deorbit planned from the drift orbit replaces
  them. CW guidance: IDA-2 sphere left after 473 s, ellipsoid after 2290 s, 0.30 m/s, 6.5 kg; IDA-3 475 s / 1610 s
  (Demo-1 6.8 / 17 min with larger burns). Session IDA-2: departure complete 33 min after the springs, 3.4 kg.
- **Validation** (`[docking]`; chains, grid and splashdown `[slow]`):

  | case | result |
  |---|---|
  | IDA-2 from Waypoint 0 | contact 61.8 min, docked 10.2 min later, 31.2 kg, comp 922 N, shear 93 N, bending 557 N m |
  | IDA-3 from Waypoint 0 | contact 98.0 min, docked 10.2 min later, 64.0 kg, comp 943 N, shear 118 N, bending 262 N m |
  | IDSS grid 2^(6-2) x 2 ports (axial, offset, angle, roll, lateral, rate) | 32/32 docked; max comp 1254 N, shear 600 N, bending 2035 N m, torsion 761 N m |
  | 0.30 m offset / 10 deg / 0.28 m/s | "Versatz zu groß" / "Winkel zu groß" / "Überlast" -> retreat to Waypoint 2 -> resume -> docked |
  | docked 24 h with a 0.3 m/s reboost | the pair moves together, port error < 1 mm |
  | undock -> departure -> capsule return | 0.0844 m/s, departure 3.6 kg, deorbit 58 h later (7-day pass search), splashed down, 1871 kg RCS left |

- **Diagnostics:** `STARSIM_DEBUG_DOCKING=1` prints the mechanism and departure events, and every second while ticked
  (or within 2 m of the port) the pose, the force and the peak compression.
- **Done in i6:** the undocking timed to a return opportunity (`crew_dragon_iss_departure`) and the game interface
  (`docking_state`, `undock`, `station_geometry`) - see the i6 section.

### ISS missions, launch window and game interface (i6)
Plan `docs/superpowers/plans/2026-10-20-iss-i6-missions.md` (six spec corrections, approved by the user 2026-10-20),
ledger `.superpowers/sdd/2026-10-20-iss-i6-missions/progress.md`.
- **Missions** (`config/missions/`, `sim/iss_mission.hpp`, dispatched by `sim::start_mission`):
  `crew_dragon_iss` (`kind = "iss_rendezvous"`: the Dragon at the predicted separation of the first launch window of
  `falcon9_crew_dragon_iss` after `epoch_utc`, or `phase_deg` behind the ISS in its J2-biased plane),
  `falcon9_crew_dragon_iss` (`kind = "iss_launch"`: launch window, then the `falcon9_crew_dragon` ascent at its liftoff)
  and `crew_dragon_iss_departure` (`kind = "iss_departure"`: docked start, undocking timed to a return pass, the
  `crew_dragon_return` capsule return). The ISS comes from `data/iss_oem/` (`sim::iss_archive`) per `iss_orbit`; a
  scenario argument of latitude on the synthetic orbit is kept as an in-plane offset from the J2-advanced one, so the
  station moves through a window search.
- **Sequence** (`FlightSession::set_iss_mission`, `flight_session_iss_mission.cpp`): the legs Ascent -> Rendezvous
  (`plan_rendezvous` once the Dragon flies free) -> Proximity (`start_proximity` at Waypoint 0) -> Docking -> Docked,
  or WaitUndock -> Departure -> Return (`plan_capsule_return` with `lead_s` aimed at the previewed pass) -> Complete.
  It runs on every tick (also with the autopilot Off / Orbit); the coast stops at every due leg change and at the
  undock epoch. A proximity abort keeps the leg (resume continues); a collision or a lost vehicle ends the mission
  ("Mission beendet: ..."). Cleared with the bodies (`clear_station`).
- **Launch window** (`mission/iss_launch_window`): `AscentGuidance` fixes its inertial target plane at liftoff through
  the pad (`gnc::ascent_plane_normal`), so the window is the upward zero of the node difference between that plane and
  the ISS plane turned east by the J2 bias (`mission::plane_bias_rad`, ~0.58 deg/day at 205 vs 415 km), sampled every
  600 s and bisected, the bias iterated twice onto the schedule's boost epoch. The separation state comes from two
  calibrated numbers [D] (`falcon9_crew_dragon_iss.toml`): **535 s** liftoff -> separation and **15.0 deg** inertial
  in-plane arc from the pad. The flown ascent then lands **0.0055 deg** off the planned plane and **0.004 deg** off the
  predicted phase (200.0 x 210.0 km). A crossing whose phasing does not close in the mission limit is rejected and the
  search moves a day on (up to `search_days`, default 3 [E]): from 2026-10-16 00:00 UTC the first northerly crossing
  (+9.45 h) is out of reach in 28 h, the window is +33.05 h (bias 0.033 deg, Waypoint 0 5.5 h after liftoff).
- **Numbers** (OEM fixture): `crew_dragon_iss` start (window + plan) < 0.1 s, hard capture at IDA-2 7.0 h after
  the window's liftoff, 583 kg RCS; IDA-3 at 40 deg on the synthetic orbit: docked 9.2 h after the start; pad ->
  hard capture **7.5 h** after liftoff, 4 rendezvous burns, 0 re-plans, 612 kg. The earliest window is flown
  (`earliest_arrival_h = 0`); real crews arrive after ~24-28 h for the crew day, set by `earliest_arrival_h`.
- **Departure timing** [S: Crew-9 2025-03-18, NASA: undock 05:05 UTC, deorbit burn ~21:11 UTC, splashdown 21:57 UTC]:
  undocking 16.1 h before the deorbit burn. The start previews the first pass at least that long after `epoch_utc`
  (`FlightSession::capsule_return_preview` on the docked Dragon, `search_window_h` 168), places the docked pair
  (`FlightSession::dock_in_place`: mated pose, mechanism `start_docked`, compound merge) 600 s [E] before the
  undocking and flies on to splashdown: undocking 42.7 h after 2026-10-16 00:00, deorbit 16.10 h later as previewed,
  629 kg RCS for departure + return.
- **Game interface** (`FlightSim`, `bindings/src/flight_snapshot_iss.cpp`): getters `iss_state`, `rendezvous_plan`,
  `docking_state`, `station_geometry`, `iss_mission` (keys in `flight_sim.hpp`), commands `proximity_hold`,
  `proximity_retreat`, `proximity_abort`, `proximity_resume`, `undock`; `start_mission`'s message carries the
  warnings (ISS source note, reboosts in the mission span). Covered by the smoke test.
- **Tests:** `[launch_window]`, `[iss_mission]` (`test_iss_launch_window.cpp`, `test_iss_mission_config.cpp`,
  `test_iss_mission_sequence.cpp`, `test_iss_mission_campaign.cpp` with the `[slow]` end-to-end cases).

## Orbital refuelling (milestone r)

Spec `docs/superpowers/specs/2026-10-20-orbital-refuelling-design.md`: depot + tankers + HLS, back-to-back docking,
boil-off by heat balance, a campaign calculator from one flown tanker flight.

### Vehicles and docking geometry (r1)
- `starship_tanker` = Block 3 unchanged (`[[stage]] from = "starship_block3:<stage>"`) + four dorsal probes
  (port `transfer_dock`, `mechanism = "probe_drogue"`, `soft_capture = "active"`) and the transfer connectors
  `lox_transfer` / `ch4_transfer`. `starship_depot` = Block 3 booster + a ship without heat shield, flaps, catch pins
  and recovery: 236.5 t dry [E] (255 - 10.5 heat shield - 10 flaps - 1 pins + 3 insulation), four dorsal drogues
  (passive). The HLS carries the tanker's probes and connectors.
- A `from` stage is the source vehicle's stage exactly; only `docking_port` and `transfer_connector` entries may be
  added (any other key, a missing stage / vehicle, a propellant mismatch or a cycle is a `ConfigError`).
  `tools/vehicle_report.py` follows `from`.
- Layout [E] (SpaceX publishes no positions): interface plane 0.5 m off the dorsal hull (`[20, 0, 5]`, axis +z,
  up +x), probes / drogues at stations 12 / 28 m, y +-1.5 m; connectors at 8 m (LOX) / 24 m (CH4), y 0. Mated
  (`vehicle::mated_stage_transform`, the i5 convention): parallel, rolled 180 deg, centrelines 10 m apart, 1 m between
  the hulls, every probe on a drogue and every connector on its counterpart (test "Tanker and HLS mate back to back
  with the depot").
- `Vehicle::set_tank_mass(stage, tank, kg)`: clamped to `[0, capacity]`; a refilled feed source re-arms starved
  engines, the cold-gas RCS follows the tanks.
- Ports mate only with equal mechanisms, probes on the chaser (`vehicle::port_mismatch`);
  `FlightSession::set_rendezvous_target` refuses others (German message). The session arms the IDSS or the
  probe/drogue mechanism for an active / passive pair of the same kind (r3).
- Tanker translation after a transfer (10 t per main tank, 310 t; pure capability, no torque): +-z (dorsal, the
  docking axis) 11.3 kN = 0.037 m/s^2, +-y 5.7 kN = 0.018 m/s^2, +x 3.0 kN = 0.010 m/s^2 (settling), -x none
  (r3: 3.0 kN with the docking pair).
- `vehicle_report`: depot ship ideal delta-v 7,328 m/s (no recovery reserve), tanker ship 6,697 m/s.
- HLS leg-strut clearance at the depot (feet 12 m out, 1.5 m below the skirt): checked in r3, clear.
- **Tests:** `[refuelling]` (`test_refuelling_vehicles.cpp`).

### Boil-off by heat balance (r2)
- **Model** (`vehicle/boil_off`): every hull tank (main tank of the stage diameter) of a stage with `[stage.thermal]`
  is a cylinder of its full-load length, 36 strips around the circumference. Per strip a quasi-steady balance
  `absorbed = eps sigma T^4 + (T - T_boil) / R_wall`, solved by Newton; the strips' leaks summed are the tank's heat,
  clamped at 0 (no condensation), over the latent heat the vented flow. Starship's hull *is* the tank wall (integral
  tanks, no vacuum jacket): the bare hull's `R_wall` is tiny; the vacuum inside tiles / felt and between MLI layers is
  part of their effective resistance.
- **Sources** (`environment/thermal_radiation`): Sun 1361 W/m^2 at 1 AU with the Earth / Moon disk occultation;
  Earth albedo 0.30 x solar flux x view factor x `max(0, cos zenith of the Sun below the vehicle)` [E]; Earth IR
  239 W/m^2 x view factor (absorbed with the emissivity). View factor: exact plate-to-sphere (tilted element,
  horizon-crossing formula). Spherical Earth of mean radius.
- **Wetted wall** (Task 5b, user 2026-10-20): only the wall the liquid touches feeds it; on the dry part the heat
  must also cross the ullage gas (`ullage_resistance_m2k_w` 100 m^2K/W [E]: gas conduction, no convection near free
  fall), so a dry wall runs hot and re-radiates. Where the liquid lies: the apparent gravity at the tank's centre =
  Earth tide + (-specific force: drag at the end of each coast, thrust in ticks) + centrifugal of the rotation
  (`vehicle::apparent_gravity_body_mps2`); Bond number rho a r^2 / sigma (`surface_tension_n_m` [E]): Bo <= 1 the
  wetting liquid spreads over the walls (Centaur's zero-g coasts), Bo >= 10 settled below a flat surface across the
  apparent gravity (`settled_wetted_fractions`), log-linear between [E]. At 200 km drag ~1.2e-5 m/s^2 and the tide
  ~4e-5 m/s^2 give Bo ~20-90 for a full-radius tank: settled. A 10 t residue loses ~1.4 t in 22 h (fully wetted it
  would have been ~20 t); methane goes relatively faster (the forward CH4 tank's residue lies along a side wall).
- **Simplifications** (switchable as a whole by the fidelity flag `boil_off`, default on; `FlightSession::set_boil_off`):
  no skin heat capacity, no conduction around the circumference (also not from the dry to the wet part); no tank
  pressure / temperature / subcooling / stratification (the loaded subcooling is not credited: conservative); the dry
  wall's small leak is vented like boil-off; the liquid geometry is quasi-static (no settling lag, no slosh), Euler and
  Coriolis terms left out; domes, ends, header and RCS tanks take no heat; no shadow of other vehicles or flaps; the
  Moon only shadows (its albedo / IR are not modelled).
- **Deorbit planning** (`FlightSession::plan_ship_return`, re-plans): the earliest pass is checked by flying a copy
  of the ship ahead to the node (free drift under the air's torque turns the hull - a fixed-attitude average was off
  by up to 80 %); LOX and CH4 boil off in another ratio than the Raptor burns them, so the scarcer one limits the burn;
  usable >= 1.10 x the burn propellant [E] (1.05 ran dry on the last tonne). Short: refused with "... verdampft ...";
  a re-plan that finds the tanks short calls the burn off (the ship stays in orbit). `DeorbitPlan::boil_off_kg`.
  A burn whose engines all starve is cut off with `FlightSession::burn_abort_reason()` instead of waiting for ever
  (found when the first r2 runs hung in `while (Burn)`).
- **Missions:** `ship_return_starbase` starts with 50 t [E] (was 45 t); the ship return campaigns size their start in
  2 t steps until the onboard plan holds (+0-10 t by orbit phase). The Block 3 lunar missions (`ship_lunar_return_*`,
  `ship_lunar_aircatch_llo`) set `[mission] boil_off = false` (user 2026-10-20): a bare ship cannot keep its
  propellant for days (78.8 t in 93.5 h fully wetted) - off until r5 (Moon radiation, thermal attitude).
- **Flight:** `FlightBody` evaluates every 10 s of free flight [E] and after every coast, vents by the trapezoid of
  the last two evaluations per tank (`Vehicle::set_tank_mass`, stops at 0), not below 100 km in air [E], not clamped /
  caught / landed / resting / docked or carrying a visitor (a docked pair: the session, r4). `FlightBody::boil_off_rate()` = flows at the last
  evaluation. Separation clears the stored rates.
- **Orbit average** `vehicle::orbit_average_boil_off` (for r6): one two-body orbit, LVLH-held attitude, Sun fixed,
  Earth shadow.
- **Validation** (`[boil_off][validation]`, `tools/validation/centaur_boil_off_calibration.py`): Centaur LO2 sidewall
  fluxes from flight reconstructions (De Kruif & Kutter, AIAA 2007-5845, Table 2; sunlit coasts, broadside, rotisserie;
  coast geometry [E]: axis along the orbit normal, Sun in the plane at noon mid-coast):

  | Flight | Insulation, orbit | Flight W/m^2 | Model W/m^2 |
  |---|---|---|---|
  | AC-206 | white foam, 167 x 1,132 km | 72.6 | 72.5 (calibration: R = 1.586 m^2K/W [D]) |
  | AC-167 | white foam, 185 x 267 km | 78.9 | 76.3 (-3 %) |
  | AV-003 | white foam, 167 x 4,165 km | 78.9 | 63.1 (-20 %) |
  | TC-11/15 | 3-layer MLI, transfer orbit | 9.46 (avg of 6.3 / 12.6) | 9.46 (calibration: R = 8.22 [D], sun only) |

  Tolerance 25 % (the sister flights TC-11 / TC-15 differ by 2x; coast geometry unknown). Paint alpha 0.25 / eps 0.88
  and MLI cover 0.30 / 0.80 [E].
- **Values:** depot `R_wall` 63.7 m^2K/W [D] = MLI-3 x 31/4 (30 effective layers [E], leak ~ 1/(n+1)), cover
  0.30 / 0.80 [E]. Block 3 ship (and tanker): bare stainless 0.40 / 0.15, 0.01 m^2K/W [E]; tiles 0.85 / 0.85,
  1.5 m^2K/W, +-90 deg around the belly [E]. Methalox: LOX 90.19 K / 213.1 kJ/kg, CH4 111.67 K / 510.8 kJ/kg
  [S: NIST]. The HLS has no `[stage.thermal]` yet (r5, with the Moon's radiation).
- **Numbers** (200 km, 26.5 deg, LVLH identity = nose +V, dorsal nadir; `STARSIM_VERBOSE=1`): depot (full)
  444 kg/day = 0.028 %/day at beta 0, 461 kg/day = 0.029 %/day at beta 60 deg; bare tanker 15.9 t/day (beta 0),
  22.9 t/day (beta 60). Plausibility only: conventionally insulated cryogenic tanks in orbit ~0.1 %/day.
- Open: active cooling (solar-powered cryocoolers, user question 2026-10-20) would be a cooling power per tank
  subtracted in the balance - not modelled, SpaceX has announced none.
- **Tests:** `[boil_off]` (`test_boil_off.cpp`).

### Starship rendezvous and probe/drogue docking (r3)
Plan `docs/superpowers/plans/2026-10-20-refuelling-r3-rendezvous-docking.md`. The ISS stack (station body, phasing
planner, proximity guidance, docking mechanism and sim, compound) is generalised; the ISS paths are unchanged.
- **-x docking thrusters** (user 2026-10-20): the tanker (`[[stage.rcs_pod]] docking` on its reused ship stage) and
  the HLS (forward pod + "aft") get two 1,500 N cold-gas thrusters pushing along -x [E] at station 46 m. Without
  along-track thrust `v_along + 2 n x_radial` is conserved (CW), so +V thrust alone can only raise the tanker under
  the depot - holds and retreats below it, and the departure, need -x. The Block 3 ship keeps none. Translation at
  10 t main propellant: equal +-x (3,000 N each). `Vehicle::rcs_supply_kg()`: main-tank cold gas counts the main
  oxidizer + fuel tanks (not the headers: the landing reserve), else the RCS tanks; proximity and phasing budget it.
- **Target block** (`[stage.station.proximity]`, `sim::with_station_limits`): keep-out radius, approach ellipsoid,
  Waypoint 0 / 1 / 2 and fly-around radius override the code's ISS defaults - and a caller's own values (f4: safety
  limits of the target a caller must not loosen) - only where the target sets them; the ISS sets none. The depot takes the ISS values [E] (200 m, 4 x 2 x 2 km, 400 / 220 / 20 m), flies the ideal LVLH
  attitude nose along +V-bar, dorsal side (the drogues) to nadir. A station is one stage: `add_station` refuses a
  stack ("the depot without its booster", `config::top_stage_config`). Target drag without a fixed area: C_D 2.2 on
  the end-on disk.
- **R-bar from below** (`gnc::DockingApproach::RBarBelow`, port axis dominant towards nadir): Waypoint 0 400 m below
  the depot lies on its port axis - the guidance flies the axial approach straight up from there (new rule: a chaser
  on the axis beyond an on-axis waypoint inside the corridor approaches along it; the ISS's Waypoint 0 lies on no IDA
  axis). No fly-around, no CW transfer.
- **Probe/drogue interface** (`contact::probe_drogue_contact`, all [E]): four probe heads (radius 0.05 m) in conical
  drogues (mouth 0.35 m, seat 0.06 m, depth 0.50 m; capture radius 0.30 m); per head the cone wall, the seat (the
  probe's attenuator) and the plate beside the mouth, each a spring-damper with friction 0.1; the heads' forces give a
  torque about their centre. A head on its seat within 2 deg tilt / roll latches (soft capture).
- **Mechanism** (`contact::probe_drogue_mechanism`): the IDSS phase machine with the probe stroke (0.30 m) as the
  ring extension (first contact at 0.80 m port distance), four structural latches as the hooks (180 s). Joint:
  translation `k = m_c w^2`, rotation `k = I w^2`, w = 0.5 rad/s, critically damped; a 0.10 m/s contact peaks at
  ~`2 m_c w v` (49 kN for 490 t). Limits [E] ("Auslegungsgrenze"): dynamic compression 100 kN, static 60 kN,
  tension 60 kN, shear 50 kN, bending / torsion 2 MN m; 0.25 m/s overloads. Separation springs 150 J (~0.03 m/s).
  Envelope (spec): closing 0.03-0.10 m/s, offset <= 0.10 m, angle and roll <= 1 deg. The session arms the mechanism
  for an active / passive pair of the same kind; contact plane, re-arm distance and the DockingSim coast limit come
  from the mechanism (`first_contact_m`, `reach_m`).
- **Hull collision:** only a nose port (axis +x, Crew Dragon) ends the sampled hull below it - the ships' dorsal port
  cut the tanker at 19 m before; the HLS legs are sampled along their struts (root 4 m above the stage bottom [E] to
  the deployed foot, 12 m out). The depot has two capsules [E] (tank section r 4.5 m, nose r 2.5 m). Mated, the
  tanker's hull clears the depot by ~1 m, 1.5 m closer it collides; the HLS legs clear it (r1 open point closed).
- **Phasing** (`plan_rendezvous` accepts Starship): burns on the deorbit set - the single belly Raptor
  (`sea_level_3`, spec correction: the vacuum group lights its three together) - with transients up to 0.6 of the
  burn; below that (~7 m/s) the burn flies on the cold-gas RCS along +x and is re-targeted with that model. Budget:
  `rcs_supply_kg` (the main tanks). Between the burns a Starship holds the docking orientation (LVLH rolled 180 deg).
  Refused with no main propellant ("Kein Rendezvous: kein Treibstoff").
- **Compound refresh** (`FlightSession::set_tank_mass`, r4's interface): a tank change of either docked body moves
  the compound CG (the station body's state), keeps the velocity (internal transfer) and the hulls in place; the
  attached mass follows the chaser's hull-fixed CG; `compound_mass()` is recomputed.
- **Numbers** (tanker 200 t main propellant, depot 600 t, 200 km / 26.5 deg):
  - phasing from 170 x 190 km, 8 deg behind: 5 burns (Raptor and cold gas), 12.0 t main propellant (burns, RCS,
    boil-off), Waypoint 0 missed by 13.9 m / 0.13 m/s;
  - Waypoint 2 -> docked (session test): contact at 0.075 m/s, offset 0.07 m; contact -> docked 10 min; peak
    compression 19.1 kN;
  - campaign from the phasing orbit (`IssMissionSequence`, port `transfer_dock`): docked after 12.6 h (Waypoint 0
    -> contact 49 min, contact at 0.075 m/s / 0.05 m), peak compression 22.0 kN;
  - launch window from Starbase: the 26.5 deg plane crosses the pad every ~22 h (the two daily crossings merge just
    above the pad latitude), but within the crew phasing limit of 28 h the 170 x 190 km tanker catches up only
    ~25 deg while the depot's phase moves ~90 deg per day: the first feasible window was the 8th crossing (177 h),
    none in the following 12 days. Open for the r6 cadence: a longer uncrewed phasing limit or a lower phasing orbit.
- Diagnostics: `STARSIM_DEBUG_RENDEZVOUS`, `STARSIM_DEBUG_PROXIMITY`, `STARSIM_DEBUG_DOCKING` as for the ISS.
- **Tests:** `[refuelling]` with `[rcs]`, `[config]`, `[station]`, `[proximity]`, `[contact]`, `[docking]`,
  `[rendezvous]`, `[launch_window]` (`test_refuelling_docking.cpp`, `test_refuelling_rendezvous.cpp`; the campaign
  and the undocking are `[slow]`).
- Deferred: the depot's attitude-hold gas (r6), HLS thermal (r5). Docked boil-off: done in r4.

### Propellant transfer (r4)
Plan `docs/superpowers/plans/2026-10-20-refuelling-r4-propellant-transfer.md`.
- **Hardware** [E] (SpaceX publishes nothing): per `[[stage.transfer_connector]]` `line_diameter_m` 0.15,
  `discharge_coefficient` 0.6, `chilldown_kg` 150 (LOX) / 60 (CH4) - ~300 kg of line and coupling steel from ~250 K
  over the latent heat, +50 %; per ship `[stage.propellant_transfer]`: the delivering ship holds its ullage at 3.5 bar
  (spec 3-4 bar), the receiving ship vents to 1.5 bar. Tanker, depot and HLS carry both.
- **Flow** (`vehicle/propellant_transfer`): orifice law `mdot = C_d A sqrt(2 rho dp)` per line, the coupled pair's
  smaller C_d A, densities of the loaded propellant (subcooled): LOX 232 kg/s, CH4 141 kg/s at 2 bar. No back-flow,
  no hydrostatic head (~0.2 kPa under the push).
- **Settling:** the chaser's +x cold gas pushes the compound at 1e-4 g [E, spec] (capped by its pure +x capability;
  gas at the RCS Isp from its main tanks in proportion to their oxidizer and fuel). Modelled as the average thrust,
  applied as one velocity impulse per 10 s step (0.01 m/s) to the compound - no pulses, no torque (the depot's ideal
  attitude takes it). Settle time `sqrt(2 L / a)` [E] for the longest involved tank (186 s for 16.9 m). A delivering
  outlet (aft dome, -x) is covered when the apparent gravity at the tank centre - tide and rotation about the compound
  CG (`LiquidEnvironment::cg_offset_body_m`), minus drag and push - gives Bo >= 10 (r2 `settled_weight` = 1) within
  30 deg [E] of -x (`vehicle::outlet_check`). Drag alone pushes the liquid to the nose (the depot flies nose along +V),
  the push brings it aft.
- **Sequence** (`sim/propellant_transfer`, pure state machine): Settling -> Chilldown (120 s [E], the chill-down
  mass vented from the delivering tanks) -> Flowing -> Complete; an uncovered outlet pauses Chilldown / Flowing
  ("Transfer unterbrochen: Treibstoff nicht am Auslass", without gas "... kein Kaltgas zum Setzen"), a covered one
  resumes; a pause longer than `max_pause_s` (600 s [E]) aborts ("Transfer abgebrochen: ...": the push would drain the
  reserve and block the undocking). A line closes at the delivering floor, a full receiving tank or its limit; floors hold at the line's closing
  (gas and boil-off after that come out of the reserve's items: tens of kg).
- **Session** (`FlightSession::start_transfer(TransferRequest)`, `stop_transfer`, `transfer_state`;
  `flight_session_transfer.cpp`): both directions (`ChaserToStation` tanker -> depot, `StationToChaser` depot -> HLS
  with per-line limits); the push always comes from the chaser. Steps every `request.step_s` (10 s) of epoch - the
  coast is limited to the next step, the step fires within half a tick (the Epoch at 8e8 s past J2000 rounds to
  ~1e-7 s, so sums of ticks missed a 1e-9 test by one tick). Tank moves through `set_tank_mass` (compound refresh).
  Refusals (German): not docked, autopilot active, no coupled connectors, no transfer pressures, nothing to move;
  a second start ("Transfer läuft bereits") and undocking ("Abkoppeln nicht möglich: Treibstofftransfer läuft") are
  refused while one runs; a cleared target, a detach or a new scenario on the session stops it.
- **Docked boil-off** (r3 M2): a station carrying a visitor (and the held chaser) no longer vents in `FlightBody`; the
  session evaluates both bodies every 10 s and after every coast in the compound's environment and vents through
  `set_tank_mass` (trapezoid), so the compound CG and the mated hulls stay exact (the depot venting by itself had
  moved the port gap by 0.11 mm in 2 h). `FlightBody::boil_off_rate()` shows the session's value.
- **Tanker reserve** (`mission::tanker_reserve`, `FlightSession::tanker_reserve` / `tanker_transfer_request`): the
  deorbit burn by the rocket equation from the mass after it (ship without main propellant, headers full, + gas
  1.5 t [E] + unusable 1 t [E]) with the delta-v of `ship_return_preview` (plan_ship_return's planner without the
  boil-off check; 8-day pass search [E]: from the 26.5 deg depot orbit no pass came within reach in 48 h), x 1.10
  (r2 commit margin), + gas + boil-off until the burn 2 t [E] + unusable; split at the deorbit set's mixture ratio.
- **Numbers** (tanker 200 t main propellant -> depot 600 t, 200 km / 26.5 deg, floors 10.0 / 2.8 t [test]):
  940 s (186 s settling + 120 s chill-down + ~630 s LOX flow), delivered LOX 144.6 t / CH4 40.3 t, settling gas 1.83 t
  at 1,333 N, chill-down 0.42 t, boil-off of both ships 12.6 kg; tanker left with LOX 10.0 t / CH4 2.75 t. Depot -> HLS
  (50 t aboard) 20 / 5.5 t: 400 s, 595 kg of the HLS's gas. Docked 2 h without transfer: tanker (200 t) 532 kg, depot
  (600 t) 32 kg. Tanker reserve at the depot: deorbit 59.4 m/s -> burn 5.6 t, reserve 10.1 t.
- **Tests:** `[refuelling][prop_transfer]` (`test_refuelling_transfer.cpp`), with `[config]`, `[boil_off]`,
  `[docking]`, `[hls]`, `[deorbit]`, `[mission]`.

### Reference flights (r5)
Plan `docs/superpowers/plans/2026-10-20-refuelling-r5-reference-flights.md`, ledger
`.superpowers/sdd/2026-10-20-refuelling-r5-reference-flights/progress.md`.
- **Mission chain** (`IssMissionSequence`, `flight_session_iss_mission.cpp`; the ISS paths unchanged): new legs
  `Raise` (after the ascent the apsis burns to `raise_to`, the window's phasing orbit - f1 below - flown as phasing
  burns: belly Raptor with transients up to 0.6, else cold gas), `Transfer` (Docked -> `start_transfer`: the tanker down to
  its reserve, `MissionTransfer::TankerReserve`, or the chaser's main tanks up to full, `FillChaser`) -> undock ->
  Departure, the ship return (`ship_return`: `release_tower`, `plan_ship_return`, Return until caught) and
  `WaitContinue` (coasts to `continue_epoch`, then `continuation`: the lunar mission). `ascent_to_station`: the
  inserted depot becomes the station (`make_station`: ideal LVLH attitude at once, the turn is not flown). The Ascent
  leg ends only after the engines' tail-off (the cutoff command left ~12 m/s at 5 g: the apoapsis was read 35 km low,
  the raise skipped) - of the controlled body only: the separated booster's tail-off does not move the ship's apsides. `max_capture_retries` (refuelling 2 [E], ISS 0: the pilot resumes) resumes a proximity flight
  waiting at Waypoint 2 after a retreat, then "Andocken fehlgeschlagen nach N Neuversuchen" (the light HLS bounced off
  the drogues once at 0.08 m/s with 6 mm/s lateral; the chain had waited forever at 200 Hz). Leg marks
  (`iss_mission_marks`: mass, main propellant, RCS supply, station main propellant) are the measured numbers.
- **Caught booster** (`release_tower`): before the tanker's deorbit burn the caught booster is removed from the
  session (indices shifted) and the tower re-armed - the real booster is lifted off within hours, the ship returns days
  later.
- **Departure burn** (gnc, all chasers): the target velocity is the free CW drift of the post-impulse state (before: a
  fixed LVLH velocity; the Coriolis term ~2 n v beat the filled HLS's ~2.5 mm/s^2 of cold gas: 466 t fired in 9.8 h,
  the HLS was lost). The filled HLS now leaves the nearly empty depot in 143 min on 1.96 t. This also changed the ISS
  departure (i5 section: its numbers predate it; the departure tests pass unchanged).
- **Missions** (`config/missions/`, `config/refuelling_mission_config.hpp`, `sim/refuelling_mission`; started at the
  config's epochs): `depot_launch` (`depot_ascent`: 200 x 200 km), `tanker_flight` / `tanker_flight_orbit`
  (`tanker_ascent`), `hls_refuel` / `hls_refuel_orbit` (`hls_ascent` on `starship_hls_launch` = Block 3 booster + HLS
  stage). The depot of the tanker / HLS flights is placed on a synthetic orbit (`[depot]`, J2-advanced mean elements);
  `hls_refuel` puts it in the lunar mission's parking plane (`lunar_parking_state` at the lunar epoch, converted to
  the true equator frame and run back by J2: 0.031 deg off after two days). The lunar landing start is split:
  `continue_lunar_landing_mission` plans the outbound flight and the budget from the live HLS ("zu spät" after its
  epoch). New config `hls_lunar_llo_depot` (26.5 deg, 2026-11-02).
- **Tanker** (user decisions 2026-10-20, [E] placeholders until the real tanker is known): with the flown Block 3
  cargo ship (255 t) it reached 170 x 190 km with 48 t and delivered 17.6 t. Lever probes (`[.r5_probe]`
  "Tanker ascent levers", all in 170 x 190 km): full booster throttle 26 t, kick 12/16 deg 51/34 t, insertion at 120 km
  +9.7 t (80 km: the coast's drag ~60 m/s brought it down), ship dry -10/-20 t +10/+20 t, tanks +100 t 40 t (the
  booster pays the mass), +200 t booster abort; lighter booster or less landing reserve break the catch - the booster
  return reserve is the bottleneck. Chosen: ship dry **209 t** (`dry_mass_kg` override on its `from` stage: no cargo
  doors, dispenser, interior; heat shield, flaps, headers stay), insertion **120 x 190 km**, raised to 170 km.
- **Tanker reserve** from the flown return [D] (r4 items [E] left the caught tanker with 0.00 t): gas 3.5 t (departure
  0.3 t + entry ~3 t), boil-off 2 t split 56 % CH4 (0.9 of 1.6 t in 19 h).
- **Numbers** (2026-10-20 epochs, depot 600 t for the tanker, 1,500 t for the HLS):
  - `depot_launch`: station 453 s after liftoff, 199.6 x 211.7 km, initial fill **55.5 t** main, boil-off 17.6 kg in
    90 min, booster caught;
  - `tanker_flight` (pad): window 79.8 h after earliest (48 h uncrewed phasing; the crew's 28 h: 177 h), SECO
    444.3 s, arc 12.17 deg [D], residue **99.2 t**, rendezvous 17.8 t (phasing 39.7 h, incl. the bare tanker's
    boil-off), delivered **LOX 54.6 / CH4 12.9 t** in 550 s, gas 0.94 t, chill 0.42 t, main at undock 12.6 t, caught
    with 2.74 t, 47.1 h;
  - `tanker_flight_orbit`: rendezvous 15.1 t, delivered 56.4 / 14.4 t, caught with 1.44 t, 34.3 h;
  - `hls_refuel` (pad): SECO 428.3 s, arc 11.68 deg [D], residue 216.4 t, after the raise 208.3 t, docked with
    184.5 t, filled to **1,585 t** (depot 1,500 -> 83 t), waited 5.3 days (2.7 t boil-off), lunar mission planned from
    **1,580 t**; `hls_refuel_orbit`: docked 195 t, TLI 3,152 m/s.
- **Moon radiation** (`environment::earth_orbit_radiation` with a Moon): albedo 0.11 [S] and infrared from the
  sub-vehicle regolith in radiative equilibrium, night 95 K [S] (Diviner); negligible in LEO (view factor < 3e-5). The
  HLS is insulated like the depot [E] (absorptivity 0.30, emissivity 0.80, wall 63.7 m^2K/W): ~0.2 t/day in transit;
  Mission 1 (LLO) and the lunar campaigns still close. The Block 3 lunar return with boil-off on (probe): the braking
  burn (1,032 m/s) is aborted for want of propellant after 3.4 d - `ship_lunar_return_*` keep `boil_off = false`.
- **Not modelled** (r6 loss items): the pressurant for the delivering tank and the vented displaced gas of the
  receiving one (~1-2 % of the transfer [E]).
- Diagnostics: `STARSIM_DEBUG_PEG` (the ascent cutoff: radius, speeds, v_go, tail-off) besides the r3/r4 switches.
- **Tests:** `[refuelling][mission]`, `[iss_mission]`, `[station]`, `[catch]`, `[departure]`, `[lunar]`
  (`test_refuelling_missions.cpp`, `test_refuelling_mission_config.cpp`; campaigns `[slow]`; probes `[.r5_probe]`).

### Campaign calculator (r6)
Plan `docs/superpowers/plans/2026-10-20-refuelling-r6-campaign.md`, ledger
`.superpowers/sdd/2026-10-20-refuelling-r6-campaign/progress.md`.
- **Calculator** `mission::refuelling_campaign` (pure): the depot loses boil-off, drag makeup and attitude gas in daily
  steps; tankers arrive at their windows (pad turnaround `launch_interval_s` 1 day [E], one depot port: arrivals 6 h
  apart [E]) and add their delivery less 1.5 % vent [E] (pressurant + displaced gas, not in the flights), clamped to
  the tank volume; once the fill reaches the demand the HLS window is checked - the campaign closes when the depot still
  holds the demand at the HLS arrival. German reasons: depot volume (also: full tanks that cannot bridge the losses
  until the HLS arrives), no window, a tanker delivering nothing (also NaN), losses eating the delivery (against the
  fill two tankers back: alternating window gaps), 60 tankers. An initial fill above the volume is clamped; the levers
  are checked by `sim::campaign_lever_error` (CLI and game).
- **Setup** `sim::plan_refuelling_campaign` (levers `CampaignLevers`, std::nullopt = reference): t = 0 at
  `tanker_flight`'s earliest (2026-10-20), the depot on its `[depot]` orbit at the altitude lever with the measured
  initial fill; windows by the flown logic (`plan_launch_window` against the depot, phasing orbits shifted with the
  depot, search 60 days [E] - the missions' 12 days found no second window; of the launches reaching the same
  Waypoint 0 (within 3 h [E]) the latest is flown - the shortest phasing, the least tanker boil-off); depot boil-off by
  `orbit_average_boil_off`
  with the day's fill and Sun (identity in LVLH); drag end-on (Cd 2.2, pi (d/2)^2, US76) made up by one Raptor
  `sea_level_3`; attitude gas 20 kg/day [E]. Tanker net = measured 67.5 t + its own boil-off over the phasing
  difference to the reference window (45.2 h separation -> Waypoint 0) + dry lever x 0.99 [D] + reserve lever 1:1 -
  altitude cost (Hohmann up from 200 km at the docked mass, deorbit difference to a 50 km perigee [E] at the return
  mass x 1.10). HLS demand: `hls_lunar_llo_depot` placed in a scratch session, outbound planned, budget legs
  (`lunar_mission_legs`) -> the smallest closing mass (`lunar_budget_required_mass`); demand = (required propellant +
  HLS boil-off while waiting - propellant at docking 184.5 t) x 1.015 + chill 0.42 t.
- **Measured numbers** `config/refuelling/reference_flights.toml` [D]: checked by the slow pad flights (`depot_launch`,
  `tanker_flight`, `hls_refuel`; 1 % or 0.3 t via `refuelling_test::check_reference`) - re-measure on any change.
- **Answer of r6** (superseded by f1 below; reference, 200 km, 48 h phasing): **11 tanker flights, 156.9 days** (HLS docks 2027-03-26 with the
  depot at 924.6 t); demand 853.6 t (the HLS needs 1,145.1 t: dry 120 t + 1,025 t propellant of 1,600 t), deliveries
  80.5-96.4 t (the latest launch to each arrival phases 7-25 h instead of the reference's 45.2 h: less tanker
  boil-off, ~12 t/day), losses boil-off 70.8 t, drag 4.3 t, gas 3.1 t, vent 14.4 t. Public counts 4 to high teens
  ("10-ish"): plausible, not tuned.
- **Cadence finding:** the arrivals (5.2, 19.0, 32.8 d, ...) recur every ~13.8 days whatever the phasing limit - with
  the fixed 170 x 190 km phasing orbit the depot's phase gap at the daily Starbase crossing shrinks by about what the
  tanker catches up in a day: launches on consecutive days reach the same Waypoint 0. A longer limit adds no arrivals
  (72 / 96 h: 11 / 157 d as 48 h), a shorter one loses some (28 h: 11 / 171 d). More arrivals need another phasing
  orbit per window (not modelled). 300 km: 13 / 112 d (altitude cost 8.0 t per tanker, windows 5 and 12 days apart),
  400 km: 14 / 125 d; tanker dry -10 t: 10 / 143 d; depot insulation x 0.1: 31 / 433 d (1,851 t boil-off); HLS waiting
  5 days: 11 / 157 d (the last delivery covers it); launch interval 12 h: unchanged (the windows rule).
- **CLI** `build/core/refuelling_report.exe [--height-km] [--interval-h] [--phasing-limit-h] [--tanker-dry-t]
  [--reserve-t] [--initial-t] [--insulation] [--hls-wait-d] [--reference-phasing]` (~1 s; exit 0 closes, 2 open,
  1 error).
- **Game:** `FlightSim.transfer_state()`, `depot_state()`, `refuelling_campaign(params)` (worker command, result in
  `result(id)["data"]`; keys in `bindings/src/flight_sim.hpp`).
- **Tests:** `[refuelling][campaign]` (`test_refuelling_campaign.cpp`: calculator against hand computations, costs,
  file, CLI parser/text; the setup's reference and lever campaigns `[slow]`), the reference checks in
  `test_refuelling_missions.cpp`, the smoke test.

### Phasing orbit per launch window (f1)
Spec `docs/superpowers/specs/2026-10-21-refuelling-followups-design.md` (f1), plan
`docs/superpowers/plans/2026-10-21-refuelling-f1-phasing-per-window.md`, ledger
`.superpowers/sdd/2026-10-21-refuelling-f1-phasing-per-window/progress.md`.
- **Why:** r6's arrivals recurred every ~13.8 d - one fixed 170 x 190 km phasing orbit closes only a narrow band of
  phase gaps in 48 h. Now each window picks its phasing orbit (the tanker only; the depot stays passive).
- **Schedule** (`mission::phasing_schedule`): a chaser **above** the target (periapsis more than
  `coelliptic_height_m` above the target's orbit, else "Bahn kreuzt die Zielbahn") waits until the lower, faster target
  has caught up to the same `required_at_boost`, then a retrograde boost descends to the coelliptic orbit
  (`PhasingSchedule::above`; the phase burn stays below-only; `PhasingPlanner` solves the signed boost unchanged).
- **Raise** (`gnc::next_raise_node`): below the depot one burn at apoapsis (periapsis), above it the apoapsis first,
  then the periapsis at the new apoapsis. Flown, the sequence is fixed at the start (`raise_periapsis_pending_`): the
  osculating apsides swing ~10 km with J2, a re-check kept adding burns. The next burn waits for the tail-off (the
  leg stays pending so the session ticks; a burn started during it counted the residual thrust as its ignition). A
  dry raise ends the mission with `burn_abort_reason`. Flown 120 x 190 -> 250 x 270 km planned: 263.8 x 270.0 km.
- **Window** (`mission::plan_launch_window`): `LaunchWindowParameters::phasing_orbits` (empty: the separation orbit,
  ISS unchanged to the microsecond - test) and `PhasingCostModel`; at each crossing every candidate's raise is
  estimated two-body (`raise_estimate`) and its schedule run from the raised state with the remaining limit; the
  cheapest feasible one wins (`phasing_cost`: raise + Hohmann boost to the coelliptic orbit (+ phase burn) by the rocket
  equation, plus boil-off to Waypoint 0). `LaunchWindow::phasing_orbit`, `cost`, `raised`, `raised_state_gcrf`.
  A candidate whose mean radius lies within 10 km [E] of the coelliptic height (below) or of the target's orbit plus
  that height (above) is refused ("zu nah an der Zielbahn", review I1: flown, the osculating apoapsis of 190 x 190 km
  swung over 197.1 km with J2 and the phasing plan refused it).
- **Config** `[phasing]` of `tanker_flight` / `hls_refuel`: `candidates_km` [E] (150, 160, 170, 180 x 190 km
  below; 220 x 240, 250 x 270 km above), `planning_mass_t` [D] 308 / 328, `planning_boil_off_t_per_day` [D] probe
  2026-10-21: tanker 15.9 (99.2 t main), HLS 0.45 (208 t), `window_margin_h` [E] 4: the window closes 4 h before the
  limit, the flown phasing keeps the full limit (a window needing 47.0 of 48 h failed its re-plan 40 h in) - counted
  from the separation as in the window (f4: the flown deadline used to start after the raise burns, ~1-3 h later).
  `periapsis_km = 170` stays as the reference set.
- **Calculator:** delivery = base + tanker boil-off over the phasing difference - (chosen raise/boost propellant -
  the reference window's); the HLS's extra over its reference window is `CampaignFlight::extra_demand_kg`, held in the
  depot at its arrival; `CampaignFlight::phasing_orbit`; the depot-height lever shifts the candidates (the insertion
  stays), so `altitude_cost_kg` keeps only the deorbit difference. Lever `reference_phasing` (CLI
  `--reference-phasing`, game key `reference_phasing`; tankers' `phasing_periapsis_km` / `phasing_apoapsis_km`).
- **Reference flights re-measured** (`reference_flights.toml`, 2026-10-21): tanker residue 99.8 t, rendezvous 15.4 t,
  delivered 56.5 / 13.9 t (70.4 t), at catch 1.94 t; HLS main at docking 187.6 t. Above-depot pad flight: 220 x 240 km,
  Waypoint 0 9.9 h after liftoff (window 10.0 h).
- **Answer** (200 km, 48 h): **12 tanker flights, 46.5 days** (HLS docks 2026-12-05; demand 850.5 t; losses boil-off
  21.0 t, drag 1.3 t, gas 0.9 t, vent 12.6 t); `--reference-phasing`: 12 / 170.7 d. Orbits chosen: 150 x 190 km when
  the depot is ahead, 220 x 240 / 250 x 270 km when it is behind; launches every ~1.95 d in runs of four per ~13.8 d
  phase cycle. **Cadence finding:** the pad turnaround of 24 h [E] against plane crossings ~23.4 h apart lets only
  every other crossing fly (`--interval-h 12`: 12 / 23.1 d); the gaps between the runs are phases no candidate closes
  in 48 h. Levers: 300 km 13 / 31.9 d; tanker dry -10 t 11 / 45.9 d; insulation x 0.1 16 / 60.2 d; 28 h 11 / 75.2 d;
  72 h 14 / 34.5 d - the calculator takes the first window (fewest days), so a looser limit opens earlier windows with
  longer phasing and more tanker boil-off: fewer days, more tankers.
- **Tests:** `[burn_node]` (`test_raise_to_orbit.cpp`), `[rendezvous][phasing]` (mirrored schedule, catch-up rate
  against 1.5 n da/a within 2 %, the planner's retrograde boost to Waypoint 0), `[iss][launch_window]` (candidates,
  ISS unchanged, a raise eating the limit, the cost by hand), `[refuelling][iss_mission]` (two-burn raise, dry raise),
  `[refuelling][config]`, `[refuelling][mission][slow]` (above-depot flight to docking), `[refuelling][campaign]`
  (HLS extra demand by hand, reference phasing and per-window campaigns `[slow]`), probe `[.f1_probe]`.

### Settling before an in-orbit ignition (f3)
Spec `docs/superpowers/specs/2026-10-21-refuelling-followups-design.md` (f3), plan
`docs/superpowers/plans/2026-10-21-refuelling-f3-settling.md`, ledger
`.superpowers/sdd/2026-10-21-refuelling-f3-settling/progress.md`. Fidelity flag `FlightEnvironment::ignition_settling`
(default on; `FlightSession::set_ignition_settling`): off means no push, no ingestion check and no reserve (the
catch tilt and the RCS allocator fixes below apply either way).
- **Settling state** (`Stage::settled` 0..1, `Vehicle::update_settling`): while every outlet of the stage's main tanks
  is covered (`outlet_check`: Bond number >= 10, apparent gravity within 30 deg of -x) the progress rises at
  1/sqrt(2L/a), otherwise it falls at the same rate; a = the weakest apparent gravity at the tank centres, no
  acceleration: unchanged. L = the longest ullage, tank length x (1 - fill), at least 1 m [E] (a near-full tank would
  settle and unsettle at once). Ticks and coasts update it (the coast with its end-of-segment residual acceleration);
  clamped, caught or landed: 1; every vehicle starts settled. Tanks with `propellant_management_device = true` [E]
  (Starship header tanks of Block 3 and the depot, Dragon `mmh_nto`) are always covered; header feed needs no settling.
  The tide is the orbited body's (`LiquidEnvironment::centre_gcrf_m`, `gm_m3ps2` from the tracked reference body;
  final review I3: the Earth's tide at the Moon, ~1e-10 instead of ~3e-5 m/s^2 in LLO, kept lunar-orbit tanks
  settled for days).
- **Gas ingestion** (`Vehicle::update_engines`, Off -> Starting): only in orbit (`FlightBody::in_orbit`: closed orbit,
  osculating periapsis above the reference body's equatorial radius) - suborbital and landed ignitions are not checked.
  An unsettled stage's engines fail (`failed`, `starved`); the session cuts off, disengages and sets
  `burn_abort_reason` "Gasansaugung: Treibstoff nicht gesetzt, Triebwerke ausgefallen"; ISS / refuelling mission
  sequences end on it.
- **Settling push** (`vehicle::settling_push`): cold gas along +x at the full pure-translation capability (Starship:
  3,000 N, 4.37 kg/s); lead = 1.25 [E] x sqrt(2L/a). The autopilot runs it from its lead until the engines give thrust
  (an RCS overlay on the command); its delta-v is credited to the burn (`BurnExecutor::credit`). An ignition before
  the stage is settled waits while the push still gives force, at most 3 leads [E]. Burns start `burn_lead_s()` =
  120 s preparation + the lead before ignition (lunar, Earth return, rendezvous schedules; the waiting, drift and coast
  limits include the lead): the turn to the burn attitude ends before the push - found: a TLI push during a ~1 deg/s
  turn (centrifugal ~1e-2 m/s^2 at the tanks against the push's 1.75e-3) left the HLS 0.91 settled at ignition, the
  ignition was held 3 s and MCC 1 grew from 3.2 to 11.1 m/s. Lunar PDI: the descent coast starts the push one lead
  before the braking engines only when not settled; while unsettled at the ignition it holds as the burns do (f4; it
  pushed always and lit without a hold before). The hold judges the push by its capability, not the
  last tick's measured push (final review I1: an ignition already due on the burn's first tick lit unsettled).
  Manual `FlightSession::settle(bool)` (game `FlightSim.settle(enabled)`): refused while docked, while a proximity
  approach flies (engaging one switches it off: the approach owns the RCS) or while an autopilot cold-gas burn or the
  rendezvous flies (f4: an RCS burn clears it), cleared by a new flight.
  Snapshot: `autopilot_status().settling`, `burn_abort_reason`, `body_stages()[i].settled`.
- **Numbers:** tanker / Block 3 / HLS leads 42-96 s, 0.2-0.75 t per push (HLS TLI 42 s / 0.49 t full, MCC at 609 t
  main 88 s / 0.74 t; tanker raise 80 s). **Finding:** gas ~ sqrt(F) for a given travel (t ~ 1/sqrt(F), mass flow ~ F):
  weaker cold gas thrusters would settle on less gas, at the cost of a longer lead.
- **Planner reservations** (Task 4): the session's phasing plans reserve the push with the tanks as now per burn
  (`PhasingParameters::reserve_per_burn_kg`), the window cost adds `PhasingCostModel::gas_per_burn_kg` per raise,
  boost (and phase) burn, the deorbit check requires the next push on top of its margin, the descent budget one push;
  budgets planned before the tanks drain use `vehicle::settling_gas_bound_kg` (the largest push gas at any fill; it
  peaks at a middle fill: longer travel against less mass) per in-orbit ignition - lunar mission budget and campaign
  HLS demand: all legs but the approach (continues the braking burn) and the surface ascent (lit landed).
- **Found and fixed:** `Vehicle::rcs_capability` rebuilt the flight's cached RCS allocator at the query's CG; the new
  planner query moved the attitude control's reference within the 1 cm tolerance and turned the depot approach into a
  retreat. Queries now use a local allocator when the cache is stale.
- **Ship catch** (b3): the pushes before TEI and the Earth braking shift the lunar-return gates by ~1.4 s / 20 m; the
  lateral phase's tilt limit is 20 deg since (see "Ship catch propellant margin"). Since the ship catch robustness
  plan all 21 direct seeds are caught (ship lateral LQR effort 0.03; upright abort; see "Ship catch propellant margin").
- **Step dependence (pre-existing, measured in f4):** the same 100 m/s prograde burn 2 h ahead, flown after 10 s or
  600 s `advance_to` steps (hidden case `[.step_dependence]`): from a 120 x 190 km orbit the states already differ at
  the ignition by 21 km / 35 m/s and end 122 km / 125 m/s apart; from 200 x 270 km by 156 m / 0.18 m/s and 13 m /
  0.011 m/s. The burn executor itself agrees (ignition within 50 ms, delta-v within 0.2 %). The cause is the free drift
  before the burn (`burn_drift_lead_s`: no attitude control): at a 120 km perigee the air torque tumbles the ship
  chaotically, so its drag area - and the orbit - depend on how the coast is segmented. Physical sensitivity of an
  uncontrolled ship in thick air, not a numerics fault; kept (decision f4). The f1 phasing orbits (>= 150 km) and the
  missions' burn re-targeting from the live state are not affected in practice.
- **Reference flights re-measured** (2026-10-21, f3): tanker residue 99.7 t, rendezvous 16.7 t (+1.3 t: the phasing
  pushes), delivered 55.4 / 13.6 t (69.0 t), at catch 2.13 t; HLS main at docking 187.3 t; full HLS departure 1.96 t
  of push gas. f4 again (below): rendezvous 15.3 t, delivered 56.5 / 13.9 t (70.4 t), at catch 1.81 t.
- **Answer** (200 km, 48 h): **13 tanker flights, 49.7 days** (f1: 12 / 46.5 d; HLS docks 2026-12-05 at 920.9 t;
  demand 861.2 t; losses boil-off 22.4 t, drag 1.35 t, gas 0.99 t, vent 13.6 t). f4: unchanged 13 / 49.7 d (demand
  868.3 t, depot at the HLS 938.8 t, vent 13.8 t).
- **Diagnostics:** `STARSIM_DEBUG_SETTLING=1` prints each push (start before ignition, settled, body rate; mean push
  against the capability; duration, gas, lead) and every held ignition.
- **Tests:** `[ignition_settling]` (`test_ignition_settling.cpp`: progress and travel, PMD and header feed, ingestion
  in / out of orbit and with the flag off, the push before a raise burn, coast stop in 10 s and 600 s steps, the held
  ignition, manual settle, the turn before the push, the allocator query, the gas bound and the session's estimates
  and reservations), `[rendezvous][phasing]` (reserve per burn), `[iss][launch_window]` (gas per burn by hand),
  `[refuelling][campaign]` (in-orbit ignitions of the lunar legs).

### Deferred minors (f4)

Plan `docs/superpowers/plans/2026-10-21-refuelling-f4-deferred-minors.md`: the minors of the r1-r6, f1 and f3 final
reviews, each fixed (with a test) or decided here.
- **Fixed - vehicle / config:** `set_tank_mass` refuses NaN / infinite masses; duplicate stage, docking-port and
  transfer-connector ids are config errors; hull sampling stops below the lowest nose port, whichever port is listed
  first; `tools/vehicle_report.py` resolves `from` stages by the C++ rules (cycle bound, `<vehicle>:<stage>`, allowed
  keys, lists appended; `tools/test_vehicle_report.py`); SRP constants from `constants.hpp` (`speed_of_light_mps`);
  probe/drogue pairing tests check one-to-one pairs and bad layouts; `[mission] port` of refuelling missions is a
  config error (it was parsed and never used: the chaser docks with its first port).
- **Fixed - boil-off / coast:** a hold (docked, caught, landed) and the release forget the last rates (the first
  coast after days docked averaged with the flow from before); a body resting on the Moon no longer boils in `step()`;
  vehicles without `[stage.thermal]` skip the evaluation; a coast that ends in the Moon's sphere tracks the Moon as the
  orbit reference (f3 M8); a 600 s-segment test through the eclipse matches the orbit average (3 %).
- **Fixed - burns:** failed engines report "Triebwerke ausgefallen" (not "kein Treibstoff"); the deorbit boil-off
  check uses the deorbit group's mixture ratio; a re-plan keeps the boil-off until its burn; rendezvous burns are
  modelled at the throttle they fly (minimum throttle between the two minimum impulses, r3 M1 - the reference tanker's
  rendezvous fell 16.7 -> 15.3 t).
- **Fixed - transfer:** the pressure difference per line; a chill-down after a pause vents again (the lines warmed);
  no transfer start while the hooks open; the settling push gives only the impulse of the gas it drew
  (`TransferState::settling_impulse_ns`).
- **Fixed - mission chain:** a generation counter keeps a sequence set by a continuation; a docked start passes the
  Docked gate (an extra Docked mark before Transfer); a collision during the transfer ends as "Kollision"; the second
  raise burn only raises the periapsis (f1 M1); the phasing deadline counts from the separation (f1 M2); the nose-cone
  body index shifts in `erase_body`; the reboost warning says "der Station".
- **Fixed - calculator:** the vent grosses up as net / (1 - f) (`mission::gross_for_net_kg`); the HLS's wait boils off
  with the Sun of its arrival (`CampaignModels::hls_wait_kg`) and must fit its free tank volume; the HLS's TLI starts
  from the depot altitude; the HLS's extra demand is signed and includes its phasing boil-off difference (the HLS
  window is checked before every tanker); every candidate's refusal reason is reported; `candidates_km = []` and
  `reference_phasing` without `periapsis_km` are refused, `window_margin_h` is read without candidates; the CLI flag
  parsing no longer relies on unsigned wrap-around; one source for the settling flag (`vehicle::ignition_settling_default`);
  lunar in-orbit ignitions per budget leg (`BudgetLeg::ignitions`: LLO 3 -> 5, NRHO 6/7 -> 9/10; a lower bound - MCCs,
  plane trim and NRHO station keeping are not budgeted legs). `reference_flights.toml` carries `inputs_digest` (FNV-1a
  of the configs the flights read): a config change without a re-measure fails the fast `[refuelling][campaign][config]`
  check (C++ model changes are not caught).
- **Fixed - bindings:** `iss_state` / `station_geometry` ports export `mechanism` and `elements_m`.
- **Decided, not changed:**
  - depot header tanks (35 t) stay aboard: transfer connectors take main tanks only, usable depot fill 1,600 - 35 t;
  - probe/drogue contact: the force steps from the cone wall to the mouth plate at the rim (mm penetration, 2 kN/cm);
    a nearest-face rule would move capture outcomes of the slow campaigns for no physical gain;
  - `load_stage_from` reloads the source vehicle per reused stage (milliseconds, once per load);
  - the FlightSim campaign command holds the simulation clock for ~0.5-1 s (`SessionRunner` accepts long calls, as a
    return plan);
  - `DeorbitParameters::boil_off_kgps` stays an offline planning knob (the session flies a copy ahead);
  - Raise tail-off wait without a timeout (f1 M7): no public path keeps an engine running with the autopilot idle in
    the Raise leg (the autopilot burn cuts pilot-lit engines; disengage + ignite re-plans the next raise);
  - the step dependence of burn end states (above): the free drift at a 120 km perigee, not the burn executor.

### Flown campaign (f2)
Plan `docs/superpowers/plans/2026-10-21-refuelling-f2-flown-campaign.md`, ledger
`.superpowers/sdd/2026-10-21-refuelling-f2-flown-campaign/progress.md`.
- **Driver** `sim::RefuellingCampaignFlight` (a `SessionSupervisor`: `FlightSession` calls `update` every tick and stops
  a time-warp call at `wake_epoch`): mission `refuelling_campaign_flown` (depot launched by `depot_launch`) or
  `refuelling_campaign_flown_placed` (depot placed with demand minus 1.5 net deliveries, two tankers). Phases
  DepotAscent -> Planning -> WaitLaunch -> Flying -> (StationKeeping) -> ... -> Complete / Ended. Every plan is the
  calculator re-planned from the flown depot (`CampaignLive`: its state, fill, last launch and arrival, 1 h pad
  readiness [E]); the first plan is the comparison's baseline. Flights are sequential (calculator rows never overlap);
  each launches with `keep_station` beside the kept depot (`launch_campaign_flight`); after it the chaser is dropped
  (`keep_only_station`). Only the first tanker flies its return, in a blocking side session (`place_vehicle`): caught,
  2.95 t main left. A failed flight re-plans from the next window. Booked on the depot: attitude gas 20 kg/day [E] (1 kg
  batches), the transfer vent 1.5 % [E].
- **Mean orbit:** `orbit::energy_mean_semi_major_axis_m` (the J2 energy, the osculating a swings +-2 km at 200 km /
  26.5 deg) and `station_mean_elements_from_state`; a pure point mass + J2 propagation holds its phase within +-20 km
  over 5 days (the eccentricity term). The raise to a phasing orbit sizes its burns from `orbit::j2_mean_orbit`
  (averaged over one revolution; the osculating apsides swing +-10 km) and raises a higher apoapsis at the periapsis.
- **Station keeping:** `[station_keeping]` below 195 km mean height (J2 energy), nothing in flight and the next
  arrival more than 24 h away -> a Hohmann pair to 200 km [E] from the energy-mean a (first burn 30 min ahead, the
  second half a transfer later, the engines' tail-off awaited). At 636 t the belly Raptor's transients exceed 60 % of a
  ~2 m/s burn, so the burns fly cold gas: 191 -> 200 km cost 6.5 t (Raptor ideal ~0.8 t). The full run needed none.
- **Depot orbit in the calculator** (user 2026-10-22): the flown depot drifts - drag lowers it (a free 572 t depot at
  198 km: 0.33 km/day) and every transfer's settling push raises it, prograde because the docked pair flies nose along
  +V-bar (0.547 m/s per tanker transfer, 0.527-0.566 = impulse / pair mass; the HLS's ~3.4 m/s). With a fixed orbit
  the first plan's depot phase was 2,600 km off after 25 days. `mission::DepotOrbitTrack` keeps the mean semi-major
  axis in segments (linear decay, the phase from the changing mean motion, -3/4 n/a da/dt t^2) with the keeping floor
  and target (also in the future the window search sees); `CampaignModels::depot_orbit_event` changes it at t = 0 and
  after every transfer (push `transfer_push_dv_mps` 0.55 [D], decay rho(US76) Cd 2.2 x end-on area x sqrt(mu a) / m x
  `drag_decay_factor` 1.076 [D]: the free depot 0.3294 vs 0.3062 km/day - its test guards this measurement; independent: the 14 flown post-transfer heights fit 1.08)
  and books the reboosts it flew. Flown heights 207.5 -> 209.8 km over 53 days; the first plan's track stayed within
  200 km of the flown depot until its windows parted (below).
- **Delivery calibration** (user 2026-10-21): the flown tankers delivered more than the calculator's model of their
  own windows - below the depot +0.6..+3.2 t (5 flights), above +1.0..+3.7 t (9), sd ~1 t (the rendezvous varies
  13-21 t); `campaign_offset_below_t` 1.7 / `campaign_offset_above_t` 2.25 [D] by the phasing orbit's periapsis
  against the depot's mean radius. The calculator books the tanker's phasing boil-off with the flown free coast's
  3.5 t/day (the orbit average at a fixed LVLH attitude said 15.9) and the HLS's 0.47 t/day [D].
- **Comparison** (user 2026-10-22; ascent Task 5 ruling 2026-10-23): count equal to the first plan; the fill after
  each delivering tanker within 2 % of the calculator's model of the flown window (the live plan made just before its
  launch, in the phasing class flown: `CampaignFlownRow::window_model` - the delivery model's check; until ascent
  Task 5 the first plan's k-th, whose class flips at thresholds); the phasing-class mix against the first plan -
  |tankers above the depot in the first plan's paired flights - tankers flown above| - at most one in four delivering
  tankers (at least 1, `max_class_mix_difference` [E]: a window-choice bias tilts the mix, the timing cascade does
  not); the pairwise class flips (by phasing orbit) only reported; each arrival (the HLS's: the duration) within 1 d of the live plan made just before
  its launch; the HLS holds its required propellant; the first return caught or upright abort. The first plan's
  duration is only reported: its windows flip at thresholds - the latest launch to the same arrival (3 h) differs
  under ~100 km of depot phase (tanker 6: plan 20.49 d, flown 19.51 d), the 24 h pad turnaround then moves every
  later launch and a window cycle drops out.
- **Full run** (`refuelling_report --fly`, ~22 min; ctest `Flown campaign: the depot launched*` `[slow]`): depot
  launched 2026-10-20, 14 tankers, 0 failures, HLS at the depot 59.44 d with 1,090.8 t (needs 1,041.4 t); first plan
  14 tankers / 53.2 d; arrivals within 0.07 d of their live plans; fills within 2 % (largest: tanker 13, 864.4 vs
  850.9 t, +1.6 %); deliveries 58.6-72.5 t; losses boil-off (remainder) 31.6 t, attitude gas 1.2 t, vent 27.3 t,
  station keeping 0. Placed variant (`--fly-placed`, ~5 min): 2 tankers + HLS, passes.
- **Calculator answer** (default, depot placed at 200 km): **12 tanker flights, 50.6 days** since ascent Task 5 (the
  planned kick: 74.25 t per reference tanker; ascent Task 2: 14 / 52.3 with 63.6 t; before 13 / 52.5, f3/f4: 13 / 49.7;
  with the per-class calibration and the depot's orbit followed). `--reference-phasing`: 14 / 224.7 d (the single
  phasing orbit against a drifting depot).
- **Known limits:** flights strictly sequential; the side return blocks the session; the calculator's keeping floor
  acts continuously (the flown rule checks at its plans and waits for 24 h to the next arrival) and books the reboosts
  at the Raptor's exhaust velocity (the flown light depot flies them on cold gas); the transfer push at the arrival
  (flown ~3 h later); reboosts after the last transfer unbooked.
- **Follow-ups:** a consistent J2-mean rendezvous (raise, schedule and planner on one mean model - the J2-mean
  schedule alone failed against the osculating raise/planner); the depot's cold gas keeps pushing with empty main
  tanks (headers counted in `main_dry`); light-depot reboosts on cold gas (6.5 t).
- **Game:** `FlightSim.campaign_flight_state()` (key `present`; rows, losses, reboosts, the first plan).
- **Tests:** `[campaign_flown]` (`test_refuelling_campaign_flight.cpp`: supervisor, keep_station, place_vehicle,
  reboost, live start, the free depot's decay, the placed and the full campaign `[slow]`), the track and the orbit
  events in `test_refuelling_campaign.cpp`.

## Crew escape (ejection seats on the crewed ship return)

Spec `docs/superpowers/specs/2026-10-22-crew-escape-design.md`, plan `docs/superpowers/plans/2026-10-22-crew-escape.md`.
User decisions (2026-10-22): every abort of a crewed ship return ends with the crew on ejection seats (a water landing
with crew is no option - the ship usually explodes); return only (ascent aborts later); the onboard computer
triggers, no pilot button; every crew member is its own body; the empty ship is flown to its impact; "early, over
water"; data + game interface first, the pilot's view rendered later; **the ship's data stay untouched** (no change to
`starship_block3.toml`: dry mass, CG, tanks, engines, aero) - seats and crew live in their own config.

- **Config:** `config/crew_escape/starship_crew.toml` (`config::load_crew_escape`): 4 seats in the ship's stage frame
  (x 43.0-46.6 m, z 3.5 m [E]), rails along body +z (the back, leeward in the belly flop) tilted 10 deg to the nose
  [E], ACES II-class numbers with sources: rocket catapult 22.2 kN for 0.55 s [S: Whiteman AFB] split into
  `catapult` + `rocket` tables, envelope 600 KEAS [S: DTIC ADA133628] / 50,000 ft [E], mode 1 below 250 KEAS and
  15,000 ft [S, secondary], drogue 0.16 s [S: LSI PAR-1610-00], separation 1.16 s [D: mortar 1.0 s after the drogue];
  stagger 0.4 s [E]; judging 20 g for > 0.1 s, 1 m clearance, plume cone 15 deg x 100 m [E], touchdown <= 8 m/s [S:
  AP3456]. Bodies `vehicles/ejection_seat.toml` (170 kg, fixed drag 0.9 m^2, drogue 0.73 m^2) and
  `vehicles/parachutist.toml` (100 kg, main 35 m^2 [D: 6.7 m/s at sea level, S: GQ 5000 / AP3456], splashdown
  recovery 8 m/s). The ship keeps its mass and CG when the seats leave (documented simplification, ~0.8 t of ~260 t).
- **Catch feasibility** (`gnc::catch_feasibility`, crewed ships only): a failed flip-group engine or propellant below
  `min_catch_propellant_kg` = 30 t [D: nominal gate 28 t caught, 27 t missed (hidden test "Ship catch: lowest
  catchable gate load") + 2 t]. Checked every tick of the entry up to the gate (the same tick before the gate
  hand-over); after the flip every abort (SCvx abort to the sea, upright descent, `abort_landing`) arms the seats,
  and so do a missed catch (phase Landed without pins on the arms) and every commanded engine starved or failed (final
  review I1: at 27 t the ship ran dry hovering at the arms at ~93 m and dropped onto them - 3 seats out, the fourth
  0.4 s stagger slot too late: "Schiff aufgeschlagen vor dem Ausstieg"). An aerocapture pass is not judged; the final
  entry after the capture is (final review I2).
- **Give-up in the entry** (`EntryGuidance::give_up`, `gnc::sea_give_up_target`): the target moves to the pad + sea
  direction x (shoreline + 2 km) [E], the gate height to 0, no gate hand-over, no flip: the entry / terminal steering fly
  the belly flop down to the impact. Unit case: 12 km / 120 m/s, 2 km west of the pad -> impact 2,500 m east (the
  target); given up at 800 m: impact 7 m beside the pad (reported as it falls).
- **Firing** (`FlightSession::run_crew_escape`, before the guidance each tick): inside the envelope (height above the
  flat site ground, EAS); an early give-up (above `min_firing_height_m` 1.5 km) waits until the ship is over water or
  at that height; a give-up at the gate or after the flip fires at once. Seats spawn at the ship's state + offset
  (velocity + omega x r), the rail thrust is an external wrench along the rail fixed at firing (pitch stabilisation and
  the few kg of seat propellant not modelled). `gnc::SeatSequencer` (mode latched at firing): mode 1 separates and
  opens the main at max(0.2 s, rocket burnout 0.55 s) - the rocket carries seat + occupant; modes 2/3 drogue, then
  separation + main (mode 3 waits for 15,000 ft). The separation swaps the seat body in place for a parachutist body
  (same index; the empty seat is not flown). Crew bodies get no `ParachuteSequencer` (`Flying::crew_member`).
- **Judging** (`sim::crew_escape.hpp`): load = |specific force| (incl. the rail thrust); hull clearance counted once
  the seat is 1 m out of the hull (riding the rails before); plume cone while engines burn; the tower box. Lost:
  sustained load, torn canopy, collision, touchdown > 8 m/s / impact, or the ship down before the seat fired
  ("außerhalb der Schleudersitz-Grenzen"). Caught with the crew aboard: rescued, no firing. A crewed ship that was given
  up returns `ReturnResult::Lost`.
- **Numbers:** entry engine failure 12 km, 2 km west -> fired at 61.7 s over water, 4 rescued 600-670 m east (water)
  at 6.7 m/s, peak 15.8 g, ship 2,500 m east in the sea at 96 m/s. Give-up at 1.2 km (20 t): fires at once, mode 1,
  crew on land ~340 m from the pad, ship on land 469 m from the tower. Catch abort in the flip: crew on land ~160 m
  from the pad (14-15 g), ship upright in the water 20 m beyond the shore. Crewed orbit return and lunar direct return
  with an entry engine failure: the ship is over the Gulf when it enters the envelope - seats fire at ~15 km in mode 3,
  4 rescued on water at 6.7 m/s, peak 16.4 g.
- **Missions:** `[mission] crew_escape = "<id>"` (`ship_return_starbase_crewed`, `ship_lunar_return_llo_crewed`);
  `start_lunar_return_mission` sets it; the orbit return (flown from the orbit) gets it from the game
  (`FlightSim.set_crew_escape(id)`) or the test. Any new placement (`clear_bodies`, `clear_for_new_body`) makes the
  flight uncrewed. Existing missions and campaigns stay uncrewed and unchanged.
- **Game:** `FlightSim.crew_escape_state()` (members with phase, mode, loads, clearance, position / attitude, touchdown,
  outcome; the ship's impact), `return_state(i)` of the ship (phase, guidance, abort, upright_descent, result),
  `set_crew_escape(id)`; each fired member is an ordinary body.
- **Known limits:** the load check is the magnitude of the specific force against 20 g for > 0.1 s [E], not per axis
  against sourced spinal / lateral limits, and there is no opening-shock reference case (final review I3, ruled: no
  per-axis source in hand); no Mach limit in the envelope (height + EAS only); no wind (M8), no zero-zero apex reference (no published ACES II number found), the seat's
  attitude is free (no STAPAC), no seat-seat collision check, crew landing on land beside the pad after a late give-up
  is reported, not avoided.
- **Tests:** `[crew_escape]` (`test_crew_escape_config.cpp`, `test_ejection_seat.cpp`, `test_catch_feasibility.cpp`,
  `test_entry_give_up.cpp`, `test_crew_escape_session.cpp`, `test_crew_escape_mission.cpp`; `[slow]`: the entry
  engine failure, the flip abort, the crewed orbit and lunar returns).

## Mission timeline

Plan: `docs/superpowers/plans/2026-10-22-external-view.md` (Tasks 1-4). Pure data in `sim::MissionTimeline`
(`mission_timeline.hpp`): recorded (past) and predicted events, each with body serial and the acting part's name.
- **T-0 rules:** the first recorded Liftoff; on a pad mission the scheduled liftoff; else the first recorded Ignition /
  Undocking, then the earliest predicted one. The mission clock is `now - T-0` (T- before, T+ after).
- **Recorded:** by `FlightSession` at exact moments (liftoff, stage / payload / trunk separation; the staging
  sequence's MECO at hot staging and the upper stage's ignition at its command - the observer skips an Ignition recorded
  within two samples -, hard capture, undocking at the physical separation - mechanism phase Separated, `hooks_open_s` 180 s after
  the command -, crew ejection) and by `MissionEventObserver` (0.1 s sampling while ticking, after each coast: engine
  starts / cutoffs, booster return phases, Max Q, entry interface, parachutes, landing outcomes; a catch - known once
  the body rests in the arms - is stamped with the pins' first touch on them, `CatchTowerSim::first_contact_epoch`:
  the catch moment as the webcast calls it, ruling 2026-10-22). A planned burn flown
  on RCS translation (no main engine fires: Crew Dragon, a small Starship burn on cold gas) records Ignition and Cutoff
  at the burn executor's start and end (both in one tick when it starts and ends in the same tick; a disengage of a
  lit burn records its Cutoff), named by the upper stage (its thrusters); RCS attitude control and proximity
  translation are no events.
- **Predicted** (`predicted_mission_events`, `predicted_events.cpp`; set at the end of every `advance_to`): the
  scheduled liftoff (while the ascent autopilot is armed in PreLaunch), a lunar ascent's liftoff, the docked pair's separation (`FlightSession::undock_separation_epoch`:
  the ISS mission's undocking epoch + hooks opening, or the hooks' end while they open - the predicted dot and the
  recorded event agree within a tick), the pending burn's ignition (from the executor's time to ignition: a finite burn
  starts before its impulsive node), the deorbit burn and entry interface of a ship / capsule return (the entry
  interface also while the burn is lit), the not yet
  commanded burns of the lunar outbound, descent (incl. PDI) and return plans (with the braking burn and the entry
  interface) and of the rendezvous plan, and the campaign supervisor's next launch (Launch). A plan burn within
  `same_burn_s` 600 s [E] of the pending or a commanded burn is that burn (re-targeted at the command) and not shown
  twice. Re-plans move the predicted events (test: the deorbit re-plan 3 h before the burn).
- **Onboard predictions** (Task 4b, user 2026-10-22: every event the onboard computer has determined, as soon as it
  has; only from what the guidance computes or its own criterion run forward - no nominal profiles), with the kind and
  body serial the event will be recorded with; an event a body has once (MECO, separation, SECO, Boostback, landing
  burn, parachutes, catch / landing / splashdown) is no longer predicted once recorded for that body:
  - MECO and stage separation (AutopilotState::Booster, from the end of the open loop's pitch kick - kick start +
    `kick_duration_s`; final review: the cutoff threshold is the return's live "MECO now" plan held constant over the
    forecast, and from the near-vertical state of the rise and the kick that plan swings - forecasts up to 96 s late in
    the first ~16 s -, so MECO, separation and the upper ignition are withheld until the kick has ended; the timeline
    test asserts none of them is shown before): the booster's propellant above the cutoff
    propellant the autopilot set on its last tick (`booster_cutoff_propellant_kg`: the return's need or the reserve) at
    the stage's actual flow now scaled with the throttle the open loop will command - since ascent Task 2 the
    operational cap and the max-Q limiter run forward on a point mass (`booster_burn_time_s` in `predicted_events.cpp`:
    the open loop's vertical rise, kick and gravity turn, the vehicle's drag table, US76, central gravity, 0.5 s steps
    [E]; free launches: within 0.02 s (Starship) / 0.32 s (Falcon 9) of the flown MECO from the kick's end on,
    final review; the first measurement was 0.6 / 2.0 s over the whole booster phase);
    the separation and the upper stage's ignition follow the staging sequence's timing (`separation_after_meco_s`,
    `upper_ignition_after_meco_s`; in the state Staging from the sequence's own epochs). Measured (ascent Task 2):
    starship_leo +5.0 s at T+30 s, at most 7.2 s within 120 s, 2.2 s within 10 s; Falcon 9 RTLS +7.9 s at T+30 s
    (with the schedule: +7.0 / +7.4 s at T+30 s). Final review (planned kick, shown from the kick's end):
    starship_leo +5.59 s at T+30 s, at most 6.58 s over every shown sample, 2.21 s within 10 s.
    The prediction is late by the return's cutoff propellant, which grows with the downrange speed (the value of the
    moment is used). At the flow now alone it was +0.9 s at T+30 s by cancellation but 12 s (Starship) / 37 s
    (Falcon 9) off in the bucket. Not on the pad: the return's cutoff propellant is only planned in flight.
  - SECO: `AscentGuidance::cutoff_epoch` (closed-loop update + PEG time to go) from the upper stage's first
    closed-loop tick: -1.9 s then, within 0.3 s over the last 120 s.
  - Booster return (`BoosterReturnGuidance::forecast`): the boostback's cutoff (Cutoff: velocity to be gained less the
    tail-off, rocket equation at full thrust; -1.6 s at its start, 0.2 s within 10 s); the landing burn (LandingBurn,
    from the apex: the ballistic impact prediction less the fall, at the sink speed now, over the height the
    landing-burn prediction takes - within 5.4 s, 1.4 s in the last 30 s; the stop height run forward linearly was
    26 s early mid-fall; with an entry burn (Falcon 9) only after it: the burn prediction does not know it and was
    130 s early before it); the touchdown (Caught / Landed: that ignition plus the burn prediction's duration, then the
    landing burn's time to go - the SCvx plan's end; Super Heavy: -16 s at the apex, 4.3 s within 30 s, 0.9 s within
    10 s of the pins' first touch).
  - Ship / capsule entry (`EntryGuidance::gate_epoch`: the corrector's or the terminal steering's predicted time to
    the gate): the ship's landing burn (flip at the gate; ship_return_starbase: +28 s 26 min ahead, 0.3 s within 2 min)
    or the capsule's drogues (Parachutes; Crew Dragon: within 3.5 s, 1.8 s at the end). Then
    `ShipLandingGuidance::catch_epoch` (the landing burn plan's end; -2.9 s 18 s ahead, -1.3 s at the end).
  - The entry interface of a capsule return whose trunk is still on carries the capsule's serial: the trunk keeps the
    stack's at the jettison, the capsule becomes the next body (`FlightSession::next_body_serial`).
  - Not predicted (no guidance holds a time): the boostback ignition (the flip ends on attitude), Falcon 9's entry
    burn, the splashdown (parachute descent), the docking (the final approach has holds) and a propellant transfer's
    end. Max Q is never predicted.
- **Transfer telemetry** (Task 4c): `FlightBody` telemetry carries `speed_inertial_mps` (|v| about the reference body,
  not the Earth's centre: the ruling is that the lunar orbit's speed is shown about the Moon), `distance_earth_m` /
  `distance_moon_m` (from the centres) and `speed_earth_mps` (|v_gcrf|) / `speed_moon_mps` (|v_gcrf - v_moon|). They are
  in the telemetry CSV and the legacy HUD and drive the webcast's transfer situation (`gauge_rules.gd`: altitude
  > 5000 km and apoapsis > 100000 km [E], with hysteresis `transfer_exit_factor` 0.8; all gauge lists, thresholds and looks live in `config/ui/webcast_gauges.toml`, read by `GaugeConfig`).
- **Tests:** `[timeline]` (`test_mission_timeline.cpp`): T-0 rules; the recorded ascent, booster return, flip and
  catch, cold staging, lunar-return loft, undocking; scheduled liftoff with pad hold (one 4000-tick call coasts 5.96 h,
  altitude within 0.1 m of the pad); waiting ISS launch; countdown of a waiting ascent; predicted deorbit events before
  and after the re-plan; the Dragon's RCS burn (Ignition / Cutoff, predicted ignition within 1 s); predicted vs recorded
  undocking; the onboard predictions of MECO / separation / SECO (starship_leo), MECO (Falcon 9), the booster's
  boostback cutoff, landing burn and catch, the ship's landing burn and catch (tolerances from the measurements above),
  the capsule's entry interface serial through the trunk jettison.
  Hidden probe `"Return prediction probe"` (STARSIM_VERBOSE=1, STARSIM_PROBE_CAPSULE for the Crew Dragon): a ship /
  capsule return from orbit with the predictions' errors; STARSIM_DEBUG_PREDICTIONS prints every sample.

## Ascent trajectory and staging

Plan: `docs/superpowers/plans/2026-10-22-ascent-trajectory-staging.md`, spec
`docs/superpowers/specs/2026-10-22-ascent-trajectory-staging-design.md`.

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
  `flip_duration_s` (default 10 s [E]; Falcon 9: 50 s [D]) plus the sequence's MECO -> separation interval (hot: the
  run-on engines burn at minimum throttle through it, as the flip engines do; starship_leo then reaches landing
  ignition with 52.2 t for 51.0 t planned - 48.6 t without the interval).
- **Open discrepancy - Falcon 9 flip time:** on its nitrogen thrusters the stage needs 50.8 s from separation to the
  boostback (96 of 196 kg N2); CRS-35 lit its boostback ~14 s after the separation (speed table). The flip is
  torque-limited: the flip runs on `return_controller_` (no rate limit; the 2 deg/s limit is the coast controller's,
  the 5 deg/s `max_pitch_yaw_rate_radps` is the pilot's rate command), 8 x 450 N [E] at the interstage give 25.8 kNm
  on a pitch inertia of 1.0e7 kg m2 = 2.6e-3 rad/s2 - bang-bang over ~165 deg ~65 s, the boostback lights at 15 deg
  pointing error. A 14 s flip needs ~15x the torque, and N2 beyond the 200 kg [E] supply at Isp 70 s. No source for
  the real thrusters' thrust was found (SpaceX / Wikipedia / spec sheets name only "nitrogen cold gas thrusters"), so
  the [E] sizing stays and the measured 50 s is the planning value; boostback_loss_fraction 0.19 follows from it.
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
- Falcon 9 MECO prediction at T+30 s: +7.35 -> +8.44 s (bound 8 -> 9 s): the cutoff propellant includes the 54.5 s
  MECO -> boostback and the loss 0.19.
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
  Follow-up: cross-flow normal force in `descent_model`.
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
    derivation), deck along the ground track there; its sea at -35 m [E] (geoid). The kick is searched with the ship at
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
  [E], sea -35 m [E]; landing_zone = ccsfs_lz40 on falcon9_rtls and falcon9_crew_dragon (now SLC-40's own landing zone).
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
  8,227 kN for nine] - 2 %, inside the G dial's resolution, kept.
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
    T+515 +- 3 s display). LZ-40 chosen (CRS-35 landed there).
- **No re-baselines:** no physics changed (the time base is the comparison's, the propellant load a mission input);
  the reference missions (starship_leo, the Falcon 9 missions, ISS, refuelling) fly as before, their numbers stand.
- **Removed:** the old telemetry tests (starship_leo with the recorded 500 t MECO and the planned kick; CRS-35 on a
  fixed 12 deg kick, T+60 / 120 s within 15 % only) and the kick sweeps "[.tuning]" (kicks are planned).
