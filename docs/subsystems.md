# Subsystems, decisions and validation record

Detailed notes moved out of `CLAUDE.md` (2026-10-15) to keep the per-session context small. Each entry records
a subsystem's design, the decisions behind it (with dates and who decided), its validation numbers and its
diagnostics. Keep it up to date when a subsystem changes; `CLAUDE.md` keeps only the rules and a short index.

## Vehicle data, ascent and booster recovery

- Only the current configuration (Block 3 / V3, Raptor 3) is modelled (decision 2026-10-13); its data come from
  the flights that flew it (12-14, `tests/validation/telemetry/flights.toml`). Earlier blocks were removed.
- Starship throttle schedules are not public; they are a guidance choice, not vehicle data. The ascent mission
  `config/missions/starship_leo.toml` (`sim::start_ascent_mission`, `FlightSim.start_mission`) carries the booster
  schedule [D] of flights 13/14 (gauge shape, level fitted to the recorded speed: ~0.85, max-Q bucket ~0.76, ~0.83, ~0.76
  towards MECO) and kick 14 deg: test "Starship ascent follows the webcast telemetry of flights 13 and 14" (flown with the
  recorded MECO of ~500 t: speed within 3-6 %, altitude within ~1 km at T+60-120 s, MECO T+135 s / 48 km / 5840 km/h vs
  ~T+140 s / ~53 km / ~5950 km/h). The game's key P and the older ascent tests still fly full thrust with the default
  pitch program. Staging mass vs the gauge (decision 2026-10-14: physics first, the gauge after staging is not
  followed): on this ascent our return plans MECO with ~310 t (301 t at the boostback start, ~2.2 km/s on 13 engines),
  flight 14's gauge shows 11.8 % (431 t) there. The gauge is not consistent with the engines after staging: it falls
  ~15 t/s on 13 engines (full mass flow ~10.5 t/s) and 3.5 -> 1.4 % (77 t) in the last ~12 s on 3 engines (~30 t);
  Falcon 9's falls 21 t in a coast without a burn. The timeline matches (test "Block 3 plans MECO ..."): boostback
  T+152-176 s (flight 14 ~152-187 s), apogee 91.8 km at T+236 s (98.9 km, T+240 s), landing-burn ignition T+390 s
  (~T+398 s) at 287 m/s / 2.2 km (flights 13/14: ~355 m/s at ~2.9 km; subsonic drag is not measured). The earlier
  "~207 t, boostback ~1.5 km/s" came from the default full-thrust ascent (MECO at ~97 km climbing at 1.7 km/s).
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
  Diagnostics: `STARSIM_DEBUG_SCVX=1`, `STARSIM_DEBUG_SCVX_SOLVER=1`, `STARSIM_DEBUG_CATCH=1`.
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
  effort 0.1 of the gimbal authority [E] (more limit-cycles after the flip), attitude controller with
  `Gains::actuator_delay_s` 0.75 s (gimbal slew in the braking; no overshoot). Flaps folded in the burn.
  `FlightSession::place_landing_ship` (scenarios); the entry hands over at the gate. Test from the gate: caught 0.01 m,
  0.94 m/s sink, 5.7 t left; campaign `[ship][catch][validation]` (+-10 m, +-10 m/s, propellant 35 t + 0-3.5 t
  main-tank residue, +-5 deg alpha / bank, belly engine failed in the flip): 20 of 21 caught (<= 0.02 m), the failure
  aborts to the sea.
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
  5.5 m/s ("verfehlt"), with full headers it is caught normally.
  Diagnostics: `STARSIM_DEBUG_FLIP=1`, `STARSIM_DEBUG_BURN=1`, `STARSIM_DEBUG_ENTRY=1` with the SCvx / catch flags;
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
  zero-lift fall (grid-fin lift). Ascent: `PitchProgram::booster_throttle` (first-stage throttle over mission time,
  empty = full) carries the Falcon 9 schedule [D] from the CRS-35 G-force dial against the simulated stack: full thrust,
  a ~72 % max-Q bucket T+50-61 s, then ~95 % (`STARSIM_DEBUG_ASCENT_G=1` prints the simulated G); pitch kick 12 deg [E];
  CRS-35 T+60 / 120 s 1219 / 4807 km/h (recorded 1188 / 4672: the real stack starts ~2.5 s slower), MECO T+137 s /
  56.4 km / 6372 km/h vs recorded 137 / 56.6 / 6441. The upper stage's plane steering nulls out-of-plane speed and offset together
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
  orientation (5th constructor argument, `spice::SpiceOrientation::moon_mean_earth()`); without it (bindings until l6)
  everything is relative to the Earth as before. Ground at the Moon: the sphere (LOLA tile: l2). Validation
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
  `complete_pilot_correction()`, the next MCC re-plans from the real state; display with l6 / M7. Missions
  `hls_lunar_llo` / `hls_lunar_nrho` (`kind = "lunar_landing"`, outbound only until l4; the game's FlightSim has no
  Moon orientation yet - l6). Results: LLO 99.97-100.00 km after one trim; NRHO 0.58 km worst perilune offset over 3
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
  status at burn end shows osculating elements (95 x 100 km for a 100 km LLO; l6 telemetry), the sub-orbital cutoff
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
  the new plan is cheaper; `lunar_return_replans()`; game key and display in l6. Botched TEI (velocity error = fraction of
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
