# Ship entry, flip, catch and return from orbit

History and the reasons behind superseded decisions: `docs/history/guidance-missions.md`.
Index of the guidance / missions topics: `docs/subsystems/guidance-missions.md`.

## Model
Spec `docs/superpowers/archive/specs/2026-10-13-ship-return-design.md`.
- Entry (b1), belly first. Hull = `SlenderBodyAero` with `aero/ship_crossflow.csv` [D, empirical from the flights] and `hull_cp_table` `aero/ship_hull_cp.csv` [E, drawn mesh; faded in over Mach 1-4, formula below]; nose 13.4 m as drawn. Four flaps (`aero/flap`,
  `[[stage.flap]]` [E] from the drawn outlines, Task 3 of the vehicle physics plan: aft 2 x 36.48 m^2, stations 2-14, span
  3.69, azimuth 0 / 180; forward 2 x 12.374 m^2, hinge 42.282-47.75 leaning 19.03 deg towards the axis from r 4.621, span
  3.154, azimuth 20 / 160; application point = the outline's centroid in the hinge frame (`centroid_u_m`, `centroid_v_m`);
  Newtonian flat plates, `aero/flap.csv`, times the plate's share out of the hull's shadow, `aero::flap_exposed_fraction`:
  0 with the centroid downstream and within the hull's width at the section the flow line crosses, growing over half a span
  beyond it, so no step and both crossflow signs load a flap out at the side; 0 deg folded leeward, 90 deg broadside; 20 deg/s;
  the deflection is limited to 0-90 deg). Rest pose d = 90 (user 2026-11-01, `initial_deflection_deg`): on the pad, in ascent, coast and orbit no law
  commands the flaps, `Vehicle::update_flaps` holds them; at d = 90 the plate is edge-on to axial flow. Entry, flip and
  landing laws start from there. `AttitudeController::Gains::flaps`: flaps cancel the modelled aero
  moment (hull + flaps), steer by a Newton step on their linearised moments, the rest goes to RCS; flap allocation is a
  box-constrained active set (free flaps re-solved, box = stops and the 0.25 s [E] flap-rate reach, all-flap
  moment-neutral centring, a secant only for a flap at a stop; the lower stop is `flap_lowest_effective_rad`: above
  `flap_lowest_usable_deflection_rad` (edge-on, shadow), going down from the flap's best moment slope in this air, the first crossing of a quarter of it [E], 5 deg grid, interpolated; with the secant all along the ramp out of the shadow).
  Beyond flaps + RCS (no gimbals; planner plan Task 4 fix round 1, controller ruling): flaps first, then the RCS yaw,
  then the cut - never a yaw demand traded into roll. When the flap step leaves more yaw than the RCS can give and spills
  roll / pitch beyond it, the flaps are asked for the largest share of the yaw (bisection, 16 halvings) that leaves no
  more roll / pitch to the RCS than with their yaw unchanged (+1 % [E] of the RCS capability; when even that does not
  hold - roll / pitch beyond the reach themselves - the flaps keep the whole demand: cutting their yaw there flew the
  crew-escape give-up entry onto land at 250 m/s); the rest of the yaw goes to the RCS, beyond it is clipped and
  reported as `ControlOutput::unmet_torque_nm` (anti-windup: the hull-moment integral does not run on a cut pitch). Mach
  ~0.6, alpha 80.5, 212 t, 1e6 N m of yaw: settled RCS 0.010 / 0.0004 / 1.0 (roll / pitch / yaw), 6.5e5 cut, the flaps'
  true yaw 2.15e5 + RCS 1.2e5 (before: RCS roll 1.5, roll unmet; 3e5 is now delivered, RCS yaw 0.6). Below 200 Pa [E] the flaps rest in the coming entry's
  pitch trim (`EntryAeroTable::trim_flap_rad`). `gnc/entry_attitude` builds the setpoint from angle of attack and bank
  (belly at `[stage.aero] belly_azimuth_deg`). Nominal alpha over Mach `aero/ship_entry_alpha.csv` [D] (55 deg
  hypersonic, 50 near Mach 8, 60-80 towards the belly flop). Ship dry 177 t, dry CG 22.03 m [D] (planner plan Task 3b, bottom-up budget;
  `vehicle-data.md`); with full headers (35 t) 212 t at 24.99 m from the base. Entry control: 0.5 rad/s, bank changes <= 1 deg/s. `FlightSession::update_hull_moment_scale`
  estimates k = true / modelled hull pitch moment (Mach > 10, q > 200 Pa, 0.3 deg deadband, 2 % apply deadband; a
  scale, not a torque integral, user 2026-10-17).
- Corridor (`[stage.heat_shield]`, judged while the ship flies alone): Sutton-Graves stagnation heat rate
  (`aero/entry_heating`, nose radius 6 m [E]), heat load, aerodynamic load (thrust not counted); beyond a limit the
  ship breaks up ("Hitzeschild versagt" / "Strukturüberlast", outcome "zerbrochen"), warning from 80 %; skip-out
  ("Abprall": below 80 km, then above 100 km). Heating integrated in `step()` only. Entry mass in tests: dry + full
  header tanks (`landing_propellant_kg`, 35 t [E]) + main-tank residue; `config::fill_tanks` fills headers first; a
  returning ship never carries less than 35 t. Campaigns: entry 35 t + 0-28 t residue, catch gate 35 t + 0-3.5 t.
- Propellant settling (`vehicle/liquid_settling`, flag `FlightEnvironment::propellant_settling`, on): oxidizer and fuel
  lie below a surface perpendicular to the apparent gravity, lagged by tau = sqrt(2 D / a) in [0.5, 60] s and held in
  coasts (< 0.01 m/s^2); per tank a tilted-cylinder table. At entry attitude the residue lies forward on the belly (64
  t: CG +0.95 m forward). Engine, grid fin and flap moments use the axial CG [E] unless
  `FlightEnvironment::actuator_moments_about_cg_point` (off; turning it on moves the Falcon 9 / Crew Dragon ascent
  apoapsis 210 -> 223 km and needs its own validation). Main-fed burns check the main tanks only
  (`Vehicle::main_propellant_kg`). The liquid does not turn with a round tank: moments act in roll about
  `MassProperties::roll_offset_m` (hull parts only), in pitch / yaw about the full CG. Tanks have flat ends; the RCS
  allocator uses the full CG point. The ship alone carries its own axial drag table [E] (`ascent_drag_table` = `aero/ship_ascent.csv`, the stack's generic trend until 2026-11-01).
- The shared near-target miss metric in `entry_predictor` changed with the capsule's corrector (`crew-dragon.md`); ship and lunar numbers compared unchanged.
- Entry guidance (b2, `gnc/entry_guidance`, `entry_predictor`, `entry_aero_table`): 3-DOF point-mass predictor
  (Earth-fixed, J2, US76, trimmed C_D A / C_L A over Mach and alpha, online drag / lift scale from the accelerometer
  `FlightBody::aero_acceleration_body_mps2()`, not learnt in an attitude transient over 2 deg in the terminal phase)
  every 2 s; bank magnitude for zero range error (Illinois); heat-rate, load and dynamic-pressure peaks over 90 % of a
  limit cut the bank back. Reversals timed by the predictor; a reversal due within the next cycle is flown only if
  reversing now ends closer across than holding. Parameters from the config (planner plan Task 4,
  `gnc::entry_guidance_parameters(vehicle, stage, EntryKind)`, `core/src/gnc/entry_guidance_parameters.cpp`; every key of
  `[stage.entry]` required for a belly-first stage, German refusal at load; the tanker inherits it): K q limit 25 kPa
  (lunar return 32 kPa), terminal Mach 5 / latest 3, terminal min alpha 50 deg and alpha authority 20 deg [D identified
  on the b1 terminal-phase campaign 2026-10-13, 12 dispersed entries, 12/12 at the gate; the flown floor is the table's
  lowest trimmed alpha], bank rate 3 deg/s below Mach 5 and attitude slew 3 deg/s
  (the flaps' authority needs the air: not derived). D at the entry mass (dry + full headers, 212 t, I 5.46e7 kg m^2)
  from the RCS's acceleration about the weaker transverse axis (yaw 1.2e5 N m: a = 2.2e-3 rad/s^2): the hypersonic bank
  rate sqrt(2 a 5 deg) = 1.12 deg/s (5 deg: the reversal's own settled band, `entry_settled_bank_rad`; RCS alone, a lower
  bound), capped at 1 deg/s [E] (the cap binds); the terminal nose turn's angular acceleration
  a = 0.126 deg/s^2, capped at 0.5 [E] (the RCS binds). Linear between Mach 5 and 10 as before. From
  Mach 5 (latest Mach 3, once a reversal has settled) the terminal phase ("Endanflug") steers the lift in the velocity
  frame relative to the alpha schedule (Levenberg-Marquardt on the predicted gate miss); the gate ("Tor") is 800 m [E]
  above the pad (flight 13 relit at ~0.95 km [D]). `FlightSession::place_entering_ship`, `entry_status`. Tests use
  `predictor_j2 = false`. Predictor additions for lunar return: radiative heating, skips carried, exit orbit, truth
  air-density factor (`FlightSession::set_air_density_factor`), great-circle range unwrapped by the swept angle, a
  target behind the start's track counted as a turn less its distance ahead.
- Entry aero table trim (planner plan Task 4): the table's flaps sit where the allocation settles - of all pitch trims of
  the two pairs the one nearest mid-range, each pair's offset in the ratio of its pitch slope (Newton from the
  antisymmetric trim, which still bounds the lowest trimmed alpha). Against the flown trim at 212 t, Mach 2.6-0.47: drag
  area -0.03 to -0.24 % (the 5 deg alpha grid), was -0.43 to -0.68 % with the antisymmetric trim (forward pair 14-33 deg
  where the flown flaps sit at 41-44). Cost: the Newton adds ~20 us per table point, 8 ms per build (396 points; 11.5 ->
  19.6 ms); the lowest-alpha walk (240 points) uses the antisymmetric trim alone.
- Terminal steering floor (Task 6b): the most-lift alpha is never below `EntryAeroTable::lowest_trimmed_alpha_rad` (flying mass and CG; 1 deg search), in `EntryPredictorModel::steering_most_lift_alpha`, so guidance and predictor count the same lift. Steering differences stay on their side of `schedule_fraction` (`steering_difference`). After the basis froze, with a nose direction, the attitude slews on the `bank_step_smooth` ramp (`terminal_nose_slew_acceleration_radps2`, the RCS-derived 0.126 deg/s^2 of the 212 t ship; at the K 0.5 deg/s^2 the turn asked ~4.8e5 N m of yaw against ~1.2e5 RCS plus ~1-2e5 of flap reach and departed). `upright_descent`'s braking feedforward scales with sink / profile sink, so a ship below the profile sinks instead of climbing.
- Flip and catch (b3, `gnc/ship_landing`): at the gate the three sea-level Raptors relight at 80 % [E], aft flaps
  fold, forward flaps go broadside until the tilt is < 30 deg; flip ~4 s and ~8 t, pushes the ship ~100 m along its
  nose. Once upright (< 15 deg, < 0.2 rad/s) SCvx plans fly 3 -> 2 engines: the belly engine shuts down, the leeward
  pair lands (layout [D] from flight 13 splashdown video). Parameters from the config (Task 3, 2026-11-03,
  `gnc::ship_landing_parameters(vehicle, stage)`, `core/src/gnc/ship_landing_parameters.cpp`; a "tower_catch" stage with
  an entry_alpha_table must have every key, German refusal at load): K `[stage.recovery]` flip_engine_groups,
  landing_engine_phases, landing_phase_max_tilt_deg (20/20/15), flip_throttle 0.8, flip_tolerance_deg 15,
  flip_settled_rate_rad_s 0.2, flaps_until_deg 30, descent_propellant_margin 1.3, landing_lateral_effort 0.03;
  `[stage.tanks] unusable_propellant_kg` header 1 t = the plans' hard floor (main 1 t for the tanker reserve). D (at the
  landing masses: dry + full headers 212 t / dry + 1 t 178 t; I 5.46e7 kg m^2, lever 24.98 m): tilt-rate limits
  min(gimbal slew, sqrt(alpha_min tilt)) = 0.301 / 0.246 / 0.213 rad/s; 3 -> 2 sink window min = (3 at minimum -
  weight) x shutdown = 1.25 m/s (three Raptors at 30 % lift the light ship), max = sqrt(2 (2 at 95 % / m - g) h) set at
  the gate from the CG height above the catch (144 m/s from 700 m; never binding - the window has effectively no maximum; the webcast 20-40 m/s is no longer
  used). The first plan starts from two guesses and keeps the better (`gnc::SwitchGuess`, every vehicle since planner Task 5; `gnc::better_descent_plan`: converged, then feasible - clear of the keep-out boxes, final-set tilt within `max_planned_tilt_excess_rad` - then the heavier end): the window's middle, and the switch from the state (as late as
  the pair can still finish in its minimum phase times, ~13 m/s, but not below what three engines cannot remove within
  the height; the window's middle is no operating point). The first plan starts from the flip engines' running thrust
  (3 x 0.8, changing at the throttle rate from the plan's start; `ScvxLandingGuidance::request`, as every re-plan; a set
  still starting up - the booster - keeps its start-up model): without it the 212 t ship's first plan asked 8.7 m/s^2
  at once, the engines gave ~19 while coming down, ~9 m/s of sink went unplanned and the catch came ~3 s later than
  planned. Catch approach time constant (`DescentLimits::
  approach_time_constant_s`) [D] = the pair's throttle range / throttle rate, (1 - 0.3) / 0.5 = 1.4 s
  (`gnc::ship_approach_time_constant_s` = `gnc::throttle_range_time_s`; the boosters derive theirs the same way). The upright descent (last resort) brakes on the
  three engines towards the landing pair's own sink profile sqrt(v_f^2 + 2 f (a_pair - g) h) until the pair at its planned
  maximum (the plans' 5 % throttle margin below full - the scvx K value `throttle_margin` reused, not derived) can stop the sink within the height (the same with f = 1, live
  mass; the margin closes in on the profile), then the pair takes over (sticky) and
  lands (`upright_descent` with a profile acceleration).
  Thrust
  of an asymmetric set goes through the CG along an axis tilted against the hull (~1.5 deg for the pair): plan states
  carry the thrust-axis attitude (`gnc::thrust_axis_body`). The plans' propellant floor counts only what the selected
  feed can burn (`gnc::feed_propellant_kg`: from the flip the header tanks, limited by the scarcer of LOX and CH4 at
  O/F 3.6; the main-tank residue never reaches the engines) minus the 1 t unusable header propellant; the upright abort's propellant check
  reads the same (Task 1c, 2026-11-03; before, the whole load counted and plans could fly the headers dry). Ship LQR effort 0.03 of the gimbal authority [D seed 18], attitude
  controller `Gains::actuator_delay_s` 0.75 s; flaps folded in the burn. Lateral phase tilt limit 20 deg [E].
  `FlightSession::place_landing_ship`; the entry hands over at the gate.
- Gate lead: `gnc::ship_entry_target` puts the entry target the derived lead (flip travel + tilt-in + brake) back
  from the catch axis. Static (the deorbit planners' aim: `ship_deorbit_plan`, `replan_ship_deorbit`, the lunar return's gate): 195 m
  from the analytic `gnc::ship_flip_push` (no air) - the flip set relights (thrust and authority ramp over the Raptor's
  3 s startup) and turns 90 deg rest-to-rest in 4.51 s, pushing 87 m and 37.8 m/s; the three at 95 % tilt in to 20 deg
  at the phase's rate limit in 1.69 s (60 m) and brake the remaining 29.9 m/s in 49 m; v0 at the gate adds
  v0 (t_flip + t_tilt-in) and brakes from v0 + 29.9 m/s (`FlipModel::initial_speed_mps`). This model overstates the
  push: at ~100 m/s sink the hull's normal force turns with the hull and pushes back (Mach 2.6 flight: thrust alone
  +40 m/s, flown +25 m/s). In flight (fix round 3) `EntryGuidance::set_terminal_aim` re-aims with `gnc::ShipGateAim`:
  the flip flown forward (`gnc::predict_ship_flip`: a copy of the ship at the predicted gate state - the terminal
  prediction's gate velocity at the hand-over height, level belly flop with the live flow side, `ship_gate_state` -
  flies `ShipLandingGuidance`'s flip with the session's ship-landing controller and `FlightBody::step` at the session
  tick, nominal aero / thrust / air, to the first plan's start `ship_flip_ready`), then tilt-in and brake
  (`ship_brake_after_flip`) from the flown end speed (`ship_gate_lead_m(p, ShipFlipPrediction, v0, mass)`). Refreshed
  every 5 s of the terminal phase [E computing budget, ~50 ms per flip] and once more when the terminal prediction puts
  the gate less than 5 s away (N3, Task 11b; `TerminalGate::time_to_gate_s`), unless the latest refresh already was;
  between refreshes the gate speed enters to first order ((v0 - v0_flip) t_flip and the brake). The analytic model is
  used only while no flown flip has settled - logged once per cause (`[gate-aim] analytic flip model ...`: "did not
  settle" / "no flip flown forward yet"), counted in `ShipGateAim::analytic_aims()` (N4). Checks: seed-18 gate from the exact state identical to the
  session's flip (4.93 s, 130.7 m, +34.6 m/s), from the re-aim's built state 4.80 s, 119.6 m, +32.2 m/s (-3/-8/-7 %);
  Mach 2.6 flight with the N3 refresh (Task 11b): the last refresh predicted the flip end 3.48 s vs flown 3.53 s, gate
  797 m, caught 0.18 m / 0.95 m/s sink with 10.3 t left (before N3 the aim's last refresh, 803 m before the gate at
  along 5.76 / sink 96.8 m/s: 3.83 s vs flown 3.93 s; from the exact gate state 3.92 s, 72.1 m / +25.8 m/s vs ~71.5 m /
  +25.4 m/s; lead ~164 m at v0 5.8 m/s, the analytic model: 252 m). Not in the lead: the plans' non-maximal braking;
  the first plan takes effect at its snapshot (in real-time play at the hand-over after the flip-thrust hold, planned
  from the state the hold leads to - see "Planner work off the sim thread"), so no latency term. The lead lies on the line tower ->
  pad with `EntryTarget::nose_direction_itrf` to the tower; only that approach is robust (the arms stand across the
  line). The terminal phase turns the nose once the fall is steep (basis frozen, -70 deg): steering into the nose
  sector at fraction 0.2 (alpha ~86 deg), the turn ending inside its nearest edge by the lag at the turn's end [D]
  (`gnc::nose_turn_end_steering`, at most at the nose direction): the commanded attitude starts the turn at rest on its
  slew ramp a up to the slew rate s (1.5 w), so the lift flown trails the turn at w by w T - a T^2 / 2 at its end
  T = turn / w (`gnc::nose_turn_lag_rad`, at most w^2 / (2 a)), and the miss that lag leaves wants about as much
  correction beyond the turn's end - on the edge it pointed out of the sector (phase 90 pinned there, 152 m across);
  with the largest lag (35.7 deg, on the nose direction) the 118 deg turn of the phase-90 pass at -291 km left the
  correction outside the far edge (105 m). Ship (w 3 deg/s, a 0.126 deg/s^2, s 4.5 deg/s): ~24 deg for that turn,
  the full 30 deg (nose direction) for turns of ~43-100 deg, less for shorter or longer ones; phases 0 / 90 / 180 / 270 at the gate 35 / 61 / 7 / 90 m,
  caught (round 3 passes). The lift direction turns about the air velocity at 3 deg/s [E],
  then Levenberg-Marquardt in sector coordinates (fraction 0.02-0.5; the sector narrows from +-30 deg 4 km above the
  gate to +-10 deg at it [E]). The predictor flies the same turn. The commanded attitude slews at 1.5x the turn rate
  and its rate is fed forward (`EntryCommand::rate_gcrf_radps`); entry controller actuator delay 0.5 s [E].
- Catch from lunar gates: catch plans keep a soft half-plane 18 m off the tower while the hull bottom is below the
  tower top; the gate hands over higher with the sink, only below 40 km (`gnc::gate_trigger_height_m`): 800 m up to
  S = 95 m/s [E], then a parabola, from J = 100^2 / S = 105.263 m/s the stopping-distance law gate x (sink / 100)^2 [E];
  no step in height or slope, S is the only value of the parabola (Task 6h).
- Structure keep-out (Task 6g, `gnc/descent_keep_out`, fidelity flag `SnapshotSettings::structure_keep_out`, on for the ship only): plans keep the hull's real extent (the capsule the tower physics collides: axis from tail to nose, radius 4.5 m, the plan's tilt) off boxes taken from `TowerGeometry`. Catch plans: both arms at standby. Abort plans: both arms open (the tower commands them open at an abort, 40 deg within 3.3 s; the strike at the 400 m abort was at opening 0.698 rad, not standby) and the tower box; the CG-only tower disc and mount rows stay. One soft row per box and interval, at the tightest of 4 instants between the nodes (the last phase's nodes lie up to ~2.4 s, ~70 m of height, apart), with normal, lever and axis frozen from the reference; rows only within 20 m [E] beyond the margin; margin 1.0 m [E] (tracking 0.2 m + thrust axis 1.5 deg off the hull over 24 m). `DescentPlan::keep_out_overlap_m` measures the settled plan. A catch plan that overlaps counts as beyond its limits (the existing refusal: first plan -> abort; 2 such re-plans -> abort). An abort plan that overlaps gives the water up, then is refused and the upright descent takes over. The mount has no hull row and the upright descent, the flip and the arms' swing are not covered.
- Reach at the abort (`gnc::estimate_sea_reach`, once, from the live state): lower bound of the flight to the water and
  the stop on it (bang-bang at the tilt limit); beyond the usable propellant / `sea_reach_margin` 1.5 [D] the abort plans
  no longer have to end in the water. The ship keeps flying those plans (not the upright law at once): right after the
  flip it carries ~31 m/s towards the tower, which 15 deg of tilt near hover cannot stop in 150 m.
- Upright abort (`gnc::upright_descent`, deterministic: tilt <= 15 deg, away from the tower inside 60 m, sink profile
  sqrt(2^2 + 2 * 0.5 * (a_max - g) * h) with deceleration fed forward, never below the engines' minimum thrust): takes
  over an abort with no converged abort plan 2 s after the abort, an abort leaning beyond 35 deg, an abort whose usable
  propellant falls below 1.3 x a vertical stop's need (`upright_stop_propellant_kg`), and a flip whose first three
  plans reach neither the catch nor the sea. `sim::ReturnResult` (caught / upright abort: contact tilt < 20 deg and
  sink <= 5 m/s [E] / lost), `FlightSession::ship_return_result`. A pilot abort during the flip (nominal gate): upright abort on land ~105 m from the pad, 4.2 t left, the water not kept. Abort scan and its limits: `docs/known-issues.md`.
- Ship return from orbit (b4, `config/missions/ship_return_starbase.toml`, `config::load_ship_return_mission`):
  `FlightSession::plan_ship_return` (game key R) plans the deorbit burn (`gnc/deorbit_planner`: coast samples over
  48 h, passes = closest approaches of the ground track; per pass burn epoch bracketed on the entry predictor's
  downrange error and burn epoch + retrograde delta-v shot (Newton) to the 120 km entry interface at -1 deg at a nominal
  50 deg bank; the first pass whose entry the guidance can fly is taken; refused without the landing reserve or a
  pass). Whether the guidance can fly a pass (`gnc::assess_entry_reach`, planner plan Task 7 fix round 2) is decided by
  flying the onboard entry guidance ahead in its own predictor from the converged shot's entry interface: the shared
  corrector cycle (`gnc::entry_corrector_cycle`, the code `EntryGuidance` runs) every 10 s [E] - bank re-solved for
  the range each cycle (and raised after a late reversal, as flown), the reversal rule, the corridor cut-back - the
  bank rate-limited between, the hand-over by the guidance's own predicate (`gnc::entry_terminal_handover`: Mach 5 once
  no reversal rolls, Mach 3 at the latest, below 40 km), then the terminal steering flown ahead from there (the
  guidance's Levenberg-Marquardt step `gnc::entry_terminal_steering_step` every 2 s [E], its step limit scaled to the
  guidance's steering rate - 0.3 per 0.5 s -, no nose sector; at 10 s near-identical entries handed over 1-8 km across
  and the gate verdict flipped; the gate within `corrector_tolerance_m`; a capsule: within the splashdown zone's
  radius). A case flies when it reaches the gate so and its flown peaks stay within the heat shield / crew limits and
  the 25 kPa q limit. Cases (planner plan Task 7 fix rounds 3-4): the nominal; the deorbit burn's execution errors
  propagated to the entry interface by the planner's own coast (`DeorbitParameters::burn_dispersion`, each alone, both
  signs): magnitude and pointing in and across the pitch plane = the vehicle's guided-cutoff residual
  (`[stage.recovery] deorbit_cutoff_residual` / `deorbit_pointing_error_deg`, required for a stage returning from orbit,
  `gnc::deorbit_burn_dispersion`, set by the session: ship 0.2 % / 0.1 deg [E, user 2026-10-17, as the lunar return
  campaign], Crew Dragon 0.1 % / 0.1 deg [E, Draco trim on GPS-aided navigation]), ignition +-1 s [E] - the engine's
  start-up transient; a burn that never reaches the interface is a failing case; the air (`gnc::entry_air_dispersions`:
  lift and drag together +-10 % [E], the entry campaigns' aero dispersion - a density error acts alike - and lift alone
  +-5 % [E], an L/D error of a ~1 deg trim error); a capsule also its mass +-3 % (capsule campaign, known onboard). Ship
  12 dispersed cases, capsule 14. Every case must fly; when no pass in the window does, the planner takes, of the passes
  whose nominal flies within the corridor, the one with the most dispersed cases flying (`DeorbitPlan::dispersion_robust`
  false, `reach_limiting_case` / `dispersion_note` German; printed as `[deorbit] Kein Überflug fliegt alle
  Streuungen...` only when the session flies the plan - not for previews or reserve searches -, in the bindings'
  `return` dictionary, the HUD's return block "Achtung" and the event line of `plan_ship_return` / `plan_capsule_return`).
  When the nominal flies but only dispersed cases of one energy side fail (shallower, faster, less drag, more lift,
  heavier, or the interface skipped; a case of energy below a quarter of one dispersion's size - an ignition-timing
  case - has no side), the planner aims the entry angle away from that side in steps of half the flight-path step (ship
  0.1 deg, capsule 0.05 deg), at most two. The reach (`DeorbitPlan::crossrange_reach_m`): the largest |crossrange| every
  case still flies to, targets moved outward in two waves of 8 (span / 64); the fly-aheads run on a pool of
  hardware-concurrency threads. The planned peaks (`DeorbitPlan::entry_peak_*`) are the nominal fly-ahead's whole-entry
  peaks. Measured (single cases, 50 deg, LEO 7.38-7.39 km/s, round 4): phases 0 / 90 / 180 / 270 fly -177.6 / -291.4 /
  -366.4 / -228.2 km after 20.3 / 20.0 / 0.7 / 0.3 h, reach 390-420 / 330-430 / 401 / 411 km, gates 35 / 61 / 7 / 90 m,
  all caught (11.8 / 7.9 / 13.2 / 13.7 t left), all robust. The earlier passes stay refused by physics: phase 0's -705 /
  -810 km and phase 90's -1,007 km fail nominally, phase 90's -522 km in the air -10 % case. Docked tanker: -81 km pass,
  robust, reach 409 km, caught (3.69 t main, 11.76 t headers). b4 aircatch: -382.5 km pass, robust, wait 102.5 h,
  caught, 13.5 t. Planned against flown peaks: heat within 2 %, q within 7 %, load within 15 % (the load peak of the
  terminal phase's pitch-up is not in the fly-ahead). Crew Dragon: the ~15 km pass is robust now (all 14 cases, gate
  misses 0.03-0.67 km in the fly-ahead; return 14.8 km at 37.1 h, gate 1.11 km, 3.49 g; ISS undock -14.0 km, reach
  43 km), no fallback; the fly-ahead itself agrees with the 6-DoF capsule campaign case 1 from the same interface (0.50
  against 0.55 km, 3.60 g both, reversal 5.38 / 5.36 km/s). `gnc::entry_crossrange_reach_m` (one-sign equilibrium-glide integral, closed form (pi^2/48) (L/D)^2
  sin(2 sigma) R; 660 km from circular speed) plus the Earth's rotation under the glide is only the pass prefilter's
  upper bound (ship 1,280 km: 664 km + 1,326 s of rotation; Crew Dragon 270 km). A mission's `max_crossrange_km` is refused (German load error). Without parameters
  (`plan_ship_return()`, bindings / key R) the ship flies its own `[stage.recovery] return_mission` (M,
  `ship_return_starbase`). The
  burn flies on `[stage.recovery] deorbit_engine_groups` - the ship: the belly sea-level Raptor alone (user
  2026-10-13), hull turned so that the thrust axis through the CG points along the burn
  (`FlightSession::execute_burn(node, groups)`; named groups only, no RCS fallback). Any burn more than 30 min [E]
  (`FlightSession::burn_drift_lead_s`) + 120 s preparation away waits in free drift (no RCS, coasted at any rate); the
  ship then turns to the burn attitude on the slow coast controller (0.5 deg/s) and holds it until the preparation.
  The first plan already coasts with the ship's own orbital drag [D] (`FlightSession::drag_area_per_mass_ahead`: a
  copy of the ship flown ahead in free drift over one 24 h tracking arc, nominal US76 air, measured with the tracker's
  estimator; 177 t-dry ship at ~217-225 t: 2.45-2.49e-3 m2/kg against 2.43-2.46e-3 tracked on the first day).
  Onboard tracking of a far deorbit: planned more than 4 h ahead, it is planned again every 24 h [E] and 3 h before
  its burn with the orbital drag `DeorbitParameters::drag_area_per_mass_m2pkg` that `gnc::estimate_drag_area_per_mass`
  solves from the arc since the first plan (C_D A / m 2.4-2.6e-3 m2/kg; the 3 h re-plan of the LEO return moves the
  burn by 0.7 s); a re-plan may take a pass up to 72 h [E] beyond the old one; a re-plan short only of the landing
  reserve is flown.
- Planner work off the sim thread (planner plan Task 11b). `FlightSession::set_realtime_planning(true)` (the game at
  warp <= 1) moves the planners that held the 200 Hz tick onto background threads; the vehicle flies its last plan /
  guidance until a result arrives. Off (tests, campaigns, time warp) each runs to its end on the sim thread at the tick
  it is due, exactly as before - the same functions on the same inputs, so the same results (checked bit for bit:
  "A first plan on the worker equals the one made at once", "The flip prediction on a worker equals the one on the sim
  thread", "A far deorbit re-plan on the worker equals the one on the sim thread").
  - First landing plans of the ship, the booster and the HLS (`ScvxLandingGuidance::begin` / `poll` with a
    `gnc::FirstPlanHold`, fix round 1): planned from the state the hold leads to (ship: flip thrust along the axis;
    booster: the unpowered fall, begun when the stop height's trend puts the ignition within the hold, lit at the
    hand-over; HLS: the braking thrust, begun the hold before the approach t_go) and in effect at the hand-over; a plan
    not there by then is waited for, so the sim time between snapshot and plan is the hold (<= 3 s, 1.3 x the latest
    measured wall time; see gnc-core). The refusal chain catch -> abort -> abort without the water plans from the same
    predicted state; an abort requested meanwhile turns a catch plan that arrives into an abort plan from it (M1).
  - Gate lead's flip prediction (`ShipGateAim`, a copy of the ship, one `vehicle::WorkerEnvironment` for all refreshes);
    the previous prediction aims until the new one arrives; switched to warp meanwhile it is waited for.
  - Return plans: key R (the binding's `SessionRunner::post_deferred` command follows request id
    `ship_return_request_outcome`: ends with "Rückkehrplanung abgebrochen: eine andere Rückkehr wurde geplant" when a
    mission leg plans another return meanwhile), the ISS departure leg (stays in Departure "Rückkehr wird geplant"),
    the lunar b4 hand-over (its status via the request's continuation), the tanker reserve at the Docked leg
    (`poll_tanker_reserve`: both return plans and the boil-off flight), the refuelling campaign's first tanker return
    (the whole side flight, booked when it arrives, at the latest at the campaign's end); far deorbit re-plans
    (`prepare_deorbit_replan` -> `compute_deorbit_plan` -> `apply_deorbit_replan`). The session polls them itself each
    step and waits for them once `ship_return_budget_s` (60 s [E]) of sim time passed since the snapshot (no coast past
    it) - at warp 1 never reached, at warp the plan lands within a minute of its snapshot. Applying checks the live
    state (I1): a return plan whose burn is no longer beyond its preparation (`burn_still_ahead`, burn_lead_s) is never
    armed and is planned again from the state then (`ship_return_resubmits`); a re-plan is dropped when the armed
    burn's preparation has begun or its new burn came too close (`deorbit_replans_dropped`; the old plan flies).
  - `vehicle::WorkerEnvironment`: the session's interpolated ephemeris and orientations fill their node caches on demand
    (not thread-safe), so each worker gets its own on the same CSPICE sources (serialised by `detail::lock_spice`) and
    grids; a node is a pure function of its index. Each job owns copies of its inputs (vehicle copies, the onboard
    guidance, request structs); results are read only after the future's get.
  - Still on the sim thread in real-time play: the entry guidance's own predictor-corrector cycles (its first after
    placement ~55 ms), the refuelling campaign's live re-plans (`plan_refuelling_campaign`); a new request or the
    session's end waits for a running job (no cancellation; up to the job's rest).
  - Diagnostics: `FlightSession::step_timing()`; `STARSIM_DEBUG_REALTIME` prints each path's sim-thread share and
    worker time (and "at once" the synchronous hold); `STARSIM_REALTIME_PLANNING` starts every session in real-time
    planning (unpaced test runs: the waits at the hand-overs show the worker times); hidden cases "Real-time play:
    entering ship from Mach 2.6 into the arms at the wall clock" (paced) and "Real-time play: a far deorbit re-plan off
    the sim thread".
  - Holds on the sim thread, synchronous -> real-time play (fix round 1, this machine, `STARSIM_DEBUG_REALTIME`):
    ship first catch plan 2.3 s step (Task 11b base) -> 0.1 ms begin, worker 1.7 s inside its 2.43 s hold, no wait
    (paced Mach 2.6: flip + burn 7719 steps, 0 over 5 ms, max 1.1 ms; caught 0.25 m / 0.97 m/s); booster first plan
    694 ms -> 0.1 ms (worker 0.7 s, hold 1.23 s; caught, pin load 1.67 MN against 1.30 MN); HLS approach plan
    430 / 473 ms -> 0.1 ms (worker 0.42 / 0.48 s); key R 9.3 s -> 12 ms begin + 0.1 ms apply (paced: 1332 steps,
    0 over, max 2.8 ms); ISS departure return 8.8 s -> same split; lunar b4 return 8.4 s -> same split; tanker reserve
    38.2 s -> 2.4 ms begin (worker 36.6 s); far re-plan 8.8 s -> max step 15.2 ms (paced). The refuelling campaign's
    side flight of the first tanker return is not measured (its plan alone ~9 s, then days of flight).
    Physics cost of the holds (forced real-time, unpaced): b4 aircatch caught with 9.1 t left against 11.6 t
    synchronous (the 2.43 s minimum-throttle hold), the booster with 53.5 t at ignition as before.
  `AutopilotState::Return` ("Rückkehr") coasts to 300 s before the entry interface, then entry guidance and flip/catch
  take over. From 200 km / 26.5 deg: burn after ~21-22 h, 65-78 m/s, 5.6-6.8 t.

## Standing decisions
- Entry hull-moment correction is a scale, not a torque integral (user 2026-10-17); flaps rest in the coming entry's
  pitch trim below 200 Pa (user 2026-10-18).
- Deorbit burn on the belly sea-level Raptor alone (user 2026-10-13); lunar braking Isp 380 vs 350
  split (user 2026-10-15).
- 10 deg flap-stop allocation margin was tried and dropped (user 2026-10-18): the cause was the liquid model, closed by
  propellant settling.

## Validation
- Ship entry replay of flights 12-14 (bank follows the recorded altitude): speed over time above Mach 3 within
  4.05 / 1.51 / **8.33** % rms (flights 13 / 12 / 14, limit 5: flight 14 outside, now a report row), below Mach 3 over altitude 4.7 / 9.0 %, belly-flop speed within 2-3 % (the nightly report "Ship entry: agreement with the webcast telemetry of flights 12-14", `webcast_report/ship_entry_telemetry.md`, parameters entry mass, CG, crossflow Cd, flap CdA). Since 2026-11-03 this is report, not gate (user 2026-11-03: "report, not gate; physical gate kept"): the normal-tier case "Ship entry flies the recorded paths of flights 12-14 without impact and with the trim in reach" gates no impact, lift fraction saturated < 10 %, > 200 speed points. The former red #1147 (flight 14 speed rms 8.33 % over the limit 5 %) is now a report row; the gate part is green or red by physics only.
- Entry guidance campaign `[ship][guidance][validation]` (12 dispersed entries to Starbase, truth aero +-10 %; planner
  plan Task 12, 177 t dry): the 9 cases with up to 45.8 t on board at the gate within 0.07 m, belly <= 0.13 deg off
  down, sink <= 84 m/s; red: cases 8 / 9 / 11 with 61.6 / 54.4 / 56.3 t (main-tank residue 19-27 t) - the schedule's
  alpha is not trimmable with that aft CG from Mach ~2 down (case 8 departs to tail first, 5.4 km; cases 9 / 11 at the
  gate 2.0 / 1.3 km off; `docs/known-issues.md` "Entry trim with main-tank residue"). Before (255 t dry): 11 within
  0.14 m, case 9 3.15 m. Load limit held. Guided 6-DOF 8.5 m.
- Flip/catch: test from the gate caught 0.01 m, 0.94 m/s sink, 5.7 t left; campaign `[ship][catch][validation]` (+-10
  m, +-10 m/s, propellant 35 t + 0-3.5 t, +-5 deg alpha / bank, belly engine failed in the flip): 20 of 21 caught, the
  engine-failure case aborts upright (lost 0; offset <= 0.17 m, sink <= 0.97 m/s, caught cases >= 3.8 t; without the sink scaling of `upright_descent` that case is lost). Session test from Mach 2.6, 28 km west: gate < 1 m, caught 0.02 m, 0.95 m/s, 2.1 t
  left. Lunar (Task 6h): direct 20 of 21 (limit 21; seed 2 upright 4.9 t, no ship lost, no strike, caught 4.2-12.2 t, peak 1171 kW/m^2; `docs/known-issues.md`), aerocapture 21 of 21 (4.9-11.3 t, raised perigee >= 197.4 km, peak 1084 kW/m^2).
- Ship return from orbit `[ship][mission][validation]` (phase 0-315 deg, orbit 190-230 km, 42-48 t, deorbit
  execution +-1 % / 0.5 deg via `FlightSession::apply_velocity_error`): 14 of 14 caught, gate 0-11 m, 5.4-8.3 t left.

## Diagnostics
- Ship: `STARSIM_DEBUG_FLIP=1` (also `[upright]`), `STARSIM_DEBUG_GATE=1`, `STARSIM_DEBUG_BURN=1`,
  `STARSIM_DEBUG_ENTRY=1` (one line per cycle; with constraint and reversal lines), `STARSIM_DEBUG_ENTRY_ATTITUDE`
  (1 Hz axis error, rates, RCS, flaps; flaps print in config order: aft right, aft left, forward right, forward left),
  `STARSIM_DEBUG_PLAN_PATH=1` (every plan: solve time, overlap, nodes in the plan frame), hidden `[.abort_clearance]` with `STARSIM_ABORT_HEIGHT`, `STARSIM_SHIP_SEED`, `STARSIM_GATE_HEIGHT`,
  `STARSIM_DEBUG_DEORBIT=1`, `STARSIM_DEBUG_RETURN` (a `return profile` line per plan); hidden tests "Ship catch single
  case" (`STARSIM_SHIP_SEED=<1-20>`), "Ship return single case" (`STARSIM_RETURN_PHASE=<deg>` or
  `STARSIM_RETURN_DISPERSED=<i>`, with `STARSIM_VERBOSE=1`).

## Open points and known limits
- Trim, 35 t (CG 25.50 m), Mach 25 / 15 / 8 / 4 / 3 / 2 / 0.8: aft 30 / 33 / 37 / 21 / 41 / 50 / 54 deg, forward 60 / 57 / 53 / 69 / 49 / 40 / 36. Frames (`task-5a-report.md`): aft 17-57 hypersonic;
  +15-19 deg over Mach 4-2 (+-5, magnitude +-30 %), the model +29, at the upper edge. 64 t (23.84 m): aft 62 / 64 / 66 / 57 / 73, at the 90 stop from Mach 2.5 down, short 42-619 m^3 per q.
  The CG question is not decided by the frames. Reds of this set: `docs/known-issues.md`.
- Flap authority at 65 km, Mach 21, 35 t, flaps alone: 3e5 N m on all axes at alpha 52-55; at alpha 60 roll ends at 1.4e5, yaw between 2e5 and 3e5 N m. Mach ~0.6, alpha 80.5, 212 t (trim 63.6 / 43.5 deg): yaw at unchanged roll / pitch +-1.8e5 N m above the authority floor (+-2.6e5 without it; the 290 t ship: +-8.4e5). Beyond flaps + RCS the yaw is cut on its own axis (fix round 1 above; the test measures the flaps' reach from the config, 2.15e5 settled, and demands 0.7 of it within reach).
- Ship catch: case 4 of the entry campaign is marginal (roundoff moves it between ~0.1 and ~1.2 km); an
  energy-short terminal phase (36 km from Mach 2.6) hits the gate but turns the nose too late and ditches safely (1.4 t
  left); since the vehicle plan's Task 6 the caught dispersed gates arrive with >= 3.8 t (see Validation).
- A low-propellant abort (21-23 t at the nominal gate; Task 6h with the structure keep-out, one uncommitted test line for the load) is lost at all three loads, open: 21 t 0.0 t left, 23 t 1.3 t left, no strike; 22 t struck at 4.0 t left (arm 7.61 m, tower box 0.93 m clear, hull bottom 18.95 m: the mount). Same result for an abort requested at 900, 600 or 400 m.
- Since planner plan Task 3b the dry mass and CG are derived (177 t, 22.03 m; entry CG 24.99 m with 35 t): the trim
  margin at alpha 60 holds (forward 54.3 deg), alpha 70 at 79.7 deg; the Mach 0.6 yaw-demand fixture and the nose turn
  at the basis freeze (gate miss 190 m) are green since Task 4, `docs/known-issues.md`. The rest of this item is the
  state before. Aero state frozen as on main (crossflow 1.5 [D], old entry alpha table, entry CG 25.50 m, dry 255 t, dry CG 23.516 m) because no tested combination (measured masses, geometry crossflow 1.07-1.10, alpha 59.5, CG 24.85 m) passes all flight replays and case 8; table and facts in `docs/known-issues.md`. Standing facts: 255 t dry is probably too high (about 175 +-25 t by judgement; entry masses 221 / 252 / 203-227 t +-20); the CG window is 24.49-25.88 m and 25.50 / 23.516 m are tuned values (bottom-up dry CG about 22.9 m, 19.4-23.5); the allocator's authority floor (fraction 0.25) has a narrow window (failures 8 / 9 / 2 / 0 / 9 of 10 at 0 / 0.15 / 0.20 / 0.25 / 0.30). Next step: the low Mach part (geometry set: flight 13 low 14.96 %, flight 14 low 22.06 %, too slow at Mach 3-6).
