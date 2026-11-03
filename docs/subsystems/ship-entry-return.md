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
  `flap_lowest_usable_deflection_rad` (edge-on, shadow), going down from the flap's best moment slope in this air, the first crossing of a quarter of it [E], 5 deg grid, interpolated; with the secant all along the ramp out of the shadow); below 200 Pa [E] the flaps rest in the coming entry's
  pitch trim (`EntryAeroTable::trim_flap_rad`). `gnc/entry_attitude` builds the setpoint from angle of attack and bank
  (belly at `[stage.aero] belly_azimuth_deg`). Nominal alpha over Mach `aero/ship_entry_alpha.csv` [D] (55 deg
  hypersonic, 50 near Mach 8, 60-80 towards the belly flop). Ship dry CG 23.516 m [D] (landing-mass CG dry + 35 t at
  25.50 m from the base). Entry control: 0.5 rad/s, bank changes <= 1 deg/s. `FlightSession::update_hull_moment_scale`
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
  reversing now ends closer across than holding. Bank rate 1 deg/s above Mach 10, 3 deg/s below Mach 5 [E]. From
  Mach 5 (latest Mach 3, once a reversal has settled) the terminal phase ("Endanflug") steers the lift in the velocity
  frame relative to the alpha schedule (Levenberg-Marquardt on the predicted gate miss); the gate ("Tor") is 800 m [E]
  above the pad (flight 13 relit at ~0.95 km [D]). `FlightSession::place_entering_ship`, `entry_status`. Tests use
  `predictor_j2 = false`. Predictor additions for lunar return: radiative heating, skips carried, exit orbit, truth
  air-density factor (`FlightSession::set_air_density_factor`), great-circle range unwrapped by the swept angle, a
  target behind the start's track counted as a turn less its distance ahead.
- Terminal steering floor (Task 6b): the most-lift alpha is never below `EntryAeroTable::lowest_trimmed_alpha_rad` (flying mass and CG; 1 deg search), in `EntryPredictorModel::steering_most_lift_alpha`, so guidance and predictor count the same lift. Steering differences stay on their side of `schedule_fraction` (`steering_difference`). After the basis froze, with a nose direction, the attitude slews on the `bank_step_smooth` ramp (`terminal_nose_slew_acceleration_radps2`, 0.5 deg/s^2 [E]). `upright_descent`'s braking feedforward scales with sink / profile sink, so a ship below the profile sinks instead of climbing.
- Flip and catch (b3, `gnc/ship_landing`): at the gate the three sea-level Raptors relight at 80 % [E], aft flaps
  fold, forward flaps go broadside until the tilt is < 30 deg; flip ~4 s and ~8 t, pushes the ship ~100 m along its
  nose. Once upright (< 15 deg, < 0.2 rad/s) SCvx plans (`ship_landing_parameters()`) fly 3 -> 2 engines: the belly
  engine shuts down at 20-40 m/s sink [D], the leeward pair lands (layout [D] from flight 13 splashdown video). Thrust
  of an asymmetric set goes through the CG along an axis tilted against the hull (~1.5 deg for the pair): plan states
  carry the thrust-axis attitude (`gnc::thrust_axis_body`). The plans' propellant floor counts only what the selected
  feed can burn (`gnc::feed_propellant_kg`: from the flip the header tanks, limited by the scarcer of LOX and CH4 at
  O/F 3.6; the main-tank residue never reaches the engines) minus the 1 t reserve; the upright abort's propellant check
  reads the same (Task 1c, 2026-11-03; before, the whole load counted and plans could fly the headers dry). Ship LQR effort 0.03 of the gimbal authority [E], attitude
  controller `Gains::actuator_delay_s` 0.75 s; flaps folded in the burn. Lateral phase tilt limit 20 deg [E].
  `FlightSession::place_landing_ship`; the entry hands over at the gate.
- Gate lead: `gnc::ship_entry_target` puts the entry target 220 m [E] back from the catch axis on the line tower ->
  pad with `EntryTarget::nose_direction_itrf` to the tower; only that approach is robust (the arms stand across the
  line). The terminal phase turns the nose once the fall is steep (basis frozen, -70 deg): steering to the nearest edge
  of the nose sector at fraction 0.2 (alpha ~86 deg), the lift direction turns about the air velocity at 3 deg/s [E],
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
  50 deg bank; the first pass within 60 km crossrange [E] is taken; refused without the landing reserve or a pass). The
  burn flies on `[stage.recovery] deorbit_engine_groups` - the ship: the belly sea-level Raptor alone (user
  2026-10-13), hull turned so that the thrust axis through the CG points along the burn
  (`FlightSession::execute_burn(node, groups)`; named groups only, no RCS fallback). Any burn more than 30 min [E]
  (`FlightSession::burn_drift_lead_s`) + 120 s preparation away waits in free drift (no RCS, coasted at any rate); the
  ship then turns to the burn attitude on the slow coast controller (0.5 deg/s) and holds it until the preparation.
  Onboard tracking of a far deorbit: planned more than 4 h ahead, it is planned again every 24 h [E] and 3 h before
  its burn with the orbital drag `DeorbitParameters::drag_area_per_mass_m2pkg` that `gnc::estimate_drag_area_per_mass`
  solves from the arc since the first plan (C_D A / m 1.9-2.5e-3 m2/kg; later re-plans move the burn by 0-16 s); a
  re-plan may take a pass up to 72 h [E] beyond the old one; a re-plan short only of the landing reserve is flown.
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
- Entry guidance campaign `[ship][guidance][validation]` (12 dispersed entries to Starbase, truth aero +-10 %): 11
  within 0.14 m, case 9 3.15 m (test: case 4 < 1.5 km, others < 100 m), belly <= 0.48 deg off down at the gate (limit 30), sink <= 102 m/s; load limit held. Guided 6-DOF 8.5 m.
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
- Flap authority at 65 km, Mach 21, 35 t, flaps alone: 3e5 N m on all axes at alpha 52-55; at alpha 60 roll ends at 1.4e5, yaw between 2e5 and 3e5 N m. Mach 0.68, alpha 80.5: yaw at zero roll +-8.4e5 N m, the demand at the terminal basis freeze ~1e6 (`docs/known-issues.md`).
- Ship catch: case 4 of the entry campaign is marginal (roundoff moves it between ~0.1 and ~1.2 km); an
  energy-short terminal phase (36 km from Mach 2.6) hits the gate but turns the nose too late and ditches safely (1.4 t
  left); since the vehicle plan's Task 6 the caught dispersed gates arrive with >= 3.8 t (see Validation).
- A low-propellant abort (21-23 t at the nominal gate; Task 6h with the structure keep-out, one uncommitted test line for the load) is lost at all three loads, open: 21 t 0.0 t left, 23 t 1.3 t left, no strike; 22 t struck at 4.0 t left (arm 7.61 m, tower box 0.93 m clear, hull bottom 18.95 m: the mount). Same result for an abort requested at 900, 600 or 400 m.
- Aero state frozen as on main (crossflow 1.5 [D], old entry alpha table, entry CG 25.50 m, dry 255 t, dry CG 23.516 m) because no tested combination (measured masses, geometry crossflow 1.07-1.10, alpha 59.5, CG 24.85 m) passes all flight replays and case 8; table and facts in `docs/known-issues.md`. Standing facts: 255 t dry is probably too high (about 175 +-25 t by judgement; entry masses 221 / 252 / 203-227 t +-20); the CG window is 24.49-25.88 m and 25.50 / 23.516 m are tuned values (bottom-up dry CG about 22.9 m, 19.4-23.5); the allocator's authority floor (fraction 0.25) has a narrow window (failures 8 / 9 / 2 / 0 / 9 of 10 at 0 / 0.15 / 0.20 / 0.25 / 0.30). Next step: the low Mach part (geometry set: flight 13 low 14.96 %, flight 14 low 22.06 %, too slow at Mach 3-6).
