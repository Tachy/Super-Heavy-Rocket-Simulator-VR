# Ship entry, flip, catch and return from orbit

History and the reasons behind superseded decisions: `docs/history/guidance-missions.md`.
Index of the guidance / missions topics: `docs/subsystems/guidance-missions.md`.

## Model
Spec `docs/superpowers/archive/specs/2026-10-13-ship-return-design.md`.
- Entry (b1), belly first. Hull = `SlenderBodyAero` with `aero/ship_crossflow.csv` [D]. Four flaps (`aero/flap`,
  `[[stage.flap]]` [E] aft 2 x 48 m^2, forward 2 x 24 m^2 moved 20 deg leeward; Newtonian flat plates, `aero/flap.csv`;
  0 deg folded leeward, 90 deg broadside; 20 deg/s). `AttitudeController::Gains::flaps`: flaps cancel the modelled aero
  moment (hull + flaps), steer by a Newton step on their linearised moments, the rest goes to RCS; flap allocation is a
  box-constrained active set (free flaps re-solved, box = stops and the 0.25 s [E] flap-rate reach, all-flap
  moment-neutral centring, a secant only for a flap at a stop); below 200 Pa [E] the flaps rest in the coming entry's
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
  allocator uses the full CG point. The ship alone carries the stack's axial drag table [E] (`ascent_drag_table`).
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
- Flip and catch (b3, `gnc/ship_landing`): at the gate the three sea-level Raptors relight at 80 % [E], aft flaps
  fold, forward flaps go broadside until the tilt is < 30 deg; flip ~4 s and ~8 t, pushes the ship ~100 m along its
  nose. Once upright (< 15 deg, < 0.2 rad/s) SCvx plans (`ship_landing_parameters()`) fly 3 -> 2 engines: the belly
  engine shuts down at 20-40 m/s sink [D], the leeward pair lands (layout [D] from flight 13 splashdown video). Thrust
  of an asymmetric set goes through the CG along an axis tilted against the hull (~1.5 deg for the pair): plan states
  carry the thrust-axis attitude (`gnc::thrust_axis_body`). Ship LQR effort 0.03 of the gimbal authority [E], attitude
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
  tower top; the gate hands over higher when the sink exceeds 100 m/s [E] (gate x (sink / 100)^2), only below 40 km.
- Upright abort (`gnc::upright_descent`, deterministic: tilt <= 15 deg, away from the tower inside 60 m, sink profile
  sqrt(2^2 + 2 * 0.5 * (a_max - g) * h) with deceleration fed forward, never below the engines' minimum thrust): takes
  over an abort with no converged abort plan 2 s after the abort, an abort leaning beyond 35 deg, an abort whose usable
  propellant falls below 1.3 x a vertical stop's need (`upright_stop_propellant_kg`), and a flip whose first three
  plans reach neither the catch nor the sea. `sim::ReturnResult` (caught / upright abort: contact tilt < 20 deg and
  sink <= 5 m/s [E] / lost), `FlightSession::ship_return_result`. A pilot abort 1 s into the flip: water at 2.85 m/s,
  14.9 deg, 1.7 t left.
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
  1.6 / 2.3 / 4.9 % rms, below Mach 3 over altitude 6.0 / 5.2 %, belly-flop speed within 2-3 % (test "Ship entry
  reproduces the webcast telemetry of flights 12-14").
- Entry guidance campaign `[ship][guidance][validation]` (12 dispersed entries to Starbase, truth aero +-10 %): 11
  within 0.11 m, case 4 marginal (test holds it to < 1.5 km), belly <= 30 deg off down at the gate; load limit held.
- Flip/catch: test from the gate caught 0.01 m, 0.94 m/s sink, 5.7 t left; campaign `[ship][catch][validation]` (+-10
  m, +-10 m/s, propellant 35 t + 0-3.5 t, +-5 deg alpha / bank, belly engine failed in the flip): 20 of 21 caught, the
  engine-failure case aborts upright. Session test from Mach 2.6, 28 km west: gate < 1 m, caught 0.02 m, 0.95 m/s, 2.1 t
  left. Direct 21 of 21, aerocapture 21 of 21.
- Ship return from orbit `[ship][mission][validation]` (phase 0-315 deg, orbit 190-230 km, 42-48 t, deorbit
  execution +-1 % / 0.5 deg via `FlightSession::apply_velocity_error`): 14 of 14 caught, gate ~1 m, 8.9-13.9 t left.

## Diagnostics
- Ship: `STARSIM_DEBUG_FLIP=1` (also `[upright]`), `STARSIM_DEBUG_GATE=1`, `STARSIM_DEBUG_BURN=1`,
  `STARSIM_DEBUG_ENTRY=1` (one line per cycle; with constraint and reversal lines), `STARSIM_DEBUG_ENTRY_ATTITUDE`
  (1 Hz axis error, rates, RCS, flaps; flaps print in config order: aft right, aft left, forward right, forward left),
  `STARSIM_DEBUG_DEORBIT=1`, `STARSIM_DEBUG_RETURN` (a `return profile` line per plan); hidden tests "Ship catch single
  case" (`STARSIM_SHIP_SEED=<1-20>`), "Ship return single case" (`STARSIM_RETURN_PHASE=<deg>` or
  `STARSIM_RETURN_DISPERSED=<i>`, with `STARSIM_VERBOSE=1`).

## Open points and known limits
- Ship catch: case 4 of the entry campaign is marginal (roundoff moves it between ~0.1 and ~1.2 km); an
  energy-short terminal phase (36 km from Mach 2.6) hits the gate but turns the nose too late and ditches safely (1.4 t
  left); 5 of 21 dispersed gates arrive with <= 0.7 t.
- A low-propellant abort (21-23 t at the gate) is carried by the flip push next to the tower and strikes it - an
  abort-planning problem (keep-out with unconverged re-plans), open.
