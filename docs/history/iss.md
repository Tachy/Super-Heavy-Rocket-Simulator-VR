# ISS rendezvous and docking (milestone i) - history

Verbatim record of this subsystem's notes as they stood in `docs/subsystems.md` on 2026-10-25, before the split. Current state: `docs/subsystems/iss.md`. Kept for the reasons behind decisions, superseded numbers and earlier approaches; read only when that history is needed.

Spec `docs/superpowers/archive/specs/2026-10-19-iss-docking-design.md` (parts i1-i6; only Crew Dragon docks).

### ISS target body (i1)
Plan `docs/superpowers/archive/plans/2026-10-19-iss-i1-target-body.md`, ledger
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
Plan `docs/superpowers/archive/plans/2026-10-19-iss-i2-dragon-translation.md`, ledger
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
Plan `docs/superpowers/archive/plans/2026-10-19-iss-i3-phasing.md` (its header has the research and the four spec
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
Plan `docs/superpowers/archive/plans/2026-10-19-iss-i4-proximity.md` (its header has the research and the five spec
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
Plan `docs/superpowers/archive/plans/2026-10-19-iss-i5-docking.md` (its header has the research and the seven spec corrections,
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
Plan `docs/superpowers/archive/plans/2026-10-20-iss-i6-missions.md` (six spec corrections, approved by the user 2026-10-20),
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

