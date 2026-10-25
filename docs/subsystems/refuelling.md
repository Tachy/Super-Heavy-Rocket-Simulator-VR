# Orbital refuelling (milestone r)

History and the reasons behind superseded decisions: `docs/history/refuelling.md`.

Spec `docs/superpowers/archive/specs/2026-10-20-orbital-refuelling-design.md` (+ follow-ups
`docs/superpowers/archive/specs/2026-10-21-refuelling-followups-design.md`): depot + tankers + HLS, back-to-back docking,
boil-off by heat balance, a campaign calculator from one flown tanker flight, a flown campaign.

## Model

### Vehicles and docking geometry (r1)
- `starship_tanker` = Block 3 unchanged (`[[stage]] from = "starship_block3:<stage>"`) + four dorsal probes (port
  `transfer_dock`, `mechanism = "probe_drogue"`, `soft_capture = "active"`) and the transfer connectors `lox_transfer`
  / `ch4_transfer`. `starship_depot` = Block 3 booster + a ship without heat shield, flaps, catch pins and recovery:
  236.5 t dry [E] (255 - 10.5 heat shield - 10 flaps - 1 pins + 3 insulation), four dorsal drogues (passive). The HLS
  carries the tanker's probes and connectors. Flown tanker: dry mass **209 t** (`dry_mass_kg` override on its `from`
  stage: no cargo doors, dispenser, interior; heat shield, flaps, headers stay) [E, user 2026-10-20].
- A `from` stage is the source vehicle's stage exactly; only `docking_port` and `transfer_connector` entries may be
  added (any other key, a missing stage / vehicle, a propellant mismatch or a cycle is a `ConfigError`).
  `tools/vehicle_report.py` follows `from` by the C++ rules (cycle bound, `<vehicle>:<stage>`, allowed keys, lists
  appended; `tools/test_vehicle_report.py`). Duplicate stage, docking-port and transfer-connector ids are config errors.
- Layout [E]: interface plane 0.5 m off the dorsal hull (`[20, 0, 5]`, axis +z, up +x), probes / drogues at stations
  12 / 28 m, y +-1.5 m; connectors at 8 m (LOX) / 24 m (CH4), y 0. Mated (`vehicle::mated_stage_transform`):
  parallel, rolled 180 deg, centrelines 10 m apart, 1 m between the hulls, every probe on a drogue and every connector
  on its counterpart (test "Tanker and HLS mate back to back with the depot").
- `Vehicle::set_tank_mass(stage, tank, kg)`: clamped to `[0, capacity]`, refuses NaN / infinite; a refilled feed
  source re-arms starved engines, the cold-gas RCS follows the tanks.
- Ports mate only with equal mechanisms, probes on the chaser (`vehicle::port_mismatch`);
  `FlightSession::set_rendezvous_target` refuses others (German message). The session arms the IDSS or the
  probe/drogue mechanism for an active / passive pair of the same kind.
- Tanker translation after a transfer (10 t per main tank, 310 t; pure capability, no torque): +-z 11.3 kN = 0.037
  m/s^2, +-y 5.7 kN = 0.018 m/s^2, +x 3.0 kN = 0.010 m/s^2 (settling), -x 3.0 kN with the docking pair.
- `vehicle_report`: depot ship ideal delta-v 7,328 m/s (no recovery reserve), tanker ship 6,697 m/s.
- Depot header tanks (35 t) stay aboard: transfer connectors take main tanks only, usable depot fill 1,600 - 35 t.

### Boil-off by heat balance (r2)
- `vehicle/boil_off`: every hull tank (main tank of the stage diameter) of a stage with `[stage.thermal]` is a cylinder
  of its full-load length, 36 strips around the circumference. Per strip a quasi-steady balance
  `absorbed = eps sigma T^4 + (T - T_boil) / R_wall`, solved by Newton; the strips' leaks summed are the tank's heat,
  clamped at 0 (no condensation), over the latent heat the vented flow. Starship's hull is the tank wall (integral
  tanks): the bare hull's `R_wall` is tiny; tiles / felt and MLI vacuum are part of the effective resistance.
- Sources (`environment/thermal_radiation`): Sun 1361 W/m^2 at 1 AU with Earth / Moon disk occultation; Earth albedo
  0.30 x solar flux x view factor x `max(0, cos zenith of the Sun below the vehicle)` [E]; Earth IR 239 W/m^2 x view
  factor (absorbed with the emissivity). View factor: exact plate-to-sphere (tilted element, horizon-crossing
  formula); spherical Earth of mean radius. Moon radiation (`environment::earth_orbit_radiation` with a Moon): albedo
  0.11 [S] and infrared from the sub-vehicle regolith in radiative equilibrium, night 95 K [S] (Diviner); negligible in
  LEO (view factor < 3e-5).
- Wetted wall (user 2026-10-20): only the wall the liquid touches feeds it; on the dry part the heat must also cross
  the ullage gas (`ullage_resistance_m2k_w` 100 m^2K/W [E]), so a dry wall runs hot and re-radiates. Liquid position:
  apparent gravity at the tank's centre = Earth tide + (-specific force: drag at the end of each coast, thrust in
  ticks) + centrifugal (`vehicle::apparent_gravity_body_mps2`); Bond number rho a r^2 / sigma (`surface_tension_n_m`
  [E]): Bo <= 1 the liquid spreads over the walls, Bo >= 10 settled below a flat surface across the apparent gravity
  (`settled_wetted_fractions`), log-linear between [E]. At 200 km Bo ~20-90 for a full-radius tank: settled. A 10 t
  residue loses ~1.4 t in 22 h.
- Simplifications (switchable as a whole by the fidelity flag `boil_off`, default on; `FlightSession::set_boil_off`):
  no skin heat capacity, no conduction around the circumference; no tank pressure / temperature / subcooling /
  stratification (conservative); the dry wall's small leak is vented like boil-off; liquid geometry quasi-static (no
  settling lag, no slosh), Euler and Coriolis terms left out; domes, ends, header and RCS tanks take no heat; no
  shadow of other vehicles or flaps; the Moon only shadows near Earth.
- Values: depot `R_wall` 63.7 m^2K/W [D] = MLI-3 x 31/4 (30 effective layers [E], leak ~ 1/(n+1)), cover 0.30 / 0.80
  [E]. Block 3 ship (and tanker): bare stainless 0.40 / 0.15, 0.01 m^2K/W [E]; tiles 0.85 / 0.85, 1.5 m^2K/W, +-90 deg
  around the belly [E]. Methalox: LOX 90.19 K / 213.1 kJ/kg, CH4 111.67 K / 510.8 kJ/kg [S: NIST]. The HLS is
  insulated like the depot [E] (absorptivity 0.30, emissivity 0.80, wall 63.7 m^2K/W): ~0.2 t/day in transit.
- Flight: `FlightBody` evaluates every 10 s of free flight [E] and after every coast, vents by the trapezoid of the
  last two evaluations per tank (`Vehicle::set_tank_mass`, stops at 0), not below 100 km in air [E], not clamped /
  caught / landed / resting / docked or carrying a visitor. A hold (docked, caught, landed) and the release forget the
  last rates; a body resting on the Moon does not boil in `step()`; vehicles without `[stage.thermal]` skip the
  evaluation; a coast that ends in the Moon's sphere tracks the Moon as the orbit reference.
  `FlightBody::boil_off_rate()` = flows at the last evaluation. Separation clears the stored rates.
- Docked boil-off: a station carrying a visitor (and the held chaser) does not vent in `FlightBody`; the session
  evaluates both bodies every 10 s and after every coast in the compound's environment and vents through
  `set_tank_mass` (trapezoid), keeping the compound CG and mated hulls exact.
- Orbit average `vehicle::orbit_average_boil_off`: one two-body orbit, LVLH-held attitude, Sun fixed, Earth shadow.
- Deorbit planning (`FlightSession::plan_ship_return`, re-plans): the earliest pass is checked by flying a copy of the
  ship ahead to the node; the scarcer of LOX / CH4 limits the burn; usable >= 1.10 x the burn propellant [E]. Short:
  refused with "... verdampft ..."; a re-plan that finds the tanks short calls the burn off. `DeorbitPlan::boil_off_kg`;
  the check uses the deorbit group's mixture ratio; a re-plan keeps the boil-off until its burn. A burn whose engines
  all starve is cut off with `FlightSession::burn_abort_reason()`; failed engines report "Triebwerke ausgefallen".
  `DeorbitParameters::boil_off_kgps` stays an offline planning knob.
- Missions: `ship_return_starbase` starts with 50 t [E]; ship return campaigns size their start in 2 t steps until the
  onboard plan holds. The Block 3 lunar missions (`ship_lunar_return_*`, `ship_lunar_aircatch_llo`) set
  `[mission] boil_off = false` (user 2026-10-20): with boil-off on the braking burn (1,032 m/s) is aborted for want of
  propellant after 3.4 d.
- Open: active cooling (solar-powered cryocoolers, user question 2026-10-20) would be a cooling power per tank
  subtracted in the balance - not modelled, SpaceX has announced none.
- Not modelled (r6 loss items): the pressurant for the delivering tank and the vented displaced gas of the receiving
  one (~1-2 % of the transfer [E]).

### Rendezvous and probe/drogue docking (r3)
- -x docking thrusters (user 2026-10-20): the tanker (`[[stage.rcs_pod]] docking` on its reused ship stage) and the HLS
  (forward pod + "aft") get two 1,500 N cold-gas thrusters pushing along -x [E] at station 46 m (without along-track
  thrust `v_along + 2 n x_radial` is conserved, CW). Translation at 10 t main propellant: equal +-x (3,000 N each).
  The Block 3 ship keeps none. `Vehicle::rcs_supply_kg()`: main-tank cold gas counts the main oxidizer + fuel tanks
  (not the headers: the landing reserve), else the RCS tanks; proximity and phasing budget it.
- Target block (`[stage.station.proximity]`, `sim::with_station_limits`): keep-out radius, approach ellipsoid, Waypoint
  0 / 1 / 2 and fly-around radius override the code's ISS defaults (and a caller's own values: safety limits of the
  target must not be loosened) only where the target sets them; the ISS sets none. The depot takes the ISS values [E]
  (200 m, 4 x 2 x 2 km, 400 / 220 / 20 m), flies the ideal LVLH attitude nose along +V-bar, dorsal side (the drogues)
  to nadir. A station is one stage: `add_station` refuses a stack ("the depot without its booster",
  `config::top_stage_config`). Target drag without a fixed area: C_D 2.2 on the end-on disk.
- R-bar from below (`gnc::DockingApproach::RBarBelow`, port axis dominant towards nadir): Waypoint 0 400 m below the
  depot lies on its port axis; a chaser on the axis beyond an on-axis waypoint inside the corridor approaches along it.
  No fly-around, no CW transfer.
- Probe/drogue interface (`contact::probe_drogue_contact`, all [E]): four probe heads (radius 0.05 m) in conical
  drogues (mouth 0.35 m, seat 0.06 m, depth 0.50 m; capture radius 0.30 m); per head the cone wall, the seat (the
  probe's attenuator) and the plate beside the mouth, each a spring-damper with friction 0.1; the heads' forces give a
  torque about their centre. A head on its seat within 2 deg tilt / roll latches (soft capture). The force steps from
  the cone wall to the mouth plate at the rim (mm penetration, 2 kN/cm); kept.
- Mechanism (`contact::probe_drogue_mechanism`): the IDSS phase machine with the probe stroke (0.30 m) as the ring
  extension (first contact at 0.80 m port distance), four structural latches as the hooks (180 s). Joint: translation
  `k = m_c w^2`, rotation `k = I w^2`, w = 0.5 rad/s, critically damped; a 0.10 m/s contact peaks at ~`2 m_c w v` (49
  kN for 490 t). Limits [E] ("Auslegungsgrenze"): dynamic compression 100 kN, static 60 kN, tension 60 kN, shear 50 kN,
  bending / torsion 2 MN m; 0.25 m/s overloads. Separation springs 150 J (~0.03 m/s). Envelope (spec): closing
  0.03-0.10 m/s, offset <= 0.10 m, angle and roll <= 1 deg. Contact plane, re-arm distance and the DockingSim coast
  limit come from the mechanism (`first_contact_m`, `reach_m`).
- Hull collision: only a nose port (axis +x, Crew Dragon) ends the sampled hull below it (whichever port is listed
  first); HLS legs are sampled along their struts (root 4 m above the stage bottom [E] to the deployed foot, 12 m out;
  clear of the depot at the mated pose). Depot has two capsules [E] (tank section r 4.5 m, nose r 2.5 m). Mated, the
  tanker's hull clears the depot by ~1 m, 1.5 m closer it collides.
- Phasing (`plan_rendezvous` accepts Starship): burns on the deorbit set - the single belly Raptor (`sea_level_3`) -
  with transients up to 0.6 of the burn; below that (~7 m/s) the burn flies on the cold-gas RCS along +x and is
  re-targeted with that model; modelled at the throttle they fly (minimum throttle between the two minimum impulses).
  Budget: `rcs_supply_kg`. Between burns a Starship holds the docking orientation (LVLH rolled 180 deg). Refused with no
  main propellant ("Kein Rendezvous: kein Treibstoff").
- Compound refresh (`FlightSession::set_tank_mass`): a tank change of either docked body moves the compound CG (the
  station body's state), keeps the velocity and the hulls in place; the attached mass follows the chaser's hull-fixed CG;
  `compound_mass()` is recomputed.
- Departure burn (gnc, all chasers): the target velocity is the free CW drift of the post-impulse state. The filled HLS
  leaves the nearly empty depot in 143 min on 1.96 t.
- Max capture retries (`max_capture_retries`: refuelling 2 [E], ISS 0, the pilot resumes) resume a proximity flight
  waiting at Waypoint 2 after a retreat, then "Andocken fehlgeschlagen nach N Neuversuchen".

### Propellant transfer (r4)
- Hardware [E]: per `[[stage.transfer_connector]]` `line_diameter_m` 0.15, `discharge_coefficient` 0.6, `chilldown_kg`
  150 (LOX) / 60 (CH4); per ship `[stage.propellant_transfer]`: the delivering ship holds its ullage at 3.5 bar (spec
  3-4 bar), the receiving ship vents to 1.5 bar. Tanker, depot and HLS carry both.
- Flow (`vehicle/propellant_transfer`): orifice law `mdot = C_d A sqrt(2 rho dp)` per line (pressure difference per
  line), the coupled pair's smaller C_d A, densities of the loaded propellant: LOX 232 kg/s, CH4 141 kg/s at 2 bar. No
  back-flow, no hydrostatic head.
- Settling: the chaser's +x cold gas pushes the compound at 1e-4 g [E, spec] (capped by its pure +x capability; gas at
  the RCS Isp from its main tanks in proportion to oxidizer and fuel). Modelled as the average thrust, one velocity
  impulse per 10 s step (0.01 m/s) to the compound; no pulses, no torque. Settle time `sqrt(2 L / a)` [E] for the
  longest involved tank (186 s for 16.9 m). A delivering outlet (aft dome, -x) is covered when the apparent gravity at
  the tank centre (tide and rotation about the compound CG `LiquidEnvironment::cg_offset_body_m`, minus drag and push)
  gives Bo >= 10 within 30 deg [E] of -x (`vehicle::outlet_check`).
- Sequence (`sim/propellant_transfer`, pure state machine): Settling -> Chilldown (120 s [E], the chill-down mass
  vented from the delivering tanks; a chill-down after a pause vents again) -> Flowing -> Complete; an uncovered outlet
  pauses ("Transfer unterbrochen: Treibstoff nicht am Auslass", without gas "... kein Kaltgas zum Setzen"); a pause
  longer than `max_pause_s` (600 s [E]) aborts ("Transfer abgebrochen: ..."). A line closes at the delivering floor, a
  full receiving tank or its limit.
- Session (`FlightSession::start_transfer(TransferRequest)`, `stop_transfer`, `transfer_state`;
  `flight_session_transfer.cpp`): both directions (`ChaserToStation` tanker -> depot, `StationToChaser` depot -> HLS
  with per-line limits); the push always comes from the chaser. Steps every `request.step_s` (10 s) of epoch, the step
  fires within half a tick. Tank moves through `set_tank_mass`. Refusals (German): not docked, autopilot active, no
  coupled connectors, no transfer pressures, nothing to move, hooks opening; a second start ("Transfer läuft bereits")
  and undocking ("Abkoppeln nicht möglich: Treibstofftransfer läuft") are refused while one runs; a cleared target, a
  detach or a new scenario stops it. The settling push gives only the impulse of the gas it drew
  (`TransferState::settling_impulse_ns`).
- Tanker reserve (`mission::tanker_reserve`, `FlightSession::tanker_reserve` / `tanker_transfer_request`): deorbit
  burn by the rocket equation from the mass after it, with the delta-v of `ship_return_preview` (8-day pass search
  [E]), x 1.10, + gas + boil-off until the burn 2 t [E] + unusable 1 t [E]; split at the deorbit set's mixture ratio.
  Reserve from the flown return [D]: gas 3.5 t (departure 0.3 t + entry ~3 t), boil-off 2 t split 56 % CH4.

### Settling before an in-orbit ignition (f3)
Fidelity flag `FlightEnvironment::ignition_settling` (default on, one source `vehicle::ignition_settling_default`;
`FlightSession::set_ignition_settling`): off means no push, no ingestion check and no reserve.
- Settling state (`Stage::settled` 0..1, `Vehicle::update_settling`): while every outlet of the stage's main tanks is
  covered (`outlet_check`) the progress rises at 1/sqrt(2L/a), otherwise it falls at the same rate; a = the weakest
  apparent gravity at the tank centres, no acceleration: unchanged. L = the longest ullage, tank length x (1 - fill), at
  least 1 m [E]. Ticks and coasts update it; clamped, caught or landed: 1; every vehicle starts settled. Tanks with
  `propellant_management_device = true` [E] (Starship header tanks of Block 3 and the depot, Dragon `mmh_nto`) are
  always covered; header feed needs no settling. The tide is the orbited body's (`LiquidEnvironment::centre_gcrf_m`,
  `gm_m3ps2` from the tracked reference body).
- Gas ingestion (`Vehicle::update_engines`, Off -> Starting): only in orbit (`FlightBody::in_orbit`: closed orbit,
  osculating periapsis above the reference body's equatorial radius). An unsettled stage's engines fail (`failed`,
  `starved`); the session cuts off, disengages and sets `burn_abort_reason` "Gasansaugung: Treibstoff nicht gesetzt,
  Triebwerke ausgefallen"; ISS / refuelling mission sequences end on it.
- Settling push (`vehicle::settling_push`): cold gas along +x at the full pure-translation capability (Starship 3,000
  N, 4.37 kg/s); lead = 1.25 [E] x sqrt(2L/a). The autopilot runs it from its lead until the engines give thrust (an
  RCS overlay on the command); its delta-v is credited to the burn (`BurnExecutor::credit`). An ignition before the
  stage is settled waits while the push still gives force (judged by its capability), at most 3 leads [E]. Burns start
  `burn_lead_s()` = 120 s preparation + the lead before ignition (lunar, Earth return, rendezvous schedules; waiting,
  drift and coast limits include the lead): the turn to the burn attitude ends before the push. Lunar PDI: the descent
  coast starts the push one lead before the braking engines only when not settled; while unsettled at the ignition it
  holds as the burns do. Manual `FlightSession::settle(bool)` (game `FlightSim.settle(enabled)`): refused while docked,
  while a proximity approach flies (engaging one switches it off) or while an autopilot cold-gas burn or the
  rendezvous flies, cleared by a new flight. Snapshot: `autopilot_status().settling`, `burn_abort_reason`,
  `body_stages()[i].settled`.
- Numbers: tanker / Block 3 / HLS leads 42-96 s, 0.2-0.75 t per push (HLS TLI 42 s / 0.49 t full, MCC at 609 t main
  88 s / 0.74 t; tanker raise 80 s). Gas ~ sqrt(F) for a given travel: weaker cold gas thrusters would settle on less
  gas, at the cost of a longer lead.
- Planner reservations: the session's phasing plans reserve the push with the tanks as now per burn
  (`PhasingParameters::reserve_per_burn_kg`), the window cost adds `PhasingCostModel::gas_per_burn_kg` per raise, boost
  (and phase) burn, the deorbit check requires the next push on top of its margin, the descent budget one push; budgets
  planned before the tanks drain use `vehicle::settling_gas_bound_kg` (the largest push gas at any fill; peaks at a
  middle fill) per in-orbit ignition - lunar mission budget and campaign HLS demand: all legs but the approach and the
  surface ascent. `BudgetLeg::ignitions`: LLO 3 -> 5, NRHO 6/7 -> 9/10 (a lower bound: MCCs, plane trim and NRHO
  station keeping are not budgeted legs).
- `Vehicle::rcs_capability` uses a local allocator when the cached one is stale (the planner query must not move the
  flight's attitude-control reference).
- Ship catch: the pushes before TEI and the Earth braking shift the lunar-return gates by ~1.4 s / 20 m; the lateral
  phase's tilt limit is 20 deg; all 21 direct seeds are caught (ship lateral LQR effort 0.03; upright abort); see
  `docs/subsystems/guidance-missions.md`, "Ship catch propellant margin".
- Step dependence of burn end states (measured, kept, decision f4): the same 100 m/s prograde burn 2 h ahead, flown
  after 10 s or 600 s `advance_to` steps (hidden case `[.step_dependence]`), from a 120 x 190 km orbit differs by 21 km /
  35 m/s at ignition (122 km / 125 m/s at the end); from 200 x 270 km by 156 m / 0.18 m/s (13 m / 0.011 m/s). Cause:
  the free drift before the burn (`burn_drift_lead_s`, no attitude control): at a 120 km perigee the air torque tumbles
  the ship chaotically. Physical sensitivity, not a numerics fault; the f1 phasing orbits (>= 150 km) are not affected.

### Mission chain and reference flights (r5)
- `IssMissionSequence` (`flight_session_iss_mission.cpp`; the ISS paths unchanged): legs `Raise` (after the ascent the
  apsis burns to `raise_to`, the window's phasing orbit: belly Raptor with transients up to 0.6, else cold gas),
  `Transfer` (Docked -> `start_transfer`: the tanker down to its reserve, `MissionTransfer::TankerReserve`, or the
  chaser's main tanks up to full, `FillChaser`) -> undock -> Departure, the ship return (`ship_return`:
  `release_tower`, `plan_ship_return`, Return until caught) and `WaitContinue` (coasts to `continue_epoch`, then
  `continuation`: the lunar mission). `ascent_to_station`: the inserted depot becomes the station (`make_station`:
  ideal LVLH attitude at once). The Ascent leg ends only after the engines' tail-off (of the controlled body only).
  Leg marks (`iss_mission_marks`: mass, main propellant, RCS supply, station main propellant) are the measured numbers.
  A generation counter keeps a sequence set by a continuation; a docked start passes the Docked gate (an extra Docked
  mark before Transfer); a collision during the transfer ends as "Kollision"; the nose-cone body index shifts in
  `erase_body`; `[mission] port` of refuelling missions is a config error (the chaser docks with its first port).
- Caught booster (`release_tower`): before the tanker's deorbit burn the caught booster is removed from the session
  (indices shifted) and the tower re-armed.
- Missions (`config/missions/`, `config/refuelling_mission_config.hpp`, `sim/refuelling_mission`; started at the
  config's epochs): `depot_launch` (`depot_ascent`: 200 x 200 km), `tanker_flight` / `tanker_flight_orbit`
  (`tanker_ascent`), `hls_refuel` / `hls_refuel_orbit` (`hls_ascent` on `starship_hls_launch` = Block 3 booster + HLS
  stage). The depot of the tanker / HLS flights is placed on a synthetic orbit (`[depot]`, J2-advanced mean elements);
  `hls_refuel` puts it in the lunar mission's parking plane (`lunar_parking_state` at the lunar epoch, converted to the
  true equator frame and run back by J2: 0.031 deg off after two days). `continue_lunar_landing_mission` plans the
  outbound flight and the budget from the live HLS ("zu spät" after its epoch). Config `hls_lunar_llo_depot` (26.5
  deg, 2026-11-02).
- Tanker ascent (user decisions 2026-10-20, [E] placeholders until the real tanker is known): insertion **120 x 190
  km**, raised to 170 km. The booster return reserve is the bottleneck (lighter booster or less landing reserve break
  the catch); probes `[.r5_probe]` "Tanker ascent levers".
- Numbers (current, planned kick 74.25 t per reference tanker; see `reference_flights.toml`, derived heights
  `docs/subsystems/earth-terrain.md`): `depot_launch`: station 453 s after liftoff, 199.6 x 211.7 km (199.7 x 211.3 km
  on the derived heights), initial fill 55.5 t main, boil-off 17.6 kg in 90 min, booster caught. `tanker_flight` (pad):
  window 79.8 h after earliest (48 h uncrewed phasing; the crew's 28 h: 177 h), SECO 444.3 s, arc 12.17 deg [D];
  residue 105.6 t, settling gas 0.98 t, main at undock 12.6 t, rendezvous 17.93 t, delivered LOX / CH4 58.9 / 14.7 t,
  main at catch 1.38 t. `hls_refuel` (pad): SECO 428.3 s, arc 11.68 deg [D], residue 216.4 t, after the raise 208.3 t,
  HLS main at docking 203.5 t, filled to 1,585 t (depot 1,500 -> 83 t), waited 5.3 days (2.7 t boil-off), lunar
  mission planned from 1,580 t; `hls_refuel_orbit`: docked 195 t, TLI 3,152 m/s. Full HLS departure 1.96 t of push gas.
- Numbers from tests (tanker 200 t main propellant, depot 600 t, 200 km / 26.5 deg): phasing from 170 x 190 km, 8 deg
  behind: 5 burns, 12.0 t main propellant, Waypoint 0 missed by 13.9 m / 0.13 m/s; Waypoint 2 -> docked: contact at
  0.075 m/s, offset 0.07 m, contact -> docked 10 min, peak compression 19.1 kN; campaign from the phasing orbit: docked
  after 12.6 h (Waypoint 0 -> contact 49 min), peak compression 22.0 kN. Transfer: 940 s (186 s settling + 120 s
  chill-down + ~630 s LOX flow), delivered LOX 144.6 t / CH4 40.3 t, settling gas 1.83 t at 1,333 N, chill-down 0.42
  t, boil-off of both ships 12.6 kg; tanker left LOX 10.0 t / CH4 2.75 t (floors 10.0 / 2.8 t [test]). Depot -> HLS
  (50 t aboard) 20 / 5.5 t: 400 s, 595 kg of the HLS's gas. Docked 2 h without transfer: tanker (200 t) 532 kg, depot
  (600 t) 32 kg. Tanker reserve at the depot: deorbit 59.4 m/s -> burn 5.6 t, reserve 10.1 t.
- Boil-off numbers (200 km, 26.5 deg, LVLH identity = nose +V, dorsal nadir; `STARSIM_VERBOSE=1`): depot (full) 444
  kg/day = 0.028 %/day at beta 0, 461 kg/day = 0.029 %/day at beta 60 deg; bare tanker 15.9 t/day (beta 0), 22.9 t/day
  (beta 60). Conventionally insulated cryogenic tanks in orbit ~0.1 %/day (plausibility only).

### Campaign calculator (r6, with f1 and the follow-ups)
Plan files `docs/superpowers/archive/plans/2026-10-20-refuelling-r6-campaign.md`,
`docs/superpowers/archive/plans/2026-10-21-refuelling-f1-phasing-per-window.md`.
- Calculator `mission::refuelling_campaign` (pure): the depot loses boil-off, drag makeup and attitude gas in daily
  steps; tankers arrive at their windows (pad turnaround `launch_interval_s` 1 day [E], one depot port: arrivals 6 h
  apart [E]) and add their delivery less 1.5 % vent [E] (pressurant + displaced gas; the vent grosses up as net /
  (1 - f), `mission::gross_for_net_kg`), clamped to the tank volume; once the fill reaches the demand the HLS window is
  checked - the campaign closes when the depot still holds the demand at the HLS arrival. German reasons: depot volume
  (also: full tanks that cannot bridge the losses until the HLS arrives), no window, a tanker delivering nothing (also
  NaN), losses eating the delivery, 60 tankers. An initial fill above the volume is clamped; levers are checked by
  `sim::campaign_lever_error` (CLI and game). Every candidate's refusal reason is reported; `candidates_km = []` and
  `reference_phasing` without `periapsis_km` are refused, `window_margin_h` is read without candidates.
- Setup `sim::plan_refuelling_campaign` (levers `CampaignLevers`, std::nullopt = reference): t = 0 at `tanker_flight`'s
  earliest (2026-10-20), the depot on its `[depot]` orbit at the altitude lever with the measured initial fill; windows
  by the flown logic (`plan_launch_window` against the depot, phasing orbits shifted with the depot, search 60 days
  [E]; of the launches reaching the same Waypoint 0 (within 3 h [E]) the latest is flown); depot boil-off by
  `orbit_average_boil_off` with the day's fill and Sun; drag end-on (Cd 2.2, pi (d/2)^2, US76) made up by one Raptor
  `sea_level_3`; attitude gas 20 kg/day [E]. HLS demand: `hls_lunar_llo_depot` placed in a scratch session, outbound
  planned, budget legs (`lunar_mission_legs`) -> the smallest closing mass (`lunar_budget_required_mass`); demand =
  (required propellant + HLS boil-off while waiting - propellant at docking) x 1.015 + chill 0.42 t; the HLS wait boils
  off with the Sun of its arrival (`CampaignModels::hls_wait_kg`) and must fit its free tank volume; TLI starts from
  the depot altitude; the HLS window is checked before every tanker.
- Delivery model: delivery = base + tanker boil-off over the phasing difference - (chosen raise/boost propellant - the
  reference window's); the HLS's extra over its reference window is `CampaignFlight::extra_demand_kg` (signed,
  includes its phasing boil-off difference), held in the depot at its arrival; `CampaignFlight::phasing_orbit`; the
  depot-height lever shifts the candidates (insertion stays), so `altitude_cost_kg` keeps only the deorbit difference
  (Hohmann up from 200 km at the docked mass, deorbit difference to a 50 km perigee [E] at the return mass x 1.10).
  Calibration (user 2026-10-21) [D]: `campaign_offset_below_t` 1.7 / `campaign_offset_above_t` 2.25 by the phasing
  orbit's periapsis against the depot's mean radius; the tanker's phasing boil-off with the flown free coast's 3.5
  t/day (orbit average at fixed LVLH said 15.9) and the HLS's 0.47 t/day [D].
- Phasing orbit per launch window (f1): each window picks its phasing orbit (tanker only; depot passive).
  `mission::phasing_schedule`: a chaser above the target (periapsis more than `coelliptic_height_m` above the target's
  orbit, else "Bahn kreuzt die Zielbahn") waits until the lower, faster target has caught up to the same
  `required_at_boost`, then a retrograde boost descends to the coelliptic orbit (`PhasingSchedule::above`; the phase
  burn stays below-only). Raise (`gnc::next_raise_node`): below the depot one burn at apoapsis (periapsis), above it the
  apoapsis first, then the periapsis at the new apoapsis; the flown sequence is fixed at the start
  (`raise_periapsis_pending_`); the next burn waits for the tail-off; a dry raise ends the mission with
  `burn_abort_reason`; the second raise burn only raises the periapsis. Flown 120 x 190 -> 250 x 270 km planned: 263.8
  x 270.0 km. `mission::plan_launch_window`: `LaunchWindowParameters::phasing_orbits` (empty: the separation orbit, ISS
  unchanged to the microsecond - test) and `PhasingCostModel`; at each crossing every candidate's raise is estimated
  two-body (`raise_estimate`) and its schedule run from the raised state with the remaining limit; the cheapest
  feasible wins (`phasing_cost`: raise + Hohmann boost (+ phase burn) by the rocket equation, plus boil-off to Waypoint
  0). `LaunchWindow::phasing_orbit`, `cost`, `raised`, `raised_state_gcrf`. A candidate whose mean radius lies within
  10 km [E] of the coelliptic height (below) or of the target's orbit plus that height (above) is refused ("zu nah an
  der Zielbahn"). Config `[phasing]` of `tanker_flight` / `hls_refuel`: `candidates_km` [E] (150, 160, 170, 180 x 190
  km below; 220 x 240, 250 x 270 km above), `planning_mass_t` [D] 308 / 328, `planning_boil_off_t_per_day` [D] tanker
  15.9 (99.2 t main), HLS 0.45 (208 t), `window_margin_h` [E] 4 (the window closes 4 h before the limit, the flown
  phasing keeps the full limit; the deadline counts from the separation). `periapsis_km = 170` stays as the reference
  set. Lever `reference_phasing` (CLI `--reference-phasing`, game key `reference_phasing`; tankers'
  `phasing_periapsis_km` / `phasing_apoapsis_km`).
- Depot orbit in the calculator (user 2026-10-22): the flown depot drifts - drag lowers it (a free 572 t depot at 198
  km: 0.33 km/day) and every transfer's settling push raises it (prograde, nose along +V-bar: 0.547 m/s per tanker
  transfer; the HLS's ~3.4 m/s). `mission::DepotOrbitTrack` keeps the mean semi-major axis in segments (linear decay,
  the phase from the changing mean motion, -3/4 n/a da/dt t^2) with the keeping floor and target;
  `CampaignModels::depot_orbit_event` changes it at t = 0 and after every transfer (push `transfer_push_dv_mps` 0.55
  [D], decay rho(US76) Cd 2.2 x end-on area x sqrt(mu a) / m x `drag_decay_factor` 1.076 [D]: the free depot 0.3294 vs
  0.3062 km/day - its test guards this; the 14 flown post-transfer heights independently fit 1.08) and books the
  reboosts it flew.
- Mean orbit: `orbit::energy_mean_semi_major_axis_m` (J2 energy; the osculating a swings +-2 km at 200 km / 26.5 deg)
  and `station_mean_elements_from_state`; point mass + J2 propagation holds its phase within +-20 km over 5 days. The
  raise to a phasing orbit sizes its burns from `orbit::j2_mean_orbit` (averaged over one revolution; osculating
  apsides swing +-10 km) and raises a higher apoapsis at the periapsis.
- Measured numbers `config/refuelling/reference_flights.toml` [D]: checked by the slow pad flights (`depot_launch`,
  `tanker_flight`, `hls_refuel`; 1 % or 0.3 t via `refuelling_test::check_reference`) - re-measure on any change;
  `inputs_digest` (FNV-1a of the configs the flights read): a config change without a re-measure fails the fast
  `[refuelling][campaign][config]` check (C++ model changes are not caught).
- CLI `build/core/refuelling_report.exe [--height-km] [--interval-h] [--phasing-limit-h] [--tanker-dry-t] [--reserve-t]
  [--initial-t] [--insulation] [--hls-wait-d] [--reference-phasing] [--fly] [--fly-placed]` (~1 s; exit 0 closes, 2
  open, 1 error).
- Game: `FlightSim.transfer_state()`, `depot_state()`, `refuelling_campaign(params)` (worker command, result in
  `result(id)["data"]`; keys in `bindings/src/flight_sim.hpp`; the command holds the simulation clock ~0.5-1 s),
  `FlightSim.campaign_flight_state()` (key `present`; rows, losses, reboosts, the first plan). `iss_state` /
  `station_geometry` ports export `mechanism` and `elements_m`.

### Flown campaign (f2)
Plan `docs/superpowers/archive/plans/2026-10-21-refuelling-f2-flown-campaign.md`.
- Driver `sim::RefuellingCampaignFlight` (a `SessionSupervisor`: `FlightSession` calls `update` every tick and stops a
  time-warp call at `wake_epoch`): mission `refuelling_campaign_flown` (depot launched by `depot_launch`) or
  `refuelling_campaign_flown_placed` (depot placed with demand minus 1.5 net deliveries, two tankers). Phases
  DepotAscent -> Planning -> WaitLaunch -> Flying -> (StationKeeping) -> ... -> Complete / Ended. Every plan is the
  calculator re-planned from the flown depot (`CampaignLive`: its state, fill, last launch and arrival, 1 h pad
  readiness [E]); the first plan is the comparison's baseline. Flights are sequential; each launches with
  `keep_station` beside the kept depot (`launch_campaign_flight`); after it the chaser is dropped (`keep_only_station`).
  Only the first tanker flies its return, in a blocking side session (`place_vehicle`): caught, 2.95 t main left. A
  failed flight re-plans from the next window. Booked on the depot: attitude gas 20 kg/day [E] (1 kg batches), the
  transfer vent 1.5 % [E].
- Station keeping: `[station_keeping]` below 195 km mean height (J2 energy), nothing in flight and the next arrival
  more than 24 h away -> a Hohmann pair to 200 km [E] from the energy-mean a (first burn 30 min ahead, the second half
  a transfer later, tail-off awaited). At 636 t the belly Raptor's transients exceed 60 % of a ~2 m/s burn, so the
  burns fly cold gas: 191 -> 200 km cost 6.5 t (Raptor ideal ~0.8 t). The full run needed none.
- Comparison (user 2026-10-22; ascent ruling 2026-10-23): count equal to the first plan; the fill after each
  delivering tanker within 2 % of the calculator's model of the flown window (`CampaignFlownRow::window_model`: the
  live plan made just before its launch, in the phasing class flown); the phasing-class mix against the first plan -
  |tankers above the depot in the first plan's paired flights - tankers flown above| - at most one in four delivering
  tankers (at least 1, `max_class_mix_difference` [E]); pairwise class flips only reported; each arrival (the HLS's:
  the duration) within 1 d of the live plan made just before its launch; the HLS holds its required propellant; the
  first return caught or upright abort. The first plan's duration is only reported (its windows flip at thresholds).

## Code

- Vehicles / contact: `vehicle/boil_off`, `vehicle/propellant_transfer`, `vehicle::mated_stage_transform`,
  `vehicle::port_mismatch`, `vehicle::outlet_check`, `vehicle::settling_push`, `vehicle::settling_gas_bound_kg`,
  `vehicle::apparent_gravity_body_mps2`, `Vehicle::update_settling`, `contact::probe_drogue_contact`,
  `contact::probe_drogue_mechanism`, `environment/thermal_radiation`.
- Sim: `FlightSession::start_transfer` (`flight_session_transfer.cpp`), `sim/propellant_transfer`,
  `IssMissionSequence` (`flight_session_iss_mission.cpp`), `sim/refuelling_mission`,
  `config/refuelling_mission_config.hpp`, `sim::RefuellingCampaignFlight`, `sim::plan_refuelling_campaign`,
  `mission::refuelling_campaign`, `mission::phasing_schedule`, `mission::plan_launch_window`, `mission::tanker_reserve`,
  `mission::DepotOrbitTrack`, `gnc::next_raise_node`, `gnc::DockingApproach::RBarBelow`.
- Tools: `build/core/refuelling_report.exe`, `tools/vehicle_report.py`, `tools/validation/centaur_boil_off_calibration.py`.

## Standing decisions

- Block 3 tanker/depot/HLS share the ship stage via `from`; layout positions are [E] (SpaceX publishes none).
- -x docking thrusters on tanker and HLS, none on the Block 3 ship (user 2026-10-20).
- Wetted-wall boil-off (user 2026-10-20); Block 3 lunar returns keep `boil_off = false`.
- Tanker ascent figures (209 t dry, 120 x 190 km insertion) are [E] placeholders (user 2026-10-20).
- Depot orbit followed in the calculator (user 2026-10-22); delivery calibration offsets (user 2026-10-21).
- Pad turnaround 24 h [E] and the HLS-first-window rule; the calculator takes the first window (fewest days).
- Raise tail-off wait without a timeout (no public path keeps an engine running with the autopilot idle in Raise).
- `load_stage_from` reloads the source vehicle per reused stage (milliseconds, once per load).
- Settling push before in-orbit ignitions; settling gas reserved by planners (f3, flag `ignition_settling`).

## Validation

- Boil-off (`[boil_off][validation]`, `tools/validation/centaur_boil_off_calibration.py`): Centaur LO2 sidewall fluxes
  from flight reconstructions (De Kruif & Kutter, AIAA 2007-5845, Table 2; sunlit coasts, broadside, rotisserie;
  coast geometry [E]: axis along the orbit normal, Sun in the plane at noon mid-coast):

  | Flight | Insulation, orbit | Flight W/m^2 | Model W/m^2 |
  |---|---|---|---|
  | AC-206 | white foam, 167 x 1,132 km | 72.6 | 72.5 (calibration: R = 1.586 m^2K/W [D]) |
  | AC-167 | white foam, 185 x 267 km | 78.9 | 76.3 (-3 %) |
  | AV-003 | white foam, 167 x 4,165 km | 78.9 | 63.1 (-20 %) |
  | TC-11/15 | 3-layer MLI, transfer orbit | 9.46 (avg of 6.3 / 12.6) | 9.46 (calibration: R = 8.22 [D], sun only) |

  Tolerance 25 % (the sister flights TC-11 / TC-15 differ by 2x). Paint alpha 0.25 / eps 0.88 and MLI cover 0.30 / 0.80
  [E]. A 600 s-segment test through the eclipse matches the orbit average (3 %).
- Tests by tag: `[refuelling]` (`test_refuelling_vehicles.cpp`, `test_refuelling_docking.cpp`,
  `test_refuelling_rendezvous.cpp`; with `[rcs]`, `[config]`, `[station]`, `[proximity]`, `[contact]`, `[docking]`,
  `[rendezvous]`, `[launch_window]`; campaign and undocking `[slow]`); `[boil_off]` (`test_boil_off.cpp`);
  `[refuelling][prop_transfer]` (`test_refuelling_transfer.cpp`, with `[hls]`, `[deorbit]`, `[mission]`);
  `[refuelling][mission]`, `[iss_mission]`, `[station]`, `[catch]`, `[departure]`, `[lunar]`
  (`test_refuelling_missions.cpp`, `test_refuelling_mission_config.cpp`; campaigns `[slow]`; probes `[.r5_probe]`,
  `[.f1_probe]`); `[burn_node]` (`test_raise_to_orbit.cpp`); `[rendezvous][phasing]` (mirrored schedule, catch-up rate
  against 1.5 n da/a within 2 %, planner's retrograde boost, reserve per burn); `[iss][launch_window]` (candidates, ISS
  unchanged, a raise eating the limit, cost and gas per burn by hand); `[refuelling][campaign]`
  (`test_refuelling_campaign.cpp`: calculator against hand computations, costs, file, CLI parser/text, track and orbit
  events, free depot decay, HLS extra demand; reference, per-window and lever campaigns `[slow]`);
  `[ignition_settling]` (`test_ignition_settling.cpp`: progress and travel, PMD and header feed, ingestion in / out of
  orbit and with the flag off, the push before a raise burn, coast stop in 10 s and 600 s steps, the held ignition,
  manual settle, the turn before the push, the allocator query, the gas bound and estimates);
  `[campaign_flown]` (`test_refuelling_campaign_flight.cpp`: supervisor, keep_station, place_vehicle, reboost, live
  start, free depot decay; placed and full campaign `[slow]`); `tools/test_vehicle_report.py`.
- Reference flights (current, derived pad heights): tanker residue 105.6 t, rendezvous 17.93 t, delivered LOX / CH4
  58.9 / 14.7 t, main at catch 1.38 t; depot fill 70.7 t (199.7 x 211.3 km); HLS main at docking 203.5 t; above-depot
  pad flight 220 x 240 km, Waypoint 0 9.9 h after liftoff (window 10.0 h).
- Calculator answer (default, depot placed at 200 km, 48 h): **12 tanker flights, 50.7 days** (planned kick 74.25 t per
  reference tanker). `--reference-phasing`: 14 / 224.7 d. Cadence: the 24 h pad turnaround against plane crossings
  ~23.4 h apart lets only every other crossing fly (`--interval-h 12`: 12 / 23.1 d); the gaps between runs are phases
  no candidate closes in 48 h; launches in runs of four per ~13.8 d phase cycle (r6 f1: 150 x 190 km when the depot is
  ahead, 220 x 240 / 250 x 270 km when behind). Levers (f1, 12 / 46.5 d baseline then): 300 km 13 / 31.9 d; tanker dry
  -10 t 11 / 45.9 d; insulation x 0.1 16 / 60.2 d; 28 h 11 / 75.2 d; 72 h 14 / 34.5 d (a looser limit opens earlier
  windows with longer phasing and more tanker boil-off: fewer days, more tankers).
- Full flown run (`refuelling_report --fly`, ~22 min; ctest `Flown campaign: the depot launched*` `[slow]`): depot
  launched 2026-10-20, 14 tankers, 0 failures, HLS at the depot 59.44 d with 1,090.8 t (needs 1,041.4 t); first plan 14
  tankers / 53.2 d; arrivals within 0.07 d of their live plans; fills within 2 % (largest: tanker 13, 864.4 vs 850.9 t,
  +1.6 %); deliveries 58.6-72.5 t; losses boil-off (remainder) 31.6 t, attitude gas 1.2 t, vent 27.3 t, station
  keeping 0. Flown depot heights 207.5 -> 209.8 km over 53 days. Placed variant (`--fly-placed`, ~5 min): 2 tankers +
  HLS, passes. Both passed against the calculator after the pad-height change.

## Diagnostics

- `STARSIM_DEBUG_RENDEZVOUS`, `STARSIM_DEBUG_PROXIMITY`, `STARSIM_DEBUG_DOCKING` as for the ISS;
  `STARSIM_DEBUG_PEG` (ascent cutoff: radius, speeds, v_go, tail-off).
- `STARSIM_DEBUG_SETTLING=1` prints each push (start before ignition, settled, body rate; mean push against the
  capability; duration, gas, lead) and every held ignition.
- `STARSIM_VERBOSE=1` prints the boil-off numbers; `refuelling_report.exe --fly` / `--fly-placed`.

## Open points and known limits

- Campaign: flights strictly sequential; the side return blocks the session; the calculator's keeping floor acts
  continuously (the flown rule checks at its plans and waits for 24 h to the next arrival) and books reboosts at the
  Raptor's exhaust velocity (the flown light depot flies them on cold gas); the transfer push at the arrival (flown
  ~3 h later); reboosts after the last transfer unbooked. The FlightSim campaign command holds the clock ~0.5-1 s.
- Follow-ups: a consistent J2-mean rendezvous (raise, schedule and planner on one mean model - the J2-mean schedule
  alone failed against the osculating raise/planner); the depot's cold gas keeps pushing with empty main tanks (headers
  counted in `main_dry`); light-depot reboosts on cold gas (6.5 t).
- More arrivals per cycle need another phasing orbit per window (the f1 candidate set closes only some phase gaps).
- Active cooling, transfer pressurant / displaced-gas losses: not modelled. `DeorbitParameters::boil_off_kgps` offline.
- Multi-pad launches (Cape, deferred): needs a depot >= 28.6 deg; not in scope.
