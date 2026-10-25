# Orbital refuelling (milestone r) - history

Verbatim record of this subsystem's notes as they stood in `docs/subsystems.md` on 2026-10-25, before the split. Current state: `docs/subsystems/refuelling.md`. Kept for the reasons behind decisions, superseded numbers and earlier approaches; read only when that history is needed.

Spec `docs/superpowers/archive/specs/2026-10-20-orbital-refuelling-design.md`: depot + tankers + HLS, back-to-back docking,
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
Plan `docs/superpowers/archive/plans/2026-10-20-refuelling-r3-rendezvous-docking.md`. The ISS stack (station body, phasing
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
Plan `docs/superpowers/archive/plans/2026-10-20-refuelling-r4-propellant-transfer.md`.
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
Plan `docs/superpowers/archive/plans/2026-10-20-refuelling-r5-reference-flights.md`, ledger
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
Plan `docs/superpowers/archive/plans/2026-10-20-refuelling-r6-campaign.md`, ledger
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
Spec `docs/superpowers/archive/specs/2026-10-21-refuelling-followups-design.md` (f1), plan
`docs/superpowers/archive/plans/2026-10-21-refuelling-f1-phasing-per-window.md`, ledger
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
Spec `docs/superpowers/archive/specs/2026-10-21-refuelling-followups-design.md` (f3), plan
`docs/superpowers/archive/plans/2026-10-21-refuelling-f3-settling.md`, ledger
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

Plan `docs/superpowers/archive/plans/2026-10-21-refuelling-f4-deferred-minors.md`: the minors of the r1-r6, f1 and f3 final
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
Plan `docs/superpowers/archive/plans/2026-10-21-refuelling-f2-flown-campaign.md`, ledger
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

